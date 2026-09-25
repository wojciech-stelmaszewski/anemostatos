import { AltitudeController } from '@/control/altitude';
import { CascadeController } from '@/control/cascade';
import { PointMassController } from '@/control/pointmass';
import type { PidTerms } from '@/control/pid';
import type { Controller, Setpoint } from '@/control/types';
import { qFromAxisAngle, qToEuler, type Quat } from '@/math/quat';
import { clone, length, scale, sub, add, v3, type Vec3 } from '@/math/vec3';
import { FOOT_HEIGHT } from '@/sim/drone';
import {
  idleActuation,
  initialState,
  stepDynamics,
  type Actuation,
  type DroneState,
  type Forces,
} from '@/sim/dynamics';
import type { Level, Params } from '@/sim/params';
import { Sensors, type Measurement } from '@/sim/sensors';
import { Wind } from '@/sim/wind';
import { Telemetry } from './telemetry';

export const PHYS_DT = 0.001;
export const TELEMETRY_HZ = 200;
const TELEMETRY_EVERY = Math.round(1 / (TELEMETRY_HZ * PHYS_DT));
const HISTORY_SECONDS = 60;
const MAX_STEPS_PER_FRAME = 200;
const POKE_DURATION = 0.05;
/** Vertical speed of the automatic take-off setpoint ramp, m/s. */
const TAKEOFF_RATE = 1;

const makeController = (level: Level): Controller =>
  level === 1
    ? new AltitudeController()
    : level === 2
      ? new PointMassController()
      : new CascadeController();

/**
 * Owns the whole simulated world: drone, wind, sensors, controller and telemetry.
 * Framework-free; the UI drives it with `advance(frameDt)` and reads its public fields.
 */
export class Simulation {
  params: Params;
  state: DroneState = initialState();
  /** Previous physics state, for render interpolation. */
  prevPos: Vec3 = clone(this.state.pos);
  prevQ: Quat = { ...this.state.q };
  /** 0…1 position between prev and current physics state, for rendering. */
  alpha = 1;

  t = 0;
  stepIndex = 0;
  armed = true;
  paused = false;
  timeScale = 1;
  /** True if the last frame had to drop simulation time to keep up. */
  lagging = false;

  /** Where the user wants the drone (target) and the rate-limited setpoint actually tracked. */
  target: Vec3;
  setpoint: Vec3;
  yaw = 0;

  wind: Wind;
  sensors: Sensors;
  controller: Controller;
  actuation: Actuation = idleActuation();
  forces: Forces = { thrust: v3(), gravity: v3(), drag: v3(), external: v3(), net: v3() };
  measurement: Measurement | null = null;
  telemetry = new Telemetry(HISTORY_SECONDS * TELEMETRY_HZ);
  /** Times of setpoint steps (for step-response analysis). */
  stepEvents: { t: number; from: number; to: number }[] = [];

  /** While true the setpoint ramps up from the ground instead of jumping. */
  takingOff = true;
  private accumulator = 0;
  private poke: { force: Vec3; until: number } | null = null;
  private resetListeners = new Set<() => void>();

  constructor(params: Params) {
    this.params = params;
    this.target = v3(params.setpoint.x, params.setpoint.y, params.setpoint.z);
    this.setpoint = clone(this.target);
    this.wind = new Wind(params.sim.seed);
    this.sensors = new Sensors(params.sim.seed);
    this.controller = makeController(params.sim.level);
    this.reset();
  }

  get level(): Level {
    return this.params.sim.level;
  }

  onReset(fn: () => void): () => void {
    this.resetListeners.add(fn);
    return () => this.resetListeners.delete(fn);
  }

  /** Accept a new parameter tree. Level or seed changes restart the run. */
  setParams(next: Params): void {
    const prev = this.params;
    this.params = next;
    const sp = next.setpoint;
    if (sp.x !== prev.setpoint.x || sp.y !== prev.setpoint.y || sp.z !== prev.setpoint.z) {
      this.setTarget(v3(sp.x, sp.y, sp.z));
    }
    this.yaw = (sp.yawDeg * Math.PI) / 180;
    if (next.sim.level !== prev.sim.level || next.sim.seed !== prev.sim.seed) this.reset();
  }

  /** Restart from the ground with the current parameters. */
  reset(): void {
    const p = this.params;
    this.state = initialState();
    this.prevPos = clone(this.state.pos);
    this.prevQ = { ...this.state.q };
    this.t = 0;
    this.stepIndex = 0;
    this.accumulator = 0;
    this.wind = new Wind(p.sim.seed);
    this.sensors = new Sensors(p.sim.seed);
    this.controller = makeController(p.sim.level);
    this.actuation = idleActuation();
    this.target = v3(p.setpoint.x, p.setpoint.y, p.setpoint.z);
    if (p.sim.level === 1) this.target = v3(0, this.target.y, 0);
    this.setpoint = v3(this.target.x, this.state.pos.y, this.target.z);
    this.takingOff = true;
    this.yaw = (p.setpoint.yawDeg * Math.PI) / 180;
    this.state.q = p.sim.level === 3 ? qFromAxisAngle(v3(0, 1, 0), 0) : this.state.q;
    this.telemetry.clear();
    this.stepEvents = [];
    this.poke = null;
    this.armed = true;
    this.measurement = null;
    for (const fn of this.resetListeners) fn();
  }

  setTarget(pos: Vec3): void {
    const next = this.level === 1 ? v3(0, pos.y, 0) : clone(pos);
    if (length(sub(next, this.target)) > 1e-9) {
      this.stepEvents.push({ t: this.t, from: this.target.y, to: next.y });
      if (this.stepEvents.length > 50) this.stepEvents.shift();
    }
    this.target = next;
  }

  setArmed(armed: boolean): void {
    if (armed && !this.armed) {
      this.controller.reset();
      if (this.onGround) {
        this.setpoint = v3(this.target.x, this.state.pos.y, this.target.z);
        this.takingOff = true;
      }
    }
    this.armed = armed;
  }

  /** Short push, N·s, applied over 50 ms. */
  applyImpulse(impulse: Vec3): void {
    this.poke = { force: scale(impulse, 1 / POKE_DURATION), until: this.t + POKE_DURATION };
  }

  triggerGust(direction: Vec3, amplitude: number, duration: number): void {
    this.wind.trigger(this.t, direction, amplitude, duration);
  }

  /** Called every animation frame with the real elapsed time. */
  advance(frameDt: number): void {
    if (this.paused) return;
    this.accumulator += Math.min(frameDt, 0.1) * this.timeScale;
    let steps = 0;
    while (this.accumulator >= PHYS_DT && steps < MAX_STEPS_PER_FRAME) {
      this.step();
      this.accumulator -= PHYS_DT;
      steps++;
    }
    this.lagging = this.accumulator >= PHYS_DT;
    if (this.lagging) this.accumulator = 0;
    this.alpha = this.accumulator / PHYS_DT;
  }

  /** Advance by one controller period (for single-stepping while paused). */
  stepController(): void {
    const n = Math.max(1, Math.round(1 / (this.params.control.rateHz * PHYS_DT)));
    for (let k = 0; k < n; k++) this.step();
    this.alpha = 1;
  }

  /** One physics step. */
  step(): void {
    const p = this.params;
    const dt = PHYS_DT;
    this.prevPos = clone(this.state.pos);
    this.prevQ = { ...this.state.q };

    // Rate-limited setpoint (always ramped during take-off).
    const rl = p.setpoint.rateLimit > 0 ? p.setpoint.rateLimit : this.takingOff ? TAKEOFF_RATE : 0;
    if (rl > 0) {
      const d = sub(this.target, this.setpoint);
      const l = length(d);
      const maxStep = rl * dt;
      this.setpoint = l <= maxStep ? clone(this.target) : add(this.setpoint, scale(d, maxStep / l));
    } else {
      this.setpoint = clone(this.target);
    }
    // Take-off ends once the ramp is done and the drone has arrived or stopped climbing.
    if (this.takingOff && length(sub(this.target, this.setpoint)) < 1e-9) {
      const arrived = length(sub(this.target, this.state.pos)) < 0.05;
      const stalled = Math.abs(this.state.vel.y) < 0.02 && !this.state.landed;
      if (arrived || stalled) this.takingOff = false;
    }

    const wind = this.wind.step(this.t, dt, p.wind);
    this.sensors.record(this.state);

    if (this.armed && !this.state.crashed) {
      if (this.state.landed || this.takingOff) this.controller.resetIntegrators();
      const sp: Setpoint = { pos: this.setpoint, yaw: this.yaw };
      this.actuation = this.controller.tick({
        params: p,
        setpoint: sp,
        physDt: dt,
        stepIndex: this.stepIndex,
        sense: () => (this.measurement = this.sensors.read(p.sensors, dt)),
      });
    } else {
      this.actuation = idleActuation();
    }

    const ext = this.poke && this.t < this.poke.until ? this.poke.force : v3();
    if (this.poke && this.t >= this.poke.until) this.poke = null;
    this.forces = stepDynamics(this.level, this.state, this.actuation, wind, ext, p.drone, dt);
    if (this.state.crashed) this.armed = false;

    this.t += dt;
    this.stepIndex++;
    if (this.stepIndex % TELEMETRY_EVERY === 0) this.record();
  }

  private record(): void {
    const tl = this.telemetry;
    const s = this.state;
    tl.set('pos.x', s.pos.x);
    tl.set('pos.y', s.pos.y);
    tl.set('pos.z', s.pos.z);
    tl.set('vel.x', s.vel.x);
    tl.set('vel.y', s.vel.y);
    tl.set('vel.z', s.vel.z);
    tl.set('sp.x', this.setpoint.x);
    tl.set('sp.y', this.setpoint.y);
    tl.set('sp.z', this.setpoint.z);
    const e = qToEuler(s.q);
    const DEG = 180 / Math.PI;
    tl.set('roll', e.roll * DEG);
    tl.set('pitch', e.pitch * DEG);
    tl.set('yaw', e.yaw * DEG);
    for (let i = 0; i < 4; i++) {
      tl.set(`motor.${i + 1}`, s.motors[i]!);
      tl.set(`motorCmd.${i + 1}`, this.actuation.motorCmd[i]!);
    }
    const thrust = s.motors[0] + s.motors[1] + s.motors[2] + s.motors[3];
    tl.set('thrust', this.level === 2 ? length(s.force) : thrust);
    tl.set('wind.x', this.wind.velocity.x);
    tl.set('wind.y', this.wind.velocity.y);
    tl.set('wind.z', this.wind.velocity.z);
    tl.set('wind.speed', length(this.wind.velocity));
    tl.set('gust', length(this.wind.gustVelocity));
    tl.set('drag.y', this.forces.drag.y);
    if (this.measurement) {
      tl.set('meas.x', this.measurement.pos.x);
      tl.set('meas.y', this.measurement.pos.y);
      tl.set('meas.z', this.measurement.pos.z);
    }
    const loops = this.controller.loops();
    for (const id in loops) recordLoop(tl, id, loops[id]!);
    const extras = this.controller.extras();
    for (const k in extras) tl.set(k, extras[k]!);
    tl.commit(this.t);
  }

  /** Interpolated pose for rendering. */
  renderPose(): { pos: Vec3; q: Quat } {
    const a = this.alpha;
    const pos = add(this.prevPos, scale(sub(this.state.pos, this.prevPos), a));
    return { pos, q: a >= 1 ? this.state.q : slerpish(this.prevQ, this.state.q, a) };
  }

  get onGround(): boolean {
    return this.state.pos.y <= FOOT_HEIGHT + 1e-6;
  }
}

function recordLoop(tl: Telemetry, id: string, t: PidTerms): void {
  tl.set(`${id}.sp`, t.setpoint);
  tl.set(`${id}.meas`, t.measurement);
  tl.set(`${id}.err`, t.error);
  tl.set(`${id}.p`, t.p);
  tl.set(`${id}.i`, t.i);
  tl.set(`${id}.d`, t.d);
  tl.set(`${id}.ff`, t.ff);
  tl.set(`${id}.pid`, t.p + t.i + t.d);
  tl.set(`${id}.u`, t.output);
  tl.set(`${id}.uraw`, t.unsaturated);
  tl.set(`${id}.sat`, t.saturated ? 1 : 0);
}

/** Normalised linear quaternion blend — plenty for 1 ms apart. */
function slerpish(a: Quat, b: Quat, t: number): Quat {
  const sign = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z < 0 ? -1 : 1;
  const q = {
    w: a.w + (sign * b.w - a.w) * t,
    x: a.x + (sign * b.x - a.x) * t,
    y: a.y + (sign * b.y - a.y) * t,
    z: a.z + (sign * b.z - a.z) * t,
  };
  const l = Math.hypot(q.w, q.x, q.y, q.z);
  return { w: q.w / l, x: q.x / l, y: q.y / l, z: q.z / l };
}
