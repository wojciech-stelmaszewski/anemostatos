import { controllerKey, makeController } from '@/control/registry';
import type { Controller, LoopTerms, Setpoint } from '@/control/types';
import { tiltError } from '@/estimation/attitude';
import { qFromAxisAngle, qRotate, qToEuler, type Quat } from '@/math/quat';
import { clone, length, scale, sub, add, v3, type Vec3 } from '@/math/vec3';
import { ARM_LENGTH, FOOT_HEIGHT } from '@/sim/drone';
import {
  idleActuation,
  initialState,
  motorTorques,
  stepDynamics,
  type Actuation,
  type DroneState,
  type Forces,
} from '@/sim/dynamics';
import { GRAVITY, type Level, type Params } from '@/sim/params';
import { Sensors, type Measurement } from '@/sim/sensors';
import { zoneDistance } from '@/sim/world';
import { Wind } from '@/sim/wind';
import { probeSignal } from './probe';
import { profileOffset, zeroReference, type Reference } from './reference';
import { FaultMonitor } from '@/estimation/fdi';
import { Tap } from './tap';
import { Telemetry } from './telemetry';

export const PHYS_DT = 0.001;
export const TELEMETRY_HZ = 200;
const TELEMETRY_EVERY = Math.round(1 / (TELEMETRY_HZ * PHYS_DT));
const HISTORY_SECONDS = 60;
const MAX_STEPS_PER_FRAME = 200;
const POKE_DURATION = 0.05;
/** Vertical speed of the automatic take-off setpoint ramp, m/s. */
const TAKEOFF_RATE = 1;

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
  /** Wall-clock cost of the controller, µs per physics step (averaged over the last frame). */
  controlMicros = 0;
  private cpuSum = 0;
  private cpuSteps = 0;

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
  /** The last eight seconds of the gyro and one motor command at the physics rate, for spectra. */
  tap = new Tap();
  /** The fault monitor of the flight computer (L3, lesson III.28). */
  fdi = new FaultMonitor();
  /** A frozen earlier run, overlaid on the charts for comparison. */
  ghost: { telemetry: Telemetry; label: string } | null = null;
  /** A lesson's script: run after every reset to (re)schedule its events. */
  script: ((sim: Simulation) => void) | null = null;
  /** The parameters the running lesson started with (scripted changes are undone on reset). */
  lessonStart: Params | null = null;
  /** Scripted events (lessons): run once when simulation time passes `t`. */
  private scheduled: { t: number; fn: (sim: Simulation) => void }[] = [];
  /** Setpoint without the automatic profile. */
  private base: Vec3 = v3();

  /** The base point the automatic profile moves around (for drawing the path). */
  get profileBase(): Vec3 {
    return this.base;
  }
  /** Times of setpoint steps (for step-response analysis). */
  stepEvents: { t: number; from: number; to: number }[] = [];

  /** While true the setpoint ramps up from the ground instead of jumping. */
  takingOff = true;
  /** The automatic setpoint motion with its derivatives (zero without a profile). */
  reference: Reference = zeroReference(v3());
  /** When the planar profiles started their first lap. */
  profileStart = 0;
  /** Seconds spent inside a keep-out zone since the last reset. */
  zoneTime = 0;
  /**
   * The probe (docs/analysis.md §3.3): the injected signal, the command that reaches the plant
   * and the controller's own part of it. For a thrust probe u = uc + in, in newtons.
   */
  probe = { in: 0, u: 0, uc: 0, tau: { x: 0, z: 0 }, tauC: { x: 0, z: 0 } };
  /** When the probe was last switched on or changed; its signal starts from zero phase there. */
  private probeT0 = 0;

  /** Seconds since the probe was switched on or last changed. */
  get probeAge(): number {
    return this.t - this.probeT0;
  }
  private accumulator = 0;
  private poke: { force: Vec3; until: number } | null = null;
  private resetListeners = new Set<() => void>();

  constructor(params: Params) {
    this.params = params;
    this.target = v3(params.setpoint.x, params.setpoint.y, params.setpoint.z);
    this.base = clone(this.target);
    this.setpoint = clone(this.target);
    this.wind = new Wind(params.sim.seed);
    this.sensors = new Sensors(params.sim.seed);
    this.controller = makeController(params);
    this.reset();
  }

  get level(): Level {
    return this.params.sim.level;
  }

  onReset(fn: () => void): () => void {
    this.resetListeners.add(fn);
    return () => this.resetListeners.delete(fn);
  }

  /** Accept a new parameter tree. Level, seed or controller changes restart the run. */
  setParams(next: Params): void {
    const prev = this.params;
    this.params = next;
    const sp = next.setpoint;
    if (sp.x !== prev.setpoint.x || sp.y !== prev.setpoint.y || sp.z !== prev.setpoint.z) {
      this.setTarget(v3(sp.x, sp.y, sp.z));
    }
    this.yaw = (sp.yawDeg * Math.PI) / 180;
    const pp = prev.setpoint;
    if (sp.profile !== pp.profile || sp.profilePeriod !== pp.profilePeriod)
      this.profileStart = this.t;
    if (next.probe !== prev.probe && JSON.stringify(next.probe) !== JSON.stringify(prev.probe))
      this.probeT0 = this.t;
    if (next.sim.seed !== prev.sim.seed || controllerKey(next) !== controllerKey(prev))
      this.reset();
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
    this.controller = makeController(p);
    this.actuation = idleActuation();
    this.target = v3(p.setpoint.x, p.setpoint.y, p.setpoint.z);
    if (p.sim.level === 1) this.target = v3(0, this.target.y, 0);
    this.base = clone(this.target);
    this.setpoint = v3(this.target.x, this.state.pos.y, this.target.z);
    this.takingOff = true;
    this.profileStart = 0;
    this.zoneTime = 0;
    this.probeT0 = 0;
    this.probe = { in: 0, u: 0, uc: 0, tau: { x: 0, z: 0 }, tauC: { x: 0, z: 0 } };
    this.reference = zeroReference(v3());
    this.yaw = (p.setpoint.yawDeg * Math.PI) / 180;
    this.state.q = p.sim.level === 3 ? qFromAxisAngle(v3(0, 1, 0), 0) : this.state.q;
    this.telemetry.clear();
    this.stepEvents = [];
    this.scheduled = [];
    this.poke = null;
    this.armed = true;
    this.measurement = null;
    this.tap.reset();
    this.fdi.reset();
    this.script?.(this);
    for (const fn of this.resetListeners) fn();
  }

  /** Run `fn` once simulation time reaches `t` (cleared on reset; lesson scripts re-add theirs). */
  schedule(t: number, fn: (sim: Simulation) => void): void {
    this.scheduled.push({ t, fn });
    this.scheduled.sort((a, b) => a.t - b.t);
  }

  clearSchedule(): void {
    this.scheduled = [];
  }

  /** Freeze the current run as the comparison ghost. */
  snapshot(label: string): void {
    this.ghost = { telemetry: this.telemetry.clone(), label };
  }

  setTarget(pos: Vec3): void {
    this.base = this.level === 1 ? v3(0, pos.y, 0) : clone(pos);
    this.applyTarget(this.withProfile(this.base), true);
  }

  private withProfile(base: Vec3): Vec3 {
    const sp = this.params.setpoint;
    if (sp.profile === 'none' || this.takingOff) {
      this.reference = zeroReference(base);
      return base;
    }
    const r = profileOffset(this.params, this.level, this.t, this.profileStart);
    const out = v3(base.x + r.pos.x, base.y + r.pos.y, base.z + r.pos.z);
    out.y = Math.max(out.y, 0.2);
    this.reference = { ...r, pos: out };
    return out;
  }

  private applyTarget(pos: Vec3, userStep: boolean): void {
    const next = this.level === 1 ? v3(0, pos.y, 0) : clone(pos);
    if (userStep && length(sub(next, this.target)) > 1e-9) {
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
    if (this.cpuSteps > 0) {
      const us = (this.cpuSum / this.cpuSteps) * 1000;
      this.controlMicros += (us - this.controlMicros) * 0.1; // smoothed for display
      this.cpuSum = this.cpuSteps = 0;
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

    // Scripted events and the automatic setpoint profile.
    while (this.scheduled.length && this.scheduled[0]!.t <= this.t)
      this.scheduled.shift()!.fn(this);
    if (p.setpoint.profile !== 'none') this.applyTarget(this.withProfile(this.base), false);
    else if (!this.takingOff && length(sub(this.target, this.base)) > 1e-9)
      this.applyTarget(this.base, false);

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
      if (arrived || stalled) {
        this.takingOff = false;
        this.profileStart = this.t;
      }
    }

    const wind = this.wind.step(this.t, dt, p.wind);
    this.sensors.record(this.state, {
      sensors: p.sensors,
      vibration: p.vibration,
      drone: p.drone,
      ahrs: p.control.ahrs,
      dt,
    });

    const probing = p.probe.point !== 'none' && this.armed && !this.state.crashed;
    const probeIn = probing ? probeSignal(p.probe, this.t - this.probeT0) : 0;

    if (this.armed && !this.state.crashed) {
      if (this.state.landed || this.takingOff) this.controller.resetIntegrators();
      // Reference derivatives only while the setpoint follows the profile exactly.
      const exact = p.setpoint.profile !== 'none' && rl === 0 && !this.takingOff;
      const r = this.reference;
      const pos =
        p.probe.point === 'ref.y' && probeIn !== 0
          ? v3(this.setpoint.x, this.setpoint.y + probeIn, this.setpoint.z)
          : this.setpoint;
      const sp: Setpoint = exact
        ? { pos, yaw: this.yaw, vel: r.vel, acc: r.acc, jerk: r.jerk, snap: r.snap }
        : { pos, yaw: this.yaw };
      const t0 = performance.now();
      this.actuation = this.controller.tick({
        params: p,
        setpoint: sp,
        physDt: dt,
        stepIndex: this.stepIndex,
        sense: () => (this.measurement = this.sensors.read(p.sensors, dt, this.t)),
        preview: (tau) => this.previewAt(tau, exact),
        fdi: this.level === 3 ? this.fdi.state : undefined,
      });
      // Measured only for display; wall-clock time never feeds back into the simulation.
      this.cpuSum += performance.now() - t0;
      this.cpuSteps++;
    } else {
      this.actuation = idleActuation();
    }
    if (probing) this.injectProbe(probeIn);
    if (this.level === 3 && this.measurement) {
      const s = p.sensors;
      this.fdi.update(
        this.measurement,
        this.measurement.q,
        p.drone,
        {
          accSigma: Math.max(s.accNoise, 0.05),
          posSigma: Math.max(s.posNoise, 0.005),
          biasSigma: 0.01,
        },
        p.control.fdi,
        GRAVITY,
        dt,
        this.armed && !this.state.landed && !this.takingOff,
      );
    }
    if (this.level === 3) {
      const deg = 180 / Math.PI;
      const seen = this.measurement?.omega.x ?? 0;
      this.tap.push(
        this.state.omega.x * deg,
        seen * deg,
        (this.controller.gyroUsed?.x ?? seen) * deg,
        this.actuation.motorCmd[0],
      );
    }

    const ext = this.poke && this.t < this.poke.until ? this.poke.force : v3();
    if (this.poke && this.t >= this.poke.until) this.poke = null;
    this.forces = stepDynamics(this.level, this.state, this.actuation, wind, ext, p.drone, dt);
    if (this.state.crashed) this.armed = false;

    if (zoneDistance(this.state.pos, p.world, this.level) < 0) this.zoneTime += dt;

    this.t += dt;
    this.stepIndex++;
    if (this.stepIndex % TELEMETRY_EVERY === 0) this.record();
  }

  /**
   * Add the probe signal at its point and note what the plant receives. The controller's own
   * actuation object is left untouched: controllers hold it between their samples.
   */
  private injectProbe(d: number): void {
    const p = this.params;
    const cmd = this.actuation.motorCmd;
    const uc = cmd[0] + cmd[1] + cmd[2] + cmd[3];
    let u = uc;
    if (p.probe.point === 'l1.thrust' && this.level === 1) {
      const fmax = p.drone.maxMotorThrust;
      const f = cmd.map((c) => Math.min(Math.max(c + d / 4, 0), fmax)) as Actuation['motorCmd'];
      this.actuation = { ...this.actuation, motorCmd: f };
      u = f[0] + f[1] + f[2] + f[3];
    }
    const tauC = motorTorques(cmd, p.drone);
    let tau = tauC;
    if ((p.probe.point === 'l3.torque.x' || p.probe.point === 'l3.torque.z') && this.level === 3) {
      // The mixer's own map from a roll or pitch torque to the four motors.
      const k = d / (4 * (ARM_LENGTH / Math.SQRT2));
      const sign = p.probe.point === 'l3.torque.x' ? [1, -1, -1, 1] : [1, 1, -1, -1];
      const fmax = p.drone.maxMotorThrust;
      const f = cmd.map((c, i) =>
        Math.min(Math.max(c + sign[i]! * k, 0), fmax),
      ) as Actuation['motorCmd'];
      this.actuation = { ...this.actuation, motorCmd: f };
      tau = motorTorques(f, p.drone);
      const x = p.probe.point === 'l3.torque.x';
      u = x ? tau.x : tau.z;
      this.probe = { in: d, u, uc: x ? tauC.x : tauC.z, tau, tauC };
      return;
    }
    this.probe = { in: d, u, uc, tau, tauC };
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
    if (this.level === 3) {
      const f = this.fdi.state;
      tl.set('fdi.nis', f.nis);
      tl.set('fdi.altAlarm', f.altAlarm ? 1 : 0);
      tl.set('fdi.cusum', Math.max(...f.cusum));
      tl.set('fdi.motor', f.motor + 1);
    }
    if (this.level === 3 && this.params.sensors.attitude !== 'truth') {
      const f = this.sensors.ahrs;
      tl.set('ahrs.err', tiltError(s.q, f.q) * DEG);
      tl.set('ahrs.tilt', Math.acos(Math.min(1, qRotate(s.q, v3(0, 1, 0)).y)) * DEG);
      tl.set('ahrs.tiltHat', Math.acos(Math.min(1, qRotate(f.q, v3(0, 1, 0)).y)) * DEG);
      tl.set('ahrs.bias.x', f.bias.x * DEG);
      tl.set('ahrs.bias.z', f.bias.z * DEG);
      tl.set('ahrs.gated', f.trusted ? 0 : 1);
    }
    if (this.measurement) {
      const m = this.measurement;
      tl.set('meas.x', m.pos.x);
      tl.set('meas.y', m.pos.y);
      tl.set('meas.z', m.pos.z);
      tl.set('imu.acc.x', m.acc.x);
      tl.set('imu.acc.y', m.acc.y);
      tl.set('imu.acc.z', m.acc.z);
    }
    // The true disturbance as the controller's model sees it (docs/physics-model.md §7).
    const d = this.disturbance();
    tl.set('dist.x', d.x);
    tl.set('dist.y', d.y);
    tl.set('dist.z', d.z);
    const zd = zoneDistance(s.pos, this.params.world, this.level);
    tl.set('zone.dist', Number.isFinite(zd) ? zd : NaN);
    tl.set('zone.inside', zd < 0 ? 1 : 0);
    if (this.params.probe.point !== 'none') {
      tl.set('probe.in', this.probe.in);
      tl.set('probe.u', this.probe.u);
      tl.set('probe.uc', this.probe.uc);
    }
    const loops = this.controller.loops();
    for (const id in loops) recordLoop(tl, id, loops[id]!);
    const extras = this.controller.extras();
    for (const k in extras) tl.set(k, extras[k]!);
    tl.commit(this.t);
  }

  /** The reference `tau` seconds ahead: the planned profile if it is followed exactly. */
  private previewAt(tau: number, exact: boolean): Vec3 {
    if (!exact) return clone(this.setpoint);
    const r = profileOffset(this.params, this.level, this.t + tau, this.profileStart);
    const b = this.base;
    return v3(b.x + r.pos.x, Math.max(b.y + r.pos.y, 0.2), b.z + r.pos.z);
  }

  /**
   * Everything the nominal model m̂·a = thrust + m̂·g does not explain, N (world frame):
   * drag, wind, pokes, mass error and ground reaction. Part II observers try to estimate this.
   */
  disturbance(): Vec3 {
    const mHat = this.params.control.model.mass;
    const nominal = add(this.forces.thrust, v3(0, -mHat * GRAVITY, 0));
    return sub(scale(this.state.accel, mHat), nominal);
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

function recordLoop(tl: Telemetry, id: string, t: LoopTerms): void {
  tl.set(`${id}.sp`, t.setpoint);
  tl.set(`${id}.meas`, t.measurement);
  tl.set(`${id}.err`, t.error);
  let fb = 0;
  for (const part of t.parts) {
    tl.set(`${id}.part.${part.key}`, part.value);
    if (!part.ff) fb += part.value;
  }
  tl.set(`${id}.fb`, fb);
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
