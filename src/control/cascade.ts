import { qConj, qFromAxisAngle, qFromTo, qMul, qToEuler, qRotate, type Quat } from '@/math/quat';
import { clamp } from '@/math/util';
import { clampLength, dot, length, normalize, v3, type Vec3 } from '@/math/vec3';
import { idleActuation, type Actuation } from '@/sim/dynamics';
import { GRAVITY, type Params } from '@/sim/params';
import type { Measurement } from '@/sim/sensors';
import { mix } from './mixer';
import { defaultGains, Pid, type PidTerms } from './pid';
import {
  AccelIndi,
  NoCompensation,
  PidRateStage,
  RateIndi,
  type Compensation,
  type RateStage,
} from './stages';
import {
  periodSteps,
  pidLoop,
  type ControlInput,
  type Controller,
  type InfoRow,
  type LoopTerms,
} from './types';

const DEG = 180 / Math.PI;
const up = v3(0, 1, 0);

/** A proportional-only loop reported in the same shape as a PID (for the inspector). */
const pTerms = (
  setpoint: number,
  measurement: number,
  error: number,
  output: number,
  limited: number,
): PidTerms => ({
  setpoint,
  measurement,
  error,
  p: output,
  i: 0,
  d: 0,
  ff: 0,
  unsaturated: output,
  output: limited,
  saturated: output !== limited,
});

/**
 * Level 3 as a pipeline of stages (docs/beyond-pid.md §3.2):
 *   position P → velocity PID → [compensation] → thrust vector → attitude P → [rate stage] → mixer.
 * The outer stage is the PID cascade; compensation (none / acceleration INDI) and the rate stage
 * (PID / INDI) are swappable. Each loop runs at its own rate; angles in degrees.
 */
export class CascadeController implements Controller {
  readonly pos = { x: new Pid(), y: new Pid(), z: new Pid() };
  readonly vel = { x: new Pid(), y: new Pid(), z: new Pid() };
  readonly rate: RateStage;
  readonly comp: Compensation;
  att: Record<'roll' | 'pitch' | 'yaw', PidTerms> = {
    roll: pTerms(0, 0, 0, 0, 0),
    pitch: pTerms(0, 0, 0, 0, 0),
    yaw: pTerms(0, 0, 0, 0, 0),
  };

  private vSp = v3();
  private aSp = v3();
  private fDes = v3(0, GRAVITY, 0);
  private qSp: Quat = { w: 1, x: 0, y: 0, z: 0 };
  private thrust = 0;
  /** Body-rate setpoint, deg/s: x roll, y yaw, z pitch. */
  private wSp = v3();
  private held: Actuation = idleActuation();
  private mixerSaturated = false;

  constructor(p?: Params) {
    this.rate = p?.control.l3.inner === 'indi' ? new RateIndi() : new PidRateStage();
    this.comp = p?.control.l3.compensation === 'indi' ? new AccelIndi() : new NoCompensation();
  }

  reset(): void {
    for (const p of [...Object.values(this.pos), ...Object.values(this.vel)]) p.reset();
    this.rate.reset();
    this.comp.reset();
    this.vSp = v3();
    this.aSp = v3();
    this.fDes = v3(0, GRAVITY, 0);
    this.qSp = { w: 1, x: 0, y: 0, z: 0 };
    this.thrust = 0;
    this.wSp = v3();
    this.held = idleActuation();
    this.mixerSaturated = false;
  }

  resetIntegrators(): void {
    for (const p of Object.values(this.vel)) p.integral = 0;
    this.rate.resetIntegrators();
  }

  tick(input: ControlInput): Actuation {
    const { params: P, physDt, stepIndex, setpoint } = input;
    const c = P.control.l3;
    const due = (hz: number) => stepIndex % periodSteps(hz, physDt) === 0;
    const dtOf = (hz: number) => periodSteps(hz, physDt) * physDt;
    let m: Measurement | null = null;
    const sense = () => (m ??= input.sense());
    const mHat = P.control.model.mass;
    const fmax = P.drone.maxMotorThrust;

    // 0. Fast estimators of the compensation stage (filters) run with the rate loop.
    if (due(c.hzRate)) this.comp.sample(sense(), P, dtOf(c.hzRate));

    // 1. Position P → velocity setpoint.
    if (due(c.hzPos)) {
      const s = sense();
      const dt = dtOf(c.hzPos);
      const gH = defaultGains({ kp: c.posKpH, ki: 0, kd: 0 });
      const gV = defaultGains({ kp: c.posKpV, ki: 0, kd: 0 });
      const vx = this.pos.x.update(gH, setpoint.pos.x, s.pos.x, dt).output;
      const vz = this.pos.z.update(gH, setpoint.pos.z, s.pos.z, dt).output;
      const vy = this.pos.y.update(gV, setpoint.pos.y, s.pos.y, dt, 0, -c.vMaxV, c.vMaxV).output;
      const h = clampLength(v3(vx, 0, vz), c.vMaxH);
      this.vSp = v3(h.x, vy, h.z);
    }

    // 2. Velocity PID → acceleration → [compensation] → thrust vector → attitude setpoint.
    if (due(c.hzVel)) {
      const s = sense();
      const dt = dtOf(c.hzVel);
      const aH = GRAVITY * Math.tan((c.maxTiltDeg * Math.PI) / 180);
      const aUp = (4 * fmax) / Math.max(mHat, 0.1) - GRAVITY;
      // With an incremental (INDI) compensation the velocity loop's I term is redundant:
      // the increment already integrates.
      const noI = this.comp.replacesIntegral;
      const velH = noI ? { ...c.velH, iOn: false } : c.velH;
      const velV = noI ? { ...c.velV, iOn: false } : c.velV;
      const ax = this.vel.x.update(velH, this.vSp.x, s.vel.x, dt, 0, -aH, aH).output;
      const az = this.vel.z.update(velH, this.vSp.z, s.vel.z, dt, 0, -aH, aH).output;
      const ay = this.vel.y.update(velV, this.vSp.y, s.vel.y, dt, 0, -0.8 * GRAVITY, aUp).output;
      this.aSp = v3(ax, ay, az);
      let f = this.comp.force(this.aSp, P);
      f = v3(f.x, Math.max(f.y, 0.05 * mHat * GRAVITY), f.z);
      // Tilt limit: keep the vertical component, shrink the horizontal one.
      const hMax = f.y * Math.tan((c.maxTiltDeg * Math.PI) / 180);
      const h = clampLength(v3(f.x, 0, f.z), hMax);
      f = v3(h.x, f.y, h.z);
      this.fDes = f;
      const qYaw = qFromAxisAngle(up, setpoint.yaw);
      this.qSp = qMul(qFromTo(up, normalize(f)), qYaw);
    }

    // 3. Attitude P → body-rate setpoint (deg/s).
    if (due(c.hzAtt)) {
      const s = sense();
      let e = qMul(qConj(s.q), this.qSp);
      if (e.w < 0) e = { w: -e.w, x: -e.x, y: -e.y, z: -e.z };
      const err = v3(2 * e.x * DEG, 2 * e.y * DEG, 2 * e.z * DEG); // small-angle error, body axes
      const lim = c.maxRateDeg;
      const raw = v3(c.attKpRP * err.x, c.attKpYaw * err.y, c.attKpRP * err.z);
      this.wSp = v3(clamp(raw.x, -lim, lim), clamp(raw.y, -lim, lim), clamp(raw.z, -lim, lim));
      const eSp = qToEuler(this.qSp);
      const eNow = qToEuler(s.q);
      this.att = {
        roll: pTerms(eSp.roll * DEG, eNow.roll * DEG, err.x, raw.x, this.wSp.x),
        yaw: pTerms(eSp.yaw * DEG, eNow.yaw * DEG, err.y, raw.y, this.wSp.y),
        pitch: pTerms(eSp.pitch * DEG, eNow.pitch * DEG, err.z, raw.z, this.wSp.z),
      };
      // Collective thrust: desired force projected on the current body up-axis.
      this.thrust = clamp(dot(this.fDes, qRotate(s.q, up)), 0, 4 * fmax);
    }

    // 4. Rate stage → torque → mixer.
    if (due(c.hzRate)) {
      const s = sense();
      const tau = this.rate.torque(this.wSp, s, dtOf(c.hzRate), P);
      const r = mix(this.thrust, tau.x, tau.y, tau.z, P.drone.torqueCoeff, fmax);
      this.mixerSaturated = r.saturated;
      this.rate.applied(r.motors);
      this.comp.applied(r.motors);
      this.held = { motorCmd: r.motors, forceCmd: this.held.forceCmd };
    }

    return this.held;
  }

  loops(): Record<string, LoopTerms> {
    return {
      'pos.x': pidLoop(this.pos.x.last),
      'pos.y': pidLoop(this.pos.y.last),
      'pos.z': pidLoop(this.pos.z.last),
      'vel.x': pidLoop(this.vel.x.last),
      'vel.y': pidLoop(this.vel.y.last),
      'vel.z': pidLoop(this.vel.z.last),
      'att.roll': pidLoop(this.att.roll),
      'att.pitch': pidLoop(this.att.pitch),
      'att.yaw': pidLoop(this.att.yaw),
      ...this.rate.loops(),
      ...this.comp.loops(this.aSp),
    };
  }

  extras(): Record<string, number> {
    const tilt = Math.acos(clamp(this.fDes.y / Math.max(length(this.fDes), 1e-9), -1, 1)) * DEG;
    return {
      'fdes.x': this.fDes.x,
      'fdes.y': this.fDes.y,
      'fdes.z': this.fDes.z,
      'fdes.tilt': tilt,
      thrust: this.thrust,
      mixerSat: this.mixerSaturated ? 1 : 0,
      ...this.rate.extras(),
      ...this.comp.extras(),
    };
  }

  describe(p: Params, loopId: string): InfoRow[] {
    return [...this.rate.describe(p, loopId), ...this.comp.describe(p, loopId)];
  }

  /** Desired thrust vector, world frame — drawn as an arrow. */
  get desiredForce(): Vec3 {
    return this.fDes;
  }
}
