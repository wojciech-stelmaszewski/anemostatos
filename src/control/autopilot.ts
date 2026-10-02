import { GRAVITY, type Params } from '@/sim/params';
import { atmosphere, dynamicPressure } from '@/sim/atmosphere';
import {
  thrustAt,
  trimAircraft,
  type AircraftInput,
  type AircraftParams,
  type AircraftState,
  type Trim,
} from '@/sim/vehicles/aircraft';
import type { LoopTerms } from './types';

/**
 * The pitch autopilot of Chapter M (docs/aerospace-gnc.md §4):
 *
 *   δe = δe_trim − G·(kθ·(θ_ref − θ) − q)
 *
 * With kθ = 0 it is a pure pitch damper (lesson IV.12); with kθ > 0 a pitch-attitude hold whose
 * inner loop is the damper (IV.13). G is the loop gain, elevator radians per rad/s of pitch-rate
 * error. A schedule on the dynamic pressure divides it by q̄/q̄_design, since the elevator's
 * moment grows with q̄ (IV.13). An auto-throttle holds a speed reference that moves toward its
 * target at a fixed rate; the trim elevator and attitude are taken at that reference speed.
 */
export interface AutopilotParams {
  /** Loop gain G, s (elevator ° per °/s of pitch-rate error). 0 flies the bare airframe. */
  gain: number;
  /** Attitude gain kθ, 1/s: 0 makes the law a pure pitch damper. */
  kTheta: number;
  /** Divide the gain by q̄/q̄(designSpeed) (`qbar`), or keep it fixed (`none`). */
  schedule: 'none' | 'qbar';
  /** The speed at which the scheduled gain equals `gain`, m/s, at the start altitude. */
  designSpeed: number;
  /** Added by lesson scripts: an elevator pulse, °, and a pitch-attitude step, °. */
  elevatorOffset: number;
  thetaOffset: number;
  /** Auto-throttle: hold a speed reference that moves to `speedTarget` at `speedRate`, m/s². */
  autothrottle: boolean;
  speedTarget: number;
  speedRate: number;
  /** Throttle per m/s of speed error. */
  speedGain: number;
  /** Delay between the sensors and the elevator command (filtering and computation), ms. */
  delayMs: number;
  /**
   * Outer loops (lesson IV.17), which then own the throttle and the pitch reference:
   * `separate` holds the speed with the throttle and the altitude with the pitch;
   * `tecs` controls the total energy with the throttle and its distribution with the pitch.
   */
  outer: 'none' | 'separate' | 'tecs';
  /** The altitude reference above the start altitude, m (lesson scripts move it). */
  altitudeOffset: number;
  /** Flight-path command per metre of altitude error: γ = climbGain·Δh/V, 1/s. */
  climbGain: number;
  /** Largest flight-path command, °, and how fast the command may change, °/s. */
  gammaMax: number;
  gammaRate: number;
  /** Separate loops: throttle per m of integrated speed error (with `speedGain`, a PI). */
  speedInt: number;
  /** Total energy control: the speed error asks for V̇ = speedLoop·ΔV, 1/s. */
  speedLoop: number;
  /** Total energy control: thrust gains on the energy-rate error (P, I) and pitch gains on the distribution error (P, I). */
  kTP: number;
  kTI: number;
  kEP: number;
  kEI: number;
}

const RAD = Math.PI / 180;
const DEG = 180 / Math.PI;

/** The gain the law uses at dynamic pressure `qbar`. */
export function scheduledGain(ap: AutopilotParams, a: AircraftParams, qbar: number): number {
  if (ap.schedule !== 'qbar') return ap.gain;
  const qDesign = dynamicPressure(atmosphere(a.altitude).density, ap.designSpeed);
  return (ap.gain * qDesign) / qbar;
}

export class Autopilot {
  /** The speed reference of the auto-throttle, m/s. */
  vRef = NaN;
  last: LoopTerms = emptyTerms();
  private trim: Trim | null = null;
  private trimV = NaN;
  /** The pitch attitude and rate the law sees, delayed by `delayMs` (a ring of 1 ms samples). */
  private ring: { theta: number; q: number }[] = [];
  private head = 0;
  /** The outer loops: the rate-limited flight-path command, rad, the last speed, integrators. */
  gammaRef = 0;
  private vPrev = NaN;
  private iV = 0;
  private iT = 0;
  private iD = 0;

  reset(p: Params): void {
    this.vRef = p.aircraft.speed;
    this.trim = null;
    this.trimV = NaN;
    this.last = emptyTerms();
    this.ring = [];
    this.head = 0;
    this.gammaRef = 0;
    this.vPrev = NaN;
    this.iV = 0;
    this.iT = 0;
    this.iD = 0;
  }

  /** The measurement `delayMs` ago (the oldest sample until the ring has filled). */
  private delayed(s: AircraftState, ap: AutopilotParams, dt: number): { theta: number; q: number } {
    const n = Math.max(0, Math.round(ap.delayMs / 1000 / dt));
    if (n === 0) return { theta: s.theta, q: s.q };
    if (this.ring.length !== n) {
      this.ring = Array.from({ length: n }, () => ({ theta: s.theta, q: s.q }));
      this.head = 0;
    }
    const out = this.ring[this.head]!;
    this.ring[this.head] = { theta: s.theta, q: s.q };
    this.head = (this.head + 1) % n;
    return out;
  }

  /** Level-flight trim at the reference speed, recomputed when it has moved by 0.05 m/s. */
  private trimAt(a: AircraftParams, V: number): Trim {
    if (!this.trim || Math.abs(V - this.trimV) > 0.05) {
      this.trim = trimAircraft(a, V, a.altitude);
      this.trimV = V;
    }
    return this.trim;
  }

  tick(s: AircraftState, p: Params, dt: number): AircraftInput {
    const a = p.aircraft;
    const ap = p.autopilot;
    if (!Number.isFinite(this.vRef)) this.vRef = a.speed;
    const ownsSpeed = ap.autothrottle || ap.outer !== 'none';
    if (ownsSpeed) {
      const step = ap.speedRate * dt;
      this.vRef += Math.min(Math.max(ap.speedTarget - this.vRef, -step), step);
    }
    const trim = this.trimAt(a, ownsSpeed ? this.vRef : a.speed);
    const qbar = dynamicPressure(atmosphere(s.h).density, s.V);
    const g = scheduledGain(ap, a, qbar);
    let thetaRef = trim.state.theta + ap.thetaOffset * RAD;
    let throttle = ap.autothrottle
      ? trim.input.throttle + ap.speedGain * (this.vRef - s.V)
      : trim.input.throttle;
    if (ap.outer !== 'none') {
      const outer = this.outer(s, p, trim, dt);
      thetaRef += outer.dTheta;
      throttle = outer.throttle;
    }
    const m = this.delayed(s, ap, dt);
    const e = thetaRef - m.theta;
    const attitude = -g * ap.kTheta * e;
    const damper = g * m.q;
    const ff = trim.input.de + ap.elevatorOffset * RAD;
    const cmd = ff + attitude + damper;
    const lim = a.elevatorMax * RAD;
    const out = Math.min(Math.max(cmd, -lim), lim);
    this.last = {
      setpoint: thetaRef * DEG,
      measurement: m.theta * DEG,
      error: e * DEG,
      parts: [
        { key: 'att', label: 'attitude', value: attitude * DEG, like: 'p' },
        { key: 'damp', label: 'damper', value: damper * DEG, like: 'd' },
        { key: 'ff', label: 'trim', value: ff * DEG, ff: true },
      ],
      unsaturated: cmd * DEG,
      output: out * DEG,
      saturated: cmd !== out,
    };
    return { de: out, throttle: Math.min(Math.max(throttle, 0), 1) };
  }

  /**
   * The outer loops of lesson IV.17. Both fly the same rate-limited flight-path command towards
   * the altitude reference. `separate`: a PI on speed moves the throttle, the pitch follows the
   * flight-path command. `tecs` [Lambregts 1983]: the specific energy rate γ + V̇/g is the
   * throttle's job, with the thrust the command needs as feedforward; the distribution γ − V̇/g
   * is the elevator's, through the pitch reference.
   */
  private outer(
    s: AircraftState,
    p: Params,
    trim: Trim,
    dt: number,
  ): { dTheta: number; throttle: number } {
    const a = p.aircraft;
    const ap = p.autopilot;
    const href = a.altitude + ap.altitudeOffset;
    const gMax = ap.gammaMax * RAD;
    const cmd = Math.min(Math.max((ap.climbGain * (href - s.h)) / s.V, -gMax), gMax);
    const rate = ap.gammaRate * RAD * dt;
    this.gammaRef += Math.min(Math.max(cmd - this.gammaRef, -rate), rate);
    const vdot = Number.isFinite(this.vPrev) ? (s.V - this.vPrev) / dt : 0;
    this.vPrev = s.V;
    const ev = this.vRef - s.V;
    const clamp = (v: number) => Math.min(Math.max(v, 0), 1);
    if (ap.outer === 'separate') {
      const throttle = trim.input.throttle + ap.speedGain * ev + ap.speedInt * this.iV;
      if (throttle === clamp(throttle)) this.iV += ev * dt;
      return { dTheta: this.gammaRef, throttle };
    }
    const gamma = s.theta - s.alpha;
    const vdotRef = ap.speedLoop * ev;
    const eT = this.gammaRef - gamma + (vdotRef - vdot) / GRAVITY;
    const eD = this.gammaRef - gamma - (vdotRef - vdot) / GRAVITY;
    const weight = a.mass * GRAVITY;
    const dT = weight * (this.gammaRef + vdotRef / GRAVITY + ap.kTP * eT + ap.kTI * this.iT);
    const throttle = trim.input.throttle + dT / thrustAt(1, s.h, a);
    if (throttle === clamp(throttle)) this.iT += eT * dt;
    this.iD += eD * dt;
    return { dTheta: this.gammaRef + ap.kEP * eD + ap.kEI * this.iD, throttle };
  }
}

function emptyTerms(): LoopTerms {
  return {
    setpoint: 0,
    measurement: 0,
    error: 0,
    parts: [
      { key: 'att', label: 'attitude', value: 0, like: 'p' },
      { key: 'damp', label: 'damper', value: 0, like: 'd' },
      { key: 'ff', label: 'trim', value: 0, ff: true },
    ],
    unsaturated: 0,
    output: 0,
    saturated: false,
  };
}
