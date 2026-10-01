import { c2d, diag, eigenvalues, mul, sub, toContinuous, transpose, type Mat } from '@/math/mat';
import { dlqr } from '@/math/riccati';
import { clamp } from '@/math/util';
import { idleActuation, type Actuation } from '@/sim/dynamics';
import { GRAVITY, type Params } from '@/sim/params';
import {
  periodSteps,
  type ControlInput,
  type Controller,
  type InfoRow,
  type LoopTerms,
} from './types';

interface Design {
  key: string;
  /** Gains in state order: e, v, [T_lag], [ξ]. */
  k: number[];
  names: string[];
  /** Closed-loop poles, continuous-time equivalent. */
  poles: { re: number; im: number }[];
  converged: boolean;
}

/**
 * The L1 plant as a linear state-space model, from the controller's point of view:
 *   ė = v,  m̂·v̇ = u                    (u = thrust minus the gravity feedforward)
 *   optionally τ̂·Ṫ = u − T, m̂·v̇ = T    (motor lag as a state)
 *   optionally ξ̇ = e                     (integral of the error, LQI)
 */
export function designLqr(p: Params, dt: number): Design {
  const c = p.control.lqr;
  const m = Math.max(p.control.model.mass, 0.05);
  const tau = Math.max(p.control.model.motorTau, 1e-3);
  const key = [c.qPos, c.qVel, c.qInt, c.r, c.integral, c.lagState, m, tau, dt].join('|');
  const names = ['e', 'v'];
  const q = [c.qPos, c.qVel];
  if (c.lagState) {
    names.push('T');
    q.push(0);
  }
  if (c.integral) {
    names.push('ξ');
    q.push(c.qInt);
  }
  const n = names.length;
  const a: Mat = Array.from({ length: n }, () => new Array<number>(n).fill(0));
  const b: Mat = Array.from({ length: n }, () => [0]);
  a[0]![1] = 1;
  if (c.lagState) {
    a[1]![2] = 1 / m;
    a[2]![2] = -1 / tau;
    b[2]![0] = 1 / tau;
  } else {
    b[1]![0] = 1 / m;
  }
  if (c.integral) a[n - 1]![0] = 1;
  const d = c2d(a, b, dt);
  const res = dlqr(d.a, d.b, diag(q.map((w) => w * dt)), [[Math.max(c.r, 1e-9) * dt]], 20000);
  const k = res.k[0]!;
  const poles = eigenvalues(sub(d.a, mul(d.b, res.k))).map((z) => toContinuous(z, dt));
  return { key, k, names, poles, converged: res.converged };
}

/**
 * The steady-state Kalman filter of the LQG option and the matrices of the loop it closes, at
 * controller period T. The model is the LQR's own, in absolute altitude: x = (y, v, [T]).
 *   x̂ = x̂⁻ + M·(y − x̂⁻₁)            correction with the altimeter (gain M)
 *   u = −K·(x̂ − (r, 0, …))          the LQR on the estimate
 *   x̂⁻⁺ = A·x̂ + B·u                 prediction with the command just sent
 * The filter assumes process noise of σ = kfQ newtons entering with the thrust, and altimeter
 * noise of σ = kfR metres. More process noise: a faster filter that trusts the model less.
 */
export function lqgMatrices(p: Params, T: number): LqgMatrices {
  const c = p.control.lqr;
  const key = JSON.stringify([c, p.control.model.mass, p.control.model.motorTau, T]);
  const hit = lqgCache.get(key);
  if (hit) return hit;
  const out = lqgDesign(p, T);
  if (lqgCache.size > 64) lqgCache.clear();
  lqgCache.set(key, out);
  return out;
}

interface LqgMatrices {
  abk: Mat;
  m: number[];
  kk: number[];
  kr: { b: number[]; d: number };
  ad: Mat;
  bd: number[];
}
const lqgCache = new Map<string, LqgMatrices>();

function lqgDesign(p: Params, T: number): LqgMatrices {
  const c = p.control.lqr;
  const m = Math.max(p.control.model.mass, 0.05);
  const tau = Math.max(p.control.model.motorTau, 1e-3);
  const n = c.lagState ? 3 : 2;
  const a: Mat = Array.from({ length: n }, () => new Array<number>(n).fill(0));
  const b: Mat = Array.from({ length: n }, () => [0]);
  a[0]![1] = 1;
  if (c.lagState) {
    a[1]![2] = 1 / m;
    a[2]![2] = -1 / tau;
    b[2]![0] = 1 / tau;
  } else b[1]![0] = 1 / m;
  const d = c2d(a, b, T);
  const kk = designLqr({ ...p, control: { ...p.control, lqr: { ...c, integral: false } } }, T).k;
  // Kalman: the dual Riccati equation gives the predicted covariance P; M = P·Cᵀ/(C·P·Cᵀ + R).
  const qn = d.b.map((ri) => d.b.map((rj) => ri[0]! * rj[0]! * c.kfQ ** 2));
  for (let i = 0; i < n; i++) qn[i]![i] = qn[i]![i]! + 1e-12;
  const ct = Array.from({ length: n }, (_, i) => [i === 0 ? 1 : 0]);
  const pk = dlqr(transpose(d.a), ct, qn, [[Math.max(c.kfR, 1e-6) ** 2]], 200000).p;
  const s0 = pk[0]![0]! + Math.max(c.kfR, 1e-6) ** 2;
  const mGain = pk.map((row) => row[0]! / s0);
  // A − B·K, applied to the corrected estimate.
  const abk = d.a.map((row, i) => row.map((v, j) => v - d.b[i]![0]! * kk[j]!));
  return {
    abk,
    m: mGain,
    kk,
    /** How the reference enters: u gains k₁·r, the prediction B·k₁·r. */
    kr: { b: d.b.map((r) => r[0]! * kk[0]!), d: kk[0]! },
    ad: d.a,
    bd: d.b.map((r) => r[0]!),
  };
}

const fmtPole = (s: { re: number; im: number }) =>
  s.im === 0
    ? s.re.toFixed(2)
    : `${s.re.toFixed(2)} ${s.im > 0 ? '+' : '−'} ${Math.abs(s.im).toFixed(2)}j`;

/** Describe the dominant (slowest) complex pair or real pole as ωₙ and ζ. */
const dominant = (poles: { re: number; im: number }[]) => {
  const slow = [...poles].sort((x, y) => Math.hypot(x.re, x.im) - Math.hypot(y.re, y.im))[0]!;
  const wn = Math.hypot(slow.re, slow.im);
  return { wn, zeta: wn > 0 ? -slow.re / wn : 1 };
};

/**
 * Level 1, LQR / LQI: u = −K·x with K from the discrete Riccati equation for the chosen costs.
 * The gains are recomputed whenever a weight, the model or the sample time changes.
 */
export class LqrController implements Controller {
  private held: Actuation = idleActuation();
  private design: Design | null = null;
  /** ∫(y − y_sp) dt, LQI only. */
  private xi = 0;
  /** Model-based estimate of the actual thrust when motor telemetry is unavailable. */
  private thrustEst = 0;
  private lastCmd = 0;
  /** LQG: the filter's predicted state (y, v, [T]) for the next fix, or null before the first. */
  private xPred: number[] | null = null;
  private last: LoopTerms = {
    setpoint: 0,
    measurement: 0,
    error: 0,
    parts: this.partsFor(null, [0, 0, 0, 0], 0),
    unsaturated: 0,
    output: 0,
    saturated: false,
  };

  reset(): void {
    this.held = idleActuation();
    this.xi = 0;
    this.thrustEst = 0;
    this.lastCmd = 0;
    this.xPred = null;
    this.last = { ...this.last, parts: this.partsFor(null, [0, 0, 0, 0], 0) };
  }

  resetIntegrators(): void {
    this.xi = 0;
  }

  private gains(p: Params, dt: number): Design {
    const c = p.control.lqr;
    const key = [c.qPos, c.qVel, c.qInt, c.r, c.integral, c.lagState, p.control.model.mass]
      .concat([p.control.model.motorTau, dt])
      .join('|');
    if (this.design?.key !== key) this.design = designLqr(p, dt);
    return this.design;
  }

  private partsFor(p: Params | null, x: number[], ff: number) {
    const d = this.design;
    const k = d?.k ?? [0, 0, 0, 0];
    const parts: LoopTerms['parts'] = [
      { key: 'pos', label: '−k₁·e', value: -(k[0] ?? 0) * x[0]!, like: 'p' },
      { key: 'vel', label: '−k₂·v', value: -(k[1] ?? 0) * x[1]!, like: 'd' },
    ];
    const lag = p?.control.lqr.lagState ?? false;
    const integral = p?.control.lqr.integral ?? false;
    if (lag) parts.push({ key: 'lag', label: '−k₃·T', value: -(k[2] ?? 0) * x[2]! });
    if (integral)
      parts.push({ key: 'int', label: '−kᵢ·∫e', value: -(k[lag ? 3 : 2] ?? 0) * x[3]!, like: 'i' });
    parts.push({ key: 'ff', label: 'FF', value: ff, ff: true, like: 'ff' });
    return parts;
  }

  tick(input: ControlInput): Actuation {
    const { params: p, physDt, stepIndex } = input;
    const n = periodSteps(p.control.rateHz, physDt);
    if (stepIndex % n !== 0) return this.held;
    const dt = n * physDt;
    const m = input.sense();
    const d = this.gains(p, dt);
    const c = p.control.lqr;
    const mHat = p.control.model.mass;
    const ff = p.control.feedforward ? mHat * GRAVITY : 0;
    const tMax = 4 * p.drone.maxMotorThrust;

    // Actual thrust deviation: from motor telemetry if available, else a first-order motor model.
    const a = 1 - Math.exp(-dt / Math.max(p.control.model.motorTau, 1e-3));
    this.thrustEst += (this.lastCmd - this.thrustEst) * a;
    const thrust = m.motors
      ? m.motors[0] + m.motors[1] + m.motors[2] + m.motors[3]
      : this.thrustEst;

    const sp = input.setpoint.pos.y;
    let e = m.pos.y - sp; // state convention: deviation from the setpoint
    let x = [e, m.vel.y, thrust - ff, this.xi];
    if (c.observer && !c.integral) {
      // LQG: velocity and thrust come from the filter, which sees only the altimeter.
      const g = lqgMatrices(p, dt);
      const n = g.ad.length;
      const pred = this.xPred ?? [m.pos.y, 0, 0].slice(0, n);
      const innov = m.pos.y - pred[0]!;
      const xh = pred.map((v, i) => v + g.m[i]! * innov);
      e = xh[0]! - sp;
      x = [e, xh[1]!, n > 2 ? xh[2]! : 0, 0];
      const u = -g.kk.reduce((s, k, i) => s + k * (i === 0 ? e : xh[i]!), 0);
      // The prediction uses the command as the motors will get it: clamped like the output.
      const uSent = clamp(u + ff, 0, tMax) - ff;
      this.xPred = g.ad.map(
        (row, i) => row.reduce((s, a, j) => s + a * xh[j]!, 0) + g.bd[i]! * uSent,
      );
    }
    this.design = d;
    const parts = this.partsFor(p, x, ff);
    const unsaturated = parts.reduce((s, q) => s + q.value, 0);
    const output = clamp(unsaturated, 0, tMax);
    if (c.integral) {
      // Conditional integration, as in the PID: freeze while saturated and pushing deeper.
      const pushing = (unsaturated > tMax && e < 0) || (unsaturated < 0 && e > 0);
      if (!pushing) this.xi += e * dt;
    } else {
      this.xi = 0;
    }
    this.lastCmd = output;
    this.last = {
      setpoint: sp,
      measurement: m.pos.y,
      error: sp - m.pos.y,
      parts,
      unsaturated,
      output,
      saturated: output !== unsaturated,
    };
    const f = output / 4;
    this.held = { motorCmd: [f, f, f, f], forceCmd: this.held.forceCmd };
    return this.held;
  }

  loops() {
    return { alt: this.last };
  }

  extras() {
    const k = this.design?.k ?? [];
    return { 'lqr.k1': k[0] ?? NaN, 'lqr.k2': k[1] ?? NaN };
  }

  describe(p: Params): InfoRow[] {
    const dt = periodSteps(p.control.rateHz, 0.001) * 0.001;
    const d = this.gains(p, dt);
    const { wn, zeta } = dominant(d.poles);
    const rows: InfoRow[] = d.names.map((name, i) => ({
      label: `k for ${name}`,
      value: d.k[i]!.toFixed(3),
      hint:
        name === 'e'
          ? 'Acts like Kp: newtons per metre of error.'
          : name === 'v'
            ? 'Acts like Kd (D on measurement): newtons per m/s.'
            : name === 'ξ'
              ? 'Acts like Ki: newtons per metre·second of accumulated error.'
              : 'Feedback of the actual motor thrust: compensates the motor lag.',
    }));
    rows.push({
      label: 'poles (s-plane)',
      value: d.poles.map(fmtPole).join(', '),
      hint: 'Closed-loop poles of the model. Further left = faster; imaginary part = oscillation.',
    });
    rows.push({ label: 'dominant ωₙ · ζ', value: `${wn.toFixed(2)} rad/s · ${zeta.toFixed(2)}` });
    if (!d.converged) rows.push({ label: 'warning', value: 'Riccati did not converge' });
    return rows;
  }
}
