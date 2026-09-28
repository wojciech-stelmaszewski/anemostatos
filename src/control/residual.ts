import { LowPass2 } from '@/estimation/filters';
import { qRotate } from '@/math/quat';
import { add, dot, scale, sub, v3, type Vec3 } from '@/math/vec3';
import { GRAVITY, type Params } from '@/sim/params';
import type { Measurement } from '@/sim/sensors';
import type { Compensation } from './stages';
import type { InfoRow, LoopTerms } from './types';

/**
 * Features of the aerodynamic residual on one world axis, from the state alone (the wind is not
 * measured): a constant (steady wind, mass error), velocity, |v|·v (quadratic drag) and thrust ×
 * in-plane velocity (rotor drag). Hand-crafted here; Neural-Fly (O'Connell et al. 2022) learns
 * such a basis offline with a neural network so that, across wind conditions, only the linear
 * coefficients need to change.
 */
export const FEATURES = ['1', 'v', '|v|·v', 'T·v⊥'] as const;

const features = (axis: 'x' | 'y' | 'z', v: Vec3, thrust: number, up: Vec3): number[] => {
  const speed = Math.hypot(v.x, v.y, v.z);
  const vPerp = sub(v, scale(up, dot(v, up)));
  return [1, v[axis], speed * v[axis], thrust * vPerp[axis]];
};

const P_MAX = 100;

/** Recursive least squares with exponential forgetting, for one axis. */
class Rls {
  theta: number[];
  private p: number[][];
  constructor(private n: number) {
    this.theta = new Array<number>(n).fill(0);
    this.p = Array.from({ length: n }, (_, i) =>
      Array.from({ length: n }, (_, j) => (i === j ? 100 : 0)),
    );
  }
  reset(): void {
    this.theta.fill(0);
    this.p = Array.from({ length: this.n }, (_, i) =>
      Array.from({ length: this.n }, (_, j) => (i === j ? 100 : 0)),
    );
  }
  predict(phi: number[]): number {
    return phi.reduce((s, f, i) => s + f * this.theta[i]!, 0);
  }
  update(phi: number[], y: number, forget: number): void {
    const n = this.n;
    const pphi = this.p.map((r) => r.reduce((s, v, j) => s + v * phi[j]!, 0));
    const denom = forget + phi.reduce((s, f, i) => s + f * pphi[i]!, 0);
    const k = pphi.map((v) => v / denom);
    const err = y - this.predict(phi);
    for (let i = 0; i < n; i++) this.theta[i]! += k[i]! * err;
    // P ← (P − K·φᵀP) / λ. Without excitation (hovering) the division by λ would inflate P
    // without bound ("covariance windup") and make the estimate jumpy: cap its diagonal.
    const phiP = Array.from({ length: n }, (_, j) =>
      phi.reduce((s, f, i) => s + f * this.p[i]![j]!, 0),
    );
    for (let i = 0; i < n; i++)
      for (let j = 0; j < n; j++) this.p[i]![j] = (this.p[i]![j]! - k[i]! * phiP[j]!) / forget;
    for (let i = 0; i < n; i++) {
      const d = this.p[i]![i]!;
      if (d > P_MAX) {
        const f = Math.sqrt(P_MAX / d);
        for (let j = 0; j < n; j++) {
          this.p[i]![j]! *= f;
          this.p[j]![i]! *= f;
        }
      }
    }
  }
}

/**
 * L3 compensation: learn the residual force as Φ(x)·θ and cancel it.
 *   measured residual  y = m̂·a − F₀ − m̂·g           (accelerometer, filtered)
 *   F = m̂·(a_sp + g) − Φ(x)·θ
 * Unlike an integrator (which only holds a constant), the fitted model predicts how the
 * disturbance changes with speed — useful when flying a trajectory through wind.
 */
export class LearnedResidual implements Compensation {
  readonly replacesIntegral = false;
  private rls = {
    x: new Rls(FEATURES.length),
    y: new Rls(FEATURES.length),
    z: new Rls(FEATURES.length),
  };
  private aFilt = [new LowPass2(10, 0.001), new LowPass2(10, 0.001), new LowPass2(10, 0.001)];
  private fFilt = [new LowPass2(10, 0.001), new LowPass2(10, 0.001), new LowPass2(10, 0.001)];
  private cmd: [number, number, number, number] = [0, 0, 0, 0];
  private model: [number, number, number, number] = [0, 0, 0, 0];
  private predicted = v3();
  private measured = v3();
  private phi: Record<'x' | 'y' | 'z', number[]> = { x: [], y: [], z: [] };
  private decimate = 0;

  reset(): void {
    for (const r of Object.values(this.rls)) r.reset();
    for (const f of [...this.aFilt, ...this.fFilt]) f.reset();
    this.cmd = [0, 0, 0, 0];
    this.model = [0, 0, 0, 0];
    this.predicted = v3();
    this.measured = v3();
    this.decimate = 0;
  }

  applied(motors: [number, number, number, number]): void {
    this.cmd = [...motors];
  }

  sample(s: Measurement, p: Params, dt: number): void {
    const c = p.control.residual;
    const mHat = Math.max(p.control.model.mass, 0.05);
    const a = 1 - Math.exp(-dt / Math.max(p.control.model.motorTau, 1e-4));
    for (let i = 0; i < 4; i++)
      this.model[i] = this.model[i]! + (this.cmd[i]! - this.model[i]!) * a;
    const r = s.motors ?? this.model;
    const thrust = r[0] + r[1] + r[2] + r[3];
    const up = qRotate(s.q, v3(0, 1, 0));
    const f0 = scale(up, thrust);
    const acc = add(qRotate(s.q, s.acc), v3(0, -GRAVITY, 0));
    const set = (f: LowPass2[], v: Vec3) => {
      for (const x of f) {
        x.cutoffHz = c.filterHz;
        x.dt = dt;
      }
      return v3(f[0]!.update(v.x), f[1]!.update(v.y), f[2]!.update(v.z));
    };
    const aF = set(this.aFilt, acc);
    const fF = set(this.fFilt, f0);
    this.measured = sub(scale(aF, mHat), add(fF, v3(0, -mHat * GRAVITY, 0)));
    for (const k of ['x', 'y', 'z'] as const) this.phi[k] = features(k, s.vel, thrust, up);
    // Fit at 100 Hz: plenty for a slowly varying model, and cheap.
    if (++this.decimate % 10 === 0)
      for (const k of ['x', 'y', 'z'] as const)
        this.rls[k].update(this.phi[k], this.measured[k], c.forgetting);
    this.predicted = v3(
      this.rls.x.predict(this.phi.x),
      this.rls.y.predict(this.phi.y),
      this.rls.z.predict(this.phi.z),
    );
  }

  force(aSp: Vec3, p: Params): Vec3 {
    const m = p.control.model.mass;
    const f = this.predicted;
    return v3(m * aSp.x - f.x, m * (aSp.y + GRAVITY) - f.y, m * aSp.z - f.z);
  }

  loops(): Record<string, LoopTerms> {
    return {};
  }

  extras(): Record<string, number> {
    const out: Record<string, number> = {
      'est.dist.x': this.predicted.x,
      'est.dist.y': this.predicted.y,
      'est.dist.z': this.predicted.z,
    };
    for (const k of ['x', 'z'] as const)
      this.rls[k].theta.forEach((t, i) => (out[`residual.${k}.${FEATURES[i]}`] = t));
    return out;
  }

  describe(p: Params, loopId: string): InfoRow[] {
    if (!loopId.startsWith('pos.') && !loopId.startsWith('vel.')) return [];
    const th = this.rls.x.theta;
    return [
      {
        label: 'model (x axis)',
        value: FEATURES.map((f, i) => `${th[i]!.toFixed(2)}·${f}`).join(' + '),
        hint: 'The fitted residual force on the x axis. Its coefficients keep adapting (forgetting factor) as the wind changes.',
      },
      { label: 'forgetting λ', value: `${p.control.residual.forgetting}` },
    ];
  }
}
