import type { Mat } from '@/math/mat';
import { QpSolver } from '@/math/qp';
import {
  cwDiscrete,
  meanMotion,
  type ChaserInput,
  type ChaserState,
  type RendezvousParams,
} from '@/sim/vehicles/chaser';

/**
 * Model-predictive control of the rendezvous (lesson IV.25). Every `mpcStep` orbital seconds it
 * plans the thrust over the next `mpcHorizon` steps with the Clohessy–Wiltshire model, within the
 * thrusters' limit, and applies the first step of the plan:
 *
 *   minimise Σₖ (zₖᵀ·Q·zₖ + aₖᵀ·R·aₖ)   subject to   z₊ = A·z + B·a,  |a| ≤ a_max
 *
 * The state is condensed out (z is a linear function of z₀ and the plan), leaving a quadratic
 * program in the 2N accelerations with box constraints, solved by the ADMM of src/math/qp.ts. The
 * unknowns are the accelerations as fractions of the limit, so the box is ±1 whatever the limit.
 * Because the model knows the orbit, the plan uses it: it lets the chaser drift where the orbit
 * carries it and pushes only for the difference.
 */
export class RendezvousMpc {
  private solver = new QpSolver();
  private model: {
    key: string;
    /** z_k = Φ_k·z₀ + Γ_k·u, stacked over k = 1…N. */
    phi: Mat[];
    gam: Mat[];
    hess: Mat;
    a: Mat;
  } | null = null;
  private next = 0;
  private hold: ChaserInput = { ax: 0, ay: 0 };
  /** The planned path of the last solve, for the chart (x, y per step). */
  plan: { x: number; y: number }[] = [];

  reset(): void {
    this.solver.reset();
    this.next = 0;
    this.hold = { ax: 0, ay: 0 };
    this.plan = [];
  }

  private build(p: RendezvousParams) {
    const key = JSON.stringify([
      p.periodMin,
      p.mpcStep,
      p.mpcHorizon,
      p.qPos,
      p.qVel,
      p.rAcc,
      p.accelMax,
    ]);
    if (this.model?.key === key) return this.model;
    const n = meanMotion(p);
    const { a, b } = cwDiscrete(n, p.mpcStep);
    const N = p.mpcHorizon;
    const nu = 2 * N;
    const mm = (x: Mat, y: Mat): Mat =>
      x.map((r) => y[0]!.map((_, j) => r.reduce((s, v, l) => s + v * y[l]![j]!, 0)));
    const phi: Mat[] = [];
    const gam: Mat[] = [];
    let ak: Mat = a.map((r) => [...r]);
    for (let k = 0; k < N; k++) {
      phi.push(ak);
      ak = mm(a, ak);
    }
    // Γ_k: column block j (input at step j ≤ k) is A^(k−j)·B.
    const powers: Mat[] = [a.map((_, i) => a.map((__, j) => (i === j ? 1 : 0)))];
    for (let k = 1; k < N; k++) powers.push(mm(a, powers[k - 1]!));
    for (let k = 0; k < N; k++) {
      const g: Mat = Array.from({ length: 4 }, () => new Array<number>(nu).fill(0));
      for (let j = 0; j <= k; j++) {
        const blk = mm(powers[k - j]!, b).map((row) => row.map((v) => v * p.accelMax));
        for (let i = 0; i < 4; i++) {
          g[i]![2 * j] = blk[i]![0]!;
          g[i]![2 * j + 1] = blk[i]![1]!;
        }
      }
      gam.push(g);
    }
    const q = [p.qPos, p.qPos, p.qVel, p.qVel];
    const hess: Mat = Array.from({ length: nu }, (_, i) =>
      Array.from({ length: nu }, (__, j) => (i === j ? p.rAcc * p.accelMax ** 2 : 0)),
    );
    for (let k = 0; k < N; k++) {
      // The last step counts ten times: it stands for everything after the horizon.
      const w = k === N - 1 ? 10 : 1;
      const g = gam[k]!;
      for (let i = 0; i < nu; i++)
        for (let j = 0; j < nu; j++) {
          let s = 0;
          for (let r = 0; r < 4; r++) s += g[r]![i]! * q[r]! * g[r]![j]!;
          hess[i]![j]! += w * s;
        }
    }
    const box: Mat = Array.from({ length: nu }, (_, i) =>
      Array.from({ length: nu }, (__, j) => (i === j ? 1 : 0)),
    );
    this.model = { key, phi, gam, hess, a: box };
    this.solver.reset();
    return this.model;
  }

  /** The thrust for this instant (orbital time `s.t`): re-planned every `mpcStep`. */
  tick(s: ChaserState, p: RendezvousParams): ChaserInput {
    if (s.t + 1e-9 < this.next) return this.hold;
    this.next = s.t + p.mpcStep;
    const m = this.build(p);
    const N = p.mpcHorizon;
    const nu = 2 * N;
    const z0 = [s.x, s.y, s.vx, s.vy];
    const q = [p.qPos, p.qPos, p.qVel, p.qVel];
    const lin = new Array<number>(nu).fill(0);
    const free: number[][] = [];
    for (let k = 0; k < N; k++) {
      const w = k === N - 1 ? 10 : 1;
      const f = m.phi[k]!.map((r) => r.reduce((acc, v, j) => acc + v * z0[j]!, 0));
      free.push(f);
      const g = m.gam[k]!;
      for (let i = 0; i < nu; i++) {
        let acc = 0;
        for (let r = 0; r < 4; r++) acc += g[r]![i]! * q[r]! * f[r]!;
        lin[i]! += w * acc;
      }
    }
    const one = new Array<number>(nu).fill(1);
    const r = this.solver.solve({ p: m.hess, q: lin, a: m.a, l: one.map((v) => -v), u: one }, m);
    // Back to accelerations, m/s² (Γ already carries the limit).
    const u = r.x.map((v) => Math.max(-1, Math.min(1, v)));
    this.plan = free.map((f, k) => {
      const g = m.gam[k]!;
      const at = (row: number) => f[row]! + g[row]!.reduce((acc, v, j) => acc + v * u[j]!, 0);
      return { x: at(0), y: at(1) };
    });
    this.hold = { ax: u[0]! * p.accelMax, ay: u[1]! * p.accelMax };
    return this.hold;
  }
}
