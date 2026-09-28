import { luFactor, luSolve, type Lu, type Mat } from './mat';

/**
 * Convex quadratic program
 *   minimise ½·xᵀPx + qᵀx   subject to   l ≤ A·x ≤ u
 * solved with the ADMM iteration of OSQP (Stellato et al. 2020), dense and small:
 *   (P + σI + ρAᵀA)·x̃ = σx − q + Aᵀ(ρz − y),   z̃ = A·x̃
 *   x ← αx̃ + (1−α)x,   z ← Π[l,u](αz̃ + (1−α)z + y/ρ),   y ← y + ρ(αz̃ + (1−α)z − z)
 * The matrix is factorised once per (P, A, ρ) and reused across iterations and calls; the solver
 * warm-starts from the previous solution. An iteration cap (not a time budget) keeps runs
 * deterministic.
 */
export interface QpProblem {
  p: Mat;
  q: number[];
  a: Mat;
  l: number[];
  u: number[];
}

export interface QpResult {
  x: number[];
  iterations: number;
  converged: boolean;
  /** Largest constraint violation and dual residual at the end. */
  primalResidual: number;
  dualResidual: number;
}

export class QpSolver {
  rho = 1;
  sigma = 1e-6;
  alpha = 1.6;
  maxIter = 200;
  epsAbs = 1e-4;
  epsRel = 1e-4;
  private x: number[] = [];
  private z: number[] = [];
  private y: number[] = [];
  private factor: { key: object; lu: Lu } | null = null;

  /** Forget the warm start (e.g. after a reset). */
  reset(): void {
    this.x = [];
    this.z = [];
    this.y = [];
  }

  /**
   * Solve. `matrixKey` identifies (P, A): pass the same object while they are unchanged so the
   * factorisation is reused; pass a new one when they change.
   */
  solve(pr: QpProblem, matrixKey: object = pr): QpResult {
    const n = pr.q.length;
    const m = pr.l.length;
    const { p, a, q, l, u } = pr;
    if (this.factor?.key !== matrixKey) {
      const k: Mat = Array.from({ length: n }, (_, i) =>
        Array.from({ length: n }, (_, j) => {
          let s = p[i]![j]! + (i === j ? this.sigma : 0);
          for (let r = 0; r < m; r++) s += this.rho * a[r]![i]! * a[r]![j]!;
          return s;
        }),
      );
      this.factor = { key: matrixKey, lu: luFactor(k) };
    }
    if (this.x.length !== n) this.x = new Array<number>(n).fill(0);
    if (this.z.length !== m) this.z = new Array<number>(m).fill(0);
    if (this.y.length !== m) this.y = new Array<number>(m).fill(0);
    let { x, z, y } = this;
    const { rho, sigma, alpha } = this;
    const ax = (v: number[]) => a.map((r) => r.reduce((s, c, j) => s + c * v[j]!, 0));
    const atv = (v: number[]) => {
      const out = new Array<number>(n).fill(0);
      for (let r = 0; r < m; r++) {
        const ar = a[r]!;
        const vr = v[r]!;
        if (vr !== 0) for (let j = 0; j < n; j++) out[j]! += ar[j]! * vr;
      }
      return out;
    };
    let it: number;
    let rPrim = Infinity;
    let rDual = Infinity;
    let converged = false;
    for (it = 1; it <= this.maxIter; it++) {
      const rhsA = atv(z.map((zi, i) => rho * zi - y[i]!));
      const xt = luSolve(
        this.factor.lu,
        x.map((xi, i) => sigma * xi - q[i]! + rhsA[i]!),
      );
      const zt = ax(xt);
      const zPrev = z;
      x = x.map((xi, i) => alpha * xt[i]! + (1 - alpha) * xi);
      const zRelax = zt.map((zti, i) => alpha * zti + (1 - alpha) * zPrev[i]!);
      z = zRelax.map((v, i) => Math.min(Math.max(v + y[i]! / rho, l[i]!), u[i]!));
      y = y.map((yi, i) => yi + rho * (zRelax[i]! - z[i]!));
      if (it % 5 === 0 || it === this.maxIter) {
        const axv = ax(x);
        rPrim = Math.max(0, ...axv.map((v, i) => Math.abs(v - z[i]!)));
        const px = p.map((r) => r.reduce((s, c, j) => s + c * x[j]!, 0));
        const aty = atv(y);
        rDual = Math.max(0, ...px.map((v, i) => Math.abs(v + q[i]! + aty[i]!)));
        const scaleP = Math.max(Math.max(0, ...axv.map(Math.abs)), Math.max(0, ...z.map(Math.abs)));
        const scaleD = Math.max(
          Math.max(0, ...px.map(Math.abs)),
          Math.max(0, ...aty.map(Math.abs)),
          Math.max(0, ...q.map(Math.abs)),
        );
        if (
          rPrim <= this.epsAbs + this.epsRel * scaleP &&
          rDual <= this.epsAbs + this.epsRel * scaleD
        ) {
          converged = true;
          break;
        }
      }
    }
    this.x = x;
    this.z = z;
    this.y = y;
    return {
      x: [...x],
      iterations: Math.min(it, this.maxIter),
      converged,
      primalResidual: rPrim,
      dualResidual: rDual,
    };
  }
}
