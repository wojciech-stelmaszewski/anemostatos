import type { Mat } from './mat';

/**
 * Small nonlinear programs (docs/aerospace-gnc.md §3.2), for trajectory optimisation:
 *   minimise f(x)   subject to   c_E(x) = 0,   c_I(x) ≥ 0
 * by the augmented Lagrangian method (Nocedal & Wright 2006, §17.4; the PHR form for the
 * inequalities):
 *   L_A = f + Σ (λᵢcᵢ + ρ/2·cᵢ²)  +  Σ (max(0, μⱼ − ρcⱼ)² − μⱼ²)/(2ρ)
 * minimised over x by BFGS, then λ ← λ + ρc_E, μ ← max(0, μ − ρc_I), and ρ grows when the
 * constraints are not shrinking fast enough. Dense, deterministic, with analytic first
 * derivatives supplied by the problem.
 */
export interface NlpEval {
  f: number;
  grad: number[];
  /** Equality constraints and their Jacobian (rows = constraints). */
  ce: number[];
  je: Mat;
  /** Inequality constraints c ≥ 0 and their Jacobian. */
  ci: number[];
  ji: Mat;
}

export interface NlpOptions {
  /** Largest constraint violation accepted at the end. */
  tol?: number;
  maxOuter?: number;
  maxInner?: number;
  rho?: number;
}

export interface NlpResult {
  x: number[];
  status: 'converged' | 'max-iterations';
  f: number;
  /** Largest violation of any constraint. */
  violation: number;
  outer: number;
  /** BFGS iterations over all outer steps. */
  inner: number;
}

const dot = (a: readonly number[], b: readonly number[]) => {
  let s = 0;
  for (let i = 0; i < a.length; i++) s += a[i]! * b[i]!;
  return s;
};

export const violation = (e: NlpEval): number =>
  Math.max(0, ...e.ce.map(Math.abs), ...e.ci.map((c) => -c));

export function solveNlp(
  evaluate: (x: readonly number[]) => NlpEval,
  x0: readonly number[],
  opts: NlpOptions = {},
): NlpResult {
  const tol = opts.tol ?? 1e-6;
  const maxOuter = opts.maxOuter ?? 30;
  const maxInner = opts.maxInner ?? 300;
  const n = x0.length;
  let x = [...x0];
  let e = evaluate(x);
  let lam = new Array<number>(e.ce.length).fill(0);
  let mu = new Array<number>(e.ci.length).fill(0);
  let rho = opts.rho ?? 10;
  let inner = 0;
  let prevViol = violation(e);

  // The augmented Lagrangian and its gradient at an evaluation.
  const merit = (ev: NlpEval) => {
    let v = ev.f;
    const g = [...ev.grad];
    ev.ce.forEach((c, i) => {
      v += lam[i]! * c + 0.5 * rho * c * c;
      const w = lam[i]! + rho * c;
      const row = ev.je[i]!;
      for (let k = 0; k < n; k++) g[k]! += w * row[k]!;
    });
    ev.ci.forEach((c, j) => {
      const s = Math.max(0, mu[j]! - rho * c);
      v += (s * s - mu[j]! * mu[j]!) / (2 * rho);
      if (s > 0) {
        const row = ev.ji[j]!;
        for (let k = 0; k < n; k++) g[k]! -= s * row[k]!;
      }
    });
    return { v, g };
  };

  let outer = 0;
  for (; outer < maxOuter; outer++) {
    // BFGS on L_A with the multipliers held, from the identity scaled at the first step.
    let { v, g } = merit(e);
    let hInv: Mat | null = null;
    const gTol = Math.max(1e-8, 1e-3 / (outer + 1) ** 2);
    for (let it = 0; it < maxInner; it++) {
      if (Math.max(...g.map(Math.abs)) < gTol) break;
      inner++;
      const d = hInv
        ? hInv.map((row) => -dot(row, g))
        : g.map((gi) => -gi / Math.max(1, Math.sqrt(dot(g, g))));
      const slope = dot(g, d);
      if (!(slope < 0)) {
        hInv = null;
        continue;
      }
      let step = 1;
      let xn: number[] = [];
      let en: NlpEval | null = null;
      let mn = { v: Infinity, g };
      for (let ls = 0; ls < 40; ls++) {
        xn = x.map((xi, i) => xi + step * d[i]!);
        en = evaluate(xn);
        mn = merit(en);
        if (mn.v <= v + 1e-4 * step * slope) break;
        step *= 0.5;
      }
      if (!en || !(mn.v <= v + 1e-4 * step * slope)) break;
      const s = xn.map((xi, i) => xi - x[i]!);
      const y = mn.g.map((gi, i) => gi - g[i]!);
      const sy = dot(s, y);
      if (sy > 1e-12) {
        if (!hInv) {
          // First update: scale the identity to the curvature seen (Nocedal & Wright 6.20).
          const k = sy / dot(y, y);
          hInv = Array.from({ length: n }, (_, i) =>
            Array.from({ length: n }, (_, j) => (i === j ? k : 0)),
          );
        }
        // H ← (I − ρsyᵀ)H(I − ρysᵀ) + ρssᵀ, ρ = 1/sᵀy.
        const r = 1 / sy;
        const hy = hInv.map((row) => dot(row, y));
        const yhy = dot(y, hy);
        for (let i = 0; i < n; i++) {
          const row = hInv[i]!;
          for (let j = 0; j < n; j++)
            row[j]! += -r * (s[i]! * hy[j]! + hy[i]! * s[j]!) + (r * r * yhy + r) * s[i]! * s[j]!;
        }
      }
      x = xn;
      e = en;
      v = mn.v;
      g = mn.g;
    }
    const viol = violation(e);
    if (viol < tol && Math.max(...g.map(Math.abs)) < 1e-4) break;
    lam = lam.map((l, i) => l + rho * e.ce[i]!);
    mu = mu.map((m, j) => Math.max(0, m - rho * e.ci[j]!));
    if (viol > 0.25 * prevViol) rho = Math.min(rho * 10, 1e8);
    prevViol = viol;
  }
  const viol = violation(e);
  return {
    x,
    status: viol < tol ? 'converged' : 'max-iterations',
    f: e.f,
    violation: viol,
    outer: outer + 1,
    inner,
  };
}
