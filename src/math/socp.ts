import { luFactor, luSolve, type Mat } from './mat';

/**
 * Small second-order cone programs (docs/aerospace-gnc.md §3.2), with an optional convex quadratic
 * objective:
 *   minimise ½·xᵀPx + cᵀx   subject to   A·x = b,   h − G·x ∈ K
 * where K is a product of `linear` nonnegative rows followed by second-order cones
 * {(s₀, s₁): ‖s₁‖ ≤ s₀} of the sizes in `soc` (each cone's first row is its bound s₀).
 *
 * Solved with the barrier method (Boyd & Vandenberghe 2004, §11.3): Newton's method on
 *   t·f₀(x) − Σ log sᵢ − Σ log(s₀² − ‖s₁‖²)
 * restricted to A·x = b, for t growing by `mu` until the duality gap θ/t is below the tolerance
 * (θ: one per linear row, two per cone). It needs a strictly feasible start; phase I finds one by
 * minimising τ over h − G·x + τ·e ∈ K and reports the problem infeasible if τ cannot go below 0.
 * Dense and small (tens to a few hundred variables), deterministic: an iteration cap, no clock.
 */
export interface ConeProblem {
  /** Quadratic term (positive semidefinite), optional. */
  p?: Mat;
  c: number[];
  /** Equality constraints, optional. */
  a?: Mat;
  b?: number[];
  /** h − G·x in the cone: the first `linear` rows ≥ 0, then the second-order cones in order. */
  g: Mat;
  h: number[];
  linear: number;
  soc: number[];
}

export type ConeStatus = 'optimal' | 'infeasible' | 'max-iterations';

export interface ConeResult {
  x: number[];
  status: ConeStatus;
  /** ½xᵀPx + cᵀx at x. */
  objective: number;
  /** Newton steps taken, phase I included. */
  iterations: number;
  /** Duality gap bound θ/t at the end. */
  gap: number;
}

export interface ConeOptions {
  /** A strictly feasible start; phase I runs if absent or not strictly feasible. */
  x0?: number[];
  /** Stop when the gap is below tol·max(1, |objective|). */
  tol?: number;
  mu?: number;
  maxNewton?: number;
}

const dot = (a: readonly number[], b: readonly number[]): number => {
  let s = 0;
  for (let i = 0; i < a.length; i++) s += a[i]! * b[i]!;
  return s;
};

/** The nonzeros of each row of G, cached per matrix (most rows touch a few variables). */
interface SparseRows {
  idx: number[][];
  val: number[][];
}
const sparseCache = new WeakMap<Mat, SparseRows>();
function sparse(g: Mat): SparseRows {
  let sp = sparseCache.get(g);
  if (!sp) {
    sp = { idx: [], val: [] };
    for (const row of g) {
      const idx: number[] = [];
      const val: number[] = [];
      row.forEach((v, j) => {
        if (v !== 0) {
          idx.push(j);
          val.push(v);
        }
      });
      sp.idx.push(idx);
      sp.val.push(val);
    }
    sparseCache.set(g, sp);
  }
  return sp;
}

/** Slacks s = h − G·x. */
function slacks(pr: ConeProblem, x: readonly number[]): number[] {
  const { idx, val } = sparse(pr.g);
  return pr.h.map((hi, i) => {
    const ix = idx[i]!;
    const vx = val[i]!;
    let v = hi;
    for (let k = 0; k < ix.length; k++) v -= vx[k]! * x[ix[k]!]!;
    return v;
  });
}

/** Whether s is strictly inside the cone. */
function interior(pr: ConeProblem, s: readonly number[]): boolean {
  for (let i = 0; i < pr.linear; i++) if (!(s[i]! > 0)) return false;
  let r = pr.linear;
  for (const q of pr.soc) {
    let n2 = 0;
    for (let k = 1; k < q; k++) n2 += s[r + k]! ** 2;
    if (!(s[r]! > 0) || !(s[r]! ** 2 - n2 > 0)) return false;
    r += q;
  }
  return true;
}

/** Barrier value at s (Infinity outside the cone). */
function barrier(pr: ConeProblem, s: readonly number[]): number {
  if (!interior(pr, s)) return Infinity;
  let v = 0;
  for (let i = 0; i < pr.linear; i++) v -= Math.log(s[i]!);
  let r = pr.linear;
  for (const q of pr.soc) {
    let n2 = 0;
    for (let k = 1; k < q; k++) n2 += s[r + k]! ** 2;
    v -= Math.log(s[r]! ** 2 - n2);
    r += q;
  }
  return v;
}

const objective = (pr: ConeProblem, x: readonly number[]): number => {
  let v = dot(pr.c, x);
  if (pr.p) for (let i = 0; i < x.length; i++) v += 0.5 * x[i]! * dot(pr.p[i]!, x);
  return v;
};

/** Gradient and Hessian of t·f₀ + barrier at x (s = h − Gx strictly inside). */
function derivatives(
  pr: ConeProblem,
  x: readonly number[],
  s: readonly number[],
  t: number,
): { grad: number[]; hess: Mat } {
  const n = x.length;
  const grad = pr.c.map((ci) => t * ci);
  const hess: Mat = Array.from({ length: n }, () => new Array<number>(n).fill(0));
  if (pr.p) {
    for (let i = 0; i < n; i++) {
      const pi = pr.p[i]!;
      grad[i]! += t * dot(pi, x);
      const hi = hess[i]!;
      for (let j = 0; j < n; j++) hi[j] = t * pi[j]!;
    }
  }
  // ∇ₓ = −Gᵀ∇ₛψ, ∇²ₓ = Gᵀ∇²ₛψ·G, row by row, over each row's nonzeros.
  const { idx, val } = sparse(pr.g);
  const addOuter = (ix: readonly number[], vx: readonly number[], w: number) => {
    for (let a = 0; a < ix.length; a++) {
      const hi = hess[ix[a]!]!;
      const ua = vx[a]! * w;
      for (let b = 0; b < ix.length; b++) hi[ix[b]!]! += ua * vx[b]!;
    }
  };
  for (let i = 0; i < pr.linear; i++) {
    const ix = idx[i]!;
    const vx = val[i]!;
    const si = s[i]!;
    // ψ = −log s: ∇ₛψ = −1/s, ∇²ₛψ = 1/s².
    for (let k = 0; k < ix.length; k++) grad[ix[k]!]! += vx[k]! / si;
    addOuter(ix, vx, 1 / (si * si));
  }
  let r = pr.linear;
  const gw = new Array<number>(n).fill(0);
  for (const q of pr.soc) {
    // ψ = −log Δ, Δ = s₀² − ‖s₁‖², w = (s₀, −s₁): ∇ₛψ = −2w/Δ, ∇²ₛψ = (2/Δ)·diag(−1, 1, …) + 4wwᵀ/Δ².
    let n2 = 0;
    for (let k = 1; k < q; k++) n2 += s[r + k]! ** 2;
    const d = s[r]! ** 2 - n2;
    // Gᵀw, over the union of the block's nonzeros.
    const used = new Set<number>();
    for (let k = 0; k < q; k++) {
      const ix = idx[r + k]!;
      const vx = val[r + k]!;
      const wk = k === 0 ? s[r]! : -s[r + k]!;
      for (let a = 0; a < ix.length; a++) {
        gw[ix[a]!]! += vx[a]! * wk;
        used.add(ix[a]!);
      }
      addOuter(ix, vx, ((k === 0 ? -1 : 1) * 2) / d);
    }
    const ui = [...used];
    const uv = ui.map((j) => gw[j]!);
    for (let a = 0; a < ui.length; a++) grad[ui[a]!]! += (2 * uv[a]!) / d;
    addOuter(ui, uv, 4 / (d * d));
    for (const j of ui) gw[j] = 0;
    r += q;
  }
  return { grad, hess };
}

/** Cholesky factor L (lower, in place in a copy) of a positive definite H, or null. */
function cholesky(h: Mat): Mat | null {
  const n = h.length;
  const l: Mat = h.map((r) => [...r]);
  for (let j = 0; j < n; j++) {
    const lj = l[j]!;
    let d = lj[j]!;
    for (let k = 0; k < j; k++) d -= lj[k]! * lj[k]!;
    if (!(d > 1e-300)) return null;
    d = Math.sqrt(d);
    lj[j] = d;
    for (let i = j + 1; i < n; i++) {
      const li = l[i]!;
      let s = li[j]!;
      for (let k = 0; k < j; k++) s -= li[k]! * lj[k]!;
      li[j] = s / d;
    }
  }
  return l;
}

/** Solve L·Lᵀ·x = b. */
function cholSolve(l: Mat, b: readonly number[]): number[] {
  const n = l.length;
  const y = [...b];
  for (let i = 0; i < n; i++) {
    const li = l[i]!;
    let s = y[i]!;
    for (let k = 0; k < i; k++) s -= li[k]! * y[k]!;
    y[i] = s / li[i]!;
  }
  for (let i = n - 1; i >= 0; i--) {
    let s = y[i]!;
    for (let k = i + 1; k < n; k++) s -= l[k]![i]! * y[k]!;
    y[i] = s / l[i]![i]!;
  }
  return y;
}

/**
 * Solve the KKT system [H Aᵀ; A 0]·[dx; ν] = [−g; 0]: by Cholesky of H and the Schur complement
 * A·H⁻¹·Aᵀ when H is positive definite (the usual case), by LU of the whole system otherwise.
 */
function newtonStep(hess: Mat, grad: readonly number[], a: Mat | undefined): number[] {
  const n = grad.length;
  const p = a?.length ?? 0;
  const l = cholesky(hess);
  if (l) {
    const w = cholSolve(l, grad);
    if (!p) return w.map((v) => -v);
    const y = a!.map((row) => cholSolve(l, row)); // rows of (H⁻¹Aᵀ)ᵀ
    const s: Mat = a!.map((ri) => y.map((yj) => dot(ri, yj)));
    try {
      const nu = luSolve(
        luFactor(s),
        a!.map((ri) => -dot(ri, w)),
      );
      const dx = w.map((v, i) => {
        let acc = v;
        for (let j = 0; j < p; j++) acc += y[j]![i]! * nu[j]!;
        return -acc;
      });
      if (dx.every(Number.isFinite)) return dx;
    } catch {
      // fall through to the full system
    }
  }
  for (let reg = 0; ; reg = reg === 0 ? 1e-10 : reg * 100) {
    const k: Mat = new Array<number[]>(n + p);
    for (let i = 0; i < n; i++) {
      const row = new Array<number>(n + p);
      const hi = hess[i]!;
      for (let j = 0; j < n; j++) row[j] = hi[j]!;
      row[i]! += reg;
      for (let j = 0; j < p; j++) row[n + j] = a![j]![i]!;
      k[i] = row;
    }
    for (let i = 0; i < p; i++) {
      const row = new Array<number>(n + p).fill(0);
      const ai = a![i]!;
      for (let j = 0; j < n; j++) row[j] = ai[j]!;
      row[n + i] = -reg;
      k[n + i] = row;
    }
    try {
      const sol = luSolve(luFactor(k), [...grad.map((v) => -v), ...new Array<number>(p).fill(0)]);
      if (sol.every(Number.isFinite)) return sol.slice(0, n);
    } catch {
      // singular: retry with more regularisation
    }
    if (reg > 1) return new Array<number>(n).fill(0);
  }
}

interface BarrierRun {
  x: number[];
  t: number;
  iterations: number;
  done: boolean;
}

/**
 * The barrier method from a strictly feasible x (satisfying A·x = b). `stopEarly` ends it as soon
 * as it returns true (phase I stops once τ < 0).
 */
function barrierMethod(
  pr: ConeProblem,
  x0: number[],
  tol: number,
  mu: number,
  maxNewton: number,
  stopEarly?: (x: number[]) => boolean,
): BarrierRun {
  const theta = pr.linear + 2 * pr.soc.length;
  let x = [...x0];
  let t = Math.max(1, theta / Math.max(1, Math.abs(objective(pr, x))));
  let iterations = 0;
  const merit = (y: readonly number[], tt: number) =>
    tt * objective(pr, y) + barrier(pr, slacks(pr, y));
  for (;;) {
    // Centering: Newton on t·f₀ + φ with equality constraints.
    for (let inner = 0; inner < 60 && iterations < maxNewton; inner++) {
      const s = slacks(pr, x);
      const { grad, hess } = derivatives(pr, x, s, t);
      const dx = newtonStep(hess, grad, pr.a);
      const slope = dot(grad, dx);
      if (-slope / 2 < 1e-8) break;
      iterations++;
      const f0 = merit(x, t);
      let step = 1;
      let next = x.map((v, i) => v + step * dx[i]!);
      while (step > 1e-12 && merit(next, t) > f0 + 0.01 * step * slope) {
        step *= 0.5;
        next = x.map((v, i) => v + step * dx[i]!);
      }
      if (step <= 1e-12) break;
      x = next;
      if (stopEarly?.(x)) return { x, t, iterations, done: true };
    }
    const gap = theta / t;
    if (gap < tol * Math.max(1, Math.abs(objective(pr, x))))
      return { x, t, iterations, done: true };
    if (iterations >= maxNewton) return { x, t, iterations, done: false };
    t *= mu;
  }
}

/** Least-norm solution of A·x = b (zeros without equality constraints). */
function leastNorm(a: Mat | undefined, b: number[] | undefined, n: number): number[] {
  if (!a || !a.length) return new Array<number>(n).fill(0);
  const aat: Mat = a.map((ri) => a.map((rj) => dot(ri, rj)));
  for (let i = 0; i < aat.length; i++) aat[i]![i]! += 1e-12;
  const y = luSolve(luFactor(aat), b!);
  const x = new Array<number>(n).fill(0);
  a.forEach((ri, i) => ri.forEach((v, j) => (x[j]! += v * y[i]!)));
  return x;
}

/** Phase I: a strictly feasible point, or null if there is none. */
function phaseOne(pr: ConeProblem, tol: number, mu: number, maxNewton: number) {
  const n = pr.c.length;
  const x0 = leastNorm(pr.a, pr.b, n);
  const s = slacks(pr, x0);
  // How far each row is from the interior: τ must exceed this.
  let need = 0;
  for (let i = 0; i < pr.linear; i++) need = Math.max(need, -s[i]!);
  let r = pr.linear;
  for (const q of pr.soc) {
    let n2 = 0;
    for (let k = 1; k < q; k++) n2 += s[r + k]! ** 2;
    need = Math.max(need, Math.sqrt(n2) - s[r]!);
    r += q;
  }
  // Variables (x, τ): rows h − Gx + τ·e, plus τ ≥ −1 to keep it bounded.
  const e = new Array<number>(pr.h.length).fill(0);
  for (let i = 0; i < pr.linear; i++) e[i] = 1;
  r = pr.linear;
  for (const q of pr.soc) {
    e[r] = 1;
    r += q;
  }
  // A large ball around the start, ‖x − x₀‖ ≤ R, keeps the barrier from running off to infinity
  // along a direction the cone leaves open (where it would rather grow the slacks than lower τ).
  const radius = 1e4 * Math.max(1, Math.hypot(...x0));
  const ball = [
    { g: new Array<number>(n + 1).fill(0), h: radius },
    ...x0.map((v, i) => {
      const g = new Array<number>(n + 1).fill(0);
      g[i] = -1;
      return { g, h: -v };
    }),
  ];
  const ph: ConeProblem = {
    c: [...new Array<number>(n).fill(0), 1],
    a: pr.a?.map((row) => [...row, 0]),
    b: pr.b,
    g: [
      new Array<number>(n + 1).fill(0),
      ...pr.g.map((row, i) => [...row, -e[i]!]),
      ...ball.map((r) => r.g),
    ],
    h: [1, ...pr.h, ...ball.map((r) => r.h)],
    linear: pr.linear + 1,
    soc: [...pr.soc, n + 1],
  };
  ph.g[0]![n] = -1; // 1 + τ ≥ 0
  const run = barrierMethod(ph, [...x0, need + 1], tol, mu, maxNewton, (y) => y[n]! < -1e-6);
  const tau = run.x[n]!;
  return { x: run.x.slice(0, n), feasible: tau < 0, iterations: run.iterations };
}

export function solveCone(pr: ConeProblem, opts: ConeOptions = {}): ConeResult {
  const tol = opts.tol ?? 1e-7;
  const mu = opts.mu ?? 20;
  const maxNewton = opts.maxNewton ?? 400;
  const n = pr.c.length;
  let x = opts.x0;
  let iterations = 0;
  const feasibleStart =
    x &&
    interior(pr, slacks(pr, x)) &&
    (!pr.a || pr.a.every((row, i) => Math.abs(dot(row, x!) - pr.b![i]!) < 1e-9));
  if (!feasibleStart) {
    const one = phaseOne(pr, tol, mu, maxNewton);
    iterations += one.iterations;
    if (!one.feasible) {
      return { x: one.x, status: 'infeasible', objective: NaN, iterations, gap: NaN };
    }
    x = one.x;
  }
  const run = barrierMethod(pr, x!.slice(0, n), tol, mu, maxNewton - iterations);
  const theta = pr.linear + 2 * pr.soc.length;
  return {
    x: run.x,
    status: run.done ? 'optimal' : 'max-iterations',
    objective: objective(pr, run.x),
    iterations: iterations + run.iterations,
    gap: theta / run.t,
  };
}

/**
 * Assembles a cone problem row by row. A linear row g·x ≤ h; a cone ‖(hᵢ − gᵢ·x)ᵢ≥₁‖ ≤ h₀ − g₀·x.
 * Linear rows are placed before the cones whatever the order they were added in.
 */
export class ConeBuilder {
  private lin: { g: number[]; h: number }[] = [];
  private cones: { g: number[]; h: number }[][] = [];
  private eq: { a: number[]; b: number }[] = [];

  constructor(readonly n: number) {}

  /** A zero row of this problem's width. */
  row(): number[] {
    return new Array<number>(this.n).fill(0);
  }

  /** g·x ≤ h. */
  le(g: number[], h: number): void {
    this.lin.push({ g, h });
  }

  /** a·x = b. */
  equal(a: number[], b: number): void {
    this.eq.push({ a, b });
  }

  /** ‖(hᵢ − gᵢ·x)ᵢ≥₁‖ ≤ h₀ − g₀·x, with rows given as (g, h). */
  cone(rows: { g: number[]; h: number }[]): void {
    this.cones.push(rows);
  }

  build(c: number[], p?: Mat): ConeProblem {
    const all = [...this.lin, ...this.cones.flat()];
    return {
      p,
      c,
      a: this.eq.length ? this.eq.map((e) => e.a) : undefined,
      b: this.eq.length ? this.eq.map((e) => e.b) : undefined,
      g: all.map((r) => r.g),
      h: all.map((r) => r.h),
      linear: this.lin.length,
      soc: this.cones.map((k) => k.length),
    };
  }
}
