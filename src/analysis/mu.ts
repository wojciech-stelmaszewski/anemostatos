// Structured uncertainty and the structured singular value μ (docs/analysis.md §4, Chapter H,
// lesson III.14).
//
// The family of lesson III.11 is three real parameters: the mass, the motor time constant and
// the extra delay. Each enters the L1 loop as a factor of its own, L = L₀·f_m·f_τ·f_d, because the
// altitude PID reads only the altitude and the plant's response to thrust is (1/m)·G_τ(z). The
// small-gain test of III.11 lumps the three into one disk around L₀. Here each gets its own disk,
//   f_i ∈ {1 + W_i(jω)·δ_i : |δ_i| ≤ 1},  W_i(jω) = max over the parameter's range of |f_i − 1|,
// and the three δ_i are kept apart: a diagonal perturbation Δ = diag(δ_m, δ_τ, δ_d) closing a
// 3 × 3 loop M(jω). The loop is guaranteed stable for every member if μ_Δ(M(jω)) < 1 at every ω.
//
// What is computed is the classical upper bound of complex μ with diagonal scalings,
// μ ≤ min_D σ̄(D·M·D⁻¹). The disks allow complex δ, so this bounds real-parametric μ from above:
// a guarantee, still conservative, but less so than one disk for all three.
import {
  cabs,
  cadd,
  cdiv,
  cmul,
  cneg,
  cscale,
  csub,
  cx,
  C_ONE,
  type Complex,
} from '@/math/complex';
import { logspace } from '@/math/margins';
import type { Params } from '@/sim/params';
import { isLoopStable, l1Loop, loopGain, sensitivities, type LoopModel } from './loop';

/** A member of the family with every range scaled by k (k = 1: the ranges of `p.uncertainty`). */
export function scaledMember(p: Params, a: number, b: number, c: number, k = 1): Params {
  const u = p.uncertainty;
  const q = structuredClone(p);
  q.drone.mass = p.drone.mass * (1 + (a * k * u.massPct) / 100);
  q.drone.motorTau = p.drone.motorTau * Math.max(u.tauFactor, 1) ** (b * k);
  q.sensors.delayMs = p.sensors.delayMs + c * k * u.delayMs;
  return q;
}

/** Largest singular value of a 3 × 3 complex matrix: the largest eigenvalue of AᴴA, exactly. */
export function sigmaMax3(a: Complex[][]): number {
  // H = AᴴA, Hermitian positive semidefinite.
  const h: Complex[][] = [0, 1, 2].map((i) =>
    [0, 1, 2].map((j) => {
      let s: Complex = cx(0);
      for (let k = 0; k < 3; k++) s = cadd(s, cmul(cx(a[k]![i]!.re, -a[k]![i]!.im), a[k]![j]!));
      return s;
    }),
  );
  const d = [h[0]![0]!.re, h[1]![1]!.re, h[2]![2]!.re];
  const n2 = (z: Complex) => z.re * z.re + z.im * z.im;
  const tr = d[0]! + d[1]! + d[2]!;
  const c1 =
    d[0]! * d[1]! + d[0]! * d[2]! + d[1]! * d[2]! - n2(h[0]![1]!) - n2(h[0]![2]!) - n2(h[1]![2]!);
  // det of a Hermitian 3 × 3: real.
  const triple = cmul(cmul(h[0]![1]!, h[1]![2]!), h[2]![0]!);
  const det =
    d[0]! * d[1]! * d[2]! +
    2 * triple.re -
    d[0]! * n2(h[1]![2]!) -
    d[1]! * n2(h[0]![2]!) -
    d[2]! * n2(h[0]![1]!);
  // λ³ − tr·λ² + c1·λ − det = 0 has three real roots; with λ = t + tr/3 it becomes
  // t³ + p·t + q = 0, solved by the trigonometric formula (largest root, k = 0).
  const p = c1 - (tr * tr) / 3;
  const q = (-2 * tr ** 3) / 27 + (tr * c1) / 3 - det;
  const shift = tr / 3;
  if (p > -1e-300) return Math.sqrt(Math.max(shift, 0));
  const arg = Math.min(1, Math.max(-1, ((3 * q) / (2 * p)) * Math.sqrt(-3 / p)));
  const lam = 2 * Math.sqrt(-p / 3) * Math.cos(Math.acos(arg) / 3) + shift;
  return Math.sqrt(Math.max(lam, 0));
}

/** The 3 × 3 loop seen by diag(δ_m, δ_τ, δ_d) for complementary sensitivity T and weights W. */
export function muMatrix(t: Complex, w: [number, number, number]): Complex[][] {
  const mt = cneg(t);
  const one = csub(C_ONE, t);
  const rows: Complex[][] = [
    [mt, one, one],
    [mt, mt, one],
    [mt, mt, mt],
  ];
  return rows.map((row, i) => row.map((v) => cscale(v, w[i]!)));
}

/** min over positive diagonal D of σ̄(D·M·D⁻¹) (D₁ = 1): Nelder–Mead in log D, convex there. */
export function muUpper(m: Complex[][]): number {
  const f = (x: number[]): number => {
    const d = [1, Math.exp(x[0]!), Math.exp(x[1]!)];
    return sigmaMax3(m.map((row, i) => row.map((v, j) => cscale(v, d[i]! / d[j]!))));
  };
  let simplex = [
    [0, 0],
    [1, 0],
    [0, 1],
  ];
  let vals = simplex.map(f);
  for (let it = 0; it < 300; it++) {
    const order = [0, 1, 2].sort((i, j) => vals[i]! - vals[j]!);
    simplex = order.map((i) => simplex[i]!);
    vals = order.map((i) => vals[i]!);
    if (Math.abs(vals[2]! - vals[0]!) < 1e-10 * Math.max(1, vals[0]!)) break;
    const c = [(simplex[0]![0]! + simplex[1]![0]!) / 2, (simplex[0]![1]! + simplex[1]![1]!) / 2];
    const at = (t: number) => [
      c[0]! + t * (simplex[2]![0]! - c[0]!),
      c[1]! + t * (simplex[2]![1]! - c[1]!),
    ];
    const xr = at(-1);
    const fr = f(xr);
    if (fr < vals[0]!) {
      const xe = at(-2);
      const fe = f(xe);
      if (fe < fr) [simplex[2], vals[2]] = [xe, fe];
      else [simplex[2], vals[2]] = [xr, fr];
    } else if (fr < vals[1]!) [simplex[2], vals[2]] = [xr, fr];
    else {
      const xc = at(0.5);
      const fc = f(xc);
      if (fc < vals[2]!) [simplex[2], vals[2]] = [xc, fc];
      else {
        // Shrink towards the best vertex.
        for (const i of [1, 2]) {
          simplex[i] = [
            (simplex[i]![0]! + simplex[0]![0]!) / 2,
            (simplex[i]![1]! + simplex[0]![1]!) / 2,
          ];
          vals[i] = f(simplex[i]!);
        }
      }
    }
  }
  return Math.min(...vals);
}

/**
 * Mixed-μ upper bound with the first block real (the mass) [Fan, Tits & Doyle 1991]: the smallest
 * β for which some D > 0 and real G (on the real block only) give
 *   σ̄( (I + G²)^(−1/4) · (D·M·D⁻¹/β − jG) · (I + G²)^(−1/4) ) ≤ 1.
 * A real δ cannot rotate a signal, and G is what removes that freedom from the bound.
 */
export function muMixedUpper(m: Complex[][], hi = muUpper(m)): number {
  const feasible = (beta: number): boolean => {
    const f = (x: number[]): number => {
      const d = [1, Math.exp(x[0]!), Math.exp(x[1]!)];
      const g = x[2]!;
      const s = (1 + g * g) ** -0.25;
      const scale = [s, 1, 1];
      return sigmaMax3(
        m.map((row, i) =>
          row.map((v, j) => {
            let z = cscale(v, d[i]! / d[j]! / beta);
            if (i === 0 && j === 0) z = csub(z, cx(0, g));
            return cscale(z, scale[i]! * scale[j]!);
          }),
        ),
      );
    };
    return nelderMead(f, [0, 0, 0]) <= 1;
  };
  let lo = 0;
  let up = hi;
  if (!(up > 0)) return 0;
  for (let i = 0; i < 18; i++) {
    const mid = (lo + up) / 2;
    if (feasible(mid)) up = mid;
    else lo = mid;
  }
  return up;
}

/** Nelder–Mead minimisation from x0 (a few hundred evaluations; enough for 2–3 dimensions). */
function nelderMead(f: (x: number[]) => number, x0: number[], iters = 400): number {
  const n = x0.length;
  let pts = [x0, ...x0.map((_, i) => x0.map((v, j) => (i === j ? v + 0.8 : v)))];
  let vals = pts.map(f);
  for (let it = 0; it < iters; it++) {
    const order = pts.map((_, i) => i).sort((i, j) => vals[i]! - vals[j]!);
    pts = order.map((i) => pts[i]!);
    vals = order.map((i) => vals[i]!);
    if (vals[0]! <= 1 - 1e-9 && it > 20) return vals[0]!; // feasibility is all that is asked
    if (Math.abs(vals[n]! - vals[0]!) < 1e-10) break;
    const c = x0.map((_, j) => pts.slice(0, n).reduce((acc, p) => acc + p[j]!, 0) / n);
    const at = (t: number) => c.map((v, j) => v + t * (pts[n]![j]! - v));
    const xr = at(-1);
    const fr = f(xr);
    if (fr < vals[0]!) {
      const xe = at(-2);
      const fe = f(xe);
      [pts[n], vals[n]] = fe < fr ? [xe, fe] : [xr, fr];
    } else if (fr < vals[n - 1]!) [pts[n], vals[n]] = [xr, fr];
    else {
      const xc = at(0.5);
      const fc = f(xc);
      if (fc < vals[n]!) [pts[n], vals[n]] = [xc, fc];
      else
        for (let i = 1; i <= n; i++) {
          pts[i] = pts[i]!.map((v, j) => (v + pts[0]![j]!) / 2);
          vals[i] = f(pts[i]!);
        }
    }
  }
  return Math.min(...vals);
}

export interface MuAnalysis {
  fHz: number[];
  /** The three weights: mass, motor lag, delay. */
  w: [number[], number[], number[]];
  /** |T| of the nominal loop. */
  t: number[];
  /** Small-gain test of III.11 on the same family: |W·T| with one lumped weight. */
  smallGain: number[];
  /** Upper bound of μ at each frequency. */
  mu: number[];
  /** Mixed-μ bound with the mass as a real parameter. */
  muMixed: number[];
  peakMu: number;
  peakMuMixed: number;
  fPeakMu: number;
  peakSmallGain: number;
}

/** Deviation of each member's loop from the nominal one, |L/L₀ − 1|, maxed over the members. */
function weightOf(nominal: LoopModel, members: LoopModel[], om: number): number {
  const l0 = loopGain(nominal, om);
  let w = 0;
  for (const m of members) w = Math.max(w, cabs(csub(cdiv(loopGain(m, om), l0), C_ONE)));
  return w;
}

/** μ analysis of the L1 loop of `p` against its family scaled by k (null without a model). */
export function muAnalysis(p: Params, k = 1, points = 160): MuAnalysis | null {
  const nominal = l1Loop(p);
  if (!nominal) return null;
  const grid = (n: number, lo: number) =>
    Array.from({ length: n }, (_, i) => lo + ((1 - lo) * i) / (n - 1));
  const loops = (pts: [number, number, number][]) =>
    pts.map(([a, b, c]) => l1Loop(scaledMember(p, a, b, c, k))!);
  const mass = loops(grid(9, -1).map((a) => [a, 0, 0]));
  const tau = loops(grid(13, -1).map((b) => [0, b, 0]));
  const delay = loops(grid(9, 0).map((c) => [0, 0, c]));
  // One lumped weight for the small-gain comparison, over every combination on a coarser grid.
  const all: LoopModel[] = [];
  for (const a of [-1, -0.5, 0, 0.5, 1])
    for (const b of [-1, -0.5, 0, 0.5, 1])
      for (const c of [0, 0.5, 1]) all.push(l1Loop(scaledMember(p, a, b, c, k))!);
  const fHz = logspace(0.05, Math.min(100, 0.49 / nominal.T), points);
  const out: MuAnalysis = {
    fHz,
    w: [[], [], []],
    t: [],
    smallGain: [],
    mu: [],
    muMixed: [],
    peakMu: 0,
    peakMuMixed: 0,
    fPeakMu: NaN,
    peakSmallGain: 0,
  };
  for (const f of fHz) {
    const om = 2 * Math.PI * f;
    const w: [number, number, number] = [
      weightOf(nominal, mass, om),
      weightOf(nominal, tau, om),
      weightOf(nominal, delay, om),
    ];
    const t = sensitivities(nominal, om).t;
    const mm = muMatrix(t, w);
    const mu = muUpper(mm);
    const mix = muMixedUpper(mm, mu);
    const sg = weightOf(nominal, all, om) * cabs(t);
    out.w[0].push(w[0]);
    out.w[1].push(w[1]);
    out.w[2].push(w[2]);
    out.t.push(cabs(t));
    out.mu.push(mu);
    out.muMixed.push(mix);
    out.peakMuMixed = Math.max(out.peakMuMixed, mix);
    out.smallGain.push(sg);
    if (mu > out.peakMu) {
      out.peakMu = mu;
      out.fPeakMu = f;
    }
    out.peakSmallGain = Math.max(out.peakSmallGain, sg);
  }
  return out;
}

/** Whether every member of the family scaled by k on a 7 × 7 × 4 grid is stable. */
export function familyStable(p: Params, k: number): boolean {
  const ab = [-1, -2 / 3, -1 / 3, 0, 1 / 3, 2 / 3, 1];
  for (const a of ab)
    for (const b of ab)
      for (const c of [0, 1 / 3, 2 / 3, 1]) {
        const m = l1Loop(scaledMember(p, a, b, c, k));
        if (m && !isLoopStable(m)) return false;
      }
  return true;
}

/** The largest k for which `ok(k)` holds, by bisection between lo (true) and hi (false). */
function bisect(ok: (k: number) => boolean, lo = 0.05, hi = 4, steps = 12): number {
  if (!ok(lo)) return 0;
  if (ok(hi)) return hi;
  for (let i = 0; i < steps; i++) {
    const mid = Math.sqrt(lo * hi);
    if (ok(mid)) lo = mid;
    else hi = mid;
  }
  return lo;
}

export interface RobustnessScale {
  /** How far the ranges can grow before each answer stops guaranteeing (or finding) stability. */
  smallGain: number;
  mu: number;
  /** Where the first unstable member appears on the grid: the truth, from above. */
  grid: number;
}

/**
 * Three answers to "how much uncertainty can this loop take?", as a multiple k of the ranges of
 * `p.uncertainty`: the small-gain test, the μ bound, and the first unstable member on a grid.
 * In theory smallGain and mu are guarantees (≤ the truth) and grid is ≥ the truth.
 */
export function robustnessScale(p: Params): RobustnessScale | null {
  if (!l1Loop(p)) return null;
  return {
    smallGain: bisect((k) => muAnalysis(p, k, 100)!.peakSmallGain < 1),
    mu: bisect((k) => muAnalysis(p, k, 100)!.peakMu < 1),
    grid: bisect((k) => familyStable(p, k)),
  };
}
