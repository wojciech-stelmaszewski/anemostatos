import { solveNlp, type NlpEval } from '@/math/nlp';
import { evalSpline, minSnap, type Spline } from '@/math/poly';
import { v3, type Vec3 } from '@/math/vec3';
import { GRAVITY, type Params } from '@/sim/params';
import type { Reference } from '@/engine/reference';

/**
 * Lesson IV.6 (docs/aerospace-gnc.md §4, Chapter K): plan a rest-to-rest manoeuvre in the
 * horizontal plane, from the start to `distance` metres along x, past a pillar in the way.
 *
 * Min-snap (lesson II.12) fixes the shape first (a waypoint beside the pillar) and the timing
 * second; nothing in it knows the limits. Direct transcription puts the limits into the problem:
 * the jerk is held constant over N intervals (so the drone's tilt rate is bounded), the state
 * follows exactly (a triple integrator), and
 *   minimise ∫|j|²dt  (or T)   s.t.   |aₖ| ≤ a_max,  |jₖ| ≤ j_max,  |pₖ − c| ≥ R,  end at rest
 * at the nodes and the midpoints. The states are eliminated: with h = T/N each node is
 * pₖ = h³Pₖ(j), vₖ = h²Vₖ(j), aₖ = h·Aₖ(j) with Pₖ, Vₖ, Aₖ linear in the jerks, so the only
 * variables are the 2N jerks and, for the least time, T. The keep-out circle is not convex,
 * and with T free neither is the rest; the augmented-Lagrangian solver of `math/nlp.ts` takes it.
 */
export interface ManoeuvreSpec {
  distance: number;
  /** Duration, s; for the least time the starting guess. */
  duration: number;
  minTime: boolean;
  accelMax: number;
  jerkMax: number;
  /** Keep-out circle in the plane, relative to the start; r = 0 for none. */
  obstacle: { x: number; z: number; r: number };
  nodes: number;
}

export type ManoeuvreMethod = 'minsnap' | 'collocation';

export interface Manoeuvre {
  method: ManoeuvreMethod;
  status: 'optimal' | 'infeasible';
  duration: number;
  /** The reference (offsets from the start, y = 0) at time t from its start. */
  at: (t: number) => Reference;
  /** Largest horizontal acceleration and jerk the plan asks for, and its closest approach. */
  peakAccel: number;
  peakJerk: number;
  clearance: number;
  /** Solver effort (collocation only). */
  iterations: number;
}

/** The plan's limits from the controller's: a fraction of the tilt limit's acceleration. */
export const JERK_MAX = 20;

/** Planner keep-out radius: the pillar plus half the drone. */
export const DRONE_RADIUS = 0.3;

/** Coefficients of node k's state per unit jerk in interval m (n = k − m − 1 ≥ 0). */
const cA = (n: number) => (n < 0 ? 0 : 1);
const cV = (n: number) => (n < 0 ? 0 : 0.5 + n);
const cP = (n: number) => (n < 0 ? 0 : 1 / 6 + n / 2 + (n * n) / 2);

/** States (per unit of h³, h², h) at node k and at the midpoint after it, from the jerks. */
interface Basis {
  /** [node][interval] coefficients. */
  p: number[][];
  v: number[][];
  a: number[][];
}
function basis(N: number): Basis {
  const p: number[][] = [];
  const v: number[][] = [];
  const a: number[][] = [];
  for (let k = 0; k <= N; k++) {
    p.push(Array.from({ length: N }, (_, m) => cP(k - m - 1)));
    v.push(Array.from({ length: N }, (_, m) => cV(k - m - 1)));
    a.push(Array.from({ length: N }, (_, m) => cA(k - m - 1)));
  }
  return { p, v, a };
}

/** The piecewise-constant-jerk trajectory at time t. */
function evalJerks(jx: number[], jz: number[], T: number, t: number): Reference {
  const N = jx.length;
  const h = T / N;
  const tt = Math.min(Math.max(t, 0), T);
  let p = v3();
  let vel = v3();
  let acc = v3();
  for (let k = 0; k < N; k++) {
    const tau = Math.min(h, tt - k * h);
    if (tau <= 0) break;
    const j = v3(jx[k]!, 0, jz[k]!);
    const adv = (s: number) =>
      v3(
        p.x + vel.x * s + (acc.x * s * s) / 2 + (j.x * s ** 3) / 6,
        0,
        p.z + vel.z * s + (acc.z * s * s) / 2 + (j.z * s ** 3) / 6,
      );
    const np = adv(tau);
    const nv = v3(
      vel.x + acc.x * tau + (j.x * tau * tau) / 2,
      0,
      vel.z + acc.z * tau + (j.z * tau * tau) / 2,
    );
    const na = v3(acc.x + j.x * tau, 0, acc.z + j.z * tau);
    p = np;
    vel = nv;
    acc = na;
    if (tau < h) {
      return { pos: p, vel, acc, jerk: j, snap: v3() };
    }
  }
  return { pos: p, vel, acc, jerk: v3(), snap: v3() };
}

/** Sample a plan for its peaks and its closest approach to the obstacle. */
function measure(at: (t: number) => Reference, T: number, o: ManoeuvreSpec['obstacle']) {
  let peakAccel = 0;
  let peakJerk = 0;
  let clearance = Infinity;
  for (let i = 0; i <= 600; i++) {
    const r = at((i / 600) * T);
    peakAccel = Math.max(peakAccel, Math.hypot(r.acc.x, r.acc.z));
    peakJerk = Math.max(peakJerk, Math.hypot(r.jerk.x, r.jerk.z));
    if (o.r > 0) clearance = Math.min(clearance, Math.hypot(r.pos.x - o.x, r.pos.z - o.z) - o.r);
  }
  return { peakAccel, peakJerk, clearance };
}

/** Min-snap through a waypoint beside the obstacle (on the +z side), two equal segments. */
export function planMinSnap(spec: ManoeuvreSpec): Manoeuvre {
  const o = spec.obstacle;
  const T = spec.duration;
  let sx: Spline;
  let sz: Spline;
  if (o.r > 0) {
    const side = o.z + o.r + 0.2;
    sx = minSnap([0, o.x, spec.distance], [T / 2, T / 2]);
    sz = minSnap([0, side, 0], [T / 2, T / 2]);
  } else {
    sx = minSnap([0, spec.distance], [T]);
    sz = minSnap([0, 0], [T]);
  }
  const at = (t: number): Reference => {
    const dx = evalSpline(sx, Math.min(Math.max(t, 0), T));
    const dz = evalSpline(sz, Math.min(Math.max(t, 0), T));
    const out = [0, 1, 2, 3, 4].map((d) => v3(dx[d]!, 0, dz[d]!)) as Vec3[];
    if (t >= T) return { pos: out[0]!, vel: v3(), acc: v3(), jerk: v3(), snap: v3() };
    return { pos: out[0]!, vel: out[1]!, acc: out[2]!, jerk: out[3]!, snap: out[4]! };
  };
  return {
    method: 'minsnap',
    status: 'optimal',
    duration: T,
    at,
    ...measure(at, T, o),
    iterations: 0,
  };
}

/** Direct transcription of the manoeuvre with the limits as constraints. */
export function planCollocation(spec: ManoeuvreSpec): Manoeuvre {
  const N = spec.nodes;
  const B = basis(N);
  const o = spec.obstacle;
  const free = spec.minTime;
  const n = 2 * N + (free ? 1 : 0);
  const D = spec.distance;
  // Variables: jx (N), jz (N), then T if it is free.
  const T0 = spec.duration;
  const tOf = (x: readonly number[]) => (free ? x[2 * N]! : T0);
  const rowsAt = (c: number[][], k: number) => c[k]!;
  // A smooth start on the +z side: a min-snap guess sampled into constant jerks.
  const guess = planMinSnap(spec);
  const h0 = T0 / N;
  const x0: number[] = [];
  for (const axis of ['x', 'z'] as const)
    for (let k = 0; k < N; k++) {
      const a0 = guess.at(k * h0).acc[axis];
      const a1 = guess.at((k + 1) * h0).acc[axis];
      x0.push((a1 - a0) / h0);
    }
  if (free) x0.push(T0);
  const w = free ? 1e-3 : 1;

  const evaluate = (x: readonly number[]): NlpEval => {
    const T = tOf(x);
    const h = T / N;
    const dh = 1 / N; // dh/dT
    const jx = x.slice(0, N);
    const jz = x.slice(N, 2 * N);
    const zero = () => new Array<number>(n).fill(0);
    // Objective: ∫|j|² dt = h Σ|j|² (scaled by w), plus T for the least time.
    let sj = 0;
    for (let k = 0; k < N; k++) sj += jx[k]! ** 2 + jz[k]! ** 2;
    const grad = zero();
    for (let k = 0; k < N; k++) {
      grad[k] = 2 * w * h * jx[k]!;
      grad[N + k] = 2 * w * h * jz[k]!;
    }
    let f = w * h * sj;
    if (free) {
      f += T;
      grad[2 * N] = 1 + w * dh * sj;
    }
    const lin = (c: number[], js: number[]) => dot(c, js);
    const ce: number[] = [];
    const je: number[][] = [];
    // End at rest at (D, 0): a_N = h·A_N(j) = 0 ⇔ A_N(j) = 0, likewise V_N; p_N = h³P_N(j).
    for (const [c, pw, target] of [
      [B.a[N]!, 0, 0],
      [B.v[N]!, 0, 0],
      [B.p[N]!, 3, null],
    ] as const) {
      for (let axis = 0; axis < 2; axis++) {
        const js = axis === 0 ? jx : jz;
        const off = axis * N;
        const val = lin(c as number[], js);
        const row = zero();
        if (pw === 0) {
          ce.push(val);
          for (let m = 0; m < N; m++) row[off + m] = c[m]!;
        } else {
          const goal = target ?? (axis === 0 ? D : 0);
          ce.push(h ** 3 * val - goal);
          for (let m = 0; m < N; m++) row[off + m] = h ** 3 * c[m]!;
          if (free) row[2 * N] = 3 * h * h * dh * val;
        }
        je.push(row);
      }
    }
    const ci: number[] = [];
    const ji: number[][] = [];
    // |aₖ|² ≤ a_max², at the interior nodes (the acceleration is piecewise linear).
    for (let k = 1; k < N; k++) {
      const ax = lin(rowsAt(B.a, k), jx);
      const az = lin(rowsAt(B.a, k), jz);
      const sa = 1 / spec.accelMax ** 2; // constraints scaled to order one
      ci.push(1 - sa * h * h * (ax * ax + az * az));
      const row = zero();
      for (let m = 0; m < N; m++) {
        row[m] = -2 * sa * h * h * ax * B.a[k]![m]!;
        row[N + m] = -2 * sa * h * h * az * B.a[k]![m]!;
      }
      if (free) row[2 * N] = -2 * sa * h * dh * (ax * ax + az * az);
      ji.push(row);
    }
    // |jₖ|² ≤ j_max².
    for (let k = 0; k < N; k++) {
      const sj = 1 / spec.jerkMax ** 2;
      ci.push(1 - sj * (jx[k]! ** 2 + jz[k]! ** 2));
      const row = zero();
      row[k] = -2 * sj * jx[k]!;
      row[N + k] = -2 * sj * jz[k]!;
      ji.push(row);
    }
    // Outside the keep-out circle at the nodes and at the midpoints of the intervals.
    if (o.r > 0) {
      for (let k = 1; k < N; k++) {
        for (const half of [false, true]) {
          // Midpoint after node k: h³(P + V/2 + A/8 + j/48 for its own interval).
          const coef = B.p[k]!.map((pk, m) =>
            half ? pk + B.v[k]![m]! / 2 + B.a[k]![m]! / 8 + (m === k ? 1 / 48 : 0) : pk,
          );
          const px = h ** 3 * lin(coef, jx) - o.x;
          const pz = h ** 3 * lin(coef, jz) - o.z;
          // As a distance (better scaled than its square): |p − c| − R ≥ 0.
          const dist = Math.max(Math.hypot(px, pz), 1e-9);
          const sd = 1 / (2 * dist);
          ci.push(dist - o.r);
          const row = zero();
          for (let m = 0; m < N; m++) {
            row[m] = sd * 2 * px * h ** 3 * coef[m]!;
            row[N + m] = sd * 2 * pz * h ** 3 * coef[m]!;
          }
          // p = h³·L(j): dp/dT = (3/h)·(dh/dT)·(p + c).
          if (free) row[2 * N] = sd * ((6 * dh) / h) * (px * (px + o.x) + pz * (pz + o.z));
          ji.push(row);
        }
      }
    }
    if (free) {
      // A sane range for T.
      ci.push(T - 0.3);
      const row = zero();
      row[2 * N] = 1;
      ji.push(row);
    }
    return { f, grad, ce, je, ci, ji };
  };
  const res = solveNlp(evaluate, x0, { tol: 1e-6, maxOuter: 25, maxInner: 400 });
  const T = tOf(res.x);
  const jx = res.x.slice(0, N);
  const jz = res.x.slice(N, 2 * N);
  const at = (t: number) => evalJerks(jx, jz, T, t);
  const m = measure(at, T, o);
  return {
    method: 'collocation',
    status: res.violation < 1e-3 ? 'optimal' : 'infeasible',
    duration: T,
    at,
    ...m,
    iterations: res.inner,
  };
}

const dot = (a: readonly number[], b: readonly number[]) => {
  let s = 0;
  for (let i = 0; i < a.length; i++) s += a[i]! * b[i]!;
  return s;
};

/** The manoeuvre lesson IV.6 flies, from the parameters (relative to the base setpoint). */
export function manoeuvreSpec(p: Params): ManoeuvreSpec {
  const aH = GRAVITY * Math.tan((p.control.l3.maxTiltDeg * Math.PI) / 180);
  const w = p.world;
  return {
    distance: p.plan.distance,
    duration: p.plan.duration,
    minTime: p.plan.method === 'collocation' && p.plan.minTime,
    accelMax: aH * p.plan.accelFraction,
    jerkMax: JERK_MAX,
    obstacle: w.pillar
      ? { x: w.pillarX - p.setpoint.x, z: w.pillarZ - p.setpoint.z, r: w.pillarR + DRONE_RADIUS }
      : { x: 0, z: 0, r: 0 },
    nodes: 20,
  };
}

let cache: { key: string; plan: Manoeuvre } | null = null;
/** The plan for these parameters, solved once and cached. */
export function manoeuvreFor(p: Params): Manoeuvre {
  const spec = manoeuvreSpec(p);
  const key = JSON.stringify([p.plan.method, spec]);
  if (cache?.key !== key) {
    cache = {
      key,
      plan: p.plan.method === 'collocation' ? planCollocation(spec) : planMinSnap(spec),
    };
  }
  return cache.plan;
}
