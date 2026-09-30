// Linear time-invariant systems in state space (docs/analysis.md §3.1):
//   continuous  ẋ = A·x + B·u,  y = C·x + D·u          (dt = 0)
//   discrete    x⁺ = A·x + B·u,  y = C·x + D·u          (dt > 0, the sample period)
import { C_ZERO, cadd, cis, cmul, csolve, cx, type CMat, type Complex } from './complex';
import {
  add,
  c2d,
  cols,
  eigenvalues,
  eye,
  mul,
  rows,
  scale,
  solve,
  sub,
  zeros,
  type Mat,
} from './mat';

export interface Lti {
  a: Mat;
  b: Mat;
  c: Mat;
  d: Mat;
  /** Sample period in seconds; 0 for a continuous-time system. */
  dt: number;
}

export const ss = (a: Mat, b: Mat, c: Mat, d?: Mat, dt = 0): Lti => ({
  a,
  b,
  c,
  d: d ?? zeros(rows(c), cols(b)),
  dt,
});

/** A static gain (no state). */
export const gain = (k: Mat, dt = 0): Lti => ({
  a: [],
  b: [],
  c: Array.from({ length: rows(k) }, () => []),
  d: k,
  dt,
});

export const order = (g: Lti): number => g.a.length;
export const inputs = (g: Lti): number => cols(g.d);
export const outputs = (g: Lti): number => rows(g.d);

/** The point on the stability boundary for the angular frequency w: jω, or e^{jωT}. */
export const boundaryPoint = (g: Lti, w: number): Complex => (g.dt > 0 ? cis(w * g.dt) : cx(0, w));

/** Transfer matrix C·(λI − A)⁻¹·B + D at the complex point λ (s or z). */
export function evalAt(g: Lti, lambda: Complex): CMat {
  const n = order(g);
  const p = outputs(g);
  const m = inputs(g);
  const out: CMat = g.d.map((r) => r.map((v) => cx(v)));
  if (n === 0) return out;
  const lhs: CMat = g.a.map((r, i) =>
    r.map((v, j) => (i === j ? { re: lambda.re - v, im: lambda.im } : cx(-v))),
  );
  const x = csolve(
    lhs,
    g.b.map((r) => r.map((v) => cx(v))),
  );
  for (let i = 0; i < p; i++)
    for (let j = 0; j < m; j++) {
      let s: Complex = C_ZERO;
      for (let k = 0; k < n; k++) s = cadd(s, cmul(cx(g.c[i]![k]!), x[k]![j]!));
      out[i]![j] = cadd(out[i]![j]!, s);
    }
  return out;
}

/** Frequency response at the angular frequency w (rad/s): an outputs × inputs complex matrix. */
export const freqResp = (g: Lti, w: number): CMat => evalAt(g, boundaryPoint(g, w));

/** Poles: the eigenvalues of A (in the s-plane, or the z-plane for a discrete system). */
export const poles = (g: Lti): Complex[] => (order(g) ? eigenvalues(g.a) : []);

/** True if every pole lies strictly inside the stability region. */
export const isStable = (g: Lti): boolean =>
  poles(g).every((z) => (g.dt > 0 ? Math.hypot(z.re, z.im) < 1 : z.re < 0));

const blocks = (parts: Mat[][]): Mat => {
  const out: Mat = [];
  for (const row of parts) {
    const h = Math.max(...row.map((m) => m.length));
    for (let i = 0; i < h; i++) out.push(row.flatMap((m) => m[i] ?? []));
  }
  return out;
};
const sameRate = (g: Lti, h: Lti): number => {
  if (g.dt !== h.dt) throw new Error('lti: systems with different sample periods');
  return g.dt;
};

/** `second` fed by `first`: y = second(first(u)). */
export function series(first: Lti, second: Lti): Lti {
  const dt = sameRate(first, second);
  const n1 = order(first);
  const n2 = order(second);
  return {
    a: blocks([
      [first.a, zeros(n1, n2)],
      [n2 ? mul(second.b, first.c) : [], second.a],
    ]),
    b: blocks([[first.b], [n2 ? mul(second.b, first.d) : []]]),
    c: blocks([[mul(second.d, first.c), second.c]]),
    d: mul(second.d, first.d),
    dt,
  };
}

/**
 * Negative feedback around a loop with no direct feedthrough: u = r − L(y).
 * Returns the closed loop from r to y of the forward system `g` with `k` in the return path
 * (the identity by default).
 */
export function feedback(g: Lti, k?: Lti): Lti {
  const h = k ?? gain(eye(outputs(g)), g.dt);
  const dt = sameRate(g, h);
  if (g.d.some((r) => r.some((v) => v !== 0)) && h.d.some((r) => r.some((v) => v !== 0)))
    throw new Error('lti: algebraic loop (both systems have direct feedthrough)');
  const ng = order(g);
  const nh = order(h);
  // u = r − (Ch·xh + Dh·y),  y = Cg·xg + Dg·u.  With Dg·Dh = 0 this resolves without an inverse
  // when Dg = 0:  y = Cg·xg.
  if (g.d.some((r) => r.some((v) => v !== 0)))
    throw new Error('lti: feedback needs a strictly proper forward system');
  const bgDh = mul(g.b, h.d);
  return {
    a: blocks([
      [sub(g.a, mul(bgDh, g.c)), nh ? scale(mul(g.b, h.c), -1) : zeros(ng, 0)],
      [nh ? mul(h.b, g.c) : [], h.a],
    ]),
    b: blocks([[g.b], [zeros(nh, inputs(g))]]),
    c: blocks([[g.c, zeros(outputs(g), nh)]]),
    d: zeros(outputs(g), inputs(g)),
    dt,
  };
}

/** Zero-order-hold discretisation of a continuous system. */
export function c2dZoh(g: Lti, dt: number): Lti {
  if (g.dt !== 0) throw new Error('lti: already discrete');
  if (!order(g)) return { ...g, dt };
  const d = c2d(g.a, g.b, dt);
  return { a: d.a, b: d.b, c: g.c, d: g.d, dt };
}

/** Tustin (bilinear) discretisation: s → (2/T)·(z − 1)/(z + 1). */
export function c2dTustin(g: Lti, dt: number): Lti {
  if (g.dt !== 0) throw new Error('lti: already discrete');
  const n = order(g);
  if (!n) return { ...g, dt };
  const h = dt / 2;
  const m = sub(eye(n), scale(g.a, h)); // I − A·T/2
  const mi = solve(m, eye(n));
  const cmi = mul(g.c, mi);
  return {
    a: mul(mi, add(eye(n), scale(g.a, h))),
    b: scale(mul(mi, g.b), dt),
    c: cmi,
    d: add(g.d, scale(mul(cmi, g.b), h)),
    dt,
  };
}

/** Response of a discrete system to a unit step on input `input`: one row of outputs per sample. */
export function stepResponse(g: Lti, steps: number, input = 0): number[][] {
  if (g.dt <= 0) throw new Error('lti: stepResponse needs a discrete system (use c2dZoh first)');
  const n = order(g);
  let x = new Array<number>(n).fill(0);
  const out: number[][] = [];
  for (let k = 0; k < steps; k++) {
    out.push(g.c.map((row, i) => row.reduce((s, v, j) => s + v * x[j]!, 0) + g.d[i]![input]!));
    x = g.a.map((row, i) => row.reduce((s, v, j) => s + v * x[j]!, 0) + g.b[i]![input]!);
  }
  return out;
}

/** Steady-state gain: −C·A⁻¹·B + D, or C·(I − A)⁻¹·B + D for a discrete system. */
export function dcGain(g: Lti): Mat {
  const n = order(g);
  if (!n) return g.d;
  const m = g.dt > 0 ? sub(eye(n), g.a) : scale(g.a, -1);
  return add(g.d, mul(g.c, solve(m, g.b)));
}
