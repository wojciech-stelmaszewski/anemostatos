/**
 * Small dense matrices as arrays of rows. Sized for control problems (≤ ~50×50), written for
 * clarity over speed; every function returns a new matrix.
 */
export type Mat = number[][];

export const zeros = (rows: number, cols = rows): Mat =>
  Array.from({ length: rows }, () => new Array<number>(cols).fill(0));

export const eye = (n: number): Mat => {
  const m = zeros(n);
  for (let i = 0; i < n; i++) m[i]![i] = 1;
  return m;
};

export const diag = (d: readonly number[]): Mat => {
  const m = zeros(d.length);
  d.forEach((v, i) => (m[i]![i] = v));
  return m;
};

/** Column vector from values. */
export const col = (v: readonly number[]): Mat => v.map((x) => [x]);

export const rows = (a: Mat): number => a.length;
export const cols = (a: Mat): number => a[0]?.length ?? 0;

export const clone = (a: Mat): Mat => a.map((r) => [...r]);

export const transpose = (a: Mat): Mat => {
  const t = zeros(cols(a), rows(a));
  for (let i = 0; i < rows(a); i++) for (let j = 0; j < cols(a); j++) t[j]![i] = a[i]![j]!;
  return t;
};

export const add = (a: Mat, b: Mat): Mat => a.map((r, i) => r.map((v, j) => v + b[i]![j]!));
export const sub = (a: Mat, b: Mat): Mat => a.map((r, i) => r.map((v, j) => v - b[i]![j]!));
export const scale = (a: Mat, s: number): Mat => a.map((r) => r.map((v) => v * s));

export const mul = (a: Mat, b: Mat): Mat => {
  const n = rows(a);
  const m = cols(b);
  const k = cols(a);
  if (k !== rows(b)) throw new Error(`mul: ${n}×${k} · ${rows(b)}×${m}`);
  const c = zeros(n, m);
  for (let i = 0; i < n; i++) {
    const ai = a[i]!;
    const ci = c[i]!;
    for (let p = 0; p < k; p++) {
      const v = ai[p]!;
      if (v === 0) continue;
      const bp = b[p]!;
      for (let j = 0; j < m; j++) ci[j]! += v * bp[j]!;
    }
  }
  return c;
};

/** Product of several matrices, left to right. */
export const mulAll = (...ms: Mat[]): Mat => ms.reduce((acc, m) => mul(acc, m));

/** Max-abs entry — used as a cheap norm for convergence tests. */
export const maxAbs = (a: Mat): number =>
  Math.max(0, ...a.map((r) => Math.max(...r.map(Math.abs))));

/** Solve A·X = B by LU decomposition with partial pivoting. Throws if A is singular. */
export function solve(a: Mat, b: Mat): Mat {
  const n = rows(a);
  const lu = clone(a);
  const x = clone(b);
  for (let k = 0; k < n; k++) {
    let piv = k;
    for (let i = k + 1; i < n; i++) if (Math.abs(lu[i]![k]!) > Math.abs(lu[piv]![k]!)) piv = i;
    if (Math.abs(lu[piv]![k]!) < 1e-14) throw new Error('solve: singular matrix');
    if (piv !== k) {
      [lu[k], lu[piv]] = [lu[piv]!, lu[k]!];
      [x[k], x[piv]] = [x[piv]!, x[k]!];
    }
    const pivot = lu[k]![k]!;
    for (let i = k + 1; i < n; i++) {
      const f = lu[i]![k]! / pivot;
      if (f === 0) continue;
      for (let j = k; j < n; j++) lu[i]![j]! -= f * lu[k]![j]!;
      for (let j = 0; j < cols(x); j++) x[i]![j]! -= f * x[k]![j]!;
    }
  }
  for (let k = n - 1; k >= 0; k--) {
    for (let j = 0; j < cols(x); j++) {
      let s = x[k]![j]!;
      for (let i = k + 1; i < n; i++) s -= lu[k]![i]! * x[i]![j]!;
      x[k]![j] = s / lu[k]![k]!;
    }
  }
  return x;
}

export const inv = (a: Mat): Mat => solve(a, eye(rows(a)));

/** Matrix exponential by scaling and squaring with a [6/6] Padé approximant. */
export function expm(a: Mat): Mat {
  const n = rows(a);
  const norm = Math.max(...a.map((r) => r.reduce((s, v) => s + Math.abs(v), 0)));
  const s = Math.max(0, Math.ceil(Math.log2(Math.max(norm, 1e-300))) + 1);
  const as = scale(a, 1 / 2 ** s);
  const c = [1, 1 / 2, 5 / 44, 1 / 66, 1 / 792, 1 / 15840, 1 / 665280];
  let num = eye(n);
  let den = eye(n);
  let p = eye(n);
  for (let k = 1; k < c.length; k++) {
    p = mul(p, as);
    num = add(num, scale(p, c[k]!));
    den = add(den, scale(p, (k % 2 ? -1 : 1) * c[k]!));
  }
  let e = solve(den, num);
  for (let k = 0; k < s; k++) e = mul(e, e);
  return e;
}

/**
 * Zero-order-hold discretisation of ẋ = A·x + B·u over dt:
 * expm([[A, B], [0, 0]]·dt) = [[Ad, Bd], [0, I]].
 */
export function c2d(a: Mat, b: Mat, dt: number): { a: Mat; b: Mat } {
  const n = rows(a);
  const m = cols(b);
  const big = zeros(n + m);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) big[i]![j] = a[i]![j]! * dt;
    for (let j = 0; j < m; j++) big[i]![n + j] = b[i]![j]! * dt;
  }
  const e = expm(big);
  return {
    a: e.slice(0, n).map((r) => r.slice(0, n)),
    b: e.slice(0, n).map((r) => r.slice(n)),
  };
}

/** Eigenvalues of a 2×2 matrix (as complex pairs) — enough for the pole readouts of L1 loops. */
export function eig2(a: Mat): { re: number; im: number }[] {
  const tr = a[0]![0]! + a[1]![1]!;
  const det = a[0]![0]! * a[1]![1]! - a[0]![1]! * a[1]![0]!;
  const disc = (tr * tr) / 4 - det;
  if (disc >= 0) {
    const r = Math.sqrt(disc);
    return [
      { re: tr / 2 + r, im: 0 },
      { re: tr / 2 - r, im: 0 },
    ];
  }
  const i = Math.sqrt(-disc);
  return [
    { re: tr / 2, im: i },
    { re: tr / 2, im: -i },
  ];
}

export interface Complex {
  re: number;
  im: number;
}

/**
 * Eigenvalues of a small square matrix: characteristic polynomial by Faddeev–LeVerrier, roots by
 * Durand–Kerner. Plenty for the ≤ 6-state pole readouts; not for large or ill-conditioned systems.
 */
export function eigenvalues(a: Mat): Complex[] {
  const n = rows(a);
  // Characteristic polynomial λⁿ + c[1]λⁿ⁻¹ + … + c[n].
  const c = [1];
  let m = zeros(n);
  for (let k = 1; k <= n; k++) {
    m = add(mul(a, m), scale(eye(n), c[k - 1]!));
    const am = mul(a, m);
    let tr = 0;
    for (let i = 0; i < n; i++) tr += am[i]![i]!;
    c.push(-tr / k);
  }
  const evalPoly = (z: Complex): Complex => {
    let re = 1;
    let im = 0;
    for (let k = 1; k <= n; k++) [re, im] = [re * z.re - im * z.im + c[k]!, re * z.im + im * z.re];
    return { re, im };
  };
  const radius = 1 + Math.max(...c.slice(1).map(Math.abs));
  let roots: Complex[] = Array.from({ length: n }, (_, k) => ({
    re: radius * 0.9 * Math.cos((2 * Math.PI * k) / n + 0.4),
    im: radius * 0.9 * Math.sin((2 * Math.PI * k) / n + 0.4),
  }));
  for (let it = 0; it < 500; it++) {
    let moved = 0;
    roots = roots.map((z, i) => {
      let dre = 1;
      let dim = 0;
      roots.forEach((w, j) => {
        if (j === i) return;
        const re = z.re - w.re;
        const im = z.im - w.im;
        [dre, dim] = [dre * re - dim * im, dre * im + dim * re];
      });
      const p = evalPoly(z);
      const den = dre * dre + dim * dim || 1e-300;
      const qre = (p.re * dre + p.im * dim) / den;
      const qim = (p.im * dre - p.re * dim) / den;
      moved = Math.max(moved, Math.hypot(qre, qim));
      return { re: z.re - qre, im: z.im - qim };
    });
    if (moved < 1e-13 * radius) break;
  }
  return roots.map((z) => ({ re: z.re, im: Math.abs(z.im) < 1e-9 * radius ? 0 : z.im }));
}

/** Continuous-time equivalent s = ln(z)/dt of a discrete pole. */
export const toContinuous = (z: Complex, dt: number): Complex => ({
  re: Math.log(Math.hypot(z.re, z.im)) / dt,
  im: Math.atan2(z.im, z.re) / dt,
});

/** LU factorisation with partial pivoting, reusable for many right-hand sides. */
export interface Lu {
  lu: Mat;
  perm: number[];
}

export function luFactor(a: Mat): Lu {
  const n = rows(a);
  const lu = clone(a);
  const perm = Array.from({ length: n }, (_, i) => i);
  for (let k = 0; k < n; k++) {
    let piv = k;
    for (let i = k + 1; i < n; i++) if (Math.abs(lu[i]![k]!) > Math.abs(lu[piv]![k]!)) piv = i;
    if (Math.abs(lu[piv]![k]!) < 1e-14) throw new Error('luFactor: singular matrix');
    if (piv !== k) {
      [lu[k], lu[piv]] = [lu[piv]!, lu[k]!];
      [perm[k], perm[piv]] = [perm[piv]!, perm[k]!];
    }
    const rk = lu[k]!;
    for (let i = k + 1; i < n; i++) {
      const ri = lu[i]!;
      const f = (ri[k]! /= rk[k]!);
      if (f === 0) continue;
      for (let j = k + 1; j < n; j++) ri[j]! -= f * rk[j]!;
    }
  }
  return { lu, perm };
}

/** Solve A·x = b for a vector b with a factorisation from `luFactor`. */
export function luSolve(f: Lu, b: readonly number[]): number[] {
  const n = f.lu.length;
  const x = f.perm.map((p) => b[p]!);
  for (let i = 0; i < n; i++) {
    const r = f.lu[i]!;
    let s = x[i]!;
    for (let j = 0; j < i; j++) s -= r[j]! * x[j]!;
    x[i] = s;
  }
  for (let i = n - 1; i >= 0; i--) {
    const r = f.lu[i]!;
    let s = x[i]!;
    for (let j = i + 1; j < n; j++) s -= r[j]! * x[j]!;
    x[i] = s / r[i]!;
  }
  return x;
}
