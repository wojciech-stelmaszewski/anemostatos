// Complex arithmetic for frequency responses and poles (docs/analysis.md §3.1).

export interface Complex {
  re: number;
  im: number;
}

export const cx = (re: number, im = 0): Complex => ({ re, im });
export const C_ZERO: Complex = { re: 0, im: 0 };
export const C_ONE: Complex = { re: 1, im: 0 };

export const cadd = (a: Complex, b: Complex): Complex => ({ re: a.re + b.re, im: a.im + b.im });
export const csub = (a: Complex, b: Complex): Complex => ({ re: a.re - b.re, im: a.im - b.im });
export const cmul = (a: Complex, b: Complex): Complex => ({
  re: a.re * b.re - a.im * b.im,
  im: a.re * b.im + a.im * b.re,
});
export const cscale = (a: Complex, s: number): Complex => ({ re: a.re * s, im: a.im * s });
export const cneg = (a: Complex): Complex => ({ re: -a.re, im: -a.im });
export const cconj = (a: Complex): Complex => ({ re: a.re, im: -a.im });
export const cabs = (a: Complex): number => Math.hypot(a.re, a.im);
/** Argument in (−π, π]. */
export const carg = (a: Complex): number => Math.atan2(a.im, a.re);

/** a / b, scaled to avoid overflow (Smith's method). */
export function cdiv(a: Complex, b: Complex): Complex {
  if (Math.abs(b.re) >= Math.abs(b.im)) {
    const r = b.im / b.re;
    const d = b.re + b.im * r;
    return { re: (a.re + a.im * r) / d, im: (a.im - a.re * r) / d };
  }
  const r = b.re / b.im;
  const d = b.re * r + b.im;
  return { re: (a.re * r + a.im) / d, im: (a.im * r - a.re) / d };
}

export const cexp = (a: Complex): Complex => {
  const m = Math.exp(a.re);
  return { re: m * Math.cos(a.im), im: m * Math.sin(a.im) };
};
/** e^{jθ}. */
export const cis = (theta: number): Complex => ({ re: Math.cos(theta), im: Math.sin(theta) });

export type CMat = Complex[][];

/** Solve A·X = B for complex matrices by Gaussian elimination with partial pivoting. */
export function csolve(a: CMat, b: CMat): CMat {
  const n = a.length;
  const m = b[0]?.length ?? 0;
  const aug = a.map((row, i) => [...row, ...b[i]!]);
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) if (cabs(aug[r]![col]!) > cabs(aug[piv]![col]!)) piv = r;
    if (cabs(aug[piv]![col]!) < 1e-300) throw new Error('csolve: singular matrix');
    [aug[col], aug[piv]] = [aug[piv]!, aug[col]!];
    const p = aug[col]![col]!;
    for (let r = col + 1; r < n; r++) {
      const f = cdiv(aug[r]![col]!, p);
      if (f.re === 0 && f.im === 0) continue;
      for (let k = col; k < n + m; k++) aug[r]![k] = csub(aug[r]![k]!, cmul(f, aug[col]![k]!));
    }
  }
  const x: CMat = Array.from({ length: n }, () => new Array<Complex>(m).fill(C_ZERO));
  for (let j = 0; j < m; j++) {
    for (let r = n - 1; r >= 0; r--) {
      let s = aug[r]![n + j]!;
      for (let k = r + 1; k < n; k++) s = csub(s, cmul(aug[r]![k]!, x[k]![j]!));
      x[r]![j] = cdiv(s, aug[r]![r]!);
    }
  }
  return x;
}
