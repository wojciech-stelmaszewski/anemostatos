// Spectra (docs/analysis.md §3.1): a radix-2 FFT, Welch's power spectral density, and the
// single-frequency correlation that the frequency-response sweeps use.
import type { Complex } from './complex';

/** In-place radix-2 FFT of the complex signal (re, im). The length must be a power of two. */
export function fft(re: number[] | Float64Array, im: number[] | Float64Array): void {
  const n = re.length;
  if (n === 0 || (n & (n - 1)) !== 0) throw new Error('fft: length must be a power of two');
  // Bit reversal.
  for (let i = 1, j = 0; i < n; i++) {
    let bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      [re[i], re[j]] = [re[j]!, re[i]!];
      [im[i], im[j]] = [im[j]!, im[i]!];
    }
  }
  for (let len = 2; len <= n; len <<= 1) {
    const ang = (-2 * Math.PI) / len;
    const wr = Math.cos(ang);
    const wi = Math.sin(ang);
    for (let i = 0; i < n; i += len) {
      let cr = 1;
      let ci = 0;
      for (let k = 0; k < len / 2; k++) {
        const a = i + k;
        const b = a + len / 2;
        const tr = re[b]! * cr - im[b]! * ci;
        const ti = re[b]! * ci + im[b]! * cr;
        re[b] = re[a]! - tr;
        im[b] = im[a]! - ti;
        re[a] = re[a]! + tr;
        im[a] = im[a]! + ti;
        const nr = cr * wr - ci * wi;
        ci = cr * wi + ci * wr;
        cr = nr;
      }
    }
  }
}

export interface Psd {
  /** Frequencies, Hz, from 0 to fs/2. */
  f: number[];
  /** One-sided power spectral density, units²/Hz. Its integral over f is the signal's variance. */
  psd: number[];
}

/**
 * Welch's method: Hann-windowed segments of `segment` samples (a power of two) with 50 % overlap,
 * each with its mean removed, averaged.
 */
export function welch(x: ArrayLike<number>, fs: number, segment = 1024): Psd {
  let n = segment;
  while (n > x.length) n >>= 1;
  if (n < 8) throw new Error('welch: signal too short');
  const win = Array.from({ length: n }, (_, i) => 0.5 - 0.5 * Math.cos((2 * Math.PI * i) / n));
  const wss = win.reduce((s, v) => s + v * v, 0);
  const half = n / 2;
  const acc = new Array<number>(half + 1).fill(0);
  let count = 0;
  for (let start = 0; start + n <= x.length; start += half) {
    let mean = 0;
    for (let i = 0; i < n; i++) mean += x[start + i]!;
    mean /= n;
    const re = new Float64Array(n);
    const im = new Float64Array(n);
    for (let i = 0; i < n; i++) re[i] = (x[start + i]! - mean) * win[i]!;
    fft(re, im);
    for (let k = 0; k <= half; k++) {
      const p = (re[k]! * re[k]! + im[k]! * im[k]!) / (fs * wss);
      acc[k] = acc[k]! + (k === 0 || k === half ? p : 2 * p);
    }
    count++;
  }
  return {
    f: acc.map((_, k) => (k * fs) / n),
    psd: acc.map((v) => v / count),
  };
}

/**
 * The complex amplitude X of the component of `x` at `freqHz`, so that the component is
 * Re(X·e^{j2πft}) with t = 0 at the first sample. Exact when the samples span a whole number of
 * periods; the caller chooses the span.
 */
export function correlate(x: ArrayLike<number>, freqHz: number, dt: number): Complex {
  const w = 2 * Math.PI * freqHz * dt;
  let re = 0;
  let im = 0;
  for (let k = 0; k < x.length; k++) {
    re += x[k]! * Math.cos(w * k);
    im -= x[k]! * Math.sin(w * k);
  }
  return { re: (2 * re) / x.length, im: (2 * im) / x.length };
}
