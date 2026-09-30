// Stability margins of a single loop from its sampled open-loop frequency response L(jω)
// (docs/analysis.md §3.1). Negative feedback is assumed: the closed loop is L/(1 + L).
import { cabs, cadd, carg, C_ONE, type Complex } from './complex';

export interface Margins {
  /** Gain crossover frequencies, rad/s (where |L| = 1), and the phase margin at each, degrees. */
  gainCrossovers: { w: number; pmDeg: number }[];
  /** Phase crossover frequencies, rad/s (where ∠L = −180° mod 360°), and the gain margin there. */
  phaseCrossovers: { w: number; gm: number; gmDb: number }[];
  /** Smallest phase margin, degrees (Infinity if |L| never crosses 1). */
  pmDeg: number;
  /** Frequency of that crossover, rad/s (NaN if none). */
  wc: number;
  /**
   * Gain margin: the smallest factor > 1 by which the loop gain can grow before instability
   * (Infinity if none). `gmLow` is the largest factor < 1 by which it can shrink (0 if none):
   * conditionally stable loops have both.
   */
  gm: number;
  gmLow: number;
  /** Frequency of the upper gain margin, rad/s (NaN if none). */
  w180: number;
  /** Delay margin: the extra pure delay that uses up the phase margin, s. */
  delayMargin: number;
  /** Peak of the sensitivity |1/(1 + L)| and its frequency. 1/ms is the distance from L to −1. */
  ms: number;
  wMs: number;
}

const DEG = 180 / Math.PI;

/** Unwrapped phase in radians, continuous along the frequency grid. */
export function unwrapPhase(l: readonly Complex[]): number[] {
  const out: number[] = [];
  let prev = 0;
  let offset = 0;
  l.forEach((v, i) => {
    const p = carg(v);
    if (i > 0) {
      const d = p - prev;
      if (d > Math.PI) offset -= 2 * Math.PI;
      else if (d < -Math.PI) offset += 2 * Math.PI;
    }
    prev = p;
    out.push(p + offset);
  });
  return out;
}

/**
 * Margins of the loop sampled at the angular frequencies `w` (ascending, ideally log-spaced and
 * dense near the crossovers). Crossings are located by interpolation in log-frequency.
 */
export function margins(w: readonly number[], l: readonly Complex[]): Margins {
  const n = w.length;
  const mag = l.map((v) => Math.log(cabs(v))); // natural log of |L|
  const ph = unwrapPhase(l);
  const lw = w.map((v) => Math.log(v));
  const gainCrossovers: Margins['gainCrossovers'] = [];
  const phaseCrossovers: Margins['phaseCrossovers'] = [];

  for (let i = 0; i + 1 < n; i++) {
    // |L| = 1  ⇔  ln|L| = 0.
    const m0 = mag[i]!;
    const m1 = mag[i + 1]!;
    if ((m0 === 0 || m0 * m1 < 0) && m0 !== m1) {
      const f = m0 / (m0 - m1);
      const phase = ph[i]! + f * (ph[i + 1]! - ph[i]!);
      // Distance of the phase from the nearest odd multiple of −180°.
      const k = Math.round((phase + Math.PI) / (2 * Math.PI));
      gainCrossovers.push({
        w: Math.exp(lw[i]! + f * (lw[i + 1]! - lw[i]!)),
        pmDeg: (phase + Math.PI - 2 * Math.PI * k) * DEG,
      });
    }
    // ∠L = −180° + 360°·k: the quantity (phase + π) crosses a multiple of 2π.
    const a0 = (ph[i]! + Math.PI) / (2 * Math.PI);
    const a1 = (ph[i + 1]! + Math.PI) / (2 * Math.PI);
    const lo = Math.min(a0, a1);
    const hi = Math.max(a0, a1);
    for (let k = Math.ceil(lo); k <= Math.floor(hi); k++) {
      if (a0 === a1 || (k === a1 && i + 2 < n)) continue; // counted in the next interval
      const f = (k - a0) / (a1 - a0);
      const gm = Math.exp(-(m0 + f * (m1 - m0)));
      phaseCrossovers.push({
        w: Math.exp(lw[i]! + f * (lw[i + 1]! - lw[i]!)),
        gm,
        gmDb: 20 * Math.log10(gm),
      });
    }
  }

  let pmDeg = Infinity;
  let wc = NaN;
  for (const c of gainCrossovers) {
    if (Math.abs(c.pmDeg) < Math.abs(pmDeg) || pmDeg === Infinity) {
      pmDeg = c.pmDeg;
      wc = c.w;
    }
  }
  let gm = Infinity;
  let w180 = NaN;
  let gmLow = 0;
  for (const c of phaseCrossovers) {
    if (c.gm > 1 && c.gm < gm) {
      gm = c.gm;
      w180 = c.w;
    }
    if (c.gm < 1 && c.gm > gmLow) gmLow = c.gm;
  }
  let ms = 0;
  let wMs = NaN;
  l.forEach((v, i) => {
    const s = 1 / cabs(cadd(C_ONE, v));
    if (s > ms) {
      ms = s;
      wMs = w[i]!;
    }
  });
  const delayMargin = Number.isFinite(pmDeg) && wc > 0 ? Math.max(pmDeg, 0) / DEG / wc : Infinity;
  return { gainCrossovers, phaseCrossovers, pmDeg, wc, gm, gmLow, w180, delayMargin, ms, wMs };
}

/** `count` logarithmically spaced values from `from` to `to` inclusive. */
export function logspace(from: number, to: number, count: number): number[] {
  const a = Math.log(from);
  const b = Math.log(to);
  return Array.from({ length: count }, (_, i) =>
    Math.exp(a + ((b - a) * i) / Math.max(count - 1, 1)),
  );
}
