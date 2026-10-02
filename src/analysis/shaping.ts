// Loop-shaping stages for the L1 PID (docs/analysis.md §4, Chapter F, lesson III.6): a lead, a
// lag and a notch in series after the PID, each a small discrete system at the controller's rate.
// The controller runs them sample by sample and the loop model multiplies them in, so the Bode
// chart and the flight see the same thing.
import { notchCoefs } from '@/estimation/filters';
import { c2dTustin, series, ss, type Lti } from '@/math/lti';
import type { Params } from '@/sim/params';

const TWO_PI = 2 * Math.PI;

/**
 * Lead: (s/ωz + 1)/(s/ωp + 1) with ωz < ωp. Unity at low frequency, ωp/ωz at high frequency, and
 * a phase lead peaking at √(ωz·ωp) of arcsin((α − 1)/(α + 1)), α = ωp/ωz.
 */
function lead(zHz: number, pHz: number): Lti {
  const wz = TWO_PI * zHz;
  const wp = TWO_PI * pHz;
  const g = wp / wz;
  return ss([[-wp]], [[1]], [[g * (wz - wp)]], [[g]]);
}

/**
 * Lag: (s + ωz)/(s + ωp) with ωp < ωz. Unity at high frequency, ωz/ωp at low frequency: more gain
 * against slow disturbances without touching the crossover, at the price of some phase there.
 */
function lag(zHz: number, pHz: number): Lti {
  const wz = TWO_PI * zHz;
  const wp = TWO_PI * pHz;
  return ss([[-wp]], [[1]], [[wz - wp]], [[1]]);
}

/** Notch as a biquad in direct form, as a state-space system (transposed form II). */
function notch(hz: number, q: number, dt: number): Lti {
  const c = notchCoefs(hz, q, dt);
  // y = b0·u + s1;  s1⁺ = b1·u − a1·y + s2;  s2⁺ = b2·u − a2·y.
  return ss(
    [
      [-c.a1, 1],
      [-c.a2, 0],
    ],
    [[c.b1 - c.a1 * c.b0], [c.b2 - c.a2 * c.b0]],
    [[1, 0]],
    [[c.b0]],
    dt,
  );
}

/** Whether any stage is switched on. */
export const shapingActive = (p: Params): boolean => {
  const s = p.control.shaping;
  return (
    (s.leadZHz > 0 && s.leadPHz > s.leadZHz) ||
    (s.lagPHz > 0 && s.lagZHz > s.lagPHz) ||
    s.notchHz > 0
  );
};

/** The active stages in series, discrete at period T; null when none is on. */
export function shapingLti(p: Params, T: number): Lti | null {
  const s = p.control.shaping;
  const parts: Lti[] = [];
  if (s.leadZHz > 0 && s.leadPHz > s.leadZHz) parts.push(c2dTustin(lead(s.leadZHz, s.leadPHz), T));
  if (s.lagPHz > 0 && s.lagZHz > s.lagPHz) parts.push(c2dTustin(lag(s.lagZHz, s.lagPHz), T));
  if (s.notchHz > 0) parts.push(notch(s.notchHz, s.notchQ, T));
  if (!parts.length) return null;
  return parts.slice(1).reduce((acc, g) => series(acc, g), parts[0]!);
}

/** Runs a single-input single-output discrete system sample by sample. */
export class StageRunner {
  private x: number[] = [];
  private key = '';

  reset(): void {
    this.x = [];
    this.key = '';
  }

  /** One sample: y = C·x + D·u, then x⁺ = A·x + B·u. The state restarts when the stages change. */
  step(g: Lti, key: string, u: number): number {
    if (key !== this.key || this.x.length !== g.a.length) {
      // Start at rest for a constant input u: the stages pass their DC gain from the first sample.
      this.x = steadyState(g, u);
      this.key = key;
    }
    const y = g.c[0]!.reduce((s, c, i) => s + c * this.x[i]!, 0) + g.d[0]![0]! * u;
    this.x = g.a.map((row, i) => row.reduce((s, a, j) => s + a * this.x[j]!, 0) + g.b[i]![0]! * u);
    return y;
  }
}

/** The state x with x = A·x + B·u for a constant u. */
function steadyState(g: Lti, u: number): number[] {
  const n = g.a.length;
  if (!n) return [];
  // (I − A)·x = B·u by Gaussian elimination (n ≤ 4).
  const m = g.a.map((row, i) => [...row.map((v, j) => (i === j ? 1 : 0) - v), g.b[i]![0]! * u]);
  for (let k = 0; k < n; k++) {
    let piv = k;
    for (let i = k + 1; i < n; i++) if (Math.abs(m[i]![k]!) > Math.abs(m[piv]![k]!)) piv = i;
    [m[k], m[piv]] = [m[piv]!, m[k]!];
    const d = m[k]![k]!;
    if (Math.abs(d) < 1e-14) return new Array<number>(n).fill(0);
    for (let i = 0; i < n; i++) {
      if (i === k) continue;
      const f = m[i]![k]! / d;
      for (let j = k; j <= n; j++) m[i]![j] = m[i]![j]! - f * m[k]![j]!;
    }
  }
  return m.map((row, i) => row[n]! / row[i]!);
}
