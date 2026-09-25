import { describe, expect, it } from 'vitest';
import { analyzeLastStep } from '@/engine/metrics';

describe('analyzeLastStep', () => {
  // Second-order underdamped response to a step 1 → 2 at t = 1 s.
  const zeta = 0.4;
  const wn = 4;
  const wd = wn * Math.sqrt(1 - zeta * zeta);
  const t: number[] = [];
  const sp: number[] = [];
  const y: number[] = [];
  for (let k = 0; k <= 1000; k++) {
    const tk = k * 0.01;
    t.push(tk);
    const on = tk >= 1;
    sp.push(on ? 2 : 1);
    const s = tk - 1;
    const r = on
      ? 1 -
        Math.exp(-zeta * wn * s) *
          (Math.cos(wd * s) + (zeta / Math.sqrt(1 - zeta * zeta)) * Math.sin(wd * s))
      : 0;
    y.push(1 + r);
  }

  it('measures overshoot of a known second-order system', () => {
    const m = analyzeLastStep(t, sp, y)!;
    const expected = 100 * Math.exp((-zeta * Math.PI) / Math.sqrt(1 - zeta * zeta));
    expect(m.t0).toBeCloseTo(1);
    expect(m.overshoot).toBeCloseTo(expected, 0);
  });

  it('reports rise and settling time and ~zero steady-state error', () => {
    const m = analyzeLastStep(t, sp, y)!;
    expect(m.riseTime).toBeGreaterThan(0.2);
    expect(m.riseTime).toBeLessThan(0.5);
    // 2 % settling ≈ 4 / (ζ ωn) = 2.5 s
    expect(m.settlingTime).toBeGreaterThan(1.8);
    expect(m.settlingTime).toBeLessThan(3);
    expect(Math.abs(m.steadyStateError!)).toBeLessThan(0.005);
  });

  it('returns null without a step', () => {
    expect(analyzeLastStep([0, 1], [1, 1], [0, 0])).toBeNull();
  });
});
