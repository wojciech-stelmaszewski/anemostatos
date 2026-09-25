import { describe, expect, it } from 'vitest';
import { v3 } from '@/math/vec3';
import { defaultParams } from '@/sim/params';
import { Wind } from '@/sim/wind';

const dt = 0.001;

describe('Wind', () => {
  it('same seed → identical wind', () => {
    const p = defaultParams().wind;
    const a = new Wind(99);
    const b = new Wind(99);
    for (let k = 0; k < 20000; k++) {
      const wa = a.step(k * dt, dt, p);
      const wb = b.step(k * dt, dt, p);
      expect(wa).toEqual(wb);
    }
  });

  it('OU turbulence has the configured standard deviation', () => {
    const p = { ...defaultParams().wind, gustsOn: false, turbSigma: 1.3, turbTau: 0.5 };
    const w = new Wind(3);
    let sq = 0;
    let n = 0;
    for (let k = 0; k < 400_000; k++) {
      const v = w.step(k * dt, dt, p);
      if (k > 5000) {
        sq += v.x * v.x;
        n++;
      }
    }
    expect(Math.sqrt(sq / n)).toBeGreaterThan(1.1);
    expect(Math.sqrt(sq / n)).toBeLessThan(1.5);
  });

  it('a 1−cos gust peaks at its amplitude halfway through', () => {
    const p = { ...defaultParams().wind, gustsOn: false, turbSigma: 0 };
    const w = new Wind(1);
    w.trigger(0, v3(0, 1, 0), 5, 2);
    expect(w.step(1, dt, p).y).toBeCloseTo(5, 6);
    expect(w.step(1.999, dt, p).y).toBeLessThan(0.01);
    expect(w.step(2.5, dt, p).y).toBe(0);
  });
});
