import { describe, expect, it } from 'vitest';
import { isLoopStable, l1Loop, loopGain } from '@/analysis/loop';
import {
  familyStable,
  muAnalysis,
  muMatrix,
  muMixedUpper,
  muUpper,
  robustnessScale,
  sigmaMax3,
} from '@/analysis/mu';
import { runSweep } from '@/analysis/sweep';
import { backsteppingGains } from '@/control/backstepping';
import { Simulation } from '@/engine/simulation';
import { LESSONS } from '@/lessons/lessons';
import { cabs, csub, cx, type Complex } from '@/math/complex';
import { Rng } from '@/math/prng';
import { defaultParams, type Params } from '@/sim/params';

const calm = (mod: (p: Params) => void = () => {}): Params => {
  const p = defaultParams();
  p.wind.enabled = false;
  mod(p);
  return p;
};
const pid = (kp: number, kd: number, mod: (p: Params) => void = () => {}) =>
  calm((p) => {
    p.control.alt.kp = kp;
    p.control.alt.kd = kd;
    mod(p);
  });

describe('the structured singular value', () => {
  it('σ̄ of a 3 × 3 complex matrix: no direction is stretched more', () => {
    const r = new Rng(3);
    for (let t = 0; t < 50; t++) {
      const a = [0, 1, 2].map(() => [0, 1, 2].map(() => cx(r.normal(), r.normal())));
      const s = sigmaMax3(a);
      for (let k = 0; k < 500; k++) {
        const x = [0, 1, 2].map(() => cx(r.normal(), r.normal()));
        const nx = Math.hypot(...x.flatMap((z) => [z.re, z.im]));
        const y = a.map((row) =>
          row.reduce(
            (acc: Complex, v, j) => ({
              re: acc.re + v.re * x[j]!.re - v.im * x[j]!.im,
              im: acc.im + v.re * x[j]!.im + v.im * x[j]!.re,
            }),
            cx(0),
          ),
        );
        expect(Math.hypot(...y.flatMap((z) => [z.re, z.im])) / nx).toBeLessThanOrEqual(s + 1e-9);
      }
    }
    expect(
      sigmaMax3([
        [cx(2), cx(0), cx(0)],
        [cx(0), cx(0, -3), cx(0)],
        [cx(0), cx(0), cx(1)],
      ]),
    ).toBeCloseTo(3, 9);
  });

  it('one perturbation at a time is the small-gain test; together μ lies between the bounds', () => {
    const t = cx(0.6, -0.4);
    // Only the delay block: the loop closes through −W·T, and μ = |W·T|.
    expect(muUpper(muMatrix(t, [0, 0, 0.8]))).toBeCloseTo(0.8 * cabs(t), 4);
    const m = muMatrix(t, [0.4, 0.3, 0.5]);
    const mu = muUpper(m);
    // Spectral radius ≤ μ ≤ σ̄ (without scaling), and the real block only lowers the bound.
    expect(mu).toBeLessThanOrEqual(sigmaMax3(m) + 1e-9);
    expect(muMixedUpper(m, mu)).toBeLessThanOrEqual(mu + 1e-6);
  });
});

describe('lesson III.14: what each test says about the family', () => {
  it('the lumped small-gain test, μ and the truth, on the fast tune', () => {
    const p = pid(30, 25);
    const a = muAnalysis(p, 1, 80)!;
    expect(a.peakSmallGain).toBeGreaterThan(1);
    // A disk per parameter is more pessimistic than the lumped disk; the real mass helps.
    expect(a.peakMu).toBeGreaterThan(a.peakSmallGain);
    expect(a.peakMuMixed).toBeLessThan(a.peakMu);
    expect(a.peakMuMixed).toBeGreaterThan(a.peakSmallGain);
    // And some members really are unstable.
    expect(familyStable(p, 1)).toBe(false);
  }, 300_000);

  it('only the delay, once measured, buys a guarantee; the truth agrees that it matters most', () => {
    const peak = (mod: (p: Params) => void) => muAnalysis(pid(30, 25, mod), 1, 80)!.peakMuMixed;
    expect(peak((p) => (p.uncertainty.delayMs = 0))).toBeLessThan(1);
    expect(peak((p) => (p.uncertainty.massPct = 0))).toBeGreaterThan(1);
    expect(peak((p) => (p.uncertainty.tauFactor = 1))).toBeGreaterThan(1);
    // The true margin, as a multiple of the ranges, on the grid.
    const k = (mod: (p: Params) => void) => robustnessScale(pid(30, 25, mod))!.grid;
    const delay = k((p) => (p.uncertainty.delayMs = 0));
    const mass = k((p) => (p.uncertainty.massPct = 0));
    const tau = k((p) => (p.uncertainty.tauFactor = 1));
    expect(delay).toBeGreaterThan(2.1);
    expect(mass).toBeGreaterThan(1.3);
    expect(mass).toBeLessThan(1.5);
    expect(tau).toBeLessThan(1.1);
    expect(delay).toBeGreaterThan(mass);
    expect(mass).toBeGreaterThan(tau);
  }, 600_000);

  it('on a gentler tune the lumped test points at the mass, the truth at the delay', () => {
    const s = (mod: (p: Params) => void) => robustnessScale(pid(20, 20, mod))!;
    const mass = s((p) => (p.uncertainty.massPct = 0));
    const delay = s((p) => (p.uncertainty.delayMs = 0));
    expect(mass.smallGain).toBeGreaterThan(delay.smallGain);
    expect(delay.grid).toBeGreaterThan(mass.grid);
    // The guarantees are guarantees: never above the truth.
    for (const r of [mass, delay]) {
      expect(r.smallGain).toBeLessThanOrEqual(r.grid + 1e-9);
      expect(r.mu).toBeLessThanOrEqual(r.grid + 1e-9);
    }
  }, 600_000);
});

/** A 1 m climb at 10 s: overshoot and 2 % settling time. */
const climb = (p: Params) => {
  const sim = new Simulation(p);
  sim.schedule(10, (s) => s.setTarget({ x: 0, y: 3, z: 0 }));
  let peak = -Infinity;
  let settle = 0;
  for (let k = 0; k < 14000; k++) {
    sim.step();
    if (sim.t < 10) continue;
    const y = sim.state.pos.y;
    peak = Math.max(peak, y);
    if (Math.abs(y - 3) > 0.02) settle = sim.t - 10;
  }
  return { overshoot: peak - 3, settle };
};
const slow = (p: Params) => {
  p.drone.motorTau = 0.15;
  p.control.model.motorTau = 0.15;
};
const pmOf = (p: Params) => {
  const m = l1Loop(p)!;
  const w = Array.from({ length: 800 }, (_, i) => 2 * Math.PI * 0.02 * 5000 ** (i / 799));
  const l = w.map((x) => loopGain(m, x));
  // Phase margin at the first gain crossover.
  for (let i = 1; i < l.length; i++)
    if (cabs(l[i - 1]!) >= 1 && cabs(l[i]!) < 1)
      return 180 + (Math.atan2(l[i]!.im, l[i]!.re) * 180) / Math.PI;
  return NaN;
};

describe('lesson III.16: backstepping through the motor lag', () => {
  it('the law has V̇ = −Σkᵢzᵢ² on the model: the closed loop is stable for any positive gains', () => {
    for (const k of [
      { k1: 1, k2: 1, k3: 1 },
      { k1: 4, k2: 4, k3: 15 },
      { k1: 10, k2: 2, k3: 30 },
    ]) {
      const p = calm((q) => {
        q.control.l1.kind = 'backstepping';
        q.control.backstepping = k;
      });
      expect(isLoopStable(l1Loop(p)!)).toBe(true);
      const g = backsteppingGains(p);
      expect(g.kE).toBeGreaterThan(0);
      expect(g.kV).toBeGreaterThan(0);
    }
  });

  it('its linear model is what the simulator flies', () => {
    const p = calm((q) => {
      q.control.l1.kind = 'backstepping';
      q.control.backstepping = { k1: 4, k2: 4, k3: 15 };
    });
    const m = l1Loop(p)!;
    for (const pt of runSweep(p, [0.3, 1, 3])) {
      const md = loopGain(m, 2 * Math.PI * pt.fHz);
      expect(cabs(csub(pt.l!, md)) / cabs(md)).toBeLessThan(0.02);
    }
  }, 300_000);

  it('with slow motors no PD or PID tune on a 36-point grid meets the goal; backstepping does', () => {
    const meets = (p: Params) => {
      const r = climb(p);
      return r.overshoot < 0.02 && r.settle < 1.6 && pmOf(p) >= 45;
    };
    let any = false;
    for (const kp of [5, 10, 15, 20, 30, 40])
      for (const kd of [3, 5, 7, 10, 14, 20])
        if (meets(pid(kp, kd, (p) => (slow(p), (p.control.alt.iOn = kp % 10 === 0))))) any = true;
    expect(any).toBe(false);
    // The fast PD tunes overshoot by 6 to 24 %.
    expect(
      climb(pid(15, 6, (p) => (slow(p), (p.control.alt.iOn = false)))).overshoot,
    ).toBeGreaterThan(0.06);
    expect(
      climb(pid(40, 12, (p) => (slow(p), (p.control.alt.iOn = false)))).overshoot,
    ).toBeLessThan(0.25);
    const bs = calm((p) => {
      slow(p);
      p.control.l1.kind = 'backstepping';
      p.control.backstepping = { k1: 4, k2: 4, k3: 15 };
    });
    expect(meets(bs)).toBe(true);
    expect(pmOf(bs)).toBeGreaterThan(75);
  }, 900_000);

  it('V never rises on a small step; on a large one with high gains the motors saturate and it can', () => {
    const vRise = (k: number, size: number) => {
      const p = calm((q) => {
        slow(q);
        q.control.l1.kind = 'backstepping';
        q.control.backstepping = { k1: k, k2: k, k3: 4 * k + 1 };
      });
      const sim = new Simulation(p);
      for (let i = 0; i < 10000; i++) sim.step();
      sim.setTarget({ x: 0, y: 2 + size, z: 0 });
      let prev = Infinity;
      let rise = 0;
      for (let i = 0; i < 8000; i++) {
        sim.step();
        if (i % 4) continue;
        const v = sim.controller.extras()['bs.V']!;
        if (v > prev) rise = Math.max(rise, v - prev);
        prev = v;
      }
      return rise;
    };
    expect(vRise(4, 0.3)).toBeLessThan(1e-9);
    expect(vRise(6, 4)).toBeGreaterThan(1e-4);
  }, 300_000);
});

describe('the lessons', () => {
  it('III.14 and III.16 are in order after III.13', () => {
    const ns = LESSONS.filter((l) => l.part === 3).map((l) => l.n ?? 0);
    expect(ns.indexOf(14)).toBeGreaterThan(ns.indexOf(13));
    expect(ns.indexOf(16)).toBeGreaterThan(ns.indexOf(14));
  });
});
