import { beforeAll, describe, expect, it } from 'vitest';
import {
  cleanRunsNeeded,
  DISPERSIONS,
  finishedCampaign,
  member,
  passRateBound,
  RATE_LIMIT_DEG,
  runDispersionCampaign,
  type Run,
} from '@/analysis/dispersion';
import { analyseRateLoop } from '@/analysis/mimo';
import { Simulation } from '@/engine/simulation';
import { LESSONS } from '@/lessons/lessons';
import { defaultParams, type Params } from '@/sim/params';

describe('what clean runs prove', () => {
  it('299 clean runs show 99 % at 95 % confidence, 298 do not', () => {
    expect(cleanRunsNeeded(0.99)).toBe(299);
    expect(cleanRunsNeeded(0.999)).toBe(2995);
    expect(passRateBound(299, 299)).toBeGreaterThanOrEqual(0.99);
    expect(passRateBound(298, 298)).toBeLessThan(0.99);
    // The rule of three: N clean runs rule out a failure rate of about 3/N.
    expect(1 - passRateBound(300, 300)).toBeCloseTo(3 / 300, 3);
  });

  it('a failure lowers the bound, and the bound stays below the observed rate', () => {
    let prev = 1;
    for (const k of [0, 1, 3, 10]) {
      const b = passRateBound(300 - k, 300);
      expect(b).toBeLessThan(prev);
      expect(b).toBeLessThan((300 - k) / 300 + 1e-12);
      prev = b;
    }
    // Clopper–Pearson, one-sided 95 %, 3 failures in 300: 0.9746 (from the beta quantile).
    expect(passRateBound(297, 300)).toBeCloseTo(0.9746, 3);
  });
});

describe('a Monte Carlo campaign of lesson III.29', () => {
  const lesson = LESSONS.find((l) => l.id === 'montecarlo')!;
  const setup = (mod?: (p: Params) => void): Params => {
    const p = defaultParams();
    p.sim.level = 3;
    lesson.setup!(p);
    mod?.(p);
    return p;
  };
  const tuned = setup((p) => lesson.solution!(p));
  let nominal: Run[];
  let retuned: Run[];

  beforeAll(() => {
    nominal = runDispersionCampaign(setup());
    retuned = runDispersionCampaign(tuned);
    // Two campaigns of 300 flights: about a minute alone, several on a busy machine.
  }, 900_000);

  it('the nominal flight passes, and a few per cent of the campaign does not', () => {
    const sim = new Simulation(setup());
    for (let k = 0; k < 20000; k++) sim.step();
    expect(sim.state.crashed).toBe(false);
    const failed = nominal.filter((r) => !r.passed);
    expect(failed.length).toBeGreaterThan(8);
    expect(failed.length).toBeLessThan(30);
    expect(passRateBound(300 - failed.length, 300)).toBeLessThan(0.97);
  });

  it('every failure sits in one corner: slow motors and a late sensor', () => {
    const tau = DISPERSIONS.findIndex((d) => d.id === 'tau');
    const delay = DISPERSIONS.findIndex((d) => d.id === 'delay');
    for (const r of nominal.filter((x) => !x.passed)) {
      expect(r.values[tau]!).toBeGreaterThan(25);
      expect(r.values[delay]!).toBeGreaterThan(30);
    }
    // And the corner is a loop without phase margin: the model agrees with the flights.
    let agree = 0;
    for (const r of nominal) {
      const stable = analyseRateLoop(member(setup(), r.values), 200)!.stable;
      if (stable === r.passed) agree++;
    }
    expect(agree).toBeGreaterThanOrEqual(295);
  });

  it('the limit sits in a wide gap: gusts stay far below it, oscillations far above', () => {
    const passed = nominal.filter((r) => r.passed).map((r) => r.peakRate);
    const failed = nominal.filter((r) => !r.passed).map((r) => r.peakRate);
    expect(Math.max(...passed)).toBeLessThan(RATE_LIMIT_DEG / 1.8);
    expect(Math.min(...failed)).toBeGreaterThan(RATE_LIMIT_DEG * 1.6);
  });

  it('a gentler attitude loop flies the same 300 drones without a failure', () => {
    expect(retuned.map((r) => r.values)).toEqual(nominal.map((r) => r.values));
    expect(retuned.filter((r) => !r.passed)).toHaveLength(0);
  });

  it('the goal reads the finished campaign of the current tune', () => {
    const goal = lesson.goal!.check;
    const ctx = (p: Params, prediction: number | null) => ({
      sim: new Simulation(p),
      metrics: null,
      prediction,
    });
    expect(goal(ctx(tuned, null))).not.toBe(true);
    expect(goal(ctx(tuned, 200))).not.toBe(true);
    expect(goal(ctx(setup(), 299))).toMatch(/failed/);
    expect(goal(ctx(tuned, 299))).toBe(true);
    // A tune nobody has flown has no campaign yet.
    const untried = setup((p) => (p.control.l3.attKpRP = 4));
    expect(finishedCampaign(untried)).toBeNull();
    expect(goal(ctx(untried, 299))).toMatch(/Fly 300/);
  });
});
