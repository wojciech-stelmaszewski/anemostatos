import { describe, expect, it } from 'vitest';
import { HoverFlight, measureBudget, windAllocation } from '@/analysis/budget';
import { runVerification } from '@/analysis/campaign';
import {
  measuredMargins,
  modelMargins,
  REVIEW_FREQS,
  runReview,
  smallerGm,
} from '@/analysis/review';
import {
  fixedDisagreement,
  flyFor,
  peakOf,
  trackingPeak,
  transferJump,
  voterError,
} from '@/analysis/software';
import { runSweep } from '@/analysis/sweep';
import { integratorDeadZone, integratorRange, wrap } from '@/control/part5/fixedpoint';
import { Simulation } from '@/engine/simulation';
import { LESSONS } from '@/lessons/lessons';
import type { Lesson } from '@/lessons/types';
import { defaultParams, type Params } from '@/sim/params';

const lesson = (id: string): Lesson => LESSONS.find((l) => l.id === id)!;
/** The parameters of a lesson's start, with a change. */
const setup = (id: string, mod: (p: Params) => void = () => {}): Params => {
  const l = lesson(id);
  const p = defaultParams();
  p.sim.level = l.level;
  l.setup!(p);
  mod(p);
  return p;
};
const solved = (id: string) => setup(id, lesson(id).solution!);
const check = (id: string, sim: Simulation) => {
  const l = lesson(id);
  return l.goal!.check({ sim, metrics: null, prediction: l.predict?.truth(sim.params) ?? null });
};
const HEAVY = 300_000;

describe('Part V', () => {
  it('is numbered V.1 to V.6, in order', () => {
    const five = LESSONS.filter((l) => l.part === 5);
    expect(five.map((l) => l.n)).toEqual([1, 2, 3, 4, 5, 6]);
    expect(five.map((l) => l.id)).toEqual([
      'budget',
      'campaign',
      'fixedpoint',
      'voter',
      'bumpless',
      'review',
    ]);
  });
});

describe('lesson V.1: the error budget', () => {
  it('leaves the wind √((8 − 3.5)² − 1.5²) = 4.24 cm', () => {
    expect(windAllocation() * 100).toBeCloseTo(4.24, 2);
  });

  it(
    'with the Part I gains the wind and the noise are over their allocations',
    () => {
      const r = measureBudget(setup('budget'));
      const over = r.entries.filter((e) => !e.within).map((e) => e.source.id);
      expect(over).toEqual(['wind', 'noise']);
      expect(r.closes).toBe(false);
      // The budget from the entries predicts the confirmation flight.
      expect(Math.abs(r.predicted / r.flown.total - 1)).toBeLessThan(0.05);
    },
    HEAVY,
  );

  it(
    'the solution closes the budget, and the budget predicts the flight within 5 %',
    () => {
      const r = measureBudget(solved('budget'));
      expect(r.entries.every((e) => e.within)).toBe(true);
      expect(r.closes).toBe(true);
      expect(Math.abs(r.predicted / r.flown.total - 1)).toBeLessThan(0.06);
      const sim = new Simulation(solved('budget'));
      expect(check('budget', sim)).toBe(true);
    },
    HEAVY,
  );

  it(
    'a 20 Hz D filter at high gain lets the noise move the mean',
    () => {
      const r = measureBudget(solved('budget'));
      const wide = measureBudget(
        setup('budget', (p) => {
          lesson('budget').solution!(p);
          p.control.alt.dFilterHz = 20;
        }),
      );
      const noise = (x: typeof r) => x.entries.find((e) => e.source.id === 'noise')!;
      expect(noise(wide).within).toBe(false);
      expect(noise(wide).stats.mean).toBeGreaterThan(0.005);
      expect(Math.abs(noise(r).stats.mean)).toBeLessThan(0.002);
    },
    HEAVY,
  );
});

describe('lesson V.2: the verification campaign', () => {
  it('20 clean flights prove 86 % with 95 % confidence', () => {
    expect(lesson('campaign').predict!.truth(defaultParams())).toBeCloseTo(86.1, 1);
  });

  it(
    'the design of V.1 overshoots on drones with slow motors; the solution is green',
    () => {
      const before = runVerification(setup('campaign'));
      const r2 = before.find((r) => r.requirement.id === 'R2')!;
      expect(r2.green).toBe(false);
      expect(r2.worst.values[1]).toBeGreaterThan(45); // motor lag, ms
      const after = runVerification(solved('campaign'));
      expect(after.every((r) => r.green)).toBe(true);
      expect(check('campaign', new Simulation(solved('campaign')))).toBe(true);
    },
    HEAVY,
  );

  it(
    'with the default gusts, two wind seeds in nine give the V.1 design 30 to 60 cm',
    () => {
      const totals = [1, 2, 3, 4, 5, 6, 7, 8, 1337].map((seed) => {
        const p = defaultParams();
        p.sim.level = 1;
        p.sim.seed = seed;
        lesson('budget').solution!(p);
        const f = new HoverFlight(p);
        f.advance(1e9);
        return f.stats.total;
      });
      expect(totals.filter((t) => t > 0.3 && t < 0.65)).toHaveLength(2);
      expect(totals.filter((t) => t > 0.1)).toHaveLength(2);
    },
    HEAVY,
  );
});

describe('lesson V.3: fixed point', () => {
  it('wraps like a 16-bit word', () => {
    expect(wrap(32767 + 1, 16)).toBe(-32768);
    expect(wrap(-32768 - 1, 16)).toBe(32767);
    expect(integratorRange(16, 11)).toBe(16);
    expect(integratorRange(16, 12)).toBe(8);
  });

  it('the dead zone at 8 fractional bits is 1.22 m', () => {
    const p = setup('fixedpoint');
    expect(lesson('fixedpoint').predict!.truth(p)).toBeCloseTo(122, 0);
    expect(integratorDeadZone(0.8, 0.004, 11)).toBeCloseTo(0.153, 3);
  });

  const worst = (mod: (p: Params) => void) =>
    fixedDisagreement(flyFor(setup('fixedpoint', mod), 30));
  const word =
    (bits: 16 | 32, frac: number, ff = false) =>
    (p: Params) => {
      p.part5.fixed.intBits = bits;
      p.part5.fixed.intFrac = frac;
      p.control.feedforward = ff;
    };

  it(
    'no 16-bit integrator holds the weight; a 32-bit one, or the feedforward, does',
    () => {
      expect(worst(word(16, 8))).toBeGreaterThan(50);
      const eleven = worst(word(16, 11));
      expect(eleven).toBeGreaterThan(10);
      expect(eleven).toBeLessThan(20);
      expect(worst(word(16, 12))).toBeGreaterThan(50); // wraps round
      expect(worst(word(32, 16))).toBeLessThan(1);
      expect(worst(word(16, 12, true))).toBeLessThan(1);
      expect(worst(word(16, 8, true))).toBeGreaterThan(1);
    },
    HEAVY,
  );

  it(
    'the solution reaches the goal; the start does not',
    () => {
      expect(check('fixedpoint', flyFor(solved('fixedpoint'), 30.1))).toBe(true);
      expect(check('fixedpoint', flyFor(setup('fixedpoint'), 30.1))).not.toBe(true);
    },
    HEAVY,
  );
});

describe('lesson V.4: three altimeters', () => {
  const fly = (mode: Params['part5']['voter']['mode']) =>
    flyFor(
      setup('voter', (p) => (p.part5.voter.mode = mode)),
      40.1,
    );

  it(
    'A alone crashes; mid-value selection masks the first failure but not the second',
    () => {
      const single = fly('single');
      expect(single.state.crashed || voterError(single) > 1).toBe(true);
      const mid = fly('mid');
      expect(peakOf(mid, 'vote.err', 1, 20)).toBeLessThan(0.05);
      expect(voterError(mid)).toBeGreaterThan(0.5);
    },
    HEAVY,
  );

  it(
    'the monitor isolates A, then B at the predicted time, and holds within 20 cm',
    () => {
      const sim = fly('monitor');
      expect(voterError(sim)).toBeLessThan(0.2);
      const w = sim.telemetry.window(['vote.healthy'], 0);
      const lost = w.t[w.series[0]!.findIndex((h) => h === 1)]!;
      const predicted = lesson('voter').predict!.truth(sim.params);
      expect(predicted).toBeCloseTo(3.2, 6);
      expect(Math.abs(lost - 20 - predicted)).toBeLessThan(0.15);
      expect(check('voter', sim)).toBe(true);
    },
    HEAVY,
  );
});

describe('lesson V.5: the hand-over', () => {
  it(
    'the kick is the payload’s weight; bumpless removes the sag',
    () => {
      const plain = flyFor(setup('bumpless'), 23.1);
      const smooth = flyFor(solved('bumpless'), 23.1);
      const jump = transferJump(plain.params);
      expect(jump).toBeCloseTo(2.45, 2);
      expect(Math.abs(peakOf(plain, 'xfer.jump', 15) / jump - 1)).toBeLessThan(0.03);
      expect(trackingPeak(plain, 11, 15)).toBeLessThan(0.02);
      expect(trackingPeak(plain, 15, 23)).toBeGreaterThan(0.15);
      expect(trackingPeak(smooth, 15, 23)).toBeLessThan(0.01);
      expect(check('bumpless', smooth)).toBe(true);
      expect(check('bumpless', plain)).not.toBe(true);
    },
    HEAVY,
  );
});

describe('lesson V.6: the design review', () => {
  it(
    'the delivered design fails model, margins, faults and software; its campaign passes',
    () => {
      const items = runReview(setup('review'));
      expect(items.filter((i) => !i.passed).map((i) => i.id)).toEqual([
        'model',
        'margins',
        'faults',
        'software',
      ]);
    },
    HEAVY,
  );

  it(
    'the solution closes every item',
    () => {
      const p = solved('review');
      const items = runReview(p);
      expect(items.every((i) => i.passed)).toBe(true);
      expect(check('review', new Simulation(p))).toBe(true);
    },
    HEAVY,
  );

  it(
    'the measured margins agree with the model for a plain PID',
    () => {
      const p = setup('review', (q) => (q.control.model.mass = 1));
      const measured = measuredMargins(runSweep(p, REVIEW_FREQS, { point: 'l1.thrust' }));
      const model = modelMargins(p)!;
      expect(Math.abs(measured.pmDeg - model.pmDeg)).toBeLessThan(2);
      expect(model.pmDeg).toBeLessThan(25);
    },
    HEAVY,
  );

  it(
    'PIDs with 45° of margin fail the campaign; PIDs that pass it have little margin',
    () => {
      const pid = (kp: number, kd: number, f: number) => {
        const p = defaultParams();
        p.sim.level = 1;
        Object.assign(p.control.alt, { kp, ki: kp / 10, kd, dFilterHz: f });
        return p;
      };
      for (const [kp, kd, f] of [
        [30, 10, 10],
        [40, 14, 15],
      ] as const) {
        const p = pid(kp, kd, f);
        expect(modelMargins(p)!.pmDeg).toBeGreaterThanOrEqual(45);
        expect(runVerification(p).every((r) => r.green)).toBe(false);
      }
      for (const [kp, kd, f] of [
        [75, 22, 6],
        [75, 22, 5],
      ] as const) {
        const p = pid(kp, kd, f);
        expect(runVerification(p).every((r) => r.green)).toBe(true);
        expect(modelMargins(p)!.pmDeg).toBeLessThan(30);
        expect(smallerGm(modelMargins(p)!)).toBeGreaterThan(5);
      }
    },
    HEAVY,
  );
});
