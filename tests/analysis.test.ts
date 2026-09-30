import { describe, expect, it } from 'vitest';
import { l1ControllerModel } from '@/analysis/controllers';
import {
  closedLoopPoles,
  isLoopStable,
  l1Loop,
  loopGain,
  referenceResponse,
  sensitivities,
} from '@/analysis/loop';
import { l1Plant, l1PlantContinuous, PLANT_OUT } from '@/analysis/plant';
import { runSweep, Sweep } from '@/analysis/sweep';
import { probeSignal } from '@/engine/probe';
import { Simulation } from '@/engine/simulation';
import { cabs, carg, cadd, C_ONE } from '@/math/complex';
import { freqResp } from '@/math/lti';
import { logspace, margins } from '@/math/margins';
import { defaultParams, type Params } from '@/sim/params';

const DEG = 180 / Math.PI;
const TWO_PI = 2 * Math.PI;
/** The acceptance band of milestone M13 (docs/analysis.md §5). */
const BAND_HZ = [0.1, 0.2, 0.5, 1, 2, 5, 10, 20];

const calm = (mod?: (p: Params) => void): Params => {
  const p = defaultParams();
  p.wind.enabled = false;
  mod?.(p);
  return p;
};

/** Largest difference between the measured and the modelled open loop: dB and degrees. */
function disagreement(p: Params, freqs = BAND_HZ) {
  const m = l1Loop(p)!;
  let db = 0;
  let deg = 0;
  for (const pt of runSweep(p, freqs)) {
    const model = loopGain(m, TWO_PI * pt.fHz);
    db = Math.max(db, Math.abs(20 * Math.log10(cabs(pt.l!) / cabs(model))));
    const d = (carg(pt.l!) - carg(model)) * DEG;
    deg = Math.max(deg, Math.abs(((d + 540) % 360) - 180));
  }
  return { db, deg };
}

describe('the probe', () => {
  it('is silent by default and leaves the run unchanged', () => {
    const a = new Simulation(defaultParams());
    const b = new Simulation({ ...defaultParams(), probe: { ...defaultParams().probe, amp: 2 } });
    for (let k = 0; k < 8000; k++) {
      a.step();
      b.step();
    }
    expect(b.state.pos.y).toBe(a.state.pos.y);
    expect(a.probe).toEqual({ in: 0, u: 0, uc: 0 });
  });

  it('adds its signal to the thrust command: u = uc + in', () => {
    const p = calm();
    const sim = new Simulation(p);
    for (let k = 0; k < 6000; k++) sim.step();
    sim.setParams({ ...p, probe: { ...p.probe, point: 'l1.thrust', amp: 0.4, freqHz: 2 } });
    for (let k = 0; k < 1500; k++) {
      sim.step();
      expect(sim.probe.u).toBeCloseTo(sim.probe.uc + sim.probe.in, 12);
    }
    expect(Math.abs(sim.probe.in)).toBeGreaterThan(0);
  });

  it('the chirp starts at f0 and ends at f1', () => {
    const pr = { ...defaultParams().probe, point: 'l1.thrust' as const, signal: 'chirp' as const };
    const crossings = (t0: number, t1: number) => {
      let n = 0;
      for (let t = t0; t < t1; t += 1e-4)
        if (probeSignal(pr, t) < 0 !== probeSignal(pr, t + 1e-4) < 0) n++;
      return n / 2 / (t1 - t0);
    };
    expect(crossings(0, 20)).toBeGreaterThan(pr.f0Hz);
    expect(crossings(0, 20)).toBeLessThan(0.5);
    expect(crossings(59, 60)).toBeCloseTo(pr.f1Hz, -0.5);
    expect(probeSignal(pr, 61)).toBe(0);
  });
});

describe('the plant model', () => {
  it('approaches the textbook 1/(m·s²·(τs + 1)) at low frequency', () => {
    const p = calm();
    for (const f of [0.1, 1, 5]) {
      const w = TWO_PI * f;
      const d = freqResp(l1Plant(p), w)[PLANT_OUT.y]![0]!;
      const c = freqResp(l1PlantContinuous(p), w)[PLANT_OUT.y]![0]!;
      expect(cabs(d) / cabs(c)).toBeCloseTo(1, 2);
      // The simulator's motor step uses the end-of-step thrust: half a physics step of lead.
      expect((carg(d) - carg(c)) * DEG).toBeCloseTo(w * 0.0005 * DEG, 1);
    }
  });
});

describe('model against measurement (M13 acceptance: 1 dB and 5° from 0.1 to 20 Hz)', () => {
  const cases: [string, (p: Params) => void][] = [
    ['the default PID', () => {}],
    [
      'PID with D on the error and 40 ms of sensor delay',
      (p) => {
        p.control.alt.derivativeOn = 'error';
        p.sensors.delayMs = 40;
      },
    ],
    ['PID sampled at 50 Hz', (p) => (p.control.rateHz = 50)],
    ['LQR', (p) => (p.control.l1.kind = 'lqr')],
    [
      'LQI with the motor-lag state',
      (p) => {
        p.control.l1.kind = 'lqr';
        p.control.lqr.integral = true;
        p.control.lqr.lagState = true;
      },
    ],
    [
      'LQR with the lag state and no motor telemetry',
      (p) => {
        p.control.l1.kind = 'lqr';
        p.control.lqr.lagState = true;
        p.sensors.motorFeedback = false;
      },
    ],
    ['ADRC', (p) => (p.control.l1.kind = 'adrc')],
  ];
  for (const [name, mod] of cases)
    it(name, () => {
      const { db, deg } = disagreement(calm(mod));
      expect(db).toBeLessThan(1);
      expect(deg).toBeLessThan(5);
      // The models are the controllers' own update rules, so the agreement is far tighter.
      expect(db).toBeLessThan(0.05);
      expect(deg).toBeLessThan(1);
    });

  it('the reference response Y/R', () => {
    const p = calm();
    const m = l1Loop(p)!;
    for (const pt of runSweep(p, [0.2, 1, 3], { point: 'ref.y', amp: 0.2 })) {
      const model = referenceResponse(m, TWO_PI * pt.fHz);
      expect(cabs(pt.yr!) / cabs(model)).toBeCloseTo(1, 1);
      expect(Math.abs(carg(pt.yr!) - carg(model)) * DEG).toBeLessThan(2);
    }
  });

  it('S and T are measured in the same run and add up to one', () => {
    const p = calm();
    const m = l1Loop(p)!;
    for (const pt of runSweep(p, [0.5, 2])) {
      const sum = cadd(pt.s!, pt.t!);
      expect(sum.re).toBeCloseTo(1, 9);
      expect(sum.im).toBeCloseTo(0, 9);
      const s = sensitivities(m, TWO_PI * pt.fHz).s;
      expect(cabs(pt.s!) / cabs(s)).toBeCloseTo(1, 2);
    }
  });

  it('controllers without a linear model are measured only', () => {
    expect(l1ControllerModel(calm((p) => (p.control.l1.kind = 'mpc')))).toBeNull();
    expect(l1ControllerModel(calm((p) => (p.control.l1.estimator = 'kalman')))).toBeNull();
    const pts = runSweep(
      calm((p) => (p.control.l1.kind = 'mpc')),
      [1],
    );
    expect(cabs(pts[0]!.l!)).toBeGreaterThan(0.1);
  });

  it('a sweep can be advanced in slices and gives the same points', () => {
    const p = calm();
    const whole = runSweep(p, [1, 4]);
    const s = new Sweep(p, [1, 4]);
    let slices = 0;
    while (!s.advance(777)) slices++;
    expect(slices).toBeGreaterThan(10);
    expect(s.points).toEqual(whole);
  });
});

describe('margins and poles of the altitude loop', () => {
  const marginsOf = (p: Params) => {
    const m = l1Loop(p)!;
    const w = logspace(TWO_PI * 0.02, TWO_PI * 120, 1500);
    return margins(
      w,
      w.map((x) => loopGain(m, x)),
    );
  };

  it('the default PID: about 62° of phase margin at 1.1 Hz', () => {
    const r = marginsOf(calm());
    expect(r.wc / TWO_PI).toBeCloseTo(1.12, 1);
    expect(r.pmDeg).toBeGreaterThan(58);
    expect(r.pmDeg).toBeLessThan(66);
    expect(r.gm).toBeGreaterThan(8);
    // Three integrators' worth of phase at low frequency: the loop is conditionally stable.
    expect(r.gmLow).toBeGreaterThan(0);
    expect(r.gmLow).toBeLessThan(0.05);
    expect(1 / r.ms).toBeCloseTo(cabs(cadd(C_ONE, loopGain(l1Loop(calm())!, r.wMs))), 6);
  });

  it('the delay margin predicts where the real loop goes unstable', () => {
    const dm = marginsOf(calm()).delayMargin * 1000; // ms
    expect(dm).toBeGreaterThan(120);
    expect(dm).toBeLessThan(190);
    const stableAt = (ms: number) => isLoopStable(l1Loop(calm((p) => (p.sensors.delayMs = ms)))!);
    expect(stableAt(Math.floor(dm) - 8)).toBe(true);
    expect(stableAt(Math.ceil(dm) + 8)).toBe(false);

    // And the simulator agrees: a small disturbance dies out below the margin and grows above it.
    const swing = (ms: number) => {
      const p = calm((q) => (q.sensors.delayMs = ms));
      const sim = new Simulation(p);
      for (let k = 0; k < 8000; k++) sim.step();
      sim.applyImpulse({ x: 0, y: 0.05, z: 0 });
      let early = 0;
      let late = 0;
      for (let k = 0; k < 20000; k++) {
        sim.step();
        const e = Math.abs(sim.state.pos.y - 2);
        if (k < 3000) early = Math.max(early, e);
        if (k >= 17000) late = Math.max(late, e);
      }
      return late / early;
    };
    // Five milliseconds either side of the predicted 154 ms.
    expect(swing(Math.round(dm) - 5)).toBeLessThan(0.5);
    expect(swing(Math.round(dm) + 6)).toBeGreaterThan(1.3);
  });

  it('P alone around real motors is unstable, at the rate the book derives', () => {
    const m = l1Loop(
      calm((p) => {
        p.control.alt.iOn = false;
        p.control.alt.dOn = false;
      }),
    )!;
    const unstable = closedLoopPoles(m).filter((z) => z.re > 0);
    expect(unstable).toHaveLength(2);
    // The book's first-order estimate is Kp·τ/(2m) = 0.16 with τ = motor lag + half a sample;
    // fitting the amplitude equation to a flight gave 0.152 (lesson 2). The exact pole:
    expect(unstable[0]!.re).toBeCloseTo(0.152, 3);
    expect(Math.abs(unstable[0]!.im)).toBeCloseTo(Math.sqrt(10), 1);
    expect(isLoopStable(m)).toBe(false);
  });

  it('the slow pole of the PID loop is the integral: about −Ki/Kp', () => {
    const slow = closedLoopPoles(l1Loop(calm())!)
      .map((z) => z.re)
      .sort((a, b) => b - a)[0]!;
    expect(slow).toBeCloseTo(-0.085, 3);
    expect(isLoopStable(l1Loop(calm())!)).toBe(true);
  });
});
