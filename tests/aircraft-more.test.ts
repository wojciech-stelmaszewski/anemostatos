import { describe, expect, it } from 'vitest';
import { meetsPitchSpec, pitchLoopMargins } from '@/analysis/aircraft';
import { pioFreeRate, pioOnset } from '@/analysis/pio';
import { ndiElevator } from '@/control/autopilot';
import { Simulation } from '@/engine/simulation';
import { LESSONS } from '@/lessons/lessons';
import { setAt } from '@/lessons/script';
import { defaultParams, type Params } from '@/sim/params';
import { aero, trimAircraft } from '@/sim/vehicles/aircraft';

const lessonParams = (id: string, mod: (p: Params) => void = () => {}): Params => {
  const p = defaultParams();
  p.sim.level = 1;
  LESSONS.find((l) => l.id === id)!.setup!(p);
  mod(p);
  return p;
};

describe('dynamic inversion of the pitch axis (IV.15)', () => {
  const p = defaultParams();
  const a = p.aircraft;
  const s = { ...trimAircraft(a, 120, a.altitude).state, q: 0.03, alpha: 0.06, theta: 0.04 };
  const qdotWith = (de: number) => aero(s, de, a).moment / a.inertia;

  it('with a perfect model the aircraft gets exactly the pitch acceleration asked for', () => {
    const ap = { ...p.autopilot, ndiBandwidth: 6, ndiAttitude: 1, modelError: 0 };
    const want = ap.ndiBandwidth * (ap.ndiAttitude * (0.1 - s.theta) - s.q);
    expect(qdotWith(ndiElevator(s, 0.1, a, ap).de)).toBeCloseTo(want, 10);
  });

  it('a model error e divides the acceleration by 1 + e: it is a gain error', () => {
    for (const e of [-0.3, 0.3]) {
      const ap = { ...p.autopilot, ndiBandwidth: 6, ndiAttitude: 1, modelError: e };
      const want = ap.ndiBandwidth * (ap.ndiAttitude * (0.1 - s.theta) - s.q);
      expect(qdotWith(ndiElevator(s, 0.1, a, ap).de)).toBeCloseTo(want / (1 + e), 10);
    }
  });

  it('the lesson’s start fails with the model 30 % too weak; k_q 6, k_a 1 pass both ways', () => {
    const start = lessonParams('ndi');
    expect(meetsPitchSpec(start).ok).toBe(false);
    expect(meetsPitchSpec(start).worstPm).toBeLessThan(36);
    const good = (e: number) =>
      lessonParams('ndi', (q) => {
        q.autopilot.ndiBandwidth = 6;
        q.autopilot.ndiAttitude = 1;
        q.autopilot.modelError = e;
      });
    for (const e of [-0.3, 0.3]) expect(meetsPitchSpec(good(e)).ok).toBe(true);
  });

  it('the crossover is a little below k_q/(1 + e), the prediction', () => {
    const wc = pitchLoopMargins(lessonParams('ndi'), 110).pitch.wc;
    const predicted = 8 / 0.7;
    expect(wc).toBeLessThan(predicted);
    expect(wc).toBeGreaterThan(0.85 * predicted);
  });

  it('the q̄ schedule with the elevator 30 % weaker loses its crossover at low speed', () => {
    const p2 = lessonParams('ndi', (q) => {
      q.autopilot.law = 'classic';
      q.autopilot.gain = 0.5 / 1.3;
    });
    const r = meetsPitchSpec(p2);
    expect(r.ok).toBe(false);
    expect(r.slowestWc).toBeLessThan(4);
    expect(r.worstPm).toBeGreaterThan(45);
  });

  it('in flight the inversion holds a 2° step at 160 m/s without a steady error', () => {
    const p2 = lessonParams('ndi', (q) => {
      q.autopilot.ndiBandwidth = 6;
      q.autopilot.ndiAttitude = 1;
    });
    const sim = new Simulation(p2);
    sim.script = LESSONS.find((l) => l.id === 'ndi')!.events!;
    sim.reset();
    let before = 0;
    let after = 0;
    for (let k = 0; k < 51000; k++) {
      sim.step();
      if (k === 45950) before = sim.aircraft!.theta;
      if (k === 50999) after = sim.aircraft!.theta;
    }
    expect(sim.aircraft!.V).toBeGreaterThan(150);
    expect(((after - before) * 180) / Math.PI).toBeCloseTo(2, 1);
  }, 120_000);
});

describe('pilot-induced oscillation (IV.16)', () => {
  const p = lessonParams('pio');

  it('the describing function puts the onset at 7.7° of command, above the start of rate limiting', () => {
    const o = pioOnset(p);
    expect(1 / o.linearGain).toBeCloseTo(1.26, 1);
    expect(o.rateLimitDeg).toBeCloseTo(4.4, 0);
    expect(o.onsetDeg).toBeGreaterThan(7.2);
    expect(o.onsetDeg).toBeLessThan(8.1);
  }, 120_000);

  it('the PIO-free rate is the onset scaled out to the travel: R·travel/onset', () => {
    const o = pioOnset(p);
    const scaled = (20 * p.aircraft.elevatorMax) / o.onsetDeg;
    const exact = pioFreeRate(p);
    expect(exact).toBeGreaterThan(50);
    expect(exact).toBeLessThan(55);
    expect(Math.abs(exact / scaled - 1)).toBeLessThan(0.03);
  }, 120_000);

  /** Fly a two-cycle elevator burst at 3.8 rad/s; the command's swing early and late, °. */
  const burst = (amp: number, rate = 20) => {
    const q = structuredClone(p);
    q.aircraft.servoRate = rate;
    const sim = new Simulation(q);
    const T = (4 * Math.PI) / 3.8;
    sim.script = (s) => {
      for (let t = 0; t <= T; t += 0.02)
        setAt(s, 2 + t, 'autopilot.elevatorOffset', amp * Math.sin(3.8 * t));
      setAt(s, 2 + T + 0.02, 'autopilot.elevatorOffset', 0);
    };
    sim.reset();
    for (let k = 0; k < 30000; k++) sim.step();
    const swing = (from: number, to: number) => {
      const w = sim.telemetry.window(['air.decmd'], from);
      const v = w.series[0]!.filter((x, i) => Number.isFinite(x) && w.t[i]! <= to);
      return (Math.max(...v) - Math.min(...v)) / 2;
    };
    return { early: swing(6, 10), late: swing(25, 30) };
  };

  it('a burst that leaves less than the onset dies out; one that leaves more locks in', () => {
    const small = burst(2.5);
    expect(small.early).toBeLessThan(7);
    expect(small.late).toBeLessThan(small.early / 2);
    const big = burst(3);
    expect(big.early).toBeGreaterThan(8);
    expect(big.late).toBeGreaterThan(big.early);
    // The PIO settles into a limit cycle of about ±14° of command.
    expect(big.late).toBeGreaterThan(12);
    expect(big.late).toBeLessThan(16);
  }, 120_000);

  it('above the PIO-free rate even a burst to full travel dies out', () => {
    const r = burst(20, 55);
    expect(r.late).toBeLessThan(r.early / 2);
    const slow = burst(8, 40);
    expect(slow.late).toBeGreaterThan(slow.early / 2);
  }, 120_000);
});
