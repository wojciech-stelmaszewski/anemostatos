import { describe, expect, it } from 'vitest';
import { analyseRateLoop, rateLoopModel, sweepMimo } from '@/analysis/mimo';
import {
  loopRates,
  lowestRate,
  quantisationCycle,
  quantiserDf,
  ratePm,
} from '@/analysis/quantisation';
import { Simulation } from '@/engine/simulation';
import { cabs, csub } from '@/math/complex';
import { defaultParams, type Params } from '@/sim/params';
import { posQuantStep } from '@/sim/sensors';

/** The racing tune of lesson III.18. */
const racing = (mod: (p: Params) => void = () => {}): Params => {
  const p = defaultParams();
  p.sim.level = 3;
  p.wind.enabled = false;
  p.control.l3.rateRP.kp = 100;
  p.control.l3.rateRP.kd = 1.5;
  p.control.l3.attKpRP = 20;
  mod(p);
  return p;
};

/** Largest roll/pitch rate in the last 3 s of a 15 s flight with a kick at 4 s, rad/s. */
const residualRate = (p: Params): number => {
  const sim = new Simulation(p);
  let late = 0;
  for (let k = 0; k < 15000; k++) {
    sim.step();
    if (k === 4000) sim.state.omega = { x: 0.3, y: 0, z: 0.1 };
    if (sim.state.crashed) return Infinity;
    if (k > 12000) late = Math.max(late, Math.hypot(sim.state.omega.x, sim.state.omega.z));
  }
  return late;
};

describe('III.18: the loop rate', () => {
  it('the rates the loop can run at are whole fractions of the physics clock', () => {
    const r = loopRates(100);
    expect(r[0]).toBe(1000);
    expect(r[1]).toBe(500);
    expect(r[3]).toBe(250);
    expect(r[r.length - 1]).toBeGreaterThanOrEqual(100);
  });

  it('each halving of the rate costs more phase than the one before', () => {
    const pm = [1000, 500, 250, 125, 62.5].map((hz) => ratePm(racing(), hz));
    expect(pm[0]).toBeCloseTo(55, 0);
    const costs = pm.slice(1).map((v, i) => pm[i]! - v);
    for (let i = 1; i < costs.length; i++) expect(costs[i]!).toBeGreaterThan(1.5 * costs[i - 1]!);
    // The crossover the rule of thumb refers to: near 10 Hz.
    const fc = analyseRateLoop(racing())!.one.wc / (2 * Math.PI);
    expect(fc).toBeGreaterThan(9);
    expect(fc).toBeLessThan(11);
  });

  it('250 Hz is the lowest rate with 45°', () => {
    expect(lowestRate(racing(), 45)).toBe(250);
    expect(ratePm(racing(), 250)).toBeGreaterThan(45);
    expect(ratePm(racing(), 200)).toBeLessThan(45);
  });

  it('the model of the sampled loop is what the simulator flies', () => {
    const p = racing((q) => (q.control.l3.hzRate = 125));
    for (const m of sweepMimo(p, [1, 4, 10])) {
      const q = rateLoopModel(p, m.fHz);
      expect(cabs(csub(m.s[0][0], q.s[0][0])) / cabs(q.s[0][0])).toBeLessThan(0.05);
    }
  }, 300_000);

  it('jitter costs like a delay of half its size', () => {
    const at = (mod: (p: Params) => void) =>
      residualRate(racing((p) => ((p.control.l3.hzRate = 62.5), mod(p))));
    expect(at(() => {})).toBeLessThan(1e-3);
    expect(at((p) => (p.sensors.jitterMs = 4))).toBeLessThan(1e-3);
    expect(at((p) => (p.sensors.jitterMs = 8))).toBeGreaterThan(0.5);
    expect(at((p) => (p.sensors.delayMs = 2))).toBeLessThan(1e-3);
    expect(at((p) => (p.sensors.delayMs = 4))).toBeGreaterThan(0.5);
  }, 300_000);

  it('a motor rate limit spares small steps and wrecks large ones', () => {
    const step = (d: number, mod: (p: Params) => void) => {
      const sim = new Simulation(racing(mod));
      for (let k = 0; k < 8000; k++) sim.step();
      sim.setTarget({ x: d, y: 2, z: 0 });
      let over = 0;
      for (let k = 0; k < 12000 && !sim.state.crashed; k++) {
        sim.step();
        over = Math.max(over, sim.state.pos.x - d);
      }
      return sim.state.crashed ? Infinity : over;
    };
    const slow = (p: Params) => (p.drone.motorSlew = 30);
    expect(step(0.5, slow)).toBeLessThan(0.1);
    expect(step(0.5, slow)).toBeCloseTo(
      step(0.5, () => {}),
      2,
    );
    expect(step(3, () => {})).toBeLessThan(0.3);
    expect(step(3, slow)).toBeGreaterThan(1);
  }, 300_000);

  it('with the defaults nothing changes: no limit, no jitter, no quantisation', () => {
    const p = defaultParams();
    expect(p.drone.motorSlew).toBe(0);
    expect(p.sensors.jitterMs).toBe(0);
    expect(p.sensors.posQuantBits).toBe(0);
  });
});

describe('III.19: a quantised altimeter', () => {
  const pid = (bits: number) => {
    const p = defaultParams();
    p.wind.enabled = false;
    p.sensors.posQuantBits = bits;
    return p;
  };
  /** Half the peak-to-peak swing and the mean error over the last 20 s of a 50 s hover, m. */
  const hover = (bits: number) => {
    const sim = new Simulation(pid(bits));
    const ys: number[] = [];
    for (let k = 0; k < 50000; k++) {
      sim.step();
      if (sim.t > 30 && k % 5 === 0) ys.push(sim.state.pos.y);
    }
    return {
      amp: (Math.max(...ys) - Math.min(...ys)) / 2,
      mean: ys.reduce((a, b) => a + b, 0) / ys.length - 2,
    };
  };

  it('the describing function of a relay of ±q/2', () => {
    expect(quantiserDf(0.04, 0.01)).toBeCloseTo((4 * 0.02) / (Math.PI * 0.01), 12);
    expect(posQuantStep(8)).toBeCloseTo(0.0390625, 12);
  });

  it('predicts the amplitude of the hunting within 25 % at 6, 8 and 10 bits', () => {
    for (const bits of [6, 8, 10]) {
      const pred = quantisationCycle(pid(bits))!.amplitude;
      const { amp } = hover(bits);
      expect(Math.abs(amp / pred - 1)).toBeLessThan(0.25);
    }
  }, 300_000);

  it('the drone rests on the boundary between the two steps nearest the setpoint', () => {
    const q = posQuantStep(8);
    const boundary = (Math.floor(2 / q) + 0.5) * q; // between the levels around 2 m
    const { mean } = hover(8);
    expect(Math.abs(mean - (boundary - 2))).toBeLessThan(0.002);
    expect(Math.abs(mean)).toBeLessThan(q / 2);
  }, 300_000);

  it('without quantisation there is nothing to hunt', () => {
    // What is left is the integral still settling: a fifth of a millimetre, not a cycle.
    expect(hover(0).amp).toBeLessThan(3e-4);
  }, 300_000);
});
