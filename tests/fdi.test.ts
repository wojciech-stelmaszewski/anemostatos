import { describe, expect, it } from 'vitest';
import { Simulation } from '@/engine/simulation';
import { LESSONS } from '@/lessons/lessons';
import { defaultParams, type Params } from '@/sim/params';

const lesson = LESSONS.find((l) => l.id === 'fdi')!;

/** Lesson III.28's drone and monitor, with its solution's thresholds unless told otherwise. */
const setup = (mod: (p: Params) => void = () => {}, seed = 1337): Params => {
  const p = defaultParams();
  p.sim.level = 3;
  lesson.setup!(p);
  lesson.solution!(p);
  p.sim.seed = seed;
  mod(p);
  return p;
};
const set = (s: Simulation, f: (p: Params) => void) => {
  const q = structuredClone(s.params);
  f(q);
  s.setParams(q);
};

describe('the altimeter test', () => {
  it('the filter is consistent: the NIS of a healthy fix averages one', () => {
    const sim = new Simulation(setup((p) => (p.control.fdi.nisWindow = 1)));
    let sum = 0;
    let n = 0;
    let last = NaN;
    for (let k = 0; k < 40000; k++) {
      sim.step();
      const v = sim.fdi.state.nis;
      if (sim.t > 5 && v !== last) {
        sum += v;
        n++;
      }
      last = v;
    }
    expect(n).toBeGreaterThan(500); // 20 fixes a second
    expect(sum / n).toBeGreaterThan(0.85);
    expect(sum / n).toBeLessThan(1.15);
  });

  it('a stuck sensor is flagged within a second by the two-sided test, and missed by the upper one', () => {
    let missed = 0;
    for (const seed of [1, 2, 3, 4]) {
      const run = (low: number) => {
        const sim = new Simulation(setup((p) => (p.control.fdi.nisLow = low), seed));
        sim.schedule(20, (s) => set(s, (q) => (q.sensors.posStuck = true)));
        for (let k = 0; k < 24000; k++) {
          sim.step();
          if (sim.t >= 20 && sim.fdi.state.altAlarm) return sim.t - 20;
        }
        return Infinity;
      };
      expect(run(0.8)).toBeLessThan(1);
      if (run(0) > 1) missed++;
    }
    expect(missed).toBeGreaterThanOrEqual(2);
  });
});

describe('the motor test', () => {
  it('names the lost motor, whichever it is, within 50 ms, and the drone flies on', () => {
    for (const lost of [0, 1, 2, 3]) {
      const sim = new Simulation(setup());
      const eff: [number, number, number, number] = [1, 1, 1, 1];
      eff[lost] = 0;
      sim.schedule(20, (s) => set(s, (q) => (q.drone.motorEfficiency = eff)));
      let named = NaN;
      let healthyMax = 0;
      for (let k = 0; k < 32000; k++) {
        sim.step();
        if (sim.t < 20) healthyMax = Math.max(healthyMax, ...sim.fdi.state.cusum);
        if (Number.isNaN(named) && sim.fdi.state.motor >= 0) named = sim.t;
      }
      expect(sim.fdi.state.motor).toBe(lost);
      expect(named - 20).toBeGreaterThan(0);
      expect(named - 20).toBeLessThan(0.05);
      // Found that fast, the loss costs well under the 3 m the lesson allows.
      expect(sim.state.pos.y).toBeGreaterThan(3);
      // The noise stays well under the alarm: no false alarm before the loss.
      expect(healthyMax).toBeLessThan(0.5 * sim.params.control.fdi.cusumThreshold);
      expect(sim.state.crashed).toBe(false);
    }
  });

  it("found a quarter of a second late, the lesson's rotor loss brings the drone down", () => {
    const sim = new Simulation(setup((p) => (p.control.fdi.cusumThreshold = 0.2)));
    sim.script = lesson.events!;
    sim.reset();
    let named = NaN;
    for (let k = 0; k < 46000 && !sim.state.crashed; k++) {
      sim.step();
      if (Number.isNaN(named) && sim.fdi.state.motor >= 0) named = sim.t;
    }
    expect(named - 40).toBeGreaterThan(0.2);
    expect(sim.state.crashed).toBe(true);
  });

  it('a threshold below the noise raises a false alarm on a healthy drone', () => {
    const sim = new Simulation(
      setup((p) => {
        p.control.fdi.cusumDrift = 0.002;
        p.control.fdi.cusumThreshold = 0.001;
      }),
    );
    for (let k = 0; k < 20000; k++) sim.step();
    expect(sim.fdi.state.motor).toBeGreaterThanOrEqual(0);
  });
});
