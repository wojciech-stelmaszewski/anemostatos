import { describe, expect, it } from 'vitest';
import { Simulation } from '@/engine/simulation';
import { defaultParams, type Params } from '@/sim/params';

const make = (setup: (p: Params) => void) => {
  const p = defaultParams();
  p.sim.level = 3;
  setup(p);
  return new Simulation(p);
};
const posErr = (s: Simulation) =>
  Math.hypot(
    s.state.pos.x - s.setpoint.x,
    s.state.pos.y - s.setpoint.y,
    s.state.pos.z - s.setpoint.z,
  );
/** Max and RMS position error after `from` seconds. */
const fly = (sim: Simulation, seconds: number, from: number) => {
  let max = 0;
  let sq = 0;
  let n = 0;
  for (let k = 0; k < seconds * 1000; k++) {
    sim.step();
    if (sim.t > from) {
      const e = posErr(sim);
      max = Math.max(max, e);
      sq += e * e;
      n++;
    }
  }
  return { max, rms: Math.sqrt(sq / n) };
};
const calm = (p: Params) => {
  p.wind.enabled = false;
  p.setpoint.x = 2;
};
const rateIndi = (p: Params) => (p.control.l3.inner = 'indi');
const accelIndi = (p: Params) => (p.control.l3.compensation = 'indi');
const brokenProp = (sim: Simulation) =>
  sim.schedule(12, (s) =>
    s.setParams({ ...s.params, drone: { ...s.params.drone, motorEfficiency: [1, 0.6, 1, 1] } }),
  );

describe('INDI (L3)', () => {
  it('flies to a point with rate INDI, with and without motor telemetry', () => {
    for (const telemetry of [true, false]) {
      const sim = make((p) => {
        calm(p);
        rateIndi(p);
        accelIndi(p);
        p.sensors.motorFeedback = telemetry;
      });
      fly(sim, 12, 0);
      expect(posErr(sim)).toBeLessThan(0.01);
      expect(sim.state.crashed).toBe(false);
    }
  });

  it('rate INDI rejects a broken prop that the PID rate loop struggles with', () => {
    const run = (setup: (p: Params) => void) => {
      const sim = make((p) => {
        calm(p);
        setup(p);
      });
      brokenProp(sim);
      return fly(sim, 25, 12);
    };
    const pid = run(() => {});
    const indi = run(rateIndi);
    const both = run((p) => {
      rateIndi(p);
      accelIndi(p);
    });
    expect(pid.max).toBeGreaterThan(0.5);
    expect(indi.max).toBeLessThan(0.3);
    expect(both.max).toBeLessThan(0.06);
  });

  it('acceleration INDI rejects gusts; rate INDI alone does not', () => {
    const run = (setup: (p: Params) => void) =>
      fly(
        make((p) => {
          p.sim.seed = 4242;
          p.wind.gustsPerMinute = 14;
          p.wind.gustAmpMin = 4;
          p.wind.gustAmpMax = 10;
          setup(p);
        }),
        40,
        8,
      ).rms;
    const pid = run(() => {});
    expect(run(rateIndi)).toBeGreaterThan(0.9 * pid);
    expect(run(accelIndi)).toBeLessThan(0.5 * pid);
  });

  it('its implicit disturbance estimate follows the true one', () => {
    const sim = make((p) => {
      p.setpoint.x = 0;
      p.wind.meanSpeed = 5;
      p.wind.turbSigma = 0;
      p.wind.gustsOn = false;
      rateIndi(p);
      accelIndi(p);
    });
    fly(sim, 15, 0);
    const truth = sim.telemetry.latest('dist.x');
    expect(Math.abs(truth)).toBeGreaterThan(1); // a real push…
    expect(sim.telemetry.latest('est.dist.x')).toBeCloseTo(truth, 1); // …and INDI knows it
  });

  it('unsynchronised slow filters make the loop wobble', () => {
    const wobble = (sync: boolean) => {
      const sim = make((p) => {
        p.wind.enabled = false;
        p.setpoint.profile = 'square';
        p.setpoint.profileAxis = 'x';
        p.setpoint.profileAmplitude = 1.5;
        p.setpoint.profilePeriod = 6;
        p.sensors.gyroNoise = 10;
        rateIndi(p);
        p.control.indi.filterHz = 5;
        p.control.indi.syncFilters = sync;
      });
      let sq = 0;
      let n = 0;
      for (let k = 0; k < 25000; k++) {
        sim.step();
        if (sim.t > 8 && k % 5 === 0) {
          const l = sim.controller.loops();
          sq += l['att.roll']!.error ** 2 + l['att.pitch']!.error ** 2;
          n++;
        }
      }
      return Math.sqrt(sq / n);
    };
    expect(wobble(false)).toBeGreaterThan(2 * wobble(true));
  });
});
