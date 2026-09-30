import { describe, expect, it } from 'vitest';
import {
  brakingLimit,
  criticalDiveSpeed,
  lyapunovEstimate,
  startAt,
  touchesFloor,
} from '@/analysis/attraction';
import { altitudeFilterModel, controllabilityRank, observabilityRank } from '@/analysis/structure';
import { isLoopStable, l1Loop } from '@/analysis/loop';
import { fliesStable, runCampaign } from '@/analysis/montecarlo';
import { familyGrid, familyMember, robustTest } from '@/analysis/uncertainty';
import { Simulation } from '@/engine/simulation';
import { AltitudeKalman } from '@/estimation/kalman';
import { rank } from '@/math/mat';
import { Rng } from '@/math/prng';
import { defaultParams, type Params } from '@/sim/params';

const calm = (mod?: (p: Params) => void): Params => {
  const p = defaultParams();
  p.wind.enabled = false;
  mod?.(p);
  return p;
};

describe('structure: controllability and observability', () => {
  it('rank counts independent directions', () => {
    expect(
      rank([
        [1, 2],
        [2, 4],
      ]),
    ).toBe(1);
    expect(
      rank([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
      ]),
    ).toBe(3);
    expect(
      rank([
        [0, 0],
        [0, 0],
      ]),
    ).toBe(0);
    expect(
      rank([
        [1, 2, 3],
        [4, 5, 6],
        [7, 8, 9],
      ]),
    ).toBe(2);
  });

  it('the drone is controllable from its thrust', () => {
    const a = [
      [0, 1],
      [0, 0],
    ];
    expect(controllabilityRank(a, [[0], [1]])).toBe(2);
    // A force that acts on nothing reaches nothing.
    expect(controllabilityRank(a, [[0], [0]])).toBe(0);
  });

  it('altitude, velocity and accelerometer bias are observable; the altimeter bias is not', () => {
    const three = altitudeFilterModel(false);
    expect(observabilityRank(three.a, three.c)).toBe(3);
    const four = altitudeFilterModel(true);
    expect(observabilityRank(four.a, four.c)).toBe(3);
  });

  it('the filter cannot shrink the uncertainty of an unobservable state', () => {
    const run = (altBiasState: boolean) => {
      const kf = new AltitudeKalman();
      const rng = new Rng(5);
      const noise = {
        accSigma: 0.5,
        posSigma: 0.05,
        biasSigma: 0.05,
        altBiasState,
        altBiasSigma: 0.5,
      };
      // A drone hovering at 2 m with an altimeter that reads 0.3 m high.
      for (let k = 0; k < 5000; k++)
        kf.step(0.004, 9.81 + 0.3 * rng.normal(), 2.3 + 0.05 * rng.normal(), noise, 9.81);
      return kf;
    };
    const plain = run(false);
    expect(plain.sigma(0)).toBeLessThan(0.03); // confident…
    expect(Math.abs(plain.y - 2)).toBeGreaterThan(0.25); // …and wrong
    const honest = run(true);
    expect(honest.sigma(3)).toBeCloseTo(0.5, 2); // twenty seconds of data taught it nothing
    expect(honest.sigma(0)).toBeGreaterThan(0.45);
    expect(Math.abs(honest.y - 2)).toBeLessThan(2 * honest.sigma(0)); // but the truth is inside
  });
});

describe('the altimeter dropout', () => {
  it('holds the last fix and marks it stale; the filter coasts and its σ grows', () => {
    const p = calm((q) => {
      q.control.l1.estimator = 'kalman';
      q.sensors.posNoise = 0.05;
      q.sensors.accNoise = 0.3;
    });
    const sim = new Simulation(p);
    for (let k = 0; k < 10000; k++) sim.step();
    const before = sim.controller.extras()['est.sigma.y']!;
    sim.setParams({ ...p, sensors: { ...p.sensors, posDropout: true } });
    for (let k = 0; k < 5000; k++) sim.step();
    expect(sim.measurement!.posFresh).toBe(false);
    const during = sim.controller.extras()['est.sigma.y']!;
    expect(during).toBeGreaterThan(5 * before);
    expect(sim.state.crashed).toBe(false);
    sim.setParams({ ...p, sensors: { ...p.sensors, posDropout: false } });
    for (let k = 0; k < 3000; k++) sim.step();
    expect(sim.controller.extras()['est.sigma.y']!).toBeLessThan(2 * before);
  });
});

describe('regions of attraction', () => {
  const weak = () =>
    calm((p) => {
      p.setpoint.y = 0.6;
      p.drone.maxMotorThrust = 3.5;
      p.control.alt.iOn = false;
    });

  it('the braking formula predicts the fastest dive survived within 10 %', () => {
    const p = weak();
    const { h, aMax } = brakingLimit(p);
    expect(aMax).toBeCloseTo(4.19, 2);
    const flown = criticalDiveSpeed(p);
    expect(Math.abs(Math.sqrt(2 * aMax * h) / flown - 1)).toBeLessThan(0.1);
    expect(touchesFloor(p, 0, -flown * 1.05)).toBe(true);
    expect(touchesFloor(p, 0, -flown * 0.95)).toBe(false);
  });

  it('inside the Lyapunov estimate nothing touches the floor and V does not rise', () => {
    const p = weak();
    const { c, ax, av } = lyapunovEstimate(p);
    expect(c).toBeGreaterThan(0);
    // The guarantee is far smaller than the truth.
    expect(av).toBeLessThan(0.3 * criticalDiveSpeed(p));
    const kp = p.control.alt.kp;
    const m = p.drone.mass;
    for (const th of [0.3, 1.4, 2.5, 3.6, 4.7, 5.8]) {
      const sim = startAt(p, 0.9 * ax * Math.cos(th), 0.9 * av * Math.sin(th));
      let vPrev = Infinity;
      for (let k = 0; k < 3000; k++) {
        sim.step();
        expect(sim.state.landed).toBe(false);
        if (k % 50 === 0) {
          const x = sim.state.pos.y - 0.6;
          const v = 0.5 * kp * x * x + 0.5 * m * sim.state.vel.y ** 2;
          // Up to what the motor lag and the sampling add, which the proof leaves out.
          expect(v).toBeLessThan(vPrev + 0.004);
          vPrev = v;
        }
      }
      expect(Math.abs(sim.state.pos.y - 0.6)).toBeLessThan(0.01);
    }
  });
});

describe('a family of plants and the small-gain test', () => {
  const tuned = (kp: number, kd: number) =>
    calm((p) => {
      p.control.alt.kp = kp;
      p.control.alt.kd = kd;
    });

  it('the family spans the stated ranges', () => {
    const p = calm();
    const heavy = familyMember(p, 1, 1, 1);
    expect(heavy.drone.mass).toBeCloseTo(1.3, 12);
    expect(heavy.drone.motorTau).toBeCloseTo(0.06, 12);
    expect(heavy.sensors.delayMs).toBe(20);
    expect(familyMember(p, -1, -1, 0).drone.motorTau).toBeCloseTo(0.015, 12);
    expect(familyMember(p, 0, 0, 0)).toEqual(p);
    expect(familyGrid(p)).toHaveLength(75);
  });

  it('a passed test means every member is stable, on the grid and in flight', () => {
    for (const p of [tuned(10, 7), tuned(20, 20)]) {
      expect(robustTest(p)!.worst).toBeLessThan(1);
      for (const q of familyGrid(p)) expect(isLoopStable(l1Loop(q)!)).toBe(true);
      expect(runCampaign(p, 25).every((t) => t.stable)).toBe(true);
    }
  });

  it('the aggressive tune fails the test, and the failures sit in one corner', () => {
    const p = tuned(50, 30);
    expect(isLoopStable(l1Loop(p)!)).toBe(true); // the nominal drone is fine
    const test = robustTest(p)!;
    expect(test.worst).toBeGreaterThan(1.2);
    const unstable = familyGrid(p).filter((q) => !isLoopStable(l1Loop(q)!));
    expect(unstable.length).toBeGreaterThan(0);
    // Every unstable member is lighter than nominal (more loop gain) and has the full extra
    // delay; the worst corner does not fly.
    for (const q of unstable) {
      expect(q.drone.mass).toBeLessThan(p.drone.mass);
      expect(q.sensors.delayMs).toBe(20);
    }
    expect(fliesStable(familyMember(p, -1, 1, 1))).toBe(false);
    expect(fliesStable(p)).toBe(true);
  });

  it('flight and model agree on every trial of a campaign, and a seed repeats it', () => {
    const p = tuned(50, 30);
    const a = runCampaign(p, 40);
    for (const t of a) expect(t.stable).toBe(t.predicted);
    expect(a.some((t) => !t.stable)).toBe(true);
    expect(runCampaign(p, 40)).toEqual(a);
  });
});
