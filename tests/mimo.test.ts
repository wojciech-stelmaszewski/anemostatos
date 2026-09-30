import { describe, expect, it } from 'vitest';
import {
  analyseRateLoop,
  diskMargin,
  rateLoopModel,
  singularValues,
  sweepMimo,
  type C2,
} from '@/analysis/mimo';
import { Simulation } from '@/engine/simulation';
import { cabs, csub, cx } from '@/math/complex';
import { logspace } from '@/math/margins';
import { defaultParams, type Params } from '@/sim/params';

/** The quadrotor in calm air with the gyro mounted `deg` degrees off. */
const quad = (deg: number, mod?: (p: Params) => void): Params => {
  const p = defaultParams();
  p.sim.level = 3;
  p.wind.enabled = false;
  p.sensors.imuYawDeg = deg;
  mod?.(p);
  return p;
};

/** Body rate left three seconds before the end of a flight that starts with a kick, rad/s. */
const residualRate = (p: Params, seconds = 20): number => {
  const sim = new Simulation(p);
  let late = 0;
  const n = seconds * 1000;
  for (let k = 0; k < n; k++) {
    sim.step();
    if (k === 4000) sim.state.omega = { x: 0.3, y: 0, z: 0.1 };
    if (sim.state.crashed) return Infinity;
    if (k > n - 3000) late = Math.max(late, Math.hypot(sim.state.omega.x, sim.state.omega.z));
  }
  return late;
};

describe('two-by-two tools', () => {
  it('singular values of a complex matrix', () => {
    const diag: C2 = [
      [cx(0, 3), cx(0)],
      [cx(0), cx(-0.5)],
    ];
    expect(singularValues(diag)).toEqual([3, 0.5]);
    // A rotation scaled by 2 stretches every direction alike.
    const c = Math.cos(0.7);
    const s = Math.sin(0.7);
    const rot: C2 = [
      [cx(2 * c), cx(2 * s)],
      [cx(-2 * s), cx(2 * c)],
    ];
    const sv = singularValues(rot);
    expect(sv[0]).toBeCloseTo(2, 12);
    expect(sv[1]).toBeCloseTo(2, 12);
    // Rank one: [1 j; 1 j] has singular values 2 and 0.
    const one: C2 = [
      [cx(1), cx(0, 1)],
      [cx(1), cx(0, 1)],
    ];
    expect(singularValues(one)[0]).toBeCloseTo(2, 12);
    expect(singularValues(one)[1]).toBeCloseTo(0, 6);
  });

  it('the disk margin of a single loop with L = k/s is the textbook one', () => {
    // S − T = (s − k)/(s + k) has magnitude one everywhere: α = 2, the whole half plane.
    const pts = logspace(0.01, 100, 200).map((fHz) => {
      const l = cx(0, -3 / (2 * Math.PI * fHz));
      const den = cabs(cx(1 + l.re, l.im)) ** 2;
      const s = cx((1 + l.re) / den, -l.im / den);
      const t = csub(cx(1), s);
      const z = cx(0);
      return {
        fHz,
        s: [
          [s, z],
          [z, s],
        ] as C2,
        t: [
          [t, z],
          [z, t],
        ] as C2,
      };
    });
    const d = diskMargin(pts);
    expect(d.alpha).toBeGreaterThan(1.99);
    expect(d.pmDeg).toBeGreaterThan(89.9);
  });
});

describe('the torque probe and the turned gyro', () => {
  it('a roll probe adds to the roll torque only: τ = τc + in', () => {
    const p = quad(0);
    const sim = new Simulation(p);
    for (let k = 0; k < 4000; k++) sim.step();
    sim.setParams({ ...p, probe: { ...p.probe, point: 'l3.torque.x', amp: 0.01, freqHz: 3 } });
    let seen = 0;
    for (let k = 0; k < 1000; k++) {
      sim.step();
      const pr = sim.probe;
      expect(pr.tau.x).toBeCloseTo(pr.tauC.x + pr.in, 12);
      expect(pr.tau.z).toBeCloseTo(pr.tauC.z, 12);
      seen = Math.max(seen, Math.abs(pr.in));
    }
    expect(seen).toBeCloseTo(0.01, 3);
    expect(sim.state.crashed).toBe(false);
  });

  it('a controller that knows the mounting angle flies as if the gyro were straight', () => {
    const fly = (p: Params) => {
      p.setpoint.profile = 'square';
      p.setpoint.profileAxis = 'x';
      const sim = new Simulation(p);
      for (let k = 0; k < 9000; k++) sim.step();
      return sim.state;
    };
    const straight = fly(quad(0));
    const known = fly(quad(30, (p) => (p.control.model.imuYawDeg = 30)));
    const unknown = fly(quad(30));
    expect(Math.abs(known.pos.x - straight.pos.x)).toBeLessThan(1e-9);
    expect(Math.abs(known.omega.x - straight.omega.x)).toBeLessThan(1e-9);
    // Uncorrected, a pitch manoeuvre leaks into roll and pushes the drone sideways.
    expect(Math.abs(unknown.pos.z - straight.pos.z)).toBeGreaterThan(1e-5);
  });
});

describe('the roll–pitch loop: model against flight', () => {
  for (const deg of [0, 30])
    it(`every entry of S within 2 % with the gyro at ${deg}°`, () => {
      const p = quad(deg);
      for (const m of sweepMimo(p, [0.5, 1, 3, 8, 20])) {
        const q = rateLoopModel(p, m.fHz);
        const scale = singularValues(q.s)[0];
        for (const i of [0, 1] as const)
          for (const j of [0, 1] as const) {
            expect(cabs(csub(m.s[i][j], q.s[i][j])) / scale).toBeLessThan(0.02);
            // S and T come from the same run and add up to the identity.
            const sum = cabs(csub(csub(cx(i === j ? 1 : 0), m.s[i][j]), m.t[i][j]));
            expect(sum).toBeLessThan(1e-9);
          }
        if (deg === 0) expect(cabs(m.s[0][1]) / scale).toBeLessThan(1e-6);
      }
    });
});

describe('one loop at a time against both channels at once', () => {
  it('with a straight gyro the three margins agree', () => {
    const a = analyseRateLoop(quad(0))!;
    expect(a.stable).toBe(true);
    expect(a.one.pmDeg).toBeCloseTo(44.9, 0);
    expect(a.both.pmDeg).toBeCloseTo(a.one.pmDeg, 6);
    // The disk margin also covers gain changes, so it is a little smaller.
    expect(a.disk.pmDeg).toBeLessThan(a.one.pmDeg);
    expect(a.disk.pmDeg).toBeGreaterThan(40);
    expect(Math.max(...a.sMax)).toBeCloseTo(Math.max(...a.s11), 9);
  });

  it('a gyro turned by 30° leaves the one-loop margin healthy and takes most of the real one', () => {
    const a = analyseRateLoop(quad(30))!;
    expect(a.stable).toBe(true);
    expect(a.one.pmDeg).toBeGreaterThan(35);
    expect(a.one.delayMargin * 1000).toBeCloseTo(34.6, 0);
    expect(a.both.pmDeg).toBeCloseTo(22.1, 0);
    expect(a.both.delayMargin * 1000).toBeCloseTo(20.7, 0);
    // The guarantee for any mix is never larger than the margin for equal changes.
    expect(a.disk.pmDeg).toBeLessThanOrEqual(a.both.pmDeg);
    expect(a.disk.pmDeg).toBeGreaterThan(21);
    expect(Math.max(...a.sMax)).toBeGreaterThan(1.5 * Math.max(...a.s11));
  });

  it('the flight sides with the margin for both channels', () => {
    // 18 ms is inside both margins, 24 ms only inside the one-loop margin of 35 ms.
    expect(residualRate(quad(30, (p) => (p.sensors.delayMs = 18)))).toBeLessThan(1e-3);
    expect(residualRate(quad(30, (p) => (p.sensors.delayMs = 24)))).toBeGreaterThan(1);
    expect(analyseRateLoop(quad(30, (p) => (p.sensors.delayMs = 24)))!.stable).toBe(false);
    // Telling the controller the angle gives the margin back.
    const fixed = quad(30, (p) => {
      p.sensors.delayMs = 24;
      p.control.model.imuYawDeg = 30;
    });
    expect(residualRate(fixed)).toBeLessThan(1e-3);
    expect(analyseRateLoop(fixed)!.both.pmDeg).toBeGreaterThan(20);
  });

  it('a gain error common to both channels fails where the eigen-loops say', () => {
    // Reducing the gain: the one-loop test allows a factor 0.24, the pair only 0.31.
    const a = analyseRateLoop(quad(30))!;
    expect(a.one.gmLow).toBeCloseTo(0.24, 2);
    const weak = (k: number) => quad(30, (p) => (p.control.model.inertiaScale = k));
    expect(analyseRateLoop(weak(0.28))!.stable).toBe(false);
    expect(residualRate(weak(0.28))).toBeGreaterThan(0.1);
    expect(analyseRateLoop(weak(0.4))!.stable).toBe(true);
    expect(residualRate(weak(0.4))).toBeLessThan(1e-2);
  });

  it('there is no model for controllers other than the PID cascade', () => {
    expect(analyseRateLoop(quad(0, (p) => (p.control.l3.inner = 'indi')))).toBeNull();
    expect(analyseRateLoop(defaultParams())).toBeNull();
  });
});
