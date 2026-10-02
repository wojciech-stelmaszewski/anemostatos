import { describe, expect, it } from 'vitest';
import { Simulation } from '@/engine/simulation';
import { Mekf } from '@/estimation/mekf';
import { NavEkf } from '@/estimation/navekf';
import { LESSONS } from '@/lessons/lessons';
import { qConj, qFromEuler, qIdentity, qRotate } from '@/math/quat';
import { v3 } from '@/math/vec3';
import { defaultParams, GRAVITY, type Params } from '@/sim/params';
import { headingError } from '@/estimation/attitude';

const DEG = Math.PI / 180;
const settings = {
  gyroSigma: 0.5 * DEG,
  biasWalk: 0.02 * DEG,
  accSigma: 3 * DEG,
  magSigma: 2 * DEG,
  gate: 0,
};

describe('the multiplicative EKF', () => {
  it('converges on attitude and gyro bias from gravity and north, and its covariance shrinks', () => {
    const truth = qFromEuler(8 * DEG, 30 * DEG, -5 * DEG);
    const bias = v3(0.4 * DEG, -0.6 * DEG, 0.3 * DEG);
    const f = new Mekf();
    f.reset(qIdentity(), 0.5, 0.02);
    const s0 = f.sigma(1);
    const up = qRotate(qConj(truth), v3(0, 1, 0));
    const north = qRotate(qConj(truth), v3(1, 0, 0));
    for (let k = 0; k < 60000; k++) {
      f.propagate(bias, settings, 0.001); // at rest: the gyro reads only its bias
      if (k % 10 === 0) {
        f.updateVector(up, v3(0, 1, 0), settings.accSigma, 0);
        f.updateHeading(north, qRotate(qConj(f.q), v3(0, 1, 0)), settings.magSigma, 0);
      }
    }
    expect(headingError(truth, f.q) / DEG).toBeLessThan(0.5);
    expect(f.bias.y / DEG).toBeCloseTo(-0.6, 1);
    expect(f.sigma(1)).toBeLessThan(s0 / 10);
  });

  it('a north reading 40° off scores far beyond the χ² gate and is rejected', () => {
    const f = new Mekf();
    f.reset(qIdentity(), 0.01, 0.001);
    const off = v3(Math.cos(40 * DEG), 0, -Math.sin(40 * DEG));
    const u = f.updateHeading(off, v3(0, 1, 0), settings.magSigma, 10.8);
    expect(u.nis).toBeGreaterThan(100);
    expect(u.accepted).toBe(false);
    expect(headingError(qIdentity(), f.q)).toBe(0);
  });
});

describe('the navigation EKF', () => {
  it('a still receiver and a still accelerometer: the estimate settles on the truth', () => {
    const f = new NavEkf();
    f.reset(v3(1, 2, 3));
    const s = { accSigma: 0.2, biasWalk: 0.01, gpsSigma: 0.1, baroSigma: 0.1, gate: 0 };
    for (let k = 0; k < 10000; k++) {
      f.propagate(v3(0, GRAVITY, 0), qIdentity(), GRAVITY, s, 0.001);
      if (k % 100 === 0) f.gps(v3(1.5, 2, 3), s);
    }
    expect(f.pos.x).toBeCloseTo(1.5, 2);
    expect(Math.hypot(f.vel.x, f.vel.y, f.vel.z)).toBeLessThan(0.01);
  });
});

/** Fly a lesson's set-up and events for `seconds`, with an optional change. */
const fly = (id: string, mod: (p: Params) => void, seconds: number) => {
  const lesson = LESSONS.find((l) => l.id === id)!;
  const p = defaultParams();
  p.sim.level = 3;
  lesson.setup!(p);
  mod(p);
  const sim = new Simulation(p);
  sim.script = lesson.events!;
  sim.reset();
  for (let k = 0; k < seconds * 1000; k++) sim.step();
  return sim;
};
const worst = (sim: Simulation, key: string, from: number) =>
  Math.max(...sim.telemetry.window([key], from).series[0]!.filter((v) => !Number.isNaN(v)));

describe('lesson III.23: steel next to the magnetometer', () => {
  it('Mahony turns with the disturbance; the MEKF without a gate too; with the gate it holds', () => {
    const mahony = fly('mekf', () => {}, 20);
    expect(worst(mahony, 'ahrs.headErr', 3)).toBeGreaterThan(30);
    const open = fly('mekf', (p) => (p.sensors.attitude = 'mekf'), 20);
    expect(worst(open, 'ahrs.headErr', 3)).toBeGreaterThan(30);
    const gated = fly(
      'mekf',
      (p) => ((p.sensors.attitude = 'mekf'), (p.control.mekf.gate = 16.3)),
      24,
    );
    expect(worst(gated, 'ahrs.headErr', 3)).toBeLessThan(3);
    // Rejected while the steel is there, used again after.
    const { t, series } = gated.telemetry.window(['mekf.magRejected'], 3);
    const at = (a: number, b: number) => series[0]!.filter((_, i) => t[i]! > a && t[i]! < b);
    expect(Math.min(...at(7, 15))).toBe(1);
    // After it, the gate is open again: an honest reading is rejected only now and then.
    const after = at(19, 24);
    expect(after.reduce((x, y) => x + y, 0) / after.length).toBeLessThan(0.05);
    // And the filter says it is less sure while it flies on the gyro alone.
    const sig = gated.telemetry.window(['mekf.sigma.yaw'], 3);
    const s = (time: number) => sig.series[0]![sig.t.findIndex((x) => x >= time)]!;
    expect(s(15.5)).toBeGreaterThan(1.5 * s(5.5));
  }, 120_000);
});

describe('lesson III.24: a GPS glitch', () => {
  const rms = (sim: Simulation) => {
    const { t, series } = sim.telemetry.window(
      ['pos.x', 'pos.y', 'pos.z', 'sp.x', 'sp.y', 'sp.z'],
      4,
    );
    let sq = 0;
    let n = 0;
    for (let i = 0; i < t.length; i++) {
      if (t[i]! > 20) break;
      sq +=
        (series[0]![i]! - series[3]![i]!) ** 2 +
        (series[1]![i]! - series[4]![i]!) ** 2 +
        (series[2]![i]! - series[5]![i]!) ** 2;
      n++;
    }
    return Math.sqrt(sq / n);
  };

  it('the ungated filter swallows the glitch and flies the drone off; the gated one ignores it', () => {
    const truth = rms(fly('ekf', (p) => (p.sensors.nav = 'truth'), 20));
    const open = fly('ekf', () => {}, 20);
    const gated = fly('ekf', (p) => (p.control.navEkf.gate = 16.3), 20);
    expect(worst(open, 'nav.err', 7)).toBeGreaterThan(1.5);
    expect(worst(gated, 'nav.err', 7)).toBeLessThan(0.4);
    expect(worst(gated, 'nav.gpsNis', 7)).toBeGreaterThan(1000);
    expect(rms(open)).toBeGreaterThan(2 * truth);
    expect(rms(gated)).toBeLessThan(2 * truth);
  }, 120_000);
});
