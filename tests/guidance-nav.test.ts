import { describe, expect, it } from 'vitest';
import { Simulation } from '@/engine/simulation';
import { insGyroError } from '@/estimation/ins';
import { RangeFilter } from '@/estimation/ukf';
import { geometry, guidanceAccel, targetState } from '@/guidance/pronav';
import { LESSONS } from '@/lessons/lessons';
import { v3 } from '@/math/vec3';
import { defaultParams, GRAVITY, type Params } from '@/sim/params';

/** Fly a Part IV lesson's set-up with a change, for `seconds`. */
const fly = (id: string, mod: (p: Params) => void, seconds: number): Simulation => {
  const lesson = LESSONS.find((l) => l.id === id)!;
  const p = defaultParams();
  p.sim.level = 3;
  lesson.setup!(p);
  mod(p);
  const sim = new Simulation(p);
  for (let k = 0; k < seconds * 1000; k++) sim.step();
  return sim;
};
const at = (sim: Simulation, key: string, t: number) => {
  const w = sim.telemetry.window([key], 0);
  return w.series[0]![w.t.findIndex((x) => x >= t)]!;
};

describe('guidance geometry', () => {
  it('a target on a collision course has a still line of sight', () => {
    // Pursuer at the origin going +x at 4 m/s, target ahead and to the side closing on the same point.
    const g = geometry(v3(0, 0, 0), v3(4, 0, 0), v3(8, 0, 4), v3(0, 0, -2));
    expect(Math.abs(g.losRate)).toBeLessThan(1e-12);
    expect(g.closing).toBeGreaterThan(0);
  });

  it('proportional navigation commands nothing sideways on a collision course', () => {
    const p = defaultParams().guidance;
    const a = guidanceAccel(
      { ...p, law: 'pn', speed: Math.hypot(4, 0) },
      v3(0, 0, 0),
      v3(4, 0, 0),
      v3(8, 0, 4),
      v3(0, 0, -2),
    );
    // Only the speed hold along the line of sight remains.
    const u = v3(8 / Math.hypot(8, 4), 0, 4 / Math.hypot(8, 4));
    const along = a.x * u.x + a.z * u.z;
    expect(Math.hypot(a.x - along * u.x, a.z - along * u.z)).toBeLessThan(1e-9);
  });

  it('the weave is centred: no drift to one side', () => {
    const p = { ...defaultParams().guidance, weaveAcc: 3, weavePeriod: 4, targetHeadingDeg: 0 };
    const a = targetState(p, 0);
    const b = targetState(p, 4);
    expect(b.pos.z - a.pos.z).toBeCloseTo(0, 9);
  });
});

describe('lesson IV.24: pursuit against proportional navigation', () => {
  it('pure pursuit tail-chases and misses the weaving target; PN hits it on a shorter path', () => {
    const pursuit = fly('pronav', () => {}, 20);
    const pn = fly('pronav', (p) => (p.guidance.law = 'pn'), 20);
    expect(pursuit.engagement.over).toBe(true);
    expect(pn.engagement.over).toBe(true);
    expect(pursuit.engagement.minRange).toBeGreaterThan(0.5);
    expect(pn.engagement.minRange).toBeLessThan(0.2);
  }, 120_000);

  it('without a weave both reach the target, and pursuit takes longer', () => {
    const time = (law: 'pursuit' | 'pn') => {
      const lesson = LESSONS.find((l) => l.id === 'pronav')!;
      const p = defaultParams();
      p.sim.level = 3;
      lesson.setup!(p);
      p.guidance.weaveAcc = 0;
      p.guidance.law = law;
      const sim = new Simulation(p);
      for (let k = 0; k < 25000 && !sim.engagement.over; k++) sim.step();
      return { t: sim.t - p.guidance.start, miss: sim.engagement.minRange };
    };
    const a = time('pursuit');
    const b = time('pn');
    expect(a.miss).toBeLessThan(0.2);
    expect(b.miss).toBeLessThan(0.2);
    expect(a.t).toBeGreaterThan(1.3 * b.t);
  }, 120_000);
});

describe('lesson IV.26: inertial drift', () => {
  it('with no bias the navigator follows the drone exactly', () => {
    const sim = fly('ins', (p) => (p.sensors.gyroBias = { x: 0, y: 0, z: 0 }), 36);
    expect(at(sim, 'ins.err', 35)).toBeLessThan(0.01);
  }, 120_000);

  it('a gyro bias makes the error grow like g·b·t³/6', () => {
    const sim = fly('ins', () => {}, 36);
    const b = (0.05 * Math.PI) / 180;
    for (const t of [15, 30]) {
      const e = at(sim, 'ins.err', 5 + t);
      expect(Math.abs(e / insGyroError(b, GRAVITY, t) - 1)).toBeLessThan(0.02);
    }
    expect(insGyroError(b, GRAVITY, 30)).toBeCloseTo(38.5, 0);
  }, 120_000);
});

describe('lesson IV.27: range-only beacon', () => {
  it('the UKF stays within its own 2σ where the EKF is many σ off', () => {
    const worst = (kind: 'ekf' | 'ukf', seed: number) => {
      const sim = fly(
        'ukf',
        (p) => {
          p.sim.seed = seed;
          p.control.beaconFilter.kind = kind;
        },
        35,
      );
      const w = sim.telemetry.window(['rng.err', 'rng.sigma2'], 15);
      let m = 0;
      for (let i = 0; i < w.t.length; i++)
        m = Math.max(m, w.series[0]![i]! / (w.series[1]![i]! / 2));
      return m;
    };
    for (const seed of [1, 2]) {
      expect(worst('ekf', seed)).toBeGreaterThan(10);
      expect(worst('ukf', seed)).toBeLessThan(2);
    }
  }, 120_000);

  it("the EKF's NIS looks healthy while its estimate is many σ off: the NIS cannot see it", () => {
    const sim = fly('ukf', (p) => (p.control.beaconFilter.kind = 'ekf'), 35);
    const w = sim.telemetry.window(['rng.nis', 'rng.err', 'rng.sigma2'], 15);
    const nis = w.series[0]!.filter((v) => Number.isFinite(v));
    const mean = nis.reduce((a, b) => a + b, 0) / nis.length;
    // For one range the NIS has one degree of freedom: an honest filter averages 1.
    expect(mean).toBeGreaterThan(0.3);
    expect(mean).toBeLessThan(3);
    const ratio = Math.max(...w.series[1]!.map((e, i) => e / (w.series[2]![i]! / 2)));
    expect(ratio).toBeGreaterThan(10);
  }, 120_000);

  it('with three beacons around it both filters are fine: one range leaves the position undetermined', () => {
    const run = (kind: 'ekf' | 'ukf') => {
      const f = new RangeFilter(kind);
      const beacons = [
        { x: 10, z: 0 },
        { x: 0, z: 10 },
        { x: -10, z: -3 },
      ];
      f.reset([4, 4, 0, 0], 5, 0.5);
      for (let k = 0; k < 200; k++) {
        f.predict(0.1, 0.3);
        f.update(
          beacons.map((b) => Math.hypot(b.x, b.z)),
          beacons,
          0.02,
        );
      }
      return Math.hypot(f.x[0]!, f.x[1]!);
    };
    expect(run('ekf')).toBeLessThan(0.05);
    expect(run('ukf')).toBeLessThan(0.05);
  });
});
