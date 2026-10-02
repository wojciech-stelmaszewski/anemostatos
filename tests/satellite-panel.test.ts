import { describe, expect, it } from 'vitest';
import { panelResidual, unshapedResidual } from '@/analysis/panel';
import { profileTime, shapedSlew, shaperSteps } from '@/control/satellite';
import { Simulation } from '@/engine/simulation';
import { LESSONS } from '@/lessons/lessons';
import { defaultParams, type Params } from '@/sim/params';
import { panelFreeHz } from '@/sim/vehicles/satellite';

const lessonParams = (mod: (p: Params) => void = () => {}): Params => {
  const p = defaultParams();
  p.sim.level = 1;
  LESSONS.find((l) => l.id === 'panel')!.setup!(p);
  mod(p);
  return p;
};
const RES = { from: 15, until: 30 };

describe('the flexible panel (IV.21)', () => {
  it('without a panel the satellite is the rigid one of IV.18–IV.20', () => {
    const p = defaultParams();
    p.sim.level = 1;
    p.sim.vehicle = 'satellite';
    const sim = new Simulation(p);
    for (let k = 0; k < 5000; k++) sim.step();
    expect(sim.satellite!.eta).toBe(0);
  });

  it('on a free hub the panel swings at f_p·√((J_z + J_p)/J_z)', () => {
    const p = lessonParams((q) => {
      q.satellite.kp = 0;
      q.satellite.kd = 0;
      q.satellite.profileTorque = 0;
      q.satellite.slewDeg = 0;
    });
    expect(panelFreeHz(p.satellite)).toBeCloseTo(0.5 * Math.sqrt(3.3 / 3), 6);
    const sim = new Simulation(p);
    sim.satellite!.eta = 0.01;
    let crossings = 0;
    let prev = sim.satellite!.eta;
    for (let k = 0; k < 20000; k++) {
      sim.step();
      const e = sim.satellite!.eta;
      if (prev * e < 0) crossings++;
      prev = e;
    }
    expect(crossings / 2 / 20).toBeCloseTo(panelFreeHz(p.satellite), 1);
    expect(Math.abs(crossings / 2 / 20 - panelFreeHz(p.satellite))).toBeLessThan(0.03);
  });

  it('the shaper’s steps add up to the whole slew, and the shaped profile ends at rest', () => {
    for (const kind of ['zv', 'zvd'] as const) {
      const st = shaperSteps(kind, 0.5);
      expect(st.reduce((a, b) => a + b.a, 0)).toBeCloseTo(1, 12);
      expect(st[1]!.t).toBeCloseTo(1, 12);
    }
    const p = lessonParams((q) => (q.satellite.shaper = 'zvd'));
    const end = profileTime(p.satellite) + 2 / p.satellite.shaperHz + 0.01;
    const [phi, rate, acc] = shapedSlew(p.satellite, end);
    expect((phi * 180) / Math.PI).toBeCloseTo(30, 9);
    expect(rate).toBe(0);
    expect(acc).toBe(0);
  });

  it('the unshaped slew leaves the panel ringing; ZV at the free frequency removes 95 %', () => {
    const p = lessonParams();
    const ref = unshapedResidual(p, RES.from, RES.until);
    expect(ref).toBeGreaterThan(0.15);
    const at = (kind: 'zv' | 'zvd', hz: number) =>
      panelResidual(
        lessonParams((q) => Object.assign(q.satellite, { shaper: kind, shaperHz: hz })),
        RES.from,
        RES.until,
      ) / ref;
    expect(at('zv', panelFreeHz(p.satellite))).toBeLessThan(0.05);
    // Tuned to the clamped frequency the ZV shaper is not enough; ZVD tolerates the error.
    expect(at('zv', 0.5)).toBeGreaterThan(0.05);
    expect(at('zvd', 0.5)).toBeLessThan(0.02);
  }, 120_000);

  it('the wheels stay below their torque limit during the profiled slew', () => {
    const sim = new Simulation(lessonParams());
    let peak = 0;
    for (let k = 0; k < 15000; k++) {
      sim.step();
      peak = Math.max(peak, Math.abs(sim.satellite!.wheelTorque.z));
    }
    expect(peak).toBeLessThan(0.2);
  });

  it('a hub sensor cannot destabilise the panel; a tip sensor does at every gain', () => {
    const swing = (kp: number, kd: number, sensor: 'hub' | 'tip') => {
      const sim = new Simulation(
        lessonParams((q) => {
          Object.assign(q.satellite, { kp, kd });
          q.satellite.panel.sensor = sensor;
        }),
      );
      let early = 0;
      let late = 0;
      for (let k = 0; k < 40000; k++) {
        sim.step();
        const e = (Math.abs(sim.satellite!.eta) * 180) / Math.PI;
        if (sim.t > 5 && sim.t < 15) early = Math.max(early, e);
        if (sim.t > 30) late = Math.max(late, e);
      }
      return { early, late };
    };
    for (const [kp, kd] of [
      [0.2, 0.2],
      [4, 4],
      [50, 50],
    ] as const) {
      const hub = swing(kp, kd, 'hub');
      expect(hub.late).toBeLessThan(hub.early);
      expect(hub.late).toBeLessThan(0.3);
      expect(swing(kp, kd, 'tip').late).toBeGreaterThan(0.6);
    }
  }, 120_000);
});
