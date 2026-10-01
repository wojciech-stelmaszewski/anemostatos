import { describe, expect, it } from 'vitest';
import { effectiveness } from '@/control/fault';
import { Simulation } from '@/engine/simulation';
import { LESSONS } from '@/lessons/lessons';
import { rank } from '@/math/mat';
import { defaultParams, GRAVITY, type Params } from '@/sim/params';

const lesson = LESSONS.find((l) => l.id === '3motors')!;

/** Lesson III.27's flight: hover at 5 m, the propeller of motor `lost` gone at 5 s. */
const fly = (mod: (p: Params) => void = () => {}, lost = 1, seconds = 26) => {
  const p = defaultParams();
  p.sim.level = 3;
  lesson.setup!(p);
  mod(p);
  const sim = new Simulation(p);
  const eff: [number, number, number, number] = [1, 1, 1, 1];
  eff[lost] = 0;
  sim.schedule(5, (s) =>
    s.setParams({ ...s.params, drone: { ...s.params.drone, motorEfficiency: eff } }),
  );
  let worst = 0;
  let crashedAt = NaN;
  for (let k = 0; k < seconds * 1000; k++) {
    sim.step();
    if (sim.state.crashed) {
      crashedAt = sim.t;
      break;
    }
    if (sim.t > 15) {
      const s = sim.state.pos;
      const sp = sim.setpoint;
      worst = Math.max(worst, Math.hypot(s.x - sp.x, s.y - sp.y, s.z - sp.z));
    }
  }
  return { sim, worst, crashedAt };
};
const saved = (p: Params) => {
  p.control.fault.enabled = true;
  p.control.fault.leadLag = true;
};

describe('losing a rotor', () => {
  it('the control effectiveness drops from rank 4 to 3', () => {
    const p = defaultParams();
    expect(rank(effectiveness(p))).toBe(4);
    for (let i = 0; i < 4; i++) {
      const q = structuredClone(p);
      q.drone.motorEfficiency[i] = 0;
      expect(rank(effectiveness(q))).toBe(3);
    }
    // A weak motor still leaves all four directions.
    p.drone.motorEfficiency = [1, 0.6, 1, 1];
    expect(rank(effectiveness(p))).toBe(4);
  });

  it('the cascade flips and crashes', () => {
    const r = fly();
    expect(r.crashedAt).toBeGreaterThan(5);
    expect(r.crashedAt).toBeLessThan(8);
  });

  it('the three-motor law needs the lead: without it the drone still falls', () => {
    expect(fly((p) => (p.control.fault.enabled = true)).crashedAt).toBeLessThan(15);
  });

  it('with it, the drone holds its place within 0.5 m for every motor lost', () => {
    for (const lost of [0, 1, 2, 3]) {
      const r = fly(saved, lost);
      expect(r.crashedAt).toBeNaN();
      expect(r.worst).toBeLessThan(0.5);
    }
  });

  it('it spins at the rate the torque balance predicts', () => {
    const { sim } = fly(saved);
    const p = sim.params;
    const predicted = (p.drone.torqueCoeff * p.drone.mass * GRAVITY) / p.drone.yawDamping;
    expect(Math.abs(sim.state.omega.y) / predicted).toBeGreaterThan(0.95);
    expect(Math.abs(sim.state.omega.y) / predicted).toBeLessThan(1.05);
    expect(lesson.predict!.truth(p)).toBeCloseTo(predicted, 9);
  });

  it('before the fault the fault-tolerant mode is the cascade', () => {
    const a = new Simulation({ ...defaultParams(), sim: { level: 3, seed: 1 } });
    const q = defaultParams();
    q.sim = { level: 3, seed: 1 };
    saved(q);
    const b = new Simulation(q);
    for (let k = 0; k < 6000; k++) {
      a.step();
      b.step();
    }
    expect(b.state.pos).toEqual(a.state.pos);
  });

  it('a knife edge: 10 ms of sensor delay is survived, 20 ms is not', () => {
    expect(fly((p) => (saved(p), (p.sensors.delayMs = 10))).crashedAt).toBeNaN();
    expect(fly((p) => (saved(p), (p.sensors.delayMs = 20))).crashedAt).toBeLessThan(15);
  });
});
