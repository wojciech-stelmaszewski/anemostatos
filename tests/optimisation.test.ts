import { describe, expect, it } from 'vitest';
import { safetyFilter } from '@/control/cbf';
import { Simulation } from '@/engine/simulation';
import { QpSolver } from '@/math/qp';
import { v3 } from '@/math/vec3';
import { defaultParams, type Params } from '@/sim/params';
import { measurementOf } from '@/sim/sensors';
import { initialState } from '@/sim/dynamics';

const run = (p: Params, seconds: number, events?: (s: Simulation) => void) => {
  const sim = new Simulation(p);
  events?.(sim);
  for (let k = 0; k < seconds * 1000; k++) sim.step();
  return sim;
};

describe('MPC (L1)', () => {
  const ceiling = (kind: 'pid' | 'mpc') => {
    const p = defaultParams();
    p.wind.enabled = false;
    p.world.ceiling = 4;
    p.control.l1.kind = kind;
    p.control.mpc = { ...p.control.mpc, qPos: 50, qVel: 5, r: 0.05 };
    return run(p, 16, (s) => s.schedule(8, (x) => x.setTarget(v3(0, 3.95, 0))));
  };

  it('respects a ceiling the PID crosses, and still arrives', () => {
    expect(ceiling('pid').zoneTime).toBeGreaterThan(1);
    const mpc = ceiling('mpc');
    expect(mpc.zoneTime).toBe(0);
    expect(mpc.state.pos.y).toBeCloseTo(3.95, 2);
  });

  it('plans within the thrust range', () => {
    const p = defaultParams();
    p.wind.enabled = false;
    p.control.l1.kind = 'mpc';
    const sim = run(p, 10, (s) => s.schedule(5, (x) => x.setTarget(v3(0, 6, 0))));
    const { series } = sim.telemetry.window(['alt.u'], 5);
    const tMax = 4 * p.drone.maxMotorThrust;
    for (const u of series[0]!) expect(u).toBeLessThanOrEqual(tMax + 1e-9);
  });
});

describe('MPC (L3 outer)', () => {
  it('tracks a feasible figure-8 using the preview of the plan', () => {
    const p = defaultParams();
    p.sim.level = 3;
    p.wind.enabled = false;
    p.setpoint.y = 3;
    p.setpoint.profile = 'figure8';
    p.setpoint.profileAmplitude = 2;
    p.setpoint.profilePeriod = 6.5;
    p.control.l3.outer = 'mpc';
    const sim = new Simulation(p);
    let sq = 0;
    let n = 0;
    for (let k = 0; k < 35000; k++) {
      sim.step();
      if (sim.t > 20) {
        sq +=
          (sim.state.pos.x - sim.setpoint.x) ** 2 +
          (sim.state.pos.y - sim.setpoint.y) ** 2 +
          (sim.state.pos.z - sim.setpoint.z) ** 2;
        n++;
      }
    }
    expect(Math.sqrt(sq / n)).toBeLessThan(0.05);
  });
});

describe('MPPI', () => {
  const pillar = (setup: (p: Params) => void = () => {}) => {
    const p = defaultParams();
    p.sim.level = 3;
    p.wind.enabled = false;
    p.world.pillar = true;
    p.world.pillarX = 3;
    p.world.pillarZ = 0.15;
    p.world.pillarR = 0.6;
    p.control.l3.outer = 'mppi';
    setup(p);
    return p;
  };
  const jump = (s: Simulation) => s.schedule(10, (x) => x.setTarget(v3(6, 2, 0)));

  it('goes around the pillar to a target behind it', () => {
    const sim = run(pillar(), 16, jump);
    expect(sim.zoneTime).toBe(0);
    expect(Math.hypot(sim.state.pos.x - 6, sim.state.pos.y - 2, sim.state.pos.z)).toBeLessThan(0.2);
  });

  it('is reproducible with the same seed', () => {
    const a = run(pillar(), 12, jump);
    const b = run(pillar(), 12, jump);
    expect(a.state.pos).toEqual(b.state.pos);
  });

  it('a very high temperature averages left and right into indecision', () => {
    const sim = run(
      pillar((p) => (p.control.mppi.lambda = 50)),
      16,
      jump,
    );
    expect(Math.hypot(sim.state.pos.x - 6, sim.state.pos.z)).toBeGreaterThan(0.5);
  });

  it('runs comfortably faster than real time', () => {
    const t0 = performance.now();
    run(pillar(), 4, jump);
    expect((performance.now() - t0) / 4).toBeLessThan(300); // ms per simulated second
  });
});

describe('CBF safety filter', () => {
  const setup = () => {
    const p = defaultParams();
    p.sim.level = 3;
    p.world.pillar = true;
    p.world.pillarX = 3;
    p.world.pillarZ = 0;
    p.world.pillarR = 0.6;
    return p;
  };

  it('leaves the command alone far from the pillar', () => {
    const s = initialState();
    s.pos = v3(-3, 2, 0);
    s.vel = v3(1, 0, 0);
    const a = v3(2, 0, 0);
    const r = safetyFilter(a, measurementOf(s), setup(), 3, new QpSolver());
    expect(r.active).toBe(0);
    expect(r.a).toEqual(a);
  });

  it('near the pillar, changes only the component towards it (minimal correction)', () => {
    const s = initialState();
    s.pos = v3(2, 2, 0);
    s.vel = v3(2, 0, 0);
    const r = safetyFilter(v3(3, 0, 1), measurementOf(s), setup(), 3, new QpSolver());
    expect(r.active).toBe(1);
    expect(r.a.x).toBeLessThan(3); // braking towards the pillar…
    expect(r.a.z).toBeCloseTo(1, 3); // …the sideways part untouched
  });

  it('keeps a controller that flies straight at the pillar out of it', () => {
    const p = setup();
    p.wind.enabled = false;
    p.setpoint.profile = 'sine';
    p.setpoint.profileAxis = 'x';
    p.setpoint.profileAmplitude = 4;
    p.setpoint.profilePeriod = 10;
    p.control.l3.outer = 'geometric';
    const without = run(p, 30);
    expect(without.zoneTime).toBeGreaterThan(1);
    p.control.l3.safety = 'cbf';
    expect(run(p, 30).zoneTime).toBe(0);
  });
});
