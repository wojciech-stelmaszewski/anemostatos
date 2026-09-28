import { describe, expect, it } from 'vitest';
import { L1Axis } from '@/control/l1ac';
import { decode, encode, forward, observe, OBS, ACT } from '@/control/policy';
import { POLICY_WEIGHTS } from '@/control/policy-weights';
import { ENTRIES, runArena, SCENARIOS, winners } from '@/engine/arena';
import { Simulation } from '@/engine/simulation';
import { v3 } from '@/math/vec3';
import { defaultParams, GRAVITY, type Params } from '@/sim/params';
import { initialState } from '@/sim/dynamics';
import { measurementOf } from '@/sim/sensors';

const maxErrorAfter = (
  p: Params,
  seconds: number,
  from: number,
  events?: (s: Simulation) => void,
) => {
  const sim = new Simulation(p);
  events?.(sim);
  let max = 0;
  let sq = 0;
  let n = 0;
  for (let k = 0; k < seconds * 1000; k++) {
    sim.step();
    if (sim.t > from) {
      const e = Math.hypot(
        sim.state.pos.x - sim.setpoint.x,
        sim.state.pos.y - sim.setpoint.y,
        sim.state.pos.z - sim.setpoint.z,
      );
      max = Math.max(max, e);
      sq += e * e;
      n++;
    }
  }
  return { max, rms: Math.sqrt(sq / n), crashed: sim.state.crashed };
};
const payload = (s: Simulation) =>
  s.schedule(12, (x) => x.setParams({ ...x.params, drone: { ...x.params.drone, mass: 1.3 } }));

describe('L1 adaptive', () => {
  it('the estimator converges to a constant force', () => {
    const axis = new L1Axis();
    const dt = 0.004;
    let v = 0;
    const d = -2; // unknown force, N
    let u = 0;
    for (let k = 0; k < 2000; k++) {
      // Closed loop: the plant is m·v̇ = u + d, and we cancel the estimate, u = −σ̂.
      u = -axis.update(u, v, 1, 20, 5, dt);
      v += (u + d) * dt;
    }
    expect(axis.filtered).toBeCloseTo(d, 0);
  });

  it('holds altitude through a payload much better than PID and ADRC', () => {
    const run = (kind: 'pid' | 'adrc' | 'l1ac') => {
      const p = defaultParams();
      p.control.l1.kind = kind;
      return maxErrorAfter(p, 25, 12, payload).max;
    };
    const l1 = run('l1ac');
    expect(l1).toBeLessThan(0.06);
    expect(l1).toBeLessThan(0.5 * run('adrc'));
    expect(l1).toBeLessThan(0.25 * run('pid'));
  });
});

describe('L3 compensation stages', () => {
  const aero = (comp: Params['control']['l3']['compensation']) => {
    const p = defaultParams();
    p.sim.level = 3;
    p.drone.rotorDrag = 0.02;
    p.wind.meanSpeed = 3;
    p.wind.gustsOn = false;
    p.wind.turbSigma = 0.5;
    p.setpoint.y = 3;
    p.setpoint.profile = 'figure8';
    p.setpoint.profileAmplitude = 2;
    p.setpoint.profilePeriod = 6.5;
    p.control.l3.outer = 'geometric';
    p.control.l3.compensation = comp;
    return maxErrorAfter(p, 45, 30).rms;
  };

  it('a learned residual model beats the integrator on a trajectory through wind', () => {
    const integrator = aero('none');
    expect(aero('learned')).toBeLessThan(0.4 * integrator);
    expect(aero('l1ac')).toBeLessThan(0.4 * integrator);
  });

  it('L1 compensation rejects a payload on L3', () => {
    const p = defaultParams();
    p.sim.level = 3;
    p.control.l3.outer = 'geometric';
    p.control.l3.compensation = 'l1ac';
    expect(maxErrorAfter(p, 25, 12, payload).max).toBeLessThan(0.12);
  });
});

describe('neural policy', () => {
  it('encode and decode are inverse within range', () => {
    const d = decode(encode(12, v3(1, -0.5, 2), 1), 1, 24.4);
    expect(d.thrust).toBeCloseTo(12, 6);
    expect(d.rates.x).toBeCloseTo(1, 6);
    expect(d.rates.y).toBeCloseTo(-0.5, 6);
    expect(d.rates.z).toBeCloseTo(2, 6);
  });

  it('the shipped weights are trained, with the right shape', () => {
    expect(POLICY_WEIGHTS.sizes).toEqual([OBS, 32, 32, ACT]);
    expect(POLICY_WEIGHTS.meta?.status).toBeUndefined(); // not the placeholder
    const s = initialState();
    s.pos = v3(0, 2, 0);
    const out = forward(POLICY_WEIGHTS, observe(measurementOf(s), { pos: v3(0, 2, 0), yaw: 0 }));
    // Hovering at the target: roughly hover thrust, small rates.
    const d = decode(out, 1, 24.4);
    expect(d.thrust).toBeGreaterThan(0.7 * GRAVITY);
    expect(d.thrust).toBeLessThan(1.3 * GRAVITY);
  });

  it('flies to a target without crashing', () => {
    const p = defaultParams();
    p.sim.level = 3;
    p.wind.enabled = false;
    p.control.l3.outer = 'policy';
    const r = maxErrorAfter(p, 20, 17, (s) => s.schedule(8, (x) => x.setTarget(v3(2, 2, 1))));
    expect(r.crashed).toBe(false);
    expect(r.rms).toBeLessThan(0.5);
  });
});

describe('arena', () => {
  it('runs every entry on a scenario and finds a winner', () => {
    const step = SCENARIOS.find((s) => s.id === 'step')!;
    const short = { ...step, seconds: 14 };
    const results = ENTRIES.filter((e) => e.id !== 'mppi').map((e) => runArena(e, short));
    for (const r of results) expect(Number.isFinite(r.rms)).toBe(true);
    expect(winners(results).step).toBeDefined();
  });
});
