import { describe, expect, it } from 'vitest';
import { bangBangLimits, brakingDistance, minimumTime, switching } from '@/control/bangbang';
import { dpModel } from '@/control/dp';
import { designLqr } from '@/control/lqr';
import { Simulation } from '@/engine/simulation';
import { LESSONS } from '@/lessons/lessons';
import { bestInput, sample, valueIteration } from '@/math/grid';
import { defaultParams, GRAVITY, type Params } from '@/sim/params';

const lesson = (id: string) => LESSONS.find((l) => l.id === id)!;
/** Fly a lesson's set-up and events, with an optional change, for `seconds`. */
const fly = (id: string, mod: (p: Params) => void, seconds: number) => {
  const l = lesson(id);
  const p = defaultParams();
  p.sim.level = l.level;
  l.setup!(p);
  mod(p);
  const sim = new Simulation(p);
  sim.script = l.events ?? null;
  sim.reset();
  for (let k = 0; k < seconds * 1000; k++) sim.step();
  return sim;
};
const settle = (sim: Simulation, t0: number, target: number, tol = 0.05) => {
  const { t, series } = sim.telemetry.window(['pos.y'], t0);
  let s = NaN;
  for (let i = 0; i < t.length; i++) {
    if (Math.abs(series[0]![i]! - target) > tol) s = NaN;
    else if (Number.isNaN(s)) s = t[i]! - t0;
  }
  return s;
};

describe('value iteration on a grid', () => {
  it('solves a known problem: the minimum time to the origin of a 1D integrator', () => {
    // x⁺ = x + u·dt, |u| ≤ 1, cost dt per step away from the origin: V(x) = |x|.
    const dt = 0.05;
    const { grid } = valueIteration(
      { x0: -2, x1: 2, nx: 81, y0: -1, y1: 1, ny: 3 },
      {
        step: (x, y, u) => [x + u * dt, y],
        cost: (x) => (Math.abs(x) > 0.025 ? dt : 0),
        inputs: [-1, 0, 1],
      },
    );
    for (const x of [-1.5, -0.5, 0.75, 1.9]) expect(sample(grid, x, 0)).toBeCloseTo(Math.abs(x), 1);
  });
});

describe('IV.1: the costate of the LQR', () => {
  it('P_ev = m·√(q·r) for the discrete design, for several weights', () => {
    for (const [q, r] of [
      [64, 0.5],
      [100, 1],
      [50, 2],
    ] as const) {
      const p = defaultParams();
      p.control.lqr.qPos = q;
      p.control.lqr.r = r;
      const d = designLqr(p, 0.004);
      expect(d.p[0]![1]! / Math.sqrt(q * r)).toBeCloseTo(1, 3);
      // u = −R⁻¹Bᵀλ: k₁ ≈ √(q/r), within 2 % (the sampling).
      expect(Math.abs(d.k[0]! / Math.sqrt(q / r) - 1)).toBeLessThan(0.02);
    }
  });

  /** Fly lesson IV.1 and read the controller at its own rate (250 Hz) after the step. */
  const samples = (tau = 0.03) => {
    const l = lesson('costate');
    const p = defaultParams();
    l.setup!(p);
    p.drone.motorTau = tau;
    p.control.model.motorTau = tau;
    const sim = new Simulation(p);
    sim.script = l.events!;
    sim.reset();
    const rows: {
      t: number;
      le: number;
      lv: number;
      e: number;
      v: number;
      ul: number;
      u: number;
    }[] = [];
    for (let k = 0; k < 14000; k++) {
      sim.step();
      if (k % 4 !== 0 || sim.t < 8) continue;
      const ex = sim.controller.extras();
      rows.push({
        t: sim.t,
        le: ex['lqr.lambda.e']!,
        lv: ex['lqr.lambda.v']!,
        e: ex['lqr.x.e']!,
        v: ex['lqr.x.v']!,
        ul: ex['lqr.u.lambda']!,
        u: sim.controller.loops().alt!.unsaturated - p.control.model.mass * 9.81,
      });
    }
    return { p, rows };
  };

  it('in flight: λ_v jumps to the predicted value and −R⁻¹Bᵀλ follows the command within 4 %', () => {
    const { p, rows } = samples();
    const truth = lesson('costate').predict!.truth(p);
    expect(truth).toBeCloseTo(-Math.sqrt(64 * 0.5), 3);
    expect(Math.abs(rows.find((r) => r.t > 8.004)!.lv / truth - 1)).toBeLessThan(0.01);
    let worst = 0;
    for (const r of rows) if (Math.abs(r.u) > 1) worst = Math.max(worst, Math.abs(r.ul / r.u - 1));
    expect(worst).toBeLessThan(0.04);
  }, 120_000);

  it('the costate equation holds along the flown path within 12 %, and 3.5 % with fast motors', () => {
    const residual = (tau: number) => {
      const { p, rows } = samples(tau);
      const q = p.control.lqr;
      let worst = 0;
      let scale = 0;
      for (let k = 1; k < rows.length - 1; k++) {
        if (rows[k]!.t < 8.05) continue;
        const dt = rows[k + 1]!.t - rows[k - 1]!.t;
        const r = rows[k]!;
        const re = (rows[k + 1]!.le - rows[k - 1]!.le) / dt + q.qPos * r.e;
        const rv = (rows[k + 1]!.lv - rows[k - 1]!.lv) / dt + q.qVel * r.v + r.le;
        worst = Math.max(worst, Math.abs(re), Math.abs(rv));
        scale = Math.max(scale, Math.abs(q.qPos * r.e), Math.abs(r.le));
      }
      return worst / scale;
    };
    expect(residual(0.03)).toBeLessThan(0.13);
    expect(residual(0.002)).toBeLessThan(0.035);
  }, 120_000);
});

describe('IV.2: bang-bang', () => {
  it('the braking distance and the minimum time', () => {
    expect(brakingDistance(3, 9.81, 0)).toBeCloseTo(9 / (2 * 9.81), 9);
    // Drag shortens the stop.
    expect(brakingDistance(3, 9.81, 0.25)).toBeLessThan(brakingDistance(3, 9.81, 0));
    const l = bangBangLimits(defaultParams());
    expect(minimumTime(1.5, GRAVITY, l.aDown)).toBeCloseTo(0.715, 2);
    // On the switching curve the function is zero.
    expect(switching(brakingDistance(2, l.aUp, 0), 2, { ...l, k: 0 }, 0)).toBeCloseTo(0, 9);
  });

  it('without a lead the motor lag makes it late; with 40 ms it reaches the floor', () => {
    const late = fly('bangbang', () => {}, 16);
    const tStar = lesson('bangbang').predict!.truth(late.params);
    expect(settle(late, 10, 1.5)).toBeGreaterThan(1.2 * tStar);
    const lead = fly('bangbang', (p) => (p.control.bangbang.lead = 0.04), 16);
    expect(settle(lead, 10, 1.5)).toBeLessThan(1.05 * tStar);
  }, 120_000);

  it('a PID with large gains rides the limits too, and gets there as fast', () => {
    const pid = fly(
      'bangbang',
      (p) => {
        p.control.l1.kind = 'pid';
        p.control.alt.kp = 60;
        p.control.alt.kd = 10;
      },
      16,
    );
    const tStar = lesson('bangbang').predict!.truth(pid.params);
    expect(settle(pid, 10, 1.5)).toBeLessThan(1.05 * tStar);
    // The default PID is many times slower.
    const slow = fly('bangbang', (p) => (p.control.l1.kind = 'pid'), 20);
    const s = settle(slow, 10, 1.5);
    expect(Number.isNaN(s) || s > 3).toBe(true);
  }, 120_000);
});

describe('IV.3: the value function', () => {
  const lqrParams = () => {
    const p = defaultParams();
    p.control.l1.kind = 'dp';
    return p;
  };

  it('the policy is the LQR inside the motor limits and the saturated LQR beyond', () => {
    const p = lqrParams();
    const dm = dpModel(p);
    const d = designLqr(p, dm.dt);
    const step = (dm.uMax - dm.uMin) / (p.control.dp.inputs - 1);
    for (const e of [-1, -0.5, 0.5, 1]) {
      const u = bestInput(dm.grid, dm.problem, e, 0);
      expect(Math.abs(u + d.k[0]! * e)).toBeLessThan(1.2 * step);
    }
    expect(bestInput(dm.grid, dm.problem, 2, 0)).toBeCloseTo(dm.uMin, 9);
    expect(bestInput(dm.grid, dm.problem, -2, 0)).toBeCloseTo(dm.uMax, 9);
  });

  it('the true cost is xᵀPx up to the threshold and above it beyond, asymmetrically', () => {
    const p = lqrParams();
    const dm = dpModel(p);
    const d = designLqr(p, dm.dt);
    const m = p.control.model.mass;
    const { qPos, qVel, r } = p.control.lqr;
    // Cost of the saturated LQR from (e0, 0) on the model.
    const cost = (e0: number) => {
      let e = e0;
      let v = 0;
      let J = 0;
      for (let k = 0; k < 1500; k++) {
        const u = Math.min(Math.max(-(d.k[0]! * e + d.k[1]! * v), dm.uMin), dm.uMax);
        J += (qPos * e * e + qVel * v * v + r * u * u) * dm.dt;
        e += v * dm.dt + (0.5 * u * dm.dt * dm.dt) / m;
        v += (u / m) * dm.dt;
      }
      return J;
    };
    const quad = (e: number) => d.p[0]![0]! * e * e;
    const threshold = lesson('bellman').predict!.truth(p);
    expect(threshold).toBeCloseTo(0.99, 2);
    expect(cost(0.9) / quad(0.9)).toBeCloseTo(1, 3);
    expect(cost(4) / quad(4)).toBeGreaterThan(1.14);
    expect(cost(-4) / quad(-4)).toBeGreaterThan(1.05);
    expect(cost(-4) / quad(-4)).toBeLessThan(cost(4) / quad(4));
    // The grid's own policy is no better than the saturated LQR at large steps (within 1 %).
    const dpCost = (e0: number) => {
      let e = e0;
      let v = 0;
      let J = 0;
      for (let k = 0; k < 1500; k++) {
        const u = bestInput(dm.grid, dm.problem, e, v);
        J += (qPos * e * e + qVel * v * v + r * u * u) * dm.dt;
        e += v * dm.dt + (0.5 * u * dm.dt * dm.dt) / m;
        v += (u / m) * dm.dt;
      }
      return J;
    };
    expect(Math.abs(dpCost(4) / cost(4) - 1)).toBeLessThan(0.01);
  });

  it('the DP controller flies both steps of the lesson', () => {
    const sim = fly('bellman', () => {}, 24);
    expect(sim.state.crashed).toBe(false);
    expect(Math.abs(sim.state.pos.y - 4)).toBeLessThan(0.1);
  }, 120_000);
});
