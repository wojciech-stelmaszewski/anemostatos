import { bestInput, valueIteration, type Grid2, type ValueProblem } from '@/math/grid';
import { clamp } from '@/math/util';
import { idleActuation, type Actuation } from '@/sim/dynamics';
import { GRAVITY, type Params } from '@/sim/params';
import {
  periodSteps,
  type ControlInput,
  type Controller,
  type InfoRow,
  type LoopTerms,
} from './types';

/**
 * Dynamic programming on the L1 drone (docs/aerospace-gnc.md §4, Chapter K, lesson IV.3).
 *
 * The state is (e, v): e = y − r and the vertical speed. The input is the thrust deviation from
 * hover, bounded by the motors: u ∈ [−m̂·g, T_max − m̂·g]. The cost is the same quadratic as the
 * LQR of lesson II.2, Σ (q_e·e² + q_v·v² + r·u²)·dt. Value iteration on a grid gives V(e, v), the
 * cost of the best possible future; the policy picks, at every sample, the input that minimises
 * one Bellman step. Without the limits V would be the LQR's xᵀPx and the policy its −Kx; with
 * them the two part where the LQR asks for thrust the motors do not have.
 */
export interface DpModel {
  problem: ValueProblem;
  grid: Grid2;
  iterations: number;
  uMin: number;
  uMax: number;
  dt: number;
}

/** The problem and its solution for these parameters, cached per setting. */
const cache = new Map<string, DpModel>();

export function dpModel(p: Params): DpModel {
  const c = p.control.lqr;
  const d = p.control.dp;
  const m = Math.max(p.control.model.mass, 0.05);
  const tMax = 4 * p.drone.maxMotorThrust;
  const uMin = -m * GRAVITY;
  const uMax = tMax - m * GRAVITY;
  const dt = d.dt;
  const key = JSON.stringify([c.qPos, c.qVel, c.r, m, tMax, d]);
  const hit = cache.get(key);
  if (hit) return hit;
  // Thrust levels evenly spaced through zero (hovering must cost nothing) and the two bounds.
  const stepU = (uMax - uMin) / (d.inputs - 1);
  const inputs = [uMin, uMax];
  for (let u = 0; u > uMin; u -= stepU) inputs.push(u);
  for (let u = stepU; u < uMax; u += stepU) inputs.push(u);
  inputs.sort((a, b) => a - b);
  // Exact one-step discretisation of the double integrator (zero-order hold).
  const problem: ValueProblem = {
    step: (e, v, u) => {
      const a = u / m;
      return [e + v * dt + 0.5 * a * dt * dt, v + a * dt];
    },
    cost: (e, v, u) => (c.qPos * e * e + c.qVel * v * v + c.r * u * u) * dt,
    inputs,
  };
  const { grid, iterations } = valueIteration(
    { x0: -d.eMax, x1: d.eMax, nx: d.n, y0: -d.vMax, y1: d.vMax, ny: d.n },
    problem,
    4000,
    1e-8,
  );
  const out = { problem, grid, iterations, uMin, uMax, dt };
  if (cache.size > 16) cache.clear();
  cache.set(key, out);
  return out;
}

export class DpController implements Controller {
  private held: Actuation = idleActuation();
  private last: LoopTerms = {
    setpoint: 0,
    measurement: 0,
    error: 0,
    parts: [],
    unsaturated: 0,
    output: 0,
    saturated: false,
  };

  reset(): void {
    this.held = idleActuation();
  }

  resetIntegrators(): void {}

  tick(input: ControlInput): Actuation {
    const { params: p, physDt, stepIndex } = input;
    const steps = periodSteps(p.control.rateHz, physDt);
    if (stepIndex % steps !== 0) return this.held;
    const model = dpModel(p);
    const m = input.sense();
    const sp = input.setpoint.pos.y;
    const e = m.pos.y - sp;
    const v = m.vel.y;
    const u = bestInput(model.grid, model.problem, e, v);
    const ff = p.control.model.mass * GRAVITY;
    const unsaturated = ff + u;
    const output = clamp(unsaturated, 0, 4 * p.drone.maxMotorThrust);
    this.last = {
      setpoint: sp,
      measurement: m.pos.y,
      error: sp - m.pos.y,
      parts: [
        { key: 'policy', label: 'argmin Bellman', value: u, like: 'p' },
        { key: 'ff', label: 'm̂·g', value: ff, ff: true, like: 'ff' },
      ],
      unsaturated,
      output,
      saturated: u <= model.uMin + 1e-9 || u >= model.uMax - 1e-9,
    };
    const f = output / 4;
    this.held = { motorCmd: [f, f, f, f], forceCmd: this.held.forceCmd };
    return this.held;
  }

  loops() {
    return { alt: this.last };
  }

  extras() {
    return {};
  }

  describe(p: Params): InfoRow[] {
    const d = dpModel(p);
    return [
      {
        label: 'value grid',
        value: `${d.grid.nx} × ${d.grid.ny}, ${d.problem.inputs.length} thrust levels`,
        hint: 'Bellman’s equation solved by value iteration; V is read between grid points by interpolation.',
      },
      { label: 'sweeps to converge', value: `${d.iterations}` },
      {
        label: 'thrust range',
        value: `${d.uMin.toFixed(1)} … +${d.uMax.toFixed(1)} N around hover`,
      },
    ];
  }
}
