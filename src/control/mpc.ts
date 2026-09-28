import { QpSolver } from '@/math/qp';
import { clamp } from '@/math/util';
import { v3, type Vec3 } from '@/math/vec3';
import { idleActuation, type Actuation } from '@/sim/dynamics';
import { GRAVITY, type Params } from '@/sim/params';
import {
  periodSteps,
  type ControlInput,
  type Controller,
  type InfoRow,
  type LoopTerms,
  type Plan,
} from './types';

export interface AxisMpcOptions {
  /** Prediction step, s, and number of steps. */
  dt: number;
  n: number;
  /** Input gain: p̈ = b·u. */
  b: number;
  qPos: number;
  qVel: number;
  r: number;
  uMin: number;
  uMax: number;
  /** Position bounds over the horizon (±Infinity = none). */
  pMin: number;
  pMax: number;
  /** Velocity bounds over the horizon (±Infinity = none). */
  vMax: number;
  iterations: number;
}

/**
 * Condensed linear MPC for one axis of a double integrator, p̈ = b·u (docs/beyond-pid.md §4):
 * the predicted positions and velocities are affine in the input sequence U,
 *   p = p̄ + G_p·U,   v = v̄ + G_v·U,
 * so tracking cost + input cost is a QP in U, with input bounds as boxes and position/velocity
 * bounds as linear constraints. Solved by ADMM, warm-started from the previous solution.
 */
export class AxisMpc {
  readonly solver = new QpSolver();
  private structure: {
    key: string;
    gp: number[][];
    gv: number[][];
    h: number[][];
    a: number[][];
  } | null = null;
  private matrixKey: object = {};
  /** The last plan: predicted positions p₁…p_N and inputs u₀…u_{N−1}. */
  plan: { p: number[]; u: number[] } = { p: [], u: [] };
  iterations = 0;
  converged = true;

  reset(): void {
    this.solver.reset();
    this.plan = { p: [], u: [] };
  }

  private build(o: AxisMpcOptions) {
    const key = [o.dt, o.n, o.b, o.qPos, o.qVel, o.r, o.pMin, o.pMax, o.vMax].join('|');
    if (this.structure?.key === key) return this.structure;
    const { n, dt, b } = o;
    // p_k = p0 + k·dt·v0 + Σ_{j<k} b·dt²·(k − j − ½)·u_j,   v_k = v0 + Σ_{j<k} b·dt·u_j   (k = 1…n)
    const gp = Array.from({ length: n }, (_, k) =>
      Array.from({ length: n }, (_, j) => (j <= k ? b * dt * dt * (k - j + 0.5) : 0)),
    );
    const gv = Array.from({ length: n }, (_, k) =>
      Array.from({ length: n }, (_, j) => (j <= k ? b * dt : 0)),
    );
    const h = Array.from({ length: n }, (_, i) =>
      Array.from({ length: n }, (_, j) => {
        let s = i === j ? 2 * o.r : 0;
        for (let k = 0; k < n; k++)
          s += 2 * (o.qPos * gp[k]![i]! * gp[k]![j]! + o.qVel * gv[k]![i]! * gv[k]![j]!);
        return s;
      }),
    );
    const a: number[][] = Array.from({ length: n }, (_, i) =>
      Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)),
    );
    if (Number.isFinite(o.pMin) || Number.isFinite(o.pMax)) a.push(...gp);
    if (Number.isFinite(o.vMax)) a.push(...gv);
    this.structure = { key, gp, gv, h, a };
    this.matrixKey = {};
    return this.structure;
  }

  /**
   * Plan from (p0, v0) to follow the reference positions ref[1…n] (and velocities vref), and
   * return the first input.
   */
  solve(p0: number, v0: number, ref: number[], vref: number[], o: AxisMpcOptions): number {
    const { gp, gv, h, a } = this.build(o);
    const n = o.n;
    const pBar = Array.from({ length: n }, (_, k) => p0 + (k + 1) * o.dt * v0);
    const q = Array.from({ length: n }, (_, j) => {
      let s = 0;
      for (let k = 0; k < n; k++)
        s +=
          2 * (o.qPos * gp[k]![j]! * (pBar[k]! - ref[k]!) + o.qVel * gv[k]![j]! * (v0 - vref[k]!));
      return s;
    });
    const l: number[] = new Array<number>(n).fill(o.uMin);
    const u: number[] = new Array<number>(n).fill(o.uMax);
    if (Number.isFinite(o.pMin) || Number.isFinite(o.pMax))
      for (let k = 0; k < n; k++) {
        l.push(o.pMin - pBar[k]!);
        u.push(o.pMax - pBar[k]!);
      }
    if (Number.isFinite(o.vMax))
      for (let k = 0; k < n; k++) {
        l.push(-o.vMax - v0);
        u.push(o.vMax - v0);
      }
    this.solver.maxIter = o.iterations;
    const r = this.solver.solve({ p: h, q, a, l, u }, this.matrixKey);
    this.iterations = r.iterations;
    this.converged = r.converged;
    const uSeq = r.x.map((x) => clamp(x, o.uMin, o.uMax));
    const p = gp.map((row, k) => pBar[k]! + row.reduce((s, g, j) => s + g * uSeq[j]!, 0));
    this.plan = { p, u: uSeq };
    return uSeq[0]!;
  }
}

/** Reference over the horizon: positions at t + k·dt (k = 1…n) and their finite-difference speeds. */
export function horizonReference(
  preview: ((tau: number) => Vec3) | undefined,
  fallback: Vec3,
  dt: number,
  n: number,
): { pos: Vec3[]; vel: Vec3[] } {
  const at = (tau: number) => (preview ? preview(tau) : fallback);
  const pos = Array.from({ length: n + 1 }, (_, k) => at(k * dt));
  const vel = pos
    .slice(1)
    .map((p, k) => v3((p.x - pos[k]!.x) / dt, (p.y - pos[k]!.y) / dt, (p.z - pos[k]!.z) / dt));
  return { pos: pos.slice(1), vel };
}

/** MPC options shared by L1 and the L3 outer stage, from the parameters. */
export const mpcTiming = (p: Params) => {
  const c = p.control.mpc;
  const hz = Math.max(c.hz, 1);
  return { dt: 1 / hz, n: Math.max(2, Math.round(c.horizon * hz)), hz };
};

/**
 * Level 1, linear MPC on altitude: plans the thrust over the horizon so that the drone reaches the
 * setpoint without ever crossing the ceiling and without asking the motors for more than they
 * have. Constraints are handled when planning, not after the fact.
 */
export class MpcAltitudeController implements Controller {
  readonly axis = new AxisMpc();
  private held: Actuation = idleActuation();
  private last: LoopTerms = {
    setpoint: 0,
    measurement: 0,
    error: 0,
    parts: parts(0, 0),
    unsaturated: 0,
    output: 0,
    saturated: false,
  };
  private planStart = 0;
  private pos0 = v3();

  reset(): void {
    this.axis.reset();
    this.held = idleActuation();
  }

  resetIntegrators(): void {}

  tick(input: ControlInput): Actuation {
    const { params: p, physDt, stepIndex } = input;
    const { dt, n, hz } = mpcTiming(p);
    const every = periodSteps(hz, physDt);
    if (stepIndex % every !== 0) return this.held;
    const m = input.sense();
    const c = p.control.mpc;
    const mHat = Math.max(p.control.model.mass, 0.05);
    const ff = p.control.feedforward ? mHat * GRAVITY : 0;
    const tMax = 4 * p.drone.maxMotorThrust;
    const ceiling = p.world.ceiling > 0 ? p.world.ceiling - c.margin : Infinity;
    const ref = horizonReference(input.preview, input.setpoint.pos, dt, n);
    const u0 = this.axis.solve(
      m.pos.y,
      m.vel.y,
      ref.pos.map((r) => Math.min(r.y, ceiling)),
      ref.vel.map((r) => r.y),
      {
        dt,
        n,
        b: 1 / mHat,
        qPos: c.qPos,
        qVel: c.qVel,
        r: c.r,
        uMin: -ff,
        uMax: tMax - ff,
        pMin: -Infinity,
        pMax: ceiling,
        vMax: Infinity,
        iterations: c.iterations,
      },
    );
    const output = clamp(ff + u0, 0, tMax);
    const sp = input.setpoint.pos.y;
    this.last = {
      setpoint: sp,
      measurement: m.pos.y,
      error: sp - m.pos.y,
      parts: parts(u0, ff),
      unsaturated: ff + u0,
      output,
      saturated: output !== ff + u0,
    };
    this.planStart = stepIndex * physDt;
    this.pos0 = v3(m.pos.x, m.pos.y, m.pos.z);
    const f = output / 4;
    this.held = { motorCmd: [f, f, f, f], forceCmd: this.held.forceCmd };
    return this.held;
  }

  loops() {
    return { alt: this.last };
  }

  extras() {
    return {
      'mpc.iterations': this.axis.iterations,
      'mpc.converged': this.axis.converged ? 1 : 0,
      'mpc.plan.max': this.axis.plan.p.length ? Math.max(...this.axis.plan.p) : NaN,
    };
  }

  plan(): Plan {
    // Drawn beside the drone (L1 flies on the vertical axis only).
    return {
      best: this.axis.plan.p.map((y) => v3(this.pos0.x + 0.35, y, this.pos0.z)),
      t0: this.planStart,
    };
  }

  describe(p: Params): InfoRow[] {
    const { dt, n } = mpcTiming(p);
    return [
      {
        label: 'horizon',
        value: `${n} steps × ${(dt * 1000).toFixed(0)} ms = ${(n * dt).toFixed(2)} s`,
        hint: 'How far ahead the controller plans. It re-plans at every step and applies only the first move (receding horizon).',
      },
      {
        label: 'constraints',
        value: `0 ≤ T ≤ ${(4 * p.drone.maxMotorThrust).toFixed(1)} N${p.world.ceiling > 0 ? `, y ≤ ${(p.world.ceiling - p.control.mpc.margin).toFixed(2)} m` : ''}`,
      },
      {
        label: 'QP iterations (cap)',
        value: `${this.axis.iterations} (${p.control.mpc.iterations})${this.axis.converged ? '' : ' — not converged'}`,
        hint: 'ADMM iterations the last solve needed. Capping them bounds the computing time — at the price of a less optimal plan.',
      },
    ];
  }
}

const parts = (u0: number, ff: number): LoopTerms['parts'] => [
  { key: 'u0', label: 'u₀ (first move of the plan)', value: u0, like: 'p' },
  { key: 'ff', label: 'FF', value: ff, ff: true, like: 'ff' },
];

/**
 * L3 outer stage: three independent axis MPCs on the point-mass model p̈ = a, previewing the
 * planned trajectory, with the drone's real limits as constraints — horizontal acceleration from
 * the tilt limit, vertical from the thrust range, speed limits, the ceiling. Its output, a desired
 * acceleration, goes through the same compensation and inner loops as any outer stage
 * ("MPC on top, INDI below", Sun et al. 2022).
 */
export class MpcOuter {
  readonly axes = { x: new AxisMpc(), y: new AxisMpc(), z: new AxisMpc() };
  private t0 = 0;

  reset(): void {
    for (const a of Object.values(this.axes)) a.reset();
  }

  update(
    s: { pos: Vec3; vel: Vec3 },
    preview: ((tau: number) => Vec3) | undefined,
    fallback: Vec3,
    p: Params,
    t: number,
  ): Vec3 {
    const c = p.control.mpc;
    const l3 = p.control.l3;
    const { dt, n } = mpcTiming(p);
    const ref = horizonReference(preview, fallback, dt, n);
    const aH = GRAVITY * Math.tan((l3.maxTiltDeg * Math.PI) / 180);
    const aUp = (4 * p.drone.maxMotorThrust) / Math.max(p.control.model.mass, 0.1) - GRAVITY;
    const ceiling = p.world.ceiling > 0 ? p.world.ceiling - c.margin : Infinity;
    const base = {
      dt,
      n,
      b: 1,
      qPos: c.qPos,
      qVel: c.qVel,
      r: c.r,
      iterations: c.iterations,
      pMin: -Infinity,
    };
    const ax = this.axes.x.solve(
      s.pos.x,
      s.vel.x,
      ref.pos.map((r) => r.x),
      ref.vel.map((r) => r.x),
      {
        ...base,
        uMin: -aH,
        uMax: aH,
        pMax: Infinity,
        vMax: l3.vMaxH,
      },
    );
    const az = this.axes.z.solve(
      s.pos.z,
      s.vel.z,
      ref.pos.map((r) => r.z),
      ref.vel.map((r) => r.z),
      {
        ...base,
        uMin: -aH,
        uMax: aH,
        pMax: Infinity,
        vMax: l3.vMaxH,
      },
    );
    const ay = this.axes.y.solve(
      s.pos.y,
      s.vel.y,
      ref.pos.map((r) => Math.min(r.y, ceiling)),
      ref.vel.map((r) => r.y),
      { ...base, uMin: -0.8 * GRAVITY, uMax: aUp, pMax: ceiling, vMax: l3.vMaxV },
    );
    this.t0 = t;
    // The per-axis boxes allow √2·aH diagonally; the tilt limit is on the norm.
    const h = Math.hypot(ax, az);
    const k = h > aH ? aH / h : 1;
    return v3(ax * k, ay, az * k);
  }

  plan(): Plan {
    const { x, y, z } = this.axes;
    return {
      best: x.plan.p.map((px, k) => v3(px, y.plan.p[k] ?? NaN, z.plan.p[k] ?? NaN)),
      t0: this.t0,
    };
  }

  get iterations(): number {
    return Math.max(this.axes.x.iterations, this.axes.y.iterations, this.axes.z.iterations);
  }
}
