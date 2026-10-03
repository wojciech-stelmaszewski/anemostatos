import type { Mat } from '@/math/mat';
import type { Params } from '../params';
import type { Environment, Vehicle } from './types';

/**
 * A chaser near a target in a circular orbit (docs/aerospace-gnc.md §4, Chapter O, lesson IV.25),
 * in the target's local frame: x radial (up, away from the Earth), y along-track (the direction of
 * flight). In the orbit plane only. The Clohessy–Wiltshire equations [Clohessy 1960]:
 *
 *   ẍ = 3n²x + 2n·ẏ + aₓ,   ÿ = −2n·ẋ + a_y
 *
 * with n the target's orbital rate. They are linear, so the step is exact: the state transition
 * over a time step is the matrix exponential of the system. The orbit is slow (a lap takes an hour
 * and a half), so simulated time runs `warp` times faster than the simulator's clock.
 */
export interface RendezvousParams {
  /** The target's orbital period, min (92.6: about 400 km up). */
  periodMin: number;
  /** Orbital seconds per simulated second. */
  warp: number;
  /** Where the chaser starts relative to the target, m: radial and along-track. */
  startX: number;
  startY: number;
  /** A single burn along-track at the start, m/s (the naive way: point at the target and go). */
  burn: number;
  /** The largest acceleration the thrusters give on each axis, m/s². */
  accelMax: number;
  /** What flies it after the burn: nothing (coast), or a model-predictive controller. */
  control: 'coast' | 'mpc';
  /** MPC: sample time, orbital s; horizon, samples; weights on position, velocity and thrust. */
  mpcStep: number;
  mpcHorizon: number;
  qPos: number;
  qVel: number;
  rAcc: number;
  /** Docking: closer than this, m, at a relative speed below `dockSpeed`, m/s. */
  dockRadius: number;
  dockSpeed: number;
}

export interface ChaserState {
  x: number;
  y: number;
  vx: number;
  vy: number;
  /** Thrust acceleration being applied, m/s². */
  ax: number;
  ay: number;
  /** Orbital time since the start, s, and the velocity change spent, m/s. */
  t: number;
  dv: number;
  /** Docked (inside the radius slowly enough) or hit (inside it too fast); then it stops. */
  docked: boolean;
  hit: boolean;
  /** The relative speed when it reached the docking radius, m/s (NaN before). */
  contactSpeed: number;
}

export interface ChaserInput {
  ax: number;
  ay: number;
}

/** Orbital rate n, rad/s. */
export const meanMotion = (p: RendezvousParams): number => (2 * Math.PI) / (p.periodMin * 60);

/** The continuous system ż = F·z + G·a, z = (x, y, ẋ, ẏ). */
export function cwSystem(n: number): { f: Mat; g: Mat } {
  return {
    f: [
      [0, 0, 1, 0],
      [0, 0, 0, 1],
      [3 * n * n, 0, 0, 2 * n],
      [0, 0, -2 * n, 0],
    ],
    g: [
      [0, 0],
      [0, 0],
      [1, 0],
      [0, 1],
    ],
  };
}

/**
 * The exact discrete model over `dt` with the input held: z⁺ = A·z + B·a. The exponential of the
 * augmented matrix [[F, G], [0, 0]]·dt, by a Taylor series (n·dt is small).
 */
export function cwDiscrete(n: number, dt: number): { a: Mat; b: Mat } {
  const { f, g } = cwSystem(n);
  const m: Mat = Array.from({ length: 6 }, (_, i) =>
    Array.from({ length: 6 }, (_, j) => (i < 4 ? (j < 4 ? f[i]![j]! : g[i]![j - 4]!) * dt : 0)),
  );
  const e: Mat = Array.from({ length: 6 }, (_, i) =>
    Array.from({ length: 6 }, (_, j) => (i === j ? 1 : 0)),
  );
  let term = e.map((r) => [...r]);
  for (let k = 1; k <= 20; k++) {
    term = term.map((r) => r.map((_, j) => r.reduce((s, v, l) => s + v * m[l]![j]!, 0) / k));
    for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) e[i]![j]! += term[i]![j]!;
  }
  return {
    a: e.slice(0, 4).map((r) => r.slice(0, 4)),
    b: e.slice(0, 4).map((r) => r.slice(4, 6)),
  };
}

/**
 * Where a burn of `dv` along-track from rest leaves the chaser one orbit later, relative to where
 * it started: x(t) = (2dv/n)(1 − cos nt), y(t) = (4dv/n)·sin nt − 3dv·t, so after a lap
 * Δy = −6π·dv/n. Pushing forward makes it climb, slow down and fall behind.
 */
export const driftPerOrbit = (dv: number, n: number): number => (-6 * Math.PI * dv) / n;

export function initialChaser(p: RendezvousParams): ChaserState {
  return {
    x: p.startX,
    y: p.startY,
    vx: 0,
    vy: p.burn,
    ax: 0,
    ay: 0,
    t: 0,
    dv: Math.abs(p.burn),
    docked: false,
    hit: false,
    contactSpeed: NaN,
  };
}

/** Cache of the discrete model per (n, dt). */
let stepModel: { key: string; a: Mat; b: Mat } | null = null;

/** Advance by `dt` of orbital time with the thrust `u` (clipped to the limit). */
export function stepChaser(s: ChaserState, u: ChaserInput, dt: number, p: RendezvousParams): void {
  if (s.docked || s.hit) return;
  const n = meanMotion(p);
  const key = `${n}|${dt}`;
  if (stepModel?.key !== key) stepModel = { key, ...cwDiscrete(n, dt) };
  const clip = (v: number) => Math.max(-p.accelMax, Math.min(p.accelMax, v));
  s.ax = clip(u.ax);
  s.ay = clip(u.ay);
  const z = [s.x, s.y, s.vx, s.vy];
  const { a, b } = stepModel;
  const next = a.map(
    (r, i) => r.reduce((acc, v, j) => acc + v * z[j]!, 0) + b[i]![0]! * s.ax + b[i]![1]! * s.ay,
  );
  [s.x, s.y, s.vx, s.vy] = next as [number, number, number, number];
  s.t += dt;
  s.dv += (Math.abs(s.ax) + Math.abs(s.ay)) * dt;
  if (Math.hypot(s.x, s.y) <= p.dockRadius) {
    const v = Math.hypot(s.vx, s.vy);
    s.contactSpeed = v;
    if (v <= p.dockSpeed) s.docked = true;
    else s.hit = true;
  }
}

export const chaser: Vehicle<ChaserState, ChaserInput> = {
  id: 'chaser',
  initial: (p: Params) => initialChaser(p.rendezvous),
  step: (s: ChaserState, u: ChaserInput, _env: Environment, dt: number, p: Params) =>
    stepChaser(s, u, dt * p.rendezvous.warp, p.rendezvous),
  stateNames: ['x', 'y', 'vx', 'vy'],
  inputNames: ['ax', 'ay'],
  toVector: (s, ref) => [s.x - ref.x, s.y - ref.y, s.vx - ref.vx, s.vy - ref.vy],
  fromVector: (x, ref) => ({
    ...ref,
    x: ref.x + x[0]!,
    y: ref.y + x[1]!,
    vx: ref.vx + x[2]!,
    vy: ref.vy + x[3]!,
  }),
  inputToVector: (u) => [u.ax, u.ay],
  inputFromVector: (v) => ({ ax: v[0]!, ay: v[1]! }),
  // At the target, at rest relative to it, nothing moves.
  trim: (p: Params) => ({
    state: { ...initialChaser(p.rendezvous), x: 0, y: 0, vy: 0, dv: 0 },
    input: { ax: 0, ay: 0 },
  }),
};
