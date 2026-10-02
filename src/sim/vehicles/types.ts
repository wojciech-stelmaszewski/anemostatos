import type { Mat } from '@/math/mat';
import type { Vec3 } from '@/math/vec3';
import type { Params } from '../params';

/**
 * The vehicle abstraction of Part IV (docs/aerospace-gnc.md §3.1).
 *
 * A vehicle is a state, an input and a discrete step. Everything else the analysis needs is
 * derived from that: `trim` finds a state and input that the step leaves where they are, and
 * `linearize` differentiates the step around them. To make the finite differences possible each
 * vehicle maps its state and input to plain vectors and back.
 *
 * The step is the simulator's own (semi-implicit Euler at the physics rate), so a linearisation
 * of it is exactly the discrete model the simulator flies, as `l1Plant` is for the quadrotor.
 */
export type VehicleId = 'quadrotor' | 'rocket' | 'tvc' | 'aircraft' | 'satellite' | 'lander';

/** What acts on the vehicle from outside: the air and any push from the user. */
export interface Environment {
  /** Wind velocity, world frame, m/s. */
  wind: Vec3;
  /** External force, world frame, N. */
  external: Vec3;
}

export interface Vehicle<S, U> {
  id: VehicleId;
  /** The state a run starts from. */
  initial(p: Params): S;
  /** Advance `s` in place by one physics step. */
  step(s: S, u: U, env: Environment, dt: number, p: Params): void;
  /** Names of the entries of `toVector`, for tables and charts. */
  stateNames: readonly string[];
  inputNames: readonly string[];
  /** The state as a vector, measured from `ref` where the state is not a vector space. */
  toVector(s: S, ref: S): number[];
  /** The state `x` away from `ref` (the inverse of `toVector`). */
  fromVector(x: readonly number[], ref: S): S;
  inputToVector(u: U): number[];
  inputFromVector(v: readonly number[]): U;
  /** A state and input that the step leaves unchanged (an equilibrium). */
  trim(p: Params): { state: S; input: U };
}

/** A discrete linear model x⁺ = A·x + B·u of a vehicle's step around a point. */
export interface Linearization {
  a: Mat;
  b: Mat;
  dt: number;
  stateNames: readonly string[];
  inputNames: readonly string[];
}
