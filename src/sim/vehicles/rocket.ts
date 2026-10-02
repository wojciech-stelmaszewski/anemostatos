import { v3, type Vec3 } from '@/math/vec3';
import { GRAVITY, type Params } from '../params';
import type { Environment, Vehicle } from './types';

/**
 * A rocket that moves only vertically (docs/aerospace-gnc.md §4, Chapter K, lesson IV.4): a
 * point mass whose mass burns away, ṁ = −T/(Isp·g₀), with an engine that is either off or
 * throttled between a minimum and a maximum thrust. Gravity and the engine, nothing else.
 */
export const G0 = 9.80665;

export interface RocketParams {
  /** Dry mass and the propellant loaded at the start, kg. */
  dryMass: number;
  fuel: number;
  /** Thrust limits while the engine burns, N; it can also be off. */
  thrustMax: number;
  thrustMin: number;
  /** Specific impulse, s. */
  isp: number;
  /** Where it starts, m, and its vertical speed there, m/s (negative: falling). */
  startAlt: number;
  startVel: number;
  /** A touchdown faster than this breaks the legs, m/s. */
  crashSpeed: number;
}

export interface RocketState {
  /** Altitude of the engine bell above the pad, and vertical speed, m and m/s. */
  h: number;
  v: number;
  /** Total mass, kg, and the propellant left in it. */
  mass: number;
  fuel: number;
  /** Thrust the engine delivered over the last step, N. */
  thrust: number;
  landed: boolean;
  crashed: boolean;
  /** Vertical speed at touchdown, m/s (NaN until it touches). */
  touchdown: number;
}

/** A thrust command: 0 is engine off, anything else is clamped to [thrustMin, thrustMax]. */
export interface RocketInput {
  thrust: number;
}

/** The thrust the engine can actually give for a command, with the fuel it has. */
export function engineThrust(cmd: number, s: RocketState, r: RocketParams): number {
  if (!(cmd > 0) || s.fuel <= 0) return 0;
  return Math.min(Math.max(cmd, r.thrustMin), r.thrustMax);
}

/** One step, semi-implicit Euler like the quadrotor. */
export function stepRocket(
  s: RocketState,
  u: RocketInput,
  ext: Vec3,
  dt: number,
  r: RocketParams,
): void {
  if (s.landed || s.crashed) {
    s.thrust = 0;
    return;
  }
  const t = engineThrust(u.thrust, s, r);
  const burn = Math.min((t / (r.isp * G0)) * dt, s.fuel);
  s.thrust = t;
  const a = (t + ext.y) / s.mass - GRAVITY;
  s.v += a * dt;
  s.h += s.v * dt;
  s.fuel -= burn;
  s.mass -= burn;
  if (s.h <= 0) {
    s.h = 0;
    s.touchdown = s.v;
    if (-s.v > r.crashSpeed) s.crashed = true;
    else s.landed = true;
    s.v = 0;
    s.thrust = 0;
  }
}

export function initialRocket(r: RocketParams): RocketState {
  return {
    h: r.startAlt,
    v: r.startVel,
    mass: r.dryMass + r.fuel,
    fuel: r.fuel,
    thrust: 0,
    landed: false,
    crashed: false,
    touchdown: NaN,
  };
}

/** The rocket as a `Vehicle`: state (h, v, m), input the thrust. */
export const rocket: Vehicle<RocketState, RocketInput> = {
  id: 'rocket',
  initial: (p: Params) => initialRocket(p.rocket),
  step: (s, u, env: Environment, dt, p) => stepRocket(s, u, env.external, dt, p.rocket),
  stateNames: ['h', 'v', 'm'],
  inputNames: ['thrust'],
  toVector: (s) => [s.h, s.v, s.mass],
  fromVector: (x, ref) => ({
    ...ref,
    h: x[0]!,
    v: x[1]!,
    mass: x[2]!,
    landed: false,
    crashed: false,
  }),
  inputToVector: (u) => [u.thrust],
  inputFromVector: (v) => ({ thrust: v[0]! }),
  /** Hovering at the start altitude with the engine holding the weight. */
  trim(p: Params) {
    const s = initialRocket(p.rocket);
    s.v = 0;
    return { state: s, input: { thrust: s.mass * GRAVITY } };
  },
};

/** Position of the rocket in the world, for the scene and the charts. */
export const rocketPosition = (s: RocketState): Vec3 => v3(0, s.h, 0);

/**
 * The altitude at which full thrust, lit from rest-free fall at `startAlt`, brings the rocket to
 * rest exactly at the pad: the ignition altitude of the fuel-optimal landing. Found by bisection
 * on the simulator's own step, with the mass burning away.
 */
export function ignitionAltitude(r: RocketParams, dt = 0.001): number {
  const stopsAt = (hIgn: number): number => {
    const s = initialRocket(r);
    let lit = false;
    for (let k = 0; k < 600000; k++) {
      if (!lit && s.h <= hIgn) lit = true;
      if (lit && s.v >= 0) return s.h; // stopped: how high above the pad
      const before = { ...s };
      stepRocket(s, { thrust: lit ? r.thrustMax : 0 }, v3(), dt, r);
      if (s.landed || s.crashed) return before.h + before.v * dt; // reached the pad still moving
    }
    return NaN;
  };
  let lo = 0;
  let hi = r.startAlt;
  for (let i = 0; i < 60; i++) {
    const mid = (lo + hi) / 2;
    if (stopsAt(mid) > 0) hi = mid;
    else lo = mid;
  }
  return (lo + hi) / 2;
}

/** The estimate with the mass held constant at its value at ignition: h₀·m·g/T for a start at rest. */
export function ignitionAltitudeConstantMass(r: RocketParams): number {
  const m = r.dryMass + r.fuel;
  const a = r.thrustMax / m - GRAVITY;
  // Free fall from (h0, v0) to h: v² = v0² + 2g(h0 − h); braking from h: v² = 2a·h.
  return (r.startVel ** 2 + 2 * GRAVITY * r.startAlt) / (2 * (a + GRAVITY));
}
