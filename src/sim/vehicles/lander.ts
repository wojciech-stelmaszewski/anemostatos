import { Rng } from '@/math/prng';
import { length, v3, type Vec3 } from '@/math/vec3';
import { GRAVITY, type Params } from '../params';
import { G0 } from './rocket';
import type { Environment, Vehicle } from './types';

/**
 * The lander of lesson IV.5 (docs/aerospace-gnc.md §4, Chapter K): the rocket of IV.4 in three
 * dimensions, as a point mass. The engine points its thrust in any direction (the attitude is
 * assumed to follow at once), between a minimum and a maximum while it burns, and the mass burns
 * away, ṁ = −‖T‖/(Isp·g₀). The pad is at the origin; y is up.
 */
export interface LanderParams {
  dryMass: number;
  fuel: number;
  thrustMin: number;
  thrustMax: number;
  isp: number;
  /** Nominal start, m and m/s (world frame, y up). */
  start: Vec3;
  startVel: Vec3;
  /** Dispersed starts: case 0 is the nominal one, case k > 0 a seeded draw within these. */
  dispersionCase: number;
  dispersePos: number;
  disperseVel: number;
  /** A steady push the guidance does not know about (wind on the body), N. */
  windForce: Vec3;
  /** Thrust delivered per thrust commanded: an engine that underperforms is below 1. */
  thrustScale: number;
  /** Glide slope: the lander must stay at least this many degrees above the horizon seen from the pad. */
  glideSlopeDeg: number;
  /** Touching down faster than this breaks the legs, m/s. */
  crashSpeed: number;
}

export interface LanderState {
  pos: Vec3;
  vel: Vec3;
  mass: number;
  fuel: number;
  /** Thrust the engine delivered over the last step, N. */
  thrust: Vec3;
  landed: boolean;
  crashed: boolean;
  /** Speed and distance from the pad centre at touchdown (NaN until then). */
  touchdownSpeed: number;
  miss: number;
}

export interface LanderInput {
  /** Commanded thrust vector, N; zero is engine off. */
  thrust: Vec3;
}

/** The start of a dispersed case: the nominal one for case 0. */
export function landerStart(l: LanderParams): { pos: Vec3; vel: Vec3 } {
  if (l.dispersionCase <= 0) return { pos: { ...l.start }, vel: { ...l.startVel } };
  const rng = new Rng(7919 * l.dispersionCase + 17);
  const u = (a: number) => rng.uniform(-a, a);
  return {
    pos: v3(
      l.start.x + u(l.dispersePos),
      l.start.y + u(l.dispersePos / 2),
      l.start.z + u(l.dispersePos),
    ),
    vel: v3(
      l.startVel.x + u(l.disperseVel),
      l.startVel.y + u(l.disperseVel),
      l.startVel.z + u(l.disperseVel),
    ),
  };
}

export function initialLander(l: LanderParams): LanderState {
  const { pos, vel } = landerStart(l);
  return {
    pos,
    vel,
    mass: l.dryMass + l.fuel,
    fuel: l.fuel,
    thrust: v3(),
    landed: false,
    crashed: false,
    touchdownSpeed: NaN,
    miss: NaN,
  };
}

/** The thrust the engine delivers for a command: off, or its magnitude clamped to the limits. */
export function landerThrust(cmd: Vec3, s: LanderState, l: LanderParams): Vec3 {
  const t = length(cmd);
  if (!(t > 0) || s.fuel <= 0) return v3();
  const k = (Math.min(Math.max(t, l.thrustMin), l.thrustMax) / t) * l.thrustScale;
  return v3(cmd.x * k, cmd.y * k, cmd.z * k);
}

/** One step, semi-implicit Euler like the other vehicles. */
export function stepLander(
  s: LanderState,
  u: LanderInput,
  ext: Vec3,
  dt: number,
  l: LanderParams,
): void {
  if (s.landed || s.crashed) {
    s.thrust = v3();
    return;
  }
  const t = landerThrust(u.thrust, s, l);
  const f = v3(ext.x + l.windForce.x, ext.y + l.windForce.y, ext.z + l.windForce.z);
  const burn = Math.min((length(t) / (l.isp * G0)) * dt, s.fuel);
  s.thrust = t;
  s.vel = v3(
    s.vel.x + ((t.x + f.x) / s.mass) * dt,
    s.vel.y + ((t.y + f.y) / s.mass - GRAVITY) * dt,
    s.vel.z + ((t.z + f.z) / s.mass) * dt,
  );
  s.pos = v3(s.pos.x + s.vel.x * dt, s.pos.y + s.vel.y * dt, s.pos.z + s.vel.z * dt);
  s.fuel -= burn;
  s.mass -= burn;
  if (s.pos.y <= 0) {
    s.pos = v3(s.pos.x, 0, s.pos.z);
    s.touchdownSpeed = length(s.vel);
    s.miss = Math.hypot(s.pos.x, s.pos.z);
    if (s.touchdownSpeed > l.crashSpeed) s.crashed = true;
    else s.landed = true;
    s.vel = v3();
    s.thrust = v3();
  }
}

/** The lander as a `Vehicle`: state (position, velocity, mass), input the thrust vector. */
export const lander: Vehicle<LanderState, LanderInput> = {
  id: 'lander',
  initial: (p: Params) => initialLander(p.lander),
  step: (s, u, env: Environment, dt, p) => stepLander(s, u, env.external, dt, p.lander),
  stateNames: ['x', 'y', 'z', 'vx', 'vy', 'vz', 'm'],
  inputNames: ['Tx', 'Ty', 'Tz'],
  toVector: (s) => [s.pos.x, s.pos.y, s.pos.z, s.vel.x, s.vel.y, s.vel.z, s.mass],
  fromVector: (x, ref) => ({
    ...ref,
    pos: v3(x[0]!, x[1]!, x[2]!),
    vel: v3(x[3]!, x[4]!, x[5]!),
    mass: x[6]!,
    landed: false,
    crashed: false,
  }),
  inputToVector: (u) => [u.thrust.x, u.thrust.y, u.thrust.z],
  inputFromVector: (v) => ({ thrust: v3(v[0]!, v[1]!, v[2]!) }),
  /** Hovering where it starts, the engine holding the weight. */
  trim(p: Params) {
    const s = initialLander(p.lander);
    s.vel = v3();
    return { state: s, input: { thrust: v3(0, s.mass * GRAVITY, 0) } };
  },
};
