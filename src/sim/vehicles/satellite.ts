import { qConj, qFromAxisAngle, qIntegrate, qMul, type Quat } from '@/math/quat';
import { add, cross, normalize, v3, type Vec3 } from '@/math/vec3';
import type { Params } from '../params';
import type { Environment, Vehicle } from './types';

/**
 * A rigid satellite (docs/aerospace-gnc.md §4, Chapter N): attitude and body rates only, no
 * gravity and no translation. Three reaction wheels along the body axes and, optionally, one pair
 * of on–off thrusters per axis. Euler's equation with the wheels' momentum h:
 *
 *   J·ω̇ = τ_w + τ_thr + τ_d − ω × (J·ω + h),   ḣ = −τ_w
 *
 * where τ_w is the torque the wheels put on the body. Without thrusters and disturbance the total
 * angular momentum J·ω + h is conserved (in the inertial frame).
 */
export interface SatelliteParams {
  /** Principal moments of inertia about the body x, y, z axes, kg·m². */
  inertia: Vec3;
  /** Largest torque one wheel can give, N·m. */
  wheelTorque: number;
  /** Largest momentum one wheel can store (its speed limit), N·m·s. */
  wheelMomentum: number;
  /** A constant disturbance torque in the body frame (gravity gradient, solar pressure), N·m. */
  disturbance: Vec3;
  /** Torque of one thruster pair while it fires, N·m, and the shortest pulse it can give, s. */
  thrusterTorque: number;
  minPulse: number;
  /** Propellant one firing pair uses, g/s. */
  thrusterFlow: number;
  /** The commanded attitude: a rotation of `slewDeg` about this axis from the start. */
  slewAxis: 'x' | 'y' | 'z' | 'diagonal';
  slewDeg: number;
  /** What holds the attitude: the wheels (quaternion feedback) or the thrusters (dead band). */
  actuator: 'wheels' | 'thrusters';
  /** Quaternion feedback τ = −kp·q_e − kd·ω, N·m and N·m·s. */
  kp: number;
  kd: number;
  /** Flip the error quaternion to the hemisphere with q₀ ≥ 0: always the shorter rotation. */
  shortest: boolean;
  /** Dump wheel momentum with the thrusters above `dumpStart`, down to `dumpStop`, N·m·s. */
  dump: boolean;
  dumpStart: number;
  dumpStop: number;
  /** Tell the wheels about the thruster torque, so they cancel it instead of the attitude error. */
  dumpFeedforward: boolean;
  /** Thruster control: dead band on each axis' angle, and the drift rate it fires to, °/s. */
  deadbandDeg: number;
  driftRateDeg: number;
  /**
   * A solar panel on a hinge about the body z axis (lesson IV.21): its inertia about that axis,
   * kg·m² (0: no panel, a rigid satellite), the frequency of the panel on a clamped hub, Hz, its
   * damping ratio, and which angle the attitude sensor measures (the hub's, or the panel tip's).
   */
  panel: { inertia: number; hz: number; zeta: number; sensor: 'hub' | 'tip' };
  /**
   * Fly the slew as a rest-to-rest profile at ± this torque, N·m, with that torque fed forward
   * and the feedback tracking the profile (0: the slew is a step of the target).
   */
  profileTorque: number;
  /** Shape the slew command into two (ZV) or three (ZVD) steps timed for a mode at `shaperHz`. */
  shaper: 'none' | 'zv' | 'zvd';
  shaperHz: number;
  /** Run the attitude estimator of lesson IV.22 (src/estimation/startracker.ts). */
  navigation: boolean;
  /** The gyro's true bias, °/h (1 °/h is 1 arc-second per second), and its noise, °/√h. */
  gyroBiasDegH: Vec3;
  gyroArw: number;
  /** Star tracker: updates per second and 1σ of each star direction, arc-seconds. */
  trackerHz: number;
  trackerArcsec: number;
  /** Estimate the gyro bias (the MEKF's last three states) or assume there is none. */
  estimateBias: boolean;
  /** The tracker is blind from `outageStart` for `outageLength` seconds (0: never). */
  outageStart: number;
  outageLength: number;
}

export interface SatelliteState {
  /** Attitude (body to inertial) and body rates, rad/s. */
  q: Quat;
  w: Vec3;
  /** Momentum stored in the three wheels, body frame, N·m·s. */
  h: Vec3;
  /** Torque the wheels and the thrusters put on the body over the last step, N·m. */
  wheelTorque: Vec3;
  thrustTorque: Vec3;
  /** Propellant used so far, g, and the thrusters' total firing time, s. */
  fuel: number;
  /** The panel's angle relative to the hub, rad, and its rate (zero without a panel). */
  eta: number;
  etaDot: number;
}

export interface SatelliteInput {
  /** Torque asked of the wheels (on the body), N·m. */
  wheel: Vec3;
  /** Each thruster pair: −1, 0 or +1. */
  thrusters: Vec3;
}

const clamp = (x: number, m: number) => Math.max(-m, Math.min(m, x));
const comps = ['x', 'y', 'z'] as const;

/** The torque a wheel actually gives: its torque limit, and nothing more once it is at full speed. */
export function wheelTorque(cmd: number, h: number, sp: SatelliteParams): number {
  const t = clamp(cmd, sp.wheelTorque);
  // ḣ = −τ: a wheel at its momentum limit cannot speed up further.
  if (h >= sp.wheelMomentum && t < 0) return 0;
  if (h <= -sp.wheelMomentum && t > 0) return 0;
  return t;
}

/**
 * The torque the panel's hinge puts on the hub about z, N·m: k·η + c·η̇ with k = J_p·ω_p² and
 * c = 2ζ·ω_p·J_p, where ω_p is the panel's frequency on a clamped hub. The hub of inertia J_z and
 * the panel J_p then obey J_z·θ̈ = τ + k·η + c·η̇ and J_p·(θ̈ + η̈) = −k·η − c·η̇.
 */
export function panelHinge(s: SatelliteState, sp: SatelliteParams): number {
  const jp = sp.panel.inertia;
  if (!(jp > 0)) return 0;
  const wn = 2 * Math.PI * sp.panel.hz;
  return jp * wn * wn * s.eta + 2 * sp.panel.zeta * wn * jp * s.etaDot;
}

/**
 * The panel's frequency on a free hub, Hz: the hub swings against the panel, so the mode is
 * stiffer than on a clamped hub by √((J_z + J_p)/J_z).
 */
export const panelFreeHz = (sp: SatelliteParams): number =>
  sp.panel.hz * Math.sqrt((sp.inertia.z + sp.panel.inertia) / sp.inertia.z);

/** One step, semi-implicit Euler: rates first, then the attitude with the new rates. */
export function stepSatellite(
  s: SatelliteState,
  u: SatelliteInput,
  dt: number,
  sp: SatelliteParams,
): void {
  const J = sp.inertia;
  const tw = v3();
  const tt = v3();
  let firing = 0;
  for (const c of comps) {
    tw[c] = wheelTorque(u.wheel[c], s.h[c], sp);
    const f = Math.sign(u.thrusters[c]);
    tt[c] = f * sp.thrusterTorque;
    firing += Math.abs(f);
  }
  const H = v3(J.x * s.w.x + s.h.x, J.y * s.w.y + s.h.y, J.z * s.w.z + s.h.z);
  const gyro = cross(s.w, H);
  const tau = add(add(tw, tt), sp.disturbance);
  // The panel pulls on the hub through its hinge: a spring and a damper on its relative angle.
  const hinge = panelHinge(s, sp);
  const wzDot = (tau.z - gyro.z + hinge) / J.z;
  s.w = v3(
    s.w.x + ((tau.x - gyro.x) / J.x) * dt,
    s.w.y + ((tau.y - gyro.y) / J.y) * dt,
    s.w.z + wzDot * dt,
  );
  if (sp.panel.inertia > 0) {
    // J_p·(θ̈ + η̈) = −hinge torque: the panel's absolute angle θ + η.
    s.etaDot += (-wzDot - hinge / sp.panel.inertia) * dt;
    s.eta += s.etaDot * dt;
  }
  s.q = qIntegrate(s.q, s.w, dt);
  s.h = v3(s.h.x - tw.x * dt, s.h.y - tw.y * dt, s.h.z - tw.z * dt);
  s.wheelTorque = tw;
  s.thrustTorque = tt;
  s.fuel += firing * sp.thrusterFlow * dt;
}

export function initialSatellite(): SatelliteState {
  return {
    q: { w: 1, x: 0, y: 0, z: 0 },
    w: v3(),
    h: v3(),
    wheelTorque: v3(),
    thrustTorque: v3(),
    fuel: 0,
    eta: 0,
    etaDot: 0,
  };
}

/** The unit axis of the commanded slew. */
export const slewAxis = (sp: SatelliteParams): Vec3 =>
  sp.slewAxis === 'diagonal'
    ? normalize(v3(1, 1, 1))
    : v3(+(sp.slewAxis === 'x'), +(sp.slewAxis === 'y'), +(sp.slewAxis === 'z'));

/** The commanded attitude. */
export const targetAttitude = (sp: SatelliteParams): Quat =>
  qFromAxisAngle(slewAxis(sp), (sp.slewDeg * Math.PI) / 180);

/** The attitude error q_t⁻¹ ⊗ q, in the hemisphere q₀ ≥ 0 when `shortest`. */
export function attitudeError(q: Quat, target: Quat, shortest: boolean): Quat {
  const e = qMul(qConj(target), q);
  return shortest && e.w < 0 ? { w: -e.w, x: -e.x, y: -e.y, z: -e.z } : e;
}

/** The angle between two attitudes, rad (always the shorter way, 0…π). */
export const errorAngle = (e: Quat): number => 2 * Math.acos(Math.min(1, Math.abs(e.w)));

/**
 * The Lyapunov function of quaternion feedback, V = ½ωᵀJω + 2kp(1 − q₀), J·s: the kinetic energy
 * plus a spring on the error. Without saturation V̇ = −kd·|ω|² ≤ 0.
 */
export function lyapunov(s: SatelliteState, e: Quat, sp: SatelliteParams): number {
  const J = sp.inertia;
  const kinetic = 0.5 * (J.x * s.w.x ** 2 + J.y * s.w.y ** 2 + J.z * s.w.z ** 2);
  return kinetic + 2 * sp.kp * (1 - e.w);
}

/** Total angular momentum in the inertial frame, N·m·s. */
export function inertialMomentum(s: SatelliteState, sp: SatelliteParams): Vec3 {
  const J = sp.inertia;
  const H = v3(J.x * s.w.x + s.h.x, J.y * s.w.y + s.h.y, J.z * s.w.z + s.h.z);
  // Rotate the body-frame vector by q: q ⊗ H ⊗ q*.
  const p = qMul(qMul(s.q, { w: 0, ...H }), qConj(s.q));
  return v3(p.x, p.y, p.z);
}

/** The satellite as a `Vehicle`: state (attitude error vector, rates, wheel momentum). */
export const satellite: Vehicle<SatelliteState, SatelliteInput> = {
  id: 'satellite',
  initial: () => initialSatellite(),
  step: (s, u, _env: Environment, dt, p: Params) => stepSatellite(s, u, dt, p.satellite),
  stateNames: ['θx', 'θy', 'θz', 'ωx', 'ωy', 'ωz', 'hx', 'hy', 'hz'],
  inputNames: ['τx', 'τy', 'τz'],
  // The attitude as twice the vector part of q_ref⁻¹ ⊗ q: small rotation angles.
  toVector: (s, ref) => {
    const e = qMul(qConj(ref.q), s.q);
    const k = e.w < 0 ? -2 : 2;
    return [k * e.x, k * e.y, k * e.z, s.w.x, s.w.y, s.w.z, s.h.x, s.h.y, s.h.z];
  },
  fromVector: (x, ref) => {
    const v = { w: 1, x: x[0]! / 2, y: x[1]! / 2, z: x[2]! / 2 };
    const n = Math.hypot(v.w, v.x, v.y, v.z);
    const dq = { w: v.w / n, x: v.x / n, y: v.y / n, z: v.z / n };
    return { ...ref, q: qMul(ref.q, dq), w: v3(x[3], x[4], x[5]), h: v3(x[6], x[7], x[8]) };
  },
  inputToVector: (u) => [u.wheel.x, u.wheel.y, u.wheel.z],
  inputFromVector: (v) => ({ wheel: v3(v[0], v[1], v[2]), thrusters: v3() }),
  /** At rest with empty wheels: any attitude is an equilibrium without a disturbance. */
  trim: () => ({ state: initialSatellite(), input: { wheel: v3(), thrusters: v3() } }),
};
