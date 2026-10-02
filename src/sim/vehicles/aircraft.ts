import { v3, type Vec3 } from '@/math/vec3';
import { atmosphere, dynamicPressure } from '../atmosphere';
import { GRAVITY, type Params } from '../params';
import type { Environment, Vehicle } from './types';

/**
 * A fixed-wing aircraft in the vertical plane (docs/aerospace-gnc.md §4, Chapter M): three
 * degrees of freedom (speed, flight-path angle, pitch), lift, drag and pitching moment linear in
 * the angle of attack, the pitch rate and the elevator, scaled by the dynamic pressure of the
 * standard atmosphere [Stevens 2016, ch. 2]. No stall, no ground effect, no wind yet; the
 * thrust acts along the body axis, lapses with the air density and follows the throttle with
 * the lag of the engines' spool-up.
 *
 * The numbers are a small business jet (5 t, 25 m² of wing): at altitude its short period is
 * lightly damped, which is why such aircraft carry a pitch damper (lesson IV.12).
 */
export interface AircraftParams {
  /** Mass, kg; wing area, m²; mean aerodynamic chord, m; pitch inertia, kg·m². */
  mass: number;
  wingArea: number;
  chord: number;
  inertia: number;
  /** Thrust at full throttle at sea level, N; it scales with the density ratio ρ/ρ₀. */
  thrustMax: number;
  /** How slowly the engines follow the throttle (spool-up), a first-order lag, s. */
  engineTau: number;
  /** Lift: C_L = cl0 + clAlpha·α + clQ·q·c/(2V) + clDe·δe (per radian). */
  cl0: number;
  clAlpha: number;
  clQ: number;
  clDe: number;
  /** Drag polar: C_D = cd0 + cdK·C_L². */
  cd0: number;
  cdK: number;
  /** Pitching moment: C_m = cm0 + cmAlpha·α + cmQ·q·c/(2V) + cmDe·δe (per radian). */
  cm0: number;
  cmAlpha: number;
  cmQ: number;
  cmDe: number;
  /** The elevator actuator: first-order lag, s; rate limit, °/s; travel, ±°. */
  servoTau: number;
  servoRate: number;
  elevatorMax: number;
  /** Where a flight starts, trimmed for level flight: true airspeed, m/s, and altitude, m. */
  speed: number;
  altitude: number;
}

export interface AircraftState {
  /** True airspeed, m/s; angle of attack, rad; pitch rate, rad/s; pitch attitude, rad. */
  V: number;
  alpha: number;
  q: number;
  theta: number;
  /** Altitude and distance flown, m. */
  h: number;
  x: number;
  /** The elevator's actual deflection, rad (positive: trailing edge down, nose down). */
  de: number;
  /** Thrust the engines deliver, N: it follows the throttle with the lag `engineTau`. */
  thrust: number;
  crashed: boolean;
}

/** Elevator command, rad, and throttle, 0…1. */
export interface AircraftInput {
  de: number;
  throttle: number;
}

const RHO0 = 1.225;
const RAD = Math.PI / 180;

/** Lift, drag and pitching moment, N and N·m, for a state with elevator `de`. */
export function aero(s: AircraftState, de: number, a: AircraftParams) {
  const rho = atmosphere(s.h).density;
  const qbar = dynamicPressure(rho, s.V);
  const qhat = (s.q * a.chord) / (2 * s.V);
  const cl = a.cl0 + a.clAlpha * s.alpha + a.clQ * qhat + a.clDe * de;
  const cd = a.cd0 + a.cdK * cl * cl;
  const cm = a.cm0 + a.cmAlpha * s.alpha + a.cmQ * qhat + a.cmDe * de;
  return {
    rho,
    qbar,
    cl,
    lift: qbar * a.wingArea * cl,
    drag: qbar * a.wingArea * cd,
    moment: qbar * a.wingArea * a.chord * cm,
  };
}

/** Thrust at `throttle` (clamped to 0…1) and altitude `h`, N. */
export const thrustAt = (throttle: number, h: number, a: AircraftParams): number =>
  Math.min(Math.max(throttle, 0), 1) * a.thrustMax * (atmosphere(h).density / RHO0);

/** One step: the actuator, then the forces, then semi-implicit Euler like the other vehicles. */
export function stepAircraft(
  s: AircraftState,
  u: AircraftInput,
  ext: Vec3,
  dt: number,
  a: AircraftParams,
): void {
  if (s.crashed) return;
  // The elevator follows its command with a lag, no faster than its rate limit, within its travel.
  const lim = a.elevatorMax * RAD;
  const cmd = Math.min(Math.max(u.de, -lim), lim);
  const rate = a.servoTau > 0 ? (cmd - s.de) / a.servoTau : (cmd - s.de) / dt;
  const rmax = a.servoRate * RAD;
  s.de += Math.min(Math.max(rate, -rmax), rmax) * dt;
  s.de = Math.min(Math.max(s.de, -lim), lim);

  const f = aero(s, s.de, a);
  const target = thrustAt(u.throttle, s.h, a);
  s.thrust += a.engineTau > 0 ? ((target - s.thrust) * dt) / a.engineTau : target - s.thrust;
  const t = s.thrust;
  const gamma = s.theta - s.alpha;
  // An external push (N, world frame) along and across the flight path.
  const fAlong = ext.x * Math.cos(gamma) + ext.y * Math.sin(gamma);
  const fUp = -ext.x * Math.sin(gamma) + ext.y * Math.cos(gamma);
  const vDot = (t * Math.cos(s.alpha) - f.drag + fAlong) / a.mass - GRAVITY * Math.sin(gamma);
  const gammaDot =
    (t * Math.sin(s.alpha) + f.lift + fUp) / (a.mass * s.V) - (GRAVITY * Math.cos(gamma)) / s.V;
  const qDot = f.moment / a.inertia;
  s.V += vDot * dt;
  s.q += qDot * dt;
  s.alpha += (s.q - gammaDot) * dt;
  s.theta += s.q * dt;
  const g2 = s.theta - s.alpha;
  s.h += s.V * Math.sin(g2) * dt;
  s.x += s.V * Math.cos(g2) * dt;
  if (s.h <= 0) {
    s.h = 0;
    s.crashed = true;
  }
}

/** Accelerations (V̇, γ̇, q̇) of a level, unaccelerated candidate trim, for the Newton solve. */
function residual(V: number, h: number, alpha: number, de: number, thr: number, a: AircraftParams) {
  const s: AircraftState = {
    V,
    alpha,
    q: 0,
    theta: alpha,
    h,
    x: 0,
    de,
    thrust: 0,
    crashed: false,
  };
  const f = aero(s, de, a);
  const t = thrustAt(thr, h, a);
  return [
    (t * Math.cos(alpha) - f.drag) / a.mass,
    (t * Math.sin(alpha) + f.lift) / (a.mass * V) - GRAVITY / V,
    f.moment / a.inertia,
  ];
}

export interface Trim {
  state: AircraftState;
  input: AircraftInput;
  /** Dynamic pressure, Pa, and whether the throttle stays within 0…1 and the elevator within travel. */
  qbar: number;
  feasible: boolean;
}

/** Level flight at speed `V` and altitude `h`: α, δe and throttle by Newton's method. */
export function trimAircraft(a: AircraftParams, V: number, h: number): Trim {
  let x = [0.05, 0, 0.3];
  for (let it = 0; it < 30; it++) {
    const r = residual(V, h, x[0]!, x[1]!, x[2]!, a);
    if (Math.hypot(r[0]!, r[1]! * V, r[2]!) < 1e-12) break;
    const jac: number[][] = [[], [], []];
    for (let j = 0; j < 3; j++) {
      const d = 1e-6;
      const xp = [...x];
      xp[j] = xp[j]! + d;
      const rp = residual(V, h, xp[0]!, xp[1]!, xp[2]!, a);
      for (let i = 0; i < 3; i++) jac[i]![j] = (rp[i]! - r[i]!) / d;
    }
    const dx = solve3(jac, r);
    x = x.map((v, i) => v - dx[i]!);
  }
  const [alpha, de, throttle] = x as [number, number, number];
  const state: AircraftState = {
    V,
    alpha,
    q: 0,
    theta: alpha,
    h,
    x: 0,
    de,
    thrust: thrustAt(throttle, h, a),
    crashed: false,
  };
  const qbar = dynamicPressure(atmosphere(h).density, V);
  const feasible = throttle >= 0 && throttle <= 1 && Math.abs(de) <= a.elevatorMax * RAD;
  return { state, input: { de, throttle }, qbar, feasible };
}

/** Cramer's rule for a 3 × 3 system. */
function solve3(m: number[][], b: number[]): number[] {
  const det = (a: number[][]) =>
    a[0]![0]! * (a[1]![1]! * a[2]![2]! - a[1]![2]! * a[2]![1]!) -
    a[0]![1]! * (a[1]![0]! * a[2]![2]! - a[1]![2]! * a[2]![0]!) +
    a[0]![2]! * (a[1]![0]! * a[2]![1]! - a[1]![1]! * a[2]![0]!);
  const d = det(m);
  return [0, 1, 2].map((j) => det(m.map((row, i) => row.map((v, k) => (k === j ? b[i]! : v)))) / d);
}

export const initialAircraft = (a: AircraftParams): AircraftState =>
  trimAircraft(a, a.speed, a.altitude).state;

/** The aircraft as a `Vehicle`: state (V, α, q, θ, h, x, δe, T), inputs the elevator and throttle. */
export const aircraft: Vehicle<AircraftState, AircraftInput> = {
  id: 'aircraft',
  initial: (p: Params) => initialAircraft(p.aircraft),
  step: (s, u, env: Environment, dt, p) => stepAircraft(s, u, env.external, dt, p.aircraft),
  stateNames: ['V', 'alpha', 'q', 'theta', 'h', 'x', 'de', 'T'],
  inputNames: ['de', 'throttle'],
  toVector: (s) => [s.V, s.alpha, s.q, s.theta, s.h, s.x, s.de, s.thrust],
  fromVector: (x, ref) => ({
    ...ref,
    V: x[0]!,
    alpha: x[1]!,
    q: x[2]!,
    theta: x[3]!,
    h: x[4]!,
    x: x[5]!,
    de: x[6]!,
    thrust: x[7]!,
    crashed: false,
  }),
  inputToVector: (u) => [u.de, u.throttle],
  inputFromVector: (v) => ({ de: v[0]!, throttle: v[1]! }),
  trim: (p: Params) => {
    const t = trimAircraft(p.aircraft, p.aircraft.speed, p.aircraft.altitude);
    return { state: t.state, input: t.input };
  },
};

/** Position of the aircraft in the world (x forward, y up), for the scene and the charts. */
export const aircraftPosition = (s: AircraftState): Vec3 => v3(s.x, s.h, 0);
