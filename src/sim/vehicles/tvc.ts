import type { Params } from '../params';
import type { Environment, Vehicle } from './types';

/**
 * A launch vehicle in the pitch plane (docs/aerospace-gnc.md §4, Chapter L), frozen at one flight
 * condition: it climbs at a constant airspeed, and only the motion across the flight path is
 * simulated. A rigid body with an engine on a gimbal at the tail, an aerodynamic normal force
 * whose centre of pressure lies ahead of the centre of mass (so the airframe is unstable), the
 * first bending mode of a free-free beam, and a slosh mass on a spring in the tank.
 *
 * Signs: x is the drift across the flight path, θ the pitch of the nose towards +x, δ the gimbal
 * angle, positive when the thrust at the tail pushes towards +x (which turns the nose to −x).
 */
export interface TvcParams {
  /** Mass (without the slosh mass), kg, and pitch inertia about the centre of mass, kg·m². */
  mass: number;
  inertia: number;
  /** Length of the vehicle, m, and the distance from the gimbal up to the centre of mass. */
  length: number;
  gimbalArm: number;
  /** Thrust, N. */
  thrust: number;
  /** Airspeed, m/s; 0 is a hover with no air load. */
  speed: number;
  /** Normal force per radian of angle of attack at this dynamic pressure, N/rad. */
  normalForce: number;
  /** Distance of the centre of pressure ahead of the centre of mass, m (positive: unstable). */
  cpAhead: number;
  /** Gimbal: travel, °, rate limit, °/s, and the actuator's time constant, s. */
  gimbalMaxDeg: number;
  gimbalRateDeg: number;
  actuatorTau: number;
  /** First bending mode: frequency, Hz, and damping ratio. */
  bendHz: number;
  bendZeta: number;
  /** Where the rate gyro sits, as a fraction of the length from the tail (0) to the nose (1). */
  gyroStation: number;
  /** Slosh: moving mass, kg (0 = none), frequency, Hz, damping, and station ahead of the c.m., m. */
  sloshMass: number;
  sloshHz: number;
  sloshZeta: number;
  sloshAhead: number;
  /** Pitch at the start, °. */
  startPitchDeg: number;
}

export interface TvcState {
  /** Drift across the flight path and its rate, m and m/s. */
  x: number;
  u: number;
  /** Pitch, rad, and pitch rate, rad/s. */
  theta: number;
  omega: number;
  /** Gimbal angle, rad. */
  delta: number;
  /** First bending mode: tail deflection, m, and its rate. */
  eta: number;
  etaDot: number;
  /** Slosh mass displacement relative to the tank, m, and its rate. */
  slosh: number;
  sloshDot: number;
}

/** A gimbal command, rad (clamped to the travel by the actuator). */
export interface TvcInput {
  delta: number;
}

const DEG = Math.PI / 180;

/** First free-free mode of a uniform beam: βL and the shape's σ. */
const BETA = 4.730040745;
const SIG = 0.982502215;

/**
 * Mode shape of the first bending mode at ξ (0 = tail, 1 = nose), normalised to 1 at the tail.
 * Both ends swing the same way and the middle the other way; the nodes are at ξ ≈ 0.224 and 0.776.
 */
export function modeShape(xi: number): number {
  const b = BETA * xi;
  return (Math.cosh(b) + Math.cos(b) - SIG * (Math.sinh(b) + Math.sin(b))) / 2;
}

/**
 * Local slope of the mode shape at ξ, rad per metre of tail deflection: what a rate gyro there
 * adds to the rigid rotation. Negative in the aft half, zero at the middle, positive forward.
 */
export function modeSlope(xi: number, length: number): number {
  const b = BETA * xi;
  const d = Math.sinh(b) - Math.sin(b) - SIG * (Math.cosh(b) + Math.cos(b));
  return (BETA * d) / 2 / length;
}

/** Generalised mass of the mode for a uniform mass distribution, m·∫φ² dξ. */
export function modalMass(r: TvcParams): number {
  const n = 400;
  let s = 0;
  for (let i = 0; i < n; i++) s += modeShape((i + 0.5) / n) ** 2;
  return (r.mass * s) / n;
}

/** The constants of the rigid pitch dynamics θ̈ = μα·α − μc·δ, s⁻². */
export const muAlpha = (r: TvcParams): number => (r.normalForce * r.cpAhead) / r.inertia;
export const muDelta = (r: TvcParams): number => (r.thrust * r.gimbalArm) / r.inertia;

const clamp = (v: number, lim: number) => Math.min(Math.max(v, -lim), lim);

/** One step, semi-implicit Euler: accelerations from the present state, then velocities, then positions. */
export function stepTvc(
  s: TvcState,
  u: TvcInput,
  env: Environment,
  dt: number,
  r: TvcParams,
  mb = modalMass(r),
): void {
  // The actuator: a first-order lag towards the clamped command, with a rate limit.
  const cmd = clamp(u.delta, r.gimbalMaxDeg * DEG);
  const rate = clamp((cmd - s.delta) / Math.max(r.actuatorTau, 1e-6), r.gimbalRateDeg * DEG);
  s.delta = clamp(s.delta + rate * dt, r.gimbalMaxDeg * DEG);

  const alpha = r.speed > 0 ? s.theta - (s.u - env.wind.x) / r.speed : 0;
  const normal = r.normalForce * alpha;
  const side = r.thrust * Math.sin(s.delta); // the gimbal's push at the tail, across the body
  const wS = 2 * Math.PI * r.sloshHz;
  const fSlosh =
    r.sloshMass > 0 ? r.sloshMass * (wS * wS * s.slosh + 2 * r.sloshZeta * wS * s.sloshDot) : 0;
  const fBody = side + normal + fSlosh + env.external.x;
  const omegaDot = (normal * r.cpAhead - side * r.gimbalArm + fSlosh * r.sloshAhead) / r.inertia;
  // Across the flight path: the axial thrust tilted with the body, and the forces across it.
  const uDot = (r.thrust * Math.sin(s.theta) + fBody) / r.mass;
  const wB = 2 * Math.PI * r.bendHz;
  const etaDdot = -2 * r.bendZeta * wB * s.etaDot - wB * wB * s.eta + side / mb;
  // The slosh mass feels the tank's acceleration across the body, opposite to it.
  const sloshDdot =
    r.sloshMass > 0
      ? -2 * r.sloshZeta * wS * s.sloshDot -
        wS * wS * s.slosh -
        (fBody / r.mass + r.sloshAhead * omegaDot)
      : 0;

  s.u += uDot * dt;
  s.x += s.u * dt;
  s.omega += omegaDot * dt;
  s.theta += s.omega * dt;
  s.etaDot += etaDdot * dt;
  s.eta += s.etaDot * dt;
  s.sloshDot += sloshDdot * dt;
  s.slosh += s.sloshDot * dt;
}

export const initialTvc = (r: TvcParams): TvcState => ({
  x: 0,
  u: 0,
  theta: r.startPitchDeg * DEG,
  omega: 0,
  delta: 0,
  eta: 0,
  etaDot: 0,
  slosh: 0,
  sloshDot: 0,
});

/** What the vehicle's sensors see: pitch and rate at the gyro (rigid plus bending), drift, drift rate. */
export function tvcMeasure(s: TvcState, r: TvcParams): [number, number, number, number] {
  const k = modeSlope(r.gyroStation, r.length);
  return [s.theta + k * s.eta, s.omega + k * s.etaDot, s.x, s.u];
}

const NAMES = ['x', 'u', 'θ', 'ω', 'δ', 'η', 'η̇', 's', 'ṡ'] as const;

/** The planar rocket as a `Vehicle`. Its equilibrium is straight flight with the gimbal centred. */
export const tvcRocket: Vehicle<TvcState, TvcInput> = {
  id: 'tvc',
  initial: (p: Params) => initialTvc(p.tvc),
  step: (s, u, env, dt, p) => stepTvc(s, u, env, dt, p.tvc),
  stateNames: NAMES,
  inputNames: ['δ'],
  toVector: (s) => [s.x, s.u, s.theta, s.omega, s.delta, s.eta, s.etaDot, s.slosh, s.sloshDot],
  fromVector: (x) => ({
    x: x[0]!,
    u: x[1]!,
    theta: x[2]!,
    omega: x[3]!,
    delta: x[4]!,
    eta: x[5]!,
    etaDot: x[6]!,
    slosh: x[7]!,
    sloshDot: x[8]!,
  }),
  inputToVector: (u) => [u.delta],
  inputFromVector: (v) => ({ delta: v[0]! }),
  trim: (p: Params) => {
    const s = initialTvc(p.tvc);
    s.theta = 0;
    return { state: s, input: { delta: 0 } };
  },
};
