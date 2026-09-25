import {
  qFromAxisAngle,
  qFromTo,
  qIdentity,
  qIntegrate,
  qRotate,
  qToEuler,
  type Quat,
} from '@/math/quat';
import { add, clone, cross, length, normalize, scale, sub, v3, type Vec3 } from '@/math/vec3';
import { FOOT_HEIGHT, MOTOR_POSITIONS } from './drone';
import { GRAVITY, type DroneParams, type Level } from './params';

export interface DroneState {
  pos: Vec3;
  vel: Vec3;
  /** Attitude, body → world. */
  q: Quat;
  /** Body angular velocity, rad/s. */
  omega: Vec3;
  /** Actual thrust of each motor (after lag), N. */
  motors: [number, number, number, number];
  /** L2 only: actual applied force vector (after lag), N. */
  force: Vec3;
  landed: boolean;
  crashed: boolean;
}

export interface Actuation {
  /** Commanded thrust per motor, N (L1, L3). */
  motorCmd: [number, number, number, number];
  /** Commanded force vector, N (L2). */
  forceCmd: Vec3;
}

/** Breakdown of forces in the last step, world frame — drawn as arrows. */
export interface Forces {
  thrust: Vec3;
  gravity: Vec3;
  drag: Vec3;
  external: Vec3;
  net: Vec3;
}

export const initialState = (): DroneState => ({
  pos: v3(0, FOOT_HEIGHT, 0),
  vel: v3(),
  q: qIdentity(),
  omega: v3(),
  motors: [0, 0, 0, 0],
  force: v3(),
  landed: true,
  crashed: false,
});

export const idleActuation = (): Actuation => ({ motorCmd: [0, 0, 0, 0], forceCmd: v3() });

/** Per-axis quadratic drag on the velocity relative to the air. */
export const dragForce = (vel: Vec3, wind: Vec3, p: DroneParams): Vec3 => {
  const r = sub(vel, wind);
  return v3(
    -p.dragH * Math.abs(r.x) * r.x,
    -p.dragV * Math.abs(r.y) * r.y,
    -p.dragH * Math.abs(r.z) * r.z,
  );
};

/** Body torques produced by the four motor thrusts (docs/physics-model.md §4.3). */
export const motorTorques = (f: readonly number[], p: DroneParams): Vec3 => {
  let tx = 0;
  let tz = 0;
  for (let i = 0; i < 4; i++) {
    const r = MOTOR_POSITIONS[i]!;
    tx += -r.z * f[i]!;
    tz += r.x * f[i]!;
  }
  const ty = p.torqueCoeff * (f[0]! - f[1]! + f[2]! - f[3]!);
  return v3(tx, ty, tz);
};

const upY = v3(0, 1, 0);

/**
 * Advance the drone by dt using semi-implicit Euler. Mutates `s`, returns the force breakdown.
 */
export function stepDynamics(
  level: Level,
  s: DroneState,
  act: Actuation,
  wind: Vec3,
  external: Vec3,
  p: DroneParams,
  dt: number,
): Forces {
  const lag = 1 - Math.exp(-dt / Math.max(p.motorTau, 1e-4));
  const fmax = p.maxMotorThrust;

  if (level === 2) {
    // Point mass pushed by a force vector; motors only mirror its magnitude for the visuals.
    s.force = add(s.force, scale(sub(act.forceCmd, s.force), lag));
    const share = Math.min(length(s.force) / 4, fmax);
    for (let i = 0; i < 4; i++) s.motors[i] = share;
  } else {
    for (let i = 0; i < 4; i++) {
      const cmd = Math.min(Math.max(act.motorCmd[i]!, 0), fmax);
      s.motors[i] = s.motors[i]! + (cmd - s.motors[i]!) * lag;
    }
  }

  const total = s.motors[0] + s.motors[1] + s.motors[2] + s.motors[3];
  const thrust =
    level === 2 ? clone(s.force) : level === 1 ? v3(0, total, 0) : qRotate(s.q, v3(0, total, 0));
  const gravity = v3(0, -p.mass * GRAVITY, 0);
  let drag = dragForce(s.vel, wind, p);
  let ext = external;
  if (level === 1) {
    drag = v3(0, drag.y, 0);
    ext = v3(0, external.y, 0);
  }
  const net = add(add(add(thrust, gravity), drag), ext);

  // Translation.
  s.vel = add(s.vel, scale(net, dt / p.mass));
  if (level === 1) s.vel = v3(0, s.vel.y, 0);
  s.pos = add(s.pos, scale(s.vel, dt));

  // Attitude.
  if (level === 3) {
    const I = p.inertia;
    const tau = motorTorques(s.motors, p);
    const w = s.omega;
    const Iw = v3(I.x * w.x, I.y * w.y, I.z * w.z);
    const gyro = cross(w, Iw);
    const rhs = sub(sub(tau, gyro), scale(w, p.angularDamping));
    s.omega = add(w, v3((rhs.x / I.x) * dt, (rhs.y / I.y) * dt, (rhs.z / I.z) * dt));
    s.q = qIntegrate(s.q, s.omega, dt);
  } else if (level === 2) {
    // Cosmetic tilt along the applied force (foreshadows L3).
    const dir = length(s.force) > 1e-3 ? normalize(s.force) : upY;
    s.q = qFromTo(upY, dir);
    s.omega = v3();
  } else {
    s.q = qIdentity();
    s.omega = v3();
  }

  // Ground contact.
  s.landed = false;
  if (s.pos.y <= FOOT_HEIGHT) {
    const tilt = Math.acos(Math.min(1, Math.max(-1, qRotate(s.q, upY).y)));
    if (s.vel.y < -p.crashSpeed || (level === 3 && tilt > (80 * Math.PI) / 180)) s.crashed = true;
    s.pos = v3(s.pos.x, FOOT_HEIGHT, s.pos.z);
    s.vel = v3(0, Math.max(0, s.vel.y), 0);
    s.landed = s.vel.y <= 0;
    if (level === 3 && s.landed) {
      s.q = qFromAxisAngle(upY, qToEuler(s.q).yaw);
      s.omega = v3();
    }
  }

  return { thrust, gravity, drag, external: ext, net };
}
