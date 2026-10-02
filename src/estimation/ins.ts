import { qIntegrate, qRotate, type Quat } from '@/math/quat';
import { v3, type Vec3 } from '@/math/vec3';

/**
 * Strapdown inertial navigation with no position fix (docs/aerospace-gnc.md §4, Chapter O,
 * lesson IV.26). The gyro is integrated into an attitude; the accelerometer, turned into the world
 * frame by that attitude and with gravity added back, is integrated twice into velocity and
 * position. Nothing corrects it, so every sensor error grows.
 *
 * A constant gyro bias b tilts the navigator's "up" by b·t. Gravity, seen through that tilt, then
 * appears as a horizontal acceleration g·b·t, which integrates to a position error
 *   e(t) ≈ g·b·t³/6
 * over intervals short against the Schuler period (84 minutes); the drone never gets there.
 */
export class Ins {
  q: Quat = { w: 1, x: 0, y: 0, z: 0 };
  pos: Vec3 = v3();
  vel: Vec3 = v3();
  started = false;
  /** Time since the navigator was started, s. */
  t = 0;

  reset(): void {
    this.started = false;
    this.t = 0;
  }

  /** Start from the truth. */
  start(q: Quat, pos: Vec3, vel: Vec3): void {
    this.q = { ...q };
    this.pos = { ...pos };
    this.vel = { ...vel };
    this.started = true;
    this.t = 0;
  }

  /** One IMU sample: body rates (rad/s) and specific force (m/s², body frame). */
  step(gyro: Vec3, acc: Vec3, gravity: number, dt: number): void {
    this.q = qIntegrate(this.q, gyro, dt);
    const f = qRotate(this.q, acc);
    this.vel = v3(this.vel.x + f.x * dt, this.vel.y + (f.y - gravity) * dt, this.vel.z + f.z * dt);
    this.pos = v3(
      this.pos.x + this.vel.x * dt,
      this.pos.y + this.vel.y * dt,
      this.pos.z + this.vel.z * dt,
    );
    this.t += dt;
  }
}

/** Horizontal position error after t seconds from a gyro bias b (rad/s): g·b·t³/6, m. */
export const insGyroError = (biasRad: number, gravity: number, t: number): number =>
  (gravity * Math.abs(biasRad) * t ** 3) / 6;
