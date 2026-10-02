import { qConj, qIdentity, qIntegrate, qRotate, type Quat } from '@/math/quat';
import { add, cross, dot, length, scale, sub, v3, type Vec3 } from '@/math/vec3';

/**
 * Attitude from a gyro and an accelerometer (docs/analysis.md §4, Chapter I): the complementary
 * filter on the rotation group, in Mahony's explicit form [Mahony 2008].
 *
 * The gyro is integrated. The accelerometer's direction is taken for "up" and the estimate is
 * turned towards it at the rate kp·e, which makes the pair a high-pass on the gyro and a low-pass
 * on the accelerometer with the crossover at kp rad/s. With ki > 0 the same error also feeds an
 * estimate of the gyro bias. A heading reference, if given, corrects rotation about "up" only.
 */
export interface AttitudeGains {
  /** Crossover between gyro and accelerometer, rad/s (1/τ of a complementary filter). */
  kp: number;
  /** Bias adaptation gain, 1/s². 0 = the gyro is believed as it is. */
  ki: number;
  /**
   * Ignore the accelerometer while its magnitude differs from g by more than this fraction:
   * a reading that is not 1 g long is not gravity alone. 0 = always trust it.
   */
  gate: number;
}

const UP = v3(0, 1, 0);
const NORTH = v3(1, 0, 0);

export class AttitudeFilter {
  q: Quat = qIdentity();
  /** Estimated gyro bias, rad/s, body frame. */
  bias: Vec3 = v3();
  /** Whether the last accelerometer sample was used. */
  trusted = true;

  reset(q: Quat = qIdentity()): void {
    this.q = { ...q };
    this.bias = v3();
    this.trusted = true;
  }

  /**
   * One step. `gyro` in rad/s and `acc` (specific force) in m/s², body frame; `north` is the
   * body-frame direction of the world x-axis from a heading sensor, or null without one.
   */
  update(
    gyro: Vec3,
    acc: Vec3,
    north: Vec3 | null,
    g: AttitudeGains,
    gravity: number,
    dt: number,
  ): Quat {
    const inv = qConj(this.q);
    const upHat = qRotate(inv, UP);
    let e = v3();
    const a = length(acc);
    this.trusted = a > 1e-6 && (g.gate <= 0 || Math.abs(a / gravity - 1) <= g.gate);
    if (this.trusted) e = cross(scale(acc, 1 / a), upHat);
    if (north) {
      // Heading error, kept about the estimated vertical so that it cannot disturb the tilt.
      const h = cross(north, qRotate(inv, NORTH));
      e = add(e, scale(upHat, dot(upHat, h)));
    }
    this.bias = sub(this.bias, scale(e, g.ki * dt));
    const omega = add(sub(gyro, this.bias), scale(e, g.kp));
    this.q = qIntegrate(this.q, omega, dt);
    return this.q;
  }
}

/** Angle between the true and the estimated body up-axis, rad: the error that tilts the thrust. */
export function tiltError(truth: Quat, estimate: Quat): number {
  const c = dot(qRotate(truth, UP), qRotate(estimate, UP));
  return Math.acos(Math.min(1, Math.max(-1, c)));
}

/** Heading difference between two attitudes, rad: the angle between the projections of the body's
 *  forward axes (x) on the horizontal plane. */
export function headingError(truth: Quat, estimate: Quat): number {
  const f = (q: Quat) => {
    const x = qRotate(q, v3(1, 0, 0));
    return Math.atan2(-x.z, x.x);
  };
  let d = f(estimate) - f(truth);
  while (d > Math.PI) d -= 2 * Math.PI;
  while (d < -Math.PI) d += 2 * Math.PI;
  return Math.abs(d);
}
