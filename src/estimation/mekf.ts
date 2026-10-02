import { qIdentity, qIntegrate, qConj, qRotate, qMul, qNormalize, type Quat } from '@/math/quat';
import { add, mul, solve, sub, transpose, type Mat } from '@/math/mat';
import { v3, type Vec3 } from '@/math/vec3';

/**
 * Multiplicative extended Kalman filter for attitude [Markley 2003] (docs/analysis.md §4,
 * Chapter I, lesson III.23).
 *
 * A quaternion has four numbers and three degrees of freedom, so a Kalman filter on it would carry
 * a singular covariance. The MEKF keeps a reference quaternion q̂ outside the filter and estimates
 * a small rotation δθ away from it (body frame), plus the gyro bias: six states, a 6 × 6
 * covariance. After every update δθ is folded into q̂ and reset to zero.
 *
 * Vector measurements (gravity from the accelerometer, north from the magnetometer) are tested
 * before they are used: the normalised innovation squared, νᵀS⁻¹ν, follows χ² with three degrees
 * of freedom when the measurement agrees with the filter. Above the gate it is rejected.
 */
export interface MekfSettings {
  /** Gyro noise density the filter assumes, rad/s. */
  gyroSigma: number;
  /** How fast the gyro bias may wander, rad/s per √s. */
  biasWalk: number;
  /** Direction noise of the two vector sensors, rad. */
  accSigma: number;
  magSigma: number;
  /** χ² gate on the NIS of each vector measurement; 0 = accept everything. */
  gate: number;
}

const skew = (v: Vec3): Mat => [
  [0, -v.z, v.y],
  [v.z, 0, -v.x],
  [-v.y, v.x, 0],
];

/** Rotation by the small angle vector δθ, as a quaternion. */
function qSmall(d: Vec3): Quat {
  const a = Math.hypot(d.x, d.y, d.z);
  if (a < 1e-12) return qIdentity();
  const s = Math.sin(a / 2) / a;
  return { w: Math.cos(a / 2), x: d.x * s, y: d.y * s, z: d.z * s };
}

export interface VectorUpdate {
  nis: number;
  accepted: boolean;
}

export class Mekf {
  q: Quat = qIdentity();
  bias: Vec3 = v3();
  /** Covariance of (δθ, δb). */
  p: Mat = diag6(0.01, 1e-4);
  /** The last update of each sensor, for the charts. */
  lastAcc: VectorUpdate = { nis: 0, accepted: true };
  lastMag: VectorUpdate = { nis: 0, accepted: true };

  reset(q: Quat = qIdentity(), attitudeSigma = 0.02, biasSigma = 0.01): void {
    this.q = { ...q };
    this.bias = v3();
    this.p = diag6(attitudeSigma ** 2, biasSigma ** 2);
    this.lastAcc = { nis: 0, accepted: true };
    this.lastMag = { nis: 0, accepted: true };
  }

  /** 1σ of attitude error about each body axis, rad (x roll, y yaw, z pitch). */
  sigma(i: 0 | 1 | 2): number {
    return Math.sqrt(Math.max(this.p[i]![i]!, 0));
  }

  /** Integrate the gyro and grow the covariance. */
  propagate(gyro: Vec3, s: MekfSettings, dt: number): void {
    const w = sub3(gyro, this.bias);
    this.q = qIntegrate(this.q, w, dt);
    const wx = skew(w);
    const f: Mat = Array.from({ length: 6 }, (_, i) =>
      Array.from({ length: 6 }, (__, j) => (i === j ? 1 : 0)),
    );
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) f[i]![j] = f[i]![j]! - wx[i]![j]! * dt;
      f[i]![i + 3] = -dt;
    }
    const q = diag6(s.gyroSigma ** 2 * dt, s.biasWalk ** 2 * dt);
    this.p = add(mul(mul(f, this.p), transpose(f)), q);
  }

  /**
   * Correct the heading only, with the measured direction of north (`meas`, body frame, unit,
   * horizontal): one degree of freedom, a rotation about the estimated vertical `up` (body frame).
   * A disturbed magnetometer then cannot tilt the estimate. The gate is the χ² quantile for one
   * degree of freedom.
   */
  updateHeading(meas: Vec3, up: Vec3, sigma: number, gate: number): VectorUpdate {
    const pred = qRotate(qConj(this.q), v3(1, 0, 0));
    const cr = v3(
      pred.y * meas.z - pred.z * meas.y,
      pred.z * meas.x - pred.x * meas.z,
      pred.x * meas.y - pred.y * meas.x,
    );
    // A rotation α about up moves the predicted north by α·(pred × up): ν = −atan2(up·(pred×meas), pred·meas).
    const nu = -Math.atan2(
      up.x * cr.x + up.y * cr.y + up.z * cr.z,
      pred.x * meas.x + pred.y * meas.y + pred.z * meas.z,
    );
    const h = [[up.x, up.y, up.z, 0, 0, 0]];
    const ph = this.p.map((row) => row.reduce((acc, v, j) => acc + v * h[0]![j]!, 0)); // P·Hᵀ
    const s = h[0]!.reduce((acc, v, i) => acc + v * ph[i]!, 0) + sigma * sigma;
    const nis = (nu * nu) / s;
    const accepted = !(gate > 0 && nis > gate);
    if (accepted) {
      const k = ph.map((v) => v / s);
      const dx = k.map((v) => v * nu);
      const ikh = sub(
        eye6(),
        k.map((ki) => h[0]!.map((hj) => ki * hj)),
      );
      const kkt = k.map((ki) => k.map((kj) => ki * kj * sigma * sigma));
      this.p = add(mul(mul(ikh, this.p), transpose(ikh)), kkt);
      this.q = qNormalize(qMul(this.q, qSmall(v3(dx[0]!, dx[1]!, dx[2]!))));
      this.bias = add3(this.bias, v3(dx[3]!, dx[4]!, dx[5]!));
    }
    return { nis, accepted };
  }

  /**
   * Correct with a measured direction `meas` (body frame, unit) of a known reference `ref`
   * (world frame, unit), with direction noise σ. Returns the NIS and whether it passed the gate.
   */
  updateVector(meas: Vec3, ref: Vec3, sigma: number, gate: number): VectorUpdate {
    const pred = qRotate(qConj(this.q), ref);
    // v_b = exp(−δθ×)·v̂_b ≈ v̂_b + v̂_b × δθ:  H = [v̂_b×, 0].
    const hTh = skew(pred);
    const h: Mat = hTh.map((row) => [...row, 0, 0, 0]);
    const r = [0, 1, 2].map((i) => [0, 1, 2].map((j) => (i === j ? sigma * sigma : 0)));
    const sMat = add(mul(mul(h, this.p), transpose(h)), r);
    const nu = [meas.x - pred.x, meas.y - pred.y, meas.z - pred.z];
    const sInvNu = solve(
      sMat,
      nu.map((v) => [v]),
    );
    const nis = nu.reduce((acc, v, i) => acc + v * sInvNu[i]![0]!, 0);
    const accepted = !(gate > 0 && nis > gate);
    if (accepted) {
      // K = P·Hᵀ·S⁻¹, as (S⁻¹·H·P)ᵀ.
      const k = transpose(solve(sMat, mul(h, this.p)));
      const dx = k.map((row) => row.reduce((acc, v, i) => acc + v * nu[i]!, 0));
      const ikh = sub(eye6(), mul(k, h));
      this.p = add(mul(mul(ikh, this.p), transpose(ikh)), mul(mul(k, r), transpose(k)));
      this.q = qNormalize(qMul(this.q, qSmall(v3(dx[0]!, dx[1]!, dx[2]!))));
      this.bias = add3(this.bias, v3(dx[3]!, dx[4]!, dx[5]!));
    }
    return { nis, accepted };
  }
}

const add3 = (a: Vec3, b: Vec3): Vec3 => v3(a.x + b.x, a.y + b.y, a.z + b.z);
const sub3 = (a: Vec3, b: Vec3): Vec3 => v3(a.x - b.x, a.y - b.y, a.z - b.z);
const eye6 = (): Mat => diag6(1, 1);
function diag6(a: number, b: number): Mat {
  return Array.from({ length: 6 }, (_, i) =>
    Array.from({ length: 6 }, (__, j) => (i === j ? (i < 3 ? a : b) : 0)),
  );
}
