import { add, eye, mul, mulAll, solve, sub, transpose, type Mat } from '@/math/mat';

/**
 * Linear Kalman filter with explicit matrices — small, readable, and it exposes its covariance so
 * the UI can draw ±2σ bands (docs/beyond-pid.md §4).
 */
export class KalmanFilter {
  x: Mat;
  p: Mat;

  constructor(x0: Mat, p0: Mat) {
    this.x = x0;
    this.p = p0;
  }

  /** x⁺ = F·x + G·u, P⁺ = F·P·Fᵀ + Q. */
  predict(f: Mat, q: Mat, g?: Mat, u?: Mat): void {
    this.x = g && u ? add(mul(f, this.x), mul(g, u)) : mul(f, this.x);
    this.p = add(mulAll(f, this.p, transpose(f)), q);
  }

  /** Correct with measurement z = H·x + noise(R). Returns the innovation. */
  update(h: Mat, r: Mat, z: Mat): Mat {
    const ht = transpose(h);
    const innovation = sub(z, mul(h, this.x));
    const s = add(mulAll(h, this.p, ht), r);
    // K = P Hᵀ S⁻¹, computed as (S⁻¹ H P)ᵀ since S and P are symmetric.
    const k = transpose(solve(s, mul(h, this.p)));
    this.x = add(this.x, mul(k, innovation));
    // Joseph form keeps P symmetric and positive definite.
    const ikh = sub(eye(this.p.length), mul(k, h));
    this.p = add(mulAll(ikh, this.p, transpose(ikh)), mulAll(k, r, transpose(k)));
    return innovation;
  }

  sigma(i: number): number {
    return Math.sqrt(Math.max(this.p[i]![i]!, 0));
  }
}

/**
 * Vertical position/velocity/accelerometer-bias filter for L1. The accelerometer drives the
 * prediction; position fixes correct it.
 *   state x = (y, v, b), a_true = a_meas − b − g
 */
export class AltitudeKalman {
  private kf = new KalmanFilter([[0], [0], [0]], eye(3));
  private started = false;

  reset(): void {
    this.started = false;
  }

  get y(): number {
    return this.kf.x[0]![0]!;
  }
  get v(): number {
    return this.kf.x[1]![0]!;
  }
  get bias(): number {
    return this.kf.x[2]![0]!;
  }
  sigma(i: 0 | 1 | 2): number {
    return this.kf.sigma(i);
  }

  /**
   * One filter step of length dt. `accY` is the measured specific force (vertical), `pos` the
   * position fix or null when no new fix arrived.
   */
  step(
    dt: number,
    accY: number,
    pos: number | null,
    noise: { accSigma: number; posSigma: number; biasSigma: number },
    gravity: number,
  ): void {
    if (!this.started) {
      if (pos === null) return;
      this.kf = new KalmanFilter(
        [[pos], [0], [0]],
        [
          [noise.posSigma ** 2 + 1e-6, 0, 0],
          [0, 1, 0],
          [0, 0, 0.25],
        ],
      );
      this.started = true;
      return;
    }
    const f = [
      [1, dt, -0.5 * dt * dt],
      [0, 1, -dt],
      [0, 0, 1],
    ];
    const g = [[0.5 * dt * dt], [dt], [0]];
    // Accelerometer noise enters like an acceleration input; the bias wanders slowly.
    const qa = noise.accSigma ** 2;
    const qb = noise.biasSigma ** 2 * dt;
    const q = [
      [(qa * dt ** 4) / 4, (qa * dt ** 3) / 2, 0],
      [(qa * dt ** 3) / 2, qa * dt * dt, 0],
      [0, 0, qb],
    ];
    this.kf.predict(f, q, g, [[accY - gravity]]);
    if (pos !== null) this.kf.update([[1, 0, 0]], [[Math.max(noise.posSigma, 1e-4) ** 2]], [[pos]]);
  }
}
