import { add, eye, mul, mulAll, solve, sub, transpose, type Mat } from '@/math/mat';

/**
 * Linear Kalman filter with explicit matrices — small, readable, and it exposes its covariance so
 * the UI can draw ±2σ bands (docs/beyond-pid.md §4).
 */
export class KalmanFilter {
  x: Mat;
  p: Mat;
  /** Normalised innovation squared of the last scalar update, νᵀS⁻¹ν (NaN before any). */
  nis = NaN;

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
    if (s.length === 1) this.nis = innovation[0]![0]! ** 2 / s[0]![0]!;
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

export interface AltitudeNoise {
  accSigma: number;
  posSigma: number;
  biasSigma: number;
  /** Carry the altimeter's bias as a fourth state. */
  altBiasState?: boolean;
  /** Prior σ of that bias, m. */
  altBiasSigma?: number;
}

/**
 * Vertical position/velocity/accelerometer-bias filter for L1. The accelerometer drives the
 * prediction; position fixes correct it.
 *   state x = (y, v, b), a_true = a_meas − b − g
 * With `altBiasState` a fourth state b_y joins, and the altimeter is modelled as y + b_y. That
 * pair is unobservable (the fixes only ever show the sum), which is the point of lesson III.9.
 */
export class AltitudeKalman {
  private kf = new KalmanFilter([[0], [0], [0]], eye(3));
  private started = false;
  private dim = 3;

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
  /** Estimated altimeter bias (0 without that state). */
  get altBias(): number {
    return this.dim === 4 ? this.kf.x[3]![0]! : 0;
  }
  /** 1σ of state i: 0 altitude, 1 velocity, 2 accelerometer bias, 3 altimeter bias. */
  sigma(i: 0 | 1 | 2 | 3): number {
    return i < this.dim ? this.kf.sigma(i) : NaN;
  }

  /** Normalised innovation squared of the last altimeter fix. */
  get nis(): number {
    return this.kf.nis;
  }
  /** Number of states: 3, or 4 with the altimeter bias. */
  get states(): number {
    return this.dim;
  }

  /**
   * One filter step of length dt. `accY` is the measured specific force (vertical), `pos` the
   * position fix or null when no new fix arrived.
   */
  step(dt: number, accY: number, pos: number | null, noise: AltitudeNoise, gravity: number): void {
    const n = noise.altBiasState ? 4 : 3;
    if (n !== this.dim) {
      this.dim = n;
      this.started = false;
    }
    const pad = (row: number[]) => (n === 4 ? [...row, 0] : row);
    if (!this.started) {
      if (pos === null) return;
      const p0 = [pad([noise.posSigma ** 2 + 1e-6, 0, 0]), pad([0, 1, 0]), pad([0, 0, 0.25])];
      const x0 = [[pos], [0], [0]];
      if (n === 4) {
        // The fix is y + b_y: what is uncertain about the bias is uncertain about the altitude.
        const vb = (noise.altBiasSigma ?? 0.5) ** 2;
        p0[0]![0] = p0[0]![0]! + vb;
        p0[0]![3] = -vb;
        p0.push([-vb, 0, 0, vb]);
        x0.push([0]);
      }
      this.kf = new KalmanFilter(x0, p0);
      this.started = true;
      return;
    }
    const f = [pad([1, dt, -0.5 * dt * dt]), pad([0, 1, -dt]), pad([0, 0, 1])];
    const g = [[0.5 * dt * dt], [dt], [0]];
    // Accelerometer noise enters like an acceleration input; the bias wanders slowly.
    const qa = noise.accSigma ** 2;
    const qb = noise.biasSigma ** 2 * dt;
    const q = [
      pad([(qa * dt ** 4) / 4, (qa * dt ** 3) / 2, 0]),
      pad([(qa * dt ** 3) / 2, qa * dt * dt, 0]),
      pad([0, 0, qb]),
    ];
    if (n === 4) {
      f.push([0, 0, 0, 1]);
      g.push([0]);
      q.push([0, 0, 0, 0]);
    }
    this.kf.predict(f, q, g, [[accY - gravity]]);
    if (pos !== null)
      this.kf.update(
        [n === 4 ? [1, 0, 0, 1] : [1, 0, 0]],
        [[Math.max(noise.posSigma, 1e-4) ** 2]],
        [[pos]],
      );
  }
}
