import { add, mul, solve, sub, transpose, type Mat } from '@/math/mat';

/**
 * Range-only position estimation (docs/aerospace-gnc.md §4, Chapter O, lesson IV.27), by an EKF
 * and by an unscented Kalman filter [Julier 2004].
 *
 * State (x, z, vx, vz) in the horizontal plane, a constant-velocity model driven by white
 * acceleration noise. Each beacon reports its distance to the drone. The distance is a curved
 * function of the position: the EKF replaces it by its tangent at the estimate, which is a good
 * description only near the estimate. The UKF pushes 2n + 1 sigma points through the true
 * function instead and recovers the mean and the spread of the prediction to second order.
 */
export interface Beacon {
  x: number;
  z: number;
}

const eye = (n: number): Mat =>
  Array.from({ length: n }, (_, i) => Array.from({ length: n }, (__, j) => (i === j ? 1 : 0)));

/** Lower Cholesky factor of a symmetric positive (semi)definite matrix. */
function cholesky(a: Mat): Mat {
  const n = a.length;
  const l: Mat = Array.from({ length: n }, () => new Array<number>(n).fill(0));
  for (let i = 0; i < n; i++)
    for (let j = 0; j <= i; j++) {
      let s = a[i]![j]!;
      for (let k = 0; k < j; k++) s -= l[i]![k]! * l[j]![k]!;
      l[i]![j] = i === j ? Math.sqrt(Math.max(s, 1e-12)) : s / l[j]![j]!;
    }
  return l;
}

const range = (x: number[], b: Beacon) => Math.hypot(x[0]! - b.x, x[1]! - b.z);

export interface RangeUpdate {
  /** Normalised innovation squared of the update (all beacons at once). */
  nis: number;
}

export class RangeFilter {
  x: number[] = [0, 0, 0, 0];
  p: Mat = eye(4);
  lastNis = 0;

  constructor(readonly kind: 'ekf' | 'ukf') {}

  reset(x: number[], sigmaPos: number, sigmaVel: number): void {
    this.x = [...x];
    this.p = eye(4).map((row, i) => row.map((v) => v * (i < 2 ? sigmaPos ** 2 : sigmaVel ** 2)));
    this.lastNis = 0;
  }

  /** 1σ of the position error the filter believes in, m (both axes together). */
  get sigmaPos(): number {
    return Math.sqrt(Math.max(this.p[0]![0]! + this.p[1]![1]!, 0));
  }

  /** Constant velocity over dt with acceleration noise σ_a (exact for this linear step). */
  predict(dt: number, accSigma: number): void {
    const f = eye(4);
    f[0]![2] = dt;
    f[1]![3] = dt;
    this.x = f.map((row) => row.reduce((s, v, j) => s + v * this.x[j]!, 0));
    const q2 = accSigma ** 2;
    const a = (q2 * dt ** 4) / 4;
    const b = (q2 * dt ** 3) / 2;
    const c = q2 * dt * dt;
    const q: Mat = [
      [a, 0, b, 0],
      [0, a, 0, b],
      [b, 0, c, 0],
      [0, b, 0, c],
    ];
    this.p = add(mul(mul(f, this.p), transpose(f)), q);
  }

  /** Correct with the measured ranges to the beacons, each with noise σ. */
  update(z: number[], beacons: Beacon[], sigma: number): RangeUpdate {
    const m = beacons.length;
    const r = eye(m).map((row) => row.map((v) => v * sigma * sigma));
    let zHat: number[];
    let s: Mat;
    let pxz: Mat; // cross-covariance, 4 × m
    if (this.kind === 'ekf') {
      zHat = beacons.map((b) => range(this.x, b));
      const h: Mat = beacons.map((b, i) => {
        const d = Math.max(zHat[i]!, 1e-6);
        return [(this.x[0]! - b.x) / d, (this.x[1]! - b.z) / d, 0, 0];
      });
      s = add(mul(mul(h, this.p), transpose(h)), r);
      pxz = mul(this.p, transpose(h));
    } else {
      // Sigma points, the standard scaled set with α = 1, β = 2, κ = 0: λ = 0.
      const n = 4;
      const lam = 0;
      const sq = cholesky(this.p.map((row) => row.map((v) => v * (n + lam))));
      const pts: number[][] = [this.x];
      for (let k = 0; k < n; k++) {
        const col = sq.map((row) => row[k]!);
        pts.push(this.x.map((v, i) => v + col[i]!));
        pts.push(this.x.map((v, i) => v - col[i]!));
      }
      const wm = [lam / (n + lam), ...new Array<number>(2 * n).fill(1 / (2 * (n + lam)))];
      const wc = [
        lam / (n + lam) + (1 - 1 + 2),
        ...new Array<number>(2 * n).fill(1 / (2 * (n + lam))),
      ];
      const zs = pts.map((x) => beacons.map((b) => range(x, b)));
      zHat = beacons.map((_, i) => zs.reduce((acc, zz, k) => acc + wm[k]! * zz[i]!, 0));
      s = r.map((row) => [...row]);
      pxz = Array.from({ length: n }, () => new Array<number>(m).fill(0));
      pts.forEach((x, k) => {
        const dz = zs[k]!.map((v, i) => v - zHat[i]!);
        const dx = x.map((v, i) => v - this.x[i]!);
        for (let i = 0; i < m; i++)
          for (let j = 0; j < m; j++) s[i]![j] = s[i]![j]! + wc[k]! * dz[i]! * dz[j]!;
        for (let i = 0; i < n; i++)
          for (let j = 0; j < m; j++) pxz[i]![j] = pxz[i]![j]! + wc[k]! * dx[i]! * dz[j]!;
      });
    }
    const nu = z.map((v, i) => v - zHat[i]!);
    const sInvNu = solve(
      s,
      nu.map((v) => [v]),
    );
    const nis = nu.reduce((acc, v, i) => acc + v * sInvNu[i]![0]!, 0);
    // K = Pxz·S⁻¹; P⁻ = P − K·S·Kᵀ.
    const k = transpose(solve(s, transpose(pxz)));
    this.x = this.x.map((v, i) => v + k[i]!.reduce((acc, kv, j) => acc + kv * nu[j]!, 0));
    this.p = sub(this.p, mul(mul(k, s), transpose(k)));
    this.p = this.p.map((row, i) => row.map((v, j) => 0.5 * (v + this.p[j]![i]!)));
    this.lastNis = nis;
    return { nis };
  }
}
