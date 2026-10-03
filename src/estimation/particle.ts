import { Rng } from '@/math/prng';
import type { Beacon } from './ukf';

/**
 * A particle filter on the range-only beacon problem (docs/aerospace-gnc.md §4, Chapter O, lesson
 * IV.28) [Arulampalam 2002].
 *
 * The belief is a cloud of N points in the horizontal plane, each a guess of where the drone is.
 * The drone knows its own velocity (an optical-flow sensor, a little noisy), not its position: each
 * step moves every particle by that velocity plus a little spread, and each range reweights them by
 * how well they explain it. When most of the weight sits on a few particles, the cloud is redrawn
 * from the weights (systematic resampling) and roughened, so it does not collapse onto copies.
 *
 * One range leaves a ring of equally good guesses, and the cloud keeps the ring: it does not pick a
 * point, as the EKF does. A straight leg leaves two, mirrored about the line through the beacon
 * along the motion; only a turn tells them apart.
 */
export class ParticleFilter {
  readonly kind = 'pf' as const;
  /** Particle positions and weights. */
  px: Float64Array;
  pz: Float64Array;
  w: Float64Array;
  /** Mean position and the velocity it moved with last (x, z, vx, vz), as the Kalman filters. */
  x: number[] = [0, 0, 0, 0];
  /** Covariance of the cloud's position, 2 × 2 (x, z). */
  cov = [
    [1, 0],
    [0, 1],
  ];
  lastNis = 0;
  /** Effective number of particles after the last update. */
  ess = 0;
  private rng: Rng;

  constructor(
    readonly n = 1000,
    seed = 1,
    /** Each particle stands for a patch of this size: it widens the likelihood of a range, m. */
    readonly patch = 0.1,
    /** Random spread added to every particle per second of motion, m/√s. */
    readonly diffusion = 0.1,
  ) {
    this.px = new Float64Array(n);
    this.pz = new Float64Array(n);
    this.w = new Float64Array(n).fill(1 / n);
    this.rng = new Rng(seed ^ 0x51ed27);
  }

  /** Spread the particles uniformly over a square of half-width `half` around (x, z). */
  reset(x: number[], half: number): void {
    const r = this.rng;
    for (let i = 0; i < this.n; i++) {
      this.px[i] = x[0]! + r.uniform(-half, half);
      this.pz[i] = x[1]! + r.uniform(-half, half);
    }
    this.w.fill(1 / this.n);
    this.lastNis = 0;
    this.summarise(0, 0);
  }

  /** 1σ of the cloud's position, m (both axes together, as the Kalman filters report it). */
  get sigmaPos(): number {
    return Math.sqrt(Math.max(this.cov[0]![0]! + this.cov[1]![1]!, 0));
  }

  /** Move every particle by the measured velocity over dt, plus its own small random step. */
  predict(dt: number, vx: number, vz: number): void {
    const s = this.diffusion * Math.sqrt(dt);
    const r = this.rng;
    for (let i = 0; i < this.n; i++) {
      this.px[i] = this.px[i]! + vx * dt + s * r.normal();
      this.pz[i] = this.pz[i]! + vz * dt + s * r.normal();
    }
    this.summarise(vx, vz);
  }

  /** Reweight by the measured ranges (noise σ each), resample when the weight is concentrated. */
  update(z: number[], beacons: Beacon[], sigma: number): { nis: number } {
    const s2 = sigma * sigma + this.patch * this.patch;
    // NIS of the first range against the cloud's predicted range and its spread.
    const b0 = beacons[0]!;
    let mean = 0;
    let sq = 0;
    for (let i = 0; i < this.n; i++) {
      const d = Math.hypot(this.px[i]! - b0.x, this.pz[i]! - b0.z);
      mean += this.w[i]! * d;
      sq += this.w[i]! * d * d;
    }
    const spread = Math.max(sq - mean * mean, 0);
    this.lastNis = (z[0]! - mean) ** 2 / (spread + sigma * sigma);
    // Squared misfit of every particle, and its prior log-weight.
    const miss = new Float64Array(this.n);
    const prior = new Float64Array(this.n);
    for (let i = 0; i < this.n; i++) {
      prior[i] = Math.log(Math.max(this.w[i]!, 1e-300));
      let m = 0;
      beacons.forEach((b, k) => {
        m += (z[k]! - Math.hypot(this.px[i]! - b.x, this.pz[i]! - b.z)) ** 2;
      });
      miss[i] = m;
    }
    // Progressive correction: when a range would leave only a handful of particles alive (the
    // first one, against a cloud spread over a square), widen it until a tenth of them survive;
    // the following ranges sharpen the ring again.
    const w = new Float64Array(this.n);
    let width = s2;
    for (let tries = 0; ; tries++) {
      let top = -Infinity;
      for (let i = 0; i < this.n; i++) top = Math.max(top, prior[i]! - (0.5 * miss[i]!) / width);
      let sum = 0;
      for (let i = 0; i < this.n; i++) {
        w[i] = Math.exp(prior[i]! - (0.5 * miss[i]!) / width - top);
        sum += w[i]!;
      }
      let sumSq = 0;
      for (let i = 0; i < this.n; i++) {
        w[i] = w[i]! / sum;
        sumSq += w[i]! ** 2;
      }
      this.ess = 1 / sumSq;
      if (this.ess >= this.n / 10 || tries >= 30) break;
      width *= 2;
    }
    this.w.set(w);
    if (this.ess < this.n / 2) this.resample();
    this.summarise(this.x[2]!, this.x[3]!);
    return { nis: this.lastNis };
  }

  /** Systematic resampling, then a roughening step of the patch size. */
  private resample(): void {
    const r = this.rng;
    const nx = new Float64Array(this.n);
    const nz = new Float64Array(this.n);
    const step = 1 / this.n;
    let u = r.next() * step;
    let c = this.w[0]!;
    let j = 0;
    for (let i = 0; i < this.n; i++) {
      while (u > c && j < this.n - 1) c += this.w[++j]!;
      nx[i] = this.px[j]! + this.patch * r.normal();
      nz[i] = this.pz[j]! + this.patch * r.normal();
      u += step;
    }
    this.px = nx;
    this.pz = nz;
    this.w.fill(step);
  }

  /** Mean and covariance of the cloud. */
  private summarise(vx: number, vz: number): void {
    let mx = 0;
    let mz = 0;
    for (let i = 0; i < this.n; i++) {
      mx += this.w[i]! * this.px[i]!;
      mz += this.w[i]! * this.pz[i]!;
    }
    let cxx = 0;
    let czz = 0;
    let cxz = 0;
    for (let i = 0; i < this.n; i++) {
      const dx = this.px[i]! - mx;
      const dz = this.pz[i]! - mz;
      cxx += this.w[i]! * dx * dx;
      czz += this.w[i]! * dz * dz;
      cxz += this.w[i]! * dx * dz;
    }
    this.x = [mx, mz, vx, vz];
    this.cov = [
      [cxx, cxz],
      [cxz, czz],
    ];
  }

  /**
   * The two halves of the cloud on either side of a line through the beacon along `dir`, as
   * (weight, mean x, mean z): after a straight leg, the guess and its mirror image.
   */
  halves(beacon: Beacon, dir: { x: number; z: number }): { w: number; x: number; z: number }[] {
    const out = [
      { w: 0, x: 0, z: 0 },
      { w: 0, x: 0, z: 0 },
    ];
    for (let i = 0; i < this.n; i++) {
      // Which side: the sign of the cross product of `dir` with the beacon-to-particle vector.
      const side = dir.x * (this.pz[i]! - beacon.z) - dir.z * (this.px[i]! - beacon.x) >= 0 ? 0 : 1;
      const h = out[side]!;
      h.w += this.w[i]!;
      h.x += this.w[i]! * this.px[i]!;
      h.z += this.w[i]! * this.pz[i]!;
    }
    for (const h of out)
      if (h.w > 0) {
        h.x /= h.w;
        h.z /= h.w;
      }
    return out;
  }
}
