import { Rng } from '@/math/prng';
import { add, normalize, scale, v3, type Vec3 } from '@/math/vec3';
import type { WindParams } from './params';

export interface Gust {
  start: number;
  duration: number;
  amplitude: number;
  direction: Vec3;
}

/**
 * Spatially uniform wind: mean + Ornstein–Uhlenbeck turbulence + discrete 1−cos gusts
 * (docs/physics-model.md §5). Turbulence and gusts draw from separate seeded streams,
 * so toggling one does not change the other's sequence.
 */
export class Wind {
  turbulence = v3();
  gustVelocity = v3();
  velocity = v3();
  gusts: Gust[] = [];
  /** Start times of all gusts so far (chart markers). */
  gustLog: number[] = [];
  private rngTurb: Rng;
  private rngGust: Rng;
  private nextGustAt = Infinity;

  constructor(seed: number) {
    this.rngTurb = new Rng(seed);
    this.rngGust = new Rng(seed ^ 0x5bd1e995);
  }

  step(t: number, dt: number, p: WindParams): Vec3 {
    if (!p.enabled) {
      this.velocity = v3();
      this.gustVelocity = v3();
      return this.velocity;
    }

    // Exact discretisation of the OU process, per axis.
    const tau = Math.max(p.turbTau, 1e-3);
    const decay = Math.exp(-dt / tau);
    const k = p.turbSigma * Math.sqrt(1 - decay * decay);
    this.turbulence = v3(
      this.turbulence.x * decay + k * this.rngTurb.normal(),
      this.turbulence.y * decay + k * this.rngTurb.normal(),
      this.turbulence.z * decay + k * this.rngTurb.normal(),
    );

    // Poisson gust arrivals.
    if (p.gustsOn && p.gustsPerMinute > 0) {
      if (!Number.isFinite(this.nextGustAt)) {
        this.nextGustAt = t + this.rngGust.exponential(p.gustsPerMinute / 60);
      }
      while (t >= this.nextGustAt) {
        this.spawnRandomGust(this.nextGustAt, p);
        this.nextGustAt += this.rngGust.exponential(p.gustsPerMinute / 60);
      }
    } else {
      this.nextGustAt = Infinity;
    }

    let gust = v3();
    this.gusts = this.gusts.filter((g) => t <= g.start + g.duration);
    for (const g of this.gusts) {
      const phase = (t - g.start) / g.duration;
      if (phase < 0) continue;
      gust = add(gust, scale(g.direction, (g.amplitude / 2) * (1 - Math.cos(2 * Math.PI * phase))));
    }
    this.gustVelocity = gust;

    const h = (p.meanHeading * Math.PI) / 180;
    const mean = v3(p.meanSpeed * Math.cos(h), p.meanVertical, p.meanSpeed * Math.sin(h));
    this.velocity = add(add(mean, this.turbulence), gust);
    return this.velocity;
  }

  trigger(t: number, direction: Vec3, amplitude: number, duration: number): void {
    this.gusts.push({ start: t, duration, amplitude, direction: normalize(direction) });
    this.gustLog.push(t);
  }

  private spawnRandomGust(t: number, p: WindParams): void {
    const r = this.rngGust;
    const heading = r.uniform(0, 2 * Math.PI);
    const vy = Math.min(1, Math.max(0, p.gustVertical)) * (r.next() < 0.5 ? -1 : 1);
    const hs = Math.sqrt(1 - vy * vy);
    const dir = v3(Math.cos(heading) * hs, vy, Math.sin(heading) * hs);
    this.trigger(
      t,
      dir,
      r.uniform(p.gustAmpMin, p.gustAmpMax),
      r.uniform(p.gustDurMin, p.gustDurMax),
    );
  }
}
