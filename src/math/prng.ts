/**
 * Seeded pseudo-random number generator (mulberry32).
 * All randomness in the simulation goes through one of these, so the same seed
 * reproduces the same wind and noise exactly.
 */
export class Rng {
  private state: number;
  private spareNormal: number | null = null;

  constructor(seed: number) {
    this.state = seed >>> 0;
  }

  /** Uniform in [0, 1). */
  next(): number {
    this.state = (this.state + 0x6d2b79f5) >>> 0;
    let t = this.state;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }

  uniform(min: number, max: number): number {
    return min + (max - min) * this.next();
  }

  /** Standard normal N(0, 1) via Box–Muller. */
  normal(): number {
    if (this.spareNormal !== null) {
      const s = this.spareNormal;
      this.spareNormal = null;
      return s;
    }
    let u = 0;
    while (u === 0) u = this.next();
    const v = this.next();
    const r = Math.sqrt(-2 * Math.log(u));
    this.spareNormal = r * Math.sin(2 * Math.PI * v);
    return r * Math.cos(2 * Math.PI * v);
  }

  /** Exponentially distributed waiting time for a Poisson process of the given rate. */
  exponential(rate: number): number {
    return -Math.log(1 - this.next()) / rate;
  }
}

export const randomSeed = (): number => Math.floor(Math.random() * 1_000_000);
