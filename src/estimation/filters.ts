/**
 * Discrete low-pass filters with explicit state. INDI (docs/beyond-pid.md §4) needs two signals
 * filtered by *identical* filters, so these are small classes that can be instantiated in pairs.
 */

/** First-order low-pass, y⁺ = α·y + (1 − α)·x with α from the cutoff. */
export class LowPass1 {
  private y = 0;
  private init = false;
  constructor(
    public cutoffHz: number,
    public dt: number,
  ) {}

  reset(value?: number): void {
    this.init = value !== undefined;
    this.y = value ?? 0;
  }

  update(x: number): number {
    if (!this.init) {
      this.y = x;
      this.init = true;
      return x;
    }
    const tau = 1 / (2 * Math.PI * this.cutoffHz);
    const a = tau / (tau + this.dt);
    this.y = a * this.y + (1 - a) * x;
    return this.y;
  }

  get value(): number {
    return this.y;
  }
}

/**
 * Second-order Butterworth low-pass (biquad, bilinear transform with pre-warping). Starts in
 * steady state on the first sample, so there is no start-up transient.
 */
export class LowPass2 {
  private b0 = 0;
  private b1 = 0;
  private b2 = 0;
  private a1 = 0;
  private a2 = 0;
  private x1 = 0;
  private x2 = 0;
  private y1 = 0;
  private y2 = 0;
  private init = false;
  private designed = { hz: NaN, dt: NaN };

  constructor(
    public cutoffHz: number,
    public dt: number,
  ) {}

  private design(): void {
    if (this.designed.hz === this.cutoffHz && this.designed.dt === this.dt) return;
    const fs = 1 / this.dt;
    const fc = Math.min(this.cutoffHz, 0.45 * fs);
    const k = Math.tan((Math.PI * fc) / fs);
    const q = Math.SQRT1_2;
    const norm = 1 / (1 + k / q + k * k);
    this.b0 = k * k * norm;
    this.b1 = 2 * this.b0;
    this.b2 = this.b0;
    this.a1 = 2 * (k * k - 1) * norm;
    this.a2 = (1 - k / q + k * k) * norm;
    this.designed = { hz: this.cutoffHz, dt: this.dt };
  }

  reset(value?: number): void {
    this.init = false;
    if (value !== undefined) this.prime(value);
  }

  private prime(v: number): void {
    this.x1 = this.x2 = this.y1 = this.y2 = v;
    this.init = true;
  }

  update(x: number): number {
    this.design();
    if (!this.init) this.prime(x);
    const y =
      this.b0 * x + this.b1 * this.x1 + this.b2 * this.x2 - this.a1 * this.y1 - this.a2 * this.y2;
    this.x2 = this.x1;
    this.x1 = x;
    this.y2 = this.y1;
    this.y1 = y;
    return y;
  }

  get value(): number {
    return this.y1;
  }
}

/** Backward-difference derivative of a signal (the caller filters it, usually). */
export class Differentiator {
  private prev = 0;
  private has = false;
  constructor(public dt: number) {}

  reset(): void {
    this.has = false;
  }

  update(x: number): number {
    const d = this.has ? (x - this.prev) / this.dt : 0;
    this.prev = x;
    this.has = true;
    return d;
  }
}

/** Coefficients of a second-order section, y = b0·x + b1·x₁ + b2·x₂ − a1·y₁ − a2·y₂. */
export interface BiquadCoefs {
  b0: number;
  b1: number;
  b2: number;
  a1: number;
  a2: number;
}

/** Butterworth low-pass at `hz` for a sample time `dt` (bilinear transform, pre-warped). */
export function lowpassCoefs(hz: number, dt: number): BiquadCoefs {
  const k = Math.tan(Math.PI * Math.min(hz * dt, 0.45));
  const q = Math.SQRT1_2;
  const norm = 1 / (1 + k / q + k * k);
  const b0 = k * k * norm;
  return { b0, b1: 2 * b0, b2: b0, a1: 2 * (k * k - 1) * norm, a2: (1 - k / q + k * k) * norm };
}

/**
 * Notch at `hz`: no gain at the centre, unity far from it. `q` is the centre frequency over the
 * −3 dB width, so a larger q is a narrower notch, which costs less phase below it.
 */
export function notchCoefs(hz: number, q: number, dt: number): BiquadCoefs {
  const w0 = 2 * Math.PI * Math.min(Math.max(hz * dt, 1e-4), 0.49);
  const alpha = Math.sin(w0) / (2 * Math.max(q, 0.1));
  const a0 = 1 + alpha;
  const c = -2 * Math.cos(w0);
  return { b0: 1 / a0, b1: c / a0, b2: 1 / a0, a1: c / a0, a2: (1 - alpha) / a0 };
}

/** Frequency response of a section at the angle ω·dt, as (re, im). */
export function biquadAt(c: BiquadCoefs, wdt: number): { re: number; im: number } {
  const c1 = Math.cos(wdt);
  const s1 = -Math.sin(wdt);
  const c2 = Math.cos(2 * wdt);
  const s2 = -Math.sin(2 * wdt);
  const nr = c.b0 + c.b1 * c1 + c.b2 * c2;
  const ni = c.b1 * s1 + c.b2 * s2;
  const dr = 1 + c.a1 * c1 + c.a2 * c2;
  const di = c.a1 * s1 + c.a2 * s2;
  const d = dr * dr + di * di;
  return { re: (nr * dr + ni * di) / d, im: (ni * dr - nr * di) / d };
}

/** A second-order section whose coefficients may change while it runs (a tracking notch). */
export class Biquad {
  private x1 = 0;
  private x2 = 0;
  private y1 = 0;
  private y2 = 0;
  private init = false;

  reset(): void {
    this.init = false;
  }

  update(c: BiquadCoefs, x: number): number {
    if (!this.init) {
      // Start at rest on the first sample: no start-up transient.
      const dc = (c.b0 + c.b1 + c.b2) / (1 + c.a1 + c.a2);
      this.x1 = this.x2 = x;
      this.y1 = this.y2 = dc * x;
      this.init = true;
    }
    const y = c.b0 * x + c.b1 * this.x1 + c.b2 * this.x2 - c.a1 * this.y1 - c.a2 * this.y2;
    this.x2 = this.x1;
    this.x1 = x;
    this.y2 = this.y1;
    this.y1 = y;
    return y;
  }
}
