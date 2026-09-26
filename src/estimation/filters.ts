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
