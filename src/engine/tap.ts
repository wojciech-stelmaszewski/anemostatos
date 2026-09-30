/**
 * A short record at the physics rate (1 kHz) of the few signals whose spectrum matters up to
 * 500 Hz. The ordinary telemetry is sampled at 200 Hz and would alias them.
 */
export const TAP_CHANNELS = ['gyro.true', 'gyro.sensor', 'gyro.used', 'motor.cmd'] as const;
export type TapChannel = (typeof TAP_CHANNELS)[number];

/** 8.192 s at 1 kHz. */
const SIZE = 8192;

export class Tap {
  private data = TAP_CHANNELS.map(() => new Float32Array(SIZE));
  private head = 0;
  private filled = 0;

  reset(): void {
    this.head = 0;
    this.filled = 0;
  }

  /** One sample per channel, in the order of TAP_CHANNELS. */
  push(a: number, b: number, c: number, d: number): void {
    const h = this.head;
    this.data[0]![h] = a;
    this.data[1]![h] = b;
    this.data[2]![h] = c;
    this.data[3]![h] = d;
    this.head = (h + 1) % SIZE;
    if (this.filled < SIZE) this.filled++;
  }

  /** How many samples are stored. */
  get length(): number {
    return this.filled;
  }

  /** The last `n` samples of a channel, oldest first (fewer if less is stored). */
  last(channel: TapChannel, n: number): Float64Array {
    const m = Math.min(n, this.filled);
    const src = this.data[TAP_CHANNELS.indexOf(channel)]!;
    const out = new Float64Array(m);
    for (let i = 0; i < m; i++) out[i] = src[(this.head - m + i + SIZE) % SIZE]!;
    return out;
  }
}
