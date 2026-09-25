/**
 * Fixed-size ring buffers of named scalar signals, sampled at the telemetry rate.
 * Charts read from here at display rate; they never touch the simulation directly.
 */
export class Telemetry {
  readonly capacity: number;
  readonly time: Float64Array;
  private channels = new Map<string, Float64Array>();
  private head = 0;
  private count = 0;
  /** Incremented on every commit; lets readers skip redraws when nothing changed. */
  version = 0;

  constructor(capacity: number) {
    this.capacity = capacity;
    this.time = new Float64Array(capacity);
  }

  /** Write a value into the row being built. Unknown channels are created on the fly. */
  set(name: string, value: number): void {
    let ch = this.channels.get(name);
    if (!ch) {
      ch = new Float64Array(this.capacity).fill(NaN);
      this.channels.set(name, ch);
    }
    ch[this.head] = value;
  }

  commit(t: number): void {
    this.time[this.head] = t;
    this.head = (this.head + 1) % this.capacity;
    this.count = Math.min(this.count + 1, this.capacity);
    // Pre-clear the next slot so channels not written in the next row read as gaps.
    for (const ch of this.channels.values()) ch[this.head] = NaN;
    this.version++;
  }

  clear(): void {
    this.head = 0;
    this.count = 0;
    this.channels.clear();
    this.version++;
  }

  has(name: string): boolean {
    return this.channels.has(name);
  }

  get length(): number {
    return this.count;
  }

  get lastTime(): number {
    return this.count ? this.time[(this.head - 1 + this.capacity) % this.capacity]! : 0;
  }

  latest(name: string): number {
    const ch = this.channels.get(name);
    return ch && this.count ? ch[(this.head - 1 + this.capacity) % this.capacity]! : NaN;
  }

  /**
   * Copy the samples newer than `since` (oldest first) into plain arrays for charting.
   * Missing channels come back as all-NaN.
   */
  window(names: readonly string[], since: number): { t: number[]; series: number[][] } {
    const start = (this.head - this.count + this.capacity) % this.capacity;
    // Binary search for the first sample with time >= since.
    let lo = 0;
    let hi = this.count;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (this.time[(start + mid) % this.capacity]! < since) lo = mid + 1;
      else hi = mid;
    }
    const n = this.count - lo;
    const t = new Array<number>(n);
    const series = names.map(() => new Array<number>(n));
    const chans = names.map((name) => this.channels.get(name));
    for (let k = 0; k < n; k++) {
      const idx = (start + lo + k) % this.capacity;
      t[k] = this.time[idx]!;
      for (let c = 0; c < names.length; c++) {
        const ch = chans[c];
        series[c]![k] = ch ? ch[idx]! : NaN;
      }
    }
    return { t, series };
  }

  /** CSV of all channels over the whole buffer. */
  toCsv(): string {
    const names = [...this.channels.keys()].sort();
    const { t, series } = this.window(names, -Infinity);
    const rows = [['t', ...names].join(',')];
    for (let k = 0; k < t.length; k++) {
      rows.push(
        [
          t[k]!.toFixed(4),
          ...series.map((s) => (Number.isNaN(s[k]) ? '' : s[k]!.toPrecision(6))),
        ].join(','),
      );
    }
    return rows.join('\n');
  }
}
