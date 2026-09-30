import { Biquad, lowpassCoefs, notchCoefs, type BiquadCoefs } from '@/estimation/filters';
import { v3, type Vec3 } from '@/math/vec3';
import { GRAVITY, type Params } from '@/sim/params';
import { rotorHz } from '@/sim/vibration';

type Motors = readonly [number, number, number, number];

/**
 * The sections the gyro passes through before the rate loop, for these settings: an optional
 * low-pass, then either one notch at a fixed frequency or one per motor at its rotor frequency
 * (the "RPM filter": docs/analysis.md §4, Chapter I). `motors` are rotor speeds as thrust, N.
 */
export function gyroFilterSections(p: Params, dt: number, motors: Motors | null): BiquadCoefs[] {
  const g = p.control.gyroFilter;
  const out: BiquadCoefs[] = [];
  if (g.lpfHz > 0) out.push(lowpassCoefs(g.lpfHz, dt));
  if (g.notch === 'fixed') out.push(notchCoefs(g.notchHz, g.notchQ, dt));
  if (g.notch === 'rpm') {
    const hover = (p.drone.mass * GRAVITY) / 4;
    for (let i = 0; i < 4; i++) {
      const f = rotorHz(motors ? motors[i]! : hover, p.drone, p.vibration.hoverHz);
      // A stopped rotor has nothing to remove; a notch near 0 Hz would remove the signal.
      if (f > 10) out.push(notchCoefs(f, g.notchQ, dt));
    }
  }
  return out;
}

/** The gyro filter's state: up to five sections on each of the three axes. */
export class GyroFilter {
  private bank = Array.from({ length: 3 }, () => Array.from({ length: 5 }, () => new Biquad()));
  private count = -1;

  reset(): void {
    for (const axis of this.bank) for (const b of axis) b.reset();
    this.count = -1;
  }

  update(omega: Vec3, p: Params, dt: number, motors: Motors | null): Vec3 {
    const sections = gyroFilterSections(p, dt, motors);
    if (sections.length === 0) return omega;
    // A change in the number of sections (a rotor starting, a setting changed) starts afresh.
    if (sections.length !== this.count) {
      this.reset();
      this.count = sections.length;
    }
    const run = (axis: number, x: number) =>
      sections.reduce((v, c, i) => this.bank[axis]![i]!.update(c, v), x);
    return v3(run(0, omega.x), run(1, omega.y), run(2, omega.z));
  }
}
