// Lesson V.4: three altimeters and a voter in front of the L1 controller. Altimeter A freezes,
// later altimeter B drifts; the voter decides which altitude the controller flies on.
import { v3 } from '@/math/vec3';
import { Rng } from '@/math/prng';
import type { Actuation } from '@/sim/dynamics';
import type { Params } from '@/sim/params';
import { periodSteps, type ControlInput, type Controller } from '../types';
import { oncePerStep } from './shadow';

export type VoterParams = Params['part5']['voter'];

const median3 = (a: number, b: number, c: number) =>
  Math.max(Math.min(a, b), Math.min(Math.max(a, b), c));

/** The three readings at time `t`, given the true altitude and each channel's own noise. */
export class Altimeters {
  readonly readings = [NaN, NaN, NaN];
  private frozen = NaN;
  private rng = new Rng(0x7a11);

  reset(seed: number): void {
    this.rng = new Rng(seed ^ 0x7a11);
    this.frozen = NaN;
    this.readings.fill(NaN);
  }

  read(truth: number, t: number, v: VoterParams): number[] {
    const noise = () => v.noise * this.rng.normal();
    let a = truth + noise();
    if (t >= v.stuckAt) {
      if (Number.isNaN(this.frozen)) this.frozen = a;
      a = this.frozen;
    }
    const b = truth + noise() + (t >= v.driftAt ? v.driftRate * (t - v.driftAt) : 0);
    const c = truth + noise();
    this.readings[0] = a;
    this.readings[1] = b;
    this.readings[2] = c;
    return this.readings;
  }
}

/**
 * Mid-value selection, and in 'monitor' mode a cross-channel monitor: a channel that stays more
 * than `threshold` from the reference for `persistMs` is isolated for good. The reference is the
 * middle of three healthy channels. With two, it cannot tell which one is wrong, so the reference
 * is the last three-channel vote carried forward by the measured vertical speed alone: a second,
 * dissimilar source.
 */
export class Voter {
  healthy = [true, true, true];
  output = NaN;
  /** Dead reckoning from the last three-channel vote: the vertical speed integrated alone. */
  coast = NaN;
  private since = [NaN, NaN, NaN];

  reset(): void {
    this.healthy = [true, true, true];
    this.since = [NaN, NaN, NaN];
    this.output = NaN;
    this.coast = NaN;
  }

  vote(r: readonly number[], vz: number, t: number, dt: number, v: VoterParams): number {
    if (v.mode === 'single') return (this.output = r[0]!);
    if (v.mode === 'mid') return (this.output = median3(r[0]!, r[1]!, r[2]!));
    const ok = [0, 1, 2].filter((i) => this.healthy[i]);
    if (ok.length === 3) this.coast = median3(r[0]!, r[1]!, r[2]!);
    else this.coast += vz * dt;
    const reference = this.coast;
    for (const i of ok) {
      if (Math.abs(r[i]! - reference) > v.threshold) {
        if (Number.isNaN(this.since[i]!)) this.since[i] = t;
        // Never isolate the last channel: a wrong altitude is better than none.
        if (t - this.since[i]! >= v.persistMs / 1000 && this.healthy.filter(Boolean).length > 1)
          this.healthy[i] = false;
      } else this.since[i] = NaN;
    }
    const left = [0, 1, 2].filter((i) => this.healthy[i]).map((i) => r[i]!);
    this.output =
      left.length === 3
        ? median3(left[0]!, left[1]!, left[2]!)
        : left.reduce((a, b) => a + b, 0) / left.length;
    return this.output;
  }
}

export class WithVoter implements Controller {
  readonly altimeters = new Altimeters();
  readonly voter = new Voter();
  private truth = NaN;

  constructor(
    readonly inner: Controller,
    seed: number,
  ) {
    this.altimeters.reset(seed);
  }

  reset(): void {
    this.inner.reset();
    this.voter.reset();
  }

  resetIntegrators(): void {
    this.inner.resetIntegrators();
  }

  tick(input: ControlInput): Actuation {
    const base = oncePerStep(input);
    const { params: p, physDt, stepIndex } = input;
    const n = periodSteps(p.control.rateHz, physDt);
    let voted = NaN;
    if (stepIndex % n === 0) {
      const m = base();
      const t = stepIndex * physDt;
      this.truth = m.pos.y;
      const r = this.altimeters.read(m.pos.y, t, p.part5.voter);
      voted = this.voter.vote(r, m.vel.y, t, n * physDt, p.part5.voter);
    }
    const sense = () => {
      const m = base();
      return { ...m, pos: v3(m.pos.x, Number.isNaN(voted) ? m.pos.y : voted, m.pos.z) };
    };
    return this.inner.tick({ ...input, sense });
  }

  loops() {
    return this.inner.loops();
  }

  describe(p: Params, loopId: string) {
    return this.inner.describe?.(p, loopId) ?? [];
  }

  extras() {
    const [a, b, c] = this.altimeters.readings;
    return {
      ...this.inner.extras(),
      'vote.a': a!,
      'vote.b': b!,
      'vote.c': c!,
      'vote.out': this.voter.output,
      'vote.truth': this.truth,
      'vote.err': Math.abs(this.voter.output - this.truth),
      'vote.healthy': this.voter.healthy.filter(Boolean).length,
    };
  }
}
