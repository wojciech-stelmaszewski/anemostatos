// Monte Carlo over a family of plants (docs/analysis.md §4): draw members at random, fly each,
// and count. Seeded, so a campaign is reproducible; sliced, so the interface stays live.
import { Rng } from '@/math/prng';
import type { Params } from '@/sim/params';
import { startAt } from './attraction';
import { isLoopStable, l1Loop } from './loop';
import { familyMember } from './uncertainty';

export interface Trial {
  /** Where in the family: mass, motor lag and delay, each normalised as in `familyMember`. */
  a: number;
  b: number;
  c: number;
  /** What the linear model says (null if the controller has none). */
  predicted: boolean | null;
  /** What the flight showed. */
  stable: boolean;
}

const FLIGHT_S = 10;
const JUDGE_S = 3;

/**
 * Fly one member from a 10 cm offset. It counts as stable if it stays off the floor and its
 * vertical speed has died down by the end: an unstable loop keeps oscillating at its limits.
 */
export class Flight {
  private sim;
  private k = 0;
  private late = 0;
  private failed = false;
  constructor(p: Params) {
    this.sim = startAt(p, 0.1, 0);
  }
  /** Returns the number of steps used; `done` tells whether the verdict is in. */
  advance(budget: number): number {
    const total = FLIGHT_S * 1000;
    const n = Math.min(budget, total - this.k);
    for (let i = 0; i < n; i++) {
      this.sim.step();
      this.k++;
      if (this.sim.state.landed || this.sim.state.crashed) this.failed = true;
      if (this.k > total - JUDGE_S * 1000)
        this.late = Math.max(this.late, Math.abs(this.sim.state.vel.y));
    }
    return n;
  }
  get done(): boolean {
    return this.k >= FLIGHT_S * 1000;
  }
  get stable(): boolean {
    return !this.failed && this.late < 0.08;
  }
}

export const fliesStable = (p: Params): boolean => {
  const f = new Flight(p);
  f.advance(Infinity);
  return f.stable;
};

/** `n` random members of the family of `p`, flown one after another. */
export class Campaign {
  readonly trials: Trial[] = [];
  private rng: Rng;
  private current: { a: number; b: number; c: number; q: Params; flight: Flight } | null = null;

  constructor(
    private p: Params,
    readonly n: number,
    seed = p.sim.seed,
  ) {
    this.rng = new Rng(seed ^ 0x5bd1e995);
  }

  get done(): boolean {
    return this.trials.length >= this.n;
  }
  get progress(): number {
    return this.trials.length / Math.max(this.n, 1);
  }
  get failures(): number {
    return this.trials.filter((t) => !t.stable).length;
  }

  advance(budget: number): boolean {
    let left = budget;
    while (left > 0 && !this.done) {
      if (!this.current) {
        const a = 2 * this.rng.next() - 1;
        const b = 2 * this.rng.next() - 1;
        const c = this.rng.next();
        const q = familyMember(this.p, a, b, c);
        this.current = { a, b, c, q, flight: new Flight(q) };
      }
      const cur = this.current;
      left -= cur.flight.advance(left);
      if (cur.flight.done) {
        const m = l1Loop(cur.q);
        this.trials.push({
          a: cur.a,
          b: cur.b,
          c: cur.c,
          predicted: m ? isLoopStable(m) : null,
          stable: cur.flight.stable,
        });
        this.current = null;
      }
    }
    return this.done;
  }
}

export function runCampaign(p: Params, n: number, seed?: number): Trial[] {
  const c = new Campaign(p, n, seed);
  while (!c.advance(10_000_000));
  return c.trials;
}
