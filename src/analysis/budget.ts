// An error budget (docs/aerospace-gnc.md, Chapter P, lesson V.1): a requirement on the altitude
// error split among its sources, each source flown alone and measured, then all of them together.
import { Simulation } from '@/engine/simulation';
import type { Params } from '@/sim/params';

/** "Hold the altitude within 10 cm, 3σ": the requirement, m, and the margin kept back. */
export const REQUIREMENT = 0.1;
export const MARGIN = 0.2;

/** Flight length and the start of the judged window, s. */
const FLIGHT_S = 60;
const JUDGE_FROM = 8;

/**
 * Mean errors add; random ones add in squares. A source is a bias if it moves the mean, random if
 * it spreads the error around it.
 */
export type SourceKind = 'bias' | 'random';

export interface BudgetSource {
  id: string;
  label: string;
  kind: SourceKind;
  /** What the systems engineer gave this source, m (|mean| + 3σ). */
  allocation: number;
  /** Is the source present in these parameters? */
  present: (p: Params) => boolean;
  /** Switch the source off. */
  remove: (p: Params) => void;
}

/** The bias and noise allocations; the wind gets what is left (the prediction of lesson V.1). */
export const BIAS_ALLOCATION = 0.035;
export const NOISE_ALLOCATION = 0.015;

/** The share the wind may take: (1 − margin)·R − bias, less the noise, in squares. */
export const windAllocation = (): number =>
  Math.sqrt((REQUIREMENT * (1 - MARGIN) - BIAS_ALLOCATION) ** 2 - NOISE_ALLOCATION ** 2);

export const SOURCES: BudgetSource[] = [
  {
    id: 'wind',
    label: 'Wind (turbulence)',
    kind: 'random',
    allocation: windAllocation(),
    present: (p) => p.wind.enabled,
    remove: (p) => (p.wind.enabled = false),
  },
  {
    id: 'noise',
    label: 'Altimeter noise',
    kind: 'random',
    allocation: NOISE_ALLOCATION,
    present: (p) => p.sensors.posNoise > 0,
    remove: (p) => (p.sensors.posNoise = 0),
  },
  {
    id: 'bias',
    label: 'Altimeter bias',
    kind: 'bias',
    allocation: BIAS_ALLOCATION,
    present: (p) => p.sensors.altBias !== 0,
    remove: (p) => (p.sensors.altBias = 0),
  },
];

/** The altitude error of one flight: its mean and standard deviation, m. */
export interface ErrorStats {
  mean: number;
  sd: number;
  /** |mean| + 3σ: what the requirement is written on. */
  total: number;
  crashed: boolean;
}

export const totalOf = (mean: number, sd: number): number => Math.abs(mean) + 3 * sd;

/** One hover flight, judged from JUDGE_FROM to FLIGHT_S: the true altitude against the setpoint. */
export class HoverFlight {
  private sim: Simulation;
  private k = 0;
  private s = 0;
  private s2 = 0;
  private n = 0;
  constructor(p: Params) {
    this.sim = new Simulation(p);
  }
  get done(): boolean {
    return this.k >= FLIGHT_S * 1000 || this.sim.state.crashed;
  }
  advance(budget: number): number {
    const n = Math.min(budget, FLIGHT_S * 1000 - this.k);
    for (let i = 0; i < n; i++) {
      this.sim.step();
      this.k++;
      if (this.sim.state.crashed) return i + 1;
      if (this.k > JUDGE_FROM * 1000 && this.k % 5 === 0) {
        const e = this.sim.state.pos.y - this.sim.setpoint.y;
        this.s += e;
        this.s2 += e * e;
        this.n++;
      }
    }
    return n;
  }
  get stats(): ErrorStats {
    const crashed = this.sim.state.crashed;
    if (crashed || this.n === 0) return { mean: NaN, sd: NaN, total: Infinity, crashed: true };
    const mean = this.s / this.n;
    const sd = Math.sqrt(Math.max(this.s2 / this.n - mean * mean, 0));
    return { mean, sd, total: totalOf(mean, sd), crashed };
  }
}

export interface EntryResult {
  source: BudgetSource;
  stats: ErrorStats;
  within: boolean;
}

export interface BudgetResult {
  entries: EntryResult[];
  /** The budget from the measured entries: Σ|mean| + 3·√Σσ². */
  predicted: number;
  /** The confirmation flight with every source at once. */
  flown: ErrorStats;
  /** Every entry within its allocation, and the confirmation flight within (1 − margin)·R. */
  closes: boolean;
}

/** The parameters with only `keep` of the sources on (all others removed). */
export function withOnly(p: Params, keep: BudgetSource | null): Params {
  const q = structuredClone(p);
  for (const s of SOURCES) if (s !== keep) s.remove(q);
  return q;
}

/** Combine measured entries as a budget does: biases add, standard deviations add in squares. */
export const combine = (stats: readonly ErrorStats[]): number =>
  stats.reduce((a, s) => a + Math.abs(s.mean), 0) +
  3 * Math.sqrt(stats.reduce((a, s) => a + s.sd * s.sd, 0));

/** The flights of a budget, one per source and one with everything, advanced in slices. */
export class BudgetRun {
  private flights: { source: BudgetSource | null; flight: HoverFlight }[];
  private index = 0;
  constructor(readonly p: Params) {
    this.flights = [
      ...SOURCES.filter((s) => s.present(p)).map((s) => ({
        source: s,
        flight: new HoverFlight(withOnly(p, s)),
      })),
      { source: null, flight: new HoverFlight(structuredClone(p)) },
    ];
  }
  get done(): boolean {
    return this.index >= this.flights.length;
  }
  get progress(): number {
    return this.index / this.flights.length;
  }
  advance(budget: number): boolean {
    let left = budget;
    while (left > 0 && !this.done) {
      const f = this.flights[this.index]!.flight;
      left -= f.advance(left);
      if (f.done) this.index++;
    }
    return this.done;
  }
  get result(): BudgetResult {
    const entries = this.flights
      .filter((f) => f.source)
      .map((f) => {
        const stats = f.flight.stats;
        return { source: f.source!, stats, within: stats.total <= f.source!.allocation };
      });
    const flown = this.flights[this.flights.length - 1]!.flight.stats;
    return {
      entries,
      predicted: combine(entries.map((e) => e.stats)),
      flown,
      closes:
        entries.every((e) => e.within) &&
        !flown.crashed &&
        flown.total <= REQUIREMENT * (1 - MARGIN),
    };
  }
}

/** What a budget depends on: the controller, the vehicle, the wind and the sensors. */
export const budgetKey = (p: Params): string =>
  JSON.stringify([p.sim.level, p.sim.seed, p.control, p.drone, p.wind, p.sensors, p.setpoint]);

const finished = new Map<string, BudgetResult>();
export const finishedBudget = (p: Params): BudgetResult | null =>
  finished.get(budgetKey(p)) ?? null;

/** Fly a whole budget at once (tests) and remember it, as the panel does. */
export function measureBudget(p: Params): BudgetResult {
  const run = new BudgetRun(p);
  while (!run.advance(1_000_000));
  storeBudget(p, run.result);
  return run.result;
}
export const storeBudget = (p: Params, r: BudgetResult): void => {
  finished.set(budgetKey(p), r);
};
