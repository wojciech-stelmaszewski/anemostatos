// Monte Carlo verification of the quadrotor (docs/analysis.md §4, Chapter J): one scenario flown
// many times with its uncertain parameters drawn from their dispersions, and the result stated as
// a pass rate with a confidence bound.
import { Simulation } from '@/engine/simulation';
import { Rng } from '@/math/prng';
import type { Params } from '@/sim/params';

/** One uncertain parameter and the range it is drawn from (uniformly, or log-uniformly). */
export interface Dispersion {
  id: string;
  label: string;
  unit: string;
  lo: number;
  hi: number;
  log?: boolean;
  /** Put a drawn value into the parameters. */
  apply: (p: Params, v: number) => void;
}

/** Mass ±20 %, motor lag 15–60 ms, sensor delay 0–50 ms, gyro bias ±2 °/s, and the wind's seed. */
export const DISPERSIONS: Dispersion[] = [
  {
    id: 'mass',
    label: 'mass',
    unit: 'kg',
    lo: 0.8,
    hi: 1.2,
    apply: (p, v) => (p.drone.mass = v),
  },
  {
    id: 'tau',
    label: 'motor lag',
    unit: 'ms',
    lo: 15,
    hi: 60,
    log: true,
    apply: (p, v) => (p.drone.motorTau = v / 1000),
  },
  {
    id: 'delay',
    label: 'sensor delay',
    unit: 'ms',
    lo: 0,
    hi: 50,
    apply: (p, v) => (p.sensors.delayMs = v),
  },
  {
    id: 'bias',
    label: 'gyro bias',
    unit: '°/s',
    lo: -2,
    hi: 2,
    apply: (p, v) => (p.sensors.gyroBias = { x: v, y: 0, z: -v }),
  },
  {
    id: 'seed',
    label: 'wind seed',
    unit: '',
    lo: 0,
    hi: 1e6,
    apply: (p, v) => (p.sim.seed = Math.floor(v)),
  },
];

/** Seconds per flight, and the last seconds in which it is judged. */
export const FLIGHT_S = 20;
const JUDGE_S = 5;
/**
 * A body rate above this in the judged seconds is an oscillation, not a gust, deg/s. In the
 * campaign of lesson III.29 gusts reach about 140 °/s and every unstable loop passes 500 °/s.
 */
export const RATE_LIMIT_DEG = 300;

export interface Run {
  /** The drawn values, in the order of DISPERSIONS. */
  values: number[];
  /** Largest body rate (roll and pitch) in the judged seconds, deg/s; Infinity after a crash. */
  peakRate: number;
  crashed: boolean;
  passed: boolean;
}

/** Draw the `i`-th set of values: the same sequence for every tune (common random numbers). */
function drawAll(n: number, seed: number): number[][] {
  const r = new Rng(seed ^ 0x2545f491);
  return Array.from({ length: n }, () =>
    DISPERSIONS.map((d) => {
      const u = r.next();
      return d.log ? d.lo * (d.hi / d.lo) ** u : d.lo + (d.hi - d.lo) * u;
    }),
  );
}

/** The parameters of one member of the campaign. */
export function member(p: Params, values: readonly number[]): Params {
  const q = structuredClone(p);
  DISPERSIONS.forEach((d, i) => d.apply(q, values[i]!));
  return q;
}

/** Fly one member and judge it. */
class Flight {
  private sim: Simulation;
  private k = 0;
  private peak = 0;
  constructor(
    p: Params,
    readonly values: number[],
  ) {
    this.sim = new Simulation(member(p, values));
  }
  advance(budget: number): number {
    const total = FLIGHT_S * 1000;
    const n = Math.min(budget, total - this.k);
    const s = this.sim.state;
    for (let i = 0; i < n; i++) {
      this.sim.step();
      this.k++;
      if (s.crashed || this.sim.state.crashed) {
        this.k = total;
        this.peak = Infinity;
        return i + 1;
      }
      if (this.k > total - JUDGE_S * 1000) {
        const w = this.sim.state.omega;
        this.peak = Math.max(this.peak, (Math.hypot(w.x, w.z) * 180) / Math.PI);
      }
    }
    return n;
  }
  get done(): boolean {
    return this.k >= FLIGHT_S * 1000;
  }
  get run(): Run {
    const crashed = this.sim.state.crashed;
    return {
      values: this.values,
      peakRate: this.peak,
      crashed,
      passed: !crashed && this.peak <= RATE_LIMIT_DEG,
    };
  }
}

/** A campaign of `n` flights, advanced in slices so that the interface stays live. */
export class DispersionCampaign {
  readonly runs: Run[] = [];
  private draws: number[][];
  private flight: Flight | null = null;

  constructor(
    private p: Params,
    readonly n = 300,
    seed = 7,
  ) {
    this.draws = drawAll(n, seed);
  }
  get done(): boolean {
    return this.runs.length >= this.n;
  }
  get failures(): number {
    return this.runs.filter((r) => !r.passed).length;
  }
  advance(budget: number): boolean {
    let left = budget;
    while (left > 0 && !this.done) {
      this.flight ??= new Flight(this.p, this.draws[this.runs.length]!);
      left -= this.flight.advance(left);
      if (this.flight.done) {
        this.runs.push(this.flight.run);
        this.flight = null;
      }
    }
    return this.done;
  }
}

/** What a tune needs for its campaign to be the same: the controller and the vehicle. */
export const campaignKey = (p: Params): string =>
  JSON.stringify([p.sim.level, p.control, p.drone, p.wind, p.setpoint]);

/** Finished campaigns by `campaignKey`: the interface fills it, lesson goals read it. */
const finished = new Map<string, Run[]>();
export const finishedCampaign = (p: Params): Run[] | null => finished.get(campaignKey(p)) ?? null;
export const storeCampaign = (p: Params, runs: Run[]): void => {
  finished.set(campaignKey(p), runs);
};

/** Fly a whole campaign at once (tests, `make bench`) and remember it. */
export function runDispersionCampaign(p: Params, n = 300, seed = 7): Run[] {
  const c = new DispersionCampaign(p, n, seed);
  while (!c.advance(10_000_000));
  if (n === 300 && seed === 7) storeCampaign(p, c.runs);
  return c.runs;
}

// ---------------------------------------------------------------------------------------------
// What a number of clean runs proves

/** ln of the binomial coefficient, by sums of logarithms (n is a few hundred at most). */
function lnChoose(n: number, k: number): number {
  let s = 0;
  for (let i = 1; i <= k; i++) s += Math.log((n - k + i) / i);
  return s;
}

/** Probability of at least `s` successes in `n` trials with success probability `p`. */
function atLeast(s: number, n: number, p: number): number {
  if (p <= 0) return s <= 0 ? 1 : 0;
  if (p >= 1) return 1;
  let sum = 0;
  for (let i = s; i <= n; i++)
    sum += Math.exp(lnChoose(n, i) + i * Math.log(p) + (n - i) * Math.log(1 - p));
  return Math.min(sum, 1);
}

/**
 * Lower confidence bound on the pass probability after `passed` of `n` runs (Clopper–Pearson,
 * one-sided): the smallest p that would still give this many passes or more with probability
 * 1 − confidence. With no failure it is (1 − confidence)^(1/n).
 */
export function passRateBound(passed: number, n: number, confidence = 0.95): number {
  if (n <= 0 || passed <= 0) return 0;
  const alpha = 1 - confidence;
  if (passed === n) return alpha ** (1 / n);
  let lo = 0;
  let hi = 1;
  for (let i = 0; i < 60; i++) {
    const mid = (lo + hi) / 2;
    if (atLeast(passed, n, mid) < alpha) lo = mid;
    else hi = mid;
  }
  return lo;
}

/** Clean runs needed, with no failure at all, to show a pass rate of `rate` at `confidence`. */
export const cleanRunsNeeded = (rate: number, confidence = 0.95): number =>
  Math.ceil(Math.log(1 - confidence) / Math.log(rate));
