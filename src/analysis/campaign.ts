// A verification campaign (docs/aerospace-gnc.md, Chapter P, lesson V.2): a requirement table, a
// dispersion table, and one Monte Carlo campaign per requirement, reported as pass rates with a
// confidence bound and the worst case found [Hanson 2010].
import { setIn } from '@/engine/schema';
import { Simulation } from '@/engine/simulation';
import { Rng } from '@/math/prng';
import type { Params } from '@/sim/params';
import { REQUIREMENT, totalOf } from './budget';
import { passRateBound, type Dispersion } from './dispersion';

/** What the altitude loop is not sure of: the drone, its motors, its altimeter and the wind. */
export const L1_DISPERSIONS: Dispersion[] = [
  { id: 'mass', label: 'mass', unit: 'kg', lo: 0.8, hi: 1.2, apply: (p, v) => (p.drone.mass = v) },
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
    hi: 20,
    apply: (p, v) => (p.sensors.delayMs = v),
  },
  {
    id: 'noise',
    label: 'altimeter noise',
    unit: 'cm',
    lo: 0.5,
    hi: 1.5,
    apply: (p, v) => (p.sensors.posNoise = v / 100),
  },
  {
    id: 'bias',
    label: 'altimeter bias',
    unit: 'cm',
    lo: -3,
    hi: 3,
    apply: (p, v) => (p.sensors.altBias = v / 100),
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

/** Watches one flight and states the number the requirement is written on. */
interface Judge {
  /** Called every 5 ms of simulated time. */
  sample(sim: Simulation): void;
  metric(): number;
}

export interface Requirement {
  id: string;
  /** The requirement as it would be written in a specification. */
  text: string;
  /** What is measured, its unit and its limit (the metric must not exceed it). */
  metric: string;
  unit: string;
  limit: number;
  /** Scale from the metric's SI value to the displayed unit. */
  scale: number;
  /** The scenario: changes to the parameters, its length and any scripted event. */
  scenario: (p: Params) => void;
  seconds: number;
  events?: (sim: Simulation) => void;
  judge: () => Judge;
}

/** The wind of requirement R1 (and of the budget of lesson V.1): turbulence, no gusts. */
export const TURBULENCE = { sigma: 1.5 };
export const turbulence = (p: Params) => {
  p.wind.enabled = true;
  p.wind.gustsOn = false;
  p.wind.turbSigma = TURBULENCE.sigma;
};

/** A 1 m step up at this time, in still air. */
const STEP = { at: 4, size: 1, band: 0.1 };
const stepScenario = (p: Params) => {
  p.wind.enabled = false;
};
const stepEvent = (sim: Simulation) => {
  const to = sim.params.setpoint.y + STEP.size;
  sim.schedule(STEP.at, (s) => s.setParams(setIn(s.params, 'setpoint.y', to)));
};

/** Overshoot of the step, % of its size. */
class Overshoot implements Judge {
  private peak = -Infinity;
  private target = NaN;
  sample(sim: Simulation): void {
    if (sim.t < STEP.at) {
      this.target = sim.params.setpoint.y + STEP.size;
      return;
    }
    if (sim.state.crashed) this.peak = Infinity;
    this.peak = Math.max(this.peak, sim.state.pos.y);
  }
  metric(): number {
    return Math.max(0, ((this.peak - this.target) / STEP.size) * 100);
  }
}

/** Time from the step until the altitude stays within ±10 cm of the target, s. */
class Settling implements Judge {
  private lastOutside = 0;
  private target = NaN;
  private crashed = false;
  sample(sim: Simulation): void {
    if (sim.t < STEP.at) {
      this.target = sim.params.setpoint.y + STEP.size;
      return;
    }
    if (sim.state.crashed) this.crashed = true;
    if (Math.abs(sim.state.pos.y - this.target) > STEP.band) this.lastOutside = sim.t - STEP.at;
  }
  metric(): number {
    return this.crashed ? Infinity : this.lastOutside;
  }
}

/** |mean| + 3σ of the altitude error from 8 s on, m. */
class HoldError implements Judge {
  private s = 0;
  private s2 = 0;
  private n = 0;
  private crashed = false;
  sample(sim: Simulation): void {
    if (sim.state.crashed) this.crashed = true;
    if (sim.t < 8) return;
    const e = sim.state.pos.y - sim.setpoint.y;
    this.s += e;
    this.s2 += e * e;
    this.n++;
  }
  metric(): number {
    if (this.crashed || !this.n) return Infinity;
    const mean = this.s / this.n;
    return totalOf(mean, Math.sqrt(Math.max(this.s2 / this.n - mean * mean, 0)));
  }
}

/** The requirement table of lesson V.2. */
export const REQUIREMENTS: Requirement[] = [
  {
    id: 'R1',
    text: 'Hold the altitude within 10 cm (|mean| + 3σ) in turbulence of 1.5 m/s.',
    metric: 'hold error',
    unit: 'cm',
    limit: REQUIREMENT,
    scale: 100,
    scenario: turbulence,
    seconds: 40,
    judge: () => new HoldError(),
  },
  {
    id: 'R2',
    text: 'Overshoot a 1 m step by no more than 10 %, in still air.',
    metric: 'overshoot',
    unit: '%',
    limit: 10,
    scale: 1,
    scenario: stepScenario,
    seconds: 10,
    events: stepEvent,
    judge: () => new Overshoot(),
  },
  {
    id: 'R3',
    text: 'Settle within ±10 cm of a 1 m step in 2 s, in still air.',
    metric: 'settling time',
    unit: 's',
    limit: 2,
    scale: 1,
    scenario: stepScenario,
    seconds: 10,
    events: stepEvent,
    judge: () => new Settling(),
  },
];

/** Flights per requirement: enough to find the corners, few enough to wait for. */
export const CAMPAIGN_RUNS = 20;

export interface CampaignRun {
  values: number[];
  metric: number;
  passed: boolean;
}

export interface RequirementResult {
  requirement: Requirement;
  runs: CampaignRun[];
  passed: number;
  /** 95 % lower bound on the pass rate. */
  bound: number;
  /** The run furthest beyond (or nearest to) the limit. */
  worst: CampaignRun;
  green: boolean;
}

/** The `n` sets of drawn values every requirement uses (common random numbers). */
export function drawValues(n: number, seed = 11): number[][] {
  const r = new Rng(seed ^ 0x1b873593);
  return Array.from({ length: n }, () =>
    L1_DISPERSIONS.map((d) => {
      const u = r.next();
      return d.log ? d.lo * (d.hi / d.lo) ** u : d.lo + (d.hi - d.lo) * u;
    }),
  );
}

/** One member of the campaign of a requirement: the design, its scenario, the drawn values. */
export function campaignMember(p: Params, req: Requirement, values: readonly number[]): Params {
  const q = structuredClone(p);
  req.scenario(q);
  L1_DISPERSIONS.forEach((d, i) => d.apply(q, values[i]!));
  return q;
}

class Flight {
  private sim: Simulation;
  private k = 0;
  private judge: Judge;
  constructor(
    p: Params,
    private req: Requirement,
    readonly values: number[],
  ) {
    this.sim = new Simulation(campaignMember(p, req, values));
    req.events?.(this.sim);
    this.judge = req.judge();
  }
  get done(): boolean {
    return this.k >= this.req.seconds * 1000;
  }
  advance(budget: number): number {
    const n = Math.min(budget, this.req.seconds * 1000 - this.k);
    for (let i = 0; i < n; i++) {
      this.sim.step();
      this.k++;
      if (this.k % 5 === 0) this.judge.sample(this.sim);
      if (this.sim.state.crashed) {
        this.judge.sample(this.sim);
        this.k = this.req.seconds * 1000;
        return i + 1;
      }
    }
    return n;
  }
  get run(): CampaignRun {
    const metric = this.judge.metric();
    return { values: this.values, metric, passed: metric <= this.req.limit };
  }
}

/** All the requirements' campaigns, one flight after another, in slices. */
export class VerificationCampaign {
  readonly results: RequirementResult[] = [];
  private draws: number[][];
  private runs: CampaignRun[] = [];
  private flight: Flight | null = null;

  constructor(
    readonly p: Params,
    readonly requirements: Requirement[] = REQUIREMENTS,
    readonly n = CAMPAIGN_RUNS,
    seed = 11,
  ) {
    this.draws = drawValues(n, seed);
  }
  get done(): boolean {
    return this.results.length >= this.requirements.length;
  }
  get progress(): number {
    return (this.results.length * this.n + this.runs.length) / (this.requirements.length * this.n);
  }
  advance(budget: number): boolean {
    let left = budget;
    while (left > 0 && !this.done) {
      const req = this.requirements[this.results.length]!;
      this.flight ??= new Flight(this.p, req, this.draws[this.runs.length]!);
      left -= this.flight.advance(left);
      if (!this.flight.done) continue;
      this.runs.push(this.flight.run);
      this.flight = null;
      if (this.runs.length === this.n) {
        this.results.push(summarise(req, this.runs));
        this.runs = [];
      }
    }
    return this.done;
  }
}

export function summarise(requirement: Requirement, runs: CampaignRun[]): RequirementResult {
  const passed = runs.filter((r) => r.passed).length;
  const worst = runs.reduce((a, b) => (b.metric > a.metric ? b : a));
  return {
    requirement,
    runs,
    passed,
    bound: passRateBound(passed, runs.length),
    worst,
    green: passed === runs.length,
  };
}

/** What a campaign depends on: the design and the nominal vehicle it is dispersed around. */
export const verificationKey = (p: Params): string =>
  JSON.stringify([p.sim.level, p.control, p.drone, p.wind, p.sensors, p.setpoint]);

const finished = new Map<string, RequirementResult[]>();
export const finishedVerification = (p: Params): RequirementResult[] | null =>
  finished.get(verificationKey(p)) ?? null;
export const storeVerification = (p: Params, r: RequirementResult[]): void => {
  finished.set(verificationKey(p), r);
};

/** Fly the whole campaign at once (tests, the review) and remember it. */
export function runVerification(p: Params, n = CAMPAIGN_RUNS): RequirementResult[] {
  const c = new VerificationCampaign(p, REQUIREMENTS, n);
  while (!c.advance(10_000_000));
  if (n === CAMPAIGN_RUNS) storeVerification(p, c.results);
  return c.results;
}
