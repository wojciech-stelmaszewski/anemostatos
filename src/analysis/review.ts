// The design review of lesson V.6 (docs/aerospace-gnc.md, Chapter P): one vehicle, a requirement
// set and a checklist — model, margins, campaign, faults, software — run by the app and reported.
import { PHYS_DT, Simulation } from '@/engine/simulation';
import { cabs, carg, cdiv, cis, cmul, type Complex } from '@/math/complex';
import { evalAt } from '@/math/lti';
import { logspace, margins, type Margins } from '@/math/margins';
import type { Params } from '@/sim/params';
import { VerificationCampaign, type RequirementResult } from './campaign';
import { l1Loop, loopGain } from './loop';
import { l1Plant, PLANT_OUT } from './plant';
import {
  FIXED,
  fixedDisagreement,
  fixedScenario,
  VOTER,
  voterError,
  voterScenario,
} from './software';
import { Sweep, type SweepPoint } from './sweep';

/** What each item of the checklist demands. */
export const REVIEW = {
  /** The model and the flight agree within this near crossover, dB and degrees. */
  modelDb: 1,
  modelDeg: 5,
  pmDeg: 45,
  gmDb: 6,
};

export type ItemId = 'model' | 'margins' | 'campaign' | 'faults' | 'software';

export interface ReviewItem {
  id: ItemId;
  title: string;
  /** What the item checks, as a reviewer would ask it. */
  question: string;
  passed: boolean;
  /** The evidence, in one line. */
  evidence: string;
}

/** Margins of the altitude loop's linear model, for comparison (null if there is none). */
export function modelMargins(p: Params): Margins | null {
  const m = l1Loop(p);
  if (!m) return null;
  const w = logspace(2 * Math.PI * 0.02, (Math.PI / m.T) * 0.98, 1200);
  return margins(
    w,
    w.map((x) => loopGain(m, x)),
  );
}

/** The frequencies of the review's probe sweep, Hz: around every crossover the designs have. */
export const REVIEW_FREQS = logspace(0.3, 15, 14);

/** Margins from the measured loop: phase, the smaller gain margin (up or down), delay. */
export function measuredMargins(points: readonly SweepPoint[]): Margins {
  return margins(
    points.map((x) => 2 * Math.PI * x.fHz),
    points.map((x) => x.l!),
  );
}
export const smallerGm = (m: Margins): number =>
  Math.min(
    Number.isFinite(m.gm) ? 20 * Math.log10(m.gm) : Infinity,
    m.gmLow > 0 ? -20 * Math.log10(m.gmLow) : Infinity,
  );

/**
 * The plant the controller was designed on: the simulator's own model of the altitude dynamics
 * with the controller's mass m̂ and motor lag τ̂ in place of the true ones, and the sensor delay.
 */
export function modelPlant(p: Params, fHz: number): Complex {
  const q = structuredClone(p);
  q.drone.mass = p.control.model.mass;
  q.drone.motorTau = p.control.model.motorTau;
  const w = 2 * Math.PI * fHz;
  const y = evalAt(l1Plant(q), cis(w * PHYS_DT))[PLANT_OUT.y]![0]!;
  return cmul(y, cis(-w * (p.sensors.delayMs / 1000)));
}

const DEG = 180 / Math.PI;
const wrapDeg = (d: number) => ((((d + 180) % 360) + 360) % 360) - 180;

type Stage =
  | { kind: 'sweep'; sweep: Sweep }
  | { kind: 'campaign'; campaign: VerificationCampaign }
  | { kind: 'flight'; item: 'faults' | 'software'; sim: Simulation; seconds: number };

const ORDER: ItemId[] = ['model', 'margins', 'campaign', 'faults', 'software'];

/** The checklist, item by item, in slices. */
export class DesignReview {
  readonly items: ReviewItem[] = [];
  private stage: Stage | null = null;

  constructor(readonly p: Params) {}

  get done(): boolean {
    return this.items.length >= ORDER.length;
  }
  get progress(): number {
    const within =
      this.stage?.kind === 'sweep'
        ? this.stage.sweep.progress
        : this.stage?.kind === 'campaign'
          ? this.stage.campaign.progress
          : this.stage?.kind === 'flight'
            ? this.stage.sim.t / this.stage.seconds
            : 0;
    return (this.items.length + within) / ORDER.length;
  }
  /** The item being worked on. */
  get current(): ItemId | null {
    return this.done ? null : ORDER[this.items.length]!;
  }

  advance(budget: number): boolean {
    let left = budget;
    while (left > 0 && !this.done) {
      const id = ORDER[this.items.length]!;
      // The sweep fills in two items: the model and the margins.
      if (id === 'model' || id === 'margins') left -= this.sweep(left);
      else if (id === 'campaign') left -= this.campaign(left);
      else left -= this.flight(id, left);
    }
    return this.done;
  }

  /**
   * One probe sweep answers two questions: does the plant behave as the model the controller was
   * designed on (near crossover), and how much margin does the loop really have.
   */
  private sweep(budget: number): number {
    if (!this.stage)
      this.stage = {
        kind: 'sweep',
        sweep: new Sweep(this.p, REVIEW_FREQS, { point: 'l1.thrust' }),
      };
    const st = this.stage as Extract<Stage, { kind: 'sweep' }>;
    st.sweep.advance(budget);
    if (!st.sweep.done) return budget;
    const pts = st.sweep.points;
    const m = measuredMargins(pts);
    const fc = (m.gainCrossovers[0]?.w ?? NaN) / (2 * Math.PI);
    const near = pts.filter((x) => x.fHz >= fc / 2 && x.fHz <= fc * 2);
    let worstDb = 0;
    let worstDeg = 0;
    for (const pt of near) {
      const ratio = cdiv(pt.pm!, modelPlant(this.p, pt.fHz));
      worstDb = Math.max(worstDb, Math.abs(20 * Math.log10(cabs(ratio))));
      worstDeg = Math.max(worstDeg, Math.abs(wrapDeg(carg(ratio) * DEG)));
    }
    const crashed = st.sweep.crashed || !near.length;
    this.items.push({
      id: 'model',
      title: 'Model',
      question: 'Is the plant the plant the controller was designed for?',
      passed: !crashed && worstDb <= REVIEW.modelDb && worstDeg <= REVIEW.modelDeg,
      evidence: crashed
        ? 'the sweep did not find a crossover'
        : `measured plant within ${worstDb.toFixed(2)} dB and ${worstDeg.toFixed(1)}° of the model (m̂ ${this.p.control.model.mass} kg, τ̂ ${this.p.control.model.motorTau * 1000} ms) from ${(fc / 2).toFixed(1)} to ${(fc * 2).toFixed(1)} Hz`,
    });
    const gm = smallerGm(m);
    const fmt = (v: number) => (Number.isFinite(v) ? v.toFixed(1) : '∞');
    this.items.push({
      id: 'margins',
      title: 'Margins',
      question: `Does the loop keep ${REVIEW.gmDb} dB of gain margin both ways and ${REVIEW.pmDeg}° of phase margin?`,
      passed: !crashed && m.pmDeg >= REVIEW.pmDeg && gm >= REVIEW.gmDb,
      evidence: `measured with the probe: PM ${fmt(m.pmDeg)}°, GM ${fmt(gm)} dB, delay margin ${(m.delayMargin * 1000).toFixed(0)} ms`,
    });
    this.stage = null;
    return budget;
  }

  private campaign(budget: number): number {
    if (!this.stage) this.stage = { kind: 'campaign', campaign: new VerificationCampaign(this.p) };
    const st = this.stage as Extract<Stage, { kind: 'campaign' }>;
    st.campaign.advance(budget);
    if (!st.campaign.done) return budget;
    const r: RequirementResult[] = st.campaign.results;
    this.items.push({
      id: 'campaign',
      title: 'Campaign',
      question: 'Does every requirement hold across the dispersions?',
      passed: r.every((x) => x.green),
      evidence: r.map((x) => `${x.requirement.id} ${x.passed}/${x.runs.length}`).join(', '),
    });
    this.stage = null;
    return budget;
  }

  private flight(id: 'faults' | 'software', budget: number): number {
    if (!this.stage) {
      const q = structuredClone(this.p);
      (id === 'faults' ? voterScenario : fixedScenario)(q);
      const seconds = id === 'faults' ? VOTER.end : FIXED.end;
      this.stage = { kind: 'flight', item: id, sim: new Simulation(q), seconds };
    }
    const st = this.stage as Extract<Stage, { kind: 'flight' }>;
    const n = Math.min(budget, Math.round((st.seconds - st.sim.t) * 1000));
    for (let k = 0; k < n; k++) st.sim.step();
    if (st.sim.t < st.seconds - 1e-9) return Math.max(n, 1);
    if (id === 'faults') {
      const e = voterError(st.sim);
      this.items.push({
        id,
        title: 'Faults',
        question: `Does it survive two altimeter failures, the altitude it flies on within ${VOTER.limit * 100} cm of the truth?`,
        passed: e <= VOTER.limit,
        evidence: Number.isFinite(e)
          ? `voter "${this.p.part5.voter.mode}": worst error of the voted altitude ${(e * 100).toFixed(1)} cm`
          : `voter "${this.p.part5.voter.mode}": crashed`,
      });
    } else {
      const e = fixedDisagreement(st.sim);
      const fx = this.p.part5.fixed;
      this.items.push({
        id,
        title: 'Software',
        question: `Does the fixed-point code agree with the reference within ${FIXED.limit} % of hover thrust?`,
        passed: e <= FIXED.limit,
        evidence: `integrator ${fx.intBits} bits, ${fx.intFrac} fractional: worst disagreement ${e.toFixed(2)} %`,
      });
    }
    this.stage = null;
    return Math.max(n, 1);
  }
}

/** Run the whole review at once (tests) and remember it, as the panel does. */
export function runReview(p: Params): ReviewItem[] {
  const r = new DesignReview(p);
  while (!r.advance(10_000_000));
  storeReview(p, r.items);
  return r.items;
}

/** What a review depends on: the design, its software, and the vehicle it flies. */
export const reviewKey = (p: Params): string =>
  JSON.stringify([p.sim.level, p.control, p.drone, p.sensors, p.wind, p.setpoint, p.part5]);
const finished = new Map<string, ReviewItem[]>();
export const finishedReview = (p: Params): ReviewItem[] | null =>
  finished.get(reviewKey(p)) ?? null;
export const storeReview = (p: Params, items: ReviewItem[]): void => {
  finished.set(reviewKey(p), items);
};
