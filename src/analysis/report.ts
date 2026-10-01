// The robustness report card (docs/analysis.md §4, lesson III.30): for every controller, the
// classical margins measured by a probe sweep, a Monte Carlo pass rate with its confidence bound,
// and the RMS error that Part II ranked them by. Everything is measured, nothing is modelled, so
// the card covers controllers that have no linear model.
import { ENTRIES, runArena, SCENARIOS } from '@/engine/arena';
import { Simulation } from '@/engine/simulation';
import { C_ONE, cdiv, csub } from '@/math/complex';
import { logspace, margins } from '@/math/margins';
import { defaultParams, type Params } from '@/sim/params';
import { DispersionCampaign, passRateBound } from './dispersion';
import { Campaign } from './montecarlo';
import { Sweep } from './sweep';

/** The classical requirement the card checks against (flight-control practice: 6 dB, 45°). */
export const REQUIRED = { gmDb: 6, pmDeg: 45 } as const;

export interface ReportRow {
  level: 1 | 3;
  id: string;
  label: string;
  /** Measured at the plant input: the thrust (L1) or the roll torque (L3). */
  pmDeg: number;
  /** The smaller of the two gain margins (up and down), dB; Infinity if neither exists. */
  gmDb: number;
  delayMs: number;
  /** Peak of |S| on the measured points. */
  ms: number;
  /**
   * Whether the sweep found a consistent linear loop. A flying controller cannot have a negative
   * margin: if the sweep says so, the loop is not linear and time-invariant (a sampling planner,
   * a neural network) and its margins mean nothing.
   */
  linear: boolean;
  /** Monte Carlo: flights that passed, out of `runs`, and the 95 % lower bound on the rate. */
  passed: number;
  runs: number;
  bound: number;
  /** RMS error in the gust scenario, m: altitude (L1) or position (L3). */
  rms: number;
  crashed: boolean;
}

export interface ReportEntry {
  level: 1 | 3;
  id: string;
  label: string;
  setup: (p: Params) => void;
}

const l1 =
  (kind: Params['control']['l1']['kind'], extra: (p: Params) => void = () => {}) =>
  (p: Params) => {
    p.control.l1.kind = kind;
    extra(p);
  };

/** The L1 controllers of Parts I and II, and the L3 entries of the arena. */
export const REPORT_ENTRIES: ReportEntry[] = [
  { level: 1, id: 'pid', label: 'PID', setup: l1('pid') },
  { level: 1, id: 'lqr', label: 'LQR', setup: l1('lqr') },
  {
    level: 1,
    id: 'lqi',
    label: 'LQI + motor-lag state',
    setup: l1('lqr', (p) => {
      p.control.lqr.integral = true;
      p.control.lqr.lagState = true;
    }),
  },
  { level: 1, id: 'adrc', label: 'ADRC', setup: l1('adrc') },
  { level: 1, id: 'mpc', label: 'MPC', setup: l1('mpc') },
  { level: 1, id: 'l1ac', label: 'L1 adaptive', setup: l1('l1ac') },
  ...ENTRIES.map((e) => ({ level: 3 as const, id: e.id, label: e.label, setup: e.setup })),
];

/** Flights per Monte Carlo campaign on the card: enough for a bound, few enough to wait for. */
export const CARD_RUNS = 50;

const base = (e: ReportEntry): Params => {
  const p = defaultParams();
  p.sim.level = e.level;
  e.setup(p);
  return p;
};

/** Margins from measured points of the loop broken at the plant input. */
function measuredMargins(fHz: number[], l: ReturnType<typeof cdiv>[]) {
  const m = margins(
    fHz.map((f) => 2 * Math.PI * f),
    l,
  );
  const up = Number.isFinite(m.gm) ? 20 * Math.log10(m.gm) : Infinity;
  const down = m.gmLow > 0 ? -20 * Math.log10(m.gmLow) : Infinity;
  const gmDb = Math.min(up, down);
  return {
    pmDeg: m.pmDeg,
    gmDb,
    delayMs: m.delayMargin * 1000,
    ms: m.ms,
    linear: m.pmDeg > 0 && gmDb > 0,
  };
}

/** RMS altitude error of an L1 controller hovering in Part I's default gusts, m. */
function l1GustRms(p: Params): { rms: number; crashed: boolean } {
  const q = structuredClone(p);
  q.sim.seed = 4242;
  const sim = new Simulation(q);
  let sq = 0;
  let n = 0;
  for (let k = 1; k <= 40000; k++) {
    sim.step();
    if (sim.state.crashed) return { rms: NaN, crashed: true };
    if (k > 8000 && k % 5 === 0) {
      sq += (sim.state.pos.y - sim.setpoint.y) ** 2;
      n++;
    }
  }
  return { rms: Math.sqrt(sq / n), crashed: false };
}

type Stage =
  | { kind: 'sweep'; sweep: Sweep }
  | { kind: 'campaign'; campaign: DispersionCampaign | Campaign }
  | { kind: 'gusts' };

/**
 * Builds the card one entry at a time, in slices: a probe sweep, a campaign, a gust flight. The
 * rows appear as they are finished.
 */
export class ReportCard {
  readonly rows: ReportRow[] = [];
  private index = 0;
  private stage: Stage | null = null;
  private partial: Partial<ReportRow> = {};

  constructor(
    readonly entries: ReportEntry[] = REPORT_ENTRIES,
    readonly runs = CARD_RUNS,
  ) {}

  get done(): boolean {
    return this.index >= this.entries.length;
  }
  /** Name of the entry being built and what is being done to it. */
  get status(): string {
    if (this.done) return 'done';
    const e = this.entries[this.index]!;
    const what = !this.stage
      ? 'starting'
      : this.stage.kind === 'sweep'
        ? 'probe sweep'
        : this.stage.kind === 'campaign'
          ? 'Monte Carlo'
          : 'gust flight';
    return `${e.label}: ${what}`;
  }

  advance(budget: number): boolean {
    let left = budget;
    while (left > 0 && !this.done) {
      const e = this.entries[this.index]!;
      const p = base(e);
      if (!this.stage) {
        const freqs = e.level === 1 ? logspace(0.1, 20, 14) : logspace(0.5, 40, 20);
        this.stage = {
          kind: 'sweep',
          sweep: new Sweep(
            p,
            freqs,
            e.level === 1
              ? { point: 'l1.thrust' }
              : { point: 'l3.torque.x', amp: 0.01, settleS: 6, transientS: 3, minSeconds: 1.5 },
          ),
        };
      }
      const st = this.stage;
      if (st.kind === 'sweep') {
        st.sweep.advance(left);
        left -= Math.max(1, Math.min(left, 20000));
        if (!st.sweep.done) continue;
        const pts = st.sweep.points;
        const l = pts.map((pt) => (e.level === 1 ? pt.l! : csub(cdiv(C_ONE, pt.sCol![0]), C_ONE)));
        this.partial = measuredMargins(
          pts.map((pt) => pt.fHz),
          l,
        );
        this.stage = {
          kind: 'campaign',
          campaign:
            e.level === 1 ? new Campaign(p, this.runs, 7) : new DispersionCampaign(p, this.runs),
        };
      } else if (st.kind === 'campaign') {
        st.campaign.advance(left);
        left -= Math.min(left, 20000);
        if (!st.campaign.done) continue;
        const failed =
          st.campaign instanceof Campaign
            ? st.campaign.trials.filter((t) => !t.stable).length
            : st.campaign.failures;
        const passed = this.runs - failed;
        this.partial = {
          ...this.partial,
          passed,
          runs: this.runs,
          bound: passRateBound(passed, this.runs),
        };
        this.stage = { kind: 'gusts' };
      } else {
        // One flight, run to the end at once.
        const g =
          e.level === 1
            ? l1GustRms(p)
            : (() => {
                const r = runArena(
                  { id: e.id, label: e.label, setup: e.setup },
                  SCENARIOS.find((s) => s.id === 'gusts')!,
                );
                return { rms: r.rms, crashed: r.crashed };
              })();
        left -= 40000;
        this.rows.push({
          level: e.level,
          id: e.id,
          label: e.label,
          ...(this.partial as Omit<ReportRow, 'level' | 'id' | 'label' | 'rms' | 'crashed'>),
          rms: g.rms,
          crashed: g.crashed,
        });
        this.partial = {};
        this.stage = null;
        this.index++;
      }
    }
    return this.done;
  }
}

/** Build the whole card at once (tests and `make report`). */
export function buildReport(entries = REPORT_ENTRIES, runs = CARD_RUNS): ReportRow[] {
  const card = new ReportCard(entries, runs);
  while (!card.advance(1_000_000));
  return card.rows;
}

/** Whether a row meets the classical margin requirement. */
export const meetsMargins = (r: ReportRow): boolean =>
  r.linear && r.pmDeg >= REQUIRED.pmDeg && r.gmDb >= REQUIRED.gmDb;
