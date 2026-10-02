import { v3 } from '@/math/vec3';
import type { Params } from '@/sim/params';
import { initialLander, stepLander } from '@/sim/vehicles/lander';
import { PdgGuidance } from './pdg';

/**
 * Lesson IV.5: a campaign of dispersed starts, each flown with the guidance of the simulator.
 * A case passes if it lands on the pad (within `PAD_RADIUS`) below the crash speed while staying
 * above the glide slope, or if the guidance reports at the start that no landing exists.
 */
export const CAMPAIGN_CASES = 20;
export const PAD_RADIUS = 1;

export type LandingOutcome = 'landed' | 'crashed' | 'missed' | 'cone' | 'infeasible';

export interface LandingRun {
  case: number;
  outcome: LandingOutcome;
  passed: boolean;
  speed: number;
  miss: number;
  /** Propellant left, kg, and what the first plan expected to burn. */
  fuelLeft: number;
  planFuel: number;
  /** Lowest elevation seen from the pad above the gate, deg. */
  minSlope: number;
}

/** Fly one dispersed case (1, 2, …) with this set-up, at a coarser step than the simulator. */
export function flyCase(p: Params, c: number, dt = 0.01): LandingRun {
  const l = { ...p.lander, dispersionCase: c };
  const s = initialLander(l);
  const g = new PdgGuidance();
  let minSlope = 90;
  let t = 0;
  for (; t < 120 && !s.landed && !s.crashed; t += dt) {
    const u = g.tick(s, t, l, p.pdg);
    if (g.infeasible) break;
    stepLander(s, u, v3(), dt, l);
    if (s.pos.y > p.pdg.gateAltitude)
      minSlope = Math.min(
        minSlope,
        (Math.atan2(s.pos.y, Math.hypot(s.pos.x, s.pos.z)) * 180) / Math.PI,
      );
  }
  const planFuel = g.first?.fuel ?? NaN;
  const base = {
    case: c,
    speed: s.touchdownSpeed,
    miss: s.miss,
    fuelLeft: s.fuel,
    planFuel,
    minSlope,
  };
  if (g.infeasible) return { ...base, outcome: 'infeasible', passed: true };
  const outcome: LandingOutcome = !s.landed
    ? 'crashed'
    : s.miss > PAD_RADIUS
      ? 'missed'
      : minSlope < l.glideSlopeDeg - 0.5
        ? 'cone'
        : 'landed';
  return { ...base, outcome, passed: outcome === 'landed' };
}

/** What makes two campaigns the same: everything but the case number. */
export const landingKey = (p: Params): string =>
  JSON.stringify([{ ...p.lander, dispersionCase: 0 }, p.pdg]);

/** A campaign flown one case at a time (one per animation frame in the app). */
export class LandingCampaign {
  runs: LandingRun[] = [];
  constructor(
    private readonly p: Params,
    readonly n = CAMPAIGN_CASES,
  ) {}

  /** Fly the next case; true when all are done. */
  advance(): boolean {
    if (this.runs.length < this.n) this.runs.push(flyCase(this.p, this.runs.length + 1));
    return this.runs.length >= this.n;
  }
}

const finished = new Map<string, LandingRun[]>();
export const finishedLandings = (p: Params): LandingRun[] | null =>
  finished.get(landingKey(p)) ?? null;
export const storeLandings = (p: Params, runs: LandingRun[]): void => {
  finished.set(landingKey(p), runs);
};
