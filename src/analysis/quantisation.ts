// The real computer (docs/analysis.md §4, Chapter I, lessons III.18 and III.19): what the loop
// rate costs the quadrotor's rate loop, and the limit cycle a coarse altimeter leaves in the
// altitude loop, predicted by a describing function.
import { dfRelay } from './describing';
import { PHYS_DT } from '@/engine/simulation';
import { logspace, margins } from '@/math/margins';
import type { Params } from '@/sim/params';
import { posQuantStep } from '@/sim/sensors';
import { analyseRateLoop } from './mimo';
import { l1Loop, loopGain } from './loop';

/** Rates the rate loop can run at: the physics step divided by a whole number, Hz. */
export const loopRates = (minHz = 20): number[] => {
  const out: number[] = [];
  for (let n = 1; 1 / (n * PHYS_DT) >= minHz; n++) out.push(1 / (n * PHYS_DT));
  return out;
};

/** Phase margin of the rate loop at a given loop rate, one loop at a time; NaN when unstable. */
export function ratePm(p: Params, hz: number): number {
  const q = structuredClone(p);
  q.control.l3.hzRate = hz;
  const a = analyseRateLoop(q, 300);
  return a && a.stable ? a.one.pmDeg : NaN;
}

/** The lowest loop rate (a whole fraction of 1 kHz) with at least `pmDeg` of phase margin. */
export function lowestRate(p: Params, pmDeg: number): number {
  let best = NaN;
  for (const hz of loopRates()) {
    const pm = ratePm(p, hz);
    if (pm >= pmDeg) best = hz;
    else break;
  }
  return best;
}

/**
 * Describing function of a quantiser of step q held at a boundary between two levels: for a small
 * sine of amplitude A it switches like a relay of ±q/2, N(A) = 4·(q/2)/(π·A) = 2q/(π·A).
 */
export const quantiserDf = (q: number, a: number): number => dfRelay(a, q / 2);

export interface QuantisationPrediction {
  /** Step of the altimeter, m. */
  q: number;
  /** The loop's gain margin and the frequency where its phase is −180°, Hz. */
  gm: number;
  f180Hz: number;
  /** Predicted amplitude of the limit cycle, m: N(A)·|L(jω₁₈₀)| = 1, so A = 2q/(π·GM). */
  amplitude: number;
}

/** The limit cycle of the L1 altitude loop with a quantised altimeter (null without a model). */
export function quantisationCycle(p: Params): QuantisationPrediction | null {
  const bits = p.sensors.posQuantBits;
  const m = l1Loop(p);
  if (!m || bits <= 0) return null;
  const w = logspace(2 * Math.PI * 0.02, (Math.PI / m.T) * 0.98, 1500);
  const r = margins(
    w,
    w.map((x) => loopGain(m, x)),
  );
  const q = posQuantStep(bits);
  return {
    q,
    gm: r.gm,
    f180Hz: r.w180 / (2 * Math.PI), // N(A)·|L| = 1 at the −180° frequency, with |L| = 1/GM: the relay's amplitude 4·(q/2)/(π·N).
    amplitude: (4 * (q / 2)) / (Math.PI * r.gm),
  };
}
