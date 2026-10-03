import type { Simulation } from '@/engine/simulation';
import { rmsError, smoothTrack } from '@/estimation/rts';

/** The reconstruction of lesson IV.29 starts after the take-off, s. */
export const SMOOTH_FROM = 3;

export interface Reconstruction {
  t: number[];
  /** Errors of the filter and of the smoother against the truth, and their 1σ, m. */
  ef: number[];
  es: number[];
  sf: number[];
  ss: number[];
  rmsF: number;
  rmsS: number;
}

/**
 * Filter and smooth the recorded altitude readings from `from` on, and compare both with the
 * recorded truth: a flight-test reconstruction (src/estimation/rts.ts).
 */
export function reconstructAltitude(sim: Simulation, from = SMOOTH_FROM): Reconstruction | null {
  const p = sim.params;
  if (!p.smoother.enabled || p.sensors.posNoise <= 0) return null;
  const w = sim.telemetry.window(['meas.y', 'pos.y'], from);
  if (w.t.length < 10) return null;
  const tr = smoothTrack(w.t, w.series[0]!, p.sensors.posNoise, p.smoother.accelSigma);
  const truth = w.series[1]!;
  return {
    t: w.t,
    ef: tr.filter.map((v, i) => v - truth[i]!),
    es: tr.smooth.map((v, i) => v - truth[i]!),
    sf: tr.filterSigma,
    ss: tr.smoothSigma,
    rmsF: rmsError(tr.filter, truth),
    rmsS: rmsError(tr.smooth, truth),
  };
}
