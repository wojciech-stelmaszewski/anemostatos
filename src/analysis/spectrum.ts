// Spectra of the signals the simulation records at the physics rate (docs/analysis.md §4,
// Chapter I): what the gyro reports, what the body really does, and what the motors are told.
import { PHYS_DT, type Simulation } from '@/engine/simulation';
import type { TapChannel } from '@/engine/tap';
import { welch } from '@/math/fft';

const FS = 1 / PHYS_DT;
/** Samples per Welch segment: about half a hertz of resolution at 1 kHz. */
const SEGMENT = 2048;

/** Where a tone at `f` appears after sampling at `fs`: folded into 0 … fs/2. */
export const aliasHz = (f: number, fs: number): number => Math.abs(f - fs * Math.round(f / fs));

/** RMS of the part of `x` between `f1` and `f2` hertz, from its power spectral density. */
export function bandRms(x: ArrayLike<number>, f1: number, f2: number, fs = FS): number {
  if (x.length < 64) return 0;
  const { f, psd } = welch(x, fs, SEGMENT);
  const df = f[1]! - f[0]!;
  let s = 0;
  for (let i = 0; i < f.length; i++) if (f[i]! >= f1 && f[i]! <= f2) s += psd[i]! * df;
  return Math.sqrt(s);
}

export interface Spectrum {
  fHz: number[];
  /** Amplitude a sine at each frequency would need to give this density, in the signal's units. */
  amp: number[];
}

/** Amplitude spectrum of a recorded channel over the last `seconds` (Hann window, averaged). */
export function spectrumOf(x: ArrayLike<number>, fs = FS): Spectrum {
  if (x.length < 256) return { fHz: [], amp: [] };
  const { f, psd } = welch(x, fs, SEGMENT);
  // A sine of amplitude A is a line of power A²/2 spread over the window's 1.5 bins.
  const df = f[1]! - f[0]!;
  return { fHz: f, amp: psd.map((v) => Math.sqrt(2 * v * 1.5 * df)) };
}

const recent = (sim: Simulation, ch: TapChannel, seconds: number) =>
  sim.tap.last(ch, Math.round(seconds * FS));

/** Where the rate loop listens: everything the gyro reports below this matters to it, Hz. */
export const LOOP_BAND_HZ = 60;

/**
 * The part of the gyro reading that is not motion, inside the band of the loop, deg/s RMS: what
 * aliasing puts there, and what no later filter can tell from real motion.
 */
export function ghostRate(sim: Simulation, seconds = 8): number {
  const s = recent(sim, 'gyro.sensor', seconds);
  const t = recent(sim, 'gyro.true', seconds);
  const d = new Float64Array(s.length);
  for (let i = 0; i < s.length; i++) d[i] = s[i]! - t[i]!;
  return bandRms(d, 1, LOOP_BAND_HZ);
}

/** Motor command content above the band of the loop, N RMS: heat and wear, no control. */
export function motorJitter(sim: Simulation, seconds = 8): number {
  return bandRms(recent(sim, 'motor.cmd', seconds), LOOP_BAND_HZ, FS / 2);
}
