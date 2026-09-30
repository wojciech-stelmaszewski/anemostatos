import type { ProbeParams } from '@/sim/params';

/**
 * The probe's test signal at `t` seconds after it was switched on (docs/analysis.md §3.3).
 * Deterministic: the same parameters give the same signal in every run.
 */
export function probeSignal(p: ProbeParams, t: number): number {
  if (p.point === 'none' || t < 0) return 0;
  if (p.signal === 'chirp') {
    // Logarithmic sweep from f0 to f1 over durationS, then silence.
    const dur = Math.max(p.durationS, 1e-3);
    if (t > dur) return 0;
    const f0 = Math.max(p.f0Hz, 1e-4);
    const ratio = Math.max(p.f1Hz, f0 * 1.0001) / f0;
    const phase = ((2 * Math.PI * f0 * dur) / Math.log(ratio)) * (ratio ** (t / dur) - 1);
    return p.amp * Math.sin(phase);
  }
  return p.amp * Math.sin(2 * Math.PI * p.freqHz * t);
}

/** Instantaneous frequency of the chirp, Hz. */
export const chirpFrequency = (p: ProbeParams, t: number): number => {
  const dur = Math.max(p.durationS, 1e-3);
  const f0 = Math.max(p.f0Hz, 1e-4);
  return f0 * (Math.max(p.f1Hz, f0 * 1.0001) / f0) ** (Math.min(Math.max(t, 0), dur) / dur);
};
