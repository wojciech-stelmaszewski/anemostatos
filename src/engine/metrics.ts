export interface StepMetrics {
  /** Time of the step. */
  t0: number;
  from: number;
  to: number;
  /** 10 % → 90 % rise time, s (null until reached). */
  riseTime: number | null;
  /** Peak beyond the target, % of the step size. */
  overshoot: number;
  /** Time after the step until the response stays within ±band, s (null while not settled). */
  settlingTime: number | null;
  /** Mean error over the last 0.5 s, once settled. */
  steadyStateError: number | null;
}

/**
 * Step-response analysis (docs/pid-primer.md §3) of the most recent setpoint jump in the data.
 * `band` is the settling tolerance as a fraction of the step (default 2 %).
 */
export function analyzeLastStep(
  t: readonly number[],
  sp: readonly number[],
  y: readonly number[],
  band = 0.02,
  minStep = 0.05,
): StepMetrics | null {
  // Find the last sample-to-sample jump of the setpoint.
  let k0 = -1;
  for (let k = sp.length - 1; k > 0; k--) {
    if (Math.abs(sp[k]! - sp[k - 1]!) >= minStep) {
      k0 = k;
      break;
    }
  }
  if (k0 < 0) return null;
  const from = sp[k0 - 1]!;
  const to = sp[k0]!;
  const step = to - from;
  const dir = Math.sign(step);
  const t0 = t[k0]!;

  // Only analyse while the setpoint stays put after the step.
  let end = sp.length;
  for (let k = k0 + 1; k < sp.length; k++) {
    if (Math.abs(sp[k]! - to) > 1e-9) {
      end = k;
      break;
    }
  }

  let t10: number | null = null;
  let t90: number | null = null;
  let peak = 0;
  let lastOutside = t0;
  for (let k = k0; k < end; k++) {
    const yk = y[k]!;
    if (Number.isNaN(yk)) continue;
    const progress = (yk - from) / step;
    if (t10 === null && progress >= 0.1) t10 = t[k]!;
    if (t90 === null && progress >= 0.9) t90 = t[k]!;
    peak = Math.max(peak, (yk - to) * dir);
    if (Math.abs(yk - to) > band * Math.abs(step)) lastOutside = t[k]!;
  }
  const tEnd = t[end - 1]!;
  // Consider it settled if it has stayed inside the band for at least 1 s.
  const settled = tEnd - lastOutside >= 1;
  let ess: number | null = null;
  if (settled) {
    let sum = 0;
    let n = 0;
    for (let k = end - 1; k >= k0 && t[k]! >= tEnd - 0.5; k--) {
      if (!Number.isNaN(y[k]!)) {
        sum += to - y[k]!;
        n++;
      }
    }
    ess = n ? sum / n : null;
  }
  return {
    t0,
    from,
    to,
    riseTime: t10 !== null && t90 !== null ? t90 - t10 : null,
    overshoot: (Math.max(0, peak) / Math.abs(step)) * 100,
    settlingTime: settled ? lastOutside - t0 : null,
    steadyStateError: ess,
  };
}
