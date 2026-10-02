// Pilot-induced oscillation (docs/aerospace-gnc.md §4, Chapter M, lesson IV.16): the pilot closes
// the attitude loop through an elevator whose rate is limited. A rate limiter is linear for small
// commands and adds phase lag once A·ω exceeds its rate; the describing function of lesson III.8,
// computed here for the simulator's own servo, predicts the amplitude at which the loop runs away.
import { cabs, carg, cis, cmul, csolve, cx, type Complex } from '@/math/complex';
import type { Params } from '@/sim/params';
import { aircraftLinear } from './aircraft';

const RAD = Math.PI / 180;
/** In the aircraft's linear model (V, α, q, θ, h, δe, T): the elevator, the pitch rate and attitude. */
const I_DE = 5;
const I_Q = 2;
const I_THETA = 3;

/** The physics step the servo is simulated at, s (the simulator's). */
const DT = 0.001;

/**
 * The linear part of the loop, broken at the elevator's actual deflection: from δe through the
 * airframe to the pilot (gain and delay on θ) and the damper (gain and delay on q), back to the
 * elevator command. Negative-feedback convention: the loop closes as 1 + N·G = 0.
 */
export function pilotLoop(p: Params): (w: number) => Complex {
  const m = aircraftLinear(p.aircraft, p.aircraft.speed);
  const keep = m.a.map((_, i) => i).filter((i) => i !== I_DE);
  const A = keep.map((i) => keep.map((j) => m.a[i]![j]!));
  const b = keep.map((i) => m.a[i]![I_DE]!);
  const iq = keep.indexOf(I_Q);
  const it = keep.indexOf(I_THETA);
  const ap = p.autopilot;
  const tp = ap.pilotDelayMs / 1000;
  const ta = ap.delayMs / 1000;
  return (w) => {
    const M = A.map((row, i) => row.map((v, j) => cx(-v, i === j ? w : 0)));
    const x = csolve(
      M,
      b.map((v) => [cx(v)]),
    ).map((r) => r[0]!);
    // δe_cmd = +K_p·θ(t − τ_p) + G·q(t − τ_a) (elevator positive is nose down).
    const pilot = cmul(cmul(x[it]!, cx(ap.pilotGain)), cis(-w * tp));
    const damper = cmul(cmul(x[iq]!, cx(ap.gain)), cis(-w * ta));
    const l = { re: pilot.re + damper.re, im: pilot.im + damper.im };
    return { re: -l.re, im: -l.im };
  };
}

/**
 * Describing function of the elevator servo (first-order lag, rate limit, travel), computed
 * numerically: a command A·sin(ωt), rad, through the same update as the simulator, and the
 * fundamental of the output over the last two periods after four to settle.
 */
export function servoDF(amp: number, w: number, p: Params): Complex {
  const a = p.aircraft;
  const lim = a.elevatorMax * RAD;
  const rmax = a.servoRate * RAD;
  const period = (2 * Math.PI) / w;
  const n = Math.round((5 * period) / DT);
  const start = Math.round((3 * period) / DT);
  let de = 0;
  let si = 0;
  let co = 0;
  let count = 0;
  for (let k = 0; k < n; k++) {
    const t = k * DT;
    const cmd = Math.min(Math.max(amp * Math.sin(w * t), -lim), lim);
    const rate = a.servoTau > 0 ? (cmd - de) / a.servoTau : (cmd - de) / DT;
    de += Math.min(Math.max(rate, -rmax), rmax) * DT;
    de = Math.min(Math.max(de, -lim), lim);
    if (k >= start) {
      // The output at the end of the step, the input at its start: half a step of lag.
      const tt = t + DT;
      si += de * Math.sin(w * tt);
      co += de * Math.cos(w * tt);
      count++;
    }
  }
  return { re: (2 * si) / count / amp, im: (2 * co) / count / amp };
}

/** The frequency, rad/s, in [lo, hi] where the phase of N·G is −180°, or NaN. */
function phaseCrossing(f: (w: number) => Complex, lo = 1, hi = 20): number {
  // Scan for the first sign change of the imaginary part with the real part negative.
  const steps = 60;
  let wPrev = lo;
  let vPrev = f(lo);
  for (let i = 1; i <= steps; i++) {
    const w = lo * (hi / lo) ** (i / steps);
    const v = f(w);
    if (vPrev.im * v.im <= 0 && (v.re < 0 || vPrev.re < 0)) {
      let a = wPrev;
      let b = w;
      let fa = vPrev.im;
      for (let k = 0; k < 40; k++) {
        const mid = Math.sqrt(a * b);
        const fm = f(mid).im;
        if (fa * fm <= 0) b = mid;
        else {
          a = mid;
          fa = fm;
        }
      }
      return Math.sqrt(a * b);
    }
    wPrev = w;
    vPrev = v;
  }
  return NaN;
}

export interface PioPoint {
  /** Amplitude of the elevator command, °. */
  ampDeg: number;
  /** Frequency where the loop's phase is −180°, rad/s, and the loop gain there. */
  w: number;
  gain: number;
}

/** The loop gain at its phase crossover, for a command oscillating with amplitude `ampDeg`. */
export function pioPoint(p: Params, ampDeg: number, g = pilotLoop(p)): PioPoint {
  const amp = ampDeg * RAD;
  const f = (w: number) => cmul(servoDF(amp, w, p), g(w));
  const w = phaseCrossing(f);
  return { ampDeg, w, gain: Number.isFinite(w) ? cabs(f(w)) : 0 };
}

export interface PioOnset {
  /** The small-amplitude loop: gain margin (1/gain at the phase crossover) and frequency. */
  linearGain: number;
  linearW: number;
  /** The command amplitude at which the loop gain at −180° reaches 1, °, and that frequency. */
  onsetDeg: number;
  onsetW: number;
  /** Where the elevator starts to rate-limit at the linear crossover frequency: R/ω, °. */
  rateLimitDeg: number;
  /** The curve of loop gain against amplitude, for the chart. */
  curve: PioPoint[];
}

/**
 * The onset of the PIO by harmonic balance: the smallest command amplitude at which N·G reaches
 * −1. Below it an oscillation shrinks; above it, it grows until the elevator's travel stops it.
 */
export function pioOnset(p: Params, maxDeg = p.aircraft.elevatorMax): PioOnset {
  const g = pilotLoop(p);
  const small = pioPoint(p, 0.01, g);
  const curve: PioPoint[] = [];
  for (let i = 0; i <= 24; i++) curve.push(pioPoint(p, 0.2 * (maxDeg / 0.2) ** (i / 24), g));
  let onsetDeg = NaN;
  let onsetW = NaN;
  if (small.gain >= 1) {
    onsetDeg = 0;
    onsetW = small.w;
  } else {
    const k = curve.findIndex((c) => c.gain >= 1);
    if (k > 0) {
      let lo = curve[k - 1]!.ampDeg;
      let hi = curve[k]!.ampDeg;
      for (let i = 0; i < 30; i++) {
        const mid = Math.sqrt(lo * hi);
        if (pioPoint(p, mid, g).gain >= 1) hi = mid;
        else lo = mid;
      }
      onsetDeg = hi;
      onsetW = pioPoint(p, hi, g).w;
    }
  }
  return {
    linearGain: small.gain,
    linearW: small.w,
    onsetDeg,
    onsetW,
    rateLimitDeg: (p.aircraft.servoRate * RAD) / small.w / RAD,
    curve,
  };
}

/** The phase of a complex number, degrees (exported for the chart's readout). */
export const phaseDeg = (z: Complex): number => (carg(z) * 180) / Math.PI;

/** Whether a PIO is possible at all within the elevator's travel: N·G reaches 1 at some amplitude. */
export function pioPossible(p: Params, g = pilotLoop(p)): boolean {
  const max = p.aircraft.elevatorMax;
  for (let i = 0; i <= 16; i++)
    if (pioPoint(p, 0.5 * (max / 0.5) ** (i / 16), g).gain >= 1) return true;
  return false;
}

/**
 * The slowest elevator rate, °/s, at which no PIO is possible within the travel (by bisection).
 * For a pure rate limiter N depends on A·ω/R alone, so the onset amplitude grows in proportion to
 * the rate R: about R·(travel/onset).
 */
export function pioFreeRate(p: Params): number {
  const g = pilotLoop(p);
  const at = (r: number) => ({ ...p, aircraft: { ...p.aircraft, servoRate: r } });
  let lo = 1;
  let hi = 400;
  if (pioPossible(at(hi), g)) return Infinity;
  for (let i = 0; i < 12; i++) {
    const mid = Math.sqrt(lo * hi);
    if (pioPossible(at(mid), g)) lo = mid;
    else hi = mid;
  }
  return hi;
}
