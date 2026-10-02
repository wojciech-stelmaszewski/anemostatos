// Linear analysis of the aircraft of Chapter M (docs/aerospace-gnc.md §4): its modes, the
// handling-qualities point of its short period, and the margins of the pitch autopilot, all from
// the generic linearisation of the simulator's own step around a level-flight trim.
import { scheduledGain, type AutopilotParams } from '@/control/autopilot';
import { cabs, cdiv, csub, cx, type Complex } from '@/math/complex';
import { eigenvalues, type Mat } from '@/math/mat';
import { logspace, margins, type Margins } from '@/math/margins';
import { GRAVITY, type Params } from '@/sim/params';
import { aircraft, trimAircraft, type AircraftParams } from '@/sim/vehicles/aircraft';
import { linearize } from './linearize';

/** The states kept: V, α, q, θ, h (the density gradient shortens the phugoid), the elevator and the thrust. */
const KEEP = [0, 1, 2, 3, 4, 6, 7];
const I_THETA = 3;
const I_Q = 2;

export interface AircraftLinear {
  /** Continuous-time A (7 × 7) and the elevator column of B, from the 1 ms step. */
  a: Mat;
  b: number[];
  qbar: number;
  /** Normal acceleration per angle of attack, g/rad: q̄·S·C_Lα/(m·g). */
  nAlpha: number;
  feasible: boolean;
}

/** Linear model at level flight with speed `V` at the start altitude. */
export function aircraftLinear(a: AircraftParams, V: number): AircraftLinear {
  const trim = trimAircraft(a, V, a.altitude);
  const p = { aircraft: a } as Params;
  const lin = linearize(aircraft, p, { state: trim.state, input: trim.input }, 0.001, 1e-6, 1e-5);
  const dt = lin.dt;
  const A = KEEP.map((i) => KEEP.map((j) => (lin.a[i]![j]! - (i === j ? 1 : 0)) / dt));
  const b = KEEP.map((i) => lin.b[i]![0]! / dt);
  return {
    a: A,
    b,
    qbar: trim.qbar,
    nAlpha: (trim.qbar * a.wingArea * a.clAlpha) / (a.mass * GRAVITY),
    feasible: trim.feasible,
  };
}

/** The feedback row of the autopilot on the kept states: δe = K·x (θ_ref fixed). */
export function autopilotRow(ap: AutopilotParams, a: AircraftParams, qbar: number): number[] {
  const g = scheduledGain(ap, a, qbar);
  const k = KEEP.map(() => 0);
  k[I_THETA] = g * ap.kTheta;
  k[I_Q] = g;
  return k;
}

/**
 * The closed loop δe = K·x(t − τ), with the delay as a first-order Padé approximant
 * (1 − sτ/2)/(1 + sτ/2) = −1 + 2a/(s + a), a = 2/τ: one extra state z, ż = −a·z + K·x.
 */
function closed(m: AircraftLinear, k: number[], tau: number): Mat {
  const n = m.a.length;
  if (!(tau > 0) || k.every((v) => v === 0))
    return m.a.map((row, i) => row.map((v, j) => v + m.b[i]! * k[j]!));
  const a = 2 / tau;
  const out: Mat = m.a.map((row, i) => [
    ...row.map((v, j) => v - m.b[i]! * k[j]!),
    2 * a * m.b[i]!,
  ]);
  out.push([...k, -a]);
  return out.slice(0, n + 1);
}

export interface Mode {
  pole: Complex;
  /** Natural frequency, rad/s; damping ratio; period of the oscillation, s. */
  wn: number;
  zeta: number;
  period: number;
}

const mode = (pole: Complex): Mode => {
  const wn = cabs(pole);
  return { pole, wn, zeta: -pole.re / wn, period: (2 * Math.PI) / Math.abs(pole.im) };
};

export interface Modes {
  poles: Complex[];
  shortPeriod: Mode | null;
  phugoid: Mode | null;
}

/** The poles, and the two oscillatory pairs: the fastest is the short period, the slowest the phugoid. */
export function modesOf(a: Mat): Modes {
  const poles = eigenvalues(a);
  const pairs = poles.filter((p) => p.im > 1e-6).sort((x, y) => cabs(y) - cabs(x));
  return {
    poles,
    shortPeriod: pairs.length >= 1 ? mode(pairs[0]!) : null,
    phugoid: pairs.length >= 2 ? mode(pairs[pairs.length - 1]!) : null,
  };
}

/** Modes of the aircraft at its start speed, with the autopilot (`withAutopilot`) or bare. */
export function aircraftModes(p: Params, withAutopilot = true, V = p.aircraft.speed): Modes {
  const m = aircraftLinear(p.aircraft, V);
  const k = withAutopilot ? autopilotRow(p.autopilot, p.aircraft, m.qbar) : KEEP.map(() => 0);
  return modesOf(closed(m, k, p.autopilot.delayMs / 1000));
}

/** Lanchester's phugoid: period π·√2·V/g, s. */
export const lanchesterPeriod = (V: number): number => (Math.PI * Math.SQRT2 * V) / GRAVITY;

/**
 * The handling-qualities point of the short period [MIL-F-8785C §3.2.2, Category A]: damping
 * ratio and the control anticipation parameter CAP = ω²/(n/α), 1/(g·s²).
 */
export interface HqPoint {
  zeta: number;
  wn: number;
  cap: number;
  level: 1 | 2 | 3;
}

/** Level 1 and level 2 boxes of Category A: ζ and CAP. */
export const HQ_LEVELS = {
  1: { zeta: [0.35, 1.3], cap: [0.28, 3.6] },
  2: { zeta: [0.25, 2.0], cap: [0.16, 10] },
} as const;

export function hqLevel(zeta: number, cap: number): 1 | 2 | 3 {
  for (const l of [1, 2] as const) {
    const b = HQ_LEVELS[l];
    if (zeta >= b.zeta[0] && zeta <= b.zeta[1] && cap >= b.cap[0] && cap <= b.cap[1]) return l;
  }
  return 3;
}

export function hqPoint(p: Params, withAutopilot = true): HqPoint | null {
  const m = aircraftLinear(p.aircraft, p.aircraft.speed);
  const k = withAutopilot ? autopilotRow(p.autopilot, p.aircraft, m.qbar) : KEEP.map(() => 0);
  const sp = modesOf(closed(m, k, p.autopilot.delayMs / 1000)).shortPeriod;
  if (!sp) return null;
  const cap = (sp.wn * sp.wn) / m.nAlpha;
  return { zeta: sp.zeta, wn: sp.wn, cap, level: hqLevel(sp.zeta, cap) };
}

/** The least pure-damper gain (kθ = 0) that brings the short period into level 1, by bisection. */
export function leastDamperGain(p: Params): number {
  const at = (g: number) =>
    hqPoint({ ...p, autopilot: { ...p.autopilot, gain: g, kTheta: 0, schedule: 'none' } })!;
  if (at(0).level === 1) return 0;
  let lo = 0;
  let hi = 0.05;
  while (at(hi).level !== 1 && hi < 5) hi *= 2;
  for (let i = 0; i < 50; i++) {
    const mid = (lo + hi) / 2;
    if (at(mid).level === 1) hi = mid;
    else lo = mid;
  }
  return hi;
}

/**
 * The pitch loop broken at the elevator command: L(jω) = −K(jωI − A)⁻¹B, negative-feedback
 * convention, and its margins.
 */
export function pitchLoopMargins(
  p: Params,
  V: number,
  w = logspace(0.5, 200, 400),
): Margins & { qbar: number } {
  const m = aircraftLinear(p.aircraft, V);
  const k = autopilotRow(p.autopilot, p.aircraft, m.qbar);
  const n = m.a.length;
  const tau = p.autopilot.delayMs / 1000;
  const l = w.map((om) => {
    // Solve (jωI − A)·x = B for x, complex Gaussian elimination.
    const M: Complex[][] = m.a.map((row, i) =>
      row.map((v, j) => csub(i === j ? cx(0, om) : cx(0), cx(v))),
    );
    const x: Complex[] = m.b.map((v) => cx(v));
    for (let c = 0; c < n; c++) {
      let piv = c;
      for (let r = c + 1; r < n; r++) if (cabs(M[r]![c]!) > cabs(M[piv]![c]!)) piv = r;
      [M[c], M[piv]] = [M[piv]!, M[c]!];
      [x[c], x[piv]] = [x[piv]!, x[c]!];
      for (let r = c + 1; r < n; r++) {
        const f = cdiv(M[r]![c]!, M[c]![c]!);
        for (let j = c; j < n; j++)
          M[r]![j] = csub(M[r]![j]!, {
            re: f.re * M[c]![j]!.re - f.im * M[c]![j]!.im,
            im: f.re * M[c]![j]!.im + f.im * M[c]![j]!.re,
          });
        x[r] = csub(x[r]!, {
          re: f.re * x[c]!.re - f.im * x[c]!.im,
          im: f.re * x[c]!.im + f.im * x[c]!.re,
        });
      }
    }
    for (let r = n - 1; r >= 0; r--) {
      let acc = x[r]!;
      for (let j = r + 1; j < n; j++)
        acc = csub(acc, {
          re: M[r]![j]!.re * x[j]!.re - M[r]![j]!.im * x[j]!.im,
          im: M[r]![j]!.re * x[j]!.im + M[r]![j]!.im * x[j]!.re,
        });
      x[r] = cdiv(acc, M[r]![r]!);
    }
    let re = 0;
    let im = 0;
    for (let j = 0; j < n; j++) {
      re -= k[j]! * x[j]!.re;
      im -= k[j]! * x[j]!.im;
    }
    // The delay of the sensors and the computation: e^(−jωτ).
    const ph = -om * tau;
    return { re: re * Math.cos(ph) - im * Math.sin(ph), im: re * Math.sin(ph) + im * Math.cos(ph) };
  });
  return { ...margins(w, l), qbar: m.qbar };
}

/** The envelope of lesson IV.13: speeds at the start altitude. */
export const ENVELOPE = { vMin: 80, vMax: 160 } as const;

/** Phase margin and crossover of the pitch loop across the envelope. */
export function envelopeMargins(p: Params, count = 9) {
  const out: { V: number; pmDeg: number; wc: number; qbar: number }[] = [];
  for (let i = 0; i < count; i++) {
    const V = ENVELOPE.vMin + ((ENVELOPE.vMax - ENVELOPE.vMin) * i) / (count - 1);
    const m = pitchLoopMargins(p, V);
    out.push({ V, pmDeg: m.pmDeg, wc: m.wc, qbar: m.qbar });
  }
  return out;
}

/** The requirement of lesson IV.13: robust everywhere and quick everywhere. */
export const PITCH_SPEC = { pmDeg: 45, wc: 4 } as const;

/** Whether the pitch loop meets `PITCH_SPEC` at every speed of the envelope, and the worst values. */
export function meetsPitchSpec(p: Params): { ok: boolean; worstPm: number; slowestWc: number } {
  const rows = envelopeMargins(p, 9);
  const worstPm = Math.min(...rows.map((r) => r.pmDeg));
  const slowestWc = Math.min(...rows.map((r) => (Number.isFinite(r.wc) ? r.wc : 0)));
  return { ok: worstPm >= PITCH_SPEC.pmDeg && slowestWc >= PITCH_SPEC.wc, worstPm, slowestWc };
}
