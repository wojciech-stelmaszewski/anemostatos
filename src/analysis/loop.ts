// The L1 loop as one object: plant, controller, sampling and delay (docs/analysis.md §3.2).
// The plant runs at the physics step, the controller every `steps` of them and holds its output
// in between; the frequency responses below are those of that two-rate loop, as the probe of the
// simulator measures them.
import { PHYS_DT } from '@/engine/simulation';
import {
  C_ONE,
  C_ZERO,
  cadd,
  cdiv,
  cis,
  cmul,
  cneg,
  cscale,
  csub,
  type Complex,
} from '@/math/complex';
import { evalAt, type Lti } from '@/math/lti';
import { eigenvalues, eye, mul, toContinuous, zeros, add, type Mat } from '@/math/mat';
import type { Params } from '@/sim/params';
import { controlPeriod, CTRL_IN, l1ControllerModel } from './controllers';
import { l1Plant, PLANT_OUT } from './plant';

export interface LoopModel {
  /** Discrete at the physics step. */
  plant: Lti;
  /** Discrete at the controller period. */
  controller: Lti;
  /** Physics steps per controller sample. */
  steps: number;
  /** Controller period, s. */
  T: number;
  /** Sensor delay in physics steps. */
  delay: number;
}

/** The loop of the selected L1 controller, or null if it has no linear model. */
export function l1Loop(p: Params): LoopModel | null {
  const controller = l1ControllerModel(p);
  if (!controller) return null;
  const { steps, T } = controlPeriod(p);
  return {
    plant: l1Plant(p),
    controller,
    steps,
    T,
    delay: Math.max(0, Math.round(p.sensors.delayMs / 1000 / PHYS_DT)),
  };
}

/**
 * Sample every `steps` physics steps and hold: the gain of that operation at the fundamental,
 * (1/n)·(1 − e^{−jΩn})/(1 − e^{−jΩ}) with Ω = ω·dt. About half a controller period of delay.
 */
function holdGain(w: number, steps: number): Complex {
  const om = w * PHYS_DT;
  if (Math.abs(om) < 1e-12 || steps === 1) return C_ONE;
  return cscale(cdiv(csub(C_ONE, cis(-om * steps)), csub(C_ONE, cis(-om))), 1 / steps);
}

interface Parts {
  /** Open loop at the plant input, −Uc/U. */
  l: Complex;
  /** From the plant input to the altitude. */
  py: Complex;
  /** From the reference to the held controller output, with the loop open. */
  kr: Complex;
}

function parts(m: LoopModel, w: number): Parts {
  const pz = evalAt(m.plant, cis(w * PHYS_DT)); // 3 × 1
  const kz = evalAt(m.controller, cis(w * m.T))[0]!; // 1 × 4
  const hold = holdGain(w, m.steps);
  const lag = cis(-w * PHYS_DT * m.delay);
  let sum: Complex = C_ZERO;
  for (const [ci, pi] of [
    [CTRL_IN.y, PLANT_OUT.y],
    [CTRL_IN.v, PLANT_OUT.v],
    [CTRL_IN.thrust, PLANT_OUT.thrust],
  ] as const)
    sum = cadd(sum, cmul(kz[ci]!, pz[pi]![0]!));
  return {
    l: cneg(cmul(hold, cmul(lag, sum))),
    py: pz[PLANT_OUT.y]![0]!,
    kr: cmul(hold, kz[CTRL_IN.r]!),
  };
}

/** Open-loop frequency response L(jω) with the loop broken at the plant input. */
export const loopGain = (m: LoopModel, w: number): Complex => parts(m, w).l;

/** Sensitivity S = 1/(1 + L) and complementary sensitivity T = L/(1 + L) at the plant input. */
export function sensitivities(m: LoopModel, w: number): { s: Complex; t: Complex } {
  const l = loopGain(m, w);
  const s = cdiv(C_ONE, cadd(C_ONE, l));
  return { s, t: cmul(l, s) };
}

/** Closed-loop response from the reference to the altitude, Y/R. */
export function referenceResponse(m: LoopModel, w: number): Complex {
  const q = parts(m, w);
  return cdiv(cmul(q.py, q.kr), cadd(C_ONE, q.l));
}

/**
 * Bandwidth of the closed loop: the lowest frequency, Hz, at which |Y/R| falls below −3 dB.
 * NaN if it never does below the controller's Nyquist frequency.
 */
export function bandwidthHz(m: LoopModel): number {
  const top = 0.5 / m.T;
  const level = Math.SQRT1_2;
  let f0 = 0.01;
  const first = referenceResponse(m, 2 * Math.PI * f0);
  let g0 = Math.hypot(first.re, first.im);
  for (let f = f0 * 1.02; f < top; f *= 1.02) {
    const v = referenceResponse(m, 2 * Math.PI * f);
    const g = Math.hypot(v.re, v.im);
    if (g0 >= level && g < level) {
      // Interpolate in log-frequency between the two grid points.
      const k = (g0 - level) / (g0 - g);
      return Math.exp(Math.log(f0) + k * Math.log(f / f0));
    }
    f0 = f;
    g0 = g;
  }
  return NaN;
}

/**
 * Closed-loop poles in the z-plane of the controller's sample period.
 *
 * The plant is lifted to that period. With a sensor delay of d = q·n + rem physics steps the
 * lifted state is taken rem steps before each controller sample, ξ_k = x_{kn − rem}, so that the
 * measurement used at sample k is exactly C·ξ_{k−q}:
 *   ξ_{k+1} = Aⁿ·ξ_k + A^{n−rem}·S_rem·B·u_{k−1} + S_{n−rem}·B·u_k,   S_j = Σ_{i<j} Aⁱ.
 */
export function closedLoopPolesZ(m: LoopModel): Complex[] {
  const A = m.plant.a;
  const B = m.plant.b;
  const C = m.plant.c;
  const np = A.length;
  const n = m.steps;
  const q = Math.floor(m.delay / n);
  const rem = m.delay - q * n;
  const pow = (k: number): Mat => {
    let r = eye(np);
    for (let i = 0; i < k; i++) r = mul(r, A);
    return r;
  };
  const sumPow = (k: number): Mat => {
    let s = zeros(np);
    let r = eye(np);
    for (let i = 0; i < k; i++) {
      s = add(s, r);
      r = mul(r, A);
    }
    return s;
  };
  const An = pow(n);
  const bPrev = mul(mul(pow(n - rem), sumPow(rem)), B); // np × 1, acts on u_{k−1}
  const bNow = mul(sumPow(n - rem), B); // np × 1, acts on u_k

  const Kc = m.controller;
  const nc = Kc.a.length;
  // Controller inputs from the plant outputs (the reference is zero for the poles).
  const pick = [PLANT_OUT.y, PLANT_OUT.v, PLANT_OUT.thrust];
  const cols = [CTRL_IN.y, CTRL_IN.v, CTRL_IN.thrust];
  // State: [ξ_k, u_{k−1}, ξ_{k−1} … ξ_{k−q}, x_c]
  const iXi = 0;
  const iU = np;
  const iHist = np + 1;
  const iC = iHist + q * np;
  const N = iC + nc;
  const M = zeros(N);
  // w_k = C·ξ_{k−q}: where the delayed state sits.
  const iW = q === 0 ? iXi : iHist + (q - 1) * np;
  // u_k = Cc·x_c + Dw·C·ξ_{k−q}, as a row over the whole state.
  const uRow = new Array<number>(N).fill(0);
  for (let j = 0; j < nc; j++) uRow[iC + j] = Kc.c[0]![j]!;
  for (let a = 0; a < 3; a++)
    for (let j = 0; j < np; j++)
      uRow[iW + j] = uRow[iW + j]! + Kc.d[0]![cols[a]!]! * C[pick[a]!]![j]!;
  // ξ⁺
  for (let i = 0; i < np; i++) {
    for (let j = 0; j < np; j++) M[i]![iXi + j] = An[i]![j]!;
    M[i]![iU] = M[i]![iU]! + bPrev[i]![0]!;
    for (let j = 0; j < N; j++) M[i]![j] = M[i]![j]! + bNow[i]![0]! * uRow[j]!;
  }
  // u_{k−1}⁺ = u_k
  for (let j = 0; j < N; j++) M[iU]![j] = uRow[j]!;
  // History shift.
  for (let h = 0; h < q; h++)
    for (let i = 0; i < np; i++)
      M[iHist + h * np + i]![(h === 0 ? iXi : iHist + (h - 1) * np) + i] = 1;
  // x_c⁺ = Ac·x_c + Bw·C·ξ_{k−q}
  for (let i = 0; i < nc; i++) {
    for (let j = 0; j < nc; j++) M[iC + i]![iC + j] = Kc.a[i]![j]!;
    for (let a = 0; a < 3; a++)
      for (let j = 0; j < np; j++)
        M[iC + i]![iW + j] = M[iC + i]![iW + j]! + Kc.b[i]![cols[a]!]! * C[pick[a]!]![j]!;
  }
  return eigenvalues(M);
}

/**
 * Closed-loop poles as s-plane equivalents, s = ln(z)/T. Poles at z = 0 (pure delays and states
 * that are overwritten every sample) have no finite equivalent and are left out.
 */
export const closedLoopPoles = (m: LoopModel): Complex[] =>
  closedLoopPolesZ(m)
    .filter((z) => Math.hypot(z.re, z.im) > 1e-6)
    .map((z) => toContinuous(z, m.T));

/** True if every closed-loop pole lies inside the unit circle. */
export const isLoopStable = (m: LoopModel): boolean =>
  closedLoopPolesZ(m).every((z) => Math.hypot(z.re, z.im) < 1 - 1e-9);
