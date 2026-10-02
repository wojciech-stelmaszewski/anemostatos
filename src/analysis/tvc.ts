// The thrust-vector-control loop of Chapter L (docs/aerospace-gnc.md §4): the planar rocket
// linearised by the generic `linearize`, the controller's own discrete model, and the sensor
// delay. The loop is broken at the gimbal command, where it is single-input single-output.
import { PHYS_DT } from '@/engine/simulation';
import { tvcControllerModel } from '@/control/tvc';
import { C_ONE, cabs, cadd, cdiv, cis, cmul, cneg, type Complex } from '@/math/complex';
import { evalAt, type Lti } from '@/math/lti';
import { logspace, margins, type Margins } from '@/math/margins';
import { eigenvalues, type Mat } from '@/math/mat';
import type { Params } from '@/sim/params';
import { modeSlope, tvcRocket } from '@/sim/vehicles/tvc';
import { linearize } from './linearize';

/** The four measured outputs of the plant (pitch and rate at the gyro, drift, drift rate). */
function plantModel(p: Params): Lti {
  const lin = linearize(tvcRocket, p, tvcRocket.trim(p));
  const n = lin.a.length;
  const k = modeSlope(p.tvc.gyroStation, p.tvc.length);
  const row = (entries: Record<number, number>) =>
    Array.from({ length: n }, (_, j) => entries[j] ?? 0);
  // State order: x, u, θ, ω, δ, η, η̇, s, ṡ.
  const c = [row({ 2: 1, 5: k }), row({ 3: 1, 6: k }), row({ 0: 1 }), row({ 1: 1 })];
  // States that cannot act on the loop are left out, so their poles on the unit circle do not
  // blur the stability test: the drift x when nothing feeds it back, the slosh with no mass.
  const keep = Array.from({ length: n }, (_, i) => i).filter(
    (i) => (i !== 0 || p.control.tvc.kx !== 0) && (i < 7 || p.tvc.sloshMass > 0),
  );
  return {
    a: keep.map((i) => keep.map((j) => lin.a[i]![j]!)),
    b: keep.map((i) => lin.b[i]!),
    c: c.map((r) => keep.map((j) => r[j]!)),
    d: [[0], [0], [0], [0]],
    dt: PHYS_DT,
  };
}

export interface TvcLoop {
  plant: Lti;
  controller: Lti;
  /** Sensor delay, physics steps. */
  delay: number;
}

export function tvcLoop(p: Params): TvcLoop {
  return {
    plant: plantModel(p),
    controller: tvcControllerModel(p.control.tvc),
    delay: Math.max(0, Math.round(p.control.tvc.delayMs / 1000 / PHYS_DT)),
  };
}

/** Open loop L(jω) broken at the gimbal command: −K(z)·z⁻ᵈ·G(z), negative-feedback convention. */
export function tvcLoopGain(m: TvcLoop, w: number): Complex {
  const z = cis(w * PHYS_DT);
  const g = evalAt(m.plant, z); // 4 × 1
  const k = evalAt(m.controller, z)[0]!; // 1 × 6
  let sum: Complex = { re: 0, im: 0 };
  for (let i = 0; i < 4; i++) sum = cadd(sum, cmul(k[i]!, g[i]![0]!));
  const lag = cis(-w * PHYS_DT * m.delay);
  return cneg(cmul(lag, sum));
}

/** Closed-loop state matrix with the controller's output scaled by `k` (the delay at the input). */
function closedLoop(m: TvcLoop, k: number): Mat {
  const P = m.plant;
  const K = m.controller;
  const np = P.a.length;
  const nc = K.a.length;
  const d = m.delay;
  const N = np + d + nc;
  const M: Mat = Array.from({ length: N }, () => new Array<number>(N).fill(0));
  const iD = np; // delay chain z1 … zd: the gimbal command d steps ago is z_d
  const iC = np + d;
  // u = k·(Cc·xc + Dc·Cp·xp): a row over the whole state.
  const uRow = new Array<number>(N).fill(0);
  for (let j = 0; j < nc; j++) uRow[iC + j] = k * K.c[0]![j]!;
  for (let a = 0; a < 4; a++)
    for (let j = 0; j < np; j++) uRow[j] = uRow[j]! + k * K.d[0]![a]! * P.c[a]![j]!;
  // The plant: driven by z_d, or by u directly without a delay.
  for (let i = 0; i < np; i++) {
    for (let j = 0; j < np; j++) M[i]![j] = P.a[i]![j]!;
    const b = P.b[i]![0]!;
    if (d > 0) M[i]![iD + d - 1] = M[i]![iD + d - 1]! + b;
    else for (let j = 0; j < N; j++) M[i]![j] = M[i]![j]! + b * uRow[j]!;
  }
  if (d > 0) {
    for (let j = 0; j < N; j++) M[iD]![j] = uRow[j]!;
    for (let h = 1; h < d; h++) M[iD + h]![iD + h - 1] = 1;
  }
  for (let i = 0; i < nc; i++) {
    for (let j = 0; j < nc; j++) M[iC + i]![iC + j] = K.a[i]![j]!;
    for (let a = 0; a < 4; a++)
      for (let j = 0; j < np; j++) M[iC + i]![j] = M[iC + i]![j]! + K.b[i]![a]! * P.c[a]![j]!;
  }
  return M;
}

/** Largest closed-loop pole magnitude with the loop gain scaled by `k`; below 1 is stable. */
export function spectralRadius(m: TvcLoop, k = 1): number {
  return Math.max(...eigenvalues(closedLoop(m, k)).map((z) => Math.hypot(z.re, z.im)));
}

/** Closed-loop poles as s-plane equivalents, s = ln(z)/dt. */
export function tvcPoles(m: TvcLoop): Complex[] {
  return eigenvalues(closedLoop(m, 1))
    .filter((z) => Math.hypot(z.re, z.im) > 1e-6)
    .map((z) => ({
      re: Math.log(Math.hypot(z.re, z.im)) / PHYS_DT,
      im: Math.atan2(z.im, z.re) / PHYS_DT,
    }));
}

const stableAt = (m: TvcLoop, k: number) => spectralRadius(m, k) < 1 - 1e-12;

/**
 * The gain margins both ways, as factors: the loop stays stable for loop gains in (low, high).
 * Found on the closed loop itself, so the crossing at zero frequency of an unstable airframe
 * counts. NaN if the loop is unstable as it stands.
 */
export function gainRange(m: TvcLoop): { low: number; high: number } {
  if (!stableAt(m, 1)) return { low: NaN, high: NaN };
  const edge = (ok: number, bad: number) => {
    for (let i = 0; i < 40; i++) {
      const mid = Math.sqrt(ok * bad);
      if (stableAt(m, mid)) ok = mid;
      else bad = mid;
    }
    return Math.sqrt(ok * bad);
  };
  let high = Infinity;
  for (let k = 1.25; k < 1e4; k *= 1.25)
    if (!stableAt(m, k)) {
      high = edge(k / 1.25, k);
      break;
    }
  let low = 0;
  for (let k = 0.8; k > 1e-4; k *= 0.8)
    if (!stableAt(m, k)) {
      low = edge(k / 0.8, k);
      break;
    }
  return { low, high };
}

export interface TvcAnalysis {
  fHz: number[];
  l: Complex[];
  margins: Margins;
  /** Gain margins as factors and in dB (the lower one as a positive number of dB to lose). */
  low: number;
  high: number;
  lowDb: number;
  highDb: number;
  stable: boolean;
  /** |L| at the bending frequency, dB, and the bending frequency, Hz. */
  bendDb: number;
  bendHz: number;
}

export function analyseTvc(p: Params): TvcAnalysis {
  const m = tvcLoop(p);
  const fHz = logspace(0.02, 50, 700);
  const l = fHz.map((f) => tvcLoopGain(m, 2 * Math.PI * f));
  const g = gainRange(m);
  const fb = p.tvc.bendHz;
  return {
    fHz,
    l,
    margins: margins(
      fHz.map((f) => 2 * Math.PI * f),
      l,
    ),
    low: g.low,
    high: g.high,
    lowDb: -20 * Math.log10(g.low),
    highDb: 20 * Math.log10(g.high),
    stable: !Number.isNaN(g.low),
    bendDb: 20 * Math.log10(cabs(tvcLoopGain(m, 2 * Math.PI * fb))),
    bendHz: fb,
  };
}

/** Sensitivity S = 1/(1 + L) at the gimbal command. */
export const tvcSensitivity = (l: Complex): Complex => cdiv(C_ONE, cadd(C_ONE, l));

/**
 * The smallest pitch gain kp that holds the airframe, with everything else as it is: the lower
 * edge of the stable range, found on the closed loop (lesson IV.7). NaN if no kp up to 50 does.
 */
export function minPitchGain(p: Params): number {
  const stableWith = (kp: number) => {
    const q = { ...p, control: { ...p.control, tvc: { ...p.control.tvc, kp } } };
    return spectralRadius(tvcLoop(q)) < 1 - 1e-12;
  };
  let hi = 0.05;
  while (hi < 50 && !stableWith(hi)) hi *= 1.25;
  if (hi >= 50) return NaN;
  let lo = hi / 1.25;
  if (stableWith(lo)) lo = 0;
  for (let i = 0; i < 40; i++) {
    const mid = (lo + hi) / 2;
    if (stableWith(mid)) hi = mid;
    else lo = mid;
  }
  return (lo + hi) / 2;
}

/**
 * The right-half-plane zero from the gimbal to the drift of a hovering rocket, rad/s (lesson
 * IV.10): the gimbal's side force T·δ and the tilt it causes, T·θ, cancel at √(T·l/I).
 */
export const wrongWayZero = (p: Params): number =>
  Math.sqrt((p.tvc.thrust * p.tvc.gimbalArm) / p.tvc.inertia);
