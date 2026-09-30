// The roll–pitch rate loop of the quadrotor as a two-input, two-output loop
// (docs/analysis.md §4, Chapter H): measured by flying two torque sweeps, modelled in the
// frequency domain, and judged by margins that hold for both channels at once.
import { PHYS_DT } from '@/engine/simulation';
import {
  C_ONE,
  cabs,
  cadd,
  cdiv,
  cis,
  cmul,
  cneg,
  cscale,
  csub,
  cx,
  type Complex,
} from '@/math/complex';
import { evalAt } from '@/math/lti';
import { logspace, margins, type Margins } from '@/math/margins';
import type { Params } from '@/sim/params';
import { CTRL_IN, pidLti } from './controllers';
import { Sweep, type SweepOptions } from './sweep';

/** A 2 × 2 complex matrix, rows first: [[m11, m12], [m21, m22]]. */
export type C2 = [[Complex, Complex], [Complex, Complex]];

export interface MimoPoint {
  fHz: number;
  /** Sensitivity and complementary sensitivity at the plant input (roll and pitch torque). */
  s: C2;
  t: C2;
}

const det = (m: C2): Complex => csub(cmul(m[0][0], m[1][1]), cmul(m[0][1], m[1][0]));
const inv = (m: C2): C2 => {
  const d = det(m);
  return [
    [cdiv(m[1][1], d), cneg(cdiv(m[0][1], d))],
    [cneg(cdiv(m[1][0], d)), cdiv(m[0][0], d)],
  ];
};

/** Singular values of a 2 × 2 complex matrix, largest first. */
export function singularValues(m: C2): [number, number] {
  const a = cabs(m[0][0]) ** 2 + cabs(m[1][0]) ** 2;
  const d = cabs(m[0][1]) ** 2 + cabs(m[1][1]) ** 2;
  // b = conj(m11)·m12 + conj(m21)·m22
  const b = Math.hypot(
    m[0][0].re * m[0][1].re +
      m[0][0].im * m[0][1].im +
      m[1][0].re * m[1][1].re +
      m[1][0].im * m[1][1].im,
    m[0][0].re * m[0][1].im -
      m[0][0].im * m[0][1].re +
      m[1][0].re * m[1][1].im -
      m[1][0].im * m[1][1].re,
  );
  const mid = (a + d) / 2;
  const r = Math.hypot((a - d) / 2, b);
  return [Math.sqrt(mid + r), Math.sqrt(Math.max(mid - r, 0))];
}

/** Hold of a signal that is updated every `n` physics steps: its gain at the fundamental. */
function hold(w: number, n: number): Complex {
  const om = w * PHYS_DT;
  if (n <= 1 || Math.abs(om) < 1e-12) return C_ONE;
  return cscale(cdiv(csub(C_ONE, cis(-om * n)), csub(C_ONE, cis(-om))), 1 / n);
}
const steps = (hz: number) => Math.max(1, Math.round(1 / (Math.max(hz, 0.1) * PHYS_DT)));

/**
 * The loop at the torque input, L = a·R(θ) + b·1, as its two coefficients: `a` is the rate-gyro
 * path (turned by the uncorrected mounting angle θ), `b` the attitude path with the velocity and
 * position loops behind it (not turned: the attitude is known in the true body axes).
 */
function paths(p: Params, w: number): { a: Complex; b: Complex; theta: number } {
  const c = p.control.l3;
  const dt = PHYS_DT;
  const z = cis(w * dt);
  const zm1 = csub(z, C_ONE);
  const J = p.drone.inertia.x;
  const Jm = J * p.control.model.inertiaScale;
  const lam = 1 - Math.exp(-dt / Math.max(p.drone.motorTau, 1e-4));
  // Plant: torque command → body rate, one physics step at a time (src/sim/dynamics.ts).
  const plant = cdiv(
    cscale(z, (dt / J) * lam),
    cmul(cadd(zm1, cx(lam)), cadd(zm1, cx((dt * p.drone.angularDamping) / J))),
  );
  const delay = cis(-w * dt * Math.max(0, Math.round(p.sensors.delayMs / 1000 / dt)));
  // Rate PID (deg units cancel): from the measured rate and from the rate setpoint.
  const nRate = steps(c.hzRate);
  const rate = evalAt(pidLti(c.rateRP, nRate * dt), cis(w * nRate * dt))[0]!;
  const hRate = hold(w, nRate);
  const cy = cmul(hRate, rate[CTRL_IN.y]!);
  const cr = cmul(hRate, rate[CTRL_IN.r]!);
  // Attitude: angle = dt·z/(z − 1)·rate; setpoint of the rate loop = Katt·(angle_sp − angle).
  const angle = cdiv(cscale(z, dt), zm1);
  const hAtt = hold(w, steps(c.hzAtt));
  // Velocity and position loops: the tilt they ask for, as a multiple of the tilt there is.
  const nVel = steps(c.hzVel);
  const vel = evalAt(pidLti(c.velH, nVel * dt), cis(w * nVel * dt))[0]!;
  const vOverTilt = cdiv(cx(dt), zm1); // velocity per unit of g·tilt
  const xOverTilt = cmul(cdiv(cscale(z, dt), zm1), vOverTilt);
  const hPos = hold(w, steps(c.hzPos));
  const outer = cmul(
    hold(w, nVel),
    cadd(
      cmul(cneg(vel[CTRL_IN.y]!), vOverTilt),
      cmul(cmul(vel[CTRL_IN.r]!, cscale(hPos, c.posKpH)), xOverTilt),
    ),
  );
  const a = cneg(cmul(cscale(cmul(cy, delay), Jm), plant));
  const b = cmul(
    cscale(cmul(cmul(cr, cscale(hAtt, c.attKpRP)), cmul(delay, cadd(C_ONE, outer))), Jm),
    cmul(angle, plant),
  );
  return { a, b, theta: ((p.sensors.imuYawDeg - p.control.model.imuYawDeg) * Math.PI) / 180 };
}

/** Model of the loop matrix L(jω) at the torque input, for small motions around hover. */
export function rateLoopGain(p: Params, w: number): C2 {
  const { a, b, theta } = paths(p, w);
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  return [
    [cadd(cscale(a, c), b), cscale(a, s)],
    [cscale(a, -s), cadd(cscale(a, c), b)],
  ];
}

/**
 * The two eigenvalues of L(jω): λ± = a·e^{±jθ} + b. The loop matrix is normal, so the pair of
 * channels is equivalent to two separate loops, one per direction of rotation of the rate
 * vector, and the mounting angle is a phase shift of ±θ in the gyro path of each.
 */
export function eigenLoops(p: Params, w: number): [Complex, Complex] {
  const { a, b, theta } = paths(p, w);
  return [cadd(cmul(a, cis(theta)), b), cadd(cmul(a, cis(-theta)), b)];
}

/**
 * Margins for a change that is the same in both channels (a gain error of the whole vehicle,
 * a delay of the whole sensor): the smaller of the classical margins of the two eigen-loops.
 * Exact, by the generalised Nyquist criterion.
 */
export function simultaneousMargins(p: Params, w: readonly number[]): Margins {
  const m = ([0, 1] as const).map((i) =>
    margins(
      w as number[],
      w.map((x) => eigenLoops(p, x)[i]),
    ),
  );
  return m[0]!.pmDeg <= m[1]!.pmDeg ? m[0]! : m[1]!;
}

/** S = (1 + L)⁻¹ and T = 1 − S from the model. */
export function rateLoopModel(p: Params, fHz: number): MimoPoint {
  const l = rateLoopGain(p, 2 * Math.PI * fHz);
  const s = inv([
    [cadd(C_ONE, l[0][0]), l[0][1]],
    [l[1][0], cadd(C_ONE, l[1][1])],
  ]);
  return {
    fHz,
    s,
    t: [
      [csub(C_ONE, s[0][0]), cneg(s[0][1])],
      [cneg(s[1][0]), csub(C_ONE, s[1][1])],
    ],
  };
}

const SWEEP: SweepOptions = { amp: 0.01, settleS: 6, transientS: 3, minSeconds: 1.5 };

/** Two torque sweeps, one per axis, advanced together in slices. */
export class MimoSweep {
  private x: Sweep;
  private z: Sweep;
  constructor(p: Params, freqsHz: readonly number[], options: SweepOptions = {}) {
    this.x = new Sweep(p, freqsHz, { ...SWEEP, ...options, point: 'l3.torque.x' });
    this.z = new Sweep(p, freqsHz, { ...SWEEP, ...options, point: 'l3.torque.z' });
  }
  get done(): boolean {
    return this.x.done && this.z.done;
  }
  get progress(): number {
    return (this.x.progress + this.z.progress) / 2;
  }
  get crashed(): boolean {
    return this.x.crashed || this.z.crashed;
  }
  advance(maxSteps: number): boolean {
    this.x.advance(Math.ceil(maxSteps / 2));
    this.z.advance(Math.ceil(maxSteps / 2));
    return this.done;
  }
  /** The points both sweeps have finished. */
  get points(): MimoPoint[] {
    const n = Math.min(this.x.points.length, this.z.points.length);
    const out: MimoPoint[] = [];
    for (let i = 0; i < n; i++) {
      const px = this.x.points[i]!;
      const pz = this.z.points[i]!;
      out.push({
        fHz: px.fHz,
        s: [
          [px.sCol![0], pz.sCol![0]],
          [px.sCol![1], pz.sCol![1]],
        ],
        t: [
          [px.tCol![0], pz.tCol![0]],
          [px.tCol![1], pz.tCol![1]],
        ],
      });
    }
    return out;
  }
}

export function sweepMimo(
  p: Params,
  freqsHz: readonly number[],
  options: SweepOptions = {},
): MimoPoint[] {
  const s = new MimoSweep(p, freqsHz, options);
  while (!s.advance(2_000_000));
  return s.points;
}

/**
 * One loop at a time: the loop seen when channel `i` is opened while the other stays closed,
 * lᵢ = 1/Sᵢᵢ − 1, and its classical margins. This is what tuning one axis after the other tests.
 */
export function loopAtATime(points: readonly MimoPoint[], i: 0 | 1): Margins {
  return margins(
    points.map((p) => 2 * Math.PI * p.fHz),
    points.map((p) => csub(cdiv(C_ONE, p.s[i][i]), C_ONE)),
  );
}

export interface DiskMargin {
  /** Radius of the disk of perturbations the loop tolerates in every channel at once. */
  alpha: number;
  /** The gain may change by any factor in [1/gm, gm] in both channels… */
  gm: number;
  gmDb: number;
  /** …or the phase by up to this many degrees in both channels. */
  pmDeg: number;
  /** Frequency at which the margin is set, Hz. */
  fHz: number;
}

/**
 * The balanced disk margin for simultaneous, independent perturbations of all channels at the
 * plant input (Seiler, Packard & Gahinet 2020): α = 1/‖(S − T)/2‖∞, with the largest singular
 * value as the norm. A guarantee; for a loop matrix that is normal, as here, it is exact for
 * equal perturbations.
 */
export function diskMargin(points: readonly MimoPoint[]): DiskMargin {
  let peak = 0;
  let fHz = NaN;
  for (const p of points) {
    const half = (i: 0 | 1, j: 0 | 1) => cscale(csub(p.s[i][j], p.t[i][j]), 0.5);
    const sv = singularValues([
      [half(0, 0), half(0, 1)],
      [half(1, 0), half(1, 1)],
    ])[0];
    if (sv > peak) {
      peak = sv;
      fHz = p.fHz;
    }
  }
  const alpha = Math.min(peak > 0 ? 1 / peak : Infinity, 1.999);
  const gm = (2 + alpha) / (2 - alpha);
  return {
    alpha,
    gm,
    gmDb: 20 * Math.log10(gm),
    pmDeg: (2 * Math.atan(alpha / 2) * 180) / Math.PI,
    fHz,
  };
}

/** True when the roll–pitch loop of these parameters is the PID cascade the model describes. */
export const hasRateLoopModel = (p: Params): boolean =>
  p.sim.level === 3 &&
  p.control.l3.inner === 'pid' &&
  p.control.l3.outer === 'pid-cascade' &&
  p.control.l3.compensation === 'none' &&
  p.control.l3.safety === 'none';

export interface RateLoopAnalysis {
  fHz: number[];
  points: MimoPoint[];
  /** |S₁₁|: what a test of the roll channel alone sees. */
  s11: number[];
  /** Largest singular value of S: the worst direction of the two channels together. */
  sMax: number[];
  /** Margins when one channel is perturbed and the other left alone. */
  one: Margins;
  /** Margins when both channels change by the same amount. */
  both: Margins;
  /** Margins guaranteed for any mix of changes in the two channels. */
  disk: DiskMargin;
  stable: boolean;
}

const stableLoop = (m: Margins): boolean => m.pmDeg > 0 && m.gm > 1 && m.gmLow < 1;

/** The model's view of the roll–pitch loop (null when the controller is not the PID cascade). */
export function analyseRateLoop(p: Params, n = 500): RateLoopAnalysis | null {
  if (!hasRateLoopModel(p)) return null;
  const fMax = Math.min(100, p.control.l3.hzRate / 2);
  const fHz = logspace(0.2, fMax, n);
  const w = fHz.map((f) => 2 * Math.PI * f);
  const points = fHz.map((f) => rateLoopModel(p, f));
  const eig = ([0, 1] as const).map((i) =>
    margins(
      w,
      w.map((x) => eigenLoops(p, x)[i]),
    ),
  );
  return {
    fHz,
    points,
    s11: points.map((q) => cabs(q.s[0][0])),
    sMax: points.map((q) => singularValues(q.s)[0]),
    one: loopAtATime(points, 0),
    both: eig[0]!.pmDeg <= eig[1]!.pmDeg ? eig[0]! : eig[1]!,
    disk: diskMargin(points),
    stable: stableLoop(eig[0]!) && stableLoop(eig[1]!),
  };
}

/** Largest singular value of S over the points: the multivariable peak sensitivity. */
export const peakSensitivity = (points: readonly MimoPoint[]): number =>
  Math.max(...points.map((p) => singularValues(p.s)[0]));
