// Linear models of the L1 controllers (docs/analysis.md §3.2): each is the controller's own
// update rule, written as a discrete state-space system at the controller's sample period.
// They hold while nothing saturates. Inputs, in order: measured altitude, measured velocity,
// measured thrust (deviation from hover), reference. Output: the thrust command (deviation).
import { designLqr } from '@/control/lqr';
import { periodSteps } from '@/control/types';
import { PHYS_DT } from '@/engine/simulation';
import { esoMatrices } from '@/estimation/eso';
import type { Lti } from '@/math/lti';
import { lowPassAlpha } from '@/math/util';
import type { Params } from '@/sim/params';

export const CTRL_IN = { y: 0, v: 1, thrust: 2, r: 3 } as const;

/** The controller's sample period, in seconds and in physics steps. */
export const controlPeriod = (p: Params): { steps: number; T: number } => {
  const steps = periodSteps(p.control.rateHz, PHYS_DT);
  return { steps, T: steps * PHYS_DT };
};

/**
 * PID of src/control/pid.ts. States: the integral (if the I term is on), then the previous
 * differentiated signal and the filtered derivative (if the D term is on). A term that is
 * switched off leaves no state behind, so the model has no poles that the loop cannot move.
 */
function pidModel(p: Params, T: number): Lti {
  const g = p.control.alt;
  const kp = g.pOn ? g.kp : 0;
  const ki = g.iOn ? g.ki : 0;
  const kd = g.dOn ? g.kd : 0;
  const a = g.dFilterHz > 0 ? lowPassAlpha(g.dFilterHz, T) : 0;
  // The differentiated signal s: −y (derivative on the measurement) or r − y (on the error).
  const sr = g.derivativeOn === 'error' ? 1 : 0;
  const A: number[][] = [];
  const B: number[][] = [];
  const C: number[] = [];
  //           y     v  F  r
  const D = [-kp, 0, 0, kp];
  const n = (ki !== 0 ? 1 : 0) + (kd !== 0 ? 2 : 0);
  const row = () => new Array<number>(n).fill(0);
  let at = 0;
  if (ki !== 0) {
    // I⁺ = I + Ki·T·(r − y), and the output uses I⁺.
    const r = row();
    r[at] = 1;
    A.push(r);
    B.push([-ki * T, 0, 0, ki * T]);
    C[at] = 1;
    D[0] = D[0]! - ki * T;
    D[3] = D[3]! + ki * T;
    at += 1;
  }
  if (kd !== 0) {
    // sPrev⁺ = s;  dF⁺ = a·dF + (1 − a)·(s − sPrev)/T, and the output uses Kd·dF⁺.
    const c = (1 - a) / T;
    const r1 = row();
    A.push(r1);
    B.push([-1, 0, 0, sr]);
    const r2 = row();
    r2[at] = -c;
    r2[at + 1] = a;
    A.push(r2);
    B.push([-c, 0, 0, sr * c]);
    C[at] = -kd * c;
    C[at + 1] = kd * a;
    D[0] = D[0]! - kd * c;
    D[3] = D[3]! + kd * sr * c;
  }
  return { a: A, b: B, c: [Array.from({ length: n }, (_, i) => C[i] ?? 0)], d: [D], dt: T };
}

/** LQR / LQI of src/control/lqr.ts: u = −K·(e, v, [T], [ξ]) with e = y − r. */
function lqrModel(p: Params, T: number): Lti {
  const c = p.control.lqr;
  const k = designLqr(p, T).k;
  const k1 = k[0]!;
  const k2 = k[1]!;
  const k3 = c.lagState ? k[2]! : 0;
  const ki = c.integral ? k[c.lagState ? 3 : 2]! : 0;
  if (p.sensors.motorFeedback || !c.lagState) {
    // State: ξ (the integral of e), used before it is updated.
    return {
      a: [[c.integral ? 1 : 0]],
      b: [[c.integral ? T : 0, 0, 0, c.integral ? -T : 0]],
      c: [[-ki]],
      d: [[-k1, -k2, -k3, k1]],
      dt: T,
    };
  }
  // No motor telemetry: the thrust is a model estimate, est⁺ = (1 − α)·est + α·uPrev, used at once.
  const al = 1 - Math.exp(-T / Math.max(p.control.model.motorTau, 1e-3));
  // States: ξ, est, uPrev.   u = −k1·e − k2·v − k3·est⁺ − ki·ξ
  const cRow = [-ki, -k3 * (1 - al), -k3 * al];
  const dRow = [-k1, -k2, 0, k1];
  return {
    a: [[c.integral ? 1 : 0, 0, 0], [0, 1 - al, al], cRow],
    b: [[c.integral ? T : 0, 0, 0, c.integral ? -T : 0], [0, 0, 0, 0], dRow],
    c: [cRow],
    d: [dRow],
    dt: T,
  };
}

/** Linear ADRC of src/control/adrc.ts: the observer runs first, fed with the previous command. */
function adrcModel(p: Params, T: number): Lti {
  const { wc, wo } = p.control.adrc;
  const b0 = 1 / Math.max(p.control.model.mass, 0.05);
  const { ad, bd } = esoMatrices(wo, b0, T);
  // z⁺ = Ad·z + Bd·(uPrev, y);  u = (ωc²·(r − z1⁺) − 2ωc·z2⁺ − z3⁺)/b0
  const kz = [(-wc * wc) / b0, (-2 * wc) / b0, -1 / b0];
  const row = (col: number) => kz.reduce((s, kv, i) => s + kv * ad[i]![col]!, 0);
  const viaU = kz.reduce((s, kv, i) => s + kv * bd[i]![0]!, 0);
  const viaY = kz.reduce((s, kv, i) => s + kv * bd[i]![1]!, 0);
  const cRow = [row(0), row(1), row(2), viaU];
  const dRow = [viaY, 0, 0, (wc * wc) / b0];
  return {
    // States: z1, z2, z3, uPrev.
    a: [[...ad[0]!, bd[0]![0]!], [...ad[1]!, bd[1]![0]!], [...ad[2]!, bd[2]![0]!], cRow],
    b: [[bd[0]![1]!, 0, 0, 0], [bd[1]![1]!, 0, 0, 0], [bd[2]![1]!, 0, 0, 0], dRow],
    c: [cRow],
    d: [dRow],
    dt: T,
  };
}

/**
 * The linear model of the selected L1 controller, or null when there is none: MPC, L1 adaptive
 * control and loops with the Kalman filter in front are analysed by measurement only.
 */
export function l1ControllerModel(p: Params): Lti | null {
  if (p.sim.level !== 1 || p.control.l1.estimator !== 'none') return null;
  const { T } = controlPeriod(p);
  switch (p.control.l1.kind) {
    case 'pid':
      return pidModel(p, T);
    case 'lqr':
      return lqrModel(p, T);
    case 'adrc':
      return adrcModel(p, T);
    default:
      return null;
  }
}
