import { care } from '@/math/care';
import { c2dTustin, series, ss, type Lti } from '@/math/lti';
import { add, eigenvalues, eye, mul, scale, solve, transpose, type Mat } from '@/math/mat';
import { clamp } from '@/math/util';
import { idleActuation, type Actuation } from '@/sim/dynamics';
import { GRAVITY, type Params } from '@/sim/params';
import {
  periodSteps,
  type ControlInput,
  type Controller,
  type InfoRow,
  type LoopTerms,
} from './types';

/**
 * H∞ loop shaping (Glover–McFarlane; docs/analysis.md §4, Chapter H) for the L1 altitude loop.
 *
 * 1. Shape. The designer chooses only a weight W(s) = k·(s + ωᵢ)/s: a gain that sets the
 *    crossover and an integral corner below it. The shaped plant is Gs = G·W, with
 *    G(s) = 1/(m̂·s²·(τ̂·s + 1)) from thrust to altitude.
 * 2. Robustify. Two Riccati equations give the controller Ks that stabilises Gs with the largest
 *    margin against perturbations of its normalised coprime factors, ε_max = 1/γ_min, in closed
 *    form. The weight alone (a PI on a double integrator) cannot even stabilise the drone; Ks
 *    supplies the phase lead around crossover, where the shape asked for it.
 * 3. The controller that flies is W·Ks, discretised at the controller's rate.
 */
export interface HinfDesign {
  /** Robust stability margin of the shaped plant, ε_max = 1/γ_min (0…1; > 0.25 is good). */
  epsMax: number;
  gammaMin: number;
  /** The γ used: a few per cent above γ_min. */
  gamma: number;
  /** Ks, discrete, from −(y − r) … see `HinfController`. Positive feedback on y. */
  ks: Lti;
  /** The weight's integral: I⁺ = I + T·v, output k·v + k·ωᵢ·I⁺. */
  k: number;
  wi: number;
  /** The lead of the weight, discrete (Tustin), or null. */
  lead: Lti | null;
  T: number;
  /** Ks continuous, for the info card. */
  order: number;
}

/** The PI part of the weight, k·(s + ωᵢ)/s, continuous. */
const piWeight = (k: number, wi: number): Lti => ss([[0]], [[1]], [[k * wi]], [[k]]);

/** The lead part, (s/ω_z + 1)/(s/ω_p + 1), continuous; null when it is off. */
function leadWeight(wz: number, wp: number): Lti | null {
  if (!(wz > 0) || !(wp > wz)) return null;
  const g = wp / wz;
  return ss([[-wp]], [[1]], [[g * (wz - wp)]], [[g]]);
}

/** The plant as the controller models it: thrust → altitude, 1/(m̂·s²·(τ̂·s + 1)). */
function modelPlant(p: Params): Lti {
  const m = Math.max(p.control.model.mass, 0.05);
  const tau = Math.max(p.control.model.motorTau, 1e-3);
  return ss(
    [
      [0, 1, 0],
      [0, 0, 1 / m],
      [0, 0, -1 / tau],
    ],
    [[0], [0], [1 / tau]],
    [[1, 0, 0]],
  );
}

/** Shaped plant Gs = G·W: input into the weight, output altitude. */
export function shapedPlant(p: Params): Lti {
  const { k, wi, wz, wp } = p.control.hinf;
  const lead = leadWeight(wz, wp);
  const w = lead ? series(piWeight(k, wi), lead) : piWeight(k, wi);
  return series(w, modelPlant(p));
}

/** Spectral radius of a square matrix. */
const rho = (a: Mat): number => Math.max(...eigenvalues(a).map((z) => Math.hypot(z.re, z.im)));

const cache = new Map<string, HinfDesign>();

/** The Glover–McFarlane design for the current weight and model, at controller period T. */
export function designHinf(p: Params, T: number): HinfDesign {
  const h = p.control.hinf;
  const key = JSON.stringify([h, p.control.model.mass, p.control.model.motorTau, T]);
  const hit = cache.get(key);
  if (hit) return hit;
  const g = shapedPlant(p);
  const { a, b, c } = g;
  const bbt = mul(b, transpose(b));
  const ctc = mul(transpose(c), c);
  // Control and filter Riccati equations of the normalised coprime-factor problem (D = 0).
  const x = care(a, bbt, ctc);
  const z = care(transpose(a), ctc, bbt);
  const gammaMin = Math.sqrt(1 + rho(mul(x, z)));
  const gamma = gammaMin * h.gammaFactor;
  const n = a.length;
  const f = scale(mul(transpose(b), x), -1); // F = −Bᵀ·X
  const l = add(scale(eye(n), 1 - gamma * gamma), mul(x, z)); // L = (1 − γ²)·I + X·Z
  const ltInvZCt = solve(transpose(l), mul(z, transpose(c))); // (Lᵀ)⁻¹·Z·Cᵀ
  const ak = add(add(a, mul(b, f)), scale(mul(ltInvZCt, c), gamma * gamma));
  const bk = scale(ltInvZCt, gamma * gamma);
  const ck = mul(transpose(b), x);
  const ks = c2dTustin(ss(ak, bk, ck), T);
  const d: HinfDesign = {
    epsMax: 1 / gammaMin,
    gammaMin,
    gamma,
    ks,
    k: h.k,
    wi: h.wi,
    lead: (() => {
      const l = leadWeight(h.wz, h.wp);
      return l ? c2dTustin(l, T) : null;
    })(),
    T,
    order: n,
  };
  cache.set(key, d);
  return d;
}

/** The whole controller, W·Ks, as one discrete system: inputs (y, r), output the thrust command. */
export function hinfControllerLti(d: HinfDesign): Lti {
  const { ks, k, wi, T } = d;
  // Ks on e = y − r; then the PI, I⁺ = I + T·v and u = k·v + k·ωᵢ·I⁺; then the lead.
  const ksE: Lti = {
    ...ks,
    b: ks.b.map((r) => [r[0]!, -r[0]!]),
    d: ks.d.map((r) => [r[0]!, -r[0]!]),
  };
  const pi = ss([[1]], [[T]], [[k * wi]], [[k + k * wi * T]], T);
  const kw = series(ksE, pi);
  return d.lead ? series(kw, d.lead) : kw;
}

/** Level 1, H∞ loop shaping: runs Ks and the PI weight W at the controller's rate. */
export class HinfController implements Controller {
  private held: Actuation = idleActuation();
  private xs: number[] = [];
  private integral = 0;
  private xl = 0;
  private last: LoopTerms = {
    setpoint: 0,
    measurement: 0,
    error: 0,
    parts: [],
    unsaturated: 0,
    output: 0,
    saturated: false,
  };
  private design: HinfDesign | null = null;

  reset(): void {
    this.held = idleActuation();
    this.xs = [];
    this.integral = 0;
    this.xl = 0;
  }

  resetIntegrators(): void {
    this.integral = 0;
  }

  tick(input: ControlInput): Actuation {
    const { params: p, physDt, stepIndex } = input;
    const steps = periodSteps(p.control.rateHz, physDt);
    if (stepIndex % steps !== 0) return this.held;
    const d = designHinf(p, steps * physDt);
    if (d !== this.design || this.xs.length !== d.ks.a.length) {
      this.xs = new Array<number>(d.ks.a.length).fill(0);
      this.design = d;
    }
    const m = input.sense();
    const sp = input.setpoint.pos.y;
    const e = m.pos.y - sp;
    const ks = d.ks;
    // Ks, then the weight. Positive feedback on y − r, as the design assumes.
    const v = ks.c[0]!.reduce((s, c, i) => s + c * this.xs[i]!, 0) + ks.d[0]![0]! * e;
    this.xs = ks.a.map(
      (row, i) => row.reduce((s, a, j) => s + a * this.xs[j]!, 0) + ks.b[i]![0]! * e,
    );
    const ff = p.control.feedforward ? p.control.model.mass * GRAVITY : 0;
    const iNext = this.integral + d.T * v;
    const w = d.k * v + d.k * d.wi * iNext;
    let fb = w;
    if (d.lead) {
      fb = d.lead.c[0]![0]! * this.xl + d.lead.d[0]![0]! * w;
      this.xl = d.lead.a[0]![0]! * this.xl + d.lead.b[0]![0]! * w;
    }
    const unsaturated = ff + fb;
    const output = clamp(unsaturated, 0, 4 * p.drone.maxMotorThrust);
    // Conditional integration: hold the integral while the motors cannot follow.
    if (output === unsaturated) this.integral = iNext;
    this.last = {
      setpoint: sp,
      measurement: m.pos.y,
      error: sp - m.pos.y,
      parts: [
        { key: 'shape', label: 'k·Ks·e', value: d.k * v, like: 'p' },
        { key: 'int', label: 'k·ωᵢ·∫Ks·e', value: d.k * d.wi * iNext, like: 'i' },
        { key: 'ff', label: 'FF', value: ff, ff: true, like: 'ff' },
      ],
      unsaturated,
      output,
      saturated: output !== unsaturated,
    };
    const f = output / 4;
    this.held = { motorCmd: [f, f, f, f], forceCmd: this.held.forceCmd };
    return this.held;
  }

  loops() {
    return { alt: this.last };
  }

  extras() {
    return { 'hinf.eps': this.design?.epsMax ?? NaN };
  }

  describe(p: Params): InfoRow[] {
    const d = designHinf(p, periodSteps(p.control.rateHz, 0.001) * 0.001);
    return [
      {
        label: 'robust stability margin ε_max',
        value: d.epsMax.toFixed(3),
        hint: 'The largest perturbation of the shaped plant’s coprime factors that the design is guaranteed to survive. Above 0.25 is a good design; below 0.2 the shape asks for too much.',
      },
      {
        label: 'γ_min · γ used',
        value: `${d.gammaMin.toFixed(2)} · ${d.gamma.toFixed(2)}`,
      },
      {
        label: 'weight W(s)',
        value:
          `${d.k.toFixed(2)}·(s + ${d.wi.toFixed(2)})/s` +
          (d.lead ? ` · (s/${p.control.hinf.wz} + 1)/(s/${p.control.hinf.wp} + 1)` : ''),
        hint: 'The only thing the designer chooses: the gain sets the crossover, ωᵢ where integral action fades out.',
      },
      { label: 'controller order', value: `${d.order} (Ks) + ${d.lead ? 2 : 1} (the weight)` },
    ];
  }
}
