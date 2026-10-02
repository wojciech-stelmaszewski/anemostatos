// System identification (docs/analysis.md §4, Chapter I, lessons III.25 and III.26): fit the
// altitude plant from measured frequency responses, run a relay experiment, and estimate a
// frequency response with its coherence from a chirp.
import { evalAt } from '@/math/lti';
import { cabs, carg, cdiv, cis, cmul, cscale, csub, C_ONE, type Complex } from '@/math/complex';
import { crossSpectrum } from '@/math/fft';
import { Simulation, PHYS_DT } from '@/engine/simulation';
import { idleActuation, type Actuation } from '@/sim/dynamics';
import type { Params } from '@/sim/params';
import type { ControlInput, Controller } from '@/control/types';
import { l1Plant, PLANT_OUT } from '@/analysis/plant';
import { controlPeriod } from '@/analysis/controllers';

/** A measured point of the plant's frequency response, from thrust (N) to altitude (m). */
export interface PlantPoint {
  fHz: number;
  p: Complex;
}

/** The altitude plant P(s) = e^{−s·d} / (m·s²·(τ·s + 1)) as the fit describes it. */
export interface PlantFit {
  mass: number;
  tau: number;
  /** Delay between the thrust and the altitude the controller reads, s. */
  delay: number;
  /** Points used, and the RMS misfit of the magnitude, dB. */
  used: number;
  rmsDb: number;
}

/** The model's response at ω. */
export const plantModel = (f: Pick<PlantFit, 'mass' | 'tau' | 'delay'>, w: number): Complex => {
  const jw = { re: 0, im: w };
  const den = cmul(cscale(cmul(jw, jw), f.mass), { re: 1, im: w * f.tau });
  return cdiv(cis(-w * f.delay), den);
};

/** Unwrap a phase sequence so that neighbours differ by less than π. */
function unwrap(ph: number[]): number[] {
  const out = [...ph];
  for (let i = 1; i < out.length; i++) {
    while (out[i]! - out[i - 1]! > Math.PI) out[i] = out[i]! - 2 * Math.PI;
    while (out[i]! - out[i - 1]! < -Math.PI) out[i] = out[i]! + 2 * Math.PI;
  }
  return out;
}

/**
 * Least squares, in two linear steps.
 * Magnitude: 1/|P|² = m²·ω⁴·(1 + ω²τ²), so z = 1/(|P|²·ω⁴) = m² + (m²τ²)·ω², a straight line in ω².
 * Phase: ∠P = −π − atan(ωτ) − ω·d, so with τ known the residual phase is a line through zero in ω.
 */
export function fitPlant(points: readonly PlantPoint[]): PlantFit {
  const pts = [...points].sort((a, b) => a.fHz - b.fHz);
  const w = pts.map((q) => 2 * Math.PI * q.fHz);
  // z = a + b·x with x = ω², weighted so that every point counts in relative terms.
  let s0 = 0;
  let s1 = 0;
  let s2 = 0;
  let t0 = 0;
  let t1 = 0;
  pts.forEach((q, i) => {
    const x = w[i]! ** 2;
    const z = 1 / (cabs(q.p) ** 2 * w[i]! ** 4);
    const wt = 1 / (z * z);
    s0 += wt;
    s1 += wt * x;
    s2 += wt * x * x;
    t0 += wt * z;
    t1 += wt * x * z;
  });
  const det = s0 * s2 - s1 * s1;
  const a = (t0 * s2 - t1 * s1) / det;
  const b = (s0 * t1 - s1 * t0) / det;
  const mass = Math.sqrt(Math.max(a, 1e-12));
  const tau = Math.sqrt(Math.max(b, 0)) / mass;
  const ph = unwrap(pts.map((q) => carg(q.p)));
  // Bring the low-frequency phase near −π (a double integrator), whole turns only.
  const shift = 2 * Math.PI * Math.round((-Math.PI - ph[0]!) / (2 * Math.PI));
  let num = 0;
  let den = 0;
  pts.forEach((_, i) => {
    const r = -(ph[i]! + shift) - Math.PI - Math.atan(w[i]! * tau);
    num += w[i]! * r;
    den += w[i]! ** 2;
  });
  const delay = Math.max(num / den, 0);
  const fit = { mass, tau, delay };
  const misfit = pts.map((q, i) => 20 * Math.log10(cabs(q.p) / cabs(plantModel(fit, w[i]!))));
  const rmsDb = Math.sqrt(misfit.reduce((acc, v) => acc + v * v, 0) / misfit.length);
  return { ...fit, used: pts.length, rmsDb };
}

// ---------------------------------------------------------------------------------------------
// The relay experiment

/**
 * A relay on the vertical velocity: thrust = T₀ ± h around the measured hover thrust T₀, switching
 * on the sign of the velocity.
 * Any loop with a phase of −180° somewhere settles into a limit cycle there; its amplitude a gives
 * the ultimate gain Ku = 4h/(π·a) and its period the ultimate period Tu [Åström 1984].
 */
export class RelayController implements Controller {
  private held: Actuation = idleActuation();
  private sign = 1;
  constructor(
    private h: number,
    private base: number,
  ) {}
  reset(): void {
    this.held = idleActuation();
  }
  resetIntegrators(): void {}
  tick(input: ControlInput): Actuation {
    const { params: p, physDt, stepIndex } = input;
    const n = controlPeriod(p).steps;
    if (stepIndex % n !== 0 && physDt > 0) return this.held;
    const m = input.sense();
    if (m.vel.y > 0) this.sign = -1;
    else if (m.vel.y < 0) this.sign = 1;
    const f = (this.base + this.sign * this.h) / 4;
    this.held = { motorCmd: [f, f, f, f], forceCmd: this.held.forceCmd };
    return this.held;
  }
  loops() {
    return {};
  }
  extras() {
    return {};
  }
}

export interface RelayResult {
  /** Amplitude of the velocity oscillation, m/s, and its period, s. */
  amplitude: number;
  period: number;
  ku: number;
  tu: number;
}

/**
 * Hover on the selected controller, then hand the motors to the relay for `seconds` and measure
 * the limit cycle of the vertical velocity over the last half.
 */
export function relayTest(p: Params, h = 0.5, seconds = 12): RelayResult {
  const q = structuredClone(p);
  q.wind.enabled = false;
  q.setpoint = { ...q.setpoint, profile: 'none' };
  const sim = new Simulation(q);
  // Hover on the selected controller and measure the thrust it takes: the relay's centre.
  let base = 0;
  for (let k = 0; k < 8000; k++) {
    sim.step();
    if (k >= 6000) base += sim.actuation.motorCmd.reduce((a, b) => a + b, 0) / 2000;
  }
  sim.controller = new RelayController(h, base);
  const v: number[] = [];
  const n = Math.round(seconds / PHYS_DT);
  for (let k = 0; k < n; k++) {
    sim.step();
    if (k > n / 2) v.push(sim.state.vel.y);
  }
  const amplitude = (Math.max(...v) - Math.min(...v)) / 2;
  // Period from upward zero crossings.
  const ups: number[] = [];
  for (let i = 1; i < v.length; i++) if (v[i - 1]! < 0 && v[i]! >= 0) ups.push(i);
  const period =
    ups.length > 1 ? ((ups[ups.length - 1]! - ups[0]!) / (ups.length - 1)) * PHYS_DT : NaN;
  return { amplitude, period, ku: (4 * h) / (Math.PI * amplitude), tu: period };
}

/**
 * What the model predicts for the relay: the velocity loop's phase crossover (−180°), with the
 * motor lag, the sensor delay and the controller's sample-and-hold, and the gain there.
 */
export function relayPrediction(p: Params): { ku: number; tu: number } {
  const plant = l1Plant(p);
  const { steps } = controlPeriod(p);
  const delay = Math.round(p.sensors.delayMs / 1000 / PHYS_DT);
  const g = (w: number): Complex => {
    const pv = evalAt(plant, cis(w * PHYS_DT))[PLANT_OUT.v]![0]!;
    const om = w * PHYS_DT;
    const hold =
      steps > 1
        ? cscale(cdiv(csub(C_ONE, cis(-om * steps)), csub(C_ONE, cis(-om))), 1 / steps)
        : C_ONE;
    return cmul(cmul(pv, hold), cis(-om * delay));
  };
  // Bisection on the phase between 0.5 and 400 rad/s (it falls from −90° towards −270°).
  let lo = 0.5;
  let hi = 400;
  const phase = (w: number) => {
    let a = carg(g(w));
    if (a > 0) a -= 2 * Math.PI;
    return a;
  };
  for (let i = 0; i < 80; i++) {
    const mid = Math.sqrt(lo * hi);
    if (phase(mid) > -Math.PI) lo = mid;
    else hi = mid;
  }
  const w = Math.sqrt(lo * hi);
  return { ku: 1 / cabs(g(w)), tu: (2 * Math.PI) / w };
}

// ---------------------------------------------------------------------------------------------
// Frequency response from a chirp, with its coherence

export interface ChirpId {
  fHz: number[];
  /**
   * Estimate of the plant, thrust → measured altitude, and the coherence behind it. `direct`:
   * from the thrust to the altitude, H = P_uy/P_uu. `viaProbe`: both measured against the test
   * signal d the probe injected, H = P_dy/P_du. In a closed loop only the second is unbiased.
   */
  direct: { h: Complex[]; coherence: number[] };
  viaProbe: { h: Complex[]; coherence: number[] };
}

/**
 * A chirp experiment that can be advanced in slices, so that the interface stays live: hover for
 * six seconds, then a logarithmic chirp on the thrust probe, recording the test signal, the thrust
 * and the altitude the controller read at every physics step.
 */
export class ChirpExperiment {
  private sim: Simulation;
  private k = -6000;
  private d: Float64Array;
  private u: Float64Array;
  private y: Float64Array;
  result: ChirpId | null = null;
  constructor(
    p: Params,
    private opts: { f0: number; f1: number; seconds: number; amp: number },
  ) {
    const q = structuredClone(p);
    q.setpoint = { ...q.setpoint, profile: 'none' };
    q.probe = { ...q.probe, point: 'none' };
    this.sim = new Simulation(q);
    const n = Math.round(opts.seconds / PHYS_DT);
    this.d = new Float64Array(n);
    this.u = new Float64Array(n);
    this.y = new Float64Array(n);
  }
  get progress(): number {
    return Math.max(0, this.k) / this.d.length;
  }
  get done(): boolean {
    return this.result !== null;
  }
  advance(budget: number): boolean {
    const sim = this.sim;
    for (let i = 0; i < budget && !this.done; i++) {
      if (this.k === 0) {
        const o = this.opts;
        sim.setParams({
          ...sim.params,
          probe: {
            ...sim.params.probe,
            point: 'l1.thrust',
            signal: 'chirp',
            amp: o.amp,
            f0Hz: o.f0,
            f1Hz: o.f1,
            durationS: o.seconds,
          },
        });
      }
      sim.step();
      if (this.k >= 0) {
        this.d[this.k] = sim.probe.in;
        this.u[this.k] = sim.probe.u;
        this.y[this.k] = sim.measurement ? sim.measurement.pos.y : sim.state.pos.y;
      }
      this.k++;
      if (this.k >= this.d.length) this.result = analyseChirp(this.d, this.u, this.y, this.opts);
    }
    return this.done;
  }
}

function analyseChirp(
  d: Float64Array,
  u: Float64Array,
  y: Float64Array,
  o: { f0: number; f1: number },
): ChirpId {
  const fs = 1 / PHYS_DT;
  const uy = crossSpectrum(u, y, fs, 8192);
  const dy = crossSpectrum(d, y, fs, 8192);
  const du = crossSpectrum(d, u, fs, 8192);
  const keep = uy.f.map((f) => f >= o.f0 && f <= o.f1);
  const pick = <T>(v: T[]) => v.filter((_, i) => keep[i]);
  return {
    fHz: pick(uy.f),
    direct: { h: pick(uy.h), coherence: pick(uy.coherence) },
    // (d → y)/(d → u): the loop's reaction to the wind is not correlated with d.
    viaProbe: { h: pick(dy.h.map((v, i) => cdiv(v, du.h[i]!))), coherence: pick(dy.coherence) },
  };
}

/** What a chirp depends on: everything but how it is analysed. */
export const chirpKey = (p: Params): string =>
  JSON.stringify([p.drone, p.wind, p.sensors, p.control, p.sim, p.sysid.amp, p.sysid.seconds]);
const chirps = new Map<string, ChirpId>();
export const storeChirp = (p: Params, id: ChirpId): void => {
  if (chirps.size > 16) chirps.clear();
  chirps.set(chirpKey(p), id);
};
export const finishedChirp = (p: Params): ChirpId | null => chirps.get(chirpKey(p)) ?? null;

/** The chirp of the lesson's settings, flown at once (tests), and remembered. */
export function runChirp(p: Params): ChirpId {
  const e = new ChirpExperiment(p, { f0: 0.2, f1: 8, seconds: p.sysid.seconds, amp: p.sysid.amp });
  while (!e.advance(1_000_000));
  storeChirp(p, e.result!);
  return e.result!;
}

/**
 * Fly a chirp on the thrust probe (f0 → f1 over `seconds`) and estimate the plant from the thrust
 * the motors received to the altitude the controller read, by Welch's cross-spectra.
 */
export function chirpIdentify(
  p: Params,
  opts: { f0?: number; f1?: number; seconds?: number; amp?: number } = {},
): ChirpId {
  const { f0 = 0.2, f1 = 8, seconds = 60, amp = 0.4 } = opts;
  const e = new ChirpExperiment(p, { f0, f1, seconds, amp });
  while (!e.advance(1_000_000));
  return e.result!;
}

/** Fit the plant to the bins of a chirp estimate whose coherence is at least `minCoherence`. */
export function fitChirp(
  id: ChirpId,
  minCoherence: number,
  estimator: 'direct' | 'viaProbe' = 'viaProbe',
): PlantFit {
  const e = id[estimator];
  const pts = id.fHz
    .map((fHz, i) => ({ fHz, p: e.h[i]!, c: e.coherence[i]! }))
    .filter((q) => q.c >= minCoherence);
  if (pts.length < 3) return { mass: NaN, tau: NaN, delay: NaN, used: pts.length, rmsDb: NaN };
  return fitPlant(pts);
}
