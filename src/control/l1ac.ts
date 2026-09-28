import { LowPass1 } from '@/estimation/filters';
import { qRotate } from '@/math/quat';
import { clamp } from '@/math/util';
import { v3, type Vec3 } from '@/math/vec3';
import { idleActuation, type Actuation } from '@/sim/dynamics';
import { GRAVITY, type Params } from '@/sim/params';
import type { Measurement } from '@/sim/sensors';
import type { Compensation } from './stages';
import {
  periodSteps,
  type ControlInput,
  type Controller,
  type InfoRow,
  type LoopTerms,
} from './types';

/**
 * L1 adaptive estimate of a matched force disturbance on one axis (Hovakimyan & Cao 2010, in the
 * velocity-predictor form used by L1Quad, Wu et al. 2025):
 *
 *   state predictor   m̂·v̂̇ = F_known + σ̂ + m̂·a_s·(v̂ − v)      (a_s < 0)
 *   adaptation        σ̂ = −m̂ · Φ⁻¹ · e^{a_s·T} · (v̂ − v),   Φ = (e^{a_s·T} − 1)/a_s
 *   filter            σ̂_f = C(s)·σ̂   (first-order low-pass)
 *
 * The piecewise-constant law makes σ̂ as fast as the sample rate allows; the low-pass C(s) is
 * what decides how much of it reaches the actuators — adaptation speed and robustness are tuned
 * separately, which is L1's point.
 */
export class L1Axis {
  private vHat = 0;
  private started = false;
  sigma = 0;
  readonly filter = new LowPass1(5, 0.004);

  reset(): void {
    this.started = false;
    this.sigma = 0;
    this.filter.reset(0);
  }

  /** Clear the estimate (on the ground). */
  clear(): void {
    this.sigma = 0;
    this.filter.reset(0);
  }

  /**
   * One step. `force` is the known force on the axis (thrust + gravity as modelled), `v` the
   * measured velocity. Returns the filtered estimate σ̂_f, N.
   */
  update(force: number, v: number, mHat: number, as: number, cutoffHz: number, dt: number): number {
    if (!this.started) {
      this.vHat = v;
      this.started = true;
    }
    const a = -Math.abs(as);
    const err = this.vHat - v;
    const ead = Math.exp(a * dt);
    const phi = (ead - 1) / a;
    this.sigma = (-mHat * ead * err) / phi;
    // Predictor, integrated over the sample with the estimate just computed.
    this.vHat += dt * ((force + this.sigma) / mHat + a * err);
    this.filter.cutoffHz = cutoffHz;
    this.filter.dt = dt;
    return this.filter.update(this.sigma);
  }

  get filtered(): number {
    return this.filter.value;
  }
}

/**
 * Level 1, L1 adaptive altitude control: a PD baseline (both poles at −ω_c, like ADRC's) plus the
 * filtered L1 estimate of whatever force the model misses.
 */
export class L1AdaptiveController implements Controller {
  readonly axis = new L1Axis();
  private held: Actuation = idleActuation();
  private applied = 0;
  private last: LoopTerms = {
    setpoint: 0,
    measurement: 0,
    error: 0,
    parts: parts(0, 0, 0, 0),
    unsaturated: 0,
    output: 0,
    saturated: false,
  };

  reset(): void {
    this.axis.reset();
    this.held = idleActuation();
    this.applied = 0;
  }

  resetIntegrators(): void {
    this.axis.clear();
  }

  tick(input: ControlInput): Actuation {
    const { params: p, physDt, stepIndex } = input;
    const n = periodSteps(p.control.rateHz, physDt);
    if (stepIndex % n !== 0) return this.held;
    const dt = n * physDt;
    const m = input.sense();
    const c = p.control.l1ac;
    const mHat = Math.max(p.control.model.mass, 0.05);
    const ff = p.control.feedforward ? mHat * GRAVITY : 0;
    // Known force during the last sample: the thrust we applied minus the modelled weight.
    const sigma = this.axis.update(
      this.applied - mHat * GRAVITY,
      m.vel.y,
      mHat,
      c.as,
      c.filterHz,
      dt,
    );
    const sp = input.setpoint.pos.y;
    const pos = mHat * c.wc * c.wc * (sp - m.pos.y);
    const vel = -mHat * 2 * c.wc * m.vel.y;
    const unsaturated = ff + pos + vel - sigma;
    const output = clamp(unsaturated, 0, 4 * p.drone.maxMotorThrust);
    this.applied = output;
    this.last = {
      setpoint: sp,
      measurement: m.pos.y,
      error: sp - m.pos.y,
      parts: parts(pos, vel, -sigma, ff),
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
    // σ̂ is already in the engine's disturbance units (N, model-based).
    return { 'est.dist.y': this.axis.filtered, 'l1.sigma.raw': this.axis.sigma };
  }

  describe(p: Params): InfoRow[] {
    const c = p.control.l1ac;
    return [
      {
        label: 'predictor pole a_s',
        value: `−${Math.abs(c.as)} 1/s`,
        hint: 'How fast the state predictor pulls towards the measurement; with the piecewise-constant law, the adaptation itself is as fast as the sample rate.',
      },
      {
        label: 'filter C(s)',
        value: `${c.filterHz} Hz`,
        hint: 'How much of the (fast, noisy) estimate reaches the motors. The robustness knob — independent of the adaptation speed.',
      },
      { label: 'baseline poles', value: `−${c.wc} (double)` },
    ];
  }
}

const parts = (pos: number, vel: number, sigma: number, ff: number): LoopTerms['parts'] => [
  { key: 'pos', label: 'baseline P', value: pos, like: 'p' },
  { key: 'vel', label: 'baseline D', value: vel, like: 'd' },
  { key: 'sigma', label: '−σ̂ (adaptive)', value: sigma, like: 'i' },
  { key: 'ff', label: 'FF', value: ff, ff: true, like: 'ff' },
];

/**
 * L3 compensation stage: an L1 estimate per world axis, subtracted from the nominal force.
 *   F = m̂·(a_sp + g) − σ̂_f
 */
export class L1Compensation implements Compensation {
  readonly replacesIntegral = true;
  private axes = { x: new L1Axis(), y: new L1Axis(), z: new L1Axis() };
  private f0 = v3();
  private cmd: [number, number, number, number] = [0, 0, 0, 0];
  private model: [number, number, number, number] = [0, 0, 0, 0];

  reset(): void {
    for (const a of Object.values(this.axes)) a.reset();
    this.f0 = v3();
    this.cmd = [0, 0, 0, 0];
    this.model = [0, 0, 0, 0];
  }

  applied(motors: [number, number, number, number]): void {
    this.cmd = [...motors];
  }

  sample(s: Measurement, p: Params, dt: number): void {
    const mHat = Math.max(p.control.model.mass, 0.05);
    const c = p.control.l1ac;
    const a = 1 - Math.exp(-dt / Math.max(p.control.model.motorTau, 1e-4));
    for (let i = 0; i < 4; i++)
      this.model[i] = this.model[i]! + (this.cmd[i]! - this.model[i]!) * a;
    const r = s.motors ?? this.model;
    this.f0 = qRotate(s.q, v3(0, r[0] + r[1] + r[2] + r[3], 0));
    const g = mHat * GRAVITY;
    this.axes.x.update(this.f0.x, s.vel.x, mHat, c.as, c.filterHz, dt);
    this.axes.y.update(this.f0.y - g, s.vel.y, mHat, c.as, c.filterHz, dt);
    this.axes.z.update(this.f0.z, s.vel.z, mHat, c.as, c.filterHz, dt);
  }

  force(aSp: Vec3, p: Params): Vec3 {
    const m = p.control.model.mass;
    const { x, y, z } = this.axes;
    return v3(m * aSp.x - x.filtered, m * (aSp.y + GRAVITY) - y.filtered, m * aSp.z - z.filtered);
  }

  loops(): Record<string, LoopTerms> {
    return {};
  }

  extras(): Record<string, number> {
    return {
      'est.dist.x': this.axes.x.filtered,
      'est.dist.y': this.axes.y.filtered,
      'est.dist.z': this.axes.z.filtered,
    };
  }

  describe(p: Params, loopId: string): InfoRow[] {
    if (!loopId.startsWith('pos.') && !loopId.startsWith('vel.')) return [];
    const c = p.control.l1ac;
    return [
      {
        label: 'compensation',
        value: 'F = m̂(a_sp + g) − σ̂',
        hint: 'L1 adaptive estimate per world axis.',
      },
      { label: 'predictor · filter', value: `−${Math.abs(c.as)} 1/s · ${c.filterHz} Hz` },
    ];
  }
}
