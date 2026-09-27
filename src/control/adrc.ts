import { Eso } from '@/estimation/eso';
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
 * Level 1, linear ADRC (Gao 2003): the model is only ÿ = b₀·(T − T_known) + f with b₀ = 1/m̂.
 * Everything else — wrong mass, drag, wind, motor lag — is the "total disturbance" f, estimated by
 * an extended state observer and cancelled. What is left is a double integrator, controlled by a
 * PD with both poles at −ω_c.
 */
export class AdrcController implements Controller {
  readonly eso = new Eso();
  private held: Actuation = idleActuation();
  private applied = 0;
  private known = 0;
  private b0 = 1;
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
    this.eso.reset();
    this.held = idleActuation();
    this.applied = 0;
    this.last = { ...this.last, parts: parts(0, 0, 0, 0) };
  }

  resetIntegrators(): void {
    this.eso.clearDisturbance();
  }

  tick(input: ControlInput): Actuation {
    const { params: p, physDt, stepIndex } = input;
    const n = periodSteps(p.control.rateHz, physDt);
    if (stepIndex % n !== 0) return this.held;
    const dt = n * physDt;
    const m = input.sense();
    const { wc, wo } = p.control.adrc;
    const mHat = Math.max(p.control.model.mass, 0.05);
    this.b0 = 1 / mHat;
    this.known = p.control.feedforward ? mHat * GRAVITY : 0;

    // Observer first, fed with what was actually applied during the last sample.
    const [z1, z2, z3] = this.eso.update(this.applied - this.known, m.pos.y, wo, this.b0, dt);

    const sp = input.setpoint.pos.y;
    const pos = (wc * wc * (sp - z1)) / this.b0;
    const vel = (-2 * wc * z2) / this.b0;
    const dist = -z3 / this.b0;
    const unsaturated = this.known + pos + vel + dist;
    const output = clamp(unsaturated, 0, 4 * p.drone.maxMotorThrust);
    // The observer must see the saturated input, or it would "explain" the missing thrust as a
    // disturbance and wind up — ADRC's version of anti-windup.
    this.applied = output;
    this.last = {
      setpoint: sp,
      measurement: m.pos.y,
      error: sp - m.pos.y,
      parts: parts(pos, vel, dist, this.known),
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
    const [z1, z2, z3] = this.eso.z;
    // The engine's `dist` is m̂·a − T − m̂·g; convert the observer's f to the same definition.
    return {
      'est.dist.y': z3 / this.b0 + (1 / this.b0) * GRAVITY - this.known,
      'eso.y': z1,
      'eso.v': z2,
    };
  }

  describe(p: Params): InfoRow[] {
    const { wc, wo } = p.control.adrc;
    const m = p.control.model.mass;
    return [
      {
        label: 'b₀ = 1/m̂',
        value: `${(1 / m).toFixed(3)} 1/kg`,
        hint: 'The only model knowledge ADRC needs: how strongly thrust accelerates the drone.',
      },
      {
        label: 'controller poles',
        value: `−${wc.toFixed(1)} (double)`,
        hint: 'With the disturbance cancelled, the loop is (s + ω_c)²: critically damped by design.',
      },
      {
        label: 'equivalent Kp · Kd',
        value: `${(wc * wc * m).toFixed(2)} · ${(2 * wc * m).toFixed(2)}`,
        hint: 'The PD part in PID units (N/m, N·s/m).',
      },
      {
        label: 'observer poles',
        value: `−${wo.toFixed(1)} (triple)`,
        hint: 'How fast the observer believes the measurements. Rule of thumb: 3–10 × ω_c.',
      },
      { label: 'ω_o / ω_c', value: (wo / Math.max(wc, 1e-6)).toFixed(1) },
    ];
  }
}

const parts = (pos: number, vel: number, dist: number, ff: number): LoopTerms['parts'] => [
  { key: 'pos', label: 'ω_c²·(r−ŷ)/b₀', value: pos, like: 'p' },
  { key: 'vel', label: '−2ω_c·v̂/b₀', value: vel, like: 'd' },
  { key: 'dist', label: '−f̂/b₀', value: dist, like: 'i' },
  { key: 'ff', label: 'FF', value: ff, ff: true, like: 'ff' },
];
