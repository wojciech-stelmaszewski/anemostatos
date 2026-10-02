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
 * Level 1, model-reference adaptive control (docs/analysis.md §4, Chapter H; [Ioannou 1996]).
 *
 * The reference model is the loop we want: ÿ_m = k_p·(r − y_m) − k_d·ẏ_m. For a drone of mass m
 * the thrust that makes the real loop behave like it is u* = θ*ᵀ·w with
 *   w = (r − y, −ẏ, 1)   and   θ* = m·(k_p, k_d, g).
 * The mass is unknown, so the controller uses an estimate θ̂ and adapts it. With x = (y − y_m,
 * ẏ − ẏ_m) and P the solution of AᵀP + PA = −I for the model's A, the Lyapunov function
 * V = xᵀPx + θ̃ᵀΓ⁻¹θ̃/m gives the adaptation law
 *   θ̂̇ = −Γ·w·(BᵀPx) − σ·Γ·(θ̂ − θ₀)
 * which makes V̇ ≤ 0 for the plant the proof assumes (σ = 0). The σ term (σ-modification) pulls the
 * two feedback gains back towards the initial guess θ₀: it gives up exact tracking to stop them
 * drifting when the plant is not the one the proof assumed (here: the motor lag and the sensor
 * delay it ignores). The hover parameter is left free.
 */
export interface MracState {
  theta: [number, number, number];
  ym: number;
  vm: number;
}

/** P of AᵀP + PA = −I for A = [[0, 1], [−kp, −kd]], as (p11, p12, p22). */
export function lyapunovP(kp: number, kd: number): [number, number, number] {
  // Three equations: −2kp·p12 = −1;  p11 − kd·p12 − kp·p22 = 0;  2(p12 − kd·p22) = −1.
  const p12 = 1 / (2 * kp);
  const p22 = (p12 + 0.5) / kd;
  const p11 = kd * p12 + kp * p22;
  return [p11, p12, p22];
}

/** The initial guess: the ideal parameters for the mass the model assumes. */
export const thetaFor = (mass: number, p: Params): [number, number, number] => {
  const c = p.control.mrac;
  return [mass * c.kp, mass * c.kd, mass * GRAVITY];
};

export class MracController implements Controller {
  private held: Actuation = idleActuation();
  private last: LoopTerms = {
    setpoint: 0,
    measurement: 0,
    error: 0,
    parts: [],
    unsaturated: 0,
    output: 0,
    saturated: false,
  };
  state: MracState | null = null;
  /** While the drone sits on the ground or takes off, the reference model follows it and θ̂ holds. */
  private hold = true;

  reset(): void {
    this.held = idleActuation();
    this.state = null;
    this.hold = true;
  }

  resetIntegrators(): void {
    this.hold = true;
  }

  tick(input: ControlInput): Actuation {
    const { params: p, physDt, stepIndex } = input;
    const steps = periodSteps(p.control.rateHz, physDt);
    if (stepIndex % steps !== 0) return this.held;
    const dt = steps * physDt;
    const c = p.control.mrac;
    const m = input.sense();
    const y = m.pos.y;
    const v = m.vel.y;
    const r = input.setpoint.pos.y;
    const theta0 = thetaFor(p.control.model.mass, p);
    this.state ??= { theta: [...theta0], ym: y, vm: v };
    const st = this.state;
    if (this.hold) {
      st.ym = y;
      st.vm = v;
    }
    const w = [r - y, -v, 1];
    const e1 = y - st.ym;
    const e2 = v - st.vm;
    const [, p12, p22] = lyapunovP(c.kp, c.kd);
    const bpx = p12 * e1 + p22 * e2;
    const unsaturated = st.theta.reduce((s, th, i) => s + th * w[i]!, 0);
    const output = clamp(unsaturated, 0, 4 * p.drone.maxMotorThrust);
    if (!this.hold) {
      // γ scales the adaptation of the two gains; the hover parameter has its own, fixed rate.
      const gains = [c.gammaP * c.gamma, c.gammaD * c.gamma, c.gammaG];
      for (let i = 0; i < 3; i++) {
        const g = gains[i]!;
        // While the motors are saturated the error is not the parameters' fault: hold them.
        if (output !== unsaturated) continue;
        // σ-modification on the two feedback gains only: the hover thrust's regressor is the
        // constant 1, always excited, and pulling it towards a wrong guess would only make the
        // drone sag.
        const leak = i < 2 ? c.sigma * g * (st.theta[i]! - theta0[i]!) : 0;
        const next = st.theta[i]! + dt * (-g * w[i]! * bpx - leak);
        st.theta[i] = clamp(next, -1e4, 1e4);
      }
    }
    // The reference model, one step on.
    const am = c.kp * (r - st.ym) - c.kd * st.vm;
    st.vm += am * dt;
    st.ym += st.vm * dt;
    this.hold = false;
    this.last = {
      setpoint: r,
      measurement: y,
      error: r - y,
      parts: [
        { key: 'p', label: 'θ̂₁·(r − y)', value: st.theta[0] * w[0]!, like: 'p' },
        { key: 'd', label: 'θ̂₂·(−ẏ)', value: st.theta[1] * w[1]!, like: 'd' },
        { key: 'g', label: 'θ̂₃ (hover)', value: st.theta[2], like: 'i' },
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

  extras(): Record<string, number> {
    const st = this.state;
    if (!st) return {};
    return {
      'mrac.theta.p': st.theta[0],
      'mrac.theta.d': st.theta[1],
      'mrac.theta.g': st.theta[2],
      'mrac.ym': st.ym,
      'mrac.err': (this.last.measurement ?? 0) - st.ym,
    };
  }

  describe(p: Params): InfoRow[] {
    const c = p.control.mrac;
    const st = this.state;
    const ideal = thetaFor(p.drone.mass, p);
    return [
      {
        label: 'reference model',
        value: `ÿ = ${c.kp}·(r − y) − ${c.kd}·ẏ`,
        hint: 'The loop the controller tries to make the drone behave like.',
      },
      {
        label: 'θ̂ (adapted) · θ* (ideal)',
        value: st
          ? `${st.theta.map((t) => t.toFixed(2)).join(', ')} · ${ideal.map((t) => t.toFixed(2)).join(', ')}`
          : '—',
        hint: 'θ* = m·(kp, kd, g) for the true mass: what the adaptation should find.',
      },
    ];
  }
}
