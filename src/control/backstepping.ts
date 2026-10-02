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
 * Level 1, backstepping through the motor lag (docs/analysis.md §3.5, lesson III.16).
 *
 * The altitude loop is a chain of three integrators as the controller models it:
 *   ė = v,   m̂·v̇ = T,   τ̂·Ṫ = u − T      (T and u: thrust and command less the hover thrust)
 * Backstepping walks down the chain one step at a time. Each step picks a "virtual control" that
 * would make the step before it converge, and the next step makes the real state follow it:
 *   z₁ = e,              α₁ = −k₁·z₁             (the velocity the position wants)
 *   z₂ = v − α₁,         α₂ = m̂·(−z₁ − k₂·z₂ − k₁·v)   (the thrust the velocity wants)
 *   z₃ = T − α₂,         u  = T + τ̂·(α̇₂ − z₂/m̂ − k₃·z₃)
 * With V = ½(z₁² + z₂² + z₃²) every cross term cancels and V̇ = −k₁z₁² − k₂z₂² − k₃z₃²: the proof
 * of stability comes with the design. For this linear chain the law is linear in (e, v, T).
 */
export interface BacksteppingGains {
  /** Gain on each step: 1/s. */
  k1: number;
  k2: number;
  k3: number;
}

/** u = −kE·e − kV·v − kT·T: the backstepping law written as a state feedback. */
export function backsteppingGains(p: Params): { kE: number; kV: number; kT: number } {
  const { k1, k2, k3 } = p.control.backstepping;
  const m = Math.max(p.control.model.mass, 0.05);
  const tau = Math.max(p.control.model.motorTau, 1e-3);
  // u is linear in x = (e, v, T); evaluate it on the unit vectors.
  const law = (e: number, v: number, t: number): number => {
    const z1 = e;
    const z2 = v + k1 * e;
    const a2 = m * (-z1 - k2 * z2 - k1 * v);
    const z3 = t - a2;
    // α̇₂ = −m̂·(1 + k₁k₂)·v − (k₁ + k₂)·T
    const a2dot = -m * (1 + k1 * k2) * v - (k1 + k2) * t;
    return t + tau * (a2dot - z2 / m - k3 * z3);
  };
  return { kE: -law(1, 0, 0), kV: -law(0, 1, 0), kT: -law(0, 0, 1) };
}

/** V = ½(z₁² + z₂² + z₃²) for the state (e, v, T): the Lyapunov function of the design. */
export function backsteppingV(p: Params, e: number, v: number, t: number): number {
  const { k1, k2 } = p.control.backstepping;
  const m = Math.max(p.control.model.mass, 0.05);
  const z1 = e;
  const z2 = v + k1 * e;
  const z3 = t - m * (-z1 - k2 * z2 - k1 * v);
  return 0.5 * (z1 * z1 + z2 * z2 + z3 * z3);
}

export class BacksteppingController implements Controller {
  private held: Actuation = idleActuation();
  /** Model estimate of the actual thrust when there is no motor telemetry. */
  private thrustEst = 0;
  private lastCmd = 0;
  private v = 0;
  private last: LoopTerms = {
    setpoint: 0,
    measurement: 0,
    error: 0,
    parts: [],
    unsaturated: 0,
    output: 0,
    saturated: false,
  };

  reset(): void {
    this.held = idleActuation();
    this.thrustEst = 0;
    this.lastCmd = 0;
    this.v = 0;
  }

  resetIntegrators(): void {}

  tick(input: ControlInput): Actuation {
    const { params: p, physDt, stepIndex } = input;
    const n = periodSteps(p.control.rateHz, physDt);
    if (stepIndex % n !== 0) return this.held;
    const dt = n * physDt;
    const m = input.sense();
    const g = backsteppingGains(p);
    const ff = p.control.feedforward ? p.control.model.mass * GRAVITY : 0;
    const a = 1 - Math.exp(-dt / Math.max(p.control.model.motorTau, 1e-3));
    this.thrustEst += (this.lastCmd - this.thrustEst) * a;
    const thrust = m.motors
      ? m.motors[0] + m.motors[1] + m.motors[2] + m.motors[3]
      : this.thrustEst;
    const sp = input.setpoint.pos.y;
    const e = m.pos.y - sp;
    const t = thrust - ff;
    const parts: LoopTerms['parts'] = [
      { key: 'pos', label: '−k_e·e', value: -g.kE * e, like: 'p' },
      { key: 'vel', label: '−k_v·v', value: -g.kV * m.vel.y, like: 'd' },
      { key: 'lag', label: '−k_T·T', value: -g.kT * t },
      { key: 'ff', label: 'FF', value: ff, ff: true, like: 'ff' },
    ];
    const unsaturated = parts.reduce((s, q) => s + q.value, 0);
    const output = clamp(unsaturated, 0, 4 * p.drone.maxMotorThrust);
    this.lastCmd = output;
    this.v = backsteppingV(p, e, m.vel.y, t);
    this.last = {
      setpoint: sp,
      measurement: m.pos.y,
      error: sp - m.pos.y,
      parts,
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
    return { 'bs.V': this.v };
  }

  describe(p: Params): InfoRow[] {
    const g = backsteppingGains(p);
    const { k1, k2, k3 } = p.control.backstepping;
    return [
      {
        label: 'steps k₁ · k₂ · k₃',
        value: `${k1} · ${k2} · ${k3} 1/s`,
        hint: 'Each step converges at its own rate: V̇ = −k₁z₁² − k₂z₂² − k₃z₃².',
      },
      {
        label: 'as a state feedback',
        value: `u = −${g.kE.toFixed(1)}·e − ${g.kV.toFixed(2)}·v − ${g.kT.toFixed(2)}·T`,
        hint: 'For a linear chain the recursive design ends in an ordinary state feedback, with gains no one had to guess.',
      },
    ];
  }
}
