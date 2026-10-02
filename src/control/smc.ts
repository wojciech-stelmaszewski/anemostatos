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
 * Level 1, sliding-mode control (docs/analysis.md §4, Chapter H; [Slotine 1991]).
 *
 * The sliding variable s = ė + λ·e (e = r − y) is zero on a line in the phase plane along which
 * the error decays as e^{−λt}. The law has two parts:
 *   u = m̂·(g + λ·ė)              the equivalent control: what keeps s constant for the model
 *     + k·sat(s/φ)               the switching term: pushes towards s = 0 with force k
 * For any disturbance force smaller than k, s reaches zero in finite time and stays there: the
 * drone slides to the target whatever the disturbance. With φ = 0 the switching term is sign(s),
 * which flips at the loop rate (chatter). Inside a boundary layer |s| < φ the law is linear, a PD
 * with Kp = k·λ/φ and Kd = k/φ + m̂·λ, which trades the chatter for a small steady error.
 */
export class SlidingModeController implements Controller {
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
  /** The sliding variable at the last sample, m/s. */
  s = 0;

  reset(): void {
    this.held = idleActuation();
    this.s = 0;
  }

  resetIntegrators(): void {}

  tick(input: ControlInput): Actuation {
    const { params: p, physDt, stepIndex } = input;
    const steps = periodSteps(p.control.rateHz, physDt);
    if (stepIndex % steps !== 0) return this.held;
    const c = p.control.smc;
    const m = input.sense();
    const sp = input.setpoint.pos.y;
    const spVel = input.setpoint.vel?.y ?? 0;
    const e = sp - m.pos.y;
    const de = spVel - m.vel.y;
    const s = de + c.lambda * e;
    this.s = s;
    const mHat = p.control.model.mass;
    const ff = mHat * GRAVITY;
    const eq = mHat * c.lambda * de;
    const sw = c.k * (c.phi > 0 ? clamp(s / c.phi, -1, 1) : Math.sign(s));
    const unsaturated = ff + eq + sw;
    const output = clamp(unsaturated, 0, 4 * p.drone.maxMotorThrust);
    this.last = {
      setpoint: sp,
      measurement: m.pos.y,
      error: e,
      parts: [
        { key: 'eq', label: 'm̂·λ·ė (equivalent)', value: eq, like: 'd' },
        { key: 'sw', label: 'k·sat(s/φ) (switching)', value: sw, like: 'p' },
        { key: 'ff', label: 'm̂·g', value: ff, ff: true, like: 'ff' },
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
    return { 'smc.s': this.s };
  }

  describe(p: Params): InfoRow[] {
    const c = p.control.smc;
    const rows: InfoRow[] = [
      {
        label: 'sliding line',
        value: `ė = −${c.lambda.toFixed(2)}·e`,
        hint: 'On the line s = 0 the error decays as e^{−λt}, whatever the drone weighs.',
      },
      {
        label: 'switching force k',
        value: `${c.k.toFixed(2)} N`,
        hint: 'Any disturbance force smaller than k is rejected exactly once the state is on the line.',
      },
    ];
    if (c.phi > 0)
      rows.push({
        label: 'inside the layer: Kp · Kd',
        value: `${((c.k * c.lambda) / c.phi).toFixed(1)} · ${(c.k / c.phi + p.control.model.mass * c.lambda).toFixed(1)}`,
        hint: 'Within |s| < φ the law is a PD controller: no chatter, but a steady error under a constant load.',
      });
    return rows;
  }
}
