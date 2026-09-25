import { GRAVITY } from '@/sim/params';
import { idleActuation, type Actuation } from '@/sim/dynamics';
import { Pid } from './pid';
import { periodSteps, type ControlInput, type Controller } from './types';

/**
 * Level 1: a single PID on altitude driving collective thrust (docs/control-architecture.md §2).
 *   T = m̂·g (feedforward) + PID(y_sp − y), limited to [0, 4·f_max].
 */
export class AltitudeController implements Controller {
  readonly alt = new Pid();
  private held: Actuation = idleActuation();

  reset(): void {
    this.alt.reset();
    this.held = idleActuation();
  }

  resetIntegrators(): void {
    this.alt.integral = 0;
  }

  tick(input: ControlInput): Actuation {
    const { params: p, physDt, stepIndex } = input;
    const n = periodSteps(p.control.rateHz, physDt);
    if (stepIndex % n !== 0) return this.held; // zero-order hold between samples

    const m = input.sense();
    const ff = p.control.feedforward ? p.control.massEstimate * GRAVITY : 0;
    const tMax = 4 * p.drone.maxMotorThrust;
    const t = this.alt.update(
      p.control.alt,
      input.setpoint.pos.y,
      m.pos.y,
      n * physDt,
      ff,
      0,
      tMax,
    );
    const f = t.output / 4;
    this.held = { motorCmd: [f, f, f, f], forceCmd: this.held.forceCmd };
    return this.held;
  }

  loops() {
    return { alt: this.alt.last };
  }

  extras() {
    return {};
  }
}
