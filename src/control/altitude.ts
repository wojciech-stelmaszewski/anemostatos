import { GRAVITY } from '@/sim/params';
import { idleActuation, type Actuation } from '@/sim/dynamics';
import { shapingActive, shapingLti, StageRunner } from '@/analysis/shaping';
import { clamp } from '@/math/util';
import { Pid } from './pid';
import { periodSteps, pidLoop, type ControlInput, type Controller } from './types';

/**
 * Level 1: a single PID on altitude driving collective thrust (docs/control-architecture.md §2).
 *   T = m̂·g (feedforward) + PID(y_sp − y), limited to [0, 4·f_max].
 */
export class AltitudeController implements Controller {
  readonly alt = new Pid();
  private held: Actuation = idleActuation();
  private stages = new StageRunner();

  reset(): void {
    this.alt.reset();
    this.held = idleActuation();
    this.stages.reset();
  }

  resetIntegrators(): void {
    this.alt.integral = 0;
  }

  tick(input: ControlInput): Actuation {
    const { params: p, physDt, stepIndex } = input;
    const n = periodSteps(p.control.rateHz, physDt);
    if (stepIndex % n !== 0) return this.held; // zero-order hold between samples

    const m = input.sense();
    const ff = p.control.feedforward ? p.control.model.mass * GRAVITY : 0;
    const tMax = 4 * p.drone.maxMotorThrust;
    const dt = n * physDt;
    let out: number;
    if (shapingActive(p)) {
      // The stages act on the PID's feedback; the limits apply to what reaches the motors.
      const t = this.alt.update(p.control.alt, input.setpoint.pos.y, m.pos.y, dt, ff, -1e9, 1e9);
      const g = shapingLti(p, dt)!;
      const shaped = this.stages.step(g, JSON.stringify(p.control.shaping) + dt, t.output - ff);
      out = clamp(ff + shaped, 0, tMax);
    } else {
      out = this.alt.update(p.control.alt, input.setpoint.pos.y, m.pos.y, dt, ff, 0, tMax).output;
    }
    const f = out / 4;
    this.held = { motorCmd: [f, f, f, f], forceCmd: this.held.forceCmd };
    return this.held;
  }

  loops() {
    return { alt: pidLoop(this.alt.last) };
  }

  extras() {
    return {};
  }
}
