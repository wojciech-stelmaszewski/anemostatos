import { clampLength, v3 } from '@/math/vec3';
import { idleActuation, type Actuation } from '@/sim/dynamics';
import { GRAVITY } from '@/sim/params';
import { Pid } from './pid';
import { periodSteps, type ControlInput, type Controller } from './types';

/**
 * Level 2: three independent position PIDs producing a force vector directly
 * (docs/control-architecture.md §3). Physically a cheat — real drones must tilt.
 */
export class PointMassController implements Controller {
  readonly x = new Pid();
  readonly y = new Pid();
  readonly z = new Pid();
  private held: Actuation = idleActuation();

  reset(): void {
    this.x.reset();
    this.y.reset();
    this.z.reset();
    this.held = idleActuation();
  }

  resetIntegrators(): void {
    this.x.integral = this.y.integral = this.z.integral = 0;
  }

  tick(input: ControlInput): Actuation {
    const { params: p, physDt, stepIndex, setpoint } = input;
    const n = periodSteps(p.control.rateHz, physDt);
    if (stepIndex % n !== 0) return this.held;

    const m = input.sense();
    const dt = n * physDt;
    const h = p.control.maxHForce;
    const ff = p.control.feedforward ? p.control.massEstimate * GRAVITY : 0;
    const fx = this.x.update(p.control.posH, setpoint.pos.x, m.pos.x, dt, 0, -h, h).output;
    const fz = this.z.update(p.control.posH, setpoint.pos.z, m.pos.z, dt, 0, -h, h).output;
    const fy = this.y.update(
      p.control.alt,
      setpoint.pos.y,
      m.pos.y,
      dt,
      ff,
      0,
      4 * p.drone.maxMotorThrust,
    ).output;
    const horizontal = clampLength(v3(fx, 0, fz), h);
    this.held = { motorCmd: this.held.motorCmd, forceCmd: v3(horizontal.x, fy, horizontal.z) };
    return this.held;
  }

  loops() {
    return { 'pos.x': this.x.last, 'pos.y': this.y.last, 'pos.z': this.z.last };
  }

  extras() {
    return {};
  }
}
