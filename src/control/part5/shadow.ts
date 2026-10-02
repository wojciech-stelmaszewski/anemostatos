// Lesson V.3: a fixed-point copy of the altitude PID runs beside the flight, on the same sensor
// readings as the floating-point reference, and the two outputs are recorded for comparison.
import { GRAVITY, type Params } from '@/sim/params';
import type { Actuation } from '@/sim/dynamics';
import type { Measurement } from '@/sim/sensors';
import { Pid } from '../pid';
import { periodSteps, type ControlInput, type Controller } from '../types';
import { FixedPid } from './fixedpoint';

/** Read the sensors at most once per physics step, so a wrapper and the controller see the same. */
export const oncePerStep = (input: ControlInput): (() => Measurement) => {
  let m: Measurement | null = null;
  return () => (m ??= input.sense());
};

export class WithFixedPoint implements Controller {
  readonly reference = new Pid();
  readonly fixed = new FixedPid();
  private ref = NaN;
  private fix = NaN;
  private hover = 1;
  private iFixed = NaN;

  constructor(readonly inner: Controller) {}

  reset(): void {
    this.inner.reset();
    this.reference.reset();
    this.fixed.reset();
    this.ref = NaN;
    this.fix = NaN;
  }

  resetIntegrators(): void {
    this.inner.resetIntegrators();
    // Flight code zeros both on the ground, as the controller does.
    this.reference.integral = 0;
    this.fixed.integral = 0;
  }

  tick(input: ControlInput): Actuation {
    const sense = oncePerStep(input);
    const { params: p, physDt, stepIndex } = input;
    const n = periodSteps(p.control.rateHz, physDt);
    if (stepIndex % n === 0) {
      const m = sense();
      const dt = n * physDt;
      const ff = p.control.feedforward ? p.control.model.mass * GRAVITY : 0;
      const tMax = 4 * p.drone.maxMotorThrust;
      const sp = input.setpoint.pos.y;
      this.ref = this.reference.update(p.control.alt, sp, m.pos.y, dt, ff, 0, tMax).output;
      this.fix = this.fixed.update(p.control.alt, p.part5.fixed, sp, m.pos.y, dt, ff, tMax);
      this.hover = p.control.model.mass * GRAVITY;
      this.iFixed = this.fixed.integral / 2 ** p.part5.fixed.intFrac;
    }
    return this.inner.tick({ ...input, sense });
  }

  loops() {
    return this.inner.loops();
  }

  describe(p: Params, loopId: string) {
    return this.inner.describe?.(p, loopId) ?? [];
  }

  extras() {
    return {
      ...this.inner.extras(),
      'fx.ref': this.ref,
      'fx.fixed': this.fix,
      // The disagreement in per cent of the hover thrust.
      'fx.err': (Math.abs(this.fix - this.ref) / this.hover) * 100,
      'fx.iref': this.reference.integral,
      'fx.ifixed': this.iFixed,
    };
  }
}
