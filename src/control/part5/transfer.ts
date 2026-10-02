// Lesson V.5: a change of controller in flight. The PID flies until `transfer.at`, then the LQI
// takes over. Without preparation its integrator starts at zero; with a bumpless transfer it is
// preset so that the first command equals the last one of the PID.
import type { Actuation } from '@/sim/dynamics';
import type { Params } from '@/sim/params';
import { AltitudeController } from '../altitude';
import { LqrController } from '../lqr';
import { periodSteps, type ControlInput, type Controller } from '../types';
import { oncePerStep } from './shadow';

export class TransferController implements Controller {
  readonly pid = new AltitudeController();
  readonly lqr = new LqrController();
  private active: 'pid' | 'lqr' = 'pid';
  /** The command the LQI would have sent at the hand-over, less the PID's, N. */
  jump = NaN;
  /** Held until the LQI's next sample, so the hand-over sample sends the PID's command. */
  private override: Actuation | null = null;

  reset(): void {
    this.pid.reset();
    this.lqr.reset();
    this.active = 'pid';
    this.jump = NaN;
    this.override = null;
  }

  resetIntegrators(): void {
    this.pid.resetIntegrators();
    this.lqr.resetIntegrators();
  }

  tick(input: ControlInput): Actuation {
    const run = { ...input, sense: oncePerStep(input) };
    const { params: p, physDt, stepIndex } = input;
    const sample = stepIndex % periodSteps(p.control.rateHz, physDt) === 0;
    if (this.active === 'lqr') {
      if (this.override && !sample) return this.override;
      this.override = null;
      return this.lqr.tick(run);
    }
    const fromPid = this.pid.tick(run);
    if (!sample || stepIndex * physDt < p.part5.transfer.at) return fromPid;

    // The hand-over, at a sample of both.
    this.active = 'lqr';
    const uPid = this.pid.alt.last.output;
    const fromLqr = this.lqr.tick(run);
    const u0 = this.lqr.loops().alt.unsaturated;
    this.jump = u0 - uPid;
    const ki = this.lqr.integralGain(p);
    if (!p.part5.transfer.bumpless || ki === 0) return fromLqr;
    // −kᵢ·Δξ = uPid − u0: the LQI starts from the command the PID left.
    this.lqr.shiftIntegral((u0 - uPid) / ki);
    this.override = fromPid;
    return fromPid;
  }

  loops() {
    return this.active === 'pid' ? this.pid.loops() : this.lqr.loops();
  }

  describe(p: Params) {
    return this.active === 'lqr' ? this.lqr.describe(p) : [];
  }

  extras() {
    return {
      ...(this.active === 'lqr' ? this.lqr.extras() : {}),
      'xfer.lqr': this.active === 'lqr' ? 1 : 0,
      'xfer.jump': this.jump,
    };
  }
}
