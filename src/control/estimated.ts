import { AltitudeKalman } from '@/estimation/kalman';
import { v3 } from '@/math/vec3';
import type { Actuation } from '@/sim/dynamics';
import { GRAVITY, type Params } from '@/sim/params';
import type { Measurement } from '@/sim/sensors';
import type { ControlInput, Controller, InfoRow } from './types';

/**
 * Puts a Kalman filter between the sensors and any L1 controller: the controller reads the
 * filter's altitude and vertical speed instead of the raw sensors. The filter runs whenever the
 * controller samples, fusing the accelerometer (every sample) with position fixes (when fresh).
 */
export class WithAltitudeKalman implements Controller {
  readonly kf = new AltitudeKalman();
  private lastStep = -1;

  constructor(readonly inner: Controller) {}

  reset(): void {
    this.inner.reset();
    this.kf.reset();
    this.lastStep = -1;
  }

  resetIntegrators(): void {
    this.inner.resetIntegrators();
  }

  tick(input: ControlInput): Actuation {
    const sense = (): Measurement => {
      const m = input.sense();
      if (this.lastStep !== input.stepIndex) {
        const dt = this.lastStep < 0 ? 0 : (input.stepIndex - this.lastStep) * input.physDt;
        this.lastStep = input.stepIndex;
        this.kf.step(
          dt,
          m.acc.y,
          m.posFresh ? m.pos.y : null,
          input.params.control.kalman,
          GRAVITY,
        );
      }
      return { ...m, pos: v3(m.pos.x, this.kf.y, m.pos.z), vel: v3(m.vel.x, this.kf.v, m.vel.z) };
    };
    return this.inner.tick({ ...input, sense });
  }

  loops() {
    return this.inner.loops();
  }

  extras() {
    const s = this.kf.sigma(0);
    return {
      ...this.inner.extras(),
      'est.y': this.kf.y,
      'est.y.hi': this.kf.y + 2 * s,
      'est.y.lo': this.kf.y - 2 * s,
      'est.v': this.kf.v,
      'est.bias': this.kf.bias,
      'est.sigma.y': s,
    };
  }

  describe(p: Params, loopId: string): InfoRow[] {
    return [
      ...(this.inner.describe?.(p, loopId) ?? []),
      {
        label: 'Kalman σ(y) · σ(v)',
        value: `${(this.kf.sigma(0) * 100).toFixed(1)} cm · ${this.kf.sigma(1).toFixed(3)} m/s`,
        hint: 'The filter’s own estimate of its uncertainty (1σ). Only as honest as its noise settings.',
      },
      {
        label: 'estimated accel. bias',
        value: `${this.kf.bias.toFixed(3)} m/s²`,
      },
    ];
  }
}
