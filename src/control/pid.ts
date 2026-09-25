import { clamp, lowPassAlpha } from '@/math/util';

export type AntiWindup = 'none' | 'clamp' | 'integral-limit' | 'back-calculation';
export type DerivativeOn = 'error' | 'measurement';

/** User-tunable settings of one PID loop. Plain data: lives in the parameter tree. */
export interface PidGains {
  kp: number;
  ki: number;
  kd: number;
  /** Per-term enable switches, for the lessons. */
  pOn: boolean;
  iOn: boolean;
  dOn: boolean;
  derivativeOn: DerivativeOn;
  /** Derivative low-pass cutoff in Hz; 0 disables the filter. */
  dFilterHz: number;
  antiWindup: AntiWindup;
  /** |I| bound for 'integral-limit'. */
  iLimit: number;
  /** Tracking gain for 'back-calculation', 1/s. */
  kb: number;
}

/** Everything the PID computed in one update — what the charts plot. */
export interface PidTerms {
  setpoint: number;
  measurement: number;
  error: number;
  p: number;
  i: number;
  d: number;
  ff: number;
  /** p + i + d + ff before clamping. */
  unsaturated: number;
  output: number;
  saturated: boolean;
}

const emptyTerms = (): PidTerms => ({
  setpoint: 0,
  measurement: 0,
  error: 0,
  p: 0,
  i: 0,
  d: 0,
  ff: 0,
  unsaturated: 0,
  output: 0,
  saturated: false,
});

/**
 * Discrete PID in parallel form with the practical features described in docs/pid-primer.md §5.
 * Gains are passed on every update, so changing them live needs no re-initialisation.
 */
export class Pid {
  /** Already scaled by Ki: I = Σ Ki·e·dt. Changing Ki therefore does not jump the output. */
  integral = 0;
  last: PidTerms = emptyTerms();
  private prevError = 0;
  private prevMeasurement = 0;
  private dFiltered = 0;
  private hasPrev = false;

  reset(integral = 0): void {
    this.integral = integral;
    this.hasPrev = false;
    this.dFiltered = 0;
    this.last = emptyTerms();
  }

  update(
    g: PidGains,
    setpoint: number,
    measurement: number,
    dt: number,
    ff = 0,
    outMin = -Infinity,
    outMax = Infinity,
  ): PidTerms {
    const error = setpoint - measurement;
    const p = g.pOn ? g.kp * error : 0;

    // Derivative: of the error (kicks on setpoint steps) or of −measurement (no kick).
    let dRaw = 0;
    if (this.hasPrev && dt > 0) {
      dRaw =
        g.derivativeOn === 'error'
          ? (error - this.prevError) / dt
          : -(measurement - this.prevMeasurement) / dt;
    }
    if (g.dFilterHz > 0 && this.hasPrev) {
      const a = lowPassAlpha(g.dFilterHz, dt);
      this.dFiltered = a * this.dFiltered + (1 - a) * dRaw;
    } else {
      this.dFiltered = dRaw;
    }
    const d = g.dOn ? g.kd * this.dFiltered : 0;

    // Integral with the selected anti-windup strategy.
    if (!g.iOn) {
      this.integral = 0;
    } else if (g.antiWindup !== 'back-calculation') {
      let next = this.integral + g.ki * error * dt;
      if (g.antiWindup === 'integral-limit') next = clamp(next, -g.iLimit, g.iLimit);
      if (g.antiWindup === 'clamp') {
        // Conditional integration: freeze while saturated and the error pushes deeper.
        const u = p + next + d + ff;
        const pushingUp = u > outMax && error > 0;
        const pushingDown = u < outMin && error < 0;
        if (pushingUp || pushingDown) next = this.integral;
      }
      this.integral = next;
    }

    const i = this.integral;
    const unsaturated = p + i + d + ff;
    const output = clamp(unsaturated, outMin, outMax);

    if (g.iOn && g.antiWindup === 'back-calculation') {
      // Bleed the integrator by the amount the actuator could not deliver.
      this.integral += (g.ki * error + g.kb * (output - unsaturated)) * dt;
    }

    this.prevError = error;
    this.prevMeasurement = measurement;
    this.hasPrev = true;

    this.last = {
      setpoint,
      measurement,
      error,
      p,
      i,
      d,
      ff,
      unsaturated,
      output,
      saturated: output !== unsaturated,
    };
    return this.last;
  }
}

export const defaultGains = (over: Partial<PidGains> = {}): PidGains => ({
  kp: 1,
  ki: 0,
  kd: 0,
  pOn: true,
  iOn: true,
  dOn: true,
  derivativeOn: 'measurement',
  dFilterHz: 20,
  antiWindup: 'clamp',
  iLimit: 5,
  kb: 2,
  ...over,
});
