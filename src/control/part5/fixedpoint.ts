// The altitude PID as flight code (lesson V.3): integers only, the way a small flight computer
// without floating point runs it, compared back to back with the floating-point reference.
import { lowPassAlpha } from '@/math/util';
import type { Params } from '@/sim/params';
import type { PidGains } from '../pid';

/** Wrap an integer into a signed word of `bits` bits, as two's-complement hardware does. */
export const wrap = (v: number, bits: number): number => {
  const m = 2 ** bits;
  const h = m / 2;
  return ((((v + h) % m) + m) % m) - h;
};

/** Signed 16-bit range of the gains; 32 bits for the signals. */
const sat16 = (v: number) => Math.max(-32768, Math.min(32767, v));
const sat32 = (v: number) => Math.max(-(2 ** 31), Math.min(2 ** 31 - 1, v));

/**
 * The smallest error the integrator can see, m: below it, ki·e·T rounds to zero counts of the
 * integrator's least significant bit 2^−intFrac.
 */
export const integratorDeadZone = (ki: number, T: number, intFrac: number): number =>
  ki > 0 ? 2 ** -intFrac / (ki * T) : Infinity;

/** The largest force the integrator can hold before it wraps, N. */
export const integratorRange = (intBits: number, intFrac: number): number =>
  2 ** (intBits - 1 - intFrac);

/**
 * The PID of `src/control/pid.ts` (derivative on the measurement, conditional integration) in
 * integers. Signals are 32-bit words: micrometres for the altitude and the error, µm/s for the
 * derivative, micronewtons for the output. Gains are Q8.8 (16 bits, steps of 1/256). The integrator
 * is a word of `intBits` bits with `intFrac` fractional bits, in newtons. Products are formed in a
 * wide accumulator and shifted back, as a DSP does.
 */
export class FixedPid {
  /** The integrator, in counts of 2^−intFrac N. */
  integral = 0;
  private prev = 0;
  private dFilt = 0;
  private hasPrev = false;
  /** The last output, N, and whether the integrator wrapped. */
  output = 0;
  wrapped = false;

  reset(): void {
    this.integral = 0;
    this.dFilt = 0;
    this.hasPrev = false;
    this.output = 0;
    this.wrapped = false;
  }

  update(
    g: PidGains,
    fx: Params['part5']['fixed'],
    setpoint: number,
    measurement: number,
    dt: number,
    ff: number,
    outMax: number,
  ): number {
    const q8 = (k: number) => sat16(Math.round(k * 256));
    const kp = q8(g.kp);
    const ki = q8(g.ki);
    const kd = q8(g.kd);
    const yUm = sat32(Math.round(measurement * 1e6));
    const eUm = sat32(Math.round(setpoint * 1e6) - yUm);

    const p = g.pOn ? Math.floor((kp * eUm) / 256) : 0; // µN

    // Derivative of −y in µm/s, through the same one-pole filter, its coefficient in Q15.
    if (this.hasPrev && dt > 0) {
      const raw = Math.round(-(yUm - this.prev) / dt);
      if (g.dFilterHz > 0) {
        const a = Math.round(lowPassAlpha(g.dFilterHz, dt) * 32768);
        this.dFilt = Math.floor((a * this.dFilt + (32768 - a) * raw) / 32768);
      } else this.dFilt = raw;
    }
    const d = g.dOn ? Math.floor((kd * this.dFilt) / 256) : 0; // µN

    const ffUn = Math.round(ff * 1e6);
    const maxUn = Math.round(outMax * 1e6);
    const scale = 2 ** fx.intFrac;
    const iUn = (counts: number) => Math.floor((counts * 1e6) / scale);

    // ki·e·dt in counts of the integrator: (Q8.8 gain × µm × s) shifted, truncated to an integer.
    let next = 0;
    if (g.iOn) {
      const inc = Math.trunc((ki * eUm * dt * scale) / (256 * 1e6));
      const sum = this.integral + inc;
      next = wrap(sum, fx.intBits);
      if (next !== sum) this.wrapped = true;
      const u = ffUn + p + iUn(next) + d;
      if ((u > maxUn && eUm > 0) || (u < 0 && eUm < 0)) next = this.integral;
    }
    this.integral = next;

    const u = Math.max(0, Math.min(maxUn, ffUn + p + iUn(this.integral) + d));
    this.prev = yUm;
    this.hasPrev = true;
    this.output = u / 1e6;
    return this.output;
  }
}
