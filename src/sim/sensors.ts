import { Rng } from '@/math/prng';
import { qFromAxisAngle, qMul, type Quat } from '@/math/quat';
import { clone, v3, type Vec3 } from '@/math/vec3';
import type { DroneState } from './dynamics';
import type { SensorParams } from './params';

/** What the controller gets to see. An idealised state estimate plus optional imperfections. */
export interface Measurement {
  pos: Vec3;
  vel: Vec3;
  q: Quat;
  omega: Vec3;
}

const HISTORY = 1001; // 1 s of delay at 1 kHz

export const measurementOf = (s: DroneState): Measurement => ({
  pos: clone(s.pos),
  vel: clone(s.vel),
  q: { ...s.q },
  omega: clone(s.omega),
});

export class Sensors {
  private history: Measurement[] = [];
  private head = 0;
  private rng: Rng;

  constructor(seed: number) {
    this.rng = new Rng(seed ^ 0x27d4eb2f);
  }

  /** Store the true state; call once per physics step. */
  record(s: DroneState): void {
    const m = measurementOf(s);
    if (this.history.length < HISTORY) this.history.push(m);
    else this.history[this.head] = m;
    this.head = (this.head + 1) % HISTORY;
  }

  /** Delayed, biased, noisy reading of the recorded state. */
  read(p: SensorParams, physDt: number): Measurement {
    const lag = Math.min(this.history.length - 1, Math.round(p.delayMs / 1000 / physDt));
    const truth = this.history[(this.head - 1 - lag + HISTORY) % HISTORY]!;
    const r = this.rng;
    const noisy = (v: Vec3, s: number): Vec3 =>
      s > 0 ? v3(v.x + s * r.normal(), v.y + s * r.normal(), v.z + s * r.normal()) : clone(v);
    const gyro = (p.gyroNoise * Math.PI) / 180;
    const pos = noisy(truth.pos, p.posNoise);
    pos.y += p.altBias;
    let q = truth.q;
    if (gyro > 0) {
      // Small attitude jitter consistent with the gyro noise level.
      const e = v3(r.normal(), r.normal(), r.normal());
      q = qMul(q, qFromAxisAngle(e, gyro * 0.01));
    }
    return { pos, vel: noisy(truth.vel, p.velNoise), q, omega: noisy(truth.omega, gyro) };
  }

  reset(): void {
    this.history = [];
    this.head = 0;
  }
}
