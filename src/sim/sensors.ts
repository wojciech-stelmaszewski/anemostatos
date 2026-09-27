import { Rng } from '@/math/prng';
import { qConj, qFromAxisAngle, qMul, qRotate, type Quat } from '@/math/quat';
import { add, clone, v3, type Vec3 } from '@/math/vec3';
import type { DroneState } from './dynamics';
import { GRAVITY, type SensorParams } from './params';

/** What the controller gets to see. An idealised state estimate plus optional imperfections. */
export interface Measurement {
  pos: Vec3;
  vel: Vec3;
  q: Quat;
  omega: Vec3;
  /** Accelerometer: specific force (acceleration minus gravity), body frame, m/s². */
  acc: Vec3;
  /** Actual thrust per motor from motor-speed telemetry, N; null if unavailable. */
  motors: [number, number, number, number] | null;
  /** True when `pos` is a new fix (false while a slow position sensor holds its last value). */
  posFresh: boolean;
}

const HISTORY = 1001; // 1 s of delay at 1 kHz

export const measurementOf = (s: DroneState): Measurement => ({
  pos: clone(s.pos),
  vel: clone(s.vel),
  q: { ...s.q },
  omega: clone(s.omega),
  acc: qRotate(qConj(s.q), add(s.accel, v3(0, GRAVITY, 0))),
  motors: [...s.motors],
  posFresh: true,
});

export class Sensors {
  private history: Measurement[] = [];
  private head = 0;
  private rng: Rng;
  /** Last position sample and when the next one is due (for posRateHz). */
  private heldPos: Vec3 | null = null;
  private nextPosT = 0;

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

  /** Delayed, biased, noisy reading of the recorded state at simulation time t. */
  read(p: SensorParams, physDt: number, t = 0): Measurement {
    const lag = Math.min(this.history.length - 1, Math.round(p.delayMs / 1000 / physDt));
    const truth = this.history[(this.head - 1 - lag + HISTORY) % HISTORY]!;
    const r = this.rng;
    const noisy = (v: Vec3, s: number): Vec3 =>
      s > 0 ? v3(v.x + s * r.normal(), v.y + s * r.normal(), v.z + s * r.normal()) : clone(v);
    const gyro = (p.gyroNoise * Math.PI) / 180;
    let pos: Vec3;
    let posFresh = true;
    if (p.posRateHz > 0 && this.heldPos && t < this.nextPosT - 1e-9) {
      pos = clone(this.heldPos); // sample-and-hold between position fixes
      posFresh = false;
    } else {
      pos = noisy(truth.pos, p.posNoise);
      pos.y += p.altBias;
      if (p.posRateHz > 0) {
        this.heldPos = clone(pos);
        this.nextPosT = (Math.floor(t * p.posRateHz + 1e-6) + 1) / p.posRateHz;
      }
    }
    let q = truth.q;
    if (gyro > 0) {
      // Small attitude jitter consistent with the gyro noise level.
      const e = v3(r.normal(), r.normal(), r.normal());
      q = qMul(q, qFromAxisAngle(e, gyro * 0.01));
    }
    const vel = noisy(truth.vel, p.velNoise);
    const omega = noisy(truth.omega, gyro);
    const acc = noisy(truth.acc, p.accNoise);
    acc.y += p.accBias;
    const motors: Measurement['motors'] =
      p.motorFeedback && truth.motors ? [...truth.motors] : null;
    return { pos, vel, q, omega, acc, motors, posFresh };
  }

  reset(): void {
    this.history = [];
    this.head = 0;
    this.heldPos = null;
    this.nextPosT = 0;
  }
}
