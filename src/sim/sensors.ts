import { Rng } from '@/math/prng';
import { qConj, qFromAxisAngle, qMul, qRotate, type Quat } from '@/math/quat';
import { add, clone, v3, type Vec3 } from '@/math/vec3';
import { AttitudeFilter, type AttitudeGains } from '@/estimation/attitude';
import { LowPass2 } from '@/estimation/filters';
import type { DroneState } from './dynamics';
import {
  GRAVITY,
  type AhrsParams,
  type DroneParams,
  type SensorParams,
  type VibrationParams,
} from './params';
import { Vibration } from './vibration';

/** What the controller gets to see. An idealised state estimate plus optional imperfections. */
export interface Measurement {
  pos: Vec3;
  vel: Vec3;
  q: Quat;
  omega: Vec3;
  /** Accelerometer: specific force (acceleration minus gravity), body frame, m/s². */
  acc: Vec3;
  /**
   * Motor-speed telemetry per motor, as the thrust a healthy motor makes at that speed, N; null if
   * unavailable. It does not see a damaged prop — only the rotor speed.
   */
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
  motors: [...s.rotors],
  posFresh: true,
});

/** Turn the horizontal components of a body vector by `angle` about the body's up-axis (y). */
export const rotateAboutUp = (v: Vec3, angle: number): Vec3 => {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return v3(c * v.x + s * v.z, v.y, -s * v.x + c * v.z);
};

/** What the IMU's front end needs to know: its own settings, the vibration and the airframe. */
export interface ImuContext {
  sensors: SensorParams;
  vibration: VibrationParams;
  drone: DroneParams;
  /** Settings of the attitude filter, which runs on the IMU's output. */
  ahrs: AhrsParams;
  dt: number;
}

/** Gains of the selected attitude filter: the complementary filter is Mahony's without bias or gate. */
export const attitudeGains = (kind: SensorParams['attitude'], a: AhrsParams): AttitudeGains =>
  kind === 'mahony'
    ? { kp: a.kp, ki: a.ki, gate: a.gate }
    : { kp: 1 / Math.max(a.tau, 1e-3), ki: 0, gate: 0 };

export class Sensors {
  private vib = new Vibration();
  /** Anti-alias filters: gyro x, y, z and accelerometer x, y, z. */
  private aa = Array.from({ length: 6 }, () => new LowPass2(100, 0.001));
  /** The IMU's last sample and the physics steps since it was taken. */
  private imuHeld: { omega: Vec3; acc: Vec3 } | null = null;
  private imuAge = 0;
  /** The attitude filter of the flight computer; it runs while `sensors.attitude` asks for it. */
  readonly ahrs = new AttitudeFilter();
  private ahrsOn = false;
  private history: Measurement[] = [];
  private head = 0;
  private rng: Rng;
  /** Last position sample and when the next one is due (for posRateHz). */
  private heldPos: Vec3 | null = null;
  private nextPosT = 0;
  /** The last fix delivered, held through a dropout. */
  private lastPos: Vec3 | null = null;

  constructor(seed: number) {
    this.rng = new Rng(seed ^ 0x27d4eb2f);
  }

  /** Store the true state; call once per physics step. */
  record(s: DroneState, ctx?: ImuContext): void {
    const m = measurementOf(s);
    if (ctx) this.imuFrontEnd(m, s, ctx);
    if (this.history.length < HISTORY) this.history.push(m);
    else this.history[this.head] = m;
    this.head = (this.head + 1) % HISTORY;
  }

  /**
   * The IMU's front end, at the physics rate (docs/analysis.md §3.4): truth plus vibration,
   * through the anti-alias filter, then sample-and-hold. Does nothing while all three are off.
   */
  private imuFrontEnd(m: Measurement, s: DroneState, c: ImuContext): void {
    const shaking = c.vibration.gyroDeg > 0 || c.vibration.acc > 0;
    const { imuRateHz, aaFilterHz, gyroBias: b } = c.sensors;
    const biased = b.x !== 0 || b.y !== 0 || b.z !== 0;
    const estimating = c.sensors.attitude !== 'truth';
    if (!estimating) this.ahrsOn = false;
    if (!shaking && !biased && !estimating && imuRateHz <= 0 && aaFilterHz <= 0) return;
    let omega = m.omega;
    let acc = m.acc;
    if (biased) {
      const k = Math.PI / 180;
      omega = add(omega, v3(b.x * k, b.y * k, b.z * k));
    }
    if (shaking) {
      const v = this.vib.step(s.rotors, c.drone, c.vibration, c.dt);
      omega = add(omega, v.gyro);
      acc = add(acc, v.acc);
    }
    if (aaFilterHz > 0) {
      const f = (i: number, x: number) => {
        const lp = this.aa[i]!;
        lp.cutoffHz = aaFilterHz;
        lp.dt = c.dt;
        return lp.update(x);
      };
      omega = v3(f(0, omega.x), f(1, omega.y), f(2, omega.z));
      acc = v3(f(3, acc.x), f(4, acc.y), f(5, acc.z));
    }
    if (imuRateHz > 0) {
      const every = Math.max(1, Math.round(1 / (imuRateHz * c.dt)));
      if (!this.imuHeld || this.imuAge >= every) {
        this.imuHeld = { omega, acc };
        this.imuAge = 0;
      }
      this.imuAge++;
      omega = this.imuHeld.omega;
      acc = this.imuHeld.acc;
    }
    m.omega = clone(omega);
    m.acc = clone(acc);
    if (estimating) {
      // Start from the truth (a drone is levelled before take-off), then run on the IMU alone,
      // with an ideal compass for the heading.
      if (!this.ahrsOn) this.ahrs.reset(s.q);
      this.ahrsOn = true;
      const north = qRotate(qConj(s.q), v3(1, 0, 0));
      const g = attitudeGains(c.sensors.attitude, c.ahrs);
      m.q = { ...this.ahrs.update(omega, acc, north, g, GRAVITY, c.dt) };
    }
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
    if (p.posDropout && this.lastPos) {
      pos = clone(this.lastPos);
      posFresh = false;
    } else if (p.posRateHz > 0 && this.heldPos && t < this.nextPosT - 1e-9) {
      pos = clone(this.heldPos); // sample-and-hold between position fixes
      posFresh = false;
    } else {
      if (p.posStuck && this.lastPos) {
        pos = clone(this.lastPos); // a new fix, at the usual rate, that repeats the old one
      } else {
        pos = noisy(truth.pos, p.posNoise);
        pos.y += p.altBias;
      }
      if (p.posRateHz > 0) {
        this.heldPos = clone(pos);
        this.nextPosT = (Math.floor(t * p.posRateHz + 1e-6) + 1) / p.posRateHz;
      }
    }
    if (posFresh) this.lastPos = clone(pos);
    let q = truth.q;
    if (gyro > 0) {
      // Small attitude jitter consistent with the gyro noise level.
      const e = v3(r.normal(), r.normal(), r.normal());
      q = qMul(q, qFromAxisAngle(e, gyro * 0.01));
    }
    const vel = noisy(truth.vel, p.velNoise);
    let omega = noisy(truth.omega, gyro);
    if (p.imuYawDeg !== 0) omega = rotateAboutUp(omega, (p.imuYawDeg * Math.PI) / 180);
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
    this.lastPos = null;
    this.vib.reset();
    for (const f of this.aa) f.reset();
    this.imuHeld = null;
    this.imuAge = 0;
    this.ahrs.reset();
    this.ahrsOn = false;
  }
}
