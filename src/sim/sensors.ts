import { Rng } from '@/math/prng';
import { qConj, qFromAxisAngle, qMul, qRotate, type Quat } from '@/math/quat';
import { add, clone, v3, type Vec3 } from '@/math/vec3';
import { AttitudeFilter, type AttitudeGains } from '@/estimation/attitude';
import { LowPass2 } from '@/estimation/filters';
import { Mekf } from '@/estimation/mekf';
import { NavEkf } from '@/estimation/navekf';
import type { DroneState } from './dynamics';
import {
  GRAVITY,
  type AhrsParams,
  type DroneParams,
  type MekfParams,
  type NavEkfParams,
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
  /** Settings of the attitude filters and the navigation filter, which run on the IMU's output. */
  ahrs: AhrsParams;
  mekf: MekfParams;
  navEkf: NavEkfParams;
  dt: number;
}

const DEG = Math.PI / 180;
/** Inclination of the earth's field: it points 65° below the horizon (central Europe). */
const DIP = 65 * DEG;
const UP = v3(0, 1, 0);
const NORTH = v3(1, 0, 0);

/** Where the sensors stand in the estimation chain, for the charts. */
export interface EstimationState {
  /** NIS of the last magnetometer and accelerometer update of the MEKF, and whether it was used. */
  magNis: number;
  magUsed: boolean;
  /** NIS of the last GPS fix of the navigation filter, and whether it was used. */
  gpsNis: number;
  gpsUsed: boolean;
}

/** Gains of the selected attitude filter: the complementary filter is Mahony's without bias or gate. */
export const attitudeGains = (kind: SensorParams['attitude'], a: AhrsParams): AttitudeGains =>
  kind === 'mahony'
    ? { kp: a.kp, ki: a.ki, gate: a.gate }
    : { kp: 1 / Math.max(a.tau, 1e-3), ki: 0, gate: 0 };

/** The part of `v` perpendicular to `up`, as a unit vector. */
function horizontal(v: Vec3, up: Vec3): Vec3 {
  const k = v.x * up.x + v.y * up.y + v.z * up.z;
  const h = v3(v.x - k * up.x, v.y - k * up.y, v.z - k * up.z);
  const l = Math.hypot(h.x, h.y, h.z) || 1;
  return v3(h.x / l, h.y / l, h.z / l);
}

/** Step of a position sensor that reads 10 m in this many bits, m. */
export const posQuantStep = (bits: number): number => 10 / 2 ** bits;

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
  readonly mekf = new Mekf();
  readonly nav = new NavEkf();
  private navOn = false;
  private steps = 0;
  private nextGps = 0;
  readonly est: EstimationState = { magNis: 0, magUsed: true, gpsNis: 0, gpsUsed: true };
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
    const navigating = c.sensors.nav === 'ekf';
    if (!estimating) this.ahrsOn = false;
    if (!navigating) this.navOn = false;
    this.steps++;
    if (!shaking && !biased && !estimating && !navigating && imuRateHz <= 0 && aaFilterHz <= 0)
      return;
    let omega = m.omega;
    let acc = m.acc;
    if (estimating || navigating) {
      // The filters see the sensors' noise; the controller then reads the same noisy samples.
      const r = this.rng;
      const g = c.sensors.gyroNoise * DEG;
      const a = c.sensors.accNoise;
      if (g > 0) omega = add(omega, v3(g * r.normal(), g * r.normal(), g * r.normal()));
      if (a > 0) acc = add(acc, v3(a * r.normal(), a * r.normal(), a * r.normal()));
    }
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
      // with an ideal compass for the heading unless the magnetometer is on.
      const kind = c.sensors.attitude;
      if (!this.ahrsOn) {
        this.ahrs.reset(s.q);
        this.mekf.reset(s.q);
      }
      this.ahrsOn = true;
      const mag = c.sensors.magnetometer ? this.magnetometer(s, c) : null;
      const qEst = kind === 'mekf' ? this.mekf.q : this.ahrs.q;
      // North as the filter sees it: the horizontal part of the field, in the body frame.
      const north = mag ? horizontal(mag, qRotate(qConj(qEst), UP)) : qRotate(qConj(s.q), NORTH);
      if (kind === 'mekf') {
        const k = c.mekf;
        const set = {
          gyroSigma: k.gyroDeg * DEG,
          biasWalk: k.biasWalkDeg * DEG,
          accSigma: k.accDeg * DEG,
          magSigma: k.magDeg * DEG,
          gate: k.gate,
        };
        this.mekf.propagate(omega, set, c.dt);
        // The vector sensors are read 100 times a second.
        if (this.steps % 10 === 0) {
          const an = Math.hypot(acc.x, acc.y, acc.z);
          if (an > 1e-6)
            this.mekf.lastAcc = this.mekf.updateVector(
              v3(acc.x / an, acc.y / an, acc.z / an),
              UP,
              set.accSigma,
              set.gate,
            );
          // Heading only, one degree of freedom: its gate is the 3-dof quantile less about 5.
          const up = qRotate(qConj(this.mekf.q), UP);
          const g1 = set.gate > 0 ? Math.max(set.gate - 5, 1) : 0;
          const u = this.mekf.updateHeading(north, up, set.magSigma, g1);
          this.mekf.lastMag = u;
          this.est.magNis = u.nis;
          this.est.magUsed = u.accepted;
        }
        m.q = { ...this.mekf.q };
      } else {
        const g = attitudeGains(kind, c.ahrs);
        m.q = { ...this.ahrs.update(omega, acc, north, g, GRAVITY, c.dt) };
      }
    }
    if (navigating) {
      const sp = c.sensors;
      const set = {
        accSigma: c.navEkf.accSigma,
        biasWalk: c.navEkf.biasWalk,
        gpsSigma: Math.max(sp.gpsNoise, 1e-3),
        baroSigma: Math.max(sp.baroNoise, 1e-3),
        gate: c.navEkf.gate,
      };
      if (!this.navOn) {
        this.nav.reset(s.pos);
        this.nextGps = 0;
      }
      this.navOn = true;
      this.nav.propagate(acc, m.q, GRAVITY, set, c.dt);
      const t = this.steps * c.dt;
      const r = this.rng;
      if (t >= this.nextGps) {
        this.nextGps = t + 1 / Math.max(sp.gpsRateHz, 0.1);
        const n = sp.gpsNoise;
        const fix = v3(
          s.pos.x + n * r.normal() + sp.gpsGlitch,
          s.pos.y + n * r.normal(),
          s.pos.z + n * r.normal(),
        );
        const u = this.nav.gps(fix, set);
        this.est.gpsNis = u.nis;
        this.est.gpsUsed = u.accepted;
      }
      if (this.steps % 20 === 0) this.nav.baro(s.pos.y + sp.baroNoise * r.normal(), set);
      m.pos = this.nav.pos;
      m.vel = this.nav.vel;
    }
  }

  /** The magnetometer: the field, turned by the local disturbance, seen from the body, unit. */
  private magnetometer(s: DroneState, c: ImuContext): Vec3 {
    const d = c.sensors.magDisturbDeg * DEG;
    const h = Math.cos(DIP);
    const field = v3(h * Math.cos(d), -Math.sin(DIP), -h * Math.sin(d));
    const b = qRotate(qConj(s.q), field);
    const n = c.sensors.magNoiseDeg * DEG;
    const r = this.rng;
    const v = v3(b.x + n * r.normal(), b.y + n * r.normal(), b.z + n * r.normal());
    const l = Math.hypot(v.x, v.y, v.z);
    return v3(v.x / l, v.y / l, v.z / l);
  }

  /** Delayed, biased, noisy reading of the recorded state at simulation time t. */
  read(p: SensorParams, physDt: number, t = 0): Measurement {
    const jitter =
      p.jitterMs > 0
        ? Math.floor(this.rng.next() * (Math.round(p.jitterMs / 1000 / physDt) + 1))
        : 0;
    const lag = Math.min(this.history.length - 1, Math.round(p.delayMs / 1000 / physDt) + jitter);
    const truth = this.history[(this.head - 1 - lag + HISTORY) % HISTORY]!;
    const r = this.rng;
    const noisy = (v: Vec3, s: number): Vec3 =>
      s > 0 ? v3(v.x + s * r.normal(), v.y + s * r.normal(), v.z + s * r.normal()) : clone(v);
    const gyro = (p.gyroNoise * Math.PI) / 180;
    let pos: Vec3;
    let posFresh = true;
    const filtered = p.attitude !== 'truth' || p.nav === 'ekf';
    if (p.nav === 'ekf') {
      pos = clone(truth.pos); // the navigation filter's estimate
    } else if (p.posDropout && this.lastPos) {
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
        if (p.posQuantBits > 0) {
          const q = posQuantStep(p.posQuantBits);
          pos = v3(Math.round(pos.x / q) * q, Math.round(pos.y / q) * q, Math.round(pos.z / q) * q);
        }
      }
      if (p.posRateHz > 0) {
        this.heldPos = clone(pos);
        this.nextPosT = (Math.floor(t * p.posRateHz + 1e-6) + 1) / p.posRateHz;
      }
    }
    if (posFresh) this.lastPos = clone(pos);
    let q = truth.q;
    if (gyro > 0 && p.attitude === 'truth') {
      // Small attitude jitter consistent with the gyro noise level.
      const e = v3(r.normal(), r.normal(), r.normal());
      q = qMul(q, qFromAxisAngle(e, gyro * 0.01));
    }
    const vel = p.nav === 'ekf' ? clone(truth.vel) : noisy(truth.vel, p.velNoise);
    // With a filter in the chain the IMU's noise is already in the recorded samples.
    let omega = filtered ? clone(truth.omega) : noisy(truth.omega, gyro);
    if (p.imuYawDeg !== 0) omega = rotateAboutUp(omega, (p.imuYawDeg * Math.PI) / 180);
    const acc = filtered ? clone(truth.acc) : noisy(truth.acc, p.accNoise);
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
    this.mekf.reset();
    this.nav.reset();
    this.navOn = false;
    this.steps = 0;
    this.nextGps = 0;
    Object.assign(this.est, { magNis: 0, magUsed: true, gpsNis: 0, gpsUsed: true });
  }
}
