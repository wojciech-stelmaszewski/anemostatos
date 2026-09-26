import { defaultGains, type PidGains } from '@/control/pid';

export type Level = 1 | 2 | 3;

/** Controller families selectable per level (docs/beyond-pid.md §3.2). Part II adds more. */
export type L1Kind = 'pid';
export type L3Outer = 'pid-cascade';
export type L3Inner = 'pid';
export type L3Compensation = 'none';
export type L3Safety = 'none';

export interface DroneParams {
  mass: number;
  maxMotorThrust: number;
  motorTau: number;
  dragH: number;
  dragV: number;
  inertia: { x: number; y: number; z: number };
  torqueCoeff: number;
  angularDamping: number;
  crashSpeed: number;
  /** Fraction of the commanded thrust each motor actually delivers (faults, worn props). */
  motorEfficiency: [number, number, number, number];
  /** Rotor drag: horizontal body-plane force −k·T·v_rel, s/m (L3). 0 = off. */
  rotorDrag: number;
}

export interface WindParams {
  enabled: boolean;
  meanSpeed: number;
  meanHeading: number; // degrees, 0 = +x, 90 = +z
  meanVertical: number;
  turbSigma: number;
  turbTau: number;
  gustsOn: boolean;
  gustsPerMinute: number;
  gustAmpMin: number;
  gustAmpMax: number;
  gustDurMin: number;
  gustDurMax: number;
  /** Vertical component of the gust direction unit vector, 0 = horizontal, 1 = straight up/down. */
  gustVertical: number;
}

export interface SensorParams {
  posNoise: number;
  velNoise: number;
  gyroNoise: number; // deg/s
  altBias: number;
  delayMs: number;
  /** Accelerometer (specific force, body frame) noise σ, m/s². */
  accNoise: number;
  /** Accelerometer bias along the body up-axis, m/s². */
  accBias: number;
  /** Position sensor sample rate, Hz; readings are held in between. 0 = fresh on every read. */
  posRateHz: number;
  /** Motor-speed telemetry (per-motor thrust) available to the controller. */
  motorFeedback: boolean;
}

/** What the controller believes about the vehicle. The truth lives in `DroneParams`. */
export interface ModelParams {
  /** Assumed mass m̂, kg. */
  mass: number;
  /** Assumed inertia as a multiple of the true one. */
  inertiaScale: number;
  /** Assumed motor time constant, s. */
  motorTau: number;
}

export interface ControlParams {
  /** Rate of the L1/L2 controller. */
  rateHz: number;
  feedforward: boolean;
  model: ModelParams;
  l1: { kind: L1Kind };
  /** Vertical position loop, L1 and L2. */
  alt: PidGains;
  /** Horizontal position loops, L2. */
  posH: PidGains;
  maxHForce: number;
  /** L3 cascade. */
  l3: {
    posKpH: number;
    posKpV: number;
    vMaxH: number;
    vMaxV: number;
    velH: PidGains;
    velV: PidGains;
    maxTiltDeg: number;
    attKpRP: number;
    attKpYaw: number;
    maxRateDeg: number;
    rateRP: PidGains;
    rateYaw: PidGains;
    hzRate: number;
    hzAtt: number;
    hzVel: number;
    hzPos: number;
    outer: L3Outer;
    inner: L3Inner;
    compensation: L3Compensation;
    safety: L3Safety;
  };
}

export interface Params {
  sim: { level: Level; seed: number };
  setpoint: {
    x: number;
    y: number;
    z: number;
    yawDeg: number;
    rateLimit: number;
    /** Automatic setpoint motion added on top of the base value. */
    profile: 'none' | 'square' | 'sine' | 'triangle';
    profileAxis: 'y' | 'x';
    profileAmplitude: number;
    profilePeriod: number;
  };
  drone: DroneParams;
  wind: WindParams;
  sensors: SensorParams;
  control: ControlParams;
}

export const defaultParams = (): Params => ({
  sim: { level: 1, seed: 1337 },
  setpoint: {
    x: 0,
    y: 2,
    z: 0,
    yawDeg: 0,
    rateLimit: 0,
    profile: 'none',
    profileAxis: 'y',
    profileAmplitude: 0.5,
    profilePeriod: 8,
  },
  drone: {
    mass: 1.0,
    maxMotorThrust: 6.1,
    motorTau: 0.03,
    dragH: 0.12,
    dragV: 0.25,
    inertia: { x: 0.0082, y: 0.0149, z: 0.0082 },
    torqueCoeff: 0.016,
    angularDamping: 0.002,
    crashSpeed: 3,
    motorEfficiency: [1, 1, 1, 1],
    rotorDrag: 0,
  },
  wind: {
    enabled: true,
    meanSpeed: 0,
    meanHeading: 0,
    meanVertical: 0,
    turbSigma: 0.8,
    turbTau: 1.5,
    gustsOn: true,
    gustsPerMinute: 8,
    gustAmpMin: 3,
    gustAmpMax: 9,
    gustDurMin: 0.6,
    gustDurMax: 2.5,
    gustVertical: 0.7,
  },
  sensors: {
    posNoise: 0,
    velNoise: 0,
    gyroNoise: 0,
    altBias: 0,
    delayMs: 0,
    accNoise: 0,
    accBias: 0,
    posRateHz: 0,
    motorFeedback: true,
  },
  control: {
    rateHz: 250,
    feedforward: true,
    model: { mass: 1.0, inertiaScale: 1, motorTau: 0.03 },
    l1: { kind: 'pid' },
    alt: defaultGains({ kp: 10, ki: 0.8, kd: 7, iLimit: 10 }),
    posH: defaultGains({ kp: 4, ki: 0.8, kd: 3, iLimit: 3 }),
    maxHForce: 5,
    l3: {
      posKpH: 1.2,
      posKpV: 1.5,
      vMaxH: 4,
      vMaxV: 2,
      velH: defaultGains({ kp: 2.2, ki: 0.6, kd: 0.1, dFilterHz: 10, iLimit: 4 }),
      velV: defaultGains({ kp: 4, ki: 1.5, kd: 0.1, dFilterHz: 10, iLimit: 5 }),
      maxTiltDeg: 35,
      attKpRP: 8,
      attKpYaw: 3,
      maxRateDeg: 360,
      rateRP: defaultGains({ kp: 18, ki: 6, kd: 0.25, dFilterHz: 60, iLimit: 20 }),
      rateYaw: defaultGains({ kp: 8, ki: 2, kd: 0, dFilterHz: 60, iLimit: 10 }),
      hzRate: 1000,
      hzAtt: 250,
      hzVel: 100,
      hzPos: 50,
      outer: 'pid-cascade',
      inner: 'pid',
      compensation: 'none',
      safety: 'none',
    },
  },
});

export const GRAVITY = 9.81;
