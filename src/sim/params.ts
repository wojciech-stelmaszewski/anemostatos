import { defaultGains, type PidGains } from '@/control/pid';

export type Level = 1 | 2 | 3;

/** Controller families selectable per level (docs/beyond-pid.md §3.2). Part II adds more. */
export type L1Kind = 'pid' | 'lqr' | 'adrc' | 'mpc' | 'l1ac';
/** State estimator in front of the L1 controller. */
export type L1Estimator = 'none' | 'kalman';
export type L3Outer = 'pid-cascade' | 'geometric' | 'mpc' | 'mppi' | 'policy';
/** How an attitude error becomes a body-rate setpoint. */
export type AttitudeLaw = 'quaternion' | 'tilt' | 'euler';
export type L3Inner = 'pid' | 'indi';
export type L3Compensation = 'none' | 'indi' | 'l1ac' | 'learned';
export type L3Safety = 'none' | 'cbf';

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
  l1: { kind: L1Kind; estimator: L1Estimator };
  /** L1 LQR / LQI (docs/beyond-pid.md §4). Cost weights of J = Σ (q·x² + r·u²)·dt. */
  lqr: {
    qPos: number;
    qVel: number;
    qInt: number;
    r: number;
    /** Augment with ∫e (LQI). */
    integral: boolean;
    /** Model the motor lag as a third state. */
    lagState: boolean;
  };
  /** L3 INDI (rate stage and acceleration compensation). */
  indi: {
    /** α_des = K·(ω_sp − ω), 1/s. */
    rateKpRP: number;
    rateKpYaw: number;
    /** Cutoff of the gyro-derivative / motor-torque filter pair, Hz. */
    filterHz: number;
    /** Cutoff of the accelerometer / thrust-vector filter pair, Hz. */
    accFilterHz: number;
    /** Pass the actuator signal through the same filter as the measurement (as INDI requires). */
    syncFilters: boolean;
  };
  /** L3 geometric tracking controller (Lee et al. 2010), in acceleration units. */
  geometric: {
    /** a = a_ref − Kp·e_p − Kv·e_v − Ki·∫e_p; 1/s². */
    kp: number;
    /** 1/s. */
    kv: number;
    /** 1/s³; 0 = no integral. */
    ki: number;
    /** Use the reference's acceleration, and body rates from its jerk (differential flatness). */
    feedforward: boolean;
  };
  /** Linear MPC (L1 altitude, L3 outer stage). */
  mpc: {
    /** Re-planning rate, Hz; also the prediction step (1/hz). */
    hz: number;
    /** Prediction horizon, s. */
    horizon: number;
    qPos: number;
    qVel: number;
    r: number;
    /** ADMM iteration cap per solve. */
    iterations: number;
    /** Distance kept from the ceiling, m. */
    margin: number;
  };
  /** MPPI sampling controller (L3 outer stage). */
  mppi: {
    hz: number;
    samples: number;
    /** Horizon, s, split into `steps`. */
    horizon: number;
    steps: number;
    /** Exploration noise σ, m/s². */
    sigma: number;
    /** Temperature λ: low = follow the best sample, high = average many. */
    lambda: number;
    qPos: number;
    zoneWeight: number;
    /** Distance kept from keep-out zones, m. */
    margin: number;
  };
  /** Control-barrier-function safety filter (L3). */
  cbf: {
    /** How early it starts braking (1/s); h-dynamics poles at −γ. */
    gamma: number;
    margin: number;
  };
  /** L1 adaptive control. */
  l1ac: {
    /** State-predictor pole magnitude, 1/s. */
    as: number;
    /** Bandwidth of the low-pass C(s) on the estimate, Hz. */
    filterHz: number;
    /** Baseline controller bandwidth (L1 altitude), rad/s. */
    wc: number;
  };
  /** Learned-basis residual compensation (L3). */
  residual: {
    /** RLS forgetting factor per update (100 Hz): closer to 1 = longer memory. */
    forgetting: number;
    /** Accelerometer / thrust filter, Hz. */
    filterHz: number;
  };
  /** L1 ADRC: controller and observer bandwidths, rad/s. */
  adrc: { wc: number; wo: number };
  /** L1 Kalman filter: what it assumes about the noise. */
  kalman: {
    /** Accelerometer noise σ (process noise), m/s². */
    accSigma: number;
    /** Position sensor noise σ (measurement noise), m. */
    posSigma: number;
    /** Accelerometer bias random walk, m/s² per √s. */
    biasSigma: number;
  };
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
    attitude: AttitudeLaw;
    inner: L3Inner;
    compensation: L3Compensation;
    safety: L3Safety;
  };
}

/** Virtual keep-out zones (no collisions: violations are counted and shown). */
export interface WorldParams {
  /** Ceiling height, m; 0 = none. */
  ceiling: number;
  /** A vertical pillar (infinitely tall cylinder), L2/L3. */
  pillar: boolean;
  pillarX: number;
  pillarZ: number;
  pillarR: number;
}

export interface Params {
  sim: { level: Level; seed: number };
  world: WorldParams;
  setpoint: {
    x: number;
    y: number;
    z: number;
    yawDeg: number;
    rateLimit: number;
    /** Automatic setpoint motion added on top of the base value. */
    profile: 'none' | 'square' | 'sine' | 'triangle' | 'circle' | 'figure8' | 'corners' | 'minsnap';
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
  world: { ceiling: 0, pillar: false, pillarX: 3, pillarZ: 0, pillarR: 0.6 },
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
    l1: { kind: 'pid', estimator: 'none' },
    lqr: { qPos: 100, qVel: 10, qInt: 5, r: 1, integral: false, lagState: false },
    adrc: { wc: 3, wo: 15 },
    l1ac: { as: 20, filterHz: 5, wc: 3 },
    residual: { forgetting: 0.995, filterHz: 10 },
    mpc: { hz: 50, horizon: 1, qPos: 20, qVel: 4, r: 0.1, iterations: 60, margin: 0.03 },
    mppi: {
      hz: 50,
      samples: 256,
      horizon: 1.5,
      steps: 30,
      sigma: 4,
      lambda: 1,
      qPos: 20,
      zoneWeight: 3000,
      margin: 0.35,
    },
    cbf: { gamma: 2.5, margin: 0.3 },
    geometric: { kp: 9, kv: 5, ki: 0.5, feedforward: true },
    indi: { rateKpRP: 20, rateKpYaw: 8, filterHz: 20, accFilterHz: 10, syncFilters: true },
    kalman: { accSigma: 0.5, posSigma: 0.05, biasSigma: 0.05 },
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
      attitude: 'quaternion',
      inner: 'pid',
      compensation: 'none',
      safety: 'none',
    },
  },
});

export const GRAVITY = 9.81;
