import type { TvcControlParams } from '@/control/tvc';
import type { RocketParams } from './vehicles/rocket';
import type { TvcParams } from './vehicles/tvc';
import type { SatelliteParams } from './vehicles/satellite';
import type { LanderParams } from './vehicles/lander';
import type { PdgSettings } from '@/guidance/pdg';
import type { VehicleId } from './vehicles/types';
import type { AircraftParams } from './vehicles/aircraft';
import type { AutopilotParams } from '@/control/autopilot';
import { defaultGains, type PidGains } from '@/control/pid';

export type Level = 1 | 2 | 3;

/** Controller families selectable per level (docs/beyond-pid.md §3.2). Part II adds more. */
export type L1Kind =
  | 'pid'
  | 'lqr'
  | 'adrc'
  | 'mpc'
  | 'l1ac'
  | 'hinf'
  | 'smc'
  | 'mrac'
  | 'backstepping'
  | 'bangbang'
  | 'dp';
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
  /**
   * Rotational drag about the up-axis, N·m·s/rad. The drag of spinning rotors makes it larger than
   * about the other axes on a real drone; it sets how fast a drone that has lost a rotor spins.
   */
  yawDamping: number;
  crashSpeed: number;
  /** Rate limit of each motor's thrust, N/s: how fast a rotor can spin up or down. 0 = none. */
  motorSlew: number;
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
  /** The position sensor delivers no new fixes (the last one is held). */
  posDropout: boolean;
  /** The position sensor is stuck: it repeats its last fix and says it is fresh. */
  posStuck: boolean;
  /**
   * Resolution of the position sensor: its range of 10 m is read in this many bits, so a reading
   * moves in steps of 10 m / 2^bits. 0 = exact.
   */
  posQuantBits: number;
  /** Timing jitter: each sample is taken late by a random 0 … this many ms. 0 = none. */
  jitterMs: number;
  /** The gyro is mounted rotated about the body's up-axis by this angle, degrees (L3). */
  imuYawDeg: number;
  /** Sample rate of the gyro and accelerometer, Hz; readings are held in between. 0 = every step. */
  imuRateHz: number;
  /** Analog anti-alias filter in front of the IMU's sampler (two poles), Hz. 0 = none. */
  aaFilterHz: number;
  /** Constant gyro bias, deg/s, body frame. */
  gyroBias: { x: number; y: number; z: number };
  /**
   * Where the attitude the controller flies on comes from (L3): the true one, or an estimate
   * from the gyro and the accelerometer (docs/analysis.md §3.4).
   */
  attitude: 'truth' | 'complementary' | 'mahony' | 'mekf';
  /** A magnetometer gives the heading instead of an ideal compass (L3, attitude filters). */
  magnetometer: boolean;
  /** Direction noise of the magnetometer, degrees. */
  magNoiseDeg: number;
  /** Local disturbance: the field the magnetometer sees is turned about the vertical, degrees. */
  magDisturbDeg: number;
  /** Where position and velocity come from (L3): the truth, or a navigation EKF on GPS and baro. */
  nav: 'truth' | 'ekf';
  /** GPS-like receiver: fixes per second and position noise σ, m. */
  gpsRateHz: number;
  gpsNoise: number;
  /** Barometric altitude noise σ, m (50 readings a second). */
  baroNoise: number;
  /** A glitch: every fix is off by this much along x while it is not 0, m. */
  gpsGlitch: number;
  /**
   * An inertial navigator runs alongside from `insStart` (s): it integrates the gyro and the
   * accelerometer with no position fix (lesson IV.26). The controller does not fly on it.
   */
  ins: boolean;
  insStart: number;
  /** Range-only beacons: a filter estimates the horizontal position from them (lesson IV.27). */
  beacons: boolean;
  /** Range noise σ, m, and readings per second. */
  beaconNoise: number;
  beaconRateHz: number;
}

/** Flight after the loss of a rotor (src/control/fault.ts, lesson III.27). */
export interface FaultParams {
  /** Switch to the three-motor law when a motor is lost (L3, PID cascade). */
  enabled: boolean;
  /** Position PID of the three-motor law: 1/s², 1/s, 1/s³. */
  posKp: number;
  posKd: number;
  posKi: number;
  /** Reduced attitude, 1/s, and body-rate, 1/s, gains. */
  attKp: number;
  rateKp: number;
  maxTiltDeg: number;
  /** Command the torque ahead of the motor lag in the spinning body. */
  leadLag: boolean;
  /** Who says which motor is lost: the truth (an oracle) or the fault monitor's CUSUM. */
  source: 'oracle' | 'monitor';
}

/** The fault monitor (src/estimation/fdi.ts, lesson III.28). */
export interface FdiParams {
  /** NIS of the altimeter summed over this many fixes, and its alarm level. */
  nisWindow: number;
  nisThreshold: number;
  /** Alarm also when the NIS sum is too small: a sensor that has stopped being noisy. */
  nisLow: number;
  /** CUSUM of the motor torque residual: tolerated deviation, N·m, and alarm level, N·m·s. */
  cusumDrift: number;
  cusumThreshold: number;
}

/** The multiplicative EKF (src/estimation/mekf.ts): what it assumes, in degrees. */
export interface MekfParams {
  /** Gyro noise, °/s, and how fast its bias may wander, °/s per √s. */
  gyroDeg: number;
  biasWalkDeg: number;
  /** Direction noise of gravity from the accelerometer and of north from the magnetometer, °. */
  accDeg: number;
  magDeg: number;
  /** χ² gate on each vector measurement (3 degrees of freedom); 0 = accept everything. */
  gate: number;
}

/** The navigation EKF (src/estimation/navekf.ts). Receiver and baro noise come from the sensors. */
export interface NavEkfParams {
  /** Accelerometer noise the filter assumes, m/s², and bias wander, m/s² per √s. */
  accSigma: number;
  biasWalk: number;
  /** χ² gate on a GPS fix (3 degrees of freedom); 0 = accept every fix. */
  gate: number;
}

/** Settings of the attitude filters (src/estimation/attitude.ts). */
export interface AhrsParams {
  /** Complementary filter: time constant of the blend between gyro and accelerometer, s. */
  tau: number;
  /** Mahony: proportional gain, rad/s, and bias adaptation gain, 1/s². */
  kp: number;
  ki: number;
  /** Mahony: ignore the accelerometer while its magnitude is off 1 g by more than this fraction. */
  gate: number;
}

/** Rotor vibration on the IMU (docs/analysis.md §3.4). Off while both amplitudes are zero. */
export interface VibrationParams {
  /** Gyro amplitude per rotor at hover thrust, deg/s. */
  gyroDeg: number;
  /** Accelerometer amplitude per rotor at hover thrust, m/s². */
  acc: number;
  /** Rotor frequency at hover, Hz. It follows the thrust: f = hoverHz·√(T/T_hover). */
  hoverHz: number;
}

/** Filters on the gyro before the rate loop (L3). */
export interface GyroFilterParams {
  /** Second-order low-pass, Hz. 0 = off. */
  lpfHz: number;
  /** A notch at a fixed frequency, or one per motor that follows its rotor speed. */
  notch: 'off' | 'fixed' | 'rpm';
  notchHz: number;
  /** Centre frequency over the width of the notch. */
  notchQ: number;
}

/**
 * A test signal injected into the loop to measure its frequency response (docs/analysis.md §3.3).
 * `l1.thrust` adds it to the thrust command (the loop broken at the plant input);
 * `ref.y` adds it to the altitude setpoint; `l3.torque.x` and `.z` add it to the roll and pitch
 * torque of the quadrotor.
 */
export type ProbePoint = 'none' | 'ref.y' | 'l1.thrust' | 'l3.torque.x' | 'l3.torque.z';
export interface ProbeParams {
  point: ProbePoint;
  signal: 'sine' | 'chirp';
  /** Amplitude in the units of the point: N for thrust, m for the setpoint, N·m for a torque. */
  amp: number;
  /** Frequency of the sine, Hz. */
  freqHz: number;
  /** The chirp sweeps logarithmically from f0 to f1 over durationS. */
  f0Hz: number;
  f1Hz: number;
  durationS: number;
}

/**
 * How far the real vehicle may differ from the nominal one (docs/analysis.md §4, Chapter H).
 * Together the three ranges define a family of plants; robust analysis asks about all of them.
 */
export interface UncertaintyParams {
  /** The mass may be off by this many per cent either way. */
  massPct: number;
  /** The motor time constant may be this many times shorter or longer. */
  tauFactor: number;
  /** Up to this much extra sensor delay, ms. */
  delayMs: number;
}

/** The filter on range-only beacons (src/estimation/ukf.ts, lesson IV.27). */
export interface BeaconFilterParams {
  kind: 'ekf' | 'ukf';
  /** Where the filter starts, relative to the truth, m, and how unsure it says it is, m. */
  guessX: number;
  guessZ: number;
  sigma0: number;
  /** When it starts, s. */
  start: number;
}

/** An interception (src/guidance/pronav.ts, lesson IV.24); the drone flies the geometric outer. */
export interface GuidanceParams {
  enabled: boolean;
  law: 'pursuit' | 'pn';
  /** Navigation constant of proportional navigation. */
  N: number;
  /** The pursuer's speed, m/s, held along the line of sight, and the gain that holds it, 1/s. */
  speed: number;
  pursuitGain: number;
  /** When the chase starts, s. */
  start: number;
  /** The target: start (x, z), heading, speed, and a sideways weave (m/s², s). */
  targetX: number;
  targetZ: number;
  targetHeadingDeg: number;
  targetSpeed: number;
  weaveAcc: number;
  weavePeriod: number;
}

/** Identification from a chirp (src/estimation/sysid.ts, lesson III.26). */
export interface SysIdParams {
  /**
   * `direct`: the plant from the thrust to the altitude, P_uy/P_uu. `viaProbe`: both against the
   * injected test signal, P_dy/P_du, which the loop's reaction to the wind does not bias.
   */
  estimator: 'direct' | 'viaProbe';
  /** Fit only the frequencies whose coherence is at least this (0…1). */
  minCoherence: number;
  /** Chirp amplitude on the thrust, N, and its duration, s (0.2 → 8 Hz). */
  amp: number;
  seconds: number;
}

/** What the controller believes about the vehicle. The truth lives in `DroneParams`. */
export interface ModelParams {
  /** Assumed mass m̂, kg. */
  mass: number;
  /** Assumed inertia as a multiple of the true one. */
  inertiaScale: number;
  /** Assumed motor time constant, s. */
  motorTau: number;
  /** Assumed mounting angle of the gyro about the up-axis, degrees; its readings are turned back by it. */
  imuYawDeg: number;
}

export interface ControlParams {
  /** Rate of the L1/L2 controller. */
  rateHz: number;
  feedforward: boolean;
  model: ModelParams;
  gyroFilter: GyroFilterParams;
  ahrs: AhrsParams;
  mekf: MekfParams;
  navEkf: NavEkfParams;
  beaconFilter: BeaconFilterParams;
  fault: FaultParams;
  fdi: FdiParams;
  /** The landing law of the rocket (src/control/rocket.ts). */
  lander: { ignitionAlt: number; touchdownSpeed: number; speedGain: number };
  /** The thrust-vector controller of the planar rocket (src/control/tvc.ts). */
  tvc: TvcControlParams;
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
    /**
     * Estimate the state from the altimeter alone with a steady-state Kalman filter (LQG),
     * instead of reading velocity and thrust from the sensors.
     */
    observer: boolean;
    /** The filter's assumed process noise at the thrust input, N, and altimeter noise, m. */
    kfQ: number;
    kfR: number;
  };
  /**
   * Loop-shaping stages after the L1 PID (src/analysis/shaping.ts): a lead from leadZHz to leadPHz,
   * a lag from lagZHz down to lagPHz, a notch at notchHz. A stage is off while its first number is 0.
   */
  shaping: {
    leadZHz: number;
    leadPHz: number;
    lagZHz: number;
    lagPHz: number;
    notchHz: number;
    notchQ: number;
  };
  /**
   * H∞ loop shaping (src/control/hinf.ts): the weight W(s) = k·(s + ωᵢ)/s · (s/ω_z + 1)/(s/ω_p + 1)
   * (the lead is off while ω_z is 0) and γ/γ_min.
   */
  hinf: { k: number; wi: number; wz: number; wp: number; gammaFactor: number };
  /**
   * Sliding mode (src/control/smc.ts): slope λ of the line s = ė + λe (1/s), switching force k
   * (N) and the boundary layer φ (m/s; 0 = pure sign).
   */
  smc: { lambda: number; k: number; phi: number };
  /**
   * MRAC (src/control/mrac.ts): the reference model ÿ = kp·(r − y) − kd·ẏ; γ (with a weight per
   * gain) adapts the two feedback gains, γ_g the hover thrust; σ of the σ-modification (0 = none).
   */
  mrac: {
    kp: number;
    kd: number;
    gamma: number;
    gammaP: number;
    gammaD: number;
    gammaG: number;
    sigma: number;
  };
  /** Backstepping through the motor lag (src/control/backstepping.ts): one gain per step, 1/s. */
  backstepping: { k1: number; k2: number; k3: number };
  /**
   * Time-optimal altitude change (src/control/bangbang.ts): whether the switching curve accounts
   * for air drag, how far ahead it switches for the motor lag (s), and the capture law near the
   * target: its band (m) and its natural frequency (rad/s).
   */
  bangbang: { drag: boolean; lead: number; band: number; wn: number };
  /**
   * Dynamic programming on (e, v) (src/control/dp.ts) with the LQR's costs: grid points per axis,
   * the half-width of the grid in e (m) and v (m/s), the number of thrust levels, the time step (s).
   */
  dp: { n: number; eMax: number; vMax: number; inputs: number; dt: number };
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
    /** Carry the altimeter's bias as a fourth state (it cannot be observed: lesson III.9). */
    altBiasState: boolean;
    /** Prior uncertainty of that bias, m. */
    altBiasSigma: number;
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
  /** `vehicle` selects what flies (Part IV); the quadrotor is the default and the only one before. */
  sim: { level: Level; seed: number; vehicle?: VehicleId };
  /** The 1D rocket of lesson IV.4 (src/sim/vehicles/rocket.ts). */
  rocket: RocketParams;
  /** The planar rocket of Chapter L (src/sim/vehicles/tvc.ts). */
  tvc: TvcParams;
  world: WorldParams;
  setpoint: {
    x: number;
    y: number;
    z: number;
    yawDeg: number;
    rateLimit: number;
    /** Automatic setpoint motion added on top of the base value. */
    profile:
      | 'none'
      | 'square'
      | 'sine'
      | 'triangle'
      | 'circle'
      | 'figure8'
      | 'corners'
      | 'minsnap'
      | 'plan';
    profileAxis: 'y' | 'x';
    profileAmplitude: number;
    profilePeriod: number;
  };
  drone: DroneParams;
  wind: WindParams;
  sensors: SensorParams;
  vibration: VibrationParams;
  control: ControlParams;
  probe: ProbeParams;
  uncertainty: UncertaintyParams;
  sysid: SysIdParams;
  guidance: GuidanceParams;
  /** The aircraft of Chapter M (src/sim/vehicles/aircraft.ts) and its pitch autopilot. */
  aircraft: AircraftParams;
  autopilot: AutopilotParams;
  /** The satellite of Chapter N (src/sim/vehicles/satellite.ts) and its attitude control. */
  satellite: SatelliteParams;
  /** The 3D lander of lesson IV.5 (src/sim/vehicles/lander.ts) and its guidance. */
  lander: LanderParams;
  pdg: PdgSettings;
  /** The planned manoeuvre of lesson IV.6 (src/guidance/collocation.ts), flown as `setpoint.profile = 'plan'`. */
  plan: PlanParams;
}

/** Lesson IV.6: rest to rest along x past the pillar, planned by min-snap or by collocation. */
export interface PlanParams {
  method: 'minsnap' | 'collocation';
  /** Duration of the manoeuvre, s (collocation with `minTime` finds its own). */
  duration: number;
  minTime: boolean;
  /** How far it goes along x, m. */
  distance: number;
  /** Fraction of the controller's horizontal acceleration limit the plan may use. */
  accelFraction: number;
  /** Seconds after take-off at which it starts. */
  delay: number;
}

export const defaultParams = (): Params => ({
  sim: { level: 1, seed: 1337, vehicle: 'quadrotor' },
  rocket: {
    dryMass: 20,
    fuel: 10,
    thrustMax: 600,
    thrustMin: 150,
    isp: 200,
    startAlt: 120,
    startVel: 0,
    crashSpeed: 1,
  },
  tvc: {
    mass: 2000,
    inertia: 24000,
    length: 12,
    gimbalArm: 5,
    thrust: 40000,
    speed: 400,
    normalForce: 32000,
    cpAhead: 3,
    gimbalMaxDeg: 8,
    gimbalRateDeg: 30,
    actuatorTau: 0.05,
    bendHz: 5,
    bendZeta: 0.01,
    gyroStation: 0.5,
    sloshMass: 0,
    sloshHz: 0.8,
    sloshZeta: 0.005,
    sloshAhead: 2,
    startPitchDeg: 2,
  },
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
    yawDamping: 0.002,
    crashSpeed: 3,
    motorSlew: 0,
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
    posDropout: false,
    posStuck: false,
    posQuantBits: 0,
    jitterMs: 0,
    imuYawDeg: 0,
    imuRateHz: 0,
    aaFilterHz: 0,
    gyroBias: { x: 0, y: 0, z: 0 },
    attitude: 'truth',
    magnetometer: false,
    magNoiseDeg: 1,
    magDisturbDeg: 0,
    nav: 'truth',
    gpsRateHz: 10,
    gpsNoise: 0.3,
    baroNoise: 0.1,
    gpsGlitch: 0,
    ins: false,
    insStart: 5,
    beacons: false,
    beaconNoise: 0.02,
    beaconRateHz: 10,
  },
  vibration: { gyroDeg: 0, acc: 0, hoverHz: 110 },
  control: {
    rateHz: 250,
    feedforward: true,
    model: { mass: 1.0, inertiaScale: 1, motorTau: 0.03, imuYawDeg: 0 },
    gyroFilter: { lpfHz: 0, notch: 'off', notchHz: 110, notchQ: 4 },
    ahrs: { tau: 1, kp: 1, ki: 0.3, gate: 0.05 },
    mekf: { gyroDeg: 0.5, biasWalkDeg: 0.02, accDeg: 3, magDeg: 2, gate: 0 },
    navEkf: { accSigma: 0.5, biasWalk: 0.01, gate: 0 },
    beaconFilter: { kind: 'ekf', guessX: 0, guessZ: 0, sigma0: 3, start: 5 },
    fault: {
      enabled: false,
      posKp: 2,
      posKd: 2.5,
      posKi: 0.3,
      attKp: 8,
      rateKp: 40,
      maxTiltDeg: 30,
      leadLag: false,
      source: 'oracle',
    },
    lander: { ignitionAlt: 30, touchdownSpeed: 0.5, speedGain: 2 },
    tvc: {
      kp: 1.5,
      kd: 0.8,
      kx: 0,
      kv: 0,
      pitchCmdDeg: 0,
      xTarget: 0,
      notch: false,
      notchHz: 5,
      notchDepthDb: 20,
      notchWidth: 0.5,
      rolloffHz: 0,
      delayMs: 20,
    },
    fdi: { nisWindow: 10, nisThreshold: 45, nisLow: 0, cusumDrift: 0.02, cusumThreshold: 0.02 },
    l1: { kind: 'pid', estimator: 'none' },
    lqr: {
      qPos: 100,
      qVel: 10,
      qInt: 5,
      r: 1,
      integral: false,
      lagState: false,
      observer: false,
      kfQ: 0.1,
      kfR: 0.02,
    },
    shaping: { leadZHz: 0, leadPHz: 0, lagZHz: 0, lagPHz: 0, notchHz: 0, notchQ: 2 },
    hinf: { k: 15, wi: 1, wz: 0, wp: 0, gammaFactor: 1.05 },
    smc: { lambda: 3, k: 6, phi: 0 },
    mrac: { kp: 9, kd: 6, gamma: 1, gammaP: 1, gammaD: 1, gammaG: 10, sigma: 0 },
    backstepping: { k1: 2, k2: 2, k3: 2 },
    bangbang: { drag: false, lead: 0, band: 0.05, wn: 6 },
    dp: { n: 81, eMax: 4, vMax: 6, inputs: 25, dt: 0.02 },
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
    kalman: {
      accSigma: 0.5,
      posSigma: 0.05,
      biasSigma: 0.05,
      altBiasState: false,
      altBiasSigma: 0.5,
    },
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
  uncertainty: { massPct: 30, tauFactor: 2, delayMs: 20 },
  sysid: { estimator: 'direct', minCoherence: 0, amp: 0.4, seconds: 60 },
  guidance: {
    enabled: false,
    law: 'pursuit',
    N: 4,
    speed: 4,
    pursuitGain: 1.5,
    start: 5,
    targetX: 20,
    targetZ: -10,
    targetHeadingDeg: 90,
    targetSpeed: 2,
    weaveAcc: 0,
    weavePeriod: 4,
  },
  probe: { point: 'none', signal: 'sine', amp: 0.3, freqHz: 1, f0Hz: 0.1, f1Hz: 20, durationS: 60 },
  aircraft: {
    mass: 5000,
    wingArea: 25,
    chord: 1.8,
    inertia: 25000,
    thrustMax: 24000,
    engineTau: 1.5,
    cl0: 0.2,
    clAlpha: 5,
    clQ: 4,
    clDe: 0.35,
    cd0: 0.022,
    cdK: 0.045,
    cm0: 0.03,
    cmAlpha: -1.2,
    cmQ: -6,
    cmDe: -1.2,
    servoTau: 0.05,
    servoRate: 60,
    elevatorMax: 20,
    speed: 110,
    altitude: 4000,
  },
  autopilot: {
    gain: 0,
    kTheta: 0,
    schedule: 'none',
    designSpeed: 110,
    elevatorOffset: 0,
    thetaOffset: 0,
    autothrottle: false,
    speedTarget: 110,
    speedRate: 2,
    speedGain: 0.05,
    delayMs: 40,
    outer: 'none',
    altitudeOffset: 0,
    climbGain: 0.1,
    gammaMax: 5,
    gammaRate: 1,
    speedInt: 0.01,
    speedLoop: 0.3,
    kTP: 1,
    kTI: 0.2,
    kEP: 1,
    kEI: 0.2,
    law: 'classic',
    ndiBandwidth: 8,
    ndiAttitude: 2,
    pilot: false,
    pilotGain: 0.5,
    pilotDelayMs: 250,
    modelError: 0,
  },
  satellite: {
    inertia: { x: 4, y: 5, z: 3 },
    wheelTorque: 0.2,
    wheelMomentum: 0.6,
    disturbance: { x: 0, y: 0, z: 0 },
    thrusterTorque: 0.5,
    minPulse: 0.1,
    thrusterFlow: 1,
    slewAxis: 'diagonal',
    slewDeg: 120,
    actuator: 'wheels',
    kp: 1,
    kd: 3,
    shortest: true,
    dump: false,
    dumpStart: 0.4,
    dumpStop: 0.1,
    dumpFeedforward: false,
    deadbandDeg: 0.1,
    driftRateDeg: 1,
    panel: { inertia: 0, hz: 0.5, zeta: 0.005, sensor: 'hub' },
    profileTorque: 0,
    shaper: 'none',
    shaperHz: 0.5,
    navigation: false,
    gyroBiasDegH: { x: 5, y: -3, z: 4 },
    gyroArw: 0.01,
    trackerHz: 1,
    trackerArcsec: 5,
    estimateBias: false,
    outageStart: 20,
    outageLength: 0,
  },
  lander: {
    dryMass: 25,
    fuel: 1.75,
    thrustMin: 150,
    thrustMax: 600,
    isp: 200,
    start: { x: 60, y: 100, z: 30 },
    startVel: { x: -6, y: -18, z: 3 },
    dispersionCase: 0,
    dispersePos: 40,
    disperseVel: 10,
    windForce: { x: 0, y: 0, z: 0 },
    thrustScale: 1,
    glideSlopeDeg: 30,
    crashSpeed: 1,
  },
  pdg: {
    nodes: 16,
    throttle: 0.9,
    replanEvery: 0.5,
    freezeBefore: 0.5,
    gateAltitude: 3,
    gateSpeed: 1.5,
    reserve: 0.45,
  },
  plan: {
    method: 'minsnap',
    duration: 3,
    minTime: false,
    distance: 6,
    accelFraction: 0.8,
    delay: 1,
  },
});

export const GRAVITY = 9.81;
