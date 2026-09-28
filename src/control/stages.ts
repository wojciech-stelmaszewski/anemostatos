import { LowPass2 } from '@/estimation/filters';
import { qRotate } from '@/math/quat';
import { add, scale, sub, v3, type Vec3 } from '@/math/vec3';
import { motorTorques } from '@/sim/dynamics';
import { GRAVITY, type Params } from '@/sim/params';
import type { Measurement } from '@/sim/sensors';
import { Pid } from './pid';
import { pidLoop, type InfoRow, type LoopTerms } from './types';

/**
 * Swappable stages of the L3 pipeline (docs/beyond-pid.md §3.2, control-architecture.md §4).
 * Rate stages turn a body-rate setpoint into torques; compensation stages turn a desired
 * acceleration into a desired force.
 */

const DEG = 180 / Math.PI;
type Motors = [number, number, number, number];

/** The inertia the controller believes in: the true one times `control.model.inertiaScale`. */
export const modelInertia = (p: Params): Vec3 => {
  const I = p.drone.inertia;
  const k = p.control.model.inertiaScale;
  return v3(I.x * k, I.y * k, I.z * k);
};

/**
 * Actual motor thrusts: from motor telemetry if the sensors provide it, otherwise from a
 * first-order model of the motors driven by the commands (the classic INDI actuator model).
 */
class MotorEstimate {
  private model: Motors = [0, 0, 0, 0];
  private cmd: Motors = [0, 0, 0, 0];

  reset(): void {
    this.model = [0, 0, 0, 0];
    this.cmd = [0, 0, 0, 0];
  }

  applied(motors: Motors): void {
    this.cmd = [...motors];
  }

  read(s: Measurement, p: Params, dt: number): Motors {
    const a = 1 - Math.exp(-dt / Math.max(p.control.model.motorTau, 1e-4));
    for (let i = 0; i < 4; i++)
      this.model[i] = this.model[i]! + (this.cmd[i]! - this.model[i]!) * a;
    return s.motors ?? this.model;
  }
}

/** Three identical filters, one per axis. */
class Filter3 {
  private f = [new LowPass2(20, 0.001), new LowPass2(20, 0.001), new LowPass2(20, 0.001)];
  reset(): void {
    for (const f of this.f) f.reset();
  }
  update(v: Vec3, hz: number, dt: number): Vec3 {
    for (const f of this.f) {
      f.cutoffHz = hz;
      f.dt = dt;
    }
    return v3(this.f[0]!.update(v.x), this.f[1]!.update(v.y), this.f[2]!.update(v.z));
  }
}

// ---------------------------------------------------------------------------------------------
// Rate stages

export interface RateStage {
  reset(): void;
  resetIntegrators(): void;
  /** Body torque (N·m) for the body-rate setpoint wSp (deg/s: x roll, y yaw, z pitch). */
  torque(wSp: Vec3, s: Measurement, dt: number, p: Params): Vec3;
  /** What the mixer actually commanded. */
  applied(motors: Motors): void;
  loops(): Record<string, LoopTerms>;
  extras(): Record<string, number>;
  describe(p: Params, loopId: string): InfoRow[];
}

/** Rate PID per axis, scaled by the model inertia so the gains are in angular-acceleration units. */
export class PidRateStage implements RateStage {
  readonly roll = new Pid();
  readonly pitch = new Pid();
  readonly yaw = new Pid();

  reset(): void {
    this.roll.reset();
    this.pitch.reset();
    this.yaw.reset();
  }

  resetIntegrators(): void {
    this.roll.integral = this.pitch.integral = this.yaw.integral = 0;
  }

  torque(wSp: Vec3, s: Measurement, dt: number, p: Params): Vec3 {
    const c = p.control.l3;
    const w = v3(s.omega.x * DEG, s.omega.y * DEG, s.omega.z * DEG);
    const aLim = 20000; // deg/s², far beyond what the motors can deliver; the mixer is the real limit
    const ar = this.roll.update(c.rateRP, wSp.x, w.x, dt, 0, -aLim, aLim).output;
    const ap = this.pitch.update(c.rateRP, wSp.z, w.z, dt, 0, -aLim, aLim).output;
    const ay = this.yaw.update(c.rateYaw, wSp.y, w.y, dt, 0, -aLim, aLim).output;
    const I = modelInertia(p);
    return v3((I.x * ar) / DEG, (I.y * ay) / DEG, (I.z * ap) / DEG);
  }

  applied(): void {}

  loops(): Record<string, LoopTerms> {
    return {
      'rate.roll': pidLoop(this.roll.last),
      'rate.pitch': pidLoop(this.pitch.last),
      'rate.yaw': pidLoop(this.yaw.last),
    };
  }

  extras(): Record<string, number> {
    return {};
  }

  describe(): InfoRow[] {
    return [];
  }
}

const emptyLoop = (): LoopTerms => ({
  setpoint: 0,
  measurement: 0,
  error: 0,
  parts: indiParts(0, 0, 0),
  unsaturated: 0,
  output: 0,
  saturated: false,
});

const indiParts = (held: number, want: number, have: number): LoopTerms['parts'] => [
  { key: 'held', label: 'τ₀/Î (motors now)', value: held, like: 'i' },
  { key: 'want', label: 'α_des', value: want, like: 'p' },
  { key: 'have', label: '−ω̇ (measured)', value: have, like: 'd' },
];

/**
 * Incremental nonlinear dynamic inversion on body rates (Smeur, Chu & de Croon 2016):
 *   τ = τ₀,f + Î·(α_des − ω̇_f),   α_des = K·(ω_sp − ω)
 * τ₀ is the torque the motors produce *right now* (from motor telemetry or a motor model) and ω̇
 * the measured angular acceleration; both pass through the same filter so they refer to the same
 * moment in time. Whatever else acts on the drone — unmodelled torques, a weak motor, a wrong
 * inertia — is already inside the measured ω̇ and needs no model.
 */
export class RateIndi implements RateStage {
  private motors = new MotorEstimate();
  private wFilt = new Filter3();
  private tauFilt = new Filter3();
  private wPrev: Vec3 | null = null;
  private last = { roll: emptyLoop(), pitch: emptyLoop(), yaw: emptyLoop() };
  private alphaF = v3();

  reset(): void {
    this.motors.reset();
    this.wFilt.reset();
    this.tauFilt.reset();
    this.wPrev = null;
    this.last = { roll: emptyLoop(), pitch: emptyLoop(), yaw: emptyLoop() };
    this.alphaF = v3();
  }

  resetIntegrators(): void {}

  applied(motors: Motors): void {
    this.motors.applied(motors);
  }

  torque(wSp: Vec3, s: Measurement, dt: number, p: Params): Vec3 {
    const c = p.control.indi;
    // Filtered rate and its derivative (the angular acceleration the drone really has).
    const wf = this.wFilt.update(s.omega, c.filterHz, dt);
    const alpha = this.wPrev ? scale(sub(wf, this.wPrev), 1 / dt) : v3();
    this.wPrev = wf;
    this.alphaF = alpha;
    // Torque of the motors right now, through the same filter — or deliberately not, to show why.
    const tau0 = motorTorques(this.motors.read(s, p, dt), p.drone);
    const tau0f = c.syncFilters ? this.tauFilt.update(tau0, c.filterHz, dt) : tau0;
    const I = modelInertia(p);
    const want = v3(
      c.rateKpRP * (wSp.x / DEG - s.omega.x),
      c.rateKpYaw * (wSp.y / DEG - s.omega.y),
      c.rateKpRP * (wSp.z / DEG - s.omega.z),
    );
    const tau = add(
      tau0f,
      v3(I.x * (want.x - alpha.x), I.y * (want.y - alpha.y), I.z * (want.z - alpha.z)),
    );
    // Report in angular-acceleration units (deg/s²), like the PID rate loop.
    const loop = (sp: number, meas: number, i: number, h: number, wnt: number, a: number) => {
      const held = (h / i) * DEG;
      const parts = indiParts(held, wnt * DEG, -a * DEG);
      const total = held + wnt * DEG - a * DEG;
      return {
        setpoint: sp,
        measurement: meas * DEG,
        error: sp - meas * DEG,
        parts,
        unsaturated: total,
        output: total,
        saturated: false,
      };
    };
    this.last = {
      roll: loop(wSp.x, s.omega.x, I.x, tau0f.x, want.x, alpha.x),
      yaw: loop(wSp.y, s.omega.y, I.y, tau0f.y, want.y, alpha.y),
      pitch: loop(wSp.z, s.omega.z, I.z, tau0f.z, want.z, alpha.z),
    };
    return tau;
  }

  loops(): Record<string, LoopTerms> {
    return {
      'rate.roll': this.last.roll,
      'rate.pitch': this.last.pitch,
      'rate.yaw': this.last.yaw,
    };
  }

  extras(): Record<string, number> {
    return {
      'indi.alpha.x': this.alphaF.x * DEG,
      'indi.alpha.y': this.alphaF.y * DEG,
      'indi.alpha.z': this.alphaF.z * DEG,
    };
  }

  describe(p: Params, loopId: string): InfoRow[] {
    if (!loopId.startsWith('rate.')) return [];
    const c = p.control.indi;
    return [
      {
        label: 'law',
        value: 'τ = τ₀ + Î·(α_des − ω̇)',
        hint: 'Keep what the motors already do, and change it by exactly the missing angular acceleration.',
      },
      {
        label: 'rate gain (roll/pitch · yaw)',
        value: `${c.rateKpRP} · ${c.rateKpYaw} 1/s`,
        hint: 'α_des = K·(ω_sp − ω): the only tuning of the rate loop.',
      },
      {
        label: 'filter',
        value: `${c.filterHz} Hz · ${c.syncFilters ? 'synchronised' : 'NOT synchronised'}`,
        hint: 'The gyro derivative and the motor torque must pass through the same filter.',
      },
      {
        label: 'assumed inertia',
        value: `${p.control.model.inertiaScale.toFixed(2)} × true`,
      },
      {
        label: 'τ₀ from',
        value: p.sensors.motorFeedback
          ? 'motor telemetry'
          : `motor model (τ̂ = ${p.control.model.motorTau} s)`,
      },
    ];
  }
}

// ---------------------------------------------------------------------------------------------
// Compensation stages

export interface Compensation {
  /** True if the stage integrates on its own, so the velocity loop's I term is bypassed. */
  readonly replacesIntegral: boolean;
  reset(): void;
  /** Update filters with a fresh measurement (called at the rate-loop frequency). */
  sample(s: Measurement, p: Params, dt: number): void;
  /** Desired world force (N) for the desired acceleration aSp (m/s², gravity excluded). */
  force(aSp: Vec3, p: Params): Vec3;
  applied(motors: Motors): void;
  loops(aSp: Vec3): Record<string, LoopTerms>;
  extras(): Record<string, number>;
  describe(p: Params, loopId: string): InfoRow[];
}

/** The nominal model: F = m̂·(a_sp + g), with the gravity term as the feedforward. */
export class NoCompensation implements Compensation {
  readonly replacesIntegral = false;
  reset(): void {}
  sample(): void {}
  applied(): void {}
  force(a: Vec3, p: Params): Vec3 {
    const m = p.control.model.mass;
    const g = p.control.feedforward ? GRAVITY : 0;
    return v3(m * a.x, m * (a.y + g), m * a.z);
  }
  loops(): Record<string, LoopTerms> {
    return {};
  }
  extras(): Record<string, number> {
    return {};
  }
  describe(): InfoRow[] {
    return [];
  }
}

/**
 * Cascaded (acceleration) INDI (Smeur, de Croon & Chu 2018; Tal & Karaman 2021):
 *   F = F₀,f + m̂·(a_sp − a_f)
 * F₀ is the thrust vector the motors produce now, a the acceleration measured by the
 * accelerometer (rotated to the world frame, gravity added back). A gust shows up in a at once —
 * long before it has moved the drone far enough to cause a position error.
 */
export class AccelIndi implements Compensation {
  readonly replacesIntegral = true;
  private motors = new MotorEstimate();
  private aFilt = new Filter3();
  private fFilt = new Filter3();
  private aF = v3();
  private fF = v3();
  private started = false;
  private mHat = 1;

  reset(): void {
    this.motors.reset();
    this.aFilt.reset();
    this.fFilt.reset();
    this.aF = v3();
    this.fF = v3();
    this.started = false;
  }

  applied(motors: Motors): void {
    this.motors.applied(motors);
  }

  sample(s: Measurement, p: Params, dt: number): void {
    const c = p.control.indi;
    this.mHat = p.control.model.mass;
    const aWorld = add(qRotate(s.q, s.acc), v3(0, -GRAVITY, 0));
    const m = this.motors.read(s, p, dt);
    const fNow = qRotate(s.q, v3(0, m[0] + m[1] + m[2] + m[3], 0));
    this.aF = this.aFilt.update(aWorld, c.accFilterHz, dt);
    this.fF = c.syncFilters ? this.fFilt.update(fNow, c.accFilterHz, dt) : fNow;
    this.started = true;
  }

  force(aSp: Vec3, p: Params): Vec3 {
    const m = p.control.model.mass;
    if (!this.started) return v3(m * aSp.x, m * (aSp.y + GRAVITY), m * aSp.z);
    return add(this.fF, scale(sub(aSp, this.aF), m));
  }

  loops(aSp: Vec3): Record<string, LoopTerms> {
    const m = Math.max(this.mHat, 1e-3);
    const loop = (sp: number, a: number, f0: number): LoopTerms => {
      const held = f0 / m;
      return {
        setpoint: sp,
        measurement: a,
        error: sp - a,
        parts: [
          { key: 'held', label: 'F₀/m̂ (motors now)', value: held, like: 'i' },
          { key: 'want', label: 'a_sp', value: sp, like: 'p' },
          { key: 'have', label: '−a (measured)', value: -a, like: 'd' },
        ],
        unsaturated: held + sp - a,
        output: held + sp - a,
        saturated: false,
      };
    };
    return {
      'acc.x': loop(aSp.x, this.aF.x, this.fF.x),
      'acc.y': loop(aSp.y, this.aF.y, this.fF.y),
      'acc.z': loop(aSp.z, this.aF.z, this.fF.z),
    };
  }

  extras(): Record<string, number> {
    // What INDI implicitly "knows" about the disturbance: m̂·a − F₀ − m̂·g (engine's definition).
    const d = sub(scale(this.aF, this.mHat), add(this.fF, v3(0, -this.mHat * GRAVITY, 0)));
    return { 'est.dist.x': d.x, 'est.dist.y': d.y, 'est.dist.z': d.z };
  }

  describe(p: Params, loopId: string): InfoRow[] {
    if (!loopId.startsWith('acc.') && !loopId.startsWith('vel.')) return [];
    return [
      {
        label: 'acceleration law',
        value: 'F = F₀ + m̂·(a_sp − a)',
        hint: 'Keep the thrust vector the motors produce now, and change it by exactly the missing acceleration.',
      },
      {
        label: 'accelerometer filter',
        value: `${p.control.indi.accFilterHz} Hz`,
      },
      {
        label: 'velocity I term',
        value: 'bypassed',
        hint: 'The incremental law already integrates; a second integrator would only add overshoot.',
      },
    ];
  }
}
