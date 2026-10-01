import { qRotate, type Quat } from '@/math/quat';
import { cross, sub, v3, type Vec3 } from '@/math/vec3';
import { motorTorques } from '@/sim/dynamics';
import { MOTOR_POSITIONS } from '@/sim/drone';
import type { DroneParams } from '@/sim/params';
import type { Measurement } from '@/sim/sensors';
import { LowPass2 } from './filters';
import { AltitudeKalman, type AltitudeNoise } from './kalman';

/**
 * Fault detection and isolation (docs/analysis.md §4, Chapter J; [Basseville 1993]).
 *
 * Two residuals, two tests:
 *  - the altimeter: a vertical Kalman filter driven by the accelerometer predicts each fix; the
 *    normalised innovation squared (NIS), summed over a window, is χ²-distributed while the
 *    sensor is healthy. The test is two-sided: a jump makes the sum too large, and a stuck
 *    sensor, which has stopped being noisy, makes it too small.
 *  - the motors: the angular acceleration the gyro shows against the one the motor telemetry
 *    implies. A propeller that makes less thrust than its rotor speed promises leaves a torque
 *    residual in a direction that names the motor. A CUSUM per motor accumulates it.
 */
export interface FdiSettings {
  /** Window of the NIS sum, fixes. */
  nisWindow: number;
  /** Alarm when the windowed NIS exceeds this (the χ² quantile the designer chooses)… */
  nisThreshold: number;
  /** …or falls below this: innovations too small for the sensor's noise, a sensor that is stuck. */
  nisLow: number;
  /** CUSUM: deviation tolerated without accumulating (N·m), and the alarm level (N·m·s). */
  cusumDrift: number;
  cusumThreshold: number;
}

export interface FdiState {
  /** Windowed NIS of the altimeter, and whether it is in alarm. */
  nis: number;
  altAlarm: boolean;
  /** CUSUM statistic of each motor, N·m·s. */
  cusum: [number, number, number, number];
  /** The motor the CUSUM names as failed (0…3), or −1. */
  motor: number;
}

/** Torque about (x, z) that one newton of motor `i` makes, normalised: its signature. */
const SIGNATURE = MOTOR_POSITIONS.map((r) => {
  const t = v3(-r.z, 0, r.x);
  const n = Math.hypot(t.x, t.z);
  return v3(t.x / n, 0, t.z / n);
});

export class FaultMonitor {
  private kf = new AltitudeKalman();
  private nisBuf: number[] = [];
  private wPrev: Vec3 | null = null;
  private fAlpha = [new LowPass2(15, 0.001), new LowPass2(15, 0.001)];
  private fTau = [new LowPass2(15, 0.001), new LowPass2(15, 0.001)];
  readonly state: FdiState = {
    nis: 0,
    altAlarm: false,
    cusum: [0, 0, 0, 0],
    motor: -1,
  };
  /** Torque residual (x, z) after filtering, N·m. */
  residual = v3();

  reset(): void {
    this.kf.reset();
    this.nisBuf = [];
    this.wPrev = null;
    for (const f of [...this.fAlpha, ...this.fTau]) f.reset();
    Object.assign(this.state, { nis: 0, altAlarm: false, cusum: [0, 0, 0, 0], motor: -1 });
    this.residual = v3();
  }

  /** One step of both tests on a fresh measurement. `armed` holds the motor test off the ground. */
  update(
    m: Measurement,
    q: Quat,
    drone: DroneParams,
    noise: AltitudeNoise,
    s: FdiSettings,
    gravity: number,
    dt: number,
    armed: boolean,
  ): FdiState {
    // Altimeter: the accelerometer's vertical specific force predicts, the fix is tested.
    const accUp = qRotate(q, m.acc).y;
    this.kf.step(dt, accUp, m.posFresh ? m.pos.y : null, noise, gravity);
    if (m.posFresh && Number.isFinite(this.kf.nis)) {
      this.nisBuf.push(this.kf.nis);
      if (this.nisBuf.length > s.nisWindow) this.nisBuf.shift();
      this.state.nis = this.nisBuf.reduce((a, b) => a + b, 0);
      this.state.altAlarm =
        this.nisBuf.length >= s.nisWindow &&
        (this.state.nis > s.nisThreshold || this.state.nis < s.nisLow);
    }

    // Motors: I·ω̇ + ω×Iω + damping·ω should equal the torque the rotor speeds promise.
    const w = m.omega;
    if (this.wPrev && m.motors && armed) {
      const I = drone.inertia;
      const a = v3((w.x - this.wPrev.x) / dt, 0, (w.z - this.wPrev.z) / dt);
      const Iw = v3(I.x * w.x, I.y * w.y, I.z * w.z);
      const g = cross(w, Iw);
      const need = v3(
        I.x * a.x + g.x + drone.angularDamping * w.x,
        0,
        I.z * a.z + g.z + drone.angularDamping * w.z,
      );
      const promised = motorTorques(m.motors, drone);
      for (const f of [...this.fAlpha, ...this.fTau]) f.dt = dt;
      const r = sub(
        v3(this.fAlpha[0]!.update(need.x), 0, this.fAlpha[1]!.update(need.z)),
        v3(this.fTau[0]!.update(promised.x), 0, this.fTau[1]!.update(promised.z)),
      );
      this.residual = r;
      // A weak motor i shows as a torque missing along its signature: −r·dᵢ grows.
      for (let i = 0; i < 4; i++) {
        const d = SIGNATURE[i]!;
        const x = -(r.x * d.x + r.z * d.z);
        this.state.cusum[i] = Math.max(0, this.state.cusum[i]! + (x - s.cusumDrift) * dt);
      }
      if (this.state.motor < 0) {
        let best = -1;
        for (let i = 0; i < 4; i++)
          if (
            this.state.cusum[i]! > s.cusumThreshold &&
            (best < 0 || this.state.cusum[i]! > this.state.cusum[best]!)
          )
            best = i;
        this.state.motor = best;
      }
    }
    this.wPrev = { ...w };
    return this.state;
  }
}
