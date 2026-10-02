import { qIdentity, type Quat } from '@/math/quat';
import { v3, type Vec3 } from '@/math/vec3';
import {
  attitudeError,
  targetAttitude,
  type SatelliteInput,
  type SatelliteParams,
  type SatelliteState,
} from '@/sim/vehicles/satellite';

const comps = ['x', 'y', 'z'] as const;

/**
 * The slowest drift the thrusters can hold, rad/s: one minimum pulse turns a drift of ω into
 * −ω exactly when ω = τ·Δt_min / (2J). Slower drifts need shorter pulses than the valves give.
 */
export const minDrift = (sp: SatelliteParams, axis: 'x' | 'y' | 'z' = 'x'): number =>
  (sp.thrusterTorque * sp.minPulse) / (2 * sp.inertia[axis]);

/** The drift the thrusters actually hold on an axis: ω_c, but never below `minDrift`, rad/s. */
const drift = (sp: SatelliteParams, axis: 'x' | 'y' | 'z') =>
  Math.max((sp.driftRateDeg * Math.PI) / 180, minDrift(sp, axis));

/** How long each pulse at a wall lasts: it turns a drift of ω into −ω, t_p = 2Jω/τ, s. */
export const pulseTime = (sp: SatelliteParams, axis: 'x' | 'y' | 'z' = 'x'): number =>
  (2 * sp.inertia[axis] * drift(sp, axis)) / sp.thrusterTorque;

/**
 * The period of the dead-band limit cycle on one axis, s: the angle drifts at ω across the band
 * (2·db, twice per period), and at each wall the pulse takes t_p and brings it back to where it
 * started firing. T = 4·db/ω + 2·t_p.
 */
export function limitCyclePeriod(sp: SatelliteParams, axis: 'x' | 'y' | 'z' = 'x'): number {
  const db = (sp.deadbandDeg * Math.PI) / 180;
  return (4 * db) / drift(sp, axis) + 2 * pulseTime(sp, axis);
}

/** Propellant rate of that limit cycle, g/s: two pulses per period, F = 2·t_p·flow / T. */
export const limitCycleFuelRate = (sp: SatelliteParams, axis: 'x' | 'y' | 'z' = 'x'): number =>
  (2 * pulseTime(sp, axis) * sp.thrusterFlow) / limitCyclePeriod(sp, axis);

/**
 * Attitude control of the satellite (lessons IV.18–IV.20).
 *
 * Wheels: quaternion feedback τ = −kp·q_e − kd·ω, globally stabilising for a rigid body [Wie
 * 2008]. With `dump`, a thruster pair fires against the momentum stored in a wheel once it passes
 * `dumpStart`, until it falls below `dumpStop`; with `dumpFeedforward` the wheels are told the
 * thruster torque and cancel it, so the pointing does not see it.
 *
 * Thrusters: on each axis a dead band on the angle; at a wall, moving outward, the pair fires
 * until the drift is −ω_c (for at least the minimum pulse). The result is the limit cycle of
 * `limitCycleFuelRate`.
 */
export class SatelliteController {
  /** The error quaternion of the last tick (sign-fixed when `shortest`). */
  error: Quat = qIdentity();
  /** Per axis: dumping momentum now. */
  private dumping = { x: false, y: false, z: false };
  /** Per axis: the direction a pulse fires in (0: none) and how long it has fired, s. */
  private fire = { x: 0, y: 0, z: 0 };
  private fired = { x: 0, y: 0, z: 0 };
  /** The torque the controller asked for, N·m (before the wheels limit it). */
  command: Vec3 = v3();

  reset(): void {
    this.error = qIdentity();
    this.dumping = { x: false, y: false, z: false };
    this.fire = { x: 0, y: 0, z: 0 };
    this.fired = { x: 0, y: 0, z: 0 };
    this.command = v3();
  }

  tick(s: SatelliteState, sp: SatelliteParams, dt: number): SatelliteInput {
    const shortest = sp.actuator === 'thrusters' || sp.shortest;
    const e = attitudeError(s.q, targetAttitude(sp), shortest);
    this.error = e;
    const wheel = v3();
    const thrusters = v3();
    if (sp.actuator === 'thrusters') {
      const db = (sp.deadbandDeg * Math.PI) / 180;
      for (const c of comps) {
        const theta = 2 * e[c];
        const w = s.w[c];
        const hold = drift(sp, c);
        if (this.fire[c] !== 0) {
          // Keep firing for the minimum pulse, and until the drift is reversed to ω_c.
          this.fired[c] += dt;
          const out = -this.fire[c] * w; // the rate away from the wall the pulse fights
          if (this.fired[c] >= sp.minPulse && out <= -hold) this.fire[c] = 0;
        } else if (Math.abs(theta) > db && Math.sign(theta) * w > -hold) {
          this.fire[c] = -Math.sign(theta);
          this.fired[c] = 0;
        }
        thrusters[c] = this.fire[c];
      }
      this.command = v3(
        thrusters.x * sp.thrusterTorque,
        thrusters.y * sp.thrusterTorque,
        thrusters.z * sp.thrusterTorque,
      );
      return { wheel, thrusters };
    }
    for (const c of comps) {
      wheel[c] = -sp.kp * e[c] - sp.kd * s.w[c];
      if (sp.dump) {
        const h = s.h[c];
        if (Math.abs(h) > sp.dumpStart) this.dumping[c] = true;
        else if (Math.abs(h) < sp.dumpStop) this.dumping[c] = false;
        if (this.dumping[c]) {
          // An outside torque against the stored momentum: dH/dt = τ_thr.
          thrusters[c] = -Math.sign(h);
          if (sp.dumpFeedforward) wheel[c] -= thrusters[c] * sp.thrusterTorque;
        }
      }
    }
    this.command = { ...wheel };
    return { wheel, thrusters };
  }
}
