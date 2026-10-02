import { qFromAxisAngle, qIdentity, type Quat } from '@/math/quat';
import { v3, type Vec3 } from '@/math/vec3';
import {
  attitudeError,
  slewAxis,
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
 * An input shaper for a mode of frequency `hz` (the damped frequency) and damping `zeta`: the
 * steps of the command, as times, s, and fractions that add up to 1 [Singer 1990]. ZV: two
 * steps half a period apart; ZVD: three, a whole period in all, and far less sensitive to an
 * error in the frequency.
 */
export function shaperSteps(
  kind: 'none' | 'zv' | 'zvd',
  hz: number,
  zeta = 0,
): { t: number; a: number }[] {
  if (kind === 'none' || !(hz > 0)) return [{ t: 0, a: 1 }];
  const k = Math.exp((-zeta * Math.PI) / Math.sqrt(1 - zeta * zeta));
  const half = 1 / (2 * hz);
  if (kind === 'zv')
    return [
      { t: 0, a: 1 / (1 + k) },
      { t: half, a: k / (1 + k) },
    ];
  const d = (1 + k) * (1 + k);
  return [
    { t: 0, a: 1 / d },
    { t: half, a: (2 * k) / d },
    { t: 2 * half, a: (k * k) / d },
  ];
}

/** Inertia about the slew axis, the panel's included (it turns with the hub in a slow slew), kg·m². */
export function slewInertia(sp: SatelliteParams): number {
  const a = slewAxis(sp);
  const J = sp.inertia;
  return J.x * a.x * a.x + J.y * a.y * a.y + (J.z + sp.panel.inertia) * a.z * a.z;
}

/** Length of the profiled slew: rest to rest at ±profileTorque, 2·√(Θ·J/τ), s (0: a step). */
export function profileTime(sp: SatelliteParams): number {
  if (!(sp.profileTorque > 0)) return 0;
  const angle = Math.abs((sp.slewDeg * Math.PI) / 180);
  return 2 * Math.sqrt((angle * slewInertia(sp)) / sp.profileTorque);
}

/** The slew angle, rate and acceleration at time `t`, before shaping: a step, or bang-bang. */
function slewProfile(sp: SatelliteParams, t: number): [number, number, number] {
  const angle = (sp.slewDeg * Math.PI) / 180;
  if (t < 0) return [0, 0, 0];
  const total = profileTime(sp);
  if (total === 0 || t >= total) return [angle, 0, 0];
  const acc = (Math.sign(angle) * sp.profileTorque) / slewInertia(sp);
  const half = total / 2;
  if (t < half) return [0.5 * acc * t * t, acc * t, acc];
  const r = total - t;
  return [angle - 0.5 * acc * r * r, acc * r, -acc];
}

/**
 * The commanded slew at time `t` from the start, through the shaper if any: the angle about the
 * slew axis, rad, its rate, rad/s, and acceleration, rad/s². The shaper sums delayed, scaled
 * copies of the profile, so a profile that ends at rest still ends at rest.
 */
export function shapedSlew(sp: SatelliteParams, t: number): [number, number, number] {
  const out: [number, number, number] = [0, 0, 0];
  for (const st of shaperSteps(sp.shaper, sp.shaperHz)) {
    const p = slewProfile(sp, t - st.t);
    for (let i = 0; i < 3; i++) out[i] = out[i]! + st.a * p[i]!;
  }
  return out;
}

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
  /** Time since the start, s (the shaper's clock). */
  private time = 0;

  reset(): void {
    this.time = 0;
    this.error = qIdentity();
    this.dumping = { x: false, y: false, z: false };
    this.fire = { x: 0, y: 0, z: 0 };
    this.fired = { x: 0, y: 0, z: 0 };
    this.command = v3();
  }

  tick(s: SatelliteState, sp: SatelliteParams, dt: number): SatelliteInput {
    const shortest = sp.actuator === 'thrusters' || sp.shortest;
    const plain = sp.shaper === 'none' && !(sp.profileTorque > 0);
    const [phi, phiDot, phiDdot] = plain ? [0, 0, 0] : shapedSlew(sp, this.time);
    const axis = slewAxis(sp);
    const target = plain ? targetAttitude(sp) : qFromAxisAngle(axis, phi);
    const e = attitudeError(s.q, target, shortest);
    this.time += dt;
    // The profile's rate and the torque it takes (zero for a step).
    const wRef = { x: axis.x * phiDot, y: axis.y * phiDot, z: axis.z * phiDot };
    const J = sp.inertia;
    const ff = {
      x: J.x * axis.x * phiDdot,
      y: J.y * axis.y * phiDdot,
      z: (J.z + sp.panel.inertia) * axis.z * phiDdot,
    };
    this.error = e;
    // A sensor on the panel's tip sees the hub's angle plus the panel's own (lesson IV.21).
    const tip = sp.panel.inertia > 0 && sp.panel.sensor === 'tip';
    const meas = tip ? { ...e, z: e.z + s.eta / 2 } : e;
    const rate = tip ? { ...s.w, z: s.w.z + s.etaDot } : s.w;
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
      wheel[c] = ff[c] - sp.kp * meas[c] - sp.kd * (rate[c] - wRef[c]);
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
