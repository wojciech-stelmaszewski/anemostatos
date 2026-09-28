import { qConj, qFromTo, qMul, qRotate, qToEuler, type Quat } from '@/math/quat';
import { clamp } from '@/math/util';
import { clampLength, cross, dot, length, scale, sub, v3, type Vec3 } from '@/math/vec3';
import { GRAVITY, type AttitudeLaw, type Params } from '@/sim/params';
import type { Measurement } from '@/sim/sensors';
import type { LoopTerms, Setpoint } from './types';

const DEG = 180 / Math.PI;
const up = v3(0, 1, 0);

/**
 * Geometric tracking controller, translational part (Lee, Leok & McClamroch 2010), in the
 * cascaded form used by modern stacks: it outputs a desired acceleration, the attitude and rate
 * loops below turn it into torques.
 *   a = a_ref − K_p·(p − p_ref) − K_v·(v − v_ref) − K_i·∫(p − p_ref)
 * With feedforward off, the reference's derivatives are ignored and it is pure feedback.
 */
export class GeometricOuter {
  private integral = v3();
  private last: Record<'x' | 'y' | 'z', LoopTerms> = {
    x: emptyLoop(),
    y: emptyLoop(),
    z: emptyLoop(),
  };

  reset(): void {
    this.integral = v3();
    this.last = { x: emptyLoop(), y: emptyLoop(), z: emptyLoop() };
  }

  resetIntegrators(): void {
    this.integral = v3();
  }

  /** Desired acceleration (m/s², gravity excluded), limited like the PID cascade's. */
  update(sp: Setpoint, s: Measurement, dt: number, p: Params, noIntegral: boolean): Vec3 {
    const g = p.control.geometric;
    const c = p.control.l3;
    const ff = g.feedforward;
    const vRef = ff && sp.vel ? sp.vel : v3();
    const aRef = ff && sp.acc ? sp.acc : v3();
    const ep = sub(s.pos, sp.pos);
    const ev = sub(s.vel, vRef);
    const ki = noIntegral ? 0 : g.ki;
    const aH = GRAVITY * Math.tan((c.maxTiltDeg * Math.PI) / 180);
    const aUp = (4 * p.drone.maxMotorThrust) / Math.max(p.control.model.mass, 0.1) - GRAVITY;
    if (ki > 0) {
      // Bounded integral (at most 3 m/s² of correction) with conditional integration, as in
      // Part I: freeze it while the command without it is saturated (a big step), so it only
      // accumulates the steady offsets it is meant for.
      const lim = 3 / ki;
      const pre = v3(
        aRef.x - g.kp * ep.x - g.kv * ev.x,
        aRef.y - g.kp * ep.y - g.kv * ev.y,
        aRef.z - g.kp * ep.z - g.kv * ev.z,
      );
      const satH = Math.hypot(pre.x, pre.z) > aH;
      const satV = pre.y > aUp || pre.y < -0.8 * GRAVITY;
      const next = v3(
        satH ? this.integral.x : this.integral.x + ep.x * dt,
        satV ? this.integral.y : this.integral.y + ep.y * dt,
        satH ? this.integral.z : this.integral.z + ep.z * dt,
      );
      this.integral = v3(
        clamp(next.x, -lim, lim),
        clamp(next.y, -lim, lim),
        clamp(next.z, -lim, lim),
      );
    } else {
      this.integral = v3();
    }
    const raw = v3(
      aRef.x - g.kp * ep.x - g.kv * ev.x - ki * this.integral.x,
      aRef.y - g.kp * ep.y - g.kv * ev.y - ki * this.integral.y,
      aRef.z - g.kp * ep.z - g.kv * ev.z - ki * this.integral.z,
    );
    const h = clampLength(v3(raw.x, 0, raw.z), aH);
    const out = v3(h.x, clamp(raw.y, -0.8 * GRAVITY, aUp), h.z);
    for (const k of ['x', 'y', 'z'] as const) {
      const parts: LoopTerms['parts'] = [
        { key: 'kp', label: '−Kp·e_p', value: -g.kp * ep[k], like: 'p' },
        { key: 'kv', label: '−Kv·e_v', value: -g.kv * ev[k], like: 'd' },
        { key: 'ki', label: '−Ki·∫e_p', value: -ki * this.integral[k], like: 'i' },
        { key: 'aref', label: 'a_ref', value: aRef[k], ff: true, like: 'ff' },
      ];
      this.last[k] = {
        setpoint: sp.pos[k],
        measurement: s.pos[k],
        error: sp.pos[k] - s.pos[k],
        parts,
        unsaturated: raw[k],
        output: out[k],
        saturated: Math.abs(raw[k] - out[k]) > 1e-9,
      };
    }
    return out;
  }

  loops(): Record<string, LoopTerms> {
    return { 'pos.x': this.last.x, 'pos.y': this.last.y, 'pos.z': this.last.z };
  }
}

const emptyLoop = (): LoopTerms => ({
  setpoint: 0,
  measurement: 0,
  error: 0,
  parts: [
    { key: 'kp', label: '−Kp·e_p', value: 0, like: 'p' },
    { key: 'kv', label: '−Kv·e_v', value: 0, like: 'd' },
    { key: 'ki', label: '−Ki·∫e_p', value: 0, like: 'i' },
    { key: 'aref', label: 'a_ref', value: 0, ff: true, like: 'ff' },
  ],
  unsaturated: 0,
  output: 0,
  saturated: false,
});

const wrapDeg = (a: number) => ((((a + 180) % 360) + 360) % 360) - 180;

/**
 * Attitude error → body-rate setpoint (deg/s: x roll, y yaw, z pitch), before limiting.
 * `err` is the error the inspector shows, in degrees, per body axis.
 *
 * - quaternion: ω = K·2·q_e,xyz (Part I; roll/pitch and yaw gains applied to one combined error).
 * - tilt: the error is split into a tilt part (rotate the thrust axis where it must point) and a
 *   yaw part about it, each with its own gain (Brescianini & D'Andrea 2020). Large yaw errors no
 *   longer disturb the tilt, which matters most for keeping the thrust where it is needed.
 * - euler: the naive textbook approach — subtract roll, pitch and yaw angles and command them as if
 *   they were body rates. Fine for small angles, wrong for large ones.
 */
export function attitudeRates(
  law: AttitudeLaw,
  q: Quat,
  qSp: Quat,
  kRP: number,
  kYaw: number,
): { raw: Vec3; err: Vec3 } {
  if (law === 'euler') {
    const now = qToEuler(q);
    const sp = qToEuler(qSp);
    const err = v3(
      wrapDeg((sp.roll - now.roll) * DEG),
      wrapDeg((sp.yaw - now.yaw) * DEG),
      wrapDeg((sp.pitch - now.pitch) * DEG),
    );
    return { raw: v3(kRP * err.x, kYaw * err.y, kRP * err.z), err };
  }
  let e = qMul(qConj(q), qSp);
  if (e.w < 0) e = { w: -e.w, x: -e.x, y: -e.y, z: -e.z };
  if (law === 'quaternion') {
    const err = v3(2 * e.x * DEG, 2 * e.y * DEG, 2 * e.z * DEG); // small-angle error, body axes
    return { raw: v3(kRP * err.x, kYaw * err.y, kRP * err.z), err };
  }
  // Tilt-prioritised: where must the body up-axis go (in body coordinates)?
  const upTarget = qRotate(e, up);
  let red = qFromTo(up, upTarget); // pure tilt, rotation axis ⊥ up
  if (red.w < 0) red = { w: -red.w, x: -red.x, y: -red.y, z: -red.z };
  let yaw = qMul(qConj(red), e); // what remains: rotation about up
  if (yaw.w < 0) yaw = { w: -yaw.w, x: -yaw.x, y: -yaw.y, z: -yaw.z };
  // Exact angles (not 2·sin(θ/2)) so a 170° tilt error really is 170°.
  const tiltAngle = 2 * Math.atan2(Math.hypot(red.x, red.z), red.w);
  const tiltAxis =
    Math.hypot(red.x, red.z) > 1e-12
      ? scale(v3(red.x, 0, red.z), 1 / Math.hypot(red.x, red.z))
      : v3();
  const yawAngle = 2 * Math.atan2(yaw.y, yaw.w);
  const err = v3(tiltAxis.x * tiltAngle * DEG, yawAngle * DEG, tiltAxis.z * tiltAngle * DEG);
  return { raw: v3(kRP * err.x, kYaw * err.y, kRP * err.z), err };
}

/**
 * Differential-flatness feedforward of the body rates: the thrust axis b = F/|F| must turn as the
 * desired force changes, ḃ = (m̂/T)·(j − (j·b)·b), so ω = b × ḃ (world frame), expressed in the
 * body frame, deg/s (Mellinger & Kumar 2011). Zero without a jerk reference.
 */
export function flatnessRates(fDes: Vec3, jerk: Vec3 | undefined, mHat: number, q: Quat): Vec3 {
  const T = length(fDes);
  if (!jerk || T < 1e-6) return v3();
  const b = scale(fDes, 1 / T);
  const jPerp = sub(jerk, scale(b, dot(jerk, b)));
  const bDot = scale(jPerp, mHat / T);
  const wWorld = cross(b, bDot);
  const wBody = qRotate(qConj(q), wWorld);
  return scale(wBody, DEG);
}
