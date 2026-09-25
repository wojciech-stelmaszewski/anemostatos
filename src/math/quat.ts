import { v3, type Vec3 } from './vec3';

/** Unit quaternion (w, x, y, z) rotating body-frame vectors into the world frame. */
export interface Quat {
  w: number;
  x: number;
  y: number;
  z: number;
}

export const qIdentity = (): Quat => ({ w: 1, x: 0, y: 0, z: 0 });

export const qMul = (a: Quat, b: Quat): Quat => ({
  w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
  y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
  z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
});

export const qConj = (q: Quat): Quat => ({ w: q.w, x: -q.x, y: -q.y, z: -q.z });

export const qNormalize = (q: Quat): Quat => {
  const l = Math.hypot(q.w, q.x, q.y, q.z);
  return l > 0 ? { w: q.w / l, x: q.x / l, y: q.y / l, z: q.z / l } : qIdentity();
};

export const qFromAxisAngle = (axis: Vec3, angle: number): Quat => {
  const l = Math.hypot(axis.x, axis.y, axis.z) || 1;
  const s = Math.sin(angle / 2) / l;
  return { w: Math.cos(angle / 2), x: axis.x * s, y: axis.y * s, z: axis.z * s };
};

/** Rotate vector v by quaternion q (body → world). */
export const qRotate = (q: Quat, v: Vec3): Vec3 => {
  // t = 2 * cross(q.xyz, v); v' = v + w t + cross(q.xyz, t)
  const tx = 2 * (q.y * v.z - q.z * v.y);
  const ty = 2 * (q.z * v.x - q.x * v.z);
  const tz = 2 * (q.x * v.y - q.y * v.x);
  return v3(
    v.x + q.w * tx + (q.y * tz - q.z * ty),
    v.y + q.w * ty + (q.z * tx - q.x * tz),
    v.z + q.w * tz + (q.x * ty - q.y * tx),
  );
};

/** Integrate body angular velocity ω over dt: q ← q ⊗ exp(ω dt / 2). */
export const qIntegrate = (q: Quat, omega: Vec3, dt: number): Quat => {
  const angle = Math.hypot(omega.x, omega.y, omega.z) * dt;
  if (angle < 1e-12) return q;
  return qNormalize(qMul(q, qFromAxisAngle(omega, angle)));
};

/** Shortest rotation taking unit vector a onto unit vector b. */
export const qFromTo = (a: Vec3, b: Vec3): Quat => {
  const d = a.x * b.x + a.y * b.y + a.z * b.z;
  if (d < -0.999999) {
    // Opposite vectors: rotate 180° around any perpendicular axis.
    const axis = Math.abs(a.x) < 0.9 ? v3(0, -a.z, a.y) : v3(a.z, 0, -a.x);
    return qFromAxisAngle(axis, Math.PI);
  }
  return qNormalize({
    w: 1 + d,
    x: a.y * b.z - a.z * b.y,
    y: a.z * b.x - a.x * b.z,
    z: a.x * b.y - a.y * b.x,
  });
};

/**
 * Euler angles for display, in the body convention of this project
 * (+x forward, +y up, +z right): roll about x, pitch about z, yaw about y.
 * Applied as yaw → pitch → roll (intrinsic Y-Z-X).
 */
export const qToEuler = (q: Quat): { roll: number; pitch: number; yaw: number } => {
  const { w, x, y, z } = q;
  const sinPitch = Math.max(-1, Math.min(1, 2 * (w * z + x * y)));
  const pitch = Math.asin(sinPitch);
  const roll = Math.atan2(2 * (w * x - y * z), 1 - 2 * (x * x + z * z));
  const yaw = Math.atan2(2 * (w * y - x * z), 1 - 2 * (y * y + z * z));
  return { roll, pitch, yaw };
};

export const qFromEuler = (roll: number, pitch: number, yaw: number): Quat =>
  qMul(
    qMul(qFromAxisAngle(v3(0, 1, 0), yaw), qFromAxisAngle(v3(0, 0, 1), pitch)),
    qFromAxisAngle(v3(1, 0, 0), roll),
  );
