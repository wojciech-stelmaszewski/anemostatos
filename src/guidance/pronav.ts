import { add, cross, dot, length, scale, sub, v3, type Vec3 } from '@/math/vec3';
import type { GuidanceParams } from '@/sim/params';

/**
 * Guidance laws for an interception (docs/aerospace-gnc.md §4, Chapter O, lesson IV.24).
 *
 * The drone is the pursuer, a point that weaves across the sky is the target. Both fly in the
 * horizontal plane at the drone's altitude. The guidance law turns the geometry into a commanded
 * acceleration, which the geometric controller of Part II flies as its feedforward.
 *
 *  - Pure pursuit points the velocity at the target: a = K·(V·û − v), û the line of sight.
 *  - Proportional navigation turns the velocity at N times the rate of the line of sight:
 *    a = N·V_c·(Ω × û), with Ω = (r × v_rel)/|r|² the rotation rate of the line of sight and
 *    V_c the closing speed. A speed-hold term along the line of sight keeps the pursuer's speed.
 */

/** The target's motion: a straight run at constant speed with a sinusoidal sideways acceleration. */
export function targetState(g: GuidanceParams, t: number): { pos: Vec3; vel: Vec3 } {
  const head = (g.targetHeadingDeg * Math.PI) / 180;
  const along = v3(Math.cos(head), 0, Math.sin(head));
  const side = v3(-Math.sin(head), 0, Math.cos(head));
  const w = (2 * Math.PI) / Math.max(g.weavePeriod, 0.1);
  const a = g.weaveAcc;
  // Sideways acceleration a·sin(ωt), centred: position −(a/ω²)·sin(ωt), velocity −(a/ω)·cos(ωt).
  const lat = -(a / (w * w)) * Math.sin(w * t);
  const latV = -(a / w) * Math.cos(w * t);
  const pos = add(
    add(v3(g.targetX, 0, g.targetZ), scale(along, g.targetSpeed * t)),
    scale(side, lat),
  );
  const vel = add(scale(along, g.targetSpeed), scale(side, latV));
  return { pos, vel };
}

export interface Geometry {
  /** Range, m; closing speed, m/s; rotation rate of the line of sight, rad/s. */
  range: number;
  closing: number;
  losRate: number;
}

export function geometry(p: Vec3, v: Vec3, tp: Vec3, tv: Vec3): Geometry {
  const r = sub(tp, p);
  const vr = sub(tv, v);
  const range = Math.hypot(r.x, r.z);
  const closing = range > 1e-9 ? -(r.x * vr.x + r.z * vr.z) / range : 0;
  const losRate = range > 1e-9 ? (r.z * vr.x - r.x * vr.z) / (range * range) : 0;
  return { range, closing, losRate };
}

/** Commanded horizontal acceleration of the pursuer, m/s². */
export function guidanceAccel(g: GuidanceParams, p: Vec3, v: Vec3, tp: Vec3, tv: Vec3): Vec3 {
  const r = v3(tp.x - p.x, 0, tp.z - p.z);
  const range = Math.max(length(r), 1e-6);
  const u = scale(r, 1 / range);
  const vh = v3(v.x, 0, v.z);
  if (g.law === 'pursuit') return scale(sub(scale(u, g.speed), vh), g.pursuitGain);
  // Proportional navigation in the plane, plus a speed hold along the line of sight.
  const vr = v3(tv.x - v.x, 0, tv.z - v.z);
  const omega = scale(cross(r, vr), 1 / (range * range)); // vertical: rotation of the LOS
  const closing = -dot(r, vr) / range;
  const lateral = scale(cross(omega, u), g.N * Math.max(closing, 0));
  const hold = scale(u, g.pursuitGain * (g.speed - dot(vh, u)));
  return add(lateral, hold);
}

/** The engagement: geometry, the closest approach so far, and whether it is over. */
export class Engagement {
  minRange = Infinity;
  over = false;
  started = false;
  last: Geometry = { range: NaN, closing: NaN, losRate: NaN };

  reset(): void {
    this.minRange = Infinity;
    this.over = false;
    this.started = false;
    this.last = { range: NaN, closing: NaN, losRate: NaN };
  }

  /** Update with the pursuer's state; returns the commanded acceleration, or null when over. */
  update(g: GuidanceParams, t: number, p: Vec3, v: Vec3): Vec3 | null {
    const tgt = targetState(g, t - g.start);
    const geo = geometry(p, v, v3(tgt.pos.x, p.y, tgt.pos.z), tgt.vel);
    this.last = geo;
    this.started = true;
    if (this.over) return null;
    this.minRange = Math.min(this.minRange, geo.range);
    // The engagement ends at the closest approach: the range starts to open again.
    if (geo.range < 0.05 || (geo.closing < 0 && geo.range < 3 && t - g.start > 1)) {
      this.over = true;
      return null;
    }
    return guidanceAccel(g, p, v, tgt.pos, tgt.vel);
  }
}
