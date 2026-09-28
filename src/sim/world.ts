import type { Vec3 } from '@/math/vec3';
import type { Level, WorldParams } from './params';

/**
 * Signed distance from a point to the nearest keep-out zone boundary, m: positive outside,
 * negative inside. Infinity if there are no zones.
 */
export function zoneDistance(p: Vec3, w: WorldParams, level: Level): number {
  let d = Infinity;
  if (w.ceiling > 0) d = Math.min(d, w.ceiling - p.y);
  if (w.pillar && level > 1)
    d = Math.min(d, Math.hypot(p.x - w.pillarX, p.z - w.pillarZ) - w.pillarR);
  return d;
}
