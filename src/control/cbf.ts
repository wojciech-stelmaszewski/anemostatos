import { QpSolver } from '@/math/qp';
import { v3, type Vec3 } from '@/math/vec3';
import type { Level, Params } from '@/sim/params';
import type { Measurement } from '@/sim/sensors';

/**
 * Control-barrier-function safety filter (Ames et al. 2019) on the desired acceleration.
 * For each keep-out zone a barrier h(p) ≥ 0 describes the safe set; since the acceleration acts on
 * h only through its second derivative, the constraint is the exponential/high-order form
 *   ḧ + k₁·ḣ + k₀·h ≥ 0,   k₁ = 2γ, k₀ = γ²,
 * which is linear in a:  gᵀa ≥ b. The filter returns the acceleration closest to the requested
 * one that satisfies all of them (a tiny QP) — it only acts near the boundary.
 */
export function safetyFilter(
  aDes: Vec3,
  s: Measurement,
  p: Params,
  level: Level,
  solver: QpSolver,
): { a: Vec3; active: number } {
  const { gamma, margin } = p.control.cbf;
  const k1 = 2 * gamma;
  const k0 = gamma * gamma;
  const w = p.world;
  const rows: { g: [number, number, number]; b: number }[] = [];
  if (w.pillar && level > 1) {
    const dx = s.pos.x - w.pillarX;
    const dz = s.pos.z - w.pillarZ;
    const R = w.pillarR + margin;
    const h = dx * dx + dz * dz - R * R;
    const hDot = 2 * (dx * s.vel.x + dz * s.vel.z);
    // ḧ = 2|v_h|² + 2·d·a_h
    rows.push({
      g: [2 * dx, 0, 2 * dz],
      b: -2 * (s.vel.x ** 2 + s.vel.z ** 2) - k1 * hDot - k0 * h,
    });
  }
  if (w.ceiling > 0) {
    const h = w.ceiling - margin - s.pos.y;
    // ḣ = −v_y, ḧ = −a_y
    rows.push({ g: [0, -1, 0], b: k1 * s.vel.y - k0 * h });
  }
  const violated = rows.filter((r) => r.g[0] * aDes.x + r.g[1] * aDes.y + r.g[2] * aDes.z < r.b);
  if (!violated.length) return { a: aDes, active: 0 };
  // min ½|a − a_des|²  s.t.  gᵢᵀa ≥ bᵢ
  solver.maxIter = 400;
  const r = solver.solve(
    {
      p: [
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
      ],
      q: [-aDes.x, -aDes.y, -aDes.z],
      a: rows.map((r) => r.g),
      l: rows.map((r) => r.b),
      u: rows.map(() => Infinity),
    },
    {},
  );
  return { a: v3(r.x[0]!, r.x[1]!, r.x[2]!), active: violated.length };
}
