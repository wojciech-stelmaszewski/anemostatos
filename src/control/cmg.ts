import { cmgJacobian, det3, gram, solve3, type CmgParams } from '@/sim/vehicles/cmg';

/** The dither of the generalised SR inverse turns at this rate, rad/s. */
const DITHER_RATE = Math.PI / 2;

/**
 * Gimbal rates for a wanted rate of change of the cluster's momentum `hdot` (lesson IV.23):
 * δ̇ = Aᵀ·(AAᵀ + λE)⁻¹·ḣ [Wie 2008, §7.3]. The pseudoinverse (λ = 0) gives exactly the momentum
 * asked for, and rates that grow without bound near a singularity. The singularity-robust inverse
 * (E = I, λ = λ₀·e^(−10m)) keeps the rates finite there by giving a little less than asked; it can
 * still stall where the command points exactly along the lost direction. The generalised SR
 * inverse puts a slowly turning dither off the diagonal of E: a small deliberate torque error that
 * walks the gimbals out of the singular set. `t` is the dither's clock, s.
 */
export function steerCmg(
  gimbal: readonly number[],
  hdot: readonly number[],
  c: CmgParams,
  t: number,
): { rates: number[]; measure: number } {
  const A = cmgJacobian(gimbal, c);
  const G = gram(A);
  const measure = det3(G) / c.h0 ** 6;
  let M = G;
  if (c.steering !== 'pinv') {
    const lam = c.lambda * c.h0 * c.h0 * Math.exp(-10 * measure);
    const e =
      c.steering === 'gsr'
        ? [0, 1, 2].map((i) => c.dither * Math.sin(DITHER_RATE * t + (i * Math.PI) / 2))
        : [0, 0, 0];
    const E = [
      [1, e[2]!, e[1]!],
      [e[2]!, 1, e[0]!],
      [e[1]!, e[0]!, 1],
    ];
    M = G.map((row, i) => row.map((v, j) => v + lam * E[i]![j]!));
  }
  const y = solve3(M, [...hdot]);
  const rates = [0, 1, 2, 3].map((n) => A.reduce((s, row, i) => s + row[n]! * y[i]!, 0));
  // At an exact singularity the pseudoinverse has no answer: ask for the largest rates instead.
  return { rates: rates.map((r) => (Number.isFinite(r) ? r : 0)), measure };
}
