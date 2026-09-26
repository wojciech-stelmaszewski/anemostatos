import { add, maxAbs, mul, mulAll, solve, sub, transpose, type Mat } from './mat';

export interface DareResult {
  /** Solution of the discrete algebraic Riccati equation. */
  p: Mat;
  /** Optimal state-feedback gain, u = −K·x. */
  k: Mat;
  iterations: number;
  converged: boolean;
}

/**
 * Discrete-time LQR: minimise Σ xᵀQx + uᵀRu subject to x⁺ = A·x + B·u, by iterating the Riccati
 * recursion P ← Q + AᵀPA − AᵀPB (R + BᵀPB)⁻¹ BᵀPA until it stops changing.
 * Simple and robust for the small, stabilisable systems of this project.
 */
export function dlqr(a: Mat, b: Mat, q: Mat, r: Mat, maxIter = 10000, tol = 1e-10): DareResult {
  const at = transpose(a);
  const bt = transpose(b);
  let p = q;
  let k: Mat = [];
  for (let it = 1; it <= maxIter; it++) {
    const btp = mul(bt, p);
    k = solve(add(r, mul(btp, b)), mul(btp, a));
    const next = add(q, sub(mulAll(at, p, a), mulAll(at, p, b, k)));
    const delta = maxAbs(sub(next, p));
    p = next;
    if (delta <= tol * Math.max(1, maxAbs(p))) {
      const btp2 = mul(bt, p);
      k = solve(add(r, mul(btp2, b)), mul(btp2, a));
      return { p, k, iterations: it, converged: true };
    }
  }
  return { p, k, iterations: maxIter, converged: false };
}
