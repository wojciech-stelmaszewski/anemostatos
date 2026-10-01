import { add, eye, maxAbs, mul, scale, solve, sub, transpose, type Mat } from './mat';

/** Determinant by LU with partial pivoting (small matrices). */
function det(a: Mat): number {
  const n = a.length;
  const m = a.map((r) => [...r]);
  let d = 1;
  for (let k = 0; k < n; k++) {
    let piv = k;
    for (let i = k + 1; i < n; i++) if (Math.abs(m[i]![k]!) > Math.abs(m[piv]![k]!)) piv = i;
    if (m[piv]![k] === 0) return 0;
    if (piv !== k) {
      [m[k], m[piv]] = [m[piv]!, m[k]!];
      d = -d;
    }
    d *= m[k]![k]!;
    for (let i = k + 1; i < n; i++) {
      const f = m[i]![k]! / m[k]![k]!;
      for (let j = k; j < n; j++) m[i]![j] = m[i]![j]! - f * m[k]![j]!;
    }
  }
  return d;
}

/**
 * The stabilising solution of the continuous algebraic Riccati equation
 *   Aᵀ·X + X·A − X·R·X + Q = 0       (R = B·R⁻¹·Bᵀ, symmetric, given as one matrix)
 * by the matrix sign function of the Hamiltonian H = [[A, −R], [−Q, −Aᵀ]] [Byers 1987]:
 * W = sign(H) by Newton's iteration with determinant scaling, then X from
 * [W₁₂; W₂₂ + I]·X = −[W₁₁ + I; W₂₁], solved in the least-squares sense. Throws when H has
 * eigenvalues on the imaginary axis (no stabilising solution).
 */
export function care(a: Mat, r: Mat, q: Mat, maxIter = 100): Mat {
  const n = a.length;
  const h: Mat = [];
  for (let i = 0; i < n; i++) h.push([...a[i]!, ...r[i]!.map((v) => -v)]);
  const at = transpose(a);
  for (let i = 0; i < n; i++) h.push([...q[i]!.map((v) => -v), ...at[i]!.map((v) => -v)]);
  let w = h;
  for (let it = 0; it < maxIter; it++) {
    const wi = solve(w, eye(2 * n));
    const c = Math.abs(det(w)) ** (1 / (2 * n));
    const next = scale(add(scale(w, 1 / c), scale(wi, c)), 0.5);
    const delta = maxAbs(sub(next, w));
    w = next;
    if (delta < 1e-12 * Math.max(1, maxAbs(w))) break;
  }
  const blk = (r0: number, c0: number): Mat =>
    w.slice(r0, r0 + n).map((row) => row.slice(c0, c0 + n));
  const i = eye(n);
  const lhs = [...blk(0, n), ...add(blk(n, n), i)];
  const rhs = [...add(blk(0, 0), i), ...blk(n, 0)].map((row) => row.map((v) => -v));
  const lt = transpose(lhs);
  const x = solve(mul(lt, lhs), mul(lt, rhs));
  return scale(add(x, transpose(x)), 0.5);
}

/** Residual of the equation, for checks: Aᵀ·X + X·A − X·R·X + Q. */
export const careResidual = (a: Mat, r: Mat, q: Mat, x: Mat): Mat =>
  add(sub(add(mul(transpose(a), x), mul(x, a)), mul(mul(x, r), x)), q);
