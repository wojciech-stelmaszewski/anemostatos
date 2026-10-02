import { describe, expect, it } from 'vitest';
import { solveNlp } from '@/math/nlp';

describe('nonlinear programs (augmented Lagrangian)', () => {
  it('an equality constraint: the closest point of a line', () => {
    // min (x − 1)² + (y − 2)²  s.t.  x + y = 1  →  (0, 1).
    const r = solveNlp(
      ([x, y]) => ({
        f: (x! - 1) ** 2 + (y! - 2) ** 2,
        grad: [2 * (x! - 1), 2 * (y! - 2)],
        ce: [x! + y! - 1],
        je: [[1, 1]],
        ci: [],
        ji: [],
      }),
      [3, 3],
    );
    expect(r.status).toBe('converged');
    expect(r.x[0]).toBeCloseTo(0, 4);
    expect(r.x[1]).toBeCloseTo(1, 4);
  });

  it('an active inequality: the closest point of a disk', () => {
    // min (x − 2)² + (y − 2)²  s.t.  1 − x² − y² ≥ 0  →  (1, 1)/√2.
    const r = solveNlp(
      ([x, y]) => ({
        f: (x! - 2) ** 2 + (y! - 2) ** 2,
        grad: [2 * (x! - 2), 2 * (y! - 2)],
        ce: [],
        je: [],
        ci: [1 - x! * x! - y! * y!],
        ji: [[-2 * x!, -2 * y!]],
      }),
      [0, 0],
    );
    expect(r.status).toBe('converged');
    expect(r.x[0]).toBeCloseTo(Math.SQRT1_2, 4);
    expect(r.x[1]).toBeCloseTo(Math.SQRT1_2, 4);
  });

  it('a nonconvex keep-out: the nearest point outside a circle', () => {
    // min x² + y²  s.t.  (x − 1)² + y² ≥ 4  →  (−1, 0) from a start on that side.
    const r = solveNlp(
      ([x, y]) => ({
        f: x! * x! + y! * y!,
        grad: [2 * x!, 2 * y!],
        ce: [],
        je: [],
        ci: [(x! - 1) ** 2 + y! * y! - 4],
        ji: [[2 * (x! - 1), 2 * y!]],
      }),
      [-0.5, 0.2],
    );
    expect(r.status).toBe('converged');
    expect(r.x[0]).toBeCloseTo(-1, 3);
    expect(r.x[1]).toBeCloseTo(0, 3);
  });

  it('Hock–Schittkowski 71: f* = 17.0140', () => {
    // min x₁x₄(x₁ + x₂ + x₃) + x₃  s.t.  x₁x₂x₃x₄ ≥ 25, Σxᵢ² = 40, 1 ≤ xᵢ ≤ 5.
    const r = solveNlp(
      (v) => {
        const [a, b, c, d] = v as [number, number, number, number];
        const bounds = v.flatMap((xi) => [xi - 1, 5 - xi]);
        const jb = v.flatMap((_, i) => {
          const lo = [0, 0, 0, 0];
          const hi = [0, 0, 0, 0];
          lo[i] = 1;
          hi[i] = -1;
          return [lo, hi];
        });
        return {
          f: a * d * (a + b + c) + c,
          grad: [d * (2 * a + b + c), a * d, a * d + 1, a * (a + b + c)],
          ce: [a * a + b * b + c * c + d * d - 40],
          je: [[2 * a, 2 * b, 2 * c, 2 * d]],
          ci: [a * b * c * d - 25, ...bounds],
          ji: [[b * c * d, a * c * d, a * b * d, a * b * c], ...jb],
        };
      },
      [1, 5, 5, 1],
    );
    expect(r.status).toBe('converged');
    expect(r.f).toBeCloseTo(17.014, 2);
    expect(r.x[0]).toBeCloseTo(1, 2);
    expect(r.x[1]).toBeCloseTo(4.743, 2);
  });
});
