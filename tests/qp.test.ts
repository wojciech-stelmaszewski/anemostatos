import { describe, expect, it } from 'vitest';
import { luFactor, luSolve } from '@/math/mat';
import { QpSolver } from '@/math/qp';

describe('LU factorisation', () => {
  it('solves repeatedly with one factorisation', () => {
    const f = luFactor([
      [0, 2, 1],
      [1, 1, 0],
      [3, 0, 1],
    ]);
    expect(luSolve(f, [5, 3, 4]).map((v) => +v.toFixed(9))).toEqual([1, 2, 1]);
    expect(luSolve(f, [0, 1, 3]).map((v) => +v.toFixed(9) + 0)).toEqual([1, 0, 0]);
  });
});

describe('QP (ADMM)', () => {
  const tight = () => {
    const s = new QpSolver();
    s.epsAbs = s.epsRel = 1e-7;
    s.maxIter = 4000;
    return s;
  };

  it('unconstrained optimum when the constraints are inactive', () => {
    // min ½(x² + y²) − x − 2y  →  (1, 2)
    const r = tight().solve({
      p: [
        [1, 0],
        [0, 1],
      ],
      q: [-1, -2],
      a: [[1, 1]],
      l: [-Infinity],
      u: [10],
    });
    expect(r.converged).toBe(true);
    expect(r.x[0]).toBeCloseTo(1, 4);
    expect(r.x[1]).toBeCloseTo(2, 4);
  });

  it('an active linear inequality', () => {
    // min ½(x² + y²) − x − y  s.t. x + y ≤ 1  →  (½, ½)
    const r = tight().solve({
      p: [
        [1, 0],
        [0, 1],
      ],
      q: [-1, -1],
      a: [[1, 1]],
      l: [-Infinity],
      u: [1],
    });
    expect(r.x[0]).toBeCloseTo(0.5, 4);
    expect(r.x[1]).toBeCloseTo(0.5, 4);
  });

  it('boxes and equalities', () => {
    // min (x − 3)² + (y + 1)²  s.t. 0 ≤ x ≤ 1.2, x − y = 1: unconstrained along the line x = 1.5,
    // so the box is active: x = 1.2, y = 0.2
    const r = tight().solve({
      p: [
        [2, 0],
        [0, 2],
      ],
      q: [-6, 2],
      a: [
        [1, 0],
        [1, -1],
      ],
      l: [0, 1],
      u: [1.2, 1],
    });
    expect(r.x[0]).toBeCloseTo(1.2, 4);
    expect(r.x[1]).toBeCloseTo(0.2, 4);
  });

  it('warm starts: a repeated solve converges in fewer iterations', () => {
    const s = tight();
    const pr = {
      p: [
        [4, 1],
        [1, 2],
      ],
      q: [1, 1],
      a: [
        [1, 1],
        [1, 0],
        [0, 1],
      ],
      l: [1, 0, 0],
      u: [1, 0.7, 0.7],
    };
    const first = s.solve(pr);
    const second = s.solve(pr);
    expect(first.x[0]).toBeCloseTo(0.3, 3);
    expect(first.x[1]).toBeCloseTo(0.7, 3);
    expect(second.iterations).toBeLessThan(first.iterations);
  });
});
