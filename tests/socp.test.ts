import { describe, expect, it } from 'vitest';
import { ConeBuilder, solveCone } from '@/math/socp';

describe('cone programs (barrier method)', () => {
  it('a linear program: the optimum sits on a vertex', () => {
    // min −x − y  s.t.  x + 2y ≤ 4, 3x + y ≤ 6, x, y ≥ 0  →  (1.6, 1.2), −2.8.
    const b = new ConeBuilder(2);
    b.le([1, 2], 4);
    b.le([3, 1], 6);
    b.le([-1, 0], 0);
    b.le([0, -1], 0);
    const r = solveCone(b.build([-1, -1]));
    expect(r.status).toBe('optimal');
    expect(r.x[0]).toBeCloseTo(1.6, 5);
    expect(r.x[1]).toBeCloseTo(1.2, 5);
    expect(r.objective).toBeCloseTo(-2.8, 5);
  });

  it('a second-order cone: the lowest point of a disk in a direction', () => {
    // min x + y  s.t.  ‖(x, y)‖ ≤ 1  →  −(1, 1)/√2.
    const b = new ConeBuilder(2);
    b.cone([
      { g: [0, 0], h: 1 },
      { g: [-1, 0], h: 0 },
      { g: [0, -1], h: 0 },
    ]);
    const r = solveCone(b.build([1, 1]));
    expect(r.status).toBe('optimal');
    expect(r.x[0]).toBeCloseTo(-Math.SQRT1_2, 5);
    expect(r.x[1]).toBeCloseTo(-Math.SQRT1_2, 5);
  });

  it('a quadratic objective with an equality constraint', () => {
    // min ½(x² + y²)  s.t.  x + y = 1, x ≤ 10  →  (0.5, 0.5).
    const b = new ConeBuilder(2);
    b.equal([1, 1], 1);
    b.le([1, 0], 10);
    const r = solveCone(
      b.build(
        [0, 0],
        [
          [1, 0],
          [0, 1],
        ],
      ),
    );
    expect(r.status).toBe('optimal');
    expect(r.x[0]).toBeCloseTo(0.5, 6);
    expect(r.x[1]).toBeCloseTo(0.5, 6);
  });

  it('a quadratic objective held back by an active bound', () => {
    // min (x − 2)²  s.t.  x ≤ 1  →  1.
    const b = new ConeBuilder(1);
    b.le([1], 1);
    const r = solveCone(b.build([-4], [[2]]));
    expect(r.status).toBe('optimal');
    expect(r.x[0]).toBeCloseTo(1, 5);
  });

  it('a parabola as a cone: y ≥ x² is ‖(2x, y − 1)‖ ≤ y + 1', () => {
    // min y  s.t.  x = 3, y ≥ x²  →  y = 9.
    const b = new ConeBuilder(2);
    b.equal([1, 0], 3);
    b.cone([
      { g: [0, -1], h: 1 },
      { g: [-2, 0], h: 0 },
      { g: [0, -1], h: -1 },
    ]);
    const r = solveCone(b.build([0, 1]));
    expect(r.status).toBe('optimal');
    expect(r.x[1]).toBeCloseTo(9, 4);
  });

  it('reports an infeasible problem', () => {
    // x ≤ −1 and x ≥ 1.
    const b = new ConeBuilder(1);
    b.le([1], -1);
    b.le([-1], -1);
    expect(solveCone(b.build([0])).status).toBe('infeasible');
  });

  it('reports a cone and a plane that do not meet', () => {
    // ‖(x, y)‖ ≤ 1 and x + y = 3.
    const b = new ConeBuilder(2);
    b.equal([1, 1], 3);
    b.cone([
      { g: [0, 0], h: 1 },
      { g: [-1, 0], h: 0 },
      { g: [0, -1], h: 0 },
    ]);
    expect(solveCone(b.build([0, 0])).status).toBe('infeasible');
  });
});
