import { describe, expect, it } from 'vitest';
import { Differentiator, LowPass1, LowPass2 } from '@/estimation/filters';
import { c2d, diag, eig2, expm, eye, inv, mul, solve, sub, maxAbs, type Mat } from '@/math/mat';
import { dlqr } from '@/math/riccati';

const close = (a: Mat, b: Mat, tol = 1e-9) => expect(maxAbs(sub(a, b))).toBeLessThan(tol);

describe('matrices', () => {
  it('solves a linear system and inverts', () => {
    const a = [
      [0, 2, 1],
      [1, 1, 0],
      [3, 0, 1],
    ];
    const x = solve(a, [[5], [3], [4]]);
    close(x, [[1], [2], [1]]);
    close(mul(a, inv(a)), eye(3));
  });

  it('throws on a singular matrix', () => {
    expect(() =>
      solve(
        [
          [1, 2],
          [2, 4],
        ],
        [[1], [2]],
      ),
    ).toThrow();
  });

  it('expm of a rotation generator is a rotation', () => {
    const t = 1.3;
    close(
      expm([
        [0, -t],
        [t, 0],
      ]),
      [
        [Math.cos(t), -Math.sin(t)],
        [Math.sin(t), Math.cos(t)],
      ],
      1e-12,
    );
  });

  it('expm handles large norms (scaling and squaring)', () => {
    close(expm(diag([5, -3])), diag([Math.exp(5), Math.exp(-3)]), 1e-8);
  });

  it('c2d of a double integrator is exact', () => {
    const dt = 0.1;
    const d = c2d(
      [
        [0, 1],
        [0, 0],
      ],
      [[0], [1]],
      dt,
    );
    close(d.a, [
      [1, dt],
      [0, 1],
    ]);
    close(d.b, [[(dt * dt) / 2], [dt]]);
  });

  it('eig2 gives complex poles of an oscillator', () => {
    const e = eig2([
      [0, 1],
      [-4, -0.4],
    ]);
    expect(e[0]!.re).toBeCloseTo(-0.2, 9);
    expect(Math.abs(e[0]!.im)).toBeCloseTo(Math.sqrt(4 - 0.04), 9);
  });
});

describe('LQR', () => {
  it('matches the continuous double-integrator solution for small dt', () => {
    // ẍ = u, J = ∫ q1·x² + q2·v² + r·u²  ⇒  K = [√(q1/r), √(q2/r + 2√(q1/r))].
    const [q1, q2, r] = [4, 1, 0.25];
    const dt = 0.001;
    const d = c2d(
      [
        [0, 1],
        [0, 0],
      ],
      [[0], [1]],
      dt,
    );
    const res = dlqr(d.a, d.b, diag([q1 * dt, q2 * dt]), [[r * dt]]);
    expect(res.converged).toBe(true);
    const k1 = Math.sqrt(q1 / r);
    const k2 = Math.sqrt(q2 / r + 2 * k1);
    expect(res.k[0]![0]!).toBeCloseTo(k1, 1);
    expect(res.k[0]![1]!).toBeCloseTo(k2, 1);
  });

  it('stabilises the closed loop', () => {
    const d = c2d(
      [
        [0, 1],
        [0, 0],
      ],
      [[0], [1]],
      0.02,
    );
    const { k } = dlqr(d.a, d.b, diag([10, 1]), [[0.1]]);
    const acl = sub(d.a, mul(d.b, k));
    for (const e of eig2(acl)) expect(Math.hypot(e.re, e.im)).toBeLessThan(1);
  });
});

describe('filters', () => {
  const sine = (hz: number, n: number, dt: number) =>
    Array.from({ length: n }, (_, k) => Math.sin(2 * Math.PI * hz * k * dt));
  const peak = (y: number[]) => Math.max(...y.slice(y.length / 2).map(Math.abs));

  it('LowPass2 passes DC and attenuates 10× the cutoff by ≥ 40 dB', () => {
    const dt = 0.001;
    const f = new LowPass2(20, dt);
    let y = 0;
    for (let k = 0; k < 2000; k++) y = f.update(3);
    expect(y).toBeCloseTo(3, 9);
    const g = new LowPass2(20, dt);
    const out = sine(200, 4000, dt).map((x) => g.update(x));
    expect(peak(out)).toBeLessThan(0.012);
    expect(peak(out)).toBeGreaterThan(0.005); // bilinear warping adds a little extra attenuation
  });

  it('LowPass2 is −3 dB at the cutoff', () => {
    const dt = 0.001;
    const f = new LowPass2(20, dt);
    const out = sine(20, 4000, dt).map((x) => f.update(x));
    expect(peak(out)).toBeCloseTo(Math.SQRT1_2, 2);
  });

  it('two identical filters stay in lock-step (INDI synchronisation)', () => {
    const a = new LowPass2(30, 0.001);
    const b = new LowPass2(30, 0.001);
    const x = sine(7, 500, 0.001);
    for (const v of x) expect(a.update(v)).toBe(b.update(v));
  });

  it('LowPass1 settles to its input and the differentiator finds the slope', () => {
    const f = new LowPass1(5, 0.01);
    let y = 0;
    for (let k = 0; k < 1000; k++) y = f.update(2);
    expect(y).toBeCloseTo(2, 9);
    const d = new Differentiator(0.01);
    d.update(0);
    expect(d.update(0.05)).toBeCloseTo(5, 9);
  });
});
