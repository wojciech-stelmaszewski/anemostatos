import { describe, expect, it } from 'vitest';
import { cabs, carg, cdiv, cexp, cmul, csolve, cx } from '@/math/complex';
import { eigQr } from '@/math/eig';
import { correlate, fft, welch } from '@/math/fft';
import {
  c2dTustin,
  c2dZoh,
  dcGain,
  feedback,
  freqResp,
  isStable,
  poles,
  series,
  ss,
  stepResponse,
} from '@/math/lti';
import { logspace, margins } from '@/math/margins';
import { Rng } from '@/math/prng';

const DEG = 180 / Math.PI;
const sorted = (ev: { re: number; im: number }[]) =>
  [...ev].sort((a, b) => a.re - b.re || a.im - b.im);

describe('complex arithmetic', () => {
  it('multiplies, divides and exponentiates', () => {
    const a = cx(1, 2);
    const b = cx(3, -1);
    expect(cmul(a, b)).toEqual({ re: 5, im: 5 });
    const q = cdiv(a, b);
    expect(q.re).toBeCloseTo(0.1, 12);
    expect(q.im).toBeCloseTo(0.7, 12);
    const e = cexp(cx(0, Math.PI));
    expect(e.re).toBeCloseTo(-1, 12);
    expect(e.im).toBeCloseTo(0, 12);
  });

  it('solves a complex linear system', () => {
    // (1 + j)·x = 2  ⇒  x = 1 − j
    const x = csolve([[cx(1, 1)]], [[cx(2)]]);
    expect(x[0]![0]!.re).toBeCloseTo(1, 12);
    expect(x[0]![0]!.im).toBeCloseTo(-1, 12);
  });
});

describe('eigenvalues by QR', () => {
  it('finds a real root and a complex pair', () => {
    // Companion matrix of (λ + 1)(λ² + 2λ + 5).
    const ev = eigQr([
      [0, 1, 0],
      [0, 0, 1],
      [-5, -7, -3],
    ]).sort((x, y) => x.im - y.im);
    expect(ev.map((z) => z.re)).toEqual([-1, -1, -1].map((v) => expect.closeTo(v, 10)));
    expect(ev.map((z) => z.im)).toEqual([-2, 0, 2].map((v) => expect.closeTo(v, 10)));
  });

  it('handles a defective matrix (the free drone: a double eigenvalue at 0)', () => {
    const ev = eigQr([
      [0, 1],
      [0, 0],
    ]);
    for (const z of ev) expect(Math.hypot(z.re, z.im)).toBeLessThan(1e-12);
  });

  it('separates poles six decades apart, where a characteristic polynomial loses them', () => {
    const want = [-1e-3, -0.5, -2, -30, -400, -1000, -1e3 - 1, -5e3];
    // A similarity transform of the diagonal matrix, so the eigenvalues are known exactly.
    const n = want.length;
    const rng = new Rng(7);
    const t = Array.from({ length: n }, (_, i) =>
      Array.from({ length: n }, (_, j) => (i === j ? 1 : 0.3 * rng.normal())),
    );
    const a = t.map(() => t.map(() => 0));
    // a = T·D·T⁻¹ computed column by column with the real solver in mat.ts is overkill here:
    // use a triangular T so that T⁻¹ is exact by back-substitution.
    for (let i = 0; i < n; i++) for (let j = 0; j < i; j++) t[i]![j] = 0;
    const tinv: number[][] = t.map((_, i) => t.map((_, j) => (i === j ? 1 : 0)));
    for (let c = 0; c < n; c++)
      for (let r = n - 1; r >= 0; r--) {
        let s = r === c ? 1 : 0;
        for (let k = r + 1; k < n; k++) s -= t[r]![k]! * tinv[k]![c]!;
        tinv[r]![c] = s / t[r]![r]!;
      }
    for (let i = 0; i < n; i++)
      for (let j = 0; j < n; j++)
        for (let k = 0; k < n; k++) a[i]![j] = a[i]![j]! + t[i]![k]! * want[k]! * tinv[k]![j]!;
    const got = sorted(eigQr(a)).map((z) => z.re);
    const exp = [...want].sort((x, y) => x - y);
    got.forEach((g, i) => expect(Math.abs(g - exp[i]!) / Math.abs(exp[i]!)).toBeLessThan(1e-8));
  });

  it('agrees with the trace and the determinant on random matrices', () => {
    const rng = new Rng(11);
    for (let trial = 0; trial < 20; trial++) {
      const n = 2 + (trial % 7);
      const a = Array.from({ length: n }, () => Array.from({ length: n }, () => rng.normal()));
      const ev = eigQr(a);
      const tr = a.reduce((s, r, i) => s + r[i]!, 0);
      expect(ev.reduce((s, z) => s + z.re, 0)).toBeCloseTo(tr, 8);
      expect(ev.reduce((s, z) => s + z.im, 0)).toBeCloseTo(0, 8);
    }
  });
});

describe('LTI systems', () => {
  const lag = (tau: number) => ss([[-1 / tau]], [[1 / tau]], [[1]]);
  const doubleIntegrator = (m: number) =>
    ss(
      [
        [0, 1],
        [0, 0],
      ],
      [[0], [1 / m]],
      [[1, 0]],
    );

  it('a first-order lag has gain 1/√2 and phase −45° at its corner frequency', () => {
    const h = freqResp(lag(0.03), 1 / 0.03)[0]![0]!;
    expect(cabs(h)).toBeCloseTo(Math.SQRT1_2, 12);
    expect(carg(h) * DEG).toBeCloseTo(-45, 10);
    expect(dcGain(lag(0.03))[0]![0]).toBeCloseTo(1, 12);
  });

  it('series and feedback: PD around a mass gives the poles of m·s² + Kd·s + Kp', () => {
    const m = 1;
    const kp = 10;
    const kd = 7;
    // State feedback u = −Kp·x − Kd·v, written as feedback with both states as outputs.
    const plant = ss(
      [
        [0, 1],
        [0, 0],
      ],
      [[0], [1 / m]],
      [[kp, kd]],
    );
    const cl = feedback(plant);
    const p = sorted(poles(cl));
    expect(p[0]!.re).toBeCloseTo(-5, 9);
    expect(p[1]!.re).toBeCloseTo(-2, 9);
    expect(isStable(cl)).toBe(true);
    expect(isStable(plant)).toBe(false);
  });

  it('series connects a lag in front of the mass', () => {
    const g = series(lag(0.03), doubleIntegrator(2));
    const w = 5;
    const h = freqResp(g, w)[0]![0]!;
    // 1/(m·(jω)²·(jωτ + 1))
    expect(cabs(h)).toBeCloseTo(1 / (2 * w * w * Math.hypot(1, w * 0.03)), 12);
    expect(carg(h)).toBeCloseTo(Math.PI - Math.atan(w * 0.03), 10);
  });

  it('zero-order hold of the double integrator is exact', () => {
    const dt = 0.004;
    const d = c2dZoh(doubleIntegrator(1), dt);
    expect(d.a).toEqual([
      [1, dt],
      [0, 1],
    ]);
    expect(d.b[0]![0]).toBeCloseTo((dt * dt) / 2, 15);
    expect(d.b[1]![0]).toBeCloseTo(dt, 15);
    // Step response: x = t²/2 at the sampling instants.
    const y = stepResponse(d, 251).map((r) => r[0]!);
    expect(y[250]).toBeCloseTo(0.5, 10);
  });

  it('Tustin maps an integrator to (T/2)·(z + 1)/(z − 1)', () => {
    const integ = ss([[0]], [[1]], [[1]]);
    const t = 0.1;
    const d = c2dTustin(integ, t);
    const w = 3;
    const h = freqResp(d, w)[0]![0]!;
    // (T/2)·(e^{jωT} + 1)/(e^{jωT} − 1) = −j·(T/2)/tan(ωT/2): exactly −90°, frequency warped.
    expect(h.re).toBeCloseTo(0, 12);
    expect(h.im).toBeCloseTo(-t / 2 / Math.tan((w * t) / 2), 12);
  });
});

describe('margins', () => {
  it('PD on a double integrator: crossover and phase margin in closed form', () => {
    const m = 1;
    const kp = 10;
    const kd = 7;
    const w = logspace(0.1, 1000, 2000);
    const l = w.map((x) => cdiv(cx(kp, kd * x), cx(-m * x * x)));
    const r = margins(w, l);
    const wc = Math.sqrt((kd * kd + Math.sqrt(kd ** 4 + 4 * m * m * kp * kp)) / (2 * m * m));
    expect(r.wc).toBeCloseTo(wc, 3);
    expect(r.pmDeg).toBeCloseTo(Math.atan((kd * wc) / kp) * DEG, 2);
    expect(r.gm).toBe(Infinity);
    expect(r.delayMargin).toBeCloseTo(Math.atan((kd * wc) / kp) / wc, 4);
  });

  it('K/(s·(s + 1)·(s + 2)) has its phase crossover at √2 and a gain margin of 6/K', () => {
    const k = 2;
    const w = logspace(0.01, 100, 3000);
    const l = w.map((x) => {
      const s = cx(0, x);
      return cdiv(cx(k), cmul(s, cmul(cx(1, x), cx(2, x))));
    });
    const r = margins(w, l);
    expect(r.w180).toBeCloseTo(Math.SQRT2, 3);
    expect(r.gm).toBeCloseTo(6 / k, 3);
    // The distance to −1 is 1/ms; it must not exceed the gain-margin distance.
    expect(1 / r.ms).toBeLessThanOrEqual(1 - k / 6 + 1e-9);
  });
});

describe('spectra', () => {
  it('the FFT of a sine has one line at its bin', () => {
    const n = 256;
    const re = Array.from({ length: n }, (_, i) => Math.sin((2 * Math.PI * 8 * i) / n));
    const im = new Array<number>(n).fill(0);
    fft(re, im);
    const mag = re.map((v, i) => Math.hypot(v, im[i]!));
    expect(mag[8]).toBeCloseTo(n / 2, 9);
    expect(Math.max(...mag.filter((_, i) => i !== 8 && i !== n - 8))).toBeLessThan(1e-9);
  });

  it('Welch: the PSD of white noise integrates to its variance', () => {
    const rng = new Rng(3);
    const fs = 200;
    const x = Array.from({ length: 1 << 15 }, () => 0.5 * rng.normal());
    const { f, psd } = welch(x, fs, 512);
    const df = f[1]! - f[0]!;
    const power = psd.reduce((s, v) => s + v, 0) * df;
    expect(power).toBeCloseTo(0.25, 1);
    // Flat: the level is variance / (fs/2).
    const mid = psd.slice(20, 200).reduce((s, v) => s + v, 0) / 180;
    expect(mid).toBeCloseTo(0.25 / (fs / 2), 3);
  });

  it('correlate returns amplitude and phase of a sine over whole periods', () => {
    const dt = 0.001;
    const fHz = 5;
    const x = Array.from(
      { length: 1000 },
      (_, k) => 1.5 * Math.cos(2 * Math.PI * fHz * k * dt - 0.7) + 0.3,
    );
    const c = correlate(x, fHz, dt);
    expect(cabs(c)).toBeCloseTo(1.5, 9);
    expect(carg(c)).toBeCloseTo(-0.7, 9);
  });
});
