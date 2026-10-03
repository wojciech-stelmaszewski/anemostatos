/**
 * Kalman filter and Rauch–Tung–Striebel smoother on a recorded track (docs/aerospace-gnc.md §4,
 * Chapter O, lesson IV.29) [Rauch 1965; Simon 2006].
 *
 * One axis, a constant-velocity model driven by white acceleration of spectral density q², and a
 * position measured with noise σ at each recorded sample. The filter runs forward and knows only
 * the past; the smoother then runs backward over the filter's results and adds what the future
 * says about each instant:
 *   x̂ₛ(k) = x̂(k|k) + C(k)·(x̂ₛ(k+1) − x̂(k+1|k)),   C(k) = P(k|k)·Fᵀ·P(k+1|k)⁻¹
 *   Pₛ(k) = P(k|k) + C(k)·(Pₛ(k+1) − P(k+1|k))·C(k)ᵀ
 * Pₛ ≤ P(k|k) at every sample, with equality at the last one, where there is no future to add.
 */
export interface Track {
  /** Position estimates and their 1σ, per sample. */
  filter: number[];
  filterSigma: number[];
  smooth: number[];
  smoothSigma: number[];
}

type M2 = [number, number, number, number]; // a b / c d, row-major

const mul = (a: M2, b: M2): M2 => [
  a[0] * b[0] + a[1] * b[2],
  a[0] * b[1] + a[1] * b[3],
  a[2] * b[0] + a[3] * b[2],
  a[2] * b[1] + a[3] * b[3],
];
const tr = (a: M2): M2 => [a[0], a[2], a[1], a[3]];
const add = (a: M2, b: M2): M2 => [a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]];
const sub = (a: M2, b: M2): M2 => [a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3]];
const inv = (a: M2): M2 => {
  const d = a[0] * a[3] - a[1] * a[2];
  return [a[3] / d, -a[1] / d, -a[2] / d, a[0] / d];
};

/**
 * Filter and smooth the measurements `z` taken at times `t` (s). Non-finite measurements are
 * skipped (prediction only). `sigma` is the measurement noise, m; `accel` the process noise, m/s²
 * per √Hz: small trusts the straight line, large follows every measurement.
 */
export function smoothTrack(t: number[], z: number[], sigma: number, accel: number): Track {
  const n = t.length;
  const r = sigma * sigma;
  const q2 = accel * accel;
  const xf: [number, number][] = [];
  const pf: M2[] = [];
  const xp: [number, number][] = [];
  const pp: M2[] = [];
  const fs: M2[] = [];
  let x: [number, number] = [Number.isFinite(z[0]) ? z[0]! : 0, 0];
  let p: M2 = [r, 0, 0, 100];
  for (let k = 0; k < n; k++) {
    const dt = k === 0 ? 0 : t[k]! - t[k - 1]!;
    const f: M2 = [1, dt, 0, 1];
    // Predict.
    x = [x[0] + dt * x[1], x[1]];
    const q: M2 = [(q2 * dt ** 3) / 3, (q2 * dt ** 2) / 2, (q2 * dt ** 2) / 2, q2 * dt];
    p = k === 0 ? p : add(mul(mul(f, p), tr(f)), q);
    fs.push(f);
    xp.push([...x]);
    pp.push(p);
    // Correct with the position.
    const zk = z[k]!;
    if (Number.isFinite(zk)) {
      const s = p[0] + r;
      const k0 = p[0] / s;
      const k1 = p[2] / s;
      const nu = zk - x[0];
      x = [x[0] + k0 * nu, x[1] + k1 * nu];
      p = [p[0] - k0 * p[0], p[1] - k0 * p[1], p[2] - k1 * p[0], p[3] - k1 * p[1]];
    }
    xf.push([...x]);
    pf.push(p);
  }
  // Backward pass.
  const xs: [number, number][] = new Array(n);
  const ps: M2[] = new Array(n);
  xs[n - 1] = xf[n - 1]!;
  ps[n - 1] = pf[n - 1]!;
  for (let k = n - 2; k >= 0; k--) {
    const c = mul(mul(pf[k]!, tr(fs[k + 1]!)), inv(pp[k + 1]!));
    const dx: [number, number] = [xs[k + 1]![0] - xp[k + 1]![0], xs[k + 1]![1] - xp[k + 1]![1]];
    xs[k] = [xf[k]![0] + c[0] * dx[0] + c[1] * dx[1], xf[k]![1] + c[2] * dx[0] + c[3] * dx[1]];
    ps[k] = add(pf[k]!, mul(mul(c, sub(ps[k + 1]!, pp[k + 1]!)), tr(c)));
  }
  return {
    filter: xf.map((v) => v[0]),
    filterSigma: pf.map((m) => Math.sqrt(Math.max(m[0], 0))),
    smooth: xs.map((v) => v[0]),
    smoothSigma: ps.map((m) => Math.sqrt(Math.max(m[0], 0))),
  };
}

/** Root-mean-square of `a − b` over the samples where both are finite. */
export function rmsError(a: number[], b: number[]): number {
  let s = 0;
  let n = 0;
  a.forEach((v, i) => {
    const d = v - b[i]!;
    if (Number.isFinite(d)) {
      s += d * d;
      n++;
    }
  });
  return n ? Math.sqrt(s / n) : NaN;
}

/**
 * The smoother's σ over the filter's in the middle of a long record, for a measurement every `dt`
 * seconds: what the future adds. In the limit of a slow model it is ½, the ratio of the
 * uncertainty of a fitted line in its middle to that at its end.
 */
export function middleSigmaRatio(sigma: number, accel: number, dt: number, samples = 4000): number {
  const t = Array.from({ length: samples }, (_, i) => i * dt);
  const tr = smoothTrack(t, new Array<number>(samples).fill(0), sigma, accel);
  const k = Math.floor(samples / 2);
  return tr.smoothSigma[k]! / tr.filterSigma[k]!;
}
