import { solve } from './mat';

/** A piecewise polynomial in one dimension: segment i has coefficients c[i] (ascending powers). */
export interface Spline {
  times: number[];
  coeffs: number[][];
}

const fact = (n: number): number => (n <= 1 ? 1 : n * fact(n - 1));
/** d-th derivative factor of τ^k: k!/(k−d)! (0 if k < d). */
const dfac = (k: number, d: number): number => (k < d ? 0 : fact(k) / fact(k - d));

/**
 * Minimum-snap trajectory through waypoints with given segment durations (Mellinger & Kumar 2011),
 * starting and ending at rest (zero velocity, acceleration and jerk).
 *
 * The minimiser of ∫(x⁽⁴⁾)² dt is a piecewise 7th-order polynomial that is C⁶ at the interior
 * waypoints. With these boundary conditions the 8N constraints determine the 8N coefficients
 * exactly, so the optimisation reduces to one linear solve.
 */
export function minSnap(waypoints: readonly number[], times: readonly number[]): Spline {
  const n = times.length;
  if (waypoints.length !== n + 1) throw new Error('minSnap: need one more waypoint than segments');
  const size = 8 * n;
  const a: number[][] = [];
  const b: number[] = [];
  const row = () => new Array<number>(size).fill(0);
  // Value of derivative d of segment s at local time τ, as a row.
  const at = (s: number, d: number, tau: number) => {
    const r = row();
    for (let k = d; k < 8; k++) r[8 * s + k] = dfac(k, d) * tau ** (k - d);
    return r;
  };
  for (let s = 0; s < n; s++) {
    a.push(at(s, 0, 0));
    b.push(waypoints[s]!);
    a.push(at(s, 0, times[s]!));
    b.push(waypoints[s + 1]!);
  }
  for (let d = 1; d <= 3; d++) {
    a.push(at(0, d, 0));
    b.push(0);
    a.push(at(n - 1, d, times[n - 1]!));
    b.push(0);
  }
  for (let s = 0; s < n - 1; s++) {
    for (let d = 1; d <= 6; d++) {
      const r = at(s, d, times[s]!);
      const next = at(s + 1, d, 0);
      a.push(r.map((v, i) => v - next[i]!));
      b.push(0);
    }
  }
  const x = solve(
    a,
    b.map((v) => [v]),
  ).map((r) => r[0]!);
  return {
    times: [...times],
    coeffs: Array.from({ length: n }, (_, s) => x.slice(8 * s, 8 * s + 8)),
  };
}

/** Derivatives 0…4 of the spline at time t (clamped to its duration). */
export function evalSpline(sp: Spline, t: number): [number, number, number, number, number] {
  let s = 0;
  let tau = Math.max(t, 0);
  while (s < sp.times.length - 1 && tau > sp.times[s]!) tau -= sp.times[s++]!;
  tau = Math.min(tau, sp.times[s]!);
  const c = sp.coeffs[s]!;
  const out: [number, number, number, number, number] = [0, 0, 0, 0, 0];
  for (let d = 0; d <= 4; d++) {
    let v = 0;
    for (let k = d; k < 8; k++) v += c[k]! * dfac(k, d) * tau ** (k - d);
    out[d] = v;
  }
  return out;
}
