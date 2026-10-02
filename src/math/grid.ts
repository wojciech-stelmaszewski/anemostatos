/**
 * Value iteration on a regular 2D grid with bilinear interpolation (docs/aerospace-gnc.md §3.2).
 *
 * For a discrete-time system x⁺ = f(x, u) with stage cost ℓ(x, u) and a finite set of inputs,
 * Bellman's equation V(x) = min_u [ℓ(x, u) + V(f(x, u))] is solved by iterating it over the grid
 * until it stops changing. Off-grid successors are read by bilinear interpolation; successors
 * outside the grid are clamped to its edge.
 */
export interface Grid2 {
  x0: number;
  x1: number;
  nx: number;
  y0: number;
  y1: number;
  ny: number;
  /** Values, row-major: v[j·nx + i] at (x_i, y_j). */
  v: Float64Array;
}

export const gridX = (g: Grid2, i: number) => g.x0 + ((g.x1 - g.x0) * i) / (g.nx - 1);
export const gridY = (g: Grid2, j: number) => g.y0 + ((g.y1 - g.y0) * j) / (g.ny - 1);

/** Bilinear interpolation of the grid at (x, y), clamped to the grid. */
export function sample(g: Grid2, x: number, y: number): number {
  const fx = Math.min(Math.max(((x - g.x0) / (g.x1 - g.x0)) * (g.nx - 1), 0), g.nx - 1);
  const fy = Math.min(Math.max(((y - g.y0) / (g.y1 - g.y0)) * (g.ny - 1), 0), g.ny - 1);
  const i = Math.min(Math.floor(fx), g.nx - 2);
  const j = Math.min(Math.floor(fy), g.ny - 2);
  const a = fx - i;
  const b = fy - j;
  const v = g.v;
  const n = g.nx;
  return (
    (1 - a) * (1 - b) * v[j * n + i]! +
    a * (1 - b) * v[j * n + i + 1]! +
    (1 - a) * b * v[(j + 1) * n + i]! +
    a * b * v[(j + 1) * n + i + 1]!
  );
}

export interface ValueProblem {
  /** One step of the dynamics from (x, y) with input u. */
  step: (x: number, y: number, u: number) => [number, number];
  /** Stage cost of one step. */
  cost: (x: number, y: number, u: number) => number;
  /** The candidate inputs. */
  inputs: readonly number[];
  /** Discount per step (1 = none). */
  discount?: number;
}

/**
 * Iterate Bellman's equation from V = 0 until the largest change is below `tol` (relative to the
 * largest value) or `maxIter` sweeps have run. Returns the grid and the number of sweeps.
 */
export function valueIteration(
  box: Omit<Grid2, 'v'>,
  pr: ValueProblem,
  maxIter = 2000,
  tol = 1e-7,
): { grid: Grid2; iterations: number } {
  const n = box.nx * box.ny;
  const grid: Grid2 = { ...box, v: new Float64Array(n) };
  // Successors and costs do not change between sweeps: precompute them.
  const m = pr.inputs.length;
  const nx = new Float64Array(n * m);
  const ny = new Float64Array(n * m);
  const cs = new Float64Array(n * m);
  for (let j = 0; j < box.ny; j++)
    for (let i = 0; i < box.nx; i++) {
      const x = gridX(grid, i);
      const y = gridY(grid, j);
      for (let k = 0; k < m; k++) {
        const u = pr.inputs[k]!;
        const [a, b] = pr.step(x, y, u);
        const idx = (j * box.nx + i) * m + k;
        nx[idx] = a;
        ny[idx] = b;
        cs[idx] = pr.cost(x, y, u);
      }
    }
  const gamma = pr.discount ?? 1;
  const next = new Float64Array(n);
  let it = 0;
  for (; it < maxIter; it++) {
    let delta = 0;
    let top = 0;
    for (let c = 0; c < n; c++) {
      let best = Infinity;
      for (let k = 0; k < m; k++) {
        const idx = c * m + k;
        const q = cs[idx]! + gamma * sample(grid, nx[idx]!, ny[idx]!);
        if (q < best) best = q;
      }
      next[c] = best;
      delta = Math.max(delta, Math.abs(best - grid.v[c]!));
      top = Math.max(top, Math.abs(best));
    }
    grid.v.set(next);
    if (delta <= tol * Math.max(top, 1e-12)) {
      it++;
      break;
    }
  }
  return { grid, iterations: it };
}

/** The input that minimises one Bellman step from (x, y). */
export function bestInput(g: Grid2, pr: ValueProblem, x: number, y: number): number {
  let best = Infinity;
  let arg = pr.inputs[0]!;
  const gamma = pr.discount ?? 1;
  for (const u of pr.inputs) {
    const [a, b] = pr.step(x, y, u);
    const q = pr.cost(x, y, u) + gamma * sample(g, a, b);
    if (q < best) {
      best = q;
      arg = u;
    }
  }
  return arg;
}
