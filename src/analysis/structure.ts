// Structural properties of a linear system (docs/analysis.md §4, Chapter G): what its inputs can
// reach and what its outputs can reveal.
import { mul, rank, type Mat } from '@/math/mat';

/** The controllability matrix (B  A·B  …  Aⁿ⁻¹·B). */
export function controllabilityMatrix(a: Mat, b: Mat): Mat {
  const n = a.length;
  const out: Mat = Array.from({ length: n }, () => []);
  let block = b;
  for (let k = 0; k < n; k++) {
    for (let i = 0; i < n; i++) out[i]!.push(...block[i]!);
    block = mul(a, block);
  }
  return out;
}

/** The observability matrix: C, C·A, …, C·Aⁿ⁻¹ stacked. */
export function observabilityMatrix(a: Mat, c: Mat): Mat {
  const out: Mat = [];
  let block = c;
  for (let k = 0; k < a.length; k++) {
    out.push(...block.map((r) => [...r]));
    block = mul(block, a);
  }
  return out;
}

/** Number of state directions the input can move; the system is controllable if it equals n. */
export const controllabilityRank = (a: Mat, b: Mat): number => rank(controllabilityMatrix(a, b));

/** Number of state directions the output reveals; the system is observable if it equals n. */
export const observabilityRank = (a: Mat, c: Mat): number => rank(observabilityMatrix(a, c));

/**
 * The model behind the L1 altitude filter, in continuous time. States: altitude, velocity,
 * accelerometer bias and, optionally, altimeter bias. The accelerometer is the input; the
 * altimeter, which reads altitude plus its own bias, is the output.
 */
export function altitudeFilterModel(withAltimeterBias: boolean): { a: Mat; c: Mat } {
  return withAltimeterBias
    ? {
        a: [
          [0, 1, 0, 0],
          [0, 0, -1, 0],
          [0, 0, 0, 0],
          [0, 0, 0, 0],
        ],
        c: [[1, 0, 0, 1]],
      }
    : {
        a: [
          [0, 1, 0],
          [0, 0, -1],
          [0, 0, 0],
        ],
        c: [[1, 0, 0]],
      };
}
