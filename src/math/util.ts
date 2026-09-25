export const clamp = (v: number, min: number, max: number): number =>
  v < min ? min : v > max ? max : v;

export const DEG = Math.PI / 180;

/** Smoothing factor of a first-order low-pass filter with the given cutoff, sampled every dt. */
export const lowPassAlpha = (cutoffHz: number, dt: number): number => {
  const tau = 1 / (2 * Math.PI * cutoffHz);
  return tau / (tau + dt);
};
