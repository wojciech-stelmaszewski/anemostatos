import { ARM_LENGTH } from '@/sim/drone';
import { clamp } from '@/math/util';

const D = ARM_LENGTH / Math.SQRT2;

export interface MixerResult {
  motors: [number, number, number, number];
  saturated: boolean;
  /** How much of the requested yaw / roll-pitch authority survived desaturation (0…1). */
  yawScale: number;
  rpScale: number;
}

/**
 * Control allocation for the X frame: f = A⁻¹·(T, τx, τy, τz) (docs/control-architecture.md §4.6),
 * with desaturation that trades collective thrust first, then yaw, then roll/pitch.
 */
export function mix(
  thrust: number,
  tx: number,
  ty: number,
  tz: number,
  torqueCoeff: number,
  fmax: number,
): MixerResult {
  const rp = [
    tx / (4 * D) + tz / (4 * D),
    -tx / (4 * D) + tz / (4 * D),
    -tx / (4 * D) - tz / (4 * D),
    tx / (4 * D) - tz / (4 * D),
  ];
  const k = ty / (4 * torqueCoeff);
  const yaw = [k, -k, k, -k];
  const range = (a: number[]) => Math.max(...a) - Math.min(...a);

  let yawScale = 1;
  let rpScale = 1;
  let att = rp.map((v, i) => v + yaw[i]!);
  if (range(att) > fmax) {
    // Not enough authority for everything: give up yaw first…
    const rpRange = range(rp);
    yawScale =
      rpRange >= fmax ? 0 : clamp((fmax - rpRange) / Math.max(range(att) - rpRange, 1e-9), 0, 1);
    att = rp.map((v, i) => v + yaw[i]! * yawScale);
    if (range(att) > fmax) {
      // …then scale roll/pitch down to fit.
      rpScale = fmax / range(att);
      att = att.map((v) => v * rpScale);
    }
  }
  // …and shift collective thrust so every motor stays within [0, fmax].
  const base = clamp(thrust / 4, -Math.min(...att), fmax - Math.max(...att));
  const motors = att.map((a) => clamp(base + a, 0, fmax)) as MixerResult['motors'];
  const saturated = yawScale < 1 || rpScale < 1 || Math.abs(base - thrust / 4) > 1e-9;
  return { motors, saturated, yawScale, rpScale };
}
