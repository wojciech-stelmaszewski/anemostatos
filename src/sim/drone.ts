import { v3, type Vec3 } from '@/math/vec3';

/** Distance from the centre of mass to each motor, metres. */
export const ARM_LENGTH = 0.17;
const D = ARM_LENGTH / Math.SQRT2;

/**
 * Motor positions in the body frame (+x forward, +y up, +z right), X configuration:
 * M1 front-left, M2 front-right, M3 rear-right, M4 rear-left.
 */
export const MOTOR_POSITIONS: readonly Vec3[] = [
  v3(D, 0, -D),
  v3(D, 0, D),
  v3(-D, 0, D),
  v3(-D, 0, -D),
];

/** Propeller spin direction about +y: M1/M3 clockwise seen from above (−1), M2/M4 CCW (+1). */
export const MOTOR_SPIN = [-1, 1, -1, 1] as const;

/** Height of the centre of mass above the floor when the drone stands on its legs. */
export const FOOT_HEIGHT = 0.052;
