import type { Lesson } from './types';

/**
 * Part IV — Aerospace GNC (docs/aerospace-gnc.md §6): optimal control and guidance on the drone,
 * then a rocket, an aircraft and a satellite. Numbering restarts at IV.1 in the lesson panel;
 * every lesson carries its plan number `n`, since the lessons are built out of order.
 */

export const K = 'K · The best possible path';
export const L = 'L · A rocket is not a drone';
export const M = 'M · Wings';
export const N = 'N · Pointing in space';
export const O = 'O · Guidance and navigation';

/** Lessons of Part IV, in plan order (by `n`). */
export const PART_FOUR: Lesson[] = [];
