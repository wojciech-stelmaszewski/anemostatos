import type { Vec3 } from '@/math/vec3';
import type { Actuation } from '@/sim/dynamics';
import type { Params } from '@/sim/params';
import type { Measurement } from '@/sim/sensors';
import type { PidTerms } from './pid';

export interface Setpoint {
  pos: Vec3;
  yaw: number; // rad
}

export interface ControlInput {
  params: Params;
  setpoint: Setpoint;
  /** Physics step, s. The controller decides itself when its loops are due. */
  physDt: number;
  stepIndex: number;
  /** Lazily sampled sensor reading — only read when a loop actually runs. */
  sense: () => Measurement;
}

export interface Controller {
  reset(): void;
  /** Zero all integrators — flight controllers do this while the drone sits on the ground. */
  resetIntegrators(): void;
  tick(input: ControlInput): Actuation;
  /** Latest terms of every PID loop, by id (e.g. 'alt', 'pos.x', 'rate.roll'). */
  loops(): Record<string, PidTerms>;
  /** Extra signals worth plotting (e.g. desired thrust vector), by name. */
  extras(): Record<string, number>;
}

/** How many physics steps between runs of a loop at `hz`. */
export const periodSteps = (hz: number, physDt: number): number =>
  Math.max(1, Math.round(1 / (Math.max(hz, 0.1) * physDt)));
