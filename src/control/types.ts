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

/** One named additive contribution to a loop's output (P, I, D, −K·x, −f̂/b₀, …). */
export interface LoopPart {
  key: string;
  label: string;
  value: number;
  /** Feedforward rather than feedback (plotted legend-only, excluded from the feedback sum). */
  ff?: boolean;
}

/** What any loop reports for the inspector — the controller-agnostic form of `PidTerms`. */
export interface LoopTerms {
  setpoint: number;
  measurement: number;
  error: number;
  /** Named contributions, in display order; they add up to `unsaturated`. */
  parts: LoopPart[];
  unsaturated: number;
  output: number;
  saturated: boolean;
}

/** A PID's terms as loop parts: P, I, D and feedforward. */
export const pidLoop = (t: PidTerms): LoopTerms => ({
  setpoint: t.setpoint,
  measurement: t.measurement,
  error: t.error,
  parts: [
    { key: 'p', label: 'P', value: t.p },
    { key: 'i', label: 'I', value: t.i },
    { key: 'd', label: 'D', value: t.d },
    { key: 'ff', label: 'FF', value: t.ff, ff: true },
  ],
  unsaturated: t.unsaturated,
  output: t.output,
  saturated: t.saturated,
});

/** Look up a part's value by key (0 if the loop has no such part). */
export const partValue = (t: LoopTerms | undefined, key: string): number =>
  t?.parts.find((p) => p.key === key)?.value ?? 0;

export interface Controller {
  reset(): void;
  /** Zero all integrators — flight controllers do this while the drone sits on the ground. */
  resetIntegrators(): void;
  tick(input: ControlInput): Actuation;
  /** Latest terms of every loop, by id (e.g. 'alt', 'pos.x', 'rate.roll'). */
  loops(): Record<string, LoopTerms>;
  /** Extra signals worth plotting (e.g. desired thrust vector), by name. */
  extras(): Record<string, number>;
}

/** How many physics steps between runs of a loop at `hz`. */
export const periodSteps = (hz: number, physDt: number): number =>
  Math.max(1, Math.round(1 / (Math.max(hz, 0.1) * physDt)));
