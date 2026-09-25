import type { ReactNode } from 'react';
import type { Simulation } from '@/engine/simulation';
import type { StepMetrics } from '@/engine/metrics';
import type { Level, Params } from '@/sim/params';

export interface GoalContext {
  sim: Simulation;
  metrics: StepMetrics | null;
}

export interface Lesson {
  id: string;
  title: string;
  level: Level;
  /** Modify a fresh copy of the default parameters. */
  setup?: (p: Params) => void;
  /** Schedule scripted events after the reset. */
  events?: (sim: Simulation) => void;
  /** Loop to show in the charts. */
  loop?: string;
  goal?: { text: string; check: (ctx: GoalContext) => boolean | string };
  body: ReactNode;
}
