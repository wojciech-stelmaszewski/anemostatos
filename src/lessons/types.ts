import type { ReactNode } from 'react';
import type { Simulation } from '@/engine/simulation';
import type { StepMetrics } from '@/engine/metrics';
import type { Level, Params } from '@/sim/params';
import type { ExtraChart } from '@/store/sim';

export interface GoalContext {
  sim: Simulation;
  metrics: StepMetrics | null;
}

/** Part I = PID (first semester), Part II = beyond PID (second semester). */
export type Part = 1 | 2;

export interface Lesson {
  id: string;
  title: string;
  level: Level;
  /** Defaults to Part I. Numbering restarts in each part. */
  part?: Part;
  /** Chapter heading within the part (Part II). */
  chapter?: string;
  /** Modify a fresh copy of the default parameters. */
  setup?: (p: Params) => void;
  /** Schedule scripted events after the reset. */
  events?: (sim: Simulation) => void;
  /** Loop to show in the charts. */
  loop?: string;
  /** What the bottom-right chart shows (default: wind). */
  chart?: ExtraChart;
  goal?: { text: string; check: (ctx: GoalContext) => boolean | string };
  /** A parameter change that reaches the goal — used by the tests to prove it is reachable. */
  solution?: (p: Params) => void;
  body: ReactNode;
}
