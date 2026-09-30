import type { ReactNode } from 'react';
import type { Simulation } from '@/engine/simulation';
import type { StepMetrics } from '@/engine/metrics';
import type { Level, Params } from '@/sim/params';
import type { AnalysisChart, ExtraChart } from '@/store/sim';

export interface GoalContext {
  sim: Simulation;
  metrics: StepMetrics | null;
}

/** Part I = PID, Part II = beyond PID, Part III = why it works (analysis). One semester each. */
export type Part = 1 | 2 | 3;

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
  /**
   * Scripted events. Run after every reset while the lesson is active (so pressing R replays
   * them); use `setAt` from ./script for parameter changes.
   */
  events?: (sim: Simulation) => void;
  /** Run once when the lesson starts (ghost runs, reference scores). */
  onStart?: (sim: Simulation) => void;
  /** Loop to show in the charts. */
  loop?: string;
  /** What the bottom-right chart shows (default: wind). */
  chart?: ExtraChart;
  /** A second analysis view beside it, in place of the motor chart (Part III). */
  chart2?: AnalysisChart;
  goal?: { text: string; check: (ctx: GoalContext) => boolean | string };
  /** A parameter change that reaches the goal — used by the tests to prove it is reachable. */
  solution?: (p: Params) => void;
  body: ReactNode;
}
