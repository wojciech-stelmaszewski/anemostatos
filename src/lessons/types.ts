import type { ReactNode } from 'react';
import type { Simulation } from '@/engine/simulation';
import type { StepMetrics } from '@/engine/metrics';
import type { Level, Params } from '@/sim/params';
import type { AnalysisChart, BodeTarget, BodeView, ExtraChart } from '@/store/sim';

export interface GoalContext {
  sim: Simulation;
  metrics: StepMetrics | null;
  /** The number the student entered, for lessons that ask for a prediction. */
  prediction?: number | null;
}

/** A number the student works out before running the experiment (docs/analysis.md §3.7). */
export interface Prediction {
  label: string;
  unit: string;
  /** The right answer for these parameters. It is never shown; the goal only says how close. */
  truth: (p: Params) => number;
  /** Accepted relative error, e.g. 0.1 for ten per cent. */
  tolerance: number;
}

/** Part I = PID, Part II = beyond PID, Part III = why it works (analysis). One semester each. */
export type Part = 1 | 2 | 3;

export interface Lesson {
  id: string;
  title: string;
  level: Level;
  /** Defaults to Part I. Numbering restarts in each part. */
  part?: Part;
  /**
   * The lesson's number within its part, when it differs from its position in the list: Part III
   * is built in two passes, and the numbers of the plan are kept while some lessons are missing.
   */
  n?: number;
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
  /** Which response the Bode chart shows (default: the open loop). */
  bode?: BodeView;
  /** A target loop shape on the open-loop Bode plot. */
  bodeTarget?: BodeTarget;
  /** Ask for a number before the run; the goal's check receives it. */
  predict?: Prediction;
  goal?: { text: string; check: (ctx: GoalContext) => boolean | string };
  /** A parameter change that reaches the goal — used by the tests to prove it is reachable. */
  solution?: (p: Params) => void;
  body: ReactNode;
}
