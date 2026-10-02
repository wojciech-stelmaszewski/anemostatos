import { create } from 'zustand';
import { Simulation } from '@/engine/simulation';
import { useParams } from './params';

/** The one simulation instance. Lives outside React; see docs/software-architecture.md §1. */
export const sim = new Simulation(useParams.getState().params);
useParams.subscribe((s) => sim.setParams(s.params));

/** What the bottom-right chart slot shows. */
export type ExtraChart =
  | 'wind'
  | 'disturbance'
  | 'phase'
  | 'estimate'
  | 'bode'
  | 'nyquist'
  | 'poles'
  | 'lyapunov'
  | 'covariance'
  | 'mimo'
  | 'spectrum'
  | 'attitude'
  | 'dispersion'
  | 'fault'
  | 'fdi'
  | 'heading'
  | 'nav'
  | 'mrac'
  | 'describing'
  | 'mu';
/** The analysis views of Part III, which can also take the place of the motor chart. */
export type AnalysisChart = 'bode' | 'nyquist' | 'poles' | 'covariance';
/**
 * What the Bode chart draws: the open loop L, the closed loop from setpoint to altitude Y/R, the
 * sensitivity S, or ln|S| on a linear frequency axis (the waterbed view).
 */
export type BodeView = 'loop' | 'ref' | 'sens' | 'waterbed' | 'robust';

/** A loop shape to aim at: a crossover window, a phase margin, and a floor at low frequency. */
export interface BodeTarget {
  fcLoHz: number;
  fcHiHz: number;
  pmDeg: number;
  /** |L| must stay above lowDb at frequencies up to lowHz. */
  lowHz: number;
  lowDb: number;
}

export type CameraMode = 'orbit' | 'follow' | 'side' | 'top';

export interface Overlays {
  forces: boolean;
  terms: boolean;
  setpoint: boolean;
  trail: boolean;
  wind: boolean;
  heightAids: boolean;
}

interface UiStore {
  paused: boolean;
  timeScale: number;
  window: number;
  loop: string;
  extraChart: ExtraChart;
  /** A second analysis view, shown instead of the motor chart (docs/analysis.md §3.6). */
  secondChart: AnalysisChart | null;
  bodeView: BodeView;
  /** A target loop shape drawn on the open-loop Bode plot (lesson III.6), or null. */
  bodeTarget: BodeTarget | null;
  /** The number a student entered before running a lesson that asks for a prediction. */
  prediction: number | null;
  camera: CameraMode;
  overlays: Overlays;
  panelsHidden: boolean;
  lesson: string | null;
  setLesson: (id: string | null) => void;
  setPaused: (p: boolean) => void;
  setTimeScale: (s: number) => void;
  setWindow: (w: number) => void;
  setLoop: (l: string) => void;
  setExtraChart: (c: ExtraChart) => void;
  setSecondChart: (c: AnalysisChart | null) => void;
  setBodeView: (v: BodeView) => void;
  setBodeTarget: (t: BodeTarget | null) => void;
  setPrediction: (v: number | null) => void;
  setCamera: (c: CameraMode) => void;
  toggleOverlay: (k: keyof Overlays) => void;
  togglePanels: () => void;
}

export const useUi = create<UiStore>((set) => ({
  paused: false,
  timeScale: 1,
  window: 10,
  loop: 'alt',
  extraChart: 'wind',
  secondChart: null,
  bodeView: 'loop',
  bodeTarget: null,
  prediction: null,
  camera: 'follow',
  overlays: {
    forces: true,
    terms: true,
    setpoint: true,
    trail: true,
    wind: true,
    heightAids: true,
  },
  panelsHidden: false,
  lesson: 'meet',
  setLesson: (lesson) => set({ lesson }),
  setPaused: (paused) => {
    sim.paused = paused;
    set({ paused });
  },
  setTimeScale: (timeScale) => {
    sim.timeScale = timeScale;
    set({ timeScale });
  },
  setWindow: (window) => set({ window }),
  setLoop: (loop) => set({ loop }),
  setExtraChart: (extraChart) => set({ extraChart }),
  setSecondChart: (secondChart) => set({ secondChart }),
  setBodeView: (bodeView) => set({ bodeView }),
  setBodeTarget: (bodeTarget) => set({ bodeTarget }),
  setPrediction: (prediction) => set({ prediction }),
  setCamera: (camera) => set({ camera }),
  toggleOverlay: (k) => set((s) => ({ overlays: { ...s.overlays, [k]: !s.overlays[k] } })),
  togglePanels: () => set((s) => ({ panelsHidden: !s.panelsHidden })),
}));

/** Default loop shown in the inspector for each level. */
export const DEFAULT_LOOP = { 1: 'alt', 2: 'pos.y', 3: 'pos.y' } as const;

useParams.subscribe((s, prev) => {
  if (s.params.sim.level !== prev.params.sim.level)
    useUi.getState().setLoop(DEFAULT_LOOP[s.params.sim.level]);
});
