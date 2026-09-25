import { create } from 'zustand';
import { Simulation } from '@/engine/simulation';
import { useParams } from './params';

/** The one simulation instance. Lives outside React; see docs/software-architecture.md §1. */
export const sim = new Simulation(useParams.getState().params);
useParams.subscribe((s) => sim.setParams(s.params));

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
  camera: CameraMode;
  overlays: Overlays;
  panelsHidden: boolean;
  setPaused: (p: boolean) => void;
  setTimeScale: (s: number) => void;
  setWindow: (w: number) => void;
  setLoop: (l: string) => void;
  setCamera: (c: CameraMode) => void;
  toggleOverlay: (k: keyof Overlays) => void;
  togglePanels: () => void;
}

export const useUi = create<UiStore>((set) => ({
  paused: false,
  timeScale: 1,
  window: 10,
  loop: 'alt',
  camera: 'orbit',
  overlays: {
    forces: true,
    terms: true,
    setpoint: true,
    trail: true,
    wind: true,
    heightAids: true,
  },
  panelsHidden: false,
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
