import { useMemo } from 'react';
import { create } from 'zustand';
import { analyseRateLoop, MimoSweep, type MimoPoint, type RateLoopAnalysis } from '@/analysis/mimo';
import { logspace } from '@/math/margins';
import { useParams } from '@/store/params';
import { loopKey } from './store';

/** The roll–pitch loop's model for the current parameters, recomputed only when the loop changes. */
export function useRateLoop(): { key: string; data: RateLoopAnalysis | null } {
  const key = useParams((s) => loopKey(s.params));
  return useMemo(() => ({ key, data: analyseRateLoop(useParams.getState().params) }), [key]);
}

interface MimoMeasureStore {
  /** The loop the points were measured on; they are shown only while it is the current one. */
  key: string | null;
  points: MimoPoint[];
  running: boolean;
  progress: number;
  start: () => void;
}

let raf = 0;
const SLICE = 20000;

/** Two torque sweeps (roll, then pitch at the same frequencies), flown in slices. */
export const useMimoMeasure = create<MimoMeasureStore>((set, get) => ({
  key: null,
  points: [],
  running: false,
  progress: 0,
  start: () => {
    cancelAnimationFrame(raf);
    const p = useParams.getState().params;
    if (p.sim.level !== 3) return;
    const sweep = new MimoSweep(p, logspace(0.3, 40, 12));
    const key = loopKey(p);
    set({ key, points: [], running: true, progress: 0 });
    const tick = () => {
      const done = sweep.advance(SLICE) || sweep.crashed;
      set({ points: sweep.points, progress: sweep.progress, running: !done });
      if (!done && get().key === key) raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
  },
}));
