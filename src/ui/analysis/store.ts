import { useMemo } from 'react';
import { create } from 'zustand';
import { closedLoopPoles, isLoopStable, l1Loop, loopGain, type LoopModel } from '@/analysis/loop';
import { Sweep, type SweepPoint } from '@/analysis/sweep';
import type { Complex } from '@/math/complex';
import { logspace, margins, type Margins } from '@/math/margins';
import type { Params } from '@/sim/params';
import { useParams } from '@/store/params';

/** Everything the analysis charts draw for one set of parameters. */
export interface LoopData {
  key: string;
  /** Null when the selected controller has no linear model (it can still be measured). */
  model: LoopModel | null;
  fHz: number[];
  l: Complex[];
  margins: Margins | null;
  /** Closed-loop poles as s-plane equivalents. */
  poles: Complex[];
  stable: boolean;
}

/** The parameters the loop depends on. The probe, the wind and the setpoint do not change it. */
export const loopKey = (p: Params): string =>
  JSON.stringify([p.sim.level, p.control, p.drone, p.sensors]);

export const F_MIN = 0.05;

export function analyse(p: Params): LoopData {
  const key = loopKey(p);
  const model = p.sim.level === 1 ? l1Loop(p) : null;
  // Up to the controller's Nyquist frequency: beyond it a sampled loop has no response of its own.
  const fMax = Math.min(100, p.control.rateHz / 2);
  const fHz = logspace(F_MIN, fMax, 400);
  if (!model) return { key, model, fHz, l: [], margins: null, poles: [], stable: true };
  const l = fHz.map((f) => loopGain(model, 2 * Math.PI * f));
  return {
    key,
    model,
    fHz,
    l,
    margins: margins(
      fHz.map((f) => 2 * Math.PI * f),
      l,
    ),
    poles: closedLoopPoles(model),
    stable: isLoopStable(model),
  };
}

/** The loop's model for the current parameters, recomputed only when the loop changes. */
export function useLoopData(): LoopData {
  const key = useParams((s) => loopKey(s.params));
  // eslint-disable-next-line react-hooks/exhaustive-deps -- the key stands for the parameters
  return useMemo(() => analyse(useParams.getState().params), [key]);
}

interface MeasureStore {
  /** The loop the points were measured on; they are shown only while it is the current one. */
  key: string | null;
  points: SweepPoint[];
  running: boolean;
  progress: number;
  start: () => void;
  cancel: () => void;
}

let raf = 0;
/** Physics steps flown per animation frame: about 10 ms of work on a laptop. */
const SLICE = 20000;

export const useMeasure = create<MeasureStore>((set, get) => ({
  key: null,
  points: [],
  running: false,
  progress: 0,
  start: () => {
    cancelAnimationFrame(raf);
    const p = useParams.getState().params;
    if (p.sim.level !== 1) return;
    const top = Math.min(20, p.control.rateHz * 0.4);
    const sweep = new Sweep(p, logspace(0.1, top, 14));
    const key = loopKey(p);
    set({ key, points: [], running: true, progress: 0 });
    const tick = () => {
      const done = sweep.advance(SLICE);
      set({ points: [...sweep.points], progress: sweep.progress, running: !done });
      if (!done && get().key === key) raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
  },
  cancel: () => {
    cancelAnimationFrame(raf);
    set({ running: false });
  },
}));

/** The measured points, if they belong to the loop on screen. */
export function useMeasured(data: LoopData): SweepPoint[] {
  const key = useMeasure((s) => s.key);
  const points = useMeasure((s) => s.points);
  return key === data.key ? points : EMPTY;
}
const EMPTY: SweepPoint[] = [];
