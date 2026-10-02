import { useEffect, useRef, useState } from 'react';

/** A job the page runs a slice at a time, so it stays responsive (budgets, campaigns, reviews). */
export interface SlicedJob {
  advance(budget: number): boolean;
  readonly progress: number;
}

/** Physics steps per timer tick. */
const SLICE = 20000;

/**
 * Run a job in slices from timers. `start` runs a new one; `onTick` sees it after every slice and
 * `onDone` when it is finished. Returns the progress (0–1) while running, or null.
 */
export function useSliced<J extends SlicedJob>(
  onDone: (job: J) => void,
  onTick?: (job: J) => void,
) {
  const [progress, setProgress] = useState<number | null>(null);
  const job = useRef<J | null>(null);
  const done = useRef(onDone);
  const ticked = useRef(onTick);
  useEffect(() => {
    done.current = onDone;
    ticked.current = onTick;
  });
  useEffect(
    () => () => {
      job.current = null;
    },
    [],
  );
  const start = (j: J) => {
    job.current = j;
    setProgress(0);
    const tick = () => {
      if (job.current !== j) return;
      const finished = j.advance(SLICE);
      ticked.current?.(j);
      if (finished) {
        job.current = null;
        setProgress(null);
        done.current(j);
      } else {
        setProgress(j.progress);
        setTimeout(tick, 0);
      }
    };
    setTimeout(tick, 0);
  };
  return { progress, start };
}
