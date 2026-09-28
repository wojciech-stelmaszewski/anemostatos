import { getIn, setIn } from '@/engine/schema';
import type { Simulation } from '@/engine/simulation';
import { useParams } from '@/store/params';
import { sim as appSim } from '@/store/sim';

/**
 * A scripted parameter change for lesson events. Lesson events run again after every reset, so
 * this first puts the parameter back to its value at the start of the lesson (a reset must not
 * start with the prop already broken), then schedules the change. It acts on the simulation it
 * is given — so headless ghosts and tests see it — and, for the app's own simulation, on the
 * store as well, so the parameter panel follows.
 */
export function setAt(sim: Simulation, t: number, path: string, value: unknown): void {
  const initial = getIn(sim.lessonStart ?? sim.params, path);
  const apply = (s: Simulation, v: unknown) => {
    if (getIn(s.params, path) === v) return;
    s.setParams(setIn(s.params, path, v));
    if (s === appSim) useParams.getState().set(path, v);
  };
  apply(sim, initial);
  sim.schedule(t, (s) => apply(s, value));
}
