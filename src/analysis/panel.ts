// The flexible panel of lesson IV.21 (docs/aerospace-gnc.md §4, Chapter N): how much it still
// swings after a slew, measured on a flight of the simulator itself.
import { Simulation } from '@/engine/simulation';
import type { Params } from '@/sim/params';

/** The panel's largest angle relative to the hub from `from` to `until` seconds, °. */
export function panelResidual(p: Params, from: number, until: number): number {
  const q = structuredClone(p);
  q.sim.level = 1;
  q.sim.vehicle = 'satellite';
  const sim = new Simulation(q);
  let peak = 0;
  const steps = Math.round(until * 1000);
  for (let k = 0; k < steps; k++) {
    sim.step();
    if (sim.t >= from) peak = Math.max(peak, Math.abs(sim.satellite!.eta));
  }
  return (peak * 180) / Math.PI;
}

/** The same slew without a shaper: the vibration a shaper is measured against, °. */
export const unshapedResidual = (p: Params, from: number, until: number): number =>
  panelResidual({ ...p, satellite: { ...p.satellite, shaper: 'none' } }, from, until);
