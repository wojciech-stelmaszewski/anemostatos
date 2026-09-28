import type { Params } from '@/sim/params';
import { AdrcController } from './adrc';
import { AltitudeController } from './altitude';
import { CascadeController } from './cascade';
import { WithAltitudeKalman } from './estimated';
import { LqrController } from './lqr';
import { PointMassController } from './pointmass';
import type { Controller } from './types';

/** Everything that selects which controller runs; a change restarts the simulation. */
export const controllerKey = (p: Params): string => {
  const l3 = p.control.l3;
  return [
    p.sim.level,
    p.control.l1.kind,
    p.control.l1.estimator,
    l3.outer,
    l3.inner,
    l3.compensation,
    l3.safety,
  ].join('|');
};

/** Build the controller selected by the parameters (docs/beyond-pid.md §3.2). */
export function makeController(p: Params): Controller {
  switch (p.sim.level) {
    case 1: {
      const kind = p.control.l1.kind;
      const c =
        kind === 'lqr'
          ? new LqrController()
          : kind === 'adrc'
            ? new AdrcController()
            : new AltitudeController();
      return p.control.l1.estimator === 'kalman' ? new WithAltitudeKalman(c) : c;
    }
    case 2:
      return new PointMassController();
    case 3:
      return new CascadeController(p);
  }
}
