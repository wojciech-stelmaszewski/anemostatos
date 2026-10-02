import type { Params } from '@/sim/params';
import { AdrcController } from './adrc';
import { AltitudeController } from './altitude';
import { FaultTolerantController } from './fault';
import { BacksteppingController } from './backstepping';
import { BangBangController } from './bangbang';
import { DpController } from './dp';
import { HinfController } from './hinf';
import { MracController } from './mrac';
import { SlidingModeController } from './smc';
import { CascadeController } from './cascade';
import { WithAltitudeKalman } from './estimated';
import { L1AdaptiveController } from './l1ac';
import { LqrController } from './lqr';
import { MpcAltitudeController } from './mpc';
import { PointMassController } from './pointmass';
import type { Controller } from './types';
import { WithFixedPoint } from './part5/shadow';
import { TransferController } from './part5/transfer';
import { WithVoter } from './part5/voter';

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
    p.control.fault.enabled,
    p.sim.vehicle ?? 'quadrotor',
    p.part5.fixed.shadow,
    p.part5.voter.enabled,
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
            : kind === 'mpc'
              ? new MpcAltitudeController()
              : kind === 'l1ac'
                ? new L1AdaptiveController()
                : kind === 'hinf'
                  ? new HinfController()
                  : kind === 'smc'
                    ? new SlidingModeController()
                    : kind === 'mrac'
                      ? new MracController()
                      : kind === 'backstepping'
                        ? new BacksteppingController()
                        : kind === 'bangbang'
                          ? new BangBangController()
                          : kind === 'dp'
                            ? new DpController()
                            : kind === 'transfer'
                              ? new TransferController()
                              : new AltitudeController();
      // Part V: the flight software around the law. The fixed-point shadow sees what the law
      // sees (after the filter); the voter picks the altitude everything downstream reads.
      let w: Controller = p.part5.fixed.shadow ? new WithFixedPoint(c) : c;
      if (p.control.l1.estimator === 'kalman') w = new WithAltitudeKalman(w);
      if (p.part5.voter.enabled) w = new WithVoter(w, p.sim.seed);
      return w;
    }
    case 2:
      return new PointMassController();
    case 3:
      return p.control.fault.enabled && p.control.l3.outer === 'pid-cascade'
        ? new FaultTolerantController(p)
        : new CascadeController(p);
  }
}
