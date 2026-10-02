import { GRAVITY, type Params } from '@/sim/params';
import { engineThrust, type RocketInput, type RocketState } from '@/sim/vehicles/rocket';
import type { LoopTerms } from './types';

/**
 * The landing law of lesson IV.4: fall with the engine off, light it at `ignitionAlt`, burn at
 * full thrust until the descent has slowed to the touchdown speed, then hold that speed with a
 * thrust near the weight. Lit too low it cannot stop in time; lit too high it hovers down and
 * may run out of propellant before the pad.
 */
export class RocketLander {
  lit = false;
  last: LoopTerms = emptyTerms();

  reset(): void {
    this.lit = false;
    this.last = emptyTerms();
  }

  tick(s: RocketState, p: Params): RocketInput {
    const r = p.rocket;
    const c = p.control.lander;
    if (!this.lit && s.h <= c.ignitionAlt) this.lit = true;
    let cmd = 0;
    if (this.lit && !s.landed) {
      const target = -c.touchdownSpeed;
      cmd =
        s.v < target
          ? r.thrustMax
          : s.mass * GRAVITY + s.mass * c.speedGain * (target - s.v);
    }
    const thrust = engineThrust(cmd, s, r);
    this.last = {
      setpoint: this.lit ? -c.touchdownSpeed : 0,
      measurement: s.v,
      error: (this.lit ? -c.touchdownSpeed : 0) - s.v,
      parts: [{ key: 'p', label: 'thrust', value: thrust, like: 'p' }],
      unsaturated: cmd,
      output: thrust,
      saturated: cmd > r.thrustMax || (cmd > 0 && thrust === 0),
    };
    return { thrust: cmd };
  }
}

function emptyTerms(): LoopTerms {
  return {
    setpoint: 0,
    measurement: 0,
    error: 0,
    parts: [{ key: 'p', label: 'thrust', value: 0, like: 'p' }],
    unsaturated: 0,
    output: 0,
    saturated: false,
  };
}
