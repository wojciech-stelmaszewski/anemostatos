import type { Params } from '@/sim/params';
import { ignitionAltitude } from '@/sim/vehicles/rocket';
import { Notice, Try } from './Bits';
import { M as Tex } from './Math';
import type { Lesson } from './types';

/**
 * Part IV — Aerospace GNC (docs/aerospace-gnc.md §6): optimal control and guidance on the drone,
 * then a rocket, an aircraft and a satellite. Numbering restarts at IV.1 in the lesson panel;
 * every lesson carries its plan number `n`, since the lessons are built out of order.
 */

export const K = 'K · The best possible path';
export const L = 'L · A rocket is not a drone';
export const M = 'M · Wings';
export const N = 'N · Pointing in space';
export const O = 'O · Guidance and navigation';

/** Lesson IV.4: the propellant on board, kg — enough for the ideal burn and a few seconds more. */
const IV4_FUEL = 3;

/** The exact ignition altitude for these parameters, cached: a bisection over whole flights. */
let ignitionCache: { key: string; h: number } | null = null;
const exactIgnition = (p: Params): number => {
  const key = JSON.stringify(p.rocket);
  if (ignitionCache?.key !== key) ignitionCache = { key, h: ignitionAltitude(p.rocket) };
  return ignitionCache.h;
};

/** Lessons of Part IV, in plan order (by `n`). */
export const PART_FOUR: Lesson[] = [
  {
    id: 'suicideburn',
    n: 4,
    part: 4,
    chapter: K,
    title: 'Land on the last drop',
    level: 1,
    loop: 'alt',
    chart: 'phase',
    setup: (p) => {
      p.sim.vehicle = 'rocket';
      p.wind.enabled = false;
      p.rocket.fuel = IV4_FUEL;
      p.control.lander.ignitionAlt = 70;
    },
    predict: {
      label: 'Ignition altitude',
      unit: 'm',
      truth: exactIgnition,
      tolerance: 0.05,
    },
    goal: {
      text: 'Predict, within 5 %, the altitude at which full thrust must be lit to stop exactly at the pad. Then set the ignition altitude and land below 1 m/s.',
      check: ({ sim, prediction }) => {
        const p = sim.params;
        if (p.sim.vehicle !== 'rocket') return 'this lesson flies the rocket';
        if (prediction == null)
          return 'first the prediction: free fall from 120 m, then full thrust down to the pad';
        const truth = exactIgnition(p);
        if (Math.abs(prediction - truth) / truth > 0.05)
          return 'not within 5 %: the fall and the burn meet where their speeds are equal';
        const r = sim.rocket;
        if (!r) return 'press R to fly again';
        if (r.crashed)
          return r.fuel <= 0
            ? 'it ran dry before the pad: lit too high. Press R'
            : `it hit the pad at ${(-r.touchdown).toFixed(1)} m/s: lit too low. Press R`;
        if (!r.landed) return 'descending…';
        return -r.touchdown < 1 || `touched down at ${(-r.touchdown).toFixed(2)} m/s`;
      },
    },
    solution: (p) => {
      p.control.lander.ignitionAlt = exactIgnition(p) + 1;
    },
    body: (
      <>
        <p>
          A small rocket hangs at 120 m with its engine off and 3 kg of propellant: enough to stop
          once, and a few seconds more. The engine cannot throttle to zero, and once lit it is
          expensive to keep lit. The cheapest landing is one a PID never tries: fall freely, then
          burn at <b>full thrust</b> and arrive at the pad just as the speed reaches zero. Lunar
          landers and reusable boosters call it the <b>suicide burn</b>.
        </p>
        <p>
          The maximum principle says why it is fuel-optimal. The propellant used is
          <Tex>{'\\int T\\,dt/(I_{sp}g_0)'}</Tex>, and every second spent fighting gravity at less
          than full thrust is a second of gravity losses. So the burn should be as short as
          possible: full thrust, as late as possible. The only question left is <i>when</i>.
        </p>
        <p>
          With the mass held constant, the fall and the burn meet where their speeds are equal.
          Falling from rest at <Tex>{'h_0'}</Tex>, <Tex>{'v^2 = 2g(h_0 - h)'}</Tex>; braking with{' '}
          <Tex>{'a = T/m - g'}</Tex>, <Tex>{'v^2 = 2ah'}</Tex>. Together:
        </p>
        <Tex display>{'h_{ign} \\approx h_0\\,\\frac{m g}{T}'}</Tex>
        <Try>
          Compute <Tex>{'h_{ign}'}</Tex> from the numbers in <b>Rocket</b> (mass is dry mass plus
          propellant) and enter it. Then set <b>Rocket → Ignition altitude</b> and press <b>R</b>.
          Try a metre lower than your number, and ten metres higher.
        </Try>
        <Notice>
          The estimate is about one per cent high: during the burn the rocket loses three per cent
          of its mass, gets lighter and brakes harder, so it could have waited a little. Being high
          is the safe side. Half a metre too low and it hits the pad at five metres per second; ten
          metres too high and it hovers down, burns the last drop on the way and falls. Real landers
          aim a little high and use the throttle to correct. A booster whose engine at its lowest
          setting still lifts more than its weight cannot even do that: it cannot hover, and must
          get the timing right.
        </Notice>
      </>
    ),
  },
];
