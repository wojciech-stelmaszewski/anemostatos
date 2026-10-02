import { bangBangLimits, minimumTime } from '@/control/bangbang';
import { designLqr } from '@/control/lqr';
import type { Simulation } from '@/engine/simulation';
import { GRAVITY, type Params } from '@/sim/params';
import { ignitionAltitude } from '@/sim/vehicles/rocket';
import { Notice, Try } from './Bits';
import { M as Tex } from './Math';
import { setAt } from './script';
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

const calm = (p: Params) => {
  p.wind.enabled = false;
};

/** The step of lesson IV.1: up by this much at this time. */
const IV1 = { at: 8, up: 1, qPos: 64, r: 0.5 };
/** λ_v at the instant of a step, from the continuous Riccati equation: m·√(q·r)·e₀, e₀ = −d. */
const costateAtStep = (p: Params): number => {
  const d = designLqr(p, 0.004);
  return d.p[0]![1]! * -IV1.up;
};

/** The descent of lesson IV.2: from 3 m down by this much at this time. */
const IV2 = { at: 10, down: 1.5, tol: 0.05 };
/** The fastest rest-to-rest descent the motors allow: fall with them off, brake at full thrust. */
const fastestDescent = (p: Params): number => {
  const l = bangBangLimits(p);
  return minimumTime(IV2.down, GRAVITY, l.aDown);
};
/** Time from the step until the altitude enters ±tol of the target and stays there, s. */
const settleTime = (sim: Simulation, t0: number, target: number, tol: number): number => {
  const { t, series } = sim.telemetry.window(['pos.y'], t0);
  let settle = NaN;
  for (let i = 0; i < t.length; i++) {
    const y = series[0]![i]!;
    if (!Number.isFinite(y)) continue;
    if (Math.abs(y - target) > tol) settle = NaN;
    else if (Number.isNaN(settle)) settle = t[i]! - t0;
  }
  return settle;
};

/** The scripted steps of lesson IV.3: down 2 m, then up 2 m. */
const IV3 = { down: 8, up: 16 };
/** Largest descent for which the LQR's first command stays within the motors: m̂·g/k₁. */
const saturationThreshold = (p: Params): number => {
  const k1 = designLqr(p, 0.004).k[0]!;
  return (p.control.model.mass * GRAVITY) / k1;
};

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
    id: 'costate',
    n: 1,
    part: 4,
    chapter: K,
    title: 'Where LQR comes from',
    level: 1,
    chart: 'costate',
    setup: (p) => {
      calm(p);
      p.control.l1.kind = 'lqr';
      p.control.lqr = { ...p.control.lqr, qPos: IV1.qPos, r: IV1.r, qVel: 10 };
    },
    events: (sim) => setAt(sim, IV1.at, 'setpoint.y', 2 + IV1.up),
    predict: {
      label: 'λ_v at the instant of the step',
      unit: '',
      truth: costateAtStep,
      tolerance: 0.05,
    },
    goal: {
      text: 'Before the step at 8 s: predict the velocity costate λ_v at the instant the setpoint jumps 1 m up, within 5 %. Then watch u = −R⁻¹Bᵀλ hold through the step.',
      check: ({ sim, prediction }) => {
        if (prediction == null)
          return 'solve the Riccati equation for P (it has a closed form here) and enter λ_v = (P·x)_v';
        const truth = costateAtStep(sim.params);
        if (Math.abs(prediction - truth) / Math.abs(truth) > 0.05)
          return 'not within 5 %: x₀ = (e, v) = (−1 m, 0), and λ = P·x₀';
        return sim.t > IV1.at + 3 || 'right; now fly through the step';
      },
    },
    solution: () => {},
    body: (
      <>
        <p>
          In lesson II.2 the LQR's gain came out of a Riccati equation and we took it on trust. Here
          is where it comes from. Minimise <Tex>{'J = \\int (x^\\top Q x + u^\\top R u)\\,dt'}</Tex>{' '}
          subject to <Tex>{'\\dot x = Ax + Bu'}</Tex>. Pontryagin's <b>maximum principle</b> adds a
          second state, the <b>costate</b> <Tex>{'\\lambda'}</Tex>, the price of a little more{' '}
          <Tex>{'x'}</Tex>, and the <b>Hamiltonian</b>
        </p>
        <Tex display>{'H = x^\\top Q x + u^\\top R u + \\lambda^\\top (Ax + Bu)'}</Tex>
        <p>
          Along the best path <Tex>{'u'}</Tex> minimises <Tex>{'H'}</Tex> at every instant, and the
          costate runs backwards from its own equation:
        </p>
        <Tex display>
          {
            '\\frac{\\partial H}{\\partial u} = 0 \\;\\Rightarrow\\; u = -\\tfrac12 R^{-1}B^\\top\\lambda, \\qquad \\dot\\lambda = -\\frac{\\partial H}{\\partial x} = -2Qx - A^\\top\\lambda'
          }
        </Tex>
        <p>
          Guess <Tex>{'\\lambda = 2Px'}</Tex> and the costate equation becomes the Riccati equation:
          the LQR is not a rule someone invented, it is the maximum principle solved. With the usual
          scaling (absorb the 2 into <Tex>{'\\lambda'}</Tex>) the law reads{' '}
          <Tex>{'u = -R^{-1}B^\\top\\lambda'}</Tex>, <Tex>{'\\lambda = Px'}</Tex>. For the drone, a
          double integrator with <Tex>{'B = (0, 1/m)'}</Tex>, the Riccati equation has a closed
          form, and its off-diagonal entry is
        </p>
        <Tex display>{'P_{ev} = m\\sqrt{q_e\\,r}'}</Tex>
        <Try>
          The weights are <Tex>{'q_e = 64'}</Tex> and <Tex>{'r = 0.5'}</Tex>, the drone weighs 1 kg,
          and at 8 s the setpoint jumps 1 m up: the state jumps to{' '}
          <Tex>{'x_0 = (e, v) = (-1, 0)'}</Tex>. Compute <Tex>{'\\lambda_v = P_{ev}\\,e_0'}</Tex>{' '}
          and enter it. Then watch the chart: the dashed line is{' '}
          <Tex>{'-R^{-1}B^\\top\\lambda'}</Tex> computed from the costate, the solid one the thrust
          the controller sends.
        </Try>
        <Notice>
          The two lines differ by up to 3.5 %: the controller is discrete, and in discrete time the
          law uses the costate one sample ahead, <Tex>{'u_k = -R^{-1}B^\\top\\lambda_{k+1}'}</Tex>.
          Along the flown path the costate equation holds within about 12 % of its terms; the rest
          is the motor lag, which the two-state model leaves out (with 2 ms motors it is 3 %). The
          costate is a sensitivity: <Tex>{'\\lambda_v'}</Tex> says how much the remaining cost would
          rise per metre per second of extra speed. That reading of it returns in every chapter of
          Part IV.
        </Notice>
      </>
    ),
  },
  {
    id: 'bangbang',
    n: 2,
    part: 4,
    chapter: K,
    title: 'Full throttle, then full brake',
    level: 1,
    chart: 'phase',
    setup: (p) => {
      calm(p);
      p.setpoint.y = 3;
      p.control.l1.kind = 'bangbang';
      p.control.bangbang = { ...p.control.bangbang, lead: 0, drag: false };
    },
    events: (sim) => setAt(sim, IV2.at, 'setpoint.y', 3 - IV2.down),
    predict: {
      label: 'Fastest possible 1.5 m descent',
      unit: 's',
      truth: fastestDescent,
      tolerance: 0.05,
    },
    goal: {
      text: 'Predict the fastest possible 1.5 m descent, within 5 %. Then make the drone settle within 5 cm of the new height in no more than 10 % above that time.',
      check: ({ sim, prediction }) => {
        if (prediction == null)
          return 'first the prediction: fall with the motors off, then brake at full thrust';
        const tStar = fastestDescent(sim.params);
        if (Math.abs(prediction - tStar) / tStar > 0.05)
          return 'not within 5 %: t = √(2d·(a₁ + a₂)/(a₁·a₂)), with a₁ = g and a₂ = T_max/m − g';
        if (sim.t < IV2.at + 4) return sim.t < IV2.at ? 'hovering…' : 'descending…';
        const ts = settleTime(sim, IV2.at, 3 - IV2.down, IV2.tol);
        return (
          ts <= 1.1 * tStar ||
          (Number.isNaN(ts)
            ? 'it never settled within 5 cm'
            : `settled in ${ts.toFixed(2)} s, against ${tStar.toFixed(2)} s possible: press R to fly again`)
        );
      },
    },
    solution: (p) => {
      p.control.bangbang.lead = 0.04;
    },
    body: (
      <>
        <p>
          What is the fastest way to change altitude? Make the cost the time itself,{' '}
          <Tex>{'J = \\int dt'}</Tex>, and bound the thrust. The Hamiltonian is then <i>linear</i>{' '}
          in the thrust, so the thrust that minimises it never sits in the middle: it is always on a
          bound. Full thrust one way, then full thrust the other way, with one switch. This is{' '}
          <b>bang-bang</b> control.
        </p>
        <p>
          For a descent of <Tex>{'d'}</Tex>: fall with the motors off (acceleration{' '}
          <Tex>{'a_1 = g'}</Tex>), then brake at full thrust (<Tex>{'a_2 = T_{max}/m - g'}</Tex>).
          The switch comes on the curve of states from which the brake, started now, stops exactly
          at the target:
        </p>
        <Tex display>
          {'d_{left} = \\frac{v^2}{2a_2}, \\qquad t^* = \\sqrt{\\frac{2d\\,(a_1 + a_2)}{a_1 a_2}}'}
        </Tex>
        <Try>
          Compute <Tex>{'t^*'}</Tex> for <Tex>{'d = 1.5'}</Tex> m (four motors of 6.1 N, a 1 kg
          drone) and enter it. Watch the phase portrait: the state falls, meets the blue switching
          curve and rides it home. It arrives late and overshoots. The motors take about 30 ms to
          spin up, and the curve does not know. Find <b>Bang-bang → Lead</b> and switch that much
          earlier.
        </Try>
        <Notice>
          Bang-bang is not a better controller than PID: a PID with large gains (Kp 60, Kd 10)
          settles the same descent in 0.67 s, because it too spends the whole manoeuvre at the
          thrust limits. What the maximum principle gives is the floor, <Tex>{'t^*'}</Tex>, and the
          shape: no controller, however clever, can beat one switch between the bounds. The same
          law, with the mass burning away, is how a rocket lands on its last drop of fuel (lesson
          IV.4).
        </Notice>
      </>
    ),
  },
  {
    id: 'bellman',
    n: 3,
    part: 4,
    chapter: K,
    title: 'The map of all futures',
    level: 1,
    chart: 'value',
    setup: (p) => {
      calm(p);
      p.setpoint.y = 4;
      p.control.l1.kind = 'dp';
    },
    events: (sim) => {
      setAt(sim, IV3.down, 'setpoint.y', 2);
      setAt(sim, IV3.up, 'setpoint.y', 4);
    },
    predict: {
      label: 'Largest descent the LQR flies unsaturated',
      unit: 'm',
      truth: saturationThreshold,
      tolerance: 0.05,
    },
    goal: {
      text: 'Predict the largest descent step for which the LQR’s first command stays within the motors, within 5 %. Beyond it the value function stops being the quadratic xᵀPx.',
      check: ({ sim, prediction }) => {
        if (prediction == null)
          return 'the LQR asks for −k₁·e; the motors give at most m·g less than hover';
        const truth = saturationThreshold(sim.params);
        if (Math.abs(prediction - truth) / truth > 0.05)
          return 'not within 5 %: the floor is zero thrust, m·g below hover, and k₁ ≈ √(q_e/r)';
        return sim.t > IV3.up + 4 || 'right; watch the two steps on the map';
      },
    },
    solution: () => {},
    body: (
      <>
        <p>
          The <b>value function</b> <Tex>{'V(x)'}</Tex> is the cost of the best possible future from
          the state <Tex>{'x'}</Tex>. It obeys <b>Bellman's equation</b>: the best future is the
          best first step followed by the best future from where it lands,
        </p>
        <Tex display>
          {'V(x) = \\min_u \\big[\\, \\ell(x, u)\\,\\Delta t + V(f(x, u)) \\,\\big]'}
        </Tex>
        <p>
          Solve it on a grid over <Tex>{'(e, v)'}</Tex> by sweeping the equation until it stops
          changing (<b>value iteration</b>), and the whole map of futures is on the screen, here for
          the LQR's own cost with the real thrust limits. The controller flying the drone simply
          picks, at every sample, the thrust that minimises one Bellman step.
        </p>
        <p>
          Without limits the map would be the quadratic bowl <Tex>{'V = x^\\top P x'}</Tex> of the
          LQR, and the best thrust its <Tex>{'-Kx'}</Tex>. The dashed lines mark where{' '}
          <Tex>{'-Kx'}</Tex> reaches the motors: zero thrust, <Tex>{'m g'}</Tex> below hover, on one
          side, full thrust on the other. They are not symmetric, and neither is the map.
        </p>
        <Try>
          The drone flies two steps: down 2 m at 8 s, up 2 m at 16 s. Before that, compute the
          largest descent the LQR flies without hitting zero thrust, <Tex>{'d^* = m g / k_1'}</Tex>{' '}
          with <Tex>{'k_1 \\approx \\sqrt{q_e/r}'}</Tex>, and enter it. Hover over the map to
          compare <Tex>{'V'}</Tex> with <Tex>{'x^\\top P x'}</Tex>.
        </Try>
        <Notice>
          Inside the dashed lines V is exactly the LQR's quadratic. Beyond them it rises above it: a
          4 m descent costs 16 % more than the quadratic says, a 4 m climb 7 % more, because falling
          is limited by gravity and climbing by four motors. The policy there is the saturated LQR,
          and it is nearly optimal: the LQR does not stop being good at the limits, it stops being
          exact. The grid costs something of its own (here a few per cent near the origin, from 0.1
          m cells and 1 N thrust steps). And the value function is a Lyapunov function, the one of
          lesson III.7 found by computation instead of by hand.
        </Notice>
      </>
    ),
  },
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
