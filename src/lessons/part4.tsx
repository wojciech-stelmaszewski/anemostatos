import { analyseTvc, minPitchGain, wrongWayZero, type TvcAnalysis } from '@/analysis/tvc';
import { bangBangLimits, minimumTime } from '@/control/bangbang';
import { designLqr } from '@/control/lqr';
import { insGyroError } from '@/estimation/ins';
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

/** Lesson IV.26: the gyro bias on roll, °/s, and how long the navigator runs alone, s. */
const DRIFT = { biasDeg: 0.05, seconds: 30 };
/** The error lesson IV.26 asks the student to predict: g·b·t³/6 after DRIFT.seconds, m. */
const driftAfter = (p: Params): number => {
  const b = p.sensors.gyroBias;
  return insGyroError((Math.hypot(b.x, b.z) * Math.PI) / 180, GRAVITY, DRIFT.seconds);
};

/** Lesson IV.27: the largest position error, in units of the filter's own σ, from `from` on. */
const worstNees = (sim: Simulation, from: number): number => {
  const w = sim.telemetry.window(['rng.err', 'rng.sigma2'], from);
  let worst = 0;
  for (let i = 0; i < w.t.length; i++) {
    const e = w.series[0]![i]!;
    const s = w.series[1]![i]! / 2;
    if (Number.isFinite(e) && s > 0) worst = Math.max(worst, e / s);
  }
  return worst;
};

/** Chapter L: the vehicle of lessons IV.7–IV.10, frozen at one flight condition with no wind. */
const pitchPlane = (p: Params) => {
  p.sim.vehicle = 'tvc';
  p.wind.enabled = false;
};
/** The design lesson IV.7 ends with, which IV.8 and IV.9 start from. */
const TVC_DESIGN = { kp: 1.5, kd: 0.8 };
/** Lesson IV.8: the gyro sits in the instrument bay, near the nose. */
const IV8_GYRO = 0.85;
/** Lesson IV.9: the slosh mass, its frequency and where the tank is. */
const IV9 = { mass: 300, hz: 0.8, ahead: 2, zeta: 0.002, enough: 0.035 };
/** Lesson IV.10: a hovering test vehicle steps sideways by this much at this time. */
const IV10 = { at: 2, step: 2, tol: 0.1, within: 6 };

/** The loop of the pitch-plane rocket, cached per parameter set (it is asked on every frame). */
let tvcCache: { key: string; a: TvcAnalysis } | null = null;
const tvcNow = (p: Params): TvcAnalysis => {
  const key = JSON.stringify([p.tvc, p.control.tvc]);
  if (tvcCache?.key !== key) tvcCache = { key, a: analyseTvc(p) };
  return tvcCache.a;
};
/** Largest |value| of a telemetry channel from `from` on. */
const peakSince = (sim: Simulation, key: string, from: number): number => {
  const w = sim.telemetry.window([key], from);
  return w.series[0]!.reduce((m, v) => (Number.isFinite(v) ? Math.max(m, Math.abs(v)) : m), 0);
};
/** Time from `t0` until the drift enters ±tol of `target` and stays there, s. */
const driftSettle = (sim: Simulation, t0: number, target: number, tol: number): number => {
  const { t, series } = sim.telemetry.window(['tvc.x'], t0);
  let settle = NaN;
  for (let i = 0; i < t.length; i++) {
    const x = series[0]![i]!;
    if (!Number.isFinite(x)) continue;
    if (Math.abs(x - target) > tol) settle = NaN;
    else if (Number.isNaN(settle)) settle = t[i]! - t0;
  }
  return settle;
};
const dB = (v: number) => `${v.toFixed(1)} dB`;

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
  {
    id: 'tvc',
    n: 7,
    part: 4,
    chapter: L,
    title: 'Balancing on a flame',
    level: 1,
    loop: 'pitch',
    chart: 'tvc',
    setup: (p) => {
      pitchPlane(p);
      p.control.tvc.kp = 0.1;
      p.control.tvc.kd = 0.5;
    },
    predict: {
      label: 'Smallest kp that holds the rocket',
      unit: '°/°',
      truth: minPitchGain,
      tolerance: 0.1,
    },
    goal: {
      text: 'Predict, within 10 %, the smallest pitch gain kp that holds the airframe. Then tune kp and kd so the loop keeps at least 6 dB of gain margin both ways, and fly it.',
      check: ({ sim, prediction }) => {
        const p = sim.params;
        if (p.sim.vehicle !== 'tvc') return 'this lesson flies the pitch-plane rocket';
        if (prediction == null) return 'first the prediction: the gimbal must out-push the air';
        const truth = minPitchGain(p);
        if (Math.abs(prediction - truth) / truth > 0.1)
          return 'not within 10 %: compare the turning moment of the air with that of the gimbal';
        const a = tvcNow(p);
        if (!a.stable) return 'the loop is unstable';
        if (a.lowDb < 6)
          return `lower gain margin ${dB(a.lowDb)}: it would fall over if the gain sagged`;
        if (a.highDb < 6) return `upper gain margin ${dB(a.highDb)}: too much gain`;
        if (sim.tvc && sim.state.crashed) return 'it broke up: press R';
        return sim.t > 8 || 'flying…';
      },
    },
    solution: (p) => {
      p.control.tvc.kp = TVC_DESIGN.kp;
      p.control.tvc.kd = TVC_DESIGN.kd;
    },
    body: (
      <>
        <p>
          A launch vehicle in the thick of the atmosphere is an arrow flying backwards. The air
          pushes on it at its <b>centre of pressure</b>, here 3 m ahead of the centre of mass, so a
          small angle of attack <Tex>{'\\alpha'}</Tex> makes a moment that turns the nose further
          off. The only thing that pushes back is the engine, on a gimbal 5 m behind the centre of
          mass. In the pitch plane:
        </p>
        <Tex display>
          {
            '\\ddot\\theta = \\mu_\\alpha\\,\\alpha - \\mu_c\\,\\delta,\\qquad \\mu_\\alpha = \\frac{N_\\alpha\\,l_{cp}}{I} = 4\\ \\mathrm{s^{-2}},\\qquad \\mu_c = \\frac{T\\,l_g}{I} = 8.3\\ \\mathrm{s^{-2}}'
          }
        </Tex>
        <p>
          With <Tex>{'\\mu_\\alpha > 0'}</Tex> the airframe has an unstable pole near{' '}
          <Tex>{'\\sqrt{\\mu_\\alpha} = 2'}</Tex> rad/s: left alone, an error doubles every third of
          a second. The rocket here is frozen at 400 m/s, and only its motion across the flight path
          is simulated. It starts 2° off, with almost no pitch gain. Press <b>R</b> and watch it go.
        </p>
        <p>
          The gimbal must at least out-push the air. If the controller answers a pitch error with{' '}
          <Tex>{'\\delta = k_p\\,\\theta'}</Tex>, the net stiffness is{' '}
          <Tex>{'\\mu_c k_p - \\mu_\\alpha'}</Tex>, which must be positive. So there is a{' '}
          <b>smallest</b> gain, not only a largest one: the loop is <b>conditionally stable</b>. On
          the Nyquist plot the curve crosses the negative real axis twice, once on each side of −1.
          Shrink the gain and the far crossing slides in over −1; grow it and the near one slides
          out. Each distance is a gain margin: the lower one says how much gain the loop may lose,
          the upper one how much it may gain. Because the open loop has unstable poles, the curve is
          not supposed to stay clear of −1: it must go around it, counter-clockwise, once for each
          of them.
        </p>
        <Try>
          Work out the smallest <Tex>{'k_p'}</Tex> from the numbers above and enter it. Then raise{' '}
          <b>Thrust-vector control → Pitch gain kp</b> past it and press <b>R</b>; the extra chart
          shows the loop. Push <b>kp</b> and <b>kd</b> high, five times the design, and watch the
          upper margin go: the actuator&apos;s 50 ms lag and the 20 ms of sensor delay are what
          limit it.
        </Try>
        <Notice>
          The formula <Tex>{'k_p > \\mu_\\alpha/\\mu_c'}</Tex> leaves out that the flight path
          itself turns: as the rocket drifts sideways the angle of attack is no longer the pitch.
          The full model gives a smallest gain a few per cent lower, and it depends a little on{' '}
          <Tex>{'k_d'}</Tex>, which the formula has no place for. The chart&apos;s margins scale the
          whole loop, kp and kd together. Six decibels both ways, the gain halved or doubled, is the
          classical requirement for the rigid body of a launch vehicle.
        </Notice>
      </>
    ),
  },
  {
    id: 'bending',
    n: 8,
    part: 4,
    chapter: L,
    title: 'The rocket is a noodle',
    level: 1,
    loop: 'pitch',
    chart: 'tvc',
    setup: (p) => {
      pitchPlane(p);
      p.control.tvc.kp = TVC_DESIGN.kp;
      p.control.tvc.kd = TVC_DESIGN.kd;
      p.tvc.gyroStation = IV8_GYRO;
    },
    goal: {
      text: 'With the gyro in the instrument bay (0.85 of the length), gain-stabilise the bending mode: |L| at the mode at most −10 dB, a phase margin of at least 30°, and a quiet gimbal in flight.',
      check: ({ sim }) => {
        const p = sim.params;
        if (p.sim.vehicle !== 'tvc') return 'this lesson flies the pitch-plane rocket';
        if (Math.abs(p.tvc.gyroStation - IV8_GYRO) > 0.005)
          return 'leave the gyro in the instrument bay (0.85) for the goal';
        const a = tvcNow(p);
        if (!a.stable) return 'the loop is unstable';
        if (a.bendDb > -10) return `|L| at the bending mode is ${dB(a.bendDb)}`;
        if (a.margins.pmDeg < 30)
          return `phase margin ${a.margins.pmDeg.toFixed(0)}°: the filter costs too much phase`;
        if (sim.t < 6) return 'flying…';
        const buzz = peakSince(sim, 'tvc.delta', sim.t - 2);
        return buzz < 0.3 || `the gimbal still moves ${buzz.toFixed(2)}°: press R`;
      },
    },
    solution: (p) => {
      p.control.tvc.notch = true;
      p.control.tvc.notchHz = p.tvc.bendHz;
      p.control.tvc.notchDepthDb = 40;
      p.control.tvc.notchWidth = 0.3;
    },
    body: (
      <>
        <p>
          The same rocket and the same controller, but the gyro now sits where gyros usually do, in
          the instrument bay near the nose. Press <b>R</b>. The pitch settles, and then the gimbal{' '}
          <b>sings</b>: a steady buzz at 5 Hz, as large as the actuator&apos;s rate limit allows.
        </p>
        <p>
          A 12 m vehicle is not rigid. Its first bending mode, at 5 Hz with 1 % damping, bends the
          middle one way and both ends the other. A rate gyro bolted to the structure measures the
          rigid rotation plus the local <b>slope</b> of the bending:
        </p>
        <Tex display>{"\\omega_{gyro} = \\dot\\theta + \\phi'(x_g)\\,\\dot\\eta"}</Tex>
        <p>
          The slope is zero at the middle and largest towards the ends, with opposite signs fore and
          aft. Near the nose the gyro sees the mode at full strength, and the controller pushes it
          through the gimbal at the tail. On the |L| chart the mode is a spike 26 dB high at 5 Hz;
          on the Nyquist plot it is a lobe that sweeps around −1.
        </p>
        <p>
          There are two cures. <b>Gain stabilisation</b> takes the loop gain at the mode well below
          1, here with a <b>notch</b> centred on it, so that whatever its phase the lobe stays
          small.
          <b> Phase stabilisation</b> leaves the gain but makes sure the lobe points away from −1,
          and that depends on where the sensor is.
        </p>
        <Try>
          Turn on <b>Thrust-vector control → Notch filter</b> at 5 Hz. Try depths of 20, 30 and 40
          dB, and widths from 0.1 to 0.5: a narrow notch misses part of the spike, a wide one costs
          phase at the 1 Hz crossover. Then switch the notch off and move{' '}
          <b>Pitch-plane rocket → Gyro station</b> to 0.4, and then to 0.6. The two places see the
          bending equally strongly, but with opposite signs: one sings, the other is stable with the
          spike still at +18 dB, because the lobe now turns away from −1.
        </Try>
        <Notice>
          A notch is only as good as the frequency it is centred on, and the bending frequency
          changes as the propellant burns; a deep narrow notch is fragile. Phase stabilisation needs
          the mode shape&apos;s sign at the sensor to be known, which it is for the first mode and
          less so for the higher ones. Real vehicles use both: phase-stabilise the first mode,
          gain-stabilise the rest. The uniform beam here puts the zero of the slope exactly at the
          middle; a real vehicle&apos;s is wherever its mass and stiffness put it.
        </Notice>
      </>
    ),
  },
  {
    id: 'slosh',
    n: 9,
    part: 4,
    chapter: L,
    title: 'Fuel that moves',
    level: 1,
    loop: 'pitch',
    chart: 'tvc',
    setup: (p) => {
      pitchPlane(p);
      p.control.tvc.kp = TVC_DESIGN.kp;
      p.control.tvc.kd = TVC_DESIGN.kd;
      p.tvc.sloshMass = IV9.mass;
      p.tvc.sloshHz = IV9.hz;
      p.tvc.sloshAhead = IV9.ahead;
      p.tvc.sloshZeta = IV9.zeta;
    },
    goal: {
      text: 'Keep the tank where it is and add baffles: the least slosh damping that keeps the whole Nyquist curve at least 0.5 from −1.',
      check: ({ sim }) => {
        const p = sim.params;
        if (p.sim.vehicle !== 'tvc') return 'this lesson flies the pitch-plane rocket';
        if (p.tvc.sloshMass !== IV9.mass || p.tvc.sloshAhead !== IV9.ahead)
          return 'leave the propellant and the tank where they are';
        const a = tvcNow(p);
        if (!a.stable) return 'the loop is unstable';
        const d = 1 / a.margins.ms;
        if (d < 0.5) return `closest approach to −1 is ${d.toFixed(2)}`;
        if (p.tvc.sloshZeta > IV9.enough)
          return `damping ${p.tvc.sloshZeta}: it works, but baffles are heavy. Find the least`;
        return true;
      },
    },
    solution: (p) => {
      p.tvc.sloshZeta = 0.03;
    },
    body: (
      <>
        <p>
          The tank 2 m ahead of the centre of mass holds liquid, and 300 kg of it can slosh. Seen
          from the controller it is a pendulum hung inside the vehicle: at 0.8 Hz, close to the 1 Hz
          crossover, and with 0.2 % damping, since nothing stops a liquid from moving. Press{' '}
          <b>R</b>. The pitch loop that was fine in lesson IV.7 settles, and then a wobble at 0.8 Hz
          comes back and grows, slowly: it doubles in about fifteen seconds.
        </p>
        <p>
          On the Nyquist plot the slosh mode is a small lobe near crossover. Small, but it sits
          where the curve passes closest to −1, and with so little damping it reaches over the
          point. Unlike the bending mode it cannot be notched out: a notch at 0.8 Hz would take the
          phase margin of the rigid body with it.
        </p>
        <p>
          The engineering cure is mechanical: <b>baffles</b>, rings inside the tank that make the
          liquid work to move, which adds damping and shrinks the lobe.
        </p>
        <Try>
          Raise <b>Pitch-plane rocket → Slosh damping (baffles)</b> step by step and watch the lobe
          shrink and the closest approach to −1 grow. Then put the damping back and move the tank
          instead, behind the centre of mass, or further ahead: where the tank is decides which way
          the lobe points, as the gyro station did for bending.
        </Try>
        <Notice>
          The pendulum&apos;s frequency is set by the vehicle&apos;s acceleration and the
          tank&apos;s size, and it rises during the flight as the acceleration grows. Large
          launchers have several tanks, each with its own slosh modes, and the margin is checked at
          every instant of the flight.
        </Notice>
      </>
    ),
  },
  {
    id: 'wrongway',
    n: 10,
    part: 4,
    chapter: L,
    title: 'First the wrong way',
    level: 1,
    loop: 'drift',
    chart: 'tvc',
    setup: (p) => {
      pitchPlane(p);
      // A hovering test vehicle: no air load, the thrust just holds the weight.
      p.tvc.speed = 0;
      p.tvc.thrust = Math.round(p.tvc.mass * GRAVITY);
      p.tvc.startPitchDeg = 0;
      p.control.tvc.kx = 0.5;
      p.control.tvc.kv = 2;
    },
    events: (sim) => setAt(sim, IV10.at, 'control.tvc.xTarget', IV10.step),
    predict: {
      label: 'Zero of the drift, z',
      unit: 'rad/s',
      truth: wrongWayZero,
      tolerance: 0.05,
    },
    goal: {
      text: `Predict, within 5 %, where the gimbal's push and the tilt cancel. Then make the ${IV10.step} m sidestep settle within ±${IV10.tol} m in under ${IV10.within} s.`,
      check: ({ sim, prediction }) => {
        const p = sim.params;
        if (p.sim.vehicle !== 'tvc') return 'this lesson flies the pitch-plane rocket';
        if (prediction == null) return 'first the prediction: when do the two pushes cancel?';
        const z = wrongWayZero(p);
        if (Math.abs(prediction - z) / z > 0.05)
          return 'not within 5 %: the side force is T·δ, the tilt it causes θ̈ = −T·l·δ/I';
        if (sim.state.crashed) return 'it fell over: press R';
        if (sim.t < IV10.at + IV10.within + 2) return 'stepping sideways…';
        const ts = driftSettle(sim, IV10.at, IV10.step, IV10.tol);
        if (!Number.isFinite(ts)) return 'it has not settled: press R';
        return ts < IV10.within || `settled in ${ts.toFixed(1)} s`;
      },
    },
    solution: (p) => {
      p.control.tvc.kx = 1.5;
      p.control.tvc.kv = 4;
    },
    body: (
      <>
        <p>
          The rocket now hovers, like a vertical-landing test vehicle, with its thrust holding its
          weight and no air load. At 2 s it is asked to move 2 m sideways. To move right it must
          lean right, and to lean right the gimbal must push the tail <b>left</b>. That push moves
          the whole rocket left before the lean has built up. Watch the drift chart: it first goes
          the wrong way, by a few centimetres.
        </p>
        <p>
          In the drift the gimbal acts twice: directly, with a side force <Tex>{'T\\delta'}</Tex>,
          and through the tilt it causes, <Tex>{'\\ddot\\theta = -\\mu_c\\delta'}</Tex>, which tips
          the thrust the other way:
        </p>
        <Tex display>
          {
            '\\frac{x(s)}{\\delta(s)} = \\frac{T}{m}\\,\\frac{s^2 - \\mu_c}{s^4},\\qquad z = \\sqrt{\\mu_c} = \\sqrt{\\frac{T\\,l_g}{I}}'
          }
        </Tex>
        <p>
          A zero in the right half-plane, at <Tex>{'z'}</Tex>. Below that frequency the tilt wins
          and the rocket goes where it leans; above it the side force wins and it goes the other
          way. A loop cannot be faster than its slowest right-half-plane zero allows, roughly{' '}
          <Tex>{'z/2'}</Tex>: past that the loop pushes ever harder against a plant that answers the
          wrong way. It is the mirror image of lesson IV.7&apos;s unstable pole, which demanded a
          bandwidth of at least about twice itself.
        </p>
        <Try>
          Enter <Tex>{'z'}</Tex>. Then raise <b>Drift gain</b> and <b>Drift-rate gain</b> in{' '}
          <b>Thrust-vector control</b> and press <b>R</b> each time. The step gets faster, the dip
          the wrong way gets deeper, from 2 cm to 20 cm, and somewhere around a drift gain of 5 °/m
          the gimbal runs into its stops and the rocket falls over. A little further, and the loop
          is unstable even without the stops.
        </Try>
        <Notice>
          The pitch loop is part of the limit too: the drift loop must be slower than the pitch loop
          inside it, and in this rocket the pitch loop&apos;s bandwidth is close to the zero. Making
          the pitch loop faster does not help here: above the zero the drift-rate feedback changes
          sign, and a faster pitch loop lets it through. Aircraft have the same zero, between the
          elevator and the altitude: pull up and the aeroplane first sinks.
        </Notice>
      </>
    ),
  },
  {
    id: 'pronav',
    n: 24,
    part: 4,
    chapter: O,
    title: 'Aim where it will be',
    level: 3,
    chart: 'engagement',
    loop: 'pos.x',
    setup: (p) => {
      p.wind.enabled = false;
      p.control.l3.outer = 'geometric';
      p.guidance = { ...p.guidance, enabled: true, law: 'pursuit', weaveAcc: 3 };
    },
    goal: {
      text: 'Catch the weaving target: a closest approach under 0.2 m.',
      check: ({ sim }) => {
        const p = sim.params;
        if (!p.guidance.enabled) return 'switch the chase on';
        if (p.control.l3.outer !== 'geometric')
          return 'the guidance law commands an acceleration: fly it with the geometric outer stage';
        const e = sim.engagement;
        if (!e.over) return sim.t < p.guidance.start ? 'taking off…' : 'chasing…';
        return (
          e.minRange < 0.2 || `closest approach ${e.minRange.toFixed(2)} m: press R to chase again`
        );
      },
    },
    solution: (p) => {
      p.guidance.law = 'pn';
    },
    body: (
      <>
        <p>
          A target crosses the sky at the drone's height, 2 m/s, swerving from side to side. The
          drone, twice as fast, must reach it. The obvious law is <b>pure pursuit</b>: always fly
          towards where the target is now.
        </p>
        <p>
          Watch the chart from above. The pursuer arrives where the target was and has to turn
          again: it curls in behind and ends in a tail chase, a third longer than it needs, and
          against a swerving target it cannot turn fast enough at the end.
        </p>
        <p>
          <b>Proportional navigation</b> [Zarchan 2012] flies to where the target <i>will be</i>. If
          the line of sight to the target does not rotate, the two are on a collision course
          (sailors know it: a ship whose bearing stays constant will hit you). So turn against the
          rotation:
        </p>
        <Tex display>{'a = N\\,V_c\\,\\dot\\lambda, \\qquad N \\approx 3\\text{–}5'}</Tex>
        <p>
          with <Tex>{'\\dot\\lambda'}</Tex> the rotation rate of the line of sight and{' '}
          <Tex>{'V_c'}</Tex> the closing speed. It needs only what a seeker measures: an angle rate
          and a closing speed, not where the target is going.
        </p>
        <Try>
          Watch the pursuit, then set <b>Guidance → Guidance law</b> to proportional navigation and
          press <b>R</b>. Then try <b>N</b> at 1, 3 and 6, and a bigger weave.
        </Try>
        <Notice>
          Proportional navigation has guided missiles since the 1950s, and a version of it brings
          spacecraft to a docking port. Its line of sight hardly turns until the very end, so the
          pursuer flies an almost straight line: here 20 m instead of pursuit's 29, and a hit
          instead of a 0.9 m miss.
        </Notice>
      </>
    ),
  },
  {
    id: 'ins',
    n: 26,
    part: 4,
    chapter: O,
    title: 'Drift',
    level: 3,
    chart: 'ins',
    loop: 'pos.x',
    setup: (p) => {
      p.sensors.ins = true;
      p.sensors.insStart = 5;
      p.sensors.gyroBias = { x: DRIFT.biasDeg, y: 0, z: 0 };
    },
    predict: {
      label: 'Navigator error after 30 s',
      unit: 'm',
      truth: driftAfter,
      tolerance: 0.1,
    },
    goal: {
      text: 'Before it happens, predict how far the inertial navigator will be from the truth after 30 seconds on its own, within 10 %.',
      check: ({ sim, prediction }) => {
        if (!sim.params.sensors.ins) return 'run the inertial navigator';
        const truth = driftAfter(sim.params);
        if (prediction == null)
          return 'first the prediction: a tilt that grows like b·t, seen by gravity';
        if (Math.abs(prediction - truth) / truth > 0.1)
          return 'not within 10 %: integrate twice — g·b·t, then t², then t³';
        const end = sim.params.sensors.insStart + DRIFT.seconds;
        if (sim.t < end) return 'drifting…';
        return true;
      },
    },
    solution: () => {},
    body: (
      <>
        <p>
          An <b>inertial navigator</b> knows where it is without looking outside. It integrates the
          gyro into an attitude, turns the accelerometer's reading into the world frame with that
          attitude, adds gravity back, and integrates twice into velocity and position. Submarines,
          airliners and rockets carry one; it cannot be jammed.
        </p>
        <p>
          Here one runs alongside the drone from 5 s, started from the truth and never corrected.
          Its gyro has a bias of 0.05 °/s on roll: a good MEMS gyro, a poor aircraft one. Without
          the bias the navigator follows the drone exactly; the error is the bias alone.
        </p>
        <p>
          A bias <Tex>{'b'}</Tex> tilts the navigator's "up" by <Tex>{'b\\,t'}</Tex>. Gravity, seen
          through that tilt, looks like a horizontal acceleration of <Tex>{'g\\,b\\,t'}</Tex>, and
          two integrations later:
        </p>
        <Tex display>{'e(t) \\approx \\tfrac{1}{6}\\,g\\,b\\,t^3'}</Tex>
        <Try>
          Compute the error after 30 s (the bias in rad/s!) and enter it. Then watch the error on a
          logarithmic scale against the dashed prediction.
        </Try>
        <Notice>
          The cube is merciless: a navigator that is 5 m off after 15 s is 38 m off after 30 s and
          300 m after a minute. That is why every inertial system is aided — by GPS, a star tracker,
          a terrain map — and why the aiding filter estimates the gyro bias, as the MEKF of III.23
          did. Over hours the error would stop growing like t³ and swing with the Schuler period of
          84 minutes: the earth's curvature turns a position error back into a tilt.
        </Notice>
      </>
    ),
  },
  {
    id: 'ukf',
    n: 27,
    part: 4,
    chapter: O,
    title: 'When the tangent lies',
    level: 3,
    chart: 'beacons',
    loop: 'pos.x',
    setup: (p) => {
      p.sensors.beacons = true;
      p.control.beaconFilter = { kind: 'ekf', guessX: 4, guessZ: 4, sigma0: 5, start: 5 };
      p.setpoint.profile = 'circle';
      p.setpoint.profileAmplitude = 1;
      p.setpoint.profilePeriod = 20;
    },
    goal: {
      text: 'Get an estimate you can trust: from 15 s to 35 s the position error never exceeds twice the filter’s own σ.',
      check: ({ sim }) => {
        if (!sim.params.sensors.beacons) return 'keep the beacon on';
        if (sim.t < 35) return 'filtering…';
        const w = worstNees(sim, 15);
        return w <= 2 || `the error reached ${w.toFixed(1)} σ of what the filter claims`;
      },
    },
    solution: (p) => {
      p.control.beaconFilter.kind = 'ukf';
    },
    body: (
      <>
        <p>
          A single beacon ten metres away tells the drone how far away it is, to 2 cm, ten times a
          second. A filter starts 6 m from the truth, says so (σ = 5 m), and estimates the position
          from the ranges alone. One range only says <i>on which circle</i> the drone is, not where
          on it: the honest answer is "on that circle, somewhere".
        </p>
        <p>
          The <b>EKF</b> replaces the circle by its tangent at the estimate. Each precise range then
          shrinks its uncertainty across the tangent, and it ends up sure of a point that can be 5,
          10, 20 metres off, with a σ of 30 cm. Its NIS looks healthy: the range it predicts matches
          the range it measures, because a wrong point on the right circle fits.
        </p>
        <p>
          The <b>unscented Kalman filter</b> [Julier 2004] does not linearise. It places 2n + 1
          sigma points around the estimate, pushes each through the true distance function, and
          reads the mean and the spread of the prediction from them. The curvature of the circle
          shows up in that spread, and the filter does not believe what it cannot know.
        </p>
        <Try>
          Watch the error against the dashed 2σ on the upper chart: the EKF's error runs far above
          its own bound. Switch <b>Beacon filter</b> to the UKF and press <b>R</b>.
        </Try>
        <Notice>
          The UKF is no better at finding the drone: it is still metres off, as one beacon must
          leave it. It is honest about it, and an honest filter can be combined, gated and trusted;
          a confident wrong one flies the vehicle into the ground. Note what the NIS could not tell:
          a consistent innovation does not prove a consistent estimate, which only the truth (here,
          the simulator) or a second, independent measurement can check.
        </Notice>
      </>
    ),
  },
];
