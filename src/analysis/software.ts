// The flight-software scenarios of Part V (lessons V.3–V.5), shared by the lessons, the design
// review of V.6 and the tests: what is flown, for how long, and how it is judged.
import { Simulation } from '@/engine/simulation';
import { GRAVITY, type Params } from '@/sim/params';

/** Peak of |channel| from `from` to `to`, ignoring samples that are not numbers. */
export function peakOf(sim: Simulation, key: string, from: number, to = Infinity): number {
  const w = sim.telemetry.window([key], from);
  let m = 0;
  w.series[0]!.forEach((v, i) => {
    if (w.t[i]! <= to && Number.isFinite(v)) m = Math.max(m, Math.abs(v));
  });
  return m;
}

/** Largest |altitude − setpoint| from `from` to `to`, m. */
export function trackingPeak(sim: Simulation, from: number, to: number): number {
  const w = sim.telemetry.window(['pos.y', 'sp.y'], from);
  let m = 0;
  w.t.forEach((t, i) => {
    if (t <= to) m = Math.max(m, Math.abs(w.series[0]![i]! - w.series[1]![i]!));
  });
  return m;
}

/** Steps of ±0.5 m every 4 s: the integrator has work to do. */
const squareSteps = (p: Params) => {
  p.setpoint.profile = 'square';
  p.setpoint.profileAmplitude = 0.5;
  p.setpoint.profilePeriod = 8;
};

// ---------------------------------------------------------------------------------------------
// V.3: the fixed-point controller, back to back

/** Lesson V.3: judged from `from` until `end`; the outputs must agree within `limit` % of hover. */
export const FIXED = { from: 5, end: 30, limit: 1 };

/** The scenario of V.3: the shadow on, steps, no wind. Feedforward is left as the design has it. */
export const fixedScenario = (p: Params) => {
  p.wind.enabled = false;
  squareSteps(p);
  p.part5.fixed.shadow = true;
};

/** Largest disagreement so far, % of the hover thrust. */
export const fixedDisagreement = (sim: Simulation): number => peakOf(sim, 'fx.err', FIXED.from);

// ---------------------------------------------------------------------------------------------
// V.4: three altimeters

/** Lesson V.4: judged until `end`; the voted altitude must stay within `limit` m of the truth. */
export const VOTER = { end: 40, limit: 0.2 };

/** The scenario of V.4: steps, no wind, clean sensors, and the three altimeters. */
export const voterScenario = (p: Params) => {
  p.wind.enabled = false;
  squareSteps(p);
  p.sensors.posNoise = 0;
  p.part5.voter.enabled = true;
};

/** Largest error of the voted altitude so far, m (Infinity after a crash). */
export const voterError = (sim: Simulation): number =>
  sim.state.crashed ? Infinity : peakOf(sim, 'vote.err', 1);

// ---------------------------------------------------------------------------------------------
// V.5: the hand-over

/** Lesson V.5: the payload, the PID it flies with before, and the window judged after. */
export const TRANSFER = { mass: 1.25, window: 8, limit: 0.05 };

/** The kick of an unprepared hand-over: the weight the PID's integrator was holding, N. */
export const transferJump = (p: Params): number => (p.drone.mass - p.control.model.mass) * GRAVITY;

/** Fly a scenario for `seconds` with these parameters and return the simulation. */
export function flyFor(p: Params, seconds: number): Simulation {
  const sim = new Simulation(p);
  for (let k = 0; k < seconds * 1000; k++) sim.step();
  return sim;
}
