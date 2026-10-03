import {
  aircraftModes,
  envelopeMargins,
  hqPoint,
  leastDamperGain,
  meetsPitchSpec,
  PITCH_SPEC,
  trimSlope,
} from '@/analysis/aircraft';
import { unshapedResidual } from '@/analysis/panel';
import { pioFreeRate } from '@/analysis/pio';
import { reconstructAltitude } from '@/analysis/smoother';
import { analyseTvc, minPitchGain, wrongWayZero, type TvcAnalysis } from '@/analysis/tvc';
import { bangBangLimits, minimumTime } from '@/control/bangbang';
import { designLqr } from '@/control/lqr';
import { limitCycleFuelRate, profileTime } from '@/control/satellite';
import { insGyroError } from '@/estimation/ins';
import { TELEMETRY_HZ, type Simulation } from '@/engine/simulation';
import { manoeuvreFor } from '@/guidance/collocation';
import { finishedLandings } from '@/guidance/pdgCampaign';
import { GRAVITY, type Params } from '@/sim/params';
import { BEACONS } from '@/sim/sensors';
import { ignitionAltitude } from '@/sim/vehicles/rocket';
import { panelFreeHz } from '@/sim/vehicles/satellite';
import { driftPerOrbit, meanMotion } from '@/sim/vehicles/chaser';
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

/** Chapter M: fly the aircraft, trimmed, with the bare airframe (no autopilot). */
const flyAircraft = (p: Params) => {
  p.sim.vehicle = 'aircraft';
  p.wind.enabled = false;
};
/** An elevator pulse of `deg` (positive: nose down) from `t` for `width` seconds. */
const elevatorPulse = (sim: Simulation, t: number, deg: number, width: number) => {
  setAt(sim, t, 'autopilot.elevatorOffset', deg);
  setAt(sim, t + width, 'autopilot.elevatorOffset', 0);
};
/** Lesson IV.11: the period of the bare airframe's phugoid, from its poles, s. */
const phugoidPeriod = (p: Params): number => aircraftModes(p, false).phugoid!.period;

/** Lesson IV.12: the least pure-damper gain for level 1, cached (a bisection over pole sets). */
let damperCache: { key: string; g: number } | null = null;
const leastGain = (p: Params): number => {
  const key = JSON.stringify([p.aircraft, p.autopilot.delayMs]);
  if (damperCache?.key !== key) damperCache = { key, g: leastDamperGain(p) };
  return damperCache.g;
};

/** Lesson IV.17: climb this much at this time; hold the speed this well; the gentlest throttle. */
const IV17 = { at: 5, climb: 300, tol: 0.3, maxGain: 0.2, until: 40 };
/** The largest speed error since the climb began, m/s. */
const speedSpread = (sim: Simulation): number => {
  const w = sim.telemetry.window(['air.V', 'air.vref'], IV17.at);
  let worst = 0;
  for (let i = 0; i < w.t.length; i++) {
    const e = Math.abs(w.series[0]![i]! - w.series[1]![i]!);
    if (Number.isFinite(e)) worst = Math.max(worst, e);
  }
  return worst;
};

/** Lesson IV.18: the command, the deadline and how long it must then stay within 0.1°. */
const IV18 = { deg: 240, deadline: 20, tol: 0.1, hold: 3 };
/** The time after which the pointing error stays below `tol`, s (NaN while it is above). */
const settledSince = (sim: Simulation, tol: number): number => {
  const w = sim.telemetry.window(['sat.err'], -Infinity);
  const err = w.series[0]!;
  let k = err.length - 1;
  if (k < 0 || !(err[k]! <= tol)) return NaN;
  while (k > 0 && err[k - 1]! <= tol) k--;
  return w.t[k]!;
};
/** The angle the shortest rotation turns through for a command of `slewDeg`, °. */
const shorterAngle = (p: Params): number => {
  const a = ((p.satellite.slewDeg % 360) + 360) % 360;
  return Math.min(a, 360 - a);
};

/** Lesson IV.19: a disturbance torque about z, and the thruster pair that dumps momentum. */
const IV19 = { torque: 0.02, thruster: 0.08, limitDeg: 0.5, until: 40 };
/** When the z wheel is full: it absorbs the disturbance at τ_d, so h(t) = τ_d·t. */
const wheelFullAt = (p: Params): number =>
  p.satellite.wheelMomentum / Math.abs(p.satellite.disturbance.z);
/** The most momentum any wheel has held in this run, N·m·s. */
const peakWheel = (sim: Simulation): number => {
  const w = sim.telemetry.window(['sat.h.x', 'sat.h.y', 'sat.h.z'], -Infinity);
  return Math.max(0, ...w.series.flatMap((s) => s.map(Math.abs)));
};

/** Lesson IV.20: judged from `from` on; pointing within `limitDeg`; fuel within `slack` of the least. */
const IV20 = { from: 15, until: 40, limitDeg: 0.5, slack: 1.3 };
/** The least fuel a dead band within the pointing limit can use: the band at the limit, the slowest drift, g/min. */
const leastFuel = (p: Params): number =>
  limitCycleFuelRate({ ...p.satellite, deadbandDeg: IV20.limitDeg, driftRateDeg: 0 }) * 60;
/** Propellant used per minute since `IV20.from`, g/min. */
const fuelRate = (sim: Simulation): number => {
  const w = sim.telemetry.window(['sat.fuel'], IV20.from);
  const f = w.series[0]!;
  const n = f.length - 1;
  return n > 0 ? ((f[n]! - f[0]!) / (w.t[n]! - w.t[0]!)) * 60 : NaN;
};

/** Lesson IV.22: judged from `from` to `until`; the tracker is blind for 10 s from `outage`. */
const IV22 = { from: 15, until: 40, limit: 10, outage: 25, drift: 30 };

/** Lesson IV.23: 90° about x on a ±0.5 N·m profile, settled by 10 s, never near a singularity. */
const IV23 = { deg: 90, torque: 0.5, h0: 1, deadline: 10, hold: 2, tol: 0.1, mMin: 0.1 };
/** The momentum the slew needs at its midpoint: τ·T/2 = √(Θ·τ·J), N·m·s. */
const slewMomentum = (p: Params): number =>
  (p.satellite.profileTorque * profileTime(p.satellite)) / 2;
/** The smallest singularity measure since the start. */
const leastMeasure = (sim: Simulation): number => {
  const w = sim.telemetry.window(['cmg.m'], -Infinity);
  return Math.min(...w.series[0]!.filter((v) => Number.isFinite(v)));
};
/** With the tracker off and the bias not estimated, the estimate drifts at |b|: 1 °/h is 1″/s. */
const unaidedDrift = (p: Params): number => {
  const b = p.satellite.gyroBiasDegH;
  return Math.hypot(b.x, b.y, b.z) * IV22.drift;
};
/** The largest knowledge error since `IV22.from`, arc-seconds. */
const worstKnowledge = (sim: Simulation): number => {
  const w = sim.telemetry.window(['st.err'], IV22.from);
  return Math.max(0, ...w.series[0]!.filter((v) => Number.isFinite(v)));
};

/** Lesson IV.14: how far, °, the flown loop's phase margin may differ from the table's. */
const IV14 = { agree: 5 };
/** The flown loop against the table's point designs across the envelope; cached per design. */
let hiddenCache: {
  key: string;
  r: { gap: number; ok: boolean; worstPm: number; slowestWc: number };
} | null = null;
const hiddenGap = (p: Params) => {
  const key = JSON.stringify([p.aircraft, { ...p.autopilot, thetaOffset: 0, elevatorOffset: 0 }]);
  if (hiddenCache?.key !== key) {
    const flown = envelopeMargins(p, 9);
    const table = envelopeMargins(p, 9, true);
    const gap = Math.max(...flown.map((f, i) => Math.abs(f.pmDeg - table[i]!.pmDeg)));
    const spec = meetsPitchSpec(p);
    hiddenCache = { key, r: { gap, ...spec } };
  }
  return hiddenCache.r;
};

/** Lesson IV.15: the model error the inversion must survive, either way. */
const IV15 = { error: 0.3 };
/** Whether the pitch requirement holds with the model `e` off, for both signs; cached per design. */
let ndiCache: { key: string; r: { e: number; ok: boolean; pm: number; wc: number }[] } | null =
  null;
const ndiRobust = (p: Params) => {
  const key = JSON.stringify([p.aircraft, p.autopilot.ndiBandwidth, p.autopilot.ndiAttitude]);
  if (ndiCache?.key !== key) {
    const r = [-IV15.error, IV15.error].map((e) => {
      const m = meetsPitchSpec({ ...p, autopilot: { ...p.autopilot, law: 'ndi', modelError: e } });
      return { e, ok: m.ok, pm: m.worstPm, wc: m.slowestWc };
    });
    ndiCache = { key, r };
  }
  return ndiCache.r;
};

/** Lesson IV.16: a startled pilot's two-cycle burst of elevator at the loop's own frequency. */
const IV16 = {
  at: 5,
  burst: 8,
  w: 3.8,
  early: [10, 15] as const,
  late: [25, 35] as const,
  slack: 1.25,
};
const startle = (sim: Simulation) => {
  const T = (4 * Math.PI) / IV16.w;
  for (let t = 0; t <= T; t += 0.02)
    setAt(sim, IV16.at + t, 'autopilot.elevatorOffset', IV16.burst * Math.sin(IV16.w * t));
  setAt(sim, IV16.at + T + 0.02, 'autopilot.elevatorOffset', 0);
};
/** Half the peak-to-peak of the elevator command between two times, °. */
const elevatorSwing = (sim: Simulation, from: number, to: number): number => {
  const w = sim.telemetry.window(['air.decmd'], from);
  const v = w.series[0]!.filter((x, i) => Number.isFinite(x) && w.t[i]! <= to);
  return v.length ? (Math.max(...v) - Math.min(...v)) / 2 : 0;
};
/** The slowest elevator rate with no PIO within the travel, cached (a bisection over describing functions). */
let pioCache: { key: string; r: number } | null = null;
const pioFree = (p: Params): number => {
  const ap = p.autopilot;
  const key = JSON.stringify([
    { ...p.aircraft, servoRate: 0 },
    ap.gain,
    ap.delayMs,
    ap.pilotGain,
    ap.pilotDelayMs,
  ]);
  if (pioCache?.key !== key) pioCache = { key, r: pioFreeRate(p) };
  return pioCache.r;
};

/** Lesson IV.21: the slew, and the window after it in which the panel must be still. */
const IV21 = { deg: 30, from: 15, until: 30, ratio: 0.05 };
/** The panel's largest angle from `IV21.from` on in this flight, °. */
const panelSwing = (sim: Simulation): number => {
  const w = sim.telemetry.window(['sat.eta'], IV21.from);
  return w.series[0]!.reduce((m, v) => (Number.isFinite(v) ? Math.max(m, Math.abs(v)) : m), 0);
};
let panelCache: { key: string; r: number } | null = null;
const unshaped = (p: Params): number => {
  const key = JSON.stringify({ ...p.satellite, shaper: 'none', shaperHz: 0 });
  if (panelCache?.key !== key) panelCache = { key, r: unshapedResidual(p, IV21.from, IV21.until) };
  return panelCache.r;
};

/** Lesson IV.5: the engine 5 % weak and a sideways push the guidance does not know about. */
const IV5 = { thrustScale: 0.95, wind: { x: 15, y: 0, z: -10 } };

/** Lesson IV.6: the dash past the pillar, and the time it must take at most. */
const IV6 = { distance: 6, pillarX: 3, pillarR: 0.5, within: 2.55 };
/** The least time with the acceleration limit alone (bang-bang, no pillar, no jerk limit), s. */
const bangBangTime = (p: Params): number =>
  2 *
  Math.sqrt(
    p.plan.distance /
      (p.plan.accelFraction * GRAVITY * Math.tan((p.control.l3.maxTiltDeg * Math.PI) / 180)),
  );
/** Saturated samples of the position loops and the closest approach to the pillar, in a window. */
const manoeuvreRecord = (sim: Simulation, from: number, to: number) => {
  const w = sim.telemetry.window(['pos.x.sat', 'pos.z.sat', 'zone.dist'], from);
  let saturated = 0;
  let closest = Infinity;
  w.t.forEach((t, i) => {
    if (t > to) return;
    if (w.series[0]![i]! > 0 || w.series[1]![i]! > 0) saturated++;
    const d = w.series[2]![i]!;
    if (Number.isFinite(d)) closest = Math.min(closest, d);
  });
  return { saturated, closest };
};

/** Lesson IV.28: calm air; a straight leg along z at `leg`, judged from `from` to `until`. */
const IV28 = { leg: 15, legZ: 4, from: 30, until: 40, limit: 2 };
/** Where the mirror image of the drone lies after a straight leg along z: across x = beacon x. */
const ghostX = (p: Params): number => 2 * BEACONS[0]!.x - p.setpoint.x;
/** The largest error and the largest σ of the beacon filter between `from` and `until`. */
const worstCloud = (sim: Simulation, from: number, until: number) => {
  const w = sim.telemetry.window(['rng.err', 'rng.sigma2'], from);
  let err = 0;
  let sigma = 0;
  w.t.forEach((t, i) => {
    if (t > until) return;
    const e = w.series[0]![i]!;
    const s2 = w.series[1]![i]!;
    if (Number.isFinite(e)) err = Math.max(err, e);
    if (Number.isFinite(s2)) sigma = Math.max(sigma, s2 / 2);
  });
  return { err, sigma };
};

/** Lesson IV.29: the altimeter's noise, the record's end, and what the smoother must reach. */
const IV29 = { noise: 0.05, until: 30, ratio: 0.55, rms: 0.01, start: 100, good: 1 };

/** Lesson IV.25: the Δv budget, m/s, and the time allowed, one orbit. */
const IV25 = { budget: 0.3 };
/** How much further behind one orbit of coasting leaves the chaser after its first burn, m. */
const fallsBehind = (p: Params): number =>
  -driftPerOrbit(p.rendezvous.burn, meanMotion(p.rendezvous));

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
    id: 'pdg',
    n: 5,
    part: 4,
    chapter: K,
    title: 'Convex landing',
    level: 1,
    loop: 'alt',
    chart: 'pdg',
    setup: (p) => {
      p.sim.vehicle = 'lander';
      p.wind.enabled = false;
      p.lander.thrustScale = IV5.thrustScale;
      p.lander.windForce = { ...IV5.wind };
      p.pdg.replanEvery = 0;
    },
    goal: {
      text: 'Fly the campaign of 20 dispersed starts with no failure: every case lands on the pad below 1 m/s without leaving the glide-slope cone, or is reported infeasible before it flies.',
      check: ({ sim }) => {
        const p = sim.params;
        if (p.sim.vehicle !== 'lander') return 'this lesson flies the 3D lander';
        const runs = finishedLandings(p);
        if (!runs) return 'press Fly 20 cases in the chart and wait for the campaign to finish';
        const failed = runs.filter((r) => !r.passed);
        if (failed.length)
          return `${failed.length} of ${runs.length} failed (${failed
            .slice(0, 5)
            .map((r) => `case ${r.case}: ${r.outcome}`)
            .join(', ')})`;
        return true;
      },
    },
    solution: (p) => {
      p.pdg.replanEvery = 0.5;
    },
    body: (
      <>
        <p>
          The rocket of IV.4 now starts 100 m up and 70 m to the side, falling at 19 m/s, and must
          land on the pad inside a <b>glide-slope cone</b>: seen from the pad it stays at least 30°
          above the horizon, so it never skims the ground. The thrust can point anywhere, between
          150 and 600 N while the engine burns.
        </p>
        <p>
          The fuel-optimal path is a nonconvex problem: the thrust has a lower bound, and the set{' '}
          <Tex>{'\\rho_1 \\le \\|T\\| \\le \\rho_2'}</Tex> is a ball with a hole in it. Açıkmeşe and
          Blackmore found the way out. With <Tex>{'u = T/m'}</Tex>, a slack{' '}
          <Tex>{'\\sigma \\ge \\|u\\|'}</Tex> and <Tex>{'z = \\ln m'}</Tex>, the bounds move onto{' '}
          <Tex>{'\\sigma'}</Tex> and the problem becomes a <b>second-order cone program</b>:
        </p>
        <Tex display>
          {
            '\\min \\sum \\sigma_k \\quad \\text{s.t.}\\quad \\|u_k\\| \\le \\sigma_k,\\;\\; \\rho_1 e^{-z} \\lesssim \\sigma_k \\lesssim \\rho_2 e^{-z},\\;\\; \\ddot r = u + g,\\;\\; \\|r_{xz}\\|\\tan\\gamma \\le r_y'
          }
        </Tex>
        <p>
          Convex, so an interior-point method finds the global optimum in a few dozen Newton steps.
          And the relaxation is <b>lossless</b>: at the optimum <Tex>{'\\|u\\| = \\sigma'}</Tex>{' '}
          again, so the answer obeys the real bounds. The readout shows the gap: zero to the
          solver's precision. The thrust chart shows the shape the maximum principle predicts: full,
          least, full.
        </p>
        <p>
          The flight time is not part of the convex problem; the guidance tries several and keeps
          the cheapest. If none works (not enough propellant, or too low and too fast), it{' '}
          <b>says so before it flies</b>. That is half of the value of convex guidance: a solver
          that cannot find a solution has proved that there is none.
        </p>
        <Try>
          Press <b>Start</b>. The engine gives 5 % less than commanded, and a steady wind pushes the
          lander sideways; the guidance knows neither. It measures the engine's shortfall from its
          own acceleration, but it plans once and flies that plan open loop, and the wind carries it
          off. Press <b>Fly 20 cases</b>: the dispersed starts crash one after another. Then set{' '}
          <b>Re-plan every</b> to 0.5 s and fly the campaign again.
        </Try>
        <Notice>
          Every re-plan starts from where the lander really is, so the wind's work is undone twice a
          second. A plan ends 3 m above the pad, descending at 1.5 m/s, and a simple feedback law
          flies the last metres, as real landers do: the fuel-optimal plan rides its thrust limit to
          the very end and leaves nothing for surprises. Re-planning is what turns an optimiser into
          a controller, and it is how Mars landers divert and boosters return to their pads.
        </Notice>
      </>
    ),
  },
  {
    id: 'collocation',
    n: 6,
    part: 4,
    chapter: K,
    title: 'Plan the whole flight',
    level: 3,
    loop: 'pos.x',
    chart: 'plan',
    setup: (p) => {
      p.wind.enabled = false;
      p.setpoint.x = 0;
      p.setpoint.y = 2;
      p.setpoint.z = 0;
      p.world.pillar = true;
      p.world.pillarX = IV6.pillarX;
      p.world.pillarZ = 0;
      p.world.pillarR = IV6.pillarR;
      p.control.l3.outer = 'geometric';
      p.control.geometric.feedforward = true;
      p.setpoint.profile = 'plan';
      p.plan = {
        ...p.plan,
        method: 'minsnap',
        duration: 2.5,
        distance: IV6.distance,
        accelFraction: 0.9,
      };
    },
    predict: {
      label: 'Least time, acceleration limit only',
      unit: 's',
      truth: bangBangTime,
      tolerance: 0.05,
    },
    goal: {
      text: `Predict the least time for the 6 m with the acceleration limit alone (within 5 %). Then dash past the pillar in at most ${IV6.within} s with a plan the controller never saturates on, without entering the pillar's zone.`,
      check: ({ sim, prediction }) => {
        const p = sim.params;
        if (p.setpoint.profile !== 'plan') return 'this lesson flies the planned manoeuvre';
        if (prediction == null)
          return 'first the prediction: full acceleration to half way, full braking after';
        const truth = bangBangTime(p);
        if (Math.abs(prediction - truth) / truth > 0.05)
          return 'not within 5 %: x = a t²/4 at the middle of a bang-bang move of length t';
        const plan = manoeuvreFor(p);
        if (plan.status !== 'optimal')
          return `no plan within the limits in ${plan.duration.toFixed(2)} s: the drone holds the start`;
        if (plan.duration > IV6.within + 1e-6)
          return `the plan takes ${plan.duration.toFixed(2)} s`;
        const from = sim.profileStart + p.plan.delay;
        const to = from + plan.duration + 0.5;
        if (sim.takingOff || sim.t < to) return 'flying the manoeuvre… (press R to fly it again)';
        const { saturated, closest } = manoeuvreRecord(sim, from, to);
        if (closest < 0) return 'it went through the pillar';
        return (
          saturated === 0 ||
          `the position loops saturated for ${(saturated / TELEMETRY_HZ).toFixed(2)} s: the plan asks for more than the drone can tilt`
        );
      },
    },
    solution: (p) => {
      p.plan.method = 'collocation';
      p.plan.minTime = true;
    },
    body: (
      <>
        <p>
          A pillar stands half way along a 6 m dash. The geometric controller of lesson II.11 flies
          whatever reference it is given, with feedforward; the question is the reference. Lesson
          II.12 planned with <b>min-snap</b>: here a waypoint beside the pillar, two polynomial
          pieces, as smooth as possible. It knows nothing of limits. The tilt limit of 35° allows{' '}
          <Tex>{'g\\tan 35^\\circ = 6.87'}</Tex> m/s² of horizontal acceleration; in 2.5 s min-snap
          asks for 7.5, and the controller saturates (shaded red).
        </p>
        <p>
          <b>Direct transcription</b> turns the planning itself into an optimisation. The jerk is
          held constant over 20 intervals, the state follows exactly, and the limits become
          constraints at every node:
        </p>
        <Tex display>
          {
            '\\min_{j_k,\\,T} \\; T \\quad\\text{s.t.}\\quad \\|a_k\\| \\le a_{max},\\;\\; \\|j_k\\| \\le j_{max},\\;\\; \\|p_k - c\\| \\ge R,\\;\\; \\text{rest to rest}'
          }
        </Tex>
        <p>
          Collocation methods keep every node's state as a variable and the dynamics as equality
          constraints; for this simple model the states can be eliminated exactly, which leaves the
          40 jerks and the time. The keep-out circle is not convex, so this is a{' '}
          <b>nonlinear program</b>, solved here by an augmented Lagrangian method. It finds a local
          optimum near its starting guess (the pillar's left side); a convex problem would promise
          the global one.
        </p>
        <Try>
          Predict the least time with only the acceleration limit, 90 % of 6.87 m/s²: accelerate
          flat out to half way, brake flat out. Then switch <b>Planned manoeuvre → Planner</b> to
          transcription, tick <b>Least time</b>, and press <b>R</b>.
        </Try>
        <Notice>
          The least time comes out about a quarter above your bang-bang bound: the detour round the
          pillar is longer, and the jerk limit (20 m/s³, how fast the drone can tilt) rounds every
          corner of the acceleration. Min-snap is not slow: with feedforward it flies cleanly up to
          about 2.6 s, where its peak meets the tilt limit. But it cannot be pushed further, because
          its shape is fixed; a plan built around the limits uses them fully and leaves 10 % for
          feedback by construction.
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
    id: 'modes',
    n: 11,
    part: 4,
    chapter: M,
    title: 'Two ways to wobble',
    level: 1,
    loop: 'pitch',
    chart: 'airmodes',
    setup: flyAircraft,
    events: (sim) => elevatorPulse(sim, 5, -2, 1),
    predict: {
      label: 'Phugoid period',
      unit: 's',
      truth: phugoidPeriod,
      tolerance: 0.1,
    },
    goal: {
      text: 'Before the pulse at 5 s, predict the period of the slow oscillation (the phugoid) within 10 %. Then read both periods off the pole map.',
      check: ({ sim, prediction }) => {
        const p = sim.params;
        if (p.sim.vehicle !== 'aircraft') return 'this lesson flies the aircraft';
        if (prediction == null)
          return 'first the prediction: a stone that trades height for speed and back';
        const truth = phugoidPeriod(p);
        if (Math.abs(prediction - truth) / truth > 0.1)
          return 'not within 10 %: Lanchester needs only the speed';
        return sim.t > 6 || 'wait for the pulse at 5 s';
      },
    },
    solution: () => {},
    body: (
      <>
        <p>
          A small jet cruises at 110 m/s, 4000 m up, trimmed: the elevator holds the nose where the
          lift equals the weight, and the throttle where the thrust equals the drag. Nobody touches
          anything. At 5 s the elevator is pulled up 2° for one second and released.
        </p>
        <p>
          Two motions follow, on very different time scales [Stevens 2016]. The nose bobs a few
          times within seconds and is still: the <b>short period</b>, a rotation about the centre of
          mass at almost constant speed, with the angle of attack as the spring and the tail as the
          damper. Then, much more slowly, the aircraft climbs and loses speed, dives and gains it,
          for minutes: the <b>phugoid</b>, an exchange of height for speed at almost constant angle
          of attack. Watch the speed chart.
        </p>
        <p>
          Lanchester's estimate of the phugoid treats the aircraft as a stone on a frictionless
          roller coaster: lift proportional to <Tex>{'V^2'}</Tex>, no drag, constant angle of
          attack. Its period depends only on the speed:
        </p>
        <Tex display>{'T_{ph} \\approx \\pi\\sqrt{2}\\,\\frac{V}{g}'}</Tex>
        <Try>
          Compute the period and enter it before 5 s. Then switch the chart window to 60 s and watch
          the speed. Hover the two panels of the pole map: the period is{' '}
          <Tex>{'2\\pi/\\omega_d'}</Tex>, with <Tex>{'\\omega_d'}</Tex> the imaginary part of the
          pole.
        </Try>
        <Notice>
          Two pairs of poles, a factor of 25 apart in frequency: −0.79 ± 3.24j for the short period
          (1.9 s, ζ 0.24) and −0.006 ± 0.127j for the phugoid (49 s, ζ 0.05). Lanchester is within
          about one per cent; the altitude, through the thinner air above, makes the true phugoid a
          little shorter. A pilot hardly notices the phugoid: it is slow enough to correct without
          thinking. The short period is the one a pilot feels and the one the next lesson fixes.
        </Notice>
      </>
    ),
  },
  {
    id: 'pitchdamper',
    n: 12,
    part: 4,
    chapter: M,
    title: 'Level 1',
    level: 1,
    loop: 'pitch',
    chart: 'hq',
    setup: flyAircraft,
    events: (sim) => elevatorPulse(sim, 5, -2, 0.5),
    goal: {
      text: 'Bring the short period into the level-1 box of the handling-qualities chart with a pure pitch damper (kθ = 0), at no more than 10 % above the least gain that does it.',
      check: ({ sim }) => {
        const p = sim.params;
        if (p.sim.vehicle !== 'aircraft') return 'this lesson flies the aircraft';
        if (p.autopilot.kTheta !== 0) return 'a pure damper: set the attitude gain kθ to 0';
        const hq = hqPoint(p);
        if (!hq) return 'no short period: is the airframe still stable?';
        if (hq.level !== 1)
          return `level ${hq.level}: ζ ${hq.zeta.toFixed(2)}, CAP ${hq.cap.toFixed(2)}`;
        const least = leastGain(p);
        if (p.autopilot.gain > 1.1 * least)
          return 'level 1, but more gain than it needs: every degree of damper fights the pilot too';
        return true;
      },
    },
    solution: (p) => {
      p.autopilot.gain = leastGain(p) * 1.03;
    },
    body: (
      <>
        <p>
          The short period of the last lesson has a damping ratio of 0.24: after a gust the nose
          overshoots and bobs three times. Is that acceptable? Aviation does not leave it to taste.
          A military standard [MIL-F-8785C] draws boxes on a chart of the short period's damping
          ratio against its <b>control anticipation parameter</b>: how hard the nose starts to move
          for the normal acceleration it will end with.
        </p>
        <Tex display>
          {
            '\\mathrm{CAP} = \\frac{\\omega_{sp}^2}{n_\\alpha}, \\qquad n_\\alpha = \\frac{\\bar q\\,S\\,C_{L_\\alpha}}{m g}'
          }
        </Tex>
        <p>
          Inside the inner box (<b>level 1</b>, ζ from 0.35 to 1.3) pilots rate the aircraft clearly
          adequate for the task; in the second (<b>level 2</b>) they can fly it with extra workload;
          outside, barely. This aircraft is at level 3, by its damping alone.
        </p>
        <p>
          The fix is a <b>pitch damper</b>: feed the pitch rate back to the elevator,{' '}
          <Tex>{'\\delta_e = \\delta_{e,trim} + G\\,q'}</Tex>. In the short-period approximation it
          adds <Tex>{'-M_{\\delta_e} G'}</Tex> to the damping term <Tex>{'2\\zeta\\omega'}</Tex> and
          hardly moves the frequency, so the point slides right along the chart.
        </p>
        <Try>
          Raise <b>Pitch loop gain G</b> from zero and watch the point move along the dashed path.
          Find the smallest gain that puts it in the level-1 box. Press <b>R</b> and compare the
          response to the pulse at 5 s.
        </Try>
        <Notice>
          The least gain is about 0.083 s: 0.083° of elevator per °/s of pitch rate. The
          short-period estimate,{' '}
          <Tex>{'G \\approx 2\\omega\\,(0.35 - \\zeta)/|M_{\\delta_e}|'}</Tex> with{' '}
          <Tex>{'M_{\\delta_e} \\approx -10.7'}</Tex> per second squared, gives 0.07. It leaves out
          that the elevator also changes the lift: the full model needs 0.079 even with an instant
          actuator and no delay, and the actuator's lag and the 40 ms of sensor and computer delay
          add the rest. Why not more gain? The damper cannot tell a gust from the pilot's own
          command, and it opposes both: too much of it, and the aircraft feels sluggish. Real
          aircraft wash the damper out at low frequency for that reason.
        </Notice>
      </>
    ),
  },
  {
    id: 'schedule',
    n: 13,
    part: 4,
    chapter: M,
    title: 'One gain does not fit all',
    level: 1,
    loop: 'pitch',
    chart: 'envelope',
    setup: (p) => {
      flyAircraft(p);
      p.aircraft.speed = 80;
      p.autopilot = {
        ...p.autopilot,
        gain: 0.5,
        kTheta: 2,
        schedule: 'none',
        designSpeed: 80,
        autothrottle: true,
        speedTarget: 160,
        speedRate: 2,
      };
    },
    events: (sim) => {
      // A 2° step slow (6 s) and fast (46 s), each held for 6 s.
      for (const t of [6, 46]) {
        setAt(sim, t, 'autopilot.thetaOffset', 2);
        setAt(sim, t + 6, 'autopilot.thetaOffset', 0);
      }
    },
    goal: {
      text: `A pitch-attitude loop that is quick and robust everywhere from 80 to 160 m/s: crossover at least ${PITCH_SPEC.wc} rad/s and phase margin at least ${PITCH_SPEC.pmDeg}° at every speed.`,
      check: ({ sim }) => {
        const p = sim.params;
        if (p.sim.vehicle !== 'aircraft') return 'this lesson flies the aircraft';
        if (!(p.autopilot.kTheta > 0)) return 'an attitude hold needs kθ above zero';
        const r = meetsPitchSpec(p);
        return (
          r.ok ||
          `worst phase margin ${r.worstPm.toFixed(0)}°, slowest crossover ${r.slowestWc.toFixed(2)} rad/s`
        );
      },
    },
    solution: (p) => {
      p.autopilot.schedule = 'qbar';
    },
    body: (
      <>
        <p>
          The same jet now starts slow, at 80 m/s, and the auto-throttle accelerates it to 160 m/s
          in 40 seconds. A pitch-attitude hold keeps the nose where level flight needs it: the
          damper of the last lesson inside, an attitude loop around it,{' '}
          <Tex>{'\\delta_e = \\delta_{e,trim} - G\\,(k_\\theta(\\theta_{ref} - \\theta) - q)'}</Tex>
          . The gain was tuned at 80 m/s, and a 2° step at 6 s is followed quickly and without
          overshoot. (Not all the way: without an integral the loop leaves part of the step as an
          error, until the slow motion of the flight path catches up. This lesson is about the
          loop's speed and damping.)
        </p>
        <p>
          The elevator's moment is proportional to the dynamic pressure{' '}
          <Tex>{'\\bar q = \\tfrac12\\rho V^2'}</Tex>. At twice the speed it is four times as
          strong, and so is the loop gain. The crossover moves up, the 40 ms of delay and the
          actuator cost more phase there, and the same 2° step at 46 s makes the nose buzz at 2 Hz:
          the damping of the short period has fallen from 0.46 to 0.25.
        </p>
        <p>
          The cure is as old as autopilots: design at several speeds and let the gain follow the
          flight condition, a <b>gain schedule</b> [Rugh 2000]. Here a single law suffices: divide
          the gain by <Tex>{'\\bar q/\\bar q_{design}'}</Tex>, so the product of gain and elevator
          effectiveness stays constant.
        </p>
        <Try>
          Look at the margins chart: the solid line is the gain as set, the dashed the other choice.
          Try to meet the requirement with a fixed gain first: lower <b>G</b> until the margin at
          160 m/s is enough, and watch the crossover at 80 m/s. Then set <b>Gain schedule</b> to{' '}
          <b>on dynamic pressure q̄</b>.
        </Try>
        <Notice>
          No fixed gain meets both lines: fast enough at 80 m/s (G 0.5) leaves 30° of phase margin
          at 160 m/s, and enough margin at 160 m/s leaves the loop sluggish at 80. Scheduled on q̄,
          the margin stays between 55° and 78°. The crossover still rises a little with speed,
          because the airframe's own short period gets faster; a real schedule is a table of
          designs, not one formula. Scheduling on a slow variable like q̄ is safe. Scheduling on a
          fast one, like the angle of attack, adds feedback that none of the point designs contained
          (lesson IV.14).
        </Notice>
      </>
    ),
  },
  {
    id: 'hidden',
    n: 14,
    part: 4,
    chapter: M,
    title: 'The trap in the table',
    level: 1,
    loop: 'pitch',
    chart: 'envelope',
    setup: (p) => {
      flyAircraft(p);
      p.aircraft.speed = 80;
      p.autopilot = {
        ...p.autopilot,
        gain: 0.5,
        kTheta: 2,
        schedule: 'alpha',
        scheduleTau: 0,
        designSpeed: 80,
        autothrottle: true,
        speedTarget: 160,
        speedRate: 2,
      };
    },
    events: (sim) => {
      for (const t of [6, 46]) {
        setAt(sim, t, 'autopilot.thetaOffset', 2);
        setAt(sim, t + 6, 'autopilot.thetaOffset', 0);
      }
    },
    predict: {
      label: 'Elevator the trim table adds per degree of α',
      unit: '°/°',
      truth: (p) => trimSlope(p.aircraft),
      tolerance: 0.1,
    },
    goal: {
      text: `Predict the slope of the trim table. Then keep the gain scheduled, but make the loop that flies the loop that was designed: at every speed from 80 to 160 m/s the flown phase margin within ${IV14.agree}° of the table's, and the requirement of lesson IV.13 met (phase margin ≥ ${PITCH_SPEC.pmDeg}°, crossover ≥ ${PITCH_SPEC.wc} rad/s).`,
      check: ({ sim, prediction }) => {
        const p = sim.params;
        if (p.sim.vehicle !== 'aircraft' || p.autopilot.law !== 'classic')
          return 'this lesson flies the aircraft on the classic pitch law';
        if (prediction == null)
          return 'first the prediction: how much elevator does the table add per degree of α?';
        const truth = trimSlope(p.aircraft);
        if (Math.abs(prediction - truth) > 0.1 * Math.abs(truth))
          return 'not within 10 %: along the level-flight trims the pitching moment is zero, C_m0 + C_mα·α + C_mδe·δe = 0';
        if (p.autopilot.schedule === 'none')
          return 'keep the gain scheduled: a fixed gain fails IV.13';
        if (!(p.autopilot.kTheta > 0)) return 'an attitude hold needs kθ above zero';
        const r = hiddenGap(p);
        if (r.gap > IV14.agree)
          return `the flown phase margin differs from the table's by up to ${r.gap.toFixed(0)}°`;
        return (
          r.ok ||
          `worst phase margin ${r.worstPm.toFixed(0)}°, slowest crossover ${r.slowestWc.toFixed(2)} rad/s`
        );
      },
    },
    solution: (p) => {
      p.autopilot.schedule = 'qbar';
    },
    body: (
      <>
        <p>
          A real schedule is not one formula but a table: a design at each of a few flight
          conditions, the gain <i>and</i> the trim elevator, read off at the condition of the
          moment. Which variable says where the aircraft is? In level flight the angle of attack
          does: slow means a high α, fast a low one, and an α vane is a cheaper sensor than air
          data. So this autopilot reads its table at the measured α. Every point design in it is the
          scheduled design of lesson IV.13, and checked one point at a time it is the same: the
          dashed line on the margins chart, 55° to 78°.
        </p>
        <p>
          The table, though, is not frozen in flight. The trim column holds the elevator that makes
          the pitching moment zero at each α,{' '}
          <Tex>{'C_{m0} + C_{m\\alpha}\\alpha + C_{m\\delta_e}\\delta_e = 0'}</Tex>, and the
          autopilot reads it at the α of the moment. When a gust or the short period moves α, the
          trim elevator moves with it: a feedback from α to the elevator that none of the point
          designs contained [Rugh 2000]. Here <Tex>{'C_{m\\alpha} = C_{m\\delta_e} = -1.2'}</Tex>.
        </p>
        <p>
          The solid line is the loop as it flies. The margin is not worse, it is <i>different</i>:
          85° instead of 55° at 80 m/s, and the crossover has fallen to 3.8 rad/s, below the
          requirement. The short period has slowed from about 1.7 s to 3.9 s and is far more damped
          than designed; the 2° step at 6 s now reaches 1.6° instead of 0.8°. The aircraft is no
          longer the one the table was designed for.
        </p>
        <Try>
          Work out the slope first: how much does <Tex>{'\\delta_{e,trim}'}</Tex> change per degree
          of α, and what is left of <Tex>{'C_{m\\alpha}'}</Tex> once it has? Then remove the path:
          give the scheduling variable a <b>low-pass on α</b> slower than the short period, or
          schedule on <b>q̄</b>, which air data measure and which changes only as fast as the speed.
        </Try>
        <Notice>
          The slope is <Tex>{'-C_{m\\alpha}/C_{m\\delta_e} = -1'}</Tex>: a degree of α moves the
          trim elevator a degree the other way, and the moment that comes back is{' '}
          <Tex>{'(C_{m\\alpha} + C_{m\\delta_e}\\cdot(-1))\\,\\Delta\\alpha = 0'}</Tex>. The table
          has cancelled the aircraft's static stability exactly, at every speed: with the
          autopilot's gain at zero, the short period breaks into a pole at 0 and one at −1.46 (the
          pitch damping alone). All the stiffness now comes from the attitude loop, which is why the
          steps follow better and the loop is slower. A low-pass of about a second puts the flown
          margins back within a few degrees of the table near crossover, but below the filter the
          cancellation is still there; the clean cure is a scheduling variable that the loop cannot
          move quickly, q̄. Scheduling on a fast variable is only safe if the hidden terms are put
          into the design, which is what dynamic inversion (next lesson) does on purpose.
        </Notice>
      </>
    ),
  },
  {
    id: 'ndi',
    n: 15,
    part: 4,
    chapter: M,
    title: 'Invert the aeroplane',
    level: 1,
    loop: 'pitch',
    chart: 'envelope',
    setup: (p) => {
      flyAircraft(p);
      p.aircraft.speed = 80;
      p.autopilot = {
        ...p.autopilot,
        gain: 0.5,
        kTheta: 2,
        schedule: 'qbar',
        designSpeed: 80,
        autothrottle: true,
        speedTarget: 160,
        speedRate: 2,
        law: 'ndi',
        ndiBandwidth: 8,
        ndiAttitude: 2,
        modelError: -IV15.error,
      };
    },
    events: (sim) => {
      for (const t of [6, 46]) {
        setAt(sim, t, 'autopilot.thetaOffset', 2);
        setAt(sim, t + 6, 'autopilot.thetaOffset', 0);
      }
    },
    predict: {
      label: 'Pitch-rate bandwidth the aircraft gets',
      unit: '1/s',
      truth: (p) => p.autopilot.ndiBandwidth / (1 + p.autopilot.modelError),
      tolerance: 0.05,
    },
    goal: {
      text: `Predict the bandwidth the aircraft really gets from the inversion. Then choose k_q and k_a so that the pitch loop meets the requirement of lesson IV.13 (phase margin ≥ ${PITCH_SPEC.pmDeg}°, crossover ≥ ${PITCH_SPEC.wc} rad/s, 80 to 160 m/s) with the model ${IV15.error * 100} % too weak and ${IV15.error * 100} % too strong.`,
      check: ({ sim, prediction }) => {
        const p = sim.params;
        if (p.sim.vehicle !== 'aircraft' || p.autopilot.law !== 'ndi')
          return 'this lesson flies the aircraft on dynamic inversion';
        if (prediction == null) return 'first the prediction: what does the model error do to k_q?';
        const truth = p.autopilot.ndiBandwidth / (1 + p.autopilot.modelError);
        if (Math.abs(prediction - truth) > 0.05 * truth)
          return 'not within 5 %: the aircraft gets the acceleration the model asks for, divided by 1 + e';
        const r = ndiRobust(p);
        const bad = r.find((x) => !x.ok);
        return (
          !bad ||
          `with the model ${bad.e > 0 ? 'too strong' : 'too weak'} by ${Math.abs(bad.e) * 100} %: worst phase margin ${bad.pm.toFixed(0)}°, slowest crossover ${bad.wc.toFixed(2)} rad/s`
        );
      },
    },
    solution: (p) => {
      p.autopilot.ndiBandwidth = 6;
      p.autopilot.ndiAttitude = 1;
    },
    body: (
      <>
        <p>
          The schedule of lesson IV.13 is a model in disguise: it assumes the elevator's moment
          grows with <Tex>{'\\bar q'}</Tex> and nothing else. <b>Nonlinear dynamic inversion</b>{' '}
          [Enns 1994] writes the model down whole. The pitching moment is
        </p>
        <Tex display>
          {
            '\\dot q = \\frac{\\bar q S c}{I}\\left(C_{m_0} + C_{m_\\alpha}\\alpha + C_{m_q}\\hat q + C_{m_{\\delta_e}}\\delta_e\\right)'
          }
        </Tex>
        <p>
          so the elevator that gives any wanted pitch acceleration <Tex>{'\\dot q_d'}</Tex> can be
          solved for, at every speed and angle of attack, with no table:
        </p>
        <Tex display>
          {
            '\\delta_e = \\frac{I\\,\\dot q_d/(\\bar q S c) - C_{m_0} - C_{m_\\alpha}\\alpha - C_{m_q}\\hat q}{C_{m_{\\delta_e}}}, \\qquad \\dot q_d = k_q\\left(k_a(\\theta_{ref} - \\theta) - q\\right)'
          }
        </Tex>
        <p>
          If the model were perfect, the aircraft would become <Tex>{'\\dot q = \\dot q_d'}</Tex> at
          every flight condition: a pitch-rate loop of bandwidth <Tex>{'k_q'}</Tex> everywhere. The
          model is not perfect. Here it believes the pitching moment is{' '}
          <b>{IV15.error * 100} % weaker</b> than it is (<b>Model error</b> −0.3), so it moves the
          elevator further than needed.
        </p>
        <Try>
          Work out the pitch-rate bandwidth the aircraft really gets with <Tex>{'k_q'}</Tex> = 8 and
          a model whose moment is <Tex>{'(1+e)'}</Tex> times the truth, and enter it. Then look at
          the margins chart: the solid line is the inversion, the dashed the q̄ schedule of the last
          lesson with the same error in the elevator's strength. Set <b>Model error</b> to +0.3 and
          back. Find <Tex>{'k_q'}</Tex> and <Tex>{'k_a'}</Tex> that pass both ways.
        </Try>
        <Notice>
          The model's error is the loop's gain error: the inversion asks for{' '}
          <Tex>{'\\dot q_d'}</Tex> and gets <Tex>{'\\dot q_d/(1+e)'}</Tex>. With <Tex>{'k_q'}</Tex>{' '}
          = 8 and e = −0.3 that is a bandwidth of 11.4; the measured crossover is 10.4 rad/s, a
          little lower because of the attitude loop around it. The phase margin then falls to 33°,
          and with e = +0.3 the loop is slow but safe. <Tex>{'k_q'}</Tex> = 6, <Tex>{'k_a'}</Tex> =
          1 holds 50° and at least 5 rad/s across the whole envelope, both ways. The flat lines are
          the point: one design for every speed, and a margin you can trust as far as you trust the
          model. The schedule tuned for its nominal elevator loses its crossover at 80 m/s when the
          elevator is 30 % weaker (3.8 rad/s). It could be retuned too; the difference is that the
          inversion's error is the same number everywhere, measured once.
        </Notice>
      </>
    ),
  },
  {
    id: 'pio',
    n: 16,
    part: 4,
    chapter: M,
    title: 'The pilot in the loop',
    level: 1,
    loop: 'pitch',
    chart: 'pio',
    setup: (p) => {
      flyAircraft(p);
      p.aircraft.servoRate = 20;
      p.autopilot = {
        ...p.autopilot,
        gain: 0.1,
        kTheta: 0,
        pilot: true,
        pilotGain: 1,
        pilotDelayMs: 150,
      };
    },
    events: startle,
    predict: {
      label: 'Slowest elevator rate with no PIO',
      unit: '°/s',
      truth: pioFree,
      tolerance: 0.1,
    },
    goal: {
      text: `Predict the slowest elevator rate at which no PIO is possible within the elevator's travel. Then choose the elevator rate so that the startle at ${IV16.at} s dies out (the swing from ${IV16.late[0]} to ${IV16.late[1]} s at most half that from ${IV16.early[0]} to ${IV16.early[1]} s), with no more than ${(IV16.slack - 1) * 100} % margin on that rate.`,
      check: ({ sim, prediction }) => {
        const p = sim.params;
        if (p.sim.vehicle !== 'aircraft' || !p.autopilot.pilot)
          return 'this lesson flies the aircraft with the pilot in the loop';
        if (prediction == null)
          return 'first the prediction: how does the onset amplitude scale with the rate?';
        const truth = pioFree(p);
        if (Math.abs(prediction - truth) > 0.1 * truth)
          return 'not within 10 %: read the onset off the chart, then scale it to the travel';
        if (p.aircraft.servoRate > IV16.slack * truth)
          return `${p.aircraft.servoRate} °/s is more actuator than needed: the least is about ${truth.toFixed(0)} °/s`;
        if (sim.t < IV16.late[1])
          return `watching the startle: ${sim.t.toFixed(0)} of ${IV16.late[1]} s`;
        const early = elevatorSwing(sim, IV16.early[0]!, IV16.early[1]!);
        const late = elevatorSwing(sim, IV16.late[0]!, IV16.late[1]!);
        return (
          late <= early / 2 ||
          `still swinging ±${late.toFixed(1)}° of elevator (±${early.toFixed(1)}° earlier): the PIO holds`
        );
      },
    },
    solution: (p) => {
      p.aircraft.servoRate = Math.ceil(pioFree(p) * 1.05);
    },
    body: (
      <>
        <p>
          A pilot flying attitude is a controller too: a gain (degrees of stick per degree of error)
          and a delay of perception and muscle, here 150 ms. A tense pilot uses a high gain. With
          this aircraft's pitch damper the small-signal loop is stable, with a thin gain margin of
          1.26.
        </p>
        <p>
          The elevator's actuator here is slow: 20°/s, as after a hydraulic failure. While the
          command moves slower than that, the actuator follows. A command{' '}
          <Tex>{'A\\sin\\omega t'}</Tex> asks for a rate of <Tex>{'A\\omega'}</Tex>; above the limit{' '}
          <Tex>{'R'}</Tex> the elevator becomes a triangle that lags the command. That lag is what
          the describing function of lesson III.8 measures. For a pure rate limiter it depends on{' '}
          <Tex>{'A\\omega/R'}</Tex> alone:
        </p>
        <Tex display>
          {
            'N \\approx \\frac{4R}{\\pi\\omega A}\\,e^{-j\\arccos\\left(\\frac{\\pi R}{2\\omega A}\\right)} \\quad \\text{for}\\ A\\omega > \\tfrac{\\pi}{2}R'
          }
        </Tex>
        <p>
          The chart computes it for the simulator's own actuator and multiplies it by the airframe
          and the pilot. At each amplitude it finds where the loop's phase is −180° and plots the
          gain there. Below 1 an oscillation of that size dies out; above 1 it grows. That is a{' '}
          <b>pilot-induced oscillation</b> [Klyde 1997]: nothing is broken, and the harder the pilot
          tries, the worse it gets.
        </p>
        <Try>
          Watch the startle at 5 s: two cycles of elevator from a surprised pilot, and the loop
          locks into a swing of ±14° that does not stop. Read the onset off the chart. Because N
          depends on <Tex>{'A\\omega/R'}</Tex> only, doubling the rate doubles the onset amplitude.
          Work out the rate at which the onset moves out to the elevator's travel of 20°, and enter
          it. Then set <b>Elevator rate limit</b> just above it, press <b>R</b>, and watch the
          startle die out.
        </Try>
        <Notice>
          The elevator starts to rate-limit at <Tex>{'R/\\omega'}</Tex> = 4.4° of command (grey
          line). The loop runs away only at 7.7°, once the lag has grown enough. In flight a burst
          that leaves less than about 6° of command dies out and one that leaves more than about 8°
          locks in. Scaling the onset to the travel gives 20 × 20/7.7 = 52°/s; the exact search
          gives 52.4°/s. The real cure is not always a faster actuator: pilots are told to let go,
          which sets their gain to zero, and modern flight controls filter the command so it never
          asks for more rate than the actuator has.
        </Notice>
      </>
    ),
  },
  {
    id: 'energy',
    n: 17,
    part: 4,
    chapter: M,
    title: 'Throttle is energy',
    level: 1,
    loop: 'pitch',
    chart: 'energy',
    setup: (p) => {
      flyAircraft(p);
      p.autopilot = {
        ...p.autopilot,
        gain: 0.25,
        kTheta: 2,
        schedule: 'qbar',
        designSpeed: 110,
        outer: 'separate',
        speedGain: 0.05,
        speedInt: 0.01,
      };
    },
    events: (sim) => setAt(sim, IV17.at, 'autopilot.altitudeOffset', IV17.climb),
    goal: {
      text: `Climb ${IV17.climb} m and hold the speed within ${IV17.tol} m/s all the way, without driving the engines harder than ${IV17.maxGain} throttle per m/s of speed error.`,
      check: ({ sim }) => {
        const p = sim.params;
        const ap = p.autopilot;
        if (p.sim.vehicle !== 'aircraft') return 'this lesson flies the aircraft';
        if (ap.outer === 'none') return 'switch the speed and altitude loops on';
        if (ap.outer === 'separate' && (ap.speedGain > IV17.maxGain || ap.speedInt > IV17.maxGain))
          return `the auto-throttle is too nervous: keep its gains at ${IV17.maxGain} or below`;
        const worst = speedSpread(sim);
        if (worst > IV17.tol)
          return `the speed strayed ${worst.toFixed(2)} m/s from its reference: press R to try again`;
        if (sim.t < IV17.until) return `climbing… (speed within ${worst.toFixed(2)} m/s so far)`;
        const a = sim.aircraft!;
        return (
          a.h - p.aircraft.altitude > IV17.climb - 50 || 'the speed is held, but it hardly climbs'
        );
      },
    },
    solution: (p) => {
      p.autopilot.outer = 'tecs';
    },
    body: (
      <>
        <p>
          The jet flies at 110 m/s with the scheduled pitch hold of the last lesson. Two more loops
          sit on top, the way a simple autopilot is built: the <b>throttle holds the speed</b> and
          the <b>elevator holds the altitude</b>. At 5 s the altitude reference jumps 300 m. Watch
          the speed.
        </p>
        <p>
          The elevator starts the climb at once. Climbing at a flight-path angle{' '}
          <Tex>{'\\gamma'}</Tex> costs <Tex>{'m g \\sin\\gamma'}</Tex> of extra thrust, and nobody
          asks the engines for it: the throttle loop only answers a speed error, so the speed has to
          drop first, and the engines take a second and a half to spool up. The two loops work on
          the same two quantities and do not know about each other.
        </p>
        <p>
          <b>Total energy control</b> [Lambregts 1983] splits the work differently. The specific
          energy of the aircraft, height plus <Tex>{'V^2/2g'}</Tex>, changes at the rate
        </p>
        <Tex display>{'\\frac{\\dot E}{m g V} = \\gamma + \\frac{\\dot V}{g}'}</Tex>
        <p>
          and only the engines can change it. The elevator can only trade one form for the other, at
          the rate <Tex>{'\\gamma - \\dot V/g'}</Tex>. So the throttle is given the sum, the
          elevator the difference, and the thrust a climb needs is asked for the moment the climb is
          commanded.
        </p>
        <Try>
          Try to meet the goal with the separate loops: raise <b>Auto-throttle gain</b> and{' '}
          <b>integral</b> up to 0.2. Then set <b>Speed and altitude</b> to{' '}
          <b>total energy control</b> and press <b>R</b>. Set its four gains to zero: what is left?
        </Try>
        <Notice>
          With the separate loops as set, the speed sags 2.5 m/s in the climb. Tuned up to 0.2 they
          get within about half a metre per second, and beyond that the engine lag makes the speed
          loop hunt. Total energy control holds 0.15 m/s, and still 0.4 m/s with all four of its
          gains at zero: most of the work is done by the feedforward, the thrust{' '}
          <Tex>{'m g\\,(\\gamma_{ref} + \\dot V_{ref}/g)'}</Tex> that the energy balance says the
          command needs. The same structure flies on the Boeing 777 and 787, and in small drones.
        </Notice>
      </>
    ),
  },
  {
    id: 'slew',
    n: 18,
    part: 4,
    chapter: N,
    title: 'Point the telescope',
    level: 1,
    loop: 'point',
    chart: 'lyapunovV',
    setup: (p) => {
      p.sim.vehicle = 'satellite';
      p.satellite.slewDeg = IV18.deg;
      p.satellite.shortest = false;
    },
    predict: {
      label: 'Angle it turns through with the shortest rotation',
      unit: '°',
      truth: shorterAngle,
      tolerance: 0.05,
    },
    goal: {
      text: 'Predict the angle the satellite turns through with Shortest rotation on. Then reach the target within 0.1° in under 20 s, and stay there for 3 s.',
      check: ({ sim, prediction }) => {
        const p = sim.params;
        if (p.sim.vehicle !== 'satellite') return 'this lesson flies the satellite';
        if (prediction == null) return 'first the prediction: q and −q are the same attitude';
        const truth = shorterAngle(p);
        if (Math.abs(prediction - truth) > 0.05 * truth)
          return 'not within 5 %: a turn of 240° one way ends where a turn of 120° the other way does';
        const since = settledSince(sim, IV18.tol);
        if (Number.isNaN(since)) return 'slewing…';
        if (since > IV18.deadline)
          return `settled after ${since.toFixed(1)} s; the deadline is ${IV18.deadline} s. Press R`;
        return sim.t - since >= IV18.hold || 'holding…';
      },
    },
    solution: (p) => {
      p.satellite.shortest = true;
      p.satellite.kp = 8;
      p.satellite.kd = 8;
    },
    body: (
      <>
        <p>
          A satellite floats with nothing to push against. It turns with three{' '}
          <b>reaction wheels</b>: spin a wheel one way and the body turns the other, and the total
          angular momentum stays what it was. The wheels here give at most 0.2 N·m each and store at
          most 0.6 N·m·s; the satellite's inertia is 4, 5 and 3 kg·m² about its axes.
        </p>
        <p>
          The attitude is a quaternion <Tex>{'q'}</Tex>, and the error from the target is another
          one, <Tex>{'q_e = q_t^{-1}\\otimes q'}</Tex>. Quaternion feedback is a PD on its vector
          part:
        </p>
        <Tex display>{'\\tau = -k_p\\,\\mathbf q_e - k_d\\,\\omega'}</Tex>
        <p>
          It is stable for <i>any</i> rigid body, and the proof fits in two lines. Take the kinetic
          energy plus a spring on the error,{' '}
          <Tex>{'V = \\tfrac12\\omega^\\top J\\omega + 2k_p(1 - q_{e0})'}</Tex>. The gyroscopic term
          does no work, and what is left is <Tex>{'\\dot V = -k_d\\,|\\omega|^2 \\le 0'}</Tex>. The
          chart plots <Tex>{'V'}</Tex>: it only falls.
        </p>
        <p>
          The command is 240° about the diagonal. But <Tex>{'q'}</Tex> and <Tex>{'-q'}</Tex> are the
          same attitude, so a law that takes the error as it comes can go the long way round, as the
          drone did in lesson II.10. Flip the error to the half with <Tex>{'q_{e0} \\ge 0'}</Tex>{' '}
          and it always takes the shorter way.
        </p>
        <Try>
          Press <b>R</b> and watch the satellite turn the long way. How far does it turn with{' '}
          <b>Satellite → Shortest rotation</b> on? Enter the angle, switch it on, and tune <b>kp</b>{' '}
          and <b>kd</b> until it settles within 0.1° in under 20 s.
        </Try>
        <Notice>
          More gain stops helping. A wheel at 0.6 N·m·s cannot spin faster, so the satellite cannot
          turn faster than about 16°/s. The long way cannot settle within 20 s with any gains (its
          best is about 23 s); the short way can in about 15. The proof assumed every torque asked
          for is given. A wheel at full speed does not break it here, but with gains high enough to
          hit the torque limit <Tex>{'V'}</Tex> rises, briefly and only by a millionth: the
          guarantee is gone, even if the flight looks the same.
        </Notice>
      </>
    ),
  },
  {
    id: 'wheels',
    n: 19,
    part: 4,
    chapter: N,
    title: 'Spin to turn',
    level: 1,
    loop: 'point',
    chart: 'wheels',
    setup: (p) => {
      p.sim.vehicle = 'satellite';
      p.satellite.slewDeg = 0;
      p.satellite.kp = 8;
      p.satellite.kd = 8;
      p.satellite.disturbance = { x: 0, y: 0, z: IV19.torque };
      p.satellite.thrusterTorque = IV19.thruster;
    },
    predict: {
      label: 'Time until the z wheel is full',
      unit: 's',
      truth: wheelFullAt,
      tolerance: 0.05,
    },
    goal: {
      text: 'Predict when the z wheel reaches full speed. Then hold the target within 0.5° for 40 s without any wheel reaching full speed.',
      check: ({ sim, prediction }) => {
        const p = sim.params;
        if (p.sim.vehicle !== 'satellite') return 'this lesson flies the satellite';
        if (prediction == null) return 'first the prediction: the wheel absorbs the torque';
        const truth = wheelFullAt(p);
        if (Math.abs(prediction - truth) > 0.05 * truth)
          return 'not within 5 %: momentum is torque times time';
        if (peakWheel(sim) >= p.satellite.wheelMomentum - 1e-6)
          return 'a wheel reached full speed and the target was lost. Press R';
        if (sim.satPeak > IV19.limitDeg)
          return `the pointing error reached ${sim.satPeak.toFixed(2)}°. Press R`;
        return sim.t >= IV19.until || `holding: ${sim.t.toFixed(0)} of ${IV19.until} s`;
      },
    },
    solution: (p) => {
      p.satellite.dump = true;
      p.satellite.dumpFeedforward = true;
    },
    body: (
      <>
        <p>
          Sunlight pushes harder on one side of a satellite than on the other, and gravity pulls
          harder on its near end. The result is a small torque that never stops. Here it is 0.02 N·m
          about z, about a hundred times stronger than in a real orbit, so that it acts within a
          minute.
        </p>
        <p>
          The wheels hold the attitude by taking the torque on themselves: the z wheel speeds up at
          exactly the rate the disturbance pushes, <Tex>{'h(t) = \\tau_d\\,t'}</Tex>. The chart
          shows its momentum climbing towards the limit. Once the wheel is at full speed it cannot
          give any more torque, and the satellite turns away.
        </p>
        <p>
          The wheels only move momentum around; they cannot throw it away. A torque from outside
          can: a pair of thrusters, or magnetic torquers pushing against the Earth's field. Firing
          against the stored momentum, <Tex>{'\\dot H = \\tau_{thr}'}</Tex>, drains the wheel. This
          is <b>momentum dumping</b>.
        </p>
        <Try>
          Predict when the wheel is full and enter it. Press <b>R</b> and watch the target go. Then
          turn on <b>Satellite → Dump momentum</b>, and after that <b>Tell the wheels</b>.
        </Try>
        <Notice>
          Dumping alone loses the target in another way. The attitude loop sees the thruster's 0.08
          N·m as one more disturbance, and the spring gives way until it pushes back: the error
          jumps to 0.8°. The thruster torque is known, so the wheels can be told to cancel it
          directly (a feedforward), and the pointing never notices. The 0.3° that is left is the
          disturbance against a PD with no integral.
        </Notice>
      </>
    ),
  },
  {
    id: 'thrusters',
    n: 20,
    part: 4,
    chapter: N,
    title: 'On or off',
    level: 1,
    loop: 'point.x',
    chart: 'phase',
    setup: (p) => {
      p.sim.vehicle = 'satellite';
      p.satellite.actuator = 'thrusters';
      p.satellite.slewAxis = 'x';
      p.satellite.slewDeg = 2;
    },
    events: (sim) => {
      sim.satSince = IV20.from;
    },
    predict: {
      label: 'Fuel rate of the limit cycle',
      unit: 'g/min',
      truth: (p) => limitCycleFuelRate(p.satellite) * 60,
      tolerance: 0.1,
    },
    goal: {
      text: 'Predict the fuel rate of the limit cycle for your settings. Then point within 0.5° from 15 s on, using at most 30 % more fuel than the least possible.',
      check: ({ sim, prediction }) => {
        const p = sim.params;
        if (p.sim.vehicle !== 'satellite' || p.satellite.actuator !== 'thrusters')
          return 'this lesson flies the satellite on its thrusters';
        if (prediction == null)
          return 'first the prediction: drift across the band, two pulses per period';
        const truth = limitCycleFuelRate(p.satellite) * 60;
        if (Math.abs(prediction - truth) > 0.1 * truth)
          return 'not within 10 %: count the time of the pulses as well as the drift';
        if (sim.satPeak > IV20.limitDeg)
          return `the angle reached ${sim.satPeak.toFixed(2)}°: band plus overshoot must stay within ${IV20.limitDeg}°`;
        if (sim.t < IV20.until) return `measuring the fuel: ${sim.t.toFixed(0)} of ${IV20.until} s`;
        const rate = fuelRate(sim);
        const least = leastFuel(p);
        return (
          rate <= IV20.slack * least ||
          `${rate.toFixed(1)} g/min; the least for ${IV20.limitDeg}° is ${least.toFixed(1)} g/min`
        );
      },
    },
    solution: (p) => {
      p.satellite.deadbandDeg = 0.45;
      p.satellite.driftRateDeg = 0;
    },
    body: (
      <>
        <p>
          Thrusters are valves: open or shut. They cannot give a little torque, so they cannot hold
          an angle exactly. Instead they let it wander inside a <b>dead band</b> of ±db. When the
          angle reaches the edge and is still moving outward, a pulse fires until it drifts back at
          a chosen rate <Tex>{'\\omega'}</Tex>. Then it crosses the band to the other edge, and the
          same happens there.
        </p>
        <p>
          In the phase portrait this is a <b>limit cycle</b>: two horizontal lines at{' '}
          <Tex>{'\\pm\\omega'}</Tex> joined at the walls. Its geometry gives everything. A pulse
          that turns <Tex>{'+\\omega'}</Tex> into <Tex>{'-\\omega'}</Tex> lasts{' '}
          <Tex>{'t_p = 2J\\omega/\\tau'}</Tex>, and in that time the angle goes out and comes back
          to the edge. Crossing the band takes <Tex>{'2\\,db/\\omega'}</Tex>. So
        </p>
        <Tex display>
          {'T = \\frac{4\\,db}{\\omega} + 2t_p, \\qquad F = \\frac{2\\,t_p\\,\\dot m}{T}'}
        </Tex>
        <p>
          The pair gives 0.5 N·m and burns 1 g/s while it fires, and <Tex>{'J_x'}</Tex> = 4 kg·m².
          The valves cannot open for less than 0.1 s, so the slowest drift they can set is{' '}
          <Tex>{'\\omega_{min} = \\tau\\,\\Delta t_{min}/2J'}</Tex> = 0.36°/s; asking for less gives
          that.
        </p>
        <Try>
          Compute the fuel rate for the settings in <b>Satellite</b> and enter it. Then find the
          cheapest settings that still keep the angle within 0.5°. The goal measures the fuel from
          15 s to 40 s.
        </Try>
        <Notice>
          Fuel goes as <Tex>{'\\omega^2/db'}</Tex> while the pulses are short. A wider band saves
          fuel in proportion; a slower drift saves it twice over. The narrow band and fast drift the
          satellite starts with burn about sixteen times the least. The pointing pays for it: a
          telescope that needs 0.01° cannot live on thrusters, and that is why the wheels exist.
        </Notice>
      </>
    ),
  },
  {
    id: 'panel',
    n: 21,
    part: 4,
    chapter: N,
    title: 'Panels that wave',
    level: 1,
    loop: 'point.z',
    chart: 'panel',
    setup: (p) => {
      p.sim.vehicle = 'satellite';
      p.satellite = {
        ...p.satellite,
        slewAxis: 'z',
        slewDeg: IV21.deg,
        kp: 4,
        kd: 4,
        profileTorque: 0.15,
        panel: { inertia: 0.3, hz: 0.5, zeta: 0.005, sensor: 'hub' },
        shaper: 'none',
        shaperHz: 0.5,
      };
    },
    predict: {
      label: "The panel's frequency in flight",
      unit: 'Hz',
      truth: (p) => panelFreeHz(p.satellite),
      tolerance: 0.03,
    },
    goal: {
      text: `Predict the frequency at which the panel swings on the free satellite. Then shape the slew so that from ${IV21.from} s on the panel swings less than ${IV21.ratio * 100} % as much as after the unshaped slew, with the sensor on the hub.`,
      check: ({ sim, prediction }) => {
        const p = sim.params;
        if (p.sim.vehicle !== 'satellite' || !(p.satellite.panel.inertia > 0))
          return 'this lesson flies the satellite with its panel';
        if (prediction == null) return 'first the prediction: does the hub stay still?';
        const truth = panelFreeHz(p.satellite);
        if (Math.abs(prediction - truth) > 0.03 * truth)
          return 'not within 3 %: the hub is not clamped; it swings against the panel';
        if (p.satellite.panel.sensor !== 'hub') return 'put the sensor back on the hub';
        if (sim.t < IV21.until) return `watching the panel: ${sim.t.toFixed(0)} of ${IV21.until} s`;
        const now = panelSwing(sim);
        const ref = unshaped(p);
        return (
          now <= IV21.ratio * ref ||
          `the panel still swings ${now.toFixed(3)}°, ${((now / ref) * 100).toFixed(0)} % of the unshaped ${ref.toFixed(3)}°`
        );
      },
    },
    solution: (p) => {
      p.satellite.shaper = 'zv';
      p.satellite.shaperHz = panelFreeHz(p.satellite);
    },
    body: (
      <>
        <p>
          Solar panels are large, light and flexible. Here one panel sits on a hinge about the z
          axis with a spring and almost no damping (ζ 0.005): held on a clamped hub it would swing
          at 0.5 Hz. The satellite slews {IV21.deg}° about z in the classic way: full torque one
          way, then the other, at 0.15 N·m, with that torque fed forward and the quaternion feedback
          of lesson IV.18 tracking the profile. The panel feels each switch of the torque as a kick
          and is left ringing when the slew is over.
        </p>
        <p>
          The fix is <b>input shaping</b> [Singer 1990]. Split the command into two halves, the
          second half a period of the mode later. The swing the first half starts, the second half
          cancels. The ZV shaper does exactly that; the ZVD shaper uses three steps over a whole
          period and tolerates a wrong frequency far better. Both need the frequency the panel
          really has in flight.
        </p>
        <Tex display>
          {
            'J_z\\ddot\\theta = \\tau + k\\eta + c\\dot\\eta, \\qquad J_p(\\ddot\\theta + \\ddot\\eta) = -k\\eta - c\\dot\\eta'
          }
        </Tex>
        <Try>
          The hub has <Tex>{'J_z'}</Tex> = 3 kg·m² and the panel <Tex>{'J_p'}</Tex> = 0.3 kg·m².
          Work out the panel's frequency when the hub is free to swing against it, and enter it.
          Then choose <b>Input shaper</b> and its frequency. Try 0.5 Hz, the clamped frequency, as
          well. Last, set <b>Attitude sensor</b> to <b>on the panel tip</b> and press <b>R</b>.
        </Try>
        <Notice>
          On a free hub the mode is stiffer by <Tex>{'\\sqrt{(J_z + J_p)/J_z}'}</Tex>: 0.524 Hz. A
          ZV shaper there leaves 4 % of the unshaped swing; tuned to 0.5 Hz it leaves 8 %, and the
          ZVD shaper, about ten times less sensitive, leaves 1 % even there. The tip sensor is the
          other half of the lesson. On the hub, the sensor sees what the wheels push on
          (collocated): any positive gains take energy out of the panel, and it cannot go unstable.
          On the tip it sees the panel's bending as well, with the opposite phase above the mode,
          and the same feedback pumps energy in: at every gain tried, from 0.2 to 50, the swing
          grows, until the wheels saturate with the panel near ±20°.
        </Notice>
      </>
    ),
  },
  {
    id: 'startracker',
    n: 22,
    part: 4,
    chapter: N,
    title: 'Stars and gyros',
    level: 1,
    loop: 'point',
    chart: 'startracker',
    setup: (p) => {
      p.sim.vehicle = 'satellite';
      p.satellite.slewDeg = 0;
      p.satellite.navigation = true;
      p.satellite.trackerHz = 0;
      p.satellite.outageStart = IV22.outage;
      p.satellite.outageLength = 10;
    },
    predict: {
      label: 'Knowledge error after 30 s on the gyro alone',
      unit: '″',
      truth: unaidedDrift,
      tolerance: 0.1,
    },
    goal: {
      text: 'Predict how far the estimate drifts in 30 s on the gyro alone. Then know the attitude within 10″ from 15 s to 40 s, through the 10-second outage of the tracker.',
      check: ({ sim, prediction }) => {
        const p = sim.params;
        if (p.sim.vehicle !== 'satellite' || !p.satellite.navigation)
          return 'this lesson runs the attitude estimator of the satellite';
        if (prediction == null) return 'first the prediction: the gyro bias, integrated';
        const truth = unaidedDrift(p);
        if (Math.abs(prediction - truth) > 0.1 * truth)
          return 'not within 10 %: 1 °/h is one arc-second per second';
        if (sim.t < IV22.from) return 'converging…';
        const worst = worstKnowledge(sim);
        if (worst > IV22.limit) return `the knowledge error reached ${worst.toFixed(0)}″. Press R`;
        return (
          sim.t >= IV22.until || `within ${IV22.limit}″: ${sim.t.toFixed(0)} of ${IV22.until} s`
        );
      },
    },
    solution: (p) => {
      p.satellite.trackerHz = 1;
      p.satellite.estimateBias = true;
    },
    body: (
      <>
        <p>
          A telescope must know where it points to a few arc-seconds, a thousandth of a degree. Two
          sensors share the job. The <b>gyro</b> measures the rate a hundred times a second, but its
          bias, a few degrees per hour, adds up. The <b>star tracker</b> photographs the sky and
          finds the attitude from the stars it recognises, to 5″, but only once a second, and not at
          all when the Sun or the Earth is in its view.
        </p>
        <p>
          The filter is the multiplicative EKF of lesson III.23. It integrates the gyro, and each
          star direction is a vector measurement, like gravity was for the drone. The difference is
          a factor of a thousand in accuracy, and that a degree per hour is exactly one arc-second
          per second.
        </p>
        <Try>
          The tracker is off: the estimate rides on the gyro alone. The bias is in <b>Satellite</b>.
          How far will the estimate be off after 30 s? Enter it and press <b>R</b> to check. Then
          set the <b>Star tracker rate</b> to 1 Hz. Is that enough to stay within 10″? If not, turn
          on <b>Estimate the gyro bias</b>.
        </Try>
        <Notice>
          With the tracker on and the bias not estimated, the filter lags about 50″ behind the truth
          while its own 2σ says 6″. It believes the gyro, and every second the gyro is wrong in the
          same direction. It is confidently wrong, as the EKF of lesson IV.27 was. With the bias as
          three more states, the filter learns it to a few tenths of a degree per hour within twenty
          seconds. Then the gyro carries the attitude through the outage almost as well as the stars
          do.
        </Notice>
      </>
    ),
  },
  {
    id: 'cmg',
    n: 23,
    part: 4,
    chapter: N,
    title: 'The singular direction',
    level: 1,
    loop: 'point',
    chart: 'cmg',
    setup: (p) => {
      p.sim.vehicle = 'satellite';
      p.satellite.actuator = 'cmg';
      p.satellite.slewAxis = 'x';
      p.satellite.slewDeg = IV23.deg;
      p.satellite.profileTorque = IV23.torque;
      p.satellite.kp = 4;
      p.satellite.kd = 4;
      p.satellite.cmg = { ...p.satellite.cmg, h0: IV23.h0, steering: 'pinv', parkDeg: 0 };
    },
    predict: {
      label: 'Momentum the slew needs at its midpoint',
      unit: 'N·m·s',
      truth: slewMomentum,
      tolerance: 0.05,
    },
    goal: {
      text: `Predict the momentum the slew needs. Then, with the same four gyros and the same slew (${IV23.deg}° about x on a ±${IV23.torque} N·m profile), settle within ${IV23.tol}° by ${IV23.deadline} s and hold it for ${IV23.hold} s, with the singularity measure never below ${IV23.mMin}.`,
      check: ({ sim, prediction }) => {
        const p = sim.params;
        const sp = p.satellite;
        if (p.sim.vehicle !== 'satellite' || sp.actuator !== 'cmg')
          return 'this lesson flies the satellite on its control-moment gyros';
        if (prediction == null) return 'first the prediction: J·ω at the midpoint of the profile';
        const truth = slewMomentum(p);
        if (Math.abs(prediction - truth) > 0.05 * truth)
          return 'not within 5 %: half the profile at τ gives J·ω = τ·T/2, with T = 2√(Θ·J/τ)';
        if (
          sp.slewAxis !== 'x' ||
          Math.abs(sp.slewDeg) !== IV23.deg ||
          sp.profileTorque < IV23.torque
        )
          return `keep the slew: ${IV23.deg}° about x at ${IV23.torque} N·m`;
        if (sp.cmg.h0 !== IV23.h0) return `keep the gyros: h₀ = ${IV23.h0} N·m·s`;
        const m = leastMeasure(sim);
        if (m < IV23.mMin)
          return `the gimbals came within m = ${m.toFixed(3)} of a singularity. Press R`;
        if (sim.t < IV23.deadline + IV23.hold) return 'slewing…';
        const since = settledSince(sim, IV23.tol);
        if (Number.isNaN(since) || since > IV23.deadline)
          return `not settled within ${IV23.tol}° by ${IV23.deadline} s. Press R`;
        return true;
      },
    },
    solution: (p) => {
      p.satellite.cmg.parkDeg = 60;
    },
    body: (
      <>
        <p>
          Wheels are gentle: 0.2 N·m here, enough for a telescope, far too little for a spacecraft
          that has to turn quickly. A <b>control-moment gyro</b> spins its rotor at a constant speed
          and turns it on a gimbal. Turning a momentum <Tex>{'h_0'}</Tex> at a rate{' '}
          <Tex>{'\\dot\\delta'}</Tex> takes a torque <Tex>{'h_0\\dot\\delta'}</Tex>, so a small
          gimbal motor commands a large torque on the body. Four of them stand on the faces of a
          pyramid, tilted by <Tex>{'\\beta = 54.7°'}</Tex>, each with <Tex>{'h_0 = 1'}</Tex> N·m·s
          [Wie 2008].
        </p>
        <p>
          The cluster's momentum <Tex>{'h(\\delta)'}</Tex> depends on the four gimbal angles, and
          the steering law has to find gimbal rates for the torque the quaternion feedback wants:{' '}
          <Tex>{'\\dot h = A(\\delta)\\,\\dot\\delta'}</Tex>, with <Tex>{'A'}</Tex> a 3 × 4 matrix.
          The pseudoinverse <Tex>{'\\dot\\delta = A^\\top(AA^\\top)^{-1}\\dot h'}</Tex> gives
          exactly the torque asked for, as long as <Tex>{'A'}</Tex> has rank three. Where it does
          not, the four rotors are lined up so that no gimbal motion makes torque along one
          direction: a <b>singularity</b>. The lower chart shows{' '}
          <Tex>{'m = \\det(AA^\\top)/h_0^6'}</Tex>, zero at one.
        </p>
        <p>
          The task is a 90° slew about x on a profile at ±0.5 N·m: full torque one way, then the
          other. At its midpoint the satellite turns fastest, and all of that momentum{' '}
          <Tex>{'J\\omega'}</Tex> is stored in the gyros. Along x the pyramid can hold{' '}
          <Tex>{'h_0(2 + 2\\cos\\beta) = 3.15'}</Tex> N·m·s: plenty. Press Start and watch the
          momentum. It stops at 1.15, the measure falls to zero, the gimbals race at their rate
          limit and the satellite falls behind its profile.
        </p>
        <Try>
          Work out the momentum the slew needs. Then try the <b>CMG steering</b> options: the
          singularity-robust inverse keeps the gimbal rates finite near the singularity by giving
          less torque than asked; with a dither it is pushed out. Then leave the steering alone and
          move the <b>Gimbals parked at s</b>: every <Tex>{'(s, -s, s, -s)'}</Tex> stores zero
          momentum, so the satellite cannot feel the difference.
        </Try>
        <Notice>
          The slew needs <Tex>{'\\sqrt{\\Theta\\tau J} = 1.77'}</Tex> N·m·s, between the 1.15 where
          the gimbals stall and the 3.15 of the envelope. From <Tex>{'\\delta = 0'}</Tex> two rotors
          point along x in opposite directions, and the symmetric path the pseudoinverse takes never
          breaks their cancellation; the other two can give at most{' '}
          <Tex>{'2h_0\\cos\\beta = 1.15'}</Tex>. That is an internal singularity, and the torque
          asked for points exactly along its lost direction. The SR inverse cannot leave it (it only
          stops the gimbals racing), and the dither leaves it too slowly: the slew still takes 14 to
          18 s. Parked at s between 45° and 75°, the same pseudoinverse takes the momentum along a
          path with no singularity on it (the measure stays above 0.5) and the slew is done in 7 s.
          Moving the gimbals along the zero-momentum family is <b>null motion</b>; real steering
          laws use it all the time, to keep the cluster away from the singular directions before
          they are needed.
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
    id: 'rendezvous',
    n: 25,
    part: 4,
    chapter: O,
    title: 'Falling around together',
    level: 1,
    chart: 'rendezvous',
    loop: 'along',
    setup: (p) => {
      p.sim.vehicle = 'chaser';
      p.wind.enabled = false;
    },
    predict: {
      label: 'How much further behind after one orbit of coasting',
      unit: 'm',
      truth: fallsBehind,
      tolerance: 0.05,
    },
    goal: {
      text: `Predict where the first burn leaves the chaser after one orbit, then dock: under 5 cm/s, within one orbit, on at most ${IV25.budget} m/s of Δv in all.`,
      check: ({ sim, prediction }) => {
        const c = sim.chaser;
        if (!c) return 'fly the chaser';
        const truth = fallsBehind(sim.params);
        if (prediction == null) return 'first the prediction: y(t) = (4Δv/n)·sin nt − 3Δv·t';
        if (Math.abs(prediction - truth) > 0.05 * Math.abs(truth))
          return 'not within 5 %: put t = one period into the along-track term';
        if (c.hit) return `it hit the target at ${(c.contactSpeed * 100).toFixed(0)} cm/s`;
        const lap = sim.params.rendezvous.periodMin * 60;
        if (!c.docked) return c.t > lap ? 'an orbit has gone by without docking' : 'approaching…';
        if (c.t > lap) return 'docked, but it took more than an orbit';
        if (c.dv > IV25.budget) return `docked on ${c.dv.toFixed(2)} m/s of Δv: over the budget`;
        return true;
      },
    },
    solution: (p) => {
      p.rendezvous.burn = 0;
      p.rendezvous.control = 'mpc';
      p.rendezvous.rAcc = 1e6;
    },
    body: (
      <>
        <p>
          A supply ship is 100 m behind a space station, in the same circular orbit, 400 km up. The
          station is ahead, so the obvious move is to point at it and push: a burn of 5 cm/s
          forwards, then coast, and it should arrive in half an hour. In the station’s own frame (x
          up, y along the flight) the motion obeys the <b>Clohessy–Wiltshire</b> equations [Clohessy
          1960], with n the orbital rate (one lap in 92.6 minutes):
        </p>
        <Tex
          display
        >{String.raw`\ddot x = 3n^2x + 2n\dot y + a_x, \qquad \ddot y = -2n\dot x + a_y`}</Tex>
        <p>
          Faster means a higher orbit, and a higher orbit is a slower one. From rest, a forward burn
          Δv gives <Tex>{String.raw`x(t) = \tfrac{2\Delta v}{n}(1-\cos nt)`}</Tex> and{' '}
          <Tex>{String.raw`y(t) = \tfrac{4\Delta v}{n}\sin nt - 3\Delta v\,t`}</Tex>: the ship
          rises, slows down and falls behind. Time runs 120 times faster here, so one orbit takes 46
          seconds.
        </p>
        <Try>
          Predict how much further behind one orbit leaves it, and press <b>Start</b>. Then set{' '}
          <b>After the burn</b> to MPC: every minute it plans the thrust for the next hour with the
          same equations and applies the first minute. It docks, but look at the Δv. Raise the{' '}
          <b>MPC thrust weight</b> by factors of 100, and drop the naive first burn.
        </Try>
        <Notice>
          The equations are linear, so a rendezvous is a linear control problem, and an MPC from
          Part II flies it: the hard part is the model, not the controller. A cheap plan uses the
          orbit: it lets the drift carry the ship and pushes only for the difference. A very cheap
          one is slow: with the naive burn still in, a weight of 10⁷ takes longer than an orbit.
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
  {
    id: 'particle',
    n: 28,
    part: 4,
    chapter: O,
    title: 'Many guesses at once',
    level: 3,
    chart: 'cloud',
    loop: 'pos.x',
    setup: (p) => {
      p.wind.enabled = false;
      p.sensors.beacons = true;
      // Centred on the beacon and ±15 m wide: the filter knows only that the drone is near it.
      p.control.beaconFilter = {
        kind: 'pf',
        guessX: BEACONS[0]!.x,
        guessZ: 0,
        sigma0: 5,
        start: 10,
      };
    },
    events: (sim) => setAt(sim, IV28.leg, 'setpoint.z', IV28.legZ),
    predict: {
      label: 'x of the mirror image after the straight leg',
      unit: 'm',
      truth: ghostX,
      tolerance: 0.05,
    },
    goal: {
      text: `Predict where the second guess sits after the straight leg, then make the cloud settle on one: from ${IV28.from} s to ${IV28.until} s the estimate within ${IV28.limit} m of the truth and the cloud’s σ below ${IV28.limit} m.`,
      check: ({ sim, prediction }) => {
        const p = sim.params;
        if (!p.sensors.beacons || p.control.beaconFilter.kind !== 'pf')
          return 'keep the beacon and the particle filter';
        if (prediction == null)
          return 'first the prediction: mirror the drone in the line through the beacon along the leg';
        if (Math.abs(prediction - ghostX(p)) > 1)
          return 'not within 1 m: the line runs through the beacon, parallel to z';
        if (sim.t < IV28.until) return 'filtering…';
        const c = worstCloud(sim, IV28.from, IV28.until);
        if (c.err > IV28.limit)
          return `the estimate was ${c.err.toFixed(1)} m off: the cloud still holds more than one guess`;
        if (c.sigma > IV28.limit) return `the cloud is still ${c.sigma.toFixed(1)} m wide`;
        return true;
      },
    },
    solution: (p) => {
      p.setpoint.profile = 'circle';
      p.setpoint.profileAmplitude = 1;
      p.setpoint.profilePeriod = 20;
    },
    body: (
      <>
        <p>
          The beacon of lesson IV.27 again, in calm air, but now the drone also knows its own
          velocity (an optical-flow camera looking at the ground, good to 5 cm/s), not where it is.
          A <b>particle filter</b> [Arulampalam 2002] does not keep a mean and a covariance. It
          keeps three thousand guesses, the particles, spread at first over a 30 m square around the
          beacon. Each step moves every particle by the measured velocity, and each range weights
          them by how well they explain it. When a few carry most of the weight, the cloud is
          redrawn from the weights.
        </p>
        <p>
          While the drone hovers, every point at the right distance explains the ranges equally
          well, and the cloud becomes a <b>ring</b> around the beacon. Its mean is near the beacon,
          10 m off, and its σ says so: the filter does not pretend. At 15 s the drone flies a
          straight leg of 4 m along z. Every particle moves the same way, and the ranges change
          differently for each one, so most of the ring dies out. But not all: a particle at the{' '}
          <b>mirror image</b> of the drone, across the line through the beacon along the leg, sees
          exactly the same ranges. Two guesses survive, and the mean sits between them.
        </p>
        <Try>
          Before the leg, predict the x of the mirror image (the drone hovers at x = 0; the beacon
          is at x = 10). Then watch the cloud split. Set <b>Setpoint → Automatic motion</b> to a
          circle and press <b>R</b>: a turn is not symmetric about any line, and the cloud keeps one
          guess.
        </Try>
        <Notice>
          The EKF of lesson IV.27 had to choose one point on the ring at the first range, and it
          chose it by the shape of its prior, not by the data. The particles keep every hypothesis
          the data allow until the data rule them out: a belief with two peaks is a belief a
          Gaussian cannot hold. The price is three thousand range predictions per update instead of
          one, and a cloud that can lose a hypothesis by chance when it is redrawn: watch the two
          halves of the ring drift apart in weight while the drone hovers.
        </Notice>
      </>
    ),
  },
  {
    id: 'smoother',
    n: 29,
    part: 4,
    chapter: O,
    title: 'Hindsight',
    level: 1,
    chart: 'smoother',
    setup: (p) => {
      p.sensors.posNoise = IV29.noise;
      p.smoother = { enabled: true, accelSigma: IV29.start };
      p.setpoint.profile = 'square';
      p.setpoint.profileAmplitude = 0.5;
      p.setpoint.profilePeriod = 8;
    },
    predict: {
      label: 'σ of the smoother ÷ σ of the filter, mid-record, slow model',
      unit: '',
      truth: () => 0.5,
      tolerance: 0.05,
    },
    goal: {
      text: `Predict how much smaller the smoother’s σ is than the filter’s in the middle of a long record, then tune the model so that over ${IV29.until} s the smoother’s RMS error is below ${IV29.rms * 100} cm and at most ${IV29.ratio * 100} % of the filter’s.`,
      check: ({ sim, prediction }) => {
        if (!sim.params.smoother.enabled) return 'keep the reconstruction on';
        if (prediction == null)
          return 'first the prediction: a line fitted to a record is surest in its middle';
        if (Math.abs(prediction - 0.5) > 0.05)
          return 'not within 5 %: compare the end of a fitted line with its middle';
        if (sim.t < IV29.until) return 'recording…';
        const r = reconstructAltitude(sim);
        if (!r) return 'the altimeter must be noisy for this';
        if (r.rmsS > IV29.rms)
          return `the smoother is ${(r.rmsS * 100).toFixed(2)} cm RMS: the model follows the noise`;
        if (r.rmsS > IV29.ratio * r.rmsF)
          return `the smoother is ${((100 * r.rmsS) / r.rmsF).toFixed(0)} % of the filter`;
        return true;
      },
    },
    solution: (p) => {
      p.smoother.accelSigma = IV29.good;
    },
    body: (
      <>
        <p>
          After a test flight nobody is in a hurry. The recorded altimeter readings (5 cm of noise,
          200 a second) can be read forwards and backwards, and the best estimate of the altitude at
          12 s may use the readings at 13 s. A <b>Kalman filter</b> running forward knows only the
          past; the <b>Rauch–Tung–Striebel smoother</b> [Rauch 1965; Simon 2006] then runs backward
          over the filter’s results and adds what the future says about each instant. This is how
          flight-test data are reconstructed.
        </p>
        <p>
          Both use the same model: a constant velocity, bent by a random acceleration of a given
          size. Small, and the model trusts a straight line through many readings; large, and it
          follows every reading, noise and all. The chart below shows both errors against the truth,
          each with its own σ dashed. The smoother’s σ is smaller everywhere except at the very end,
          where there is no future to add.
        </p>
        <Try>
          Predict the ratio first. A slow model is a straight line fitted to the readings around
          each instant: the filter sees them only on one side, and a fitted line is least sure at
          its end. The smoother sees both sides and evaluates the line in its middle. Then lower{' '}
          <b>Model acceleration noise</b> from 100 towards 1 and watch both errors shrink; go on to
          0.01 and watch the filter lag behind every step of the setpoint.
        </Try>
        <Notice>
          For a slow model the ratio tends to ½ whatever the noise and the rate: it is geometry. For
          a line fitted to N points the variance at its end is four times that in its middle, so the
          σ is twice. The smoother cannot fly the drone, since it needs the future, but it is the
          reference every filter is judged against, and how the truth is known after a flight that
          had no truth to compare with.
        </Notice>
      </>
    ),
  },
];
