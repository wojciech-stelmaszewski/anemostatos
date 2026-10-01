import { criticalDiveSpeed } from '@/analysis/attraction';
import { analyseRateLoop, type RateLoopAnalysis } from '@/analysis/mimo';
import { aliasHz, ghostRate, motorJitter } from '@/analysis/spectrum';
import { cleanRunsNeeded, finishedCampaign, passRateBound } from '@/analysis/dispersion';
import {
  bandwidthHz,
  closedLoopPoles,
  isLoopStable,
  l1Loop,
  loopGain,
  sensitivities,
} from '@/analysis/loop';
import type { Simulation } from '@/engine/simulation';
import { cabs } from '@/math/complex';
import { logspace, margins, type Margins } from '@/math/margins';
import { robustTest } from '@/analysis/uncertainty';
import { v3 } from '@/math/vec3';
import { GRAVITY, type Params } from '@/sim/params';
import { Notice, Try } from './Bits';
import { M } from './Math';
import { setAt } from './script';
import type { Lesson } from './types';

/**
 * Part III — the third semester: why loops work (docs/analysis.md §6).
 * Parts I and II design controllers; Part III predicts what a design will do and checks the
 * prediction against the simulator. Numbering restarts at III.1 in the lesson panel.
 */

const F = 'F · The loop as a filter';
const G = 'G · Stability and structure';
const H = 'H · Robust by construction';
const I = 'I · The real loop';
const J = 'J · When things break, and how we know';
const TWO_PI = 2 * Math.PI;

const calm = (p: Params) => {
  p.wind.enabled = false;
};
const smallSteps = (p: Params) => {
  p.setpoint.profile = 'square';
  p.setpoint.profileAmplitude = 0.3;
  p.setpoint.profilePeriod = 8;
};

/** Margins of the altitude loop for these parameters (null if the controller has no model). */
const marginsOf = (p: Params): Margins | null => {
  const m = l1Loop(p);
  if (!m) return null;
  const w = logspace(TWO_PI * 0.02, (Math.PI / m.T) * 0.98, 1200);
  return margins(
    w,
    w.map((x) => loopGain(m, x)),
  );
};

/** The roll–pitch loop of these parameters, kept until the loop changes. */
let rateLoopMemo: { key: string; value: RateLoopAnalysis | null } | null = null;
const rateLoopOf = (p: Params): RateLoopAnalysis | null => {
  const key = JSON.stringify([p.sim.level, p.control, p.drone, p.sensors, p.vibration.hoverHz]);
  if (rateLoopMemo?.key !== key) rateLoopMemo = { key, value: analyseRateLoop(p, 300) };
  return rateLoopMemo.value;
};
/** Delay at which the drone as handed over (no correction, no delay yet) starts to oscillate, ms. */
const criticalGyroDelay = (p: Params): number => {
  const q = structuredClone(p);
  q.sensors.delayMs = 0;
  q.control.model.imuYawDeg = 0;
  return (analyseRateLoop(q, 300)?.both.delayMargin ?? NaN) * 1000;
};

/** The IMU rate lesson III.20 starts with, Hz: too slow for rotors at 110 Hz. */
const SLOW_IMU_HZ = 100;

/** The manoeuvre of lesson III.22: hover, then four laps of a circle. */
const TURN = { radius: 3, period: 5, start: 10, end: 30 };
/** By how many per cent the accelerometer reads more than 1 g in the steady turn. */
const turnExcess = (p: Params): number => {
  const w = (2 * Math.PI) / p.setpoint.profilePeriod;
  const a = p.setpoint.profileAmplitude * w * w;
  return (Math.hypot(1, a / GRAVITY) - 1) * 100;
};

/** Half the peak-to-peak swing of the altitude over the last `seconds`. */
const swing = (sim: Simulation, seconds: number): number => {
  const { series } = sim.telemetry.window(['pos.y'], sim.t - seconds);
  const y = series[0]!.filter((v) => !Number.isNaN(v));
  return y.length ? (Math.max(...y) - Math.min(...y)) / 2 : 0;
};

/** The oscillating pole pair closest to the origin: the one that shapes the response. */
const dominantPair = (p: Params) => {
  const m = l1Loop(p);
  if (!m) return null;
  const pair = closedLoopPoles(m)
    .filter((z) => z.im > 1e-6)
    .sort((a, b) => Math.hypot(a.re, a.im) - Math.hypot(b.re, b.im))[0];
  if (!pair) return { zeta: 1, wn: 0, stable: isLoopStable(m), pair: null };
  const wn = Math.hypot(pair.re, pair.im);
  return { zeta: -pair.re / wn, wn, stable: isLoopStable(m), pair };
};

/** The fastest dive the drone survives, flown headlessly; cached, because goals poll it. */
let diveCache: { key: string; v: number } | null = null;
const criticalDive = (p: Params): number => {
  const key = JSON.stringify([p.control, p.drone, p.setpoint.y, p.sensors]);
  if (diveCache?.key !== key) diveCache = { key, v: criticalDiveSpeed(p, 0) };
  return diveCache.v;
};
/** The small-gain test of lesson III.11; cached, because goals poll it. */
let robustCache: { key: string; worst: number } | null = null;
const worstWT = (p: Params): number | null => {
  const key = JSON.stringify([p.control, p.drone, p.sensors, p.uncertainty]);
  if (robustCache?.key !== key) {
    const t = robustTest(p);
    if (!t) return null;
    robustCache = { key, worst: t.worst };
  }
  return robustCache.worst;
};
/** Downward speeds of the scripted dives of lesson III.7, m/s. */
const DIVES = [0.8, 1.4, 2.0, 2.6];

/** Share of the last `seconds` in which the true altitude lay inside the filter's ±2σ band. */
const coverage = (sim: Simulation, seconds: number): number => {
  const { series } = sim.telemetry.window(['pos.y', 'est.y', 'est.sigma.y'], sim.t - seconds);
  const [y, est, sigma] = series as [number[], number[], number[]];
  let n = 0;
  let inside = 0;
  for (let k = 0; k < y.length; k++) {
    if (Number.isNaN(est[k]!) || Number.isNaN(sigma[k]!)) continue;
    n++;
    if (Math.abs(y[k]! - est[k]!) <= 2 * sigma[k]!) inside++;
  }
  return n ? inside / n : 0;
};

export const PART_THREE: Lesson[] = [
  {
    id: 'bode',
    part: 3,
    chapter: F,
    title: 'The drone as a filter',
    level: 1,
    chart: 'bode',
    bode: 'ref',
    setup: (p) => {
      calm(p);
      p.probe = { ...p.probe, point: 'ref.y', signal: 'sine', amp: 0.3, freqHz: 0.1 };
    },
    goal: {
      text: 'Find the bandwidth: the probe frequency at which the drone swings 71 % as far as the setpoint does (−3 dB).',
      check: ({ sim }) => {
        const pr = sim.params.probe;
        if (pr.point !== 'ref.y' || pr.signal !== 'sine') return 'put a sine on the setpoint';
        const window = Math.max(2 / pr.freqHz, 3);
        if (sim.probeAge < window + Math.max(3 / pr.freqHz, 4)) return 'let the swing settle…';
        const ratio = swing(sim, window) / pr.amp;
        return (
          (ratio > 0.67 && ratio < 0.75) ||
          `at ${pr.freqHz.toFixed(2)} Hz the drone swings ${(100 * ratio).toFixed(0)} % of the setpoint's ${pr.amp.toFixed(2)} m`
        );
      },
    },
    solution: (p) => {
      p.probe.freqHz = bandwidthHz(l1Loop(p)!);
    },
    body: (
      <>
        <p>
          Until now we asked how the drone answers a <i>step</i>. Here the setpoint moves as a slow
          sine instead, and the question is: does the drone keep up? A linear loop answers a sine
          with a sine of the <b>same frequency</b>; only its size and its delay change. How they
          change with frequency is the <b>frequency response</b>.
        </p>
        <M display>
          {
            '\\frac{Y}{R}(j\\omega) = \\frac{\\text{swing of the drone}}{\\text{swing of the setpoint}}\\,e^{\\,j\\,\\text{phase lag}}'
          }
        </M>
        <p>
          The chart on the right is a <b>Bode plot</b> of it: size in decibels above (0 dB = same
          size, −20 dB = one tenth), lag in degrees below, against frequency on a logarithmic scale.
          The line is computed from the controller and the physics. The dotted marker is the probe.
        </p>
        <Try>
          Raise <b>Probe → Frequency</b> (the last group of the parameter panel) from 0.1 Hz step by
          step and watch the drone. Then press <b>Measure</b>: the simulator flies fourteen
          frequencies and puts a dot at each. Where do the dots land?
        </Try>
        <Notice>
          The drone is a <b>low-pass filter</b>: slow motion of the setpoint passes, fast motion
          does not. The frequency where the swing has dropped to 71 % is called the <b>bandwidth</b>
          . For this tune it is surprisingly low. Set <b>D acts on → error</b> and it rises sixfold:
          the reason is in Part I, lesson 7.
        </Notice>
      </>
    ),
  },
  {
    id: 'poles',
    part: 3,
    chapter: F,
    title: 'Poles tell the story',
    level: 1,
    chart: 'poles',
    setup: (p) => {
      calm(p);
      smallSteps(p);
      p.control.alt = { ...p.control.alt, iOn: false, kd: 2 };
    },
    goal: {
      text: 'Place the oscillating pole pair on the lines of damping ratio ζ = 0.7 (between 0.65 and 0.75).',
      check: ({ sim }) => {
        const d = dominantPair(sim.params);
        if (!d) return 'this lesson needs the PID controller';
        if (!d.pair) return 'no oscillating pair: every pole is on the real axis (ζ ≥ 1)';
        return (
          (d.stable && d.zeta > 0.65 && d.zeta < 0.75) ||
          `pair at ${d.pair.re.toFixed(2)} ± ${d.pair.im.toFixed(2)}j: ζ = ${d.zeta.toFixed(2)}`
        );
      },
    },
    solution: (p) => {
      p.control.alt.kd = 4.4;
    },
    body: (
      <>
        <p>
          In Part I every step response was a sum of exponentials <M>{'e^{st}'}</M>, one for each{' '}
          <b>pole</b> <M>s</M> of the closed loop. The chart shows them all at once, in the plane of
          complex numbers: to the left of the vertical axis a term decays, to the right it grows,
          and the height above the horizontal axis is the frequency it oscillates at.
        </p>
        <M display>{'s = -\\zeta\\omega_n \\pm j\\,\\omega_n\\sqrt{1-\\zeta^2}'}</M>
        <p>
          The angle from the negative real axis is the damping: the dashed lines mark{' '}
          <M>{'\\zeta = 0.7'}</M>. The poles come from the model of the whole loop, motors and
          sampling included, which is why there are more than two.
        </p>
        <Try>
          Drag <b>Kd</b> from 2 upwards and watch two things together: the pair of poles moving (it
          leaves a trail) and the overshoot of the steps in the tracking chart. Then push <b>Kd</b>{' '}
          past 6: where does the pair go?
        </Try>
        <Notice>
          The trail is a <b>root locus</b>: the path of the poles as one gain changes. Tuning is
          choosing a point on it. The two poles far to the left belong to the motors and the D
          filter; they decay in a few hundredths of a second and do not shape what you see.
        </Notice>
      </>
    ),
  },
  {
    id: 'margins',
    part: 3,
    chapter: F,
    title: 'How far from the edge?',
    level: 1,
    chart: 'bode',
    chart2: 'nyquist',
    setup: (p) => {
      calm(p);
      smallSteps(p);
    },
    goal: {
      text: 'Make the loop as fast as the margins allow: a crossover at 2.5 Hz or more, with a phase margin of at least 45° and a gain margin of at least 6 dB.',
      check: ({ sim }) => {
        const m = marginsOf(sim.params);
        const loop = l1Loop(sim.params);
        if (!m || !loop) return 'this lesson needs a controller with a linear model';
        const fc = m.wc / TWO_PI;
        const gmDb = 20 * Math.log10(m.gm);
        return (
          (isLoopStable(loop) && fc >= 2.5 && m.pmDeg >= 45 && gmDb >= 6) ||
          `crossover ${fc.toFixed(2)} Hz · PM ${m.pmDeg.toFixed(0)}° · GM ${Number.isFinite(gmDb) ? gmDb.toFixed(0) + ' dB' : '∞'}`
        );
      },
    },
    solution: (p) => {
      p.control.alt = { ...p.control.alt, kp: 20, kd: 20 };
    },
    body: (
      <>
        <p>
          Cut the loop open at the thrust command and send a sine in. It travels through the motors,
          the drone, the sensor and the controller and comes back multiplied by the <b>loop gain</b>{' '}
          <M>{'L(j\\omega)'}</M>. If it comes back the same size and exactly upside down,{' '}
          <M>{'L = -1'}</M>, the closed loop can sustain it for ever: that is the edge of stability.
        </p>
        <p>
          The two <b>margins</b> say how far <M>L</M> stays from −1. At the <b>crossover</b>{' '}
          frequency, where <M>{'|L| = 1'}</M>, the <b>phase margin</b> is the angle still missing to
          −180°. Where the phase is −180°, the <b>gain margin</b> is the factor still missing to 1.
          In the Nyquist plot below, both are distances from the curve to the cross at −1.
        </p>
        <Try>
          Raise <b>Kp</b> and <b>Kd</b> to push the crossover up. The loop gets faster and the
          margins shrink. Press <b>Measure</b> whenever you want to check the line against a flight.
        </Try>
        <Notice>
          A phase margin of 45° to 60° and a gain margin of 6 dB are what flight-control
          requirements usually ask for. They are the engineer's version of "not too close to the
          edge", stated as numbers that can be verified before anything flies.
        </Notice>
      </>
    ),
  },
  {
    id: 'delay',
    part: 3,
    chapter: F,
    title: 'Delay is a phase thief',
    level: 1,
    chart: 'bode',
    chart2: 'poles',
    setup: (p) => {
      calm(p);
      smallSteps(p);
    },
    predict: {
      label: 'Critical delay',
      unit: 'ms',
      truth: (p) => {
        const clean = structuredClone(p);
        clean.sensors.delayMs = 0;
        return (marginsOf(clean)?.delayMargin ?? NaN) * 1000;
      },
      tolerance: 0.1,
    },
    goal: {
      text: 'Before touching the delay: predict, within 10 %, how much sensor delay makes this loop unstable.',
      check: ({ sim, prediction }) => {
        const truth = PART_THREE.find((l) => l.id === 'delay')!.predict!.truth(sim.params);
        if (prediction == null) return 'read PM and the crossover frequency, compute, and enter it';
        const off = Math.abs(prediction - truth) / truth;
        return off <= 0.1 || 'not within 10 %: check the units (degrees to radians, Hz to rad/s)';
      },
    },
    solution: () => {},
    body: (
      <>
        <p>
          A delay of <M>{'\\tau'}</M> seconds changes no amplitude: a sine comes out as large as it
          went in, only later. In the frequency response that is a gain of exactly one and a phase
          of
        </p>
        <M display>{'\\angle\\,e^{-j\\omega\\tau} = -\\,\\omega\\,\\tau .'}</M>
        <p>
          So a delay lowers the phase curve, more at high frequency, and leaves the magnitude curve
          alone. The crossover stays where it is and the phase margin shrinks by{' '}
          <M>{'\\omega_c\\tau'}</M>. The loop is at the edge when the margin is used up:
        </p>
        <M display>{'\\tau_{max} = \\frac{PM\\ \\text{[rad]}}{\\omega_c\\ \\text{[rad/s]}}'}</M>
        <Try>
          Read <b>PM</b> and the crossover frequency from the Bode plot, compute{' '}
          <M>{'\\tau_{max}'}</M> and enter it. Only then raise <b>Sensors → Delay</b> towards your
          number and watch the phase curve, the pole map and the drone.
        </Try>
        <Notice>
          This is why fast loops need fast sensors and fast computers: the same delay costs a loop
          with twice the crossover frequency twice the phase. Every filter, every sample-and-hold
          and every bus in a flight computer is, to the loop, a delay.
        </Notice>
      </>
    ),
  },
  {
    id: 'waterbed',
    part: 3,
    chapter: F,
    title: 'The waterbed',
    level: 1,
    chart: 'bode',
    bode: 'waterbed',
    setup: (p) => {
      p.wind.gustsOn = false;
      p.wind.turbSigma = 1.2;
    },
    goal: {
      text: 'Reject slow disturbances five times better, |S| ≤ −14 dB at 0.5 Hz, without letting the peak of |S| rise above 4 dB.',
      check: ({ sim }) => {
        const loop = l1Loop(sim.params);
        const m = marginsOf(sim.params);
        if (!loop || !m) return 'this lesson needs a controller with a linear model';
        const low = 20 * Math.log10(cabs(sensitivities(loop, TWO_PI * 0.5).s));
        const peak = 20 * Math.log10(m.ms);
        return (
          (isLoopStable(loop) && low <= -14 && peak <= 4) ||
          `|S| at 0.5 Hz ${low.toFixed(1)} dB · peak ${peak.toFixed(1)} dB at ${(m.wMs / TWO_PI).toFixed(1)} Hz`
        );
      },
    },
    solution: (p) => {
      p.control.alt = { ...p.control.alt, kp: 40, kd: 14 };
    },
    body: (
      <>
        <p>
          A disturbance that would move the drone by some amount without feedback moves it by{' '}
          <M>{'|S|'}</M> times that amount with feedback. <M>{'S = 1/(1+L)'}</M> is the{' '}
          <b>sensitivity</b>: below 0 dB the loop helps, above 0 dB it makes things <i>worse</i>{' '}
          than no control at all.
        </p>
        <p>
          It would be pleasant to push <M>{'|S|'}</M> down everywhere. Bode proved in 1945 that it
          cannot be done. For a loop like ours
        </p>
        <M display>{'\\int_0^\\infty \\ln|S(j\\omega)|\\,d\\omega = 0 ,'}</M>
        <p>
          so the area below zero always equals the area above it. The chart draws both, on a{' '}
          <i>linear</i> frequency axis, where the areas can be compared by eye.
        </p>
        <Try>
          Raise <b>Kp</b> and <b>Kd</b> to press the curve down at low frequency, and watch the hump
          on its right grow by the same area. Switch the chart to <b>sensitivity S</b> for the usual
          logarithmic view and press <b>Measure</b>.
        </Try>
        <Notice>
          Push a waterbed down in one place and it rises in another. The turbulence in this lesson
          contains all frequencies: the slow part is rejected better as you tune, and the part near
          the peak is amplified. A good design puts the hump where the disturbances are weak and
          keeps it low.
        </Notice>
      </>
    ),
  },
  {
    id: 'lyapunov',
    n: 7,
    part: 3,
    chapter: G,
    title: 'Energy that only goes down',
    level: 1,
    chart: 'lyapunov',
    setup: (p) => {
      calm(p);
      p.setpoint.y = 0.6;
      p.drone.maxMotorThrust = 3.5;
      p.control.alt = { ...p.control.alt, iOn: false };
    },
    events: (sim) => {
      DIVES.forEach((v, i) =>
        sim.schedule(8 + 5 * i, (s) => s.applyImpulse(v3(0, -s.params.drone.mass * v, 0))),
      );
    },
    predict: {
      label: 'Fastest dive survived',
      unit: 'm/s',
      truth: criticalDive,
      tolerance: 0.1,
    },
    goal: {
      text: 'Four dives are coming, each faster than the last. Predict, within 10 %, the downward speed at the setpoint height beyond which the drone touches the floor.',
      check: ({ sim, prediction }) => {
        if (prediction == null)
          return 'work it out from the thrust limit and the height, and enter it';
        const truth = criticalDive(sim.params);
        return (
          Math.abs(prediction - truth) / truth <= 0.1 ||
          'not within 10 %: how hard can the motors brake, and over what distance?'
        );
      },
    },
    solution: () => {},
    body: (
      <>
        <p>Lesson 4 proved that a PD loop comes to rest without solving its equation. The energy</p>
        <M display>
          {
            'V = \\tfrac12 K_p e^2 + \\tfrac12 m\\,\\dot e^2, \\qquad \\dot V = -K_d\\,\\dot e^2 \\le 0'
          }
        </M>
        <p>
          can only fall, so the state sinks through the level sets of <M>V</M>, the grey ellipses,
          towards the centre. Such a <M>V</M> is a <b>Lyapunov function</b>. But the proof assumed
          that the motors deliver what the controller asks. These motors are weak: they can brake a
          fall with at most
        </p>
        <M display>{'a_{max} = \\frac{4 f_{max}}{m} - g .'}</M>
        <p>
          The <b>blue</b> ellipse is the largest level set in which the command never saturates:
          inside it the proof holds. The <b>orange</b> line is the truth, flown by the simulator
          from a grid of starting states: above it the drone touches the floor. The dashed curve is
          what the motors alone allow, <M>{'\\dot e^2 = 2a_{max}(h - e)'}</M>; the air helps a
          little, so the flown line can lie above it.
        </p>
        <Try>
          A drone diving through the setpoint height <M>h</M> above the floor at speed <M>v</M>{' '}
          needs the distance <M>{'v^2/(2a_{max})'}</M> to stop at full thrust. Compute the speed for
          which that distance is <M>h</M>, enter it, and watch the four dives.
        </Try>
        <Notice>
          The proven region is a small part of the true one: a Lyapunov function gives a{' '}
          <i>guarantee</i>, not the boundary. Everything between blue and orange works, without a
          proof. Watch a fast dive cross the grey ellipses <i>outwards</i>: while the thrust is
          saturated, <M>V</M> rises.
        </Notice>
      </>
    ),
  },
  {
    id: 'observable',
    n: 9,
    part: 3,
    chapter: G,
    title: "What the filter can't see",
    level: 1,
    chart: 'estimate',
    chart2: 'covariance',
    setup: (p) => {
      calm(p);
      p.sensors.posNoise = 0.05;
      p.sensors.posRateHz = 50;
      p.sensors.accNoise = 0.3;
      p.sensors.accBias = 0.2;
      p.sensors.altBias = 0.3;
      p.control.l1.estimator = 'kalman';
    },
    events: (sim) => {
      setAt(sim, 20, 'sensors.posDropout', true);
      setAt(sim, 25, 'sensors.posDropout', false);
    },
    goal: {
      text: 'Make the filter honest: from t = 12 s on, the true altitude must lie inside its ±2σ band at least 95 % of the time.',
      check: ({ sim }) => {
        if (sim.t < 27) return 'wait for the dropout at 20 s to pass…';
        const c = coverage(sim, 15);
        return c >= 0.95 || `the truth was inside the band ${(100 * c).toFixed(0)} % of the time`;
      },
    },
    solution: (p) => {
      p.control.kalman.altBiasState = true;
    },
    body: (
      <>
        <p>
          The altimeter of this drone reads 30 cm too high. The Kalman filter of lesson II.4
          averages the noise away, reports the altitude to within a centimetre or two, and is wrong
          by thirty. It is <i>confident and wrong</i>: the true altitude is far outside its ±2σ
          band.
        </p>
        <p>
          Can the filter estimate the altimeter's bias, as it estimates the accelerometer's? Give it
          the bias as a state and ask what the measurements reveal. The altimeter reads
        </p>
        <M display>{'z = y + b_y ,'}</M>
        <p>
          only ever the sum. Raise <M>y</M> and lower <M>{'b_y'}</M> by the same amount and no
          measurement changes. The state is <b>not observable</b>: the info card shows the rank of
          the observability matrix, 3 for 4 states.
        </p>
        <Try>
          Switch on <b>Kalman filter → Altimeter-bias state</b>. The bias cannot be estimated, and
          the chart below shows it: its σ never falls. But the band around the estimate is now wide
          enough to contain the truth. At 20 s the altimeter drops out for five seconds: watch every
          σ grow while the filter coasts on the accelerometer.
        </Try>
        <Notice>
          An unobservable state cannot be fixed by tuning or by more data of the same kind. It needs
          a sensor that sees the states differently. What the fourth state buys is honesty: a filter
          that knows what it does not know. Navigation systems are full of such states.
        </Notice>
      </>
    ),
  },
  {
    id: 'smallgain',
    n: 11,
    part: 3,
    chapter: H,
    title: 'How wrong can I be?',
    level: 1,
    chart: 'bode',
    bode: 'robust',
    setup: (p) => {
      calm(p);
      smallSteps(p);
      p.control.alt = { ...p.control.alt, kp: 50, kd: 30 };
    },
    goal: {
      text: 'Retune until the test guarantees the whole family (|W·T| < 1 at every frequency), and keep the loop fast: a crossover of at least 2.7 Hz.',
      check: ({ sim }) => {
        const worst = worstWT(sim.params);
        const m = marginsOf(sim.params);
        if (worst === null || !m) return 'this lesson needs a controller with a linear model';
        const fc = m.wc / TWO_PI;
        return (
          (worst < 1 && fc >= 2.7) ||
          `largest |W·T| ${worst.toFixed(2)} · crossover ${fc.toFixed(2)} Hz`
        );
      },
    },
    solution: (p) => {
      p.control.alt = { ...p.control.alt, kp: 20, kd: 20 };
    },
    body: (
      <>
        <p>
          This tune flies well: the nominal drone has 35° of phase margin. But no real drone is the
          nominal one. Its mass may be off by 30 %, its motors may be twice as slow, its sensor may
          be 20 ms late. That is not one plant but a <b>family</b> of them, and a design is only as
          good as its worst member.
        </p>
        <p>
          Write every member's loop as the nominal one times a relative error,{' '}
          <M>{'L_\\Delta = L\\,(1 + \\Delta)'}</M>, and let <M>{'|W(j\\omega)|'}</M> be the largest
          error any member has at each frequency. The <b>small-gain theorem</b> then gives a test on
          the nominal loop alone:
        </p>
        <M display>
          {
            '|W(j\\omega)\\,T(j\\omega)| < 1 \\ \\text{for all } \\omega \\ \\Rightarrow\\ \\text{all members stable}'
          }
        </M>
        <p>
          The chart draws it as a ceiling: <M>{'|T|'}</M> must stay below <M>{'1/|W|'}</M>. Where it
          pokes through, the chart turns red.
        </p>
        <Try>
          Press <b>Fly 60 members</b>: sixty drones drawn at random from the family, ten seconds
          each. Then lower <b>Kp</b> and <b>Kd</b> until the curve fits under the ceiling, and fly
          the family again. The ranges are in <b>Uncertainty</b>, near the end of the parameter
          panel.
        </Try>
        <Notice>
          Only three of sixty random drones fail here, and they sit in one corner of the family: a
          light drone (the same thrust accelerates it more, so the loop gain is higher) with a late
          sensor. Random sampling finds such corners by luck; the test finds them by construction,
          without flying anything. The price is that the test is <i>sufficient</i>, not necessary:
          it may reject a design that every member would fly.
        </Notice>
      </>
    ),
  },
  {
    id: 'mimo',
    n: 12,
    part: 3,
    chapter: H,
    title: 'One loop at a time lies',
    level: 3,
    chart: 'mimo',
    loop: 'rate.roll',
    setup: (p) => {
      calm(p);
      p.sensors.imuYawDeg = 30;
      p.setpoint.profile = 'square';
      p.setpoint.profileAxis = 'x';
      p.setpoint.profileAmplitude = 1;
      p.setpoint.profilePeriod = 6;
    },
    predict: {
      label: 'Critical delay',
      unit: 'ms',
      truth: criticalGyroDelay,
      tolerance: 0.15,
    },
    goal: {
      text: 'Predict, within 15 %, the sensor delay at which this drone starts to shake. Then set the delay to 25 ms and make the drone fly it, with at least 20° of phase margin for both channels at once.',
      check: ({ sim, prediction }) => {
        const p = sim.params;
        if (prediction == null)
          return 'first the prediction: use the margin for both channels at once, τ = PM / ω_c';
        const truth = criticalGyroDelay(p);
        if (Math.abs(prediction - truth) / truth > 0.15)
          return 'not within 15 %: PM in radians, divided by the crossover in rad/s';
        if (p.sensors.delayMs < 25) return 'now set Sensors → Delay to 25 ms';
        const a = rateLoopOf(p);
        if (!a) return 'this lesson needs the PID cascade';
        if (sim.state.crashed) return 'crashed: press R to fly again';
        return (
          (a.stable && a.both.pmDeg >= 20 && sim.t > 6) ||
          (a.stable
            ? `both channels at once: PM ${a.both.pmDeg.toFixed(0)}°`
            : 'the loop is unstable')
        );
      },
    },
    solution: (p) => {
      p.sensors.delayMs = 25;
      p.control.model.imuYawDeg = 30;
    },
    body: (
      <>
        <p>
          Someone mounted the sensor board of this drone turned by 30° about the vertical axis. The
          gyro now reports part of every roll rate as pitch rate and the other way round, so roll
          and pitch are no longer two loops. They are <b>one loop with two channels</b>, and its
          loop gain is a matrix:
        </p>
        <M display>{'L(j\\omega) = a(j\\omega)\\,R(\\theta) + b(j\\omega)\\,I'}</M>
        <p>
          Here <M>{'R(\\theta)'}</M> is the rotation by the mounting angle, <M>{'a'}</M> is the path
          through the gyro, which it turns, and <M>{'b'}</M> the path through the attitude, which is
          not turned. The classical test opens <b>one loop at a time</b>: break the roll channel,
          leave pitch closed, read the margins. The chart shows that answer in its first bar, and it
          looks healthy.
        </p>
        <p>
          But a late sensor is late in both channels, and a wrong inertia is wrong in both. For a
          change that is the same in both channels the pair splits into two independent loops, one
          for each direction in which the rate vector can whirl:
        </p>
        <M display>{'\\lambda_{\\pm}(j\\omega) = a(j\\omega)\\,e^{\\pm j\\theta} + b(j\\omega)'}</M>
        <p>
          So the mounting angle is a <b>phase shift of θ</b> in the gyro path, a lead for one
          direction and a lag for the other. The second bar is the margin of the worse of the two.
          The third is the <b>disk margin</b>: what is guaranteed if the two channels change
          independently, by any mix of gain and phase.
        </p>
        <Try>
          Read the phase margin and crossover for <b>both channels by the same amount</b>, compute{' '}
          <M>{'\\tau_{max} = PM/\\omega_c'}</M> and enter it. Then raise <b>Sensors → Delay</b> past
          your number: the drone shakes long before the one-loop delay margin in the chart's summary
          is used up. To repair it, do not detune: tell the controller the truth with{' '}
          <b>Rate PID → Assumed gyro angle</b>. Press <b>Measure</b> to fly both torque sweeps and
          put the dots on the curves.
        </Try>
        <Notice>
          The solid curve is the largest singular value of S: the amplification of the worst{' '}
          <i>direction</i> of disturbance torque, which no single channel shows. Where it rises
          above the dashed curve, a loop-at-a-time test is reporting margins that the vehicle does
          not have. This is why flight-control clearance asks for multi-loop margins, and why the
          alignment of an inertial unit is calibrated rather than assumed.
        </Notice>
      </>
    ),
  },
  {
    id: 'alias',
    n: 20,
    part: 3,
    chapter: I,
    title: 'Ghost frequencies',
    level: 3,
    chart: 'spectrum',
    loop: 'rate.roll',
    setup: (p) => {
      calm(p);
      p.vibration = { gyroDeg: 5, acc: 0, hoverHz: 110 };
      p.sensors.imuRateHz = SLOW_IMU_HZ;
    },
    predict: {
      label: 'Ghost frequency',
      unit: 'Hz',
      truth: (p) => aliasHz(p.vibration.hoverHz, SLOW_IMU_HZ),
      tolerance: 0.05,
    },
    goal: {
      text: 'Predict the frequency of the wobble the gyro reports. Then get rid of it: less than 0.1 °/s of ghost below 60 Hz, with at least 40° of phase margin left.',
      check: ({ sim, prediction }) => {
        const p = sim.params;
        if (prediction == null)
          return 'first the prediction: where does a rotor tone land after sampling?';
        const truth = aliasHz(p.vibration.hoverHz, SLOW_IMU_HZ);
        if (Math.abs(prediction - truth) / truth > 0.05)
          return 'not it: the distance from the rotor frequency to the nearest multiple of the sample rate';
        const a = rateLoopOf(p);
        if (!a) return 'this lesson needs the PID cascade';
        if (sim.t < 9) return 'flying…';
        const ghost = ghostRate(sim);
        return (
          (ghost < 0.1 && a.stable && a.both.pmDeg >= 40) ||
          `ghost ${ghost.toFixed(2)} °/s · PM ${a.stable ? a.both.pmDeg.toFixed(0) + '°' : 'none'}`
        );
      },
    },
    solution: (p) => {
      p.sensors.imuRateHz = 250;
    },
    body: (
      <>
        <p>
          The rotors of this drone turn 110 times a second at hover, and each turn shakes the gyro a
          little. The airframe hardly moves at that frequency. The flight computer reads the gyro
          100 times a second.
        </p>
        <p>
          A sampler sees a signal only at its sampling instants. Between two samples the 110 Hz tone
          completes one turn and a tenth, so the samples advance by a tenth of a turn each: a slow
          wave that was never there. In general a tone at <M>{'f'}</M> sampled at <M>{'f_s'}</M>{' '}
          appears at
        </p>
        <M display>
          {
            'f_{alias} = \\left|\\,f - k\\,f_s\\,\\right| \\quad\\text{for the } k \\text{ that brings it below } f_s/2 .'
          }
        </M>
        <p>
          The chart shows the spectrum of the roll rate: what the body really does, and what the
          gyro reports. Everything right of the <b>IMU Nyquist</b> line is folded back to the left
          of it.
        </p>
        <Try>
          Compute where the rotor tone lands and enter it. Then look for it in the gyro curve, and
          watch the rate loop answer it with the motors. Try to remove it with{' '}
          <b>Gyro filter → Low-pass</b> or a notch on the ghost: both act after the sampler, where
          the ghost already looks like motion. Then use what works: <b>IMU sample rate</b> above
          twice the rotor frequency, or the <b>Anti-alias filter</b> in front of the sampler.
        </Try>
        <Notice>
          Once sampled, a ghost cannot be told from real motion at the same frequency: a filter that
          removes one removes the other, and takes phase with it. The anti-alias filter must be
          analog, before the sampler, and it costs phase too: read the margin in the chart. This is
          why flight controllers sample their gyros at several kilohertz although their loops need
          far less.
        </Notice>
      </>
    ),
  },
  {
    id: 'notch',
    n: 21,
    part: 3,
    chapter: I,
    title: 'Notch the noise',
    level: 3,
    chart: 'spectrum',
    loop: 'rate.roll',
    setup: (p) => {
      calm(p);
      p.vibration = { gyroDeg: 15, acc: 0, hoverHz: 110 };
      p.setpoint.profile = 'sine';
      p.setpoint.profileAxis = 'y';
      p.setpoint.profileAmplitude = 1.5;
      p.setpoint.profilePeriod = 4;
    },
    goal: {
      text: 'Keep the vibration out of the motors while the drone climbs and descends: motor jitter of at most 0.007 N (what it is with no vibration, plus 10 %), and at least 43° of phase margin.',
      check: ({ sim }) => {
        const a = rateLoopOf(sim.params);
        if (!a) return 'this lesson needs the PID cascade';
        if (sim.t < 12) return 'flying…';
        const jitter = motorJitter(sim);
        return (
          (jitter <= 0.007 && a.stable && a.both.pmDeg >= 43) ||
          `motor jitter ${jitter.toFixed(4)} N · PM ${a.stable ? a.both.pmDeg.toFixed(0) + '°' : 'none'}`
        );
      },
    },
    solution: (p) => {
      p.control.gyroFilter.notch = 'rpm';
    },
    body: (
      <>
        <p>
          The gyro is now sampled fast enough, so the rotor vibration appears where it is: near 110
          Hz. The rate loop differentiates its input, and a derivative multiplies a tone by its
          frequency. The motors are told to shake at 110 Hz. They cannot follow, so the drone flies
          on, but the commands cost current, heat and bearings.
        </p>
        <p>There are three ways to keep the tone out of the loop:</p>
        <ul>
          <li>
            a <b>low-pass</b>, which removes everything above its cutoff and delays everything below
            it;
          </li>
          <li>
            a <b>notch</b>, which removes one narrow band and leaves the rest nearly untouched: the
            phase it costs at a frequency <M>{'f'}</M> far below its centre <M>{'f_0'}</M> is about{' '}
            <M>{'f/(Q f_0)'}</M> radians;
          </li>
          <li>
            a notch per motor that <b>follows the rotor speed</b>, because thrust goes with the
            square of the speed and the tone moves as <M>{'f = f_{hover}\\sqrt{T/T_{hover}}'}</M>.
          </li>
        </ul>
        <Try>
          The drone flies a slow sine in altitude. Switch the chart to <b>motor command</b> and
          back. Try a <b>Low-pass</b> low enough to reach the goal and read what it does to PM. Then
          a <b>fixed notch</b> at 110 Hz: pause at the top and at the bottom of the sine and see
          where the tone is. Then the notch that follows each motor.
        </Try>
        <Notice>
          At hover the fixed notch is perfect. In the climb the rotors speed up to about 130 Hz and
          in the descent they drop below 90 Hz, and the tone walks out of the notch. Widening it
          (lower Q) costs phase, like the low-pass. The RPM filter needs one thing the others do
          not: a measurement of each rotor's speed. With it, the notch can be narrow, and the phase
          it costs at crossover is about one degree.
        </Notice>
      </>
    ),
  },
  {
    id: 'attitude',
    n: 22,
    part: 3,
    chapter: I,
    title: 'Where is up?',
    level: 3,
    chart: 'attitude',
    loop: 'att.roll',
    setup: (p) => {
      calm(p);
      p.control.l3.outer = 'geometric';
      p.sensors.attitude = 'complementary';
      p.sensors.gyroBias = { x: 1, y: 0, z: -0.5 };
      p.control.ahrs = { tau: 1, kp: 1, ki: 0.3, gate: 0 };
      p.setpoint.profileAmplitude = TURN.radius;
      p.setpoint.profilePeriod = TURN.period;
    },
    events: (sim) => {
      setAt(sim, TURN.start, 'setpoint.profile', 'circle');
      setAt(sim, TURN.end, 'setpoint.profile', 'none');
    },
    predict: {
      label: 'Accelerometer reading in the turn, above 1 g',
      unit: '%',
      truth: turnExcess,
      tolerance: 0.1,
    },
    goal: {
      text: 'Predict by how much the accelerometer reads more than 1 g in the turn. Then fly the whole manoeuvre, ten seconds of hover and four laps, with the estimated "up" never more than 2° from the true one.',
      check: ({ sim, prediction }) => {
        if (sim.params.sensors.attitude === 'truth')
          return 'fly on an estimate: the true attitude is not available to a real controller';
        if (prediction == null)
          return 'first the prediction: centripetal acceleration v²/r, then the length of the specific force';
        const truth = turnExcess(sim.params);
        if (Math.abs(prediction - truth) / truth > 0.1)
          return 'not within 10 %: the accelerometer reads √(g² + a²), with a = v²/r';
        if (sim.state.crashed) return 'crashed: press R to fly again';
        if (sim.t < TURN.end + 2) return 'flying…';
        const { series } = sim.telemetry.window(['ahrs.err'], TURN.start);
        const worst = Math.max(...series[0]!.filter((v) => !Number.isNaN(v)));
        return worst < 2 || `largest error ${worst.toFixed(1)}°: press R to fly it again`;
      },
    },
    solution: (p) => {
      p.sensors.attitude = 'mahony';
      p.control.ahrs.gate = 0.03;
    },
    body: (
      <>
        <p>
          Every lesson so far handed the controller the true attitude. A real one has two sensors
          and neither knows where up is. The <b>gyro</b> measures how fast the body turns:
          integrated, it follows every motion, and its bias (here 1 °/s) grows into an error of a
          degree per second. The <b>accelerometer</b> measures every force except gravity. At rest
          that is the push that holds the drone up, so it points up. In a turn it is the thrust, and
          the thrust points wherever the drone does.
        </p>
        <p>
          A <b>complementary filter</b> believes the gyro for anything faster than its time constant{' '}
          <M>{'\\tau'}</M> and the accelerometer for anything slower. That leaves two errors which
          pull in opposite directions:
        </p>
        <M display>
          {
            'e_{bias} \\approx b\\,\\tau \\qquad\\qquad e_{turn} \\approx \\frac{\\theta}{\\sqrt{1 + (\\Omega\\tau)^2}}'
          }
        </M>
        <p>
          with <M>{'\\theta'}</M> the tilt in a turn that goes round at <M>{'\\Omega'}</M> rad/s.
          The <b>Mahony filter</b> is the same blend with two additions. An error that stays is
          taken for a gyro bias and subtracted (the integral gain). And the accelerometer is used
          only while it reads close to 1 g (the gate), because a reading of another length cannot be
          gravity alone.
        </p>
        <Try>
          The drone hovers for ten seconds, then flies four laps of a 3 m circle in 5 s each. Watch
          the error with the complementary filter, then press <b>R</b> with other values of{' '}
          <b>Attitude estimate → τ</b>: 0.5, 2, 5, 10 s. No value reaches 2°. Switch to the Mahony
          filter: the bias is gone before the turn starts, and the turn still fools it. Compute how
          long the accelerometer's reading is in the turn, enter it, and set the <b>gate</b> well
          below that: on the way into the turn and out of it the reading passes through every length
          in between.
        </Try>
        <Notice>
          With the gate closed the filter flies the turn on the gyro alone, so everything rests on
          the bias it learned in the ten seconds of hover: a filter that is switched on in the
          middle of a manoeuvre has nothing to learn from. And a gate cannot help in a turn that
          never ends. For that the filter needs to know the acceleration itself, from a velocity
          measurement, which is what a navigation filter adds (lesson III.24).
        </Notice>
      </>
    ),
  },
  {
    id: 'montecarlo',
    n: 29,
    part: 3,
    chapter: J,
    title: 'One flight proves nothing',
    level: 3,
    chart: 'dispersion',
    loop: 'att.roll',
    setup: (p) => {
      p.wind.gustsPerMinute = 10;
      p.wind.gustAmpMin = 2;
      p.wind.gustAmpMax = 6;
      p.wind.turbSigma = 0.8;
    },
    predict: {
      label: 'Clean runs for 99 % at 95 % confidence',
      unit: 'runs',
      truth: () => cleanRunsNeeded(0.99),
      tolerance: 0.05,
    },
    goal: {
      text: 'Predict how many flights without a single failure show a pass rate of 99 % with 95 % confidence. Then retune the cascade until a campaign of 300 flights has no failure at all.',
      check: ({ sim, prediction }) => {
        if (prediction == null)
          return 'first the prediction: if the true pass rate were 99 %, how likely are N clean flights in a row?';
        if (Math.abs(prediction - cleanRunsNeeded(0.99)) / cleanRunsNeeded(0.99) > 0.05)
          return 'not within 5 %: find N with 0.99^N = 0.05';
        const runs = finishedCampaign(sim.params);
        if (!runs) return 'press Fly 300 with this tune and wait for the campaign to finish';
        const failed = runs.filter((r) => !r.passed).length;
        const bound = passRateBound(runs.length - failed, runs.length);
        return (
          failed === 0 ||
          `${failed} of ${runs.length} failed: pass rate ≥ ${(bound * 100).toFixed(1)} % with 95 % confidence`
        );
      },
    },
    solution: (p) => {
      p.control.l3.attKpRP = 5;
    },
    body: (
      <>
        <p>
          The drone you have flown all along hovers in gusts. It flies well, and the rate loop has
          45° of phase margin. But you know its mass to within 20 %, its motors' lag to within a
          factor of two, its sensor delay to within 50 ms, its gyro bias to within 2 °/s, and
          nothing about tomorrow's wind. A single flight checks one combination.
        </p>
        <p>
          A <b>Monte Carlo campaign</b> flies the scenario again and again, each time with the
          uncertain values drawn at random from their ranges, and counts the failures. A flight
          fails if it crashes or if its body rate still exceeds 300 °/s in its last five seconds.
          Gusts never take it above about 140 °/s; an unstable loop takes it past 500 °/s.
        </p>
        <p>
          A count is not yet a guarantee. If the true failure rate were 1 %, a run of <M>{'N'}</M>{' '}
          clean flights would still happen with probability <M>{'0.99^N'}</M>. The statement "at
          least 99 % with 95 % confidence" needs <M>{'N'}</M> large enough that{' '}
          <M>{'0.99^N \\le 0.05'}</M>: roughly <M>{'3/N'}</M> is the failure rate that <M>{'N'}</M>{' '}
          clean runs can rule out.
        </p>
        <Try>
          Compute <M>{'N'}</M> and enter it. Press <b>Fly 300</b> and watch the points arrive: one
          per flight, placed by its motor lag and its sensor delay. Find where the crosses sit, and
          think which loop that corner hurts. Then change one gain and fly the same 300 drones
          again: every campaign draws the same values, so only your change differs.
        </Try>
        <Notice>
          The failures are not spread at random: they sit where slow motors meet a late sensor, the
          two things that eat phase. The nominal flight never goes there, and a test pilot might not
          either. This is how launch vehicles and landers are cleared for flight [Hanson 2010]:
          requirements become pass rates with confidence bounds, and thousands of dispersed
          simulations find the corners. A clean campaign proves the corners you dispersed, and
          nothing about the ones you did not.
        </Notice>
      </>
    ),
  },
];
