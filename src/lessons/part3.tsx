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
import type { Params } from '@/sim/params';
import { Notice, Try } from './Bits';
import { M } from './Math';
import type { Lesson } from './types';

/**
 * Part III — the third semester: why loops work (docs/analysis.md §6).
 * Parts I and II design controllers; Part III predicts what a design will do and checks the
 * prediction against the simulator. Numbering restarts at III.1 in the lesson panel.
 */

const F = 'F · The loop as a filter';
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
];
