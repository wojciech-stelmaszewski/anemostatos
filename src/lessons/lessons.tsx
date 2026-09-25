import { Simulation } from '@/engine/simulation';
import { defaultParams, type Params } from '@/sim/params';
import { useParams } from '@/store/params';
import { SIGNAL } from '@/ui/colors';
import { K, Notice, Try } from './Bits';
import { M } from './Math';
import type { Lesson } from './types';

/** Change a parameter through the store, so the panel follows scripted events. */
const setParam = (path: string, value: unknown) => useParams.getState().set(path, value);

const calm = (p: Params) => {
  p.wind.enabled = false;
};

// Gust challenge: the reference score is the default tune on exactly the same wind.
const CHALLENGE_SEED = 4242;
const CHALLENGE_FROM = 5;
const CHALLENGE_TO = 65;
const challengeSetup = (p: Params) => {
  p.sim.seed = CHALLENGE_SEED;
  p.wind.gustsPerMinute = 14;
  p.wind.gustAmpMin = 4;
  p.wind.gustAmpMax = 10;
  p.wind.gustVertical = 1;
  p.wind.turbSigma = 1.2;
};
const rmsError = (sim: Simulation, from: number, to: number) => {
  const { t, series } = sim.telemetry.window(['alt.err'], from);
  let sq = 0;
  let n = 0;
  for (let k = 0; k < t.length; k++) {
    if (t[k]! > to) break;
    sq += series[0]![k]! ** 2;
    n++;
  }
  return n ? Math.sqrt(sq / n) : NaN;
};
let reference: number | null = null;
const referenceScore = () => {
  if (reference === null) {
    const p = defaultParams();
    challengeSetup(p);
    const s = new Simulation(p);
    for (let k = 0; k < CHALLENGE_TO * 1000; k++) s.step();
    reference = rmsError(s, CHALLENGE_FROM, CHALLENGE_TO);
  }
  return reference;
};

export const LESSONS: Lesson[] = [
  {
    id: 'meet',
    title: 'Meet the loop',
    level: 1,
    body: (
      <>
        <p>
          The drone wants to hover at the <b>setpoint</b> (the translucent sphere). Every 4 ms the
          controller reads the altimeter (the{' '}
          <b style={{ color: SIGNAL.measurement }}>measurement</b>), computes the{' '}
          <b style={{ color: SIGNAL.error }}>error</b> <M>{'e = r - y'}</M> and turns it into a
          thrust command <M>u</M>:
        </p>
        <M display>
          {'u = K_p\\,e + K_i\\!\\int e\\,dt + K_d\\,\\dot e \\;(+\\;\\text{feedforward})'}
        </M>
        <p>
          The controller never sees the wind. It only reacts to the error it causes — that is the
          whole idea of <i>feedback</i>.
        </p>
        <Try>
          <ul className="list-disc space-y-0.5 pl-4">
            <li>
              Press <kbd>↑</kbd> a few times (or drag the sphere) and watch all charts react.
            </li>
            <li>
              Press <kbd>G</kbd> for a gust. The error chart shows how far it knocked the drone.
            </li>
            <li>
              Press <kbd>M</kbd> to switch the controller off. Watch it fall — then <kbd>R</kbd> to
              reset.
            </li>
          </ul>
        </Try>
      </>
    ),
  },
  {
    id: 'spring',
    title: 'Just a spring',
    level: 1,
    setup: (p) => {
      calm(p);
      p.control.alt = { ...p.control.alt, iOn: false, dOn: false };
    },
    events: (sim) => sim.schedule(6, () => setParam('setpoint.y', 3)),
    body: (
      <>
        <p>
          Only the <K k="p">P</K> term is on (plus the gravity <K k="ff">feedforward</K>, which
          holds the weight). A proportional controller behaves like a <b>spring</b>: the further
          from the setpoint, the harder it pulls back.
        </p>
        <p>
          At <i>t</i> = 6 s the setpoint jumps to 3 m. A mass on a spring with nothing to absorb
          energy just keeps oscillating, with natural frequency
        </p>
        <M display>
          {
            '\\omega_n = \\sqrt{K_p/m}\\quad\\Rightarrow\\quad T = 2\\pi/\\omega_n \\approx 2\\,\\text{s for } K_p = 10'
          }
        </M>
        <Notice>
          Only the small air drag slowly damps the oscillation. The info card says “undamped”.
        </Notice>
        <Try>
          Set <K k="p">Kp</K> to 40. The oscillation gets <i>faster</i> (period halves) but not
          calmer. Stiffness alone cannot fix it — we need a damper.
        </Try>
      </>
    ),
  },
  {
    id: 'droop',
    title: 'The gravity droop',
    level: 1,
    setup: (p) => {
      calm(p);
      p.control.feedforward = false;
      p.control.alt = { ...p.control.alt, iOn: false };
    },
    body: (
      <>
        <p>
          Feedforward is <b>off</b> and so is the <K k="i">I</K> term. Now the only way to produce
          thrust is through the error. To hold the weight <M>{'mg'}</M>, the error must stay at
        </p>
        <M display>
          {
            'K_p\\,e_{ss} = mg \\quad\\Rightarrow\\quad e_{ss} = \\frac{mg}{K_p} = \\frac{1 \\cdot 9.81}{10} \\approx 0.98\\,\\text{m}'
          }
        </M>
        <Notice>
          The drone hovers about a metre <i>below</i> the setpoint — exactly as predicted in the
          info card. The <K k="p">P</K> term in the chart sits at ≈ 9.81 N: it is doing the
          feedforward's job.
        </Notice>
        <Try>
          <ul className="list-disc space-y-0.5 pl-4">
            <li>
              Raise <K k="p">Kp</K> to 40: the droop shrinks to ≈ 0.25 m but never vanishes.
            </li>
            <li>
              Switch <K k="i">I</K> on: the integral slowly grows to 9.81 N and the error goes to
              zero.
            </li>
            <li>
              Or switch <K k="ff">feedforward</K> back on instead — the droop disappears instantly.
            </li>
          </ul>
        </Try>
      </>
    ),
  },
  {
    id: 'damper',
    title: 'Add a damper',
    level: 1,
    setup: (p) => {
      calm(p);
      p.control.alt = { ...p.control.alt, iOn: false, kd: 0.8 };
      p.setpoint.profile = 'square';
      p.setpoint.profileAmplitude = 0.5;
      p.setpoint.profilePeriod = 8;
    },
    goal: {
      text: 'Overshoot below 2 % and settling time below 2.5 s.',
      check: ({ metrics }) =>
        !!metrics &&
        metrics.overshoot < 2 &&
        metrics.settlingTime !== null &&
        metrics.settlingTime < 2.5,
    },
    body: (
      <>
        <p>
          The <K k="d">D</K> term reacts to the rate of change — for a fixed setpoint, to the
          vertical velocity. It is the <b>shock absorber</b> of the spring. P + D turn the drone
          into a mass–spring–damper with damping ratio
        </p>
        <M display>{'\\zeta = \\frac{K_d}{2\\sqrt{K_p\\,m}}'}</M>
        <p>
          The setpoint now steps up and down every 4 s. <M>{'\\zeta < 1'}</M> overshoots,{' '}
          <M>{'\\zeta = 1'}</M> is <i>critically damped</i> (fastest without overshoot),{' '}
          <M>{'\\zeta > 1'}</M> is sluggish.
        </p>
        <Try>
          Raise <K k="d">Kd</K> step by step and watch overshoot and settling time in the step
          metrics. Critical damping for <M>{'K_p = 10'}</M> is{' '}
          <M>{'K_d = 2\\sqrt{10} \\approx 6.3'}</M>. Then go past it (15, 30) and see the response
          get lazy.
        </Try>
      </>
    ),
  },
  {
    id: 'integral',
    title: 'Integral to the rescue',
    level: 1,
    setup: (p) => {
      calm(p);
      p.drone.mass = 1.4;
      p.control.alt = { ...p.control.alt, iOn: false };
    },
    goal: {
      text: 'Steady-state error below 1 cm.',
      check: ({ sim }) => {
        const e = sim.controller.loops().alt?.error ?? 1;
        return Math.abs(e) < 0.01 && Math.abs(sim.state.vel.y) < 0.02 && sim.t > 5;
      },
    },
    body: (
      <>
        <p>
          The drone picked up a 400 g payload, but the controller still believes{' '}
          <M>{'\\hat m = 1'}</M> kg. The <K k="ff">feedforward</K> is short by{' '}
          <M>{'0.4 \\cdot 9.81 \\approx 3.9'}</M> N, so P + D settle with an error of{' '}
          <M>{'3.9/K_p \\approx 0.39'}</M> m.
        </p>
        <p>
          The <K k="i">I</K> term accumulates the error over time. As long as <i>any</i> error
          remains, it keeps growing — so the only place it can come to rest is <M>{'e = 0'}</M>,
          holding exactly the missing 3.9 N.
        </p>
        <Try>
          Switch <K k="i">I</K> on and watch the green curve climb to ≈ 3.9 N in the PID-terms
          chart. Try <K k="i">Ki</K> = 5: faster, but look at the overshoot.
        </Try>
        <Notice>
          With an integrator on a mass, overshoot after a step is <i>unavoidable</i>: the integral
          must return to its old value, so the error has to spend some time on the other side of
          zero.
        </Notice>
      </>
    ),
  },
  {
    id: 'windup',
    title: 'Windup',
    level: 1,
    setup: (p) => {
      calm(p);
      p.drone.maxMotorThrust = 3;
      p.control.alt = { ...p.control.alt, ki: 3, antiWindup: 'none' };
    },
    events: (sim) => sim.schedule(8, () => setParam('setpoint.y', 6)),
    body: (
      <>
        <p>
          The motors are weak now: 4 × 3 N = 12 N, barely more than the 9.81 N weight. At <i>t</i> =
          8 s the setpoint jumps to 6 m. The controller asks for far more than the motors can give —
          the output <b>saturates</b> (red shading in the PID chart).
        </p>
        <p>
          During the slow climb the error stays large, so the integral keeps growing (“winds up”).
          When the drone finally arrives, the bloated integral keeps pushing: a big overshoot.
        </p>
        <Try>
          <ol className="list-decimal space-y-0.5 pl-4">
            <li>
              Wait until it settles, then press <b>Snapshot</b> (or <kbd>S</kbd>).
            </li>
            <li>
              In <i>Practical details</i> set anti-windup to <b>conditional integration</b>, press{' '}
              <kbd>R</kbd>. The grey ghost is the old run.
            </li>
            <li>
              Try <b>back-calculation</b> too.
            </li>
          </ol>
        </Try>
      </>
    ),
  },
  {
    id: 'kick',
    title: 'Derivative kick',
    level: 1,
    setup: (p) => {
      calm(p);
      p.control.alt = { ...p.control.alt, derivativeOn: 'error', dFilterHz: 0 };
      p.setpoint.profile = 'square';
      p.setpoint.profileAmplitude = 0.5;
      p.setpoint.profilePeriod = 6;
    },
    body: (
      <>
        <p>
          The <K k="d">D</K> term is computed from the <i>error</i>. When the setpoint jumps, the
          error jumps too, and its derivative is enormous for one sample:
        </p>
        <M display>
          {
            'D = K_d\\,\\frac{\\Delta e}{\\Delta t} = 7 \\cdot \\frac{1\\,\\text{m}}{0.004\\,\\text{s}} = 1750\\,\\text{N}'
          }
        </M>
        <Notice>
          Violet spikes in the PID chart and the motors slamming to their limits at every step — the
          “derivative kick”.
        </Notice>
        <Try>
          Set <i>D acts on</i> → <b>measurement</b>. For a constant setpoint{' '}
          <M>{'\\dot e = -\\dot y'}</M>, so damping is identical — but the drone's position cannot
          jump, so there is no kick.
        </Try>
      </>
    ),
  },
  {
    id: 'noise',
    title: 'Noisy sensor',
    level: 1,
    setup: (p) => {
      calm(p);
      p.sensors.posNoise = 0.01;
      p.control.alt = { ...p.control.alt, dFilterHz: 0 };
    },
    body: (
      <>
        <p>
          The altimeter now has 1 cm of random noise — tiny. But the <K k="d">D</K> term
          differentiates it: two readings 4 ms apart differing by 1 cm look like a velocity of 2.5
          m/s.
        </p>
        <Notice>
          The violet D curve is a hairball, the motors jitter, and the drone hovers worse than
          without D. The <K k="p">P</K> term is barely affected.
        </Notice>
        <Try>
          <ul className="list-disc space-y-0.5 pl-4">
            <li>
              Set the <i>D filter</i> to 20 Hz, then 5 Hz, then 1 Hz. Smooth — but at some point the
              filter's lag makes the loop oscillate again.
            </li>
            <li>
              Or lower the controller rate: fewer, further-apart samples also mean less noise gain.
            </li>
          </ul>
        </Try>
      </>
    ),
  },
  {
    id: 'slow',
    title: 'Too slow to react',
    level: 1,
    setup: (p) => {
      calm(p);
      p.setpoint.profile = 'square';
      p.setpoint.profileAmplitude = 0.5;
      p.setpoint.profilePeriod = 8;
    },
    body: (
      <>
        <p>
          A digital controller only looks every <M>{'\\Delta t'}</M> and holds its output in
          between. Waiting is <b>delay</b>, and delay eats stability: by the time the controller
          reacts, the situation has changed.
        </p>
        <Try>
          <ul className="list-disc space-y-0.5 pl-4">
            <li>
              Lower the <i>controller rate</i>: 50 Hz, 10 Hz, 7 Hz, 5 Hz. Watch the staircase in the
              thrust and the growing overshoot.
            </li>
            <li>
              Back to 250 Hz, then add <i>sensor delay</i> (Sensors group): 100 ms, 200 ms.
            </li>
          </ul>
        </Try>
        <Notice>
          The same gains that were perfect at 250 Hz become oscillatory with enough delay.
        </Notice>
      </>
    ),
  },
  {
    id: 'edge',
    title: 'Find the edge',
    level: 1,
    setup: (p) => {
      calm(p);
      p.control.alt = { ...p.control.alt, iOn: false };
      p.setpoint.profile = 'square';
      p.setpoint.profileAmplitude = 0.3;
      p.setpoint.profilePeriod = 6;
    },
    body: (
      <>
        <p>
          How far can you push <K k="p">Kp</K>? Keep <K k="d">Kd</K> = 7 and raise Kp: 30, 50, 100,
          200, 400. The motors cannot follow instantly (30 ms time constant) and the controller
          samples every 4 ms; together they limit how stiff the loop can be.
        </p>
        <Notice>
          Near the edge the loop “rings” longer and longer; past it, oscillation grows by itself.
        </Notice>
        <p className="mt-2">
          <b>About Ziegler–Nichols:</b> the classic recipe raises a P-only gain to the{' '}
          <i>ultimate gain</i> <M>{'K_u'}</M> where oscillation is sustained, measures its period{' '}
          <M>{'T_u'}</M> and uses <M>{'K_p = 0.6K_u,\\ K_i = 1.2K_u/T_u,\\ K_d = 0.075K_uT_u'}</M>.
          It assumes a plant that is stable on its own. A hovering drone is a{' '}
          <i>double integrator</i>: with P alone it oscillates at <i>every</i> gain (lesson 2), so
          there is no clean <M>{'K_u'}</M>. Heuristics come with assumptions.
        </p>
      </>
    ),
  },
  {
    id: 'challenge',
    title: 'Gust challenge',
    level: 1,
    setup: challengeSetup,
    goal: {
      text: `Beat the default tune: lower RMS altitude error from ${CHALLENGE_FROM} s to ${CHALLENGE_TO} s on the same wind.`,
      check: ({ sim }) => {
        const ref = referenceScore();
        const mine = rmsError(sim, CHALLENGE_FROM, Math.min(sim.t, CHALLENGE_TO));
        const cm = (v: number) => `${(v * 100).toFixed(1)} cm`;
        if (sim.t < CHALLENGE_TO)
          return `running… RMS ${Number.isFinite(mine) ? cm(mine) : '—'} (reference ${cm(ref)})`;
        return mine < ref ? true : `RMS ${cm(mine)} vs reference ${cm(ref)} — tune and press R`;
      },
    },
    body: (
      <>
        <p>
          Strong, mostly vertical gusts on a fixed seed ({CHALLENGE_SEED}) — every run gets exactly
          the same wind. The score is the RMS altitude error between {CHALLENGE_FROM} s and{' '}
          {CHALLENGE_TO} s.
        </p>
        <Try>
          Tune the gains, press <kbd>R</kbd>, wait a minute (or use 4× speed). Use <b>Snapshot</b>{' '}
          to compare against your previous attempt. What limits you: noise? saturation? overshoot?
        </Try>
      </>
    ),
  },
  {
    id: 'tilt',
    title: 'Why drones tilt',
    level: 2,
    setup: (p) => {
      p.wind.meanSpeed = 4;
      p.wind.gustVertical = 0.2;
    },
    loop: 'pos.x',
    body: (
      <>
        <p>
          Level 2: a point mass with <b>three independent PIDs</b>, one per axis, each pushing
          directly along its axis. The arrows now are 3D vectors: <K k="p">P</K>, <K k="i">I</K>,{' '}
          <K k="d">D</K>, <K k="ff">FF</K>. There is a 4 m/s wind along +X.
        </p>
        <Notice>
          The <K k="i">I</K> vector points into the wind: it holds the steady drag, exactly like it
          held the missing weight in lesson 5.
        </Notice>
        <p className="mt-2">
          This is a cheat: a real quadrotor can only push along its own up-axis. To move sideways it
          must <b>tilt</b> — and tilting takes time, which is why real drones need a <i>cascade</i>{' '}
          of controllers.
        </p>
        <Try>Switch to L3 (key 3) with the same wind and watch the drone lean into it.</Try>
      </>
    ),
  },
  {
    id: 'cascade',
    title: 'The cascade',
    level: 3,
    setup: (p) => {
      p.wind.meanSpeed = 5;
      p.wind.gustsOn = false;
      p.wind.turbSigma = 0.3;
    },
    loop: 'vel.x',
    body: (
      <>
        <p>
          Level 3 flies with four nested loops, each one's output being the next one's setpoint:
        </p>
        <M display>
          {
            '\\underbrace{\\text{pos}}_{P,\\ 50\\,Hz}\\!\\to v_{sp}\\to\\underbrace{\\text{vel}}_{PID,\\ 100\\,Hz}\\!\\to a_{sp}\\to\\text{tilt}\\to\\underbrace{\\text{att}}_{P,\\ 250\\,Hz}\\!\\to\\omega_{sp}\\to\\underbrace{\\text{rate}}_{PID,\\ 1\\,kHz}\\!\\to\\text{motors}'
          }
        </M>
        <p>
          Pick loops in the chart selector: the rate loop reacts in milliseconds, the position loop
          in seconds. The amber arrow is the <b>desired thrust vector</b> from the velocity loop;
          the white one is the actual thrust.
        </p>
        <Try>
          In <i>Velocity PID — horizontal</i> switch <K k="i">I</K> off. The 5 m/s wind now pushes
          the drone about a metre downwind and it stays there: the position P alone cannot hold a
          constant force. Switch I back on.
        </Try>
      </>
    ),
  },
  {
    id: 'inversion',
    title: 'Cascade inversion',
    level: 3,
    setup: (p) => {
      calm(p);
      p.setpoint.profile = 'square';
      p.setpoint.profileAxis = 'x';
      p.setpoint.profileAmplitude = 1.5;
      p.setpoint.profilePeriod = 8;
    },
    loop: 'att.pitch',
    body: (
      <>
        <p>
          A cascade only works if each inner loop is much <b>faster</b> than the loop around it: the
          outer loop assumes its request is fulfilled almost instantly.
        </p>
        <Try>
          <ul className="list-disc space-y-0.5 pl-4">
            <li>
              Lower the <i>Attitude P</i> loop rate: 30 Hz, 10 Hz, 5 Hz, 3 Hz, 2 Hz. The outer
              velocity loop (100 Hz) now outruns it — watch the wobble grow until the drone flips.
            </li>
            <li>
              Reset, then lower <i>Kp roll/pitch</i> of the attitude loop to 1.5: a slow inner loop,
              and the position response overshoots.
            </li>
          </ul>
        </Try>
      </>
    ),
  },
];
