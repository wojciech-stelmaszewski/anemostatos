import { Simulation } from '@/engine/simulation';
import { defaultParams, type Params } from '@/sim/params';
import { K, Notice, Try } from './Bits';
import { qFromAxisAngle, qMul } from '@/math/quat';
import { v3 } from '@/math/vec3';
import { M } from './Math';
import { setAt } from './script';
import type { Lesson } from './types';

/**
 * Part II — the second semester: contemporary controllers beyond PID (docs/beyond-pid.md §6).
 * Numbering restarts at II.1 in the lesson panel.
 */

const calm = (p: Params) => {
  p.wind.enabled = false;
};

const squareSteps = (p: Params) => {
  p.setpoint.profile = 'square';
  p.setpoint.profileAmplitude = 0.5;
  p.setpoint.profilePeriod = 8;
};

/** Run a variant of the current lesson headlessly and show it as the grey ghost. */
const ghostOf = (
  sim: Simulation,
  change: (p: Params) => void,
  label: string,
  seconds: number,
  events?: (s: Simulation) => void,
): Simulation => {
  const p = structuredClone(sim.params);
  change(p);
  const g = new Simulation(p);
  events?.(g);
  for (let k = 0; k < seconds * 1000; k++) g.step();
  sim.ghost = { telemetry: g.telemetry, label };
  return g;
};

/** Largest value of `sign·channel` in [from, to] (sign −1 turns it into a minimum). */
const maxOf = (sim: Simulation, key: string, from: number, to = Infinity, sign = 1) => {
  const { t, series } = sim.telemetry.window([key], from);
  let m = -Infinity;
  for (let k = 0; k < t.length && t[k]! <= to; k++) m = Math.max(m, sign * series[0]![k]!);
  return m;
};

/** RMS of the true altitude error and the sample-to-sample thrust jitter over [from, now]. */
const precision = (sim: Simulation, from: number) => {
  const { series } = sim.telemetry.window(['sp.y', 'pos.y', 'alt.u'], from);
  const [sp, y, u] = series as [number[], number[], number[]];
  let se = 0;
  let du = 0;
  for (let k = 0; k < sp.length; k++) {
    se += (sp[k]! - y[k]!) ** 2;
    if (k) du += (u[k]! - u[k - 1]!) ** 2;
  }
  return {
    rmsCm: 100 * Math.sqrt(se / Math.max(sp.length, 1)),
    jitter: Math.sqrt(du / Math.max(sp.length - 1, 1)),
  };
};

// II.2: the PD ghost's peak thrust, measured when the lesson starts.
let pdPeak = Infinity;

const noisySensors = (p: Params) => {
  calm(p);
  p.sensors.posNoise = 0.05;
  p.sensors.posRateHz = 50;
  p.sensors.accNoise = 0.3;
  p.sensors.accBias = 0.2;
};

/** 3D position error statistics over [from, to]. */
const positionError = (sim: Simulation, from: number, to = Infinity) => {
  const keys = ['sp.x', 'pos.x', 'sp.y', 'pos.y', 'sp.z', 'pos.z'];
  const { t, series: s } = sim.telemetry.window(keys, from);
  let se = 0;
  let max = 0;
  let n = 0;
  for (let k = 0; k < t.length && t[k]! <= to; k++) {
    const e = Math.hypot(s[0]![k]! - s[1]![k]!, s[2]![k]! - s[3]![k]!, s[4]![k]! - s[5]![k]!);
    se += e * e;
    max = Math.max(max, e);
    n++;
  }
  return { rms: Math.sqrt(se / Math.max(n, 1)), max };
};

/** Roll/pitch tracking error (RMS, degrees) and motor jitter (RMS sample-to-sample change, N). */
const attitudeQuality = (sim: Simulation, from: number) => {
  const keys = ['att.roll.err', 'att.pitch.err', 'motor.1', 'motor.2', 'motor.3', 'motor.4'];
  const { series: s } = sim.telemetry.window(keys, from);
  const n = s[0]!.length;
  let att = 0;
  let du = 0;
  for (let k = 0; k < n; k++) {
    att += s[0]![k]! ** 2 + s[1]![k]! ** 2;
    if (k) for (let m = 2; m < 6; m++) du += (s[m]![k]! - s[m]![k - 1]!) ** 2;
  }
  return { attRms: Math.sqrt(att / Math.max(n, 1)), jitter: Math.sqrt(du / Math.max(n - 1, 1)) };
};

const FAULT_AT = 12;
const brokenProp = (s: Simulation) => setAt(s, FAULT_AT, 'drone.motorEfficiency.1', 0.6);
const payload = (s: Simulation) => setAt(s, 12, 'drone.mass', 1.3);
const damagedProp = (p: Params) => {
  p.wind.enabled = false;
  p.setpoint.x = 2;
};

// II.8: gusty wind, scored like Part I's gust challenge (reference = PID cascade, same wind).
const gusty = (p: Params) => {
  p.sim.seed = 4242;
  p.wind.gustsPerMinute = 14;
  p.wind.gustAmpMin = 4;
  p.wind.gustAmpMax = 10;
  p.wind.turbSigma = 1.2;
};
const GUST_FROM = 8;
const GUST_TO = 40;
let gustReference: number | null = null;
const gustReferenceScore = () => {
  if (gustReference === null) {
    const p = defaultParams();
    p.sim.level = 3;
    gusty(p);
    const s = new Simulation(p);
    for (let k = 0; k < GUST_TO * 1000; k++) s.step();
    gustReference = positionError(s, GUST_FROM, GUST_TO).rms;
  }
  return gustReference;
};

// Chapter C
const C = 'C · Geometry and trajectories';
const FLIP_AT = 10;
const FIG8_PERIOD = 8.9; // 2 m/s peak with a 2 m amplitude
const figureEight = (p: Params) => {
  p.wind.enabled = false;
  p.setpoint.y = 3;
  p.setpoint.profile = 'figure8';
  p.setpoint.profileAmplitude = 2;
  p.setpoint.profilePeriod = FIG8_PERIOD;
};
const minOf = (sim: Simulation, key: string, from: number) => -maxOf(sim, key, from, Infinity, -1);
/** Lowest altitude since `from`. */
const lowest = (sim: Simulation, from: number) => -maxOf(sim, 'pos.y', from, Infinity, -1);

const A = 'A · From knobs to models';
const B = 'B · Estimate the disturbance';

export const PART_TWO: Lesson[] = [
  {
    id: 'state',
    part: 2,
    chapter: A,
    title: 'State, not error',
    level: 1,
    chart: 'phase',
    setup: (p) => {
      calm(p);
      squareSteps(p);
      p.control.alt = { ...p.control.alt, kd: 6.3, iOn: false };
    },
    goal: {
      text: 'Overshoot below 1 % and settling time below 1.5 s.',
      check: ({ metrics }) =>
        !!metrics &&
        metrics.overshoot < 1 &&
        metrics.settlingTime !== null &&
        metrics.settlingTime < 1.5,
    },
    solution: (p) => {
      p.control.alt = { ...p.control.alt, kp: 20, kd: 8 };
    },
    body: (
      <>
        <p>
          Welcome to the second semester. Part I looked at the <b>error</b>. Part II looks at the{' '}
          <b>state</b>: the few numbers that, together with the inputs, determine the whole future.
          For a mass on a thrust they are position and velocity — and a PD controller is already a
          function of the state:
        </p>
        <M display>
          {
            'u = K_p\\,e + K_d\\,\\dot e = -\\begin{pmatrix}K_p & K_d\\end{pmatrix}\\begin{pmatrix}y - r\\\\ \\dot y\\end{pmatrix} = -K\\,x'
          }
        </M>
        <p>
          The bottom-right chart is now the <b>phase portrait</b>: the state drawn as a point in the
          <M>{'(e, \\dot e)'}</M> plane. A setpoint step makes the point hop sideways; the
          controller then steers it back to the origin. An underdamped loop spirals in, a critically
          damped one slides in, an overdamped one crawls.
        </p>
        <Try>
          <K k="d">Kd</K> = 6.3 is critical damping for <K k="p">Kp</K> = 10, but it takes almost 2
          s. Make the trajectory come home faster <i>without</i> crossing the vertical axis
          (overshoot): you will need to raise <b>both</b> gains. Watch the shape of the curve, not
          just the time plots.
        </Try>
        <Notice>
          Every controller in this semester is a different answer to one of two questions:{' '}
          <i>how to choose K</i> (LQR, MPC, learning) or <i>how to know x</i> when the sensors lie
          (Kalman filter, observers).
        </Notice>
      </>
    ),
  },
  {
    id: 'lqr',
    part: 2,
    chapter: A,
    title: 'Pay for what you want',
    level: 1,
    setup: (p) => {
      calm(p);
      squareSteps(p);
      p.control.l1.kind = 'lqr';
      p.control.lqr.r = 10;
    },
    onStart: (sim) => {
      const g = ghostOf(
        sim,
        (p) => {
          p.control.l1.kind = 'pid';
          p.control.alt = { ...p.control.alt, iOn: false };
        },
        'PD (Kp 10, Kd 7)',
        40,
      );
      pdPeak = maxOf(g, 'alt.u', 16, 40);
    },
    goal: {
      text: 'Settle within 1.5 s (the PD ghost needs 2.3 s) with overshoot below 2 %, using no more peak thrust than the PD.',
      check: ({ sim, metrics }) => {
        if (sim.t < 16 || !metrics || metrics.settlingTime === null) return false;
        const peak = maxOf(sim, 'alt.u', sim.t - 8);
        const ok = metrics.settlingTime < 1.5 && metrics.overshoot < 2 && peak <= pdPeak + 0.05;
        return (
          ok ||
          `settling ${metrics.settlingTime.toFixed(2)} s · overshoot ${metrics.overshoot.toFixed(1)} % · peak ${peak.toFixed(1)} N (PD ${pdPeak.toFixed(1)} N)`
        );
      },
    },
    solution: (p) => {
      p.control.lqr.r = 1;
    },
    body: (
      <>
        <p>
          Tuning a PID is haggling with three knobs. The <b>linear–quadratic regulator</b> asks a
          different question: <i>what do you care about, and how much?</i> Write it down as a cost
        </p>
        <M display>{'J = \\sum_k \\big(q_e\\,e_k^2 + q_v\\,v_k^2 + r\\,u_k^2\\big)\\,\\Delta t'}</M>
        <p>
          and the <b>Riccati equation</b> returns the gain <M>{'K'}</M> of <M>{'u = -Kx'}</M> that
          makes <M>J</M> as small as possible — for the model <M>{'\\hat m\\,\\ddot y = u'}</M>. The
          info card shows the resulting gains (they are a <K k="p">Kp</K> and a <K k="d">Kd</K>!)
          and the closed-loop poles.
        </p>
        <Try>
          The ghost is a hand-tuned PD. Lower the thrust cost <b>r</b> and watch the gains and the
          poles move; raise <b>q_v</b> to calm it down. Only the <i>ratios</i> matter: multiply all
          three costs by 10 and nothing changes.
        </Try>
        <Notice>
          You no longer tune gains, you state preferences. That scales: the same recipe designs
          controllers with 12 states and 4 motors, where hand-tuning is hopeless.
        </Notice>
      </>
    ),
  },
  {
    id: 'lqi',
    part: 2,
    chapter: A,
    title: 'Integral, the optimal way',
    level: 1,
    setup: (p) => {
      calm(p);
      p.control.l1.kind = 'lqr';
      p.control.model.mass = 0.8;
    },
    events: (sim) => setAt(sim, 15, 'setpoint.y', 3),
    goal: {
      text: 'Steady-state error below 1 cm, with the model still believing m̂ = 0.8 kg.',
      check: ({ sim }) => {
        const e = sim.controller.loops().alt?.error ?? 1;
        return (
          sim.params.control.model.mass <= 0.8 &&
          Math.abs(e) < 0.01 &&
          Math.abs(sim.state.vel.y) < 0.02 &&
          sim.t > 5
        );
      },
    },
    solution: (p) => {
      p.control.lqr.integral = true;
    },
    body: (
      <>
        <p>
          The model is wrong: it believes <M>{'\\hat m = 0.8'}</M> kg, the drone weighs 1 kg. The
          feedforward is 2 N short and the LQR — optimal for the <i>model</i> — hangs about 20 cm
          low, exactly like the P controller of Part I.
        </p>
        <p>
          The fix is the same idea as the <K k="i">I</K> term, but done the LQR way: make the
          accumulated error a <b>state</b>, <M>{'\\dot\\xi = e'}</M>, give it a cost{' '}
          <M>{'q_i\\,\\xi^2'}</M>, and let the Riccati equation choose its gain. This is <b>LQI</b>.
        </p>
        <Try>
          Switch on <i>Integral state (LQI)</i>. The offset disappears. At <i>t</i> = 15 s the
          setpoint steps to 3 m: raise <b>q_i</b> and you will meet an old friend — the
          integral-induced overshoot of Part I, lesson 5.
        </Try>
        <Notice>
          Optimal control does not make a wrong model right. It only makes the best of the model it
          is given — integral action is how it hedges.
        </Notice>
      </>
    ),
  },
  {
    id: 'kalman',
    part: 2,
    chapter: A,
    title: 'Trust issues',
    level: 1,
    chart: 'estimate',
    setup: noisySensors,
    goal: {
      text: 'RMS altitude error below 2 cm and thrust jitter below 1.5 N over the last 10 s.',
      check: ({ sim }) => {
        if (sim.t < 15) return false;
        const { rmsCm, jitter } = precision(sim, sim.t - 10);
        return (
          (rmsCm < 2 && jitter < 1.5) ||
          `RMS error ${rmsCm.toFixed(1)} cm · thrust jitter ${jitter.toFixed(2)} N`
        );
      },
    },
    solution: (p) => {
      p.control.l1.estimator = 'kalman';
    },
    body: (
      <>
        <p>
          A cheap barometer: 5 cm of noise, only 50 fixes per second. The PID differentiates that
          staircase and the <K k="d">D</K> term turns it into violent thrust jitter. Part I's answer
          was a low-pass filter — which trades noise for delay.
        </p>
        <p>
          The <b>Kalman filter</b> uses a second sensor and a model. The accelerometer is fast and
          smooth but, integrated, drifts; the barometer is noisy but does not drift. The filter
          <b> predicts</b> with the accelerometer and <b>corrects</b> with every barometer fix,
          weighting the two by how much it trusts each:
        </p>
        <M display>
          {
            '\\hat x^- = F\\hat x + G\\,a_{meas},\\qquad \\hat x = \\hat x^- + K\\,(y_{meas} - H\\hat x^-)'
          }
        </M>
        <p>
          It even estimates the accelerometer's <b>bias</b> as a third state. The chart shows truth,
          sensor, estimate and the filter's own ±2σ band.
        </p>
        <Try>
          <ol className="list-decimal space-y-0.5 pl-4">
            <li>
              <i>Controller → State estimate → Kalman filter.</i> Same PID, same gains.
            </li>
            <li>
              Set <i>Assumed position σ</i> to 0.005 m (over-trusting the barometer), then 0.5 m
              (distrusting it). Watch the band and the thrust.
            </li>
            <li>
              Set <i>Bias drift</i> to 0: the filter can no longer learn the 0.2 m/s² bias.
            </li>
          </ol>
        </Try>
      </>
    ),
  },
  {
    id: 'adrc',
    part: 2,
    chapter: B,
    title: 'Everything is a disturbance',
    level: 1,
    chart: 'disturbance',
    setup: (p) => {
      p.control.l1.kind = 'adrc';
    },
    events: (sim) => payload(sim),
    onStart: (sim) =>
      ghostOf(sim, (p) => (p.control.l1.kind = 'pid'), 'PID (default)', 40, payload),
    body: (
      <>
        <p>
          <b>Active disturbance rejection control</b> takes a radical view: the only model is{' '}
          <M>{'\\ddot y = b_0\\,u + f'}</M> with <M>{'b_0 = 1/\\hat m'}</M>. Everything else — wind,
          drag, a wrong mass, even the motor lag — is lumped into one <b>total disturbance</b>{' '}
          <M>f</M>.
        </p>
        <p>
          An <b>extended state observer</b> treats <M>f</M> as one more state and estimates it from
          the altitude alone. The controller then simply subtracts it and flies a clean double
          integrator with a PD:
        </p>
        <M display>
          {'u = \\frac{\\omega_c^2\\,(r - \\hat y) - 2\\omega_c\\,\\hat v - \\hat f}{b_0}'}
        </M>
        <p>
          The bottom-right chart shows the <b>true</b> disturbance force (solid, the simulator knows
          it) and the observer's <b>estimate</b> (dashed). At <i>t</i> = 12 s a 300 g payload lands
          on the drone.
        </p>
        <Notice>
          The ghost is the default PID on the same wind and payload. Its I term must wait for an
          error to build up; the observer sees the extra weight within a fraction of a second.
        </Notice>
        <Try>
          Set <i>Assumed mass</i> to 0.6 kg — ADRC barely cares. Switch the gravity feedforward off:
          the observer learns gravity as just another disturbance.
        </Try>
      </>
    ),
  },
  {
    id: 'adrc-bw',
    part: 2,
    chapter: B,
    title: 'How fast should you believe?',
    level: 1,
    setup: (p) => {
      p.control.l1.kind = 'adrc';
      p.control.adrc.wo = 40;
      p.sensors.posNoise = 0.02;
    },
    goal: {
      text: 'Score below 7.5 over the last 15 s: RMS error in cm + 10 × thrust jitter in N.',
      check: ({ sim }) => {
        if (sim.t < 20) return false;
        const { rmsCm, jitter } = precision(sim, sim.t - 15);
        const score = rmsCm + 10 * jitter;
        return (
          score < 7.5 ||
          `score ${score.toFixed(1)} = ${rmsCm.toFixed(1)} cm + 10 × ${jitter.toFixed(2)} N`
        );
      },
    },
    solution: (p) => {
      p.control.adrc.wo = 10;
    },
    body: (
      <>
        <p>
          The observer bandwidth <M>{'\\omega_o'}</M> is how quickly the observer believes the
          sensor. Fast: disturbances are caught early — but 2 cm of sensor noise is also
          "disturbance", and it goes straight into the motors. Slow: smooth thrust, but gusts are
          noticed late.
        </p>
        <Try>
          Start at <M>{'\\omega_o = 40'}</M> rad/s and look at the thrust. Go down (25, 15, 10, 6)
          and up (60, 100). Find the sweet spot for the score.
        </Try>
        <Notice>
          Beyond ≈ 60 rad/s it gets worse than noisy: the observer ignores the 30 ms motor lag, and
          at high bandwidth that ignorance destabilises the loop. A model can be simple — but not
          simpler than the bandwidth you ask for. Rule of thumb: <M>{'\\omega_o'}</M> ≈ 3–10 ×{' '}
          <M>{'\\omega_c'}</M>.
        </Notice>
      </>
    ),
  },

  {
    id: 'indi',
    part: 2,
    chapter: B,
    title: "Don't model it, measure it",
    level: 3,
    loop: 'rate.roll',
    setup: damagedProp,
    events: (sim) => brokenProp(sim),
    onStart: (sim) => ghostOf(sim, () => {}, 'PID rate loop', 30, brokenProp),
    goal: {
      text: 'After the prop breaks, keep the position error below 30 cm (the PID ghost drifts 77 cm).',
      check: ({ sim }) => {
        if (sim.t < FAULT_AT + 8) return false;
        const { max } = positionError(sim, FAULT_AT);
        return max < 0.3 || `max error after the fault ${(max * 100).toFixed(0)} cm`;
      },
    },
    solution: (p) => {
      p.control.l3.inner = 'indi';
    },
    body: (
      <>
        <p>
          At <i>t</i> = {FAULT_AT} s the propeller of motor 2 chips: it keeps spinning at the
          commanded speed but gives only 60 % of the thrust. Nothing in the controller's model
          knows. The PID rate loop sees a rate error, lets its <K k="i">I</K> term build up, and
          meanwhile the drone tips and drifts.
        </p>
        <p>
          <b>Incremental nonlinear dynamic inversion</b> asks a different question:{' '}
          <i>what angular acceleration do I have right now, and what do I want?</i> It keeps the
          torque the motors produce at this moment and changes it by exactly the missing part:
        </p>
        <M display>
          {
            '\\tau = \\tau_0 + \\hat I\\,(\\alpha_{des} - \\dot\\omega),\\qquad \\alpha_{des} = K\\,(\\omega_{sp} - \\omega)'
          }
        </M>
        <p>
          <M>{'\\tau_0'}</M> comes from the motor speeds, <M>{'\\dot\\omega'}</M> from
          differentiating the gyro. The broken prop, a shifted payload, any unmodelled torque — it
          is all already <i>inside</i> the measured <M>{'\\dot\\omega'}</M>. The only model left is
          the inertia <M>{'\\hat I'}</M>, and even that may be off by a factor of two.
        </p>
        <Try>
          <i>Controller → Inner stage → attitude P + rate INDI</i>, press <kbd>R</kbd>. The chart
          shows the three parts of the INDI command: <K k="i">what the motors do now</K>,{' '}
          <K k="p">what we want</K>, <K k="d">what we measure</K>.
        </Try>
        <Notice>
          Try the <i>Assumed inertia</i> slider: ×0.5 and ×2 fly fine, ×3 makes the loop too
          aggressive. INDI is robust to <i>unmodelled</i> effects, not to a wildly wrong idea of its
          own actuators.
        </Notice>
      </>
    ),
  },
  {
    id: 'indi-wind',
    part: 2,
    chapter: B,
    title: 'Feel the push',
    level: 3,
    loop: 'vel.x',
    chart: 'disturbance',
    setup: gusty,
    onStart: (sim) => {
      ghostOf(sim, () => {}, 'PID cascade', GUST_TO);
    },
    goal: {
      text: `RMS position error between ${GUST_FROM} s and ${GUST_TO} s at least 40 % below the PID cascade on the same wind.`,
      check: ({ sim }) => {
        if (sim.t < GUST_TO) return false;
        const ref = gustReferenceScore();
        const { rms } = positionError(sim, GUST_FROM, GUST_TO);
        return (
          rms < 0.6 * ref ||
          `${(rms * 100).toFixed(1)} cm (PID cascade ${(ref * 100).toFixed(1)} cm, target ${(60 * ref).toFixed(1)} cm)`
        );
      },
    },
    solution: (p) => {
      p.control.l3.compensation = 'indi';
    },
    body: (
      <>
        <p>
          A gust is a <b>force</b>. The PID cascade only notices it once it has pushed the drone far
          enough to cause a velocity error, and its <K k="i">I</K> term needs more time still. An
          accelerometer feels the push the moment it happens.
        </p>
        <p>
          <b>Cascaded INDI</b> applies the same idea one level up: keep the thrust vector the motors
          produce now, <M>{'F_0'}</M>, and correct it by the acceleration that is missing:
        </p>
        <M display>{'F = F_0 + \\hat m\\,(a_{sp} - a_{meas})'}</M>
        <p>
          The bottom-right chart shows the true disturbance force. With acceleration INDI on, a
          dashed line appears: what INDI implicitly <i>knows</i> about the disturbance, without ever
          computing it.
        </p>
        <Try>
          <ol className="list-decimal space-y-0.5 pl-4">
            <li>
              First switch only the <i>Inner stage</i> to INDI and reset. No better — the rate loop
              was never the problem.
            </li>
            <li>
              Now set <i>Disturbance compensation → acceleration INDI</i>. Pick the new{' '}
              <i>Accel X (INDI)</i> loop in the chart selector to see its parts.
            </li>
          </ol>
        </Try>
        <Notice>
          Each INDI loop rejects disturbances at its own level: torques in the rate loop (last
          lesson), forces in the acceleration loop. That is why research stacks (and the comparisons
          in docs/beyond-pid.md) put INDI under everything else.
        </Notice>
      </>
    ),
  },
  {
    id: 'indi-sync',
    part: 2,
    chapter: B,
    title: 'Keep your filters in sync',
    level: 3,
    loop: 'rate.pitch',
    setup: (p) => {
      p.wind.enabled = false;
      p.setpoint.profile = 'square';
      p.setpoint.profileAxis = 'x';
      p.setpoint.profileAmplitude = 1.5;
      p.setpoint.profilePeriod = 6;
      p.sensors.gyroNoise = 10;
      p.control.l3.inner = 'indi';
      p.control.indi.filterHz = 5;
      p.control.indi.syncFilters = false;
    },
    goal: {
      text: 'Over the last 10 s: roll/pitch error below 8.5° RMS and motor jitter below 0.12 N.',
      check: ({ sim }) => {
        if (sim.t < 15) return false;
        const { attRms, jitter } = attitudeQuality(sim, sim.t - 10);
        return (
          (attRms < 8.5 && jitter < 0.12) ||
          `attitude ${attRms.toFixed(1)}° RMS · motor jitter ${jitter.toFixed(3)} N`
        );
      },
    },
    solution: (p) => {
      p.control.indi.syncFilters = true;
    },
    body: (
      <>
        <p>
          A noisy gyro (10 °/s) forces a slow 5 Hz filter before differentiating it — and a slow
          filter means the measured <M>{'\\dot\\omega'}</M> describes the drone as it was some 30 ms
          ago. The motor torque <M>{'\\tau_0'}</M>, however, is taken fresh. INDI subtracts one from
          the other, so it now compares <i>today's</i> torque with <i>yesterday's</i> acceleration
          and keeps correcting for a mismatch that no longer exists.
        </p>
        <Try>
          Watch the wobble, then switch on <i>INDI → Synchronised filters</i>: the motor signal
          passes through the same filter, both halves describe the same moment, the wobble is gone.
          Tempted to just raise the filter cutoff instead? Try it — and look at the motors.
        </Try>
        <Notice>
          This is the most common INDI implementation bug in practice. The published fix is exactly
          this: filter the actuator feedback with the same filter as the sensor.
        </Notice>
      </>
    ),
  },
  {
    id: 'geometric',
    part: 2,
    chapter: C,
    title: 'Flying on a sphere',
    level: 3,
    loop: 'att.roll',
    setup: (p) => {
      p.wind.enabled = false;
      p.setpoint.y = 6;
      p.control.l3.attitude = 'euler';
    },
    events: (sim) =>
      sim.schedule(FLIP_AT, (s) => {
        s.state.q = qMul(qFromAxisAngle(v3(1, 0, 1), (170 * Math.PI) / 180), s.state.q);
        s.state.omega = v3();
      }),
    goal: {
      text: 'Recover from the 170° flip losing less than 1.8 m of altitude.',
      check: ({ sim }) => {
        if (sim.t < FLIP_AT + 4) return false;
        const loss = 6 - lowest(sim, FLIP_AT);
        return loss < 1.8 || `altitude lost ${loss.toFixed(2)} m`;
      },
    },
    solution: (p) => {
      p.control.l3.attitude = 'quaternion';
    },
    body: (
      <>
        <p>
          Attitudes live on a sphere-like space (the rotation group), not on three independent
          number lines. At <i>t</i> = {FLIP_AT} s something flips the drone 170° about a diagonal
          axis, 6 m up. It has to turn its thrust back upwards before gravity wins.
        </p>
        <p>
          The attitude law is set to the <b>naive</b> one: subtract the roll, pitch and yaw angles
          and command them as body rates. Euler angles are fine for small errors, but near ±90°
          pitch they fold over (<i>gimbal lock</i>): roll and yaw jump by 180°, and the commanded
          rotation points the wrong way for a while.
        </p>
        <Try>
          Press <kbd>R</kbd> and watch the recovery a few times. Then set{' '}
          <i>Controller → Attitude law</i> to <b>quaternion error</b> (the shortest rotation,
          computed on the sphere itself) or <b>tilt-prioritised</b>, and press <kbd>R</kbd>.
        </Try>
        <Notice>
          Measured here: Euler loses 2.6 m and needs 1.4 s to level; the rotation-based laws lose
          1.4 m and level in 0.4 s. Tilt-prioritised and quaternion behave alike in this test — the
          tilt-first split pays off when a large <i>yaw</i> error would otherwise steal authority
          from the thrust direction.
        </Notice>
      </>
    ),
  },
  {
    id: 'flatness',
    part: 2,
    chapter: C,
    title: 'Feedforward from the future',
    level: 3,
    loop: 'pos.x',
    setup: (p) => {
      figureEight(p);
      p.control.l3.outer = 'geometric';
      p.control.geometric.feedforward = false;
    },
    onStart: (sim) => {
      ghostOf(sim, (p) => (p.control.l3.outer = 'pid-cascade'), 'PID cascade', 40);
    },
    goal: {
      text: 'RMS position error below 5 cm over the last lap of the 2 m/s figure-8.',
      check: ({ sim }) => {
        if (sim.t < 25) return false;
        const { rms } = positionError(sim, sim.t - FIG8_PERIOD);
        return rms < 0.05 || `RMS error ${(rms * 100).toFixed(1)} cm over the last lap`;
      },
    },
    solution: (p) => {
      p.control.geometric.feedforward = true;
    },
    body: (
      <>
        <p>
          The setpoint now flies a figure-8 at up to 2 m/s. The <b>geometric tracking controller</b>{' '}
          (Lee et al.) closes position and velocity in one law,
        </p>
        <M display>{'a = a_{ref} - K_p\\,e_p - K_v\\,e_v'}</M>
        <p>
          but its feedforward is off: it only reacts to errors, so it is always a step behind — it
          cuts the corners of the 8, like the PID cascade (the ghost).
        </p>
        <p>
          A quadrotor is <b>differentially flat</b>: from the position trajectory and its
          derivatives you can compute everything else. The acceleration tells you the thrust vector,
          hence the attitude; the <b>jerk</b> tells you how fast that attitude must turn, hence the
          body rates:
        </p>
        <M display>
          {
            '\\dot b = \\tfrac{\\hat m}{T}\\big(j - (j\\cdot b)\\,b\\big),\\qquad \\omega = b \\times \\dot b'
          }
        </M>
        <Try>
          Switch on <i>Geometric tracking → Trajectory feedforward</i>. The tracking error drops
          from about 80 cm to 2 cm. Feedback now only corrects what the plan could not foresee.
        </Try>
      </>
    ),
  },
  {
    id: 'minsnap',
    part: 2,
    chapter: C,
    title: 'Plan the motion',
    level: 3,
    loop: 'pos.x',
    setup: (p) => {
      p.wind.enabled = false;
      p.setpoint.y = 3;
      p.setpoint.profile = 'corners';
      p.setpoint.profileAmplitude = 1.5;
      p.setpoint.profilePeriod = 8;
      p.control.l3.outer = 'geometric';
    },
    goal: {
      text: 'Over the last lap: follow the plan within 10 cm RMS, with body rates below 150 °/s.',
      check: ({ sim }) => {
        if (sim.t < 30) return false;
        const from = sim.t - 8;
        const { rms } = positionError(sim, from);
        const rate = Math.max(
          maxOf(sim, 'rate.roll.meas', from),
          maxOf(sim, 'rate.pitch.meas', from),
          -minOf(sim, 'rate.roll.meas', from),
          -minOf(sim, 'rate.pitch.meas', from),
        );
        return (
          (rms < 0.1 && rate < 150) ||
          `plan error ${(rms * 100).toFixed(0)} cm RMS · peak body rate ${rate.toFixed(0)} °/s`
        );
      },
    },
    solution: (p) => {
      p.setpoint.profile = 'minsnap';
    },
    body: (
      <>
        <p>
          The drone flies a 3 × 3 m square, 2 s per side. Here the setpoint simply <b>jumps</b> to
          the next corner every 2 s — a step, and the controller does the rest as hard as it can: it
          lunges, tilts to the limit, whips round at almost 250 °/s and stops.
        </p>
        <p>
          A <b>minimum-snap trajectory</b> (Mellinger &amp; Kumar) plans the same motion instead:
          for each side, the polynomial path from rest to rest in 2 s that minimises{' '}
          <M>{'\\int \\|\\ddddot p\\|^2 dt'}</M>. Snap is what the motors feel (it maps to changes
          of thrust and torque), so the smoothest path for them.
        </p>
        <Try>
          Set <i>Setpoint → Automatic motion → square: min-snap trajectory</i>. Same corners, same
          schedule, half the peak body rate — and the drone is exactly where the plan says, at every
          moment. Then shorten the period to 7 s and look at the info card: the plan now needs 45°
          of tilt, over the 35° limit. The trajectory tells you it is infeasible <i>before</i> you
          fly it.
        </Try>
        <Notice>
          Not faster: with a good controller the steps arrive about as soon. What planning buys is
          smoothness, a schedule you can rely on, and an early warning when the plan is impossible.
        </Notice>
      </>
    ),
  },
];
