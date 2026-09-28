import { Simulation } from '@/engine/simulation';
import { defaultParams, type Params } from '@/sim/params';
import { K, Notice, Try } from './Bits';
import { qFromAxisAngle, qMul } from '@/math/quat';
import { v3 } from '@/math/vec3';
import { ArenaPanel } from '@/ui/arena/ArenaPanel';
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

// Chapter D
const D = 'D · Optimisation in the loop';
const STEP_AT = 8;
const JUMP_AT = 10;
const ceilingScene = (p: Params) => {
  p.wind.enabled = false;
  p.world.ceiling = 4;
  // Crisper weights than the L3 defaults: the altitude loop has no attitude lag to respect.
  p.control.mpc = { ...p.control.mpc, qPos: 50, qVel: 5, r: 0.05 };
};
const pillarScene = (p: Params) => {
  p.world.pillar = true;
  p.world.pillarX = 3;
  p.world.pillarZ = 0.15;
  p.world.pillarR = 0.6;
};
/** First time ≥ from when |channel − target| < tol (Infinity if never). */
const firstWithin = (sim: Simulation, key: string, target: number, tol: number, from: number) => {
  const { t, series } = sim.telemetry.window([key], from);
  for (let k = 0; k < t.length; k++) if (Math.abs(series[0]![k]! - target) < tol) return t[k]!;
  return Infinity;
};

// Chapter E
const E = 'E · Adapting and learning';
/** RMS sample-to-sample change of a telemetry channel since `from`. */
const channelJitter = (sim: Simulation, key: string, from: number) => {
  const { series } = sim.telemetry.window([key], from);
  const x = series[0]!;
  let d = 0;
  for (let k = 1; k < x.length; k++) d += (x[k]! - x[k - 1]!) ** 2;
  return Math.sqrt(d / Math.max(x.length - 1, 1));
};

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
  {
    id: 'mpc',
    part: 2,
    chapter: D,
    title: 'Look before you leap',
    level: 1,
    setup: (p) => {
      ceilingScene(p);
    },
    events: (sim) => setAt(sim, STEP_AT, 'setpoint.y', 3.95),
    goal: {
      text: 'Reach 3.95 m (± 5 cm) without ever touching the ceiling at 4 m.',
      check: ({ sim }) => {
        if (sim.t < STEP_AT + 4) return false;
        const settled = Math.abs(sim.state.pos.y - 3.95) < 0.05;
        return (
          (settled && sim.zoneTime === 0) ||
          `time above the ceiling ${sim.zoneTime.toFixed(2)} s · altitude ${sim.state.pos.y.toFixed(3)} m`
        );
      },
    },
    solution: (p) => {
      p.control.l1.kind = 'mpc';
    },
    body: (
      <>
        <p>
          A ceiling at 4 m (the red plane). At <i>t</i> = {STEP_AT} s the setpoint jumps to 3.95 m —
          5 cm below it. The PID only learns about the ceiling by hitting it: its I term makes it
          overshoot, and it spends seconds above the line. Part I's anti-windup only cleaned up{' '}
          <i>after</i> saturation; nothing ever planned for a limit.
        </p>
        <p>
          <b>Model predictive control</b> does: at every step it computes the best thrust sequence
          for the next second — subject to the thrust range <i>and</i> the ceiling — applies only
          the first move, and plans again 20 ms later (receding horizon):
        </p>
        <M display>
          {
            '\\min_{u_0..u_{N-1}} \\sum_k q\\,(y_k - r)^2 + r_u\\,u_k^2 \\quad\\text{s.t.}\\quad 0 \\le T_k \\le T_{max},\\ y_k \\le y_{ceil}'
          }
        </M>
        <Try>
          <i>Controller → Altitude controller → MPC</i>. The amber line beside the drone is its
          plan: watch it bend under the ceiling before the drone gets there.
        </Try>
      </>
    ),
  },
  {
    id: 'horizon',
    part: 2,
    chapter: D,
    title: 'How far ahead?',
    level: 1,
    setup: (p) => {
      ceilingScene(p);
      p.control.l1.kind = 'mpc';
      p.control.mpc.horizon = 2;
    },
    events: (sim) => setAt(sim, STEP_AT, 'setpoint.y', 3.95),
    goal: {
      text: 'Reach 3.95 m within 1.6 s of the step, with a horizon of 0.6 s or less.',
      check: ({ sim }) => {
        if (sim.t < STEP_AT + 4) return false;
        const reach = firstWithin(sim, 'pos.y', 3.95, 0.05, STEP_AT) - STEP_AT;
        const h = sim.params.control.mpc.horizon;
        return (
          (reach < 1.6 && h <= 0.6 && sim.zoneTime === 0) ||
          `horizon ${h.toFixed(2)} s · reached after ${Number.isFinite(reach) ? reach.toFixed(2) : '—'} s`
        );
      },
    },
    solution: (p) => {
      p.control.mpc.horizon = 0.5;
    },
    body: (
      <>
        <p>
          Planning costs computing time — look at <i>controller … µs/step</i> in the chart header. A
          2 s horizon means a 100-variable optimisation 50 times a second. Real flight computers
          have a budget.
        </p>
        <Try>
          Shorten the horizon (then <kbd>R</kbd>): 1 s, 0.5 s, 0.3 s, 0.1 s. The cost drops — and
          the controller becomes <b>myopic</b>: it still never crosses the ceiling (the constraint
          is in every plan), but it no longer sees far enough to hurry, and the climb gets slow.
          Also try capping the <i>QP iterations</i> at 5.
        </Try>
        <Notice>
          The horizon must cover the time the system needs to react — here, to brake from climbing
          speed. Beyond that, extra horizon buys little and costs a lot.
        </Notice>
      </>
    ),
  },
  {
    id: 'mpc-l3',
    part: 2,
    chapter: D,
    title: 'MPC on top, INDI below',
    level: 3,
    loop: 'pos.x',
    setup: (p) => {
      p.setpoint.y = 3;
      p.setpoint.profile = 'figure8';
      p.setpoint.profileAmplitude = 2;
      p.setpoint.profilePeriod = 6;
      gusty(p);
      p.control.l3.outer = 'mpc';
    },
    goal: {
      text: 'RMS position error below 2.5 cm over the last lap, in the gusts.',
      check: ({ sim }) => {
        if (sim.t < 25) return false;
        const { rms } = positionError(sim, sim.t - 6);
        return rms < 0.025 || `RMS error ${(rms * 100).toFixed(1)} cm over the last lap`;
      },
    },
    solution: (p) => {
      p.control.l3.inner = 'indi';
      p.control.l3.compensation = 'indi';
    },
    body: (
      <>
        <p>
          The MPC now flies the outer loop of the quadrotor: three small QPs (x, y, z) on a
          point-mass model, looking 1 s ahead along the planned figure-8, with the tilt and speed
          limits as constraints. The amber line is its plan. In the gusty wind it tracks to about 4
          cm.
        </p>
        <Try>
          Add <i>Inner stage → rate INDI</i> and <i>Disturbance compensation → acceleration INDI</i>
          . Then switch the outer stage to <i>geometric tracking</i> for comparison — with INDI
          underneath, both land around 1.5 cm.
        </Try>
        <Notice>
          This is the conclusion of Sun et al. (2022), comparing nonlinear MPC and flatness-based
          control on real racing quadrotors: an INDI inner loop made both robust, and mattered more
          than the choice on top. (Our MPC is linear, on a point-mass model; it wins on feasible
          trajectories but, unlike a full nonlinear MPC, not beyond the drone's limits — the
          geometric controller holds on longer there.)
        </Notice>
      </>
    ),
  },
  {
    id: 'mppi',
    part: 2,
    chapter: D,
    title: 'A thousand futures',
    level: 3,
    loop: 'pos.x',
    setup: (p) => {
      p.wind.enabled = false;
      pillarScene(p);
      p.control.l3.outer = 'geometric';
    },
    events: (sim) => setAt(sim, JUMP_AT, 'setpoint.x', 6),
    goal: {
      text: 'Reach the new setpoint (x = 6 m) without entering the pillar.',
      check: ({ sim }) => {
        if (sim.t < JUMP_AT + 3) return false;
        const e = Math.hypot(sim.state.pos.x - 6, sim.state.pos.y - 2, sim.state.pos.z);
        return (
          (e < 0.2 && sim.zoneTime === 0) ||
          `distance to target ${e.toFixed(2)} m · time inside the pillar ${sim.zoneTime.toFixed(2)} s`
        );
      },
    },
    solution: (p) => {
      p.control.l3.outer = 'mppi';
    },
    body: (
      <>
        <p>
          The target jumps to the far side of a pillar. Every controller so far goes straight
          through it: a pillar is not convex, and no quadratic program can say "left <i>or</i>{' '}
          right".
        </p>
        <p>
          <b>MPPI</b> (model predictive path integral) does not optimise, it <i>samples</i>: 256
          random variations of its current plan, each flown forward 1.5 s on a simple model and
          scored — tracking, effort, and a heavy penalty for every moment inside the pillar. The new
          plan is the average of all of them, weighted by <M>{'e^{-\\text{cost}/\\lambda}'}</M>. The
          coloured cloud is those futures (green cheap, red expensive).
        </p>
        <Try>
          <i>Outer stage → MPPI</i>, press <kbd>R</kbd>. Then play with the temperature λ: 0.2
          follows the best sample (decisive), 50 averages many — and the average of "left" and
          "right" is "straight into the pillar": it hesitates in front. At 500 it wanders.
        </Try>
        <Notice>
          Sampling is expensive (see the µs/step meter), but it is embarrassingly parallel — which
          is why MPPI is popular on GPUs for off-road driving and drone racing.
        </Notice>
      </>
    ),
  },
  {
    id: 'cbf',
    part: 2,
    chapter: D,
    title: 'The safety filter',
    level: 3,
    loop: 'pos.x',
    setup: (p) => {
      p.wind.enabled = false;
      pillarScene(p);
      p.setpoint.profile = 'sine';
      p.setpoint.profileAxis = 'x';
      p.setpoint.profileAmplitude = 4;
      p.setpoint.profilePeriod = 10;
      p.control.l3.outer = 'geometric';
    },
    goal: {
      text: 'For 20 s: never inside the pillar, while still following the setpoint on the free side (x below −3.5 m).',
      check: ({ sim }) => {
        if (sim.t < 30) return false;
        const from = sim.t - 20;
        const inside = maxOf(sim, 'zone.inside', from);
        const far = maxOf(sim, 'pos.x', from);
        const near = -maxOf(sim, 'pos.x', from, Infinity, -1);
        return (
          (inside === 0 && near < -3.5) ||
          `${inside ? 'entered the pillar' : 'stayed out'} · reached x from ${near.toFixed(1)} to ${far.toFixed(1)} m`
        );
      },
    },
    solution: (p) => {
      p.control.l3.safety = 'cbf';
    },
    body: (
      <>
        <p>
          The setpoint swings back and forth straight through a pillar — imagine a pilot, or a buggy
          planner, asking for it. The geometric controller obeys perfectly, and flies through.
        </p>
        <p>
          A <b>control barrier function</b> filter sits between <i>any</i> controller and the
          motors. With <M>{'h(p) = |p - c|^2 - R^2'}</M> (positive outside), it only allows
          accelerations that keep <M>h</M> from reaching zero too fast:
        </p>
        <M display>{'\\ddot h + 2\\gamma\\,\\dot h + \\gamma^2 h \\ge 0'}</M>
        <p>
          — a linear constraint on the acceleration. The filter returns the closest allowed
          acceleration to what was asked: far from the pillar it changes nothing; near it, the red
          arrow shows the correction.
        </p>
        <Try>
          <i>Safety filter → control barrier function</i>. The drone now brakes before the pillar
          and never enters it. Vary γ: small brakes early and gently, large lets it come close. Then
          try <i>Outer stage → MPPI</i> with the filter still on.
        </Try>
        <Notice>
          Watch what happens when the setpoint is behind the pillar: the drone stops in front of it
          and waits. A safety filter guarantees <b>safety, not progress</b> — it removes the part of
          the command that approaches the boundary, but it does not plan a way around (a{' '}
          <i>deadlock</i>). Going around is a planning problem: put MPPI on top (previous lesson)
          and keep the filter as the last line of defence. The filter does not care what is above it
          — PID, MPC, a learned policy or a human — which is why it is the usual way to add
          "provably safe" to controllers that are not.
        </Notice>
      </>
    ),
  },
  {
    id: 'l1ac',
    part: 2,
    chapter: E,
    title: 'Adapt fast, act smooth',
    level: 1,
    chart: 'disturbance',
    setup: (p) => {
      p.control.l1.kind = 'l1ac';
      p.control.l1ac.filterHz = 50;
      p.sensors.velNoise = 0.1;
    },
    events: (sim) => payload(sim),
    goal: {
      text: 'After the payload lands: max error below 6 cm and thrust jitter below 0.3 N.',
      check: ({ sim }) => {
        if (sim.t < 20) return false;
        const { max } = positionError(sim, 12);
        const jitter = channelJitter(sim, 'thrust', 12);
        return (
          (max < 0.06 && jitter < 0.3) ||
          `max error ${(max * 100).toFixed(1)} cm · thrust jitter ${jitter.toFixed(2)} N`
        );
      },
    },
    solution: (p) => {
      p.control.l1ac.filterHz = 3;
    },
    body: (
      <>
        <p>
          <b>L1 adaptive control</b> estimates the missing force very fast: a state predictor runs
          next to the drone, and every sample the estimate <M>{'\\hat\\sigma'}</M> is set to
          whatever would make the predictor agree with the measurement. Fast — and as noisy as the
          sensor.
        </p>
        <p>
          Its trick is to separate <i>how fast it learns</i> from <i>how much of it acts</i>: only a
          low-pass filtered estimate <M>{'C(s)\\,\\hat\\sigma'}</M> reaches the motors.
        </p>
        <M display>{'u = u_{baseline} - C(s)\\,\\hat\\sigma'}</M>
        <p>
          The velocity sensor is noisy (0.1 m/s) and at <i>t</i> = 12 s a 300 g payload lands. The
          filter is wide open at 50 Hz: the estimate (dashed, bottom right) follows the truth — and
          the noise goes straight into the motors.
        </p>
        <Try>
          Lower <i>L1 adaptive → Filter C(s)</i> (then <kbd>R</kbd>): 20, 10, 5, 3, 1 Hz. Watch the
          thrust calm down and the payload response get slower. The estimate itself stays just as
          fast — only what reaches the motors changes.
        </Try>
        <Notice>For comparison the same payload costs the PID 28 cm and ADRC 12 cm.</Notice>
      </>
    ),
  },
  {
    id: 'residual',
    part: 2,
    chapter: E,
    title: 'Learn the shape of the wind',
    level: 3,
    loop: 'pos.x',
    chart: 'disturbance',
    setup: (p) => {
      p.drone.rotorDrag = 0.02;
      p.wind.meanSpeed = 3;
      p.wind.gustsOn = false;
      p.wind.turbSigma = 0.5;
      p.setpoint.y = 3;
      p.setpoint.profile = 'figure8';
      p.setpoint.profileAmplitude = 2;
      p.setpoint.profilePeriod = 6.5;
      p.control.l3.outer = 'geometric';
    },
    goal: {
      text: 'RMS error below 5 cm over the last lap of the fast figure-8 in the wind.',
      check: ({ sim }) => {
        if (sim.t < 30) return false;
        const { rms } = positionError(sim, sim.t - 6.5);
        return rms < 0.05 || `RMS error ${(rms * 100).toFixed(1)} cm over the last lap`;
      },
    },
    solution: (p) => {
      p.control.l3.compensation = 'learned';
    },
    body: (
      <>
        <p>
          A fast figure-8 through a 3 m/s wind, with rotor drag on. The aerodynamic force now{' '}
          <i>changes with the drone's own velocity</i> — upwind, downwind, sideways, every second.
          An integrator can only hold a constant: it is always chasing (≈ 14 cm here).
        </p>
        <p>
          <b>Neural-Fly</b> (O'Connell et al. 2022) learns the <i>shape</i> of the residual force as
          a function of the state, <M>{'f \\approx \\Phi(x)\\,\\theta'}</M>, and adapts only the
          coefficients <M>{'\\theta'}</M> online. Here <M>{'\\Phi'}</M> is hand-made from the
          physics — a constant, <M>v</M>, <M>{'|v|v'}</M>, thrust × in-plane velocity — and{' '}
          <M>{'\\theta'}</M> is fitted by recursive least squares to the accelerometer's residual.
        </p>
        <Try>
          <i>Disturbance compensation → learned residual model</i>. The dashed estimate now
          anticipates the force along the path. The info card shows the fitted model.
        </Try>
        <Notice>
          Honest comparison: with this simulator's clean accelerometer, L1 adaptive and INDI do even
          better (≈ 2 cm) because they simply measure the force fast. The learned model's advantage
          shows with poor sensors: with 3 m/s² accelerometer noise and 30 ms delay all three track
          to ≈ 6–7 cm, but the learned model's thrust is 7× smoother than L1's. Neural-Fly's claim
          is exactly that: a good model lets you adapt slowly, and slow is smooth.
        </Notice>
      </>
    ),
  },
  {
    id: 'policy',
    part: 2,
    chapter: E,
    title: 'A controller nobody designed',
    level: 3,
    loop: 'pos.x',
    setup: (p) => {
      p.wind.enabled = false;
      p.control.l3.outer = 'policy';
    },
    events: (sim) => {
      setAt(sim, 10, 'setpoint.x', 3);
      setAt(sim, 20, 'drone.mass', 1.5);
    },
    onStart: (sim) => {
      const hop = (s: Simulation) => {
        setAt(s, 10, 'setpoint.x', 3);
        setAt(s, 20, 'drone.mass', 1.5);
      };
      ghostOf(sim, (p) => (p.control.l3.outer = 'geometric'), 'geometric controller', 35, hop);
    },
    body: (
      <>
        <p>
          The quadrotor is now flown by a small neural network (15 inputs, two layers of 32, 4
          outputs: collective thrust and body rates). Nobody wrote its control law. It was trained
          by <code>make train</code> in two steps, in about two minutes:
        </p>
        <ol className="list-decimal space-y-0.5 pl-4">
          <li>
            <b>Imitation</b>: fly the geometric controller in randomised conditions (mass 0.85–1.15
            kg, wind up to 3 m/s), record what it commands, fit the network to it.
          </li>
          <li>
            <b>Evolution strategies</b>: perturb the weights, let each variant fly, move towards the
            variants that stayed closer to their targets — reinforcement learning without gradients.
          </li>
        </ol>
        <p>
          The info card shows the numbers from training. At <i>t</i> = 10 s the target moves 3 m; at{' '}
          <i>t</i> = 20 s the mass jumps to 1.5 kg — well outside anything it has seen.
        </p>
        <Notice>
          What to look for, compared with the geometric ghost: it flies and reacts quickly, but it
          never quite settles — it keeps hunting around the target in a small limit cycle (about ±15
          cm), because nothing in its training rewarded perfect stillness and it has no integrator
          (no memory of past errors). After the mass jump, far outside its training range, it gets
          much worse. Careful comparisons of learned and classical controllers (Kunapuli et al.
          2025) find the same division: learning helps in fast transients and in problems too hard
          to model; classical control keeps its guarantees and its steady state. State-of-the-art
          policies are far larger, see a history of observations, and train on millions of flights.
        </Notice>
      </>
    ),
  },
  {
    id: 'arena',
    part: 2,
    chapter: E,
    title: 'The grand comparison',
    level: 3,
    body: (
      <>
        <p>
          The end of the semester. Every controller of Part II flies the same six scenarios on
          identical, seeded conditions: a step, Part I's gust challenge, a payload, a broken prop, a
          2 m/s figure-8, and noisy, delayed sensors. Two more columns: how hard the motors work and
          what it costs to compute.
        </p>
        <ArenaPanel />
        <Notice>
          No controller wins every column. INDI underneath (with MPC or geometric control on top)
          dominates the disturbances; the PID's calm motors come from being sluggish; the neural
          policy is the cheapest to run; MPPI pays for its generality. Measured once with{' '}
          <code>make bench</code> on the author's machine: docs/arena.md. The real lesson of Part II
          is not which controller is best, but which one is best <i>for what</i> — and knowing why.
        </Notice>
      </>
    ),
  },
];
