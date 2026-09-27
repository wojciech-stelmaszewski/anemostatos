import { Simulation } from '@/engine/simulation';
import type { Params } from '@/sim/params';
import { useParams } from '@/store/params';
import { K, Notice, Try } from './Bits';
import { M } from './Math';
import type { Lesson } from './types';

/**
 * Part II — the second semester: contemporary controllers beyond PID (docs/beyond-pid.md §6).
 * Numbering restarts at II.1 in the lesson panel.
 */

const setParam = (path: string, value: unknown) => useParams.getState().set(path, value);

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

const maxOf = (sim: Simulation, key: string, from: number, to = Infinity) => {
  const { t, series } = sim.telemetry.window([key], from);
  let m = -Infinity;
  for (let k = 0; k < t.length && t[k]! <= to; k++) m = Math.max(m, series[0]![k]!);
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
    events: (sim) => {
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
    events: (sim) => sim.schedule(15, () => setParam('setpoint.y', 3)),
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
    events: (sim) => {
      const payload = (s: Simulation) =>
        s.schedule(12, (x) =>
          x.setParams({ ...x.params, drone: { ...x.params.drone, mass: 1.3 } }),
        );
      ghostOf(sim, (p) => (p.control.l1.kind = 'pid'), 'PID (default)', 40, payload);
      sim.schedule(12, () => setParam('drone.mass', 1.3));
    },
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
];
