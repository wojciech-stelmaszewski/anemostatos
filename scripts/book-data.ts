// Experiments for the course book (book/). Every lesson's setup is flown headlessly and the whole
// telemetry buffer is written to book/data/<name>.csv.gz, which book/tools/figures.py plots.
// Run with `make book-data` (a few minutes). Optional argument: a name filter (substring).
import { mkdirSync, writeFileSync } from 'node:fs';
import { gzipSync } from 'node:zlib';
import { setIn } from '@/engine/schema';
import { Simulation } from '@/engine/simulation';
import { qFromAxisAngle, qMul } from '@/math/quat';
import { v3 } from '@/math/vec3';
import { defaultParams, type Params } from '@/sim/params';

type Exp = {
  name: string;
  seconds: number;
  setup?: (p: Params) => void;
  events?: (sim: Simulation) => void;
  /** Keep every telemetry sample (5 ms) instead of every second one. */
  full?: boolean;
};

const OUT = new URL('../book/data/', import.meta.url);
mkdirSync(OUT, { recursive: true });

/** Scripted parameter change at time t (as the lessons' setAt, without the app store). */
const at = (sim: Simulation, t: number, path: string, value: unknown) =>
  sim.schedule(t, (s) => s.setParams(setIn(s.params, path, value)));

const calm = (p: Params) => {
  p.wind.enabled = false;
};
const square =
  (amp: number, period: number, axis: 'x' | 'y' = 'y') =>
  (p: Params) => {
    p.setpoint.profile = 'square';
    p.setpoint.profileAxis = axis;
    p.setpoint.profileAmplitude = amp;
    p.setpoint.profilePeriod = period;
  };
const gustChallenge = (p: Params) => {
  p.sim.seed = 4242;
  p.wind.gustsPerMinute = 14;
  p.wind.gustAmpMin = 4;
  p.wind.gustAmpMax = 10;
  p.wind.gustVertical = 1;
  p.wind.turbSigma = 1.2;
};
const gusty = (p: Params) => {
  p.sim.seed = 4242;
  p.wind.gustsPerMinute = 14;
  p.wind.gustAmpMin = 4;
  p.wind.gustAmpMax = 10;
  p.wind.turbSigma = 1.2;
};
const l3 = (p: Params) => {
  p.sim.level = 3;
};
const noisySensors = (p: Params) => {
  calm(p);
  p.sensors.posNoise = 0.05;
  p.sensors.posRateHz = 50;
  p.sensors.accNoise = 0.3;
  p.sensors.accBias = 0.2;
};
const figure8 = (period: number) => (p: Params) => {
  p.setpoint.y = 3;
  p.setpoint.profile = 'figure8';
  p.setpoint.profileAmplitude = 2;
  p.setpoint.profilePeriod = period;
};
const pillar = (p: Params) => {
  p.world.pillar = true;
  p.world.pillarX = 3;
  p.world.pillarZ = 0.15;
  p.world.pillarR = 0.6;
};
const ceiling = (p: Params) => {
  calm(p);
  p.world.ceiling = 4;
  p.control.mpc = { ...p.control.mpc, qPos: 50, qVel: 5, r: 0.05 };
};
const alt = (p: Params, g: Partial<Params['control']['alt']>) => {
  p.control.alt = { ...p.control.alt, ...g };
};

const E: Exp[] = [];
const add = (e: Exp) => E.push(e);

// ─── Part I ────────────────────────────────────────────────────────────────
add({ name: 'meet', seconds: 40 });
add({
  name: 'meet-off',
  seconds: 14,
  setup: calm,
  events: (s) => s.schedule(10, (x) => x.setArmed(false)),
});
// Prediction against measurement: a drone 50 g heavier than the controller believes. The integral
// carries the missing 0.49 N until t = 10 s; then all feedback is removed (open loop), or only the
// integral is (P and D remain).
for (const [tag, off] of [
  ['open', ['pOn', 'iOn', 'dOn']],
  ['pd', ['iOn']],
] as const)
  add({
    name: `meet-wrongmass-${tag}`,
    seconds: 16,
    setup: (p) => {
      calm(p);
      p.drone.mass = 1.05;
      alt(p, { ki: 5 }); // so that the integral has settled by t = 10 s
    },
    events: (s) => {
      for (const k of off) at(s, 10, `control.alt.${k}`, false);
    },
  });
for (const kp of [10, 40])
  add({
    name: `spring-kp${kp}`,
    seconds: 20,
    setup: (p) => {
      calm(p);
      alt(p, { kp, iOn: false, dOn: false });
    },
    events: (s) => at(s, 6, 'setpoint.y', 3),
  });
// The limit cycle attracts from both sides: a small step grows towards it, the large one shrinks.
add({
  name: 'spring-small',
  seconds: 70,
  setup: (p) => {
    calm(p);
    alt(p, { iOn: false, dOn: false });
  },
  events: (s) => at(s, 6, 'setpoint.y', 2.1),
});
add({
  name: 'spring-long',
  seconds: 70,
  setup: (p) => {
    calm(p);
    alt(p, { iOn: false, dOn: false });
  },
  events: (s) => at(s, 6, 'setpoint.y', 3),
});
for (const kp of [10, 40])
  add({
    name: `droop-kp${kp}`,
    seconds: 16,
    setup: (p) => {
      calm(p);
      p.control.feedforward = false;
      alt(p, { kp, iOn: false });
    },
  });
add({
  name: 'droop-ion',
  seconds: 40,
  setup: (p) => {
    calm(p);
    p.control.feedforward = false;
    alt(p, { iOn: false });
  },
  events: (s) => at(s, 10, 'control.alt.iOn', true),
});
// Which inputs leave an error: a setpoint ramp of 0.5 m/s with the derivative on the measurement
// and on the error (I off), and a load that ramps (the drone loses 50 g/s for 10 s) with I on.
for (const on of ['measurement', 'error'] as const)
  add({
    name: `droop-ramp-${on}`,
    seconds: 22,
    setup: (p) => {
      calm(p);
      alt(p, { iOn: false, derivativeOn: on });
      p.setpoint.rateLimit = 0.5;
    },
    events: (s) => at(s, 8, 'setpoint.y', 6),
  });
for (const ki of [0.8, 3])
  add({
    name: `droop-burn-ki${ki}`,
    seconds: 36,
    setup: (p) => {
      calm(p);
      p.drone.mass = 1.5;
      p.control.model.mass = 1.5;
      alt(p, { ki });
    },
    events: (s) => {
      for (let k = 1; k <= 100; k++) at(s, 10 + k * 0.1, 'drone.mass', 1.5 - 0.005 * k);
    },
  });
for (const kd of [0.8, 3, 6.3, 15, 30])
  add({
    name: `damper-kd${kd}`,
    seconds: 24,
    setup: (p) => {
      calm(p);
      alt(p, { iOn: false, kd });
      square(0.5, 8)(p);
    },
  });
// A design from specifications (settling within 1 s, overshoot below 5 %), on a step the motors
// can follow and on one they cannot.
for (const step of [0.4, 1])
  add({
    name: `damper-spec-step${step}`,
    seconds: 12,
    setup: (p) => {
      calm(p);
      alt(p, { iOn: false, kp: 33.6, kd: 8 });
    },
    events: (s) => at(s, 6, 'setpoint.y', 2 + step),
  });
for (const [tag, iOn, ki] of [
  ['off', false, 0.8],
  ['ki0.8', true, 0.8],
  ['ki5', true, 5],
] as const)
  add({
    name: `integral-${tag}`,
    seconds: 40,
    setup: (p) => {
      calm(p);
      p.drone.mass = 1.4;
      alt(p, { iOn: false, ki });
    },
    events: (s) => iOn && at(s, 10, 'control.alt.iOn', true),
  });
for (const ki of [0.8, 3])
  add({
    name: `integral-step-ki${ki}`,
    seconds: 30,
    setup: (p) => {
      calm(p);
      alt(p, { ki });
    },
    events: (s) => at(s, 10, 'setpoint.y', 3),
  });
for (const aw of ['none', 'clamp', 'back-calculation'] as const)
  add({
    name: `windup-${aw}`,
    seconds: 30,
    setup: (p) => {
      calm(p);
      p.drone.maxMotorThrust = 3;
      alt(p, { ki: 3, antiWindup: aw });
    },
    events: (s) => at(s, 8, 'setpoint.y', 6),
  });
for (const on of ['error', 'measurement'] as const)
  add({
    name: `kick-${on}`,
    seconds: 20,
    setup: (p) => {
      calm(p);
      alt(p, { derivativeOn: on, dFilterHz: 0 });
      square(0.5, 6)(p);
    },
  });
for (const hz of [0, 20, 5, 1])
  add({
    name: `noise-f${hz}`,
    seconds: 20,
    setup: (p) => {
      calm(p);
      p.sensors.posNoise = 0.01;
      alt(p, { dFilterHz: hz });
    },
  });
for (const hz of [250, 50, 10, 7, 5])
  add({
    name: `slow-${hz}hz`,
    seconds: 30,
    setup: (p) => {
      calm(p);
      square(0.5, 8)(p);
      p.control.rateHz = hz;
    },
  });
for (const ms of [100, 200])
  add({
    name: `slow-delay${ms}`,
    seconds: 30,
    setup: (p) => {
      calm(p);
      square(0.5, 8)(p);
      p.sensors.delayMs = ms;
    },
  });
for (const kp of [10, 30, 100, 200, 400])
  add({
    name: `edge-kp${kp}`,
    seconds: 20,
    setup: (p) => {
      calm(p);
      alt(p, { iOn: false, kp });
      square(0.3, 6)(p);
    },
  });
add({ name: 'challenge', seconds: 65, setup: gustChallenge });
add({
  name: 'challenge-tuned',
  seconds: 65,
  setup: (p) => {
    gustChallenge(p);
    alt(p, { kp: 40, kd: 12, ki: 8, dFilterHz: 30 });
  },
});
add({
  name: 'tilt-l2',
  seconds: 30,
  setup: (p) => {
    p.sim.level = 2;
    p.wind.meanSpeed = 4;
    p.wind.gustVertical = 0.2;
  },
});
add({
  name: 'tilt-l3',
  seconds: 30,
  setup: (p) => {
    l3(p);
    p.wind.meanSpeed = 4;
    p.wind.gustVertical = 0.2;
  },
});
const cascadeWind = (p: Params) => {
  l3(p);
  p.wind.meanSpeed = 5;
  p.wind.gustsOn = false;
  p.wind.turbSigma = 0.3;
};
add({ name: 'cascade', seconds: 30, setup: cascadeWind });
add({
  name: 'cascade-noi',
  seconds: 30,
  setup: (p) => {
    cascadeWind(p);
    p.control.l3.velH = { ...p.control.l3.velH, iOn: false };
  },
});
add({
  name: 'cascade-step',
  seconds: 16,
  setup: (p) => {
    l3(p);
    calm(p);
  },
  events: (s) => s.schedule(10, (x) => x.setTarget(v3(1, 2, 0))),
});
for (const hz of [250, 10, 5, 3])
  add({
    name: `inversion-att${hz}`,
    seconds: 30,
    setup: (p) => {
      l3(p);
      calm(p);
      square(1.5, 8, 'x')(p);
      p.control.l3.hzAtt = hz;
    },
  });
add({
  name: 'inversion-kp1.5',
  seconds: 30,
  setup: (p) => {
    l3(p);
    calm(p);
    square(1.5, 8, 'x')(p);
    p.control.l3.attKpRP = 1.5;
  },
});

// ─── Part I, lessons 5–14: experiments behind the predictions of the book ──────────────────────
// Lesson 5: the integral gain near its Routh limit K_i < K_p K_d / m = 50 (m = 1.4 kg, small step).
for (const ki of [20, 45, 55])
  add({
    name: `integral-limit-ki${ki}`,
    seconds: 30,
    setup: (p) => {
      calm(p);
      p.drone.mass = 1.4;
      p.control.model.mass = 1.4;
      alt(p, { ki, antiWindup: 'none' });
    },
    events: (s) => at(s, 10, 'setpoint.y', 2.05),
  });
// Lesson 6: back-calculation with a gentle tracking gain, and the hard limit on the integral.
add({
  name: 'windup-kb0.3',
  seconds: 30,
  setup: (p) => {
    calm(p);
    p.drone.maxMotorThrust = 3;
    alt(p, { ki: 3, antiWindup: 'back-calculation', kb: 0.3 });
  },
  events: (s) => at(s, 8, 'setpoint.y', 6),
});
add({
  name: 'windup-ilimit',
  seconds: 30,
  setup: (p) => {
    calm(p);
    p.drone.maxMotorThrust = 3;
    alt(p, { ki: 3, antiWindup: 'integral-limit', iLimit: 3 });
  },
  events: (s) => at(s, 8, 'setpoint.y', 6),
});
// Lesson 7: the kick spread out by the D filter, and removed by a setpoint ramp.
add({
  name: 'kick-error-f5',
  seconds: 20,
  setup: (p) => {
    calm(p);
    alt(p, { derivativeOn: 'error', dFilterHz: 5 });
    square(0.5, 6)(p);
  },
});
add({
  name: 'kick-ramp',
  seconds: 20,
  setup: (p) => {
    calm(p);
    alt(p, { derivativeOn: 'error', dFilterHz: 0 });
    p.setpoint.rateLimit = 1;
  },
  events: (s) => {
    at(s, 9, 'setpoint.y', 3);
    at(s, 14, 'setpoint.y', 2);
  },
});
// Lesson 8: more cutoffs, for the curve of D-term noise against the filter.
for (const hz of [50, 10, 2])
  add({
    name: `noise-f${hz}`,
    seconds: 20,
    setup: (p) => {
      calm(p);
      p.sensors.posNoise = 0.01;
      alt(p, { dFilterHz: hz });
    },
  });
// The price of the filter: the same square wave with a fast, a slow and a very slow D filter (no noise).
for (const hz of [20, 5, 1])
  add({
    name: `noise-step-f${hz}`,
    seconds: 24,
    setup: (p) => {
      calm(p);
      alt(p, { iOn: false, dFilterHz: hz });
      square(0.5, 8)(p);
    },
  });
// Lesson 9: closer to the edge, in rate and in delay.
for (const hz of [4, 3])
  add({
    name: `slow-${hz}hz`,
    seconds: 30,
    setup: (p) => {
      calm(p);
      square(0.5, 8)(p);
      p.control.rateHz = hz;
    },
  });
for (const ms of [50, 130, 150, 170])
  add({
    name: `slow-delay${ms}`,
    seconds: 30,
    setup: (p) => {
      calm(p);
      square(0.5, 8)(p);
      p.sensors.delayMs = ms;
    },
  });
// Lesson 10: small steps (5 cm, no saturation) across the predicted edge K_p ≈ 200 at K_d = 7.
for (const kp of [50, 100, 150, 180, 200, 220, 250, 300])
  add({
    name: `edge-small-kp${kp}`,
    seconds: 14,
    setup: (p) => {
      calm(p);
      alt(p, { iOn: false, kp });
    },
    events: (s) => at(s, 6, 'setpoint.y', 2.05),
  });
// Lesson 14: one position step with a fast, a medium and a soft attitude loop.
for (const k of [3, 1.5])
  add({
    name: `inversion-step-att${k}`,
    seconds: 18,
    setup: (p) => {
      l3(p);
      calm(p);
      p.control.l3.attKpRP = k;
    },
    events: (s) => s.schedule(10, (x) => x.setTarget(v3(1, 2, 0))),
  });

// ─── Part II · A ───────────────────────────────────────────────────────────
for (const [kp, kd] of [
  [10, 2],
  [10, 6.3],
  [10, 20],
  [20, 8],
  [40, 12.65],
])
  add({
    name: `state-kp${kp}-kd${kd}`,
    seconds: 24,
    setup: (p) => {
      calm(p);
      square(0.5, 8)(p);
      alt(p, { kp, kd, iOn: false });
    },
  });
add({
  name: 'lqr-pd',
  seconds: 40,
  setup: (p) => {
    calm(p);
    square(0.5, 8)(p);
    alt(p, { iOn: false });
  },
});
for (const r of [10, 1, 0.1])
  add({
    name: `lqr-r${r}`,
    seconds: 40,
    setup: (p) => {
      calm(p);
      square(0.5, 8)(p);
      p.control.l1.kind = 'lqr';
      p.control.lqr.r = r;
    },
  });
for (const [tag, integral, qInt] of [
  ['lqr', false, 5],
  ['lqi', true, 5],
  ['lqi-q50', true, 50],
] as const)
  add({
    name: `lqi-${tag}`,
    seconds: 30,
    setup: (p) => {
      calm(p);
      p.control.l1.kind = 'lqr';
      p.control.model.mass = 0.8;
      p.control.lqr.integral = integral;
      p.control.lqr.qInt = qInt;
    },
    events: (s) => at(s, 15, 'setpoint.y', 3),
  });
for (const est of ['none', 'kalman'] as const)
  add({
    name: `kalman-${est}`,
    seconds: 30,
    setup: (p) => {
      noisySensors(p);
      p.control.l1.estimator = est;
    },
  });
add({
  name: 'kalman-overtrust',
  seconds: 30,
  setup: (p) => {
    noisySensors(p);
    p.control.l1.estimator = 'kalman';
    p.control.kalman.posSigma = 0.005;
  },
});
add({
  name: 'kalman-nobias',
  seconds: 30,
  setup: (p) => {
    noisySensors(p);
    p.control.l1.estimator = 'kalman';
    p.control.kalman.biasSigma = 0;
  },
});
add({
  name: 'kalman-distrust',
  seconds: 30,
  setup: (p) => {
    noisySensors(p);
    p.control.l1.estimator = 'kalman';
    p.control.kalman.posSigma = 0.5;
  },
});

// ─── Part II · B ───────────────────────────────────────────────────────────
for (const kind of ['pid', 'adrc'] as const)
  add({
    name: `adrc-${kind}`,
    seconds: 30,
    setup: (p) => {
      p.control.l1.kind = kind;
    },
    events: (s) => at(s, 12, 'drone.mass', 1.3),
  });
for (const wo of [4, 6, 8, 10, 15, 20, 25, 30, 40, 50, 60, 70])
  add({
    name: `adrcbw-wo${wo}`,
    seconds: 35,
    setup: (p) => {
      p.control.l1.kind = 'adrc';
      p.control.adrc.wo = wo;
      p.sensors.posNoise = 0.02;
    },
  });
for (const wo of [40, 60, 100, 150, 250])
  add({
    name: `adrcbw-clean-wo${wo}`,
    seconds: 20,
    setup: (p) => {
      calm(p);
      p.control.l1.kind = 'adrc';
      p.control.adrc.wo = wo;
    },
    events: (s) => at(s, 12, 'setpoint.y', 2.5),
  });
const damaged = (p: Params) => {
  l3(p);
  calm(p);
  p.setpoint.x = 2;
};
const brokenProp = (s: Simulation) => at(s, 12, 'drone.motorEfficiency.1', 0.6);
for (const [tag, inner, comp] of [
  ['pid', 'pid', 'none'],
  ['rate', 'indi', 'none'],
  ['acc', 'pid', 'indi'],
  ['both', 'indi', 'indi'],
] as const)
  add({
    name: `indi-prop-${tag}`,
    seconds: 25,
    setup: (p) => {
      damaged(p);
      p.control.l3.inner = inner;
      p.control.l3.compensation = comp;
    },
    events: brokenProp,
  });
for (const k of [0.5, 2, 3, 4, 6, 8, 12])
  add({
    name: `indi-prop-inertia${k}`,
    seconds: 25,
    setup: (p) => {
      damaged(p);
      p.control.l3.inner = 'indi';
      p.control.model.inertiaScale = k;
    },
    events: brokenProp,
  });
for (const [tag, inner, comp] of [
  ['pid', 'pid', 'none'],
  ['rate', 'indi', 'none'],
  ['acc', 'pid', 'indi'],
  ['both', 'indi', 'indi'],
] as const)
  add({
    name: `indi-gust-${tag}`,
    seconds: 40,
    setup: (p) => {
      l3(p);
      gusty(p);
      p.control.l3.inner = inner;
      p.control.l3.compensation = comp;
    },
  });
for (const m of [0.7, 1.5])
  add({
    name: `indi-gust-mass${m}`,
    seconds: 40,
    setup: (p) => {
      l3(p);
      gusty(p);
      p.control.l3.compensation = 'indi';
      p.control.model.mass = m;
    },
  });
// The same mismatch without any gyro noise: the wobble is an instability, not noise.
add({
  name: 'indisync-off-quiet',
  seconds: 25,
  full: true,
  setup: (p) => {
    l3(p);
    calm(p);
    square(1.5, 6, 'x')(p);
    p.control.l3.inner = 'indi';
    p.control.indi.filterHz = 5;
    p.control.indi.syncFilters = false;
  },
});
for (const [tag, sync, hz] of [
  ['off', false, 5],
  ['on', true, 5],
  ['off-6hz', false, 6],
  ['off-7hz', false, 7],
  ['off-8hz', false, 8],
  ['off-10hz', false, 10],
  ['off-20hz', false, 20],
  ['off-40hz', false, 40],
  ['on-2hz', true, 2],
  ['on-10hz', true, 10],
  ['on-20hz', true, 20],
] as const)
  add({
    name: `indisync-${tag}`,
    seconds: 25,
    full: true,
    setup: (p) => {
      l3(p);
      calm(p);
      square(1.5, 6, 'x')(p);
      p.sensors.gyroNoise = 10;
      p.control.l3.inner = 'indi';
      p.control.indi.filterHz = hz;
      p.control.indi.syncFilters = sync;
    },
  });

// ─── Part II · C ───────────────────────────────────────────────────────────
// The same flip with a yaw component: 170° about an axis tilted 35° up from the horizontal.
for (const law of ['euler', 'quaternion', 'tilt'] as const)
  add({
    name: `flipyaw-${law}`,
    seconds: 16,
    setup: (p) => {
      l3(p);
      calm(p);
      p.setpoint.y = 6;
      p.control.l3.attitude = law;
    },
    events: (s) =>
      s.schedule(10, (x) => {
        x.state.q = qMul(qFromAxisAngle(v3(1, 1, 1), (170 * Math.PI) / 180), x.state.q);
        x.state.omega = v3();
      }),
  });
for (const law of ['euler', 'quaternion', 'tilt'] as const)
  add({
    name: `flip-${law}`,
    seconds: 16,
    setup: (p) => {
      l3(p);
      calm(p);
      p.setpoint.y = 6;
      p.control.l3.attitude = law;
    },
    events: (s) =>
      s.schedule(10, (x) => {
        x.state.q = qMul(qFromAxisAngle(v3(1, 0, 1), (170 * Math.PI) / 180), x.state.q);
        x.state.omega = v3();
      }),
  });
for (const [tag, outer, ff] of [
  ['pid', 'pid-cascade', false],
  ['geo', 'geometric', false],
  ['geo-ff', 'geometric', true],
] as const)
  add({
    name: `flat-${tag}`,
    seconds: 40,
    setup: (p) => {
      l3(p);
      calm(p);
      figure8(8.9)(p);
      p.control.l3.outer = outer;
      p.control.geometric.feedforward = ff;
    },
  });
add({
  name: 'flat-geo-kp18',
  seconds: 40,
  setup: (p) => {
    l3(p);
    calm(p);
    figure8(8.9)(p);
    p.control.l3.outer = 'geometric';
    p.control.geometric.feedforward = false;
    p.control.geometric.kp = 18;
  },
});
add({
  name: 'flat-geo-ff-mass1.3',
  seconds: 40,
  setup: (p) => {
    l3(p);
    calm(p);
    figure8(8.9)(p);
    p.control.l3.outer = 'geometric';
    p.control.geometric.feedforward = true;
    p.control.model.mass = 1.3;
  },
});
for (const prof of ['corners', 'minsnap'] as const)
  add({
    name: `snap-${prof}`,
    seconds: 40,
    setup: (p) => {
      l3(p);
      calm(p);
      p.setpoint.y = 3;
      p.setpoint.profile = prof;
      p.setpoint.profileAmplitude = 1.5;
      p.setpoint.profilePeriod = 8;
      p.control.l3.outer = 'geometric';
    },
  });

// ─── Part II · D ───────────────────────────────────────────────────────────
for (const kind of ['pid', 'mpc'] as const)
  add({
    name: `mpc-${kind}`,
    seconds: 25,
    setup: (p) => {
      ceiling(p);
      p.control.l1.kind = kind;
    },
    events: (s) => at(s, 8, 'setpoint.y', 3.95),
  });
for (const h of [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1, 1.5, 2])
  add({
    name: `horizon-${h}`,
    seconds: 20,
    setup: (p) => {
      ceiling(p);
      p.control.l1.kind = 'mpc';
      p.control.mpc.horizon = h;
    },
    events: (s) => at(s, 8, 'setpoint.y', 3.95),
  });
// II.14: the lesson's 2 s horizon with the QP capped at 5 ADMM iterations per solve.
add({
  name: 'horizon-cap5',
  seconds: 20,
  setup: (p) => {
    ceiling(p);
    p.control.l1.kind = 'mpc';
    p.control.mpc.horizon = 2;
    p.control.mpc.iterations = 5;
  },
  events: (s) => at(s, 8, 'setpoint.y', 3.95),
});
for (const [tag, outer, indi] of [
  ['mpc', 'mpc', false],
  ['mpc-indi', 'mpc', true],
  ['geo-indi', 'geometric', true],
  ['geo', 'geometric', false],
] as const)
  add({
    name: `mpcl3-${tag}`,
    seconds: 30,
    setup: (p) => {
      l3(p);
      figure8(6)(p);
      gusty(p);
      p.control.l3.outer = outer;
      if (indi) {
        p.control.l3.inner = 'indi';
        p.control.l3.compensation = 'indi';
      }
    },
  });
for (const [tag, outer, lambda] of [
  ['geo', 'geometric', 0],
  ['mppi', 'mppi', 0],
  ['mppi-l50', 'mppi', 50],
  ['mppi-l0.2', 'mppi', 0.2],
  ['mppi-l500', 'mppi', 500],
] as const)
  add({
    name: `mppi-${tag}`,
    seconds: 16,
    setup: (p) => {
      l3(p);
      calm(p);
      pillar(p);
      p.control.l3.outer = outer;
      if (lambda) p.control.mppi.lambda = lambda;
    },
    events: (s) => at(s, 10, 'setpoint.x', 6),
  });
for (const [tag, outer, safety, gamma] of [
  ['geo', 'geometric', 'none', 0],
  ['cbf', 'geometric', 'cbf', 0],
  ['cbf-g1', 'geometric', 'cbf', 1],
  ['cbf-g4', 'geometric', 'cbf', 4],
] as const)
  add({
    name: `cbf-${tag}`,
    seconds: 40,
    setup: (p) => {
      l3(p);
      calm(p);
      pillar(p);
      p.setpoint.profile = 'sine';
      p.setpoint.profileAxis = 'x';
      p.setpoint.profileAmplitude = 4;
      p.setpoint.profilePeriod = 10;
      p.control.l3.outer = outer;
      p.control.l3.safety = safety;
      if (gamma) p.control.cbf.gamma = gamma;
    },
  });

// ─── Part II · E ───────────────────────────────────────────────────────────
for (const hz of [50, 10, 3, 1])
  add({
    name: `l1ac-f${hz}`,
    seconds: 25,
    setup: (p) => {
      p.control.l1.kind = 'l1ac';
      p.control.l1ac.filterHz = hz;
      p.sensors.velNoise = 0.1;
    },
    events: (s) => at(s, 12, 'drone.mass', 1.3),
  });
for (const comp of ['none', 'learned', 'l1ac', 'indi'] as const)
  add({
    name: `residual-${comp}`,
    seconds: 40,
    setup: (p) => {
      l3(p);
      p.drone.rotorDrag = 0.02;
      p.wind.meanSpeed = 3;
      p.wind.gustsOn = false;
      p.wind.turbSigma = 0.5;
      figure8(6.5)(p);
      p.control.l3.outer = 'geometric';
      p.control.l3.compensation = comp;
    },
  });
for (const outer of ['geometric', 'policy'] as const)
  add({
    name: `policy-${outer}`,
    seconds: 35,
    setup: (p) => {
      l3(p);
      calm(p);
      p.control.l3.outer = outer;
    },
    events: (s) => {
      at(s, 10, 'setpoint.x', 3);
      at(s, 20, 'drone.mass', 1.5);
    },
  });

// ─── Run ───────────────────────────────────────────────────────────────────
const filter = process.argv[2];
const t0 = performance.now();
for (const e of E) {
  if (filter && !e.name.includes(filter)) continue;
  const p = defaultParams();
  e.setup?.(p);
  const sim = new Simulation(p);
  if (e.events) {
    sim.script = e.events;
    sim.reset();
  }
  for (let k = 0; k < e.seconds * 1000; k++) sim.step();
  const tl = sim.telemetry;
  const names = [...(tl as unknown as { channels: Map<string, Float64Array> }).channels.keys()];
  const { t, series } = tl.window(names, 0);
  const keep = names
    .map((n, i) => [n, series[i]!] as const)
    .filter(([, s]) => s.some((v) => Number.isFinite(v)));
  const rows = [`t,${keep.map(([n]) => n).join(',')}`];
  for (let k = 0; k < t.length; k += e.full ? 1 : 2) {
    const row = [t[k]!.toFixed(3)];
    for (const [, s] of keep) {
      const v = s[k]!;
      row.push(Number.isFinite(v) ? v.toPrecision(6) : '');
    }
    rows.push(row.join(','));
  }
  writeFileSync(new URL(`${e.name}.csv.gz`, OUT), gzipSync(rows.join('\n')));
  const crash = sim.state.crashed ? ' CRASHED' : '';
  console.log(
    `${e.name.padEnd(22)} ${keep.length} channels, zone ${sim.zoneTime.toFixed(2)} s${crash}`,
  );
}
console.log(`${((performance.now() - t0) / 1000).toFixed(0)} s`);
