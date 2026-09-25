import type { Level } from '@/sim/params';

/**
 * Declarative description of every tunable parameter. The parameter panel is generated from this,
 * and the short `help` texts are the in-app explanations (docs/ui-and-visualization.md §4).
 */
export type Field =
  | {
      kind: 'number';
      path: string;
      label: string;
      unit?: string;
      min: number;
      max: number;
      step: number;
      log?: boolean;
      help?: string;
      levels?: Level[];
    }
  | { kind: 'bool'; path: string; label: string; help?: string; levels?: Level[] }
  | {
      kind: 'select';
      path: string;
      label: string;
      options: { value: string; label: string }[];
      help?: string;
      levels?: Level[];
    };

export interface Group {
  id: string;
  title: string;
  levels?: Level[];
  fields: Field[];
  /** Colour hint for PID groups: shows which loop this is. */
  loop?: string;
}

const pid = (
  prefix: string,
  opts: {
    kp: [number, number];
    ki: [number, number];
    kd: [number, number];
    unit?: string;
    levels?: Level[];
  },
): Field[] => [
  {
    kind: 'number',
    path: `${prefix}.kp`,
    label: 'Kp',
    min: opts.kp[0],
    max: opts.kp[1],
    step: 0.01,
    log: true,
    help: 'Proportional gain — reacts to where you are. Higher: stiffer and faster, but more overshoot and eventually oscillation.',
  },
  {
    kind: 'number',
    path: `${prefix}.ki`,
    label: 'Ki',
    min: opts.ki[0],
    max: opts.ki[1],
    step: 0.01,
    log: true,
    help: 'Integral gain — reacts to where you have been. Removes steady-state error (constant loads like gravity or steady wind), adds overshoot.',
  },
  {
    kind: 'number',
    path: `${prefix}.kd`,
    label: 'Kd',
    min: opts.kd[0],
    max: opts.kd[1],
    step: 0.01,
    log: true,
    help: 'Derivative gain — reacts to where you are going. Damps motion like a shock absorber; amplifies sensor noise.',
  },
  {
    kind: 'bool',
    path: `${prefix}.pOn`,
    label: 'P term',
    help: 'Switch the proportional term on/off.',
  },
  {
    kind: 'bool',
    path: `${prefix}.iOn`,
    label: 'I term',
    help: 'Switch the integral term on/off (clears the integral).',
  },
  {
    kind: 'bool',
    path: `${prefix}.dOn`,
    label: 'D term',
    help: 'Switch the derivative term on/off.',
  },
  {
    kind: 'select',
    path: `${prefix}.derivativeOn`,
    label: 'D acts on',
    options: [
      { value: 'measurement', label: 'measurement' },
      { value: 'error', label: 'error' },
    ],
    help: 'Differentiating the error makes the D term spike on every setpoint step ("derivative kick"). Differentiating the measurement gives the same damping without the kick.',
  },
  {
    kind: 'number',
    path: `${prefix}.dFilterHz`,
    label: 'D filter',
    unit: 'Hz',
    min: 0,
    max: 200,
    step: 0.5,
    help: 'Low-pass cutoff on the derivative. Lower = smoother but more lag. 0 = no filter.',
  },
  {
    kind: 'select',
    path: `${prefix}.antiWindup`,
    label: 'Anti-windup',
    options: [
      { value: 'none', label: 'none' },
      { value: 'clamp', label: 'conditional integration' },
      { value: 'integral-limit', label: 'integral limit' },
      { value: 'back-calculation', label: 'back-calculation' },
    ],
    help: 'What to do with the integral while the actuator is saturated. Without protection it keeps growing ("winds up") and causes a big overshoot later.',
  },
  {
    kind: 'number',
    path: `${prefix}.iLimit`,
    label: 'I limit',
    unit: opts.unit,
    min: 0,
    max: 50,
    step: 0.1,
    help: 'Maximum |I| for the "integral limit" anti-windup strategy.',
  },
  {
    kind: 'number',
    path: `${prefix}.kb`,
    label: 'Kb',
    unit: '1/s',
    min: 0,
    max: 50,
    step: 0.1,
    help: 'Tracking gain of back-calculation anti-windup: how fast the integral is bled off during saturation.',
  },
];

const L12: Level[] = [1, 2];

export const SCHEMA: Group[] = [
  {
    id: 'setpoint',
    title: 'Setpoint',
    fields: [
      {
        kind: 'number',
        path: 'setpoint.y',
        label: 'Altitude',
        unit: 'm',
        min: 0.2,
        max: 10,
        step: 0.05,
        help: 'Target height of the centre of mass above the floor. Keys ↑/↓ change it by 0.1 m (Shift: 1 m).',
      },
      {
        kind: 'number',
        path: 'setpoint.x',
        label: 'X',
        unit: 'm',
        min: -10,
        max: 10,
        step: 0.05,
        levels: [2, 3],
      },
      {
        kind: 'number',
        path: 'setpoint.z',
        label: 'Z',
        unit: 'm',
        min: -10,
        max: 10,
        step: 0.05,
        levels: [2, 3],
      },
      {
        kind: 'number',
        path: 'setpoint.yawDeg',
        label: 'Yaw',
        unit: '°',
        min: -180,
        max: 180,
        step: 1,
        levels: [3],
      },
      {
        kind: 'number',
        path: 'setpoint.rateLimit',
        label: 'Rate limit',
        unit: 'm/s',
        min: 0,
        max: 5,
        step: 0.05,
        help: 'Move the setpoint gradually instead of jumping. 0 = instant steps (best for studying step responses).',
      },
    ],
  },
  {
    id: 'alt',
    title: 'Altitude PID',
    loop: 'alt',
    levels: L12,
    fields: [
      {
        kind: 'bool',
        path: 'control.feedforward',
        label: 'Gravity feedforward',
        help: 'Add the thrust needed to hover (m̂·g) directly. The PID then only fights deviations. Turn it off to see the "gravity droop" of a P controller.',
      },
      {
        kind: 'number',
        path: 'control.massEstimate',
        label: 'Assumed mass m̂',
        unit: 'kg',
        min: 0.2,
        max: 3,
        step: 0.01,
        help: 'What the controller believes the mass is (used by the feedforward). A wrong value leaves an error only the I term can remove.',
      },
      {
        kind: 'number',
        path: 'control.rateHz',
        label: 'Controller rate',
        unit: 'Hz',
        min: 2,
        max: 1000,
        step: 1,
        log: true,
        help: 'How often the controller samples the sensor and updates its output. Too slow → delay → oscillation → instability.',
      },
      ...pid('control.alt', { kp: [0.1, 200], ki: [0.01, 100], kd: [0.01, 100], unit: 'N' }),
    ],
  },
  {
    id: 'posH',
    title: 'Horizontal PID (X, Z)',
    loop: 'pos.x',
    levels: [2],
    fields: [
      {
        kind: 'number',
        path: 'control.maxHForce',
        label: 'Max horizontal force',
        unit: 'N',
        min: 0.5,
        max: 20,
        step: 0.1,
      },
      ...pid('control.posH', { kp: [0.1, 100], ki: [0.01, 50], kd: [0.01, 50], unit: 'N' }),
    ],
  },
  {
    id: 'l3pos',
    title: 'Position P',
    loop: 'pos.x',
    levels: [3],
    fields: [
      {
        kind: 'number',
        path: 'control.l3.posKpH',
        label: 'Kp horizontal',
        unit: '1/s',
        min: 0.05,
        max: 10,
        step: 0.01,
        log: true,
        help: 'Position error (m) → velocity setpoint (m/s). Pure P is enough: the velocity loop below provides integral action.',
      },
      {
        kind: 'number',
        path: 'control.l3.posKpV',
        label: 'Kp vertical',
        unit: '1/s',
        min: 0.05,
        max: 10,
        step: 0.01,
        log: true,
      },
      {
        kind: 'number',
        path: 'control.l3.vMaxH',
        label: 'Max horiz. speed',
        unit: 'm/s',
        min: 0.2,
        max: 15,
        step: 0.1,
      },
      {
        kind: 'number',
        path: 'control.l3.vMaxV',
        label: 'Max vert. speed',
        unit: 'm/s',
        min: 0.2,
        max: 8,
        step: 0.1,
      },
      {
        kind: 'number',
        path: 'control.l3.hzPos',
        label: 'Loop rate',
        unit: 'Hz',
        min: 1,
        max: 1000,
        step: 1,
        log: true,
      },
    ],
  },
  {
    id: 'l3velH',
    title: 'Velocity PID — horizontal',
    loop: 'vel.x',
    levels: [3],
    fields: [
      {
        kind: 'number',
        path: 'control.l3.maxTiltDeg',
        label: 'Max tilt',
        unit: '°',
        min: 5,
        max: 60,
        step: 1,
        help: 'The drone accelerates sideways by tilting; this caps how far.',
      },
      {
        kind: 'number',
        path: 'control.l3.hzVel',
        label: 'Loop rate',
        unit: 'Hz',
        min: 1,
        max: 1000,
        step: 1,
        log: true,
      },
      ...pid('control.l3.velH', { kp: [0.05, 50], ki: [0.01, 20], kd: [0.001, 10], unit: 'm/s²' }),
    ],
  },
  {
    id: 'l3velV',
    title: 'Velocity PID — vertical',
    loop: 'vel.y',
    levels: [3],
    fields: [
      { kind: 'bool', path: 'control.feedforward', label: 'Gravity feedforward' },
      {
        kind: 'number',
        path: 'control.massEstimate',
        label: 'Assumed mass m̂',
        unit: 'kg',
        min: 0.2,
        max: 3,
        step: 0.01,
      },
      ...pid('control.l3.velV', { kp: [0.05, 50], ki: [0.01, 20], kd: [0.001, 10], unit: 'm/s²' }),
    ],
  },
  {
    id: 'l3att',
    title: 'Attitude P',
    loop: 'att.roll',
    levels: [3],
    fields: [
      {
        kind: 'number',
        path: 'control.l3.attKpRP',
        label: 'Kp roll/pitch',
        unit: '1/s',
        min: 0.5,
        max: 40,
        step: 0.1,
        log: true,
        help: 'Angle error (°) → body-rate setpoint (°/s).',
      },
      {
        kind: 'number',
        path: 'control.l3.attKpYaw',
        label: 'Kp yaw',
        unit: '1/s',
        min: 0.1,
        max: 20,
        step: 0.1,
        log: true,
      },
      {
        kind: 'number',
        path: 'control.l3.maxRateDeg',
        label: 'Max rate',
        unit: '°/s',
        min: 30,
        max: 1500,
        step: 10,
      },
      {
        kind: 'number',
        path: 'control.l3.hzAtt',
        label: 'Loop rate',
        unit: 'Hz',
        min: 1,
        max: 1000,
        step: 1,
        log: true,
        help: 'Make this slower than the velocity loop to see why inner loops must be faster than outer ones.',
      },
    ],
  },
  {
    id: 'l3rateRP',
    title: 'Rate PID — roll/pitch',
    loop: 'rate.roll',
    levels: [3],
    fields: [
      {
        kind: 'number',
        path: 'control.l3.hzRate',
        label: 'Loop rate',
        unit: 'Hz',
        min: 10,
        max: 1000,
        step: 1,
        log: true,
      },
      ...pid('control.l3.rateRP', {
        kp: [0.5, 200],
        ki: [0.01, 100],
        kd: [0.001, 5],
        unit: '°/s²',
      }),
    ],
  },
  {
    id: 'l3rateYaw',
    title: 'Rate PID — yaw',
    loop: 'rate.yaw',
    levels: [3],
    fields: [
      ...pid('control.l3.rateYaw', {
        kp: [0.5, 200],
        ki: [0.01, 100],
        kd: [0.001, 5],
        unit: '°/s²',
      }),
    ],
  },
  {
    id: 'physics',
    title: 'Physics',
    fields: [
      {
        kind: 'number',
        path: 'drone.mass',
        label: 'True mass',
        unit: 'kg',
        min: 0.3,
        max: 2.5,
        step: 0.01,
        help: 'The real mass. Change it mid-flight to simulate picking up or dropping a payload.',
      },
      {
        kind: 'number',
        path: 'drone.maxMotorThrust',
        label: 'Max thrust / motor',
        unit: 'N',
        min: 3,
        max: 15,
        step: 0.1,
        help: 'Actuator limit. Total available thrust is 4× this. Low values make saturation and windup easy to provoke.',
      },
      {
        kind: 'number',
        path: 'drone.motorTau',
        label: 'Motor time constant',
        unit: 's',
        min: 0.001,
        max: 0.3,
        step: 0.001,
        log: true,
        help: 'Motors need time to spin up (first-order lag). Lag limits how aggressive the gains can be.',
      },
      {
        kind: 'number',
        path: 'drone.dragV',
        label: 'Vertical drag',
        unit: 'N·s²/m²',
        min: 0,
        max: 2,
        step: 0.01,
        help: 'Quadratic air drag on vertical motion relative to the air. This is how vertical wind pushes the drone.',
      },
      {
        kind: 'number',
        path: 'drone.dragH',
        label: 'Horizontal drag',
        unit: 'N·s²/m²',
        min: 0,
        max: 2,
        step: 0.01,
        levels: [2, 3],
      },
    ],
  },
  {
    id: 'wind',
    title: 'Wind',
    fields: [
      { kind: 'bool', path: 'wind.enabled', label: 'Wind' },
      {
        kind: 'number',
        path: 'wind.meanSpeed',
        label: 'Mean speed',
        unit: 'm/s',
        min: 0,
        max: 15,
        step: 0.1,
        levels: [2, 3],
        help: 'A constant wind is a constant disturbance — exactly what the I term exists for.',
      },
      {
        kind: 'number',
        path: 'wind.meanHeading',
        label: 'Mean heading',
        unit: '°',
        min: 0,
        max: 360,
        step: 1,
        levels: [2, 3],
      },
      {
        kind: 'number',
        path: 'wind.meanVertical',
        label: 'Mean vertical',
        unit: 'm/s',
        min: -8,
        max: 8,
        step: 0.1,
        help: 'Steady updraft (+) or downdraft (−).',
      },
      {
        kind: 'number',
        path: 'wind.turbSigma',
        label: 'Turbulence σ',
        unit: 'm/s',
        min: 0,
        max: 5,
        step: 0.05,
        help: 'Intensity of continuous random fluctuation (Ornstein–Uhlenbeck process).',
      },
      {
        kind: 'number',
        path: 'wind.turbTau',
        label: 'Turbulence τ',
        unit: 's',
        min: 0.05,
        max: 10,
        step: 0.05,
        log: true,
        help: 'Correlation time. Small = jittery, large = slow wandering.',
      },
      { kind: 'bool', path: 'wind.gustsOn', label: 'Random gusts' },
      {
        kind: 'number',
        path: 'wind.gustsPerMinute',
        label: 'Gusts / minute',
        min: 0,
        max: 60,
        step: 0.5,
      },
      {
        kind: 'number',
        path: 'wind.gustAmpMin',
        label: 'Gust min',
        unit: 'm/s',
        min: 0,
        max: 20,
        step: 0.1,
      },
      {
        kind: 'number',
        path: 'wind.gustAmpMax',
        label: 'Gust max',
        unit: 'm/s',
        min: 0,
        max: 25,
        step: 0.1,
      },
      {
        kind: 'number',
        path: 'wind.gustDurMin',
        label: 'Gust min duration',
        unit: 's',
        min: 0.1,
        max: 10,
        step: 0.1,
      },
      {
        kind: 'number',
        path: 'wind.gustDurMax',
        label: 'Gust max duration',
        unit: 's',
        min: 0.1,
        max: 10,
        step: 0.1,
      },
      {
        kind: 'number',
        path: 'wind.gustVertical',
        label: 'Gust vertical share',
        min: 0,
        max: 1,
        step: 0.01,
        help: 'Vertical component of the gust direction. In the altitude-only level only this part matters.',
      },
    ],
  },
  {
    id: 'sensors',
    title: 'Sensors',
    fields: [
      {
        kind: 'number',
        path: 'sensors.posNoise',
        label: 'Position noise σ',
        unit: 'm',
        min: 0,
        max: 0.2,
        step: 0.001,
        help: 'Random error on every position reading. Watch what it does to the D term.',
      },
      {
        kind: 'number',
        path: 'sensors.velNoise',
        label: 'Velocity noise σ',
        unit: 'm/s',
        min: 0,
        max: 1,
        step: 0.005,
        levels: [3],
      },
      {
        kind: 'number',
        path: 'sensors.gyroNoise',
        label: 'Gyro noise σ',
        unit: '°/s',
        min: 0,
        max: 50,
        step: 0.1,
        levels: [3],
      },
      {
        kind: 'number',
        path: 'sensors.altBias',
        label: 'Altimeter bias',
        unit: 'm',
        min: -1,
        max: 1,
        step: 0.01,
        help: 'A constant sensor offset. Feedback cannot fix a lying sensor: the drone settles at the wrong height.',
      },
      {
        kind: 'number',
        path: 'sensors.delayMs',
        label: 'Sensor delay',
        unit: 'ms',
        min: 0,
        max: 500,
        step: 1,
        help: 'Transport delay of the measurement. Delay eats stability margin.',
      },
    ],
  },
];

export const getIn = (obj: unknown, path: string): unknown =>
  path.split('.').reduce<unknown>((o, k) => (o as Record<string, unknown>)?.[k], obj);

export function setIn<T>(obj: T, path: string, value: unknown): T {
  const [head, ...rest] = path.split('.');
  const o = obj as Record<string, unknown>;
  return { ...o, [head!]: rest.length ? setIn(o[head!], rest.join('.'), value) : value } as T;
}
