import { rank } from '@/math/mat';
import { qConj, qRotate } from '@/math/quat';
import { clamp } from '@/math/util';
import { add, cross, dot, length, normalize, scale, sub, v3, type Vec3 } from '@/math/vec3';
import { idleActuation, type Actuation } from '@/sim/dynamics';
import { MOTOR_POSITIONS } from '@/sim/drone';
import { GRAVITY, type Params } from '@/sim/params';
import { CascadeController } from './cascade';
import { modelInertia } from './stages';
import type { ControlInput, Controller, InfoRow, LoopTerms } from './types';

const UP = v3(0, 1, 0);

/**
 * Flight after the loss of a rotor (docs/analysis.md §4, Chapter J; [Mueller 2014]).
 *
 * Four motors set thrust and three torques. Three motors set only three of those four, and with
 * thrust and roll and pitch held the yaw torque is whatever is left: the drone spins. So the
 * controller gives up yaw. It controls the direction of the thrust axis (a reduced attitude, two
 * angles) and the thrust, and solves for the three working motors exactly. The spin settles where
 * the yaw torque meets the rotor drag.
 *
 * Before the fault it is the ordinary cascade. Which motor failed comes from the fault source:
 * here an oracle that reads the true motor efficiency (lesson III.27); lesson III.28 replaces it
 * with a detector.
 */
export class FaultTolerantController implements Controller {
  readonly cascade: CascadeController;
  /** Index of the motor flown without, or −1 while all four work. */
  lost = -1;
  private held: Actuation = idleActuation();
  private integral = v3();
  private last = { tilt: 0, spin: 0, thrust: 0, err: 0 };
  /** The position loop's terms per axis, for the charts once the three-motor law flies. */
  private posTerms: Record<'x' | 'y' | 'z', LoopTerms> | null = null;

  constructor(p: Params) {
    this.cascade = new CascadeController(p);
  }

  get gyroUsed(): Vec3 {
    return this.cascade.gyroUsed;
  }

  reset(): void {
    this.cascade.reset();
    this.lost = -1;
    this.held = idleActuation();
    this.integral = v3();
    this.posTerms = null;
  }

  resetIntegrators(): void {
    this.cascade.resetIntegrators();
    this.integral = v3();
  }

  /** The motor the fault source reports as lost, or −1. */
  private source(p: Params, input: ControlInput): number {
    if (p.control.fault.source === 'monitor') return input.fdi?.motor ?? -1;
    const eff = p.drone.motorEfficiency;
    for (let i = 0; i < 4; i++) if (eff[i]! < 0.5) return i;
    return -1;
  }

  tick(input: ControlInput): Actuation {
    const P = input.params;
    if (this.lost < 0) this.lost = this.source(P, input);
    if (this.lost < 0) return this.cascade.tick(input);

    const f = P.control.fault;
    const s = input.sense();
    const dt = input.physDt;
    const m = P.control.model.mass;
    const I = modelInertia(P);
    const fmax = P.drone.maxMotorThrust;

    // 1. Position PID → desired acceleration → desired direction of the thrust axis.
    const ep = sub(input.setpoint.pos, s.pos);
    this.integral = add(this.integral, scale(ep, dt));
    const iLim = 2;
    this.integral = v3(
      clamp(this.integral.x, -iLim, iLim),
      clamp(this.integral.y, -iLim, iLim),
      clamp(this.integral.z, -iLim, iLim),
    );
    let a = add(add(scale(ep, f.posKp), scale(s.vel, -f.posKd)), scale(this.integral, f.posKi));
    const sp = input.setpoint.pos;
    const terms = (k: 'x' | 'y' | 'z'): LoopTerms => {
      const p = f.posKp * ep[k];
      const d = -f.posKd * s.vel[k];
      const i = f.posKi * this.integral[k];
      return {
        setpoint: sp[k],
        measurement: s.pos[k],
        error: ep[k],
        parts: [
          { key: 'p', label: 'P', value: p, like: 'p' },
          { key: 'i', label: 'I', value: i, like: 'i' },
          { key: 'd', label: 'D (on velocity)', value: d, like: 'd' },
        ],
        unsaturated: p + i + d,
        output: p + i + d,
        saturated: false,
      };
    };
    this.posTerms = { x: terms('x'), y: terms('y'), z: terms('z') };
    a = add(a, v3(0, GRAVITY, 0));
    a = v3(a.x, Math.max(a.y, 0.3 * GRAVITY), a.z);
    const hMax = a.y * Math.tan((f.maxTiltDeg * Math.PI) / 180);
    const h = Math.hypot(a.x, a.z);
    if (h > hMax) a = v3((a.x * hMax) / h, a.y, (a.z * hMax) / h);
    const nDes = normalize(a);

    // 2. Reduced attitude: turn the body's up-axis onto nDes. Yaw is left free.
    const n = qRotate(s.q, UP);
    const eB = qRotate(qConj(s.q), cross(n, nDes));
    const w = s.omega;
    const wDes = v3(f.attKp * eB.x, w.y, f.attKp * eB.z);

    // 3. Body rates → torque, with the gyroscopic coupling of the spin cancelled.
    const alpha = scale(sub(wDes, w), f.rateKp);
    const Iw = v3(I.x * w.x, I.y * w.y, I.z * w.z);
    const gyro = cross(w, Iw);
    let tx = I.x * alpha.x + gyro.x + P.drone.angularDamping * w.x;
    let tz = I.z * alpha.z + gyro.z + P.drone.angularDamping * w.z;
    // The motors lag by τ and the sensor by its delay; in the spinning body the torque turns at
    // the spin rate, so command it ahead by the angle both would cost.
    if (f.leadLag) {
      const phi = Math.atan(w.y * P.control.model.motorTau) + (w.y * P.sensors.delayMs) / 1000;
      const k = Math.hypot(1, w.y * P.control.model.motorTau);
      const c = Math.cos(phi);
      const sn = Math.sin(phi);
      [tx, tz] = [k * (c * tx - sn * tz), k * (sn * tx + c * tz)];
    }
    const thrust = Math.max(m * dot(a, n), 0);

    // 4. Three motors, three demands: thrust, roll torque, pitch torque. Solve exactly.
    const use = [0, 1, 2, 3].filter((i) => i !== this.lost);
    const A = [
      use.map(() => 1),
      use.map((i) => -MOTOR_POSITIONS[i]!.z),
      use.map((i) => MOTOR_POSITIONS[i]!.x),
    ];
    const sol = solve3(A, [thrust, tx, tz]);
    const cmd: [number, number, number, number] = [0, 0, 0, 0];
    use.forEach((i, k) => (cmd[i] = clamp(sol[k]!, 0, fmax)));
    this.held = { motorCmd: cmd, forceCmd: v3() };
    this.last = {
      tilt: (Math.acos(clamp(dot(n, nDes), -1, 1)) * 180) / Math.PI,
      spin: w.y,
      thrust,
      err: length(ep),
    };
    return this.held;
  }

  loops(): Record<string, LoopTerms> {
    const cascade = this.cascade.loops();
    if (!this.posTerms) return cascade;
    // After the fault the cascade no longer runs: its position loops would stand still.
    return {
      ...cascade,
      'pos.x': this.posTerms.x,
      'pos.y': this.posTerms.y,
      'pos.z': this.posTerms.z,
    };
  }

  extras(): Record<string, number> {
    return {
      ...this.cascade.extras(),
      'fault.lost': this.lost + 1,
      'fault.tiltErr': this.last.tilt,
      'fault.spin': this.last.spin,
    };
  }

  describe(p: Params, loopId: string): InfoRow[] {
    const r = rank(effectiveness(p));
    const rows = [
      {
        label: 'control effectiveness',
        value: `rank ${r} of 4`,
        hint:
          r === 4
            ? 'Four motors set thrust, roll, pitch and yaw torque independently.'
            : 'The working motors set only three of thrust, roll, pitch and yaw: with the first three held, the yaw torque is whatever is left over.',
      },
      ...this.cascade.describe(p, loopId),
    ];
    if (this.lost < 0) return rows;
    return [
      {
        label: 'fault-tolerant mode',
        value: `motor ${this.lost + 1} lost: yaw given up, spin ${this.last.spin.toFixed(1)} rad/s`,
        hint: 'Three motors control thrust and the direction of the thrust axis; the yaw torque left over spins the drone.',
      },
      ...rows,
    ];
  }
}

/** Solve a 3 × 3 linear system by Cramer's rule (the allocation is always well conditioned). */
function solve3(A: number[][], b: number[]): number[] {
  const det = (m: number[][]) =>
    m[0]![0]! * (m[1]![1]! * m[2]![2]! - m[1]![2]! * m[2]![1]!) -
    m[0]![1]! * (m[1]![0]! * m[2]![2]! - m[1]![2]! * m[2]![0]!) +
    m[0]![2]! * (m[1]![0]! * m[2]![1]! - m[1]![1]! * m[2]![0]!);
  const d = det(A);
  return [0, 1, 2].map((c) => det(A.map((row, r) => row.map((v, k) => (k === c ? b[r]! : v)))) / d);
}

/**
 * The control effectiveness of the motors: thrust and the three torques per newton of each motor
 * (the inverse of the mixer). Its rank is how many of the four the motors can still set.
 */
export function effectiveness(p: Params): number[][] {
  const e = p.drone.motorEfficiency;
  const c = p.drone.torqueCoeff;
  const spin = [1, -1, 1, -1];
  return [
    [0, 1, 2, 3].map((i) => e[i]!),
    [0, 1, 2, 3].map((i) => -MOTOR_POSITIONS[i]!.z * e[i]!),
    [0, 1, 2, 3].map((i) => c * spin[i]! * e[i]!),
    [0, 1, 2, 3].map((i) => MOTOR_POSITIONS[i]!.x * e[i]!),
  ];
}
