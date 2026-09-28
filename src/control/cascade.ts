import { QpSolver } from '@/math/qp';
import { qFromAxisAngle, qFromTo, qMul, qToEuler, qRotate, type Quat } from '@/math/quat';
import { clamp } from '@/math/util';
import { clampLength, dot, length, normalize, v3, type Vec3 } from '@/math/vec3';
import { idleActuation, type Actuation } from '@/sim/dynamics';
import { GRAVITY, type Params } from '@/sim/params';
import type { Measurement } from '@/sim/sensors';
import { attitudeRates, flatnessRates, GeometricOuter } from './geometric';
import { safetyFilter } from './cbf';
import { mix } from './mixer';
import { mpcTiming, MpcOuter } from './mpc';
import { MppiOuter } from './mppi';
import { defaultGains, Pid, type PidTerms } from './pid';
import {
  AccelIndi,
  NoCompensation,
  PidRateStage,
  RateIndi,
  type Compensation,
  type RateStage,
} from './stages';
import {
  periodSteps,
  pidLoop,
  type ControlInput,
  type Controller,
  type InfoRow,
  type LoopTerms,
  type Plan,
} from './types';

const DEG = 180 / Math.PI;
const up = v3(0, 1, 0);

/** A proportional-only loop reported in the same shape as a PID (for the inspector). */
const pTerms = (
  setpoint: number,
  measurement: number,
  error: number,
  output: number,
  limited: number,
): PidTerms => ({
  setpoint,
  measurement,
  error,
  p: output,
  i: 0,
  d: 0,
  ff: 0,
  unsaturated: output,
  output: limited,
  saturated: output !== limited,
});

/**
 * Level 3 as a pipeline of stages (docs/beyond-pid.md §3.2):
 *   position P → velocity PID → [compensation] → thrust vector → attitude P → [rate stage] → mixer.
 * The outer stage is the PID cascade; compensation (none / acceleration INDI) and the rate stage
 * (PID / INDI) are swappable. Each loop runs at its own rate; angles in degrees.
 */
export class CascadeController implements Controller {
  readonly pos = { x: new Pid(), y: new Pid(), z: new Pid() };
  readonly vel = { x: new Pid(), y: new Pid(), z: new Pid() };
  readonly rate: RateStage;
  readonly comp: Compensation;
  /** Set when the outer stage is the geometric tracking controller instead of pos P + vel PID. */
  readonly geo: GeometricOuter | null;
  readonly mpc: MpcOuter | null;
  readonly mppi: MppiOuter | null;
  private readonly cbfSolver = new QpSolver();
  /** Desired acceleration before the safety filter, and how many barriers were active. */
  private aNominal = v3();
  private cbfActive = 0;
  private aSafe = v3();
  private refNow = v3();
  private lastPos = v3();
  att: Record<'roll' | 'pitch' | 'yaw', PidTerms> = {
    roll: pTerms(0, 0, 0, 0, 0),
    pitch: pTerms(0, 0, 0, 0, 0),
    yaw: pTerms(0, 0, 0, 0, 0),
  };

  private vSp = v3();
  private aSp = v3();
  private fDes = v3(0, GRAVITY, 0);
  private qSp: Quat = { w: 1, x: 0, y: 0, z: 0 };
  private thrust = 0;
  /** Body-rate setpoint, deg/s: x roll, y yaw, z pitch. */
  private wSp = v3();
  /** Body-rate feedforward from the reference's jerk, deg/s. */
  private wFf = v3();
  private jerk: Vec3 | undefined;
  private held: Actuation = idleActuation();
  private mixerSaturated = false;

  constructor(p?: Params) {
    this.rate = p?.control.l3.inner === 'indi' ? new RateIndi() : new PidRateStage();
    this.comp = p?.control.l3.compensation === 'indi' ? new AccelIndi() : new NoCompensation();
    this.geo = p?.control.l3.outer === 'geometric' ? new GeometricOuter() : null;
    this.mpc = p?.control.l3.outer === 'mpc' ? new MpcOuter() : null;
    this.mppi = p?.control.l3.outer === 'mppi' ? new MppiOuter(p.sim.seed) : null;
  }

  reset(): void {
    for (const p of [...Object.values(this.pos), ...Object.values(this.vel)]) p.reset();
    this.rate.reset();
    this.comp.reset();
    this.geo?.reset();
    this.mpc?.reset();
    this.mppi?.reset();
    this.cbfSolver.reset();
    this.aNominal = v3();
    this.aSafe = v3();
    this.cbfActive = 0;
    this.wFf = v3();
    this.jerk = undefined;
    this.vSp = v3();
    this.aSp = v3();
    this.fDes = v3(0, GRAVITY, 0);
    this.qSp = { w: 1, x: 0, y: 0, z: 0 };
    this.thrust = 0;
    this.wSp = v3();
    this.held = idleActuation();
    this.mixerSaturated = false;
  }

  resetIntegrators(): void {
    for (const p of Object.values(this.vel)) p.integral = 0;
    this.rate.resetIntegrators();
    this.geo?.resetIntegrators();
  }

  tick(input: ControlInput): Actuation {
    const { params: P, physDt, stepIndex, setpoint } = input;
    const c = P.control.l3;
    const due = (hz: number) => stepIndex % periodSteps(hz, physDt) === 0;
    const dtOf = (hz: number) => periodSteps(hz, physDt) * physDt;
    let m: Measurement | null = null;
    const sense = () => (m ??= input.sense());
    const mHat = P.control.model.mass;
    const fmax = P.drone.maxMotorThrust;

    // 0. Fast estimators of the compensation stage (filters) run with the rate loop.
    if (due(c.hzRate)) this.comp.sample(sense(), P, dtOf(c.hzRate));

    const predictive = this.mpc || this.mppi;
    // 1. Position P → velocity setpoint (PID cascade only).
    if (!this.geo && !predictive && due(c.hzPos)) {
      const s = sense();
      const dt = dtOf(c.hzPos);
      const gH = defaultGains({ kp: c.posKpH, ki: 0, kd: 0 });
      const gV = defaultGains({ kp: c.posKpV, ki: 0, kd: 0 });
      const vx = this.pos.x.update(gH, setpoint.pos.x, s.pos.x, dt).output;
      const vz = this.pos.z.update(gH, setpoint.pos.z, s.pos.z, dt).output;
      const vy = this.pos.y.update(gV, setpoint.pos.y, s.pos.y, dt, 0, -c.vMaxV, c.vMaxV).output;
      const h = clampLength(v3(vx, 0, vz), c.vMaxH);
      this.vSp = v3(h.x, vy, h.z);
    }

    // 2. Velocity PID → acceleration → [compensation] → thrust vector → attitude setpoint.
    if (due(c.hzVel)) {
      const s = sense();
      const dt = dtOf(c.hzVel);
      const aH = GRAVITY * Math.tan((c.maxTiltDeg * Math.PI) / 180);
      const aUp = (4 * fmax) / Math.max(mHat, 0.1) - GRAVITY;
      // With an incremental (INDI) compensation the velocity loop's I term is redundant:
      // the increment already integrates.
      const noI = this.comp.replacesIntegral;
      if (this.geo) {
        this.aSp = this.geo.update(setpoint, s, dt, P, noI);
        this.jerk = P.control.geometric.feedforward ? setpoint.jerk : undefined;
      } else if (predictive) {
        // Re-plan at the planner's own rate; hold the first move in between.
        this.refNow = setpoint.pos;
        const t = stepIndex * physDt;
        if (this.mpc && due(mpcTiming(P).hz)) {
          this.lastPos = s.pos;
          this.aSp = this.mpc.update(s, input.preview, setpoint.pos, P, t);
        }
        if (this.mppi && due(P.control.mppi.hz)) {
          this.aSp = this.mppi.update(s, input.preview ?? (() => setpoint.pos), P, t);
        }
      } else {
        const velH = noI ? { ...c.velH, iOn: false } : c.velH;
        const velV = noI ? { ...c.velV, iOn: false } : c.velV;
        const ax = this.vel.x.update(velH, this.vSp.x, s.vel.x, dt, 0, -aH, aH).output;
        const az = this.vel.z.update(velH, this.vSp.z, s.vel.z, dt, 0, -aH, aH).output;
        const ay = this.vel.y.update(velV, this.vSp.y, s.vel.y, dt, 0, -0.8 * GRAVITY, aUp).output;
        this.aSp = v3(ax, ay, az);
      }
      // Safety filter: the closest acceleration that keeps out of the zones.
      this.aNominal = this.aSp;
      this.cbfActive = 0;
      let aCmd = this.aSp;
      if (c.safety === 'cbf') {
        const r = safetyFilter(this.aSp, s, P, 3, this.cbfSolver);
        aCmd = r.a;
        this.cbfActive = r.active;
      }
      this.aSafe = aCmd;
      this.lastPos = s.pos;
      let f = this.comp.force(aCmd, P);
      f = v3(f.x, Math.max(f.y, 0.05 * mHat * GRAVITY), f.z);
      // Tilt limit: keep the vertical component, shrink the horizontal one.
      const hMax = f.y * Math.tan((c.maxTiltDeg * Math.PI) / 180);
      const h = clampLength(v3(f.x, 0, f.z), hMax);
      f = v3(h.x, f.y, h.z);
      this.fDes = f;
      const qYaw = qFromAxisAngle(up, setpoint.yaw);
      this.qSp = qMul(qFromTo(up, normalize(f)), qYaw);
    }

    // 3. Attitude P → body-rate setpoint (deg/s).
    if (due(c.hzAtt)) {
      const s = sense();
      const att = attitudeRates(c.attitude, s.q, this.qSp, c.attKpRP, c.attKpYaw);
      const err = att.err;
      this.wFf = flatnessRates(this.fDes, this.jerk, mHat, s.q);
      const raw = v3(att.raw.x + this.wFf.x, att.raw.y + this.wFf.y, att.raw.z + this.wFf.z);
      const lim = c.maxRateDeg;
      this.wSp = v3(clamp(raw.x, -lim, lim), clamp(raw.y, -lim, lim), clamp(raw.z, -lim, lim));
      const eSp = qToEuler(this.qSp);
      const eNow = qToEuler(s.q);
      this.att = {
        roll: pTerms(eSp.roll * DEG, eNow.roll * DEG, err.x, raw.x, this.wSp.x),
        yaw: pTerms(eSp.yaw * DEG, eNow.yaw * DEG, err.y, raw.y, this.wSp.y),
        pitch: pTerms(eSp.pitch * DEG, eNow.pitch * DEG, err.z, raw.z, this.wSp.z),
      };
      // Collective thrust: desired force projected on the current body up-axis.
      this.thrust = clamp(dot(this.fDes, qRotate(s.q, up)), 0, 4 * fmax);
    }

    // 4. Rate stage → torque → mixer.
    if (due(c.hzRate)) {
      const s = sense();
      const tau = this.rate.torque(this.wSp, s, dtOf(c.hzRate), P);
      const r = mix(this.thrust, tau.x, tau.y, tau.z, P.drone.torqueCoeff, fmax);
      this.mixerSaturated = r.saturated;
      this.rate.applied(r.motors);
      this.comp.applied(r.motors);
      this.held = { motorCmd: r.motors, forceCmd: this.held.forceCmd };
    }

    return this.held;
  }

  loops(): Record<string, LoopTerms> {
    const outer: Record<string, LoopTerms> = this.geo
      ? this.geo.loops()
      : this.mpc || this.mppi
        ? this.plannerLoops()
        : {
            'pos.x': pidLoop(this.pos.x.last),
            'pos.y': pidLoop(this.pos.y.last),
            'pos.z': pidLoop(this.pos.z.last),
            'vel.x': pidLoop(this.vel.x.last),
            'vel.y': pidLoop(this.vel.y.last),
            'vel.z': pidLoop(this.vel.z.last),
          };
    return {
      ...outer,
      'att.roll': pidLoop(this.att.roll),
      'att.pitch': pidLoop(this.att.pitch),
      'att.yaw': pidLoop(this.att.yaw),
      ...this.rate.loops(),
      ...this.comp.loops(this.aSp),
    };
  }

  extras(): Record<string, number> {
    const tilt = Math.acos(clamp(this.fDes.y / Math.max(length(this.fDes), 1e-9), -1, 1)) * DEG;
    return {
      'fdes.x': this.fDes.x,
      'fdes.y': this.fDes.y,
      'fdes.z': this.fDes.z,
      'fdes.tilt': tilt,
      thrust: this.thrust,
      mixerSat: this.mixerSaturated ? 1 : 0,
      'cbf.active': this.cbfActive,
      'cbf.dx': this.aSafe.x - this.aNominal.x,
      'cbf.dy': this.aSafe.y - this.aNominal.y,
      'cbf.dz': this.aSafe.z - this.aNominal.z,
      'ff.rate.roll': this.wFf.x,
      'ff.rate.pitch': this.wFf.z,
      'q.sp.w': this.qSp.w,
      'q.sp.x': this.qSp.x,
      'q.sp.y': this.qSp.y,
      'q.sp.z': this.qSp.z,
      ...this.rate.extras(),
      ...this.comp.extras(),
    };
  }

  describe(p: Params, loopId: string): InfoRow[] {
    const rows: InfoRow[] = [];
    if (this.geo && loopId.startsWith('pos.')) {
      const g = p.control.geometric;
      const wn = Math.sqrt(g.kp);
      rows.push(
        {
          label: 'law',
          value: 'a = a_ref − Kp·e_p − Kv·e_v − Ki·∫e',
          hint: 'Position and velocity closed in one law; the reference acceleration is fed forward.',
        },
        {
          label: 'error dynamics ωₙ · ζ',
          value: `${wn.toFixed(2)} rad/s · ${(g.kv / (2 * wn)).toFixed(2)}`,
          hint: 'ë + Kv·ė + Kp·e = 0: the error behaves like a mass–spring–damper, whatever the trajectory.',
        },
        {
          label: 'feedforward',
          value: g.feedforward ? 'a_ref + body rates from jerk' : 'off (feedback only)',
        },
      );
    }
    return [...rows, ...this.rate.describe(p, loopId), ...this.comp.describe(p, loopId)];
  }

  /** Position loops of a planner: reference vs measurement, output = first planned acceleration. */
  private plannerLoops(): Record<string, LoopTerms> {
    const out: Record<string, LoopTerms> = {};
    for (const k of ['x', 'y', 'z'] as const) {
      const pos = this.lastPos[k];
      out[`pos.${k}`] = {
        setpoint: this.refNow[k],
        measurement: pos,
        error: this.refNow[k] - pos,
        parts: [
          { key: 'plan', label: 'a₀ (first move of the plan)', value: this.aNominal[k], like: 'p' },
        ],
        unsaturated: this.aNominal[k],
        output: this.aNominal[k],
        saturated: false,
      };
    }
    return out;
  }

  plan(): Plan | null {
    return this.mpc?.plan() ?? this.mppi?.plan() ?? null;
  }

  /** Correction the safety filter applied to the desired acceleration (zero when inactive). */
  get safetyCorrection(): Vec3 {
    return v3(
      this.aSafe.x - this.aNominal.x,
      this.aSafe.y - this.aNominal.y,
      this.aSafe.z - this.aNominal.z,
    );
  }

  /** Desired thrust vector, world frame — drawn as an arrow. */
  get desiredForce(): Vec3 {
    return this.fDes;
  }
}
