// Measured frequency response (docs/analysis.md §3.2): fly the real simulation with a sine on the
// probe and correlate what comes back. Works for any controller, linear or not.
import { PHYS_DT, Simulation } from '@/engine/simulation';
import { cdiv, cneg, type Complex } from '@/math/complex';
import type { Params } from '@/sim/params';

export interface SweepPoint {
  /** The frequency actually used, Hz (snapped so that the window holds whole periods). */
  fHz: number;
  /** Open loop at the plant input, L = −Uc/U (thrust probe only). */
  l?: Complex;
  /** Sensitivity S = U/D and complementary sensitivity T = −Uc/D (thrust probe only). */
  s?: Complex;
  t?: Complex;
  /** Closed loop from the reference to the altitude, Y/R (reference probe only). */
  yr?: Complex;
  /**
   * Torque probes: one column of the 2 × 2 sensitivity and complementary sensitivity at the
   * plant input, as [roll, pitch] responses to the probe on this point's axis.
   */
  sCol?: [Complex, Complex];
  tCol?: [Complex, Complex];
}

export interface SweepOptions {
  point?: 'l1.thrust' | 'ref.y' | 'l3.torque.x' | 'l3.torque.z';
  /** Probe amplitude: N for the thrust probe, m for the reference probe. */
  amp?: number;
  /** Time to take off and settle before the first frequency, s. */
  settleS?: number;
  /** Time to let the transient die after each change of frequency: at least this, s… */
  transientS?: number;
  /** …and at least this many periods. */
  transientPeriods?: number;
  /** The measurement window: at least this many periods and this many seconds. */
  minPeriods?: number;
  minSeconds?: number;
}

const DEFAULTS: Required<SweepOptions> = {
  point: 'l1.thrust',
  amp: 0.3,
  settleS: 8,
  transientS: 20,
  transientPeriods: 3,
  minPeriods: 4,
  minSeconds: 2,
};

/** Calm air, clean sensors and a fixed setpoint: the conditions a sweep is flown in. */
export function sweepParams(p: Params): Params {
  const q = structuredClone(p);
  q.wind.enabled = false;
  q.sensors = { ...q.sensors, posNoise: 0, velNoise: 0, accNoise: 0, gyroNoise: 0 };
  q.setpoint = { ...q.setpoint, profile: 'none', rateLimit: 0 };
  q.probe = { ...q.probe, point: 'none' };
  return q;
}

/**
 * A sweep that can be advanced in slices, so that the interface stays responsive
 * (`advance` from an animation frame) and tests can run it to the end (`runSweep`).
 */
export class Sweep {
  readonly points: SweepPoint[] = [];
  private sim: Simulation;
  private opt: Required<SweepOptions>;
  private index = -1;
  private phase: 'settle' | 'transient' | 'measure' | 'done' = 'settle';
  private left: number;
  private total = 0;
  private fHz = 0;
  /** Running correlation sums, real and imaginary parts, of: in, u, uc, y, τx, τz, τc,x, τc,z. */
  private acc = new Float64Array(16);
  private k = 0;

  constructor(
    params: Params,
    private freqsHz: readonly number[],
    options: SweepOptions = {},
  ) {
    this.opt = { ...DEFAULTS, ...options };
    this.sim = new Simulation(sweepParams(params));
    this.left = Math.round(this.opt.settleS / PHYS_DT);
  }

  get done(): boolean {
    return this.phase === 'done';
  }

  /** Fraction of the frequencies finished. */
  get progress(): number {
    return this.done ? 1 : Math.max(0, this.index) / Math.max(this.freqsHz.length, 1);
  }

  /** True if the flight ended on the ground or in saturation of the run (crash). */
  get crashed(): boolean {
    return this.sim.state.crashed;
  }

  private next(): void {
    this.index++;
    if (this.index >= this.freqsHz.length || this.sim.state.crashed) {
      this.phase = 'done';
      return;
    }
    const f = this.freqsHz[this.index]!;
    const periods = Math.max(this.opt.minPeriods, Math.ceil(this.opt.minSeconds * f));
    // Snap the frequency so that the window is a whole number of samples.
    this.total = Math.max(8, Math.round(periods / (f * PHYS_DT)));
    this.fHz = periods / (this.total * PHYS_DT);
    const p = this.sim.params;
    this.sim.setParams({
      ...p,
      probe: {
        ...p.probe,
        point: this.opt.point,
        signal: 'sine',
        amp: this.opt.amp,
        freqHz: this.fHz,
      },
    });
    this.phase = 'transient';
    this.left = Math.round(
      Math.max(this.opt.transientS, this.opt.transientPeriods / this.fHz) / PHYS_DT,
    );
  }

  private finish(): void {
    const a = this.acc;
    const c = (i: number): Complex => ({ re: a[2 * i]!, im: a[2 * i + 1]! });
    const d = c(0);
    const u = c(1);
    const uc = c(2);
    const y = c(3);
    if (this.opt.point === 'ref.y') this.points.push({ fHz: this.fHz, yr: cdiv(y, d) });
    else if (this.opt.point === 'l1.thrust')
      this.points.push({
        fHz: this.fHz,
        l: cneg(cdiv(uc, u)),
        s: cdiv(u, d),
        t: cneg(cdiv(uc, d)),
      });
    else
      this.points.push({
        fHz: this.fHz,
        sCol: [cdiv(c(4), d), cdiv(c(5), d)],
        tCol: [cneg(cdiv(c(6), d)), cneg(cdiv(c(7), d))],
      });
  }

  /** Run up to `maxSteps` physics steps. Returns true when the sweep is finished. */
  advance(maxSteps: number): boolean {
    const sim = this.sim;
    let budget = maxSteps;
    while (budget > 0 && this.phase !== 'done') {
      if (this.phase === 'measure') {
        const w = 2 * Math.PI * this.fHz * PHYS_DT;
        const n = Math.min(budget, this.left);
        const a = this.acc;
        for (let i = 0; i < n; i++) {
          // The altitude before the step belongs to the same instant as the step's commands.
          const v3 = sim.state.pos.y;
          sim.step();
          const cs = Math.cos(w * this.k);
          const sn = Math.sin(w * this.k);
          const v0 = sim.probe.in;
          const v1 = sim.probe.u;
          const v2 = sim.probe.uc;
          a[0] = a[0]! + v0 * cs;
          a[1] = a[1]! - v0 * sn;
          a[2] = a[2]! + v1 * cs;
          a[3] = a[3]! - v1 * sn;
          a[4] = a[4]! + v2 * cs;
          a[5] = a[5]! - v2 * sn;
          a[6] = a[6]! + v3 * cs;
          a[7] = a[7]! - v3 * sn;
          const pr = sim.probe;
          const more = [pr.tau.x, pr.tau.z, pr.tauC.x, pr.tauC.z];
          for (let j = 0; j < 4; j++) {
            a[8 + 2 * j] = a[8 + 2 * j]! + more[j]! * cs;
            a[9 + 2 * j] = a[9 + 2 * j]! - more[j]! * sn;
          }
          this.k++;
        }
        budget -= n;
        this.left -= n;
        if (this.left === 0) {
          this.finish();
          this.next();
        }
      } else {
        const n = Math.min(budget, this.left);
        for (let i = 0; i < n; i++) sim.step();
        budget -= n;
        this.left -= n;
        if (this.left === 0) {
          if (this.phase === 'settle') this.next();
          else {
            this.phase = 'measure';
            this.left = this.total;
            this.k = 0;
            this.acc.fill(0);
          }
        }
      }
    }
    return this.done;
  }
}

/** Fly a whole sweep and return its points. */
export function runSweep(
  params: Params,
  freqsHz: readonly number[],
  options: SweepOptions = {},
): SweepPoint[] {
  const s = new Sweep(params, freqsHz, options);
  while (!s.advance(1_000_000));
  return s.points;
}
