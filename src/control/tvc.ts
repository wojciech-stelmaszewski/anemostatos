import { PHYS_DT } from '@/engine/simulation';
import { c2dTustin, gain, series, ss, type Lti } from '@/math/lti';
import type { TvcState, TvcParams, TvcInput } from '@/sim/vehicles/tvc';
import { tvcMeasure } from '@/sim/vehicles/tvc';
import type { LoopTerms } from './types';

/**
 * The thrust-vector controller of Chapter L (docs/aerospace-gnc.md §4): a PD on the pitch the
 * gyro reports, an optional outer loop on the drift across the flight path, and two filters on
 * the gimbal command, a first-order roll-off and a notch. Linear, so it is one discrete LTI;
 * the simulator runs exactly the matrices the loop analysis uses.
 *
 *   θr = θc + kx·(xr − x) − kv·u
 *   δc = F(s)·[kp·(θm − θr) + kd·ωm]
 *
 * A positive gimbal angle turns the nose to −x, so a pitch above the reference asks for +δ.
 */
export interface TvcControlParams {
  /** Pitch loop: gimbal per pitch error, °/°, and per pitch rate, °/(°/s). */
  kp: number;
  kd: number;
  /** Drift loop: pitch reference per metre of drift, °/m, and per m/s of drift rate (0: off). */
  kx: number;
  kv: number;
  /** Pitch command, °, and drift target, m (lessons step them). */
  pitchCmdDeg: number;
  xTarget: number;
  /** Notch on the gimbal command: on, centre, Hz, depth at the centre, dB, and width (pole damping). */
  notch: boolean;
  notchHz: number;
  notchDepthDb: number;
  notchWidth: number;
  /** First-order roll-off on the gimbal command, Hz (0: none). */
  rolloffHz: number;
  /** Delay of the gyro and navigation data, ms. */
  delayMs: number;
}

/** Order of the controller's inputs. */
export const TVC_IN = { theta: 0, omega: 1, x: 2, u: 3, xr: 4, thetaCmd: 5 } as const;

const DEG = Math.PI / 180;

/** The continuous-time filter on the gimbal command (identity when both filters are off). */
function filter(c: TvcControlParams): Lti {
  let f: Lti = gain([[1]]);
  if (c.rolloffHz > 0) {
    const w = 2 * Math.PI * c.rolloffHz;
    f = series(f, ss([[-w]], [[w]], [[1]], [[0]]));
  }
  if (c.notch && c.notchHz > 0) {
    // (s² + 2ζz·ω·s + ω²)/(s² + 2ζp·ω·s + ω²): at ω its gain is ζz/ζp.
    const w = 2 * Math.PI * c.notchHz;
    const zp = Math.max(c.notchWidth, 1e-3);
    const zz = zp * 10 ** (-Math.abs(c.notchDepthDb) / 20);
    // Controllable form; y = u + (2(ζz − ζp)ω)·x2.
    f = series(
      f,
      ss(
        [
          [0, 1],
          [-w * w, -2 * zp * w],
        ],
        [[0], [1]],
        [[0, 2 * (zz - zp) * w]],
        [[1]],
      ),
    );
  }
  return f;
}

/** The whole controller as a discrete LTI at the physics step: inputs `TVC_IN`, output δc (rad). */
export function tvcControllerModel(c: TvcControlParams, dt = PHYS_DT): Lti {
  const kp = c.kp;
  const kd = c.kd; // °/(°/s) = s
  const kx = c.kx * DEG;
  const kv = c.kv * DEG;
  // e = kp·(θm − θc − kx·(xr − x) + kv·u) + kd·ωm
  const row = [kp, kd, kp * kx, kp * kv, -kp * kx, -kp];
  return c2dTustin(series(gain([row]), filter(c)), dt);
}

/** The controller at run time: the model's matrices, with the measurements delayed. */
export class TvcController {
  private model: Lti | null = null;
  private key = '';
  private xc: number[] = [];
  private buffer: [number, number, number, number][] = [];
  last: LoopTerms = emptyTerms();
  /** The outer loop on the drift across the flight path; its output is the pitch reference, °. */
  drift: LoopTerms = emptyTerms();

  reset(): void {
    this.key = '';
    this.model = null;
    this.xc = [];
    this.buffer = [];
    this.last = emptyTerms();
    this.drift = emptyTerms();
  }

  tick(s: TvcState, r: TvcParams, c: TvcControlParams): TvcInput {
    const key = JSON.stringify(c);
    if (key !== this.key) {
      // A new design: keep the filter states only if the structure is the same.
      const m = tvcControllerModel(c);
      if (!this.model || m.a.length !== this.xc.length) this.xc = new Array(m.a.length).fill(0);
      this.model = m;
      this.key = key;
    }
    const m = this.model!;
    const d = Math.max(0, Math.round(c.delayMs / 1000 / PHYS_DT));
    this.buffer.push(tvcMeasure(s, r));
    while (this.buffer.length > d + 1) this.buffer.shift();
    const meas = this.buffer[0]!;
    const y = [meas[0], meas[1], meas[2], meas[3], c.xTarget, c.pitchCmdDeg * DEG];
    const n = this.xc.length;
    let out = m.d[0]!.reduce((acc, k, j) => acc + k * y[j]!, 0);
    for (let j = 0; j < n; j++) out += m.c[0]![j]! * this.xc[j]!;
    const next = new Array<number>(n).fill(0);
    for (let i = 0; i < n; i++) {
      let v = 0;
      for (let j = 0; j < n; j++) v += m.a[i]![j]! * this.xc[j]!;
      for (let j = 0; j < y.length; j++) v += m.b[i]![j]! * y[j]!;
      next[i] = v;
    }
    this.xc = next;
    const lim = r.gimbalMaxDeg * DEG;
    const gimbal = Math.min(Math.max(out, -lim), lim) / DEG;
    const ex = c.xTarget - meas[2];
    const thetaRef = c.pitchCmdDeg + c.kx * ex - c.kv * meas[3]; // °
    this.last = {
      setpoint: thetaRef,
      measurement: meas[0] / DEG,
      error: thetaRef - meas[0] / DEG,
      parts: [{ key: 'p', label: 'gimbal', value: gimbal, like: 'p' }],
      unsaturated: out / DEG,
      output: gimbal,
      saturated: Math.abs(out) > lim,
    };
    this.drift = {
      setpoint: c.xTarget,
      measurement: meas[2],
      error: ex,
      parts: [
        { key: 'p', label: 'kx·e', value: c.kx * ex, like: 'p' },
        { key: 'd', label: '−kv·u', value: -c.kv * meas[3], like: 'd' },
        { key: 'ff', label: 'pitch command', value: c.pitchCmdDeg, ff: true, like: 'ff' },
      ],
      unsaturated: thetaRef,
      output: thetaRef,
      saturated: false,
    };
    return { delta: out };
  }
}

function emptyTerms(): LoopTerms {
  return {
    setpoint: 0,
    measurement: 0,
    error: 0,
    parts: [{ key: 'p', label: 'gimbal', value: 0, like: 'p' }],
    unsaturated: 0,
    output: 0,
    saturated: false,
  };
}
