import { add, mul, solve, sub, transpose, type Mat } from '@/math/mat';
import { qRotate, type Quat } from '@/math/quat';
import { v3, type Vec3 } from '@/math/vec3';

/**
 * Navigation EKF, loosely coupled (docs/analysis.md §4, Chapter I, lesson III.24).
 *
 * State (p, v, b_a): position and velocity in the world frame and the accelerometer bias in the
 * body frame, nine numbers. The accelerometer, turned into the world frame with the attitude the
 * flight computer believes, drives the prediction at the IMU rate; a GPS-like receiver corrects
 * the position at 10 Hz and a barometer the altitude. Each fix is tested by its normalised
 * innovation squared against a χ² gate before it is used.
 */
export interface NavSettings {
  /** Accelerometer noise the filter assumes, m/s², and how fast its bias may wander. */
  accSigma: number;
  biasWalk: number;
  /** The receiver's and the barometer's noise, m. */
  gpsSigma: number;
  baroSigma: number;
  /** χ² gate on the NIS of a GPS fix (3 degrees of freedom) and of the barometer (1); 0 = none. */
  gate: number;
}

export interface FixUpdate {
  nis: number;
  accepted: boolean;
}

const eye = (n: number): Mat =>
  Array.from({ length: n }, (_, i) => Array.from({ length: n }, (__, j) => (i === j ? 1 : 0)));

export class NavEkf {
  x: number[] = new Array<number>(9).fill(0);
  p: Mat = eye(9);
  started = false;
  lastGps: FixUpdate = { nis: 0, accepted: true };
  lastBaro: FixUpdate = { nis: 0, accepted: true };

  reset(pos?: Vec3): void {
    this.x = new Array<number>(9).fill(0);
    if (pos) [this.x[0], this.x[1], this.x[2]] = [pos.x, pos.y, pos.z];
    this.p = eye(9).map((row, i) => row.map((v) => v * (i < 3 ? 0.25 : i < 6 ? 0.04 : 0.01)));
    this.started = !!pos;
    this.lastGps = { nis: 0, accepted: true };
    this.lastBaro = { nis: 0, accepted: true };
  }

  get pos(): Vec3 {
    return v3(this.x[0]!, this.x[1]!, this.x[2]!);
  }
  get vel(): Vec3 {
    return v3(this.x[3]!, this.x[4]!, this.x[5]!);
  }
  sigma(i: number): number {
    return Math.sqrt(Math.max(this.p[i]![i]!, 0));
  }

  /** Predict with the specific force `acc` (body frame) and the attitude `q` (body → world). */
  propagate(acc: Vec3, q: Quat, gravity: number, s: NavSettings, dt: number): void {
    const b = v3(this.x[6]!, this.x[7]!, this.x[8]!);
    const fw = qRotate(q, v3(acc.x - b.x, acc.y - b.y, acc.z - b.z));
    const a = [fw.x, fw.y - gravity, fw.z];
    for (let i = 0; i < 3; i++) {
      this.x[i] = this.x[i]! + this.x[i + 3]! * dt + 0.5 * a[i]! * dt * dt;
      this.x[i + 3] = this.x[i + 3]! + a[i]! * dt;
    }
    // F: ṗ = v, v̇ = −R·b_a.
    const r = [v3(1, 0, 0), v3(0, 1, 0), v3(0, 0, 1)].map((e) => qRotate(q, e)); // columns of R
    const f = eye(9);
    for (let i = 0; i < 3; i++) {
      f[i]![i + 3] = dt;
      for (let j = 0; j < 3; j++) {
        const rij = [r[j]!.x, r[j]!.y, r[j]!.z][i]!;
        f[i + 3]![j + 6] = -rij * dt;
      }
    }
    const qn = eye(9).map((row, i) =>
      row.map((v) => v * (i < 3 ? 0 : i < 6 ? s.accSigma ** 2 * dt : s.biasWalk ** 2 * dt)),
    );
    this.p = add(mul(mul(f, this.p), transpose(f)), qn);
  }

  private update(h: Mat, z: number[], sigma: number, gate: number): FixUpdate {
    const m = h.length;
    const r = eye(m).map((row) => row.map((v) => v * sigma * sigma));
    const s = add(mul(mul(h, this.p), transpose(h)), r);
    const nu = z.map((v, i) => v - h[i]!.reduce((acc, hv, j) => acc + hv * this.x[j]!, 0));
    const sInvNu = solve(
      s,
      nu.map((v) => [v]),
    );
    const nis = nu.reduce((acc, v, i) => acc + v * sInvNu[i]![0]!, 0);
    const accepted = !(gate > 0 && nis > gate);
    if (accepted) {
      const k = transpose(solve(s, mul(h, this.p)));
      for (let i = 0; i < 9; i++)
        this.x[i] = this.x[i]! + k[i]!.reduce((acc, kv, j) => acc + kv * nu[j]!, 0);
      const ikh = sub(eye(9), mul(k, h));
      this.p = add(mul(mul(ikh, this.p), transpose(ikh)), mul(mul(k, r), transpose(k)));
    }
    return { nis, accepted };
  }

  /** A position fix. The gate is the χ² quantile for three degrees of freedom. */
  gps(fix: Vec3, s: NavSettings): FixUpdate {
    const h = [0, 1, 2].map((i) => Array.from({ length: 9 }, (_, j) => (j === i ? 1 : 0)));
    this.lastGps = this.update(h, [fix.x, fix.y, fix.z], s.gpsSigma, s.gate);
    return this.lastGps;
  }

  /** An altitude from the barometer; its gate is the same quantile scaled to one degree. */
  baro(alt: number, s: NavSettings): FixUpdate {
    const h = [Array.from({ length: 9 }, (_, j) => (j === 1 ? 1 : 0))];
    // χ²₁ at the probability where χ²₃ = gate: about gate − 5 for the usual quantiles.
    const gate1 = s.gate > 0 ? Math.max(s.gate - 5, 1) : 0;
    this.lastBaro = this.update(h, [alt], s.baroSigma, gate1);
    return this.lastBaro;
  }
}
