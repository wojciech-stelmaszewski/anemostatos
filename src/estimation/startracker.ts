import { Rng } from '@/math/prng';
import { qConj, qMul, qRotate } from '@/math/quat';
import { normalize, v3, type Vec3 } from '@/math/vec3';
import type { SatelliteParams, SatelliteState } from '@/sim/vehicles/satellite';
import { Mekf, type MekfSettings } from './mekf';

/** 1 arc-second in radians; and 1 °/h is exactly 1 arc-second per second. */
export const ARCSEC = Math.PI / (180 * 3600);
/** The gyro is sampled and the filter propagated at this rate, Hz. */
const GYRO_HZ = 100;
/** Two stars the tracker sees, fixed in inertial space (unit vectors). */
const STARS: Vec3[] = [normalize(v3(1, 0.2, 0.1)), normalize(v3(-0.1, 0.3, 1))];

/**
 * Attitude knowledge of the satellite (lesson IV.22): the multiplicative EKF of lesson III.23,
 * propagated with a gyro that has a bias and noise, and corrected by a star tracker that measures
 * the directions of two stars every 1/`trackerHz` seconds. The controller still flies on the
 * truth; this only shows how well the attitude is known.
 */
export class StarNavigator {
  mekf = new Mekf();
  private rng = new Rng(1);
  private nextGyro = 0;
  private nextTracker = 0;
  /** The tracker is blind now (an outage: the Sun in its field of view). */
  blind = false;

  reset(s: SatelliteState, sp: SatelliteParams, seed: number): void {
    this.rng = new Rng(seed ^ 0x5a7);
    const biasSigma = sp.estimateBias ? 10 * ARCSEC : 0; // 10 °/h
    this.mekf.reset(s.q, 20 * ARCSEC, biasSigma);
    this.nextGyro = 0;
    this.nextTracker = 0;
    this.blind = false;
  }

  /** The filter's assumptions: the gyro noise it has, and a bias that may drift only if estimated. */
  private settings(sp: SatelliteParams): MekfSettings {
    return {
      gyroSigma: (sp.gyroArw * Math.PI) / 180 / 60, // °/√h → rad/√s
      biasWalk: sp.estimateBias ? 1e-3 * ARCSEC : 0,
      accSigma: 0,
      magSigma: 0,
      gate: 0,
    };
  }

  tick(s: SatelliteState, sp: SatelliteParams, t: number): void {
    if (t + 1e-9 >= this.nextGyro) {
      const dt = 1 / GYRO_HZ;
      this.nextGyro += dt;
      const sigma = (sp.gyroArw * Math.PI) / 180 / 60 / Math.sqrt(dt);
      const b = sp.gyroBiasDegH;
      const gyro = v3(
        s.w.x + b.x * ARCSEC + sigma * this.rng.normal(),
        s.w.y + b.y * ARCSEC + sigma * this.rng.normal(),
        s.w.z + b.z * ARCSEC + sigma * this.rng.normal(),
      );
      this.mekf.propagate(gyro, this.settings(sp), dt);
    }
    this.blind = sp.outageLength > 0 && t >= sp.outageStart && t < sp.outageStart + sp.outageLength;
    if (sp.trackerHz > 0 && t + 1e-9 >= this.nextTracker) {
      this.nextTracker += 1 / sp.trackerHz;
      if (this.blind) return;
      const n = sp.trackerArcsec * ARCSEC;
      for (const star of STARS) {
        // The star's direction in the body frame, turned by a small random error.
        const body = qRotate(qConj(s.q), star);
        const e = v3(n * this.rng.normal(), n * this.rng.normal(), n * this.rng.normal());
        const meas = normalize(
          v3(
            body.x + (e.y * body.z - e.z * body.y),
            body.y + (e.z * body.x - e.x * body.z),
            body.z + (e.x * body.y - e.y * body.x),
          ),
        );
        this.mekf.updateVector(meas, star, n, 0);
      }
    }
  }

  /** The knowledge error: the angle between the true and the estimated attitude, arc-seconds. */
  error(s: SatelliteState): number {
    const d = qMul(qConj(s.q), this.mekf.q);
    return (2 * Math.acos(Math.min(1, Math.abs(d.w)))) / ARCSEC;
  }

  /** The filter's own 2σ of that error, arc-seconds. */
  sigma2(): number {
    const m = this.mekf;
    return (2 * Math.hypot(m.sigma(0), m.sigma(1), m.sigma(2))) / ARCSEC;
  }

  /** How far the bias estimate is from the truth, °/h. */
  biasError(sp: SatelliteParams): number {
    const b = this.mekf.bias;
    const t = sp.gyroBiasDegH;
    return Math.hypot(b.x / ARCSEC - t.x, b.y / ARCSEC - t.y, b.z / ARCSEC - t.z);
  }
}
