import { v3, type Vec3 } from '@/math/vec3';
import { GRAVITY, type DroneParams, type VibrationParams } from './params';

/**
 * Rotor vibration as the IMU feels it (docs/analysis.md §3.4): one sinusoid per motor at its
 * rotor frequency. A sensor-level signal: it shakes the readings, not the rigid body.
 */

/** Rotor frequency at a given thrust, Hz: thrust grows with the square of the rotor speed. */
export const rotorHz = (thrust: number, drone: DroneParams, hoverHz: number): number =>
  hoverHz * Math.sqrt(Math.max(thrust, 0) / ((drone.mass * GRAVITY) / 4));

/** Where each rotor starts in its turn, rad: unequal, so that four equal rotors do not cancel. */
const PHASE0 = [0, 1.3, 2.9, 4.1] as const;

export class Vibration {
  private phase: number[] = [...PHASE0];

  reset(): void {
    this.phase = [...PHASE0];
  }

  /**
   * Advance the rotor phases by dt and return what is added to the gyro (rad/s) and to the
   * accelerometer (m/s²), body frame. An unbalanced rotor pushes on its arm once per turn, in the
   * rotor plane, with a force that grows like its thrust.
   */
  step(
    rotors: readonly number[],
    drone: DroneParams,
    p: VibrationParams,
    dt: number,
  ): { gyro: Vec3; acc: Vec3 } {
    const hover = (drone.mass * GRAVITY) / 4;
    const g = (p.gyroDeg * Math.PI) / 180;
    let gx = 0;
    let gz = 0;
    let ax = 0;
    let ay = 0;
    let az = 0;
    for (let i = 0; i < 4; i++) {
      const thrust = Math.max(rotors[i]!, 0);
      const turn = 2 * Math.PI * rotorHz(thrust, drone, p.hoverHz) * dt;
      this.phase[i] = (this.phase[i]! + turn) % (2 * Math.PI);
      const k = thrust / hover;
      const c = Math.cos(this.phase[i]!);
      const s = Math.sin(this.phase[i]!);
      gx += g * k * c;
      gz += g * k * s;
      ax += p.acc * k * c;
      az += p.acc * k * s;
      ay += 0.5 * p.acc * k * Math.sin(this.phase[i]! + 1);
    }
    return { gyro: v3(gx, 0, gz), acc: v3(ax, ay, az) };
  }
}
