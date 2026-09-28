import { qRotate } from '@/math/quat';
import { clamp } from '@/math/util';
import { v3, type Vec3 } from '@/math/vec3';
import { GRAVITY, type Params } from '@/sim/params';
import type { Measurement } from '@/sim/sensors';
import type { InfoRow, LoopTerms, Setpoint } from './types';

/**
 * A small neural network policy (MLP) for the quadrotor, trained offline by `make train`
 * (scripts/train-policy.ts): imitation of the geometric controller, then evolution strategies
 * on the closed loop with randomised mass and wind. Plain TypeScript inference, no ML library.
 *
 * Interface (Kaufmann et al. 2023): observations → collective thrust + body rates, which the
 * existing rate loop tracks. It bypasses the thrust-vector and attitude stages.
 */
export interface MlpWeights {
  sizes: number[];
  /** Per layer: weights[out][in] and biases[out]. */
  layers: { w: number[][]; b: number[] }[];
  /** How it was made — shown in the info card. */
  meta?: Record<string, string | number>;
}

export const OBS = 15;
export const ACT = 4;
/** Observation scales: the network sees quantities of order one. */
const POS_CLIP = 2;
const VEL_SCALE = 3;
const RATE_SCALE = 4;
/** Output scales. */
export const MAX_RATE = 6; // rad/s

/** Observation vector from the measurement and the reference (world frame). */
export function observe(s: Measurement, sp: Setpoint): number[] {
  const ep = v3(
    clamp(s.pos.x - sp.pos.x, -POS_CLIP, POS_CLIP),
    clamp(s.pos.y - sp.pos.y, -POS_CLIP, POS_CLIP),
    clamp(s.pos.z - sp.pos.z, -POS_CLIP, POS_CLIP),
  );
  const vr = sp.vel ?? v3();
  const up = qRotate(s.q, v3(0, 1, 0));
  const fwd = qRotate(s.q, v3(1, 0, 0));
  return [
    ep.x / POS_CLIP,
    ep.y / POS_CLIP,
    ep.z / POS_CLIP,
    (s.vel.x - vr.x) / VEL_SCALE,
    (s.vel.y - vr.y) / VEL_SCALE,
    (s.vel.z - vr.z) / VEL_SCALE,
    up.x,
    up.y,
    up.z,
    fwd.x,
    fwd.y,
    fwd.z,
    s.omega.x / RATE_SCALE,
    s.omega.y / RATE_SCALE,
    s.omega.z / RATE_SCALE,
  ];
}

/** Forward pass: tanh hidden layers, linear output. Returns every layer's activations if asked. */
export function forward(net: MlpWeights, x: number[], keep?: number[][]): number[] {
  let a = x;
  keep?.push(a);
  net.layers.forEach((layer, li) => {
    const last = li === net.layers.length - 1;
    const out = layer.b.map((b, i) => {
      const w = layer.w[i]!;
      let s = b;
      for (let j = 0; j < a.length; j++) s += w[j]! * a[j]!;
      return last ? s : Math.tanh(s);
    });
    a = out;
    keep?.push(a);
  });
  return a;
}

/** Network output → (collective thrust N, body rates rad/s as x roll, y yaw, z pitch). */
export function decode(o: number[], mHat: number, tMax: number): { thrust: number; rates: Vec3 } {
  const hover = mHat * GRAVITY;
  return {
    thrust: clamp(hover * (1 + Math.tanh(o[0]!)), 0, tMax),
    rates: v3(
      MAX_RATE * Math.tanh(o[1]!),
      MAX_RATE * Math.tanh(o[2]!),
      MAX_RATE * Math.tanh(o[3]!),
    ),
  };
}

/** Inverse of `decode` for training targets (clamped into the representable range). */
export function encode(thrust: number, rates: Vec3, mHat: number): number[] {
  const hover = mHat * GRAVITY;
  const at = (x: number) => Math.atanh(clamp(x, -0.999, 0.999));
  return [
    at(thrust / hover - 1),
    at(rates.x / MAX_RATE),
    at(rates.y / MAX_RATE),
    at(rates.z / MAX_RATE),
  ];
}

/** The L3 outer stage that asks the network. */
export class PolicyOuter {
  private loopsCache: Record<string, LoopTerms> = {};

  constructor(private readonly net: MlpWeights) {}

  reset(): void {
    this.loopsCache = {};
  }

  /** Collective thrust (N) and body-rate setpoint (deg/s). */
  update(s: Measurement, sp: Setpoint, p: Params): { thrust: number; ratesDeg: Vec3 } {
    const out = decode(
      forward(this.net, observe(s, sp)),
      p.control.model.mass,
      4 * p.drone.maxMotorThrust,
    );
    const DEG = 180 / Math.PI;
    for (const k of ['x', 'y', 'z'] as const) {
      this.loopsCache[`pos.${k}`] = {
        setpoint: sp.pos[k],
        measurement: s.pos[k],
        error: sp.pos[k] - s.pos[k],
        parts: [{ key: 'net', label: 'network output (thrust, N)', value: out.thrust, like: 'p' }],
        unsaturated: out.thrust,
        output: out.thrust,
        saturated: false,
      };
    }
    return {
      thrust: out.thrust,
      ratesDeg: v3(out.rates.x * DEG, out.rates.y * DEG, out.rates.z * DEG),
    };
  }

  loops(): Record<string, LoopTerms> {
    return this.loopsCache;
  }

  describe(): InfoRow[] {
    const m = this.net.meta ?? {};
    return [
      {
        label: 'network',
        value: `${this.net.sizes.join(' → ')} (tanh), ${this.net.layers.reduce((n, l) => n + l.b.length * (l.w[0]!.length + 1), 0)} parameters`,
        hint: 'Observations: position and velocity error, orientation (up and forward axes), body rates. Actions: collective thrust and body rates.',
      },
      ...Object.entries(m).map(([k, v]) => ({ label: k, value: String(v) })),
    ];
  }
}
