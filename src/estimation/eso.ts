import { c2d, mul, type Mat } from '@/math/mat';

/**
 * Third-order extended state observer for ÿ = b₀·u + f (Han 2009, Gao 2003):
 * z = (ŷ, v̂, f̂), with all observer poles at −ω_o. The unknown "total disturbance" f becomes a
 * state and gets estimated like any other. Discretised exactly (zero-order hold) at the loop rate,
 * so high observer bandwidths stay stable.
 */
export class Eso {
  z: [number, number, number] = [0, 0, 0];
  private started = false;
  private cache: { key: string; ad: Mat; bd: Mat } | null = null;

  reset(y?: number): void {
    this.started = y !== undefined;
    this.z = [y ?? 0, 0, 0];
  }

  /** Forget the disturbance estimate (held at zero while the drone sits on the ground). */
  clearDisturbance(): void {
    this.z[2] = 0;
  }

  private matrices(wo: number, b0: number, dt: number) {
    const key = `${wo}|${b0}|${dt}`;
    if (this.cache?.key !== key) {
      const l = [3 * wo, 3 * wo * wo, wo ** 3];
      // ż = (A − L·C)·z + B·u + L·y
      const a = [
        [-l[0]!, 1, 0],
        [-l[1]!, 0, 1],
        [-l[2]!, 0, 0],
      ];
      const b = [
        [0, l[0]!],
        [b0, l[1]!],
        [0, l[2]!],
      ];
      const d = c2d(a, b, dt);
      this.cache = { key, ad: d.a, bd: d.b };
    }
    return this.cache;
  }

  /** Advance one sample with the applied input u and the measurement y. */
  update(u: number, y: number, wo: number, b0: number, dt: number): [number, number, number] {
    if (!this.started) {
      this.reset(y);
      return this.z;
    }
    const { ad, bd } = this.matrices(wo, b0, dt);
    const next = mul(ad, [[this.z[0]], [this.z[1]], [this.z[2]]]);
    const inp = mul(bd, [[u], [y]]);
    this.z = [next[0]![0]! + inp[0]![0]!, next[1]![0]! + inp[1]![0]!, next[2]![0]! + inp[2]![0]!];
    return this.z;
  }
}
