// A tiny MLP trainer (flat parameters, backprop, Adam) for scripts/train-policy.ts.
import { forward, type MlpWeights } from '@/control/policy';
import type { Rng } from '@/math/prng';

export function network(sizes: number[]) {
  const count = sizes.slice(1).reduce((n, out, i) => n + out * (sizes[i]! + 1), 0);
  const offsets: number[] = [];
  let k0 = 0;
  sizes.slice(1).forEach((o, i) => {
    offsets.push(k0);
    k0 += o * (sizes[i]! + 1);
  });

  /** Flat vector → layers (per output neuron: its input weights, then its bias). */
  function unflatten(theta: ArrayLike<number>, meta?: MlpWeights['meta']): MlpWeights {
    let k = 0;
    const layers = sizes.slice(1).map((out, i) => {
      const inp = sizes[i]!;
      const w: number[][] = [];
      const b: number[] = [];
      for (let o = 0; o < out; o++) {
        w.push(Array.from({ length: inp }, () => theta[k++]!));
        b.push(theta[k++]!);
      }
      return { w, b };
    });
    return { sizes: [...sizes], layers, meta };
  }

  function init(rng: Rng): Float64Array {
    const theta = new Float64Array(count);
    let k = 0;
    sizes.slice(1).forEach((out, i) => {
      const inp = sizes[i]!;
      const scale = Math.sqrt(6 / (inp + out));
      for (let o = 0; o < out; o++) {
        for (let j = 0; j < inp; j++) theta[k++] = rng.uniform(-scale, scale);
        theta[k++] = 0;
      }
    });
    return theta;
  }

  /** Gradient of ½‖out − y‖² w.r.t. the parameters, accumulated into `grad`. Returns the loss. */
  function backprop(net: MlpWeights, x: number[], y: number[], grad: Float64Array): number {
    const acts: number[][] = [];
    const out = forward(net, x, acts);
    let delta = out.map((o, i) => o - y[i]!);
    const loss = 0.5 * delta.reduce((s, d) => s + d * d, 0);
    for (let l = net.layers.length - 1; l >= 0; l--) {
      const layer = net.layers[l]!;
      const input = acts[l]!;
      const inp = input.length;
      let off = offsets[l]!;
      for (let o = 0; o < layer.b.length; o++) {
        const d = delta[o]!;
        for (let j = 0; j < inp; j++) grad[off + j]! += d * input[j]!;
        grad[off + inp]! += d;
        off += inp + 1;
      }
      if (l > 0) {
        const next = new Array<number>(inp).fill(0);
        for (let o = 0; o < layer.b.length; o++) {
          const w = layer.w[o]!;
          for (let j = 0; j < inp; j++) next[j]! += w[j]! * delta[o]!;
        }
        // Through tanh: input[j] is the hidden activation.
        delta = next.map((v, j) => v * (1 - input[j]! ** 2));
      }
    }
    return loss;
  }

  return { count, unflatten, init, backprop };
}

export class Adam {
  private m: Float64Array;
  private v: Float64Array;
  private t = 0;
  constructor(
    n: number,
    private lr: number,
  ) {
    this.m = new Float64Array(n);
    this.v = new Float64Array(n);
  }
  /** Descent step: theta ← theta − lr·m̂/(√v̂ + ε). */
  step(theta: Float64Array, grad: Float64Array): void {
    this.t++;
    const [b1, b2] = [0.9, 0.999];
    for (let i = 0; i < theta.length; i++) {
      this.m[i] = b1 * this.m[i]! + (1 - b1) * grad[i]!;
      this.v[i] = b2 * this.v[i]! + (1 - b2) * grad[i]! ** 2;
      const mh = this.m[i]! / (1 - b1 ** this.t);
      const vh = this.v[i]! / (1 - b2 ** this.t);
      theta[i]! -= (this.lr * mh) / (Math.sqrt(vh) + 1e-8);
    }
  }
}
