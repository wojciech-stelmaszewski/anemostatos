import type { MlpWeights } from './policy';
import { POLICY_WEIGHTS } from './policy-weights';

/**
 * The weights the policy controller uses: the trained ones shipped in policy-weights.ts, unless
 * the training script swaps in candidates while it evaluates them.
 */
let current: MlpWeights = POLICY_WEIGHTS;

export const policyWeights = (): MlpWeights => current;
export const setPolicyWeights = (w: MlpWeights): void => {
  current = w;
};
