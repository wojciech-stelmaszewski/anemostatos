import { L1_DISPERSIONS } from '@/analysis/campaign';

/** The drawn values of a run, as "mass 0.92 kg, motor lag 57 ms, …" (the wind's seed left out). */
export const describeRun = (values: readonly number[]): string =>
  L1_DISPERSIONS.filter((d) => d.id !== 'seed')
    .map((d) => {
      const v = values[L1_DISPERSIONS.indexOf(d)]!;
      return `${d.label} ${v.toFixed(d.id === 'mass' ? 2 : 1)}${d.unit ? ' ' + d.unit : ''}`;
    })
    .join(', ');
