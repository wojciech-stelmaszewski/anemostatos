import { create } from 'zustand';
import { setIn } from '@/engine/schema';
import { defaultParams, type Level, type Params } from '@/sim/params';

interface ParamStore {
  params: Params;
  set: (path: string, value: unknown) => void;
  replace: (params: Params) => void;
  setLevel: (level: Level) => void;
}

const fromHash = (): Params | null => {
  try {
    const h = decodeURIComponent(window.location.hash.slice(1));
    if (!h.startsWith('p=')) return null;
    return deepMerge(defaultParams(), JSON.parse(h.slice(2))) as Params;
  } catch {
    return null;
  }
};

function deepMerge(base: unknown, over: unknown): unknown {
  if (typeof base !== 'object' || base === null || typeof over !== 'object' || over === null) {
    return over === undefined ? base : over;
  }
  const out: Record<string, unknown> = { ...(base as Record<string, unknown>) };
  for (const [k, v] of Object.entries(over as Record<string, unknown>)) {
    if (k in out) out[k] = deepMerge(out[k], v);
  }
  return out;
}

export const useParams = create<ParamStore>((set) => ({
  params: fromHash() ?? defaultParams(),
  set: (path, value) => set((s) => ({ params: setIn(s.params, path, value) })),
  replace: (params) => set({ params }),
  setLevel: (level) => set((s) => ({ params: setIn(s.params, 'sim.level', level) })),
}));

export const shareUrl = (): string => {
  const json = JSON.stringify(useParams.getState().params);
  return `${window.location.origin}${window.location.pathname}#p=${encodeURIComponent(json)}`;
};
