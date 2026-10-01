import { create } from 'zustand';
import { campaignKey, DispersionCampaign, storeCampaign, type Run } from '@/analysis/dispersion';
import { useParams } from '@/store/params';

interface DispersionStore {
  /** The tune the runs belong to; they are shown only while it is the current one. */
  key: string | null;
  runs: Run[];
  n: number;
  running: boolean;
  start: (n?: number) => void;
}

let raf = 0;
/** Physics steps per animation frame: about 60 ms of work for the quadrotor. */
const SLICE = 20000;

/** The Monte Carlo campaign of lesson III.29, flown in slices from animation frames. */
export const useDispersion = create<DispersionStore>((set, get) => ({
  key: null,
  runs: [],
  n: 0,
  running: false,
  start: (n = 300) => {
    cancelAnimationFrame(raf);
    const p = useParams.getState().params;
    if (p.sim.level !== 3) return;
    const campaign = new DispersionCampaign(p, n);
    const key = campaignKey(p);
    set({ key, runs: [], n, running: true });
    const tick = () => {
      const done = campaign.advance(SLICE);
      set({ runs: [...campaign.runs], running: !done });
      if (done && n === 300) storeCampaign(p, campaign.runs);
      if (!done && get().key === key) raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
  },
}));
