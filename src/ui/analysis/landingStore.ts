import { create } from 'zustand';
import {
  CAMPAIGN_CASES,
  LandingCampaign,
  landingKey,
  storeLandings,
  type LandingRun,
} from '@/guidance/pdgCampaign';
import { useParams } from '@/store/params';

interface LandingStore {
  /** The set-up the runs belong to; they are shown only while it is the current one. */
  key: string | null;
  runs: LandingRun[];
  running: boolean;
  start: () => void;
}

let raf = 0;

/** The landing campaign of lesson IV.5, one dispersed case per animation frame. */
export const useLandings = create<LandingStore>((set, get) => ({
  key: null,
  runs: [],
  running: false,
  start: () => {
    cancelAnimationFrame(raf);
    const p = useParams.getState().params;
    const campaign = new LandingCampaign(p, CAMPAIGN_CASES);
    const key = landingKey(p);
    set({ key, runs: [], running: true });
    const tick = () => {
      const done = campaign.advance();
      set({ runs: [...campaign.runs], running: !done });
      if (done) storeLandings(p, campaign.runs);
      else if (get().key === key) raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
  },
}));
