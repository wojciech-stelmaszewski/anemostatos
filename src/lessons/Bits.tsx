import type { ReactNode } from 'react';
import { SIGNAL } from '@/ui/colors';

export const K = ({ k, children }: { k: 'p' | 'i' | 'd' | 'ff'; children: ReactNode }) => (
  <b style={{ color: SIGNAL[k] }}>{children}</b>
);
export const Try = ({ children }: { children: ReactNode }) => (
  <div className="mt-2 rounded border border-accent/30 bg-accent/5 px-2 py-1.5">
    <div className="mb-0.5 text-[10px] font-semibold uppercase tracking-wider text-accent">
      Try this
    </div>
    {children}
  </div>
);
export const Notice = ({ children }: { children: ReactNode }) => (
  <div className="mt-2 rounded border border-border px-2 py-1.5">
    <div className="mb-0.5 text-[10px] font-semibold uppercase tracking-wider text-muted">
      What to notice
    </div>
    {children}
  </div>
);
