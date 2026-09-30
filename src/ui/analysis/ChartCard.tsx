import type { ReactNode, RefObject } from 'react';

/** The card every analysis chart sits in: title, legend, summary, optional controls, readout. */
export function ChartCard({
  title,
  legend,
  summary,
  controls,
  readout,
  children,
}: {
  title: string;
  legend?: { label: string; color: string; mark: 'line' | 'dot' | 'cross' }[];
  summary?: ReactNode;
  controls?: ReactNode;
  readout: RefObject<HTMLSpanElement | null>;
  children: ReactNode;
}) {
  return (
    <div className="flex h-full min-h-0 flex-col rounded-md border border-border bg-bg/60">
      <div className="flex flex-wrap items-center gap-x-3 gap-y-0.5 px-2 pt-1.5 text-[11px]">
        <span className="font-semibold text-fg">{title}</span>
        {legend?.map((s) => (
          <span key={s.label} className="flex items-center gap-1 text-muted">
            {s.mark === 'line' ? (
              <span className="inline-block h-0.5 w-3.5 rounded" style={{ background: s.color }} />
            ) : s.mark === 'dot' ? (
              <span className="inline-block size-2 rounded-full" style={{ background: s.color }} />
            ) : (
              <span className="font-mono text-[11px] leading-none" style={{ color: s.color }}>
                ×
              </span>
            )}
            {s.label}
          </span>
        ))}
        <span className="flex-1" />
        {controls}
      </div>
      <div className="flex min-h-4 flex-wrap items-center gap-x-3 px-2 text-[11px] text-muted">
        {summary}
        <span className="flex-1" />
        <span ref={readout} className="font-mono text-fg" />
      </div>
      {children}
    </div>
  );
}
