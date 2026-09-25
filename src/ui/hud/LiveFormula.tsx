import { useRef } from 'react';
import { getIn } from '@/engine/schema';
import { useParams } from '@/store/params';
import { sim, useUi } from '@/store/sim';
import { loopMeta } from '@/ui/charts/loops';
import { SIGNAL } from '@/ui/colors';
import { useRaf } from './useRaf';

const kpPath = (id: string, level: number) =>
  id === 'alt' || (level === 2 && id === 'pos.y')
    ? 'control.alt.kp'
    : level === 2
      ? 'control.posH.kp'
      : id === 'vel.y'
        ? 'control.l3.velV.kp'
        : id.startsWith('vel.')
          ? 'control.l3.velH.kp'
          : id === 'rate.yaw'
            ? 'control.l3.rateYaw.kp'
            : id.startsWith('rate.')
              ? 'control.l3.rateRP.kp'
              : id.startsWith('att.')
                ? id === 'att.yaw'
                  ? 'control.l3.attKpYaw'
                  : 'control.l3.attKpRP'
                : id.startsWith('pos.')
                  ? id === 'pos.y'
                    ? 'control.l3.posKpV'
                    : 'control.l3.posKpH'
                  : 'control.alt.kp';

const n = (v: number, d = 2) =>
  Number.isFinite(v) ? (v >= 0 ? '+' : '−') + Math.abs(v).toFixed(d) : '—';

/** u = Kp·e + I + D + FF with live numbers substituted — the PID equation, running. */
export function LiveFormula() {
  const level = useParams((s) => s.params.sim.level);
  const loopId = useUi((s) => s.loop);
  const meta = loopMeta(level, loopId);
  const kp = useParams((s) => getIn(s.params, kpPath(meta.id, level)) as number);
  const refs = useRef<Record<string, HTMLSpanElement | null>>({});
  const r = (k: string) => (el: HTMLSpanElement | null) => void (refs.current[k] = el);

  useRaf(() => {
    const t = sim.controller.loops()[meta.id];
    if (!t) return;
    const set = (k: string, s: string) => {
      const el = refs.current[k];
      if (el && el.textContent !== s) el.textContent = s;
    };
    set('e', t.error.toFixed(3));
    set('p', n(t.p));
    set('i', n(t.i));
    set('d', n(t.d));
    set('ff', n(t.ff));
    set('u', t.output.toFixed(2));
    set('sat', t.saturated ? `saturated (wanted ${t.unsaturated.toFixed(2)})` : '');
  });

  return (
    <div className="pointer-events-none rounded-lg border border-border/80 bg-panel/85 px-3 py-2 font-mono text-xs shadow-lg backdrop-blur">
      <div className="mb-1 font-sans text-[10px] uppercase tracking-wider text-muted">
        {meta.name} controller, live
      </div>
      <div className="whitespace-nowrap">
        <span className="text-muted">u = </span>
        <span style={{ color: SIGNAL.p }}>
          {kp}·<span ref={r('e')} />
        </span>{' '}
        <span style={{ color: SIGNAL.i }} ref={r('i')} />{' '}
        <span style={{ color: SIGNAL.d }} ref={r('d')} />{' '}
        <span style={{ color: SIGNAL.ff }} ref={r('ff')} />
      </div>
      <div className="whitespace-nowrap text-muted">
        {'  '}= <span style={{ color: SIGNAL.p }}>P</span>{' '}
        <span ref={r('p')} style={{ color: SIGNAL.p }} />
        {'  '}
        → <span className="font-semibold text-fg" ref={r('u')} /> {meta.outUnit}
      </div>
      <div className="text-[10px] text-[#e66767]" ref={r('sat')} />
      <div className="mt-1 font-sans text-[10px] text-muted">
        <span style={{ color: SIGNAL.p }}>Kp·e</span> + <span style={{ color: SIGNAL.i }}>I</span> +{' '}
        <span style={{ color: SIGNAL.d }}>D</span> +{' '}
        <span style={{ color: SIGNAL.ff }}>feedforward</span>
      </div>
    </div>
  );
}
