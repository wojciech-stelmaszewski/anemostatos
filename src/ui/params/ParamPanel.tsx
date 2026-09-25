import { Info, RotateCcw } from 'lucide-react';
import { useState } from 'react';
import { useShallow } from 'zustand/react/shallow';
import { getIn, SCHEMA, type Field } from '@/engine/schema';
import { defaultParams } from '@/sim/params';
import { useParams } from '@/store/params';
import { useUi } from '@/store/sim';
import { Section } from '@/ui/components/section';
import { Select } from '@/ui/components/select';
import { Slider } from '@/ui/components/slider';
import { Switch } from '@/ui/components/switch';
import { Tooltip } from '@/ui/components/tooltip';
import { SIGNAL } from '@/ui/colors';

const DEFAULTS = defaultParams();
const RES = 1000;

const toSlider = (f: Extract<Field, { kind: 'number' }>, v: number) => {
  if (!f.log) return ((v - f.min) / (f.max - f.min)) * RES;
  return (Math.log(Math.max(v, f.min) / f.min) / Math.log(f.max / f.min)) * RES;
};
const fromSlider = (f: Extract<Field, { kind: 'number' }>, s: number) => {
  const v = f.log ? f.min * Math.pow(f.max / f.min, s / RES) : f.min + ((f.max - f.min) * s) / RES;
  const decimals = Math.max(0, -Math.floor(Math.log10(f.step)));
  const snapped = f.log ? Number(v.toPrecision(3)) : Math.round(v / f.step) * f.step;
  return Number(snapped.toFixed(decimals));
};

function Label({ field }: { field: Field }) {
  return (
    <Tooltip content={field.help}>
      <span className="flex min-w-0 items-center gap-1 truncate text-xs text-muted">
        {field.label}
        {field.help && <Info className="size-3 shrink-0 opacity-50" />}
      </span>
    </Tooltip>
  );
}

function NumberField({ field }: { field: Extract<Field, { kind: 'number' }> }) {
  const value = useParams((s) => getIn(s.params, field.path)) as number;
  const set = useParams((s) => s.set);
  const [draft, setDraft] = useState<string | null>(null);
  const def = getIn(DEFAULTS, field.path) as number;
  return (
    <div className="grid grid-cols-[1fr_auto] items-center gap-x-2 gap-y-1">
      <Label field={field} />
      <div className="flex items-center gap-1">
        <input
          className="w-16 rounded border border-border bg-panel-2 px-1.5 py-0.5 text-right font-mono text-xs text-fg outline-none focus:border-accent/60"
          value={draft ?? String(value)}
          onFocus={(e) => {
            setDraft(String(value));
            e.target.select();
          }}
          onChange={(e) => setDraft(e.target.value)}
          onBlur={() => {
            const v = Number(draft);
            if (draft !== null && Number.isFinite(v)) set(field.path, v);
            setDraft(null);
          }}
          onKeyDown={(e) => {
            if (e.key === 'Enter') (e.target as HTMLInputElement).blur();
            if (e.key === 'Escape') {
              setDraft(null);
              (e.target as HTMLInputElement).blur();
            }
            e.stopPropagation();
          }}
        />
        <span className="w-10 truncate text-[10px] text-muted">{field.unit}</span>
        <button
          title={`Reset to ${def}`}
          className={`text-muted hover:text-fg ${value === def ? 'invisible' : ''}`}
          onClick={() => set(field.path, def)}
        >
          <RotateCcw className="size-3" />
        </button>
      </div>
      <Slider
        className="col-span-2"
        min={0}
        max={RES}
        step={1}
        value={[toSlider(field, value)]}
        onValueChange={([s]) => set(field.path, fromSlider(field, s!))}
      />
    </div>
  );
}

function BoolField({ field }: { field: Extract<Field, { kind: 'bool' }> }) {
  const value = useParams((s) => getIn(s.params, field.path)) as boolean;
  const set = useParams((s) => s.set);
  return (
    <div className="flex items-center justify-between">
      <Label field={field} />
      <Switch checked={value} onCheckedChange={(v) => set(field.path, v)} />
    </div>
  );
}

function SelectField({ field }: { field: Extract<Field, { kind: 'select' }> }) {
  const value = useParams((s) => getIn(s.params, field.path)) as string;
  const set = useParams((s) => s.set);
  return (
    <div className="flex items-center justify-between gap-2">
      <Label field={field} />
      <Select value={value} onValueChange={(v) => set(field.path, v)} options={field.options} />
    </div>
  );
}

/** P / I / D toggles rendered as one compact coloured row. */
function TermToggles({ prefix }: { prefix: string }) {
  const set = useParams((s) => s.set);
  const on = useParams(
    useShallow((s) => ({
      p: getIn(s.params, `${prefix}.pOn`) as boolean,
      i: getIn(s.params, `${prefix}.iOn`) as boolean,
      d: getIn(s.params, `${prefix}.dOn`) as boolean,
    })),
  );
  return (
    <div className="flex gap-1">
      {(['p', 'i', 'd'] as const).map((k) => (
        <button
          key={k}
          onClick={() => set(`${prefix}.${k}On`, !on[k])}
          className="flex-1 rounded border px-2 py-0.5 font-mono text-xs font-semibold transition-colors"
          style={{
            borderColor: on[k] ? SIGNAL[k] : 'var(--color-border)',
            color: on[k] ? SIGNAL[k] : 'var(--color-muted)',
            background: on[k] ? `${SIGNAL[k]}1a` : 'transparent',
          }}
        >
          {k.toUpperCase()} {on[k] ? 'on' : 'off'}
        </button>
      ))}
    </div>
  );
}

function FieldView({ field }: { field: Field }) {
  if (field.kind === 'number') return <NumberField field={field} />;
  if (field.kind === 'bool') return <BoolField field={field} />;
  return <SelectField field={field} />;
}

const ADVANCED = /\.(derivativeOn|dFilterHz|antiWindup|iLimit|kb)$/;

export function ParamPanel() {
  const level = useParams((s) => s.params.sim.level);
  const setLoop = useUi((s) => s.setLoop);
  return (
    <div>
      {SCHEMA.filter((g) => !g.levels || g.levels.includes(level)).map((g) => {
        const fields = g.fields.filter((f) => !f.levels || f.levels.includes(level));
        const termPrefix = fields.find((f) => f.path.endsWith('.pOn'))?.path.replace(/\.pOn$/, '');
        const basic = fields.filter(
          (f) => !/\.(pOn|iOn|dOn)$/.test(f.path) && !ADVANCED.test(f.path),
        );
        const advanced = fields.filter((f) => ADVANCED.test(f.path));
        return (
          <Section
            key={`${g.id}-${level}`}
            title={
              <span onClick={() => g.loop && setLoop(g.loop)} className="flex-1">
                {g.title}
              </span>
            }
            defaultOpen={g.id === 'alt' || g.id === 'setpoint' || (level > 1 && g.id === 'posH')}
            accent={g.loop ? SIGNAL.output : undefined}
          >
            {termPrefix && <TermToggles prefix={termPrefix} />}
            {basic.map((f) => (
              <FieldView key={f.path} field={f} />
            ))}
            {advanced.length > 0 && (
              <details className="group rounded border border-border/60 px-2 py-1">
                <summary className="cursor-pointer text-[11px] text-muted hover:text-fg">
                  Practical details (D filter, anti-windup…)
                </summary>
                <div className="space-y-2 pt-2">
                  {advanced.map((f) => (
                    <FieldView key={f.path} field={f} />
                  ))}
                </div>
              </details>
            )}
          </Section>
        );
      })}
    </div>
  );
}
