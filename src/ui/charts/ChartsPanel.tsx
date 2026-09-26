import { useMemo, useRef } from 'react';
import { controllerKey } from '@/control/registry';
import { useParams } from '@/store/params';
import { sim, useUi, type ExtraChart } from '@/store/sim';
import { useRaf } from '@/ui/hud/useRaf';
import { partColor, SIGNAL } from '@/ui/colors';
import { Select } from '@/ui/components/select';
import { InfoCard } from './InfoCard';
import { PhasePortrait } from './PhasePortrait';
import { isPidParts, LOOPS, loopMeta, useLoopParts } from './loops';
import { TimeChart, type SeriesSpec } from './TimeChart';

const WINDOWS = [5, 10, 30, 60];
const EXTRA: { value: ExtraChart; label: string }[] = [
  { value: 'wind', label: 'wind' },
  { value: 'disturbance', label: 'disturbance' },
  { value: 'phase', label: 'phase portrait' },
];

/** Wall-clock cost of the controller — what Part II's optimisers pay for being clever. */
function ComputeMeter() {
  const el = useRef<HTMLSpanElement>(null);
  useRaf(() => {
    const s = `controller ${sim.controlMicros.toFixed(sim.controlMicros < 1 ? 2 : sim.controlMicros < 10 ? 1 : 0)} µs/step`;
    if (el.current && el.current.textContent !== s) el.current.textContent = s;
  });
  return (
    <span
      ref={el}
      className="font-mono text-[11px] text-muted"
      title="Average wall-clock time the controller takes per 1 ms physics step (measured in your browser; it never affects the simulation)."
    />
  );
}

export function ChartsPanel() {
  const level = useParams((s) => s.params.sim.level);
  const loopId = useUi((s) => s.loop);
  const setLoop = useUi((s) => s.setLoop);
  const windowSec = useUi((s) => s.window);
  const extra = useUi((s) => s.extraChart);
  const setExtra = useUi((s) => s.setExtraChart);
  const ctrlKey = useParams((s) => controllerKey(s.params));
  const setWindow = useUi((s) => s.setWindow);
  const meta = loopMeta(level, loopId);
  const id = meta.id;

  const tracking = useMemo<SeriesSpec[]>(() => {
    const s: SeriesSpec[] = [
      { key: `${id}.sp`, label: 'setpoint', color: SIGNAL.setpoint, dash: [6, 4] },
      { key: `${id}.meas`, label: 'measured', color: SIGNAL.measurement, width: 2 },
    ];
    if (meta.truth) s.push({ key: meta.truth, label: 'true', color: SIGNAL.truth, dash: [2, 3] });
    s.push({ key: `${id}.meas`, label: 'ghost', color: SIGNAL.ghost, width: 1.5, ghost: true });
    return s;
  }, [id, meta.truth]);

  const parts = useLoopParts(id);
  const pid = isPidParts(parts);
  const terms = useMemo<SeriesSpec[]>(
    () => [
      ...parts
        .filter((p) => !p.ff)
        .map((p, i) => ({
          key: `${id}.part.${p.key}`,
          label: p.label,
          color: partColor(p.key, i),
        })),
      {
        key: `${id}.fb`,
        label: pid ? 'P+I+D' : 'sum',
        color: SIGNAL.output,
        width: 2,
      },
      ...parts
        .filter((p) => p.ff)
        .map((p, i) => ({
          key: `${id}.part.${p.key}`,
          label: p.label,
          color: partColor(p.key, i + 3),
          legendOnly: true,
        })),
      { key: `${id}.u`, label: 'u', color: SIGNAL.output, legendOnly: true },
    ],
    [id, parts, pid],
  );

  const motors = useMemo<SeriesSpec[]>(
    () =>
      [1, 2, 3, 4].map((i) => ({
        key: `motor.${i}`,
        label: `M${i}`,
        color: SIGNAL.motors[i - 1]!,
      })),
    [],
  );

  const wind = useMemo<SeriesSpec[]>(
    () =>
      level === 1
        ? [{ key: 'wind.y', label: 'vertical', color: SIGNAL.wind, width: 2 }]
        : (['x', 'y', 'z'] as const).map((a, i) => ({
            key: `wind.${a}`,
            label: a,
            color: SIGNAL.windAxes[i]!,
          })),
    [level],
  );

  const disturbance = useMemo<SeriesSpec[]>(() => {
    const axes = level === 1 ? (['y'] as const) : (['x', 'y', 'z'] as const);
    const estimated = sim.controller.extras();
    return axes.flatMap((a, i) => {
      const color = level === 1 ? SIGNAL.error : SIGNAL.windAxes[i]!;
      const s: SeriesSpec[] = [{ key: `dist.${a}`, label: `true ${a}`, color, width: 1.5 }];
      if (`est.dist.${a}` in estimated)
        s.push({ key: `est.dist.${a}`, label: `est. ${a}`, color, dash: [5, 3], width: 2 });
      return s;
    });
    // eslint-disable-next-line react-hooks/exhaustive-deps -- re-read when the controller changes
  }, [level, ctrlKey]);

  return (
    <div className="flex h-full min-h-0 flex-col gap-1.5 p-2">
      <div className="flex items-center gap-2 text-xs">
        <span className="font-semibold uppercase tracking-wider text-muted">Charts</span>
        <span className="text-muted">loop</span>
        <Select
          value={id}
          onValueChange={setLoop}
          options={LOOPS[level].map((l) => ({ value: l.id, label: l.name }))}
        />
        <span className="text-muted">window</span>
        <Select
          value={String(windowSec)}
          onValueChange={(v) => setWindow(Number(v))}
          options={WINDOWS.map((w) => ({ value: String(w), label: `${w} s` }))}
        />
        <span className="text-muted">extra</span>
        <Select value={extra} onValueChange={(v) => setExtra(v as ExtraChart)} options={EXTRA} />
        <span className="ml-auto text-[11px] text-muted">
          Hover a chart to read values; red shading = output saturated.
        </span>
        <ComputeMeter />
      </div>
      <div className="grid min-h-0 flex-1 grid-cols-3 grid-rows-2 gap-1.5">
        <TimeChart title={`${meta.name}: tracking`} unit={meta.unit} series={tracking} />
        <TimeChart
          title={pid ? 'PID terms (feedback part of u)' : 'Contributions (feedback part of u)'}
          unit={meta.outUnit}
          series={terms}
          shadeKey={`${id}.sat`}
          includeZero
        />
        <InfoCard meta={meta} />
        <TimeChart
          title="Error e = setpoint − measured"
          unit={meta.unit}
          series={[
            { key: `${id}.err`, label: 'e', color: SIGNAL.error, width: 2 },
            { key: `${id}.err`, label: 'ghost', color: SIGNAL.ghost, ghost: true },
          ]}
          includeZero
        />
        <TimeChart
          title="Motor thrust"
          unit="N"
          series={motors}
          includeZero
          hlines={() => [{ value: sim.params.drone.maxMotorThrust, label: 'max' }]}
        />
        {extra === 'phase' ? (
          <PhasePortrait meta={meta} />
        ) : extra === 'disturbance' ? (
          <TimeChart
            title="Disturbance force (what the model does not explain)"
            unit="N"
            series={disturbance}
            includeZero
            vlines={() => sim.wind.gustLog}
          />
        ) : (
          <TimeChart
            title="Wind"
            unit="m/s"
            series={wind}
            includeZero
            vlines={() => sim.wind.gustLog}
          />
        )}
      </div>
    </div>
  );
}
