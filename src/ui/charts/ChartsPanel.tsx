import { useMemo } from 'react';
import { useParams } from '@/store/params';
import { sim, useUi } from '@/store/sim';
import { SIGNAL } from '@/ui/colors';
import { Select } from '@/ui/components/select';
import { InfoCard } from './InfoCard';
import { LOOPS, loopMeta } from './loops';
import { TimeChart, type SeriesSpec } from './TimeChart';

const WINDOWS = [5, 10, 30, 60];

export function ChartsPanel() {
  const level = useParams((s) => s.params.sim.level);
  const loopId = useUi((s) => s.loop);
  const setLoop = useUi((s) => s.setLoop);
  const windowSec = useUi((s) => s.window);
  const setWindow = useUi((s) => s.setWindow);
  const meta = loopMeta(level, loopId);
  const id = meta.id;

  const tracking = useMemo<SeriesSpec[]>(() => {
    const s: SeriesSpec[] = [
      { key: `${id}.sp`, label: 'setpoint', color: SIGNAL.setpoint, dash: [6, 4] },
      { key: `${id}.meas`, label: 'measured', color: SIGNAL.measurement, width: 2 },
    ];
    if (meta.truth) s.push({ key: meta.truth, label: 'true', color: SIGNAL.truth, dash: [2, 3] });
    return s;
  }, [id, meta.truth]);

  const terms = useMemo<SeriesSpec[]>(
    () => [
      { key: `${id}.p`, label: 'P', color: SIGNAL.p },
      { key: `${id}.i`, label: 'I', color: SIGNAL.i },
      { key: `${id}.d`, label: 'D', color: SIGNAL.d },
      { key: `${id}.pid`, label: 'P+I+D', color: SIGNAL.output, width: 2 },
      { key: `${id}.ff`, label: 'FF', color: SIGNAL.ff, legendOnly: true },
      { key: `${id}.u`, label: 'u', color: SIGNAL.output, legendOnly: true },
    ],
    [id],
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
        <span className="ml-auto text-[11px] text-muted">
          Hover a chart to read values; red shading = output saturated.
        </span>
      </div>
      <div className="grid min-h-0 flex-1 grid-cols-3 grid-rows-2 gap-1.5">
        <TimeChart title={`${meta.name}: tracking`} unit={meta.unit} series={tracking} />
        <TimeChart
          title="PID terms (feedback part of u)"
          unit={meta.outUnit}
          series={terms}
          shadeKey={`${id}.sat`}
          includeZero
        />
        <InfoCard meta={meta} />
        <TimeChart
          title="Error e = setpoint − measured"
          unit={meta.unit}
          series={[{ key: `${id}.err`, label: 'e', color: SIGNAL.error, width: 2 }]}
          includeZero
        />
        <TimeChart
          title="Motor thrust"
          unit="N"
          series={motors}
          includeZero
          hlines={() => [{ value: sim.params.drone.maxMotorThrust, label: 'max' }]}
        />
        <TimeChart
          title="Wind"
          unit="m/s"
          series={wind}
          includeZero
          vlines={() => sim.wind.gustLog}
        />
      </div>
    </div>
  );
}
