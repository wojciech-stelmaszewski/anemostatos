import { useMemo, useRef } from 'react';
import { controllerKey } from '@/control/registry';
import { useParams } from '@/store/params';
import { sim, useUi, type AnalysisChart, type ExtraChart } from '@/store/sim';
import { BodeChart } from '@/ui/analysis/BodeChart';
import { MimoChart } from '@/ui/analysis/MimoChart';
import { NyquistChart } from '@/ui/analysis/NyquistChart';
import { PoleMap } from '@/ui/analysis/PoleMap';
import { useRaf } from '@/ui/hud/useRaf';
import { partColor, SIGNAL } from '@/ui/colors';
import { Select } from '@/ui/components/select';
import { InfoCard } from './InfoCard';
import { PhasePortrait } from './PhasePortrait';
import { isPidParts, loopMeta, loopsFor, useLoopParts } from './loops';
import { TimeChart, type SeriesSpec } from './TimeChart';

const WINDOWS = [5, 10, 30, 60];
const EXTRA: { value: ExtraChart; label: string }[] = [
  { value: 'wind', label: 'wind' },
  { value: 'disturbance', label: 'disturbance' },
  { value: 'phase', label: 'phase portrait' },
  { value: 'estimate', label: 'estimate vs truth' },
  { value: 'bode', label: 'Bode plot' },
  { value: 'nyquist', label: 'Nyquist plot' },
  { value: 'poles', label: 'pole map' },
  { value: 'lyapunov', label: 'phase portrait + Lyapunov' },
  { value: 'covariance', label: 'filter uncertainty' },
  { value: 'mimo', label: 'two channels at once' },
];
/** What can stand in the motor chart's place. */
const SECOND: { value: AnalysisChart | 'motors'; label: string }[] = [
  { value: 'motors', label: 'motor thrust' },
  { value: 'bode', label: 'Bode plot' },
  { value: 'nyquist', label: 'Nyquist plot' },
  { value: 'poles', label: 'pole map' },
  { value: 'covariance', label: 'filter uncertainty' },
];
/** 1σ of each state of the altitude filter, from its own covariance matrix. */
const COVARIANCE: SeriesSpec[] = [
  { key: 'est.sigma.y', label: 'altitude [m]', color: SIGNAL.motors[0], width: 2 },
  { key: 'est.sigma.by', label: 'altimeter bias [m]', color: SIGNAL.motors[1], width: 2 },
  { key: 'est.sigma.v', label: 'velocity [m/s]', color: SIGNAL.motors[2], width: 2 },
  { key: 'est.sigma.ba', label: 'acc. bias [m/s²]', color: SIGNAL.motors[3], width: 2 },
];
const analysisChart = (kind: string) =>
  kind === 'bode' ? (
    <BodeChart />
  ) : kind === 'nyquist' ? (
    <NyquistChart />
  ) : kind === 'poles' ? (
    <PoleMap />
  ) : kind === 'mimo' ? (
    <MimoChart />
  ) : kind === 'covariance' ? (
    <TimeChart title="Filter uncertainty: 1σ of each state (log scale)" series={COVARIANCE} logY />
  ) : null;

const ESTIMATE: SeriesSpec[] = [
  { key: 'pos.y', label: 'true', color: SIGNAL.truth, width: 2 },
  { key: 'meas.y', label: 'sensor', color: SIGNAL.measurement, width: 1 },
  { key: 'est.y', label: 'estimate', color: SIGNAL.output, width: 2 },
  { key: 'est.y.hi', label: '+2σ', color: SIGNAL.output, dash: [3, 3], width: 1 },
  { key: 'est.y.lo', label: '−2σ', color: SIGNAL.output, dash: [3, 3], width: 1 },
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
  const second = useUi((s) => s.secondChart);
  const setSecond = useUi((s) => s.setSecondChart);
  const tall =
    extra === 'bode' ||
    extra === 'nyquist' ||
    extra === 'poles' ||
    extra === 'lyapunov' ||
    extra === 'mimo';
  const ctrlKey = useParams((s) => controllerKey(s.params));
  const loopOptions = useMemo(
    () => loopsFor(useParams.getState().params).map((l) => ({ value: l.id, label: l.name })),
    // eslint-disable-next-line react-hooks/exhaustive-deps -- the list depends on the controller
    [ctrlKey],
  );
  const setWindow = useUi((s) => s.setWindow);
  const metaKey = useParams((s) => controllerKey(s.params));
  const meta = useMemo(
    () => loopMeta(useParams.getState().params, loopId),
    // eslint-disable-next-line react-hooks/exhaustive-deps -- depends on the controller, via metaKey
    [metaKey, loopId, level],
  );
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
          color: partColor(p, i),
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
          color: partColor(p, i + 3),
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
        <Select value={id} onValueChange={setLoop} options={loopOptions} />
        <span className="text-muted">window</span>
        <Select
          value={String(windowSec)}
          onValueChange={(v) => setWindow(Number(v))}
          options={WINDOWS.map((w) => ({ value: String(w), label: `${w} s` }))}
        />
        <span className="text-muted">extra</span>
        <Select value={extra} onValueChange={(v) => setExtra(v as ExtraChart)} options={EXTRA} />
        <span className="text-muted">beside it</span>
        <Select
          value={second ?? 'motors'}
          onValueChange={(v) => setSecond(v === 'motors' ? null : (v as AnalysisChart))}
          options={SECOND}
        />
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
        {/* An analysis view needs height: it takes the whole right column, both rows. */}
        {tall ? (
          <div className="row-span-2 min-h-0">
            {extra === 'lyapunov' ? <PhasePortrait meta={meta} lyapunov /> : analysisChart(extra)}
          </div>
        ) : (
          <InfoCard meta={meta} />
        )}
        <TimeChart
          title="Error e = setpoint − measured"
          unit={meta.unit}
          series={[
            { key: `${id}.err`, label: 'e', color: SIGNAL.error, width: 2 },
            { key: `${id}.err`, label: 'ghost', color: SIGNAL.ghost, ghost: true },
          ]}
          includeZero
        />
        {second ? (
          analysisChart(second)
        ) : (
          <TimeChart
            title="Motor thrust"
            unit="N"
            series={motors}
            includeZero
            hlines={() => [{ value: sim.params.drone.maxMotorThrust, label: 'max' }]}
          />
        )}
        {tall ? null : extra === 'covariance' ? (
          analysisChart(extra)
        ) : extra === 'phase' ? (
          <PhasePortrait meta={meta} />
        ) : extra === 'estimate' ? (
          <TimeChart title="Altitude: estimate vs truth" unit="m" series={ESTIMATE} />
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
