import { useMemo, useRef } from 'react';
import { controllerKey } from '@/control/registry';
import { useParams } from '@/store/params';
import { sim, useUi, type AnalysisChart, type ExtraChart } from '@/store/sim';
import { AircraftPoles, EnvelopeChart, HqChart } from '@/ui/analysis/AircraftCharts';
import { BodeChart } from '@/ui/analysis/BodeChart';
import { DescribingChart } from '@/ui/analysis/DescribingChart';
import { DispersionChart } from '@/ui/analysis/DispersionChart';
import { EngagementChart } from '@/ui/analysis/EngagementChart';
import { MimoChart } from '@/ui/analysis/MimoChart';
import { MuChart } from '@/ui/analysis/MuChart';
import { ValueMap } from '@/ui/analysis/ValueMap';
import { NyquistChart } from '@/ui/analysis/NyquistChart';
import { PoleMap } from '@/ui/analysis/PoleMap';
import { SpectrumChart } from '@/ui/analysis/SpectrumChart';
import { TvcChart } from '@/ui/analysis/TvcChart';
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
  { value: 'spectrum', label: 'spectrum' },
  { value: 'attitude', label: 'attitude estimate' },
  { value: 'dispersion', label: 'Monte Carlo campaign' },
  { value: 'fault', label: 'after a rotor loss' },
  { value: 'engagement', label: 'the chase, from above' },
  { value: 'ins', label: 'inertial drift' },
  { value: 'beacons', label: 'range-only filter' },
  { value: 'airmodes', label: 'aircraft: speed and poles' },
  { value: 'hq', label: 'aircraft: handling qualities' },
  { value: 'envelope', label: 'aircraft: margins across speed' },
  { value: 'energy', label: 'aircraft: speed and altitude' },
  { value: 'lyapunovV', label: 'satellite: Lyapunov function' },
  { value: 'wheels', label: 'satellite: wheel momentum' },
  { value: 'startracker', label: 'satellite: attitude knowledge' },
  { value: 'fdi', label: 'fault detection' },
  { value: 'heading', label: 'heading estimate' },
  { value: 'nav', label: 'navigation filter' },
  { value: 'mrac', label: 'MRAC parameters' },
  { value: 'mu', label: 'μ analysis' },
  { value: 'costate', label: 'costate (LQR)' },
  { value: 'value', label: 'value function' },
  { value: 'describing', label: 'describing function' },
  { value: 'tvc', label: 'thrust-vector loop (Nyquist)' },
];
/** What can stand in the motor chart's place. */
const SECOND: { value: AnalysisChart | 'motors'; label: string }[] = [
  { value: 'motors', label: 'motor thrust' },
  { value: 'bode', label: 'Bode plot' },
  { value: 'nyquist', label: 'Nyquist plot' },
  { value: 'poles', label: 'pole map' },
  { value: 'covariance', label: 'filter uncertainty' },
];
/** Lesson III.17: the adapted parameters of MRAC. */
const MRAC_THETA: SeriesSpec[] = [
  { key: 'mrac.theta.p', label: 'θ̂ position gain', color: SIGNAL.p, width: 2 },
  { key: 'mrac.theta.d', label: 'θ̂ velocity gain', color: SIGNAL.d, width: 2 },
  { key: 'mrac.theta.g', label: 'θ̂ hover thrust', color: SIGNAL.i, width: 2 },
];
/** Lesson III.23: the heading error, and the MEKF's own 2σ for it. */
/** Lesson IV.1: the thrust the costate asks for, −R⁻¹Bᵀλ, against the thrust commanded. */
const COSTATE: SeriesSpec[] = [
  { key: 'alt.fb', label: 'commanded u', color: SIGNAL.output, width: 2 },
  { key: 'lqr.u.lambda', label: '−R⁻¹Bᵀλ', color: SIGNAL.measurement, dash: [5, 4], width: 2 },
  { key: 'lqr.lambda.v', label: 'λ_v', color: SIGNAL.d, legendOnly: true },
  { key: 'lqr.lambda.e', label: 'λ_e', color: SIGNAL.p, legendOnly: true },
];
const HEADING: SeriesSpec[] = [
  { key: 'ahrs.headErr', label: 'heading error', color: SIGNAL.error, width: 2 },
  { key: 'mekf.sigma2.yaw', label: 'MEKF 2σ', color: SIGNAL.measurement, dash: [4, 3], width: 1.5 },
];
/** Lesson III.24: how far the navigation estimate is from the truth, and its NIS. */
const NAV_ERR: SeriesSpec[] = [
  { key: 'nav.err', label: 'estimate error', color: SIGNAL.error, width: 2 },
  { key: 'nav.sigma2', label: 'EKF 2σ', color: SIGNAL.measurement, dash: [4, 3], width: 1.5 },
];
const NAV_NIS: SeriesSpec[] = [
  { key: 'nav.gpsNis', label: 'NIS of each fix', color: SIGNAL.measurement, width: 1.5 },
];
/** The two tests of the fault monitor (lesson III.28). */
const NIS: SeriesSpec[] = [{ key: 'fdi.nis', label: 'NIS', color: SIGNAL.measurement, width: 2 }];
const CUSUM: SeriesSpec[] = [{ key: 'fdi.cusum', label: 'CUSUM', color: SIGNAL.error, width: 2 }];
/** Lesson IV.26: the inertial navigator's error against the prediction from the gyro bias. */
const INS: SeriesSpec[] = [
  { key: 'ins.err', label: 'navigator error', color: SIGNAL.error, width: 2 },
  { key: 'ins.pred', label: 'g·b·t³/6', color: SIGNAL.measurement, dash: [4, 3], width: 1.5 },
];
/** Lesson IV.27: the range-only filter's error against its own 2σ. */
const BEACON_ERR: SeriesSpec[] = [
  { key: 'rng.err', label: 'position error', color: SIGNAL.error, width: 2 },
  { key: 'rng.sigma2', label: 'filter 2σ', color: SIGNAL.measurement, dash: [4, 3], width: 1.5 },
];
/** Chapter M: the aircraft's speed and the auto-throttle's reference. */
const AIRSPEED: SeriesSpec[] = [
  { key: 'air.vref', label: 'reference', color: SIGNAL.setpoint, dash: [4, 3], width: 1.5 },
  { key: 'air.V', label: 'airspeed', color: SIGNAL.measurement, width: 2 },
];
const ALTITUDE: SeriesSpec[] = [
  { key: 'air.href', label: 'reference', color: SIGNAL.setpoint, dash: [4, 3], width: 1.5 },
  { key: 'air.h', label: 'altitude', color: SIGNAL.measurement, width: 2 },
];
const BEACON_NIS: SeriesSpec[] = [
  { key: 'rng.nis', label: 'NIS of each range', color: SIGNAL.measurement, width: 1.5 },
];
/** Lesson IV.18: the Lyapunov function of quaternion feedback. */
const SAT_V: SeriesSpec[] = [
  { key: 'sat.V', label: 'V = ½ωᵀJω + 2kp(1 − q₀)', color: SIGNAL.measurement, width: 2 },
];
/** Lesson IV.19: the momentum stored in each reaction wheel. */
const WHEELS: SeriesSpec[] = (['x', 'y', 'z'] as const).map((c, i) => ({
  key: `sat.h.${c}`,
  label: `wheel ${c}`,
  color: SIGNAL.windAxes[i]!,
  width: 1.5,
}));
/** Lesson IV.22: attitude knowledge from the gyro and the star tracker, against the filter's 2σ. */
const STAR: SeriesSpec[] = [
  { key: 'st.err', label: 'knowledge error', color: SIGNAL.error, width: 2 },
  { key: 'st.sigma2', label: 'filter 2σ', color: SIGNAL.measurement, dash: [4, 3], width: 1.5 },
];
/** After a rotor loss: how far the thrust axis is from where it should point. */
const FAULT: SeriesSpec[] = [
  { key: 'fault.tiltErr', label: 'thrust-axis error', color: SIGNAL.error, width: 2 },
];
/** How far the attitude filter's "up" is from the true one, next to the tilt itself. */
const ATTITUDE: SeriesSpec[] = [
  { key: 'ahrs.tilt', label: 'true tilt', color: SIGNAL.truth, width: 1.5 },
  { key: 'ahrs.tiltHat', label: 'estimated tilt', color: SIGNAL.measurement, width: 1.5 },
  { key: 'ahrs.err', label: 'error', color: SIGNAL.error, width: 2 },
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
  ) : kind === 'mu' ? (
    <MuChart />
  ) : kind === 'value' ? (
    <ValueMap />
  ) : kind === 'mimo' ? (
    <MimoChart />
  ) : kind === 'spectrum' ? (
    <SpectrumChart />
  ) : kind === 'describing' ? (
    <DescribingChart />
  ) : kind === 'dispersion' ? (
    <DispersionChart />
  ) : kind === 'tvc' ? (
    <TvcChart />
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
    extra === 'mimo' ||
    extra === 'spectrum' ||
    extra === 'dispersion' ||
    extra === 'fdi' ||
    extra === 'nav' ||
    extra === 'describing' ||
    extra === 'mu' ||
    extra === 'value' ||
    extra === 'engagement' ||
    extra === 'beacons' ||
    extra === 'tvc' ||
    extra === 'airmodes' ||
    extra === 'hq' ||
    extra === 'envelope' ||
    extra === 'energy';
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
            {extra === 'lyapunov' ? (
              <PhasePortrait meta={meta} lyapunov />
            ) : extra === 'engagement' ? (
              <EngagementChart />
            ) : extra === 'airmodes' ? (
              <div className="flex h-full min-h-0 flex-col gap-1.5">
                <div className="min-h-0 flex-1">
                  <TimeChart title="Airspeed" unit="m/s" series={AIRSPEED} />
                </div>
                <div className="min-h-0 flex-[1.4]">
                  <AircraftPoles />
                </div>
              </div>
            ) : extra === 'energy' ? (
              <div className="flex h-full min-h-0 flex-col gap-1.5">
                <div className="min-h-0 flex-1">
                  <TimeChart title="Airspeed" unit="m/s" series={AIRSPEED} />
                </div>
                <div className="min-h-0 flex-1">
                  <TimeChart title="Altitude" unit="m" series={ALTITUDE} />
                </div>
              </div>
            ) : extra === 'hq' ? (
              <HqChart />
            ) : extra === 'envelope' ? (
              <EnvelopeChart />
            ) : extra === 'beacons' ? (
              <div className="flex h-full min-h-0 flex-col gap-1.5">
                <div className="min-h-0 flex-1">
                  <TimeChart
                    title="Range-only filter: error against its own 2σ (log scale)"
                    unit="m"
                    series={BEACON_ERR}
                    logY
                  />
                </div>
                <div className="min-h-0 flex-1">
                  <TimeChart
                    title="NIS of the ranges (χ², 1 degree of freedom)"
                    series={BEACON_NIS}
                    logY
                  />
                </div>
              </div>
            ) : extra === 'nav' ? (
              <div className="flex h-full min-h-0 flex-col gap-1.5">
                <div className="min-h-0 flex-1">
                  <TimeChart
                    title="Navigation: estimate against truth (shaded: fix rejected)"
                    unit="m"
                    series={NAV_ERR}
                    shadeKey="nav.gpsRejected"
                    includeZero
                  />
                </div>
                <div className="min-h-0 flex-1">
                  <TimeChart
                    title="NIS of the GPS fixes (χ², 3 degrees of freedom)"
                    series={NAV_NIS}
                    includeZero
                    logY
                    hlines={() =>
                      sim.params.control.navEkf.gate > 0
                        ? [{ value: sim.params.control.navEkf.gate, label: 'gate' }]
                        : []
                    }
                  />
                </div>
              </div>
            ) : extra === 'fdi' ? (
              <div className="flex h-full min-h-0 flex-col gap-1.5">
                <div className="min-h-0 flex-1">
                  <TimeChart
                    title="Altimeter: NIS over the window (shaded: alarm)"
                    series={NIS}
                    shadeKey="fdi.altAlarm"
                    includeZero
                    hlines={() => [
                      { value: sim.params.control.fdi.nisThreshold, label: 'high' },
                      { value: sim.params.control.fdi.nisLow, label: 'low' },
                    ]}
                  />
                </div>
                <div className="min-h-0 flex-1">
                  <TimeChart
                    title="Motors: largest CUSUM"
                    unit="N·m·s"
                    series={CUSUM}
                    includeZero
                    hlines={() => [
                      { value: sim.params.control.fdi.cusumThreshold, label: 'alarm' },
                    ]}
                  />
                </div>
              </div>
            ) : (
              analysisChart(extra)
            )}
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
        ) : extra === 'lyapunovV' ? (
          <TimeChart title="Lyapunov function (log scale)" unit="J" series={SAT_V} logY />
        ) : extra === 'startracker' ? (
          <TimeChart
            title="Attitude knowledge, log scale (shaded: tracker blind)"
            unit="″"
            series={STAR}
            shadeKey="st.blind"
            logY
          />
        ) : extra === 'wheels' ? (
          <TimeChart
            title="Wheel momentum"
            unit="N·m·s"
            series={WHEELS}
            includeZero
            hlines={() => [
              { value: sim.params.satellite.wheelMomentum, label: 'full speed' },
              { value: -sim.params.satellite.wheelMomentum, label: '' },
            ]}
          />
        ) : extra === 'ins' ? (
          <TimeChart
            title="Inertial navigator: position error (log scale)"
            unit="m"
            series={INS}
            logY
          />
        ) : extra === 'fault' ? (
          <TimeChart
            title="Thrust axis: error from its target"
            unit="°"
            series={FAULT}
            includeZero
          />
        ) : extra === 'mrac' ? (
          <TimeChart title="MRAC: the adapted parameters" series={MRAC_THETA} includeZero />
        ) : extra === 'heading' ? (
          <TimeChart
            title="Heading error (shaded: magnetometer rejected)"
            unit="°"
            series={HEADING}
            shadeKey="mekf.magRejected"
            includeZero
          />
        ) : extra === 'costate' ? (
          <TimeChart
            title="Costate λ = P·x and the thrust it commands, u = −R⁻¹Bᵀλ"
            unit="N"
            series={COSTATE}
            includeZero
          />
        ) : extra === 'attitude' ? (
          <TimeChart
            title="Attitude estimate (shaded: accelerometer ignored)"
            unit="°"
            series={ATTITUDE}
            shadeKey="ahrs.gated"
            includeZero
          />
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
