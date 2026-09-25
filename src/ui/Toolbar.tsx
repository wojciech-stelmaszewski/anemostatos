import {
  Camera,
  Dices,
  Download,
  Link2,
  Pause,
  Play,
  Power,
  RotateCcw,
  StepForward,
  Wind,
} from 'lucide-react';
import { useRef, useState } from 'react';
import { randomSeed } from '@/math/prng';
import type { Level } from '@/sim/params';
import { shareUrl, useParams } from '@/store/params';
import { sim, useUi, type CameraMode, type Overlays } from '@/store/sim';
import { downloadCsv, gustNow } from './actions';
import { Button } from './components/button';
import { Select } from './components/select';
import { Tooltip } from './components/tooltip';
import { useRaf } from './hud/useRaf';

const SPEEDS = [0.1, 0.25, 0.5, 1, 2, 4];
const LEVELS: { level: Level; label: string; hint: string }[] = [
  {
    level: 1,
    label: 'L1 Altitude',
    hint: 'One PID on height. The drone only moves up and down. Start here.',
  },
  {
    level: 2,
    label: 'L2 Point mass',
    hint: 'Three independent PIDs push a point mass in 3D (a cheat: real drones must tilt).',
  },
  {
    level: 3,
    label: 'L3 Quadrotor',
    hint: 'Full 6-DoF quadrotor with a cascade of controllers: position → velocity → attitude → rate → motors.',
  },
];
const OVERLAYS: { key: keyof Overlays; label: string }[] = [
  { key: 'forces', label: 'forces' },
  { key: 'terms', label: 'P/I/D arrows' },
  { key: 'setpoint', label: 'setpoint' },
  { key: 'heightAids', label: 'height' },
  { key: 'trail', label: 'trail' },
  { key: 'wind', label: 'wind' },
];

function Clock() {
  const el = useRef<HTMLSpanElement>(null);
  useRaf(() => {
    if (el.current) el.current.textContent = `t = ${sim.t.toFixed(2)} s`;
  });
  return <span ref={el} className="w-24 font-mono text-xs text-fg" />;
}

function ArmButton() {
  const [armed, setArmed] = useState(sim.armed);
  useRaf(() => {
    if (sim.armed !== armed) setArmed(sim.armed);
  });
  return (
    <Tooltip content="Switch the controller on/off (M). Off = motors idle: watch the drone fall.">
      <Button variant={armed ? 'active' : 'default'} onClick={() => sim.setArmed(!sim.armed)}>
        <Power /> {armed ? 'Controller on' : 'Controller off'}
      </Button>
    </Tooltip>
  );
}

export function Toolbar() {
  const { paused, timeScale, setPaused, setTimeScale, camera, setCamera, overlays, toggleOverlay } =
    useUi();
  const level = useParams((s) => s.params.sim.level);
  const seed = useParams((s) => s.params.sim.seed);
  const set = useParams((s) => s.set);
  const [copied, setCopied] = useState(false);

  return (
    <header className="flex items-center gap-2 overflow-x-auto border-b border-border bg-panel px-3">
      <span className="mr-2 whitespace-nowrap font-semibold tracking-wide">
        <span className="text-accent">✕</span> Anemostatos
      </span>

      <div className="flex rounded-md border border-border p-0.5">
        {LEVELS.map((l) => (
          <Tooltip key={l.level} content={l.hint}>
            <button
              onClick={() => set('sim.level', l.level)}
              className={`rounded px-2.5 py-1 text-xs whitespace-nowrap ${level === l.level ? 'bg-accent text-black font-semibold' : 'text-muted hover:text-fg'}`}
            >
              {l.label}
            </button>
          </Tooltip>
        ))}
      </div>

      <div className="mx-1 h-5 w-px bg-border" />
      <Tooltip content={paused ? 'Resume (Space)' : 'Pause (Space)'}>
        <Button
          size="icon"
          variant={paused ? 'accent' : 'default'}
          onClick={() => setPaused(!paused)}
        >
          {paused ? <Play /> : <Pause />}
        </Button>
      </Tooltip>
      <Tooltip content="Single step: one controller period (.)">
        <Button size="icon" onClick={() => sim.stepController()} disabled={!paused}>
          <StepForward />
        </Button>
      </Tooltip>
      <Select
        value={String(timeScale)}
        onValueChange={(v) => setTimeScale(Number(v))}
        options={SPEEDS.map((s) => ({ value: String(s), label: `${s}×` }))}
      />
      <Clock />
      <Tooltip content="Restart the run from the ground with the current parameters (R)">
        <Button onClick={() => sim.reset()}>
          <RotateCcw /> Reset
        </Button>
      </Tooltip>
      <ArmButton />

      <div className="mx-1 h-5 w-px bg-border" />
      <Tooltip content="Trigger a strong gust now (G)">
        <Button onClick={gustNow}>
          <Wind /> Gust
        </Button>
      </Tooltip>
      <Tooltip content="Random seed: same seed + same parameters = exactly the same wind. Compare tunings fairly.">
        <span className="flex items-center gap-1 text-xs text-muted">
          seed
          <input
            className="w-16 rounded border border-border bg-panel-2 px-1.5 py-0.5 font-mono text-xs text-fg outline-none focus:border-accent/60"
            value={seed}
            onChange={(e) => {
              const v = Number(e.target.value);
              if (Number.isInteger(v) && v >= 0) set('sim.seed', v);
            }}
            onKeyDown={(e) => e.stopPropagation()}
          />
          <Button size="icon" variant="ghost" onClick={() => set('sim.seed', randomSeed())}>
            <Dices />
          </Button>
        </span>
      </Tooltip>

      <span className="flex-1" />

      <div className="flex items-center gap-0.5 text-[11px]">
        {OVERLAYS.map((o) => (
          <button
            key={o.key}
            onClick={() => toggleOverlay(o.key)}
            className={`rounded px-1.5 py-0.5 whitespace-nowrap ${overlays[o.key] ? 'text-fg' : 'text-muted/50 line-through'}`}
          >
            {o.label}
          </button>
        ))}
      </div>
      <Tooltip content="Camera (C): orbit freely, follow the drone, side view, top view">
        <span className="flex items-center gap-1">
          <Camera className="size-3.5 text-muted" />
          <Select
            value={camera}
            onValueChange={(v) => setCamera(v as CameraMode)}
            options={[
              { value: 'orbit', label: 'orbit' },
              { value: 'follow', label: 'follow' },
              { value: 'side', label: 'side' },
              { value: 'top', label: 'top' },
            ]}
          />
        </span>
      </Tooltip>
      <Tooltip content="Copy a link with all current parameters">
        <Button
          size="icon"
          variant="ghost"
          onClick={() => {
            void navigator.clipboard.writeText(shareUrl());
            setCopied(true);
            setTimeout(() => setCopied(false), 1200);
          }}
        >
          {copied ? '✓' : <Link2 />}
        </Button>
      </Tooltip>
      <Tooltip content="Download the last 60 s of telemetry as CSV">
        <Button size="icon" variant="ghost" onClick={downloadCsv}>
          <Download />
        </Button>
      </Tooltip>
    </header>
  );
}
