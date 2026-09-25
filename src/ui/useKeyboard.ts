import { useEffect } from 'react';
import type { Level } from '@/sim/params';
import { useParams } from '@/store/params';
import { sim, useUi, type CameraMode } from '@/store/sim';
import { gustNow, snapshot } from './actions';

const CAMERAS: CameraMode[] = ['orbit', 'follow', 'side', 'top'];

/** Global shortcuts (docs/ui-and-visualization.md §6). */
export function useKeyboard(): void {
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      const t = e.target as HTMLElement;
      if (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.isContentEditable) return;
      const ui = useUi.getState();
      const params = useParams.getState();
      const sp = params.params.setpoint;
      const stepSize = e.shiftKey ? 1 : 0.1;
      const clampY = (v: number) => Math.round(Math.min(10, Math.max(0.2, v)) * 100) / 100;
      switch (e.key) {
        case ' ':
          ui.setPaused(!ui.paused);
          break;
        case '.':
          if (ui.paused) sim.stepController();
          break;
        case 'r':
        case 'R':
          sim.reset();
          break;
        case 'm':
        case 'M':
          sim.setArmed(!sim.armed);
          break;
        case 'g':
        case 'G':
          gustNow();
          break;
        case 'ArrowUp':
          params.set('setpoint.y', clampY(sp.y + stepSize));
          break;
        case 'ArrowDown':
          params.set('setpoint.y', clampY(sp.y - stepSize));
          break;
        case '1':
        case '2':
        case '3':
          params.set('sim.level', Number(e.key) as Level);
          break;
        case 'c':
        case 'C':
          ui.setCamera(CAMERAS[(CAMERAS.indexOf(ui.camera) + 1) % CAMERAS.length]!);
          break;
        case 's':
        case 'S':
          snapshot();
          break;
        case 'h':
        case 'H':
          ui.togglePanels();
          break;
        default:
          return;
      }
      e.preventDefault();
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, []);
}
