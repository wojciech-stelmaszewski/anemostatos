import { Scene } from '@/scene/Scene';
import { useUi } from '@/store/sim';
import { ChartsPanel } from '@/ui/charts/ChartsPanel';
import { TooltipProvider } from '@/ui/components/tooltip';
import { LiveFormula } from '@/ui/hud/LiveFormula';
import { StatusBadges } from '@/ui/hud/StatusBadges';
import { WindHud } from '@/ui/hud/WindHud';
import { ParamPanel } from '@/ui/params/ParamPanel';
import { Toolbar } from '@/ui/Toolbar';
import { useKeyboard } from '@/ui/useKeyboard';

export function App() {
  useKeyboard();
  const hidden = useUi((s) => s.panelsHidden);
  return (
    <TooltipProvider delayDuration={250}>
      <div
        className="grid h-full"
        style={{
          gridTemplateColumns: hidden ? '1fr' : '1fr 330px',
          gridTemplateRows: hidden ? '44px 1fr' : '44px 1fr 330px',
        }}
      >
        <div className="col-span-full grid">
          <Toolbar />
        </div>
        <main className="relative min-h-0 min-w-0 bg-bg">
          <Scene />
          <div className="pointer-events-none absolute inset-x-0 top-3 flex justify-center">
            <StatusBadges />
          </div>
          <div className="absolute bottom-3 left-3">
            <LiveFormula />
          </div>
          <div className="absolute bottom-3 right-3">
            <WindHud />
          </div>
          <div className="pointer-events-none absolute left-3 top-3 text-[10px] leading-relaxed text-muted/80">
            drag: orbit · scroll: zoom · drag the target sphere: move setpoint · click drone: poke
            <br />
            ↑/↓ setpoint · Space pause · G gust · R reset · M controller · C camera · H hide panels
          </div>
        </main>
        {!hidden && (
          <>
            <aside className="row-span-2 min-h-0 overflow-y-auto border-l border-border bg-panel">
              <ParamPanel />
            </aside>
            <section className="min-h-0 min-w-0 border-t border-border bg-panel">
              <ChartsPanel />
            </section>
          </>
        )}
      </div>
    </TooltipProvider>
  );
}
