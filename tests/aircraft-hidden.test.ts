import { describe, expect, it } from 'vitest';
import {
  aircraftModes,
  envelopeMargins,
  hiddenPath,
  meetsPitchSpec,
  trimSlope,
} from '@/analysis/aircraft';
import { alphaLookup } from '@/control/autopilot';
import { Simulation } from '@/engine/simulation';
import { LESSONS } from '@/lessons/lessons';
import { defaultParams, type Params } from '@/sim/params';
import { trimAircraft } from '@/sim/vehicles/aircraft';

const lesson = LESSONS.find((l) => l.id === 'hidden')!;
const setup = (mod: (p: Params) => void = () => {}): Params => {
  const p = defaultParams();
  p.sim.level = lesson.level;
  lesson.setup!(p);
  mod(p);
  return p;
};
const gap = (p: Params) => {
  const f = envelopeMargins(p, 9);
  const t = envelopeMargins(p, 9, true);
  return Math.max(...f.map((r, i) => Math.abs(r.pmDeg - t[i]!.pmDeg)));
};

describe('lesson IV.14: the hidden path of a schedule on α', () => {
  it('the table is the level-flight trims, and its slope is −C_mα/C_mδe = −1', () => {
    const p = setup();
    for (const V of [80, 120, 160]) {
      const t = trimAircraft(p.aircraft, V, p.aircraft.altitude);
      const r = alphaLookup(p.aircraft, t.state.alpha);
      expect(r.de).toBeCloseTo(t.input.de, 4);
      expect(r.qbar / t.qbar).toBeCloseTo(1, 2);
      expect(hiddenPath(p, V)!.gain).toBeCloseTo(-1, 6);
    }
    expect(trimSlope(p.aircraft)).toBeCloseTo(-1, 12);
  });

  it('point by point the α table is the q̄ schedule of IV.13: the same margins', () => {
    const a = envelopeMargins(setup(), 9, true);
    const q = envelopeMargins(
      setup((p) => (p.autopilot.schedule = 'qbar')),
      9,
    );
    a.forEach((r, i) => expect(Math.abs(r.pmDeg - q[i]!.pmDeg)).toBeLessThan(0.5));
    expect(Math.min(...a.map((r) => r.pmDeg))).toBeGreaterThan(53);
    expect(Math.max(...a.map((r) => r.pmDeg))).toBeLessThan(80);
  });

  it('as flown the loop differs: 85° instead of 55° at 80 m/s, crossover below 4 rad/s', () => {
    const p = setup();
    const flown = envelopeMargins(p, 9);
    const table = envelopeMargins(p, 9, true);
    expect(flown[0]!.pmDeg).toBeGreaterThan(80);
    expect(table[0]!.pmDeg).toBeLessThan(57);
    expect(flown[0]!.wc).toBeLessThan(4);
    expect(flown[0]!.wc).toBeGreaterThan(3.6);
    expect(meetsPitchSpec(p).ok).toBe(false);
    expect(gap(p)).toBeGreaterThan(25);
  });

  it('the short period slows from about 1.7 s to 3.9 s and is far more damped', () => {
    const flown = aircraftModes(setup(), true, 80).shortPeriod!;
    const table = aircraftModes(
      setup((p) => (p.autopilot.schedule = 'qbar')),
      true,
      80,
    ).shortPeriod!;
    expect(table.period).toBeGreaterThan(1.6);
    expect(table.period).toBeLessThan(1.8);
    expect(table.zeta).toBeCloseTo(0.47, 1);
    expect(flown.period).toBeGreaterThan(3.7);
    expect(flown.period).toBeLessThan(4.1);
    expect(flown.zeta).toBeCloseTo(0.8, 1);
  });

  it('with the gain at zero the static stability is gone: a pole at 0 and one at −1.46', () => {
    const m = aircraftModes(
      setup((p) => ((p.autopilot.gain = 0), (p.autopilot.kTheta = 0))),
      true,
      80,
    );
    // No short-period pair any more (the phugoid is the only oscillation left).
    expect(m.poles.filter((z) => z.im > 1e-6 && Math.hypot(z.re, z.im) > 1).length).toBe(0);
    const real = m.poles.filter((z) => Math.abs(z.im) < 1e-9).map((z) => z.re);
    expect(real.some((r) => Math.abs(r + 1.46) < 0.02)).toBe(true);
    expect(real.filter((r) => Math.abs(r) < 0.01).length).toBeGreaterThanOrEqual(1);
    // The bare airframe, by contrast, has its short period.
    const bare = aircraftModes(
      setup(
        (p) => ((p.autopilot.gain = 0), (p.autopilot.kTheta = 0), (p.autopilot.schedule = 'qbar')),
      ),
      true,
      80,
    );
    expect(bare.shortPeriod!.period).toBeLessThan(3);
  });

  it('in flight the 2° step goes twice as far: 1.6° against 0.8° after 1.5 s', () => {
    const rise = (mod: (p: Params) => void) => {
      const sim = new Simulation(setup(mod));
      lesson.events!(sim);
      for (let k = 0; k < 8000; k++) sim.step();
      const w = sim.telemetry.window(['air.theta'], 5);
      const at = (t: number) => w.series[0]![w.t.findIndex((x) => x >= t)]!;
      return at(7.5) - at(5.95);
    };
    const alpha = rise(() => {});
    const qbar = rise((p) => (p.autopilot.schedule = 'qbar'));
    const filtered = rise((p) => (p.autopilot.scheduleTau = 2));
    expect(alpha).toBeGreaterThan(1.45);
    expect(qbar).toBeLessThan(0.9);
    // A slow filter still leaves the cancellation below its corner: the step still goes further.
    expect(filtered).toBeGreaterThan(qbar + 0.15);
  }, 120_000);

  it('a low-pass of a second brings the flown margins within 5° of the table; half a second does not', () => {
    expect(gap(setup((p) => (p.autopilot.scheduleTau = 0.5)))).toBeGreaterThan(5);
    const p = setup((p) => (p.autopilot.scheduleTau = 1));
    expect(gap(p)).toBeLessThan(5);
    expect(meetsPitchSpec(p).ok).toBe(true);
  });

  it('on q̄ the flown loop is the table: no gap and the requirement met', () => {
    const p = setup((p) => (p.autopilot.schedule = 'qbar'));
    expect(gap(p)).toBeLessThan(1e-9);
    expect(meetsPitchSpec(p).ok).toBe(true);
  });
});
