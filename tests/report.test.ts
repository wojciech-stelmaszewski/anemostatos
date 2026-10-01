import { beforeAll, describe, expect, it } from 'vitest';
import { l1Loop, loopGain } from '@/analysis/loop';
import { analyseRateLoop } from '@/analysis/mimo';
import { buildReport, meetsMargins, REPORT_ENTRIES, type ReportRow } from '@/analysis/report';
import { logspace, margins } from '@/math/margins';
import { defaultParams } from '@/sim/params';

/** Four rows of the card: enough to check it, few enough for `make check`. */
const pick = (level: 1 | 3, id: string) =>
  REPORT_ENTRIES.find((e) => e.level === level && e.id === id)!;

describe('the robustness report card', () => {
  let rows: ReportRow[];
  const row = (level: 1 | 3, id: string) => rows.find((r) => r.level === level && r.id === id)!;

  beforeAll(() => {
    rows = buildReport([pick(1, 'pid'), pick(3, 'pid'), pick(3, 'geo-indi'), pick(3, 'policy')]);
    // About fifteen seconds alone; much longer on a busy machine.
  }, 600_000);

  it('the measured margins of the PID loops are the ones the models predict', () => {
    // L1: the altitude loop of lessons III.1–III.4.
    const p1 = defaultParams();
    p1.wind.enabled = false;
    const m = l1Loop(p1)!;
    const w = logspace(2 * Math.PI * 0.02, 2 * Math.PI * 100, 1500);
    const model1 = margins(
      w,
      w.map((x) => loopGain(m, x)),
    );
    expect(Math.abs(row(1, 'pid').pmDeg - model1.pmDeg)).toBeLessThan(1.5);
    expect(Math.abs(row(1, 'pid').delayMs - model1.delayMargin * 1000)).toBeLessThan(8);
    // L3: one channel of the roll–pitch loop of lesson III.12.
    const p3 = defaultParams();
    p3.sim.level = 3;
    const model3 = analyseRateLoop(p3)!.one;
    expect(Math.abs(row(3, 'pid').pmDeg - model3.pmDeg)).toBeLessThan(1.5);
    expect(Math.abs(row(3, 'pid').delayMs - model3.delayMargin * 1000)).toBeLessThan(5);
  });

  it('the controller that wins on RMS is not the robust one', () => {
    const pid = row(3, 'pid');
    const indi = row(3, 'geo-indi');
    expect(indi.rms).toBeLessThan(pid.rms / 3);
    expect(indi.pmDeg).toBeLessThan(pid.pmDeg - 5);
    expect(indi.delayMs).toBeLessThan(pid.delayMs);
    expect(indi.passed).toBeLessThan(pid.passed - 5);
    expect(meetsMargins(indi)).toBe(false);
    // The bound is below the observed rate, and both are below 1 with failures.
    expect(indi.bound).toBeLessThan(indi.passed / indi.runs);
  });

  it('a neural policy flies, yet the sweep finds no linear loop to put a margin on', () => {
    const policy = row(3, 'policy');
    expect(policy.crashed).toBe(false);
    expect(policy.linear).toBe(false);
    expect(meetsMargins(policy)).toBe(false);
    for (const r of [row(1, 'pid'), row(3, 'pid'), row(3, 'geo-indi')]) expect(r.linear).toBe(true);
  });

  it('the Part I altitude PID meets the classical requirement with room to spare', () => {
    const pid = row(1, 'pid');
    expect(meetsMargins(pid)).toBe(true);
    expect(pid.passed).toBe(pid.runs);
    expect(pid.gmDb).toBeGreaterThan(20);
  });
});
