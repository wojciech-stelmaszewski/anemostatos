import { describe, expect, it } from 'vitest';
import { Simulation } from '@/engine/simulation';
import { manoeuvreFor, manoeuvreSpec, planCollocation, planMinSnap } from '@/guidance/collocation';
import { planLanding } from '@/guidance/pdg';
import { CAMPAIGN_CASES, flyCase, storeLandings, type LandingRun } from '@/guidance/pdgCampaign';
import { LESSONS } from '@/lessons/lessons';
import { length } from '@/math/vec3';
import { defaultParams, GRAVITY, type Params } from '@/sim/params';
import { initialLander } from '@/sim/vehicles/lander';

const lesson = (id: string) => LESSONS.find((l) => l.id === id)!;
const setUp = (id: string, solved = false): Params => {
  const p = defaultParams();
  const l = lesson(id);
  p.sim.level = l.level;
  l.setup!(p);
  if (solved) l.solution!(p);
  return p;
};
const start = (p: Params) => {
  const s = initialLander(p.lander);
  return { r: s.pos, v: s.vel, m: s.mass };
};

describe('powered-descent guidance (lesson IV.5)', () => {
  const p = setUp('pdg');
  const plan = planLanding(start(p), p.lander, p.pdg);

  it('the relaxation is lossless: ‖u‖ = σ at every interval', () => {
    expect(plan.status).toBe('optimal');
    expect(plan.slackGap).toBeLessThan(1e-6);
  });

  it('the fuel-optimal thrust is full, least, full; the plan stays in the cone and ends at the gate', () => {
    const thrust = plan.sigma.map((s, k) => s * plan.m[k]!);
    const top = p.lander.thrustMax * p.pdg.throttle;
    expect(thrust[0]!).toBeGreaterThan(0.95 * top);
    expect(thrust.at(-1)!).toBeGreaterThan(0.95 * top);
    expect(Math.min(...thrust)).toBeLessThan(1.05 * p.lander.thrustMin);
    const tg = Math.tan((p.lander.glideSlopeDeg * Math.PI) / 180);
    for (const r of plan.r.slice(1, -1))
      expect(r.y).toBeGreaterThanOrEqual(tg * Math.hypot(r.x, r.z) - 1e-6);
    const end = plan.r.at(-1)!;
    expect(Math.hypot(end.x, end.y - p.pdg.gateAltitude, end.z)).toBeLessThan(1e-4);
    expect(plan.fuel).toBeLessThan(p.lander.fuel - p.pdg.reserve);
  }, 120_000);

  it('reports a start with too little propellant as infeasible', () => {
    const q = setUp('pdg');
    q.lander.fuel = 0.9;
    expect(planLanding(start(q), q.lander, q.pdg).status).toBe('infeasible');
  }, 120_000);

  const fly = (q: Params) => {
    const sim = new Simulation(q);
    for (let k = 0; k < 30000 && !sim.pdgLander!.landed && !sim.pdgLander!.crashed; k++) sim.step();
    return sim;
  };

  it('open loop the wind carries it off and it crashes; re-planning lands it on the pad', () => {
    const open = fly(setUp('pdg')).pdgLander!;
    expect(open.crashed).toBe(true);
    const sim = fly(setUp('pdg', true));
    const s = sim.pdgLander!;
    expect(s.landed).toBe(true);
    expect(s.touchdownSpeed).toBeLessThan(1);
    expect(s.miss).toBeLessThan(1);
    // The engine's shortfall is measured in flight.
    expect(sim.pdg.efficiency).toBeCloseTo(0.95, 3);
    expect(sim.pdg.replans).toBeGreaterThan(5);
  }, 120_000);

  it('open loop the dispersed starts crash', () => {
    const q = setUp('pdg');
    const runs = [1, 2, 5].map((c) => flyCase(q, c));
    expect(runs.every((r) => r.outcome === 'crashed')).toBe(true);
  }, 120_000);

  it('with re-planning the campaign has no failure, and a start that cannot land is reported', () => {
    const q = setUp('pdg', true);
    const runs: LandingRun[] = [];
    for (let c = 1; c <= CAMPAIGN_CASES; c++) runs.push(flyCase(q, c));
    expect(runs.filter((r) => !r.passed)).toEqual([]);
    expect(runs.some((r) => r.outcome === 'infeasible')).toBe(true);
    for (const r of runs.filter((x) => x.outcome === 'landed')) {
      expect(r.speed).toBeLessThan(1);
      expect(r.miss).toBeLessThan(1);
      expect(r.minSlope).toBeGreaterThanOrEqual(q.lander.glideSlopeDeg - 0.5);
    }
    // The lesson's goal reads the finished campaign.
    storeLandings(q, runs);
    const sim = new Simulation(q);
    const ok = lesson('pdg').goal!.check({ sim, metrics: null as never, prediction: null });
    expect(ok).toBe(true);
  }, 300_000);
});

describe('planning the whole flight (lesson IV.6)', () => {
  const p = setUp('collocation');
  const aH = GRAVITY * Math.tan((p.control.l3.maxTiltDeg * Math.PI) / 180);
  const spec = manoeuvreSpec(p);
  const bound = lesson('collocation').predict!.truth(p);

  it('the bang-bang bound with 90 % of g·tan 35°: 2√(D/a) = 1.97 s', () => {
    expect(bound).toBeCloseTo(2 * Math.sqrt(6 / (0.9 * aH)), 9);
    expect(bound).toBeCloseTo(1.97, 2);
  });

  it('min-snap in 2.5 s asks for more than the tilt limit; at 2.62 s it fits', () => {
    expect(planMinSnap(spec).peakAccel).toBeGreaterThan(1.08 * aH);
    expect(planMinSnap({ ...spec, duration: 2.62 }).peakAccel).toBeLessThan(aH);
  });

  it('least-time transcription keeps every limit and takes about a quarter more than the bound', () => {
    const m = planCollocation({ ...spec, minTime: true });
    expect(m.status).toBe('optimal');
    expect(m.peakAccel).toBeLessThan(spec.accelMax * 1.001);
    expect(m.peakJerk).toBeLessThan(spec.jerkMax * 1.001);
    expect(m.clearance).toBeGreaterThan(-1e-3);
    expect(m.duration).toBeLessThan(2.55);
    expect(m.duration / bound).toBeGreaterThan(1.2);
    expect(m.duration / bound).toBeLessThan(1.32);
    const end = m.at(m.duration);
    expect(Math.hypot(end.pos.x - spec.distance, end.pos.z)).toBeLessThan(1e-3);
    expect(length(end.vel)).toBeLessThan(1e-3);
  }, 120_000);

  it('a fixed time too short for the limits is reported, and the drone holds the start', () => {
    const q = setUp('collocation');
    q.plan.method = 'collocation';
    q.plan.duration = 2.2;
    expect(manoeuvreFor(q).status).toBe('infeasible');
  }, 120_000);

  const flight = (q: Params) => {
    const sim = new Simulation(q);
    for (let k = 0; k < 10000; k++) sim.step();
    const from = sim.profileStart + q.plan.delay;
    const w = sim.telemetry.window(['pos.x.sat', 'pos.z.sat', 'zone.dist', 'pos.x'], from);
    return {
      saturated: w.series[0]!.filter((v, i) => v > 0 || w.series[1]![i]! > 0).length,
      closest: Math.min(...w.series[2]!.filter(Number.isFinite)),
      x: sim.state.pos.x,
    };
  };

  it('flown, min-snap in 2.5 s saturates; least-time transcription does not, and both clear the pillar', () => {
    const snap = flight(setUp('collocation'));
    expect(snap.saturated).toBeGreaterThan(50);
    expect(snap.closest).toBeGreaterThan(0);
    const best = flight(setUp('collocation', true));
    expect(best.saturated).toBe(0);
    expect(best.closest).toBeGreaterThan(0);
    expect(Math.abs(best.x - 6)).toBeLessThan(0.3);
  }, 120_000);
});
