import { describe, expect, it } from 'vitest';
import { linearize } from '@/analysis/linearize';
import { l1Plant } from '@/analysis/plant';
import { Simulation } from '@/engine/simulation';
import { LESSONS } from '@/lessons/lessons';
import { atmosphere, dynamicPressure, mach } from '@/sim/atmosphere';
import { defaultParams, GRAVITY, type Params } from '@/sim/params';
import { quadrotor } from '@/sim/vehicles/quadrotor';
import {
  G0,
  ignitionAltitude,
  ignitionAltitudeConstantMass,
  initialRocket,
  rocket,
  stepRocket,
} from '@/sim/vehicles/rocket';
import { v3 } from '@/math/vec3';

describe('the vehicle abstraction', () => {
  it('the quadrotor trims to hover: one step leaves the trim point where it is', () => {
    const p = defaultParams();
    const { state, input } = quadrotor.trim(p);
    const s = structuredClone(state);
    quadrotor.step(s, input, { wind: v3(), external: v3() }, 0.001, p);
    const before = quadrotor.toVector(state, state);
    const after = quadrotor.toVector(s, state);
    for (let i = 0; i < before.length; i++) expect(after[i]).toBeCloseTo(before[i]!, 9);
  });

  it('linearising the step at hover gives the hand-made L1 plant of Part III', () => {
    const p = defaultParams();
    const lin = linearize(quadrotor, p, quadrotor.trim(p));
    const pl = l1Plant(p);
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++)
        expect(Math.abs(lin.a[i]![j]! - pl.a[i]![j]!)).toBeLessThan(
          1e-6 * Math.max(1, Math.abs(pl.a[i]![j]!)) + 1e-9,
        );
      expect(lin.b[i]![0]! / pl.b[i]![0]!).toBeCloseTo(1, 4);
    }
  });

  it('the rocket trims to a hover and linearises to a double integrator with a mass state', () => {
    const p = defaultParams();
    const at = rocket.trim(p);
    const lin = linearize(rocket, p, at);
    const dt = 0.001;
    // h⁺ = h + dt·v⁺, v⁺ = v + dt·(T/m − g), m⁺ = m − dt·T/(Isp·g0)
    expect(lin.a[0]![1]).toBeCloseTo(dt, 7);
    expect(lin.b[1]![0]).toBeCloseTo(dt / at.state.mass, 9);
    expect(lin.b[2]![0]).toBeCloseTo(-dt / (p.rocket.isp * G0), 9);
    // A lighter rocket accelerates more for the same thrust: ∂v⁺/∂m = −dt·T/m².
    expect(lin.a[1]![2]).toBeCloseTo((-dt * at.input.thrust) / at.state.mass ** 2, 9);
  });
});

describe('the standard atmosphere', () => {
  it('matches the ISA table at sea level, 5 km and 11 km', () => {
    expect(atmosphere(0).density).toBeCloseTo(1.225, 3);
    expect(atmosphere(0).speedOfSound).toBeCloseTo(340.29, 1);
    expect(atmosphere(5000).density).toBeCloseTo(0.7364, 3);
    expect(atmosphere(11000).temperature).toBeCloseTo(216.65, 2);
    expect(atmosphere(11000).density).toBeCloseTo(0.3639, 3);
    expect(atmosphere(15000).temperature).toBeCloseTo(216.65, 2);
    expect(dynamicPressure(1.225, 100)).toBeCloseTo(6125, 6);
    expect(mach(340.29, 0)).toBeCloseTo(1, 3);
  });
});

describe('the 1D rocket', () => {
  it('burns propellant at T/(Isp·g0) and cannot burn what it does not have', () => {
    const p = defaultParams();
    const r = { ...p.rocket, fuel: 0.5 };
    const s = initialRocket(r);
    for (let k = 0; k < 1000; k++) stepRocket(s, { thrust: r.thrustMax }, v3(), 0.001, r);
    expect(r.fuel - s.fuel).toBeCloseTo(r.thrustMax / (r.isp * G0), 3);
    for (let k = 0; k < 10000; k++) stepRocket(s, { thrust: r.thrustMax }, v3(), 0.001, r);
    expect(s.fuel).toBe(0);
    expect(s.mass).toBeCloseTo(r.dryMass, 9);
    expect(s.thrust === 0 || s.landed || s.crashed).toBe(true);
  });

  it('a lit engine gives at least its minimum thrust', () => {
    const p = defaultParams();
    const s = initialRocket(p.rocket);
    stepRocket(s, { thrust: 1 }, v3(), 0.001, p.rocket);
    expect(s.thrust).toBe(p.rocket.thrustMin);
  });

  it('the constant-mass estimate of the ignition altitude is about one per cent high', () => {
    const r = { ...defaultParams().rocket, fuel: 3 };
    const exact = ignitionAltitude(r);
    const est = ignitionAltitudeConstantMass(r);
    expect(est).toBeCloseTo((r.startAlt * (r.dryMass + r.fuel) * GRAVITY) / r.thrustMax, 9);
    expect(est / exact - 1).toBeGreaterThan(0.005);
    expect(est / exact - 1).toBeLessThan(0.02);
  });
});

describe('lesson IV.4: land on the last drop', () => {
  const lesson = LESSONS.find((l) => l.id === 'suicideburn')!;
  const land = (ignitionAbove: number) => {
    const p = defaultParams();
    p.sim.level = 1;
    lesson.setup!(p);
    p.control.lander.ignitionAlt = ignitionAltitude(p.rocket) + ignitionAbove;
    const sim = new Simulation(p);
    const m0 = sim.rocket!.mass;
    let burned = 0;
    let litMass = NaN;
    for (let k = 0; k < 60000 && !sim.rocket!.landed && !sim.rocket!.crashed; k++) {
      sim.step();
      if (Number.isNaN(litMass) && sim.rocket!.thrust > 0) litMass = sim.rocket!.mass;
      if (sim.rocket!.v >= -0.5 && !Number.isNaN(litMass) && burned === 0)
        burned = (litMass - sim.rocket!.mass) / m0;
    }
    return { r: sim.rocket!, burned };
  };

  it('lit at the exact altitude it lands softly; the burn costs about three per cent of the mass', () => {
    const { r, burned } = land(0);
    expect(r.landed).toBe(true);
    expect(-r.touchdown).toBeLessThan(1);
    expect(burned).toBeGreaterThan(0.02);
    expect(burned).toBeLessThan(0.04);
  });

  it('half a metre too low it hits the pad at about five metres per second', () => {
    const { r } = land(-0.5);
    expect(r.crashed).toBe(true);
    expect(-r.touchdown).toBeGreaterThan(4);
    expect(-r.touchdown).toBeLessThan(6);
  });

  it('lit at the constant-mass estimate it lands; ten metres high it runs dry and falls', () => {
    const p = defaultParams();
    lesson.setup!(p);
    const est = ignitionAltitudeConstantMass(p.rocket) - ignitionAltitude(p.rocket);
    expect(land(est).r.landed).toBe(true);
    const high = land(10).r;
    expect(high.crashed).toBe(true);
    expect(high.fuel).toBe(0);
  });

  it('the setup lights it far too high: it hovers down, runs dry and falls', () => {
    const p: Params = defaultParams();
    lesson.setup!(p);
    const sim = new Simulation(p);
    for (let k = 0; k < 45000 && !sim.rocket!.crashed; k++) sim.step();
    expect(sim.rocket!.crashed).toBe(true);
    expect(sim.rocket!.fuel).toBe(0);
  });
});
