import { describe, expect, it } from 'vitest';
import { analyseRateLoop, rateLoopModel, singularValues, sweepMimo } from '@/analysis/mimo';
import { aliasHz, bandRms, ghostRate, motorJitter, spectrumOf } from '@/analysis/spectrum';
import { Simulation } from '@/engine/simulation';
import { Biquad, biquadAt, lowpassCoefs, notchCoefs } from '@/estimation/filters';
import { cabs, csub } from '@/math/complex';
import { defaultParams, GRAVITY, type Params } from '@/sim/params';
import { rotorHz } from '@/sim/vibration';

const quad = (mod?: (p: Params) => void): Params => {
  const p = defaultParams();
  p.sim.level = 3;
  p.wind.enabled = false;
  mod?.(p);
  return p;
};
const shaking =
  (gyroDeg: number, mod?: (p: Params) => void) =>
  (p: Params): void => {
    p.vibration = { gyroDeg, acc: 0, hoverHz: 110 };
    mod?.(p);
  };
const fly = (p: Params, seconds = 14): Simulation => {
  const sim = new Simulation(p);
  for (let k = 0; k < seconds * 1000; k++) sim.step();
  return sim;
};
/** Frequency of the largest line of a spectrum, Hz. */
const peakHz = (x: ArrayLike<number>): number => {
  const s = spectrumOf(x);
  let best = 1;
  for (let i = 2; i < s.amp.length; i++) if (s.amp[i]! > s.amp[best]!) best = i;
  return s.fHz[best]!;
};
const gain = (c: Parameters<typeof biquadAt>[0], hz: number, dt = 0.001) => {
  const v = biquadAt(c, 2 * Math.PI * hz * dt);
  return Math.hypot(v.re, v.im);
};

describe('sampling folds frequencies', () => {
  it('the alias of a tone', () => {
    expect(aliasHz(110, 100)).toBeCloseTo(10, 12);
    expect(aliasHz(180, 200)).toBeCloseTo(20, 12);
    expect(aliasHz(110, 250)).toBeCloseTo(110, 12);
    expect(aliasHz(110, 200)).toBeCloseTo(90, 12);
    expect(aliasHz(30, 100)).toBeCloseTo(30, 12);
  });

  it('band power picks one tone out of two', () => {
    const x = Array.from(
      { length: 8000 },
      (_, k) =>
        3 * Math.sin(2 * Math.PI * 10 * k * 0.001) + Math.sin(2 * Math.PI * 110 * k * 0.001),
    );
    expect(bandRms(x, 1, 60)).toBeCloseTo(3 / Math.SQRT2, 1);
    expect(bandRms(x, 60, 500)).toBeCloseTo(1 / Math.SQRT2, 1);
    // The amplitude spectrum reads the amplitude of a line directly.
    const s = spectrumOf(x);
    expect(Math.max(...s.amp)).toBeGreaterThan(2.5);
    expect(Math.max(...s.amp)).toBeLessThan(3.2);
  });
});

describe('second-order sections', () => {
  it('a notch removes its centre and nothing far from it', () => {
    const c = notchCoefs(110, 4, 0.001);
    expect(gain(c, 110)).toBeLessThan(1e-9);
    expect(gain(c, 2)).toBeCloseTo(1, 3);
    expect(gain(c, 400)).toBeCloseTo(1, 1);
    // The −3 dB width is the centre over Q.
    expect(gain(c, 110 - 13.75)).toBeGreaterThan(0.6);
    expect(gain(c, 110 - 13.75)).toBeLessThan(0.8);
    // Phase lost far below the centre: about f/(Q·f0) radians.
    const v = biquadAt(c, 2 * Math.PI * 2.5 * 0.001);
    expect(-Math.atan2(v.im, v.re)).toBeCloseTo(2.5 / (4 * 110), 3);
  });

  it('the low-pass is −3 dB at its cutoff', () => {
    const c = lowpassCoefs(40, 0.001);
    expect(gain(c, 40)).toBeCloseTo(Math.SQRT1_2, 3);
    expect(gain(c, 0.01)).toBeCloseTo(1, 6);
    expect(gain(c, 400)).toBeLessThan(0.02);
  });

  it('a running section does what its frequency response says', () => {
    const c = notchCoefs(110, 2, 0.001);
    const f = new Biquad();
    let peak = 0;
    for (let k = 0; k < 4000; k++) {
      const y = f.update(c, Math.sin(2 * Math.PI * 90 * k * 0.001));
      if (k > 3000) peak = Math.max(peak, Math.abs(y));
    }
    expect(peak).toBeCloseTo(gain(c, 90), 2);
  });
});

describe('rotor vibration and the IMU front end', () => {
  it('the rotor frequency goes with the square root of the thrust', () => {
    const d = defaultParams().drone;
    const hover = (d.mass * GRAVITY) / 4;
    expect(rotorHz(hover, d, 110)).toBeCloseTo(110, 9);
    expect(rotorHz(2 * hover, d, 110)).toBeCloseTo(110 * Math.SQRT2, 9);
    expect(rotorHz(0, d, 110)).toBe(0);
  });

  it('is off by default: the gyro reads the body rate and the flight is unchanged', () => {
    const plain = fly(
      quad((p) => (p.setpoint.profile = 'square')),
      6,
    );
    const tapT = plain.tap.last('gyro.true', 3000);
    const tapS = plain.tap.last('gyro.sensor', 3000);
    // The sensor reading is the state before the step, the truth the state after it.
    for (let i = 1; i < 3000; i++) expect(tapS[i]).toBeCloseTo(tapT[i - 1]!, 4);
  });

  it('shakes the reading at the rotor frequency and barely moves the body', () => {
    const sim = fly(quad(shaking(5)));
    expect(peakHz(sim.tap.last('gyro.sensor', 8000))).toBeCloseTo(110, 0);
    const read = bandRms(sim.tap.last('gyro.sensor', 8000), 60, 500);
    const real = bandRms(sim.tap.last('gyro.true', 8000), 60, 500);
    expect(read).toBeGreaterThan(1);
    expect(real).toBeLessThan(0.02 * read);
    expect(ghostRate(sim)).toBeLessThan(0.01);
  });

  it('a 100 Hz IMU reports the 110 Hz tone at 10 Hz, and the drone answers it', () => {
    const sim = fly(quad(shaking(5, (p) => (p.sensors.imuRateHz = 100))));
    expect(peakHz(sim.tap.last('gyro.sensor', 8000))).toBeCloseTo(10, 0);
    expect(ghostRate(sim)).toBeGreaterThan(0.5);
    // The motion the loop makes out of the ghost is real.
    expect(bandRms(sim.tap.last('gyro.true', 8000), 5, 20)).toBeGreaterThan(0.05);
  });

  it('only sampling faster or filtering before the sampler removes the ghost', () => {
    const at = (mod: (p: Params) => void) => {
      const p = quad(shaking(5, mod));
      return { ghost: ghostRate(fly(p)), pm: analyseRateLoop(p)!.both.pmDeg };
    };
    const fast = at((p) => (p.sensors.imuRateHz = 250));
    expect(fast.ghost).toBeLessThan(0.05);
    expect(fast.pm).toBeGreaterThan(43);
    // The anti-alias filter works and costs phase.
    const aa = at((p) => {
      p.sensors.imuRateHz = 100;
      p.sensors.aaFilterHz = 20;
    });
    expect(aa.ghost).toBeLessThan(0.1);
    expect(aa.pm).toBeLessThan(35);
    // A digital notch on the ghost leaves it in the reading and takes phase as well.
    const notch = at((p) => {
      p.sensors.imuRateHz = 100;
      p.control.gyroFilter = { lpfHz: 0, notch: 'fixed', notchHz: 10, notchQ: 2 };
    });
    expect(notch.ghost).toBeGreaterThan(0.5);
    expect(notch.pm).toBeLessThan(37);
  });
});

describe('the gyro chain in the loop model', () => {
  it('anti-alias filter, sample-and-hold, low-pass and notch: S within 3 % of the flight', () => {
    const p = quad((q) => {
      q.sensors.imuRateHz = 250;
      q.sensors.aaFilterHz = 80;
      q.control.gyroFilter = { lpfHz: 60, notch: 'fixed', notchHz: 110, notchQ: 2 };
    });
    for (const m of sweepMimo(p, [1, 3, 8])) {
      const q = rateLoopModel(p, m.fHz);
      expect(cabs(csub(m.s[0][0], q.s[0][0])) / singularValues(q.s)[0]).toBeLessThan(0.03);
    }
    // And the chain costs what it should: several degrees of phase margin.
    expect(analyseRateLoop(p)!.both.pmDeg).toBeLessThan(analyseRateLoop(quad())!.both.pmDeg - 4);
  });
});

describe('keeping the vibration out of the motors', () => {
  const climbing = (mod?: (p: Params) => void) =>
    quad(
      shaking(15, (p) => {
        p.setpoint.profile = 'sine';
        p.setpoint.profileAxis = 'y';
        p.setpoint.profileAmplitude = 1.5;
        p.setpoint.profilePeriod = 4;
        mod?.(p);
      }),
    );
  const result = (p: Params) => ({
    jitter: motorJitter(fly(p, 16)),
    pm: analyseRateLoop(p)!.both.pmDeg,
  });

  it('unfiltered, the D term sends the tone to the motors', () => {
    const quiet = result(climbing((p) => (p.vibration.gyroDeg = 0)));
    expect(quiet.jitter).toBeLessThan(0.0066);
    expect(result(climbing()).jitter).toBeGreaterThan(20 * quiet.jitter);
  });

  it('the rotor tone moves from below 90 Hz to almost 130 Hz over the manoeuvre', () => {
    const p = climbing();
    const sim = new Simulation(p);
    let lo = Infinity;
    let hi = 0;
    for (let k = 0; k < 16000; k++) {
      sim.step();
      if (k > 8000) {
        const f = rotorHz(sim.state.rotors[0]!, p.drone, 110);
        lo = Math.min(lo, f);
        hi = Math.max(hi, f);
      }
    }
    expect(lo).toBeLessThan(90);
    expect(hi).toBeGreaterThan(125);
  });

  it('a low-pass that reaches the goal costs eight degrees; the fixed notch misses the tone', () => {
    const lpf = result(climbing((p) => (p.control.gyroFilter.lpfHz = 20)));
    expect(lpf.jitter).toBeLessThan(0.01);
    expect(lpf.pm).toBeLessThan(38);
    const fixed = result(climbing((p) => (p.control.gyroFilter.notch = 'fixed')));
    expect(fixed.pm).toBeGreaterThan(44);
    expect(fixed.jitter).toBeGreaterThan(0.08);
    // At hover the same notch is perfect.
    const hover = climbing((p) => {
      p.control.gyroFilter.notch = 'fixed';
      p.setpoint.profile = 'none';
    });
    expect(motorJitter(fly(hover))).toBeLessThan(1e-3);
  });

  it('the notch that follows each rotor reaches the quiet baseline for one degree', () => {
    const rpm = result(climbing((p) => (p.control.gyroFilter.notch = 'rpm')));
    expect(rpm.jitter).toBeLessThan(0.0066);
    expect(rpm.pm).toBeGreaterThan(43.5);
  });
});
