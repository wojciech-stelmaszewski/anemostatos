import { describe, expect, it } from 'vitest';
import { defaultGains, Pid } from '@/control/pid';

const dt = 0.01;

describe('Pid', () => {
  it('P term is Kp·e', () => {
    const pid = new Pid();
    const t = pid.update(defaultGains({ kp: 3, ki: 0, kd: 0 }), 2, 0.5, dt);
    expect(t.p).toBeCloseTo(4.5);
    expect(t.output).toBeCloseTo(4.5);
  });

  it('I term accumulates Ki·e·dt', () => {
    const pid = new Pid();
    const g = defaultGains({ kp: 0, ki: 2, kd: 0 });
    for (let k = 0; k < 100; k++) pid.update(g, 1, 0, dt);
    expect(pid.last.i).toBeCloseTo(2 * 1 * 1.0, 6); // 1 s of unit error
  });

  it('changing Ki does not jump the output (integral stored pre-scaled)', () => {
    const pid = new Pid();
    const g = defaultGains({ kp: 0, ki: 1, kd: 0 });
    for (let k = 0; k < 100; k++) pid.update(g, 1, 1, dt); // zero error, nothing accumulates
    pid.integral = 3;
    const before = pid.update(g, 1, 1, dt).output;
    const after = pid.update({ ...g, ki: 10 }, 1, 1, dt).output;
    expect(after).toBeCloseTo(before);
  });

  it('D on error kicks on a setpoint step; D on measurement does not', () => {
    const onError = new Pid();
    const onMeas = new Pid();
    const gE = defaultGains({ kp: 0, ki: 0, kd: 1, derivativeOn: 'error', dFilterHz: 0 });
    const gM = { ...gE, derivativeOn: 'measurement' as const };
    onError.update(gE, 0, 0, dt);
    onMeas.update(gM, 0, 0, dt);
    expect(onError.update(gE, 1, 0, dt).d).toBeCloseTo(100); // Δe/dt = 1/0.01
    expect(onMeas.update(gM, 1, 0, dt).d).toBeCloseTo(0);
  });

  it('D on measurement opposes motion', () => {
    const pid = new Pid();
    const g = defaultGains({ kp: 0, ki: 0, kd: 2, dFilterHz: 0 });
    pid.update(g, 0, 0, dt);
    expect(pid.update(g, 0, 0.1, dt).d).toBeCloseTo(-20); // moving up at 10 m/s
  });

  it('first update after reset has no derivative', () => {
    const pid = new Pid();
    const g = defaultGains({ kp: 0, ki: 0, kd: 5, derivativeOn: 'error', dFilterHz: 0 });
    expect(pid.update(g, 10, 0, dt).d).toBe(0);
  });

  it('D filter smooths a derivative step with the expected time constant', () => {
    const pid = new Pid();
    const cutoff = 5;
    const g = defaultGains({ kp: 0, ki: 0, kd: 1, dFilterHz: cutoff });
    const step = 0.001;
    pid.update(g, 0, 0, step);
    // Constant velocity -1 m/s → raw D = +1. After one time constant ~63 %.
    const tau = 1 / (2 * Math.PI * cutoff);
    let y = 0;
    let d = 0;
    for (let t = 0; t < tau; t += step) {
      y -= step;
      d = pid.update(g, 0, y, step).d;
    }
    expect(d).toBeGreaterThan(0.6);
    expect(d).toBeLessThan(0.67);
  });

  it('clamps the output and flags saturation', () => {
    const pid = new Pid();
    const t = pid.update(defaultGains({ kp: 10 }), 1, 0, dt, 0, -2, 2);
    expect(t.output).toBe(2);
    expect(t.unsaturated).toBe(10);
    expect(t.saturated).toBe(true);
  });

  describe('anti-windup', () => {
    // Saturated for 2 s with a persistent positive error.
    const windUp = (mode: 'none' | 'clamp' | 'integral-limit' | 'back-calculation') => {
      const pid = new Pid();
      const g = defaultGains({ kp: 1, ki: 1, kd: 0, antiWindup: mode, iLimit: 0.5, kb: 5 });
      for (let k = 0; k < 200; k++) pid.update(g, 5, 0, dt, 0, -1, 1);
      return pid.integral;
    };

    it('none: integral grows without bound', () => {
      expect(windUp('none')).toBeCloseTo(10, 6);
    });
    it('clamp: integral frozen while saturated', () => {
      expect(windUp('clamp')).toBeCloseTo(0, 6);
    });
    it('integral-limit: integral capped', () => {
      expect(windUp('integral-limit')).toBeCloseTo(0.5, 6);
    });
    it('back-calculation: integral settles far below the unprotected value', () => {
      const i = windUp('back-calculation');
      expect(i).toBeLessThan(0);
      expect(Math.abs(i)).toBeLessThan(5);
    });
  });

  it('disabling I clears the integral', () => {
    const pid = new Pid();
    const g = defaultGains({ kp: 0, ki: 1, kd: 0 });
    for (let k = 0; k < 50; k++) pid.update(g, 1, 0, dt);
    expect(pid.update({ ...g, iOn: false }, 1, 0, dt).i).toBe(0);
  });
});
