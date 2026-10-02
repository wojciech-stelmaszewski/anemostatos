/**
 * The International Standard Atmosphere up to 20 km (docs/aerospace-gnc.md §3.1): a troposphere
 * whose temperature falls 6.5 K per km, then an isothermal layer at 216.65 K. Enough for the
 * aircraft and the rocket of Part IV; above 20 km the layer is simply continued.
 */
const T0 = 288.15; // K
const P0 = 101325; // Pa
const L = 0.0065; // K/m
const R = 287.05287; // J/(kg·K), dry air
const G0 = 9.80665; // m/s²
const GAMMA = 1.4;
const H_TROPO = 11000; // m
const T_TROPO = T0 - L * H_TROPO;
const P_TROPO = P0 * (T_TROPO / T0) ** (G0 / (R * L));

export interface AirState {
  /** Temperature, K. */
  temperature: number;
  /** Pressure, Pa. */
  pressure: number;
  /** Density, kg/m³. */
  density: number;
  /** Speed of sound, m/s. */
  speedOfSound: number;
}

/** The standard atmosphere at geometric altitude `h`, m (below sea level it is extrapolated). */
export function atmosphere(h: number): AirState {
  let temperature: number;
  let pressure: number;
  if (h <= H_TROPO) {
    temperature = T0 - L * h;
    pressure = P0 * (temperature / T0) ** (G0 / (R * L));
  } else {
    temperature = T_TROPO;
    pressure = P_TROPO * Math.exp((-G0 * (h - H_TROPO)) / (R * T_TROPO));
  }
  return {
    temperature,
    pressure,
    density: pressure / (R * temperature),
    speedOfSound: Math.sqrt(GAMMA * R * temperature),
  };
}

/** Dynamic pressure ½·ρ·V², Pa. */
export const dynamicPressure = (density: number, speed: number): number =>
  0.5 * density * speed * speed;

/** Mach number of `speed` at altitude `h`. */
export const mach = (speed: number, h: number): number => speed / atmosphere(h).speedOfSound;
