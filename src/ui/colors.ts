/**
 * Semantic colours shared by charts, 3D overlays and the parameter panel.
 * One quantity = one hue, everywhere.
 */
export const SIGNAL = {
  // Neutral references
  setpoint: '#9aa3b2',
  truth: '#6b7280',
  output: '#e6e9ef',
  // Categorical — validated with the dataviz palette checker on the dark surface
  // (P/I/D/FF adjacent pairs: CVD ΔE ≥ 9.4, normal-vision ΔE ≥ 19.7).
  measurement: '#3987e5',
  error: '#e66767',
  p: '#d95926',
  i: '#199e70',
  d: '#9085e9',
  ff: '#c98500',
  wind: '#3fb950',
  windAxes: ['#e66767', '#199e70', '#3987e5'],
  // Forces in the 3D view: thrust is the controller output, so it shares its colour.
  thrust: '#e6e9ef',
  gravity: '#9aa3b2',
  drag: '#3fb950',
  net: '#f5f5f5',
  motors: ['#3987e5', '#d95926', '#199e70', '#c98500'],
} as const;

export const SCENE = {
  background: '#0b0e13',
  skyTop: '#05070b',
  skyHorizon: '#1a2230',
  gridMinor: '#2a3342',
  gridMajor: '#3d4a5e',
  gridSection: '#5b6b84',
  axisX: '#e5484d',
  axisZ: '#3e8ef7',
  accent: '#ff7a1a',
} as const;
