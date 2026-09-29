// Screenshots of the running app for the book. Needs `make dev` (or any Vite dev server) and Chrome.
// Usage: node book/tools/shots.mjs [baseUrl] [nameFilter]
// Each shot starts a lesson, fast-forwards the simulation headlessly to `t`, pauses and captures.
import { spawn } from 'node:child_process';
import { mkdirSync, writeFileSync } from 'node:fs';

const base = process.argv[2] ?? 'http://localhost:5173/';
const only = process.argv[3];
const OUT = new URL('../screens/raw/', import.meta.url);
mkdirSync(OUT, { recursive: true });

/** name, lesson id, simulation time to stop at, extra JS run before fast-forwarding. */
const SHOTS = [
  { name: 'meet', lesson: 'meet', t: 14 },
  { name: 'droop', lesson: 'droop', t: 9 },
  { name: 'damper', lesson: 'damper', t: 18.5 },
  { name: 'windup', lesson: 'windup', t: 17 },
  { name: 'kick', lesson: 'kick', t: 12.3 },
  { name: 'noise', lesson: 'noise', t: 9 },
  { name: 'tilt', lesson: 'tilt', t: 14 },
  { name: 'cascade', lesson: 'cascade', t: 14 },
  { name: 'state', lesson: 'state', t: 17.2 },
  { name: 'kalman', lesson: 'kalman', t: 14 },
  { name: 'adrc', lesson: 'adrc', t: 15 },
  { name: 'indi', lesson: 'indi', t: 14, js: `P.useParams.getState().set('control.l3.inner', 'indi');` },
  { name: 'flip', lesson: 'geometric', t: 10.35 },
  { name: 'flatness', lesson: 'flatness', t: 26, js: `P.useParams.getState().set('control.geometric.feedforward', true);` },
  { name: 'mpc', lesson: 'mpc', t: 8.25, js: `P.useParams.getState().set('control.l1.kind', 'mpc');` },
  { name: 'mppi', lesson: 'mppi', t: 10.5, js: `P.useParams.getState().set('control.l3.outer', 'mppi');` },
  { name: 'cbf', lesson: 'cbf', t: 13.2, js: `P.useParams.getState().set('control.l3.safety', 'cbf');` },
  { name: 'policy', lesson: 'policy', t: 18 },
  { name: 'spring', lesson: 'spring', t: 12.4 },
  { name: 'integral', lesson: 'integral', t: 16, js: `P.useParams.getState().set('control.alt.iOn', true);` },
  { name: 'slow', lesson: 'slow', t: 20, js: `P.useParams.getState().set('control.rateHz', 7);` },
  { name: 'edge', lesson: 'edge', t: 14, js: `P.useParams.getState().set('control.alt.kp', 200);` },
  { name: 'challenge', lesson: 'challenge', t: 30 },
  { name: 'inversion', lesson: 'inversion', t: 16, js: `P.useParams.getState().set('control.l3.hzAtt', 5);` },
  { name: 'lqr', lesson: 'lqr', t: 20 },
  { name: 'lqi', lesson: 'lqi', t: 20, js: `P.useParams.getState().set('control.lqr.integral', true);` },
  { name: 'adrcbw', lesson: 'adrc-bw', t: 20 },
  { name: 'indiwind', lesson: 'indi-wind', t: 20, js: `P.useParams.getState().set('control.l3.compensation', 'indi');` },
  { name: 'indisync', lesson: 'indi-sync', t: 14 },
  { name: 'minsnap', lesson: 'minsnap', t: 26, js: `P.useParams.getState().set('setpoint.profile', 'minsnap');` },
  { name: 'horizon', lesson: 'horizon', t: 9 },
  { name: 'mpcl3', lesson: 'mpc-l3', t: 20 },
  { name: 'l1ac', lesson: 'l1ac', t: 16 },
  { name: 'residual', lesson: 'residual', t: 30, js: `P.useParams.getState().set('control.l3.compensation', 'learned');` },
];

const chrome = '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome';
const port = 9334;
const proc = spawn(
  chrome,
  [
    '--headless=new',
    `--remote-debugging-port=${port}`,
    '--use-angle=swiftshader',
    '--enable-unsafe-swiftshader',
    '--window-size=1600,1000',
    '--user-data-dir=/tmp/anemo/profile-book',
    'about:blank',
  ],
  { stdio: 'ignore' },
);
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
let target;
for (let i = 0; i < 50 && !target; i++) {
  await sleep(200);
  try {
    target = (await (await fetch(`http://127.0.0.1:${port}/json`)).json()).find(
      (t) => t.type === 'page',
    );
  } catch {}
}
const ws = new WebSocket(target.webSocketDebuggerUrl);
await new Promise((r) => (ws.onopen = r));
let id = 0;
const pending = new Map();
ws.onmessage = (e) => {
  const m = JSON.parse(e.data);
  if (m.id && pending.has(m.id)) {
    pending.get(m.id)(m.result ?? m.error);
    pending.delete(m.id);
  }
  if (m.method === 'Runtime.exceptionThrown')
    console.log('[exception]', m.params.exceptionDetails.exception?.description?.slice(0, 600));
};
const send = (method, params = {}) =>
  new Promise((r) => {
    const i = ++id;
    pending.set(i, r);
    ws.send(JSON.stringify({ id: i, method, params }));
  });
const evaluate = async (js) => {
  const r = await send('Runtime.evaluate', {
    expression: `(async () => { ${js} })()`,
    awaitPromise: true,
    returnByValue: true,
  });
  if (r?.exceptionDetails) console.log('[eval error]', r.exceptionDetails.exception?.description);
  return r?.result?.value;
};
await send('Runtime.enable');
await send('Page.enable');
await send('Emulation.setDeviceMetricsOverride', {
  width: 1600,
  height: 1000,
  deviceScaleFactor: 2,
  mobile: false,
});

for (const s of SHOTS) {
  if (only && !only.split(',').includes(s.name)) continue;
  await send('Page.navigate', { url: base });
  await sleep(6000);
  const info = await evaluate(`
    const R = await import('/src/lessons/runner.ts');
    const L = await import('/src/lessons/lessons.tsx');
    const S = await import('/src/store/sim.ts');
    const P = await import('/src/store/params.ts');
    R.startLesson(L.LESSONS.find((l) => l.id === '${s.lesson}'));
    ${s.js ?? ''}
    await new Promise((r) => setTimeout(r, 300));
    S.useUi.getState().setPaused(true);
    const sim = S.sim;
    while (sim.t < ${s.t}) sim.step();
    return sim.t.toFixed(2);`);
  await sleep(3500);
  const shot = await send('Page.captureScreenshot', { format: 'png' });
  writeFileSync(new URL(`${s.name}.png`, OUT), Buffer.from(shot.data, 'base64'));
  console.log(s.name, 't =', info);
}
ws.close();
proc.kill();
process.exit(0);
