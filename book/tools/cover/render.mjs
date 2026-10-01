// Capture the cover render (index.html in this folder) with headless Chrome.
// Usage: start the dev server (npx vite --port 5287 --strictPort; COVER_PORT overrides), then
//   node book/tools/cover/render.mjs [out.png] [cssSize] [scale]
// Default: book/tools/cover/render.png, 1500 css px square at scale 2 (3000 × 3000 px).
import { spawn } from 'node:child_process';
import { writeFileSync } from 'node:fs';

const out = process.argv[2] ?? 'book/tools/cover/render.png';
const size = Number(process.argv[3] ?? 1500);
const scale = Number(process.argv[4] ?? 2);
const url = `http://localhost:${process.env.COVER_PORT ?? 5287}/book/tools/cover/index.html`;
const chrome = '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome';
const port = 9334;
const proc = spawn(
  chrome,
  [
    '--headless=new',
    `--remote-debugging-port=${port}`,
    '--use-angle=swiftshader',
    '--enable-unsafe-swiftshader',
    `--window-size=${size},${size}`,
    '--user-data-dir=/tmp/anemo/cover-profile',
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
  if (m.method === 'Runtime.consoleAPICalled' && ['error', 'warning'].includes(m.params.type))
    console.log(
      `[console.${m.params.type}]`,
      m.params.args
        .map((a) => a.value ?? a.description)
        .join(' ')
        .slice(0, 400),
    );
  if (m.method === 'Runtime.exceptionThrown')
    console.log('[exception]', m.params.exceptionDetails.exception?.description?.slice(0, 600));
};
const send = (method, params = {}) =>
  new Promise((r) => {
    const i = ++id;
    pending.set(i, r);
    ws.send(JSON.stringify({ id: i, method, params }));
  });
await send('Runtime.enable');
await send('Page.enable');
await send('Emulation.setDeviceMetricsOverride', {
  width: size,
  height: size,
  deviceScaleFactor: scale,
  mobile: false,
});
await send('Page.navigate', { url });
let ready = false;
for (let i = 0; i < 240 && !ready; i++) {
  await sleep(500);
  const r = await send('Runtime.evaluate', {
    expression: 'window.__coverReady === true',
    returnByValue: true,
  });
  ready = r?.result?.value === true;
}
if (!ready) console.log('the scene never reported ready; capturing anyway');
const shot = await send('Page.captureScreenshot', { format: 'png' });
writeFileSync(out, Buffer.from(shot.data, 'base64'));
console.log('saved', out);
ws.close();
proc.kill();
process.exit(0);
