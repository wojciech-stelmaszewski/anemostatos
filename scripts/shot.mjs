// Dev helper: open the app in headless Chrome, print console errors, save screenshots.
// Usage: node scripts/shot.mjs [url] [outPrefix] [waitMs...]   (Chrome must be installed)
import { spawn } from 'node:child_process';
import { writeFileSync } from 'node:fs';

const url = process.argv[2] ?? 'http://localhost:5173/';
const out = process.argv[3] ?? '/tmp/anemo/shot';
const waits = (process.argv.slice(4).length ? process.argv.slice(4) : ['4000']).map(Number);
const actions = JSON.parse(process.env.ACTIONS ?? '[]'); // [{at: index, js: "..."}]
const chrome = '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome';
const port = 9333;
const proc = spawn(
  chrome,
  [
    '--headless=new',
    `--remote-debugging-port=${port}`,
    '--use-angle=swiftshader',
    '--enable-unsafe-swiftshader',
    '--window-size=1600,1000',
    '--user-data-dir=/tmp/anemo/profile',
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
  width: 1600,
  height: 1000,
  deviceScaleFactor: 1,
  mobile: false,
});
await send('Page.navigate', { url });
for (let k = 0; k < waits.length; k++) {
  await sleep(waits[k]);
  for (const a of actions.filter((a) => a.at === k)) {
    const r = await send('Runtime.evaluate', { expression: `(async () => { ${a.js} })()`, awaitPromise: true, returnByValue: true });
    if (r?.exceptionDetails) console.log('[eval error]', r.exceptionDetails.exception?.description);
    if (r?.result?.value !== undefined) console.log('[eval]', JSON.stringify(r.result.value));
  }
  const shot = await send('Page.captureScreenshot', { format: 'png' });
  writeFileSync(`${out}${k}.png`, Buffer.from(shot.data, 'base64'));
  console.log('saved', `${out}${k}.png`);
}
ws.close();
proc.kill();
process.exit(0);
