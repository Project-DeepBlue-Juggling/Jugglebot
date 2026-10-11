/**
 * replay_lane_trace.mjs — where does a long-tail far-tier fill spend its time?  Times every worker request in a real
 * browser (post -> reply on the main thread, against the worker's own `ms`), the WebGL bufferSubData calls, and how
 * long the ball-history layer takes to reach its final slot count.
 *
 * Ticket 09 (2026-10-11), busy bag 2026-10-02_12-43-28, tail = whole recording, playhead at the end, Jetson headless:
 *   - queue waits on both worker lanes <= 30 ms; a filtered (/balls + /mocap_data) load is ~150-190 ms of worker
 *     time, a full lite load ~330 ms; the worker was busy ~21 s of a ~240 s fill -> NOT lane contention.
 *   - a CPU profile put 122 s of 136 s in bufferSubData: ball-history-layer.js re-uploaded its whole buffer on every
 *     arrival (Three uploads the whole CAPACITY when no update range is set), 1.5 GB per fill.  Software GL made
 *     that block the main thread, so worker replies sat undelivered (the 20-560 ms "load" times of
 *     replay_slot_load_timing.mjs were that delay, not load cost).
 *   - with uploads skipped (`skipgl`): 109 slots in 27.4 s; after the layer went append-only: 26.1 s with uploads,
 *     bufferSubData 20 ms in total.
 * Software GL exaggerates upload cost; on a real GPU the old re-uploads were cheaper but still O(slots^2).
 *
 * Needs a GUI server and a headless Chromium with CDP (recipe in replay_slot_load_timing.mjs).  Run (Node >= 22):
 *   node tools/probes/replay_lane_trace.mjs [recording-id] [seconds] [tail|notail] [t] [skipgl]
 * e.g. `node tools/probes/replay_lane_trace.mjs 2026-10-02_12-43-28 120 tail 99999`.  Writes the raw trace to
 * temp/probes/replay_lane_trace_<iso>.json.  The software-rendered page keeps the GPU process near 300 % CPU while
 * it runs: stop the server and browser BY PID afterwards.
 */
import fs from 'fs';
const BAG = process.argv[2] || '2026-10-02_12-43-28', GUI = 8099, CDP = 9333, DUR = +(process.argv[3] || 90);
const TAIL = process.argv[4] !== 'notail'; const SKIPGL = process.argv[6] === 'skipgl';
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const tab = await (await fetch(`http://localhost:${CDP}/json/new?about:blank`, { method: 'PUT' })).json();
const ws = new WebSocket(tab.webSocketDebuggerUrl); await new Promise((r) => { ws.onopen = r; });
let id = 0; const pend = new Map();
ws.onmessage = (e) => { const m = JSON.parse(e.data); if (m.id && pend.has(m.id)) { pend.get(m.id)(m); pend.delete(m.id); } };
const send = (method, params = {}) => new Promise((r) => { const i = ++id; pend.set(i, r); ws.send(JSON.stringify({ id: i, method, params })); });
const ev = async (expr) => { const r = await send('Runtime.evaluate', { expression: expr, awaitPromise: true, returnByValue: true }); if (r.result.exceptionDetails) throw new Error(r.result.exceptionDetails.exception?.description || r.result.exceptionDetails.text); return r.result.result.value; };
for (const d of ['Runtime', 'Page']) await send(d + '.enable');
await send('Page.bringToFront'); await send('Emulation.setFocusEmulationEnabled', { enabled: true });
await send('Page.addScriptToEvaluateOnNewDocument', { source: `const SKIPGL=${SKIPGL};
  window.__lt = []; const W = window.Worker;
  window.Worker = function (...a) { const w = new W(...a); const sent = new Map(); const wid = window.__lt.length;
    const pm = w.postMessage.bind(w);
    w.postMessage = (m, t) => { if (m && m.req != null) sent.set(m.req, { op: m.op, i: m.i, lite: !!m.lite, topics: m.topics ? m.topics.join(',') : '', at: performance.now(), q: sent.size }); return pm(m, t); };
    w.addEventListener('message', (e) => { const d = e.data; const s = d && sent.get(d.req); if (!s) return; sent.delete(d.req);
      window.__lt.push({ ...s, wall: performance.now() - s.at, wms: d.ms, bytes: d.bytes, rep: d.op, end: performance.now() }); });
    return w; };
  window.Worker.prototype = W.prototype;
  window.__gl = { n: 0, bytes: 0, ms: 0, skip: SKIPGL };
  for (const P of [WebGLRenderingContext.prototype, WebGL2RenderingContext.prototype]) { const o = P.bufferSubData;
    P.bufferSubData = function (...a) { const v = a[2]; window.__gl.n++; window.__gl.bytes += (a.length > 4 && v) ? a[4] * v.BYTES_PER_ELEMENT : ((v && v.byteLength) || 0);
      if (window.__gl.skip) return; const t = performance.now(); const r = o.apply(this, a); window.__gl.ms += performance.now() - t; return r; }; }` });
await send('Page.navigate', { url: `http://localhost:${GUI}/index.html?recording=${BAG}&t=${process.argv[5] || 0}` });
const ui = `import('/js/replay/ui/index.js').then(m => m.getReplayUi())`;
let ready = false;
for (let i = 0; i < 600 && !ready; i++) { await sleep(100); ready = await ev(`${ui}.then(u => !!(u.mode.engine() && u.mode.engine().state().mode === 'paused'))`).catch(() => false); }
if (!ready) { console.error('never opened'); process.exit(2); }
if (TAIL) await ev(`import('/js/trail-settings.js').then(m=>m.setTailMs(100000000))`);
const tS = Date.now(); let last = null, full = null;
while (Date.now() - tS < DUR * 1000) { await sleep(1000);
  const st = await ev(`import('/js/trails-scene.js').then(m=>m.getBallHistoryLayer().stats().slots)`);
  if (st !== last) { last = st; full = Date.now() - tS; } }
console.log('slots', last, 'last change at s', (full / 1e3).toFixed(1), 'gl', JSON.stringify(await ev('window.__gl')));
const tr = await ev('window.__lt');
fs.mkdirSync('temp/probes', { recursive: true }); fs.writeFileSync(`temp/probes/replay_lane_trace_${new Date().toISOString().replace(/[:.]/g, '-')}.json`, JSON.stringify(tr));
const grp = {};
for (const r of tr) { if (r.op !== 'load') continue; const k = r.lite ? (r.topics ? 'filtered' : 'lite') : 'normal';
  (grp[k] ||= []).push(r); }
const pct = (a, p) => a.length ? Math.round(a.slice().sort((x, y) => x - y)[Math.min(a.length - 1, Math.floor(p * a.length))]) : null;
for (const [k, a] of Object.entries(grp)) {
  const svc = a.map((r) => r.wms), wait = a.map((r) => r.wall - r.wms);
  console.log(k, 'n', a.length, 'svc p50/p90/max', pct(svc, .5), pct(svc, .9), pct(svc, 1 - 1e-9), 'wait p50/p90/max', pct(wait, .5), pct(wait, .9), pct(wait, 1 - 1e-9), 'MB', (a.reduce((s, r) => s + (r.bytes || 0), 0) / 1e6).toFixed(1));
}
await send('Target.closeTarget', { targetId: tab.id }).catch(() => {}); process.exit(0);
