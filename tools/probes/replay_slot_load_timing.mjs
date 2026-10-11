/**
 * replay_slot_load_timing.mjs — wall time of one replay slot load per topic filter, in a real browser.
 *
 * Answers "what does a far-tier slot cost?" (Phase 6, 2026-10-11): the filtered lite load
 * (`source.load(i, {lite: true, topics})`) is timed for `/balls` only, `/mocap_data` only, both, and `/balls` again,
 * on four DIFFERENT never-resident slots per row (a repeat on one slot would hit the source memo).
 * Each load reports {ms, balls_n, mocap_n} (messages returned).  Findings 2026-10-11, busy bag, slots 10..83: single
 * loads range ~20-560 ms with NO consistent dependence on the topic filter or on how many messages come back (a
 * /balls-only load with 1018 balls took 73 ms; empty /balls-only loads took 21-386 ms).  CAUSE (measured later the same
 * day, replay_lane_trace.mjs): not the worker lane - the main thread was blocked in WebGL bufferSubData by the
 * ball-history layer's whole-buffer re-uploads (software GL), so replies sat undelivered; these figures are that delay.
 * The layer is append-only since; this probe's wall times are only meaningful while no long-tail fill is running.
 *
 * Needs: a GUI server and a headless Chromium with CDP, e.g. from the repo root
 *   /usr/bin/python3 ros_ws/gui/gui_server.py --port 8099 --rosbags-dir ~/Desktop/rosbags --overview-dir $PWD/temp/replay_overview &
 *   /snap/bin/chromium --headless=new --remote-debugging-port=9333 --user-data-dir=<scratch dir> &
 * Run (Node >= 22, built-in WebSocket):
 *   node tools/probes/replay_slot_load_timing.mjs [recording-id] [gui-port] [cdp-port] [slot,slot,...]
 * Defaults: 2026-10-02_12-43-28 (the busy 18 min bag), 8099, 9333, 60,70,80.
 * Output: one JSON line to stdout and temp/probes/replay_slot_load_timing_<iso>.json.
 * Headless renders with software GL: trust these load times, never frame times.  Stop the server/browser BY PID.
 */
import fs from 'fs';

const BAG = process.argv[2] || '2026-10-02_12-43-28';
const GUI = +(process.argv[3] || 8099);
const CDP = +(process.argv[4] || 9333);
const SLOTS = (process.argv[5] || '60,70,80').split(',').map(Number);
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

const tab = await (await fetch(`http://localhost:${CDP}/json/new?about:blank`, { method: 'PUT' })).json();
const ws = new WebSocket(tab.webSocketDebuggerUrl);
await new Promise((r) => { ws.onopen = r; });
let id = 0; const pend = new Map();
ws.onmessage = (e) => {
  const m = JSON.parse(e.data);
  if (m.id && pend.has(m.id)) { pend.get(m.id)(m); pend.delete(m.id); }
  if (m.method === 'Page.javascriptDialogOpening') ws.send(JSON.stringify({ id: 1e6 + (++id), method: 'Page.handleJavaScriptDialog', params: { accept: false } }));
};
const send = (method, params = {}) => new Promise((r) => { const i = ++id; pend.set(i, r); ws.send(JSON.stringify({ id: i, method, params })); });
const ev = async (expr) => {
  const r = await send('Runtime.evaluate', { expression: expr, awaitPromise: true, returnByValue: true });
  if (r.result.exceptionDetails) throw new Error(r.result.exceptionDetails.exception?.description || r.result.exceptionDetails.text);
  return r.result.result.value;
};
for (const d of ['Runtime', 'Page']) await send(d + '.enable');
// the Phase 6 deep link opens the recording paused at its start
await send('Page.navigate', { url: `http://localhost:${GUI}/index.html?recording=${BAG}&t=0` });
const ui = `import('/js/replay/ui/index.js').then(m => m.getReplayUi())`;
let ready = false;
for (let i = 0; i < 600 && !ready; i++) {
  await sleep(100);
  ready = await ev(`${ui}.then(u => !!(u.mode.engine() && u.mode.engine().state().mode === 'paused'))`).catch(() => false);
}
if (!ready) { console.error('recording never opened'); process.exit(2); }
await sleep(4000);   // let the opening pre-roll settle off the lane
const rows = await ev(`${ui}.then(async u => {
  const s = u.mode.source(); const out = [];
  const T = async (i, topics) => { const a = performance.now(); const ch = await s.load(i, { lite: true, topics });
    const n = (k) => (ch && ch.topics && ch.topics[k] ? ch.topics[k].n : 0);
    return { slot: i, ms: Math.round(performance.now() - a), balls_n: n('/balls'), mocap_n: n('/mocap_data') }; };
  for (const i of ${JSON.stringify(SLOTS)}) out.push({ i,
    balls: await T(i, ['/balls']), mocap: await T(i + 1, ['/mocap_data']),
    both: await T(i + 2, ['/balls', '/mocap_data']), balls_again: await T(i + 3, ['/balls']) });
  return out; })`);
const res = { probe: 'replay_slot_load_timing', recording: BAG, at: new Date().toISOString(), rows };
console.log(JSON.stringify(res));
fs.mkdirSync('temp/probes', { recursive: true });
fs.writeFileSync(`temp/probes/replay_slot_load_timing_${res.at.replace(/[:.]/g, '-')}.json`, JSON.stringify(res, null, 2));
await send('Target.closeTarget', { targetId: tab.id }).catch(() => {});
process.exit(0);
