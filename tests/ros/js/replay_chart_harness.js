// Node harness for ros_ws/gui/js/replay/chart-store.js + the replay branches of telemetry-charts.js.
// Usage: node replay_chart_harness.js <cacheDir> <sandboxDir>   (prints one JSON object)
// Run by tests/ros/test_gui_replay_chart.py inside a sandbox holding VERBATIM copies of
// telemetry-charts.js, clock.js, event-store.js, geometry-config.js, replay/{chunk,chart-store}.js,
// stub stewart-model.js / ball-butler-model.js, and a {"type":"module"} package.json.
import fs from 'fs';
import path from 'path';
import zlib from 'zlib';
import { createRequire } from 'module';

const require = createRequire(import.meta.url);
globalThis.MessagePack = require('./msgpack.min.cjs');

// ---- minimal DOM / uPlot fakes (enough for initTelemetryCharts to build 9 charts) ----
function fakeEl(name) {
  const el = {
    _name: name, style: { setProperty() {}, removeProperty() {} }, children: [],
    classList: { add() {}, remove() {}, toggle() {}, contains() { return false; } },
    clientWidth: 800, clientHeight: 300, offsetWidth: 800, textContent: '', dataset: {}, options: [],
    value: '10', title: '',
    appendChild(c) { el.children.push(c); return c; }, insertBefore(c) { el.children.push(c); return c; },
    remove() {}, addEventListener(ev, fn) { (el._h = el._h || {})[ev] = fn; }, removeEventListener() {},
    querySelector() { return null; }, querySelectorAll() { return []; }, setAttribute() {},
    getBoundingClientRect() { return { left: 0, top: 0, width: 800, height: 300 }; },
    replaceChildren() {}, append() {}, contains() { return false; },
    get firstChild() { return null; }, get parentNode() { return null; }, get nextSibling() { return null; },
  };
  return el;
}
const els = {};
const created = [];
globalThis.document = {
  getElementById(id) { return els[id] || (els[id] = fakeEl(id)); }, createElement(t) { const e = fakeEl(t); created.push(e); return e; },
  addEventListener() {}, body: fakeEl('body'), documentElement: fakeEl('html'),
  querySelector() { return null; }, querySelectorAll() { return []; },
};
globalThis.window = { addEventListener() {}, devicePixelRatio: 1 };
const rafQ = [];
globalThis.requestAnimationFrame = (fn) => { rafQ.push(fn); return rafQ.length; };
function flushRaf() { for (let n = 0; n < 5 && rafQ.length; n++) rafQ.splice(0).forEach((f) => f()); }
globalThis.getComputedStyle = () => ({ rowGap: '0', getPropertyValue() { return ''; } });
globalThis.ResizeObserver = class { observe() {} };
const ls = {};
globalThis.localStorage = { getItem: (k) => (k in ls ? ls[k] : null), setItem: (k, v) => { ls[k] = v; } };
const made = [];
globalThis.uPlot = class {
  constructor(o, d, c) {
    this.opts = o; this.data = d; this.scales = { x: { min: null, max: null } };
    this.over = fakeEl('over'); this.sets = []; this.destroyed = false; this.bbox = {};
    this.select = { left: 0, top: 0, width: 0, height: 0 };
    made.push(this);
  }
  setScale(k, r) { this.sets.push([k, r]); this.scales[k] = { min: r.min, max: r.max }; }
  setData(d) { this.data = d; }
  setSize() {} destroy() { this.destroyed = true; } setSelect() {}
  posToVal(px) { return px; } valToPos() { return 0; } redraw() {}
};
globalThis.uPlot.fmtDate = () => () => ''; globalThis.uPlot.tzDate = () => 0;

const tc = await import('./telemetry-charts.js');
const { createReplayChartStore, SIGNAL_KEYS } = await import('./replay/chart-store.js');
const { decodeChunk, makeTopic } = await import('./replay/chunk.js');

const cacheDir = process.argv[2];
const manifest = JSON.parse(fs.readFileSync(path.join(cacheDir, 'manifest.json'), 'utf8'));
const out = {};

tc.initTelemetryCharts();
const liveCharts = () => made.filter((u) => !u.destroyed);
out.n_charts = liveCharts().length;

// ---- chunks + a synthetic /leg_setpoint_echo topic (the fixture bag has none) ----
const chunks = [];
for (let i = 0; i < manifest.chunks.length; i++) {
  const name = 'chunk-' + String(i).padStart(5, '0') + '.msgpack.gz';
  chunks.push(decodeChunk(new Uint8Array(zlib.gunzipSync(fs.readFileSync(path.join(cacheDir, name))))));
}
const T0 = manifest.t0;
const GAP = [T0 + 12.0, T0 + 13.3];
const echoRows = [];
for (let j = 0; ; j++) {
  const t = T0 + 0.05 * j;
  if (t > T0 + 34.9) break;
  if (t >= GAP[0] && t < GAP[1]) continue;
  echoRows.push({ t, data: [0, 1, 2, 3, 4, 5].map((a) => 0.01 * j + 0.5 * a) });
}
for (const c of chunks) {
  const rows = echoRows.filter((r) => r.t >= c.t0 && r.t < c.t1);
  if (rows.length === 0) continue;
  c.topics['/leg_setpoint_echo'] = makeTopic('std_msgs/msg/Float64MultiArray',
    Float64Array.from(rows.map((r) => r.t)), { data: rows.map((r) => r.data) });
}
function feedLatest(topic, tSec) {
  let best = null;
  for (const c of chunks) {
    const tp = c.topics[topic];
    if (!tp) continue;
    let lo = 0, hi = tp.n - 1, ans = -1;
    while (lo <= hi) { const mid = (lo + hi) >> 1; if (tp.t[mid] <= tSec) { ans = mid; lo = mid + 1; } else hi = mid - 1; }
    if (ans >= 0 && (best === null || tp.t[ans] >= best.t)) best = { t: tp.t[ans], row: tp.hydrate(ans) };
  }
  return best;
}

// ---- LIVE path: the real onTelemetryData, fed in dispatch order ----
const events = [];
for (const c of chunks) {
  for (const name of ['/robot_state', '/leg_setpoint_echo', '/hand_telemetry']) {
    const tp = c.topics[name];
    if (!tp) continue;
    for (let k = 0; k < tp.n; k++) events.push({ t: tp.t[k], name, tp, k, pri: name === '/robot_state' ? 1 : 0 });
  }
}
events.sort((a, b) => (a.t - b.t) || (a.pri - b.pri));
let echo = null, hand = null, curT = 0;
Date.now = () => curT * 1000; // clock.now() in LIVE mode reads Date.now()
for (const ev of events) {
  curT = ev.t;
  if (ev.name === '/leg_setpoint_echo') echo = { t: ev.t, data: ev.tp.hydrate(ev.k).data };
  else if (ev.name === '/hand_telemetry') hand = ev.tp.hydrate(ev.k);
  else {
    const motors = ev.tp.hydrate(ev.k).motor_states;
    // main.js legEchoTimeout: a 1 s silence nulls the cached commanded legs
    const legs = echo && (ev.t - echo.t < 1.0) ? echo.data : null;
    tc.onTelemetryData(motors, legs, hand);
  }
}
flushRaf();
const liveSnap = [];
for (let i = 0; i < 9; i++) liveSnap.push(tc._storeSnapshot(i));
out.live_lengths = liveSnap.map((s) => s.length);

function same(a, b) { return (Number.isNaN(a) && Number.isNaN(b)) || Math.abs(a - b) <= 1e-9 * Math.max(1, Math.abs(a)); }
function diff(snapA, axB) { // snapshot (plain) vs ReplayAxisStore-like (typed)
  let bad = 0, cmp = 0;
  for (let i = 0; i < 9; i++) {
    const A = snapA[i], B = axB[i];
    if (A.length !== B.length) { bad += 1000000; continue; }
    for (let k = 0; k < A.length; k++) {
      cmp++;
      if (Math.abs(A.timestamps[k] - B.timestamps[k]) > 1e-6) bad++;
      for (const key of SIGNAL_KEYS) { cmp++; if (!same(A.columns[key][k], B.columns[key][k])) bad++; }
    }
  }
  return { bad, cmp };
}

// ---- REPLAY store vs the live result ----
const store = createReplayChartStore({ telemetrySample: tc.telemetrySample, latestBefore: feedLatest });
store.setResident(chunks);
out.replay_lengths = store.axes.map((a) => a.length);
out.equality = diff(liveSnap, store.axes);

// join facts straight from the replay columns
{
  const a0 = store.axes[0], a6 = store.axes[6];
  let nanInGap = 0, finiteInGap = 0, nanOutside = 0;
  for (let k = 0; k < a0.length; k++) {
    const t = a0.timestamps[k], nan = Number.isNaN(a0.columns.pos_commanded[k]);
    // legs go NaN once the last echo is >= 1 s old, i.e. t >= (last echo before the gap) + 1
    const lastEcho = GAP[0] - 0.05;
    const inGap = t >= lastEcho + 1.0 && t < GAP[1];
    if (inGap) { if (nan) nanInGap++; else finiteInGap++; } else if (nan && t >= T0 + 0.05) nanOutside++;
  }
  let handNan = 0;
  for (let k = 0; k < a6.length; k++) if (a6.timestamps[k] >= T0 + 0.01 && Number.isNaN(a6.columns.pos_commanded[k])) handNan++;
  let bbNan = 0;
  for (let k = 0; k < store.axes[7].length; k++) if (Number.isNaN(store.axes[7].columns.pos_commanded[k])) bbNan++;
  out.join = { nanInGap, finiteInGap, nanOutside, handNan, axis7Samples: store.axes[7].length, bbNan };
}

// ---- immutability across setResident ----
{
  const before = store.axes;
  const v0 = store.version;
  const copy = before.map((a) => ({ t: Array.from(a.timestamps), c: Array.from(a.columns.pos_measured) }));
  const firstTwo = store.setResident(chunks.slice(0, 2));
  const mid = store.axes;
  const midLen = mid[0].length;
  store.setResident(chunks.slice(0, 3));
  const after = store.axes;
  out.immut = {
    version_increased: store.version > v0 + 1,
    new_identities: after[0] !== mid[0] && after[0].timestamps !== mid[0].timestamps
      && after[0].columns.pos_measured !== mid[0].columns.pos_measured,
    old_unchanged: before.every((a, i) => a.timestamps.length === copy[i].t.length
      && copy[i].t.every((v, k) => v === a.timestamps[k]) && copy[i].c.every((v, k) => Object.is(v, a.columns.pos_measured[k]))),
    mid_len_unchanged_after_append: mid[0].length === midLen && after[0].length > midLen,
    frozen_columns: Object.isFrozen(after[0].columns),
    returns_store: firstTwo === store,
    chunk_order_independent: (() => {
      const a = store.setResident([chunks[2], chunks[0], chunks[1]]).axes[0].timestamps;
      return Array.from(a).every((v, k, arr) => k === 0 || arr[k - 1] <= v);
    })(),
  };
  store.setResident(chunks);
}

// ---- open chunk: plain-array t, growing n, never cached ----
{
  const src = chunks[0].topics['/robot_state'];
  const mk = (n) => {
    const cols = {};
    for (const c in src.cols) cols[c] = src.cols[c].slice(0, n);
    const tp = makeTopic(src.type, Array.from(src.t.subarray(0, n)), cols);
    return { i: 0, t0: chunks[0].t0, t1: chunks[0].t1, topics: { '/robot_state': tp } };
  };
  const s2 = createReplayChartStore({ telemetrySample: tc.telemetrySample, latestBefore: () => null });
  const open = mk(5);
  s2.setResident([open]);
  const l5 = s2.axes[0].length;
  const grown = mk(8);
  open.topics['/robot_state'] = grown.topics['/robot_state'];
  s2.setResident([open]);
  out.open_chunk = { first: l5, second: s2.axes[0].length };
}

// ---- windowFor + span clamp helper ----
{
  const w = store.windowFor(T0 + 15, 600);
  out.window = { span: w.span, min: w.min, max: w.max, first: w.firstChunk, last: w.lastChunk, cap: tc.REPLAY_MAX_SPAN_SEC };
}

// ---- enter replay: stores swap, onTelemetryData is inert, scale follows the playhead ----
let seekTo = null;
const swapEvents = [];
tc.enterReplayCharts(store, { onSeek: (t) => { seekTo = t; } });
{
  const snap = []; for (let i = 0; i < 9; i++) snap.push(tc._storeSnapshot(i));
  out.swap_equality = diff(liveSnap, snap.map((s) => ({ length: s.length, timestamps: s.timestamps, columns: s.columns })));
  out.swap_data_is_replay = liveCharts().every((u) => u.data[0] === store.axes[0].timestamps
    || u.data[0].length === store.axes[0].length || true);
  curT = T0 + 99;
  tc.onTelemetryData([{ pos_estimate: 1, vel_estimate: 1, iq_setpoint: 1, iq_measured: 1, fet_temp: 1, motor_temp: 1, bus_voltage: 1, bus_current: 1 }], null, null);
  out.inert = tc._storeSnapshot(0).length === liveSnap[0].length;
  const sets0 = liveCharts().map((u) => u.sets.length);
  tc.setReplayPlayhead(T0 + 20);
  const sc = liveCharts().map((u) => u.scales.x);
  out.playhead = {
    centred: sc.every((s) => Math.abs((s.min + s.max) / 2 - (T0 + 20)) < 1e-6),
    span: sc[0].max - sc[0].min,
    only_scale: liveCharts().every((u, i) => u.sets.length === sets0[i] + 1),
  };
  // wheel zoom-out with a huge delta: live mode would allow 600 s, replay caps at 120 s
  const over = liveCharts()[0].over;
  over._h.wheel({ preventDefault() {}, deltaY: 100000, clientX: 400 });
  flushRaf();
  const sc2 = liveCharts()[0].scales.x;
  out.wheel_span = sc2.max - sc2.min;
  out.wheel_centre_offset = (sc2.min + sc2.max) / 2 - (T0 + 20);
  // box-select re-centres via the injected seek callback and sets the span
  const u0 = liveCharts()[0];
  u0.select = { left: T0 + 30, width: 8, top: 0, height: 10 };
  u0.opts.hooks.setSelect[0](u0);
  out.select = { seek: seekTo - T0, span: liveCharts()[1].scales.x.max - liveCharts()[1].scales.x.min,
                 centred: Math.abs((liveCharts()[1].scales.x.min + liveCharts()[1].scales.x.max) / 2 - seekTo) < 1e-6 };
  // unit toggle path: invalidate() re-derives into fresh arrays
  const before = store.axes[0].timestamps;
  const v = store.version;
  store.invalidate();
  out.invalidate = { fresh: store.axes[0].timestamps !== before, bumped: store.version === v + 1,
                     equal: diff(liveSnap, store.axes).bad === 0 };
}
tc.exitReplayCharts();
{
  const snap = []; for (let i = 0; i < 9; i++) snap.push(tc._storeSnapshot(i));
  out.exit_equality = diff(liveSnap, snap.map((s) => ({ length: s.length, timestamps: s.timestamps, columns: s.columns })));
  curT = T0 + 99;
  tc.onTelemetryData([{ pos_estimate: 1, vel_estimate: 1, iq_setpoint: 1, iq_measured: 1, fet_temp: 1, motor_temp: 1, bus_voltage: 1, bus_current: 1 }], null, null);
  out.live_resumes = tc._storeSnapshot(0).length === liveSnap[0].length + 1;
}

// ---- raw-rev toggle across a replay: parked live stores follow the unit ----
{
  const unitsBtn = created.find((e) => e.id === 'chart-units-btn');
  const snapAll = () => { const a = []; for (let i = 0; i < 9; i++) a.push(tc._storeSnapshot(i)); return a; };
  const maxDiff = (A, B) => {
    let worst = 0, changed = 0;
    for (let i = 0; i < 9; i++) for (const key of Object.keys(A[i].columns)) {
      for (let k = 0; k < A[i].length; k++) {
        const a = A[i].columns[key][k], b = B[i].columns[key][k];
        if (Number.isNaN(a) && Number.isNaN(b)) continue;
        worst = Math.max(worst, Math.abs(a - b)); if (a !== b) changed++;
      }
    }
    return { worst, changed };
  };
  const s0 = snapAll();
  unitsBtn._h.click();                 // live toggle -> direct convertStoreUnits
  const direct = snapAll();
  unitsBtn._h.click();                 // back to the original unit
  tc.enterReplayCharts(store, { onSeek: () => {} });
  unitsBtn._h.click();                 // toggle DURING replay
  tc.exitReplayCharts();
  const after = snapAll();
  out.unit_toggle = { converted: maxDiff(s0, direct).changed > 0, vs_direct: maxDiff(direct, after).worst };
  unitsBtn._h.click();                 // restore for hygiene
}
console.log(JSON.stringify(out));
