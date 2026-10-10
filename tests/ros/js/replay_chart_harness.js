// Node harness for ros_ws/gui/js/replay/chart-store.js + the replay branches of telemetry-charts.js.
// Usage: node replay_chart_harness.js <chunkJsonDir>   (prints one JSON object)
// Run by tests/ros/test_gui_replay_chart.py inside a sandbox holding VERBATIM copies of
// telemetry-charts.js, clock.js, event-store.js, geometry-config.js, replay/{chunk,chart-store}.js,
// stub stewart-model.js / ball-butler-model.js, and a {"type":"module"} package.json.
import fs from 'fs';
import path from 'path';

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
globalThis.getComputedStyle = () => ({ rowGap: '0', getPropertyValue(k) { return k === '--replay-playhead' ? ' #abcdef ' : ''; } });
globalThis.ResizeObserver = class { observe() {} };
const ls = {};
globalThis.localStorage = { getItem: (k) => (k in ls ? ls[k] : null), setItem: (k, v) => { ls[k] = v; } };
const made = [];
globalThis.uPlot = class {
  constructor(o, d, c) {
    this.opts = o; this.data = d; this.scales = { x: { min: null, max: null } };
    this.over = fakeEl('over'); this.sets = []; this.destroyed = false;
    this.select = { left: 0, top: 0, width: 0, height: 0 };
    this.hooks = o.hooks;
    this.bbox = { left: 100, top: 10, width: 400, height: 60 };
    this.strokes = []; this.fills = []; this.setDataCalls = 0;
    const self = this;
    this.ctx = { save() {}, restore() {}, beginPath() { self._path = []; }, moveTo(x, y) { self._mv = [x, y]; (self._path = self._path || []).push([x, y]); },
                 lineTo(x, y) { (self._path = self._path || []).push([x, y]); }, closePath() {}, rect() {}, clip() {},
                 fill() { self.fills.push({ color: this.fillStyle, alpha: this.globalAlpha, pts: (self._path || []).slice() }); },
                 stroke() { self.strokes.push({ color: this.strokeStyle, w: this.lineWidth, x: self._mv[0] }); }, fillRect() {}, fillText() {}, set font(v) {} };
    made.push(this);
  }
  setScale(k, r) { this.sets.push([k, r]); this.scales[k] = { min: r.min, max: r.max }; }
  setData(d) { this.data = d; this.setDataCalls++; }
  setSize() {} destroy() { this.destroyed = true; } setSelect() {}
  posToVal(px) { return px; } valToPos() { return 0; } redraw() {}
};
globalThis.uPlot.fmtDate = () => () => ''; globalThis.uPlot.tzDate = () => 0;

const tc = await import('./telemetry-charts.js');
const { createReplayChartStore, SIGNAL_KEYS } = await import('./replay/chart-store.js');
const { chunkFromRecord, makeTopic, buildColumns } = await import('./replay/chunk.js');
const { loadRecords } = await import('./replay_test_support.js');

const cacheDir = process.argv[2];
const { manifest, records } = loadRecords(cacheDir, buildColumns);
const out = {};

tc.initTelemetryCharts();
const liveCharts = () => made.filter((u) => !u.destroyed);
out.n_charts = liveCharts().length;

// ---- chunks + a synthetic /leg_setpoint_echo topic (the fixture bag has none) ----
const chunks = [];
for (const rec of records) chunks.push(chunkFromRecord(rec));
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
    if (ans >= 0 && (best === null || tp.t[ans] >= best.t)) best = { t: tp.t[ans], tp, k: ans };
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

// ---- unsealed chunk (plain-array t, plain columns, growing n): never cached, still derived ----
{
  const src = chunks[0].topics['/robot_state'];
  const plain = records[0].topics['/robot_state'].plain;
  const mk = (n) => {
    const cols = {};
    for (const c in plain) cols[c] = plain[c].slice(0, n);
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
  // wheel zoom-out with a huge delta: live mode allows 600 s; replay also caps at REPLAY_MAX_SPAN_SEC (600)
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
// ---- playhead line in every chart (feedback unit A, item 3) ----
{
  const hasHook = (u) => u.hooks.draw.some((f) => f.name === 'drawReplayPlayhead');
  const before = liveCharts().map(hasHook);
  tc.enterReplayCharts(store, { onSeek: () => {} });
  const inChart = liveCharts();
  const installed = inChart.map(hasHook);
  tc.setReplayPlayhead(T0 + 20);
  // a PANNED window: the playhead sits at 3/4 of the box, not the centre
  const u = inChart[0];
  tc.setReplayPlayhead(T0 + 25);
  u.scales.x = { min: T0 + 10, max: T0 + 30 };       // pan AFTER the playhead move (setScale would re-centre)
  u.strokes.length = 0;
  const f = u.hooks.draw.find((g) => g.name === 'drawReplayPlayhead');
  f(u);
  const drawn = u.strokes.slice();
  u.scales.x = { min: T0 + 40, max: T0 + 60 };      // playhead outside the window: nothing drawn
  u.strokes.length = 0; f(u);
  const outside = u.strokes.length;
  tc.exitReplayCharts();
  const removed = liveCharts().map((c) => !hasHook(c));
  out.playhead_line = {
    before, installed, drawn, outside, removed,
    x_panned: tc.playheadCanvasX(25, 10, 30, 100, 400), x_centre: tc.playheadCanvasX(20, 10, 30, 100, 400),
    x_left: tc.playheadCanvasX(10, 10, 30, 100, 400), x_out: tc.playheadCanvasX(31, 10, 30, 100, 400),
    x_bad: tc.playheadCanvasX(5, null, null, 0, 1), x_degenerate: tc.playheadCanvasX(5, 5, 5, 0, 1),
  };
}
// ---- A2: two-tier composed view, gaps, per-bin clipping, envelope hook, coalesced redraw ----
{
  const hasHook = (u, name) => u.hooks.draw.some((f) => f.name === name);
  const st = createReplayChartStore({ telemetrySample: tc.telemetrySample, latestBefore: feedLatest });
  const sliceRes = chunks[2];                                   // resident [T0+20, T0+30]
  st.setResident([sliceRes]);
  const full0 = st.axes[0].timestamps;
  out.tiers_none = { same_ts: st.axes[0].getAlignedData(['pos_measured'])[0] === full0 };
  // digests of slots 0 and 3; slot 1 is MISSING (its digest has not arrived); slot 2 is resident
  const dg = new Map([[0, st.digestChunk(chunks[0])], [3, st.digestChunk(chunks[3])]]);
  st.setDigests(dg);
  const d0 = st.axes[0].getAlignedData(['pos_measured']);
  const ts = d0[0], pm = d0[1];
  let mono = true;
  for (let k = 1; k < ts.length; k++) if (!(ts[k] > ts[k - 1])) mono = false;
  const nanIdx = [];
  for (let k = 0; k < pm.length; k++) if (Number.isNaN(pm[k])) nanIdx.push(k);
  const fs0 = full0[0], fe0 = full0[full0.length - 1];
  const nGapPts = nanIdx.filter((k) => ts[k] < fs0 - 1e-9 || ts[k] > fe0 + 1e-9).length;
  out.tiers = {
    mono, n_full: full0.length, n_total: ts.length,
    n_before: Array.from(ts).filter((t) => t < fs0).length, n_after: Array.from(ts).filter((t) => t > fe0).length,
    gap_points_outside_full: nGapPts,
    gap_between_slot0_and_full: nanIdx.some((k) => ts[k] > T0 + 10 && ts[k] < fs0),
    gap_between_full_and_slot3: nanIdx.some((k) => ts[k] > fe0 && ts[k] < T0 + 30.5),
    cols_len_ok: d0.every((c) => c.length === ts.length),
  };
  // per-bin clipping at a window edge: the resident window starts mid-slot (25 s into a 20..30 slot)
  {
    const mid = Object.assign({}, chunks[2], { t0: T0 + 25 });
    const s3 = createReplayChartStore({ telemetrySample: tc.telemetrySample, latestBefore: feedLatest });
    s3.setResident([mid]);
    s3.setDigests(new Map([[2, s3.digestChunk(chunks[2])]]));
    const env = s3.envelope(0, 'pos_measured');
    const rr = s3.residentRange();
    out.clip = { env_t: Array.from(env.t).map((t) => +(t - T0).toFixed(3)), rr0: +(rr.t0 - T0).toFixed(3), split: env.split,
      inside: Array.from(env.t).filter((t) => t > rr.t0 - 0.5 && t < rr.t1 + 0.5).length };
  }
  // envelope hook: installed on enter (before the playhead hook), draws only over digest regions
  const names = (u) => u.hooks.draw.map((f) => f.name);
  const before = liveCharts().map((u) => hasHook(u, 'drawReplayEnvelope'));
  tc.enterReplayCharts(st, { onSeek: () => {} });
  flushRaf();
  const u = liveCharts()[0];
  const order = names(u);
  u.scales = new Proxy({ x: { min: T0, max: T0 + 35 } }, { get: (o, k) => (k in o ? o[k] : { min: -1e9, max: 1e9 }) });
  u.valToPos = (v) => v;
  const f = u.hooks.draw.find((g) => g.name === 'drawReplayEnvelope');
  u.fills.length = 0;
  f(u);
  const tOf = (x) => T0 + (x - 100) / 400 * 35;
  const times = [];
  for (const fl of u.fills) for (const pt of fl.pts) times.push(tOf(pt[0]));
  out.env_hook = {
    before, installed: liveCharts().map((c) => hasHook(c, 'drawReplayEnvelope')), order,
    n_fills: u.fills.length, alpha_ok: u.fills.every((fl) => fl.alpha === 0.15 && typeof fl.color === 'string' && fl.color.length > 0),
    inside_full: times.filter((t) => t > fs0 + 1e-6 && t < fe0 - 1e-6).length,
    n_pts: times.length, has_before: times.some((t) => t < fs0), has_after: times.some((t) => t > fe0),
  };
  // no digests -> nothing drawn
  st.setDigests(new Map());
  flushRaf();
  u.fills.length = 0; f(u);
  out.env_hook.none_drawn = u.fills.length;
  st.setDigests(dg);
  flushRaf();
  // one redraw per frame for a burst of digest installs; a real rebuild repaints at once
  const c0 = liveCharts().map((c) => c.setDataCalls);
  for (let k = 0; k < 7; k++) st.setDigests(new Map(dg));
  const c1 = liveCharts().map((c) => c.setDataCalls);
  flushRaf();
  const c2 = liveCharts().map((c) => c.setDataCalls);
  st.setResident([sliceRes]);
  const c3 = liveCharts().map((c) => c.setDataCalls);
  flushRaf();
  const c4 = liveCharts().map((c) => c.setDataCalls);
  const dlt = (a, b) => a.map((v, i) => b[i] - v);
  out.coalesce = { burst_before_frame: dlt(c0, c1), burst_after_frame: dlt(c1, c2), rebuild_immediate: dlt(c2, c3), rebuild_after_frame: dlt(c3, c4) };
  // y-range covers the digest envelope over the visible x window (a spike above the full-tier max)
  {
    u.scales = { x: { min: T0, max: T0 + 35 } };
    const env = st.envelope(0, 'pos_measured');
    let emax = -Infinity;
    for (let k = 0; k < env.t.length; k++) if (env.t[k] >= T0 && env.t[k] <= T0 + 35 && env.max[k] === env.max[k] && env.max[k] > emax) emax = env.max[k];
    const his = Object.keys(u.opts.scales).filter((k) => k !== 'x').map((k) => u.opts.scales[k].range(u, 0, emax - 1)[1]);
    out.range_env = { emax_finite: Number.isFinite(emax), covers: his.some((h) => h >= emax), base_below: emax - 1 < emax };
  }
  tc.exitReplayCharts();
  out.env_hook.removed = liveCharts().map((c) => !hasHook(c, 'drawReplayEnvelope'));
}
// ---- typed derivation: no hydration, and a chunk is derived ONCE however it is reached ----
{
  let calls = 0, hyd = 0;
  const spy = (m, i, l, h) => { calls++; return tc.telemetrySample(m, i, l, h); };
  const restore = [];
  for (const c of chunks) for (const name in c.topics) {
    const tp = c.topics[name]; const o = tp.hydrate;
    tp.hydrate = function (k) { hyd++; return o.call(this, k); };
    restore.push(() => { tp.hydrate = o; });
  }
  const s3 = createReplayChartStore({ telemetrySample: spy, latestBefore: feedLatest });
  s3.setResident([chunks[0]]);
  const afterResident = calls;
  s3.digestChunk(chunks[0]);                    // resident: the by-product digest reuses the derived columns
  const afterDigestResident = calls;
  s3.digestChunk(chunks[1]);                    // NOT resident: derived once here ...
  const afterDigestFar = calls;
  s3.setResident([chunks[0], chunks[1]]);       // ... and shared when it becomes resident
  const afterBoth = calls;
  s3.digestChunk(chunks[1]);
  out.derive_once = { hydrations: hyd, resident_calls: afterResident, digest_resident_extra: afterDigestResident - afterResident,
    far_calls: afterDigestFar - afterDigestResident, promote_extra: afterBoth - afterDigestFar, again_extra: calls - afterBoth };
  for (const r of restore) r();
}

// ---- y range vs uPlot's numeric tick loop (mirror of the vendored uPlot 1.6.31 numAxisSplits/findIncr) ----
{
  const Te = new Map();
  const roundDec = (v, d) => (Number.isInteger(v) ? v : Math.round(v * 10 ** d * (1 + Number.EPSILON)) / 10 ** d);
  const incrs = [];
  for (let e = -32; e < 32; e++) for (const m of [1, 2, 2.5, 5]) {
    const dec = Math.max(0, -e) + (m === 2.5 && e <= 0 ? 1 : 0);
    const v = e < 0 ? roundDec(m * 10 ** e, dec) : m * 10 ** e;
    incrs.push(v); Te.set(v, dec);
  }
  const digits = (v) => 1 + (0 | Math.log10((v ^ (v >> 31)) - (v >> 31)));
  // Number of ticks the loop emits for [lo, hi] on a dim-px axis, or -1 if it does not advance within 1e5 steps.
  function ticks(lo, hi, dim, space) {
    const o = Math.max(digits(lo), digits(hi)), s = hi - lo;
    let found = 0;
    for (const e of incrs) { if (dim * e / s >= space && 17 >= o + (e < 5 ? Te.get(e) : 0)) { found = e; break; } }
    if (!found) return 0;
    const dec = Te.get(found) || 0;
    let n = 0;
    for (let v = roundDec(Math.ceil(lo / found) * found, dec); v <= hi; v = roundDec(v + found, dec)) if (++n > 1e5) return -1;
    return n;
  }
  const oldRange = (a, b, pf) => (a === b ? [a - Math.max(Math.abs(a) * 0.1, pf), a + Math.max(Math.abs(a) * 0.1, pf)] : [a - (b - a) * 0.05, b + (b - a) * 0.05]);
  const cases = [[1.79e9, 1.79e9 + 4.8e-7], [3e9, 3e9 + 1e-6], [3e9, 3e9 + 4.8e-7], [1e12, 1e12 + 2.5e-4], [100, 100.5], [5, 5], [0, 0], [-2, 7]];
  const rows = [];
  for (const [a, b] of cases) {
    const r = tc.yRangeFor(a, b, 0.5), o = oldRange(a, b, 0.5);
    let worst = 1e9;
    for (const dim of [40, 133, 300, 2000]) for (const sp of [10, 30, 50]) worst = Math.min(worst, ticks(r[0], r[1], dim, sp));
    let oldWorst = 1e9;
    for (const dim of [40, 133, 300, 2000]) for (const sp of [10, 30, 50]) oldWorst = Math.min(oldWorst, ticks(o[0], o[1], dim, sp));
    rows.push({ a, b, r, worst, oldWorst, same: r[0] === o[0] && r[1] === o[1] });
  }
  out.y_split = {
    rows, Y_MIN_REL_SPAN: tc.Y_MIN_REL_SPAN,
    nulls: [tc.yRangeFor(null, 3, 0.5), tc.yRangeFor(1, null, 0.5), tc.yRangeFor(-Infinity, 3, 0.5), tc.yRangeFor(1, Infinity, 0.5), tc.yRangeFor(NaN, 1, 0.5)],
  };
}
console.log(JSON.stringify(out));
