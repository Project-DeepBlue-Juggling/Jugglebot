// Node harness for ros_ws/gui/js/replay/{chunk,sources}.js.
// Usage: node replay_feed_harness.js <cacheDir> <sandboxDir>   (prints one JSON object)
// Run by tests/ros/test_gui_replay_feed.py inside a sandbox holding verbatim copies of
// chunk.js, sources.js, msgpack.min.js (as msgpack.min.cjs) and a {"type":"module"} package.json.
import fs from 'fs';
import path from 'path';
import zlib from 'zlib';
import { createRequire } from 'module';

const require = createRequire(import.meta.url);
globalThis.MessagePack = require('./msgpack.min.cjs');

const { decodeChunk, flattenMessage, indexLatestBefore, makeHydrator } = await import('./chunk.js');
const { RecordingSource, SessionBufferSource, createSessionBuffer, SESSION_BUFFER_SEC } = await import('./sources.js');

const cacheDir = process.argv[2];
const sandbox = process.argv[3];
const manifest = JSON.parse(fs.readFileSync(path.join(cacheDir, 'manifest.json'), 'utf8'));
const overview = JSON.parse(fs.readFileSync(path.join(cacheDir, 'overview.json'), 'utf8'));
const N = manifest.chunks.length;
const out = {};

function chunkBytes(i) {
  const name = 'chunk-' + String(i).padStart(5, '0') + '.msgpack.gz';
  return new Uint8Array(zlib.gunzipSync(fs.readFileSync(path.join(cacheDir, name))));
}

// ---- (1)(2)(3) decode / hydrate / flatten round-trip / index edges --------
const WANT = ['/robot_state', '/mocap_data', '/orchestrator_state', '/skills/attempt'];
const hydrated = {};
const rt = { rows: 0, mismatches: 0 };
const idx = { before: null, exact: null, between: null, after: null, last_k: null };
for (let i = 0; i < N; i++) {
  const ch = decodeChunk(chunkBytes(i));
  for (const name of WANT) {
    const tp = ch.topics[name];
    if (!tp) continue;
    if (!(tp.t instanceof Float64Array) || tp.n !== tp.t.length) throw new Error('t must be Float64Array');
    for (let k = 0; k < tp.n; k++) {
      const row = tp.hydrate(k);
      (hydrated[name] = hydrated[name] || []).push({ t: tp.t[k], msg: row });
      const flat = flattenMessage(row);
      rt.rows++;
      const keys = Object.keys(tp.cols);
      let bad = Object.keys(flat).length !== keys.length;
      for (const c of keys) if (JSON.stringify(flat[c]) !== JSON.stringify(tp.cols[c][k])) bad = true;
      if (bad) rt.mismatches++;
    }
    if (name === '/robot_state' && i === 0) {
      const t = tp.t;
      idx.before = indexLatestBefore(t, t[0] - 1);
      idx.exact = indexLatestBefore(t, t[5]);
      idx.between = indexLatestBefore(t, (t[5] + t[6]) / 2);
      idx.after = indexLatestBefore(t, t[t.length - 1] + 100);
      idx.last_k = t.length - 1;
    }
  }
}
out.hydrated = hydrated;
out.roundtrip = rt;
out.index = idx;

// ---- non-finite rule ------------------------------------------------------
{
  const f = path.join(sandbox, 'nan_chunk.msgpack');
  if (fs.existsSync(f)) {
    const ch = decodeChunk(new Uint8Array(fs.readFileSync(f)));
    const tp = ch.topics['/nan'];
    out.nan = [0, 1].map((k) => tp.hydrate(k));
    out.nan_cols_untouched = Number.isNaN(tp.cols['x'][0]);
  }
  const h = makeHydrator(['a.b', 'a.c', 'z']);
  out.hydrator = h({ 'a.b': [1], 'a.c': [[1, 2]], z: [undefined] }, 0);
  out.flatten_typed = flattenMessage({ a: { b: new Float32Array([1, 2]) }, l: [{ x: 1 }], n: null });
}

// ---- fake server over the cache dir ---------------------------------------
let done = 1;
const log = [];
function resp(status, body) {
  return {
    ok: status >= 200 && status < 300, status,
    json: async () => body,
    arrayBuffer: async () => body.buffer.slice(body.byteOffset, body.byteOffset + body.byteLength),
  };
}
async function fakeFetch(url, init) {
  log.push((init && init.method || 'GET') + ' ' + url);
  const m = /\/recordings\/([^/]+)\/(.*)$/.exec(url);
  const [id, rest] = [m[1], m[2]];
  if (id === 'busy') return resp(409, { status: 'refused', reason: 'recording_in_progress' });
  const complete = done >= N;
  if (rest === 'open') return resp(200, { status: complete ? 'complete' : 'converting' });
  if (rest === 'status') {
    return resp(200, { status: complete ? 'complete' : 'converting', chunks_done: done,
      chunks_total: N, t0: manifest.t0, t1: complete ? manifest.t1 : null, error: null });
  }
  if (rest === 'manifest') {
    const mm = JSON.parse(JSON.stringify(manifest));
    if (!complete) { mm.status = 'converting'; mm.chunks = []; mm.t1 = null; mm.chunks_done = done; }
    return resp(200, mm);
  }
  if (rest === 'overview') return complete ? resp(200, overview) : resp(404, { error: 'no overview' });
  const c = /^chunks\/(\d+)$/.exec(rest);
  if (c) {
    const i = Number(c[1]);
    return i < done ? resp(200, chunkBytes(i)) : resp(404, {});
  }
  return resp(404, {});
}
const mk = (extra) => RecordingSource(Object.assign({ id: 'rec', fetch: fakeFetch, pollMs: 0 }, extra || {}));
const T0 = manifest.t0;

// ---- (4) RecordingSource ---------------------------------------------------
{
  const r = {};
  done = 1;
  const src = mk();
  const events = [];
  src.onChange((s) => events.push(s.state + ':' + s.chunksDone));
  await src.open();
  r.open_status = src.status();
  r.range_after_open = src.range();
  r.t0 = T0;
  let rejected = null;
  try { await src.load(1); } catch (e) { rejected = e.constructor.name; }
  r.load_beyond_frontier = rejected;
  done = 3;
  await src.poll();
  r.range_after_poll3 = src.range();
  r.status_after_poll3 = src.status();
  done = 4; // all chunks sealed; status flips to complete on the next poll
  await src.poll();
  r.status_final = src.status();
  r.range_final = src.range();
  r.events = events;
  r.manifest_chunks = (src.manifest().chunks || []).length;

  const w = await src.window(T0 + 5, T0 + 15, ['/robot_state']);
  r.window = w.map((c) => ({ i: c.i, topics: Object.keys(c.topics), n: c.topics['/robot_state'].n }));
  r.window_n_total = w.reduce((a, c) => a + c.topics['/robot_state'].n, 0);
  const tl = await src.timeline();
  r.timeline_is_overview = JSON.stringify(tl) === JSON.stringify(overview);

  r.lb = {};
  const queries = [
    ['/robot_state', 0.0001], ['/robot_state', 9.9999], ['/robot_state', 10.0], ['/robot_state', 34.5],
    ['/orchestrator_state', 10.0], ['/skills/attempt', 34.9], ['/cone/catch_event', 34.9],
    ['/skills/attempt', 3.0], ['/robot_state', -5],
  ];
  for (const [topic, off] of queries) {
    const hit = await src.latestBefore(T0 + off, topic);
    (r.lb[topic] = r.lb[topic] || []).push({ off, t: hit ? hit.t : null, keys: hit ? Object.keys(hit.msg).slice(0, 3) : null });
  }
  src.close();

  // converting-mode scan-back (frontier chunk 3, last /skills/attempt in chunk 2)
  done = 4;
  const conv = (extra) => RecordingSource(Object.assign({ id: 'rec', pollMs: 0, fetch: async (u, i) => {
    const res = await fakeFetch(u, i);
    if (/\/(status|manifest)$/.test(u)) { // present a still-converting recording
      const b = await res.json();
      b.status = 'converting'; b.chunks = []; b.t1 = null;
      return resp(200, b);
    }
    return res;
  } }, extra || {}));
  const s2 = conv();
  await s2.open();
  const h2 = await s2.latestBefore(T0 + 34.9, '/skills/attempt');
  r.scan_default = h2 ? h2.t : null;
  r.scan_state = s2.status().state;
  s2.close();
  const s3 = conv({ scanBackChunks: 1 });
  await s3.open();
  const h3 = await s3.latestBefore(T0 + 34.9, '/skills/attempt');
  r.scan_limit1 = h3 ? h3.t : null;
  const h3b = await s3.latestBefore(T0 + 34.9, '/robot_state');
  r.scan_limit1_present = h3b ? h3b.t : null;
  s3.close();

  // manifest-driven lookup needs no scan-back: complete + scanBackChunks 1 still finds chunk 2
  const s4 = mk({ scanBackChunks: 1 });
  await s4.open();
  const h4 = await s4.latestBefore(T0 + 34.9, '/skills/attempt');
  r.manifest_lookup = h4 ? h4.t : null;
  s4.close();

  // polling timer: injected timers, stops at complete and on close()
  const ticks = { set: 0, cleared: 0, fn: null };
  const timers = { setInterval: (fn, ms) => { ticks.set++; ticks.ms = ms; ticks.fn = fn; return 7; },
                   clearInterval: (id) => { if (id === 7) ticks.cleared++; } };
  done = 1;
  const s5 = RecordingSource({ id: 'rec', fetch: fakeFetch, timers });
  await s5.open();
  r.timer_ms = ticks.ms;
  done = 4;
  await s5.poll();
  r.timer_cleared_at_complete = ticks.cleared;
  s5.close();

  // 409 refusal
  let refusal = null;
  try { await RecordingSource({ id: 'busy', fetch: fakeFetch, pollMs: 0 }).open(); }
  catch (e) { refusal = { reason: e.reason, status: e.status }; }
  r.refusal = refusal;
  r.posted_open = log.filter((l) => l.startsWith('POST ')).length >= 1;
  out.recording = r;
}

// ---- (5) SessionBufferSource -----------------------------------------------
{
  const r = {};
  const base = 1_791_532_600; // epoch-ish, 10 s aligned
  let nowT = base;
  const buf = createSessionBuffer({ now: () => nowT });
  r.horizon = SESSION_BUFFER_SEC;
  buf.record('/control_mode_topic', { data: 'JOG' }, base, 'std_msgs/msg/String');
  let maxChunks = 0;
  for (let k = 0; k < 700 * 20; k++) {
    nowT = base + k / 20;
    buf.record('/robot_state', { x: k, a: { b: k * 2 }, arr: [k, k + 1], motors: [{ p: k }] }, nowT);
    maxChunks = Math.max(maxChunks, buf.chunkCount());
  }
  r.max_chunks = maxChunks;
  r.chunks_final = buf.chunkCount();
  const rg = buf.range();
  r.oldest_age = nowT - rg.t0;
  r.range_t1_is_now = rg.t1 === nowT;
  const old = await buf.latestBefore(nowT, '/control_mode_topic');
  r.sidecar = old ? { t: old.t - base, msg: old.msg } : null;
  const evicted = await buf.latestBefore(base + 1, '/robot_state');
  r.before_ring = evicted;
  const mid = nowT - 300;
  const hit = await buf.latestBefore(mid, '/robot_state');
  r.mid = { x: hit.msg.x, expect: Math.round((mid - base) * 20), b: hit.msg.a.b, arr: hit.msg.arr, motors: hit.msg.motors };
  const wnd = await buf.window(nowT - 25, nowT);
  r.window_chunks = wnd.map((c) => c.i);
  r.window_sealed = wnd.map((c) => !!c.sealed);

  // snapshot is frozen against later writes and eviction
  const snap = buf.snapshot();
  const before = (await snap.window(0, 1e12)).reduce((a, c) => a + (c.topics['/robot_state'] ? c.topics['/robot_state'].n : 0), 0);
  buf.record('/robot_state', { x: -1, a: { b: -1 }, arr: [], motors: [] }, nowT + 1);
  nowT += 400; buf.record('/robot_state', { x: -2, a: { b: -2 }, arr: [], motors: [] }, nowT);
  snap.record('/robot_state', { x: -3 }, nowT);
  const after = (await snap.window(0, 1e12)).reduce((a, c) => a + (c.topics['/robot_state'] ? c.topics['/robot_state'].n : 0), 0);
  r.snapshot_rows = [before, after];
  r.live_chunks_after_jump = buf.chunkCount();
  buf.clear();
  r.cleared = [buf.chunkCount(), buf.range().frontier];
  r.snapshot_survives_clear = (await snap.latestBefore(base + 300, '/robot_state')) !== null;
  out.session = r;
}

// ---- (5b) session vs recording on the same data -------------------------------
{
  const r = {};
  done = N;
  const rec = mk();
  await rec.open();
  const ses = SessionBufferSource({ horizonSec: 600 });
  const all = [];
  for (const name of ['/robot_state', '/orchestrator_state', '/skills/attempt']) {
    for (const row of hydrated[name]) all.push([row.t, name, row.msg]);
  }
  all.sort((a, b) => a[0] - b[0]);
  for (const [t, name, msg] of all) ses.record(name, msg, t);
  const topics = ['/robot_state', '/orchestrator_state', '/skills/attempt'];
  const win = async (src, a, b) => {
    const o = {};
    for (const c of await src.window(a, b, topics)) {
      for (const name in c.topics) {
        const tp = c.topics[name];
        for (let k = 0; k < tp.n; k++) (o[name] = o[name] || []).push(tp.t[k]);
      }
    }
    return o;
  };
  const a = T0 + 8, b = T0 + 22;
  // chunk-aligned windows differ between the two sources; compare after clipping to [a, b]
  const clip = (o) => { const c = {}; for (const n in o) c[n] = o[n].filter((t) => t >= a && t <= b); return c; };
  r.windows_equal = JSON.stringify(clip(await win(rec, a, b))) === JSON.stringify(clip(await win(ses, a, b)));
  r.window_counts = Object.fromEntries(Object.entries(clip(await win(ses, a, b))).map(([k, v]) => [k, v.length]));
  r.lb_equal = [];
  for (const topic of topics) {
    for (const off of [0.5, 9.99, 10.0, 17.3, 34.9]) {
      const x = await rec.latestBefore(T0 + off, topic);
      const y = await ses.latestBefore(T0 + off, topic);
      r.lb_equal.push(JSON.stringify(x) === JSON.stringify(y));
    }
  }
  const tl = await ses.timeline();
  r.session_band = tl.bands[0].segments.map((s) => s[2]);
  r.session_presence_topics = Object.keys(tl.presence).sort();
  out.agree = r;
}

process.stdout.write(JSON.stringify(out));
