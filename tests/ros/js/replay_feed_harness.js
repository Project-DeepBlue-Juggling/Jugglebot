// Node harness for ros_ws/gui/js/replay/{chunk,sources}.js (chunk shape, session buffer, and the
// McapSource-vs-session agreement check). The McapSource contract itself is replay_mcap_source_harness.js.
// Usage: node replay_feed_harness.js <chunkJsonDir> <sandboxDir>   (prints one JSON object)
// Run by tests/ros/test_gui_replay_feed.py inside a sandbox holding verbatim copies of chunk.js,
// sources.js, slot.js, replay_test_support.js and a {"type":"module"} package.json. The chunk JSON is the
// Python oracle's output (tests/ros/_replay_chunks.py).
import fs from 'fs';
import path from 'path';

const { chunkFromRecord, flattenMessage, indexLatestBefore, makeHydrator } = await import('./chunk.js');
const { McapSource, SessionBufferSource, createSessionBuffer, SESSION_BUFFER_SEC } = await import('./sources.js');
const { loadRecords, revive, fakeWorkerFactory } = await import('./replay_test_support.js');

const cacheDir = process.argv[2];
const sandbox = process.argv[3];
const { manifest, records } = loadRecords(cacheDir);
const N = records.length;
const out = {};

// ---- (1)(2)(3) decode / hydrate / flatten round-trip / index edges --------
const WANT = ['/robot_state', '/mocap_data', '/orchestrator_state', '/skills/attempt'];
const hydrated = {};
const rt = { rows: 0, mismatches: 0 };
const idx = { before: null, exact: null, between: null, after: null, last_k: null };
for (let i = 0; i < N; i++) {
  const ch = chunkFromRecord(records[i]);
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
  const f = path.join(sandbox, 'nan_chunk.json');
  if (fs.existsSync(f)) {
    const ch = chunkFromRecord(revive(JSON.parse(fs.readFileSync(f, 'utf8'))));
    const tp = ch.topics['/nan'];
    out.nan = [0, 1].map((k) => tp.hydrate(k));
    out.nan_cols_untouched = Number.isNaN(tp.cols['x'][0]);
  }
  const h = makeHydrator(['a.b', 'a.c', 'z']);
  out.hydrator = h({ 'a.b': [1], 'a.c': [[1, 2]], z: [undefined] }, 0);
  out.flatten_typed = flattenMessage({ a: { b: new Float32Array([1, 2]) }, l: [{ x: 1 }], n: null });
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
  const T0 = manifest.t0;
  const rec = McapSource({ id: 'rec', fetch: async () => ({ status: 503, json: async () => ({}) }),
    makeWorker: fakeWorkerFactory(manifest, records), overviewPollMs: 0, fileUrl: 'x' });
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
