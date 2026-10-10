// Node harness for ros_ws/gui/js/replay/{chunk,sources}.js (chunk shape; the live session buffer was
// deleted 2026-10-10). The McapSource contract itself is replay_mcap_source_harness.js.
// Usage: node replay_feed_harness.js <chunkJsonDir> <sandboxDir>   (prints one JSON object)
// Run by tests/ros/test_gui_replay_feed.py inside a sandbox holding verbatim copies of chunk.js,
// sources.js, slot.js, replay_test_support.js and a {"type":"module"} package.json. The chunk JSON is the
// Python oracle's output (tests/ros/_replay_chunks.py).
import fs from 'fs';
import path from 'path';

const { chunkFromRecord, flattenMessage, indexLatestBefore, makeHydrator, buildColumns } = await import('./chunk.js');
const { McapSource } = await import('./sources.js');
const { loadRecords, revive, fakeWorkerFactory } = await import('./replay_test_support.js');

const cacheDir = process.argv[2];
const sandbox = process.argv[3];
const { manifest, records } = loadRecords(cacheDir, buildColumns);
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
      // by value against the ORACLE's plain columns (the typed record must hydrate to exactly these)
      const plain = records[i].topics[name].plain;
      const keys = Object.keys(plain);
      let bad = Object.keys(flat).length !== keys.length;
      for (const c of keys) if (JSON.stringify(flat[c]) !== JSON.stringify(plain[c][k])) bad = true;
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
{ // chunkFromRecord adopts the worker's typed buffers as is: no per-row work, no copies
  const rec = records[0];
  const ch = chunkFromRecord(rec);
  let same = true, typed = true;
  for (const name in rec.topics) {
    const a = rec.topics[name], b = ch.topics[name];
    if (b.t !== a.t || b.cols !== a.cols || b.kinds !== a.kinds) same = false;
    if (!b.kinds) typed = false;
  }
  out.adopt = { same, typed, topics: Object.keys(rec.topics).length };
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

process.stdout.write(JSON.stringify(out));
