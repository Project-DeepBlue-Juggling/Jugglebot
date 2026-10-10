// Node harness for ros_ws/gui/js/replay/{digest,chart-store}.js over the Python oracle's chunks.
// Usage: node replay_digest_harness.js <chunkJsonDir>   (prints one JSON object)
// The sandbox holds VERBATIM copies of chunk.js, chart-store.js, digest.js, replay_test_support.js and a
// {"type":"module"} package.json. telemetrySample is a pure stand-in with the real key set (the real one is
// covered against the live path by test_gui_replay_chart.py); the binning is what is under test here.
const { createReplayChartStore, SIGNAL_KEYS, MOTOR_COUNT } = await import('./replay/chart-store.js');
const { createDigester, SLOT_S } = await import('./replay/digest.js');
const { chunkFromRecord } = await import('./replay/chunk.js');
const { loadRecords } = await import('./replay_test_support.js');

const { manifest, records } = loadRecords(process.argv[2]);
const T0 = manifest.t0;
const chunks = records.map((r) => chunkFromRecord(r));
const out = {};

function telemetrySample(m, idx, legs, hand) {
  return {
    pos_measured: m.pos_estimate * 2 + idx, vel_measured: m.vel_estimate, iq_setpoint: m.iq_setpoint,
    iq_measured: m.iq_measured, fet_temp: m.fet_temp, motor_temp: m.motor_temp,
    bus_voltage: m.bus_voltage, bus_current: m.bus_current,
    pos_commanded: idx < 6 && legs ? legs[idx] : (idx === 6 && hand ? hand.pos_cmd : NaN),
  };
}
const mkStore = () => createReplayChartStore({ telemetrySample });

// ---------------- 1. bins vs a brute-force reference ----------------
{
  const store = mkStore();
  let bad = 0, cmp = 0, nSum = 0, samples = 0, centresOk = true, withNan = 0, emptyBins = 0;
  const approx = (a, b) => (Number.isNaN(a) && Number.isNaN(b)) || Math.abs(a - b) <= 1e-9 * Math.max(1, Math.abs(a), Math.abs(b));
  const results = [];
  for (const ch of chunks) {
    store.setResident([ch]);
    const full = store.axes.map((a) => ({ t: a.timestamps, cols: a.columns }));
    const res = store.digestChunk(ch);          // resident path
    store.setResident([]);
    const far = store.digestChunk(ch);          // far path (not resident): own-first joins
    results.push([res, far]);
    for (const d of [res, far]) {
      for (let a = 0; a < MOTOR_COUNT; a++) {
        // brute force: group the full-rate samples by floor(t - t0)
        const groups = new Map();
        for (let k = 0; k < full[a].t.length; k++) {
          const b = Math.min(SLOT_S - 1, Math.max(0, Math.floor(full[a].t[k] - ch.t0)));
          if (!groups.has(b)) groups.set(b, []);
          groups.get(b).push(k);
        }
        const keys = Array.from(groups.keys()).sort((x, y) => x - y);
        samples += full[a].t.length;
        if (d.axes[a].t.length !== keys.length) { bad += 1000; continue; }
        keys.forEach((b, j) => {
          cmp++;
          if (!approx(d.axes[a].t[j], ch.t0 + b + 0.5)) centresOk = false;
          nSum += d.axes[a].n[j];
          if (d.axes[a].n[j] !== groups.get(b).length) bad++;
          for (const key of SIGNAL_KEYS) {
            const vs = groups.get(b).map((k) => full[a].cols[key][k]).filter((v) => !Number.isNaN(v));
            const want = vs.length ? { min: Math.min(...vs), max: Math.max(...vs), mean: vs.reduce((s, v) => s + v, 0) / vs.length }
              : { min: NaN, max: NaN, mean: NaN };
            if (!vs.length) withNan++;
            for (const f of ['min', 'max', 'mean']) if (!approx(d.axes[a].cols[key][f][j], want[f])) bad++;
            const c = d.axes[a].cols[key];
            if (!(c.min instanceof Float64Array && c.mean instanceof Float64Array && c.max instanceof Float64Array)) bad++;
            if (!(c.min[j] <= c.mean[j] + 1e-9 && c.mean[j] <= c.max[j] + 1e-9) && !Number.isNaN(c.mean[j])) bad++;
          }
        });
      }
    }
  }
  out.bins = { bad, cmp, centresOk, withNan, n_sum_is_samples: nSum === samples, samples,
    resident_equals_far: JSON.stringify(results.map(([a]) => a.axes.map((x) => Array.from(x.t)))) ===
      JSON.stringify(results.map(([, b]) => b.axes.map((x) => Array.from(x.t)))) };
  // open (unsealed) chunk -> null
  const open = { i: 0, t0: T0, t1: T0 + 10, topics: { '/robot_state': { n: 0, t: [], cols: {}, hydrate() {} } } };
  out.bins.open_chunk_null = store.digestChunk(open) === null;
  out.bins.axes_per_digest = results[0][0].axes.length;
}

// ---------------- fakes for the digester ----------------
function fakeSource(list, opts) {
  opts = opts || {};
  const log = [];
  const gate = { hold: null };
  const t0 = T0;
  const n = list.length;
  const src = {
    kind: 'recording', log, gate, memo: new Map(),
    range() { return { t0, t1: t0 + n * 10, frontier: t0 + n * 10 }; },
    chunkIndex(t) { return Math.floor((t - t0) / 10); },
    bounds(i) { return [t0 + i * 10, t0 + (i + 1) * 10]; },
    peek(i) { return src.memo.get(i) || null; },
    status() { return { state: 'complete' }; },
    onChange() { return () => {}; },
    async load(i, o) {
      log.push({ i, lite: !!(o && o.lite) });
      if (gate.hold) await gate.hold;
      if (opts.fail && opts.fail.has(i)) throw new Error('boom');
      return list[i];
    },
  };
  return src;
}
function fakeEngine(p) {
  const l = { playhead: new Set(), buffering: new Set() };
  const e = {
    mode: 'playing', p,
    state() { return { mode: e.mode, playhead: e.p }; },
    on(evt, cb) { l[evt].add(cb); return () => l[evt].delete(cb); },
    move(x) { e.p = x; for (const cb of Array.from(l.playhead)) cb(x); },
    setBuffering(b) { e.mode = b ? 'buffering' : 'playing'; for (const cb of Array.from(l.buffering)) cb(b); },
  };
  return e;
}
function fakeCache() {
  const ls = new Set();
  const c = {
    res: new Map(), busy: false,
    peek(i) { return c.res.get(i) || null; }, loading() { return c.busy; },
    onChange(cb) { ls.add(cb); return () => ls.delete(cb); },
    fire() { for (const cb of Array.from(ls)) cb({ reason: 'residency' }); },
  };
  return c;
}
function manualDefer() {
  const q = [];
  return { q, fn: (f) => { q.push(f); } };
}
async function settle(d) {
  for (let n = 0; n < 400; n++) {
    await Promise.resolve(); await Promise.resolve();
    if (d.q.length) { const f = d.q.shift(); f(); } else { await new Promise((r) => setTimeout(r, 0)); if (!d.q.length) break; }
  }
}
function make(list, span, over) {
  over = over || {};
  const src = over.src || fakeSource(list);
  const eng = fakeEngine(over.p === undefined ? T0 + 55 : over.p);
  const cache = fakeCache();
  const store = mkStore();
  const d = manualDefer();
  const updates = [];
  const dg = createDigester({ source: src, cache, engine: eng, store, spanSec: span, defer: d.fn, onUpdate: (r) => updates.push(r.size) });
  return { src, eng, cache, store, d, dg, updates };
}

// ---------------- 2. nearest-first fill ----------------
{
  const x = make(chunks, 25);
  await settle(x.d);
  out.fill = { order: x.src.log.map((e) => e.i), all_lite: x.src.log.every((e) => e.lite), slots: Array.from(x.dg.digests().keys()).sort((a, b) => a - b),
    store_count: x.store.digestCount, status_wanted: x.dg.status().wanted, digestAt5: !!x.dg.digestAt(5), digestAt0: x.dg.digestAt(0) === undefined };
  // ---------------- 3. retarget + eviction ----------------
  x.src.log.length = 0;
  x.eng.move(T0 + 65);   // 10 s: no re-target
  await settle(x.d);
  const small = { loads: x.src.log.length, centre: x.dg.status().centre };
  x.eng.move(T0 + 15);   // 40 s jump from the centre (55): re-target
  await settle(x.d);
  out.retarget = { small, centre: x.dg.status().centre, slots: Array.from(x.dg.digests().keys()).sort((a, b) => a - b),
    new_order: x.src.log.map((e) => e.i), store_count: x.store.digestCount };
  // ---------------- unit toggle ----------------
  x.src.log.length = 0;
  x.store.invalidate();
  const cleared = x.dg.digests().size;
  await settle(x.d);
  out.invalidate = { cleared, refilled: x.dg.digests().size, reloaded: x.src.log.length };
  x.dg.dispose();
  out.dispose = { ring: x.dg.digests().size };
}

// ---------------- 4. paused ----------------
{
  const x = make(chunks, 25);
  x.eng.setBuffering(true);
  await settle(x.d);
  const buffering = x.src.log.length;
  x.eng.setBuffering(false);
  await settle(x.d);
  const resumed = x.src.log.length;
  // a cache load in flight pauses it, and the cache's own event resumes it
  const y = make(chunks, 25);
  y.cache.busy = true;
  await settle(y.d);
  const busy = y.src.log.length;
  y.cache.busy = false; y.cache.fire();
  await settle(y.d);
  // by-product work is NOT paused: a cache-resident chunk is digested for free while buffering
  const z = make(chunks, 25);
  z.eng.setBuffering(true);
  z.cache.res.set(5, chunks[5]);
  z.cache.fire();
  await settle(z.d);
  out.paused = { buffering, resumed_loads: resumed, busy, after_busy: y.src.log.length, by_product_while_buffering: !!z.dg.digestAt(5),
    z_loads: z.src.log.length };
  // in-flight lite load when a seek starts: finishes, then nothing new is issued
  const w = make(chunks, 25);
  let release; w.src.gate.hold = new Promise((r) => { release = r; });
  await settle(w.d);
  const first = w.src.log.length;
  w.eng.setBuffering(true);
  release();
  await settle(w.d);
  out.paused.inflight = { first, after: w.src.log.length, ring: w.dg.digests().size };
}

// ---------------- 5. by-product digests ----------------
{
  const x = make(chunks, 25);
  x.cache.res.set(5, chunks[5]); x.cache.res.set(6, chunks[6]);
  x.src.memo.set(4, chunks[4]);
  x.cache.fire();
  await settle(x.d);
  const loaded = x.src.log.map((e) => e.i);
  out.byproduct = { loaded, not_loaded: [4, 5, 6].every((i) => !loaded.includes(i)), digested: [4, 5, 6].every((i) => !!x.dg.digestAt(i)),
    rest_loaded: [3, 7, 8].every((i) => loaded.includes(i)) };
  // a failing slot is skipped, not retried in a loop
  const f = make(chunks, 25, { src: fakeSource(chunks, { fail: new Set([6]) }) });
  await settle(f.d);
  out.failed = { tries6: f.src.log.filter((e) => e.i === 6).length, others: Array.from(f.dg.digests().keys()).sort((a, b) => a - b) };
  // an unsealed chunk (digestChunk -> null) is marked failed, not hot-looped
  const un = chunks.slice();
  un[6] = { i: 6, t0: T0 + 60, t1: T0 + 70, topics: { '/robot_state': { n: 0, t: [], cols: {}, hydrate() {} } } };
  const u = make(un, 25, { src: fakeSource(un) });
  await settle(u.d);
  out.unsealed = { tries6: u.src.log.filter((e) => e.i === 6).length, has6: u.dg.digests().has(6) };
}

// ---------------- 6. memory bound over a long virtual recording ----------------
{
  const N = 200;
  const virt = [];
  for (let i = 0; i < N; i++) virt.push({ i, t0: T0 + i * 10, t1: T0 + (i + 1) * 10, topics: {} });
  const span = 330;
  const x = make(virt, span, { p: T0 + 1000 });
  let maxRing = 0;
  const watch = () => { maxRing = Math.max(maxRing, x.dg.digests().size, x.store.digestCount); };
  await settle(x.d); watch();
  const fullRing = x.dg.digests().size;
  for (const p of [T0 + 1500, T0 + 400, T0 + 1990, T0 + 5, T0 + 1000]) { x.eng.move(p); await settle(x.d); watch(); }
  out.bound = { span, full_ring: fullRing, max_ring: maxRing, max_update: Math.max(...x.updates), limit: 2 * span / 10 + 2,
    lo_hi: Array.from(x.dg.digests().keys()).sort((a, b) => a - b).slice(0, 1).concat(Array.from(x.dg.digests().keys()).sort((a, b) => a - b).slice(-1)) };
}

// ---------------- 7. store: composed two-tier data + envelope ----------------
{
  const store = mkStore();
  const bySlot = new Map();
  for (const ch of chunks) bySlot.set(ch.i, store.digestChunk(ch));
  store.setResident([chunks[4], chunks[5]]);
  const v0 = store.version;
  const plain = store.axes.map((a) => ({ ts: a.timestamps, len: a.length }));
  const preData = store.axes[0].getAlignedData(['pos_measured']);
  const noDigestSame = preData[0] === store.axes[0].timestamps && preData[1] === store.axes[0].columns.pos_measured;
  const noEnv = store.axes[0].envelope('pos_measured');
  const before = store.axes[0];
  store.setDigests(bySlot);
  const ax = store.axes[0];
  const keys = ['pos_measured', 'iq_measured'];
  const data = ax.getAlignedData(keys);
  const env = ax.envelope('pos_measured');
  const nbins = (rng, a) => rng.reduce((s, i) => s + bySlot.get(i).axes[a].t.length, 0);
  const bIdx = [0, 1, 2, 3], aIdx = [6, 7, 8, 9];
  const res = {};
  res.version_bumped = store.version > v0;
  res.old_snapshot_untouched = before.getAlignedData(['pos_measured'])[0].length === plain[0].len && before !== ax;
  res.no_digest_same = noDigestSame;
  res.no_env = { t: noEnv.t.length, min: noEnv.min.length, split: noEnv.split };
  let mono = true;
  for (let k = 1; k < data[0].length; k++) if (!(data[0][k] > data[0][k - 1])) mono = false;
  res.monotonic = mono;
  const nb = nbins(bIdx, 0), na = nbins(aIdx, 0);
  res.lengths = { total: data[0].length, want: nb + ax.length + na, nb, na, full: ax.length };
  res.cols_len = data.every((c) => c.length === data[0].length);
  // full tier exact where resident
  let exact = true;
  for (let k = 0; k < ax.length; k++) {
    if (data[0][nb + k] !== ax.timestamps[k]) exact = false;
    if (!Object.is(data[1][nb + k], ax.columns.pos_measured[k])) exact = false;
    if (!Object.is(data[2][nb + k], ax.columns.iq_measured[k])) exact = false;
  }
  res.full_exact = exact;
  // means elsewhere
  let meanOk = true, o = 0;
  for (const i of bIdx) { const d = bySlot.get(i).axes[0]; for (let j = 0; j < d.t.length; j++, o++) if (!Object.is(data[1][o], d.cols.pos_measured.mean[j]) || data[0][o] !== d.t[j]) meanOk = false; }
  o = nb + ax.length;
  for (const i of aIdx) { const d = bySlot.get(i).axes[0]; for (let j = 0; j < d.t.length; j++, o++) if (!Object.is(data[1][o], d.cols.pos_measured.mean[j]) || data[0][o] !== d.t[j]) meanOk = false; }
  res.means_ok = meanOk;
  // envelope only in digest regions
  const rr = store.residentRange();
  res.env = { n: env.t.length, want: nb + na, split: env.split, want_split: nb,
    outside_resident: Array.from(env.t).every((t) => t < rr.t0 || t >= rr.t1),
    min_le_max: Array.from(env.min).every((v, k) => !(v > env.max[k])) , min_len: env.min.length === env.t.length && env.max.length === env.t.length };
  res.store_env_delegate = store.envelope(0, 'pos_measured').t === env.t;
  // partial overlap of a digest with the resident window is dropped, not double counted
  store.setResident([chunks[5]]);
  const d2 = store.axes[0].getAlignedData(['pos_measured']);
  const bb = nbins([0, 1, 2, 3, 4], 0), aa = nbins([6, 7, 8, 9], 0);
  res.after_shift = { total: d2[0].length, want: bb + store.axes[0].length + aa };
  // no resident chunks: digests alone
  store.setResident([]);
  res.no_resident = { total: store.axes[0].getAlignedData(['pos_measured'])[0].length, want: nbins([0, 1, 2, 3, 4, 5, 6, 7, 8, 9], 0) };
  // clear digests -> back to the full tier
  store.setResident([chunks[4]]);
  store.setDigests(new Map());
  res.cleared = store.axes[0].getAlignedData(['pos_measured'])[0] === store.axes[0].timestamps;
  // listeners
  let fired = 0; store.onRebuild(() => { fired++; });
  store.setDigests(bySlot);
  res.listener_fired = fired === 1;
  let inv = 0; store.onInvalidate(() => { inv++; });
  store.invalidate();
  res.invalidate = { inv, digests_dropped: store.digestCount === 0 };
  out.store = res;
}

process.stdout.write(JSON.stringify(out));
