/**
 * Far-tier ball history (Phase 5b): every ball flight in [p - tail, p] for the long-tail review view.
 *
 *   createBallHistory({source, cache, engine, getTailMs, onTailChange, layer, defer, frame, nearMaxMs = 5000})
 *     onPlayhead(p)   engine playhead feed (forward from replay/mode.js)
 *     status()        {active, centre, tailMs, paused, inflight, slots, wanted, rows, publishes}
 *     dispose()
 *   layer: {setSlots(arrayOfRows), clear()} (js/ball-history-layer.js)
 *   getTailMs: the EFFECTIVE tail (trail-settings getEffectiveTailMs).
 *
 * ACTIVE only while getTailMs() > nearMaxMs (the near tier of trail-window.js already draws up to nearMaxMs); otherwise the
 * ring is empty and the layer is cleared.  Shape follows digest.js:
 *   - window = slots overlapping [centre - tail - RETARGET_S, centre + RETARGET_S] (the playhead may sit RETARGET_S off the centre), clamped to what the source can serve; the
 *     centre re-targets to the playhead only after it moved more than RETARGET_S (10 s) or the tail changed;
 *     slots outside [centre - tail - RETARGET_S, centre + RETARGET_S] are evicted.
 *   - Fill order is the playhead slot first, then BACKWARD slot by slot (nearest first), then the slack ahead.
 *     One load in flight, a `defer` hop between steps.
 *   - A slot whose full chunk is already in memory (cache / source.peek) is read in place; otherwise
 *     source.load(i, {lite: true, topics: ['/balls']}): the worker decodes only /balls on its low-priority lane and the
 *     partial chunk is dropped as soon as its rows are extracted.
 *   - PAUSED (no new load) while the engine is buffering or the cache has a load in flight.
 * Per slot it keeps compact typed rows {i, n, t:Float64Array, id:Int32Array, x,y,z:Float32Array (robot mm)} from the
 * /balls CSR leaves, decimated to >= MIN_DT_S per id.  The slot set reaches the layer at most once per `frame` tick
 * (rAF in the browser) after it changed.  No clock reads: pacing is event-driven.
 */

export const SLOT_S = 10;
export const RETARGET_S = 10;
export const MIN_DT_S = 0.01;
const BALLS = '/balls';

function csrGroup(tp, name) {
  if (!tp || !tp.kinds || !tp.cols) return null;
  const kd = tp.kinds[name];
  const col = tp.cols[name];
  if (!kd || typeof kd.csr !== 'object' || !col || !col.offsets || !col.leaves) return null;
  return col;
}

/** Compact rows of one chunk's /balls topic, decimated to >= MIN_DT_S per id; null when the chunk has no usable /balls. */
export function extractRows(i, chunk) {
  const tp = chunk && chunk.topics && chunk.topics[BALLS];
  if (!tp) return null;
  const g = csrGroup(tp, 'balls');
  if (!g || !(tp.t instanceof Float64Array)) return null;
  const off = g.offsets, L = g.leaves;
  const total = off[tp.n];
  const t = new Float64Array(total), id = new Int32Array(total);
  const x = new Float32Array(total), y = new Float32Array(total), z = new Float32Array(total);
  const last = new Map();   // id -> last kept t (load-time only)
  let n = 0;
  for (let k = 0; k < tp.n; k++) {
    const tk = tp.t[k];
    for (let j = off[k]; j < off[k + 1]; j++) {
      const b = L.id[j];
      const lt = last.get(b);
      if (lt !== undefined && tk - lt < MIN_DT_S - 1e-9) continue;
      last.set(b, tk);
      t[n] = tk; id[n] = b; x[n] = L['position.x'][j]; y[n] = L['position.y'][j]; z[n] = L['position.z'][j];
      n++;
    }
  }
  return { i, n, t: t.slice(0, n), id: id.slice(0, n), x: x.slice(0, n), y: y.slice(0, n), z: z.slice(0, n) };
}

export function createBallHistory(opts) {
  const source = opts.source;
  const cache = opts.cache || null;
  const engine = opts.engine;
  const getTailMs = opts.getTailMs;
  const layer = opts.layer;
  const nearMaxMs = opts.nearMaxMs === undefined ? 5000 : opts.nearMaxMs;
  const defer = opts.defer || ((fn) => { setTimeout(fn, 0); });
  const frame = opts.frame || defer;

  const ring = new Map();       // slot -> rows
  const failed = new Set();
  const unsubs = [];
  let centre = null;
  let tailSnap = 0;             // tail (ms) the current window was computed for
  let inflight = null;
  let token = 0;
  let scheduled = false;
  let publishPending = false;
  let publishes = 0;
  let disposed = false;
  let wasActive = false;

  const tailMs = () => +getTailMs() || 0;
  const active = () => tailMs() > nearMaxMs;

  function paused() {
    const st = engine.state();
    if (st && st.mode === 'buffering') return true;
    return !!(cache && typeof cache.loading === 'function' && cache.loading());
  }

  function wanted() {
    if (centre === null || !active()) return [];
    const r = source.range();
    if (r.t0 === 0 && r.frontier === 0) return [];
    const lo = source.chunkIndex(r.t0);
    let hi = source.chunkIndex(r.frontier);
    if (source.bounds(hi)[0] >= r.frontier) hi -= 1;
    const tail = tailSnap / 1000;
    // the playhead may sit up to RETARGET_S behind the centre, so [p - tail, p] reaches RETARGET_S further back (same bound as keepSet)
    const a = Math.max(lo, source.chunkIndex(centre - tail - RETARGET_S));
    const b = Math.min(hi, source.chunkIndex(centre + RETARGET_S));
    const c = Math.min(Math.max(source.chunkIndex(centre), lo), hi);
    const out = [];
    for (let i = a; i <= b; i++) out.push(i);
    // playhead slot, then backward nearest-first, then the forward slack
    out.sort((p, q) => ((p <= c ? c - p : 1e6 + (p - c)) - (q <= c ? c - q : 1e6 + (q - c))));
    return out;
  }

  function keepSet() {
    if (centre === null || !active()) return new Set();
    const r = source.range();
    const tail = tailSnap / 1000;
    const a = source.chunkIndex(Math.max(r.t0, centre - tail - RETARGET_S));
    const b = source.chunkIndex(centre + RETARGET_S);
    const s = new Set();
    for (let i = a; i <= b; i++) s.add(i);
    return s;
  }

  function markDirty() {
    if (publishPending || disposed) return;
    publishPending = true;
    frame(() => {
      publishPending = false;
      if (disposed) return;
      publishes++;
      if (ring.size === 0) layer.clear();
      else layer.setSlots(Array.from(ring.values()).sort((p, q) => p.i - q.i));
    });
  }

  function evict() {
    const keep = keepSet();
    let changed = false;
    for (const i of Array.from(ring.keys())) if (!keep.has(i)) { ring.delete(i); changed = true; }
    if (changed) markDirty();
  }

  function inMemory(i) { return (cache && cache.peek(i)) || source.peek(i) || null; }

  function take(i, chunk) {
    const rows = extractRows(i, chunk);
    if (!rows) return false;
    ring.set(i, rows);
    markDirty();
    return true;
  }

  function kick() {
    if (disposed || scheduled) return;
    scheduled = true;
    defer(step);
  }

  function step() {
    scheduled = false;
    if (disposed) return;
    if (!active()) {
      if (wasActive) { wasActive = false; ring.clear(); failed.clear(); token++; inflight = null; markDirty(); }
      return;
    }
    wasActive = true;
    evict();
    const want = wanted();
    for (const i of want) {   // free rows from a resident full chunk (CPU only: allowed while paused)
      if (ring.has(i) || failed.has(i)) continue;
      const ch = inMemory(i);
      if (ch) { if (!take(i, ch)) failed.add(i); kick(); return; }
    }
    if (inflight !== null || paused()) return;
    const next = want.find((i) => !ring.has(i) && !failed.has(i));
    if (next === undefined) return;
    const my = ++token;
    inflight = next;
    Promise.resolve().then(() => source.load(next, { lite: true, topics: [BALLS] })).then((ch) => {
      if (disposed || my !== token) return;
      inflight = null;
      if (ch && wanted().indexOf(next) >= 0) { if (!take(next, ch)) failed.add(next); }
      else if (!ch) failed.add(next);
      kick();
    }, () => {
      if (disposed || my !== token) return;
      inflight = null;
      failed.add(next);
      kick();
    });
  }

  function retarget(p, force) {
    if (!force && centre !== null && Math.abs(p - centre) <= RETARGET_S) return;
    centre = p;
    failed.clear();
    kick();
  }

  let lastP = null;
  function onPlayhead(p) {
    if (disposed || !Number.isFinite(p)) return;
    lastP = p;
    if (!active()) { if (wasActive) kick(); return; }   // the idle tier schedules nothing per frame
    retarget(p, false);
  }

  const st0 = engine.state();
  if (st0 && Number.isFinite(st0.playhead)) lastP = st0.playhead;
  tailSnap = tailMs();
  if (active() && lastP !== null) centre = lastP;
  if (typeof opts.onTailChange === 'function') {
    unsubs.push(opts.onTailChange(() => {
      if (disposed) return;
      tailSnap = tailMs();
      if (active() && lastP !== null) { token++; inflight = null; retarget(lastP, true); } else kick();
    }));
  }
  unsubs.push(engine.on('buffering', (b) => { if (!b) kick(); }));
  if (cache && typeof cache.onChange === 'function') unsubs.push(cache.onChange(() => kick()));
  if (typeof source.onChange === 'function') unsubs.push(source.onChange(() => { failed.clear(); kick(); }));
  kick();

  return {
    onPlayhead,
    status() {
      return { active: active(), centre, tailMs: tailSnap, paused: paused(), inflight, slots: ring.size, wanted: wanted().length,
        rows: Array.from(ring.values()).reduce((s, r) => s + r.n, 0), publishes };
    },
    dispose() {
      if (disposed) return;
      disposed = true;
      token++;
      inflight = null;
      for (const u of unsubs) { if (typeof u === 'function') u(); }
      unsubs.length = 0;
      ring.clear();
      layer.clear();
    },
  };
}
