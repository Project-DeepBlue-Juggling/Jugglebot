/**
 * Far-tier digester (post-Phase-4 "10-minute view").
 *
 *   createDigester({source, cache, engine, store, spanSec = 330, onUpdate, defer})
 *     digestAt(i)    the digest of slot i or undefined
 *     digests()      Map slot -> digest (the live ring, do not mutate)
 *     status()       {centre, paused, inflight, slots, wanted}
 *     dispose()
 *
 * Keeps one digest (chart-store.digestChunk: 1 s min/max/mean bins of the derived chart signals) per
 * 10 s slot in a ring around the playhead:
 *   - window = slots overlapping [centre - spanSec, centre + spanSec], clamped to what the source can
 *     serve; the ring therefore never exceeds 2*spanSec/10 + 2 slots.  The centre re-targets to the
 *     playhead only after it has moved more than RETARGET_S (30 s); slots outside the window are evicted.
 *   - Fill order is NEAREST-FIRST around the centre (ahead wins a tie); one slot per step, one load in
 *     flight, a `defer` hop between steps so the main thread is never held.
 *   - BY-PRODUCT: a slot whose chunk is already in memory (cache-resident, or source.peek) is digested
 *     synchronously instead of being re-loaded.  Sealed chunks only (an open session chunk is covered by
 *     the full tier and is retried later).
 *   - Otherwise source.load(i, {lite: true}): the worker's low-priority lane, never memoised by the
 *     source - the chunk object is dropped as soon as it is digested.
 *   - PAUSED (no new load issued) while the engine is buffering (a seek or a stall) or the cache has a
 *     load in flight: playback never waits on the digester.  A lite load already in flight is not
 *     cancelled (the worker finishes the decode; its reply is still used if the slot is wanted).
 *   - A unit toggle (store.onInvalidate) clears the ring and refills it.
 * No clock reads: pacing is event-driven (engine / cache / source events and the defer hop).
 */

export const SLOT_S = 10;
export const RETARGET_S = 30;
export const DEFAULT_SPAN_S = 330;

export function createDigester(opts) {
  const source = opts.source;
  const cache = opts.cache || null;
  const engine = opts.engine;
  const store = opts.store;
  const spanSec = opts.spanSec === undefined ? DEFAULT_SPAN_S : opts.spanSec;
  const onUpdate = opts.onUpdate || null;
  const defer = opts.defer || ((fn) => { setTimeout(fn, 0); });

  const ring = new Map();       // slot -> digest
  const failed = new Set();     // slots whose lite load failed since the last retarget
  const unsubs = [];
  let centre = null;
  let inflight = null;          // slot of the lite load in flight
  let token = 0;
  let scheduled = false;
  let disposed = false;

  function paused() {
    const st = engine.state();
    if (st && st.mode === 'buffering') return true;
    return !!(cache && typeof cache.loading === 'function' && cache.loading());
  }

  /** Nearest-first wanted slot list for the current centre ([] until a centre exists). */
  function wanted() {
    if (centre === null) return [];
    const r = source.range();
    if (r.t0 === 0 && r.frontier === 0) return [];
    let lo = source.chunkIndex(r.t0);
    let hi = source.chunkIndex(r.frontier);
    if (source.bounds(hi)[0] >= r.frontier) hi -= 1;
    const a = Math.max(lo, source.chunkIndex(centre - spanSec));
    const b = Math.min(hi, source.chunkIndex(centre + spanSec));
    const c = Math.min(Math.max(source.chunkIndex(centre), lo), hi);
    const out = [];
    for (let i = a; i <= b; i++) out.push(i);
    out.sort((x, y) => (Math.abs(x - c) - Math.abs(y - c)) || (y - x));
    return out;
  }

  function publish() {
    store.setDigests(ring);
    if (onUpdate) onUpdate(ring);
  }

  function evict(want) {
    const keep = new Set(want);
    let changed = false;
    for (const i of Array.from(ring.keys())) if (!keep.has(i)) { ring.delete(i); changed = true; }
    return changed;
  }

  function inMemory(i) {
    return (cache && cache.peek(i)) || source.peek(i) || null;
  }

  function take(i, chunk) {
    const d = store.digestChunk(chunk);
    if (!d) return false;
    ring.set(i, d);
    return true;
  }

  function kick() {
    if (disposed || scheduled) return;
    scheduled = true;
    defer(step);
  }

  /** One unit of work: a by-product digest, else a lite load; reschedules itself while work remains. */
  function step() {
    scheduled = false;
    if (disposed) return;
    const want = wanted();
    const evicted = evict(want);
    let did = false;
    // 1. free digests, nearest first (CPU only: allowed while paused)
    for (const i of want) {
      if (ring.has(i)) continue;
      const ch = inMemory(i);
      if (ch && take(i, ch)) { did = true; break; }
    }
    if (did || evicted) publish();
    if (did) { kick(); return; }
    // 2. a lite load, only when playback does not need the worker
    if (inflight !== null || paused()) return;
    const next = want.find((i) => !ring.has(i) && !failed.has(i));
    if (next === undefined) return;
    const my = ++token;
    inflight = next;
    Promise.resolve().then(() => source.load(next, { lite: true })).then((ch) => {
      if (disposed || my !== token) return;
      inflight = null;
      if (ch && centreWants(next)) {
        if (take(next, ch)) publish(); else failed.add(next);   // unsealed: do not hot-loop
      } else if (!ch) {
        failed.add(next);   // nothing to digest (session gap): do not spin on it
      }
      kick();
    }, () => {
      if (disposed || my !== token) return;
      inflight = null;
      failed.add(next);
      kick();
    });
  }

  function centreWants(i) { return wanted().indexOf(i) >= 0; }

  function retarget(p) {
    if (centre !== null && Math.abs(p - centre) <= RETARGET_S) return;
    centre = p;
    failed.clear();
    kick();
  }

  const st0 = engine.state();
  centre = st0 && Number.isFinite(st0.playhead) ? st0.playhead : null;
  unsubs.push(engine.on('playhead', (p) => { if (Number.isFinite(p)) retarget(p); }));
  unsubs.push(engine.on('buffering', (b) => { if (!b) kick(); }));
  if (cache && typeof cache.onChange === 'function') unsubs.push(cache.onChange(() => kick()));
  if (typeof source.onChange === 'function') unsubs.push(source.onChange(() => { failed.clear(); kick(); }));
  if (typeof store.onInvalidate === 'function') {
    unsubs.push(store.onInvalidate(() => {
      ring.clear(); failed.clear(); token++; inflight = null;
      if (onUpdate) onUpdate(ring);
      kick();
    }));
  }
  kick();

  return {
    digestAt(i) { return ring.get(i); },
    digests() { return ring; },
    status() { return { centre, paused: paused(), inflight, slots: ring.size, wanted: wanted().length }; },
    dispose() {
      disposed = true;
      token++;
      inflight = null;
      for (const u of unsubs) { if (typeof u === 'function') u(); }
      unsubs.length = 0;
      ring.clear();
    },
  };
}
