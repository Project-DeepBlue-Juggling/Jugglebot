/**
 * Replay feed sources: one read interface over a converted recording
 * (RecordingSource) and the live 600 s session buffer (SessionBufferSource).
 *
 *   source.kind                      'recording' | 'session'
 *   source.range()                   sync  {t0, t1, frontier}   (frontier = end of sealed data)
 *   source.chunkIndex(t) / bounds(i) sync
 *   source.load(i)                   async Chunk | null
 *   source.window(t0, t1, topics?)   async Chunk[]
 *   source.latestBefore(t, topic)    async {t, msg} | null
 *   source.timeline()                async overview-shaped object, {partial:true,...} when incomplete
 *   source.status()                  sync  {state, chunksDone, chunksTotal}
 *   source.onChange(cb)              -> unsubscribe
 *   source.peek(i)                   sync  resident chunk or null (tiny scan-back memo; cache.js owns playback residency)
 *
 * Chunk shape: see chunk.js. Times are wall-clock seconds.
 *
 * The recording URL layout is `${baseUrl}/recordings/${id}/...` with baseUrl
 * defaulting to '/api/replay' (gui_server.py routes).
 */
import { decodeChunk, flattenMessage, indexLatestBefore, makeTopic } from './chunk.js';

export const SESSION_BUFFER_SEC = 600;
const CHUNK_S = 10;
// U5 decision: the chunk cache (cache.js) owns residency for playback. This memo survives only as a
// tiny scan-back buffer for latestBefore()/timeline() (which call load() directly and walk up to
// scanBackChunks = 6 chunks per call; a smaller FIFO would refetch the whole walk every time). It holds
// the same chunk objects the cache does, so it adds memory only for chunks the cache already evicted
// (<= 6 x ~10 MB), down from the 16-entry duplicate of the cache's own cap.
const MEMO_CAP = 6;

function mergeRuns(runs) {
  const out = [];
  for (const r of runs) {
    const last = out[out.length - 1];
    if (last && r[0] <= last[1]) last[1] = Math.max(last[1], r[1]);
    else out.push([r[0], r[1]]);
  }
  return out;
}

/** Presence {topic: [[t0,t1],...]} of merged runs of chunks holding the topic. */
function presenceOf(chunks) {
  const per = {};
  for (const ch of chunks) {
    for (const name in ch.topics) {
      if (ch.topics[name].n >= 1) (per[name] = per[name] || []).push([ch.t0, ch.t1]);
    }
  }
  const out = {};
  for (const name in per) out[name] = mergeRuns(per[name].sort((a, b) => a[0] - b[0]));
  return out;
}

/** Subset a chunk to the requested topics (shallow, shares columns). */
function pickTopics(ch, topics) {
  if (!topics) return ch;
  const sel = {};
  for (const name of topics) if (ch.topics[name]) sel[name] = ch.topics[name];
  return { i: ch.i, t0: ch.t0, t1: ch.t1, topics: sel };
}

function topicRow(ch, topic, t) {
  const tp = ch.topics[topic];
  if (!tp || tp.n < 1) return undefined;
  const k = indexLatestBefore(tp.t, t, tp.n);
  if (k < 0) return null; // topic present but starts after t
  return { t: tp.t[k], msg: tp.hydrate(k) };
}

// ---------------------------------------------------------------------------
// Recording source
// ---------------------------------------------------------------------------

/**
 * @param {{id:string, baseUrl?:string, fetch:function, decoder?:object,
 *          pollMs?:number, timers?:{setInterval:function, clearInterval:function},
 *          scanBackChunks?:number}} opts
 *   pollMs 0 disables the timer (tests call poll() themselves).
 */
export function RecordingSource(opts) {
  const base = (opts.baseUrl === undefined ? '/api/replay' : opts.baseUrl) +
    '/recordings/' + encodeURIComponent(opts.id);
  const doFetch = opts.fetch;
  const scanBack = opts.scanBackChunks === undefined ? 6 : opts.scanBackChunks;
  const pollMs = opts.pollMs === undefined ? 1000 : opts.pollMs;
  const timers = opts.timers || globalThis;

  let state = 'none';
  let manifest = null;
  let t0 = null;
  let t1 = null;
  let chunksDone = 0;
  let chunksTotal = null;
  let timer = null;
  let closed = false;
  let polling = false;
  const listeners = new Set();
  const memo = new Map();     // i -> Chunk
  const inflight = new Map(); // i -> Promise<Chunk>

  function emit() { for (const cb of Array.from(listeners)) cb(self.status()); }

  async function getJson(path) {
    const res = await doFetch(base + path);
    if (!res.ok) return null;
    return res.json();
  }

  async function loadManifest() {
    const m = await getJson('/manifest');
    if (m) {
      manifest = m;
      if (m.t0 !== null && m.t0 !== undefined) t0 = m.t0;
      if (state === 'complete' && m.t1 !== null) t1 = m.t1;
    }
  }

  function applyStatus(s) {
    const prev = state + '|' + chunksDone + '|' + chunksTotal;
    state = s.status;
    chunksDone = s.chunks_done || 0;
    chunksTotal = s.chunks_total === undefined ? null : s.chunks_total;
    if (s.t0 !== null && s.t0 !== undefined) t0 = s.t0;
    if (s.t1 !== null && s.t1 !== undefined) t1 = s.t1;
    return prev !== state + '|' + chunksDone + '|' + chunksTotal;
  }

  const self = {
    kind: 'recording',
    id: opts.id,

    /** POST .../open; rejects (with .reason) on a 409 refusal. */
    async open() {
      const res = await doFetch(base + '/open', { method: 'POST' });
      let body = null;
      try { body = await res.json(); } catch (e) { body = null; }
      if (res.status === 409) {
        const err = new Error('replay open refused: ' + (body && body.reason));
        err.reason = body && body.reason;
        err.status = 'refused';
        throw err;
      }
      if (!res.ok) {
        const err = new Error('replay open failed: HTTP ' + res.status);
        err.httpStatus = res.status;
        throw err;
      }
      state = body.status;
      const st = await getJson('/status');
      if (st) applyStatus(st);
      await loadManifest();
      if (state !== 'complete' && state !== 'failed' && pollMs > 0 && !closed) {
        timer = timers.setInterval(() => { self.poll(); }, pollMs);
      }
      emit();
      return body.status;
    },

    /** One status poll (the timer calls this every pollMs). */
    async poll() {
      if (polling || closed) return;
      polling = true;
      try {
        const st = await getJson('/status');
        if (!st) return;
        const changed = applyStatus(st);
        if (manifest === null || state === 'complete') await loadManifest();
        if ((state === 'complete' || state === 'failed') && timer !== null) {
          timers.clearInterval(timer);
          timer = null;
        }
        if (changed) emit();
      } finally {
        polling = false;
      }
    },

    range() {
      if (t0 === null) return { t0: 0, t1: 0, frontier: 0 };
      const done = state === 'complete';
      const end = done && t1 !== null ? t1 : t0 + chunksDone * CHUNK_S;
      return { t0, t1: done && t1 !== null ? t1 : end, frontier: end };
    },

    chunkIndex(t) { return Math.floor((t - (t0 || 0)) / CHUNK_S); },
    bounds(i) { return [(t0 || 0) + i * CHUNK_S, (t0 || 0) + (i + 1) * CHUNK_S]; },

    peek(i) { return memo.get(i) || null; },

    async load(i) {
      if (!(i >= 0 && i < chunksDone)) throw new RangeError('chunk ' + i + ' not available (chunks_done ' + chunksDone + ')');
      const hit = memo.get(i);
      if (hit) return hit;
      let p = inflight.get(i);
      if (!p) {
        p = (async () => {
          const res = await doFetch(base + '/chunks/' + i);
          if (!res.ok) throw new Error('chunk ' + i + ': HTTP ' + res.status);
          const ch = decodeChunk(await res.arrayBuffer(), opts.decoder);
          memo.set(i, ch);
          while (memo.size > MEMO_CAP) memo.delete(memo.keys().next().value);
          return ch;
        })();
        inflight.set(i, p);
        p.then(() => inflight.delete(i), () => inflight.delete(i));
      }
      return p;
    },

    async window(wt0, wt1, topics) {
      const last = chunksDone - 1;
      const a = Math.max(0, self.chunkIndex(wt0));
      const b = Math.min(last, self.chunkIndex(wt1));
      const out = [];
      for (let i = a; i <= b; i++) out.push(pickTopics(await self.load(i), topics));
      return out;
    },

    async latestBefore(t, topic) {
      if (chunksDone < 1) return null;
      let i = Math.min(chunksDone - 1, self.chunkIndex(t));
      if (i < 0) return null;
      if (state === 'complete' && manifest && manifest.chunks && manifest.chunks.length) {
        // manifest-driven: nearest chunk at or before i that holds the topic
        for (let j = Math.min(i, manifest.chunks.length - 1); j >= 0; j--) {
          const cnt = manifest.chunks[j].topics && manifest.chunks[j].topics[topic];
          if (cnt > 0) {
            const row = topicRow(await self.load(j), topic, t);
            if (row) return row;
          }
        }
        return null;
      }
      const stop = Math.max(0, i - scanBack + 1);
      for (; i >= stop; i--) {
        const row = topicRow(await self.load(i), topic, t);
        if (row) return row;
      }
      return null;
    },

    async timeline() {
      if (state === 'complete') {
        const ov = await getJson('/overview');
        if (ov) return ov;
      }
      return { partial: true, presence: presenceOf(Array.from(memo.values())) };
    },

    status() { return { state, chunksDone, chunksTotal }; },

    onChange(cb) { listeners.add(cb); return () => listeners.delete(cb); },

    manifest() { return manifest; },

    close() {
      closed = true;
      if (timer !== null) { timers.clearInterval(timer); timer = null; }
      listeners.clear();
    },
  };
  return self;
}

// ---------------------------------------------------------------------------
// Session buffer
// ---------------------------------------------------------------------------

function newOpenTopic(type) { return makeTopic(type, [], {}); }

function sealTopic(tp) {
  tp.t = Float64Array.from(tp.t);
  tp.n = tp.t.length;
  return tp;
}

function sealChunk(ch) {
  for (const name in ch.topics) sealTopic(ch.topics[name]);
  ch.sealed = true;
  return ch;
}

/** Immutable copy of an open chunk (for snapshot()). */
function copyChunk(ch) {
  const out = { i: ch.i, t0: ch.t0, t1: ch.t1, topics: {}, sealed: true };
  for (const name in ch.topics) {
    const tp = ch.topics[name];
    const cols = {};
    for (const c in tp.cols) cols[c] = tp.cols[c].slice();
    out.topics[name] = makeTopic(tp.type, Float64Array.from(tp.t), cols);
  }
  return out;
}

/**
 * @param {{horizonSec?:number, chunkSec?:number, now?:function}} [opts]
 *   now: () => seconds, the eviction reference (default: newest record time).
 */
export function SessionBufferSource(opts) {
  opts = opts || {};
  const horizon = opts.horizonSec === undefined ? SESSION_BUFFER_SEC : opts.horizonSec;
  const chunkSec = opts.chunkSec || CHUNK_S;
  const nowFn = opts.now || null;

  let ring = new Map();       // chunk index -> Chunk (insertion order == ascending)
  let openIdx = null;
  let lastT = null;
  let firstT = null;
  let last = {};              // topic -> {t, msg}  every record
  let evictedLast = {};       // topic -> {t, msg}  newest message that left the ring
  let frozen = !!opts._frozen;
  const listeners = new Set();

  function emit() { for (const cb of Array.from(listeners)) cb(self.status()); }

  function evict() {
    const ref = nowFn ? nowFn() : lastT;
    if (ref === null) return;
    const cutoff = ref - horizon;
    for (const [i, ch] of Array.from(ring)) {
      if (i === openIdx || (i + 1) * chunkSec > cutoff) break;
      for (const name in ch.topics) {
        const tp = ch.topics[name];
        if (tp.n < 1) continue;
        const prev = evictedLast[name];
        const tt = tp.t[tp.n - 1];
        if (!prev || tt >= prev.t) evictedLast[name] = { t: tt, msg: tp.hydrate(tp.n - 1) };
      }
      ring.delete(i);
    }
  }

  const self = {
    kind: 'session',

    /**
     * Append one message. Flattened into the epoch-aligned open chunk; the
     * chunk seals when a later one opens. Ignored on a frozen snapshot.
     * @param {string} topic  '/x' (leading slash added if missing)
     * @param {object} msg
     * @param {number} tSec   wall-clock seconds
     * @param {string} [type]
     */
    record(topic, msg, tSec, type) {
      if (frozen) return;
      if (topic[0] !== '/') topic = '/' + topic;
      const prevLast = last[topic];
      if (prevLast && tSec < prevLast.t) tSec = prevLast.t; // keep each topic's t ascending
      const idx = Math.floor(tSec / chunkSec);
      let ch;
      if (openIdx !== null && idx <= openIdx) {
        ch = ring.get(openIdx);
      } else {
        if (openIdx !== null) sealChunk(ring.get(openIdx));
        ch = { i: idx, t0: idx * chunkSec, t1: (idx + 1) * chunkSec, topics: {}, sealed: false };
        ring.set(idx, ch);
        openIdx = idx;
      }
      let tp = ch.topics[topic];
      if (!tp) { tp = newOpenTopic(type || null); ch.topics[topic] = tp; }
      const flat = flattenMessage(msg);
      const n = tp.t.length;
      for (const c in flat) {
        let col = tp.cols[c];
        if (!col) { col = new Array(n); tp.cols[c] = col; } // backfill: undefined entries
        col.push(flat[c]);
      }
      for (const c in tp.cols) if (!(c in flat)) tp.cols[c].push(undefined);
      tp.t.push(tSec);
      tp.n = tp.t.length;
      last[topic] = { t: tSec, msg: tp.hydrate(tp.n - 1) };
      lastT = lastT === null ? tSec : Math.max(lastT, tSec);
      if (firstT === null) firstT = tSec;
      evict();
      emit();
    },

    range() {
      if (lastT === null) return { t0: 0, t1: 0, frontier: 0 };
      let first = null;
      for (const ch of ring.values()) {
        for (const name in ch.topics) {
          const tp = ch.topics[name];
          if (tp.n > 0 && (first === null || tp.t[0] < first)) first = tp.t[0];
        }
        if (first !== null) break;
      }
      return { t0: first === null ? lastT : first, t1: lastT, frontier: lastT };
    },

    chunkIndex(t) { return Math.floor(t / chunkSec); },
    bounds(i) { return [i * chunkSec, (i + 1) * chunkSec]; },
    peek(i) { return ring.get(i) || null; },
    async load(i) { return ring.get(i) || null; },

    async window(wt0, wt1, topics) {
      const out = [];
      const a = self.chunkIndex(wt0);
      const b = self.chunkIndex(wt1);
      for (const [i, ch] of ring) if (i >= a && i <= b) out.push(pickTopics(ch, topics));
      return out;
    },

    async latestBefore(t, topic) {
      const idxs = Array.from(ring.keys()).filter((i) => i <= self.chunkIndex(t)).reverse();
      for (const i of idxs) {
        const row = topicRow(ring.get(i), topic, t);
        if (row) return row;
      }
      const ev = evictedLast[topic];
      if (ev && ev.t <= t) return { t: ev.t, msg: ev.msg };
      const lm = last[topic];
      if (lm && lm.t <= t) return { t: lm.t, msg: lm.msg };
      return null;
    },

    async timeline() {
      const chunks = Array.from(ring.values());
      const segments = [];
      for (const ch of chunks) {
        const tp = ch.topics['/orchestrator_state'];
        if (!tp || tp.n < 1) continue;
        const col = tp.cols['data'];
        for (let k = 0; k < tp.n; k++) {
          const v = String(col[k]);
          const prev = segments[segments.length - 1];
          if (prev && prev[2] === v) prev[1] = tp.t[k];
          else {
            if (prev) prev[1] = tp.t[k];
            segments.push([tp.t[k], tp.t[k], v]);
          }
        }
      }
      if (segments.length && lastT !== null) segments[segments.length - 1][1] = lastT;
      const r = self.range();
      return {
        format: 1, t0: r.t0, t1: r.t1,
        bands: [{ topic: '/orchestrator_state', segments }],
        ticks: [],
        presence: presenceOf(chunks),
      };
    },

    // A frozen snapshot is a COMPLETE source (range().frontier == t1) so the engine pauses at t1
    // instead of buffering at its frontier forever (U6 decision, mirrors a finished recording).
    status() {
      return frozen
        ? { state: 'complete', chunksDone: ring.size, chunksTotal: ring.size }
        : { state: 'live', chunksDone: ring.size, chunksTotal: null };
    },
    onChange(cb) { listeners.add(cb); return () => listeners.delete(cb); },

    /** Number of chunks currently in the ring. */
    chunkCount() { return ring.size; },

    /**
     * Frozen read-only copy for replay: sealed chunks are shared by reference
     * (immutable), the open chunk is copied, so later record()/eviction on the
     * live buffer cannot change what the replay sees.
     */
    snapshot() {
      const s = SessionBufferSource({ horizonSec: horizon, chunkSec, _frozen: true });
      s._adopt(ring, openIdx, lastT, firstT, last, evictedLast);
      return s;
    },

    _adopt(srcRing, srcOpen, lt, ft, lastMap, evMap) {
      ring = new Map();
      for (const [i, ch] of srcRing) ring.set(i, i === srcOpen && !ch.sealed ? copyChunk(ch) : ch);
      openIdx = null;
      lastT = lt;
      firstT = ft;
      last = Object.assign({}, lastMap);
      evictedLast = Object.assign({}, evMap);
      frozen = true;
    },

    clear() {
      if (frozen) return;
      ring = new Map();
      openIdx = null;
      lastT = null;
      firstT = null;
      last = {};
      evictedLast = {};
      emit();
    },

    close() { listeners.clear(); },
  };
  return self;
}

/** Factory alias: the live session buffer the ros-bridge tap records into. */
export function createSessionBuffer(opts) {
  return SessionBufferSource(Object.assign({ horizonSec: SESSION_BUFFER_SEC, chunkSec: CHUNK_S }, opts || {}));
}
