/**
 * Replay feed sources: one read interface over a recording read directly from its
 * MCAP in a module worker (McapSource).
 *
 *   source.kind                      'recording'
 *   source.range()                   sync  {t0, t1, frontier}   (frontier = end of sealed data)
 *   source.chunkIndex(t) / bounds(i) sync
 *   source.load(i, {lite}?)          async Chunk | null; lite = low-priority far-tier load (digest.js): served after
 *                                    every normal load by the worker and NEVER memoised (far slots must not stay resident)
 *   source.window(t0, t1, topics?)   async Chunk[]
 *   source.latestBefore(t, topic)    async {t, msg} | null
 *   source.timeline()                async overview-shaped object, {partial:true,...} until the overview arrives
 *   source.status()                  sync  {state, chunksDone, chunksTotal}  (a recording is always 'complete')
 *   source.onChange(cb)              -> unsubscribe
 *   source.peek(i)                   sync  resident chunk or null (tiny scan-back memo; cache.js owns playback residency)
 *
 * Chunk shape: see chunk.js. Times are wall-clock seconds.
 *
 * The recording URLs are `${baseUrl}/recordings/${id}/{file,overview}` with baseUrl
 * defaulting to '/api/replay' (gui_server.py routes).
 */
import { CHUNK_FORMAT, chunkFromRecord, indexLatestBefore, makeTopic } from './chunk.js';
import { slotOf } from './slot.js';

const CHUNK_S = 10;
// U5 decision: the chunk cache (cache.js) owns residency for playback. This memo survives only as a
// tiny scan-back buffer for latestBefore()/timeline() (which call load() directly and walk up to
// scanBackChunks = 6 chunks per call; a smaller FIFO would refetch the whole walk every time). It holds
// the same chunk objects the cache does, so it adds memory only for chunks the cache already evicted
// (<= 6 x ~10 MB), down from the 16-entry duplicate of the cache's own cap.
const MEMO_CAP = 6;

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
// Recording source: a direct MCAP reader in a module Web Worker (design § 3)
// ---------------------------------------------------------------------------

/**
 * @param {{id:string, baseUrl?:string, fetch:function, makeWorker:function,
 *          fileUrl?:string, overviewPollMs?:number,
 *          timers?:{setTimeout:function, clearTimeout:function}}} opts
 *   makeWorker() -> {postMessage(msg, transfer?), onmessage, onerror?, terminate()}; the worker
 *   answers with the mcap-worker.js protocol. fileUrl overrides the Range URL (tests).
 */
export function McapSource(opts) {
  const base = (opts.baseUrl === undefined ? '/api/replay' : opts.baseUrl) +
    '/recordings/' + encodeURIComponent(opts.id);
  const doFetch = opts.fetch;
  const pollMs = opts.overviewPollMs === undefined ? 2000 : opts.overviewPollMs;
  const timers = opts.timers || globalThis;

  let worker = null;
  let nextReq = 1;
  const pending = new Map();   // req -> {resolve, reject}
  let t0 = null;
  let t1 = null;
  let slots = 0;
  let topics = {};             // "/x" -> {type, count}
  let skipped = {};
  let dropped = {};
  let overview = null;
  let unavailable = null;
  let ovTimer = null;
  let closed = false;
  const listeners = new Set();
  const memo = new Map();      // i -> Chunk
  const inflight = new Map();  // i -> Promise<Chunk>
  const liteInflight = new Map(); // i -> Promise<Chunk> (lite loads: dedupe only, never memoised)

  function emit() { for (const cb of Array.from(listeners)) cb(self.status()); }

  function refusal(code, message) {
    const err = new Error(message || code);
    err.reason = code;
    err.status = code === 'http' || code === 'decode' || code === 'closed' ? undefined : 'refused';
    return err;
  }

  function onMessage(ev) {
    const m = ev.data;
    const p = pending.get(m.req);
    if (!p) return;
    pending.delete(m.req);
    if (m.op === 'error') p.reject(refusal(m.code, m.message));
    else p.resolve(m);
  }

  function send(msg, transfer) {
    if (closed || !worker) return Promise.reject(refusal('closed', 'replay source closed'));
    return new Promise((resolve, reject) => {
      msg.req = nextReq++;
      pending.set(msg.req, { resolve, reject });
      worker.postMessage(msg, transfer || []);
    });
  }

  function failAll(err) {
    for (const p of Array.from(pending.values())) p.reject(err);
    pending.clear();
  }

  // ---- overview: the only poller (the trackbar's retry just re-reads timeline()) ----
  async function pollOverview(attempt) {
    ovTimer = null;
    if (closed) return;
    let delay = null;
    try {
      const res = await doFetch(base + '/overview');
      if (closed) return;
      if (res.status === 200) {
        const body = await res.json();
        if (body && body.format === CHUNK_FORMAT) overview = body;
        else unavailable = 'format';
        emit();
        return;
      }
      if (res.status === 202) { delay = pollMs; attempt = 0; }
      else {
        let why = null;
        try { const b = await res.json(); why = b && (b.reason || b.status); } catch (e) { why = null; }
        unavailable = why || ('http_' + res.status);
        emit();
        return;
      }
    } catch (e) {
      delay = Math.min(pollMs * Math.pow(2, attempt), 15000); // network error: back off
      attempt++;
    }
    if (!closed) ovTimer = timers.setTimeout(() => { pollOverview(attempt); }, delay);
  }

  const self = {
    kind: 'recording',
    id: opts.id,

    /** Start the worker and read the MCAP summary; rejects with `.reason` on a refusal. */
    async open() {
      worker = opts.makeWorker();
      worker.onmessage = onMessage;
      worker.onerror = (e) => failAll(refusal('decode', 'replay worker failed: ' + (e && e.message)));
      const path = base + '/file';
      const url = opts.fileUrl || (typeof location !== 'undefined' ? new URL(path, location.href).href : path);
      const o = await send({ op: 'open', url });
      t0 = o.t0; t1 = o.t1; slots = o.slots;
      topics = o.topics || {}; skipped = o.skipped || {};
      pollOverview(0);   // the first GET enqueues the pass; not awaited, open never waits on it
      emit();
      return 'complete';
    },

    range() {
      if (t0 === null) return { t0: 0, t1: 0, frontier: 0 };
      return { t0, t1, frontier: t1 };
    },

    chunkIndex(t) { return slotOf(t, t0 || 0); },
    bounds(i) { return [(t0 || 0) + i * CHUNK_S, (t0 || 0) + (i + 1) * CHUNK_S]; },

    peek(i) { return memo.get(i) || null; },

    async load(i, loadOpts) {
      if (!(i >= 0 && i < slots)) throw new RangeError('chunk ' + i + ' outside [0,' + slots + ')');
      const hit = memo.get(i);
      if (hit) return hit;
      if (loadOpts && loadOpts.lite) {
        // A normal load already in flight serves a lite caller too; otherwise a lite lane message.
        const full = inflight.get(i);
        if (full) return full;
        let lp = liteInflight.get(i);
        if (!lp) {
          lp = (async () => {
            const s = await send({ op: 'load', i, lite: true });
            for (const name in s.dropped) dropped[name] = (dropped[name] || 0) + s.dropped[name];
            return chunkFromRecord(s);
          })();
          liteInflight.set(i, lp);
          lp.then(() => liteInflight.delete(i), () => liteInflight.delete(i));
        }
        return lp;
      }
      let p = inflight.get(i);
      const lp0 = liteInflight.get(i);
      if (lp0 && !p) {   // adopt an in-flight lite decode of this slot: one decode, not two
        p = lp0.then((ch) => { memo.set(i, ch); while (memo.size > MEMO_CAP) memo.delete(memo.keys().next().value); return ch; });
        inflight.set(i, p);
        p.then(() => inflight.delete(i), () => inflight.delete(i));
      }
      if (!p) {
        p = (async () => {
          const s = await send({ op: 'load', i });
          for (const name in s.dropped) dropped[name] = (dropped[name] || 0) + s.dropped[name];
          const ch = chunkFromRecord(s);
          memo.set(i, ch);
          while (memo.size > MEMO_CAP) memo.delete(memo.keys().next().value);
          return ch;
        })();
        inflight.set(i, p);
        p.then(() => inflight.delete(i), () => inflight.delete(i));
      }
      return p;
    },

    async window(wt0, wt1, topicList) {
      const a = Math.max(0, self.chunkIndex(wt0));
      const b = Math.min(slots - 1, self.chunkIndex(wt1));
      const out = [];
      for (let i = a; i <= b; i++) out.push(pickTopics(await self.load(i), topicList));
      return out;
    },

    async latestBefore(t, topic) {
      if (!(topic in topics) || t0 === null || t < t0) return null;
      // Memo shortcut (sync-resident slots): slot(t), then slot(t)-1 only when slot(t) is
      // resident too, so "no row <= t in slot(t)" is known rather than assumed.
      const s = Math.min(slots - 1, self.chunkIndex(t));
      const cur = memo.get(s);
      if (cur) {
        const row = topicRow(cur, topic, t);
        if (row) return row;
        const prev = memo.get(s - 1);
        if (prev) { const r2 = topicRow(prev, topic, t); if (r2) return r2; }
      }
      const m = await send({ op: 'latest', topic, t });
      if (!m.row) return null;
      const cols = {};
      for (const k in m.row.flat) cols[k] = [m.row.flat[k]];
      const tp = makeTopic(topics[topic].type, [m.row.t], cols);
      return { t: m.row.t, msg: tp.hydrate(0) };
    },

    /** Never touches the network: the source polls /overview itself and fires onChange. */
    async timeline() {
      if (overview) return overview;
      const out = { partial: true, presence: {} };
      if (unavailable) out.unavailable = unavailable;
      return out;
    },

    status() { return { state: 'complete', chunksDone: slots, chunksTotal: slots, dropped }; },
    onChange(cb) { listeners.add(cb); return () => listeners.delete(cb); },
    topicSet() { return Object.keys(topics); },
    manifest() { return { t0, t1, topics, skipped_topics: skipped }; },

    close() {
      if (closed) return;
      closed = true;
      if (ovTimer !== null) { timers.clearTimeout(ovTimer); ovTimer = null; }
      failAll(refusal('closed', 'replay source closed'));
      if (worker) {
        try { worker.postMessage({ op: 'close' }); } catch (e) { /* ignore */ }
        try { worker.terminate(); } catch (e) { /* ignore */ }
        worker = null;
      }
      listeners.clear();
      memo.clear();
    },
  };
  return self;
}
