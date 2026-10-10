// Module Web Worker over mcap-decode.js (Phase 4 design § 2). One indexed
// reader per `open`; a single async FIFO, so reader state and byte spans never
// race (no cancellation: cache.js fetches one slot at a time).
//
// Two lanes, still one reader: every message is served FIFO within its lane, and the normal lane is
// always drained before the next LITE message starts ({op:'load', lite:true}: the digester's far-tier
// loads, digest.js). A lite decode already running is not interrupted - playback waits at most one slot.
//
//   main -> worker: {op:'open', req, url} {op:'load', req, i, lite?} {op:'latest', req, topic, t} {op:'close'}
//   worker -> main: {op:'opened', req, t0, t1, slots, topics, skipped, chunkCount, size, etag, ms}
//                   {op:'slot', req, i, t0, t1, topics:{"/x":{type,n,t,cols,kinds}}, dropped, bytes, ms}
//                   {op:'latest', req, row:{t, flat}|null}
//                   {op:'error', req, code, message}
//
// `createWorkerHandler` is exported so node tests drive the same handler inline;
// the global onmessage binds only inside a real worker scope.
import { ALLOWLIST } from './allowlist.js';
import { openRecording, decodeSlot, latestRow, httpReadable, topicBuffers } from './mcap-decode.js';

const CODES = new Set(['no_index', 'compressed', 'in_progress', 'changed', 'http', 'empty', 'decode', 'range']);

/**
 * @param {{makeReadable:function(string):object, post:function(object, Transferable[]=):void,
 *          allowlist?:string[], now?:function():number}} deps
 * @returns {{handle:function(object):Promise<void>}}
 */
export function createWorkerHandler(deps) {
  const allowlist = deps.allowlist || ALLOWLIST;
  const now = deps.now || (() => (typeof performance !== 'undefined' ? performance.now() : Date.now()));   // wall-clock: decode timing diagnostics (ms) for the worker's replies, not recorded time
  let rec = null;
  const hi = [];   // normal lane (open / load / latest / close)
  const lo = [];   // lite lane (low-priority far-tier loads)
  let draining = false;

  function fail(req, e) {
    let code = e && e.reason;
    if (!CODES.has(code)) code = e instanceof RangeError ? 'range' : 'decode';
    deps.post({ op: 'error', req, code, message: String((e && e.message) || e) });
  }

  async function run(msg) {
    const t0 = now();
    try {
      if (msg.op === 'open') {
        const readable = deps.makeReadable(msg.url);
        rec = await openRecording(readable, allowlist);
        const size = Number(await readable.size());
        deps.post({
          op: 'opened', req: msg.req, t0: rec.t0, t1: rec.t1, slots: rec.slots, topics: rec.topics,
          skipped: rec.skipped, chunkCount: rec.chunkCount, size, etag: readable.etag ? readable.etag() : null,
          ms: now() - t0,
        });
      } else if (msg.op === 'load') {
        if (!rec) throw new RangeError('load before open');
        const s = await decodeSlot(rec, msg.i);
        const bufs = new Set();   // EVERY typed buffer moves (t, f64/u8 columns, CSR offsets/flat/leaves); only string/any arrays clone
        for (const name in s.topics) topicBuffers(s.topics[name], bufs);
        const transfer = [...bufs];
        deps.post({
          op: 'slot', req: msg.req, i: s.i, t0: s.t0, t1: s.t1, topics: s.topics, dropped: s.dropped,
          bytes: s.bytes, ms: now() - t0,
        }, transfer);
      } else if (msg.op === 'latest') {
        if (!rec) throw new RangeError('latest before open');
        deps.post({ op: 'latest', req: msg.req, row: await latestRow(rec, msg.topic, msg.t) });
      } else if (msg.op === 'close') {
        rec = null;
      }
    } catch (e) {
      fail(msg.req, e);
    }
  }

  async function drain() {
    try {
      while (hi.length || lo.length) {
        const item = hi.length ? hi.shift() : lo.shift();
        await run(item.msg);
        item.done();
      }
    } finally { draining = false; }
  }

  return {
    handle(msg) {
      const p = new Promise((done) => { (msg.op === 'load' && msg.lite ? lo : hi).push({ msg, done }); });
      // Start on a microtask so messages posted in the same tick are ordered by LANE, not arrival.
      if (!draining) { draining = true; Promise.resolve().then(drain); }
      return p;
    },
  };
}

if (typeof WorkerGlobalScope !== 'undefined' && typeof self !== 'undefined' && self instanceof WorkerGlobalScope) {
  const h = createWorkerHandler({
    makeReadable: (url) => httpReadable(url),
    post: (m, transfer) => self.postMessage(m, transfer || []),
  });
  self.onmessage = (ev) => { h.handle(ev.data); };
}
