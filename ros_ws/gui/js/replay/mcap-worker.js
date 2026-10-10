// Module Web Worker over mcap-decode.js (Phase 4 design § 2). One indexed
// reader per `open`; a single async FIFO, so reader state and byte spans never
// race (no cancellation: cache.js fetches one slot at a time).
//
//   main -> worker: {op:'open', req, url} {op:'load', req, i} {op:'latest', req, topic, t} {op:'close'}
//   worker -> main: {op:'opened', req, t0, t1, slots, topics, skipped, chunkCount, size, etag, ms}
//                   {op:'slot', req, i, t0, t1, topics:{"/x":{type,n,t,cols}}, dropped, bytes, ms}
//                   {op:'latest', req, row:{t, flat}|null}
//                   {op:'error', req, code, message}
//
// `createWorkerHandler` is exported so node tests drive the same handler inline;
// the global onmessage binds only inside a real worker scope.
import { ALLOWLIST } from './allowlist.js';
import { openRecording, decodeSlot, latestRow, httpReadable } from './mcap-decode.js';

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
  let tail = Promise.resolve();

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
        const transfer = [];
        for (const name in s.topics) transfer.push(s.topics[name].t.buffer);
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

  return {
    handle(msg) {
      tail = tail.then(() => run(msg));
      return tail;
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
