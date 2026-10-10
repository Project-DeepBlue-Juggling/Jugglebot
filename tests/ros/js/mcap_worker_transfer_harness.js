// Node harness for the worker transfer test: drive the REAL createWorkerHandler over a real bag file. `post` does
// what Worker.postMessage does - structuredClone with the transfer list - so a buffer missing from the list is
// COPIED (still attached after post) and a listed one is DETACHED on the worker side.
//   node mcap_worker_transfer_harness.js <bag.mcap> <slot>
import fs from 'node:fs';
import { createWorkerHandler } from './js/replay/mcap-worker.js';
import { chunkFromRecord } from './js/replay/chunk.js';

const [, , bag, slotArg] = process.argv;
const fd = fs.openSync(bag, 'r');
const size = fs.fstatSync(fd).size;
const readable = {
  size: async () => BigInt(size),
  read: async (off, len) => { const b = Buffer.alloc(Number(len)); fs.readSync(fd, b, 0, Number(len), Number(off)); return new Uint8Array(b.buffer, b.byteOffset, b.length); },
};
let sent = null, transferList = null, received = null;
const h = createWorkerHandler({
  makeReadable: () => readable,
  post: (m, transfer) => {
    if (m.op === 'slot') { sent = m; transferList = transfer || []; received = structuredClone(m, { transfer: transfer || [] }); }
  },
});
await h.handle({ op: 'open', req: 1, url: 'x' });
await h.handle({ op: 'load', req: 2, i: Number(slotArg) });

const typedBuffers = new Set(), allViews = [];
const visit = (v) => { if (ArrayBuffer.isView(v)) { allViews.push(v); typedBuffers.add(v.buffer); } };
for (const tp of Object.values(sent.topics)) {
  visit(tp.t);
  for (const c of Object.values(tp.cols)) {
    if (ArrayBuffer.isView(c)) visit(c);
    else if (c && c.offsets) { visit(c.offsets); visit(c.flat); for (const l of Object.values(c.leaves || {})) visit(l); }
  }
}
const ch = chunkFromRecord({ i: received.i, t0: received.t0, t1: received.t1, topics: received.topics });
let hydrated = 0;
for (const tp of Object.values(ch.topics)) for (let k = 0; k < tp.n; k++) { tp.hydrate(k); hydrated++; }
console.log(JSON.stringify({
  typedBufferCount: typedBuffers.size,
  transferCount: transferList.length,
  everyTypedBufferListed: [...typedBuffers].every((b) => transferList.includes(b)),
  senderViewsDetached: allViews.filter((v) => v.byteLength === 0).length,
  senderViewsTotal: allViews.length,
  receivedTypedBytes: Object.values(received.topics).reduce((a, tp) => a + tp.t.byteLength, 0),
  hydratedRows: hydrated,
  adopted: Object.entries(ch.topics).every(([n, tp]) => tp.t === received.topics[n].t && tp.cols === received.topics[n].cols),
}));
