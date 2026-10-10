// Node harness for the two-lane FIFO of mcap-worker.js (createWorkerHandler) over a STUB mcap-decode.js whose
// decodeSlot is gated by the harness (the sandbox writes the stub beside the real mcap-worker.js).
// Usage: node replay_digest_lane_harness.js    (prints one JSON object)
const { createWorkerHandler } = await import('./mcap-worker.js');
const stub = await import('./mcap-decode.js');

const posted = [];
const h = createWorkerHandler({ makeReadable: () => ({ size: async () => 1 }), post: (m) => posted.push(m), allowlist: [] });
const out = {};
const tick = () => new Promise((r) => setTimeout(r, 0));

await h.handle({ op: 'open', req: 1, url: 'x' });
// same-tick posting: lite 7, full 2, lite 8, full 3, latest -> full lane first (FIFO), lites after, FIFO among themselves
stub.state.started.length = 0; stub.state.gated = true;
const ps = [
  h.handle({ op: 'load', req: 10, i: 7, lite: true }),
  h.handle({ op: 'load', req: 11, i: 2 }),
  h.handle({ op: 'load', req: 12, i: 8, lite: true }),
  h.handle({ op: 'load', req: 13, i: 3 }),
  h.handle({ op: 'latest', req: 14, topic: '/t', t: 1 }),
];
for (let n = 0; n < 6; n++) { await tick(); stub.state.releaseAll(); }
await Promise.all(ps);
out.same_tick = stub.state.started.slice();
out.replies = posted.filter((m) => m.op === 'slot').map((m) => m.i);

// a lite already RUNNING is not preempted; a full posted meanwhile runs before the NEXT lite
stub.state.started.length = 0; posted.length = 0;
const a = h.handle({ op: 'load', req: 20, i: 5, lite: true });
await tick();
const b = h.handle({ op: 'load', req: 21, i: 6, lite: true });
const c = h.handle({ op: 'load', req: 22, i: 1 });
await tick();
out.running_lite_first = stub.state.started.slice();   // [5] only: gated, running
for (let n = 0; n < 6; n++) { await tick(); stub.state.releaseAll(); }
await Promise.all([a, b, c]);
out.then_full_before_next_lite = stub.state.started.slice();
out.reply_ops = posted.map((m) => m.op + ':' + (m.i === undefined ? '' : m.i));
process.stdout.write(JSON.stringify(out));
