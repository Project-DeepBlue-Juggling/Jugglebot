// Node harness for tests/ros/test_replay_mcap_oracle.py: decode an MCAP directly
// with js/replay/mcap-decode.js (the browser worker's decode core, no worker).
//   node mcap_oracle_harness.js <bag.mcap> <out.json> [file|http] [--vector]
// `http` serves the same file through a fake fetch (Range/ETag semantics) so
// httpReadable is pinned against the file readable and its request count is reported.
import fs from 'node:fs';
import { openRecording, decodeSlot, latestRow, slotOf, httpReadable } from './js/replay/mcap-decode.js';
import { ALLOWLIST } from './js/replay/allowlist.js';

const [, , bag, out, mode = 'file'] = process.argv;

function fileReadable(path) {
  const fd = fs.openSync(path, 'r');
  const size = fs.fstatSync(fd).size;
  return {
    size: async () => BigInt(size),
    read: async (off, len) => {
      const b = Buffer.alloc(Number(len));
      fs.readSync(fd, b, 0, Number(len), Number(off));
      return new Uint8Array(b.buffer, b.byteOffset, b.length);
    },
  };
}

function fakeFetch(path) {
  const data = fs.readFileSync(path);
  const etag = '"' + data.length + '-1"';
  return async (url, init = {}) => {
    const h = new Map([['ETag', etag], ['Content-Length', String(data.length)]]);
    const headers = { get: (k) => h.get(k) ?? null };
    if (init.method === 'HEAD') return { ok: true, status: 200, headers };
    const m = /bytes=(\d+)-(\d+)/.exec(init.headers.Range);
    const a = Number(m[1]), b = Math.min(Number(m[2]), data.length - 1);
    const body = data.subarray(a, b + 1);
    const ab = body.buffer.slice(body.byteOffset, body.byteOffset + body.length);
    return { ok: true, status: 206, headers, arrayBuffer: async () => ab, json: async () => ({}) };
  };
}

// NaN/Inf survive JSON as {"$f": ...}; everything else is plain JSON.
const tag = (k, v) => {
  if (typeof v === 'number' && !Number.isFinite(v)) return { $f: Number.isNaN(v) ? 'nan' : (v > 0 ? 'inf' : '-inf') };
  return v;
};

const readable = mode === 'http' ? httpReadable('http://fake/file', { fetch: fakeFetch(bag) }) : fileReadable(bag);
const t0 = process.hrtime.bigint();
const rec = await openRecording(readable, ALLOWLIST);
const openMs = Number(process.hrtime.bigint() - t0) / 1e6;
const res = {
  opened: { t0: rec.t0, t1: rec.t1, slots: rec.slots, topics: rec.topics, skipped: rec.skipped, chunkCount: rec.chunkCount },
  slots: [], openMs, decodeMs: 0,
};
for (let i = 0; i < rec.slots; i++) {
  const s0 = process.hrtime.bigint();
  const s = await decodeSlot(rec, i);
  res.decodeMs += Number(process.hrtime.bigint() - s0) / 1e6;
  const topics = {};
  for (const [name, o] of Object.entries(s.topics)) topics[name] = { type: o.type, n: o.n, t: Array.from(o.t), cols: o.cols };
  res.slots.push({ i, t0: s.t0, t1: s.t1, topics, dropped: s.dropped });
}
// latest-row spot checks: a rarely published topic far back, and a miss before the first message
res.latest = {};
for (const [topic, t] of [['/skills/attempt', rec.t0 + 34], ['/cone/catch_event', rec.t0 + 30], ['/skills/attempt', rec.t0 + 1]]) {
  res.latest[topic + '@' + (t - rec.t0).toFixed(0)] = await latestRow(rec, topic, t);
}
if (process.argv.includes('--vector')) {
  res.vector = [];
  for (const [t, t0v] of [[1791532610.0, 1791532600.0], [1.0000000000000002e9 + 10, 1e9], [0.3, 0.1], [29.999999999, 0], [30, 0], [9.999999999999998, 0], [1791532609.9999998, 1791532600.003], [1791532630.003, 1791532600.003], [40, 0], [-0.5, 0]]) res.vector.push([t, t0v, slotOf(t, t0v)]);
}
if (readable.stats) res.http = readable.stats;
fs.writeFileSync(out, JSON.stringify(res, tag));
