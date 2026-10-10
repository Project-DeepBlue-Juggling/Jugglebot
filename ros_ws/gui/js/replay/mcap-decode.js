// Direct MCAP decode core (Phase 4, design § 2). Pure ESM: no worker or DOM
// globals, so node imports it for the Python-oracle test and the worker wraps it.
// Builds the schema.py chunk record ({t, cols} per topic) for one 10 s slot
// straight from an indexed, uncompressed rosbag2 MCAP.
import { McapIndexedReader, parse, MessageReader } from '../../lib/mcap-bundle.min.js';
import { buildColumns } from './chunk.js';

// The typed columnar layout (buildColumn/buildColumns) lives in chunk.js beside its readers.
export { buildColumn, buildColumns } from './chunk.js';

export const CHUNK_S = 10;
const MS_NS = 1_000_000n;
const SLOT_NS = 10_000_000_000n;

export class McapError extends Error {
  constructor(reason, message) { super(message || reason); this.reason = reason; this.status = 'refused'; }
}

// CPython float.__floordiv__ then int(): mirrors schema.chunk_index bit-for-bit.
export function slotOf(t, t0) {
  const a = t - t0, b = CHUNK_S;
  let mod = a % b;
  let div = (a - mod) / b;
  if (mod && mod < 0) { mod += b; div -= 1; }
  let f;
  if (div) { f = Math.floor(div); if (div - f > 0.5) f += 1; } else { f = 0; }
  return Math.trunc(f);
}

function plain(v) {
  if (typeof v === 'bigint') return Number(v);
  if (ArrayBuffer.isView(v)) return Array.from(v, plain);
  if (Array.isArray(v)) return v.map(plain);
  if (v && typeof v === 'object') {
    const o = {};
    for (const k of Object.keys(v)) if (!k.startsWith('__')) o[k] = plain(v[k]);
    if ('sec' in o && 'nsec' in o && !('nanosec' in o)) { o.nanosec = o.nsec; delete o.nsec; }
    return o;
  }
  return v;
}

// schema.py flattening rule: nested messages recurse with dotted paths, time /
// duration expand to .sec/.nanosec, arrays are one column, constants and
// `__` names are never columns.
export function flatten(msg, defs) {
  const byName = new Map(defs.map((d) => [d.name, d]));
  const out = {};
  const walk = (def, val, prefix) => {
    for (const f of def.definitions) {
      if (f.isConstant || f.name.startsWith('__')) continue;
      const path = prefix ? prefix + '.' + f.name : f.name;
      if ((f.type === 'time' || f.type === 'duration') && !f.isArray) {
        const x = val[f.name];
        out[path + '.sec'] = plain(x.sec);
        out[path + '.nanosec'] = plain(x.nsec ?? x.nanosec);
      } else if (f.isComplex && !f.isArray) walk(byName.get(f.type), val[f.name], path);
      else out[path] = plain(val[f.name]);
    }
  };
  walk(defs[0], msg, '');
  return out;
}

const nsOf = (t) => BigInt(Math.round(t * 1e9));

// readable: {size():Promise<bigint>, read(off:bigint,len:bigint):Promise<Uint8Array>, prefetch?(off,len)}
export async function openRecording(readable, allowlist) {
  const allow = new Set(allowlist);
  let reader;
  try { reader = await McapIndexedReader.Initialize({ readable }); }
  catch (e) {
    if (e && e.reason) throw e;
    throw new McapError('no_index', 'no MCAP summary: ' + (e && e.message));
  }
  for (const ci of reader.chunkIndexes) {
    if (ci.compression !== '') throw new McapError('compressed', 'chunk compression ' + ci.compression);
  }
  const decoders = new Map();
  const skipped = {};
  const topics = {};
  const counts = reader.statistics ? reader.statistics.channelMessageCounts : new Map();
  for (const [id, ch] of reader.channelsById) {
    const sch = reader.schemasById.get(ch.schemaId);
    const n = Number(counts.get(id) ?? 0n);
    if (!allow.has(ch.topic)) continue;
    if (!sch || sch.encoding !== 'ros2msg' || ch.messageEncoding !== 'cdr') {
      if (n >= 1) { const e = skipped[ch.topic] ??= { type: sch ? sch.name : '', count: 0, why: 'unsupported encoding' }; e.count += n; }
      continue;
    }
    try {
      const defs = parse(new TextDecoder().decode(sch.data), { ros2: true });
      decoders.set(id, { topic: ch.topic, type: sch.name, defs, reader: new MessageReader(defs) });
      if (n >= 1) { const e = topics[ch.topic] ??= { type: sch.name, count: 0 }; e.count += n; }
    } catch (e) {
      if (n >= 1) { const s = skipped[ch.topic] ??= { type: sch.name, count: 0, why: 'schema: ' + (e && e.message) }; s.count += n; }
    }
  }
  const names = [...new Set([...decoders.values()].map((d) => d.topic))];
  let first = null, last = null;
  if (names.length) {
    for await (const m of reader.readMessages({ topics: names, validateCrcs: false })) { if (decoders.has(m.channelId)) { first = m; break; } }
    for await (const m of reader.readMessages({ topics: names, reverse: true, validateCrcs: false })) { if (decoders.has(m.channelId)) { last = m; break; } }
  }
  if (!first) throw new McapError('empty', 'no allow-listed messages');
  const t0 = Number(first.logTime) / 1e9, t1 = Number(last.logTime) / 1e9;
  return {
    reader, readable, decoders, names, t0Ns: first.logTime, t0, t1, slots: slotOf(t1, t0) + 1,
    topics, skipped, chunkCount: reader.chunkIndexes.length,
  };
}

function slotSpan(rec, loNs, hiNs) {
  let a = null, b = null;
  for (const ci of rec.reader.chunkIndexes) {
    if (ci.messageEndTime < loNs || ci.messageStartTime > hiNs) continue;
    const idx = [...ci.messageIndexOffsets.values()];
    const start = ci.chunkStartOffset;
    const end = idx.length ? idx.reduce((m, x) => (x < m ? x : m)) + ci.messageIndexLength : start + ci.chunkLength;
    if (a === null || start < a) a = start;
    if (b === null || end > b) b = end;
  }
  return a === null ? null : [a, b];
}


// Every transferable ArrayBuffer under a decoded topic record (deduped), for postMessage's transfer list.
export function topicBuffers(topic, out = new Set()) {
  out.add(topic.t.buffer);
  for (const k in topic.cols) {
    const c = topic.cols[k];
    if (ArrayBuffer.isView(c)) out.add(c.buffer);
    else if (c && c.offsets) {
      out.add(c.offsets.buffer);
      if (ArrayBuffer.isView(c.flat)) out.add(c.flat.buffer);
      if (c.leaves) for (const l in c.leaves) if (ArrayBuffer.isView(c.leaves[l])) out.add(c.leaves[l].buffer);
    }
  }
  return out;
}

// Decode slot i into {topics:{"/x":{type,n,t:Float64Array,cols,kinds}}, dropped, bytes}.
export async function decodeSlot(rec, i) {
  if (!(i >= 0 && i < rec.slots)) throw new RangeError('slot ' + i + ' outside [0,' + rec.slots + ')');
  const lo = rec.t0Ns + BigInt(i) * SLOT_NS - MS_NS, hi = rec.t0Ns + BigInt(i + 1) * SLOT_NS + MS_NS;
  const span = slotSpan(rec, lo, hi);
  if (span && rec.readable.prefetch) await rec.readable.prefetch(span[0], span[1] - span[0]);
  const acc = {};
  const dropped = {};
  for await (const m of rec.reader.readMessages({ topics: rec.names, startTime: lo, endTime: hi, validateCrcs: false })) {
    const d = rec.decoders.get(m.channelId);
    if (!d) continue;
    const t = Number(m.logTime) / 1e9;
    if (slotOf(t, rec.t0) !== i) continue;
    let f;
    try { f = flatten(d.reader.readMessage(m.data), d.defs); }
    catch (e) { dropped[d.topic] = (dropped[d.topic] || 0) + 1; continue; }
    const o = acc[d.topic] ??= { type: d.type, n: 0, ts: [], cols: {} };
    o.n++; o.ts.push(t);
    for (const k of Object.keys(f)) (o.cols[k] ??= []).push(f[k]);
  }
  const topics = {};
  for (const name of Object.keys(acc).sort()) {
    const o = acc[name];
    const { cols, kinds } = buildColumns(o.cols);
    topics[name] = { type: o.type, n: o.n, t: Float64Array.from(o.ts), cols, kinds };
  }
  return { i, t0: rec.t0 + i * CHUNK_S, t1: rec.t0 + (i + 1) * CHUNK_S, topics, dropped,
    bytes: span ? Number(span[1] - span[0]) : 0 };
}

// Newest message of `topic` at or before t, found by a reverse read (one chunk at any distance).
export async function latestRow(rec, topic, t) {
  const end = nsOf(t) + MS_NS;
  for await (const m of rec.reader.readMessages({ topics: [topic], endTime: end, reverse: true, validateCrcs: false })) {
    const d = rec.decoders.get(m.channelId);
    if (!d) continue;
    const tm = Number(m.logTime) / 1e9;
    if (tm > t) continue;
    try { return { t: tm, flat: flatten(d.reader.readMessage(m.data), d.defs) }; } catch (e) { return null; }
  }
  return null;
}

const REASONS = { 409: null, 412: 'changed' };

// IReadable over HTTP Range (design § 2). Keeps the 2 most recent prefetched spans.
export function httpReadable(url, { fetch: f = globalThis.fetch } = {}) {
  let etag = null, total = null;
  const spans = []; // {off:number, buf:Uint8Array}
  const stats = { requests: 0, heads: 0, bytes: 0 };
  async function fail(resp) {
    if (resp.status === 409) {
      // The size probe is a HEAD (no body), so the server also sends the reason as X-Replay-Reason.
      let raw = null;
      try { raw = resp.headers && resp.headers.get ? resp.headers.get('X-Replay-Reason') : null; } catch (e) { /* none */ }
      if (!raw) { try { const j = await resp.json(); raw = j && j.reason; } catch (e) { /* HEAD: no body */ } }
      throw new McapError(raw === 'no_index' ? 'no_index' : (!raw || raw === 'recording_in_progress') ? 'in_progress' : raw);
    }
    if (REASONS[resp.status]) throw new McapError(REASONS[resp.status]);
    throw new McapError('http', 'HTTP ' + resp.status);
  }
  async function range(off, len) {
    const headers = { Range: 'bytes=' + off + '-' + (off + len - 1) };
    if (etag) headers['If-Match'] = etag;
    stats.requests++;
    let r;
    try { r = await f(url, { headers }); } catch (e) { throw new McapError('http', String((e && e.message) || e)); }
    if (r.status !== 206 && r.status !== 200) await fail(r);
    const e = r.headers.get('ETag');
    if (etag && e && e !== etag) throw new McapError('changed');
    const buf = new Uint8Array(await r.arrayBuffer());
    stats.bytes += buf.length;
    return buf;
  }
  const covering = (off, len) => spans.find((s) => off >= s.off && off + len <= s.off + s.buf.length);
  return {
    stats,
    async size() {
      if (total !== null) return BigInt(total);
      stats.heads++;
      let r;
      try { r = await f(url, { method: 'HEAD' }); } catch (e) { throw new McapError('http', String((e && e.message) || e)); }
      if (!r.ok) await fail(r);
      etag = r.headers.get('ETag');
      total = Number(r.headers.get('Content-Length'));
      return BigInt(total);
    },
    async read(offB, lenB) {
      const off = Number(offB), len = Number(lenB);
      const s = covering(off, len);
      if (s) return s.buf.subarray(off - s.off, off - s.off + len);
      return range(off, len);
    },
    async prefetch(offB, lenB) {
      const off = Number(offB), len = Number(lenB);
      if (covering(off, len)) return;
      spans.push({ off, buf: await range(off, len) });
      while (spans.length > 2) spans.shift();
    },
  };
}
