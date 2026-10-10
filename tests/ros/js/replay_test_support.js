// Shared node-harness helpers for the replay source tests: oracle chunk JSON loading and a scripted
// fake module worker that speaks the mcap-worker.js protocol over those chunks.
// Copied verbatim into each test sandbox beside the shipped modules.
import fs from 'fs';
import path from 'path';

/** Undo tests/ros/_replay_chunks.py's {"$f": ...} non-finite tags. */
export function revive(v) {
  if (Array.isArray(v)) return v.map(revive);
  if (v && typeof v === 'object') {
    const ks = Object.keys(v);
    if (ks.length === 1 && ks[0] === '$f') return v.$f === 'nan' ? NaN : (v.$f === 'inf' ? Infinity : -Infinity);
    const o = {};
    for (const k of ks) o[k] = revive(v[k]);
    return o;
  }
  return v;
}

export function loadRecords(dir) {
  const manifest = JSON.parse(fs.readFileSync(path.join(dir, 'manifest.json'), 'utf8'));
  const records = [];
  for (let i = 0; i < manifest.chunks.length; i++) {
    const f = path.join(dir, 'chunk-' + String(i).padStart(5, '0') + '.json');
    records.push(revive(JSON.parse(fs.readFileSync(f, 'utf8'))));
  }
  const overview = fs.existsSync(path.join(dir, 'overview.json'))
    ? JSON.parse(fs.readFileSync(path.join(dir, 'overview.json'), 'utf8')) : null;
  return { manifest, records, overview };
}

/**
 * A fake worker over oracle records. opts: {openError?: code, loadError?: {i: code}, hold?: Promise (gate
 * every `load` until it resolves), log?: array (records every posted op)}.
 * Replies are asynchronous (microtask), like a real worker.
 */
export function fakeWorkerFactory(manifest, records, opts) {
  opts = opts || {};
  const topics = {};
  for (const r of records) {
    for (const name in r.topics) {
      const e = topics[name] || (topics[name] = { type: r.topics[name].type, count: 0 });
      e.count += r.topics[name].n;
    }
  }
  return function makeWorker() {
    const w = {
      terminated: false,
      onmessage: null,
      postMessage(msg) {
        if (opts.log) opts.log.push(msg.op + (msg.i !== undefined ? ':' + msg.i : ''));
        Promise.resolve().then(async () => {
          if (w.terminated) return;
          const reply = (m) => { if (!w.terminated && w.onmessage) w.onmessage({ data: Object.assign({ req: msg.req }, m) }); };
          if (msg.op === 'open') {
            if (opts.openError) return reply({ op: 'error', code: opts.openError, message: opts.openError });
            return reply({ op: 'opened', t0: manifest.t0, t1: manifest.t1, slots: records.length, topics,
              skipped: {}, chunkCount: records.length * 4, size: 1, etag: null, ms: 1 });
          }
          if (msg.op === 'load') {
            if (opts.hold) await opts.hold;
            if (opts.loadError && opts.loadError[msg.i]) return reply({ op: 'error', code: opts.loadError[msg.i], message: 'x' });
            const r = records[msg.i];
            const tp = {};
            for (const name in r.topics) {
              tp[name] = { type: r.topics[name].type, n: r.topics[name].n,
                t: Float64Array.from(r.topics[name].t), cols: r.topics[name].cols };
            }
            return reply({ op: 'slot', i: r.i, t0: r.t0, t1: r.t1, topics: tp, dropped: {}, bytes: 0, ms: 1 });
          }
          if (msg.op === 'latest') {
            for (let i = records.length - 1; i >= 0; i--) {
              const tp = records[i].topics[msg.topic];
              if (!tp) continue;
              for (let k = tp.t.length - 1; k >= 0; k--) {
                if (tp.t[k] <= msg.t) {
                  const flat = {};
                  for (const c in tp.cols) flat[c] = tp.cols[c][k];
                  return reply({ op: 'latest', row: { t: tp.t[k], flat } });
                }
              }
            }
            return reply({ op: 'latest', row: null });
          }
        });
      },
      terminate() { w.terminated = true; },
    };
    return w;
  };
}
