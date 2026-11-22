// span_events_harness.js - the REAL replay/span-events.js (+ ball-events.js, event-store.js, clock.js) under node. Sandbox built by
// tests/ros/test_gui_span_events.py.
import { scanBallEvents } from './replay/span-events.js';
import * as store from './event-store.js';

const T0 = 1795175800;
const ts = (s) => ({ sec: Math.floor(T0 + s), nanosec: Math.round(((T0 + s) % 1) * 1e9) });
const ball = (o) => Object.assign({
  id: 1, status: 1, source: 'jugglebot', destination: 'jugglebot', landing_from_fit: true,
  header: { stamp: ts(0) }, throw_time: ts(0), time_at_land: ts(0),
}, o);

// A 40 s recording, 10 s chunks. Ball i is thrown at 4 + 3i and caught 1.2 s later (i = 0..11 -> 4 .. 37.2 s).
const msgs = [];                                     // {t, balls}
for (let i = 0; i < 12; i++) {
  const rel = 4 + 3 * i, land = rel + 1.2;
  for (let k = 0; k <= 20; k++) {                    // 10 Hz while the ball is in the air and just after
    const t = rel + 0.1 * k + 0.1;
    const caught = t > land + 0.3;
    msgs.push({ t: T0 + t, balls: [ball({ id: i + 1, status: caught ? 2 : 1, throw_time: ts(rel), time_at_land: ts(land) })] });
  }
}
msgs.sort((a, b) => a.t - b.t);
const calls = [];
const source = {
  range: () => ({ t0: T0, t1: T0 + 40, frontier: T0 + 40 }),
  chunkIndex: (t) => Math.max(0, Math.floor((t - T0) / 10)),
  load: async (i, opts) => {
    calls.push({ i, lite: !!(opts && opts.lite), topics: opts && opts.topics });
    const rows = msgs.filter((m) => m.t >= T0 + i * 10 && m.t < T0 + (i + 1) * 10);
    return { i, topics: { '/balls': { n: rows.length, t: Float64Array.from(rows.map((r) => r.t)), hydrate: (k) => ({ balls: rows[k].balls }) } } };
  },
};
const out = {};
const yieldFn = async () => {};

// 1. a span in the middle of the recording: every Throw and Catch inside, none outside, regardless of any playhead
const mid = await scanBallEvents(source, T0 + 10, T0 + 25, { yieldFn });
out.midThrows = mid.filter((e) => e.type === 'throw').map((e) => Math.round(e.t - T0));
out.midCatches = mid.filter((e) => e.type === 'catch').map((e) => Math.round((e.t - T0) * 10) / 10);
out.midSorted = mid.every((e, i) => i === 0 || mid[i - 1].t <= e.t);
out.onlyBallsLite = calls.every((c) => c.lite && c.topics.length === 1 && c.topics[0] === '/balls');
out.chunksTouched = Array.from(new Set(calls.map((c) => c.i))).sort();

// 2. the whole recording: 12 + 12
const all = await scanBallEvents(source, T0, T0 + 40, { yieldFn });
out.allCounts = [all.filter((e) => e.type === 'throw').length, all.filter((e) => e.type === 'catch').length];

// 3. cancellation returns null; an empty / reversed span returns []
out.cancelled = await scanBallEvents(source, T0, T0 + 40, { yieldFn, isCancelled: () => true });
out.reversed = (await scanBallEvents(source, T0 + 10, T0, { yieldFn })).length;

// 4. event store: span markers are merged only on request, deduped against the ring, and never reach the Event Log
store.snapshotAndBeginReplayEvents();
store.emitEvent({ type: 'throw', label: all[0].label, t: all[0].t });          // the past, as the playhead emitted it
store.emitEvent({ type: 'command', label: 'Home', t: T0 + 1 });
store.setSpanEvents(all);
const plain = store.getEventsInRange(T0, T0 + 40);
const spanned = store.getEventsInRange(T0, T0 + 40, { span: true });
out.plainCount = plain.length;
out.spannedCount = spanned.length;                                              // 24 + the command, the shared first Throw counted once
out.spannedSorted = spanned.every((e, i) => i === 0 || spanned[i - 1].t <= e.t);
out.logTypes = store.getRecentEvents(null, 50).map((e) => e.type);
out.spanIdsUnique = new Set(spanned.map((e) => e.id)).size === spanned.length;
store.trimEventsAfter(T0 + 2);                                                  // a backward seek leaves the span list alone
out.afterTrimSpanned = store.getEventsInRange(T0, T0 + 40, { span: true }).length;
store.setSpanEvents([]);
out.cleared = store.getEventsInRange(T0, T0 + 40, { span: true }).length;
store.setSpanEvents(all);
store.restoreEvents({ events: [], chartEvents: [] });                           // leaving replay drops the span list
out.afterRestore = store.getEventsInRange(T0, T0 + 40, { span: true }).length;
process.stdout.write(JSON.stringify(out));
