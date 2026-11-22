// event_store_harness.js - the REAL ros_ws/gui/js/event-store.js (+ clock.js) under node. Sandbox built by tests/ros/test_gui_event_store.py.
import * as store from './event-store.js';

const out = {};
const T = 1795175000;

// 1. 600 Throw/Catch events must not evict an earlier command; the chart query still returns the newest chart-only ones
store.emitEvent({ type: 'command', label: 'Home', t: T });
store.emitEvent({ type: 'fault', label: 'Fault X', t: T + 1 });
for (let i = 0; i < 600; i++) store.emitEvent({ type: i % 2 ? 'catch' : 'throw', label: 'B ' + i, t: T + 10 + i * 0.1 });
const list = store.getRecentEvents(null, 500);
out.listTypes = list.map((e) => e.type);
const chart = store.getEventsInRange(T - 1, T + 1000);
out.chartCount = chart.length;
out.chartHasCommand = chart.some((e) => e.type === 'command');
out.chartNewest = chart[chart.length - 1].label;
out.chartSorted = chart.every((e, i) => i === 0 || chart[i - 1].t <= e.t);
out.chartOnlyKept = chart.filter((e) => e.type === 'throw' || e.type === 'catch').length;

// 2. replay: snapshot parks both rings; trim handles out-of-order t (a Catch is stamped before its emission) in both rings
const snap = store.snapshotAndBeginReplayEvents();
out.afterBegin = store.getEventsInRange(0, 1e12).length;
store.emitEvent({ type: 'command', label: 'Late', t: T + 100 });
store.emitEvent({ type: 'catch', label: 'Early catch', t: T + 50 });     // emitted later, stamped earlier
store.emitEvent({ type: 'command', label: 'Mid', t: T + 60 });            // later t AFTER an earlier-t chart event
store.emitEvent({ type: 'command', label: 'Early', t: T + 40 });          // last ordinary entry has the SMALLEST t
store.trimEventsAfter(T + 55);
out.trimmed = store.getEventsInRange(0, 1e12).map((e) => e.label);
// the trimmed events can be replayed again (dedup keys were released)
store.emitEvent({ type: 'command', label: 'Late', t: T + 100 });
out.replayedAgain = store.getEventsInRange(0, 1e12).map((e) => e.label);
store.restoreEvents(snap);
out.restored = store.getEventsInRange(T - 1, T + 1000).length;
out.restoredListLen = store.getRecentEvents(null, 500).length;

// 3. the subscriber sees chart-only events with their type, and the cleared notification as null
const seen = [];
store.subscribeEvents((e) => seen.push(e ? e.type : null));
store.emitEvent({ type: 'throw', label: 'x', t: T + 2000 });
store.clearEvents();
out.seen = seen;
out.afterClear = store.getEventsInRange(0, 1e12).length;

console.log(JSON.stringify(out));
