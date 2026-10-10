// replay_mode_harness.js — drives the REAL replay/mode.js (+ engine, cache, sources, session, event-store,
// clock, fence, ros-bridge) under node with a stubbed ROSLIB/DOM; prints one JSON object that
// tests/ros/test_gui_replay_mode.py asserts on.
//
// Sandbox layout (built by the pytest fixture, verbatim copies): clock.js, event-store.js, ros-bridge.js,
// replay/{chunk,sources,session,policy,engine,cache,fence,mode}.js, this file, package.json {"type":"module"}.
// Faked: ROSLIB, document.body, the chart/link/main collaborators (they only LOG, so the order is assertable).
let fakeNow = 1791532600000;
Date.now = () => fakeNow;

const rosInstances = [];
const topics = [];
const published = [];
class Ros {
    constructor() { this.handlers = {}; this.isConnected = false; rosInstances.push(this); }
    on(evt, cb) { this.handlers[evt] = cb; }
    connect() {}
    close() {}
    fire(evt) { this.isConnected = evt === 'connection'; if (this.handlers[evt]) this.handlers[evt](); }
}
class Topic {
    constructor(o) { this.o = o; topics.push(this); }
    subscribe(cb) { this.cb = cb; }
    unsubscribe() {}
    publish(m) { published.push(m); }
}
globalThis.ROSLIB = { Ros, Topic, Service: function () {}, Message: function (m) { return m; }, ServiceRequest: function (r) { return r; } };
globalThis.window = { location: { hostname: 'localhost' }, addEventListener() {} };
const classes = new Set();
globalThis.document = { hidden: false, body: { classList: { toggle(c, on) { if (on) classes.add(c); else classes.delete(c); } } } };

const clock = await import('./clock.js');
const ros = await import('./ros-bridge.js');
const ev = await import('./event-store.js');
const { setReplayFence } = await import('./replay/fence.js');
const { getSessionBuffer } = await import('./replay/session.js');
const { createChunkCache } = await import('./replay/cache.js');
const { createEngine } = await import('./replay/engine.js');
const { indexLatestBefore } = await import('./replay/chunk.js');
const { createReplayMode } = await import('./replay/mode.js');

const tick = () => new Promise((r) => setImmediate(r));
async function settle(n) { for (let i = 0; i < (n || 6); i++) await tick(); }

// ---- live handlers (the shape of main.js: a latch + an event on change) ----
const seen = [];
let lastOrch = null;
ros.subscribe('robot_state', 'T', (m) => seen.push(['robot_state', m.v]), 0);
ros.subscribe('orchestrator_state', 'T', (m) => {
    seen.push(['orch', m.data]);
    if (m.data !== lastOrch) {
        if (lastOrch !== null) ev.emitEvent({ type: 'state', label: lastOrch + ' -> ' + m.data, t: clock.now() / 1000 });
        lastOrch = m.data;
    }
}, 0);

const out = {};
const order = [];
const T0 = fakeNow / 1000;

// ---- connect, then feed 30 s live (the session-buffer tap) ----
ros.init('ws://localhost:9090');
const r0 = rosInstances[rosInstances.length - 1];
r0.fire('connection');
const topicsByName = {};
for (const t of topics) topicsByName[t.o.name] = t;
const feedStamps = [];
for (let k = 0; k < 60; k++) {
    fakeNow = (T0 + k * 0.5) * 1000;
    topicsByName.robot_state.cb({ v: k });
    topicsByName.orchestrator_state.cb({ data: k < 10 ? 'BOOT' : (k < 30 ? 'IDLE' : (k < 45 ? 'ACTIVE' : 'IDLE')) });
    feedStamps.push(fakeNow / 1000);
}
const buf = getSessionBuffer();
const lastRS = await buf.latestBefore(T0 + 1e6, '/robot_state');
const lastOS = await buf.latestBefore(T0 + 1e6, '/orchestrator_state');
const liveRangeBefore = buf.range();
out.tap = {
    n_robot_state_recorded: seen.filter((s) => s[0] === 'robot_state').length,
    last_robot_state: lastRS ? { t: lastRS.t, v: lastRS.msg.v } : null,
    last_orch: lastOS ? { t: lastOS.t, data: lastOS.msg.data } : null,
    expected_last_t: feedStamps[feedStamps.length - 1],
    range: liveRangeBefore,
};

const liveEventsBefore = JSON.stringify(ev.getRecentEvents(null, 100));

// ---- spy deps ----
const links = { can: null, udp: null, hw: null };
const fakeStore = {
    setResident(c) { order.push('store.setResident'); fakeStore.last = c.length; },
    chunks: 0,
};
const mkdeps = (extra) => Object.assign({
    ros, clock,
    fence: { setReplayFence(on) { order.push('fence.' + on); setReplayFence(on); } },
    events: {
        snapshotAndBeginReplayEvents() { order.push('events.snapshot'); return ev.snapshotAndBeginReplayEvents(); },
        restoreEvents(s) { order.push('events.restore'); ev.restoreEvents(s); },
        setEventsMuted: ev.setEventsMuted,
        trimEventsAfter: ev.trimEventsAfter,
    },
    charts: {
        createStore() { order.push('charts.createStore'); return fakeStore; },
        enter(store, o) { order.push('charts.enter'); fakeStore.onSeek = o.onSeek; },
        exit() { order.push('charts.exit'); },
        setPlayhead(p) { fakeStore.playhead = p; },
        telemetrySample() {},
    },
    links: {
        can(v) { order.push('link.can:' + v); },
        udp(v) { order.push('link.udp:' + v); },
        hw(v) { order.push('link.hw:' + v); },
    },
    resetForSeek() { order.push('resetForSeek'); lastOrch = null; },
    blankDisconnectedState() { order.push('blank'); },
    resetTrafficRings() { order.push('trafficReset'); },
    makeRecordingSource() {
        const s = getSessionBuffer().snapshot();
        s.kind = 'recording';
        s.open = async () => 'complete';
        return s;
    },
    getSessionBuffer,
    createCache: (source) => createChunkCache({ source }),
    createEngine,
    indexLatestBefore,
    visibleSpanSec: () => 10,
    raf: null,
}, extra || {});

// clock spy (the real clock under a logging wrapper; the engine uses it too)
const clockSpy = new Proxy(clock, {
    get(t, k) {
        if (k === '_enterReplay') return () => { order.push('clock.enter'); clock._enterReplay(); };
        if (k === '_exitReplay') return () => { order.push('clock.exit'); clock._exitReplay(); };
        return t[k];
    },
});

const mode = createReplayMode(mkdeps({ clock: clockSpy }));

// (1) entry refused while connected
let refused = null;
try { await mode.enterReplay({ kind: 'session' }); } catch (e) { refused = e.reason || e.message; }
out.refused_connected = { reason: refused, state: mode.state().mode, order_len: order.length };

// connection listener recorder (registered AFTER mode's hook, like main.js is)
const listenerLog = [];
ros.onConnectionStateChange((s) => {
    listenerLog.push({
        state: s, order_len: order.length, last: order[order.length - 1],
        isReplay: clock.isReplay(), fenced: classes.has('replay'),
        mode: mode.state().mode,
        events: JSON.stringify(ev.getRecentEvents(null, 100)),
    });
});

// (disconnect -> LOBBY)
r0.fire('close');
out.lobby = mode.state().mode;
listenerLog.length = 0;

// (6)+(2) replay last session: paused at t1 - span; ordered entry
await mode.enterReplay({ kind: 'session' });
const e0 = mode.engine();
const st0 = mode.state();
out.entry = {
    order_prefix: order.slice(0, 8),
    after_entry_sample: order.slice(8, 12),
    state: st0,
    t1: liveRangeBefore.t1,
    expect_playhead: liveRangeBefore.t1 - 10,
    fenced: classes.has('replay'),
    isReplay: clock.isReplay(),
    resident_chunks: fakeStore.last,
    events_in_replay_store_at_entry: ev.getRecentEvents(null, 100).length,
};

// (5) the tap is skipped in replay: a stray socket callback does not reach the live buffer
const beforeStray = getSessionBuffer().range();
topicsByName.robot_state.cb({ v: 12345 });
out.tap_replay_skip = { before: beforeStray.t1, after: getSessionBuffer().range().t1 };

// play forward through the whole snapshot
async function run(n, dt) { for (let i = 0; i < n; i++) { e0.tick(dt); await settle(2); } }
await e0.play();
await run(400, 100);
const keysOf = () => ev.getRecentEvents(null, 100).map((e) => e.t + '|' + e.type + '|' + e.label).sort();
const keys1 = keysOf();
out.forward = { mode: mode.state().sub, playhead: mode.state().playhead, t1: liveRangeBefore.t1, n_events: keys1.length, keys: keys1 };

// (7) reverse then forward again: no duplicates
await e0.seek(liveRangeBefore.t1 - 1);
e0.setSpeed(-1);
await e0.play();
await run(100, 100);
out.reverse = { events_during_reverse: keysOf().length, playhead: mode.state().playhead };
e0.setSpeed(1);
await e0.seek(liveRangeBefore.t1 - 10);
await e0.play();
await run(400, 100);
const keys2 = keysOf();
out.reverse_forward = { n_events: keys2.length, unique: new Set(keys2).size, same_as_first: JSON.stringify(keys1) === JSON.stringify(keys2) };

// direct dedup: the same (t, type, label) emitted twice lands once
{
    const n0 = ev.getRecentEvents(null, 100).length;
    ev.emitEvent({ type: 'fault', label: 'dup', t: T0 + 1 });
    ev.emitEvent({ type: 'fault', label: 'dup', t: T0 + 1 });
    out.dedup_direct = { added: ev.getRecentEvents(null, 100).length - n0 };
    ev.trimEventsAfter(T0);   // cleanup: it is in the replay store only
}

// (3) explicit exit order
order.length = 0;
await mode.exitReplay('user');
out.exit_order = order.slice();
out.after_exit = {
    state: mode.state().mode, isReplay: clock.isReplay(), fenced: classes.has('replay'),
    events_restored: JSON.stringify(ev.getRecentEvents(null, 100)) === liveEventsBefore,
};

// (4) a connected edge mid-replay exits BEFORE main's listener
order.length = 0;
listenerLog.length = 0;
await mode.enterReplay({ kind: 'recording', id: 'x' });
const midOrderLen = order.length;
order.length = 0;
r0.fire('connection');
out.connected_edge = {
    listener: listenerLog[0] || null,
    order: order.slice(),
    events_restored: listenerLog[0] ? listenerLog[0].events === liveEventsBefore : false,
    state_after: mode.state().mode,
    mid_entry_steps: midOrderLen,
};

// (8) flicker class: the failed-reconnect loop's connecting<->disconnected edges must reach NO live
// listener while OPENING/REPLAY (one enforcement point: the ros-bridge suppressor installed by mode.js).
{
    const newest = () => rosInstances[rosInstances.length - 1];
    const loopEdge = () => { ros.init('ws://localhost:9090'); newest().fire('close'); };  // connecting -> disconnected
    newest().fire('close');                      // connected -> disconnected (idle: delivered)
    await mode.enterReplay({ kind: 'session' });
    order.length = 0; listenerLog.length = 0;
    for (let i = 0; i < 3; i++) loopEdge();
    const during = {
        listener_states: listenerLog.map((l) => l.state), blanks: order.filter((x) => x === 'blank').length,
        mode: mode.state().mode, conn: ros.getConnectionState(),
    };
    order.length = 0; listenerLog.length = 0;
    ros.init('ws://localhost:9090'); newest().fire('connection');     // connected edge exits, as before
    const connected = {
        listener_states: listenerLog.map((l) => l.state), blanks: order.filter((x) => x === 'blank').length,
        mode: mode.state().mode,
    };
    // exitReplay path: listeners hear transitions again afterwards
    newest().fire('close');
    await mode.enterReplay({ kind: 'session' });
    loopEdge();
    order.length = 0; listenerLog.length = 0;
    await mode.exitReplay('user');
    const exitBlanks = order.filter((x) => x === 'blank').length;
    loopEdge();
    out.flicker_class = {
        during, connected, exit_blanks: exitBlanks,
        after_exit_states: listenerLog.map((l) => l.state), mode_after: mode.state().mode,
    };
}

// refusal path: a recording whose open() rejects with the 409 reason leaves the GUI untouched
r0.fire('close');
order.length = 0;
const mode2deps = mkdeps({
    clock: clockSpy,
    makeRecordingSource() {
        return { kind: 'recording', open: async () => { const e = new Error('refused'); e.reason = 'ros_running'; throw e; }, status() { return {}; }, onChange() { return () => {}; } };
    },
});
mode2deps.ros = Object.assign({}, ros, { setBeforeConnectedHook() {} });
const m2 = createReplayMode(mode2deps);
let r409 = null;
try { await m2.enterReplay({ kind: 'recording', id: 'y' }); } catch (e) { r409 = e.reason; }
out.refusal_409 = { reason: r409, order_len: order.length, state: m2.state().mode, isReplay: clock.isReplay() };

// (a) rollback: a throwing entry step leaves the GUI exactly as it was (the session buffer is still populated)
{
    const noHook = (extra) => {
        const d = mkdeps(Object.assign({ clock: clockSpy }, extra));
        d.ros = Object.assign({}, ros, { setBeforeConnectedHook() {} });
        return d;
    };
    const origErr = console.error; const errs = [];
    console.error = (...a) => errs.push(a.map(String).join(' '));
    order.length = 0;
    const m3 = createReplayMode(noHook({
        charts: Object.assign({}, mkdeps().charts, { enter() { order.push('charts.enter'); throw new Error('boom-enter'); } }),
        getReplayLatches: () => { order.push('latches.get'); return { s: 1 }; },
        restoreReplayLatches: (v) => { order.push('latches.restore:' + v.s); },
    }));
    let e3 = null;
    try { await m3.enterReplay({ kind: 'session' }); } catch (e) { e3 = e.message; }
    out.entry_throw = {
        error: e3, isReplay: clock.isReplay(), fenced: classes.has('replay'), state: m3.state().mode,
        active: m3.isActive(), order: order.slice(),
        events_restored: JSON.stringify(ev.getRecentEvents(null, 100)) === liveEventsBefore,
    };
    // a throwing exit step still runs the fence / clock undo
    order.length = 0;
    const m4 = createReplayMode(noHook({
        charts: Object.assign({}, mkdeps().charts, { exit() { order.push('charts.exit'); throw new Error('boom-exit'); } }),
    }));
    await m4.enterReplay({ kind: 'session' });
    const wasReplay = clock.isReplay();
    order.length = 0;
    await m4.exitReplay('user');
    out.exit_throw = {
        was_replay: wasReplay, isReplay: clock.isReplay(), fenced: classes.has('replay'), active: m4.isActive(),
        order: order.slice(), logged: errs.some((x) => x.includes('boom-exit')),
    };
    console.error = origErr;
}

// no session buffer
getSessionBuffer().clear();
let nb = null;
try { await mode.enterReplay({ kind: 'session' }); } catch (e) { nb = e.reason; }
out.no_session = { reason: nb, state: mode.state().mode };

console.log(JSON.stringify(out));
process.exit(0);
