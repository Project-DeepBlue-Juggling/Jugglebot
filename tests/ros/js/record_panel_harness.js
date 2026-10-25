// record_panel_harness.js — the REAL ros_ws/gui/js/camera-panel.js (Wave camera record panel) under node.
// Sandbox layout (tests/ros/test_gui_record_panel.py builds it): camera-panel.js, replay/fence.js and
// replay/ui/format.js copied verbatim, a fake ./ros-bridge.js written by the test (state + callService driven
// through globalThis.__fakeRos), this file, package.json {"type":"module"}.
// Prints one JSON object the test asserts on.

// ---------------- fakes installed BEFORE the import ----------------
let mono = 1000;
Object.defineProperty(globalThis, 'performance', { value: { now: () => mono }, configurable: true, writable: true });
const timeouts = [];
globalThis.setTimeout = (fn, ms) => { const t = { fn, ms, done: false }; timeouts.push(t); return t; };
globalThis.clearTimeout = (t) => { if (t) t.done = true; };
const intervals = [];
globalThis.setInterval = (fn, ms) => { intervals.push({ fn, ms }); return intervals.length; };
const fireTimeouts = () => { for (const t of timeouts.splice(0)) if (!t.done) t.fn(); };

const calls = [];
let nextResult = null;     // {resolve: obj} | {reject: Error} | null (stay pending)
let pendingResolve = null;
globalThis.__fakeRos = {
    state: 'connected',
    callService(name, type) {
        calls.push({ name, type });
        return new Promise((res, rej) => {
            if (nextResult && nextResult.resolve) res(nextResult.resolve);
            else if (nextResult && nextResult.reject) rej(nextResult.reject);
            else pendingResolve = res;
        });
    },
};

class El {
    constructor(id) {
        this.id = id; this.hidden = false; this.disabled = false; this.title = ''; this.textContent = '';
        this._cls = new Set(); this.listeners = {};
        const self = this;
        this.classList = {
            toggle(c, on) { if (on) self._cls.add(c); else self._cls.delete(c); },
            add(c) { self._cls.add(c); }, remove(c) { self._cls.delete(c); },
            contains(c) { return self._cls.has(c); },
        };
    }
    addEventListener(t, f) { (this.listeners[t] = this.listeners[t] || []).push(f); }
    click() { for (const f of this.listeners.click || []) f({ target: this }); }
}
const ids = ['camera-panel', 'camera-rec-btn', 'camera-rec-dot', 'camera-rec-elapsed', 'camera-rec-warn', 'camera-rec-status'];
const dom = Object.fromEntries(ids.map((i) => [i, new El(i)]));
globalThis.document = { getElementById: (i) => dom[i] || null, body: null };

const P = await import('./camera-panel.js');
const { setReplayFence } = await import('./replay/fence.js');

const out = {};
const S = P.STATE;
const hb = (o) => Object.assign({
    connected: true, state: S.IDLE, camera_on: true, uart_seen: true, recording: false, elapsed_ms: 0,
    start_stamp: { sec: 0, nanosec: 0 }, recordings_this_session: 0, time_synced: true, sync_age_ms: 100,
    fw_version: 1, uptime_s: 10, fault_bits: 0, console_owns: false, intent_fresh: true, offset_spread_us: 50,
}, o);
const model = (o, at = 0) => P.reduceHeartbeat(null, hb(o), at);
const live = { rosConnected: true, fenced: false, pending: false };

// ---- 1. enable truth table ----
const cases = {
    idle: [model({}), live],
    recording: [model({ state: S.RECORDING, recording: true }), live],
    starting: [model({ state: S.STARTING }), live],
    stopping: [model({ state: S.STOPPING }), live],
    boot: [model({ state: S.BOOT }), live],
    fault: [model({ state: S.FAULT, fault_bits: 2 }), live],
    adapter_down: [model({ connected: false }), live],
    camera_off: [model({ camera_on: false }), live],
    console_owns: [model({ console_owns: true }), live],
    ros_down: [model({}), { ...live, rosConnected: false }],
    fenced: [model({}), { ...live, fenced: true }],
    pending: [model({}), { ...live, pending: true }],
    no_hb: [null, live],
    unsynced_and_faulted_idle: [model({ time_synced: false, fault_bits: 0x20 }), live],
};
out.enable = {};
for (const [k, [m, ctx]] of Object.entries(cases)) out.enable[k] = P.canPress(m, 500, ctx);
out.enable.stale = P.canPress(model({}, 0), 1001, live);
out.enable.just_fresh = P.canPress(model({}, 0), 1000, live);

// ---- 2. status text ----
out.status = {
    no_hb: P.statusText(null),
    idle0: P.statusText(P.normalizeHeartbeat(hb({}))),
    idle1: P.statusText(P.normalizeHeartbeat(hb({ recordings_this_session: 1 }))),
    idle3: P.statusText(P.normalizeHeartbeat(hb({ recordings_this_session: 3 }))),
    recording: P.statusText(P.normalizeHeartbeat(hb({ state: S.RECORDING, recording: true }))),
    starting: P.statusText(P.normalizeHeartbeat(hb({ state: S.STARTING }))),
    stopping: P.statusText(P.normalizeHeartbeat(hb({ state: S.STOPPING }))),
    boot: P.statusText(P.normalizeHeartbeat(hb({ state: S.BOOT }))),
    fault: P.statusText(P.normalizeHeartbeat(hb({ state: S.FAULT, fault_bits: 0b1010 }))),
    fault_none: P.statusText(P.normalizeHeartbeat(hb({ state: S.FAULT }))),
    adapter_down: P.statusText(P.normalizeHeartbeat(hb({ connected: false, camera_on: false }))),
    camera_off: P.statusText(P.normalizeHeartbeat(hb({ camera_on: false, state: S.FAULT, fault_bits: 2 }))),
    faults_alias: P.statusText(P.normalizeHeartbeat({ connected: true, camera_on: true, state: S.FAULT, faults: 1 })),
};

// ---- 3. fault names + warnings ----
out.faultNames = {
    all: P.faultNames(0x7f),
    none: P.faultNames(0),
    b0: P.faultNames(1),
    b5b6: P.faultNames(0x60),
    b7: P.faultNames(0x80),
};
out.warn = {
    clean: P.warnings(P.normalizeHeartbeat(hb({}))),
    unsynced: P.warnings(P.normalizeHeartbeat(hb({ time_synced: false, sync_age_ms: 25500 }))),
    faults: P.warnings(P.normalizeHeartbeat(hb({ fault_bits: 0x09 }))),
    console: P.warnings(P.normalizeHeartbeat(hb({ console_owns: true }))),
    adapter_down: P.warnings(P.normalizeHeartbeat(hb({ connected: false, time_synced: false }))),
};

// ---- 4. elapsed interpolation + stale freeze ----
const rec = P.reduceHeartbeat(null, hb({ state: S.RECORDING, recording: true, elapsed_ms: 61500 }), 10000);
out.elapsed = {
    at_hb: P.interpolatedElapsedMs(rec, 10000),
    plus250: P.interpolatedElapsedMs(rec, 10250),
    plus999: P.interpolatedElapsedMs(rec, 10999),
    plus5000: P.interpolatedElapsedMs(rec, 15000),           // clamped: frozen at +1 s
    before_hb: P.interpolatedElapsedMs(rec, 9990),            // clock never runs backwards
    idle: P.interpolatedElapsedMs(model({ elapsed_ms: 999 }), 5000),
    none: P.interpolatedElapsedMs(null, 0),
};
out.fmt = {
    t0: P.formatElapsed(0), t999: P.formatElapsed(999), t61500: P.formatElapsed(61500),
    t_hour: P.formatElapsed(3600 * 1000 + 62 * 1000), t_null: P.formatElapsed(null),
};
// the view at points along a take, then stale, then a snapping heartbeat
const v = (m, t, ctx = live) => P.cameraPanelView(m, t, ctx);
out.view = {
    fresh: v(rec, 10500),
    stale: v(rec, 11500),
    stale_later: v(rec, 20000),
    snapped: v(P.reduceHeartbeat(rec, hb({ state: S.RECORDING, recording: true, elapsed_ms: 62000 }), 20000), 20000),
    ros_down: v(rec, 10500, { ...live, rosConnected: false }),
    no_hb: v(null, 0),
    pending: v(model({}), 0, { ...live, pending: true }),
    flash: v(model({}), 0, { ...live, flash: 'Start failed: camera off' }),
    idle_warn: v(model({ console_owns: true }), 0),
};

// ---- 5. glue: init, heartbeat, click, failure flash, disconnect blank ----
P.initCameraPanel();
const snap = () => ({
    disabled: dom['camera-rec-btn'].disabled, label: dom['camera-rec-btn'].textContent,
    red: dom['camera-rec-btn'].classList.contains('rec-active'),
    dot: dom['camera-rec-dot'].classList.contains('recording'),
    elapsed: dom['camera-rec-elapsed'].textContent, status: dom['camera-rec-status'].textContent,
    warnHidden: dom['camera-rec-warn'].hidden, stale: dom['camera-panel'].classList.contains('stale'),
    blank: dom['camera-panel'].classList.contains('blank'),
});
const tick = () => new Promise((r) => setImmediate(r));
const g = {};
g.intervals = intervals.map((i) => i.ms);
g.initial = snap();
mono = 2000; P.cameraPanelOnHeartbeat(hb({ recordings_this_session: 2 }));
g.idle = snap();
nextResult = null;                                   // the call stays in flight
dom['camera-rec-btn'].click();
g.inflight = snap();
g.inflight_calls = calls.slice();
dom['camera-rec-btn'].click();                       // ignored while pending
g.calls_after_double_click = calls.length;
pendingResolve({ success: true, message: '' }); await tick(); await tick();
mono = 2100; P.cameraPanelOnHeartbeat(hb({ state: S.RECORDING, recording: true, elapsed_ms: 4000, recordings_this_session: 3 }));
g.recording = snap();
mono = 2600; intervals[0].fn();
g.recording_plus500 = snap();
mono = 4000; intervals[0].fn();
g.recording_stale = snap();
mono = 4100; P.cameraPanelOnHeartbeat(hb({ state: S.RECORDING, recording: true, elapsed_ms: 6000 }));
nextResult = { resolve: { success: false, message: 'timeout waiting for STOP' } };
dom['camera-rec-btn'].click();
await tick(); await tick();
g.stop_call = calls[calls.length - 1];
g.failed = snap();
fireTimeouts();
g.after_flash = snap();
nextResult = { reject: new Error('Not connected to ROS') };
dom['camera-rec-btn'].click();
await tick(); await tick();
g.rejected = snap();
fireTimeouts();
setReplayFence(true); intervals[0].fn();
g.fenced = snap();
const before = calls.length; dom['camera-rec-btn'].disabled = false; dom['camera-rec-btn'].click();
g.fenced_click_calls = calls.length - before;
setReplayFence(false);
globalThis.__fakeRos.state = 'disconnected';
P.setCameraPanelDisconnected();
g.disconnected = snap();
out.glue = g;

process.stdout.write(JSON.stringify(out) + '\n');
