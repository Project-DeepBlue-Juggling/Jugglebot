// hand_fan_panel_harness.js — the REAL ros_ws/gui/js/hand-fan-panel.js under node.
// Sandbox layout (tests/ros/test_gui_hand_fan_panel.py builds it): hand-fan-panel.js and
// replay/fence.js copied verbatim, a fake ./ros-bridge.js written by the test, this file,
// package.json {"type":"module"}. Prints one JSON object the test asserts on.

let mono = 1000;
Object.defineProperty(globalThis, 'performance', { value: { now: () => mono }, configurable: true, writable: true });
const timeouts = [];
globalThis.setTimeout = (fn, ms) => { const t = { fn, ms, done: false }; timeouts.push(t); return t; };
globalThis.clearTimeout = (t) => { if (t) t.done = true; };
globalThis.setInterval = () => 1;
const fireTimeouts = () => { for (const t of timeouts.splice(0)) if (!t.done) t.fn(); };
console.warn = () => {};
console.error = () => {};

const calls = [];
let nextResult = null;     // {resolve: obj} | {reject: Error} | null (stay pending)
let release = null;
globalThis.__fakeRos = {
    state: 'connected',
    callService(name, type, req) {
        calls.push({ name, type, req });
        return new Promise((res, rej) => {
            if (nextResult && nextResult.resolve) res(nextResult.resolve);
            else if (nextResult && nextResult.reject) rej(nextResult.reject);
            else release = res;
        });
    },
};

class El {
    constructor(id) {
        this.id = id; this.hidden = false; this.disabled = false; this.title = ''; this.textContent = '';
        this.className = ''; this._cls = new Set(); this.listeners = {};
        const self = this;
        this.classList = {
            toggle(c, on) { if (on) self._cls.add(c); else self._cls.delete(c); },
            contains(c) { return self._cls.has(c); },
        };
    }
    addEventListener(t, f) { (this.listeners[t] = this.listeners[t] || []).push(f); }
    click() { for (const f of this.listeners.click || []) f({ target: this }); }
}
const ids = ['flag-hand-fan-pill', 'hand-fan-off', 'hand-fan-on', 'hand-fan-auto', 'hand-fan-mode'];
const dom = Object.fromEntries(ids.map((i) => [i, new El(i)]));
globalThis.document = { getElementById: (i) => dom[i] || null, body: null };

const P = await import('./hand-fan-panel.js');
const flush = async () => { for (let i = 0; i < 10; i++) await Promise.resolve(); };

const live = { rosConnected: true, fenced: false, pending: false, flash: null };
const status = (over = {}) => ({
    mode: 'auto', fan_on: false, reading_valid: true,
    on_above_c: 35, off_below_c: 34, motor_temp_c: 28.25, ...over,
});
const view = (msg, ctx = live, age = 0) => P.handFanView(P.reduceStatus(msg, 0), age, ctx);
const snap = () => ({
    pill: dom['flag-hand-fan-pill'].textContent, pillClass: dom['flag-hand-fan-pill'].className,
    offDisabled: dom['hand-fan-off'].disabled, onDisabled: dom['hand-fan-on'].disabled,
    offActive: dom['hand-fan-off'].classList.contains('active'),
    onActive: dom['hand-fan-on'].classList.contains('active'),
    autoHidden: dom['hand-fan-auto'].hidden, mode: dom['hand-fan-mode'].textContent,
});

const out = {
    request: P.handFanRequest('off'),
    service: [P.HAND_FAN_SERVICE, P.HAND_FAN_SERVICE_TYPE],
    views: {
        no_status: P.handFanView(null, 0, live),
        auto_off: view(status()),
        auto_on: view(status({ fan_on: true })),
        forced_on: view(status({ mode: 'on', fan_on: true })),
        forced_off: view(status({ mode: 'off' })),
        no_reading: view(status({ reading_valid: false, motor_temp_c: 0 })),
        just_fresh: view(status(), live, P.STATUS_STALE_MS),
        stale: view(status({ fan_on: true }), live, P.STATUS_STALE_MS + 1),
        ros_down: view(status({ mode: 'on', fan_on: true }), { ...live, rosConnected: false }),
        pending: view(status({ mode: 'on', fan_on: true }), { ...live, pending: true }),
        replay: view(status({ mode: 'on', fan_on: true }), { ...live, fenced: true }, 1e9),
        flash: view(status(), { ...live, flash: 'REFUSED' }),
    },
};

P.initHandFanPanel();
out.born = snap();

P.handFanPanelOnStatus(status());
out.after_status = snap();

nextResult = { resolve: { success: true, message: 'ok' } };
dom['hand-fan-on'].click();
await flush();
out.click_on = { calls: calls.slice(), snap: snap() };

// The toggle follows the status frame, not the click: still auto until the board says so.
P.handFanPanelOnStatus(status({ mode: 'on', fan_on: true }));
out.after_forced_status = snap();

calls.length = 0;
dom['hand-fan-auto'].click();
await flush();
out.click_auto = calls.slice();

P.handFanPanelOnStatus(status({ mode: 'auto', fan_on: true }));
calls.length = 0;
nextResult = null;
dom['hand-fan-off'].click();
out.while_pending = { disabled: snap().onDisabled };
dom['hand-fan-on'].click();     // a second click while one is in flight
out.while_pending.calls = calls.length;
release({ success: false, message: 'refused' });
await flush();
out.refused = snap().mode;
fireTimeouts();
out.flash_cleared = snap().mode;

nextResult = { reject: new Error('timeout') };
dom['hand-fan-off'].click();
await flush();
out.no_reply = snap().mode;
fireTimeouts();

mono += P.STATUS_STALE_MS + 1;
dom['hand-fan-off'].click();    // a click reaches setMode only if the browser allowed it
await flush();
P.setHandFanPanelDisconnected();
out.disconnected = snap();

console.log(JSON.stringify(out));
