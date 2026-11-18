// imu_upload_panel_harness.js — the REAL ros_ws/gui/js/imu-upload-panel.js under node (sandbox built by
// tests/ros/test_gui_imu_upload_panel.py, as hand_fan_panel_harness.js). Prints one JSON object.
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
let nextResult = null;
globalThis.__fakeRos = {
    state: 'connected',
    callService(name, type, req) {
        calls.push({ name, type, req });
        return new Promise((res, rej) => {
            if (nextResult && nextResult.resolve) res(nextResult.resolve);
            else if (nextResult && nextResult.reject) rej(nextResult.reject);
        });
    },
};

class El {
    constructor(id) {
        this.id = id; this.hidden = false; this.disabled = false; this.title = ''; this.textContent = '';
        this.checked = false; this.value = 0; this._cls = new Set(); this.listeners = {};
        const self = this;
        this.classList = { toggle(c, on) { if (on) self._cls.add(c); else self._cls.delete(c); },
                           contains(c) { return self._cls.has(c); } };
    }
    addEventListener(t, f) { (this.listeners[t] = this.listeners[t] || []).push(f); }
    change() { for (const f of this.listeners.change || []) f({ target: this }); }
}
const ids = ['imu-upload-toggle', 'imu-upload-state', 'imu-upload-bar', 'imu-upload-last'];
const dom = Object.fromEntries(ids.map((i) => [i, new El(i)]));
globalThis.document = { getElementById: (i) => dom[i] || null, body: null };

const P = await import('./imu-upload-panel.js');
const flush = async () => { for (let i = 0; i < 10; i++) await Promise.resolve(); };

const live = { rosConnected: true, fenced: false, pending: false, flash: null };
const msg = (over = {}) => {
    const kv = { busy: '0', upload_after_attempt: '1', phase: 'idle', frames_done: '0', frames_total: '0',
                 eta_s: '', platform_state: 'RUNNING', last_label: '', last_complete: '', last_crc_ok: '',
                 last_failure: '', last_skip: '', ...over };
    return { values: Object.entries(kv).map(([key, value]) => ({ key, value })) };
};
const view = (over = {}, ctx = live, age = 0) => P.imuView(P.reduceStatus(msg(over), 0), age, ctx);
const snap = () => ({
    checked: dom['imu-upload-toggle'].checked, disabled: dom['imu-upload-toggle'].disabled,
    state: dom['imu-upload-state'].textContent, barHidden: dom['imu-upload-bar'].hidden,
    bar: dom['imu-upload-bar'].value, last: dom['imu-upload-last'].textContent,
    lastFailed: dom['imu-upload-last'].classList.contains('failed'),
});
const upl = { busy: '1', phase: 'samples', frames_done: '32000', frames_total: '64000', eta_s: '24.4' };

const out = {
    request: P.toggleRequest(false), service: [P.IMU_TOGGLE_SERVICE, P.IMU_TOGGLE_SERVICE_TYPE],
    views: {
        no_status: P.imuView(null, 0, live),
        recording: view(),
        no_imu: view({ platform_state: 'PROBING', last_skip: 'IMU state PROBING (no sensor)' }),
        uploading: view(upl),
        off: view({ upload_after_attempt: '0' }),
        incomplete: view({ last_complete: '0', last_crc_ok: 'True' }),
        crc: view({ last_complete: '1', last_crc_ok: 'False' }),
        failed: view({ last_failure: 'reboot' }),
        complete: view({ last_complete: '1', last_crc_ok: 'True' }),
        just_fresh: view({}, live, P.STATUS_STALE_MS),
        stale: view({}, live, P.STATUS_STALE_MS + 1),
        ros_down: view({}, { ...live, rosConnected: false }),
        replay: view({}, { ...live, fenced: true }),
    },
};

P.initImuUploadPanel();
out.born = snap();
P.imuUploadPanelOnStatus(msg());
out.after_status = snap();

nextResult = { resolve: { success: true, message: 'ok' } };
dom['imu-upload-toggle'].checked = false;
dom['imu-upload-toggle'].change();
await flush();
out.click_off = { calls: calls.slice(), snap: snap() };
// the toggle follows the node's status, not the click
P.imuUploadPanelOnStatus(msg({ upload_after_attempt: '0' }));
out.after_off_status = snap();

P.imuUploadPanelOnStatus(msg(upl));
out.uploading = snap();

nextResult = { resolve: { success: false, message: 'refused' } };
dom['imu-upload-toggle'].change();
await flush();
out.refused = snap().state;
fireTimeouts();
nextResult = { reject: new Error('timeout') };
dom['imu-upload-toggle'].change();
await flush();
out.no_reply = snap().state;
fireTimeouts();

P.setImuUploadPanelDisconnected();
out.disconnected = snap();
console.log(JSON.stringify(out));
