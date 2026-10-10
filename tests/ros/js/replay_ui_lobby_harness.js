// replay_ui_lobby_harness.js — the REAL replay/ui/{dom,format,toast,picker,lobby}.js under node with a minimal fake
// DOM, a fake fetch, a fake ros (connection state) and a fake mode (enterReplay resolves/rejects on demand).
// Prints one JSON object that tests/ros/test_gui_replay_ui_lobby.py asserts on.
// Sandbox layout: replay/ui/*.js (all but index.js), this file, package.json {"type":"module"}.

// ---------------- fake DOM ----------------
class El {
    constructor(tag) {
        this.tag = tag; this.children = []; this.parentNode = null; this.listeners = {}; this.attrs = {};
        this.className = ''; this.id = ''; this.hidden = false; this.disabled = false; this.value = ''; this.title = '';
        this._text = ''; this._cls = new Set();
        const self = this;
        this.classList = {
            toggle(c, on) { if (on) self._cls.add(c); else self._cls.delete(c); },
            add(c) { self._cls.add(c); }, remove(c) { self._cls.delete(c); },
            contains(c) { return self._cls.has(c) || self.className.split(/\s+/).includes(c); },
        };
    }
    get textContent() { return this.children.length ? this.children.map((c) => c.textContent).join('') : this._text; }
    set textContent(v) { this.children = []; this._text = String(v); }
    get firstChild() { return this.children[0] || null; }
    appendChild(c) { c.parentNode = this; this.children.push(c); return c; }
    removeChild(c) { this.children = this.children.filter((x) => x !== c); c.parentNode = null; }
    remove() { if (this.parentNode) this.parentNode.removeChild(this); }
    replaceChildren() { this.children = []; this._text = ''; }
    setAttribute(k, v) { this.attrs[k] = v; }
    addEventListener(t, f) { (this.listeners[t] = this.listeners[t] || []).push(f); }
    click() { for (const f of this.listeners.click || []) f({ target: this }); }
    find(pred, out = []) { if (pred(this)) out.push(this); for (const c of this.children) c.find(pred, out); return out; }
}
const body = new El('body');
const docListeners = {};
const doc = {
    body,
    createElement: (t) => new El(t),
    getElementById(id) { return body.find((e) => e.id === id)[0] || null; },
    addEventListener(t, f) { (docListeners[t] = docListeners[t] || new Set()).add(f); },
    removeEventListener(t, f) { if (docListeners[t]) docListeners[t].delete(f); },
    key(k) { for (const f of Array.from(docListeners.keydown || [])) f({ key: k, target: body }); },
};
const viewer = new El('div'); viewer.id = 'viewer-pane'; body.appendChild(viewer);
const overlay = new El('div'); overlay.id = 'command-overlay'; viewer.appendChild(overlay);
const liveBtn = new El('button'); liveBtn.id = 'cmd-home'; overlay.appendChild(liveBtn);
const dot = new El('span'); dot.id = 'conn-dot'; body.appendChild(dot);
const ctext = new El('span'); ctext.id = 'conn-text'; ctext._text = 'Disconnected'; body.appendChild(ctext);

// ---------------- fakes ----------------
let connState = 'disconnected';
const rosListeners = [];
const ros = {
    getConnectionState: () => connState,
    onConnectionStateChange(cb) { rosListeners.push(cb); cb(connState); },
};
const setConn = (s) => { connState = s; rosListeners.forEach((f) => f(s)); };

let active = false, srcRange = { t0: 1791532600, t1: 1791532700, frontier: 1791532700 };
const modeListeners = { state: [], notice: [] };
let pending = null, enterCalls = [];
const mode = {
    isActive: () => active,
    source: () => ({ range: () => srcRange, id: 'x' }),
    on(evt, cb) { modeListeners[evt].push(cb); return () => {}; },
    enterReplay(spec) {
        enterCalls.push(spec);
        return new Promise((resolve, reject) => { pending = { resolve, reject }; });
    },
};
const emitMode = (evt, v) => modeListeners[evt].forEach((f) => f(v));

let sessionData = false;
const sessionBuf = {
    range: () => (sessionData ? { t0: 100, t1: 142, frontier: 142 } : { t0: 0, t1: 0, frontier: 0 }),
    chunkCount: () => (sessionData ? 5 : 0),
};

const REC = (id, startSec, extra) => Object.assign({
    id, path: '/x/' + id, size_bytes: 2.5e9, mtime: startSec + 100, closed: true, in_progress: false,
    duration_s: 100, message_count: 10, start_ns: startSec * 1e9, cache: 'none',
}, extra || {});
const T0 = 1791532600;
let listing, fetchLog = [], failList = false;
const resetListing = () => {
    listing = {
        recordings: [
            REC('r_complete', T0 + 5000, { cache: 'complete' }),
            REC('r_conv', T0 + 4000, { cache: 'converting' }),
            REC('r_noindex', T0 + 3000, { duration_s: null, start_ns: null, mtime: T0 + 3000, cache: 'none' }),
            REC('r_live', T0 + 6000, { closed: false, in_progress: true }),
            REC('r_stale', T0 + 2000, { cache: 'stale' }),
            REC('r_empty', 0, { start_ns: 9223372036854775807, duration_s: 0, mtime: T0 + 2500, cache: 'none' }),   // empty bag: INT64_MAX sentinel
        ],
        cache: { bytes: 0, cap_bytes: 1, disk_free_bytes: 1 }, worker_available: true,
    };
};
resetListing();
const jres = (j, ok = true) => Promise.resolve({ ok, status: ok ? 200 : 500, json: () => Promise.resolve(j) });
let lastSignal = null;
const fetchFn = (url, opt) => {
    fetchLog.push(url);
    if (opt && opt.signal) lastSignal = opt.signal;
    if (failList) return Promise.reject(new Error('ECONNREFUSED'));
    if (url === '/api/replay/recordings') return jres(listing);
    let m = /recordings\/(\w+)\/manifest$/.exec(url);
    if (m) {
        if (m[1] === 'r_complete') return jres({ topics: { '/robot_state': { count: 9 }, '/balls': { count: 0 }, '/orchestrator_state': { count: 3 } } });
        if (m[1] === 'r_conv') return jres({ topics: { '/robot_state': { count: 9 }, '/balls': { count: 4 }, '/cone/catch_event': { count: 1 }, '/bb/heartbeat': { count: 2 } } });
        return jres({ topics: { '/robot_state': { count: 1 }, '/balls': { count: 1 }, '/cone/catch_event': { count: 1 }, '/bb/heartbeat': { count: 1 } } });
    }
    m = /recordings\/(\w+)\/status$/.exec(url);
    if (m) return jres({ chunks_done: 31, chunks_total: 50 });
    return jres({}, false);
};
const settle = async () => { for (let i = 0; i < 60; i++) await new Promise((r) => setImmediate(r)); };

// ---------------- modules ----------------
const { createToaster } = await import('./replay/ui/toast.js');
const { createPicker } = await import('./replay/ui/picker.js');
const { createLobby } = await import('./replay/ui/lobby.js');

const timers = [];
const toaster = createToaster({ document: doc, timers: { set: (f, ms) => { timers.push({ f, ms }); return timers.length; }, clear() {} } });
const picker = createPicker({ document: doc, fetch: fetchFn, mode, getSessionBuffer: () => sessionBuf });
const lobby = createLobby({ document: doc, ros, mode, picker, toaster, getSessionBuffer: () => sessionBuf });
await settle();

const out = {};
const q = (id) => doc.getElementById(id);
const rowEls = () => body.find((e) => e.className.split(/\s+/).includes('replay-pk-row') && e.attrs['data-key']);
const rowByKey = (k) => rowEls().find((e) => e.attrs['data-key'] === k);
const cellText = (row) => row.find((e) => e.className.split(/\s+/).includes('replay-st'))[0].textContent;
const chipState = (row) => row.find((e) => e.className.split(/\s+/).includes('replay-chip')).map((c) => [c.textContent, c.className.includes('missing')]);

// 1. lobby follows the connection state
const snap = () => ({
    lobby_hidden: q('replay-lobby').hidden, banner_hidden: q('replay-banner').hidden, banner: q('replay-banner').textContent,
    cls_lobby: overlay.classList.contains('replay-lobby-on'), dock_hidden: q('replay-dock').hidden,
});
// N2: the initial 'disconnected' default is NOT a failed connect: no lobby flash, no probe, on a cold page load
out.initial = Object.assign(snap(), { probe_fetches: fetchLog.filter((u) => u === '/api/replay/recordings').length });
setConn('connecting'); out.first_connecting = snap();
setConn('disconnected'); out.disconnected = snap();           // a real failed attempt (disconnected edge after connecting)
setConn('connecting'); out.connecting = snap();               // retrying: the lobby stays
setConn('connected'); out.connected = snap();
setConn('disconnected');
out.live_button_untouched = liveBtn.hidden === false;
out.probe_fetches = fetchLog.filter((u) => u === '/api/replay/recordings').length;

// 2. session button / row only with buffer data
out.session_empty = { disabled: q('replay-btn-session').disabled, text: q('replay-btn-session').textContent };
sessionData = true; lobby.update();
out.session_data = { disabled: q('replay-btn-session').disabled, text: q('replay-btn-session').textContent };

// 3. picker list states
await picker.open(); await settle();
let m = picker.model();
out.rows_keys = m.rows.map((r) => r.key);
out.empty_row_when = rowByKey('r_empty').children[0].children[0].textContent;
out.session_row = m.rows[0].when;
out.states = {};
for (const k of ['r_complete', 'r_conv', 'r_noindex', 'r_live', 'r_stale']) {
    const el = rowByKey(k);
    out.states[k] = { cell: cellText(el), refused: el.className.includes('refused'), chips: chipState(el), when: el.children[0].children[0].textContent,
        dur: el.children[1].textContent, topics: el.children[3].textContent };
}
out.sessions_cell = cellText(rowByKey('session'));
out.banner_hidden_ok = q('replay-pk-banner').hidden;

// 4. filter
picker.setFilter('stale'); out.filter_stale = picker.model().rows.map((r) => r.key);
picker.setFilter('/robot'.slice(1)); out.filter_topic = picker.model().rows.map((r) => r.key);
picker.setFilter('zzz'); out.filter_none = picker.model().rows.map((r) => r.key);
picker.setFilter('');

// 5. choose: in-progress is refused, a recording calls enterReplay and closes on success
out.choose_live = await picker.choose('r_live'); out.calls_after_live = enterCalls.length;
const p = picker.choose('r_complete');
await settle();
out.opening_cell = cellText(rowByKey('r_complete'));
out.calls = enterCalls.slice();
pending.resolve(); out.chose = await p; out.open_after_success = picker.isOpen();

// 6. refusals keep the picker open with a message
out.refusals = {};
for (const reason of ['recording_in_progress', 'disk_low', 'worker_unavailable', 'busy', 'no session buffer', 'connected']) {
    await picker.open(); await settle();
    const pr = picker.choose('r_complete'); await settle();
    const err = new Error('x'); err.reason = reason; pending.reject(err);
    const ok = await pr;
    out.refusals[reason] = { ok, open: picker.isOpen(), msg: q('replay-pk-msg').textContent, msg_visible: !q('replay-pk-msg').hidden };
    picker.close();
}
// session row choice
await picker.open(); await settle();
enterCalls = [];
const ps = picker.choose('session'); await settle(); pending.resolve(); await ps;
out.session_call = enterCalls.slice();

// 7. Esc closes; backdrop closes
await picker.open(); doc.key('Escape'); out.esc_closed = !picker.isOpen();
await picker.open(); q('replay-picker-bg').listeners.click[0]({ target: q('replay-picker-bg') }); out.backdrop_closed = !picker.isOpen();
await picker.open(); q('replay-picker-bg').listeners.click[0]({ target: q('replay-picker') }); out.inner_click_keeps = picker.isOpen(); picker.close();

// 8. backend unavailable -> banner + retry; lobby disables Open recording
listing.worker_available = false;
await picker.open(); await settle();
out.backend_down = { banner_visible: !q('replay-pk-banner').hidden, text: q('replay-pk-banner').textContent, has_retry: !!q('replay-pk-retry'),
    open_disabled: q('replay-btn-open').disabled, backend_banner_hidden: q('replay-backend-banner').hidden };
listing.worker_available = true;
q('replay-pk-retry').click(); await settle();
out.backend_back = { banner_hidden: q('replay-pk-banner').hidden, open_disabled: q('replay-btn-open').disabled };
picker.close();
failList = true; await picker.open(); await settle();
out.list_unreachable = { banner_visible: !q('replay-pk-banner').hidden, text: q('replay-pk-banner').textContent };
failList = false; picker.close();

// 9. header + notice toast
active = true; emitMode('state', {});
out.header_replay = { text: ctext.textContent, dot: dot.className, cls_active: overlay.classList.contains('replay-active'),
    dock_hidden: q('replay-dock').hidden, lobby_hidden: q('replay-lobby').hidden };
active = false; emitMode('state', {});
out.header_after = { text: ctext.textContent, dot: dot.className, lobby_hidden: q('replay-lobby').hidden };
emitMode('notice', { kind: 'toast', text: 'Replay ended: rosbridge connected' });
out.toasts = q('replay-toasts').children.map((t) => t.textContent);
timers[timers.length - 1].f(); out.toasts_after_timer = q('replay-toasts').children.length;

// N3: double-open registers one keydown handler; probe during load still renders; close aborts enrichment
picker.close();
await picker.open(); const n1 = (docListeners.keydown || new Set()).size; picker.open(); const n2 = (docListeners.keydown || new Set()).size;
picker.close(); out.double_open = { first: n1, second: n2, after_close: (docListeners.keydown || new Set()).size };
{
    const base = fetchLog.length;
    const pl = picker.open();                 // list fetch in flight ...
    const pp = picker.probe();                // ... probe must not orphan it
    await pl; await pp; await settle();
    out.probe_during_load = { rows: rowEls().length, loading: picker.model().loading };
    picker.close();
}
{
    // enrichment of rows already in extras is skipped on re-open; close stops further manifest fetches
    await picker.open(); await settle(); picker.close();
    const m0 = fetchLog.length;
    await picker.open(); await settle();
    out.reopen_manifest_fetches = fetchLog.slice(m0).filter((u) => /manifest$/.test(u)).length;
    const sig = lastSignal; picker.close();
    out.close_aborts = !!sig && sig.aborted === true;
}

console.log(JSON.stringify(out));
process.exit(0);
