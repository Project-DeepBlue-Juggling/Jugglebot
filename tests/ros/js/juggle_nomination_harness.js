// juggle_nomination_harness.js — the REAL ros_ws/gui/js/juggle-panel.js under node with a fake DOM, a virtual clock
// and a scriptable transport.  Sandbox (tests/ros/test_gui_juggle_nomination.py builds it): juggle-panel.js,
// clock.js, event-store.js, hold-to-confirm.js copied verbatim, a fake ./ros-bridge.js, this file,
// package.json {"type":"module"}.  Prints one JSON object the test asserts on.

// ---------------- virtual clock (installed BEFORE the import) ----------------
let vnow = 0;
let timers = [];
let tid = 0;
globalThis.setTimeout = (fn, ms) => { const t = { id: ++tid, fn, at: vnow + (ms || 0), done: false }; timers.push(t); return t; };
globalThis.clearTimeout = (t) => { if (t) t.done = true; };
globalThis.requestAnimationFrame = () => 0;
globalThis.cancelAnimationFrame = () => {};
Object.defineProperty(globalThis, 'performance', { value: { now: () => vnow }, configurable: true, writable: true });
async function flush() { for (let i = 0; i < 12; i++) await Promise.resolve(); }
async function advance(ms) {
    const end = vnow + ms;
    for (;;) {
        await flush();
        const due = timers.filter((t) => !t.done && t.at <= end).sort((a, b) => a.at - b.at || a.id - b.id)[0];
        if (!due) break;
        due.done = true; vnow = Math.max(vnow, due.at); due.fn();
    }
    vnow = end;
    timers = timers.filter((t) => !t.done);
    await flush();
}

// ---------------- fake DOM ----------------
const byId = new Map();
class El {
    constructor(tag) {
        this.tagName = tag; this.children = []; this.listeners = {}; this._cls = new Set();
        this.hidden = false; this.disabled = false; this.title = ''; this.value = ''; this.attrs = {};
        this.style = { setProperty() {} }; this.isConnected = true; this._text = ''; this._id = '';
        const self = this;
        this.classList = {
            add(c) { self._cls.add(c); }, remove(c) { self._cls.delete(c); },
            toggle(c, on) { if (on === undefined) on = !self._cls.has(c); if (on) self._cls.add(c); else self._cls.delete(c); },
            contains(c) { return self._cls.has(c); },
        };
    }
    set id(v) { this._id = v; byId.set(v, this); }
    get id() { return this._id; }
    set className(v) { this._cls = new Set(String(v).split(/\s+/).filter(Boolean)); }
    get className() { return [...this._cls].join(' '); }
    set textContent(v) { this._text = String(v); this.children = []; }
    get textContent() { return this._text + this.children.map((c) => c.textContent).join(''); }
    set innerHTML(v) { this.children = []; this._text = ''; }
    get innerHTML() { return ''; }
    appendChild(c) { this.children.push(c); return c; }
    append(...cs) { for (const c of cs) this.children.push(c); }
    setAttribute(k, v) { this.attrs[k] = String(v); }
    getAttribute(k) { return this.attrs[k] == null ? null : this.attrs[k]; }
    addEventListener(t, f) { (this.listeners[t] = this.listeners[t] || []).push(f); }
    dispatch(t, ev) { for (const f of this.listeners[t] || []) f(Object.assign({ target: this, stopPropagation() {}, preventDefault() {} }, ev)); }
    querySelector() { return null; }
    querySelectorAll() { return []; }
    getBoundingClientRect() { return { left: 0, top: 0, width: 100, height: 20 }; }
    closest() { return null; }
    focus() {} blur() {} remove() {}
}
globalThis.document = {
    hidden: false,
    getElementById: (i) => byId.get(i) || null,
    createElement: (t) => new El(t),
    createElementNS: (ns, t) => new El(t),
    addEventListener() {},
    documentElement: new El('html'),
};
globalThis.localStorage = { getItem: () => null, setItem() {} };
globalThis.window = globalThis;

// ---------------- scriptable transport ----------------
const calls = [];
const state = { nominate: () => Promise.resolve({ success: true, message: 'nomination OK' }), params: 'ok' };  // nominate: (req) => Promise
globalThis.__fakeRos = {
    callService(name, type, req) {
        calls.push({ name, type, req, t: vnow });
        if (name === '/skill_node/get_parameters') {
            const d = { apex_m: 0.95, separation_mm: 125, dwell_s: 0.27, n_throws: 4, columns_feed_site: 'P1' };
            return Promise.resolve({ values: req.names.map((n) => (n === 'n_throws' ? { type: 2, integer_value: 4 }
                : n === 'columns_feed_site' ? { type: 4, string_value: 'P1' } : { type: 3, double_value: d[n] })) });
        }
        if (name === '/skill_node/set_parameters') return Promise.resolve({ results: [{ successful: true, reason: '' }] });
        if (name === 'skills/nominate') return state.nominate(req);
        if (name === 'jugglebot/juggle_request') return Promise.resolve({ success: true, message: 'Juggle dispatched.' });
        return Promise.reject(new Error('no fake for ' + name));
    },
};

const P = await import('./juggle-panel.js');

// ---------------- helpers ----------------
const $ = (id) => byId.get(id);
const deferred = () => { let res, rej; const p = new Promise((a, b) => { res = a; rej = b; }); return { p, res, rej }; };
const nominateCalls = () => calls.filter((c) => c.name === 'skills/nominate');
const view = () => ({
    hidden: $('juggle-nomination').hidden,
    summary: $('juggle-nomination-summary').textContent,
    cls: $('juggle-nomination-summary').className,
    lines: $('juggle-nomination-lines').children.map((c) => c.textContent),
});
async function pick(pattern) { const s = $('juggle-pattern'); s.value = pattern; s.dispatch('change'); await flush(); }
async function type(id, v) { const i = $(id); i.value = v; i.dispatch('input'); await flush(); }
async function holdStart() {
    const b = $('juggle-start');
    b.dispatch('pointerdown', { button: 0 });
    await advance(900);
    b.dispatch('pointerup', { button: 0 });
}
const gate = (o) => P.renderJugglePanel(Object.assign({ connected: true, active: true, expanded: true, mainState: 'ACTIVE' }, o));

const REFUSED = {
    success: false,
    message: 'nomination REFUSED: columns_highlow apex 1.00 / 0.90 m, separation 125 mm, dwell 0.27 s: 3 refusals; nearest viable: separation 100 mm\n'
        + 'CATCH Ball 2 at P2 -> P2, rung -20 %: LIMIT_VEL leg 2 at t=0.188 s, margin -0.04 (vel)\n'
        + 'THROW Ball 1 at P1 -> P2, rung +10 %: LIMIT_ACC leg 4 at t=0.512 s, margin -0.11 (acc)\n'
        + 'CARRY Ball 2 at P2 -> P1, rung 0 %: MARGIN below floor, worst margin 0.02 (jerk), no leg or knot time is recorded for a margin refusal',
};
const OK = { success: true, message: 'nomination OK: columns apex 0.95 m, separation 125 mm, dwell 0.27 s: worst margin 0.34 (acc, CATCH Ball 1 at P1, rung +10 %)' };

const out = {};
const mount = new El('div');
P.buildJugglePanel(mount);
out.hidden_before_connect = view().hidden;
gate();
await advance(500);                       // connect: judges the pre-filled point (self_toss, the first pattern)
out.connect_calls = nominateCalls().length;

// ---- 1. request string equals Start's; param written first for a param-backed pattern ----
state.nominate = () => Promise.resolve(REFUSED);
calls.length = 0;
await pick('columns_highlow');
out.checking_after_select = view();       // before the debounce has elapsed
out.calls_before_debounce = calls.length;
await advance(450);
out.highlow_calls = calls.map((c) => c.name);
out.highlow_set_param = (calls.find((c) => c.name === '/skill_node/set_parameters') || {}).req;
out.nominate_data = nominateCalls().map((c) => c.req.data);
out.refused_view = view();
calls.length = 0;
await holdStart();
out.start_calls = calls.map((c) => c.name);
out.start_data = (calls.find((c) => c.name === 'jugglebot/juggle_request') || {}).req;
await advance(5000);                      // cooldown over

// ---- 1b. turned pattern: angle rides the nomination, nothing is written ----
calls.length = 0;
await pick('columns_turned');
await advance(450);
out.turned_calls = calls.map((c) => c.name);
out.turned_nominate_data = nominateCalls().map((c) => c.req.data);
calls.length = 0;
await holdStart();
out.turned_start_data = (calls.find((c) => c.name === 'jugglebot/juggle_request') || {}).req.data;
await advance(5000);
// blank param field: key omitted
await type('juggle-columns-angle-deg', '');
calls.length = 0;
await advance(450);
out.blank_angle_data = nominateCalls().map((c) => c.req.data);

// ---- 2. OK reply colouring; typed overrides ride the same string ----
state.nominate = () => Promise.resolve(OK);
calls.length = 0;
await pick('columns');
await type('juggle-apex-m', '0.9');
await advance(450);
out.ok_view = view();
out.typed_nominate_data = nominateCalls().map((c) => c.req.data);
out.typed_start_data = null;
calls.length = 0;
await holdStart();
out.typed_start_data = (calls.find((c) => c.name === 'jugglebot/juggle_request') || {}).req.data;
await advance(5000);

// ---- 3. debounce: three rapid edits make one call ----
calls.length = 0;
await type('juggle-apex-m', '0.91'); await advance(100);
await type('juggle-apex-m', '0.92'); await advance(100);
await type('juggle-apex-m', '0.93'); await advance(450);
out.debounce_calls = nominateCalls().map((c) => c.req.data);

// ---- 4. stale reply dropped ----
const d1 = deferred(), d2 = deferred();
let n = 0;
state.nominate = () => (++n === 1 ? d1.p : d2.p);
calls.length = 0;
await type('juggle-apex-m', '0.94'); await advance(450);     // call 1 out
await type('juggle-apex-m', '0.96'); await advance(450);     // call 2 out; call 1 now stale
d1.res(REFUSED); await flush();
out.stale_after_old_reply = view();
d2.res(OK); await flush();
out.stale_after_new_reply = view();

// ---- 5. service missing: neutral line, Start usable, no error status ----
state.nominate = () => Promise.reject(new Error('Service skills/nominate does not exist'));
await type('juggle-apex-m', '0.95'); await advance(450);
out.missing_view = view();
out.missing_start_disabled = $('juggle-start').disabled;
out.missing_status = { text: $('juggle-status').textContent, cls: $('juggle-status').className };
calls.length = 0;
await holdStart();
out.missing_start_calls = calls.map((c) => c.name);
await advance(5000);

// ---- 6. timeout (never answers) ----
state.nominate = () => new Promise(() => {});
await type('juggle-apex-m', '0.94'); await advance(450);
out.pending_view = view();
await advance(20500);
out.timeout_view = view();
out.timeout_start_disabled = $('juggle-start').disabled;

// ---- 7. a pending nomination is dropped when Start dispatches; no late param write ----
state.nominate = () => Promise.resolve(OK);
await pick('columns_highlow'); await advance(500);
calls.length = 0;
{
    const b = $('juggle-start');
    b.dispatch('pointerdown', { button: 0 });
    await advance(700);
    await type('juggle-apex-m', '0.99');      // schedules a nomination for +400 ms, i.e. after the hold confirms at +800
    await advance(200);                       // Start dispatches here
    b.dispatch('pointerup', { button: 0 });
}
await advance(1000);
out.dispatch_cancels = { nominate_calls: nominateCalls().length, set_params: calls.filter((c) => c.name === '/skill_node/set_parameters').length };

// ---- 7b. a "nomination unavailable:" reply (Attempt running) is neutral, not red ----
await advance(5000);
state.nominate = () => Promise.resolve({ success: false, message: 'nomination unavailable: an Attempt is running' });
await pick('columns'); await advance(450);
out.unavailable_reply_view = view();

// ---- 8. disconnect clears the block ----
await advance(5000);
gate({ connected: false, active: false, mainState: null });
await advance(500);
out.disconnected_hidden = view().hidden;

console.log(JSON.stringify(out));
