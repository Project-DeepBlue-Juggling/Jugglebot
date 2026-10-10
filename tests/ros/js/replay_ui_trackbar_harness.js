// replay_ui_trackbar_harness.js — the REAL replay/ui/{dom,overview,trackbar}.js under node with a minimal fake DOM,
// a fake engine (state object + recorded calls) and a fake mode. Prints one JSON object that
// tests/ros/test_gui_replay_ui_trackbar.py asserts on.
// Sandbox layout: replay/ui/*.js, this file, package.json {"type":"module"}.
import { createTrackbar, nextSpeed } from './replay/ui/trackbar.js';
import * as ov from './replay/ui/overview.js';

class El {
    constructor(tag) {
        this.tag = tag; this.children = []; this.parentNode = null; this.listeners = {}; this.attrs = {};
        this.className = ''; this.id = ''; this.hidden = false; this.title = ''; this._text = ''; this.style = {};
        this.rect = { left: 0, width: 1000 };
    }
    get textContent() { return this.children.length ? this.children.map((c) => c.textContent).join('') : this._text; }
    set textContent(v) { this.children = []; this._text = String(v); }
    appendChild(c) { c.parentNode = this; this.children.push(c); return c; }
    replaceChildren() { this.children = []; this._text = ''; }
    setAttribute(k, v) { this.attrs[k] = v; }
    addEventListener(t, f) { (this.listeners[t] = this.listeners[t] || []).push(f); }
    getBoundingClientRect() { return this.rect; }
    fire(t, ev) { for (const f of this.listeners[t] || []) f(ev || {}); }
    find(pred, out = []) { if (pred(this)) out.push(this); for (const c of this.children) c.find(pred, out); return out; }
}
const body = new El('body');
const dl = {};
const doc = {
    createElement: (t) => new El(t),
    getElementById: (id) => body.find((e) => e.id === id)[0] || null,
    // dl[t] is a Map listener -> capture flag; removal only matches when the flag matches (like the DOM).
    addEventListener(t, f, cap) { (dl[t] = dl[t] || new Map()).set(f, !!cap); },
    removeEventListener(t, f, cap) { if (dl[t] && dl[t].get(f) === !!cap) dl[t].delete(f); },
};
// capture listeners first, then bubble; stopImmediatePropagation halts the rest (the DOM's dispatch order on one node)
const emitDoc = (t, ev) => {
    ev.stopImmediatePropagation = () => { ev.stopped = true; };
    const all = Array.from(dl[t] || []);
    for (const [f] of all.filter((x) => x[1]).concat(all.filter((x) => !x[1]))) { if (ev.stopped) break; f(ev); }
};
const nListeners = (t) => (dl[t] ? dl[t].size : 0);
const dock = new El('div'); dock.id = 'replay-dock'; body.appendChild(dock);

const T0 = 1791532800;   // epoch s
const state = { mode: 'paused', playhead: T0 + 10, speed: 1, frontier: T0 + 100, range: { t0: T0, t1: T0 + 100, frontier: T0 + 100 } };
let calls = [];
const rec = (n) => (...a) => { calls.push([n].concat(a)); return Promise.resolve(); };
const eng = {
    state: () => state, play: rec('play'), pause: rec('pause'), seek: rec('seek'), scrub: rec('scrub'),
    setSpeed: rec('setSpeed'), step: rec('step'),
};
let engine = null;
let active = false;
const lis = [];
let timelineObj = {
    bands: [{ topic: '/orchestrator_state', segments: [[T0, T0 + 20, 'IDLE'], [T0 + 20, T0 + 60, 'ACTIVE'], [T0 + 60, T0 + 100, 'FAULT']] }],
    ticks: [{ t: T0 + 30, kind: 'skill_attempt', label: 'hop' }, { t: T0 + 50, kind: 'catch_event', label: '' },
            { t: T0 + 5, kind: 'homed', label: '' }, { t: T0 + 80, kind: 'fault', label: 'X' }],
};
let srcState = 'complete';
const source = { range: () => state.range, status: () => ({ state: srcState }), timeline: () => Promise.resolve(timelineObj) };
const mode = {
    isActive: () => active, engine: () => engine, source: () => source,
    on(evt, cb) { lis.push(cb); return () => {}; },
    exitReplay: rec('exitReplay'),
};
const emitMode = () => lis.forEach((f) => f({}));
const keyEv = (key, o) => { const ev = Object.assign({ key, target: { tag: 'div' }, prevented: false }, o || {}); ev.preventDefault = () => { ev.prevented = true; }; return ev; };
const tick = () => new Promise((r) => setImmediate(r));
const clockOf = (t) => { const d = new Date(t * 1000); const p = (n) => (n < 10 ? '0' : '') + n; return p(d.getHours()) + ':' + p(d.getMinutes()) + ':' + p(d.getSeconds()); };

const out = {};
const ids = (id) => body.find((e) => e.id === id)[0];

// ---- mount / unmount from mode state events ----
const tb = createTrackbar({ document: doc, mode, dock, raf: null });
out.idle = { mounted: tb.mounted(), dock_kids: dock.children.length, keydown: nListeners('keydown') };
active = true; engine = null; emitMode();                     // OPENING: no engine yet
out.opening = { mounted: tb.mounted(), dock_kids: dock.children.length, keydown: nListeners('keydown') };
engine = eng; tb.frame(); await tick(); tb.frame();
out.replay = { mounted: tb.mounted(), keydown: nListeners('keydown'), mousemove: nListeners('mousemove') };

// ---- readouts ----
out.readouts = {
    clock: ids('rp-clock').textContent, expect_clock: clockOf(state.playhead),
    elapsed: ids('rp-elapsed').textContent, speed: ids('rp-speed').textContent,
    play_btn: ids('rp-play').textContent, head_left: dock.find((e) => e.className === 'rp-playhead')[0].style.left,
    buffering_hidden: ids('rp-buffering').hidden,
    conv_els: dock.find((e) => e.className === 'rp-conv').length, front_els: dock.find((e) => e.className === 'rp-front').length,
    date: dock.find((e) => e.className === 'rp-date')[0].textContent,
    bands: dock.find((e) => e.className === 'rp-band').length, ticks: dock.find((e) => e.className === 'rp-tick').length,
    tick_titles: dock.find((e) => e.className === 'rp-tick').map((e) => e.title),
};

// ---- buffering (a gap in the resident window; the converted-span fill / frontier marker are gone) ----
state.mode = 'buffering'; state.frontier = T0 + 40; state.range = { t0: T0, t1: T0 + 40, frontier: T0 + 40 }; state.playhead = T0 + 40;
tb.frame();
out.progressive = {
    buffering_hidden: ids('rp-buffering').hidden, elapsed: ids('rp-elapsed').textContent,
    play_btn: ids('rp-play').textContent,
};
state.mode = 'paused'; state.frontier = T0 + 100; state.range = { t0: T0, t1: T0 + 100, frontier: T0 + 100 }; state.playhead = T0 + 10; state.speed = 1;
tb.frame();

// ---- buttons ----
const click = (id) => { calls = []; ids(id).fire('click', {}); return calls.map((c) => c.join(':')); };
out.buttons = {
    play: click('rp-play'), home: click('rp-home'), end: click('rp-end'), stepb: click('rp-stepb'), stepf: click('rp-stepf'),
    exit: click('rp-exit'),
};
state.mode = 'playing';
out.buttons.pause = click('rp-play');
state.mode = 'paused';

// ---- ladder ----
const seq = (start, dir, n) => { const o = []; let s = start; for (let i = 0; i < n; i++) { s = nextSpeed(s, dir); o.push(s); } return o; };
out.ladder = {
    ff_from_1: seq(1, 1, 5), rw_from_1: seq(1, -1, 6), ff_from_rev: seq(-2, 1, 4), ff_cap: seq(8, 1, 1), rw_cap: seq(-8, -1, 1),
    ff_click: (() => { state.speed = 1; return click('rp-ff'); })(), rw_click: (() => { state.speed = 1; return click('rp-rw'); })(),
};

// ---- hotkeys ----
const key = (k, o) => { calls = []; const ev = keyEv(k, o); const handled = emitDoc('keydown', ev); return { calls: calls.map((c) => c.map((x) => (typeof x === 'number' && x > 1e8 ? +(x - T0).toFixed(3) : x)).join(':')), prevented: ev.prevented }; };
state.playhead = T0 + 50; state.mode = 'paused';
out.keys = {
    space_play: key(' '), left: key('ArrowLeft'), right: key('ArrowRight'), sleft: key('ArrowLeft', { shiftKey: true }), sright: key('ArrowRight', { shiftKey: true }),
    comma: key(','), period: key('.'), home: key('Home'), end: key('End'), unrelated: key('q'),
    in_input: (() => { calls = []; const ev = keyEv(' ', { target: { tag: 'INPUT' } }); emitDoc('keydown', ev); return { calls: calls.length, prevented: ev.prevented }; })(),
    ctrl: (() => { calls = []; const ev = keyEv('ArrowLeft', { ctrlKey: true }); emitDoc('keydown', ev); return calls.length; })(),
};
state.mode = 'playing'; out.keys.space_pause = key(' '); state.mode = 'paused';
state.playhead = T0 + 99.5; out.keys.right_clamped = key('ArrowRight');
state.playhead = T0 + 0.4; out.keys.left_clamped = key('ArrowLeft');
state.playhead = T0 + 50;

// ---- scrub / seek on the bar ----
const bar = ids('rp-bar'); bar.rect = { left: 100, width: 1000 };
const rel = (calls_) => calls_.map((c) => c.map((x) => (typeof x === 'number' && x > 1e8 ? +(x - T0).toFixed(3) : x)).join(':'));
// 60 moves inside one frame -> exactly one scrub (the last position); one seek on release
calls = []; bar.fire('mousedown', { clientX: 600, preventDefault() {} });
out.drag_before_frame = rel(calls);                 // nothing reaches the engine until the frame
for (let i = 0; i < 60; i++) emitDoc('mousemove', { clientX: 600 + i * 5 });   // last = 895 -> 79.5 s
tb.frame();
out.drag_after_frame = rel(calls);
tb.frame(); tb.frame();                              // no new move: no further scrub
emitDoc('mouseup', { clientX: 800 });
out.drag = rel(calls);
// pointer rest: REST frames after the last move run one seek, the drag stays live; mouseup seeks again
calls = []; bar.fire('mousedown', { clientX: 600, preventDefault() {} }); tb.frame();
for (let i = 0; i < 20; i++) tb.frame();
out.drag_rest = rel(calls);
emitDoc('mousemove', { clientX: 700 }); tb.frame();
emitDoc('mouseup', { clientX: 700 });
out.drag_rest_after = rel(calls);

// ---- zoom: keys, drag-select, clamp ----
state.playhead = T0 + 50;
emitDoc('keydown', keyEv('+')); tb.frame();
out.zoom_in = { view: tb.getView() && { v0: +(tb.getView().v0 - T0).toFixed(3), v1: +(tb.getView().v1 - T0).toFixed(3) }, tag: dock.find((e) => e.className === 'rp-zoomtag')[0].textContent };
out.zoom_ticks = dock.find((e) => e.className === 'rp-tick').length;
emitDoc('keydown', keyEv('-')); out.zoom_out = tb.getView();
const ovEl = ids('rp-ov'); ovEl.rect = { left: 0, width: 1000 };
ovEl.fire('mousedown', { clientX: 200, preventDefault() {} }); emitDoc('mousemove', { clientX: 400 }); emitDoc('mouseup', { clientX: 400 });
out.drag_select = { v0: +(tb.getView().v0 - T0).toFixed(3), v1: +(tb.getView().v1 - T0).toFixed(3) };
tb.frame();
out.drag_select_ticks = dock.find((e) => e.className === 'rp-tick').map((e) => e.title);
calls = []; ovEl.fire('mousedown', { clientX: 500, preventDefault() {} }); emitDoc('mouseup', { clientX: 500 });
out.ov_click = calls.map((c) => c.map((x) => (typeof x === 'number' && x > 1e8 ? +(x - T0).toFixed(3) : x)).join(':'));
emitDoc('keydown', keyEv('Escape')); out.esc = tb.getView();
emitDoc('keydown', keyEv('+')); ovEl.fire('dblclick', {}); out.dbl = tb.getView();

// ---- pure math ----
const full = { t0: 0, t1: 100 };
out.math = {
    clamp_lo: ov.clampRange(-5, 15, full), clamp_hi: ov.clampRange(95, 115, full), clamp_all: ov.clampRange(-10, 200, full), clamp_min: ov.clampRange(50, 50.5, full),
    zoom_about: ov.zoomAbout({ v0: 0, v1: 100 }, 2, 25, full), zoom_edge: ov.zoomAbout({ v0: 0, v1: 100 }, 4, 0, full),
    zoom_out: ov.zoomAbout({ v0: 40, v1: 60 }, 0.5, 50, full),
    drag: ov.dragToRange(0.2, 0.4, { v0: 0, v1: 100 }, full, 0.01), drag_rev: ov.dragToRange(0.4, 0.2, { v0: 50, v1: 100 }, full, 0.01),
    drag_click: ov.dragToRange(0.5, 0.505, { v0: 0, v1: 100 }, full, 0.01),
    bands_full: ov.layoutBands(timelineObj, T0, T0 + 100).map((b) => [b.name, +b.l.toFixed(4), +b.w.toFixed(4)]),
    bands_zoom: ov.layoutBands(timelineObj, T0 + 10, T0 + 70).map((b) => [b.name, +b.l.toFixed(4), +b.w.toFixed(4)]),
    ticks_full: ov.layoutTicks(timelineObj, T0, T0 + 100).map((k) => [k.kind, +k.l.toFixed(4)]),
    ticks_zoom: ov.layoutTicks(timelineObj, T0 + 25, T0 + 55).map((k) => [k.kind, +k.l.toFixed(4)]),
};

// ---- W1: Space must not also reach the live GUI's own bubble-phase shortcut (chart pause) ----
{
    let livePause = 0, liveQ = 0;
    const live = (ev) => { if (ev.key === ' ') livePause++; if (ev.key === 'q') liveQ++; };
    doc.addEventListener('keydown', live);               // bubble phase, registered BEFORE/independently of the trackbar
    calls = []; state.mode = 'paused';
    const sp = keyEv(' '); emitDoc('keydown', sp);
    const un = keyEv('q'); emitDoc('keydown', un);
    out.space_double_fire = { trackbar_calls: calls.map((c) => c[0]), live_pause: livePause, live_unhandled_q: liveQ, prevented: sp.prevented };
    const inInput = keyEv(' ', { target: { tag: 'INPUT' } }); emitDoc('keydown', inInput);
    out.space_double_fire.live_pause_in_input = livePause;   // typing in an input is not ours: live handler still sees it
    doc.removeEventListener('keydown', live);
}

// ---- W3: a failing /overview is retried at most a couple of times over 300 frames ----
{
    const tb2dock = new El('div'); tb2dock.id = 'dock2';
    let fetches = 0;
    const failingSource = { range: () => state.range, status: () => ({ state: 'complete' }),
        timeline: () => { fetches++; return Promise.resolve({ partial: true, bands: [], ticks: [] }); } };
    const mode2 = { isActive: () => true, engine: () => eng, source: () => failingSource, on: () => () => {}, exitReplay: rec('exitReplay') };
    const tb2 = createTrackbar({ document: doc, mode: mode2, dock: tb2dock, raf: null });
    for (let i = 0; i < 300; i++) { tb2.frame(); if (i % 50 === 0) await tick(); }
    out.overview_refetch = { fetches };
    tb2.dispose();
}

// ---- Phase 4: a source onChange (the /overview poll landing) re-reads a partial timeline at once ----
{
    const tb3dock = new El('div'); tb3dock.id = 'dock3';
    let tl3 = { partial: true, presence: {} }; let cb3 = null, unsubs = 0;
    const src3 = { range: () => state.range, status: () => ({ state: 'complete' }), timeline: () => Promise.resolve(tl3),
        onChange: (cb) => { cb3 = cb; return () => { unsubs++; cb3 = null; }; } };
    const mode3 = { isActive: () => true, engine: () => eng, source: () => src3, on: () => () => {}, exitReplay: rec('exitReplay') };
    const tb3 = createTrackbar({ document: doc, mode: mode3, dock: tb3dock, raf: null });
    for (let i = 0; i < 5; i++) { tb3.frame(); await tick(); }
    const before = tb3dock.find((e) => e.className === 'rp-band').length;
    tl3 = Object.assign({}, timelineObj);
    cb3();                                   // the source's overview poll landed
    await tick(); tb3.frame();
    out.overview_onchange = { bands_before: before, bands_after: tb3dock.find((e) => e.className === 'rp-band').length, subscribed: cb3 !== null };
    tb3.dispose();
    out.overview_onchange.unsubscribed = unsubs;
}

// ---- unmount ----
active = false; emitMode();
out.exit = { mounted: tb.mounted(), dock_kids: dock.children.length, keydown: nListeners('keydown'), mousemove: nListeners('mousemove'), mouseup: nListeners('mouseup') };
calls = []; emitDoc('keydown', keyEv(' ')); out.exit.key_calls = calls.length;
active = true; emitMode(); out.remount = { mounted: tb.mounted(), kids: dock.children.length };
console.log(JSON.stringify(out));
