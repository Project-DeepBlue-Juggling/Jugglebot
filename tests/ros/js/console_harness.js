/** console_harness.js — drives the REAL console-logic.js (pure) and the CONSOLE mode switch in
 *  udp-traffic.js / console-view.js under node with a fake DOM; prints one JSON object. */
const els = new Map();
function mk(id) {
    const cls = new Set();
    const e = {
        id, innerHTML: '', textContent: '', title: '', value: '', style: { setProperty() {} }, dataset: {},
        clientWidth: 400, clientHeight: 200, scrollTop: 0, scrollHeight: 0, append() {}, replaceChildren() {},
        classList: { add: (c) => cls.add(c), remove: (c) => cls.delete(c), toggle: (c, on) => (on ? cls.add(c) : cls.delete(c)), contains: (c) => cls.has(c) },
        _l: {}, addEventListener(t, fn) { this._l[t] = fn; }, setAttribute() {}, appendChild() {}, remove() {},
        querySelectorAll: () => [], querySelector: () => null,
    };
    return e;
}
globalThis.document = {
    getElementById: (id) => { if (!els.has(id)) els.set(id, mk(id)); return els.get(id); },
    createElement: (t) => mk(t),
    documentElement: {},
};
globalThis.window = { devicePixelRatio: 1 };
globalThis.getComputedStyle = () => ({ getPropertyValue: () => '' });
const store = new Map();
globalThis.localStorage = {
    getItem: (k) => (store.has(k) ? store.get(k) : null),
    setItem: (k, v) => store.set(k, String(v)),
};
const L = await import('./console-logic.js');
const udp = await import('./udp-traffic.js');
const cv = await import('./console-view.js');
const out = {};

// ---- mode switch ----
out.modes = udp.TOPIC_MODES;
out.parse = { console: udp.parseMode('console'), garbled: udp.parseMode('nope'), nul: udp.parseMode(null) };
store.set('jugglebot-topics-mode', 'console');
for (const id of ['udp-table-container', 'console-container', 'topic-table-container', 'topics-panel-title']) els.set(id, mk(id));
udp.initUdpTrafficPanel();
out.title = els.get('topics-panel-title').textContent;
out.display = { console: els.get('console-container').style.display, udp: els.get('udp-table-container').style.display, ros: els.get('topic-table-container').style.display };

// ---- filter + search combine ----
const lines = [
    { t: 10, level: 20, name: 'a_node', msg: 'hello world' },
    { t: 11, level: 30, name: 'b_node', msg: 'Hello <b>there</b>' },
    { t: 12, level: 40, name: 'a_node', msg: 'boom' },
    { t: 13, level: 10, name: 'c_node', msg: 'HELLO debug' },
    { t: 14, level: 50, name: 'hello_node', msg: 'dead' },
];
const all = { DEBUG: true, INFO: true, WARN: true, ERROR: true, FATAL: true };
out.f_all = L.filterIndices(lines, all, '');
out.f_q = L.filterIndices(lines, all, 'hello');
out.f_q_nodeonly = L.filterIndices(lines, all, 'B_NODE');
out.f_q_warnoff = L.filterIndices(lines, { ...all, WARN: false }, 'hello');
out.f_set = L.filterIndices(lines, new Set(['ERROR', 'FATAL']), '');
out.counts_q = L.levelCounts(lines, 'hello');
out.counts_all = L.levelCounts(lines, '');
out.lvl = [10, 20, 30, 40, 50, 0, 25, 99].map(L.levelName);

// ---- escaping ----
const evil = { t: 1, level: 20, name: 'n<script>', msg: '<img src=x onerror=alert(1)> & "q"' };
out.html = L.lineHtml(evil, 0, '', false);
out.html_hl = L.lineHtml(evil, 0, 'img', false);
out.hl = L.highlightHtml('a<b>B', 'b');
out.norm = L.normalizeLine({ stamp: { sec: 5, nanosec: 500000000 }, level: 30, name: 'x', msg: 'm' }, 0);
out.parseEnabled = [L.parseEnabled(null), L.parseEnabled('{bad'), L.parseEnabled('{"INFO":false}')];

// ---- ring cap ----
const ring = [];
for (let i = 0; i < L.RING_CAP + 123; i++) L.pushCapped(ring, { t: i, level: 20, name: 'n', msg: String(i) });
out.ring = { len: ring.length, first: ring[0].msg, last: ring[ring.length - 1].msg, cap: L.RING_CAP };
cv.initConsoleView();
// ---- window label + pinned startup lines (JB-163) ----
{
    const on = { DEBUG: true, INFO: true, WARN: true, ERROR: true, FATAL: true };
    const st = L.newLive();
    const fw = { t: 1, level: 20, name: 'bridge', msg: 'BRIDGE_FW_CHECK: OK - can-bridge v38' };
    L.pushLive(st, fw);
    for (let i = 0; i < L.RING_CAP + 10; i++) L.pushLive(st, { t: 2 + i, level: 20, name: 'n', msg: String(i) });
    const all = L.liveLines(st);
    out.pin = { len: all.length, first: all[0].msg, dropped: st.dropped, ringLen: st.ring.length,
                find: L.filterIndices(all, on, 'v38'),
                infoOff: L.filterIndices(all, { ...on, INFO: false }, 'v38'),
                isPinned: [L.isPinned(fw), L.isPinned({ msg: 'x' }), L.isPinned(null)] };
    const mid = L.newLive();   // time order: a pinned line arriving mid-stream sits between its neighbours
    L.pushLive(mid, { t: 1, level: 20, name: 'n', msg: 'a' });
    L.pushLive(mid, { t: 2, level: 20, name: 'n', msg: 'PLATFORM_FW_CHECK: OK' });
    L.pushLive(mid, { t: 3, level: 20, name: 'n', msg: 'b' });
    out.pinOrder = L.liveLines(mid).map((l) => l.msg);
    const pb = L.newLive();
    for (let i = 0; i < L.PIN_CAP + 7; i++) L.pushLive(pb, { t: i, level: 20, name: 'n', msg: 'X_FW_CHECK ' + i });
    out.pinBound = { len: pb.pinned.length, first: pb.pinned[0].msg, dropped: pb.dropped, cap: L.PIN_CAP };
    out.winFresh = L.windowLabel(L.newLive(), 3600);
    out.winDropped = L.windowLabel(st, 3600);
    out.winPinCap = L.windowLabel(pb, 3600);      // pinned cap dropped, ring never did
    const small = (pin) => {                      // ring cap 3 over 5 lines t=10..14 keeps t=12..14
        const s = L.newLive();
        if (pin !== undefined) L.pushLive(s, { t: pin, level: 20, name: 'n', msg: 'A_FW_CHECK' });
        for (let i = 0; i < 5; i++) L.pushLive(s, { t: 10 + i, level: 20, name: 'n', msg: 'm' + i }, 3);
        return s;
    };
    out.winRingOnly = L.windowLabel(small(), 3600);
    out.winPinOld = L.windowLabel(small(1), 3600);
    const tie = L.newLive();
    L.pushLive(tie, { t: 5, level: 20, name: 'n', msg: 'm' });
    L.pushLive(tie, { t: 5, level: 20, name: 'n', msg: 'B_FW_CHECK' });
    out.winTie = { label: L.windowLabel(small(12), 3600), order: L.liveLines(tie).map((l) => l.msg) };
    out.winRingT = L.formatTime(12).slice(0, 8);
    out.winDroppedExpect = L.formatTime(st.ring[0].t).slice(0, 8);
    out.winFreshExpect = L.formatTime(3600).slice(0, 8);
}
out.viewHost = els.get('console-container').innerHTML.includes('console-search');

// ---- replay hit selection ----
const rl = [1, 2, 3, 4, 5].map((t) => ({ t, level: 20, name: 'n', msg: t % 2 ? 'odd' : 'even' }));
const hits = L.filterIndices(rl, all, 'odd');   // idx 0,2,4 at t=1,3,5
out.hit = {
    hits,
    cur_before: L.currentHit(rl, hits, 0.5), cur_mid: L.currentHit(rl, hits, 3.4), cur_on: L.currentHit(rl, hits, 3),
    prev_mid: L.prevHit(rl, hits, 3.4), prev_on: L.prevHit(rl, hits, 3), prev_start: L.prevHit(rl, hits, 1),
    next_mid: L.nextHit(rl, hits, 3.4), next_on: L.nextHit(rl, hits, 3), next_end: L.nextHit(rl, hits, 5),
    none: [L.currentHit(rl, [], 3), L.prevHit(rl, [], 3), L.nextHit(rl, [], 3)],
};
// ---- JB-135: hover marker ----
const row = (t) => ({ closest: (sel) => (sel === '.console-line' ? { dataset: { t } } : null) });
out.hover = {
    row: L.hoverTime(row('12.5')), nan: L.hoverTime(row('abc')), missing: L.hoverTime(row(undefined)),
    nonrow: L.hoverTime({ closest: () => null }), nul: L.hoverTime(null), nofn: L.hoverTime({}),
};
const markCalls = [];
let modeCb = null;
const fakeMode = {
    on: (evt, cb) => { if (evt === 'state') modeCb = cb; return () => {}; },
    engine: () => ({ on: () => () => {}, state: () => ({ playhead: 0 }), seek() {} }),
    source: () => ({ range: () => ({ t0: 0, t1: 0 }), chunkIndex: () => 0, load: async () => null }),
    setMarker: (t) => markCalls.push(t),
};
cv.attachConsoleReplay(fakeMode);
const listEl = els.get('console-list');
out.hover.registered = Object.keys(listEl._l).sort();
listEl._l.mouseover({ target: row('7') });                       // live: nothing published
out.hover.live = markCalls.slice();
modeCb({ mode: 'REPLAY' });
listEl._l.mouseover({ target: row('7') });
listEl._l.mouseout({ relatedTarget: null });
listEl._l.mouseout({ relatedTarget: row('9') });                 // onto another row: that row's time
out.hover.replay = markCalls.slice();
cv.setConsoleVisible(false);                                     // hidden by keyboard, mouse still on row 9
out.hover.hidden = markCalls.slice();
modeCb({ mode: 'LIVE' });                                        // leaving replay clears once
const n = markCalls.length;
listEl._l.mouseover({ target: row('7') });
out.hover.after_exit = { calls: markCalls.slice(), extra: markCalls.length - n };
console.log(JSON.stringify(out));
process.exit(0);
