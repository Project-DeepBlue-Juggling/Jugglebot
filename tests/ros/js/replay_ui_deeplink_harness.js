// replay_ui_deeplink_harness.js — the REAL replay/ui/deeplink.js under node (TZ pinned by the pytest wrapper) with fake
// ros / mode / toaster / document. Prints one JSON object tests/ros/test_gui_replay_ui_deeplink.py asserts on.
const { parseDeepLink, resolveT, buildLink, copyText, createDeepLink } = await import('./replay/ui/deeplink.js');

const out = {};
// ---- parse ----
const P = (s) => parseDeepLink(s);
out.parse = {
    offset: P('?recording=2026-10-09_14-00-00&t=83.4'),
    int: P('?recording=a&t=5'),
    clock: P('?recording=a&t=14:03:20'),
    clock_frac: P('?recording=a&t=14:03:20.5'),
    clock_encoded: P('?recording=a&t=14%3A03%3A20'),
    none: P('?recording=a'),
    junk: P('?recording=a&t=abc'),
    bad_clock: P('?recording=a&t=25:00:00'),
    neg: P('?recording=a&t=-3'),
    no_rec: P('?t=5'),
    empty: P(''),
    other_params: P('?foo=1'),
};
// ---- resolve ----
const t0 = new Date(2026, 9, 9, 14, 0, 0).getTime() / 1000;          // local 14:00:00
const R = (t, dur) => resolveT(t, t0, dur);
out.resolve = {
    offset: R({ kind: 'offset', sec: 83.4 }, 600),
    clamp_hi: R({ kind: 'offset', sec: 9999 }, 600),
    clock: R({ kind: 'clock', sod: 14 * 3600 + 3 * 60 + 20.5 }, 600),
    clock_before: R({ kind: 'clock', sod: 13 * 3600 }, 600),
    clock_after: R({ kind: 'clock', sod: 15 * 3600 }, 600),
    none: R({ kind: 'none' }, 600),
    invalid: R({ kind: 'invalid', raw: 'x' }, 600),
};
const tm = new Date(2026, 9, 9, 23, 58, 0).getTime() / 1000;         // 23:58, 10-minute recording crossing midnight
out.midnight = [
    resolveT({ kind: 'clock', sod: 23 * 3600 + 59 * 60 }, tm, 600).sec,      // before midnight: 60 s
    resolveT({ kind: 'clock', sod: 4 * 60 }, tm, 600).sec,                   // 00:04 next day: 360 s
    resolveT({ kind: 'clock', sod: 4 * 3600 }, tm, 600),                      // nowhere near: clamped
];
// ---- build round trip ----
const url = buildLink('http://jetson:8081', '/index.html', '2026-10-09_14-00-00', 83.449);
const q = url.slice(url.indexOf('?'));
const endUrl = buildLink('', '/', 'r', 83.46);       // a playhead at the very end must not round up past it
const endBack = parseDeepLink(endUrl.slice(endUrl.indexOf('?')));
out.build = { url, back: parseDeepLink(q), end_t: endUrl.split('t=')[1], end_resolved: resolveT(endBack.t, 0, 83.46),
    exact: [0.3, 0.7, 1.1, 2.9].map((x) => buildLink('', '/', 'r', x).split('t=')[1]) };

// ---- copyText ----
const mkDoc = (execOk) => {
    const log = { appended: 0, removed: 0, value: null, cmd: null };
    const ta = { style: {}, setAttribute() {}, select() {}, remove() { log.removed++; } };
    return { log, body: { appendChild() { log.appended++; } },
        createElement: () => ta, execCommand: (c) => { log.cmd = c; log.value = ta.value; return execOk; } };
};
let d1 = mkDoc(true);
out.copy_fallback = { ok: await copyText(d1, 'U1', undefined), log: d1.log };
let written = null;
let d2 = mkDoc(true);
out.copy_clip = { ok: await copyText(d2, 'U2', { clipboard: { writeText: async (t) => { written = t; } } }), written, appended: d2.log.appended };
let d3 = mkDoc(true);
out.copy_clip_rejects = { ok: await copyText(d3, 'U3', { clipboard: { writeText: async () => { throw new Error('denied'); } } }), cmd: d3.log.cmd };
out.copy_fails = await copyText(mkDoc(false), 'U4', undefined);

// ---- controller ----
const T0R = new Date(2026, 9, 9, 14, 0, 0).getTime() / 1000;
function rig(search, ids, opts) {
    opts = opts || {};
    let state = 'disconnected';
    const cbs = [];
    const toasts = [], seeks = [], enters = [];
    let active = false;
    const eng = { seek: async (t) => { seeks.push(t); } };
    const mode = {
        isActive: () => active,
        enterReplay: async (spec) => { enters.push(spec); if (opts.reject) { const e = new Error('x'); e.reason = opts.reject; throw e; } active = true; },
        engine: () => (active ? eng : null),
        source: () => ({ range: () => ({ t0: T0R, t1: T0R + 600, frontier: T0R + 600 }) }),
    };
    const ros = { onConnectionStateChange(cb) { cbs.push(cb); cb(state); } };
    const dl = createDeepLink({ search, ros, mode, toaster: { show: (t) => toasts.push(t) },
        list: async () => ids, refusalMessage: (r) => 'REFUSED ' + r });
    const set = async (s) => { state = s; cbs.forEach((f) => f(s)); for (let i = 0; i < 20; i++) await new Promise((r) => setImmediate(r)); };
    return { dl, set, toasts, seeks, enters, exit: () => { active = false; } };
}
const IDS = ['2026-10-09_14-00-00', 'other'];
const L = '?recording=2026-10-09_14-00-00&t=83.4';
{   // honoured from the first failed connect, paused at t, once
    const r = rig(L, IDS);
    out.initial_no_action = { enters: r.enters.length, outcome: r.dl.state().outcome };   // initial 'disconnected' default: not settled
    await r.set('connecting'); const mid = r.enters.length;
    await r.set('disconnected');
    out.honour = { mid, enters: r.enters, seeks: r.seeks, toasts: r.toasts, outcome: r.dl.state().outcome };
    r.exit(); await r.set('connecting'); await r.set('disconnected');
    out.honour_once = { enters: r.enters.length, seeks: r.seeks.length };
}
{   // connected -> refuse
    const r = rig(L, IDS);
    await r.set('connecting'); await r.set('connected');
    await r.set('disconnected'); await r.set('connecting'); await r.set('disconnected');
    out.refuse = { enters: r.enters.length, toasts: r.toasts, outcome: r.dl.state().outcome };
}
{   // already connected when the listener registers
    const r = (() => { let st = 'connected'; const toasts = []; let enters = 0;
        const dl = createDeepLink({ search: L, ros: { onConnectionStateChange(cb) { cb(st); } },
            mode: { isActive: () => false, enterReplay: async () => { enters++; } }, toaster: { show: (t) => toasts.push(t) }, list: async () => IDS });
        return { dl, toasts, enters: () => enters }; })();
    out.refuse_at_register = { enters: r.enters(), toasts: r.toasts };
}
{   // not found
    const r = rig('?recording=nope&t=3', IDS);
    await r.set('connecting'); await r.set('disconnected');
    out.not_found = { enters: r.enters.length, toasts: r.toasts, outcome: r.dl.state().outcome };
}
{   // clamp + invalid + no t + clock
    const run = async (s) => { const r = rig(s, IDS); await r.set('connecting'); await r.set('disconnected'); return { seeks: r.seeks, toasts: r.toasts }; };
    out.clamp = await run('?recording=2026-10-09_14-00-00&t=9999');
    out.invalid = await run('?recording=2026-10-09_14-00-00&t=abc');
    out.no_t = await run('?recording=2026-10-09_14-00-00');
    out.clock_t = await run('?recording=2026-10-09_14-00-00&t=14:02:00');
}
{   // refusal at open
    const r = rig(L, IDS, { reject: 'in_progress' });
    await r.set('connecting'); await r.set('disconnected');
    out.open_refused = { toasts: r.toasts, seeks: r.seeks.length };
}
{   // no link -> inert
    const r = rig('?foo=1', IDS);
    await r.set('connecting'); await r.set('disconnected');
    out.no_link = { enters: r.enters.length, outcome: r.dl.state().outcome };
}
out.T0R = T0R;
console.log(JSON.stringify(out));
