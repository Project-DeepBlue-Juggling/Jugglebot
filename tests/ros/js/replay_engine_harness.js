// replay_engine_harness.js — drives ros_ws/gui/js/replay/engine.js over a
// synthetic 60 s feed and prints one JSON object of measurements;
// tests/ros/test_gui_replay_engine.py asserts on it.
//
// Sandbox layout (built by the pytest fixture, verbatim copies):
//   clock.js, replay/{chunk,policy,engine}.js, this file, package.json {"type":"module"}
//
// Feed (in-browser chunk shape, built with the real chunk.js makeTopic):
//   /robot_state        98 Hz, has_fatal_odrive_error true on [25, 35)
//   /orchestrator_state 10 Hz, BOOT, IDLE@5, ACTIVE@20, IDLE@21.5, ERROR@40 (0.3 s), IDLE@40.3
//   /mocap_data         180 Hz
//   /hand_telemetry     100 Hz, stops at 40 (the watchdog scenario)
//   /skills/attempt     3 events at 12.3, 33.7, 51.2
import * as clock from './clock.js';
import { makeTopic, flattenMessage } from './replay/chunk.js';
import { createEngine, makeEagerCache } from './replay/engine.js';
import { SPEED_LADDER, TOPIC_POLICY } from './replay/policy.js';

const T0 = 1791532600.0;
const DUR = 60;
const NCH = 6;

function orchAt(x) {
    if (x < 5) return 'BOOT';
    if (x < 20) return 'IDLE';
    if (x < 21.5) return 'ACTIVE';
    if (x < 40) return 'IDLE';
    if (x < 40.3 - 1e-9) return 'ERROR';
    return 'IDLE';
}

function series(rate, from, to, mk) {
    const out = [];
    const n = Math.ceil((to - from) * rate - 1e-9);
    for (let k = 0; k < n; k++) {
        const x = from + k / rate;
        out.push({ t: T0 + x, msg: mk(x, k) });
    }
    return out;
}

const RECORDS = {
    '/robot_state': series(98, 0, DUR, (x, k) => ({
        has_fatal_odrive_error: x >= 25 && x < 35, has_fatal_can_error: false,
        has_undervoltage: false, motor_states: [], seq: k })),
    '/orchestrator_state': series(10, 0, DUR, (x) => ({ data: orchAt(x) })),
    '/mocap_data': series(180, 0, DUR, (x, k) => ({ aligned: true, markers: [], seq: k })),
    '/hand_telemetry': series(100, 0, 40, (x, k) => ({ ball_held: true, seq: k })),
    '/skills/attempt': [12.3, 33.7, 51.2].map((x, k) => ({
        t: T0 + x, msg: { name: 'juggle_start', message: 'ev' + k, values: [] } })),
};
const STATE_TOPICS = Object.keys(RECORDS).filter((t) => ['state-render', 'state-edge'].includes(TOPIC_POLICY[t].cls));

function buildChunks() {
    const chunks = [];
    for (let i = 0; i < NCH; i++) {
        const c0 = T0 + 10 * i;
        const c1 = c0 + 10;
        const topics = {};
        for (const name in RECORDS) {
            const recs = RECORDS[name].filter((r) => r.t >= c0 && r.t < c1);
            if (!recs.length) continue;
            const cols = {};
            recs.forEach((r, k) => {
                const flat = flattenMessage(r.msg);
                for (const c in flat) (cols[c] = cols[c] || new Array(recs.length))[k] = flat[c];
            });
            topics[name] = makeTopic('t/' + name, Float64Array.from(recs.map((r) => r.t)), cols);
        }
        chunks.push({ i, t0: c0, t1: c1, topics });
    }
    return chunks;
}
const CHUNKS = buildChunks();

function makeSource(opt) {
    opt = opt || {};
    const st = { frontier: opt.frontier === undefined ? T0 + DUR : opt.frontier, state: opt.state || 'complete' };
    return {
        kind: 'recording',
        _st: st,
        range() { return { t0: T0, t1: st.state === 'complete' ? T0 + DUR : st.frontier, frontier: st.frontier }; },
        chunkIndex(t) { return Math.floor((t - T0) / 10); },
        bounds(i) { return [T0 + 10 * i, T0 + 10 * (i + 1)]; },
        async load(i) {
            if (!(i >= 0 && T0 + 10 * i < st.frontier && i < NCH)) throw new RangeError('chunk ' + i);
            return CHUNKS[i];
        },
        peek(i) { return CHUNKS[i] || null; },
        status() { return { state: st.state, chunksDone: NCH, chunksTotal: NCH }; },
    };
}

let curTick = 0;
function makeSpy() {
    const log = [];
    const fires = [];
    let wd = null;
    function dispatch(topic, msg, t, o) {
        log.push({ topic, t, muted: !!(o && o.muted), tick: curTick, data: msg.data });
        if (topic === '/hand_telemetry') {
            // Handler-armed staleness watchdog, the main.js onHandTelemetry idiom.
            if (wd !== null) clock.clearTimeout(wd);
            wd = clock.setTimeout(() => { wd = null; fires.push(clock.now() / 1000 - T0); }, 1000);
        }
    }
    return { log, fires, dispatch };
}

async function newEngine(opt) {
    opt = opt || {};
    clock._exitReplay();
    clock._enterReplay();
    const source = opt.source || makeSource();
    const base = makeEagerCache(source);
    await base.ensure();
    const cache = opt.wrapCache ? opt.wrapCache(base) : base;
    const spy = makeSpy();
    const resets = { n: 0 };
    const eng = createEngine({ source, cache, dispatch: spy.dispatch, clock,
        hooks: { onResetForSeek() { resets.n++; } } });
    return { eng, spy, source, cache, base, resets };
}

/** Play from the current playhead to `end` at `speed` with ~16 ms ticks landing exactly on `end`. */
function playTo(eng, end, speed) {
    eng.setSpeed(speed);
    eng.play();
    for (let guard = 0; guard < 100000; guard++) {
        const p = eng.state().playhead;
        const rem = (end - p) * Math.sign(speed);
        if (rem < 1e-7) break;
        curTick++;
        eng.tick(Math.min(16, (rem / Math.abs(speed)) * 1000));
        if (eng.state().mode !== 'playing') break;
    }
}

function lastPerState(log) {
    const m = {};
    for (const e of log) if (STATE_TOPICS.includes(e.topic)) m[e.topic] = e.t - T0;
    return m;
}
function throttleCount(topic, from, to, lt, g) {
    let n = 0;
    for (const r of RECORDS[topic]) {
        if (r.t <= from || r.t > to) continue;
        if (r.t - lt >= g - 1e-6) { n++; lt = r.t; }
    }
    return n;
}
function dedup(a) { return a.filter((v, i) => i === 0 || v !== a[i - 1]); }

const out = {};

// ---- (1) + (2): rates per speed over 10 s of playhead, merged t order per tick ----
out.rates = {};
out.order_violations = 0;
for (const s of SPEED_LADDER) {
    const { eng, spy } = await newEngine();
    const p0 = T0 + 5;
    await eng.seek(p0);
    const before = {};
    for (const e of spy.log) before[e.topic] = e.t;
    const mark = spy.log.length;
    playTo(eng, p0 + 10, s);
    const win = spy.log.slice(mark);
    const counts = {};
    for (const e of win) counts[e.topic] = (counts[e.topic] || 0) + 1;
    const exact1x = {};
    for (const [topic, g] of [['/robot_state', 0.05], ['/mocap_data', 0.05], ['/hand_telemetry', 0.1]]) {
        exact1x[topic] = throttleCount(topic, p0, p0 + 10, before[topic], g);
    }
    const orchVals = RECORDS['/orchestrator_state'].filter((r) => r.t > p0 && r.t <= p0 + 10).map((r) => r.msg.data);
    const orchTransitions = dedup([orchAt(5 - 1e-9)].concat(orchVals)).length - 1;
    const byTick = {};
    for (const e of win) (byTick[e.tick] = byTick[e.tick] || []).push(e.t);
    for (const k in byTick) for (let i = 1; i < byTick[k].length; i++) if (byTick[k][i] < byTick[k][i - 1]) out.order_violations++;
    out.rates[String(s)] = { counts, exact1x, orchTransitions, muted: win.filter((e) => e.muted).length,
        mode: eng.state().mode, playhead: eng.state().playhead - T0 };
}

// ---- (3): 8x through the whole recording ----
{
    const { eng, spy } = await newEngine();
    await eng.seek(T0);
    playTo(eng, T0 + DUR, 8);
    out.full8 = {
        events: spy.log.filter((e) => e.topic === '/skills/attempt').map((e) => e.t - T0),
        events_recorded: RECORDS['/skills/attempt'].map((r) => r.t - T0),
        orch_dispatched: dedup(spy.log.filter((e) => e.topic === '/orchestrator_state').map((e) => e.data)),
        orch_recorded: dedup(RECORDS['/orchestrator_state'].map((r) => r.msg.data)),
        end_mode: eng.state().mode,
        end_playhead: eng.state().playhead - T0,
    };
}

// ---- (4): seek(p) == play-to-p + pause, for state topics; event list == (p-30, p] ----
out.seek_equiv = [];
for (const px of [27.3, 44.1, 58.0]) {
    const p = T0 + px;
    const A = await newEngine();
    await A.eng.seek(p);
    const seekState = lastPerState(A.spy.log);
    const seekEvents = A.spy.log.filter((e) => e.topic === '/skills/attempt').map((e) => ({ t: e.t - T0, muted: e.muted }));
    const expectEvents = RECORDS['/skills/attempt'].filter((r) => r.t > p - 30 && r.t <= p).map((r) => r.t - T0);
    const earlyUnmuted = A.spy.log.filter((e) => e.t <= p - 30 && !e.muted).length;
    const played = {};
    for (const s of [1, 4]) {
        const B = await newEngine();
        await B.eng.seek(T0);
        playTo(B.eng, p, s);
        await B.eng.pause();
        played[String(s)] = lastPerState(B.spy.log);
    }
    out.seek_equiv.push({ p: px, seekState, played, seekEvents, expectEvents, earlyUnmuted,
        resets: A.resets.n, playhead: A.eng.state().playhead - T0, mode: A.eng.state().mode });
}

// ---- (5): reverse p -> p-20 mutes events/history, pause rebuilds seek state ----
{
    const R = await newEngine();
    await R.eng.seek(T0 + 50);
    const mark = R.spy.log.length;
    playTo(R.eng, T0 + 30, -1);
    const during = R.spy.log.slice(mark);
    const atPause = R.eng.state().playhead;
    const resetsBefore = R.resets.n;
    await R.eng.pause();
    const after = R.spy.log.slice(mark + during.length);
    const F = await newEngine();
    await F.eng.seek(atPause);
    out.reverse = {
        dispatched: during.length,
        event_dispatches: during.filter((e) => TOPIC_POLICY[e.topic].cls === 'event' || TOPIC_POLICY[e.topic].cls === 'history-ring').length,
        unmuted: during.filter((e) => !e.muted).length,
        fault_records_seen: during.filter((e) => e.topic === '/robot_state').length,
        playhead: atPause - T0,
        reseeked: R.resets.n > resetsBefore,
        mode: R.eng.state().mode,
        state_after_pause: lastPerState(R.spy.log),
        state_fresh_seek: lastPerState(F.spy.log),
        events_after_pause: after.filter((e) => e.topic === '/skills/attempt').map((e) => e.t - T0),
        events_fresh_seek: F.spy.log.filter((e) => e.topic === '/skills/attempt').map((e) => e.t - T0),
    };
}

// ---- (6): buffering at a missing chunk and at the frontier ----
{
    const hidden = new Set([3]);
    const G = await newEngine({ wrapCache: (base) => ({
        peek: (i) => (hidden.has(i) ? null : base.peek(i)),
        ensure: (a, b, d) => base.ensure(a, b, d),
    }) });
    const bufEvents = [];
    G.eng.on('buffering', (b) => bufEvents.push(b));
    const chunksEntered = [];
    G.eng.on('chunk', (i) => chunksEntered.push(i));
    await G.eng.seek(T0 + 25);
    playTo(G.eng, T0 + 35, 1);
    const stuck = { mode: G.eng.state().mode, playhead: G.eng.state().playhead - T0,
        max_t: Math.max(...G.spy.log.map((e) => e.t)) - T0 };
    for (let k = 0; k < 5; k++) { curTick++; G.eng.tick(16); }
    const still = { mode: G.eng.state().mode, playhead: G.eng.state().playhead - T0 };
    hidden.delete(3);
    curTick++; G.eng.tick(16);
    const resumed = { mode: G.eng.state().mode, playhead: G.eng.state().playhead - T0 };

    const src = makeSource({ frontier: T0 + 20, state: 'converting' });
    const H = await newEngine({ source: src });
    await H.eng.seek(T0 + 15);
    playTo(H.eng, T0 + 25, 1);
    const atFrontier = { mode: H.eng.state().mode, playhead: H.eng.state().playhead - T0 };
    src._st.frontier = T0 + 40;
    await H.base.ensure();
    for (let k = 0; k < 3; k++) { curTick++; H.eng.tick(16); }
    const extended = { mode: H.eng.state().mode, playhead: H.eng.state().playhead - T0 };
    out.buffering = { stuck, still, resumed, bufEvents, chunksEntered, atFrontier, extended };
}

// ---- (7): virtual watchdog scales with speed and is cleared by seek ----
out.watchdog = {};
for (const s of [1, 4]) {
    const W = await newEngine();
    await W.eng.seek(T0 + 35);
    playTo(W.eng, T0 + 48, s);
    const hands = W.spy.log.filter((e) => e.topic === '/hand_telemetry');
    out.watchdog[String(s)] = { fires: W.spy.fires, last_hand: hands[hands.length - 1].t - T0 };
}
{
    const C = await newEngine();
    await C.eng.seek(T0 + 5);
    let probeFired = false;
    clock.setTimeout(() => { probeFired = true; }, 500);
    const pendingBefore = clock._pending();
    await C.eng.seek(T0 + 10);
    playTo(C.eng, T0 + 12, 1);
    out.watchdog.cleared = { probeFired, pendingBefore, firesAfter: C.spy.fires.length };
}

// ---- scrub + step smoke ----
{
    const S = await newEngine();
    await S.eng.seek(T0 + 10);
    const mark = S.spy.log.length;
    S.eng.scrub(T0 + 30);
    const scrubbed = S.spy.log.slice(mark);
    const p1 = await S.eng.step(1);
    const p2 = await S.eng.step(-1);
    out.scrub = { all_muted: scrubbed.every((e) => e.muted), n: scrubbed.length,
        topics: Array.from(new Set(scrubbed.map((e) => e.topic))).sort(),
        step_fwd: p1 - T0, step_back: p2 - T0, mode: S.eng.state().mode };
}

// ---- hardening: throwing timer / throwing seek / scrub supersedes a pending seek ----
{
    const origErr = console.error; const errs = [];
    console.error = (...a) => errs.push(a.map(String).join(' '));

    // (b1) a throwing virtual-timer callback is logged and the next tick still advances
    const A = await newEngine();
    await A.eng.seek(T0 + 5);
    clock.setTimeout(() => { throw new Error('boom-timer'); }, 5);
    A.eng.setSpeed(1);
    await A.eng.play();
    const a0 = A.eng.state().playhead;
    let aThrew = false;
    try { A.eng.tick(16); A.eng.tick(16); } catch (e) { aThrew = true; }
    out.throw_timer = { threw: aThrew, advanced: A.eng.state().playhead - a0, logged: errs.some((x) => x.includes('boom-timer')) };

    // (b2) a seek whose synchronous section throws leaves seekBusy false: the next tick advances
    let boom = false;
    const B = await newEngine({ wrapCache: (base) => ({
        peek: (i) => { if (boom) throw new Error('boom-peek'); return base.peek(i); },
        ensure: (a, b, d) => base.ensure(a, b, d),
    }) });
    await B.eng.seek(T0 + 5);
    boom = true;
    let bErr = null;
    try { await B.eng.seek(T0 + 20); } catch (e) { bErr = e.message; }
    boom = false;
    B.eng.setSpeed(1);
    await B.eng.play();
    const b0 = B.eng.state().playhead;
    B.eng.tick(16);
    out.throw_seek = { error: bErr, advanced: B.eng.state().playhead - b0 };

    // (f) scrub during a pending seek supersedes it: the seek's synchronous section never runs
    let release = null;
    const gate = new Promise((r) => { release = r; });
    const hide = new Set([2]);
    const C = await newEngine({ wrapCache: (base) => ({
        peek: (i) => (hide.has(i) ? null : base.peek(i)),
        ensure: () => gate,
    }) });
    await C.eng.seek(T0 + 5);
    const resets0 = C.resets.n;
    const pending = C.eng.seek(T0 + 25);          // chunk 2 "missing": parks on the gate
    const pAfterScrub = C.eng.scrub(T0 + 45);
    const resetsAfterScrub = C.resets.n;
    release();
    const pSeek = await pending;
    out.scrub_supersedes = {
        scrub_playhead: pAfterScrub - T0, seek_returned: pSeek - T0, final_playhead: C.eng.state().playhead - T0,
        seek_sync_section_ran: C.resets.n !== resetsAfterScrub, resets_gained_by_scrub: resetsAfterScrub - resets0,
    };
    console.error = origErr;
}

clock._exitReplay();
process.stdout.write(JSON.stringify(out));
