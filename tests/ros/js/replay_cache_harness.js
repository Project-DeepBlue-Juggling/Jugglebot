// replay_cache_harness.js — drives ros_ws/gui/js/replay/cache.js (and, for scenario 7, engine.js)
// against a scripted source and prints one JSON object; tests/ros/test_gui_replay_cache.py asserts on it.
//
// Sandbox layout (built by the pytest fixture, verbatim copies):
//   clock.js, replay/{chunk,policy,engine,cache}.js, this file, package.json {"type":"module"}
import * as clock from './clock.js';
import { makeTopic, flattenMessage } from './replay/chunk.js';
import { createEngine } from './replay/engine.js';
import { createChunkCache } from './replay/cache.js';

const T0 = 1000;
const tick = () => new Promise((r) => setImmediate(r));
async function settle(n) { for (let k = 0; k < (n || 6); k++) await tick(); }

/**
 * Scripted source. mode 'auto': load resolves on a later macrotask; 'manual': load parks until
 * release(i). `hold` is a set of indices that always park (released by release(i)).
 */
function makeSource(o) {
    o = o || {};
    const st = { state: o.state || 'complete', chunksDone: o.chunksDone === undefined ? 100 : o.chunksDone };
    const listeners = new Set();
    const log = { loads: [], inflight: 0, maxInflight: 0 };
    const parked = new Map();
    const hold = new Set(o.hold || []);
    const fail = new Set(o.fail || []);
    const nullIdx = new Set(o.nulls || []);
    const src = {
        kind: o.kind || 'recording',
        _st: st, log, hold, fail,
        range() {
            const end = T0 + st.chunksDone * 10;
            return { t0: T0, t1: end, frontier: end };
        },
        chunkIndex(t) { return Math.floor((t - T0) / 10); },
        bounds(i) { return [T0 + 10 * i, T0 + 10 * (i + 1)]; },
        status() { return { state: st.state, chunksDone: st.chunksDone, chunksTotal: null }; },
        onChange(cb) { listeners.add(cb); return () => listeners.delete(cb); },
        grow(n, state) { st.chunksDone = n; if (state) st.state = state; for (const cb of Array.from(listeners)) cb(src.status()); },
        fire() { for (const cb of Array.from(listeners)) cb(src.status()); },
        load(i) {
            log.loads.push(i);
            log.inflight++;
            log.maxInflight = Math.max(log.maxInflight, log.inflight);
            if (i >= st.chunksDone) { log.inflight--; return Promise.reject(new RangeError('chunk ' + i + ' beyond frontier')); }
            return new Promise((resolve, reject) => {
                const done = () => {
                    log.inflight--;
                    if (fail.has(i)) reject(new Error('boom ' + i));
                    else resolve(nullIdx.has(i) ? null : o.make ? o.make(i) : { i, t0: T0 + 10 * i, t1: T0 + 10 * i + 10, topics: {} });
                };
                if (hold.has(i)) parked.set(i, done);
                else setImmediate(done);
            });
        },
        release(i) { hold.delete(i); const d = parked.get(i); if (d) { parked.delete(i); d(); } },
        parkedList() { return Array.from(parked.keys()); },
    };
    return src;
}

function residentSet(cache, n) {
    const out = [];
    for (let i = 0; i < n; i++) if (cache.peek(i)) out.push(i);
    return out;
}

const out = {};

// ---- (1) ahead bias, both directions ----
{
    const p = T0 + 503;
    const res = {};
    for (const d of [1, -1]) {
        const src = makeSource();
        const cache = createChunkCache({ source: src });
        cache.setPlayhead(p, d, 0);
        await settle(30);
        res[d] = { order: src.log.loads.slice(), resident: residentSet(cache, 100) };
    }
    // wider window to show the asymmetry unambiguously: ahead 40 s, behind 10 s
    const src2 = makeSource();
    const c2 = createChunkCache({ source: src2, aheadSec: 40 });
    c2.setPlayhead(p, 1, 0);
    await settle(30);
    const src3 = makeSource();
    const c3 = createChunkCache({ source: src3, aheadSec: 40 });
    c3.setPlayhead(p, -1, 0);
    await settle(30);
    res.wideFwd = residentSet(c2, 100);
    res.wideRev = residentSet(c3, 100);
    res.cur = 50;
    out.ahead = res;
}

// ---- (2) cap: 30 chunks wanted, only 16 resident, farthest evicted ----
{
    const src = makeSource();
    const cache = createChunkCache({ source: src });
    cache.setPlayhead(T0 + 503, 1, 300); // 150+20 s ahead, 150+10 behind
    await settle(80);
    const first = residentSet(cache, 100);
    cache.setPlayhead(T0 + 703, 1, 300); // jump: old set must go
    await settle(80);
    out.cap = { first, second: residentSet(cache, 100), size: cache.size(), cur1: 50, cur2: 70 };
}

// ---- (3) serial fetch in priority order ----
{
    const src = makeSource({ hold: [50, 51, 52, 49, 48] });
    const cache = createChunkCache({ source: src });
    cache.setPlayhead(T0 + 503, 1, 0);
    await settle(4);
    const steps = [{ parked: src.parkedList(), inflight: src.log.inflight }];
    for (const i of [50, 51, 52, 49, 48]) {
        src.release(i);
        await settle(4);
        steps.push({ parked: src.parkedList(), inflight: src.log.inflight });
    }
    out.serial = { steps, order: src.log.loads.slice(), maxInflight: src.log.maxInflight };
}

// ---- (4) bufferedRanges: resident vs loading, merged, time order ----
{
    const src = makeSource({ hold: [51] });
    const cache = createChunkCache({ source: src });
    cache.setPlayhead(T0 + 503, 1, 0);
    await settle(10);
    out.ranges = { ranges: cache.bufferedRanges(), resident: residentSet(cache, 100) };
}

// ---- (5) frontier ----
{
    const src = makeSource({ state: 'converting', chunksDone: 3 });
    const cache = createChunkCache({ source: src });
    cache.setPlayhead(T0 + 5, 1, 0);
    await settle(20);
    const beforeLoads = src.log.loads.slice();
    let resolved = false;
    const pr = cache.ensure(2, 5, 1).then(() => { resolved = true; });
    await settle(20);
    const stillPending = !resolved;
    const loadsAtFrontier = src.log.loads.slice();
    src.grow(6);
    await settle(30);
    await pr;
    out.frontier = { beforeLoads, loadsAtFrontier, stillPending, resolved, after: src.log.loads.slice(),
        resident: residentSet(cache, 10), frontier: cache.frontier() };
}

// ---- (5b) a FINAL source resolves ensure past its end ----
{
    const src = makeSource({ chunksDone: 4 });
    const cache = createChunkCache({ source: src });
    await cache.ensure(2, 9, 1);
    out.finalEnd = { resident: residentSet(cache, 10), loads: src.log.loads.slice() };
}

// ---- (6) failure -> {error}; source failed -> {error}; gap -> empty chunk ----
{
    const src = makeSource({ fail: [1], nulls: [2] });
    const cache = createChunkCache({ source: src, retryMs: 60000 });
    const events = [];
    cache.onChange((e) => events.push({ reason: e.reason, msg: e.error ? e.error.message : null }));
    await cache.ensure(0, 2, 1);
    out.failure = { events: events.filter((e) => e.error !== null || e.reason === 'error'),
        peek1: cache.peek(1) === null, gapEmpty: !!cache.peek(2) && Object.keys(cache.peek(2).topics).length === 0,
        peek0: !!cache.peek(0) };
    const src2 = makeSource({ state: 'converting', chunksDone: 2 });
    const c2 = createChunkCache({ source: src2 });
    const ev2 = [];
    c2.onChange((e) => ev2.push(e.reason + ':' + (e.error ? e.error.message : '')));
    src2.grow(2, 'failed');
    out.sourceFailed = ev2.filter((s) => s.startsWith('error'));
}

// ---- (7) through the engine: gap -> buffering, chunk arrives -> playing ----
{
    function make(i) {
        const c0 = T0 + 10 * i;
        const recs = [];
        for (let k = 0; k < 200; k++) recs.push({ t: c0 + k * 0.05, msg: { seq: i * 200 + k, has_fatal_odrive_error: false } });
        const cols = {};
        recs.forEach((r, k) => {
            const flat = flattenMessage(r.msg);
            for (const c in flat) (cols[c] = cols[c] || new Array(recs.length))[k] = flat[c];
        });
        return { i, t0: c0, t1: c0 + 10, topics: { '/robot_state': makeTopic('t/rs', Float64Array.from(recs.map((r) => r.t)), cols) } };
    }
    clock._exitReplay();
    clock._enterReplay();
    const src = makeSource({ chunksDone: 6, make, hold: [1] });
    src.latestBefore = async () => null;
    const cache = createChunkCache({ source: src });
    const seen = [];
    const eng = createEngine({ source: src, cache, dispatch: (t, m, ts) => seen.push(ts), clock, hooks: {} });
    const bufEvents = [];
    eng.on('buffering', (b) => bufEvents.push(b));
    const sp = eng.seek(T0 + 4);
    await settle(10);
    await sp;
    eng.setSpeed(1);
    eng.play();
    let guard = 0;
    while (eng.state().mode === 'playing' && guard++ < 2000) eng.tick(16);
    const stuck = { mode: eng.state().mode, playhead: eng.state().playhead, parked: src.parkedList() };
    for (let k = 0; k < 10; k++) eng.tick(16);
    const still = eng.state().playhead;
    src.release(1);
    await settle(10);
    for (let k = 0; k < 20; k++) eng.tick(16);
    out.engine = { stuck, still, resumed: { mode: eng.state().mode, playhead: eng.state().playhead },
        bufEvents, resident0: !!cache.peek(0), ranges: cache.bufferedRanges().length };
    eng.dispose();
}

console.log(JSON.stringify(out));
process.exit(0);
