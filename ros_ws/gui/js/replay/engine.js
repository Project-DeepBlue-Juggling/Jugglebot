/**
 * engine.js — the replay engine: playhead, per-topic dispatch at live rates,
 * seek with baseline + pre-roll, reverse, buffering (replay design §§ 3, 4, 6, 7).
 *
 *   const eng = createEngine({source, cache, dispatch, clock, hooks, preRollSec});
 *
 * Inputs
 *   source    a sources.js Source (range, chunkIndex, bounds, status, optional latestBefore)
 *   cache     {peek(i) -> Chunk|null (sync), ensure(iFrom, iTo, direction) -> Promise}
 *             iFrom <= iTo always; direction (+1 | -1) is the priority hint.
 *             Default (none injected): createChunkCache({source}) from cache.js, which also
 *             receives setPlayhead(p, dir, span) on every chunk/direction change.
 *             makeEagerCache(source) below loads every chunk (tests, dev page).
 *   dispatch  (topic '/x', msg, tSec, {muted}) => void. Production (U6):
 *             setEventsMuted(muted); ros.dispatchLocal(topic, msg); setEventsMuted(false).
 *   clock     the clock.js module (default: imported). The engine calls
 *             _setNow(t*1000) before each record's dispatch and _travel(|dp|*1000, speed)
 *             as the playhead moves (interleaved with records, so a watchdog
 *             fires at the right playhead), _clearTimers() on seek / scrub.
 *   hooks     optional {onPlayhead(p), onState(s), onChunkEntered(i), onResetForSeek(), onSettle()}
 *   preRollSec optional () => seconds (visible chart span); W = min(W_PREROLL_SEC, it).
 *
 * Public API
 *   play()             intent playing (a scrubbed / reversed state is first rebuilt by seek)
 *   pause()            -> Promise; forward: settle(); after reverse or scrub: seek(p)
 *   seek(t)            -> Promise<playhead>; full § 4 order. Synchronous when every
 *                      needed chunk is resident and no latestBefore fallback is needed.
 *   scrub(t)           sync, LIGHT: playhead (-> charts re-centre, cache follows) only; no reset,
 *                      no dispatch. Marks state dirty: always follow a drag with seek(t) on release
 *   setSpeed(x)        snaps |x| to SPEED_LADDER, sign kept (negative = reverse) -> speed
 *   step(n)            -> Promise<playhead>; pauses, moves +-|n| records of the fastest state topic
 *   tick(dtMs)         the rAF body; dp = clamp(dt, 0, 100) * speed; returns the playhead
 *   state()            {mode: 'paused'|'playing'|'buffering', playhead, speed, frontier, range}
 *   on(evt, cb)        evt in playhead|state|chunk|buffering; returns an unsubscribe fn
 *   settle()           § 4 pause rule: latest-before(p) for every state topic not yet at it
 *   bufferedRanges()   pass-through to cache.bufferedRanges() when the cache has one, else []
 *   dispose()          stops everything; pending seeks are dropped
 *
 * Dispatch rules (policy.js holds the per-topic table):
 *   forward  records in (p, p'] per topic through the gate
 *            (t - t_last >= gate * max(1,|speed|), on-change exceptions, events
 *            never gated), merged across topics in t order, unmuted.
 *   reverse  per state topic the latest record <= p' when it differs from the
 *            last dispatched one and passes the gate on |dt|, MUTED; history and
 *            event topics are not dispatched. Pause or a flip to forward runs seek(p).
 *   seek(p)  clear timers; ensure [p-W, p]; reset (hooks.onResetForSeek);
 *            baseline latest-before(p-W) of every state topic, muted; pre-roll
 *            (p-W, p] for edge/history/event topics at the 1x gates, events on,
 *            interleaved with each render-only topic's latest record in (p-W, p]
 *            (muted); settle at p; playhead = p. Virtual time travels at 1x
 *            through baseline and pre-roll, so watchdogs end in the state a
 *            1x play-through would leave them.
 */
import * as defaultClock from '../clock.js';
import { indexLatestBefore } from './chunk.js';
import { createChunkCache } from './cache.js';
import {
    TOPIC_POLICY, CLASSES, W_PREROLL_SEC, gateFor, isStateClass, snapSpeed,
} from './policy.js';

/** Time comparison slack (s). Epoch-second float64 differences carry ~5e-7 s of error. */
const EPS = 1e-6;
const MAX_DT_MS = 100;
const AHEAD_CHUNKS = 2;

const TOPICS = Object.keys(TOPIC_POLICY).filter((t) => TOPIC_POLICY[t].cls !== CLASSES.COLUMNS);
const ORDER = new Map(TOPICS.map((t, i) => [t, i]));
const STATE_TOPICS = TOPICS.filter((t) => isStateClass(TOPIC_POLICY[t].cls));
const RENDER_TOPICS = TOPICS.filter((t) => TOPIC_POLICY[t].cls === CLASSES.STATE_RENDER);
const PREROLL_TOPICS = TOPICS.filter((t) => TOPIC_POLICY[t].cls !== CLASSES.STATE_RENDER);

function byTime(a, b) { return (a.t - b.t) || (a.ord - b.ord) || ((a.k || 0) - (b.k || 0)); }

/**
 * A cache that loads every available chunk on the first ensure() (and any
 * newly converted ones on later calls). A load that resolves null (a session
 * buffer index with no data) is stored as an empty chunk, so a gap is "no
 * data", not "loading".
 * @param {object} source
 */
export function makeEagerCache(source) {
    const chunks = new Map();
    const inflight = new Map();
    function indices() {
        const r = source.range();
        if (r.t0 === 0 && r.frontier === 0) return [];
        const lo = source.chunkIndex(r.t0);
        const hi = source.chunkIndex(r.frontier);
        const out = [];
        for (let i = lo; i <= hi; i++) out.push(i);
        return out;
    }
    function loadOne(i) {
        if (chunks.has(i)) return Promise.resolve(chunks.get(i));
        let pr = inflight.get(i);
        if (pr) return pr;
        pr = Promise.resolve()
            .then(() => source.load(i))
            .then((ch) => {
                const b = source.bounds(i);
                const v = ch || { i, t0: b[0], t1: b[1], topics: {} };
                chunks.set(i, v);
                return v;
            }, () => null)
            .finally(() => inflight.delete(i));
        inflight.set(i, pr);
        return pr;
    }
    return {
        peek(i) { return chunks.get(i) || null; },
        ensure() {
            // Second pass: an in-flight load joined here may have been issued
            // against an older frontier and failed; retry what is still absent.
            return Promise.all(indices().map(loadOne)).then(() => {
                const miss = indices().filter((i) => !chunks.has(i));
                return miss.length ? Promise.all(miss.map(loadOne)) : [];
            });
        },
        size() { return chunks.size; },
    };
}

/**
 * @param {{source:object, cache:object, dispatch:function, clock?:object,
 *          hooks?:object, preRollSec?:function}} opts
 */
export function createEngine(opts) {
    const source = opts.source;
    const cache = opts.cache || createChunkCache({ source });
    const dispatch = opts.dispatch;
    const clk = opts.clock || defaultClock;
    const hooks = opts.hooks || {};
    const preRollFn = opts.preRollSec || null;

    let p = source.range().t0;
    let speed = 1;
    let intent = 'paused';
    let stalled = false;       // forward/reverse tick could not move (missing chunk / frontier)
    let seekWaiting = false;   // a seek is awaiting chunks / fallbacks
    let seekToken = 0;
    let seekBusy = false;
    let dirty = false;         // state built by reverse or scrub: rebuild with seek before forward
    let disposed = false;
    let curChunk = null;
    let travelPos = p;
    let lastEnsureKey = '';
    let lastMode = 'paused';
    let lastSpeedEmitted = speed;
    const lastT = new Map();
    const lastVal = new Map();
    const listeners = { playhead: new Set(), state: new Set(), chunk: new Set(), buffering: new Set() };

    function emit(evt, v) {
        for (const cb of Array.from(listeners[evt])) {
            try { cb(v); } catch (e) { console.error('replay engine listener', evt, e); }
        }
    }
    function callHook(name, v) {
        const fn = hooks[name];
        if (typeof fn === 'function') {
            try { fn(v); } catch (e) { console.error('replay engine hook', name, e); }
        }
    }

    function mode() {
        if (seekWaiting) return 'buffering';
        if (intent === 'paused') return 'paused';
        return stalled ? 'buffering' : 'playing';
    }
    function state() {
        const r = source.range();
        return { mode: mode(), playhead: p, speed, frontier: r.frontier, range: r };
    }
    function syncState() {
        const m = mode();
        if (m === lastMode && speed === lastSpeedEmitted) return;
        const wasBuf = lastMode === 'buffering';
        lastMode = m;
        lastSpeedEmitted = speed;
        const s = state();
        callHook('onState', s);
        emit('state', s);
        if (wasBuf !== (m === 'buffering')) emit('buffering', m === 'buffering');
    }

    function requestEnsure(force) {
        if (curChunk === null) return;
        const dir = speed < 0 ? -1 : 1;
        const lo = dir > 0 ? curChunk : curChunk - AHEAD_CHUNKS;
        const hi = dir > 0 ? curChunk + AHEAD_CHUNKS : curChunk;
        const key = lo + ':' + hi + ':' + dir;
        if (!force && key === lastEnsureKey) return;
        lastEnsureKey = key;
        if (typeof cache.setPlayhead === 'function') {
            cache.setPlayhead(p, dir, preRollFn ? +preRollFn() || undefined : undefined);
        }
        const pr = cache.ensure(lo, hi, dir);
        if (pr && typeof pr.catch === 'function') pr.catch(() => {});
    }

    function setPlayhead(x) {
        p = x;
        const ci = source.chunkIndex(p);
        if (ci !== curChunk) {
            curChunk = ci;
            callHook('onChunkEntered', ci);
            emit('chunk', ci);
            requestEnsure(false);
        }
        callHook('onPlayhead', p);
        emit('playhead', p);
    }

    // ---- record access ----

    function rec(topic, ch, k) {
        return { topic, t: ch.topics[topic].t[k], ch, k, ord: ORDER.get(topic), msg: null };
    }
    function msgOf(r) {
        if (r.msg === null) r.msg = r.ch.topics[r.topic].hydrate(r.k);
        return r.msg;
    }
    function dataOf(r) {
        if (r.msg !== null) return r.msg ? r.msg.data : undefined;
        const col = r.ch.topics[r.topic].cols.data;
        return col ? col[r.k] : undefined;
    }

    /**
     * Latest resident record of `topic` with t <= x, scanning back to the
     * range start. Returns null when the topic has none, {missing: true} when
     * the scan hit a non-resident chunk first.
     */
    function latestResident(topic, x) {
        const iMin = source.chunkIndex(source.range().t0);
        for (let i = source.chunkIndex(x); i >= iMin; i--) {
            const ch = cache.peek(i);
            if (!ch) return { missing: true };
            const tp = ch.topics[topic];
            if (!tp || !(tp.n > 0)) continue;
            const k = indexLatestBefore(tp.t, x, tp.n);
            if (k >= 0) return rec(topic, ch, k);
        }
        return null;
    }

    /** Latest record of `topic` in (a, b] from resident chunks, or null. */
    function latestIn(topic, a, b) {
        for (let i = source.chunkIndex(b); i >= source.chunkIndex(a); i--) {
            const ch = cache.peek(i);
            if (!ch) continue;
            const tp = ch.topics[topic];
            if (!tp || !(tp.n > 0)) continue;
            const k = indexLatestBefore(tp.t, b, tp.n);
            if (k >= 0) return tp.t[k] > a ? rec(topic, ch, k) : null;
        }
        return null;
    }

    /**
     * Gated records with t in (a, b] for `topics`, merged in t order. Gate
     * state starts from lastT/lastVal and is advanced locally (every
     * collected record is then dispatched synchronously).
     */
    function collectForward(a, b, spd, topics) {
        const out = [];
        const iA = source.chunkIndex(a);
        const iB = source.chunkIndex(b);
        for (const topic of topics) {
            const pol = TOPIC_POLICY[topic];
            let lt = lastT.has(topic) ? lastT.get(topic) : -Infinity;
            let lv = lastVal.get(topic);
            for (let i = iA; i <= iB; i++) {
                const ch = cache.peek(i);
                if (!ch) continue;
                const tp = ch.topics[topic];
                if (!tp || !(tp.n > 0)) continue;
                const g = gateFor(topic, spd, ch);
                const dcol = pol.onChange ? tp.cols.data : null;
                for (let k = indexLatestBefore(tp.t, a, tp.n) + 1; k < tp.n; k++) {
                    const t = tp.t[k];
                    if (t > b) break;
                    let pass = g === 0 || (t - lt >= g - EPS);
                    const v = dcol ? dcol[k] : undefined;
                    if (!pass && dcol) pass = v !== lv;
                    if (!pass) continue;
                    out.push(rec(topic, ch, k));
                    lt = t;
                    if (dcol) lv = v;
                }
            }
        }
        out.sort(byTime);
        return out;
    }

    function doDispatch(r, muted) {
        const msg = msgOf(r);
        clk._setNow(r.t * 1000);
        try {
            dispatch(r.topic, msg, r.t, { muted: !!muted });
        } catch (e) {
            console.error('replay dispatch', r.topic, e);
        }
        lastT.set(r.topic, r.t);
        if (TOPIC_POLICY[r.topic].onChange) lastVal.set(r.topic, dataOf(r));
    }

    /** Advance virtual time to playhead x (|dx| of travel at spd). */
    function travelTo(x, spd) {
        clk._setNow(x * 1000);
        if (x !== travelPos) clk._travel(Math.abs(x - travelPos) * 1000, spd);
        travelPos = x;
    }

    function settleSync(muted) {
        const list = [];
        for (const topic of STATE_TOPICS) {
            const L = latestResident(topic, p);
            if (!L || L.missing) continue;
            if (lastT.get(topic) === L.t) continue;
            list.push(L);
        }
        list.sort(byTime);
        for (const r of list) doDispatch(r, muted);
        clk._setNow(p * 1000);
        return list.length;
    }

    function sourceDone() {
        const st = source.status ? source.status() : null;
        return !!st && (st.state === 'complete' || st.state === 'failed');
    }

    // ---- ticks ----

    function forwardTick(dt) {
        const r = source.range();
        let target = p + (dt / 1000) * speed;
        if (target > r.frontier) target = r.frontier;
        const iA = source.chunkIndex(p);
        const iB = source.chunkIndex(target);
        let missing = false;
        for (let i = iA; i <= iB; i++) {
            if (!cache.peek(i)) {
                missing = true;
                target = Math.min(target, Math.max(p, source.bounds(i)[0]));
                break;
            }
        }
        if (target > p) {
            const list = collectForward(p, target, speed, TOPICS);
            for (const rr of list) {
                travelTo(rr.t, speed);
                doDispatch(rr, false);
            }
            travelTo(target, speed);
            setPlayhead(target);
        }
        if (p >= r.frontier - EPS && sourceDone()) {
            stalled = false;
            pauseInternal();
            return;
        }
        stalled = missing || p >= r.frontier - EPS;
        if (stalled) requestEnsure(true);
    }

    function reverseTick(dt) {
        const r = source.range();
        let target = p + (dt / 1000) * speed;
        if (target < r.t0) target = r.t0;
        if (!cache.peek(source.chunkIndex(target))) {
            stalled = true;
            requestEnsure(true);
            return;
        }
        stalled = false;
        const list = [];
        for (const topic of STATE_TOPICS) {
            const L = latestResident(topic, target);
            if (!L || L.missing) continue;
            const lt = lastT.get(topic);
            if (lt === L.t) continue;
            const g = gateFor(topic, speed, L.ch);
            let pass = lt === undefined || g === 0 || Math.abs(L.t - lt) >= g - EPS;
            if (!pass && TOPIC_POLICY[topic].onChange) pass = dataOf(L) !== lastVal.get(topic);
            if (pass) list.push(L);
        }
        list.sort(byTime);
        for (const rr of list) doDispatch(rr, true);
        clk._setNow(target * 1000);
        if (target !== p) clk._travel(Math.abs(target - p) * 1000, speed);
        travelPos = target;
        dirty = true;
        setPlayhead(target);
        if (p <= r.t0 + EPS) pauseInternal();
    }

    function tick(dtMs) {
        if (disposed || seekBusy || intent !== 'playing') return p;
        const dt = Math.min(Math.max(+dtMs || 0, 0), MAX_DT_MS);
        if (speed > 0) forwardTick(dt); else reverseTick(dt);
        syncState();
        return p;
    }

    // ---- seek ----

    async function seek(tSec) {
        if (disposed) return p;
        const token = ++seekToken;
        seekBusy = true;
        try {
            clk._clearTimers();
            const r = source.range();
            let t = +tSec;
            if (!Number.isFinite(t)) t = p;
            t = Math.min(Math.max(t, r.t0), Math.max(r.t0, r.frontier));
            const W = Math.max(0, Math.min(W_PREROLL_SEC, preRollFn ? +preRollFn() || W_PREROLL_SEC : W_PREROLL_SEC));
            const a = Math.max(r.t0, t - W);
            const iA = source.chunkIndex(a);
            const iB = source.chunkIndex(t);
            const dir = speed < 0 ? -1 : 1;

            let need = false;
            for (let i = iA; i <= iB; i++) if (!cache.peek(i)) { need = true; break; }
            if (need) {
                seekWaiting = true;
                syncState();
                try { await cache.ensure(iA, iB, dir); } catch (e) { /* missing chunks are skipped */ }
                if (token !== seekToken || disposed) return p;
            }

            // Render-only topics: their latest record in (a, t] (one record each).
            // When there is one, it supersedes the baseline record, so the baseline
            // skips the topic (a baseline-armed watchdog would otherwise fire as a
            // transient while virtual time travels through the window).
            const renderLatest = [];
            const renderHas = new Set();
            for (const topic of RENDER_TOPICS) {
                const L = latestIn(topic, a, t);
                if (L) { L.render = true; renderLatest.push(L); renderHas.add(topic); }
            }

            // Baseline: latest-before(a) of every state topic (resident, else the source's fallback).
            const baseline = [];
            for (const topic of STATE_TOPICS) {
                if (renderHas.has(topic)) continue;
                const L = latestResident(topic, a);
                if (L && L.missing) {
                    if (typeof source.latestBefore !== 'function') continue;
                    if (!seekWaiting) { seekWaiting = true; syncState(); }
                    let fb = null;
                    try { fb = await source.latestBefore(a, topic); } catch (e) { fb = null; }
                    if (token !== seekToken || disposed) return p;
                    if (fb) baseline.push({ topic, t: fb.t, msg: fb.msg, ch: null, k: 0, ord: ORDER.get(topic) });
                } else if (L) {
                    baseline.push(L);
                }
            }

            // ---- synchronous from here: nothing can interleave ----
            seekWaiting = false;
            stalled = false;
            lastT.clear();
            lastVal.clear();
            dirty = false;
            clk._clearTimers();
            callHook('onResetForSeek');

            baseline.sort(byTime);
            travelPos = baseline.length ? Math.min(baseline[0].t, a) : a;
            for (const b of baseline) {
                travelTo(b.t, 1);
                doDispatch(b, true);
            }
            travelTo(a, 1);

            const pre = collectForward(a, t, 1, PREROLL_TOPICS).concat(renderLatest);
            pre.sort(byTime);
            for (const rr of pre) {
                travelTo(rr.t, 1);
                doDispatch(rr, !!rr.render);
            }
            travelTo(t, 1);
            p = t;
            settleSync(false);
            seekBusy = false;
            setPlayhead(t);
            syncState();
            return p;
        } finally {
            // a throwing handler/hook must not wedge every later tick; a superseding seek/scrub owns the flag
            if (token === seekToken) seekBusy = false;
        }
    }

    // ---- transport ----

    function pauseInternal() {
        intent = 'paused';
        stalled = false;
        if (dirty) {
            seek(p);
        } else {
            settleSync(false);
            callHook('onSettle');
        }
    }

    function pause() {
        if (disposed) return Promise.resolve(p);
        const wasDirty = dirty;
        intent = 'paused';
        stalled = false;
        let pr;
        if (wasDirty) {
            pr = seek(p);
        } else {
            settleSync(false);
            callHook('onSettle');
            pr = Promise.resolve(p);
        }
        syncState();
        return pr;
    }

    function play() {
        if (disposed) return Promise.resolve(p);
        let pr = Promise.resolve(p);
        if (dirty && speed > 0) pr = seek(p);
        intent = 'playing';
        stalled = false;
        requestEnsure(true);
        syncState();
        return pr;
    }

    function setSpeed(x) {
        const s = snapSpeed(x);
        const old = speed;
        speed = s;
        if (old < 0 && s > 0 && dirty && intent === 'playing') seek(p);
        if ((old < 0) !== (s < 0)) requestEnsure(true);
        syncState();
        return speed;
    }

    function scrub(tSec) {
        // LIGHT PATH (drag preview): move the playhead only. No reset, no dispatch, no panel/3D work -
        // the hooks' onPlayhead re-centres the charts (one x-scale move) and the chunk cache follows
        // the playhead. State is marked dirty, so the caller MUST follow the drag with seek(t)
        // (release / rest), which runs the full baseline + pre-roll pipeline once.
        if (disposed) return p;
        ++seekToken;           // a pending seek is superseded
        seekBusy = false;
        seekWaiting = false;
        const r = source.range();
        let t = +tSec;
        if (!Number.isFinite(t)) t = p;
        t = Math.min(Math.max(t, r.t0), Math.max(r.t0, r.frontier));
        travelPos = t;
        dirty = true;
        setPlayhead(t);
        syncState();
        return p;
    }

    /** The state topic with the most records in the chunk under the playhead. */
    function fastestStateTopic() {
        const ch = cache.peek(source.chunkIndex(p));
        let best = null;
        let bestN = 0;
        if (ch) {
            for (const topic of STATE_TOPICS) {
                const tp = ch.topics[topic];
                if (tp && tp.n > bestN) { best = topic; bestN = tp.n; }
            }
        }
        return best;
    }

    async function step(n) {
        if (disposed) return p;
        n = Math.trunc(+n || 0);
        if (intent !== 'paused' || dirty) await pause();
        if (n === 0) return p;
        const topic = fastestStateTopic();
        if (!topic) return p;
        const r = source.range();
        if (n > 0) {
            let x = p;
            let found = null;
            for (let left = n; left > 0; left--) {
                found = null;
                for (let i = source.chunkIndex(x); i <= source.chunkIndex(r.frontier); i++) {
                    const ch = cache.peek(i);
                    if (!ch) break;
                    const tp = ch.topics[topic];
                    if (!tp || !(tp.n > 0)) continue;
                    const k = indexLatestBefore(tp.t, x, tp.n) + 1;
                    if (k < tp.n) { found = tp.t[k]; break; }
                }
                if (found === null) break;
                x = found;
            }
            if (!(x > p)) return p;
            const list = collectForward(p, x, 1, TOPICS);
            for (const rr of list) { travelTo(rr.t, 1); doDispatch(rr, false); }
            travelTo(x, 1);
            p = x;
            settleSync(false);
            setPlayhead(x);
            syncState();
            return p;
        }
        let x = p;
        for (let left = -n; left > 0; left--) {
            const L = latestResident(topic, x - EPS);
            if (!L || L.missing) break;
            x = L.t;
        }
        if (!(x < p)) return p;
        return seek(x);
    }

    function on(evt, cb) {
        const set = listeners[evt];
        if (!set) throw new Error('unknown replay engine event ' + evt);
        set.add(cb);
        return () => set.delete(cb);
    }

    function settle() {
        const n = settleSync(false);
        callHook('onSettle');
        return n;
    }

    function dispose() {
        disposed = true;
        ++seekToken;
        intent = 'paused';
        for (const k in listeners) listeners[k].clear();
    }

    return {
        play, pause, seek, scrub, setSpeed, step, tick, state, on, settle, dispose,
        bufferedRanges() { return typeof cache.bufferedRanges === 'function' ? cache.bufferedRanges() : []; },
    };
}
