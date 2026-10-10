/**
 * Chunk residency cache for the replay engine (design § 6).
 *
 *   createChunkCache({source, spanSec, maxResident = 16, aheadSec = 20, behindSec = 10, retryMs = 2000, now})
 *     peek(i)                          sync  resident chunk or null (a session gap is an EMPTY chunk, never null)
 *     ensure(iFrom, iTo, direction)    async resolves when every covering chunk is resident
 *                                      (chunks past a FINAL source's end count as satisfied)
 *     setPlayhead(pSec, dir, spanSec)  recompute the wanted set, evict, (re)start the serial fetcher
 *     bufferedRanges()                 [{t0, t1, state: 'resident'|'loading'}] time order, merged runs
 *     onChange(cb)                     cb({reason: 'residency'|'frontier'|'error', error: null|Error}) -> unsubscribe
 *     frontier()                       source.range().frontier
 *     size()                           resident chunk count
 *     dispose()                        cancel pending work (waiters resolve)
 *
 * Wanted set (forward): chunks covering [p - span/2 - behind, p + span/2 + ahead] plus one extra chunk
 * behind; reverse mirrors it. Priority order: current, ahead (nearest first), behind (nearest first);
 * chunks a pending ensure() asked for come before all of it. Fetch order is that priority
 * order. When the set exceeds maxResident the chunks NEAREST the playhead are kept (ahead wins ties),
 * so eviction always removes the farthest.
 * Fetches are issued ONE AT A TIME through source.load. Nothing at or past the source's frontier
 * chunk is requested (bounds(i)[0] >= frontier); source.onChange with a larger frontier re-plans.
 * A load that resolves null (a session-buffer gap) is stored as an empty chunk.
 */

const DEFAULTS = { spanSec: 30, maxResident: 16, aheadSec: 20, behindSec: 10, retryMs: 2000 };

export function createChunkCache(opts) {
    const source = opts.source;
    const maxResident = opts.maxResident || DEFAULTS.maxResident;
    const aheadSec = opts.aheadSec === undefined ? DEFAULTS.aheadSec : opts.aheadSec;
    const behindSec = opts.behindSec === undefined ? DEFAULTS.behindSec : opts.behindSec;
    const retryMs = opts.retryMs === undefined ? DEFAULTS.retryMs : opts.retryMs;
    const nowFn = opts.now || (() => Date.now());   // wall-clock: chunk-fetch retry backoff paces real HTTP failures, not recorded time
    let spanSec = opts.spanSec === undefined ? DEFAULTS.spanSec : opts.spanSec;

    const resident = new Map();   // i -> Chunk
    const failedAt = new Map();   // i -> ms of last failed load
    const waiters = new Map();    // 'lo:hi' -> {lo, hi, promise, resolve}
    const listeners = new Set();
    let playT = null;             // null until setPlayhead
    let dir = 1;
    let loadingIdx = null;
    let loadToken = 0;
    let disposed = false;
    let lastLimit = null;
    let lastFailedReported = false;
    let plan = [];                // kept indices in priority order
    let retryTimer = null;

    function emit(reason, error) {
        const ev = { reason, error: error || null };
        for (const cb of Array.from(listeners)) {
            try { cb(ev); } catch (e) { console.error('replay cache listener', e); }
        }
    }

    /** [lo, limit] loadable index range, or null when the source is empty. */
    function loadable() {
        const r = source.range();
        if (r.t0 === 0 && r.frontier === 0) return null;
        const lo = source.chunkIndex(r.t0);
        let hi = source.chunkIndex(r.frontier);
        if (source.bounds(hi)[0] >= r.frontier) hi -= 1;
        return hi >= lo ? { lo, hi } : null;
    }

    function isFinal() {
        const st = source.status();
        return !!st && (st.state === 'complete' || st.state === 'failed');
    }

    function emptyChunk(i) {
        const b = source.bounds(i);
        return { i, t0: b[0], t1: b[1], topics: {} };
    }

    function requiredIndices(rng) {
        const out = [];
        const seen = new Set();
        for (const w of waiters.values()) {
            const lo = Math.max(w.lo, rng.lo);
            const hi = Math.min(w.hi, rng.hi);
            const idx = [];
            for (let i = lo; i <= hi; i++) idx.push(i);
            if (dir < 0) idx.reverse();
            for (const i of idx) if (!seen.has(i)) { seen.add(i); out.push(i); }
        }
        return out;
    }

    function wantedIndices(rng) {
        if (playT === null) return [];
        const half = spanSec / 2;
        const rev = dir < 0;
        const tA = playT - half - (rev ? aheadSec : behindSec);
        const tB = playT + half + (rev ? behindSec : aheadSec);
        let iA = source.chunkIndex(tA);
        let iB = source.chunkIndex(tB);
        if (rev) iB += 1; else iA -= 1;
        const cur = Math.min(Math.max(source.chunkIndex(playT), rng.lo), rng.hi);
        iA = Math.max(iA, rng.lo);
        iB = Math.min(iB, rng.hi);
        const out = [cur];
        if (rev) {
            for (let i = cur - 1; i >= iA; i--) out.push(i);
            for (let i = cur + 1; i <= iB; i++) out.push(i);
        } else {
            for (let i = cur + 1; i <= iB; i++) out.push(i);
            for (let i = cur - 1; i >= iA; i--) out.push(i);
        }
        return out;
    }

    function recompute() {
        const rng = loadable();
        if (!rng) { plan = []; return; }
        const req = requiredIndices(rng);
        const seen = new Set(req);
        const list = req.slice();
        for (const i of wantedIndices(rng)) if (!seen.has(i)) { seen.add(i); list.push(i); }
        const cap = Math.max(maxResident, req.length);
        if (list.length > cap) {
            // Over the cap: keep the required chunks, then the wanted ones NEAREST the playhead
            // (ahead side wins a tie); fetch order stays priority order. Farthest are never kept.
            const cur = Math.min(Math.max(source.chunkIndex(playT), rng.lo), rng.hi);
            const rest = list.slice(req.length);
            const rank = rest.slice().sort((a, b) => {
                const da = Math.abs(a - cur), db = Math.abs(b - cur);
                if (da !== db) return da - db;
                return (dir > 0 ? b - a : a - b);
            });
            const keep = new Set(req.concat(rank.slice(0, cap - req.length)));
            plan = list.filter((i) => keep.has(i));
        } else {
            plan = list;
        }
    }

    function evict() {
        const keep = new Set(plan);
        let changed = false;
        for (const i of Array.from(resident.keys())) {
            if (!keep.has(i)) { resident.delete(i); changed = true; }
        }
        return changed;
    }

    function settleWaiters() {
        const rng = loadable();
        const fin = isFinal();
        for (const [key, w] of Array.from(waiters)) {
            let ok = true;
            for (let i = w.lo; i <= w.hi; i++) {
                if (resident.has(i)) continue;
                const beyond = !rng || i > rng.hi || i < rng.lo;
                if (beyond && fin) continue;                  // final source: that chunk will never exist
                if (failedAt.has(i) && !beyond) continue;     // load failed: skipped, like the eager cache
                ok = false;
                break;
            }
            if (ok) { waiters.delete(key); w.resolve(); }
        }
    }

    function pump() {
        if (disposed || loadingIdx !== null) return;
        const now = nowFn();
        let next = null;
        for (const i of plan) {
            if (resident.has(i)) continue;
            const f = failedAt.get(i);
            if (f !== undefined && now - f < retryMs) continue;
            next = i;
            break;
        }
        if (next === null) return;
        const idx = next;
        const token = ++loadToken;
        loadingIdx = idx;
        Promise.resolve().then(() => source.load(idx)).then((ch) => {
            if (disposed || token !== loadToken) return;
            loadingIdx = null;
            failedAt.delete(idx);
            // A chunk dropped from the plan while in flight is discarded, not stored.
            if (plan.indexOf(idx) >= 0) resident.set(idx, ch || emptyChunk(idx));
            evict();
            emit('residency');
            settleWaiters();
            pump();
        }, (err) => {
            if (disposed || token !== loadToken) return;
            loadingIdx = null;
            failedAt.set(idx, nowFn());
            if (retryTimer === null && retryMs > 0) {
                // The engine only re-ensures on a chunk/direction change, so a failed chunk needs its own retry tick.
                retryTimer = setTimeout(() => { retryTimer = null; if (!disposed) replan(); }, retryMs);
            }
            emit('error', err instanceof Error ? err : new Error(String(err)));
            settleWaiters();
            pump();
        });
        emit('residency'); // the loading hatch changed
    }

    function replan() {
        recompute();
        const ev = evict();
        if (ev) emit('residency');
        settleWaiters();
        pump();
    }

    const unsubSource = typeof source.onChange === 'function' ? source.onChange(() => {
        if (disposed) return;
        const st = source.status();
        if (st && st.state === 'failed' && !lastFailedReported) {
            lastFailedReported = true;
            emit('error', new Error('replay source failed'));
        }
        const rng = loadable();
        const key = rng ? rng.lo + ':' + rng.hi : 'none';
        if (key === lastLimit) { settleWaiters(); return; } // cheap path: session sources fire per record
        lastLimit = key;
        for (const i of Array.from(failedAt.keys())) failedAt.delete(i);
        replan();
        emit('frontier');
    }) : null;

    const self = {
        peek(i) { return resident.get(i) || null; },

        ensure(iFrom, iTo, direction) {
            if (disposed) return Promise.resolve();
            if (direction) dir = direction < 0 ? -1 : 1;
            const lo = Math.min(iFrom, iTo);
            const hi = Math.max(iFrom, iTo);
            const key = lo + ':' + hi;
            let w = waiters.get(key);
            if (!w) {
                w = { lo, hi };
                w.promise = new Promise((res) => { w.resolve = res; });
                waiters.set(key, w);
            }
            replan();
            return w.promise;
        },

        setPlayhead(pSec, direction, span) {
            if (disposed) return;
            playT = pSec;
            if (direction) dir = direction < 0 ? -1 : 1;
            if (Number.isFinite(span) && span >= 0) spanSec = span;
            replan();
        },

        bufferedRanges() {
            const cells = [];
            const rng = loadable();
            const idxs = new Set(resident.keys());
            const pending = new Set();
            if (rng) for (const i of plan) if (!resident.has(i) && i <= rng.hi) pending.add(i);
            if (loadingIdx !== null) pending.add(loadingIdx);
            for (const i of pending) idxs.add(i);
            const sorted = Array.from(idxs).sort((a, b) => a - b);
            for (const i of sorted) {
                const b = source.bounds(i);
                cells.push({ t0: b[0], t1: b[1], state: resident.has(i) ? 'resident' : 'loading' });
            }
            const out = [];
            for (const c of cells) {
                const last = out[out.length - 1];
                if (last && last.state === c.state && c.t0 <= last.t1) last.t1 = Math.max(last.t1, c.t1);
                else out.push({ t0: c.t0, t1: c.t1, state: c.state });
            }
            return out;
        },

        onChange(cb) { listeners.add(cb); return () => listeners.delete(cb); },
        frontier() { return source.range().frontier; },
        size() { return resident.size; },

        dispose() {
            disposed = true;
            loadToken++;
            loadingIdx = null;
            if (retryTimer !== null) { clearTimeout(retryTimer); retryTimer = null; }
            if (typeof unsubSource === 'function') unsubSource();
            for (const w of waiters.values()) w.resolve();
            waiters.clear();
            listeners.clear();
            resident.clear();
            plan = [];
        },
    };
    return self;
}
