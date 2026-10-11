/**
 * trail-window.js - the replay trail feeder (Phase 5, design D4: the one trail writer while replaying).
 *
 * Feeds js/trail-feed.js from the RESIDENT typed chunk columns of /mocap_data and /balls at recorded rate:
 *   - a forward step no longer than the tail APPENDS the records in (lastP, p];
 *   - every other playhead move (seek, scrub, reverse step, a jump past the tail, a tail change, a
 *     non-resident hole that has since arrived) REBUILDS: feed.reset(), then the records in [max(t0, p - tail), p].
 *   The fed span is max(tail, BALL_STALE_SEC): tail 0 hides the trails, never the ball spheres.
 * Then feed.tick(p) (the feed only self-ticks when its render time is null) and feed.setRenderTime(p).
 *
 * Records are read straight from the typed columns (mocap `markers` CSR leaves position.x|y|z + label, `balls`
 * CSR leaves id, status, position.x|y|z) through primitive-argument calls on the feed: no hydrate, no
 * allocation per record. The two topics are merged in time order with two cursors (mocap first on a tie).
 * A topic without a typed `kinds` descriptor (or without the expected CSR group) is skipped.
 *
 * Known approximation (by design): D1's 150 ms staleness end is evaluated by the feed on the next /balls
 * record and by the final tick(p); a rebuild that spans a ball-less gap relies on the feed doing the same
 * on the first record after the gap, which is what the live path does too.
 */

import { BALL_STALE_SEC } from '../trail-feed.js';

const MOCAP = '/mocap_data';
const BALLS = '/balls';

/** First index in sorted `t[0..n)` with t[i] > x (upper) or t[i] >= x (lower, when `incl`). */
function bound(t, n, x, incl) {
    let lo = 0, hi = n;
    while (lo < hi) {
        const mid = (lo + hi) >>> 1;
        if (incl ? t[mid] < x : t[mid] <= x) lo = mid + 1; else hi = mid;
    }
    return lo;
}

/** The typed CSR group `name` of a topic, or null. */
function csrGroup(tp, name) {
    if (!tp || !tp.kinds || !tp.cols) return null;
    const kd = tp.kinds[name];
    const col = tp.cols[name];
    if (!kd || typeof kd.csr !== 'object' || !col || !col.offsets || !col.leaves) return null;
    return col;
}

/**
 * @param {{source:object, cache:object, feed:object, getTailMs:function, onTailChange?:function}} o
 */
export function createTrailWindow(o) {
    const { source, cache, feed, getTailMs } = o;
    let lastP = null;
    let resetPending = false;
    let incomplete = false;
    let disposed = false;
    let rebuilds = 0, appends = 0, lastRebuildMs = 0, lastRecords = 0;
    let nRec = 0;

    // Feed every record with lo < t <= hi (or lo <= t when loIncl). Sets `incomplete` if a needed chunk is absent.
    function feedRange(lo, hi, loIncl) {
        const a = source.chunkIndex(lo);
        const b = source.chunkIndex(hi);
        for (let i = a; i <= b; i++) {
            const ch = cache.peek(i);
            if (!ch || !ch.topics) { incomplete = true; continue; }
            const mt = ch.topics[MOCAP], bt = ch.topics[BALLS];
            const mk = csrGroup(mt, 'markers');
            const bl = csrGroup(bt, 'balls');
            let mi = 0, me = 0, bi = 0, be = 0;
            if (mk) { mi = bound(mt.t, mt.n, lo, loIncl); me = bound(mt.t, mt.n, hi, false); }
            if (bl) { bi = bound(bt.t, bt.n, lo, loIncl); be = bound(bt.t, bt.n, hi, false); }
            const mt_t = mk ? mt.t : null, bt_t = bl ? bt.t : null;
            const mOff = mk ? mk.offsets : null, mL = mk ? mk.leaves : null;
            const bOff = bl ? bl.offsets : null, bL = bl ? bl.leaves : null;
            while (mi < me || bi < be) {
                if (mi < me && (bi >= be || mt_t[mi] <= bt_t[bi])) {
                    const t = mt_t[mi];
                    feed.beginMocap(t);
                    const e = mOff[mi + 1];
                    for (let j = mOff[mi]; j < e; j++) {
                        feed.marker(t, mL.label[j], mL['position.x'][j], mL['position.y'][j], mL['position.z'][j]);
                    }
                    feed.endMocap(t);
                    mi++;
                } else {
                    const t = bt_t[bi];
                    feed.beginBalls(t);
                    const e = bOff[bi + 1];
                    for (let j = bOff[bi]; j < e; j++) {
                        feed.ball(t, bL.id[j], bL.status[j], bL['position.x'][j], bL['position.y'][j], bL['position.z'][j]);
                    }
                    feed.endBalls(t);
                    bi++;
                }
                nRec++;
            }
        }
    }

    // The fed span is at least the ball staleness: tail 0 turns the TRAILS off (layer.render draws nothing at
    // tail <= 0), never the ball spheres, which read the feed's ball state (live parity, audit 2026-10-11).
    function span() { return Math.max(getTailMs() / 1000, BALL_STALE_SEC); }

    function doRebuild(p) {
        const tail = span();
        feed.reset();
        incomplete = false;
        resetPending = false;
        {
            const t0 = source.range().t0;
            const lo = Math.max(t0, p - tail);
            nRec = 0;
            const w0 = performance.now(); // wall-clock: profiling the rebuild cost only, never feeds state
            feedRange(lo, p, true);
            lastRebuildMs = performance.now() - w0; // wall-clock: profiling only
            lastRecords = nRec;
        }
        rebuilds++;
    }

    function onPlayhead(p) {
        if (disposed) return;
        const tail = span();
        if (!resetPending && lastP !== null && p >= lastP && p <= lastP + tail) {
            if (p > lastP) { feedRange(lastP, p, false); appends++; }
        } else {
            doRebuild(p);
        }
        feed.tick(p);
        feed.setRenderTime(p);
        lastP = p;
    }

    function onResetForSeek() { resetPending = true; }

    function rebuild() {
        if (disposed || lastP === null) return;
        doRebuild(lastP);
        if (getTailMs() > 0) feed.tick(lastP);
        feed.setRenderTime(lastP);
    }

    const unsubCache = cache.onChange((ev) => {
        if (incomplete && ev && ev.reason === 'residency') rebuild();
    });
    const unsubTail = typeof o.onTailChange === 'function' ? o.onTailChange(() => rebuild()) : null;

    function dispose() {
        if (disposed) return;
        disposed = true;
        if (unsubCache) unsubCache();
        if (typeof unsubTail === 'function') unsubTail();
        feed.reset();
        feed.setRenderTime(null);
    }

    return {
        onPlayhead, onResetForSeek, rebuild, dispose,
        stats: () => ({ rebuilds, appends, incomplete, lastP, lastRebuildMs, lastRecords }),
    };
}
