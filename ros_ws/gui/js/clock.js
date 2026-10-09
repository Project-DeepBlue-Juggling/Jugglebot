/**
 * clock.js — the GUI's injectable time source (replay design § 2).
 *
 * LIVE: `now()` is `Date.now()` and `setTimeout`/`clearTimeout` are the window
 * timers, so migrated call sites behave exactly as before.
 *
 * REPLAY (between `_enterReplay()` and `_exitReplay()`): `now()` returns the
 * value last given to `_setNow(ms)` (the replay engine sets the record's
 * `t*1000` before that record's handlers run, the playhead otherwise), and
 * timers go to a VIRTUAL queue that only advances when the engine calls
 * `_travel(absDeltaMs, speed)`. A paused engine never calls it, so every
 * watchdog is frozen. `_clearTimers()` drops the queue on seek.
 *
 * Timer scaling rule: a timer armed for `ms` fires when the accumulated
 * playhead travel reaches `ms * max(1, |speed|)`, so a watchdog measures
 * wall-equivalent dispatch silence (the engine decimates records to the live
 * rate at 1x and to the same wall rate above it). Implemented by accumulating
 * `absDeltaMs / max(1, |speed|)` of "wall-equivalent" time per `_travel` and
 * arming each timer at `wallEq + ms`, so a speed change mid-wait is handled.
 * Worked example: a 1000 ms watchdog at 4x fires after 4000 ms of playhead
 * travel (= 1000 ms of wall time at 4x); at 1x or 0.25x after 1000 ms of
 * travel. The engine passes |delta| so reverse playback ages timers too.
 *
 * `setInterval` is deliberately NOT provided: intervals (repaint/tick) stay on
 * wall time. Dependency-free on purpose (loaded by node harnesses).
 */

let replay = false;
let replayNow = 0;
let wallEq = 0;          // accumulated wall-equivalent travel (ms)
let nextId = 1;           // virtual ids are NEGATIVE (-nextId) so they can never collide with a real timer handle
const FUSE = 10000;      // max timers fired in one _travel (a zero-delay re-arming loop)
let seq = 0;
/** @type {Map<number, {id:number, fn:Function, due:number, seq:number}>} */
const timers = new Map();

/** @returns {number} ms, Date.now()-compatible. */
export function now() {
    return replay ? replayNow : Date.now(); // wall-clock: live source of clock.now()
}

/** @returns {boolean} */
export function isReplay() { return replay; }

/**
 * @param {Function} fn
 * @param {number} ms
 * @returns {number|object} id for clearTimeout
 */
export function setTimeout(fn, ms) {
    if (!replay) return globalThis.setTimeout(fn, ms);
    const id = -(nextId++);
    timers.set(id, { id, fn, due: wallEq + Math.max(0, +ms || 0), seq: seq++ });
    return id;
}

/** @param {number|object} id */
export function clearTimeout(id) {
    if (id === undefined || id === null) return;
    if (typeof id === 'number' && id < 0) { timers.delete(id); return; }
    globalThis.clearTimeout(id);
}

/** Switch to the playhead source and the virtual timer queue. */
export function _enterReplay() {
    replay = true;
    timers.clear();
    wallEq = 0;
}

/** Restore wall time and cancel every virtual timer. */
export function _exitReplay() {
    replay = false;
    timers.clear();
    wallEq = 0;
}

/** @param {number} ms value `now()` returns in replay. */
export function _setNow(ms) { replayNow = ms; }

/**
 * Advance the virtual queue by `absDeltaMs` of playhead travel at `speed`
 * (see the scaling rule above). Due timers fire synchronously, in deadline
 * order (ties by arming order), each at most once. A timer armed by a firing
 * callback is due relative to the already-advanced clock.
 * @param {number} absDeltaMs
 * @param {number} speed
 */
export function _travel(absDeltaMs, speed) {
    if (!replay) return;
    const d = Math.abs(+absDeltaMs || 0);
    if (!(d > 0)) return;
    wallEq += d / Math.max(1, Math.abs(+speed || 0));
    let fired = 0;
    for (;;) {
        let best = null;
        for (const t of timers.values()) {
            if (t.due <= wallEq && (best === null || t.due < best.due
                || (t.due === best.due && t.seq < best.seq))) best = t;
        }
        if (best === null) return;
        timers.delete(best.id);
        if (++fired > FUSE) {
            console.error('replay timer fuse: >' + FUSE + ' timers in one _travel; queue cleared');
            timers.clear();
            return;
        }
        try { best.fn(); } catch (e) { console.error('replay timer', e); }
    }
}

/** Drop every pending virtual timer (engine calls this on seek). */
export function _clearTimers() { timers.clear(); }

/** @returns {number} pending virtual timers (tests). */
export function _pending() { return timers.size; }
