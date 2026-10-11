/**
 * trail-settings.js — the viewer's trail tail length (ms), persisted per viewer.
 *
 * STORED value = the owner's request (non-negative, 0 = trails off, default 1000), persisted. The range it
 * can take at the moment is 0..max: max is 5000 live and the recording duration in replay (set by
 * replay/mode.js). The stored value is NOT clamped to the max (so a short recording does not forget a long
 * request); consumers read the EFFECTIVE value = min(stored, max). The NEAR tier (ribbons, markers, balls) draws
 * getNearTailMs() = min(effective, NEAR_MAX_MS).
 *
 * Snapping of the stored value: 100 ms below 5 s, 1 s from 5 s to 60 s, 5 s above.
 * onTailChange(cb) fires cb({effectiveMs, maxMs, storedMs}) whenever the EFFECTIVE value or the max changes.
 * Dependency-free; every storage access is guarded (node, private windows).
 */
const KEY = 'jugglebot-trail-tail-ms';
export const NEAR_MAX_MS = 5000;               // the near tier's cap, and the live max
export const TAIL_DEFAULT_MS = 1000;

const listeners = [];
let value = null;           // stored request, lazily loaded
let maxMs = NEAR_MAX_MS;

export function snapMs(ms) {
    const v = Number(ms);
    if (!Number.isFinite(v) || v < 0) return v < 0 ? 0 : TAIL_DEFAULT_MS;
    const step = v < 5000 ? 100 : (v < 60000 ? 1000 : 5000);
    return Math.round(v / step) * step;
}

function load() {
    try {
        const raw = globalThis.localStorage && globalThis.localStorage.getItem(KEY);
        if (raw !== null && raw !== undefined && raw !== '') return snapMs(raw);
    } catch (e) { /* storage unavailable */ }
    return TAIL_DEFAULT_MS;
}

export function getTailMs() {
    if (value === null) value = load();
    return value;
}
export function getTailMaxMs() { return maxMs; }
export function getEffectiveTailMs() { return Math.min(getTailMs(), maxMs); }
export function getNearTailMs() { return Math.min(getEffectiveTailMs(), NEAR_MAX_MS); }

function fire(effBefore, maxBefore) {
    if (getEffectiveTailMs() === effBefore && maxMs === maxBefore) return;
    const ev = { effectiveMs: getEffectiveTailMs(), maxMs, storedMs: getTailMs() };
    for (let i = 0; i < listeners.length; i++) listeners[i](ev);
}

export function setTailMs(ms) {
    const effBefore = getEffectiveTailMs();
    // a request for exactly the current max (the slider's right end) is stored unsnapped, so it reaches the
    // recording's start whatever the max's rounding (audit 2026-10-11); a longer request is kept as asked
    const v = Number(ms) === maxMs ? maxMs : snapMs(ms);
    value = v;
    try { if (globalThis.localStorage) globalThis.localStorage.setItem(KEY, String(v)); } catch (e) { /* ignore */ }
    fire(effBefore, maxMs);
    return v;
}

/** Live 5000; replay mode sets the recording duration (ms) and restores 5000 on every exit path. */
export function setTailMaxMs(ms) {
    const effBefore = getEffectiveTailMs();
    const maxBefore = maxMs;
    const v = Number(ms);
    maxMs = Number.isFinite(v) && v > 0 ? Math.round(v) : NEAR_MAX_MS;
    fire(effBefore, maxBefore);
    return maxMs;
}

/** @returns {() => void} unsubscribe */
export function onTailChange(cb) {
    listeners.push(cb);
    return () => { const i = listeners.indexOf(cb); if (i >= 0) listeners.splice(i, 1); };
}

/** Test hook: forget the cached stored value and restore the live max. */
export function _resetForTest() { value = null; maxMs = NEAR_MAX_MS; listeners.length = 0; }

// ---- slider mapping (pure) ----
/** Fraction of slider travel spent on the linear 0..5 s part when max > 5 s. */
export const LIN_FRAC = 0.6;

/** position in [0,1] -> ms (unsnapped-then-snapped), for a given max. */
export function positionToMs(pos, max) {
    const p = Math.min(1, Math.max(0, +pos || 0));
    if (!(max > NEAR_MAX_MS)) return snapMs(p * Math.max(0, max));
    if (p <= LIN_FRAC) return snapMs((p / LIN_FRAC) * NEAR_MAX_MS);
    if (p >= 1) return Math.round(max);
    const f = (p - LIN_FRAC) / (1 - LIN_FRAC);
    return Math.min(Math.round(max), snapMs(NEAR_MAX_MS * Math.pow(max / NEAR_MAX_MS, f)));
}

/** ms -> position in [0,1] for a given max (inverse of positionToMs up to snapping). */
export function msToPosition(ms, max) {
    const m = Math.min(Math.max(0, +ms || 0), Math.max(0, max));
    if (!(max > NEAR_MAX_MS)) return max > 0 ? m / max : 0;
    if (m <= NEAR_MAX_MS) return (m / NEAR_MAX_MS) * LIN_FRAC;
    return LIN_FRAC + (1 - LIN_FRAC) * (Math.log(m / NEAR_MAX_MS) / Math.log(max / NEAR_MAX_MS));
}

/** "off", "0.8 s", "12 s", "3:20", "1:02:00". */
export function formatTail(ms) {
    if (!(ms > 0)) return 'off';
    if (ms < 10000) return (ms / 1000).toFixed(1) + ' s';
    const s = Math.round(ms / 1000);
    if (s < 60) return s + ' s';
    const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), r = s % 60;
    const two = (n) => (n < 10 ? '0' : '') + n;
    return h > 0 ? h + ':' + two(m) + ':' + two(r) : m + ':' + two(r);
}

/** Max label: whole seconds below a minute ("5 s"), else m:ss / h:mm:ss. */
export function formatMax(ms) {
    return ms < 60000 ? Math.round(ms / 1000) + ' s' : formatTail(ms);
}
