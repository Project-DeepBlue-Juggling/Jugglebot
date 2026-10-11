/**
 * trail-settings.js — the viewer's trail tail length (ms), persisted per viewer.
 * 0 = trails off; range 0..5000 snapped to 100; default 1000. Dependency-free;
 * every storage access is guarded so it also works with no localStorage (node,
 * private windows).
 */
const KEY = 'jugglebot-trail-tail-ms';
export const TAIL_MIN_MS = 0;
export const TAIL_MAX_MS = 5000;
export const TAIL_STEP_MS = 100;
export const TAIL_DEFAULT_MS = 1000;

const listeners = [];
let value = null;   // lazily loaded

function normalise(ms) {
    const v = Number(ms);
    if (!Number.isFinite(v)) return TAIL_DEFAULT_MS;
    const snapped = Math.round(v / TAIL_STEP_MS) * TAIL_STEP_MS;
    return Math.min(TAIL_MAX_MS, Math.max(TAIL_MIN_MS, snapped));
}

function load() {
    try {
        const raw = globalThis.localStorage && globalThis.localStorage.getItem(KEY);
        if (raw !== null && raw !== undefined && raw !== '') return normalise(raw);
    } catch (e) { /* storage unavailable */ }
    return TAIL_DEFAULT_MS;
}

export function getTailMs() {
    if (value === null) value = load();
    return value;
}

export function setTailMs(ms) {
    const v = normalise(ms);
    const changed = v !== getTailMs();
    value = v;
    try { if (globalThis.localStorage) globalThis.localStorage.setItem(KEY, String(v)); } catch (e) { /* ignore */ }
    if (changed) for (let i = 0; i < listeners.length; i++) listeners[i](v);
    return v;
}

/** @returns {() => void} unsubscribe */
export function onTailChange(cb) {
    listeners.push(cb);
    return () => { const i = listeners.indexOf(cb); if (i >= 0) listeners.splice(i, 1); };
}
