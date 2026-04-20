/**
 * event-store.js — Shared store of time-stamped notable events.
 *
 * Events feed two UI features: vertical "what happened here?" markers on the
 * telemetry charts (#10), and the command-history panel (#14).  Times use
 * the same seconds-since-epoch axis as telemetry-charts.js — events emitted
 * from ROS topic handlers line up exactly with chart x-values.
 *
 * Everything is in-memory; nothing persists across reload (by design — see
 * the Phase-2 plan).
 */

// ---- Types & palette ----

export const EVENT_TYPES = Object.freeze({
    STATE:       'state',
    FAULT:       'fault',
    COMMAND:     'command',
    CALIBRATION: 'calibration',
});

/**
 * Stroke colour for each event type on the chart draw-overlay.  Kept in JS
 * (rather than CSS vars) because uPlot draws on canvas and needs concrete
 * strings at stroke time — if we ever want theme-aware markers, these can
 * move to CSS vars with a read-on-rebuild shim.
 */
export const EVENT_COLORS = Object.freeze({
    state:       '#f59e0b',  // amber
    fault:       '#ef4444',  // red
    command:     '#06b6d4',  // cyan
    calibration: '#a78bfa',  // violet
});

// ---- Store ----

/** Maximum events retained in memory.  Older events are dropped. */
const MAX_EVENTS = 500;

/** Monotonic counter used to stamp each event with a stable ID — lets the
 *  UI reference a specific event (e.g. on hover) without relying on object
 *  identity across modules. */
let nextEventId = 1;

/** @type {Array<{id: number, t: number, type: string, label: string, detail?: string}>} */
const events = [];

/** @type {Set<(event) => void>} Subscribers notified on each new event. */
const listeners = new Set();

/** Currently-highlighted event ID — set by the command-history panel on row
 *  hover, read by the chart draw hook to paint that marker more prominently. */
let highlightedEventId = null;

/** @type {Set<(id: number|null) => void>} */
const highlightListeners = new Set();

/**
 * Emit a new event into the store.  `t` defaults to "now" (wall-clock
 * seconds).  If a caller wants to time-stamp against a ROS header, they
 * should pass the converted value.
 *
 * @param {{type: string, label: string, detail?: string, t?: number}} ev
 */
export function emitEvent(ev) {
    if (!ev || !ev.type || !ev.label) return;
    const entry = {
        id: nextEventId++,
        t: typeof ev.t === 'number' ? ev.t : Date.now() / 1000,
        type: ev.type,
        label: ev.label,
        detail: ev.detail || '',
    };
    events.push(entry);
    if (events.length > MAX_EVENTS) events.shift();
    for (const cb of listeners) {
        try { cb(entry); } catch { /* listener error — skip */ }
    }
}

/**
 * Return all events whose timestamp falls within [tMin, tMax].  Used by the
 * chart draw hook to figure out which markers are currently on screen.
 */
export function getEventsInRange(tMin, tMax) {
    const out = [];
    for (const ev of events) {
        if (ev.t >= tMin && ev.t <= tMax) out.push(ev);
    }
    return out;
}

/**
 * Return the most recent `limit` events of the given type (or any type when
 * `type` is null).  Newest first.  Used by the command-history panel.
 */
export function getRecentEvents(type, limit) {
    const out = [];
    for (let i = events.length - 1; i >= 0 && out.length < limit; i--) {
        if (type == null || events[i].type === type) out.push(events[i]);
    }
    return out;
}

/** Subscribe to new-event notifications.  Returns an unsubscribe fn. */
export function subscribeEvents(cb) {
    listeners.add(cb);
    return () => listeners.delete(cb);
}

/** Clear every stored event.  Not currently called — provided for future use. */
export function clearEvents() {
    events.length = 0;
    for (const cb of listeners) {
        try { cb(null); } catch { /* ignore */ }
    }
}

// ---- Highlight ----

/** Mark event `id` (or null to clear) as the currently-highlighted event. */
export function setHighlightedEvent(id) {
    if (highlightedEventId === id) return;
    highlightedEventId = id;
    for (const cb of highlightListeners) {
        try { cb(id); } catch { /* ignore */ }
    }
}

export function getHighlightedEventId() {
    return highlightedEventId;
}

/** Subscribe to highlight changes.  Returns an unsubscribe fn. */
export function subscribeHighlight(cb) {
    highlightListeners.add(cb);
    return () => highlightListeners.delete(cb);
}
