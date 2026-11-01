/**
 * console-logic.js — pure, DOM-free logic of the CONSOLE view (console-view.js).
 *
 * The CONSOLE is a READ-ONLY view of /rosout (rcl_interfaces/msg/Log).  Everything
 * that can be wrong without a browser lives here so a node test can pin it:
 * level naming, the severity-chip + search filter, HTML-safe highlight, the ring
 * buffer cap, and the replay hit selection relative to a playhead.
 *
 * A line is {t, level, name, msg}: t = epoch SECONDS (live: the message stamp;
 * replay: the MCAP log time, which is the time the trackbar seeks in), level =
 * the numeric Log level (Foxy: DEBUG 10, INFO 20, WARN 30, ERROR 40, FATAL 50).
 */

export const LEVELS = Object.freeze(['DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL']);
/** Ring buffer capacity (live).  ~10 msg/s measured => ~8 min of history. */
export const RING_CAP = 5000;
/** A playhead within this many seconds of a line counts as ON it (prev/next skip it). */
export const HIT_EPS_S = 0.0005;

/** Numeric Log level -> chip key.  Unknown/odd values bucket to the nearest floor. */
export function levelName(v) {
    const n = +v;
    if (n >= 50) return 'FATAL';
    if (n >= 40) return 'ERROR';
    if (n >= 30) return 'WARN';
    if (n >= 20) return 'INFO';
    return 'DEBUG';
}

/** The ONLY place message text becomes markup.  Never use innerHTML on raw text. */
export function escapeHtml(s) {
    return String(s === undefined || s === null ? '' : s)
        .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;')
        .replace(/"/g, '&quot;').replace(/'/g, '&#39;');
}

/** Escape `text`, wrapping case-insensitive matches of `query` in <mark>. */
export function highlightHtml(text, query) {
    const s = String(text === undefined || text === null ? '' : text);
    const q = (query || '').toLowerCase();
    if (!q) return escapeHtml(s);
    const low = s.toLowerCase();
    let out = '';
    let from = 0;
    for (;;) {
        const at = low.indexOf(q, from);
        if (at < 0) break;
        out += escapeHtml(s.slice(from, at)) + '<mark>' + escapeHtml(s.slice(at, at + q.length)) + '</mark>';
        from = at + q.length;
    }
    return out + escapeHtml(s.slice(from));
}

/** rosbridge JSON / MCAP-hydrated Log -> line, or null when unusable. */
export function normalizeLine(msg, fallbackT) {
    if (!msg || typeof msg !== 'object') return null;
    const st = msg.stamp || {};
    const sec = +st.sec, ns = +(st.nanosec !== undefined ? st.nanosec : st.nanosecs);
    let t = Number.isFinite(sec) && sec > 0 ? sec + (Number.isFinite(ns) ? ns : 0) / 1e9 : NaN;
    if (!Number.isFinite(t)) t = Number.isFinite(fallbackT) ? fallbackT : 0;
    return { t, level: +msg.level || 0, name: String(msg.name === undefined ? '' : msg.name), msg: String(msg.msg === undefined ? '' : msg.msg) };
}

/** Append to a capped ring (oldest dropped).  Mutates and returns `arr`. */
export function pushCapped(arr, line, cap = RING_CAP) {
    arr.push(line);
    if (arr.length > cap) arr.splice(0, arr.length - cap);
    return arr;
}

/** True when the line passes the search (case-insensitive substring over name + message). */
export function matchesQuery(line, query) {
    const q = (query || '').toLowerCase();
    if (!q) return true;
    return line.name.toLowerCase().indexOf(q) >= 0 || line.msg.toLowerCase().indexOf(q) >= 0;
}

/** enabled: object/Set-like map LEVELS key -> bool, or a Set of keys. */
function isOn(enabled, key) {
    return typeof enabled.has === 'function' ? enabled.has(key) : !!enabled[key];
}

/** Indices of the lines passing BOTH the severity chips and the search, ascending. */
export function filterIndices(lines, enabled, query) {
    const out = [];
    for (let i = 0; i < lines.length; i++) {
        const l = lines[i];
        if (isOn(enabled, levelName(l.level)) && matchesQuery(l, query)) out.push(i);
    }
    return out;
}

/** Per-level counts over the lines passing the SEARCH (chip state ignored, so a chip
 *  that is off still says how many lines it is hiding). */
export function levelCounts(lines, query) {
    const c = { DEBUG: 0, INFO: 0, WARN: 0, ERROR: 0, FATAL: 0 };
    for (const l of lines) if (matchesQuery(l, query)) c[levelName(l.level)]++;
    return c;
}

/** Replay: the hit nearest at-or-before the playhead (the "current" line); the first hit
 *  when the playhead precedes them all; -1 when there are no hits.  `hits` = ascending line indices. */
export function currentHit(lines, hits, playhead) {
    if (!hits.length) return -1;
    let cur = hits[0];
    for (const i of hits) { if (lines[i].t <= playhead + HIT_EPS_S) cur = i; else break; }
    return cur;
}

/** Replay: the nearest hit strictly BEFORE the playhead, or -1. */
export function prevHit(lines, hits, playhead) {
    let r = -1;
    for (const i of hits) { if (lines[i].t < playhead - HIT_EPS_S) r = i; else break; }
    return r;
}

/** Replay: the nearest hit strictly AFTER the playhead, or -1. */
export function nextHit(lines, hits, playhead) {
    for (const i of hits) if (lines[i].t > playhead + HIT_EPS_S) return i;
    return -1;
}

/** Local-time HH:MM:SS.mmm of an epoch-seconds value. */
export function formatTime(t) {
    if (!Number.isFinite(t) || t <= 0) return '--:--:--.---';
    const d = new Date(Math.round(t * 1000));
    const p = (n, w) => String(n).padStart(w, '0');
    return `${p(d.getHours(), 2)}:${p(d.getMinutes(), 2)}:${p(d.getSeconds(), 2)}.${p(d.getMilliseconds(), 3)}`;
}

/** Markup of one line.  `cur` marks the replay playhead line. */
export function lineHtml(line, idx, query, cur) {
    const lv = levelName(line.level);
    return `<div class="console-line console-${lv.toLowerCase()}${cur ? ' console-current' : ''}" data-k="${idx}" data-t="${line.t}">` +
        `<span class="console-time">${formatTime(line.t)}</span>` +
        `<span class="console-lvl console-lvl-${lv.toLowerCase()}">${lv}</span>` +
        `<span class="console-node" title="${escapeHtml(line.name)}">${highlightHtml(line.name, query)}</span>` +
        `<span class="console-msg">${highlightHtml(line.msg, query)}</span></div>`;
}

/** Persisted chip state: object LEVELS key -> bool; unknown/garbled storage => all on. */
export function parseEnabled(raw) {
    const all = {};
    for (const k of LEVELS) all[k] = true;
    if (!raw) return all;
    try {
        const o = JSON.parse(raw);
        if (o && typeof o === 'object') for (const k of LEVELS) if (typeof o[k] === 'boolean') all[k] = o[k];
    } catch (e) { /* garbled => all on */ }
    return all;
}
