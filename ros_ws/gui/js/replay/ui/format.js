/**
 * format.js — pure view-model helpers for the replay lobby / picker (no DOM, node-testable).
 */

/** Key topics the picker shows a chip for (label -> manifest topic). A chip is struck when the
 *  recording has zero messages on it (charting decision 18). */
export const KEY_TOPICS = [
    { label: 'robot_state', topic: '/robot_state' },
    { label: 'balls', topic: '/balls' },
    { label: 'cone catch', topic: '/cone/catch_event' },
    { label: 'bb state', topic: '/bb/heartbeat' },
];

/** Human messages for every enterReplay / list rejection reason (McapSource `.reason` vocabulary + mode). */
const REFUSALS = {
    recording_in_progress: 'This recording is still being written; open it once the recording has stopped.',
    in_progress: 'This recording is still being written; open it once the recording has stopped.',
    no_index: 'This recording has no index (the recorder was killed before closing it), so it cannot be opened.',
    compressed: 'This recording uses compressed chunks, which the replay reader does not support.',
    changed: 'The recording changed on disk while it was being read. Close and reopen it.',
    http: 'The recording could not be read from the server. Check the connection and try again.',
    empty: 'This recording contains no replayable messages.',
    decode: 'This recording could not be decoded.',
    range: 'The server could not serve that part of the recording.',
    ros_running: 'Replay is unavailable while the robot stack is running.',
    connected: 'Connected to the robot. Disconnect first; replay is only available while disconnected.',
};

/** @param {string} reason @returns {string} */
export function refusalMessage(reason) {
    if (REFUSALS[reason]) return REFUSALS[reason];
    return 'Could not open the recording' + (reason ? ' (' + reason + ')' : '') + '.';
}

const p2 = (n) => (n < 10 ? '0' : '') + n;

/** @param {number} sec @returns {string} MM:SS, or H:MM:SS from one hour. */
export function fmtDuration(sec) {
    if (sec === null || sec === undefined || !isFinite(sec)) return '?';
    const s = Math.max(0, Math.round(sec));
    const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), r = s % 60;
    return h > 0 ? h + ':' + p2(m) + ':' + p2(r) : p2(m) + ':' + p2(r);
}

/** @param {number} bytes @returns {string} */
export function fmtSize(bytes) {
    if (!isFinite(bytes) || bytes === null || bytes === undefined) return '?';
    const u = ['B', 'kB', 'MB', 'GB', 'TB'];
    let v = bytes, i = 0;
    while (v >= 1000 && i < u.length - 1) { v /= 1000; i++; }
    return (i === 0 ? String(Math.round(v)) : v >= 100 ? v.toFixed(0) : v.toFixed(1)) + ' ' + u[i];
}

/** @param {number} epochSec @returns {string} local 'YYYY-MM-DD HH:MM:SS' */
export function fmtDateTime(epochSec) {
    const d = new Date(epochSec * 1000);
    return d.getFullYear() + '-' + p2(d.getMonth() + 1) + '-' + p2(d.getDate()) + ' '
        + p2(d.getHours()) + ':' + p2(d.getMinutes()) + ':' + p2(d.getSeconds());
}

/** 'YYYY-MM-DD_HH-MM-SS' (rosbag folder name) -> 'YYYY-MM-DD HH:MM:SS' or null. */
export function dateFromId(id) {
    const m = /^(\d{4}-\d{2}-\d{2})[_ T](\d{2})[-:](\d{2})[-:](\d{2})/.exec(id || '');
    return m ? m[1] + ' ' + m[2] + ':' + m[3] + ':' + m[4] : null;
}

// An empty bag reports start_ns = INT64_MAX (9.22e18): not a real stamp, so it must not sort to the top.
function nsOk(r) { return typeof r.start_ns === 'number' && r.start_ns > 0 && r.start_ns < 4e18; }

function rowStart(r) {
    if (nsOk(r)) return r.start_ns / 1e9;
    if (typeof r.mtime === 'number') return r.mtime - (r.duration_s || 0);
    return 0;
}

/** Topic-name set of a manifest-ish object ({topics:{'/x':{count}}}) with count >= 1, or null. */
function topicSet(topics) {
    if (!topics) return null;
    if (Array.isArray(topics)) return new Set(topics);
    const s = new Set();
    for (const k of Object.keys(topics)) {
        const v = topics[k];
        const n = v && typeof v === 'object' ? v.count : v;
        if (n === undefined || n >= 1) s.add(k);
    }
    return s;
}

/**
 * Build the picker rows.
 * @param {object|null} listing  GET /api/replay/recordings body (or null)
 * @param {string} filter text over date + id + topic names
 * @returns {object[]} rows: {key, kind, id, when, name, duration, size, topicCount, chips, state,
 *   selectable, first, haystack}; `state` is 'ready' | 'noindex' | 'inprogress'
 */
export function buildRows(listing, filter) {
    const rows = [];
    const recs = ((listing && listing.recordings) || []).slice();
    recs.sort((a, b) => rowStart(b) - rowStart(a));
    for (const r of recs) {
        const topics = topicSet(r.topics);
        const chips = KEY_TOPICS.map((k) => ({ label: k.label, missing: topics ? !topics.has(k.topic) : false, known: !!topics }));
        const state = r.in_progress ? 'inprogress' : (r.indexed === false ? 'noindex' : 'ready');
        const noIndex = r.duration_s === null || r.duration_s === undefined;
        const when = (startOk(r) ? fmtDateTime(rowStart(r)) : (dateFromId(r.id) || r.id));
        rows.push({
            key: r.id, kind: 'recording', id: r.id, first: false, selectable: state === 'ready',
            when, name: r.id,
            duration: noIndex ? null : fmtDuration(r.duration_s), noIndex,
            size: fmtSize(r.size_bytes),
            topicCount: topics ? topics.size : null,
            chips, state,
            haystack: (when + ' ' + r.id + ' ' + (topics ? Array.from(topics).join(' ') : '')).toLowerCase(),
        });
    }
    const f = (filter || '').trim().toLowerCase();
    return f ? rows.filter((r) => r.first || r.haystack.indexOf(f) >= 0) : rows;
}

function startOk(r) {
    return nsOk(r) || typeof r.mtime === 'number';
}

/** Status cell text + css modifier for a row (no conversion/cache states exist: a recording is read in place). */
export function stateCell(row, opening) {
    if (opening) return { text: 'opening…', cls: 'conv' };
    switch (row.state) {
        case 'inprogress': return { text: 'recording in progress', cls: 'refuse' };
        case 'noindex': return { text: 'no index', cls: 'warn' };
        default: return { text: 'ready', cls: 'none' };
    }
}
