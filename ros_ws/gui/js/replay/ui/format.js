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

/** Human messages for every enterReplay / list rejection reason. */
const REFUSALS = {
    recording_in_progress: 'This recording is still being written; open it once the recording has stopped.',
    disk_low: 'Not enough free disk space to convert this recording. Free some space or shrink the replay cache.',
    worker_unavailable: 'The replay converter is not running, so this recording cannot be opened.',
    busy: 'The converter is busy with another recording. Try again in a moment.',
    ros_running: 'Replay is unavailable while the robot stack is running.',
    'no session buffer': 'There is no live session in this page\'s memory to replay.',
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
 * @param {{[id:string]:{topics?:object, pct?:number}}} extras per-id enrichment (manifest topics, status pct)
 * @param {{has:boolean, seconds:number}|null} session  the in-memory session buffer
 * @param {string} filter text over date + id + topic names
 * @returns {object[]} rows: {key, kind, id, when, name, duration, size, topicCount, chips, cache, pct,
 *   selectable, first, haystack}
 */
export function buildRows(listing, extras, session, filter) {
    const rows = [];
    const recs = ((listing && listing.recordings) || []).slice();
    recs.sort((a, b) => rowStart(b) - rowStart(a));
    if (session && session.has) {
        rows.push({
            key: 'session', kind: 'session', id: null, first: true, selectable: true,
            when: 'Last live session (in memory, ' + fmtDuration(session.seconds) + ' of data)',
            name: '', duration: fmtDuration(session.seconds), size: '', topicCount: null,
            chips: [], cache: 'memory', pct: null, haystack: 'last live session in memory',
        });
    }
    for (const r of recs) {
        const ex = (extras && extras[r.id]) || {};
        const topics = topicSet(ex.topics || r.topics);
        const chips = KEY_TOPICS.map((k) => ({ label: k.label, missing: topics ? !topics.has(k.topic) : false, known: !!topics }));
        let cache = r.cache || 'none';
        let pct = null;
        if (r.in_progress) cache = 'in_progress';
        else if (cache === 'converting' && typeof ex.pct === 'number') pct = Math.max(0, Math.min(100, Math.round(ex.pct)));
        const noIndex = r.duration_s === null || r.duration_s === undefined;
        const when = (startOk(r) ? fmtDateTime(rowStart(r)) : (dateFromId(r.id) || r.id));
        rows.push({
            key: r.id, kind: 'recording', id: r.id, first: false, selectable: !r.in_progress,
            when, name: r.id,
            duration: noIndex ? null : fmtDuration(r.duration_s), noIndex,
            size: fmtSize(r.size_bytes),
            topicCount: topics ? topics.size : null,
            chips, cache, pct,
            haystack: (when + ' ' + r.id + ' ' + (topics ? Array.from(topics).join(' ') : '')).toLowerCase(),
        });
    }
    const f = (filter || '').trim().toLowerCase();
    return f ? rows.filter((r) => r.first || r.haystack.indexOf(f) >= 0) : rows;
}

function startOk(r) {
    return nsOk(r) || typeof r.mtime === 'number';
}

/** Cache cell text + css modifier for a row. */
export function cacheCell(row, opening) {
    if (opening) return { text: 'opening…', cls: 'conv' };
    switch (row.cache) {
        case 'memory': return { text: 'in memory', cls: 'complete' };
        case 'complete': return { text: 'cached', cls: 'complete' };
        case 'stale': return { text: 'stale — will reconvert', cls: 'warn' };
        case 'converting': return { text: row.pct === null ? 'converting' : 'converting ' + row.pct + ' %', cls: 'conv' };
        case 'in_progress': return { text: 'recording in progress', cls: 'refuse' };
        case 'failed': return { text: 'conversion failed', cls: 'refuse' };
        default: return row.noIndex ? { text: 'duration unknown, no index', cls: 'warn' } : { text: 'not converted', cls: 'none' };
    }
}
