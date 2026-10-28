/**
 * records-model.js — the DOM-free half of the records panel (JB-35).
 *
 * Input is GET /api/records (gui_server.py -> records.to_json of the repo's
 * docs/records.md): { rule, patterns: [{ pattern, label, balls, history,
 * record: null | { when_local, throws, catches, end_code, category, note } }] },
 * already in file order (balls, then Pattern id).  Imports nothing, so
 * tests/ros/js/records_panel_harness.js runs it under node as-is.
 */

/** localStorage key: the JSON array of Pattern ids the operator hid in View. */
export const RECORDS_HIDDEN_KEY = 'jugglebot-records-hidden';

/** How long a just-beaten record's row stays highlighted. */
export const RECORD_FLASH_MS = 6000;

/** One display row per Pattern, hidden ones dropped. */
export function recordRows(json, hidden) {
    const off = hidden || new Set();
    const pats = (json && Array.isArray(json.patterns)) ? json.patterns : [];
    const rows = [];
    for (const p of pats) {
        if (!p || typeof p.pattern !== 'string' || off.has(p.pattern)) continue;
        const r = p.record;
        rows.push({
            pattern: p.pattern,
            label: String(p.label || p.pattern),
            balls: Number(p.balls) || 0,
            score: r ? r.catches + '/' + r.throws : '—',
            date: r ? String(r.when_local).slice(0, 10) : '',
            title: r
                ? (r.catches + ' Catches in ' + r.throws + ' Throws, ' + r.when_local
                   + ', ' + r.category + ' (' + r.end_code + ')'
                   + (r.note ? ' — ' + r.note : '')
                   + (p.history > 1 ? ' · beaten ' + (p.history - 1) + '× before' : ''))
                : 'No record yet',
            empty: !r,
        });
    }
    return rows;
}

/** The View-menu entries: every Pattern in the ledger, checked = shown. */
export function menuEntries(json, hidden) {
    const off = hidden || new Set();
    const pats = (json && Array.isArray(json.patterns)) ? json.patterns : [];
    return pats.filter(p => p && typeof p.pattern === 'string')
        .map(p => ({ pattern: p.pattern, label: 'Records: ' + String(p.label || p.pattern),
                     checked: !off.has(p.pattern) }));
}

/** The hidden set from storage; anything unreadable is "all shown". */
export function loadHidden(storage) {
    try {
        const raw = storage && storage.getItem(RECORDS_HIDDEN_KEY);
        const arr = raw ? JSON.parse(raw) : [];
        return new Set(Array.isArray(arr) ? arr.filter(x => typeof x === 'string') : []);
    } catch (_e) {
        return new Set();
    }
}

/** Persist the hidden set; a blocked storage is not an error. */
export function saveHidden(storage, hidden) {
    try {
        if (storage) storage.setItem(RECORDS_HIDDEN_KEY, JSON.stringify([...hidden].sort()));
    } catch (_e) { /* private window / blocked site data */ }
}

/** A skills/attempt message -> null, or { pattern, beaten } for a juggle_end. */
export function attemptEnd(msg) {
    if (!msg || msg.name !== 'juggle_end') return null;
    const f = {};
    for (const v of (msg.values || [])) f[v.key] = v.value;
    return { pattern: String(f.pattern || ''), beaten: f.record_beaten === '1' };
}
