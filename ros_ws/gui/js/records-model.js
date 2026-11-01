/**
 * records-model.js — the DOM-free half of the records panel (JB-35).
 *
 * Input is GET /api/records (gui_server.py -> records.to_json of the repo's
 * docs/records.md): { rule, patterns: [{ pattern, label, balls, history,
 * record: null | { when_local, throws, catches, end_code, category, note, params } }] },
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
                   + (p.history > 1 ? ' · beaten ' + (p.history - 1) + '× before' : '')
                   + '\n' + paramsText(r.params))
                : 'No record yet',
            empty: !r,
        });
    }
    return rows;
}

/** Unit suffix of a ledger params key -> the unit shown after the value (JB-90). */
const PARAM_UNITS = [['_mm', 'mm'], ['_deg', '°'], ['_m', 'm'], ['_s', 's']];
/** Keys whose generic "strip the unit, capitalise" label would read badly. */
const PARAM_LABELS = { apex_m_ball2: ['Ball 2 apex', 'm'] };

/** One ledger params key -> [label, unit]: apex_m -> ['Apex', 'm'], planner -> ['Planner', '']. */
export function paramLabel(key) {
    if (Object.prototype.hasOwnProperty.call(PARAM_LABELS, key)) return PARAM_LABELS[key];
    let base = key, unit = '';
    for (const [suf, u] of PARAM_UNITS) {
        if (key.endsWith(suf) && key.length > suf.length) { base = key.slice(0, -suf.length); unit = u; break; }
    }
    const words = base.replace(/_/g, ' ');
    return [words.charAt(0).toUpperCase() + words.slice(1), unit];
}

/**
 * The hover line for a record's run parameters (JB-90): the ledger's `params`
 * ({key: value-string}, insertion order, records.py to_json) as
 * "Apex 0.95 m · Separation 125 mm · Dwell 0.27 s". A record set before the
 * ledger kept them (v1 rows, `{}` or absent) says so instead of showing nothing.
 */
export function paramsText(params) {
    const entries = (params && typeof params === 'object' && !Array.isArray(params))
        ? Object.entries(params) : [];
    if (entries.length === 0) return 'Parameters not recorded';
    return entries.map(([k, v]) => {
        const [label, unit] = paramLabel(k);
        return label + ' ' + String(v) + (unit ? (unit === '°' ? unit : ' ' + unit) : '');
    }).join(' · ');
}

/**
 * Whether the Records panel must sit right of the collapsed State Machine card
 * (JB-103). It sits hard left, under the connection indicator, and moves right
 * only when the card (bottom-anchored, growing up) reaches its vertical band.
 * Only vertical extents are compared: moving the panel sideways never changes
 * them, so the decision cannot oscillate. `gapPx` keeps the two from touching.
 */
export function dodgesMinimap(recordsRect, minimapRect, collapsed, gapPx) {
    if (!collapsed || !recordsRect || !minimapRect) return false;
    if (!(minimapRect.height > 0) || !(recordsRect.height > 0)) return false;
    return recordsRect.bottom + (Number(gapPx) || 0) > minimapRect.top;
}

/**
 * Rows grouped by ball count, fewest first (JB-93): [{ balls, title, rows }].
 * Ties keep ledger order; a ball count with no visible row yields no section.
 */
export function recordSections(rows) {
    const by = new Map();
    for (const r of rows || []) {
        if (!by.has(r.balls)) by.set(r.balls, []);
        by.get(r.balls).push(r);
    }
    return [...by.keys()].sort((a, b) => a - b).map(balls => ({
        balls,
        title: balls + (balls === 1 ? ' ball' : ' balls'),
        rows: by.get(balls),
    }));
}

/** The View-menu entries: every Pattern in the ledger, checked = shown. */
export function menuEntries(json, hidden) {
    const off = hidden || new Set();
    const pats = (json && Array.isArray(json.patterns)) ? json.patterns : [];
    return pats.filter(p => p && typeof p.pattern === 'string')
        .map(p => ({ pattern: p.pattern, label: String(p.label || p.pattern),
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
