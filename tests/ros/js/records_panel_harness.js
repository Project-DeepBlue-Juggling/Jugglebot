// records_panel_harness.js — the REAL ros_ws/gui/js/records-model.js (IDEA-3 records panel, DOM-free half)
// under node. Sandbox (tests/ros/test_gui_records_panel.py builds it): records-model.js copied verbatim,
// this file, package.json {"type":"module"}. Prints one JSON object the test asserts on.
import {
    dropsText, dropsDetail, dropsBumped, panelVisible, recordRows, recordSections, menuEntries, loadHidden, saveHidden, attemptEnd, RECORDS_HIDDEN_KEY,
} from './records-model.js';

const json = {
    rule: 'r',
    patterns: [
        { pattern: 'hop', label: 'Hop', balls: 1, history: 0, record: null },
        { pattern: 'self_toss', label: 'Self-throw', balls: 1, history: 3,
          record: { when_local: '2026-10-21 10:05', throws: 14, catches: 12, end_code: 'MISSED_CATCH',
                    category: 'drop', note: 'warm box' } },
        { pattern: 'columns', label: 'Columns', balls: 2, history: 1,
          record: { when_local: '2026-10-22 18:40', throws: 33, catches: 32, end_code: 'STOPPED',
                    category: 'stopped', note: '' } },
    ],
};

// Ledger order deliberately NOT ball-ascending, a 3-ball group that is fully hidden.
const mixed = { patterns: [
    { pattern: 'c2', label: 'C2', balls: 2, history: 0, record: null },
    { pattern: 'a1', label: 'A1', balls: 1, history: 0, record: null },
    { pattern: 't3', label: 'T3', balls: 3, history: 0, record: null },
    { pattern: 'b2', label: 'B2', balls: 2, history: 0, record: null },
    { pattern: 'd1', label: 'D1', balls: 1, history: 0, record: null },
] };
const sec = (j, h) => recordSections(recordRows(j, h)).map(g => ({ balls: g.balls, title: g.title, rows: g.rows.map(r => r.pattern) }));

class Store {
    constructor(init) { this.m = new Map(Object.entries(init || {})); }
    getItem(k) { return this.m.has(k) ? this.m.get(k) : null; }
    setItem(k, v) { this.m.set(k, String(v)); }
}
const throwing = { getItem() { throw new Error('blocked'); }, setItem() { throw new Error('blocked'); } };

const s = new Store();
saveHidden(s, new Set(['self_toss', 'columns']));
const kv = (name, values) => ({ name, values: Object.entries(values).map(([key, value]) => ({ key, value })) });

console.log(JSON.stringify({
    sections: sec(mixed, new Set()),
    sectionsHidden: sec(mixed, new Set(['t3'])),
    sectionsNone: [recordSections(null), recordSections([])],
    all: recordRows(json, new Set()),
    filtered: recordRows(json, new Set(['hop', 'columns'])).map(r => r.pattern),
    garbage: [recordRows(null, null), recordRows({ patterns: 'x' }), recordRows({ patterns: [null, { pattern: 7 }] })],
    menu: menuEntries(json, new Set(['columns'])),
    stored: s.getItem(RECORDS_HIDDEN_KEY),
    reloaded: [...loadHidden(s)],
    badJson: [...loadHidden(new Store({ [RECORDS_HIDDEN_KEY]: '{not json' }))],
    notArray: [...loadHidden(new Store({ [RECORDS_HIDDEN_KEY]: '{"a":1}' }))],
    blocked: [...loadHidden(throwing)],
    blockedSave: (() => { saveHidden(throwing, new Set(['x'])); return 'ok'; })(),
    panelVisible: [
        panelVisible({ patterns: [], drops: { total: 0, live: 0, since: null, history: null } }, []),
        panelVisible({ patterns: mixed.patterns, drops: { total: 3 } }, recordRows(mixed, new Set(['c2', 'a1', 't3', 'b2', 'd1']))),
        panelVisible({ patterns: mixed.patterns }, mixed.patterns),
        panelVisible({ patterns: [] }, []),
        panelVisible(null, []),
    ],
    drops: [
        dropsText({ drops: { total: 12, live: 12, since: '2026-11-10', history: null } }),
        dropsText({ drops: { total: 112, live: 12, since: '2026-11-10', history: 100 } }),
        dropsText({ drops: { total: 840, live: 12, since: '2026-11-12', history: 828, history_from: '2026-07-23', history_through: '2026-11-11' } }),
        dropsText({ drops: null }), dropsText({}), dropsText(null),
    ],
    dropsDetail: [
        dropsDetail({ drops: { total: 12, live: 12, since: '2026-11-10', history: null } }),
        dropsDetail({ drops: { total: 112, live: 12, since: '2026-11-10', history: 100 } }),
        dropsDetail({ drops: { total: 840, live: 12, since: '2026-11-12', history: 828, history_from: '2026-07-23', history_through: '2026-11-11' } }),
        dropsDetail({ drops: null }),
    ],
    dropsBumped: [dropsBumped(null, 819), dropsBumped(819, 819), dropsBumped(819, 820), dropsBumped(820, 819), dropsBumped(819, null)],
    ends: [
        attemptEnd(kv('juggle_end', { pattern: 'columns', record_beaten: '1', record_catches: '33' })),
        attemptEnd(kv('juggle_end', { pattern: 'hop', record_beaten: '0' })),
        attemptEnd(kv('juggle_end', { pattern: 'hop' })),
        attemptEnd(kv('juggle_start', { pattern: 'hop' })),
        attemptEnd(null),
    ],
}));
