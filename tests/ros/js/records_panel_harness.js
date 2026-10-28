// records_panel_harness.js — the REAL ros_ws/gui/js/records-model.js (JB-35 records panel, DOM-free half)
// under node. Sandbox (tests/ros/test_gui_records_panel.py builds it): records-model.js copied verbatim,
// this file, package.json {"type":"module"}. Prints one JSON object the test asserts on.
import {
    recordRows, menuEntries, loadHidden, saveHidden, attemptEnd, RECORDS_HIDDEN_KEY,
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
    ends: [
        attemptEnd(kv('juggle_end', { pattern: 'columns', record_beaten: '1', record_catches: '33' })),
        attemptEnd(kv('juggle_end', { pattern: 'hop', record_beaten: '0' })),
        attemptEnd(kv('juggle_end', { pattern: 'hop' })),
        attemptEnd(kv('juggle_start', { pattern: 'hop' })),
        attemptEnd(null),
    ],
}));
