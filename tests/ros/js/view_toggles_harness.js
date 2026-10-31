// view_toggles_harness.js — the REAL ros_ws/gui/js/view-toggles.js (JB-89) under node. Prints one JSON object.
import { loadViewToggles, saveViewToggles, toggleState, VIEW_TOGGLES_KEY } from './view-toggles.js';

class Store {
    constructor(init) { this.m = new Map(Object.entries(init || {})); }
    getItem(k) { return this.m.has(k) ? this.m.get(k) : null; }
    setItem(k, v) { this.m.set(k, String(v)); }
}
const throwing = { getItem() { throw new Error('blocked'); }, setItem() { throw new Error('blocked'); } };

const s = new Store();
saveViewToggles(s, { 'Jugglebot model': false, Ball: true });
let threw = false;
try { saveViewToggles(throwing, { a: true }); } catch (_e) { threw = true; }
const saved = loadViewToggles(s);

console.log(JSON.stringify({
    key: VIEW_TOGGLES_KEY,
    stored: s.getItem(VIEW_TOGGLES_KEY),
    roundTrip: saved,
    absent: loadViewToggles(new Store()),
    garbage: loadViewToggles(new Store({ [VIEW_TOGGLES_KEY]: '{nope' })),
    array: loadViewToggles(new Store({ [VIEW_TOGGLES_KEY]: '[true]' })),
    nonBool: loadViewToggles(new Store({ [VIEW_TOGGLES_KEY]: '{"a":1,"b":false}' })),
    nullStorage: loadViewToggles(null),
    throwingLoad: loadViewToggles(throwing),
    saveThrew: threw,
    savedOff: toggleState(saved, 'Jugglebot model', true),
    savedOn: toggleState(saved, 'Ball', false),
    defaultTrue: toggleState({}, 'x', true),
    defaultFalse: toggleState({}, 'x', false),
    protoName: toggleState({}, 'toString', false),
}));
