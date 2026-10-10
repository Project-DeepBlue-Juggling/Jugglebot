// replay_ui_hide_harness.js — the REAL replay/ui/hide.js with a fake document/mode.
import { createHide, HIDE_CLASS, HIDE_SELECTORS } from './replay/ui/hide.js';

class El { constructor(inline) { this.style = { display: inline || '' }; this._c = new Set();
    const s = this; this.classList = { add: (c) => s._c.add(c), remove: (c) => s._c.delete(c), contains: (c) => s._c.has(c) }; } }
const els = {};
for (const sel of HIDE_SELECTORS) els[sel] = new El();
els['#panel-jog'].style.display = 'none';          // live code's own inline hide
const keep = new El(); els['#panel-bb'] = keep;     // status part of BB must never be touched
const late = '#minimap-juggle'; const lateEl = els[late]; delete els[late];   // minimap builds it late
const doc = { querySelector: (sel) => els[sel] || null };

let active = false; const ls = [];
const mode = { isActive: () => active, on: (e, f) => { if (e === 'state') ls.push(f); return () => {}; } };
const emit = () => ls.forEach((f) => f());
const calls = [];
const h = createHide({ document: doc, mode, setJuggleHidden: (v) => calls.push(v) });
const hiddenSet = () => HIDE_SELECTORS.filter((s) => els[s] && els[s].classList.contains(HIDE_CLASS)).sort();
const out = { selectors: HIDE_SELECTORS.slice(), cls: HIDE_CLASS };
out.initial = hiddenSet(); out.initialCalls = calls.slice();
active = true; els[late] = lateEl; emit();
out.onHidden = hiddenSet(); out.onCalls = calls.slice();
out.bbKept = !keep.classList.contains(HIDE_CLASS);
out.jogInline = els['#panel-jog'].style.display;
emit(); out.repeatCalls = calls.slice();                       // idempotent: no second setJuggleHidden(true)
active = false; emit();
out.offHidden = hiddenSet(); out.offCalls = calls.slice();
out.jogInlineAfter = els['#panel-jog'].style.display;
out.hiddenFlag = h.hidden();
console.log(JSON.stringify(out));
