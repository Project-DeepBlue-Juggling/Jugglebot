// replay_ui_fence_dom_harness.js — the REAL replay/ui/fence-dom.js with a fake document + fake MutationObserver.
import { createFenceDom, FENCE_SURFACES, FENCE_TITLE, FENCE_ROOTS } from './replay/ui/fence-dom.js';

class El { constructor(own) { this.isConnected = true; this.disabled = false; this.title = ''; this._own = !!own;
    const s = this; this.classList = { contains: (c) => c === 'replay-btn' && s._own }; } }
const bySel = {};
for (const [name, sels] of Object.entries(FENCE_SURFACES)) for (const sel of sels) bySel[sel] = [new El()];
bySel['#command-overlay .cmd-btn'][0].title = 'Home';
bySel['#bb-throw-btn'][0].disabled = true;           // already disabled before replay
const own = new El(true); bySel['#command-overlay .cmd-btn'].push(own);   // a replay-owned control
// W2: a chart signal-toggle pill carries .hold-fillable but is NOT a command affordance and is never fenced
const pill = new El(); pill.classList.contains = (c) => c === 'signal-toggle' || c === 'hold-fillable';
const pillMatches = new Set(['.signal-toggle.hold-fillable', '.hold-fillable']);
const doc = { body: {}, querySelector: (sel) => ({ sel }),
    querySelectorAll: (sel) => (bySel[sel] || []).concat(sel === '.hold-fillable' ? [pill] : []) };
let moCb = null, moObserved = 0, moDisc = 0;
const moTargets = [];
class MO { constructor(cb) { moCb = cb; } observe(t) { moObserved++; moTargets.push(t.sel); } disconnect() { moDisc++; } }

let active = false; const ls = [];
const mode = { isActive: () => active, on: (e, f) => { if (e === 'state') ls.push(f); return () => {}; } };
const emit = () => ls.forEach((f) => f());
const fd = createFenceDom({ document: doc, mode, MutationObserver: MO });

const all = () => Object.values(bySel).flat().filter((e) => !e._own);
const out = { surfaces: Object.keys(FENCE_SURFACES) };
out.beforeDisabled = all().filter((e) => e.disabled).length;
active = true; emit();
out.onAllDisabled = all().every((e) => e.disabled);
out.onAllTitled = all().every((e) => e.title === FENCE_TITLE);
out.ownUntouched = !own.disabled && own.title === '';
out.perSurface = {};
for (const [name, sels] of Object.entries(FENCE_SURFACES)) out.perSurface[name] = sels.every((s) => bySel[s].every((e) => e._own || e.disabled));
// live code re-enables a control mid-replay -> the observer re-fences it
bySel['#minimap-action'][0].disabled = false; moCb();
out.reFenced = bySel['#minimap-action'][0].disabled;
out.observed = moObserved;
out.observed_roots = moTargets;
out.expected_roots = FENCE_ROOTS.slice();
out.pill_enabled = !pill.disabled && pill.title === '';
active = false; emit();
out.homeTitle = bySel['#command-overlay .cmd-btn'][0].title;
out.afterEnabled = all().filter((e) => e.disabled).length;
out.throwStillDisabled = bySel['#bb-throw-btn'][0].disabled;
out.disconnected = moDisc;
out.fencedFlag = fd.fenced();
console.log(JSON.stringify(out));
