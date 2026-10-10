// replay_ui_absent_harness.js — the REAL replay/ui/absent.js under node with a fake document and fake modes/sources.
import { createAbsent, absentRegions, topicsOf, ABSENT_TABLE, ABSENT_TITLE } from './replay/ui/absent.js';

class El {
    constructor(sel) { this.sel = sel; this.title = ''; this._c = new Set();
        const s = this; this.classList = { add: (c) => s._c.add(c), remove: (c) => s._c.delete(c), contains: (c) => s._c.has(c) }; }
}
const els = {};
for (const r of ABSENT_TABLE) for (const sel of r.regions) els[sel] = new El(sel);
els['#panel-bb'].title = 'orig bb';
const doc = { querySelector: (sel) => els[sel] || null };

let active = false, source = null; const ls = [];
const mode = { isActive: () => active, source: () => source, on: (e, f) => { if (e === 'state') ls.push(f); return () => {}; } };
const emit = () => ls.forEach((f) => f());
const dimmed = () => Object.keys(els).filter((k) => els[k].classList.contains('replay-absent')).sort();

const ab = createAbsent({ document: doc, mode });
const out = {};

const man = (names) => { const t = {}; names.forEach((n) => { t[n] = { count: 5 }; }); return { topics: t }; };
let srcListeners = [];
const rec = (state, names) => ({ kind: 'recording', status: () => ({ state }), manifest: () => man(names),
    onChange: (cb) => { srcListeners.push(cb); return () => { srcListeners = srcListeners.filter((x) => x !== cb); }; } });

// 1. complete recording without bb/cone/motion topics
active = true; source = rec('complete', ['/robot_state', '/orchestrator_state', '/profile']);
emit(); out.complete = dimmed(); out.bbTitle = els['#panel-bb'].title; out.flagsTitle = els['#panel-flags'].title;
// 2. exit clears and restores titles
active = false; emit(); out.afterExit = dimmed(); out.bbTitleAfter = els['#panel-bb'].title;
// 3. a source that is not complete is not judged (guard; McapSource itself is complete from `opened`)
active = true; source = rec('converting', ['/robot_state']); emit(); out.converting = dimmed();
// 4. it completes -> source.onChange re-evaluates
source.status = () => ({ state: 'complete' }); srcListeners.forEach((f) => f()); out.completedLater = dimmed().length;
// 5. re-open a different source clears the old marks
source = rec('complete', ABSENT_TABLE.flatMap((r) => r.topics)); emit(); out.allPresent = dimmed();
// 6. session source with topicSet
source = { kind: 'session', topicSet: () => ['/robot_state', '/orchestrator_state'] }; emit(); out.session = dimmed();
// 7. zero-count topic counts as absent; pure helpers
out.zeroCount = Array.from(topicsOf({ kind: 'recording', status: () => ({ state: 'complete' }), manifest: () => ({ topics: { '/robot_state': { count: 0 }, '/balls': { count: 2 } } }) })).sort();
out.nullWhenUnknown = [topicsOf(null), topicsOf({ kind: 'session' })];
out.pureNull = absentRegions(null);
// 8. W4: any-present semantics on the multi-topic regions (tracking is fed by robot_state; BB/cone by their result topics)
source = rec('complete', ['/robot_state', '/bb/calibration_result', '/cone/timing_result']); emit(); out.w4_partial = dimmed();
source = rec('complete', ['/mocap_data', '/udp_diag']); emit(); out.w4_wrong_topics = dimmed();
out.title = ABSENT_TITLE;
console.log(JSON.stringify(out));
