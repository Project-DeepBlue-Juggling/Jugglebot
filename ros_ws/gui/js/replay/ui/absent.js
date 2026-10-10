/**
 * absent.js — "not in this recording" (charting decision 18).
 *
 * An element whose source topic is not in the opened source shows a dimmed `replay-absent` state with the
 * tooltip "not in this recording" and keeps its '--' placeholder; never a frozen live value. Topic -> DOM
 * region lives in ABSENT_TABLE (keys are policy.js TOPIC_POLICY chunk names). A region is dimmed when NONE
 * of its topics is present. Nothing here touches the overview (decision 18: no per-topic rows there).
 *
 * The topic set is only judged when authoritative: a session snapshot (its own chunks) or an OPENED
 * recording (manifest.topics = the topic census the McapSource worker read from the MCAP summary on
 * `opened`; status() is always complete, so it is judged from the first frame; a later `source.onChange`
 * re-evaluates).
 *
 * createAbsent({document, mode}) -> {update, applied}; pure helpers exported for tests.
 */

export const ABSENT_CLASS = 'replay-absent';
export const ABSENT_TITLE = 'not in this recording';

/** {topics:[chunk names], regions:[CSS selectors]} — a region is absent iff none of its topics is present. */
export const ABSENT_TABLE = Object.freeze([
    { topics: ['/robot_state'], regions: ['#panel-flags', '#bus-voltage-value'] },
    { topics: ['/orchestrator_state'], regions: ['#state-badge', '#state-sub-mode'] },
    { topics: ['/bb/heartbeat', '/bb/calibration_result', '/bb/calibration_attempt'], regions: ['#panel-bb'] },
    { topics: ['/cone/heartbeat', '/cone/timing_result'], regions: ['#panel-catching-cone'] },
    { topics: ['/motion/diagnostics'], regions: ['#panel-motion'] },
    { topics: ['/robot_state', '/leg_setpoint_echo', '/hand_telemetry'], regions: ['#panel-tracking'] },
    { topics: ['/profile', '/link_status'], regions: ['#panel-can'] },
]);

/** @param {Set<string>|null} present @returns {string[]} selectors to dim (empty when `present` is null). */
export function absentRegions(present, table = ABSENT_TABLE) {
    const out = [];
    if (!present) return out;
    for (const row of table) {
        if (!row.topics.some((t) => present.has(t))) for (const r of row.regions) out.push(r);
    }
    return out;
}

/** Authoritative topic set of a source, or null while it cannot be judged. */
export function topicsOf(source) {
    if (!source) return null;
    if (source.kind === 'session') {
        return typeof source.topicSet === 'function' ? new Set(source.topicSet()) : null;
    }
    const st = source.status ? source.status() : null;
    const man = source.manifest ? source.manifest() : null;
    if (!st || st.state !== 'complete' || !man || !man.topics) return null;
    const out = new Set();
    for (const name of Object.keys(man.topics)) {
        const row = man.topics[name];
        if (!row || typeof row.count !== 'number' || row.count > 0) out.add(name);
    }
    return out;
}

export function createAbsent(deps) {
    const doc = deps.document;
    const mode = deps.mode;
    let marked = [];          // [{el, title}]
    let unsubSource = null;
    let watched = null;

    function clear() {
        for (const m of marked) {
            m.el.classList.remove(ABSENT_CLASS);
            m.el.title = m.title;
        }
        marked = [];
    }

    function evaluate() {
        clear();
        if (!mode.isActive()) return;
        const sels = absentRegions(topicsOf(mode.source()));
        for (const sel of sels) {
            const el = doc.querySelector(sel);
            if (!el) continue;
            marked.push({ el, title: el.title || '' });
            el.classList.add(ABSENT_CLASS);
            el.title = ABSENT_TITLE;
        }
    }

    function watch() {
        const src = mode.isActive() ? mode.source() : null;
        if (src === watched) return;
        if (unsubSource) { unsubSource(); unsubSource = null; }
        watched = src;
        if (src && typeof src.onChange === 'function') unsubSource = src.onChange(evaluate);
    }

    function update() { watch(); evaluate(); }

    mode.on('state', update);
    return { update, applied: () => marked.map((m) => m.el) };
}
