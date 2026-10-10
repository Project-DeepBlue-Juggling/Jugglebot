/**
 * fence-dom.js — the DOM face of the replay command fence (charting decision 10).
 *
 * Phase 2's fence.js refuses every dispatch; this makes the page say so: while replay is active every
 * command affordance in FENCE_SURFACES is `disabled` with the tooltip "unavailable in replay", and restored
 * on exit. Live code re-renders (updateCommandStates, juggle/minimap/BB panels) may re-enable a control on
 * the next recorded message, so a MutationObserver re-applies the fence to anything added or re-enabled.
 * The replay UI's own controls (`.replay-btn`, `#replay-dock`) are never touched.
 *
 * createFenceDom({document, mode, MutationObserver?}) -> {setReplayFenceDom, fenced}
 */

export const FENCE_TITLE = 'unavailable in replay';

/** The six surfaces of decision 10 -> CSS selectors of their controls. */
export const FENCE_SURFACES = Object.freeze({
    commandOverlay: ['#command-overlay .cmd-btn'],
    jogPanel: ['#panel-jog button', '#panel-jog input', '#panel-speed-limits input'],
    jugglePanel: ['#minimap-juggle button', '#minimap-juggle input', '#minimap-juggle select'],
    minimapSequencer: ['#minimap-action'],
    ballButler: ['#bb-calibrate-btn', '#bb-throw-btn', '#bb-aim-release', '#bb-throw-content input', '#bb-throw-content button'],
    holdToConfirm: ['.cmd-btn.hold-fillable'],   // NOT bare .hold-fillable: the chart signal-toggle pills carry it too
});

const ALL = [].concat(...Object.values(FENCE_SURFACES));
/** Observed subtrees (N1): the surface roots only, not document.body. */
export const FENCE_ROOTS = Object.freeze(['#command-overlay', '#panel-jog', '#panel-speed-limits', '#state-minimap', '#panel-bb']);

export function createFenceDom(deps) {
    const doc = deps.document;
    const MO = deps.MutationObserver || null;
    let active = false;
    let saved = new Map();        // el -> {disabled, title}
    let observer = null;
    let applying = false;

    function isOwn(el) {
        return !!(el.classList && el.classList.contains('replay-btn'));
    }

    function applyAll() {
        if (applying) return;
        applying = true;
        try {
            for (const [el] of saved) if (!el.isConnected) saved.delete(el);   // prune detached controls
            for (const sel of ALL) {
                for (const el of doc.querySelectorAll(sel)) {
                    if (isOwn(el)) continue;
                    if (!saved.has(el)) saved.set(el, { disabled: !!el.disabled, title: el.title || '' });
                    if (!el.disabled) el.disabled = true;
                    if (el.title !== FENCE_TITLE) el.title = FENCE_TITLE;
                }
            }
        } finally { applying = false; }
    }

    function restoreAll() {
        for (const [el, s] of saved) {
            el.disabled = s.disabled;
            el.title = s.title;
        }
        saved = new Map();
    }

    function setReplayFenceDom(on) {
        on = !!on;
        if (on === active) { if (on) applyAll(); return; }
        active = on;
        if (on) {
            applyAll();
            if (MO && doc.body && !observer) {
                observer = new MO(() => { if (active) applyAll(); });
                const opts = { childList: true, subtree: true, attributes: true, attributeFilter: ['disabled'] };
                for (const sel of FENCE_ROOTS) { const root = doc.querySelector(sel); if (root) observer.observe(root, opts); }
            }
        } else {
            if (observer) { observer.disconnect(); observer = null; }
            restoreAll();
        }
    }

    deps.mode.on('state', () => setReplayFenceDom(deps.mode.isActive()));
    return { setReplayFenceDom, fenced: () => active };
}
