/**
 * hide.js — while replay is active, HIDE the input panels that say nothing about a recording
 * (owner ask 2026-10-10): the Juggle panel, the Ball Butler target/aim inputs, the jog panel and the
 * speed-limits panel. A `replay-hidden` class (display:none !important) is added on entry and removed on
 * exit, so any inline display the live code set (e.g. the jog panel's own display:none toggle) is untouched
 * and comes back by itself. The Juggle panel also stops its rAF loop and renders (setJuggleHidden).
 * Fenced-but-hidden controls stay fenced (fence-dom.js is unchanged).
 *
 * createHide({document, mode, setJuggleHidden?}) -> {sync, hidden}
 */
export const HIDE_CLASS = 'replay-hidden';
export const HIDE_SELECTORS = Object.freeze(['#minimap-juggle', '#bb-throw-content', '#panel-jog', '#panel-speed-limits']);

export function createHide(deps) {
    const doc = deps.document;
    const mode = deps.mode;
    const setJuggleHidden = deps.setJuggleHidden || (() => {});
    let on = false;

    function apply(want) {
        for (const sel of HIDE_SELECTORS) {
            const el = doc.querySelector(sel);
            if (!el) continue;
            if (want) el.classList.add(HIDE_CLASS); else el.classList.remove(HIDE_CLASS);
        }
    }

    function sync() {
        const want = !!mode.isActive();
        // #minimap-juggle is built late by the minimap: re-apply while active so a late element is still hidden.
        if (want === on) { if (want) apply(true); return; }
        on = want;
        apply(on);
        setJuggleHidden(on);
    }

    const unsub = mode.on('state', sync);
    sync();
    return { sync, hidden: () => on, dispose: () => { if (typeof unsub === 'function') unsub(); } };
}
