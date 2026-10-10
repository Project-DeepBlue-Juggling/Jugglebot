/**
 * toast.js — the replay UI's one-line transient notice (the GUI had no toast mechanism).
 *
 *   const toaster = createToaster({document, timers}); toaster.show('Replay ended: rosbridge connected');
 *
 * Container `#replay-toasts` (created on demand under <body>); each toast removes itself after `ms`
 * (default 6000, UI wall-clock — deliberately not clock.js, it must tick during replay).
 */
import { h, ensure } from './dom.js';

/** @param {{document:Document, timers?:{set:function, clear:function}}} deps */
export function createToaster(deps) {
    const doc = deps.document;
    const timers = deps.timers || { set: (f, ms) => setTimeout(f, ms), clear: (t) => clearTimeout(t) };
    let box = null;
    const getBox = () => box || (box = ensure(doc, 'replay-toasts', doc.body, 'div', 'replay-toasts'));
    return {
        /** @param {string} text @param {number} [ms] @returns {Element} */
        show(text, ms) {
            const t = h(doc, 'div', { cls: 'replay-toast', text, attrs: { role: 'status' } });
            getBox().appendChild(t);
            timers.set(() => { if (t.remove) t.remove(); else getBox().removeChild(t); }, ms || 6000);
            return t;
        },
    };
}
