/**
 * hold-to-confirm.js — Convert a button into a "hold for N ms" confirm
 * affordance.  Used to gate destructive commands (Home / Clear Errors /
 * jog) so an accidental tap during bench work doesn't run the platform.
 *
 * The button shows a CSS-driven fill animation while held; releasing the
 * pointer (or dragging off) before the timer elapses cancels.  Wire this
 * INSTEAD of an `addEventListener('click', ...)` — the click event is
 * never used.
 */

const DEFAULT_HOLD_MS = 800;

/**
 * @param {HTMLButtonElement} btn
 * @param {() => void} onConfirm  Fires once the hold completes.
 * @param {number} [holdMs]       Default 800 ms.
 */
export function holdToConfirm(btn, onConfirm, holdMs = DEFAULT_HOLD_MS) {
    let timer = null;

    const start = (ev) => {
        if (ev && ev.button !== undefined && ev.button !== 0) return;
        if (btn.disabled) return;
        if (timer != null) return;
        btn.classList.add('hold-active');
        btn.style.setProperty('--hold-ms', holdMs + 'ms');
        timer = setTimeout(() => {
            timer = null;
            btn.classList.remove('hold-active');
            btn.classList.add('hold-confirmed');
            setTimeout(() => btn.classList.remove('hold-confirmed'), 350);
            try { onConfirm(); } catch { /* swallow */ }
        }, holdMs);
    };

    const cancel = () => {
        if (timer != null) {
            clearTimeout(timer);
            timer = null;
        }
        btn.classList.remove('hold-active');
    };

    btn.addEventListener('pointerdown', start);
    btn.addEventListener('pointerup', cancel);
    btn.addEventListener('pointerleave', cancel);
    btn.addEventListener('pointercancel', cancel);
}
