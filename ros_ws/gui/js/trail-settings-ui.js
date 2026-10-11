/**
 * trail-settings-ui.js — the always-visible "Trail" length control at the bottom right of the 3D scene.
 * Built ONCE (main.js init); bound both ways to trail-settings. ONE element, moved between two parents:
 * outside replay it lives INSIDE #command-overlay (the live / lobby button bar), right-aligned on the buttons'
 * row; in replay (#command-overlay carries .replay-active: the bar is the replay dock) it sits in #viewer-pane,
 * bottom right, just above the dock (--trail-bottom mirrors the dock's height).
 *
 * Slider = [0,1] position via positionToMs/msToPosition (linear to 5 s, then logarithmic to the max).
 */
import {
    getEffectiveTailMs, getTailMaxMs, setTailMs, onTailChange, positionToMs, msToPosition, formatTail, formatMax,
} from './trail-settings.js';

const STEPS = 1000;     // slider resolution
let mounted = false;

export function initTrailSettingsUi() {
    if (mounted) return;
    const pane = document.getElementById('viewer-pane');
    if (!pane) return;
    mounted = true;

    const box = document.createElement('div');
    box.id = 'trail-overlay';
    box.className = 'scene-trail-overlay';
    box.title = 'Marker / ball trail length (0 = off). Capped at the recording length in replay.';

    const label = document.createElement('span');
    label.className = 'scene-trail-label';
    label.textContent = 'Trail';
    const slider = document.createElement('input');
    slider.type = 'range';
    slider.className = 'scene-trail-slider';
    slider.min = '0';
    slider.max = String(STEPS);
    slider.step = '1';
    slider.setAttribute('aria-label', 'Trail length');
    const out = document.createElement('span');
    out.className = 'scene-trail-readout';
    const maxLab = document.createElement('span');
    maxLab.className = 'scene-trail-max';
    box.append(label, slider, out, maxLab);
    const cmd = document.getElementById('command-overlay');

    // Stored request, but displayed clamped to the current max (a long request on a short recording shows
    // as the max; the stored value is untouched).
    const show = () => {
        const max = getTailMaxMs();
        const ms = getEffectiveTailMs();
        slider.value = String(Math.round(msToPosition(ms, max) * STEPS));
        out.textContent = formatTail(ms);
        maxLab.textContent = formatMax(max);
    };
    show();

    slider.addEventListener('input', () => {
        const max = getTailMaxMs();
        const pos = Number(slider.value) / STEPS;
        // the slider's right end means "the whole max", whatever the snap
        setTailMs(pos >= 1 ? max : positionToMs(pos, max));
    });
    // release returns keyboard focus to the page so the replay hotkeys (ignored while an input has focus) work again
    slider.addEventListener('pointerup', () => slider.blur());
    for (const ev of ['pointerdown', 'pointermove', 'pointerup', 'mousedown', 'click', 'dblclick', 'touchstart', 'wheel', 'contextmenu']) {
        box.addEventListener(ev, (e) => e.stopPropagation(), { passive: true });
    }
    onTailChange(show);

    // Placement: in the bar unless replay is active. The signal is the class lobby.js puts on the bar.
    let inBar = null;
    const place = () => {
        const wantBar = !!cmd && !cmd.classList.contains('replay-active');
        if (wantBar === inBar) return;
        inBar = wantBar;
        box.classList.toggle('in-bar', wantBar);
        if (cmd) cmd.classList.toggle('trail-in-bar', wantBar);
        if (wantBar) cmd.appendChild(box);
        else pane.appendChild(box);
    };
    place();
    if (cmd && typeof MutationObserver !== 'undefined') {
        new MutationObserver(place).observe(cmd, { attributes: true, attributeFilter: ['class'] });
    }
    // Replay placement only: the dock's height varies (trackbar + transport), so mirror it into --trail-bottom.
    if (cmd && typeof ResizeObserver !== 'undefined') {
        const sync = () => box.style.setProperty('--trail-bottom', cmd.offsetHeight + 'px');
        sync();
        new ResizeObserver(sync).observe(cmd);
    }
}
