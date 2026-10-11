/**
 * trail-settings-ui.js — "Trail length" slider row in the Scene menu dropdown,
 * bound both ways to trail-settings. Appended by main.js next to
 * initCameraPresets() (the dropdown is rebuilt there, so the row is rebuilt
 * with it; the previous row's listener is dropped first).
 */

import { getTailMs, setTailMs, onTailChange, TAIL_MIN_MS, TAIL_MAX_MS, TAIL_STEP_MS } from './trail-settings.js';

let unsubscribe = null;

function readout(ms) { return ms <= 0 ? 'off' : (ms / 1000).toFixed(1) + ' s'; }

export function initTrailSettingsUi() {
    const dropdown = document.getElementById('scene-menu-dropdown');
    if (!dropdown) return;
    if (unsubscribe) { unsubscribe(); unsubscribe = null; }

    const section = document.createElement('div');
    section.className = 'scene-menu-section';

    const heading = document.createElement('div');
    heading.className = 'scene-menu-heading';
    heading.textContent = 'Trail length';
    section.appendChild(heading);

    const row = document.createElement('div');
    row.className = 'scene-trail-row';

    const slider = document.createElement('input');
    slider.type = 'range';
    slider.className = 'scene-trail-slider';
    slider.min = String(TAIL_MIN_MS);
    slider.max = String(TAIL_MAX_MS);
    slider.step = String(TAIL_STEP_MS);
    slider.title = 'Marker / ball trail length (0 = off)';
    const out = document.createElement('span');
    out.className = 'scene-trail-readout';

    const show = (ms) => { slider.value = String(ms); out.textContent = readout(ms); };
    show(getTailMs());

    slider.addEventListener('input', () => { setTailMs(Number(slider.value)); });
    slider.addEventListener('click', (ev) => ev.stopPropagation());
    unsubscribe = onTailChange(show);

    row.appendChild(slider);
    row.appendChild(out);
    section.appendChild(row);
    dropdown.appendChild(section);
}
