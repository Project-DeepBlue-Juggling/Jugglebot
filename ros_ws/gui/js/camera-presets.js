/**
 * camera-presets.js — Scene menu camera shortcuts.
 *
 * Four built-in viewpoints (Top / Front / Side / Iso) plus three user
 * save-slots.  Slots persist across reload via localStorage — "saved" is
 * the whole point of the feature, so the general "only theme persists"
 * rule gets a narrow exception here.
 */

import { getCameraState, setCameraState } from './viewer.js';
import { INITIAL_HEIGHT_MM } from './geometry-config.js';

const SLOT_STORAGE_KEY = 'jugglebot-camera-slots';
const SLOT_COUNT = 3;
const MM_TO_M = 0.001;

/** Built-in presets.  Target at the platform mid-height so all views frame
 *  the machine consistently regardless of where the robot currently is. */
function builtinPresets() {
    const midY = INITIAL_HEIGHT_MM * 0.5 * MM_TO_M;
    return {
        top:   { position: { x: 0,     y: 2.5,  z: 0.001 }, target: { x: 0, y: midY, z: 0 } },
        front: { position: { x: 0,     y: midY, z: 2.2   }, target: { x: 0, y: midY, z: 0 } },
        side:  { position: { x: 2.2,   y: midY, z: 0     }, target: { x: 0, y: midY, z: 0 } },
        iso:   { position: { x: 1.2,   y: 1.0,  z: 1.2   }, target: { x: 0, y: midY, z: 0 } },
    };
}

function loadSlots() {
    try {
        const raw = localStorage.getItem(SLOT_STORAGE_KEY);
        if (!raw) return new Array(SLOT_COUNT).fill(null);
        const arr = JSON.parse(raw);
        if (!Array.isArray(arr)) return new Array(SLOT_COUNT).fill(null);
        const out = new Array(SLOT_COUNT).fill(null);
        for (let i = 0; i < SLOT_COUNT; i++) out[i] = arr[i] || null;
        return out;
    } catch {
        return new Array(SLOT_COUNT).fill(null);
    }
}

function saveSlots(slots) {
    try { localStorage.setItem(SLOT_STORAGE_KEY, JSON.stringify(slots)); }
    catch { /* quota / blocked — skip */ }
}

/** Inject the camera section into the scene-menu dropdown.  The base
 *  scene-group toggles are already populated by initSceneMenu() in
 *  main.js; we append our section below them. */
export function initCameraPresets() {
    const dropdown = document.getElementById('scene-menu-dropdown');
    if (!dropdown) return;

    const presets = builtinPresets();
    const slots = loadSlots();

    const section = document.createElement('div');
    section.className = 'scene-menu-section';

    const hdrView = document.createElement('div');
    hdrView.className = 'scene-menu-heading';
    hdrView.textContent = 'Camera';
    section.appendChild(hdrView);

    const presetRow = document.createElement('div');
    presetRow.className = 'scene-preset-row';
    for (const [key, state] of Object.entries(presets)) {
        const btn = document.createElement('button');
        btn.className = 'scene-preset';
        btn.textContent = key[0].toUpperCase() + key.slice(1);
        btn.title = `Snap camera to ${key} view`;
        btn.addEventListener('click', (ev) => {
            ev.stopPropagation();
            setCameraState(state);
        });
        presetRow.appendChild(btn);
    }
    section.appendChild(presetRow);

    const hdrSave = document.createElement('div');
    hdrSave.className = 'scene-menu-heading scene-menu-heading-sub';
    hdrSave.textContent = 'Saved (shift-click to save)';
    section.appendChild(hdrSave);

    const slotRow = document.createElement('div');
    slotRow.className = 'scene-preset-row';
    const slotButtons = [];
    for (let i = 0; i < SLOT_COUNT; i++) {
        const btn = document.createElement('button');
        btn.className = 'scene-preset scene-slot';
        slotButtons.push(btn);
        slotRow.appendChild(btn);

        btn.addEventListener('click', (ev) => {
            ev.stopPropagation();
            if (ev.shiftKey) {
                const s = getCameraState();
                if (!s) return;
                slots[i] = s;
                saveSlots(slots);
                refreshSlotButton(btn, i);
            } else if (slots[i]) {
                setCameraState(slots[i]);
            }
        });
    }
    section.appendChild(slotRow);

    dropdown.appendChild(section);

    for (let i = 0; i < SLOT_COUNT; i++) {
        refreshSlotButton(slotButtons[i], i);
    }

    function refreshSlotButton(btn, idx) {
        const saved = !!slots[idx];
        btn.classList.toggle('empty', !saved);
        btn.textContent = `Slot ${idx + 1}${saved ? ' \u2713' : ''}`;
        btn.title = saved
            ? `Recall slot ${idx + 1} · shift-click to overwrite`
            : `Empty slot — shift-click to save the current view here`;
    }
}
