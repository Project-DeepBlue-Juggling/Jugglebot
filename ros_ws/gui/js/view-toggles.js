// view-toggles.js — the View menu's scene-group toggles remembered across a page refresh (JB-89).
// Pure and DOM-free (main.js passes localStorage), so tests/ros/js/view_toggles_harness.js runs it
// under node. One key, JSON {groupName: bool}; same failure policy as records-model.js
// loadHidden/saveHidden: unreadable or blocked storage means "no saved choice", never an error.

export const VIEW_TOGGLES_KEY = 'jugglebot-view-toggles';

/** The saved {name: bool} map; absent, garbage or wrong-shaped storage gives {}. */
export function loadViewToggles(storage) {
    try {
        const raw = storage && storage.getItem(VIEW_TOGGLES_KEY);
        const obj = raw ? JSON.parse(raw) : {};
        const out = {};
        if (obj && typeof obj === 'object' && !Array.isArray(obj)) {
            for (const [k, v] of Object.entries(obj)) if (typeof v === 'boolean') out[k] = v;
        }
        return out;
    } catch (_e) {
        return {};
    }
}

/** Persist the map; a blocked storage is not an error. */
export function saveViewToggles(storage, map) {
    try {
        if (storage) storage.setItem(VIEW_TOGGLES_KEY, JSON.stringify(map));
    } catch (_e) { /* private window / blocked site data */ }
}

/** The checked state for a group: the saved choice, else the scene's own default. */
export function toggleState(saved, name, sceneDefault) {
    return Object.prototype.hasOwnProperty.call(saved, name) ? saved[name] : !!sceneDefault;
}
