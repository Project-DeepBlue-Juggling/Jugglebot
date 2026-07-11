/**
 * theme.js — Dark/light theme toggle with localStorage persistence.
 *
 * The palette lives in css/theme.css; setting `data-theme="light"` on the
 * <html> root swaps the CSS custom properties.  JS-driven surfaces (uPlot
 * axes on canvas, the Three.js scene background) read their colours from
 * the CSS vars at rebuild time, so we also re-render them on toggle.
 *
 * Persistence: `jugglebot-theme` — `"dark"` or `"light"`.  No other prefs
 * from Phase 1 are persisted.
 */

import * as THREE from 'three';
import { scene } from './viewer.js';
import { rebuildCharts } from './telemetry-charts.js';
import { rebuildCanChart } from './can-traffic.js';

const STORAGE_KEY = 'jugglebot-theme';
const VALID_THEMES = new Set(['dark', 'light']);

/** Current theme name — seeded from the data-theme attribute the head-script
 *  sets before CSS paints, so we avoid a spurious chart rebuild on load. */
let currentTheme = document.documentElement.dataset.theme === 'light' ? 'light' : 'dark';

/**
 * Apply a theme to the document and downstream surfaces that don't
 * auto-pick-up CSS var changes.  Charts are only rebuilt when the palette
 * actually changes, so the initial load doesn't trigger a wasted rebuild.
 */
function applyTheme(theme) {
    if (!VALID_THEMES.has(theme)) theme = 'dark';
    const changed = theme !== currentTheme;
    currentTheme = theme;
    document.documentElement.dataset.theme = theme;

    // 3D scene background — read from the now-current CSS var.
    if (scene) {
        const hex = getComputedStyle(document.documentElement)
            .getPropertyValue('--viewer-bg').trim() || '#0f172a';
        scene.background = new THREE.Color(hex);
    }

    // uPlot renders axes on canvas, so a CSS var swap alone won't change
    // already-drawn ticks. Rebuild charts when the theme actually changed.
    if (changed) {
        try { rebuildCharts(); } catch { /* charts may not be built yet */ }
        try { rebuildCanChart(); } catch { /* chart may not be built yet */ }
    }

    // Update the toggle button's label to match the current state.
    const btn = document.getElementById('theme-toggle');
    if (btn) {
        btn.textContent = theme === 'dark' ? 'Light' : 'Dark';
        btn.title = `Switch to ${theme === 'dark' ? 'light' : 'dark'} theme`;
    }
}

/**
 * Initialise the theme system: read the saved preference (falling back to
 * dark) and wire the toggle button.  Call once after the DOM and scene are
 * available.
 */
export function initTheme() {
    const saved = localStorage.getItem(STORAGE_KEY);
    const theme = VALID_THEMES.has(saved) ? saved : 'dark';
    applyTheme(theme);

    const btn = document.getElementById('theme-toggle');
    if (!btn) return;
    btn.addEventListener('click', () => {
        const next = currentTheme === 'dark' ? 'light' : 'dark';
        localStorage.setItem(STORAGE_KEY, next);
        applyTheme(next);
    });
}
