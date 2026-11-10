/**
 * records-panel.js — "Jugglebot records" overlay (IDEA-3): one line per
 * Pattern (label, catches/throws, date) under #connection-status.
 *
 * Source: GET /api/records (the repo's docs/records.md, which skill_node
 * rewrites when a record is beaten).  Fetched on init and again on every
 * skills/attempt juggle_end (state-minimap.js minimapOnSkillAttempt forwards
 * it here); a juggle_end with record_beaten '1' flashes that Pattern's row.
 * A row's hover (title) carries the record's run parameters (FEAT-29).
 * Placement (BUG-38): hard left under the connection indicator; it moves right
 * of the collapsed State Machine card only while the card reaches its band
 * (placePanel, re-run when the panel, the card or the pane resizes).
 * The View menu (#scene-menu-dropdown) gets one "<Pattern>" checkbox under a "Records" heading
 * per Pattern, remembered in localStorage.  Hidden in replay (css/replay.css
 * body.replay), and a replayed juggle_end is ignored (isReplayFenced).
 */
import { isReplayFenced } from './replay/fence.js';
import {
    recordRows, recordSections, dropsText, panelVisible, menuEntries, loadHidden, saveHidden, attemptEnd, RECORD_FLASH_MS,
    dodgesMinimap,
} from './records-model.js';

let data = null;
let hidden = new Set();
let flash = null;           // { pattern, until }
let menuSection = null;
let inFlight = false;

function storage() {
    try { return window.localStorage; } catch (_e) { return null; }
}

function render() {
    const el = document.getElementById('records-panel');
    if (!el) return;
    const rows = data ? recordRows(data, hidden) : [];
    el.hidden = !panelVisible(data, rows);
    const dropsEl = el.querySelector('.records-drops');
    if (dropsEl) dropsEl.textContent = data ? dropsText(data) : '';
    const list = el.querySelector('.records-rows');
    if (!list) return;
    list.textContent = '';
    const now = performance.now();  // wall-clock: UI flash timer; replayed juggle_ends are ignored
    for (const sec of recordSections(rows)) {
        const head = document.createElement('div');
        head.className = 'records-group';
        head.textContent = sec.title;
        list.appendChild(head);
        for (const r of sec.rows) list.appendChild(buildRow(r, now));
    }
    renderMenu();
    placePanel();
}

/** BUG-38: dodge the collapsed State Machine card only when they would overlap. */
function placePanel() {
    const el = document.getElementById('records-panel');
    const card = document.getElementById('state-minimap');
    if (!el) return;
    if (el.hidden || !card) { el.classList.remove('dodge-minimap'); return; }
    const gapPx = parseFloat(getComputedStyle(el).getPropertyValue('--gap')) || 8;
    el.classList.toggle('dodge-minimap', dodgesMinimap(
        el.getBoundingClientRect(), card.getBoundingClientRect(),
        card.classList.contains('collapsed'), gapPx));
}

function watchPlacement() {
    if (typeof ResizeObserver === 'undefined') return;
    const ro = new ResizeObserver(() => placePanel());
    // The card is bottom-anchored: a pane resize moves it without resizing it,
    // so the pane is observed too. Expand/collapse resizes the card itself.
    for (const id of ['records-panel', 'state-minimap', 'viewer-pane']) {
        const node = document.getElementById(id);
        if (node) ro.observe(node);
    }
}

function buildRow(r, now) {
    const row = document.createElement('div');
    row.className = 'records-row' + (r.empty ? ' empty' : '')
        + (flash && flash.pattern === r.pattern && now < flash.until ? ' beaten' : '');
    row.title = r.title;
    for (const [cls, text] of [['records-label', r.label], ['records-score', r.score],
                               ['records-date', r.date]]) {
        const sp = document.createElement('span');
        sp.className = cls;
        sp.textContent = text;
        row.appendChild(sp);
    }
    return row;
}

function renderMenu() {
    if (!menuSection || !data) return;
    menuSection.textContent = '';
    const head = document.createElement('div');
    head.className = 'scene-menu-heading';
    head.textContent = 'Records';
    menuSection.appendChild(head);
    for (const e of menuEntries(data, hidden)) {
        const label = document.createElement('label');
        label.className = 'scene-toggle';
        const cb = document.createElement('input');
        cb.type = 'checkbox';
        cb.checked = e.checked;
        cb.addEventListener('change', () => {
            if (cb.checked) hidden.delete(e.pattern); else hidden.add(e.pattern);
            saveHidden(storage(), hidden);
            render();
        });
        label.appendChild(cb);
        label.appendChild(document.createTextNode(e.label));
        menuSection.appendChild(label);
    }
}

export async function refreshRecords() {
    if (inFlight) return;
    inFlight = true;
    try {
        const resp = await fetch('/api/records', { cache: 'no-store' });
        if (resp.ok) data = await resp.json();
    } catch (_e) {
        /* GUI served without the endpoint: keep the last good view */
    } finally {
        inFlight = false;
    }
    render();
}

export function initRecordsPanel() {
    hidden = loadHidden(storage());
    watchPlacement();
    refreshRecords();
}

/** Called from main.js initSceneMenu after it (re)builds the dropdown. */
export function attachRecordsMenu(dropdown) {
    if (!dropdown) return;
    menuSection = document.createElement('div');
    menuSection.className = 'scene-menu-section records-menu';
    dropdown.appendChild(menuSection);
    renderMenu();
}

/** skills/attempt forward (state-minimap.js minimapOnSkillAttempt). */
export function recordsPanelOnSkillAttempt(msg) {
    const end = attemptEnd(msg);
    if (!end || isReplayFenced()) return;
    if (end.beaten) {
        flash = { pattern: end.pattern, until: performance.now() + RECORD_FLASH_MS };  // wall-clock: UI flash timer
        setTimeout(render, RECORD_FLASH_MS + 50);
    }
    refreshRecords();
}
