/**
 * command-history.js — Rolling log of notable events.
 *
 * Presents the event-store feed as a compact scrollable list pinned to the
 * bottom of the right sidebar.  Each entry shows a coloured type dot, the
 * timestamp, and a terse label; the full `detail` string is in the title
 * attribute for hover.  The list is not persisted across reload — matches
 * the Phase-2 design decision.
 */

import {
    subscribeEvents, getRecentEvents, setHighlightedEvent, EVENT_COLORS,
} from './event-store.js';

/** How many entries to render.  Older events stay in the event-store (for
 *  chart markers) but fall off the visible list. */
const VISIBLE_LIMIT = 50;

let listEl = null;
let emptyEl = null;

/** Event types the user has hidden from the list via legend-dot click.
 *  Does NOT affect chart markers — only filters the visible log. */
const hiddenTypes = new Set();

/** Format a seconds-since-epoch timestamp as HH:MM:SS.mmm (local). */
function formatTime(t) {
    const d = new Date(t * 1000);
    const hh = String(d.getHours()).padStart(2, '0');
    const mm = String(d.getMinutes()).padStart(2, '0');
    const ss = String(d.getSeconds()).padStart(2, '0');
    const ms = String(d.getMilliseconds()).padStart(3, '0');
    return `${hh}:${mm}:${ss}.${ms}`;
}

/** Re-render the visible list from the event-store.  Cheap enough at N=50
 *  that we don't bother with per-entry DOM diffing.  Hidden types filtered
 *  *after* fetching so the cap still gives us ~VISIBLE_LIMIT visible rows
 *  even when several types are hidden. */
function renderList() {
    if (!listEl) return;
    // Pull more than VISIBLE_LIMIT when filtering is active so we don't end
    // up with an empty list just because the most recent 50 are all hidden.
    const pullLimit = hiddenTypes.size > 0 ? 500 : VISIBLE_LIMIT;
    const all = getRecentEvents(null, pullLimit);
    const events = [];
    for (const ev of all) {
        if (hiddenTypes.has(ev.type)) continue;
        events.push(ev);
        if (events.length >= VISIBLE_LIMIT) break;
    }
    listEl.innerHTML = '';
    if (events.length === 0) {
        if (emptyEl) emptyEl.style.display = '';
        return;
    }
    if (emptyEl) emptyEl.style.display = 'none';
    for (const ev of events) {
        const row = document.createElement('div');
        row.className = 'history-entry';
        row.title = ev.detail || ev.label;
        row.dataset.eventId = String(ev.id);

        const dot = document.createElement('span');
        dot.className = 'history-dot';
        dot.style.background = EVENT_COLORS[ev.type] || '#94a3b8';
        row.appendChild(dot);

        const time = document.createElement('span');
        time.className = 'history-time';
        time.textContent = formatTime(ev.t);
        row.appendChild(time);

        const label = document.createElement('span');
        label.className = 'history-label';
        label.textContent = ev.label;
        row.appendChild(label);

        // Hovering a row emphasises that event's marker on every chart; the
        // cost is one chart redraw per hover (coalesced via rAF upstream).
        row.addEventListener('mouseenter', () => setHighlightedEvent(ev.id));
        row.addEventListener('mouseleave', () => setHighlightedEvent(null));

        listEl.appendChild(row);
    }
}

/**
 * Initialise the command history panel.  The panel's DOM shell lives in
 * index.html; this wires up the collapse toggle + event subscription.
 */
export function initCommandHistory() {
    const panel = document.getElementById('panel-history');
    if (!panel) return;
    listEl = panel.querySelector('#history-list');
    emptyEl = panel.querySelector('#history-empty');

    const header = panel.querySelector('.panel-header');
    header?.addEventListener('click', () => {
        panel.classList.toggle('collapsed');
    });

    // Legend-dot filters — each click toggles that event type in/out of
    // the list.  stopPropagation so the click doesn't also collapse the
    // panel via the header handler above.
    for (const item of panel.querySelectorAll('.history-legend-item')) {
        const type = item.dataset.type;
        if (!type) continue;
        item.addEventListener('click', (ev) => {
            ev.stopPropagation();
            if (hiddenTypes.has(type)) {
                hiddenTypes.delete(type);
                item.classList.remove('filtered-out');
            } else {
                hiddenTypes.add(type);
                item.classList.add('filtered-out');
            }
            renderList();
        });
    }

    // Coalesce repaint: if many events fire in one tick we render once.
    let pending = false;
    subscribeEvents(() => {
        if (pending) return;
        pending = true;
        requestAnimationFrame(() => {
            pending = false;
            renderList();
        });
    });

    renderList();
}
