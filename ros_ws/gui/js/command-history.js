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
    subscribeEvents, getRecentEvents, setHighlightedEvent, subscribeChartHover,
    EVENT_COLORS,
} from './event-store.js';

/** How many entries to render.  Older events stay in the event-store (for
 *  chart markers) but fall off the visible list. */
const VISIBLE_LIMIT = 50;

let listEl = null;
let emptyEl = null;

/** Event types the user has hidden from the list via legend-dot click.
 *  Does NOT affect chart markers — only filters the visible log. */
const hiddenTypes = new Set();

/** IDs whose chart marker is under the cursor — tinted in the list.  Kept
 *  here so a re-render (new event arriving mid-hover) re-applies the tint. */
let chartHoveredIds = [];

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
        row.style.setProperty('--row-color', EVENT_COLORS[ev.type] || '#94a3b8');

        const dot = document.createElement('span');
        dot.className = 'history-dot';
        dot.style.background = EVENT_COLORS[ev.type] || '#94a3b8';
        row.appendChild(dot);

        // Label sits right after the dot; the timestamp is right-aligned in
        // the last column (panels.css .history-entry).
        const label = document.createElement('span');
        label.className = 'history-label';
        label.textContent = ev.label;
        row.appendChild(label);

        const time = document.createElement('span');
        time.className = 'history-time';
        time.textContent = formatTime(ev.t);
        row.appendChild(time);

        // Hovering a row emphasises that event's marker on every chart; the
        // cost is one chart redraw per hover (coalesced via rAF upstream).
        row.addEventListener('mouseenter', () => setHighlightedEvent(ev.id));
        row.addEventListener('mouseleave', () => setHighlightedEvent(null));

        listEl.appendChild(row);
    }
    applyChartHoverTint(false);
}

/**
 * Tint the rows whose chart marker is hovered, and (when `scroll`) scroll the
 * list so the first of them is visible.  Scrolls only .history-body — never
 * the sidebar or page, which scrollIntoView would also move.  Rows that are
 * filtered out or older than the visible limit simply aren't found.
 */
function applyChartHoverTint(scroll) {
    if (!listEl) return;
    let first = null;
    for (const row of listEl.children) {
        const on = chartHoveredIds.includes(Number(row.dataset.eventId));
        row.classList.toggle('chart-hovered', on);
        if (on && !first) first = row;
    }
    const body = listEl.parentElement;
    const panel = document.getElementById('panel-history');
    if (!scroll || !first || !body || panel?.classList.contains('collapsed')) return;
    const r = first.getBoundingClientRect();
    const b = body.getBoundingClientRect();
    if (r.top < b.top) body.scrollTop += r.top - b.top;
    else if (r.bottom > b.bottom) body.scrollTop += r.bottom - b.bottom;
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

    subscribeChartHover((ids) => {
        chartHoveredIds = ids;
        applyChartHoverTint(true);
    });

    renderList();
}
