/**
 * console-view.js — the CONSOLE view of the topics panel (#panel-topics): a READ-ONLY
 * ROS log viewer over /rosout, live and in replay.  Nothing here publishes, calls a
 * service, or writes to ROS; the only side effects are localStorage prefs and, in replay,
 * engine.seek() on the trackbar's own engine.
 *
 * LIVE   main.js subscribes /rosout through the shared rosbridge connection at page load
 *        and feeds consoleOnLog(); lines go into a RING_CAP ring whether or not the view is
 *        showing (history is there when the operator switches to CONSOLE).  DOM work happens
 *        only while the view is visible.  Messages dispatched by a replay (/rosout is in the
 *        replay allowlist) are IGNORED here: the replay view has its own buffer.
 * REPLAY attachConsoleReplay(mode) follows the replay mode singleton.  On REPLAY it walks
 *        the recording slot by slot with source.load(i, {lite:true, topics:['/rosout']}) —
 *        the worker's low-priority filtered lane, the same one the ball history uses (no
 *        second MCAP reader) — into a bag-lifetime line list.  Click a line, or prev/next,
 *        calls engine.seek(t); the line nearest at-or-before the playhead is highlighted.
 *
 * Hovering a replay line publishes its time through mode.setMarker(t) (null on leave); the trackbar
 * subscribes to the mode's 'marker' event, so this file never touches the trackbar's DOM.
 *
 * Pure logic (filter, search, hit selection, escaping, ring cap): console-logic.js.
 */
import * as clock from './clock.js';
import {
    LEVELS, RING_CAP, normalizeLine, pushCapped, filterIndices, levelCounts,
    currentHit, prevHit, nextHit, lineHtml, parseEnabled, hoverTime,
} from './console-logic.js';

const FILTER_KEY = 'jugglebot-console-levels';
const RENDER_MS = 250;
const RENDER_WINDOW = 6000;
const REPLAY_TOPIC = '/rosout';

const live = [];                 // live ring
let replayLines = null;          // null = not in replay; else ascending-by-t lines of the bag
let replayLoading = '';          // '' | 'loading i/N'
let enabled = parseEnabled(null);
let query = '';
let visible = false;
let dirty = true;
let follow = true;               // auto-scroll while the view is at the bottom
let renderTimer = null;
let host = null;
let replayCtl = null;            // {mode, engine, unsubPlayhead, token}
let curIdx = -1;
let lastHits = [];
let lastShownLines = null;

const $ = (id) => document.getElementById(id);

export function initConsoleView() {
    host = $('console-container');
    if (!host) return;
    try { enabled = parseEnabled(localStorage.getItem(FILTER_KEY)); } catch (e) { /* default all on */ }
    query = '';   // the search is deliberately NOT persisted: a stale filter would hide log lines after a reload
    host.innerHTML = `
        <div class="console-toolbar">
            <span class="console-chips" id="console-chips" role="group" aria-label="Severity filter">${
                LEVELS.map((k) => `<button type="button" class="console-chip console-chip-${k.toLowerCase()}" data-lv="${k}" aria-pressed="true" title="Show or hide ${k} lines">${k}<span class="console-chip-n" id="console-n-${k}">0</span></button>`).join('')
            }</span>
        </div>
        <div class="console-toolbar">
            <input type="search" id="console-search" class="console-search" placeholder="Search node + message" autocomplete="off" spellcheck="false" aria-label="Search the console">
            <button type="button" class="console-nav" id="console-prev" title="Previous matching line (replay)" style="display:none">&#9650;</button>
            <button type="button" class="console-nav" id="console-next" title="Next matching line (replay)" style="display:none">&#9660;</button>
            <span class="console-count" id="console-count"></span>
        </div>
        <div class="console-list" id="console-list" role="log" aria-readonly="true"></div>
        <button type="button" class="console-jump" id="console-jump" style="display:none">Jump to latest</button>`;
    const search = $('console-search');
    search.value = query;
    search.addEventListener('input', () => {
        query = search.value;
        markDirty();
    });
    $('console-chips').querySelectorAll('button[data-lv]').forEach((b) => {
        b.addEventListener('click', () => {
            enabled[b.dataset.lv] = !enabled[b.dataset.lv];
            try { localStorage.setItem(FILTER_KEY, JSON.stringify(enabled)); } catch (e) { /* ignore */ }
            markDirty();
        });
    });
    const list = $('console-list');
    list.addEventListener('scroll', () => {
        if (replayLines) return;
        follow = list.scrollHeight - list.scrollTop - list.clientHeight < 24;
        $('console-jump').style.display = follow ? 'none' : '';
    });
    $('console-jump').addEventListener('click', () => { follow = true; markDirty(); });
    list.addEventListener('click', (ev) => {
        if (!replayLines) return;
        const row = ev.target && ev.target.closest ? ev.target.closest('.console-line') : null;
        if (row) seekTo(parseFloat(row.dataset.t));
    });
    // Replay only: hovering a row marks its time on the trackbar (mode.setMarker -> the trackbar's thin line).
    list.addEventListener('mouseover', (ev) => setMarker(ev.target));
    list.addEventListener('mouseout', (ev) => setMarker(ev.relatedTarget));   // onto another row: that row's time; off the rows: clears
    $('console-prev').addEventListener('click', () => step(prevHit));
    $('console-next').addEventListener('click', () => step(nextHit));
}

/** Show/hide (called by udp-traffic.js applyMode).  Rendering runs only while shown. */
export function setConsoleVisible(on) {
    visible = !!on;
    const panel = $('panel-topics');
    if (panel && panel.classList) panel.classList.toggle('console-on', visible);
    if (renderTimer) { clearInterval(renderTimer); renderTimer = null; }
    if (visible) {
        follow = true;
        renderTimer = setInterval(() => { if (dirty) render(); }, RENDER_MS);
        markDirty();
        render();
    }
}

/** Live /rosout callback (main.js).  Ignored during replay. */
export function consoleOnLog(msg) {
    if (clock.isReplay()) return;
    const line = normalizeLine(msg, clock.now() / 1000);
    if (!line) return;
    pushCapped(live, line, RING_CAP);
    if (!replayLines) markDirty();
}

function markDirty() { dirty = true; }

function activeLines() { return replayLines || live; }

function playhead() {
    const e = replayCtl && replayCtl.engine;
    const s = e ? e.state() : null;
    return s && Number.isFinite(s.playhead) ? s.playhead : NaN;
}

function render() {
    if (!host || !visible) return;
    dirty = false;
    const lines = activeLines();
    const hits = filterIndices(lines, enabled, query);
    lastHits = hits;
    lastShownLines = lines;
    const counts = levelCounts(lines, query);
    $('console-chips').querySelectorAll('button[data-lv]').forEach((b) => {
        b.classList.toggle('active', !!enabled[b.dataset.lv]);
        b.setAttribute('aria-pressed', String(!!enabled[b.dataset.lv]));
        $('console-n-' + b.dataset.lv).textContent = String(counts[b.dataset.lv]);
    });
    const inReplay = !!replayLines;
    $('console-prev').style.display = inReplay ? '' : 'none';
    $('console-next').style.display = inReplay ? '' : 'none';
    curIdx = inReplay ? currentHit(lines, hits, playhead()) : -1;
    let from = 0;
    if (hits.length > RENDER_WINDOW) {
        const pos = curIdx >= 0 ? hits.indexOf(curIdx) : hits.length - 1;
        from = Math.max(0, Math.min(hits.length - RENDER_WINDOW, pos - (RENDER_WINDOW >> 1)));
    }
    const shown = hits.slice(from, from + RENDER_WINDOW);
    const q = query;
    let html = '';
    for (const i of shown) html += lineHtml(lines[i], i, q, i === curIdx);
    const list = $('console-list');
    list.innerHTML = html || '<div class="topic-empty">' + (replayLoading || (lines.length ? 'No lines match' : 'No log lines yet')) + '</div>';
    $('console-count').textContent = (q ? hits.length + ' match' + (hits.length === 1 ? '' : 'es') + ' / ' : '') +
        lines.length + ' lines' + (replayLoading ? ' (' + replayLoading + ')' : '');
    if (inReplay) scrollCurrent(); else if (follow) list.scrollTop = list.scrollHeight;
}

function scrollCurrent() {
    const list = $('console-list');
    const row = list.querySelector('.console-current');
    if (row && row.scrollIntoView) row.scrollIntoView({ block: 'nearest' });
}

// ---- replay ----

function seekTo(t) {
    const e = replayCtl && replayCtl.engine;
    if (e && Number.isFinite(t)) e.seek(t);
}

function setMarker(target) {
    const m = replayCtl && replayCtl.mode;
    if (replayLines && m && m.setMarker) m.setMarker(hoverTime(target));
}

function step(pick) {
    if (!replayLines) return;
    const hits = filterIndices(replayLines, enabled, query);
    const i = pick(replayLines, hits, playhead());
    if (i >= 0) seekTo(replayLines[i].t);
}

function onPlayhead() {
    if (!visible || !replayLines) return;
    const cur = currentHit(replayLines, lastHits, playhead());
    if (cur === curIdx) return;
    const list = $('console-list');
    const old = list.querySelector('.console-current');
    if (old) old.classList.remove('console-current');
    const nu = list.querySelector('[data-k="' + cur + '"]');
    curIdx = cur;
    if (nu) { nu.classList.add('console-current'); if (nu.scrollIntoView) nu.scrollIntoView({ block: 'nearest' }); }
    else markDirty();
}

function detach() {
    if (replayCtl) {
        replayCtl.token++;
        if (replayCtl.mode && replayCtl.mode.setMarker) replayCtl.mode.setMarker(null);
        if (replayCtl.unsubPlayhead) replayCtl.unsubPlayhead();
        replayCtl = null;
    }
    replayLines = null;
    replayLoading = '';
    curIdx = -1;
    markDirty();
}

async function loadBag(ctl, source) {
    const r = source.range();
    const a = source.chunkIndex(r.t0), b = source.chunkIndex(r.t1);
    for (let i = a; i <= b; i++) {
        if (ctl !== replayCtl) return;
        replayLoading = 'loading ' + (i - a + 1) + '/' + (b - a + 1);
        markDirty();
        let ch = null;
        try { ch = await source.load(i, { lite: true, topics: [REPLAY_TOPIC] }); } catch (e) { ch = null; }
        if (ctl !== replayCtl) return;
        const tp = ch && ch.topics ? ch.topics[REPLAY_TOPIC] : null;
        if (tp) {
            for (let k = 0; k < tp.n; k++) {
                const l = normalizeLine(tp.hydrate(k), tp.t[k]);
                if (l) { l.t = tp.t[k]; replayLines.push(l); }   // log time: the trackbar's clock
            }
        }
    }
    replayLoading = '';
    markDirty();
}

/** Follow the replay mode singleton (main.js, after getReplayMode).  `mode` = createReplayMode(...) result. */
export function attachConsoleReplay(mode) {
    mode.on('state', (s) => {
        const inReplay = s && s.mode === 'REPLAY';
        if (inReplay && !replayCtl) {
            const engine = mode.engine(), source = mode.source();
            if (!engine || !source) return;
            const ctl = { mode, engine, token: 1, unsubPlayhead: null };
            replayCtl = ctl;
            replayLines = [];
            follow = false;
            ctl.unsubPlayhead = engine.on('playhead', onPlayhead);
            markDirty();
            loadBag(ctl, source);
        } else if (!inReplay && replayCtl) {
            detach();
        }
    });
}

/** Test/debug seam. */
export function _consoleState() { return { live: live.length, replay: replayLines ? replayLines.length : null }; }
