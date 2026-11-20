/**
 * imu-charts.js — the Platform IMU chart group in the bottom chart panel: two uPlot charts (linear
 * acceleration, angular rate / angular acceleration) of one IMU capture, on the SAME x axis as the
 * ODrive charts (telemetry-charts.js).
 *
 * The logic (binary parse, gaps, derivative, decimation, selection / following, the "do not yank" rule)
 * is in imu-charts-core.js and node-tested by tests/ros/test_gui_imu_charts.py; this file is the glue:
 * fetches, DOM, uPlot.
 *
 * Time linking. telemetry-charts.js broadcasts every x range its charts take (onSharedXRange) and these
 * charts draw on it; their wheel / middle-drag / double-click go through the shared handlers
 * (attachZoomPanControls) and their drag box through selectRangeOnAllCharts, so all eleven charts move
 * together. LIVE: choosing a capture pauses the ODrive charts and shows its span (showRangeOnAllCharts);
 * an automatic follow does that only when the operator has not moved the view (core.autoJumpDecision),
 * otherwise a "Jump to capture" button appears. REPLAY: the playhead owns the window; the selection
 * follows the playhead and the button seeks the playhead to the capture's start (seekReplayTo).
 *
 * Nothing is fetched while the group is hidden (the default).
 */

import * as clock from './clock.js';
import { TIME_INCRS, timeTickSpace, numTickSpace, timeTickLabels } from './axis-ticks.js';
import { layoutCallouts } from './callout-layout.js';
import {
    SYNC_KEY, nanGaps, setCsvExtraProvider, onSharedXRange, getSharedXRange, getSharedViewState,
    showRangeOnAllCharts, selectRangeOnAllCharts, seekReplayTo, attachZoomPanControls, drawReplayPlayhead,
    drawEventMarkers, attachEventMarkerHover, releaseEventMarkerHover, onEventMarkerVisibility,
} from './telemetry-charts.js';
import { subscribeEvents, subscribeHighlight } from './event-store.js';
import * as core from './imu-charts-core.js';

const API = '/api/imu/captures';
const RECORDINGS_API = '/api/replay/recordings';
const LAUNCH_REFRESH_MS = 60000;
const DATA_CACHE_MAX = 4;

// ---- State ----

let shown = false;
let enabled = new Set(core.DEFAULT_SERIES);
let gyroAccel = false;                       // right chart shows angular acceleration
let filterCutoff = 0;                        // low-pass Hz for both time charts, 0 = off
let spectrum = false;                        // both charts show amplitude spectra of the visible range
let specMax = core.SPEC_MAX_DEFAULT;         // spectrum x max in Hz, 0 = Nyquist
let specState = null;                        // {error} | {res, kind data...} of the last spectrum computation
let specTimer = null;
let wantedImuHeight = core.IMU_HEIGHT_DEFAULT; // the operator's IMU share (px); the applied one is clamped to the panel
let mode = 'live';                           // 'live' | 'replay'
let replayRange = null;                      // {t0, t1} while replaying
let xRange = { min: 0, max: 1 };
let captures = [];
let listState = 'idle';                      // 'idle' | 'loading' | 'ok' | 'error'
let listStatus = 0;
let launch = null;                           // {from, fallback, at}
let sel = { selected: null, follow: true, hold: null };
let lastApplied = null;
let capData = null;                          // {stem, cols?, dt?, ang?, error?, loading?}
const dataCache = new Map();
let listToken = 0;
let dataToken = 0;
let lastStatus = null;
let lastStatusAt = 0;
let pollTimer = null;
let redrawPending = false;

const charts = { accel: null, gyro: null };
const callouts = { accel: null, gyro: null };
const els = {};

// ---- Settings ----

function loadSettings() {
    try { shown = localStorage.getItem(core.IMU_SHOWN_KEY) === 'true'; } catch { shown = false; }
    try {
        const s = JSON.parse(localStorage.getItem(core.IMU_SERIES_KEY) || 'null');
        if (Array.isArray(s)) enabled = new Set(s.filter((k) => core.DEFAULT_SERIES.includes(k)));
    } catch { /* keep default */ }
    try { gyroAccel = localStorage.getItem(core.IMU_GYRO_MODE_KEY) === 'accel'; } catch { /* keep */ }
    try { filterCutoff = core.parseCutoff(localStorage.getItem(core.IMU_FILTER_KEY)); } catch { /* keep */ }
    try { spectrum = localStorage.getItem(core.IMU_SPECTRUM_KEY) === 'true'; } catch { /* keep */ }
    try { specMax = core.parseSpecMax(localStorage.getItem(core.IMU_SPEC_MAX_KEY) || core.SPEC_MAX_DEFAULT); } catch { /* keep */ }
}

function save(key, value) {
    try { localStorage.setItem(key, value); } catch { /* private window: per-viewer convenience only */ }
}

// ---- Init ----

export function initImuCharts() {
    for (const id of ['imu-charts-toggle-btn', 'imu-capture-select', 'imu-jump-btn', 'imu-chart-group',
        'imu-accel-plot', 'imu-gyro-plot', 'imu-accel-toggles', 'imu-gyro-toggles', 'imu-gyro-mode-btn',
        'imu-gyro-title', 'imu-chart-message', 'imu-filter-select', 'imu-spectrum-btn', 'imu-spec-max-select',
        'imu-accel-title', 'imu-accel-badge', 'imu-gyro-badge', 'imu-accel-peaks', 'imu-gyro-peaks']) {
        els[id] = document.getElementById(id);
    }
    if (!els['imu-chart-group'] || !els['imu-charts-toggle-btn']) return;
    loadSettings();
    xRange = getSharedXRange();
    onSharedXRange(onRange);

    els['imu-charts-toggle-btn'].addEventListener('click', () => setShown(!shown));
    els['imu-capture-select'].addEventListener('change', () => {
        const stem = els['imu-capture-select'].value;
        els['imu-capture-select'].blur();     // give R / Space / [ ] back to the chart shortcuts
        handPick(stem);
    });
    els['imu-jump-btn'].addEventListener('click', () => jumpToCapture());
    buildPills('imu-accel-toggles', core.ACCEL_SERIES, 'accel');
    buildPills('imu-gyro-toggles', core.GYRO_SERIES, 'gyro');
    const gm = els['imu-gyro-mode-btn'];
    if (gm) {
        gm.addEventListener('click', () => {
            gyroAccel = !gyroAccel;
            save(core.IMU_GYRO_MODE_KEY, gyroAccel ? 'accel' : 'rate');
            applyGyroModeUI();
            rebuildChart('gyro');
        });
    }
    applyGyroModeUI();
    const fsel = els['imu-filter-select'];
    if (fsel) {
        fsel.value = String(filterCutoff);
        fsel.addEventListener('change', () => {
            filterCutoff = core.parseCutoff(fsel.value);
            fsel.blur();
            save(core.IMU_FILTER_KEY, String(filterCutoff));
            applyHeadings();
            scheduleRedraw();
        });
    }
    const sbtn = els['imu-spectrum-btn'];
    if (sbtn) sbtn.addEventListener('click', () => setSpectrum(!spectrum));
    const msel = els['imu-spec-max-select'];
    if (msel) {
        msel.value = String(specMax);
        msel.addEventListener('change', () => {
            specMax = core.parseSpecMax(msel.value);
            msel.blur();
            save(core.IMU_SPEC_MAX_KEY, String(specMax));
            scheduleRedraw();
        });
    }
    // The event markers on the time charts follow the event store and the toolbar's hide toggle.
    const repaintMarkers = () => { if (!spectrum) for (const k of ['accel', 'gyro']) if (charts[k]) charts[k].redraw(false, false); };
    subscribeEvents(repaintMarkers);
    subscribeHighlight(repaintMarkers);
    onEventMarkerVisibility(repaintMarkers);

    for (const kind of ['accel', 'gyro']) {
        const el = els['imu-' + kind + '-plot'];
        if (el && typeof ResizeObserver !== 'undefined') {
            new ResizeObserver(() => {
                if (!shown) return;
                if (!charts[kind]) { rebuildChart(kind); return; }
                const w = el.clientWidth, h = el.clientHeight;
                if (w >= 10 && h >= 10) { charts[kind].setSize({ width: w, height: h }); scheduleRedraw(); }
            }).observe(el);
        }
    }
    // uPlot paints axes on canvas: follow a theme switch (theme.js sets data-theme) by rebuilding.
    if (typeof MutationObserver !== 'undefined' && document.documentElement) {
        new MutationObserver(() => {
            if (shown) { rebuildChart('accel'); rebuildChart('gyro'); }
        }).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
    }
    initHeightDivider();
    setCsvExtraProvider(csvExtra);
    setShown(shown, true);
}

function setShown(on, initial = false) {
    shown = on;
    if (!initial) save(core.IMU_SHOWN_KEY, on ? 'true' : 'false');
    els['imu-chart-group'].hidden = !on;
    // The IMU row adds its own height to the panel (charts.css) so the motor grid keeps its size.
    const panelEl = document.getElementById('chart-panel');
    if (panelEl) panelEl.classList.toggle('imu-shown', on);
    applyImuHeight();
    const btn = els['imu-charts-toggle-btn'];
    btn.classList.toggle('active', on);
    btn.textContent = on ? 'Hide IMU charts' : 'IMU charts';
    els['imu-capture-select'].hidden = !on;
    applyControlsUI();
    if (on) {
        xRange = getSharedXRange();
        requestAnimationFrame(() => { rebuildChart('accel'); rebuildChart('gyro'); });
        refreshList('show');
        startPoll();
    } else {
        stopPoll();
        listToken++; dataToken++;              // drop answers to requests made while shown
        for (const k of ['accel', 'gyro']) destroyChart(k);
        els['imu-jump-btn'].hidden = true;
    }
}

// ---- CSV export (telemetry-charts.js "Export CSV"): the shown capture's raw samples as extra columns ----

/** null when the group is hidden or no capture is loaded (then the export is exactly the ODrive one). */
function csvExtra() {
    if (!shown || !capData || !capData.cols) return null;
    const c = capData.cols;
    const f = (v) => v.toFixed(6);
    return {
        header: core.IMU_CSV_HEADER,
        merge: (ts, n) => core.mergeCsvRows(ts, n, c),
        values: (j) => [f(c.ax[j]), f(c.ay[j]), f(c.az[j]), f(c.gx[j]), f(c.gy[j]), f(c.gz[j])],
    };
}

// ---- IMU/ODrive split: the divider between the two groups (the panel's total height is the top handle's) ----

function appEl() { return document.getElementById('app'); }

function bothShown() {
    const panel = document.getElementById('chart-panel');
    return !!(shown && panel && !panel.classList.contains('charts-hidden'));
}

/** px the two groups share: the panel's inner height minus the toolbar and the divider. */
function availSpace() {
    const panel = document.getElementById('chart-panel');
    const toolbar = document.getElementById('chart-toolbar');
    const divider = document.getElementById('imu-chart-divider');
    if (!panel || !panel.clientHeight) return NaN;
    return panel.clientHeight - (toolbar ? toolbar.offsetHeight : 0) - (divider ? divider.offsetHeight : 0);
}

function currentImuHeight() {
    const g = els['imu-chart-group'];
    const h = g && g.getBoundingClientRect ? g.getBoundingClientRect().height : 0;
    return h > 0 ? h : core.IMU_HEIGHT_DEFAULT;
}

/** Set --imu-chart-height: the wanted share, clamped so both groups keep their minimum (wanted is never overwritten). */
function applyImuHeight() {
    const app = appEl();
    if (!app) return null;
    const eff = bothShown() ? core.clampImuShare(wantedImuHeight, availSpace()) : core.clampImuHeight(wantedImuHeight);
    app.style.setProperty('--imu-chart-height', eff + 'px');
    return eff;
}

/** main.js calls this when the top handle changed the panel's total height. */
export function imuChartsRelayout() { applyImuHeight(); }

function initHeightDivider() {
    const divider = document.getElementById('imu-chart-divider');
    let saved = null;
    try { saved = localStorage.getItem(core.IMU_HEIGHT_KEY); } catch { /* storage blocked */ }
    wantedImuHeight = core.clampImuHeight(saved);
    applyImuHeight();
    window.addEventListener('resize', applyImuHeight);
    // The panel's size changes with the top handle, the window, and the grid / group toggles: re-clamp the share.
    const panel = document.getElementById('chart-panel');
    if (panel && typeof ResizeObserver !== 'undefined') new ResizeObserver(applyImuHeight).observe(panel);
    if (!divider) return;
    let startY = 0, startH = 0, dragging = false;
    divider.addEventListener('pointerdown', (e) => {
        dragging = true;
        startY = e.clientY;
        startH = currentImuHeight();
        divider.classList.add('active');
        if (divider.setPointerCapture) divider.setPointerCapture(e.pointerId);
        document.body.style.cursor = 'row-resize';
        document.body.style.userSelect = 'none';
        e.preventDefault();
    });
    // The panel's top edge and total height are fixed: the divider follows the cursor, down = taller IMU row.
    document.addEventListener('pointermove', (e) => {
        if (!dragging) return;
        wantedImuHeight = core.clampImuHeight(startH + (e.clientY - startY));
        const eff = applyImuHeight();
        if (eff !== null && bothShown()) wantedImuHeight = eff;   // the divider cannot go past a limit and "wind up"
    });
    document.addEventListener('pointerup', () => {
        if (!dragging) return;
        dragging = false;
        divider.classList.remove('active');
        document.body.style.cursor = '';
        document.body.style.userSelect = '';
        save(core.IMU_HEIGHT_KEY, wantedImuHeight);
    });
    divider.addEventListener('dblclick', () => {
        wantedImuHeight = core.evenSplit(availSpace());
        applyImuHeight();
        save(core.IMU_HEIGHT_KEY, wantedImuHeight);
    });
}

// ---- Replay / status hooks (main.js) ----

/** Wire the replay mode singleton: REPLAY lists the recording's captures, LIVE the launch's. */
export function attachImuChartsReplay(replayMode) {
    if (!replayMode || typeof replayMode.on !== 'function') return;
    replayMode.on('state', onReplayState);
    try { onReplayState(replayMode.state()); } catch { /* not started */ }
}

function onReplayState(s) {
    if (!s) return;
    if (s.mode === 'REPLAY' && s.range && Number.isFinite(s.range.t0) && Number.isFinite(s.range.t1)) {
        const same = mode === 'replay' && replayRange && replayRange.t0 === s.range.t0 && replayRange.t1 === s.range.t1;
        if (same) return;
        mode = 'replay';
        replayRange = { t0: s.range.t0, t1: s.range.t1 };
        resetSelection();
        if (shown) refreshList('mode');
    } else if (mode === 'replay' && s.mode !== 'REPLAY' && s.mode !== 'OPENING') {
        mode = 'live';
        replayRange = null;
        resetSelection();
        xRange = getSharedXRange();
        if (shown) refreshList('mode');
    }
}

function resetSelection() {
    sel = { selected: null, follow: mode === 'live', hold: null };
    lastApplied = null;
    captures = [];
    capData = null;
    dataToken++;   // drop a capture response still in flight from the previous mode
    listState = 'idle';
    renderSelect();
    scheduleRedraw();
}

/** platform_imu/status (main.js): a finished upload refreshes the list (LIVE, group shown). */
export function imuChartsOnStatus(msg) {
    if (clock.isReplay()) return;
    const d = core.statusDigest(msg);
    const moved = core.statusMeansNewCapture(lastStatus, d);
    lastStatus = d;
    lastStatusAt = Date.now(); // wall-clock: live-only path (replay returns above)
    if (moved && shown && mode === 'live') {
        refreshList('status');
        // the sidecar may land a moment after the status flips
        setTimeout(() => { if (shown && mode === 'live') refreshList('status'); }, 1500);
    }
}

function startPoll() {
    if (pollTimer !== null) return;
    pollTimer = setInterval(() => {
        if (!shown || mode !== 'live') return;
        if (Date.now() - lastStatusAt < core.LIST_POLL_MS) return; // wall-clock: status freshness (live)
        refreshList('poll');
    }, core.LIST_POLL_MS);
}

function stopPoll() {
    if (pollTimer !== null) clearInterval(pollTimer);
    pollTimer = null;
}

// ---- Capture list ----

async function liveWindow() {
    const now = Date.now(); // wall-clock: live launch window
    if (launch && now - launch.at < LAUNCH_REFRESH_MS) return launch;
    let listing = null;
    try {
        const r = await fetch(RECORDINGS_API, { cache: 'no-store' });
        if (r.ok) listing = await r.json();
    } catch { /* fall back below */ }
    const w = core.liveLaunchWindow(listing, now / 1000);
    launch = { from: w.from, fallback: w.fallback, at: now };
    return launch;
}

async function refreshList(reason) {
    const token = ++listToken;
    if (listState !== 'ok') { listState = 'loading'; renderMessage(); }
    let url;
    if (mode === 'replay' && replayRange) {
        url = API + '?from=' + replayRange.t0 + '&to=' + replayRange.t1;
    } else {
        const w = await liveWindow();
        if (token !== listToken) return;
        url = API + '?from=' + w.from;
    }
    let body = null;
    let status = 0;
    try {
        const r = await fetch(url, { cache: 'no-store' });
        status = r.status;
        if (r.ok) body = await r.json();
    } catch { status = 0; }
    if (token !== listToken) return;
    if (!body) {
        listState = 'error'; listStatus = status; captures = [];
        renderSelect(); renderMessage(); scheduleRedraw();
        return;
    }
    listState = 'ok';
    captures = core.normaliseCaptures(body);
    if (mode === 'replay') {
        const r = core.replayFollow(sel, captures, playhead());
        sel = { selected: r.selected, follow: false, hold: r.hold };
        renderSelect();
        if (r.changed || !capData || capData.stem !== sel.selected) loadSelected();
    } else {
        const r = core.liveAfterRefresh(sel, captures);
        sel = { selected: r.selected, follow: r.follow, hold: null };
        renderSelect();
        if (r.changed || reason === 'show' || reason === 'mode') {
            loadSelected();
            if (sel.selected) autoApply();
        } else if (capData && capData.error && capData.error !== 'nowall' && capData.stem === sel.selected) {
            // A failed load (server blip) is retried at the next refresh instead of sticking.
            loadSelected();
        }
    }
    renderMessage();
    renderJump();
}

function selectedCapture() {
    return captures.find((c) => c.stem === sel.selected) || null;
}

function playhead() {
    return (xRange.min + xRange.max) / 2;
}

function renderSelect() {
    const s = els['imu-capture-select'];
    if (!s) return;
    s.replaceChildren();
    if (!captures.length) {
        const o = document.createElement('option');
        o.value = '';
        o.textContent = listState === 'error' ? 'capture list unavailable' : 'no captures';
        s.appendChild(o);
        s.disabled = true;
        return;
    }
    s.disabled = false;
    // newest first, as the recordings picker
    for (let i = captures.length - 1; i >= 0; i--) {
        const c = captures[i];
        const o = document.createElement('option');
        o.value = c.stem;
        o.textContent = core.captureLabel(c);
        o.title = c.stem + (c.wall_ok === false ? ' (no wall-clock times)' : '');
        s.appendChild(o);
    }
    s.value = sel.selected || '';
}

// ---- Selection ----

function handPick(stem) {
    if (!captures.some((c) => c.stem === stem)) return;
    if (mode === 'replay') {
        const r = core.replayPick(captures, stem, playhead());
        sel = { selected: r.selected, follow: false, hold: r.hold };
        loadSelected();
    } else {
        sel = { ...core.livePick(captures, stem), hold: null };
        loadSelected();
        jumpToCapture();
    }
    renderJump();
}

/** LIVE automatic selection change: move the view only if the operator has not (core.autoJumpDecision). */
function autoApply() {
    const c = selectedCapture();
    if (!c || c.wall_ok === false) return;
    if (core.autoJumpDecision(getSharedViewState(), lastApplied) === 'jump') jumpToCapture();
}

function jumpToCapture() {
    const c = selectedCapture();
    if (!c || c.wall_ok === false) return;
    if (mode === 'replay') {
        seekReplayTo(c.t_start);
    } else if (showRangeOnAllCharts(c.t_start, c.t_end)) {
        lastApplied = getSharedXRange();
    }
    renderJump();
}

function renderJump() {
    const b = els['imu-jump-btn'];
    if (!b) return;
    const c = selectedCapture();
    b.hidden = !(shown && c && c.wall_ok !== false && !core.captureInView(c, xRange));
}

// ---- Capture data ----

async function loadSelected() {
    const c = selectedCapture();
    const token = ++dataToken;
    if (!c) { capData = null; renderMessage(); scheduleRedraw(); return; }
    if (c.wall_ok === false) { capData = { stem: c.stem, error: 'nowall' }; renderMessage(); scheduleRedraw(); return; }
    const hit = dataCache.get(c.stem);
    if (hit) {
        dataCache.delete(c.stem); dataCache.set(c.stem, hit);    // LRU touch
        capData = hit; renderMessage(); scheduleRedraw(); return;
    }
    capData = { stem: c.stem, loading: true };
    renderMessage(); scheduleRedraw();
    let entry;
    try {
        const r = await fetch(API + '/' + encodeURIComponent(c.stem) + '/data', { cache: 'no-store' });
        if (!r.ok) {
            let msg = 'HTTP ' + r.status;
            try { const j = await r.json(); if (j && (j.message || j.error)) msg = j.message || j.error; } catch { /* not JSON */ }
            entry = { stem: c.stem, error: msg };
        } else {
            const cap = core.parseImuCapture(await r.arrayBuffer());
            const cols = core.buildPlotColumns(cap);
            entry = { stem: c.stem, cols, dt: core.medianDt(cols.t), ang: null };
            dataCache.set(c.stem, entry);
            while (dataCache.size > DATA_CACHE_MAX) dataCache.delete(dataCache.keys().next().value);
        }
    } catch (e) {
        entry = { stem: c.stem, error: (e && e.message) || String(e) };
    }
    if (token !== dataToken) return;
    capData = entry;
    renderMessage();
    scheduleRedraw();
}

function renderMessage() {
    const m = els['imu-chart-message'];
    if (!m) return;
    let text = '';
    if (listState === 'error') text = core.listErrorMessage(listStatus);
    else if (listState === 'loading' && !captures.length) text = 'Loading IMU captures…';
    else if (listState === 'ok' && !captures.length) {
        text = mode === 'replay' ? 'No IMU captures in this recording'
            : (launch && launch.fallback ? 'No IMU captures in the last 12 h (launch start unknown)' : 'No IMU captures for this launch');
    } else if (capData && capData.error === 'nowall') {
        text = 'This capture has no wall-clock times (fit.wall_ok false), so it cannot be placed on the time axis: not drawn';
    } else if (capData && capData.error) {
        text = 'IMU capture file unreadable: ' + capData.error;
    } else if (capData && capData.loading) {
        text = 'Loading capture…';
    } else if (spectrum && specState && specState.error) {
        text = specState.error;
    }
    m.textContent = text;
    m.hidden = !text;
}

// ---- Shared x range ----

function onRange(min, max) {
    xRange = { min, max };
    if (!shown) return;
    if (mode === 'replay' && captures.length) {
        const r = core.replayFollow(sel, captures, playhead());
        sel = { selected: r.selected, follow: false, hold: r.hold };
        if (r.changed) {
            const s = els['imu-capture-select'];
            if (s) s.value = sel.selected || '';
            loadSelected();
        }
    }
    renderJump();
    scheduleRedraw();
}

// ---- Charts ----

function cssVar(name, fallback) {
    const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
    return v || fallback;
}

function seriesFor(kind) {
    return kind === 'accel' ? core.ACCEL_SERIES : core.GYRO_SERIES;
}

function chartUnit(kind) {
    if (kind === 'accel') return { scale: 'g', unit: 'g', decimals: 3, padFloor: 0.05 };
    return gyroAccel ? { scale: 'dps2', unit: 'deg/s^2', decimals: 0, padFloor: 50 }
        : { scale: 'dps', unit: 'deg/s', decimals: 1, padFloor: 5 };
}

function destroyChart(kind) {
    if (charts[kind]) { releaseEventMarkerHover(charts[kind]); charts[kind].destroy(); }
    charts[kind] = null;
    callouts[kind] = null;
    const el = els['imu-' + kind + '-plot'];
    if (el) { const w = el.querySelector('.u-wrap'); if (w) w.remove(); }
}

function rebuildChart(kind) {
    destroyChart(kind);
    if (!shown) return;
    const el = els['imu-' + kind + '-plot'];
    if (!el) return;
    const w = el.clientWidth, h = el.clientHeight;
    if (w < 10 || h < 10) return;               // collapsed panel: the ResizeObserver builds it later
    if (spectrum) { buildSpectrumChart(kind, el, w, h); return; }
    const defs = seriesFor(kind);
    const U = chartUnit(kind);
    const axisStroke = cssVar('--chart-axis-stroke', '#94a3b8');
    const gridStroke = cssVar('--chart-grid-stroke', 'rgba(51,65,85,0.5)');
    const gridStrokeMinor = cssVar('--chart-grid-stroke-minor', 'rgba(51,65,85,0.3)');
    const ticksStroke = cssVar('--chart-ticks-stroke', '#334155');
    const textStroke = cssVar('--text-secondary', '#cbd5e1');
    const rec = createCallouts(defs, U);
    const opts = {
        width: w, height: h,
        series: [{}, ...defs.map((d) => ({
            label: d.label, scale: U.scale, stroke: d.color, width: 1.25 * window.devicePixelRatio,
            points: { show: false }, gaps: nanGaps, show: enabled.has(d.key),
        }))],
        scales: {
            x: { time: true, auto: false },
            [U.scale]: { auto: true, range: (u, mn, mx) => core.imuYRange(mn, mx, U.padFloor) },
        },
        axes: [
            {
                stroke: axisStroke, grid: { stroke: gridStroke, width: 1 },
                ticks: { stroke: ticksStroke, width: 1, size: 6 },
                font: '11px JetBrains Mono, monospace', size: 28,
                incrs: TIME_INCRS, space: timeTickSpace(50), values: (u, splits) => timeTickLabels(splits),
            },
            {
                scale: U.scale, side: 3, label: U.unit, labelSize: 16, size: 64,
                stroke: textStroke, grid: { stroke: gridStrokeMinor, width: 1 },
                ticks: { stroke: textStroke, width: 1 }, space: numTickSpace(30),
                font: '12px JetBrains Mono, monospace', labelFont: '12px JetBrains Mono, monospace',
            },
        ],
        cursor: {
            show: true,
            // Same group as the ODrive charts; x is matched by value, y is not shared (other units).
            sync: { key: SYNC_KEY, setSeries: false, scales: ['x', null] },
            points: { show: false },
            drag: { x: true, y: false, setScale: false },
        },
        hooks: {
            setSelect: [(u) => {
                if (u.select.width < 3) return;
                const min = u.posToVal(u.select.left, 'x');
                const max = u.posToVal(u.select.left + u.select.width, 'x');
                u.setSelect({ left: 0, top: 0, width: 0, height: 0 }, false);
                selectRangeOnAllCharts(min, max);
            }],
            setCursor: [(u) => updateCallouts(u, rec, kind)],
            draw: [drawEventMarkers, drawReplayPlayhead],   // event markers and the REPLAY playhead, as on the motor charts
        },
        legend: { show: false },
        padding: [8, 8, 0, 0],
    };
    const u = new uPlot(opts, emptyData(defs.length), el);
    u.setScale('x', { min: xRange.min, max: xRange.max });
    attachZoomPanControls(u);
    attachEventMarkerHover(u);
    u.over.appendChild(rec.overlay);
    charts[kind] = u;
    callouts[kind] = rec;
    applyHeadings();
    scheduleRedraw();
}

function emptyData(n) {
    return [new Float64Array(0), ...Array.from({ length: n }, () => new Float64Array(0))];
}

/** The columns the time charts show: raw, or low-passed at the selected cutoff (computed once per
 *  (capture, cutoff) and kept on capData; capData.cols itself stays raw for the CSV export). */
function shownCols() {
    if (!capData || !capData.cols) return null;
    if (!(filterCutoff > 0)) return capData.cols;
    if (!capData.filt) capData.filt = new Map();
    let f = capData.filt.get(filterCutoff);
    if (!f) { f = { cols: core.lowpassColumns(capData.cols, filterCutoff), ang: null }; capData.filt.set(filterCutoff, f); }
    return f.cols;
}

function columnsFor(kind) {
    const c = shownCols();
    if (!c) return null;
    if (kind === 'accel') return [c.ax, c.ay, c.az, c.amag];
    if (!gyroAccel) return [c.gx, c.gy, c.gz];
    // The derivative is taken from the rate the chart would show, i.e. the filtered one when a filter is on.
    const holder = filterCutoff > 0 ? capData.filt.get(filterCutoff) : capData;
    if (!holder.ang) {
        holder.ang = [core.angularAcceleration(c.t, c.gx), core.angularAcceleration(c.t, c.gy),
            core.angularAcceleration(c.t, c.gz)];
    }
    return holder.ang;
}

function scheduleRedraw() {
    if (spectrum) { scheduleSpectrum(); return; }
    if (redrawPending) return;
    redrawPending = true;
    requestAnimationFrame(() => {
        redrawPending = false;
        for (const kind of ['accel', 'gyro']) redrawChart(kind);
    });
}

/** Re-slice (and decimate) for the current x range; called on range / data / size changes only. */
function redrawChart(kind) {
    const u = charts[kind];
    if (!u) return;
    if (spectrum) { scheduleSpectrum(); return; }
    const cols = columnsFor(kind);
    const n = seriesFor(kind).length;
    let data;
    if (!cols) {
        data = emptyData(n);
    } else {
        const width = u.bbox ? u.bbox.width / window.devicePixelRatio : u.over.clientWidth;
        data = core.decimateMinMax(capData.cols.t, cols, xRange.min, xRange.max, width).data;
    }
    u.setData(data);
    u.setScale('x', { min: xRange.min, max: xRange.max });
}

// ---- Series pills and the gyro switch ----

function buildPills(containerId, defs, kind) {
    const box = els[containerId];
    if (!box) return;
    defs.forEach((d, j) => {
        const b = document.createElement('button');
        b.type = 'button';
        b.className = 'signal-toggle' + (enabled.has(d.key) ? ' active' : '');
        b.dataset.series = d.key;
        b.style.setProperty('--signal-color', d.color);
        b.title = d.key === 'amag' ? 'Magnitude of the measured acceleration, sqrt(x^2 + y^2 + z^2), in g'
            : 'IMU ' + d.label + ' (sensor frame: the mount calibration is not done)';
        b.innerHTML = '<span class="color-dot"></span>' + d.label;
        b.addEventListener('click', () => {
            if (enabled.has(d.key)) enabled.delete(d.key); else enabled.add(d.key);
            b.classList.toggle('active', enabled.has(d.key));
            save(core.IMU_SERIES_KEY, JSON.stringify([...enabled]));
            const u = charts[kind];
            // Not u.setSeries(): it also moves each series' cursor-point element, and this chart
            // has none (cursor.points off), which throws inside uPlot. setData in redrawChart
            // re-runs the y auto-range over the visible series.
            if (u) { u.series[j + 1].show = enabled.has(d.key); redrawChart(kind); }
        });
        box.appendChild(b);
    });
}

function applyGyroModeUI() {
    const b = els['imu-gyro-mode-btn'];
    if (b) {
        b.classList.toggle('active', gyroAccel);
        b.title = gyroAccel
            ? 'Showing angular acceleration: the gyro after a ' + core.ANG_ACCEL_SMOOTH_MS + ' ms boxcar, central difference. Click for angular rate.'
            : 'Show angular acceleration (deg/s^2) instead: the gyro after a ' + core.ANG_ACCEL_SMOOTH_MS + ' ms boxcar, central difference.';
    }
    applyHeadings();
}

/** Chart titles, the filter badge and the spectrum peak readout (time charts: title + "low-pass N Hz"). */
function applyHeadings() {
    const set = (id, text) => { const e = els[id]; if (e) e.textContent = text; };
    if (!spectrum) {
        set('imu-accel-title', core.ACCEL_TITLE);
        set('imu-gyro-title', gyroAccel ? core.ANG_ACCEL_TITLE : core.GYRO_TITLE);
        set('imu-accel-badge', core.filterBadge(filterCutoff));
        set('imu-gyro-badge', core.filterBadge(filterCutoff));
        set('imu-accel-peaks', '');
        set('imu-gyro-peaks', '');
        return;
    }
    set('imu-accel-title', 'Acceleration spectrum (' + SPEC_UNITS.accel + ')');
    set('imu-gyro-title', 'Angular rate spectrum (' + SPEC_UNITS.gyro + ')');
    const note = 'raw samples' + (filterCutoff > 0 ? ', no filter' : '');
    set('imu-accel-badge', note);
    set('imu-gyro-badge', note);
    const detail = specState && !specState.error
        ? 'Welch over ' + specState.windowSec.toFixed(1) + ' s in view: ' + specState.seg + '-point Hann segments, 50 % overlap, ' + specState.nSeg + ' averaged'
        : 'Welch spectrum of the raw samples in the visible time range';
    for (const id of ['imu-accel-badge', 'imu-gyro-badge']) if (els[id]) els[id].title = detail;
    for (const kind of ['accel', 'gyro']) {
        const k = specState && !specState.error ? specState.kinds[kind] : null;
        const parts = [];
        if (k) k.peaks.forEach((p, j) => { if (p) parts.push(seriesFor(kind)[j].label + ' ' + p.f.toFixed(2) + ' Hz'); });
        set('imu-' + kind + '-peaks', parts.length ? 'peaks ' + parts.join(' · ') : '');
    }
}

// ---- Controls visibility ----

function applyControlsUI() {
    const hide = (id, h) => { const e = els[id]; if (e) e.hidden = h; };
    hide('imu-filter-select', !shown);
    hide('imu-spectrum-btn', !shown);
    hide('imu-spec-max-select', !shown || !spectrum);
    hide('imu-gyro-mode-btn', spectrum);
    const f = els['imu-filter-select'];
    if (f) f.disabled = spectrum;
    const b = els['imu-spectrum-btn'];
    if (b) b.classList.toggle('active', spectrum);
}

// ---- Spectrum mode: amplitude spectra of the raw samples inside the visible x range ----

const SPEC_UNITS = { accel: 'g/√Hz', gyro: '(deg/s)/√Hz' };

function setSpectrum(on) {
    spectrum = on;
    save(core.IMU_SPECTRUM_KEY, on ? 'true' : 'false');
    applyControlsUI();
    specState = null;
    if (specTimer !== null) { clearTimeout(specTimer); specTimer = null; }
    applyHeadings();
    rebuildChart('accel');
    rebuildChart('gyro');
    // Spectrum on: rebuildChart re-arms the debounce timer, which would run a second, identical Welch pass 150 ms
    // after this direct one; drop it.
    if (on) { if (specTimer !== null) { clearTimeout(specTimer); specTimer = null; } computeSpectrum(); }
    else { renderMessage(); scheduleRedraw(); }
}

/** Debounced recompute: a range / capture / series change burst costs one Welch pass, never one per frame.
 *  Trailing debounce with a max wait: while the shared x range moves every frame (live follow) the timer is
 *  re-armed forever, so a pass is forced once SPEC_MAX_WAIT_MS has passed since the first un-served request. */
const SPEC_MAX_WAIT_MS = 500;
let specFirstReq = 0;
function scheduleSpectrum() {
    const now = Date.now();   // wall-clock: UI debounce timing, not data time
    if (specTimer === null) specFirstReq = now;
    else clearTimeout(specTimer);
    const wait = Math.max(0, Math.min(core.SPEC_DEBOUNCE_MS, specFirstReq + SPEC_MAX_WAIT_MS - now));
    specTimer = setTimeout(() => { specTimer = null; computeSpectrum(); }, wait);
}

function computeSpectrum() {
    if (!spectrum || !shown) return;
    if (!capData || !capData.cols) { specState = null; renderSpectrum(); return; }
    const c = capData.cols;
    const kinds = {};
    let err = null, fs = NaN, seg = 0, nSeg = 0;
    for (const kind of ['accel', 'gyro']) {
        const defs = seriesFor(kind);
        const idx = defs.map((d, j) => j).filter((j) => enabled.has(defs[j].key));
        const entry = { freqs: null, series: defs.map(() => null), peaks: defs.map(() => null) };
        kinds[kind] = entry;
        if (!idx.length) continue;
        const r = core.welchSpectra(c.t, idx.map((j) => c[defs[j].col]), xRange.min, xRange.max);
        if (r.error) { err = r.error; break; }
        fs = r.fs; seg = Math.max(seg, r.seg); nSeg = Math.max(nSeg, ...r.nSeg);
        entry.freqs = r.freqs;
        const xMax = core.specXMax(specMax, r.fs);
        idx.forEach((j, q) => {
            entry.series[j] = r.asd[q];
            entry.peaks[j] = r.asd[q] ? core.spectrumPeak(r.freqs, r.asd[q], core.SPEC_PEAK_MIN_HZ, xMax) : null;
        });
    }
    specState = err ? { error: err } : {
        kinds, fs, seg, nSeg, xMax: core.specXMax(specMax, Number.isFinite(fs) ? fs : 1000),
        windowSec: Math.max(0, xRange.max - xRange.min),
    };
    renderSpectrum();
    renderMessage();
}

function renderSpectrum() {
    applyHeadings();
    for (const kind of ['accel', 'gyro']) {
        const u = charts[kind];
        if (!u) continue;
        const defs = seriesFor(kind);
        defs.forEach((d, j) => { u.series[j + 1].show = enabled.has(d.key); });
        const k = specState && !specState.error ? specState.kinds[kind] : null;
        if (!k || !k.freqs) {
            u.setData(emptyData(defs.length), false);
            u.setScale('x', { min: 0, max: specState && specState.xMax ? specState.xMax : 50 });
            u.setScale('asd', { min: 1e-4, max: 1 });
            continue;
        }
        const xMax = specState.xMax;
        let kmax = 1;
        while (kmax + 1 < k.freqs.length && k.freqs[kmax + 1] <= xMax) kmax++;
        const data = [k.freqs.slice(1, kmax + 1), ...k.series.map((a) => (a ? a.slice(1, kmax + 1) : new Float64Array(kmax).fill(NaN)))];
        let lo = Infinity, hi = 0;
        k.series.forEach((a, j) => {
            if (!a || !enabled.has(defs[j].key)) return;
            for (let i = 1; i <= kmax; i++) { const v = a[i]; if (v > 0) { if (v < lo) lo = v; if (v > hi) hi = v; } }
        });
        let yMin = 1e-4, yMax = 1;
        if (hi > 0 && Number.isFinite(lo)) {
            yMax = Math.pow(10, Math.ceil(Math.log10(hi)));
            yMin = Math.max(Math.pow(10, Math.floor(Math.log10(lo))), yMax * 1e-6);
            if (yMin * 10 > yMax) yMin = yMax / 100;
        }
        u.setData(data, false);
        u.setScale('x', { min: 0, max: xMax });
        u.setScale('asd', { min: yMin, max: yMax });
    }
}

function buildSpectrumChart(kind, el, w, h) {
    const defs = seriesFor(kind);
    const unit = SPEC_UNITS[kind];
    const axisStroke = cssVar('--chart-axis-stroke', '#94a3b8');
    const gridStroke = cssVar('--chart-grid-stroke', 'rgba(51,65,85,0.5)');
    const gridStrokeMinor = cssVar('--chart-grid-stroke-minor', 'rgba(51,65,85,0.3)');
    const ticksStroke = cssVar('--chart-ticks-stroke', '#334155');
    const textStroke = cssVar('--text-secondary', '#cbd5e1');
    const rec = createCallouts(defs, { scale: 'asd', unit, decimals: 3, padFloor: 0 });
    const opts = {
        width: w, height: h,
        series: [{}, ...defs.map((d) => ({
            label: d.label, scale: 'asd', stroke: d.color, width: 1.25 * window.devicePixelRatio,
            points: { show: false }, gaps: nanGaps, show: enabled.has(d.key),
        }))],
        scales: {
            x: { time: false, auto: false },
            asd: { distr: 3, log: 10, auto: false },
        },
        axes: [
            {
                stroke: axisStroke, grid: { stroke: gridStroke, width: 1 },
                ticks: { stroke: ticksStroke, width: 1, size: 6 },
                font: '11px JetBrains Mono, monospace', size: 28, label: 'Hz', labelSize: 14,
                labelFont: '11px JetBrains Mono, monospace', space: numTickSpace(60),
            },
            {
                scale: 'asd', side: 3, label: unit, labelSize: 16, size: 64,
                stroke: textStroke, grid: { stroke: gridStrokeMinor, width: 1 },
                ticks: { stroke: textStroke, width: 1 },
                values: (u, splits) => splits.map((v) => (v == null ? "" : v >= 1e-2 ? String(+v.toPrecision(3)) : v.toExponential(0))),
                font: '12px JetBrains Mono, monospace', labelFont: '12px JetBrains Mono, monospace',
            },
        ],
        // No sync key and no zoom / pan / select: a spectrum is not a time chart, so it neither follows nor
        // moves the shared cursor and range.
        cursor: { show: true, points: { show: false }, drag: { x: false, y: false, setScale: false } },
        hooks: {
            setCursor: [(u) => updateSpecCallouts(u, rec, kind)],
            draw: [(u) => drawSpecPeaks(u, kind)],
        },
        legend: { show: false },
        padding: [8, 8, 0, 0],
    };
    const u = new uPlot(opts, emptyData(defs.length), el);
    u.setScale('x', { min: 0, max: 50 });
    u.setScale('asd', { min: 1e-4, max: 1 });
    u.over.appendChild(rec.overlay);
    charts[kind] = u;
    callouts[kind] = rec;
    applyHeadings();
    scheduleSpectrum();
}

function updateSpecCallouts(u, rec, kind) {
    const { overlay, pills } = rec;
    const k = specState && !specState.error ? specState.kinds[kind] : null;
    const left = u.cursor.left;
    if (!k || !k.freqs || left == null || left < 0) { overlay.style.opacity = '0'; return; }
    const f = u.posToVal(left, 'x');
    const df = k.freqs[1] - k.freqs[0];
    const bin = Math.round(f / df);
    if (bin < 1 || bin >= k.freqs.length) { overlay.style.opacity = '0'; return; }
    overlay.style.opacity = '1';
    const xPx = u.valToPos(k.freqs[bin], 'x');
    const plotW = u.bbox ? u.bbox.width / devicePixelRatio : u.over.clientWidth;
    const flipLeft = xPx > plotW * 0.75;
    const shownPills = [];
    for (let j = 0; j < pills.length; j++) {
        const { el, text, def } = pills[j];
        const a = k.series[j];
        const v = a ? a[bin] : NaN;
        const yPx = v > 0 ? u.valToPos(v, 'asd') : NaN;
        if (!enabled.has(def.key) || !Number.isFinite(yPx)) { el.style.display = 'none'; continue; }
        el.style.display = '';
        el.style.left = xPx + 'px';
        el.classList.toggle('flip-left', flipLeft);
        text.textContent = def.label + ' ' + k.freqs[bin].toFixed(2) + ' Hz  ' + v.toPrecision(3) + ' ' + SPEC_UNITS[kind];
        shownPills.push({ el, yPx });
    }
    const pillH = (shownPills.length && shownPills[0].el.offsetHeight) || 16;
    const plotH = u.bbox ? u.bbox.height / devicePixelRatio : u.over.clientHeight;
    const ys = layoutCallouts(shownPills.map((p) => p.yPx), pillH, plotH);
    for (let q = 0; q < shownPills.length; q++) shownPills[q].el.style.top = ys[q] + 'px';
}

/** Dot and "f Hz" label on each enabled series' largest peak (1 Hz .. x max); labels de-overlapped vertically. */
function drawSpecPeaks(u, kind) {
    const ctx = u.ctx;
    const k = specState && !specState.error ? specState.kinds[kind] : null;
    if (!ctx || !u.bbox || !k) return;
    const defs = seriesFor(kind);
    const dpr = window.devicePixelRatio || 1;
    const { left, top, width, height } = u.bbox;
    const items = [];
    defs.forEach((d, j) => {
        const p = k.peaks[j];
        if (!p || !enabled.has(d.key)) return;
        items.push({ d, p, x: u.valToPos(p.f, 'x', true), y: u.valToPos(p.v, 'asd', true) });
    });
    if (!items.length) return;
    ctx.save();
    ctx.beginPath(); ctx.rect(left, top, width, height); ctx.clip();
    const lh = 13 * dpr;
    const ys = layoutCallouts(items.map((it) => it.y - 10 * dpr), lh, top + height);
    ctx.font = (11 * dpr) + 'px JetBrains Mono, monospace';
    ctx.textBaseline = 'middle';
    items.forEach((it, q) => {
        ctx.fillStyle = it.d.color;
        ctx.beginPath(); ctx.arc(it.x, it.y, 3 * dpr, 0, 2 * Math.PI); ctx.fill();
        const label = it.d.label + ' ' + it.p.f.toFixed(2) + ' Hz';
        const wText = ctx.measureText(label).width;
        const flip = it.x + 8 * dpr + wText > left + width;
        ctx.textAlign = flip ? 'right' : 'left';
        ctx.fillText(label, it.x + (flip ? -8 : 8) * dpr, Math.max(top + lh / 2, ys[q]));
    });
    ctx.restore();
}

// ---- Callouts (own lookup: nearest FULL-resolution sample by binary search on time) ----

function createCallouts(defs, U) {
    const overlay = document.createElement('div');
    overlay.className = 'chart-callouts';
    const pills = defs.map((d) => {
        const el = document.createElement('div');
        el.className = 'chart-callout';
        el.style.setProperty('--signal-color', d.color);
        el.innerHTML = '<span class="chart-callout-dot"></span><span class="chart-callout-text"></span>';
        el.style.display = 'none';
        overlay.appendChild(el);
        return { el, text: el.querySelector('.chart-callout-text'), def: d };
    });
    return { overlay, pills, U };
}

function updateCallouts(u, rec, kind) {
    if (!rec) return;
    const { overlay, pills, U } = rec;
    const cols = columnsFor(kind);
    const left = u.cursor.left;
    if (!cols || left == null || left < 0) { overlay.style.opacity = '0'; return; }
    const ts = capData.cols.t;
    const t = u.posToVal(left, 'x');
    const i = core.nearestIndex(ts, t, 2 * (capData.dt || 0.001));
    if (i < 0) { overlay.style.opacity = '0'; return; }
    overlay.style.opacity = '1';
    const xPx = u.valToPos(ts[i], 'x');
    const plotW = u.bbox ? u.bbox.width / devicePixelRatio : u.over.clientWidth;
    const flipLeft = xPx > plotW * 0.75;
    const shownPills = [];
    for (let j = 0; j < pills.length; j++) {
        const { el, text, def } = pills[j];
        const v = cols[j][i];
        const yPx = Number.isFinite(v) ? u.valToPos(v, U.scale) : NaN;
        if (!enabled.has(def.key) || !Number.isFinite(yPx)) { el.style.display = 'none'; continue; }
        el.style.display = '';
        el.style.left = xPx + 'px';
        el.classList.toggle('flip-left', flipLeft);
        text.textContent = def.label + ' ' + v.toFixed(U.decimals) + ' ' + U.unit;
        shownPills.push({ el, yPx });
    }
    const pillH = (shownPills.length && shownPills[0].el.offsetHeight) || 16;
    const plotH = u.bbox ? u.bbox.height / devicePixelRatio : u.over.clientHeight;
    const ys = layoutCallouts(shownPills.map((s) => s.yPx), pillH, plotH);
    for (let k = 0; k < shownPills.length; k++) shownPills[k].el.style.top = ys[k] + 'px';
}
