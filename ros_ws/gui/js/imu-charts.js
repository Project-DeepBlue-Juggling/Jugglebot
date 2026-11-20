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
    SYNC_KEY, nanGaps, onSharedXRange, getSharedXRange, getSharedViewState,
    showRangeOnAllCharts, selectRangeOnAllCharts, seekReplayTo, attachZoomPanControls, drawReplayPlayhead,
} from './telemetry-charts.js';
import * as core from './imu-charts-core.js';

const API = '/api/imu/captures';
const RECORDINGS_API = '/api/replay/recordings';
const LAUNCH_REFRESH_MS = 60000;
const DATA_CACHE_MAX = 4;

// ---- State ----

let shown = false;
let enabled = new Set(core.DEFAULT_SERIES);
let gyroAccel = false;                       // right chart shows angular acceleration
let wantedImuHeight = core.IMU_HEIGHT_DEFAULT; // the operator's choice; the applied one may be capped
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
}

function save(key, value) {
    try { localStorage.setItem(key, value); } catch { /* private window: per-viewer convenience only */ }
}

// ---- Init ----

export function initImuCharts() {
    for (const id of ['imu-charts-toggle-btn', 'imu-capture-select', 'imu-jump-btn', 'imu-chart-group',
        'imu-accel-plot', 'imu-gyro-plot', 'imu-accel-toggles', 'imu-gyro-toggles', 'imu-gyro-mode-btn',
        'imu-gyro-title', 'imu-chart-message']) {
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

// ---- IMU row height: the divider above the motor grid (changes ONLY this row) ----

function appEl() { return document.getElementById('app'); }

function currentImuHeight() {
    const g = els['imu-chart-group'];
    const h = g && g.getBoundingClientRect ? g.getBoundingClientRect().height : 0;
    return h > 0 ? h : core.IMU_HEIGHT_DEFAULT;
}

/** Set --imu-chart-height from the wanted height, shrunk if the 3D view would fall under its minimum. */
function applyImuHeight() {
    const app = appEl();
    if (!app) return;
    let eff = core.clampImuHeight(wantedImuHeight);
    const viewer = document.getElementById('viewer-pane');
    if (shown && viewer) eff = core.effectiveImuHeight(wantedImuHeight, currentImuHeight(), viewer.clientHeight);
    app.style.setProperty('--imu-chart-height', eff + 'px');
}

function initHeightDivider() {
    const divider = document.getElementById('imu-chart-divider');
    let saved = null;
    try { saved = localStorage.getItem(core.IMU_HEIGHT_KEY); } catch { /* storage blocked */ }
    wantedImuHeight = core.clampImuHeight(saved);
    applyImuHeight();
    window.addEventListener('resize', applyImuHeight);
    const viewer = document.getElementById('viewer-pane');
    if (viewer && typeof ResizeObserver !== 'undefined') new ResizeObserver(applyImuHeight).observe(viewer);
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
    // The panel is bottom-anchored: the divider stays put, the row's top edge moves.  Up = taller.
    document.addEventListener('pointermove', (e) => {
        if (!dragging) return;
        wantedImuHeight = core.clampImuHeight(startH + (startY - e.clientY));
        applyImuHeight();
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
        wantedImuHeight = core.IMU_HEIGHT_DEFAULT;
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
    if (charts[kind]) charts[kind].destroy();
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
            draw: [drawReplayPlayhead],           // REPLAY playhead, same line as the motor charts
        },
        legend: { show: false },
        padding: [8, 8, 0, 0],
    };
    const u = new uPlot(opts, emptyData(defs.length), el);
    u.setScale('x', { min: xRange.min, max: xRange.max });
    attachZoomPanControls(u);
    u.over.appendChild(rec.overlay);
    charts[kind] = u;
    callouts[kind] = rec;
    scheduleRedraw();
}

function emptyData(n) {
    return [new Float64Array(0), ...Array.from({ length: n }, () => new Float64Array(0))];
}

function columnsFor(kind) {
    if (!capData || !capData.cols) return null;
    const c = capData.cols;
    if (kind === 'accel') return [c.ax, c.ay, c.az, c.amag];
    if (!gyroAccel) return [c.gx, c.gy, c.gz];
    if (!capData.ang) {
        capData.ang = [core.angularAcceleration(c.t, c.gx), core.angularAcceleration(c.t, c.gy),
            core.angularAcceleration(c.t, c.gz)];
    }
    return capData.ang;
}

function scheduleRedraw() {
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
    const t = els['imu-gyro-title'];
    if (t) t.textContent = gyroAccel ? core.ANG_ACCEL_TITLE : core.GYRO_TITLE;
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
