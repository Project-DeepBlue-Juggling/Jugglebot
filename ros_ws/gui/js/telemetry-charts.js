/**
 * telemetry-charts.js — Live time-series charts for motor telemetry.
 *
 * Creates a 3x3 grid of uPlot charts (one per ODrive motor) in the bottom
 * panel.  Each chart plots user-selected signals (position, velocity, current,
 * temperature, bus voltage/current) with shared Y-axes for same-unit signals
 * and synchronized crosshair cursors.
 *
 * Hover UX:
 *   - Entering a chart cell highlights the matching element in the 3D scene
 *     (leg 0..5 → Stewart leg, Hand → Stewart hand axis, BB Pitch/Hand → BB
 *     pitch group / hand sphere).
 *   - The vertical crosshair shows per-curve value callouts at each series
 *     intersection, synced across all charts via uPlot's cursor sync group.
 */

import { setStewartHighlight } from './stewart-model.js';
import { setBallButlerHighlight } from './ball-butler-model.js';
import {
    getEventsInRange, subscribeEvents, subscribeHighlight,
    getHighlightedEventId, EVENT_COLORS,
} from './event-store.js';

// ---- Signal definitions ----

/**
 * Each signal group describes one plottable series.
 *  key      – unique identifier & localStorage toggle key
 *  label    – toolbar button text
 *  color    – line colour (consistent across all 9 charts)
 *  unit     – physical unit (for Y-axis label)
 *  scale    – scale group key (signals sharing a scale share a Y-axis)
 *  extract  – fn(motorState) → number   (null if external data)
 *  dash     – optional uPlot dash pattern [on, off]
 */
const SIGNAL_GROUPS = [
    { key: 'pos_measured',  label: 'Pos (meas)',    color: '#3b82f6', unit: 'rev',   scale: 'position',    extract: m => m.pos_estimate },
    { key: 'pos_commanded', label: 'Pos (cmd)',     color: '#60a5fa', unit: 'rev',   scale: 'position',    extract: null, dash: [5, 3] },
    { key: 'vel_measured',  label: 'Velocity',      color: '#22c55e', unit: 'rev/s', scale: 'velocity',    extract: m => m.vel_estimate },
    { key: 'iq_setpoint',   label: 'Current (set)', color: '#f59e0b', unit: 'A',     scale: 'current',     extract: m => m.iq_setpoint },
    { key: 'iq_measured',   label: 'Current (meas)',color: '#fbbf24', unit: 'A',     scale: 'current',     extract: m => m.iq_measured, dash: [5, 3] },
    { key: 'fet_temp',      label: 'FET Temp',      color: '#ef4444', unit: '\u00b0C', scale: 'temperature', extract: m => m.fet_temp },
    { key: 'motor_temp',    label: 'Motor Temp',    color: '#f87171', unit: '\u00b0C', scale: 'temperature', extract: m => m.motor_temp },
    { key: 'bus_voltage',   label: 'Bus Voltage',   color: '#a78bfa', unit: 'V',     scale: 'voltage',     extract: m => m.bus_voltage },
    { key: 'bus_current',   label: 'Bus Current',   color: '#c084fc', unit: 'A',     scale: 'bus_current', extract: m => m.bus_current },
];

/** Map from scale key → display metadata */
const SCALE_META = {
    position:    { unit: 'rev',   decimals: 3 },
    velocity:    { unit: 'rev/s', decimals: 2 },
    current:     { unit: 'A',     decimals: 2 },
    temperature: { unit: '\u00b0C', decimals: 1 },
    voltage:     { unit: 'V',     decimals: 2 },
    bus_current: { unit: 'A',     decimals: 2 },
};

// ---- Motor layout ----

const CHART_LABELS = ['Leg 0', 'Leg 1', 'Leg 2', 'Leg 3', 'Leg 4', 'Leg 5', 'Hand', 'BB Pitch', 'BB Hand'];
const MOTOR_COUNT = 9;

/**
 * Human-readable tooltip for each chart cell — describes which physical
 * actuator the motor drives.  Surfaced on hover via a title attr on the
 * chart-cell-title pill.
 */
const CHART_TOOLTIPS = [
    'Leg 0 — Stewart platform prismatic leg, CAN motor 0.',
    'Leg 1 — Stewart platform prismatic leg, CAN motor 1.',
    'Leg 2 — Stewart platform prismatic leg, CAN motor 2.',
    'Leg 3 — Stewart platform prismatic leg, CAN motor 3.',
    'Leg 4 — Stewart platform prismatic leg, CAN motor 4.',
    'Leg 5 — Stewart platform prismatic leg, CAN motor 5.',
    'Hand — linear hand actuator on the Stewart platform, CAN motor 6.',
    'BB Pitch — Ball Butler pitch servo, CAN motor 7.',
    'BB Hand — Ball Butler hand/throw servo, CAN motor 8.',
];

/**
 * Tooltip copy shown on each signal-toggle pill, explaining units and (where
 * non-obvious) sign conventions.  Deliberately terse — longer docs live in
 * the engineering logbook.
 */
const SIGNAL_TOOLTIPS = {
    pos_measured:  'Encoder position (revolutions from home). Toggle to show/hide on every chart.',
    pos_commanded: 'Commanded position target (dashed). Source: leg_lengths_topic for legs, hand_telemetry for the hand.',
    vel_measured:  'Motor velocity (rev/s). Signed — follows the motor\u2019s own direction convention.',
    iq_setpoint:   'Commanded quadrature current (A) \u2014 proportional to commanded torque.',
    iq_measured:   'Measured quadrature current (A, dashed) \u2014 proportional to actual torque.',
    fet_temp:      'ODrive MOSFET temperature (\u00b0C).',
    motor_temp:    'Motor winding temperature (\u00b0C).',
    bus_voltage:   'DC bus voltage at the ODrive (V).',
    bus_current:   'DC bus current (A). Negative values indicate regeneration back to the supply.',
};

// ---- State ----

const SIGNALS_STORAGE_KEY    = 'jugglebot-chart-signals';
const WINDOW_STORAGE_KEY     = 'jugglebot-chart-window';
const VISIBILITY_STORAGE_KEY = 'jugglebot-chart-visibility';
const DEFAULT_SIGNALS      = ['pos_measured', 'vel_measured'];
const DEFAULT_WINDOW_SEC   = 10;
const DATA_RATE_HZ         = 20;
// Retain 10 min of history regardless of the display window.  The dropdown
// caps the *live* window at 60 s, but wheel-zoom can expand out to this
// cache limit so users can look further back at paused / ended sessions
// without reloading.
const CACHE_WINDOW_SEC     = 600;

let activeSignals = new Set();
let currentWindowSec = DEFAULT_WINDOW_SEC;
/** Effective live window — can be widened beyond `currentWindowSec` by
 *  wheel-zooming while in live mode, so the stream keeps tracking but the
 *  visible span grows.  Reset to `currentWindowSec` whenever the user
 *  explicitly picks a new preset or presses R. */
let liveWindowSec = DEFAULT_WINDOW_SEC;
let maxPoints = 0;
/** Set of visible chart indices (0..8).  Defaults to all visible. */
let visibleCharts = new Set();

/** Per-motor data stores */
const stores = [];
/** Per-motor uPlot instances */
const charts = [];
/** Per-chart callout metadata — populated during buildAllCharts. */
const chartCallouts = [];
/** Per-chart delta-readout callout metadata. */
const chartDeltaCallouts = [];
/** Whether a rAF repaint is already scheduled */
let pendingRepaint = false;
/** Whether chart updates are paused (data still accumulates) */
let paused = false;
/** Frozen window edges while paused */
let pausedWindowStart = 0;
let pausedWindowEnd = 0;
/** Cursor sync key shared by all charts */
const SYNC_KEY = 'telemetry';

/**
 * View mode for the x-axis.
 *   - 'live'   : axis follows getViewAnchor() each repaint (default).
 *   - 'manual' : user has pan/zoomed — we honour manualXRange until reset.
 */
let viewMode = 'live';
/** X-axis range while in manual view mode — {min, max} in seconds. */
let manualXRange = null;
/** Zoom factor applied per wheel notch (standard delta = ±100). */
const WHEEL_ZOOM_STEP = 1.1;
/** Minimum x-axis span (seconds) — stops zoom from collapsing to a point. */
const MIN_X_SPAN_SEC = 0.05;
/** Maximum x-axis span (seconds) — never exceed the cache retention. */
const MAX_X_SPAN_SEC = CACHE_WINDOW_SEC;

// ---- Data store ----

class ChartDataStore {
    constructor(capacity) {
        this.capacity = capacity;
        this.length = 0;
        this.timestamps = new Float64Array(capacity);
        this.columns = {};
        for (const sg of SIGNAL_GROUPS) {
            this.columns[sg.key] = new Float64Array(capacity);
        }
    }

    push(timestamp, values) {
        if (this.length >= this.capacity) {
            // Shift left by 1 using fast memcpy
            this.timestamps.copyWithin(0, 1);
            for (const col of Object.values(this.columns)) {
                col.copyWithin(0, 1);
            }
            this.length = this.capacity - 1;
        }
        const i = this.length;
        this.timestamps[i] = timestamp;
        for (const key in values) {
            if (this.columns[key]) {
                this.columns[key][i] = values[key];
            }
        }
        this.length++;
    }

    /**
     * Build uPlot-compatible aligned data for active signals within the time window.
     * Returns [timestamps, series1, series2, ...] as TypedArray views.
     */
    getAlignedData(signalKeys, windowStart) {
        // Binary search for the first timestamp >= windowStart
        let lo = 0, hi = this.length;
        while (lo < hi) {
            const mid = (lo + hi) >>> 1;
            if (this.timestamps[mid] < windowStart) lo = mid + 1;
            else hi = mid;
        }
        const start = lo;
        const data = [this.timestamps.subarray(start, this.length)];
        for (const key of signalKeys) {
            data.push(this.columns[key].subarray(start, this.length));
        }
        return data;
    }

    /** Reset all data (e.g. on window size increase) */
    resize(newCapacity) {
        if (newCapacity === this.capacity) return;
        const newTs = new Float64Array(newCapacity);
        const keep = Math.min(this.length, newCapacity);
        const srcStart = this.length - keep;
        newTs.set(this.timestamps.subarray(srcStart, this.length));

        const newCols = {};
        for (const [key, col] of Object.entries(this.columns)) {
            const nc = new Float64Array(newCapacity);
            nc.set(col.subarray(srcStart, this.length));
            newCols[key] = nc;
        }
        this.timestamps = newTs;
        this.columns = newCols;
        this.capacity = newCapacity;
        this.length = keep;
    }
}

// ---- Initialisation ----

export function initTelemetryCharts() {
    loadSettings();
    computeMaxPoints();
    createStores();
    initSignalToggles();
    initChartVisibilityToggles();
    initTimeWindowSelector();
    initPauseButton();
    initExportButton();
    addChartTitles();
    applyChartLayout();  // apply grid-template + hidden classes before building
    buildAllCharts();
    initKeyboardShortcuts();

    // Repaint event markers whenever the event store changes OR the user
    // hovers a different row in the history panel.  Both coalesce through
    // the same rAF slot so rapid hovers don't thrash 9 charts × redraws.
    let overlayRepaintQueued = false;
    const queueOverlayRepaint = () => {
        if (overlayRepaintQueued) return;
        overlayRepaintQueued = true;
        requestAnimationFrame(() => {
            overlayRepaintQueued = false;
            redrawAllOverlays();
        });
    };
    subscribeEvents(queueOverlayRepaint);
    subscribeHighlight(queueOverlayRepaint);

    // Resize charts when the grid container changes size
    const grid = document.getElementById('chart-grid');
    if (grid) {
        const ro = new ResizeObserver(() => resizeAllCharts());
        ro.observe(grid);
    }
}

// ---- Settings persistence ----

function loadSettings() {
    // Active signals
    try {
        const saved = localStorage.getItem(SIGNALS_STORAGE_KEY);
        if (saved) {
            const arr = JSON.parse(saved);
            activeSignals = new Set(arr.filter(k => SIGNAL_GROUPS.some(s => s.key === k)));
        }
    } catch { /* ignore */ }
    if (activeSignals.size === 0) {
        activeSignals = new Set(DEFAULT_SIGNALS);
    }

    // Time window
    const savedWin = localStorage.getItem(WINDOW_STORAGE_KEY);
    if (savedWin) {
        const v = parseInt(savedWin, 10);
        if ([5, 10, 30, 60].includes(v)) currentWindowSec = v;
    }
    liveWindowSec = currentWindowSec;

    // Chart visibility — default to all 9 visible if missing/invalid.
    try {
        const saved = localStorage.getItem(VISIBILITY_STORAGE_KEY);
        if (saved) {
            const arr = JSON.parse(saved);
            if (Array.isArray(arr)) {
                visibleCharts = new Set(
                    arr.map(n => parseInt(n, 10))
                        .filter(n => Number.isInteger(n) && n >= 0 && n < MOTOR_COUNT)
                );
            }
        }
    } catch { /* ignore */ }
    if (visibleCharts.size === 0) {
        visibleCharts = new Set(Array.from({ length: MOTOR_COUNT }, (_, i) => i));
    }
}

function saveActiveSignals() {
    localStorage.setItem(SIGNALS_STORAGE_KEY, JSON.stringify([...activeSignals]));
}

function saveTimeWindow() {
    localStorage.setItem(WINDOW_STORAGE_KEY, currentWindowSec.toString());
}

function saveChartVisibility() {
    localStorage.setItem(VISIBILITY_STORAGE_KEY, JSON.stringify([...visibleCharts]));
}

function computeMaxPoints() {
    maxPoints = CACHE_WINDOW_SEC * DATA_RATE_HZ + 20; // small margin
}

// ---- Stores ----

function createStores() {
    stores.length = 0;
    for (let i = 0; i < MOTOR_COUNT; i++) {
        stores.push(new ChartDataStore(maxPoints));
    }
}

// ---- Chart titles ----

function addChartTitles() {
    for (let i = 0; i < MOTOR_COUNT; i++) {
        const cell = document.getElementById(`chart-${i}`);
        if (!cell || cell.querySelector('.chart-cell-title')) continue;
        // Tooltip on the cell (not the title pill) so the cursor tracker
        // inside uPlot keeps receiving mouse events — the native tooltip
        // still surfaces after a ~1 s stationary hover.
        cell.title = CHART_TOOLTIPS[i];
        const title = document.createElement('span');
        title.className = 'chart-cell-title';
        title.textContent = CHART_LABELS[i];
        cell.appendChild(title);
    }
}

// ---- Signal toggle toolbar ----

function initSignalToggles() {
    const container = document.getElementById('signal-toggles');
    if (!container) return;

    for (const sig of SIGNAL_GROUPS) {
        const btn = document.createElement('button');
        btn.className = 'signal-toggle' + (activeSignals.has(sig.key) ? ' active' : '');
        btn.dataset.signal = sig.key;
        btn.style.setProperty('--signal-color', sig.color);
        btn.title = SIGNAL_TOOLTIPS[sig.key] || sig.label;

        // Use a line indicator for dashed signals, dot for solid
        const indicator = sig.dash
            ? `<span class="color-line"></span>`
            : `<span class="color-dot"></span>`;
        btn.innerHTML = `${indicator}${sig.label}`;

        btn.addEventListener('click', () => {
            if (activeSignals.has(sig.key)) {
                activeSignals.delete(sig.key);
                btn.classList.remove('active');
            } else {
                activeSignals.add(sig.key);
                btn.classList.add('active');
            }
            saveActiveSignals();
            rebuildAllCharts();
        });

        container.appendChild(btn);
    }
}

// ---- Chart visibility toolbar ----

/**
 * Shape of the chart grid for N visible charts.  Column-major fill: charts
 * go top-to-bottom in column 1, then column 2, etc., in motor-numeric order.
 */
function gridShapeFor(n) {
    if (n <= 1) return { cols: 1, rows: 1 };
    if (n === 2) return { cols: 2, rows: 1 };
    if (n === 3) return { cols: 3, rows: 1 };
    if (n === 4) return { cols: 2, rows: 2 };
    if (n <= 6) return { cols: 3, rows: 2 };
    return { cols: 3, rows: 3 };
}

/**
 * Indices of charts that sit at the bottom of their column in the current
 * layout — these are the ones that render the x-axis tick labels.
 *
 * The grid uses CSS `grid-auto-flow: column`, so visible cells fill
 * top-to-bottom in column 1, then column 2, and so on, in DOM (motor-
 * numeric) order.  For visible position k in a layout with `rows` per
 * column, the bottom-of-column slot is reached when k % rows == rows - 1
 * OR when k is the last visible chart (partial trailing column).
 */
function computeBottomOfColumnIds() {
    const visible = [];
    for (let i = 0; i < MOTOR_COUNT; i++) {
        if (visibleCharts.has(i)) visible.push(i);
    }
    const n = visible.length;
    if (n === 0) return new Set();
    const { rows } = gridShapeFor(n);
    const bottom = new Set();
    for (let k = 0; k < n; k++) {
        if (k % rows === rows - 1 || k === n - 1) bottom.add(visible[k]);
    }
    return bottom;
}

/**
 * Apply the grid template and hidden classes based on the current
 * visibleCharts set, then resize + repaint so uPlot adapts and the
 * freshly-unhidden charts immediately display their cached history.
 */
function applyChartLayout() {
    const n = visibleCharts.size;
    const { cols, rows } = gridShapeFor(n);

    const grid = document.getElementById('chart-grid');
    if (grid) {
        grid.style.gridTemplateColumns = `repeat(${cols}, 1fr)`;
        grid.style.gridTemplateRows = `repeat(${rows}, 1fr)`;
    }

    for (let i = 0; i < MOTOR_COUNT; i++) {
        const cell = document.getElementById(`chart-${i}`);
        if (!cell) continue;
        cell.classList.toggle('chart-hidden', !visibleCharts.has(i));
    }

    // Rebuild (rather than just resize) so the x-axis assignment refreshes:
    // the bottom-of-column chart changes whenever the visible set changes,
    // and the axis-tick `showXAxis` flag is baked into each uPlot at build
    // time.  rebuildAllCharts() also repaints from cache so newly-visible
    // cells render their history immediately.
    rebuildAllCharts();
}

/** References to the visibility pill buttons — populated by init, read by
 *  syncVisibilityButtonStates() whenever visibleCharts changes through a
 *  non-button path (e.g. keyboard solo). */
const visibilityButtons = [];

/** Italicize / grey the window-size select when liveWindowSec no longer
 *  matches the dropdown value — signals "wheel-zoomed; press R to snap
 *  back".  Pure cosmetic; doesn't mutate liveWindowSec. */
function syncWindowSelectState() {
    const select = document.getElementById('chart-window-select');
    if (!select) return;
    const current = parseInt(select.value, 10);
    const offPreset = !Number.isFinite(current) ||
        Math.abs(liveWindowSec - current) > 0.01;
    select.classList.toggle('off-preset', offPreset);
}

function syncVisibilityButtonStates() {
    const onlyOne = visibleCharts.size === 1;
    for (let i = 0; i < MOTOR_COUNT; i++) {
        const btn = visibilityButtons[i];
        if (!btn) continue;
        btn.classList.toggle('active', visibleCharts.has(i));
        btn.classList.toggle('disabled', onlyOne && visibleCharts.has(i));
    }
}

function initChartVisibilityToggles() {
    const container = document.getElementById('chart-visibility-toggles');
    if (!container) return;

    for (let i = 0; i < MOTOR_COUNT; i++) {
        const btn = document.createElement('button');
        btn.className = 'signal-toggle' + (visibleCharts.has(i) ? ' active' : '');
        btn.dataset.chart = String(i);
        btn.textContent = CHART_LABELS[i];
        btn.title = `Toggle ${CHART_LABELS[i]} chart visibility (click) · Shift-click or press ${i + 1} to solo`;

        btn.addEventListener('click', (ev) => {
            // Shift-click = solo this chart (same as pressing the number
            // key).  Double-solo returns to "show all".
            if (ev.shiftKey) {
                toggleSoloChart(i);
                return;
            }
            if (visibleCharts.has(i)) {
                // Enforce ≥1 visible chart.
                if (visibleCharts.size === 1) return;
                visibleCharts.delete(i);
            } else {
                visibleCharts.add(i);
            }
            saveChartVisibility();
            syncVisibilityButtonStates();
            applyChartLayout();
        });

        container.appendChild(btn);
        visibilityButtons.push(btn);
    }

    syncVisibilityButtonStates();
}

// ---- Time window selector ----

function initTimeWindowSelector() {
    const select = document.getElementById('chart-window-select');
    if (!select) return;

    select.value = currentWindowSec.toString();

    select.addEventListener('change', () => {
        currentWindowSec = parseInt(select.value, 10);
        liveWindowSec = currentWindowSec;
        saveTimeWindow();
        // Changing the window width is an explicit redefinition of the
        // visible slice — drop manual pan/zoom so the new width actually
        // takes effect.
        viewMode = 'live';
        manualXRange = null;
        syncWindowSelectState();
        rebuildAllCharts();
    });
}

// ---- Pause button ----

function applyPauseButtonUI() {
    const btn = document.getElementById('chart-pause-btn');
    if (!btn) return;
    if (paused) {
        btn.textContent = 'Resume';
        btn.classList.add('active');
        btn.style.setProperty('--signal-color', '#f59e0b');
    } else {
        btn.textContent = 'Pause';
        btn.classList.remove('active');
        btn.style.removeProperty('--signal-color');
    }
}

/**
 * Enter paused state.  No-op if already paused.  Snapshots the current
 * on-screen span so un-pause can resume at the same width.
 */
function pauseCharts() {
    if (paused) return;
    paused = true;
    const now = Date.now() / 1000;
    pausedWindowStart = now - liveWindowSec;
    pausedWindowEnd = now;
    applyPauseButtonUI();
}

/**
 * Leave paused state and snap to live tracking.  The zoom width
 * (liveWindowSec) is preserved — only explicit R-reset or dropdown changes
 * return the width to the dropdown preset.
 */
function resumeCharts() {
    if (!paused) return;
    paused = false;
    applyPauseButtonUI();
    viewMode = 'live';
    manualXRange = null;
    clearDeltaBookmarks();
    if (!pendingRepaint) {
        pendingRepaint = true;
        requestAnimationFrame(() => repaintAllCharts());
    }
}

function initPauseButton() {
    const container = document.getElementById('chart-time-window');
    if (!container) return;

    const btn = document.createElement('button');
    btn.id = 'chart-pause-btn';
    btn.className = 'signal-toggle';
    btn.textContent = 'Pause';
    btn.title = 'Pause/resume chart updates (shortcut: Space)';

    btn.addEventListener('click', () => {
        if (paused) resumeCharts();
        else pauseCharts();
    });

    container.insertBefore(btn, container.firstChild);
}

// ---- uPlot chart management ----

function getActiveSignalList() {
    return SIGNAL_GROUPS.filter(s => activeSignals.has(s.key));
}

/** Read a CSS custom property, falling back to a default if unset. */
function cssVar(name, fallback) {
    const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
    return v || fallback;
}

function buildUPlotOpts(width, height, showXAxis = true, onCursor = null) {
    const signalList = getActiveSignalList();

    // Chart surface colours — re-read each build so theme switches pick up
    // without needing a page reload.
    const axisStroke = cssVar('--chart-axis-stroke', '#94a3b8');
    const gridStroke = cssVar('--chart-grid-stroke', 'rgba(51,65,85,0.5)');
    const gridStrokeMinor = cssVar('--chart-grid-stroke-minor', 'rgba(51,65,85,0.3)');
    const ticksStroke = cssVar('--chart-ticks-stroke', '#334155');

    // Determine which scale groups are in use
    const usedScales = new Map(); // scaleKey → first signal's unit
    for (const sig of signalList) {
        if (!usedScales.has(sig.scale)) {
            usedScales.set(sig.scale, SCALE_META[sig.scale]?.unit || '');
        }
    }

    // Build scales — custom range function to keep axes tight to data
    const scales = { x: { time: true } };
    for (const scaleKey of usedScales.keys()) {
        scales[scaleKey] = {
            auto: true,
            range: (u, dataMin, dataMax) => {
                // When no data or all values identical, show a small range around the value
                if (dataMin == null || dataMax == null) return [0, 1];
                if (dataMin === dataMax) {
                    const v = dataMin;
                    const pad = Math.max(Math.abs(v) * 0.1, 0.5);
                    return [v - pad, v + pad];
                }
                // Add 5% padding above and below
                const span = dataMax - dataMin;
                const pad = span * 0.05;
                return [dataMin - pad, dataMax + pad];
            },
        };
    }

    // Build axes — alternate left (3) and right (1)
    const axes = [
        {
            // x-axis (time) — ISO 8601 formatted labels
            stroke: axisStroke,
            grid: { stroke: gridStroke, width: 1 },
            ticks: { stroke: showXAxis ? ticksStroke : 'transparent', width: 1, size: showXAxis ? 6 : 0 },
            font: '11px JetBrains Mono, monospace',
            size: showXAxis ? 28 : 4,
            values: showXAxis
                ? (u, splits) => splits.map(v => {
                    const d = new Date(v * 1000);
                    const hh = String(d.getHours()).padStart(2, '0');
                    const mm = String(d.getMinutes()).padStart(2, '0');
                    const ss = String(d.getSeconds()).padStart(2, '0');
                    return `${hh}:${mm}:${ss}`;
                })
                : (u, splits) => splits.map(() => ''),
        },
    ];

    let sideToggle = 3; // start left
    for (const [scaleKey, unit] of usedScales) {
        axes.push({
            scale: scaleKey,
            side: sideToggle,
            label: unit,
            labelSize: 16,
            size: 60,
            stroke: axisStroke,
            grid: { stroke: gridStrokeMinor, width: 1 },
            ticks: { stroke: ticksStroke, width: 1 },
            font: '14px JetBrains Mono, monospace',
            labelFont: '14px JetBrains Mono, monospace',
        });
        sideToggle = sideToggle === 3 ? 1 : 3;
    }

    // Build series (index 0 = time placeholder)
    const series = [{}];
    for (const sig of signalList) {
        series.push({
            label: sig.label,
            scale: sig.scale,
            stroke: sig.color,
            width: 1.5 * window.devicePixelRatio,
            dash: sig.dash,
            points: { show: false },
        });
    }

    return {
        width: Math.max(width, 60),
        height: Math.max(height, 40),
        series,
        scales,
        axes,
        cursor: {
            show: true,
            sync: { key: SYNC_KEY, setSeries: false },
            points: { show: false },
            // Enable LMB-drag x-selection for box-zoom, but with setScale off
            // so we can broadcast the new range to every chart via the
            // setSelect hook below (keeps all charts synced).
            drag: { x: true, y: false, setScale: false },
        },
        hooks: {
            setSelect: [(u) => {
                if (u.select.width < 3) return;  // ignore accidental clicks
                const min = u.posToVal(u.select.left, 'x');
                const max = u.posToVal(u.select.left + u.select.width, 'x');
                viewMode = 'manual';
                setXRangeAllCharts(min, max, true);
                // Clear the selection rectangle so it doesn't linger.
                u.setSelect({ left: 0, top: 0, width: 0, height: 0 }, false);
                // Chromium fires a `click` after mouseup even when the
                // pointer dragged — swallow that one so box-zoom doesn't
                // plant an A bookmark at the drag-release position.
                suppressNextClick = true;
            }],
            setCursor: onCursor ? [onCursor] : [],
            // Drawn after every series/axis paint — lets us overlay event
            // markers and delta-cursor bookmarks without touching the
            // plotted data.
            draw: [drawChartOverlays],
        },
        legend: { show: false },
        padding: [8, 8, 0, 0],
    };
}

function buildAllCharts() {
    const signalList = getActiveSignalList();
    const signalKeys = signalList.map(s => s.key);
    // Preserve manual pan/zoom across signal-toggle rebuilds.
    let windowStart, windowEnd;
    if (viewMode === 'manual' && manualXRange) {
        windowStart = manualXRange.min;
        windowEnd = manualXRange.max;
    } else {
        windowEnd = getViewAnchor();
        windowStart = windowEnd - liveWindowSec;
    }

    const bottomIds = computeBottomOfColumnIds();

    // Invalidate callout records — they'll be rebuilt alongside each chart.
    chartCallouts.length = 0;
    chartDeltaCallouts.length = 0;

    for (let i = 0; i < MOTOR_COUNT; i++) {
        const cell = document.getElementById(`chart-${i}`);
        if (!cell) continue;

        // Destroy existing
        if (charts[i]) {
            charts[i].destroy();
            charts[i] = null;
        }

        // Remove old uPlot wrapper (but keep title and no-data overlay)
        const oldWrap = cell.querySelector('.u-wrap');
        if (oldWrap) oldWrap.remove();

        const w = cell.clientWidth;
        const h = cell.clientHeight;
        if (w < 10 || h < 10) continue; // too small, skip

        // X-axis labels render only on the bottom chart in each column —
        // computed dynamically because the visible set (and therefore the
        // grid shape) changes at runtime.
        const isBottomRow = bottomIds.has(i);

        // Callout overlay — created now and captured by the setCursor hook.
        // Rebuilt every time buildAllCharts runs (so it stays in sync with
        // the active signal list).  One label per active series, anchored to
        // the vertical crosshair at the curve's Y value.
        const calloutRecord = createCalloutRecord(signalList);

        const opts = buildUPlotOpts(w, h, isBottomRow, (u) => {
            updateCallouts(u, calloutRecord);
        });

        // Initialise with real buffered data so we never flash empty after a
        // toggle / window change — falls back to empty arrays before stores exist.
        const initialData = stores[i]
            ? stores[i].getAlignedData(signalKeys, windowStart)
            : [new Float64Array(0), ...signalKeys.map(() => new Float64Array(0))];

        charts[i] = new uPlot(opts, initialData, cell);
        charts[i].setScale('x', { min: windowStart, max: windowEnd });
        attachMouseControls(charts[i]);

        // The overlay must live inside u.over so uPlot's valToPos CSS-pixel
        // coords line up (they're measured from the plot area top-left).
        charts[i].over.appendChild(calloutRecord.overlay);
        chartCallouts[i] = calloutRecord;

        // Delta-callout overlay — absolutely positioned, pinned top-right;
        // only visible once both delta bookmarks are placed.
        const deltaRecord = createDeltaCalloutRecord(signalList);
        charts[i].over.appendChild(deltaRecord.overlay);
        chartDeltaCallouts[i] = deltaRecord;

        // Hover → 3D highlight.  Attached to the uPlot over-layer (not the
        // cell) so the title overlay's pointer-events don't swallow events.
        attachHighlightHandlers(charts[i].over, i);
    }

    // A fresh build might have happened while bookmarks were still set
    // (e.g. signal toggle during pause).  Re-run the delta UI so the new
    // overlays show values instead of starting blank.
    refreshDeltaUI();
}

function rebuildAllCharts() {
    buildAllCharts();
    // User-driven rebuild: force repaint even if paused so the new UI
    // state reflects the cached buffer instead of a blank chart.
    repaintAllCharts(true);
}

/** Public entry point for triggering a chart rebuild (e.g. after un-collapsing). */
export function rebuildCharts() {
    rebuildAllCharts();
}

/**
 * Briefly pulse the border of chart cell `idx` — used by the click-in-3D
 * pick feature to confirm which chart corresponds to the clicked mesh.
 * If the chart is hidden, pulses the visibility pill instead so the user
 * knows where to find it.
 */
export function flashChart(idx) {
    if (idx < 0 || idx >= MOTOR_COUNT) return;
    const target = visibleCharts.has(idx)
        ? document.getElementById(`chart-${idx}`)
        : visibilityButtons[idx];
    if (!target) return;
    target.classList.remove('chart-pick-flash');
    // Force reflow so re-adding the class restarts the animation even
    // when two picks fire back-to-back.
    void target.offsetWidth;
    target.classList.add('chart-pick-flash');
    setTimeout(() => target.classList.remove('chart-pick-flash'), 900);
}

function resizeAllCharts() {
    let anyMissing = false;
    for (let i = 0; i < MOTOR_COUNT; i++) {
        const cell = document.getElementById(`chart-${i}`);
        if (!cell) continue;
        const w = cell.clientWidth;
        const h = cell.clientHeight;
        if (w < 10 || h < 10) continue;
        if (!charts[i]) {
            anyMissing = true;
            continue;
        }
        charts[i].setSize({ width: w, height: h });
    }
    // If charts were never created (e.g. panel was collapsed at init), build now
    if (anyMissing) rebuildAllCharts();
}

// ---- Data ingestion ----

/**
 * Called from main.js onRobotState with the full motor state array.
 * @param {object[]} motorStates  – robot_state.motor_states (up to 9)
 * @param {number[]|null} commandedLegs – leg_lengths_topic data (revs, indices 0-5) or null
 * @param {object|null} handTelemetry – HandTelemetryMessage or null
 */
export function onTelemetryData(motorStates, commandedLegs, handTelemetry) {
    if (!motorStates || motorStates.length === 0) return;

    const now = Date.now() / 1000; // uPlot uses seconds

    for (let i = 0; i < Math.min(motorStates.length, MOTOR_COUNT); i++) {
        const m = motorStates[i];
        const values = {
            pos_measured:  m.pos_estimate,
            vel_measured:  m.vel_estimate,
            iq_setpoint:   m.iq_setpoint,
            iq_measured:   m.iq_measured,
            fet_temp:      m.fet_temp,
            motor_temp:    m.motor_temp,
            bus_voltage:   m.bus_voltage,
            bus_current:   m.bus_current,
        };

        // Commanded position — legs from leg_lengths_topic, hand from hand_telemetry
        if (i < 6 && commandedLegs && commandedLegs.length > i) {
            values.pos_commanded = commandedLegs[i];
        } else if (i === 6 && handTelemetry) {
            values.pos_commanded = handTelemetry.pos_cmd;
        } else {
            values.pos_commanded = NaN;
        }

        stores[i].push(now, values);
    }

    // Schedule a coalesced repaint
    if (!pendingRepaint) {
        pendingRepaint = true;
        requestAnimationFrame(() => repaintAllCharts());
    }
}

/**
 * Pick the "latest visible time" for the x-axis.
 *   - While paused: the frozen edge captured at pause time.
 *   - While streaming: wall-clock now.
 *   - After the stream goes stale (>1 s without a sample, e.g. session ended):
 *     the last sample's timestamp, so data stays visible instead of walking off.
 */
function getViewAnchor() {
    if (paused) return pausedWindowEnd;
    const store = stores[0];
    if (!store || store.length === 0) return Date.now() / 1000;
    const lastT = store.timestamps[store.length - 1];
    const wall = Date.now() / 1000;
    return (wall - lastT) < 1.0 ? wall : lastT;
}

/** Clamp a requested [min,max] span to the cache window and min-span guard. */
function clampXRange(min, max) {
    let span = max - min;
    if (span < MIN_X_SPAN_SEC) {
        const mid = (min + max) / 2;
        min = mid - MIN_X_SPAN_SEC / 2;
        max = mid + MIN_X_SPAN_SEC / 2;
        span = MIN_X_SPAN_SEC;
    }
    if (span > MAX_X_SPAN_SEC) {
        const mid = (min + max) / 2;
        min = mid - MAX_X_SPAN_SEC / 2;
        max = mid + MAX_X_SPAN_SEC / 2;
    }
    return { min, max };
}

/**
 * Apply the same x-axis range to every chart, so pan/zoom stays synced.
 *
 * Direction matters:
 *   - If the requested range reaches or overshoots wall-clock now, the
 *     user has panned forward to (or past) the live edge — snap back to
 *     live tracking at whatever span they're at, and auto-resume if paused.
 *   - Otherwise they're viewing history — enter manual mode and auto-pause
 *     so the stream doesn't keep drifting past the frozen snapshot.  The
 *     store is re-sliced to fill the new range (repaintAllCharts is a
 *     no-op while paused).
 */
function setXRangeAllCharts(min, max, fromSelection = false) {
    const clamped = clampXRange(min, max);
    const wallNow = Date.now() / 1000;
    const FUTURE_EPSILON = 0.1;

    // Box-zoom selections honour the user's exact range — skip the live-
    // snap branch even if their rightmost drag coord landed near now.
    // Wheel-zoom and middle-drag pan keep the snap so panning forward past
    // the live edge still resumes live tracking.
    if (!fromSelection && clamped.max >= wallNow - FUTURE_EPSILON) {
        // Caught up to (or overshot) live — return to live tracking.
        const span = Math.max(
            MIN_X_SPAN_SEC,
            Math.min(MAX_X_SPAN_SEC, clamped.max - clamped.min),
        );
        liveWindowSec = span;
        manualXRange = null;
        viewMode = 'live';
        syncWindowSelectState();
        if (paused) resumeCharts();
        else if (!pendingRepaint) {
            pendingRepaint = true;
            requestAnimationFrame(() => repaintAllCharts());
        }
        return;
    }

    // Historical view — freeze it.
    manualXRange = clamped;
    viewMode = 'manual';
    for (let i = 0; i < MOTOR_COUNT; i++) {
        const c = charts[i];
        if (!c) continue;
        c.setScale('x', clamped);
    }
    if (!paused) pauseCharts();
    reloadChartDataFromStores(clamped.min);
}

/**
 * Push fresh data slices from the stores to every chart for the given
 * start timestamp, bypassing the paused-repaint guard.  Used whenever the
 * visible x range changes while paused (zoom / pan) so the newly-visible
 * region actually shows curves, not just event markers.
 */
function reloadChartDataFromStores(windowStart) {
    const signalKeys = getActiveSignalList().map(s => s.key);
    for (let i = 0; i < MOTOR_COUNT; i++) {
        if (!charts[i] || !stores[i]) continue;
        const data = stores[i].getAlignedData(signalKeys, windowStart);
        charts[i].setData(data, false);
    }
}

/** Drop out of manual view mode and snap back to the live-anchored window
 *  at the dropdown preset width (discarding any wheel-zoom-driven widening). */
function resetViewToLive() {
    viewMode = 'live';
    manualXRange = null;
    liveWindowSec = currentWindowSec;
    syncWindowSelectState();
    const anchor = getViewAnchor();
    const windowStart = anchor - liveWindowSec;
    for (let i = 0; i < MOTOR_COUNT; i++) {
        if (!charts[i]) continue;
        charts[i].setScale('x', { min: windowStart, max: anchor });
    }
    // Force a repaint so data re-aligns to the live window immediately.
    repaintAllCharts(true);
}

/**
 * Attach wheel-zoom, middle-click-pan, and dblclick-reset to a uPlot instance.
 * All handlers set viewMode='manual' and delegate to setXRangeAllCharts() so
 * every chart moves together.
 */
function attachMouseControls(u) {
    const over = u.over;   // the event-capture layer inside uPlot
    if (!over) return;

    // --- Wheel: zoom x-axis ---------------------------------------------
    // Two distinct behaviours so the stream never appears to freeze:
    //   - Live + streaming: widen/narrow the tracked window width without
    //     leaving live mode.  Right edge keeps following the anchor.
    //   - Paused or already-manual: cursor-anchored zoom that flips (or
    //     stays) in manual mode.  reloadChartDataFromStores() in
    //     setXRangeAllCharts handles the paused-data-gap case.
    over.addEventListener('wheel', (ev) => {
        ev.preventDefault();
        const scale = u.scales.x;
        if (scale.min == null || scale.max == null) return;

        const factor = Math.pow(WHEEL_ZOOM_STEP, ev.deltaY / 100);

        if (viewMode === 'live' && !paused) {
            const currentSpan = scale.max - scale.min;
            const newSpan = currentSpan * factor;
            liveWindowSec = Math.max(MIN_X_SPAN_SEC,
                                     Math.min(MAX_X_SPAN_SEC, newSpan));
            syncWindowSelectState();
            // Immediate repaint so the zoom feels instant — next telemetry
            // frame would otherwise take up to 50 ms to land.
            if (!pendingRepaint) {
                pendingRepaint = true;
                requestAnimationFrame(() => repaintAllCharts());
            }
        } else {
            const rect = over.getBoundingClientRect();
            const px = ev.clientX - rect.left;
            const cursorT = u.posToVal(px, 'x');
            const newMin = cursorT - (cursorT - scale.min) * factor;
            const newMax = cursorT + (scale.max - cursorT) * factor;
            viewMode = 'manual';
            setXRangeAllCharts(newMin, newMax);
        }
    }, { passive: false });

    // --- Middle-click drag: pan x-axis ----------------------------------
    let panStart = null;  // { px, min, max }

    over.addEventListener('mousedown', (ev) => {
        if (ev.button !== 1) return;  // middle button only
        ev.preventDefault();
        const scale = u.scales.x;
        if (scale.min == null || scale.max == null) return;
        const rect = over.getBoundingClientRect();
        panStart = {
            px: ev.clientX - rect.left,
            min: scale.min,
            max: scale.max,
            width: rect.width,
        };
        viewMode = 'manual';
    });

    window.addEventListener('mousemove', (ev) => {
        if (!panStart) return;
        const rect = over.getBoundingClientRect();
        const dxPx = (ev.clientX - rect.left) - panStart.px;
        const span = panStart.max - panStart.min;
        const dxT = -(dxPx / panStart.width) * span;
        setXRangeAllCharts(panStart.min + dxT, panStart.max + dxT);
    });

    window.addEventListener('mouseup', (ev) => {
        if (ev.button === 1) panStart = null;
    });

    // Block the browser's middle-click autoscroll bubble inside the chart.
    over.addEventListener('auxclick', (ev) => {
        if (ev.button === 1) ev.preventDefault();
    });

    // --- Double-click: reset to live view -------------------------------
    over.addEventListener('dblclick', (ev) => {
        ev.preventDefault();
        resetViewToLive();
    });

    // --- Click: place / clear delta-cursor bookmarks (paused only) ------
    // uPlot fires its own pointerdown for box-zoom, but `click` only fires
    // when the pointer didn't move significantly — so drag-to-zoom and
    // click-to-bookmark don't fight.
    over.addEventListener('click', (ev) => {
        if (ev.button !== 0) return;  // LMB only
        if (suppressNextClick) {
            suppressNextClick = false;
            return;
        }
        handleDeltaCursorClick(u, ev.clientX, ev.clientY);
    });
}

// ---- Canvas overlays: event markers + delta-cursor bookmarks ------------

/**
 * Delta-cursor bookmarks — active only while the chart panel is paused.
 * `deltaA` is placed on the first click, `deltaB` on the second.  A third
 * click clears them.  Values are timestamps in seconds since epoch.
 */
let deltaA = null;
let deltaB = null;
/** Set by the setSelect (box-zoom) hook so the click event Chromium fires
 *  on the same mouseup doesn't plant a spurious A bookmark. */
let suppressNextClick = false;

/** Force every chart to redraw its canvas so the draw hook re-runs.  Both
 *  flags false means "re-paint without rebuilding series paths or axes" —
 *  the cheapest redraw uPlot offers, which is all we need for overlay
 *  changes (event markers / delta bookmarks). */
function redrawAllOverlays() {
    for (let i = 0; i < MOTOR_COUNT; i++) {
        const c = charts[i];
        if (c) c.redraw(false, false);
    }
    refreshDeltaUI();
}

/**
 * Draw hook — invoked by uPlot after its own series/axis paint.  We draw:
 *   1. Event markers: faint vertical lines at each in-range event timestamp.
 *   2. Delta-cursor bookmarks: bright vertical lines (A cyan, B cyan dashed)
 *      with the A/B labels at the top of the plot area.
 *
 * Everything is in canvas pixels — u.valToPos with `canvas=true` returns
 * the right coord system for the 2D ctx.
 */
function drawChartOverlays(u) {
    const ctx = u.ctx;
    if (!ctx || !u.bbox) return;
    const { left, top, width, height } = u.bbox;
    const xMin = u.scales.x.min;
    const xMax = u.scales.x.max;
    if (xMin == null || xMax == null) return;

    ctx.save();
    // Clip to the plot area so marker lines don't bleed into tick labels.
    ctx.beginPath();
    ctx.rect(left, top, width, height);
    ctx.clip();

    // ---- Event markers ----
    // One pass: un-highlighted events drawn faint, the highlighted one drawn
    // thicker + fully opaque on top so it pops against a marker forest.
    const evs = getEventsInRange(xMin, xMax);
    const hlId = getHighlightedEventId();
    let hlEvent = null;
    ctx.lineWidth = 1 * devicePixelRatio;
    ctx.globalAlpha = 0.55;
    for (const ev of evs) {
        if (ev.id === hlId) { hlEvent = ev; continue; }
        const xPx = u.valToPos(ev.t, 'x', true);
        ctx.strokeStyle = EVENT_COLORS[ev.type] || '#94a3b8';
        ctx.beginPath();
        ctx.moveTo(xPx, top);
        ctx.lineTo(xPx, top + height);
        ctx.stroke();
    }
    ctx.globalAlpha = 1;

    if (hlEvent) {
        const xPx = u.valToPos(hlEvent.t, 'x', true);
        const color = EVENT_COLORS[hlEvent.type] || '#94a3b8';
        ctx.strokeStyle = color;
        ctx.lineWidth = 2.5 * devicePixelRatio;
        ctx.beginPath();
        ctx.moveTo(xPx, top);
        ctx.lineTo(xPx, top + height);
        ctx.stroke();

        // Label tag at the top, colour-matched to event type.
        const tagH = 14 * devicePixelRatio;
        const padX = 4 * devicePixelRatio;
        ctx.font = `${10 * devicePixelRatio}px JetBrains Mono, monospace`;
        const tagW = Math.ceil(ctx.measureText(hlEvent.label).width) + padX * 2;
        // Skip the tag when it can't fit inside the plot width — the clip
        // region would otherwise paint a one-sided truncation.  The marker
        // line alone is enough to identify the event; the full label is
        // visible in the history panel row being hovered.
        if (tagW <= width) {
            // Keep the tag inside the plot area so it's readable even when
            // the marker is near the edge.
            let tagX = xPx - tagW / 2;
            if (tagX < left) tagX = left;
            if (tagX + tagW > left + width) tagX = left + width - tagW;
            ctx.fillStyle = color;
            ctx.fillRect(tagX, top, tagW, tagH);
            ctx.fillStyle = '#0f172a';
            ctx.textAlign = 'left';
            ctx.textBaseline = 'middle';
            ctx.fillText(hlEvent.label, tagX + padX, top + tagH / 2);
        }
    }

    // ---- Delta-cursor bookmarks ----
    // Drawn brighter and with a little A/B tag so they stand out from the
    // event-marker forest.
    const bookmarkColor = '#e0f2fe';  // near-white cyan — visible in both themes
    ctx.lineWidth = 1.5 * devicePixelRatio;

    const drawBookmark = (t, tag, dashed) => {
        if (t == null || t < xMin || t > xMax) return;
        const xPx = u.valToPos(t, 'x', true);
        ctx.strokeStyle = bookmarkColor;
        ctx.setLineDash(dashed ? [4 * devicePixelRatio, 3 * devicePixelRatio] : []);
        ctx.beginPath();
        ctx.moveTo(xPx, top);
        ctx.lineTo(xPx, top + height);
        ctx.stroke();
        ctx.setLineDash([]);

        // A/B tag
        const tagW = 14 * devicePixelRatio;
        const tagH = 14 * devicePixelRatio;
        ctx.fillStyle = bookmarkColor;
        ctx.fillRect(xPx - tagW / 2, top, tagW, tagH);
        ctx.fillStyle = '#0f172a';
        ctx.font = `${10 * devicePixelRatio}px JetBrains Mono, monospace`;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(tag, xPx, top + tagH / 2);
    };

    drawBookmark(deltaA, 'A', false);
    drawBookmark(deltaB, 'B', true);

    ctx.restore();
}

/**
 * Route an in-paused click on any chart to the delta-bookmark state
 * machine.  Three clicks = place A, place B, clear.
 */
function handleDeltaCursorClick(u, clientX, clientY) {
    if (!paused) return;  // bookmarks only valid while paused
    const rect = u.over.getBoundingClientRect();
    const px = clientX - rect.left;
    const t = u.posToVal(px, 'x');
    if (!Number.isFinite(t)) return;

    if (deltaA == null) {
        deltaA = t;
    } else if (deltaB == null) {
        deltaB = t;
        // Ensure A <= B so Δt stays positive regardless of click order.
        if (deltaB < deltaA) {
            const tmp = deltaA; deltaA = deltaB; deltaB = tmp;
        }
    } else {
        deltaA = null;
        deltaB = null;
    }
    redrawAllOverlays();
}

/** Binary search: smallest index into a sorted timestamps array whose value
 *  is >= t.  Returns timestamps.length if nothing qualifies. */
function nearestTsIndex(timestamps, t) {
    let lo = 0, hi = timestamps.length;
    while (lo < hi) {
        const mid = (lo + hi) >>> 1;
        if (timestamps[mid] < t) lo = mid + 1;
        else hi = mid;
    }
    // Clamp — caller usually wants the nearest valid index, and the typed-
    // array view can be empty.
    return Math.min(Math.max(lo, 0), Math.max(0, timestamps.length - 1));
}

/**
 * Create the per-chart delta-callout overlay DOM.  Lives inside u.over so
 * absolute positioning lines up with the plot area.  One pill per active
 * series, all hidden until both bookmarks are set.
 */
function createDeltaCalloutRecord(signalList) {
    const overlay = document.createElement('div');
    overlay.className = 'chart-delta-callouts';

    const header = document.createElement('div');
    header.className = 'chart-delta-header';
    header.textContent = '\u0394 (B \u2212 A)';
    overlay.appendChild(header);

    const pills = signalList.map(sig => {
        const el = document.createElement('div');
        el.className = 'chart-delta-pill';
        el.style.setProperty('--signal-color', sig.color);

        const dot = document.createElement('span');
        dot.className = 'chart-delta-dot';
        el.appendChild(dot);

        const label = document.createElement('span');
        label.className = 'chart-delta-pill-label';
        label.textContent = sig.label;
        el.appendChild(label);

        const value = document.createElement('span');
        value.className = 'chart-delta-pill-value';
        el.appendChild(value);

        overlay.appendChild(el);
        return { el, textNode: value, sig };
    });

    return { overlay, pills };
}

/**
 * Recompute the per-chart Δy pills for every chart.  Called from
 * refreshDeltaUI whenever the bookmark state changes (and after chart
 * rebuilds while bookmarks are still live).
 */
function refreshChartDeltaCallouts() {
    const haveBoth = deltaA != null && deltaB != null;
    for (let i = 0; i < MOTOR_COUNT; i++) {
        const chart = charts[i];
        const record = chartDeltaCallouts[i];
        if (!record) continue;
        if (!chart || !haveBoth) {
            record.overlay.classList.remove('active');
            continue;
        }

        const data = chart.data;
        const ts = data && data[0];
        if (!ts || ts.length === 0) {
            record.overlay.classList.remove('active');
            continue;
        }
        // Hide the pill when either bookmark is outside the visible x range
        // — otherwise nearestTsIndex would clamp silently and the pill would
        // display vB − v[first_visible_sample] instead of true vB − vA.
        // Matches the bookmark-line culling in drawChartOverlays.
        const xMin = chart.scales.x.min;
        const xMax = chart.scales.x.max;
        if (xMin == null || xMax == null ||
            deltaA < xMin || deltaA > xMax ||
            deltaB < xMin || deltaB > xMax) {
            record.overlay.classList.remove('active');
            continue;
        }
        const idxA = nearestTsIndex(ts, deltaA);
        const idxB = nearestTsIndex(ts, deltaB);

        // uPlot's data is [timestamps, series0, series1, …] so series j sits
        // at column j+1.  Pills are built in the same order as signalList.
        for (let j = 0; j < record.pills.length; j++) {
            const { el, textNode, sig } = record.pills[j];
            const vA = data[j + 1] ? data[j + 1][idxA] : undefined;
            const vB = data[j + 1] ? data[j + 1][idxB] : undefined;
            if (vA == null || vB == null || !Number.isFinite(vA) || !Number.isFinite(vB)) {
                el.style.display = 'none';
                continue;
            }
            el.style.display = '';
            const dv = vB - vA;
            const sign = dv >= 0 ? '+' : '\u2212';
            const absStr = formatCalloutValue(Math.abs(dv), sig.scale);
            textNode.textContent = `${sign}${absStr}`;
            textNode.classList.toggle('positive', dv >= 0);
            textNode.classList.toggle('negative', dv < 0);
        }
        record.overlay.classList.add('active');
    }
}

/**
 * Update the Δt toolbar badge AND the per-chart Δy pills.  Single entry
 * point so bookmark state and UI stay in lock-step.
 */
function refreshDeltaUI() {
    const el = document.getElementById('chart-delta-readout');
    if (el) {
        if (deltaA != null && deltaB != null) {
            el.textContent = `\u0394t = ${(deltaB - deltaA).toFixed(3)} s`;
            el.classList.add('active');
        } else if (deltaA != null) {
            el.textContent = '\u0394t: click B\u2026';
            el.classList.add('active');
        } else {
            el.textContent = '';
            el.classList.remove('active');
        }
    }
    refreshChartDeltaCallouts();
}

/** Hide bookmarks and clear the readout — called whenever pause is released. */
function clearDeltaBookmarks() {
    if (deltaA == null && deltaB == null) return;
    deltaA = null;
    deltaB = null;
    redrawAllOverlays();
}

// ---- CSV export ---------------------------------------------------------

/**
 * Build an ISO 8601 basic-format timestamp suitable for a filename — colons
 * swapped for dashes so it's portable across file systems, fractional
 * seconds stripped.  Example: 2026-04-21T14-32-05.
 */
function isoFilenameStamp() {
    return new Date().toISOString().replace(/:/g, '-').replace(/\..+$/, '');
}

/**
 * Dump the current contents of every per-motor cache to one wide CSV
 * (timestamp + one column per signal × motor).  Drives a browser download
 * via an object-URL Blob, filename stamped with the local ISO time.
 *
 * All stores are fed in lock-step from onTelemetryData, so they share the
 * master timestamp array from stores[0].
 */
function exportTelemetryCSV() {
    const master = stores[0];
    if (!master || master.length === 0) {
        alert('No telemetry data to export yet.');
        return;
    }

    const header = ['timestamp_s'];
    for (let i = 0; i < MOTOR_COUNT; i++) {
        const motor = CHART_LABELS[i].replace(/\s+/g, '_').toLowerCase();
        for (const sg of SIGNAL_GROUPS) {
            header.push(`${motor}.${sg.key}`);
        }
    }

    const rows = [header.join(',')];
    const N = master.length;
    for (let k = 0; k < N; k++) {
        const cells = [master.timestamps[k].toFixed(3)];
        for (let i = 0; i < MOTOR_COUNT; i++) {
            const store = stores[i];
            for (const sg of SIGNAL_GROUPS) {
                const col = store && store.columns[sg.key];
                const v = col ? col[k] : NaN;
                cells.push(Number.isFinite(v) ? v.toFixed(6) : '');
            }
        }
        rows.push(cells.join(','));
    }

    const blob = new Blob([rows.join('\n')], { type: 'text/csv;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `jugglebot-telemetry-${isoFilenameStamp()}.csv`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    // Revoke on the next tick — Safari needs the URL to still resolve during
    // click dispatch, so give it a frame before we free it.
    requestAnimationFrame(() => URL.revokeObjectURL(url));
}

function initExportButton() {
    const container = document.getElementById('chart-time-window');
    if (!container) return;
    const btn = document.createElement('button');
    btn.id = 'chart-export-btn';
    btn.className = 'signal-toggle';
    btn.textContent = 'Export CSV';
    btn.title = 'Download the cached telemetry window as a CSV file';
    btn.addEventListener('click', exportTelemetryCSV);
    // Slot it just before the pause button (first-child) — left-to-right
    // reading order becomes: Export, Pause, Window.
    container.insertBefore(btn, container.firstChild);
}

// ---- Keyboard shortcuts -------------------------------------------------

/**
 * Return true if the focused element is something the user is typing into —
 * we suppress chart shortcuts in that case so they don't hijack inputs.
 */
function isTypingTarget(el) {
    if (!el) return false;
    const tag = el.tagName;
    return tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || el.isContentEditable;
}

/** Toggle the pause button by replaying its click — keeps state in one place. */
function togglePauseShortcut() {
    document.getElementById('chart-pause-btn')?.click();
}

/**
 * Step through the window-size dropdown by `dir` positions (±1).  Done
 * through the <select> + change event so we reuse the existing persistence
 * and viewMode-reset behaviour.
 */
function cycleWindowSize(dir) {
    const select = document.getElementById('chart-window-select');
    if (!select) return;
    const options = Array.from(select.options).map(o => o.value);
    let idx = options.indexOf(select.value);
    if (idx < 0) idx = 0;
    idx = Math.max(0, Math.min(options.length - 1, idx + dir));
    if (options[idx] === select.value) return;
    select.value = options[idx];
    select.dispatchEvent(new Event('change'));
}

/**
 * Solo chart `idx`: hide every other chart.  If `idx` is already the only
 * visible chart, restore the full 9-chart grid instead.
 *
 * Deliberately stateless — no snapshot to drift out of sync with manual
 * pill clicks.  The trade-off is that toggling off always returns to
 * "show all", not to whatever subset the user had before soloing.
 */
function toggleSoloChart(idx) {
    if (idx < 0 || idx >= MOTOR_COUNT) return;
    const currentlySoloed = visibleCharts.size === 1 && visibleCharts.has(idx);
    if (currentlySoloed) {
        visibleCharts = new Set(Array.from({ length: MOTOR_COUNT }, (_, i) => i));
    } else {
        visibleCharts = new Set([idx]);
    }
    saveChartVisibility();
    syncVisibilityButtonStates();
    applyChartLayout();
}

function initKeyboardShortcuts() {
    document.addEventListener('keydown', (ev) => {
        // Never steal keys from real inputs, and leave modifier combos alone
        // (browser shortcuts like Ctrl+R reload, etc).
        if (isTypingTarget(document.activeElement)) return;
        if (ev.ctrlKey || ev.metaKey || ev.altKey) return;

        // Skip if the chart panel is collapsed — shortcuts would be confusing
        // when the target UI isn't visible.
        const panel = document.getElementById('chart-panel');
        if (panel && panel.classList.contains('collapsed')) return;

        switch (ev.key) {
            case ' ':
                ev.preventDefault();
                togglePauseShortcut();
                return;
            case '[':
                ev.preventDefault();
                cycleWindowSize(-1);
                return;
            case ']':
                ev.preventDefault();
                cycleWindowSize(+1);
                return;
            case 'r':
            case 'R':
                ev.preventDefault();
                resetViewToLive();
                return;
        }
        if (/^[1-9]$/.test(ev.key)) {
            ev.preventDefault();
            toggleSoloChart(parseInt(ev.key, 10) - 1);
        }
    });
}

// ---- Per-curve value callouts -------------------------------------------

/**
 * Build the DOM scaffolding for one chart's callout overlay — a positioned
 * container plus one pill per active signal.  The pills are absolutely
 * positioned on setCursor via updateCallouts().
 *
 * Returned record is owned by chartCallouts[i] and replaced whenever the
 * chart is rebuilt (active signal set changed, layout changed, etc).
 */
function createCalloutRecord(signalList) {
    const overlay = document.createElement('div');
    overlay.className = 'chart-callouts';

    const pills = signalList.map(sig => {
        const el = document.createElement('div');
        el.className = 'chart-callout';
        el.style.setProperty('--signal-color', sig.color);
        el.innerHTML = '<span class="chart-callout-dot"></span><span class="chart-callout-text"></span>';
        el.style.display = 'none';
        el.title = `${sig.label} \u2014 click to copy value`;

        // Click-to-copy: copies the raw numeric value (no unit) so it can be
        // pasted straight into a script or calculator.  A brief .copied class
        // flashes the pill green as confirmation.
        el.addEventListener('click', async (ev) => {
            ev.stopPropagation();
            ev.preventDefault();
            const raw = el.dataset.rawValue;
            if (raw == null) return;
            try {
                await navigator.clipboard.writeText(raw);
                el.classList.add('copied');
                setTimeout(() => el.classList.remove('copied'), 450);
            } catch {
                // Clipboard can fail in unfocused windows / insecure contexts.
                // Silent fail is fine here — the user will see no confirmation
                // flash and can retry.
            }
        });

        overlay.appendChild(el);
        return { el, textNode: el.querySelector('.chart-callout-text'), sig };
    });

    return { overlay, pills, signalList };
}

function formatCalloutValue(v, scale) {
    const meta = SCALE_META[scale] || {};
    const decimals = meta.decimals ?? 2;
    const unit = meta.unit || '';
    return `${v.toFixed(decimals)}${unit ? ' ' + unit : ''}`;
}

/**
 * Position the per-curve callouts at the current crosshair x.  uPlot fires
 * setCursor on every mouse move (and on cursor sync from peer charts), so
 * this runs at ~60 Hz while the user drags across the grid.
 */
function updateCallouts(u, record) {
    if (!record) return;
    const { overlay, pills } = record;
    const idx = u.cursor.idx;
    const data = u.data;
    if (idx == null || idx < 0 || !data || !data[0] || idx >= data[0].length) {
        overlay.style.opacity = '0';
        return;
    }
    overlay.style.opacity = '1';

    const t = data[0][idx];
    const xPx = u.valToPos(t, 'x');
    const plotW = u.bbox ? (u.bbox.width / devicePixelRatio) : u.over.clientWidth;

    // Flip callout side when near the right edge so labels stay readable.
    const flipLeft = xPx > plotW * 0.75;

    for (let j = 0; j < pills.length; j++) {
        const { el, textNode, sig } = pills[j];
        const val = data[j + 1] ? data[j + 1][idx] : undefined;
        if (val == null || !Number.isFinite(val)) {
            el.style.display = 'none';
            continue;
        }
        const yPx = u.valToPos(val, sig.scale);
        if (!Number.isFinite(yPx)) {
            el.style.display = 'none';
            continue;
        }
        el.style.display = '';
        el.style.left = `${xPx}px`;
        el.style.top = `${yPx}px`;
        el.classList.toggle('flip-left', flipLeft);
        textNode.textContent = formatCalloutValue(val, sig.scale);
        // Raw numeric (no unit) for click-to-copy.  Kept at full precision so
        // copy-paste into a script preserves everything we have.
        el.dataset.rawValue = String(val);
    }
}

// ---- Chart-cell hover → 3D scene highlight ------------------------------

/**
 * Map a chart index (0..8) to the (subsystem, target) pair understood by the
 * two 3D model modules.  Matches the CHART_LABELS layout.
 */
function highlightTargetFor(chartIdx) {
    if (chartIdx >= 0 && chartIdx <= 5) return { subsystem: 'stewart', target: `leg${chartIdx}` };
    if (chartIdx === 6) return { subsystem: 'stewart', target: 'hand' };
    if (chartIdx === 7) return { subsystem: 'bb', target: 'pitch' };
    if (chartIdx === 8) return { subsystem: 'bb', target: 'hand' };
    return null;
}

function applyHighlight(chartIdx) {
    const h = highlightTargetFor(chartIdx);
    // Always clear both subsystems first so hovering between a Stewart and BB
    // chart doesn't leave a stale glow on whichever side we just left.
    setStewartHighlight(null);
    setBallButlerHighlight(null);
    if (!h) return;
    if (h.subsystem === 'stewart') setStewartHighlight(h.target);
    else if (h.subsystem === 'bb') setBallButlerHighlight(h.target);
}

function clearHighlight() {
    setStewartHighlight(null);
    setBallButlerHighlight(null);
}

function attachHighlightHandlers(el, chartIdx) {
    el.addEventListener('mouseenter', () => applyHighlight(chartIdx));
    el.addEventListener('mouseleave', clearHighlight);
}

function repaintAllCharts(force = false) {
    pendingRepaint = false;

    // Skip repaint if panel is collapsed, or paused unless explicitly forced
    // (user-driven rebuilds must reflect UI state even while paused).
    const panel = document.getElementById('chart-panel');
    if (panel && panel.classList.contains('collapsed')) return;
    if (paused && !force) return;

    const signalList = getActiveSignalList();
    if (signalList.length === 0) return;

    const signalKeys = signalList.map(s => s.key);

    // Pick the visible window: manual pan/zoom wins over the live anchor.
    let windowStart, windowEnd;
    if (viewMode === 'manual' && manualXRange) {
        windowStart = manualXRange.min;
        windowEnd = manualXRange.max;
    } else {
        windowEnd = getViewAnchor();
        windowStart = windowEnd - liveWindowSec;
    }

    for (let i = 0; i < MOTOR_COUNT; i++) {
        if (!charts[i] || !stores[i]) continue;
        const data = stores[i].getAlignedData(signalKeys, windowStart);
        // In live mode we drive the x-scale every frame; in manual mode the
        // user owns the scale (already set by setXRangeAllCharts) — don't
        // fight them.
        if (viewMode === 'live') {
            charts[i].setScale('x', { min: windowStart, max: windowEnd });
        }
        charts[i].setData(data, false);
    }
}
