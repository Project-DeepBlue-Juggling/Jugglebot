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

// ---- State ----

const SIGNALS_STORAGE_KEY    = 'jugglebot-chart-signals';
const WINDOW_STORAGE_KEY     = 'jugglebot-chart-window';
const VISIBILITY_STORAGE_KEY = 'jugglebot-chart-visibility';
const DEFAULT_SIGNALS      = ['pos_measured', 'vel_measured'];
const DEFAULT_WINDOW_SEC   = 10;
const DATA_RATE_HZ         = 20;
// Retain 5 min of history regardless of the display window, so paused / ended
// sessions keep their data when the user adjusts the view.
const CACHE_WINDOW_SEC     = 300;

let activeSignals = new Set();
let currentWindowSec = DEFAULT_WINDOW_SEC;
let maxPoints = 0;
/** Set of visible chart indices (0..8).  Defaults to all visible. */
let visibleCharts = new Set();

/** Per-motor data stores */
const stores = [];
/** Per-motor uPlot instances */
const charts = [];
/** Per-chart callout metadata — populated during buildAllCharts. */
const chartCallouts = [];
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
    addChartTitles();
    applyChartLayout();  // apply grid-template + hidden classes before building
    buildAllCharts();

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

function initChartVisibilityToggles() {
    const container = document.getElementById('chart-visibility-toggles');
    if (!container) return;

    const buttons = [];

    const refreshDisabledState = () => {
        const onlyOne = visibleCharts.size === 1;
        for (let i = 0; i < MOTOR_COUNT; i++) {
            const btn = buttons[i];
            if (!btn) continue;
            const isOnlyActive = onlyOne && visibleCharts.has(i);
            btn.classList.toggle('disabled', isOnlyActive);
        }
    };

    for (let i = 0; i < MOTOR_COUNT; i++) {
        const btn = document.createElement('button');
        btn.className = 'signal-toggle' + (visibleCharts.has(i) ? ' active' : '');
        btn.dataset.chart = String(i);
        btn.textContent = CHART_LABELS[i];
        btn.title = `Toggle ${CHART_LABELS[i]} chart visibility`;

        btn.addEventListener('click', () => {
            if (visibleCharts.has(i)) {
                // Enforce ≥1 visible chart.
                if (visibleCharts.size === 1) return;
                visibleCharts.delete(i);
                btn.classList.remove('active');
            } else {
                visibleCharts.add(i);
                btn.classList.add('active');
            }
            saveChartVisibility();
            refreshDisabledState();
            applyChartLayout();
        });

        container.appendChild(btn);
        buttons.push(btn);
    }

    refreshDisabledState();
}

// ---- Time window selector ----

function initTimeWindowSelector() {
    const select = document.getElementById('chart-window-select');
    if (!select) return;

    select.value = currentWindowSec.toString();

    select.addEventListener('change', () => {
        currentWindowSec = parseInt(select.value, 10);
        saveTimeWindow();
        // Changing the window width is an explicit redefinition of the
        // visible slice — drop manual pan/zoom so the new width actually
        // takes effect.
        viewMode = 'live';
        manualXRange = null;
        rebuildAllCharts();
    });
}

// ---- Pause button ----

function initPauseButton() {
    const container = document.getElementById('chart-time-window');
    if (!container) return;

    const btn = document.createElement('button');
    btn.id = 'chart-pause-btn';
    btn.className = 'signal-toggle';
    btn.textContent = 'Pause';
    btn.title = 'Pause/resume chart updates';

    btn.addEventListener('click', () => {
        paused = !paused;
        if (paused) {
            const now = Date.now() / 1000;
            pausedWindowStart = now - currentWindowSec;
            pausedWindowEnd = now;
            btn.textContent = 'Resume';
            btn.classList.add('active');
            btn.style.setProperty('--signal-color', '#f59e0b');
        } else {
            btn.textContent = 'Pause';
            btn.classList.remove('active');
            btn.style.removeProperty('--signal-color');
            // Snap to live — exit manual pan/zoom too.
            viewMode = 'live';
            manualXRange = null;
            if (!pendingRepaint) {
                pendingRepaint = true;
                requestAnimationFrame(() => repaintAllCharts());
            }
        }
    });

    container.insertBefore(btn, container.firstChild);
}

// ---- uPlot chart management ----

function getActiveSignalList() {
    return SIGNAL_GROUPS.filter(s => activeSignals.has(s.key));
}

function buildUPlotOpts(width, height, showXAxis = true, onCursor = null) {
    const signalList = getActiveSignalList();

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
            stroke: '#94a3b8',
            grid: { stroke: 'rgba(51,65,85,0.5)', width: 1 },
            ticks: { stroke: showXAxis ? '#334155' : 'transparent', width: 1, size: showXAxis ? 6 : 0 },
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
            stroke: '#94a3b8',
            grid: { stroke: 'rgba(51,65,85,0.3)', width: 1 },
            ticks: { stroke: '#334155', width: 1 },
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
                setXRangeAllCharts(min, max);
                // Clear the selection rectangle so it doesn't linger.
                u.setSelect({ left: 0, top: 0, width: 0, height: 0 }, false);
            }],
            setCursor: onCursor ? [onCursor] : [],
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
        windowStart = windowEnd - currentWindowSec;
    }

    const bottomIds = computeBottomOfColumnIds();

    // Invalidate callout records — they'll be rebuilt alongside each chart.
    chartCallouts.length = 0;

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

        // Hover → 3D highlight.  Attached to the uPlot over-layer (not the
        // cell) so the title overlay's pointer-events don't swallow events.
        attachHighlightHandlers(charts[i].over, i);
    }
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
 * Callers set viewMode='manual' beforehand (for wheel/pan) to stop the
 * live repaint loop from overwriting the range on the next frame.
 */
function setXRangeAllCharts(min, max) {
    const clamped = clampXRange(min, max);
    manualXRange = clamped;
    for (let i = 0; i < MOTOR_COUNT; i++) {
        const c = charts[i];
        if (!c) continue;
        c.setScale('x', clamped);
    }
}

/** Drop out of manual view mode and snap back to the live-anchored window. */
function resetViewToLive() {
    viewMode = 'live';
    manualXRange = null;
    const anchor = getViewAnchor();
    const windowStart = anchor - currentWindowSec;
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

    // --- Wheel: zoom x-axis around the cursor ---------------------------
    over.addEventListener('wheel', (ev) => {
        ev.preventDefault();
        const rect = over.getBoundingClientRect();
        const px = ev.clientX - rect.left;
        const cursorT = u.posToVal(px, 'x');
        const scale = u.scales.x;
        if (scale.min == null || scale.max == null) return;

        const factor = Math.pow(WHEEL_ZOOM_STEP, ev.deltaY / 100);
        const newMin = cursorT - (cursorT - scale.min) * factor;
        const newMax = cursorT + (scale.max - cursorT) * factor;

        viewMode = 'manual';
        setXRangeAllCharts(newMin, newMax);
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
        windowStart = windowEnd - currentWindowSec;
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
