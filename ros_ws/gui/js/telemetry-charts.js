/**
 * telemetry-charts.js — Live time-series charts for motor telemetry.
 *
 * Creates a 3x3 grid of uPlot charts (one per ODrive motor) in the bottom
 * panel.  Each chart plots user-selected signals (position, velocity, current,
 * temperature, bus voltage/current) with shared Y-axes for same-unit signals
 * and synchronized crosshair cursors.
 *
 * Units:
 *   Position and velocity are plotted in PHYSICAL units, not motor revs — mm
 *   and mm/s for the legs and both hands, absolute barrel deg and deg/s for BB
 *   pitch.  The conversion happens once, at ingestion (onTelemetryData), using
 *   the per-chart table in axisUnitsFor(); every downstream consumer (axes,
 *   callouts, Δ pills, y-range padding, CSV export) inherits it for free.
 *   The toolbar's units button (mm ↔ rev) swaps every chart to raw motor revs
 *   and back; the cached history is re-expressed in place (setRawRevMode), so
 *   the store is always in ONE unit — whichever the button shows.
 *
 * Hover UX:
 *   - Entering a chart cell highlights the matching element in the 3D scene
 *     (leg 0..5 → Stewart leg, Hand → Stewart hand axis, BB Pitch/Hand → BB
 *     pitch group / hand sphere).
 *   - The vertical crosshair shows per-curve value callouts at each series
 *     intersection, synced across all charts via uPlot's cursor sync group.
 *   - While the pointer is over any chart, every chart dots its REAL samples
 *     (drawSampleDots) so data and line interpolation can be told apart.  The
 *     dots fade in with zoom and are not drawn at all once samples sit closer
 *     than SAMPLE_DOT_MIN_SPACING_PX — at that density they are just noise.
 */

import * as clock from './clock.js';
import { setStewartHighlight } from './stewart-model.js';
import { setBallButlerHighlight } from './ball-butler-model.js';
import {
    getEventsInRange, subscribeEvents, subscribeHighlight,
    getHighlightedEventId, setChartHoveredEvents, EVENT_COLORS,
} from './event-store.js';
import {
    MM_TO_REV, HAND_MM_PER_REV, BB_HAND_MM_PER_REV,
    BB_PITCH_DEG_PER_REV, BB_PITCH_DEG_OFFSET,
} from './geometry-config.js';

// ---- Signal definitions ----

/**
 * Each signal group describes one plottable series.
 *  key      – unique identifier & localStorage toggle key
 *  label    – toolbar button text
 *  color    – line colour (consistent across all 9 charts)
 *  unit     – physical unit (for Y-axis label).  'per-chart' means the unit
 *             depends on which axis the chart plots — axisUnitsFor(chartIdx)
 *             is the authority for those, not this table.
 *  scale    – scale group key (signals sharing a scale share a Y-axis)
 *  extract  – fn(motorState) → number   (null if external data)
 *  dash     – optional uPlot dash pattern [on, off]
 */
const SIGNAL_GROUPS = [
    { key: 'pos_measured',  label: 'Pos (meas)',    color: '#3b82f6', unit: 'per-chart', scale: 'position',    extract: m => m.pos_estimate },
    { key: 'pos_commanded', label: 'Pos (cmd)',     color: '#60a5fa', unit: 'per-chart', scale: 'position',    extract: null, dash: [5, 3] },
    { key: 'vel_measured',  label: 'Velocity',      color: '#22c55e', unit: 'per-chart', scale: 'velocity',    extract: m => m.vel_estimate },
    { key: 'iq_measured',   label: 'Current (meas)',color: '#f59e0b', unit: 'A',     scale: 'current',     extract: m => m.iq_measured },
    { key: 'iq_setpoint',   label: 'Current (cmd)', color: '#fbbf24', unit: 'A',     scale: 'current',     extract: m => m.iq_setpoint, dash: [5, 3] },
    { key: 'fet_temp',      label: 'FET Temp',      color: '#ef4444', unit: '\u00b0C', scale: 'temperature', extract: m => m.fet_temp },
    { key: 'motor_temp',    label: 'Motor Temp',    color: '#f87171', unit: '\u00b0C', scale: 'temperature', extract: m => m.motor_temp },
    { key: 'bus_voltage',   label: 'Bus Voltage',   color: '#a78bfa', unit: 'V',     scale: 'voltage',     extract: m => m.bus_voltage },
    { key: 'bus_current',   label: 'Bus Current',   color: '#c084fc', unit: 'A',     scale: 'bus_current', extract: m => m.bus_current },
];

/**
 * Map from scale key → display metadata.
 *
 * `position` and `velocity` are the two PER-CHART scales: their unit, decimal
 * count and y-range pad floor come from axisUnitsFor(chartIdx), because leg 3
 * (mm), the hand (mm) and BB pitch (deg) share the scale key but not the unit.
 * The entries kept here for those two are the raw-rev fallback used when a
 * chart index is out of range; everything else on this table is genuinely
 * chart-independent.
 */
const SCALE_META = {
    position:    { unit: 'rev',   decimals: 3 },
    velocity:    { unit: 'rev/s', decimals: 2 },
    current:     { unit: 'A',     decimals: 2 },
    temperature: { unit: '\u00b0C', decimals: 1 },
    voltage:     { unit: 'V',     decimals: 2 },
    bus_current: { unit: 'A',     decimals: 2 },
};

/**
 * Per-unit display rules.
 *
 *  decimals – digits after the point on axis ticks, callouts and Δ pills.
 *  padFloor – minimum half-height of the y-range when the data is flat
 *             (dataMin === dataMax).  Stated in the PLOTTED unit, so it has to
 *             be per-unit: the old hardcoded 0.5 meant half a rev, i.e. ±35 mm
 *             on a leg — it swamped the very signal it was padding.
 *  csv      – header slug ('/' and '°' stripped so the CSV column name stays
 *             a plain identifier).
 */
const UNIT_FORMAT = {
    'mm':          { decimals: 1, padFloor: 2.0, csv: 'mm' },
    'mm/s':        { decimals: 1, padFloor: 5.0, csv: 'mm_s' },
    'deg':         { decimals: 2, padFloor: 1.0, csv: 'deg' },
    'deg/s':       { decimals: 2, padFloor: 5.0, csv: 'deg_s' },
    'rev':         { decimals: 3, padFloor: 0.5, csv: 'rev' },
    'rev/s':       { decimals: 2, padFloor: 0.5, csv: 'rev_s' },
    'A':           { decimals: 2, padFloor: 0.5, csv: 'a' },
    '\u00b0C': { decimals: 1, padFloor: 0.5, csv: 'degC' },
    'V':           { decimals: 2, padFloor: 0.5, csv: 'v' },
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
    pos_measured:  'Encoder position in this chart\u2019s physical unit: mm of travel from home for the legs and both hands, absolute barrel angle (deg) for BB Pitch — or raw motor revs on every chart when the units button reads "rev". Toggle to show/hide on every chart.',
    pos_commanded: 'Commanded position target (dashed), converted with the SAME per-axis factor as the measured trace — so the gap between the two is real tracking error, not a unit mismatch. Source: leg_setpoint_echo (accepted setpoints echoed by the Teensy bridge) for legs, hand_telemetry for the hand; the BB axes have no commanded source, so the trace is absent there.',
    vel_measured:  'Axis velocity in this chart\u2019s physical unit (mm/s for the legs and both hands, deg/s for BB Pitch; rev/s everywhere when the units button reads "rev"). Signed — follows the motor\u2019s own direction convention.',
    iq_measured:   'Measured quadrature current (A) \u2014 proportional to actual torque.',
    iq_setpoint:   'Commanded quadrature current (A, dashed) \u2014 proportional to commanded torque.',
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
/** Replay span cap (seconds).  The charts plot the store's COMPOSED two-tier data: full-rate samples
 *  only for the resident chunks (tens of seconds) and 1 s digest means elsewhere, so a 10-minute
 *  view is ~600 + resident points per series - inside the live point budget (maxPoints) whose
 *  paint cost is proven.  The default window (liveWindowSec) is unchanged. */
export const REPLAY_MAX_SPAN_SEC = 600;

/** The replay chart store while in replay, else null (replay design § 5). */
let replayStore = null;
/** Playhead (seconds, record time) the replay window is centred on. */
let replayPlayhead = 0;
/** Injected `(tSec) => void` that routes a pan/zoom re-centre to engine.seek. */
let replaySeek = null;
/** Live view state parked while in replay, restored byte-for-byte on exit. */
let replaySaved = null;
let replayUnsub = null;

function maxSpanSec() {
    return replayStore ? REPLAY_MAX_SPAN_SEC : MAX_X_SPAN_SEC;
}

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
     * Returns [timestamps, series1, series2, ...].
     *
     * By default these are zero-copy TypedArray *views* into the ring buffer.
     * That is only safe when the chart will be handed fresh data before the
     * next push: once the store is full, push() shifts the whole buffer left
     * in place, so any view uPlot is still holding silently advances by one
     * sample per push (50 ms per tick at 20 Hz).  The canvas isn't redrawn,
     * so the curves look right, but cursor.idx → data[0][idx] and the
     * callouts walk forward in real time.  Pass copy=true whenever the chart
     * keeps the data across pushes (paused) so it owns an immutable snapshot.
     */
    getAlignedData(signalKeys, windowStart, copy = false) {
        // Binary search for the first timestamp >= windowStart
        let lo = 0, hi = this.length;
        while (lo < hi) {
            const mid = (lo + hi) >>> 1;
            if (this.timestamps[mid] < windowStart) lo = mid + 1;
            else hi = mid;
        }
        const start = lo;
        const take = copy
            ? (arr) => arr.slice(start, this.length)
            : (arr) => arr.subarray(start, this.length);
        const data = [take(this.timestamps)];
        for (const key of signalKeys) {
            data.push(take(this.columns[key]));
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
    initChartGridToggle();
    initTimeWindowSelector();
    initPauseButton();
    initExportButton();
    addChartTitles();
    applyChartLayout();  // also rebuilds all charts via rebuildAllCharts()
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

    // Pos/vel unit (physical vs raw revs).  Stores are still empty here, so
    // no history conversion is needed — ingestion just uses this mode.
    loadUnitsSetting();

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
    // While isolated, persist the PRE-ISOLATE snapshot: isolation is a
    // temporary zoom, so a page reload must return to the user's real layout
    // rather than stranding them on the one chart they had isolated.
    const set = (isolatedChart !== null && preIsolateVisible)
        ? preIsolateVisible : visibleCharts;
    localStorage.setItem(VISIBILITY_STORAGE_KEY, JSON.stringify([...set]));
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
 * Visible chart indices grouped by grid column.  Column-major fill in
 * motor-numeric order: `rows` charts per column, so only the trailing
 * column can be short (e.g. 8 visible → [0,1,2] [3,4,5] [6,7]).
 */
function visibleColumns() {
    const visible = [];
    for (let i = 0; i < MOTOR_COUNT; i++) {
        if (visibleCharts.has(i)) visible.push(i);
    }
    if (visible.length === 0) return [];
    const { rows } = gridShapeFor(visible.length);
    const columns = [];
    for (let k = 0; k < visible.length; k += rows) {
        columns.push(visible.slice(k, k + rows));
    }
    return columns;
}

/**
 * Indices of charts that sit at the bottom of their column in the current
 * layout — these are the ones that render the x-axis tick labels.
 */
function computeBottomOfColumnIds() {
    return new Set(visibleColumns().map(col => col[col.length - 1]));
}

function gcd(a, b) {
    return b === 0 ? a : gcd(b, a % b);
}

/** x-axis height (px) on a bottom-of-column chart (tick labels) and on every
 *  other chart (a stub, no labels).  Shared by buildUPlotOpts and the grid
 *  layout, which gives the bottom row the difference back. */
const X_AXIS_SIZE = 28;
const X_AXIS_STUB_SIZE = 4;

/**
 * Apply the grid template and hidden classes based on the current
 * visibleCharts set, then resize + repaint so uPlot adapts and the
 * freshly-unhidden charts immediately display their cached history.
 *
 * Every column fills the full grid height, however many charts it holds:
 * the row track count is the LCM of the column lengths, and a chart in a
 * column of m spans tracks/m of them.  So with BB Hand hidden, Hand and
 * BB Pitch take half the height each (3 tracks of 6); with both BB charts
 * hidden, Hand takes all of it.
 *
 * The bottom chart of each column draws the time-axis labels, which would
 * eat X_AXIS_SIZE - X_AXIS_STUB_SIZE px of its plot area.  A fixed extra
 * track under the grid gives that back: bottom charts span into it (and
 * the row gap before it), so every chart in a column plots at one height.
 */
function applyChartLayout() {
    const columns = visibleColumns();
    const tracks = columns.reduce((acc, col) => acc * col.length / gcd(acc, col.length), 1);

    const grid = document.getElementById('chart-grid');
    if (grid) {
        const rowGap = parseFloat(getComputedStyle(grid).rowGap) || 0;
        const axisTrack = Math.max(X_AXIS_SIZE - X_AXIS_STUB_SIZE - rowGap, 0);
        grid.style.gridTemplateColumns = `repeat(${Math.max(columns.length, 1)}, 1fr)`;
        grid.style.gridTemplateRows = `repeat(${tracks}, 1fr) ${axisTrack}px`;
    }

    for (let i = 0; i < MOTOR_COUNT; i++) {
        const cell = document.getElementById(`chart-${i}`);
        if (!cell) continue;
        cell.classList.toggle('chart-hidden', !visibleCharts.has(i));
        cell.style.gridColumn = '';
        cell.style.gridRow = '';
    }
    columns.forEach((col, c) => {
        const span = tracks / col.length;
        col.forEach((idx, j) => {
            const cell = document.getElementById(`chart-${idx}`);
            if (!cell) return;
            const isBottom = j === col.length - 1;
            cell.style.gridColumn = `${c + 1}`;
            cell.style.gridRow = `${j * span + 1} / span ${isBottom ? span + 1 : span}`;
        });
    });

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

/** Isolate state: the chart index currently isolated (null = not isolated) and
 *  the snapshot of `visibleCharts` taken when isolation began, so ANY short
 *  pill click can put the user's own layout back. */
let isolatedChart = null;
let preIsolateVisible = null;
/** Set when the long-press timer fires so the pill click generated by the
 *  release is swallowed exactly once.  (Distinct from the chart-canvas
 *  `suppressNextClick` further down, which guards the box-zoom mouseup.) */
let suppressNextPillClick = false;
/** Fallback timer that clears the swallow flag if no click ever arrives;
 *  tracked so a fresh gesture can cancel a stale one. */
let suppressClearTimer = null;
function armPillClickSwallow() {
    suppressNextPillClick = true;
    if (suppressClearTimer != null) clearTimeout(suppressClearTimer);
    suppressClearTimer = setTimeout(() => {
        suppressNextPillClick = false;
        suppressClearTimer = null;
    }, 600);
}
function disarmPillClickSwallow() {
    suppressNextPillClick = false;
    if (suppressClearTimer != null) {
        clearTimeout(suppressClearTimer);
        suppressClearTimer = null;
    }
}
/** Press duration (ms) that counts as a long-press "isolate" gesture. */
const LONG_PRESS_MS = 500;

/** Last-applied per-axis armed flags.  setChartArmedStates() is called at
 *  telemetry rate, so the DOM is only touched on a change. */
const chartArmedState = new Array(MOTOR_COUNT).fill(false);

/**
 * Mark the charts whose axis is in CLOSED_LOOP ("armed") — a violet pill and
 * a violet outline on the in-cell title.  Red stays reserved for faults.
 *
 * @param {boolean[]} armed – MOTOR_COUNT booleans, index = axis index
 */
export function setChartArmedStates(armed) {
    if (!Array.isArray(armed)) return;
    for (let i = 0; i < MOTOR_COUNT; i++) {
        const on = !!armed[i];
        if (on === chartArmedState[i]) continue;
        chartArmedState[i] = on;
        const btn = visibilityButtons[i];
        if (btn) btn.classList.toggle('armed', on);
        const title = document.getElementById(`chart-${i}`)
            ?.querySelector('.chart-cell-title');
        if (title) title.classList.toggle('armed', on);
    }
}

/** Value of the transient dropdown entry that shows an off-preset span. */
const CUSTOM_WINDOW_VALUE = 'custom';

/** The x span actually on screen: the frozen range in manual view, else the
 *  live (possibly wheel-widened) window. */
function visibleSpanSec() {
    if (viewMode === 'manual' && manualXRange) return manualXRange.max - manualXRange.min;
    return liveWindowSec;
}

/** Preset window lengths (s), ascending, read from the dropdown's options. */
function windowPresets(select) {
    return Array.from(select.options)
        .filter(o => o.value !== CUSTOM_WINDOW_VALUE)
        .map(o => parseInt(o.value, 10));
}

/** The preset `span` sits on, or null.  Tolerance absorbs float drift from
 *  the wheel-zoom factor round trip. */
function matchingPreset(presets, span) {
    return presets.find(p => Math.abs(span - p) <= 0.01) ?? null;
}

/** "0.25s" / "4.3s" / "87s" / "2m 13s".  `presets` guards against a label
 *  that would read like a preset it isn't (60.4 s → "60.4s", not "60s"). */
function formatSpan(span, presets) {
    if (span < 1) return `${span.toFixed(2)}s`;
    if (span < 10) return `${span.toFixed(1)}s`;
    const total = Math.round(span);
    if (total < 60) {
        return presets.includes(total) ? `${span.toFixed(1)}s` : `${total}s`;
    }
    const m = Math.floor(total / 60);
    const s = total % 60;
    return s === 0 ? `${m}m` : `${m}m ${s}s`;
}

/**
 * Make the window dropdown show the span actually on screen.  On a preset →
 * that preset is selected.  Off every preset (wheel-zoom, box-zoom, pan) → a
 * disabled "custom" entry carrying the real span is inserted in sorted
 * position and selected, styled italic/dim.  Because the dropdown then no
 * longer sits on the old preset, picking that preset again is a real
 * `change` — the fix for "zoom out, re-pick 60s, nothing happens".
 * Display-only; never mutates the window state.
 */
function syncWindowSelectState() {
    const select = document.getElementById('chart-window-select');
    if (!select) return;
    const presets = windowPresets(select);
    const span = visibleSpanSec();
    const preset = matchingPreset(presets, span);
    let custom = select.querySelector(`option[value="${CUSTOM_WINDOW_VALUE}"]`);

    if (preset !== null) {
        custom?.remove();
        select.value = String(preset);
    } else {
        if (!custom) {
            custom = document.createElement('option');
            custom.value = CUSTOM_WINDOW_VALUE;
            custom.disabled = true;
        }
        custom.textContent = formatSpan(span, presets);
        const before = Array.from(select.options)
            .find(o => o !== custom && parseInt(o.value, 10) > span) || null;
        if (custom.nextSibling !== before || custom.parentNode !== select) {
            select.insertBefore(custom, before);
        }
        select.value = CUSTOM_WINDOW_VALUE;
    }
    select.classList.toggle('off-preset', preset === null);
}

function syncVisibilityButtonStates() {
    const onlyOne = visibleCharts.size === 1;
    for (let i = 0; i < MOTOR_COUNT; i++) {
        const btn = visibilityButtons[i];
        if (!btn) continue;
        btn.classList.toggle('active', visibleCharts.has(i));
        // Don't dim while isolated: the lone visible pill is not "locked on"
        // there — any short click restores the pre-isolate layout.
        btn.classList.toggle('disabled',
            onlyOne && visibleCharts.has(i) && isolatedChart === null);
        btn.classList.toggle('isolated', isolatedChart === i);
    }
    const showAllBtn = document.getElementById('chart-show-all-btn');
    if (showAllBtn) {
        showAllBtn.classList.toggle('disabled', visibleCharts.size === MOTOR_COUNT);
    }
}

function initChartVisibilityToggles() {
    const container = document.getElementById('chart-visibility-toggles');
    if (!container) return;

    for (let i = 0; i < MOTOR_COUNT; i++) {
        const btn = document.createElement('button');
        // hold-fillable = the shared left-to-right hold sweep (viewer.css),
        // the same affordance the state-machine / command buttons use.
        btn.className = 'signal-toggle hold-fillable' + (visibleCharts.has(i) ? ' active' : '');
        btn.dataset.chart = String(i);
        btn.textContent = CHART_LABELS[i];
        btn.title = `Toggle ${CHART_LABELS[i]} chart (click) · Long-press, Shift-click or key ${i + 1} to isolate · click any pill to restore`;

        btn.addEventListener('click', (ev) => {
            // Swallow the click the long-press release generates, once.
            if (suppressNextPillClick) {
                disarmPillClickSwallow();
                return;
            }
            // Shift-click = isolate this chart (same as the number key or a
            // long-press); a second time on the same chart restores.
            if (ev.shiftKey) {
                isolateChart(i);
                return;
            }
            // Any short click on any pill while isolated puts the user's own
            // layout back and does nothing else.
            if (restoreFromIsolate()) return;
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

        // Long-press (500 ms) = isolate, for touch and mouse alike.
        let pressTimer = null;
        const clearPressTimer = () => {
            if (pressTimer != null) {
                clearTimeout(pressTimer);
                pressTimer = null;
            }
            btn.classList.remove('hold-active');
        };
        btn.addEventListener('pointerdown', (ev) => {
            // Primary button of the primary pointer only: a right/middle press
            // (or a second finger) produces no click, so it must neither
            // isolate nor arm the swallow flag.
            if (ev.button !== 0 || !ev.isPrimary) return;
            clearPressTimer();
            // A new press means no stale click from an earlier long-press is
            // still coming — drop any leftover swallow.
            disarmPillClickSwallow();
            // Hold-progress sweep, timed to the same LONG_PRESS_MS the timer
            // uses, so the fill reaching the right edge IS the trigger point.
            btn.classList.remove('hold-confirmed');
            btn.style.setProperty('--hold-ms', LONG_PRESS_MS + 'ms');
            btn.classList.add('hold-active');
            pressTimer = setTimeout(() => {
                pressTimer = null;
                btn.classList.remove('hold-active');
                btn.classList.add('hold-confirmed');
                setTimeout(() => btn.classList.remove('hold-confirmed'), 350);
                isolateChart(i);
                // The release fires a click on top of this — swallow it once,
                // with a timeout in case no click ever arrives (pointer moved
                // off the pill before release, cancelled gesture, …).
                armPillClickSwallow();
            }, LONG_PRESS_MS);
        });
        btn.addEventListener('pointerup', clearPressTimer);
        btn.addEventListener('pointerleave', clearPressTimer);
        btn.addEventListener('pointercancel', clearPressTimer);
        // A touch long-press would otherwise raise the context menu over us.
        btn.addEventListener('contextmenu', (ev) => ev.preventDefault());

        container.appendChild(btn);
        visibilityButtons.push(btn);
    }

    syncVisibilityButtonStates();
}

// ---- Hide/show chart grid (toolbar stays visible) ----

/**
 * localStorage key for the grid-hidden toggle.  Distinct from the
 * resize-handle's 'jugglebot-chart-collapsed' (which collapses the WHOLE
 * panel, toolbar included) — this one hides only #chart-grid so the toolbar
 * banner stays usable and the button can toggle its own label in place.
 */
const CHART_GRID_HIDDEN_KEY = 'jugglebot-chart-grid-hidden';

/** Reflect the current grid-hidden state onto the toggle button's label. */
function applyChartGridToggleUI(hidden) {
    const btn = document.getElementById('chart-grid-toggle-btn');
    if (!btn) return;
    btn.textContent = hidden ? 'Show charts' : 'Hide charts';
    btn.classList.toggle('active', hidden);
}

/**
 * Set the grid-hidden state: toggles .charts-hidden on #chart-panel, updates
 * the button label, and persists to localStorage.  When un-hiding, the charts
 * may have been sized (window resize / layout change) while hidden, so mirror
 * the resize-handle restore idiom and rebuild on the next frame.
 */
function setChartGridHidden(hidden, { rebuild = true } = {}) {
    const panel = document.getElementById('chart-panel');
    if (!panel) return;
    panel.classList.toggle('charts-hidden', hidden);
    applyChartGridToggleUI(hidden);
    try { localStorage.setItem(CHART_GRID_HIDDEN_KEY, hidden ? 'true' : 'false'); } catch { /* ignore */ }
    if (!hidden && rebuild) {
        requestAnimationFrame(() => rebuildCharts());
    }
}

/**
 * Public helper — clear the grid-hidden state if set (idempotent + cheap when
 * already shown).  Called from the resize-handle drag in main.js so dragging
 * the handle re-reveals the grid consistently (and updates the button label +
 * localStorage to match).
 */
export function clearChartGridHidden() {
    const panel = document.getElementById('chart-panel');
    if (!panel || !panel.classList.contains('charts-hidden')) return;
    setChartGridHidden(false);
}

// ---- Hide/show event-log markers on the charts ----

/** localStorage key for the event-marker toggle.  Hides only the chart
 *  marker lines (and so their hover tooltips); the Event Log panel and the
 *  event store keep running, so re-showing restores every marker. */
const EVENT_MARKERS_HIDDEN_KEY = 'jugglebot-chart-event-markers-hidden';

let eventMarkersHidden = false;

function setEventMarkersHidden(hidden) {
    eventMarkersHidden = hidden;
    const btn = document.getElementById('chart-event-markers-btn');
    if (btn) {
        btn.textContent = hidden ? 'Show event logs' : 'Hide event logs';
        btn.classList.toggle('active', hidden);
    }
    try { localStorage.setItem(EVENT_MARKERS_HIDDEN_KEY, hidden ? 'true' : 'false'); } catch { /* ignore */ }
    if (hidden) clearMarkerHover();
    redrawAllOverlays();
}

function initChartGridToggle() {
    const rightGroup = document.getElementById('chart-right-group');
    const visToggles = document.getElementById('chart-visibility-toggles');
    if (!rightGroup || !visToggles) return;

    const btn = document.createElement('button');
    btn.id = 'chart-grid-toggle-btn';
    btn.className = 'signal-toggle';
    btn.title = 'Hide/show the chart grid — the toolbar stays visible so you can bring it back';

    const showAllBtn = document.createElement('button');
    showAllBtn.id = 'chart-show-all-btn';
    showAllBtn.className = 'signal-toggle';
    showAllBtn.textContent = 'Show all charts';
    showAllBtn.title = 'Show all 9 charts (also exits isolate)';

    const divider = document.createElement('span');
    divider.className = 'chart-toolbar-divider';

    const markersBtn = document.createElement('button');
    markersBtn.id = 'chart-event-markers-btn';
    markersBtn.className = 'signal-toggle';
    markersBtn.title = 'Hide/show the event-log marker lines on every chart (the Event Log panel is unaffected)';

    // Global pos/vel unit toggle — shows the CURRENT unit; click to swap.
    const unitsBtn = document.createElement('button');
    unitsBtn.id = 'chart-units-btn';
    unitsBtn.className = 'signal-toggle';
    unitsBtn.addEventListener('click', () => setRawRevMode(!rawRevMode));

    const unitsDivider = document.createElement('span');
    unitsDivider.className = 'chart-toolbar-divider';

    // Insert [units][divider][hide][show-all][event-logs][divider]
    // immediately before the visibility pills so the buttons sit adjacent to
    // them, separated by the vertical rules.
    rightGroup.insertBefore(unitsBtn, visToggles);
    rightGroup.insertBefore(unitsDivider, visToggles);
    applyUnitsButtonUI();
    rightGroup.insertBefore(btn, visToggles);
    rightGroup.insertBefore(showAllBtn, visToggles);
    rightGroup.insertBefore(markersBtn, visToggles);
    rightGroup.insertBefore(divider, visToggles);

    markersBtn.addEventListener('click', () => setEventMarkersHidden(!eventMarkersHidden));
    let markersHidden = false;
    try { markersHidden = localStorage.getItem(EVENT_MARKERS_HIDDEN_KEY) === 'true'; } catch { /* ignore */ }
    setEventMarkersHidden(markersHidden);

    showAllBtn.addEventListener('click', () => {
        // Un-isolate as well as un-hide: this is the "put everything back"
        // button.  The grid-hidden state is deliberately untouched.
        // Nothing to do when already showing all — skip the full rebuild.
        if (isolatedChart === null && visibleCharts.size === MOTOR_COUNT) return;
        isolatedChart = null;
        preIsolateVisible = null;
        visibleCharts = new Set(Array.from({ length: MOTOR_COUNT }, (_, i) => i));
        saveChartVisibility();
        syncVisibilityButtonStates();
        applyChartLayout();
    });

    btn.addEventListener('click', () => {
        const panel = document.getElementById('chart-panel');
        const isHidden = panel ? panel.classList.contains('charts-hidden') : false;
        setChartGridHidden(!isHidden);
    });

    // Restore persisted state.  No rebuild on restore: the normal init flow
    // (applyChartLayout) builds the charts regardless, and if restoring to
    // hidden there's nothing to draw.
    let hidden = false;
    try { hidden = localStorage.getItem(CHART_GRID_HIDDEN_KEY) === 'true'; } catch { /* ignore */ }
    setChartGridHidden(hidden, { rebuild: false });

    // The pills were built before this button existed, so their sync couldn't
    // set its initial dim state — do it now.
    syncVisibilityButtonStates();
}

// ---- Time window selector ----

function initTimeWindowSelector() {
    const select = document.getElementById('chart-window-select');
    if (!select) return;

    select.value = currentWindowSec.toString();

    select.addEventListener('change', () => {
        const v = parseInt(select.value, 10);
        if (!Number.isFinite(v)) return;  // the disabled custom entry
        currentWindowSec = v;
        liveWindowSec = v;
        saveTimeWindow();
        if (paused) {
            // Paused (plain pause, or a zoomed/panned look at history):
            // resize around what's on screen — keep its right edge, change
            // only the width, stay paused.  R / Resume returns to live.
            const right = viewMode === 'manual' && manualXRange
                ? manualXRange.max : pausedWindowEnd;
            manualXRange = clampXRange(right - v, right);
            viewMode = 'manual';
        } else {
            // Live: an explicit redefinition of the visible slice — drop any
            // wheel-widening so the new width actually takes effect.
            viewMode = 'live';
            manualXRange = null;
        }
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
    if (paused || replayStore) return; // replay is never paused: the playhead owns the scale
    // Capture the anchor BEFORE flipping `paused`: getViewAnchor() returns
    // pausedWindowEnd once paused, and wall-clock now would jump the window
    // past the data when the stream has already gone stale (>1 s).
    const anchor = getViewAnchor();
    paused = true;
    pausedWindowEnd = anchor;
    pausedWindowStart = anchor - liveWindowSec;
    applyPauseButtonUI();
    // From here the chart KEEPS whatever data it holds across pushes, and
    // what it holds right now is a zero-copy view from the last live repaint
    // that the store will shift under it.  Force one repaint so
    // getAlignedData (paused === true) hands it an owned copy.
    repaintAllCharts(true);
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
    // A manual range may have been on screen; the live width is what shows now.
    syncWindowSelectState();
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

/**
 * Per-series uPlot `gaps` hook — makes NaN render as a GAP, never a bridge.
 *
 * ChartDataStore columns are Float64Array, which cannot hold `null` (uPlot's
 * only gap sentinel) — missing samples are NaN (e.g. pos_commanded while the
 * leg_setpoint_echo stream is stale, or on the BB charts where no commanded
 * source exists).  The vendored uPlot 1.6.31 linear path builder only treats
 * strict `null` as a gap: NaN vertices become canvas no-ops, so on stream
 * resume the stroke would draw a false straight ramp from the last finite
 * point across the whole un-commanded window to the next finite point.
 *
 * This hook scans the drawn index range for non-finite runs and appends the
 * equivalent pixel-space gap tuples; uPlot then builds its clip Path2D from
 * them, which removes the bridging segment at render time — the exact
 * mechanism it uses for its own null gaps (a null run's stroke also contains
 * the bridge lineTo; the clip is what hides it).  Semantics match uPlot's
 * null gaps precisely: a gap spans from the x-pixel of the last finite
 * sample before the run to the x-pixel of the first finite sample after it
 * (verified tuple-for-tuple against the vendored lib by the committed
 * regression probe, tools/probes/uplot_nan_gap_probe.js — assertion S1).
 *
 * Edge cases: runs touching the window edges — including the all-NaN case
 * (BB charts' pos_commanded) — produce no tuple; there is no bridging
 * segment to clip because canvas lineTo/moveTo with non-finite coords are
 * no-ops, so nothing is drawn there.  Dashed strokes are unaffected: the
 * dash pattern applies at stroke time, after the clip.  Zero per-frame
 * allocation beyond the gap tuples themselves (none for all-finite data).
 *
 * Exported: also consumed by can-traffic.js (single source — do not copy;
 * the probe above validates this one implementation for all consumers).
 */
export function nanGaps(u, sidx, i0, i1, nullGaps) {
    const xs = u.data[0];
    const ys = u.data[sidx];
    const series = u.series[sidx];
    const pxRound = (series && series.pxRound) || Math.round;
    const hadNullGaps = nullGaps.length > 0;
    let added = false;
    let lastFinite = -1;   // index of the last finite sample seen
    let inRun = false;     // currently inside a non-finite run?
    for (let i = i0; i <= i1; i++) {
        if (Number.isFinite(ys[i])) {
            if (inRun && lastFinite >= 0) {
                // Interior run closed: clip from the last finite sample
                // before the run to this first finite sample after it.
                const x0 = pxRound(u.valToPos(xs[lastFinite], 'x', true));
                const x1 = pxRound(u.valToPos(xs[i], 'x', true));
                if (x1 > x0) {
                    nullGaps.push([x0, x1]);
                    added = true;
                }
            }
            lastFinite = i;
            inRun = false;
        } else {
            inRun = true;
        }
    }
    // Typed-array columns can never contain null, so nullGaps is empty in
    // production and the appended tuples are already sorted.  A plain-Array
    // series mixing null and NaN would double-report null runs (null is
    // non-finite too) — sort + merge so uPlot's clip builder, which assumes
    // ordered non-overlapping gaps, stays correct even in that case.
    if (added && hadNullGaps) {
        nullGaps.sort((a, b) => a[0] - b[0]);
        let w = 0;
        for (let r = 1; r < nullGaps.length; r++) {
            if (nullGaps[r][0] <= nullGaps[w][1]) {
                nullGaps[w][1] = Math.max(nullGaps[w][1], nullGaps[r][1]);
            } else {
                nullGaps[++w] = nullGaps[r];
            }
        }
        nullGaps.length = w + 1;
    }
    return nullGaps;
}

/**
 * Build the uPlot options for ONE chart.
 *
 * `chartIdx` is required (not defaulted): the y-axis label and the flat-data
 * pad floor are per-chart now, and a silently-defaulted index would label a
 * BB-pitch chart in millimetres.  (Tick decimals stay on uPlot's auto
 * formatter; per-chart decimals apply to callouts and Δ pills only.)
 * Sole call site: buildAllCharts.
 */
function buildUPlotOpts(chartIdx, width, height, showXAxis = true, onCursor = null) {
    const signalList = getActiveSignalList();

    // Chart surface colours — re-read each build so theme switches pick up
    // without needing a page reload.
    const axisStroke = cssVar('--chart-axis-stroke', '#94a3b8');
    const gridStroke = cssVar('--chart-grid-stroke', 'rgba(51,65,85,0.5)');
    const gridStrokeMinor = cssVar('--chart-grid-stroke-minor', 'rgba(51,65,85,0.3)');
    const ticksStroke = cssVar('--chart-ticks-stroke', '#334155');
    // Sample-dot outline = the chart surface, so a dot reads as a punched
    // marker on its line.  Module-level: the draw hook can't afford a
    // getComputedStyle per frame.
    sampleDotOutline = cssVar('--bg-card', '#1e293b');

    // Determine which scale groups are in use, and the colour to tint the
    // axis with.  When several signals share a scale (e.g. measured +
    // commanded position), the first signal's colour wins — the
    // measured/commanded pairings happen to be hue-matched, so this still
    // visually identifies the data on that axis.
    const usedScales = new Map(); // scaleKey → { unit, color }
    for (const sig of signalList) {
        if (!usedScales.has(sig.scale)) {
            usedScales.set(sig.scale, {
                unit: unitFor(chartIdx, sig.scale),
                color: sig.color,
            });
        }
    }

    // Build scales — custom range function to keep axes tight to data
    const scales = { x: { time: true } };
    for (const scaleKey of usedScales.keys()) {
        // Captured per scale: the flat-data floor is in the plotted unit, so
        // it differs between a mm position axis and a deg/s velocity axis.
        const padFloor = padFloorFor(chartIdx, scaleKey);
        scales[scaleKey] = {
            auto: true,
            range: (u, dataMin, dataMax) => {
                // Replay: the drawn envelope can exceed the full-tier data; cover it over the visible x window.
                if (replayStore) {
                    const ax = replayStore.axes[chartIdx];
                    const xs = u.scales.x;
                    if (ax && typeof ax.envelope === 'function' && xs && xs.min != null && xs.max != null) {
                        for (const sig of signalList) {
                            if (sig.scale !== scaleKey || !activeSignals.has(sig.key)) continue;
                            const env = ax.envelope(sig.key);
                            const t = env.t;
                            for (let i = 0; i < t.length; i++) {
                                if (t[i] < xs.min) continue;
                                if (t[i] > xs.max) break;
                                const lo = env.min[i], hi = env.max[i];
                                if (lo === lo && (dataMin == null || lo < dataMin)) dataMin = lo;
                                if (hi === hi && (dataMax == null || hi > dataMax)) dataMax = hi;
                            }
                        }
                    }
                }
                return yRangeFor(dataMin, dataMax, padFloor);
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
            size: showXAxis ? X_AXIS_SIZE : X_AXIS_STUB_SIZE,
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
    for (const [scaleKey, meta] of usedScales) {
        // Physical units need more gutter than revs did: a leg position tick
        // is now "-345.0" and a hand velocity tick can reach "-5000.0", where
        // the old rev ticks were "-1.234".  60 px clipped those; 72 px does
        // not.  Chart-independent scales (A, °C, V) keep the narrower gutter
        // so a 3×3 grid doesn't lose plot width for nothing.
        const physical = (scaleKey === 'position' || scaleKey === 'velocity');
        axes.push({
            scale: scaleKey,
            side: sideToggle,
            label: meta.unit,
            labelSize: 16,
            size: physical ? 72 : 60,
            // Tint the tick labels and axis label with the series colour
            // so multi-axis charts are unambiguous at a glance.  Grid
            // strokes stay neutral — colour-tinted gridlines would be
            // distracting.
            stroke: meta.color,
            grid: { stroke: gridStrokeMinor, width: 1 },
            ticks: { stroke: meta.color, width: 1 },
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
            // NaN-as-gap (see nanGaps): applied to EVERY series because the
            // Float64Array columns can only encode "missing" as NaN.  Today
            // pos_commanded is the one signal that legitimately carries NaN
            // (stale leg echo / absent hand telemetry / BB charts), but the
            // hook is a no-op for all-finite data, so uniform wiring costs
            // nothing and no future NaN-bearing signal can bridge falsely.
            gaps: nanGaps,
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
                // plant an A bookmark at the drag-release position.  Clear
                // on the next tick so a *long* drag (which suppresses the
                // synthetic click entirely) doesn't strand the flag and
                // eat a future legitimate click.
                suppressNextClick = true;
                setTimeout(() => { suppressNextClick = false; }, 0);
            }],
            setCursor: onCursor ? [onCursor] : [],
            // Drawn after every series/axis paint — lets us overlay event
            // markers and delta-cursor bookmarks without touching the
            // plotted data.
            draw: replayStore ? [drawChartOverlays, drawReplayEnvelope, drawReplayPlayhead] : [drawChartOverlays],
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
    // The hovered chart is about to be destroyed, and mouseleave won't fire.
    clearMarkerHover();
    sampleDotsOver = null;

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
        const calloutRecord = createCalloutRecord(signalList, i);

        const opts = buildUPlotOpts(i, w, h, isBottomRow, (u) => {
            updateCallouts(u, calloutRecord);
        });

        // Initialise with real buffered data so we never flash empty after a
        // toggle / window change — falls back to empty arrays before stores exist.
        const initialData = stores[i]
            ? stores[i].getAlignedData(signalKeys, windowStart, paused)
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

// ---- Replay charts (replay design § 5) ----

/** Point `stores` at the replay store's current axes and hand the charts the
 *  whole resident buffer (immutable, so zero-copy views are safe); only the
 *  x scale moves per frame. */
function applyReplayAxes() {
    if (!replayStore) return;
    const ax0 = replayStore.axes[0];
    const fullTs = ax0 ? ax0.timestamps : null;
    // A rebuild that kept the full-rate arrays (only the digest tier changed - the digester's
    // trickle of slot installs) is coalesced to ONE repaint per animation frame; a real rebuild
    // (new resident chunks / unit toggle) repaints immediately.
    const digestOnly = fullTs !== null && replayFullTs === fullTs;
    replayFullTs = fullTs;
    if (digestOnly) {
        if (!replayRepaintPending) {
            replayRepaintPending = true;
            requestAnimationFrame(() => {
                if (!replayRepaintPending) return;
                replayRepaintPending = false;
                applyReplayAxesNow();
            });
        }
        return;
    }
    replayRepaintPending = false;
    applyReplayAxesNow();
}

let replayFullTs = null;
let replayRepaintPending = false;

function applyReplayAxesNow() {
    if (!replayStore) return;
    stores.splice(0, stores.length, ...replayStore.axes);
    if (charts.some(Boolean)) repaintAllCharts(true);
}

// ---- Replay min-max envelope of the far (digest) tier ----
// A faint band in the series colour from store.axes[i].envelope(key): the per-second min..max the
// 1 s mean line cannot show.  Drawn ONLY over the digest regions (the envelope arrays hold nothing
// else), broken wherever neighbouring bins are > ENV_GAP_SEC apart (a missing digest) and at the
// resident window (`split`).  No allocation: scalar maths on the shared Float64Arrays, direct ctx
// path calls, one fill per run.
const ENV_ALPHA = 0.15;
const ENV_GAP_SEC = 1.5;

function drawEnvRun(u, ctx, sc, t, lo, hi, s, e, x0, kx) {
    // polygon: forward along max, back along min (run = [s, e))
    ctx.beginPath();
    for (let i = s; i < e; i++) {
        const x = x0 + (t[i] - kx.min) * kx.k;
        const y = u.valToPos(hi[i], sc.scale, true);
        if (i === s) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    for (let i = e - 1; i >= s; i--) {
        ctx.lineTo(x0 + (t[i] - kx.min) * kx.k, u.valToPos(lo[i], sc.scale, true));
    }
    ctx.closePath();
    ctx.fill();
}

const envKx = { min: 0, k: 0 };
const envSc = { scale: '', min: 0, max: 0 };

function drawReplayEnvelope(u) {
    const ctx = u.ctx;
    if (!replayStore || !ctx || !u.bbox) return;
    const idx = charts.indexOf(u);
    const axis = idx >= 0 ? replayStore.axes[idx] : null;
    if (!axis || typeof axis.envelope !== 'function') return;
    const xMin = u.scales.x.min, xMax = u.scales.x.max;
    if (xMin == null || xMax == null || !(xMax > xMin)) return;
    const { left, top, width, height } = u.bbox;
    envKx.min = xMin;
    envKx.k = width / (xMax - xMin);
    let started = false;
    for (let g = 0; g < SIGNAL_GROUPS.length; g++) {
        const sig = SIGNAL_GROUPS[g];
        if (!activeSignals.has(sig.key)) continue;
        const env = axis.envelope(sig.key);
        const t = env.t, n = t.length;
        if (n === 0) continue;
        const ys = u.scales[sig.scale];
        if (!ys || ys.min == null || ys.max == null) continue;
        if (!started) {
            started = true;
            ctx.save();
            ctx.beginPath();
            ctx.rect(left, top, width, height);
            ctx.clip();
            ctx.globalAlpha = ENV_ALPHA;
        }
        ctx.fillStyle = sig.color;
        envSc.scale = sig.scale; envSc.min = ys.min; envSc.max = ys.max;
        // visible index range (t ascending within each region; the two regions are time-ordered too)
        let a = 0;
        { let l = 0, h = n; while (l < h) { const m = (l + h) >> 1; if (t[m] < xMin - ENV_GAP_SEC) l = m + 1; else h = m; } a = l; }
        let s = -1;
        for (let i = a; i < n; i++) {
            if (t[i] > xMax + ENV_GAP_SEC) break;
            const ok = env.min[i] === env.min[i] && env.max[i] === env.max[i];
            const brk = s >= 0 && (!ok || i === env.split || t[i] - t[i - 1] > ENV_GAP_SEC);
            if (brk) { drawEnvRun(u, ctx, envSc, t, env.min, env.max, s, i, left, envKx); s = -1; }
            if (ok && s < 0) s = i;
        }
        if (s >= 0) {
            let e = n;
            while (e > s && t[e - 1] > xMax + ENV_GAP_SEC) e--;
            drawEnvRun(u, ctx, envSc, t, env.min, env.max, s, e, left, envKx);
        }
    }
    if (started) ctx.restore();
}

function installEnvelopeHook(u) {
    if (!u || !u.hooks) return;
    const a = u.hooks.draw || (u.hooks.draw = []);
    if (a.indexOf(drawReplayEnvelope) < 0) a.push(drawReplayEnvelope);
}

function removeEnvelopeHook(u) {
    if (!u || !u.hooks || !u.hooks.draw) return;
    const i = u.hooks.draw.indexOf(drawReplayEnvelope);
    if (i >= 0) u.hooks.draw.splice(i, 1);
}

// ---- Replay playhead line in every chart ----
// Same colour as the trackbar playhead (CSS var --replay-playhead, css/replay.css),
// read once per enter.  Positioned from the playhead VALUE, not the box centre, so a
// panned / zoomed chart still shows the true playhead.
let playheadColor = '#06b6d4';

function readPlayheadColor() {
    try {
        const v = getComputedStyle(document.documentElement).getPropertyValue('--replay-playhead');
        if (v && v.trim()) playheadColor = v.trim();
    } catch (e) { /* keep the default */ }
}

/** Canvas-pixel x of value `p` on a linear x scale spanning [xMin, xMax] across the plot
 *  box [left, left+width]; null when p is outside the visible window. */
export function playheadCanvasX(p, xMin, xMax, left, width) {
    if (xMin == null || xMax == null || !(xMax > xMin) || !(p >= xMin && p <= xMax)) return null;
    return left + (p - xMin) / (xMax - xMin) * width;
}

function drawReplayPlayhead(u) {
    const ctx = u.ctx;
    if (!replayStore || !ctx || !u.bbox) return;
    const { left, top, width, height } = u.bbox;
    const x = playheadCanvasX(replayPlayhead, u.scales.x.min, u.scales.x.max, left, width);
    if (x === null) return;
    ctx.save();
    ctx.strokeStyle = playheadColor;
    ctx.lineWidth = 2 * (window.devicePixelRatio || 1);
    ctx.beginPath();
    ctx.moveTo(x, top);
    ctx.lineTo(x, top + height);
    ctx.stroke();
    ctx.restore();
}

function installPlayheadHook(u) {
    if (!u || !u.hooks) return;
    const a = u.hooks.draw || (u.hooks.draw = []);
    if (a.indexOf(drawReplayPlayhead) < 0) a.push(drawReplayPlayhead);
}

function removePlayheadHook(u) {
    if (!u || !u.hooks || !u.hooks.draw) return;
    const i = u.hooks.draw.indexOf(drawReplayPlayhead);
    if (i >= 0) u.hooks.draw.splice(i, 1);
}

/**
 * Enter replay charts: park the live ring and view state, point `stores` at
 * the replay store's axes.  Call BEFORE the first `store.setResident()`.
 * @param {{axes:object[], onRebuild:Function, invalidate:Function}} store
 * @param {{onSeek?:(tSec:number)=>void}} [opts]  onSeek routes pan/zoom re-centres to engine.seek
 */
export function enterReplayCharts(store, opts) {
    if (replayStore) exitReplayCharts();
    replaySaved = {
        stores: stores.slice(), paused, viewMode, manualXRange, liveWindowSec,
        pausedWindowStart, pausedWindowEnd, rawRevMode,
    };
    replayStore = store;
    replaySeek = (opts && opts.onSeek) || null;
    readPlayheadColor();
    replayFullTs = null;
    replayRepaintPending = false;
    // envelope BEFORE the playhead: the playhead line is always the topmost draw
    for (let i = 0; i < charts.length; i++) { installEnvelopeHook(charts[i]); installPlayheadHook(charts[i]); }
    paused = false;
    viewMode = 'live';
    manualXRange = null;
    liveWindowSec = Math.min(liveWindowSec, REPLAY_MAX_SPAN_SEC);
    applyPauseButtonUI();
    syncWindowSelectState();
    replayUnsub = store.onRebuild(applyReplayAxes);
    applyReplayAxes();
}

/** Leave replay charts: the very same live store objects go back into `stores`. */
export function exitReplayCharts() {
    if (!replayStore) return;
    if (replayUnsub) replayUnsub();
    replayUnsub = null;
    replayStore = null;
    replaySeek = null;
    replayFullTs = null;
    replayRepaintPending = false;
    for (let i = 0; i < charts.length; i++) { removeEnvelopeHook(charts[i]); removePlayheadHook(charts[i]); }
    const sv = replaySaved;
    replaySaved = null;
    if (sv) {
        // The unit toggle may have flipped during replay (it only invalidated
        // the replay store); bring the parked live history to the current unit
        // so live charts never mix units.
        if (sv.rawRevMode !== rawRevMode) {
            const now = rawRevMode;
            rawRevMode = sv.rawRevMode;
            const before = sv.stores.map((_, i) => axisUnitsFor(i));
            rawRevMode = now;
            for (let i = 0; i < sv.stores.length; i++) {
                convertStoreUnits(sv.stores[i], before[i], axisUnitsFor(i));
            }
        }
        stores.splice(0, stores.length, ...sv.stores);
        paused = sv.paused;
        viewMode = sv.viewMode;
        manualXRange = sv.manualXRange;
        liveWindowSec = sv.liveWindowSec;
        pausedWindowStart = sv.pausedWindowStart;
        pausedWindowEnd = sv.pausedWindowEnd;
    }
    applyPauseButtonUI();
    syncWindowSelectState();
    if (charts.some(Boolean)) repaintAllCharts(true);
}

/** Move the replay window: ONLY the x scale changes (centred on pSec). */
export function setReplayPlayhead(pSec) {
    replayPlayhead = pSec;
    if (!replayStore) return;
    const min = pSec - liveWindowSec / 2;
    const max = pSec + liveWindowSec / 2;
    for (let i = 0; i < MOTOR_COUNT; i++) {
        if (charts[i]) charts[i].setScale('x', { min, max });
    }
}

/** Test hook: a snapshot of axis i of the CURRENT stores (live ring or replay). */
export function _storeSnapshot(i) {
    const s = stores[i];
    if (!s) return null;
    const cols = {};
    for (const k in s.columns) cols[k] = Array.from(s.columns[k].subarray(0, s.length));
    return { length: s.length, timestamps: Array.from(s.timestamps.subarray(0, s.length)), columns: cols };
}

// ---- Data ingestion ----

/**
 * Called from main.js onRobotState with the full motor state array.
 *
 * UNIT BOUNDARY.  Everything upstream of this function is in motor revs (the
 * wire unit of /robot_state, /leg_setpoint_echo and /hand_telemetry);
 * everything downstream — stores, charts, callouts, Δ pills, CSV — is in the
 * chart's physical unit (mm, mm/s, deg, deg/s).  Converting here, once, is
 * what keeps measured and commanded on the same scale: any per-consumer
 * conversion eventually gets applied to one and not the other, and the chart
 * then shows a constant tracking error that does not exist.
 *
 * NaN survives the affine map (NaN*k + c === NaN), so the gap semantics the
 * nanGaps hook depends on — an absent commanded sample must render as pen-up,
 * never as a bridge — are unchanged by the conversion.
 *
 * @param {object[]} motorStates  – robot_state.motor_states (up to 9)
 * @param {number[]|null} commandedLegs – leg_setpoint_echo data (6 legs, motor revs) or null
 * @param {object|null} handTelemetry – HandTelemetryMessage or null
 */
/**
 * The pure per-axis sample: one motor_state (+ the joined commanded sources)
 * -> the values object a store column set holds, in the axis's display unit.
 * Shared by the live path (onTelemetryData) and the replay chart store, so the
 * two cannot drift apart.  Reads only rawRevMode (via axisUnitsFor).
 *
 * @param {object} m  one robot_state.motor_states entry
 * @param {number} idx  axis index 0..8
 * @param {number[]|null} commandedLegs  leg_setpoint_echo data (6 legs, motor revs) or null
 * @param {object|null} hand  HandTelemetryMessage (pos_cmd in motor revs) or null
 * @returns {object} values keyed by SIGNAL_GROUPS key
 */
export function telemetrySample(m, idx, commandedLegs, hand) {
    const u = axisUnitsFor(idx);
    const values = {
        pos_measured:  m.pos_estimate * u.posScale + u.posOffset,
        vel_measured:  m.vel_estimate * u.velScale,
        iq_setpoint:   m.iq_setpoint,
        iq_measured:   m.iq_measured,
        fet_temp:      m.fet_temp,
        motor_temp:    m.motor_temp,
        bus_voltage:   m.bus_voltage,
        bus_current:   m.bus_current,
    };

    // Commanded position — legs from leg_setpoint_echo, hand from
    // hand_telemetry.  Both arrive in motor revs, so both go through the
    // SAME per-axis map as pos_measured above (see the unit-boundary note).
    // The BB axes have no commanded source: NaN in, NaN out, gap drawn.
    if (idx < 6 && commandedLegs && commandedLegs.length > idx) {
        values.pos_commanded = commandedLegs[idx] * u.posScale + u.posOffset;
    } else if (idx === 6 && hand) {
        values.pos_commanded = hand.pos_cmd * u.posScale + u.posOffset;
    } else {
        values.pos_commanded = NaN;
    }
    return values;
}

export function onTelemetryData(motorStates, commandedLegs, handTelemetry) {
    if (!motorStates || motorStates.length === 0) return;
    // Replay charts are fed from chunk columns (replay/chart-store.js), never
    // from the dispatched handlers — a second feed would double every sample.
    if (replayStore || clock.isReplay()) return;

    const now = clock.now() / 1000; // uPlot uses seconds

    for (let i = 0; i < Math.min(motorStates.length, MOTOR_COUNT); i++) {
        stores[i].push(now, telemetrySample(motorStates[i], i, commandedLegs, handTelemetry));
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
    // Replay: the window is centred on the playhead (decision 15).
    if (replayStore) return replayPlayhead + liveWindowSec / 2;
    if (paused) return pausedWindowEnd;
    const store = stores[0];
    if (!store || store.length === 0) return Date.now() / 1000; // wall-clock: live branch (replay returns above)
    const lastT = store.timestamps[store.length - 1];
    const wall = Date.now() / 1000; // wall-clock: live branch (replay returns above)
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
    const maxSpan = maxSpanSec();
    if (span > maxSpan) {
        const mid = (min + max) / 2;
        min = mid - maxSpan / 2;
        max = mid + maxSpan / 2;
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
    if (replayStore) {
        // Replay: there is no live edge and no history to freeze.  A pan / zoom /
        // box-select re-centres the playhead on the requested range (decision
        // 15) and sets the span; the engine's seek moves the data window.
        liveWindowSec = clamped.max - clamped.min;
        manualXRange = null;
        viewMode = 'live';
        syncWindowSelectState();
        setReplayPlayhead((clamped.min + clamped.max) / 2);
        if (replaySeek) replaySeek(replayPlayhead);
        return;
    }
    const wallNow = Date.now() / 1000; // wall-clock: live branch (replay returns above)
    const FUTURE_EPSILON = 0.1;

    // Box-zoom selections honour the user's exact range — skip the live-
    // snap branch even if their rightmost drag coord landed near now.
    // Wheel-zoom and middle-drag pan keep the snap so panning forward past
    // the live edge still resumes live tracking.
    if (!fromSelection && clamped.max >= wallNow - FUTURE_EPSILON) {
        // Caught up to (or overshot) live — return to live tracking.
        const span = Math.max(
            MIN_X_SPAN_SEC,
            Math.min(maxSpanSec(), clamped.max - clamped.min),
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
    syncWindowSelectState();
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
let pendingReloadStart = null;
function reloadChartDataFromStores(windowStart) {
    // Coalesce through one rAF slot, latest-wins: the middle-drag pan calls
    // this per mousemove, and each reload now COPIES (see below) — at the
    // 600 s span that is up to ~2.6 MB per call, so per-frame is the cap.
    const first = pendingReloadStart === null;
    pendingReloadStart = windowStart;
    if (!first) return;
    requestAnimationFrame(() => {
        const ws = pendingReloadStart;
        pendingReloadStart = null;
        const signalKeys = getActiveSignalList().map(s => s.key);
        for (let i = 0; i < MOTOR_COUNT; i++) {
            if (!charts[i] || !stores[i]) continue;
            // Paused-only path: the chart keeps this data across pushes, so
            // it must own a copy (see getAlignedData).
            const data = stores[i].getAlignedData(signalKeys, ws, true);
            charts[i].setData(data, false);
        }
    });
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
                                     Math.min(maxSpanSec(), newSpan));
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
    // The window-level mousemove/mouseup handlers are installed once at
    // module load — see `activePan` near the top of this file — so chart
    // rebuilds don't accumulate listeners.  This per-chart mousedown
    // simply hands ownership of the pan to that singleton.
    over.addEventListener('mousedown', (ev) => {
        if (ev.button !== 1) return;  // middle button only
        ev.preventDefault();
        const scale = u.scales.x;
        if (scale.min == null || scale.max == null) return;
        const rect = over.getBoundingClientRect();
        activePan = {
            over,
            panStart: {
                px: ev.clientX - rect.left,
                min: scale.min,
                max: scale.max,
                width: rect.width,
            },
        };
        viewMode = 'manual';
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

    // --- Hover: event-marker tooltip at the cursor (this chart only) ----
    over.addEventListener('mousemove', (ev) => {
        const prevIds = markerHover && markerHover.u === u ? markerHover.ids : [];
        markerHover = { u, clientX: ev.clientX, clientY: ev.clientY, ids: prevIds };
        const ids = applyMarkerHover();
        // Repaint so the hovered marker thickens (or un-thickens).
        if (!sameIds(ids, prevIds)) u.redraw(false, false);
    });
    over.addEventListener('mouseleave', clearMarkerHover);

    // --- Hover: dot the real samples on every chart ---------------------
    over.addEventListener('mouseenter', () => setSampleDotsHover(over));
    over.addEventListener('mouseleave', () => setSampleDotsHover(null, over));
}

// ---- Event-marker hover tooltip ----------------------------------------

/** Half-width of the hover band around a marker line, CSS px. */
const MARKER_HOVER_TOL_PX = 5;

/** The chart the cursor is over plus the last cursor position, so the draw
 *  hook can re-hit-test while live data scrolls markers under a still
 *  cursor.  `ids` = the events currently hovered.  Null when not hovering. */
let markerHover = null;  // { u, clientX, clientY, ids: number[] }

/** Lazily-created tooltip element (one for the whole page). */
let markerTooltipEl = null;

function sameIds(a, b) {
    return a.length === b.length && a.every((id, i) => id === b[i]);
}

/** Events whose marker line lies within the hover band of the cursor on `u`,
 *  in time order.  Empty when markers are hidden. */
function hitTestMarkers(u, clientX) {
    if (eventMarkersHidden) return [];
    const xMin = u.scales.x.min;
    const xMax = u.scales.x.max;
    if (xMin == null || xMax == null) return [];
    const px = clientX - u.over.getBoundingClientRect().left;
    const hits = [];
    for (const ev of getEventsInRange(xMin, xMax)) {
        // valToPos without `canvas` = CSS px from the plot-area left edge,
        // the same frame as `px`.
        if (Math.abs(u.valToPos(ev.t, 'x') - px) <= MARKER_HOVER_TOL_PX) hits.push(ev);
    }
    return hits;
}

/** Re-hit-test the current hover, refresh the tooltip, and publish the hovered
 *  IDs to the Event Log.  No chart redraw — safe to call from the draw hook.
 *  Returns the hovered IDs. */
function applyMarkerHover() {
    if (!markerHover) return [];
    const { u, clientX, clientY } = markerHover;
    const hits = hitTestMarkers(u, clientX);
    const ids = hits.map(ev => ev.id);
    const unchanged = sameIds(ids, markerHover.ids);
    markerHover.ids = ids;
    setChartHoveredEvents(ids);

    // The chart cell's native title tooltip would pop up over ours after a
    // stationary second — park it while a marker is hovered.
    const cell = u.root.parentElement;
    if (cell) {
        if (hits.length && cell.title) {
            cell.dataset.parkedTitle = cell.title;
            cell.title = '';
        } else if (!hits.length && cell.dataset.parkedTitle !== undefined) {
            cell.title = cell.dataset.parkedTitle;
            delete cell.dataset.parkedTitle;
        }
    }

    if (!hits.length) {
        if (markerTooltipEl) markerTooltipEl.style.display = 'none';
        return ids;
    }
    if (!markerTooltipEl) {
        markerTooltipEl = document.createElement('div');
        markerTooltipEl.id = 'chart-event-tooltip';
        document.body.appendChild(markerTooltipEl);
    }
    // The draw hook calls this every live frame — rebuild only on change.
    const visible = markerTooltipEl.style.display === 'block';
    if (!unchanged || !visible) markerTooltipEl.replaceChildren(...hits.map((ev) => {
        const row = document.createElement('div');
        row.className = 'chart-event-tooltip-row';
        const dot = document.createElement('span');
        dot.className = 'chart-event-tooltip-dot';
        dot.style.background = EVENT_COLORS[ev.type] || '#94a3b8';
        const label = document.createElement('span');
        label.textContent = ev.label;
        row.append(dot, label);
        return row;
    }));
    markerTooltipEl.style.display = 'block';
    // Below-right of the cursor, flipped to the other side at a viewport edge.
    const OFFSET = 12;
    const { offsetWidth: w, offsetHeight: h } = markerTooltipEl;
    let x = clientX + OFFSET;
    let y = clientY + OFFSET;
    if (x + w > window.innerWidth - 4) x = Math.max(4, clientX - OFFSET - w);
    if (y + h > window.innerHeight - 4) y = Math.max(4, clientY - OFFSET - h);
    markerTooltipEl.style.left = `${x}px`;
    markerTooltipEl.style.top = `${y}px`;
    return ids;
}

/** Drop any marker hover: hide the tooltip, clear the Event Log tint, restore
 *  the cell title, and repaint the chart that was hovered. */
function clearMarkerHover() {
    if (!markerHover) return;
    const { u } = markerHover;
    markerHover.clientX = -Infinity;  // hit-test finds nothing → full teardown
    applyMarkerHover();
    markerHover = null;
    u.redraw(false, false);
}

// ---- Sample dots: where the real data points are -----------------------

/** Mean on-screen sample spacing (CSS px) below which no dots are drawn: the
 *  20 Hz stream at the default 10 s window sits near 2 px, where dots would
 *  merge into a thick, noisy line. */
const SAMPLE_DOT_MIN_SPACING_PX = 4;
/** Spacing at which the dots reach full opacity.  Between the two they fade
 *  in, so a wheel zoom never pops them on and off at a single threshold. */
const SAMPLE_DOT_FULL_SPACING_PX = 10;

/** The uPlot `over` element the pointer is in, or null — dots are drawn on
 *  EVERY chart while it is set, matching the synced crosshair. */
let sampleDotsOver = null;
/** Dot outline colour — the chart surface, re-read at each chart build. */
let sampleDotOutline = '#1e293b';
let sampleDotsLeavePending = false;

/** Enter (`over` set) or leave (`over` null, `leaving` = the element left)
 *  a chart.  Leaving one chart and entering the next fires leave→enter in
 *  the same task, so the off-redraw waits a frame and is skipped if another
 *  chart was entered meanwhile — moving across the grid costs no redraws. */
function setSampleDotsHover(over, leaving = null) {
    if (over) {
        const wasOff = sampleDotsOver === null;
        sampleDotsOver = over;
        if (wasOff && !sampleDotsLeavePending) redrawAllOverlays();
        return;
    }
    if (sampleDotsOver !== leaving) return;
    sampleDotsOver = null;
    if (sampleDotsLeavePending) return;
    sampleDotsLeavePending = true;
    requestAnimationFrame(() => {
        sampleDotsLeavePending = false;
        if (sampleDotsOver === null) redrawAllOverlays();
    });
}

/** First index i with ts[i] >= t (ts ascending). */
function lowerBoundTs(ts, t) {
    let lo = 0, hi = ts.length;
    while (lo < hi) {
        const mid = (lo + hi) >>> 1;
        if (ts[mid] < t) lo = mid + 1;
        else hi = mid;
    }
    return lo;
}

/**
 * Dot every real sample of every shown series while a chart is hovered —
 * the line between dots is uPlot's linear interpolation, not data.  Opacity
 * and radius follow the mean on-screen spacing of the samples in view, so
 * the dots appear as you zoom in and vanish at the wider windows.  Called
 * from drawChartOverlays with the plot-area clip already applied.
 */
function drawSampleDots(u) {
    if (sampleDotsOver === null) return;
    const ts = u.data && u.data[0];
    if (!ts || ts.length === 0) return;
    const xMin = u.scales.x.min;
    const xMax = u.scales.x.max;
    // Visible samples, plus one either side so a dot straddling the plot
    // edge is half-drawn (clipped) rather than missing.
    const vis0 = lowerBoundTs(ts, xMin);
    const vis1 = lowerBoundTs(ts, xMax);          // exclusive
    if (vis1 <= vis0) return;
    const spacing = vis1 - vis0 > 1
        ? (u.valToPos(ts[vis1 - 1], 'x') - u.valToPos(ts[vis0], 'x')) / (vis1 - 1 - vis0)
        : Infinity;
    if (!(spacing >= SAMPLE_DOT_MIN_SPACING_PX)) return;
    const alpha = Math.min(1, (spacing - SAMPLE_DOT_MIN_SPACING_PX)
        / (SAMPLE_DOT_FULL_SPACING_PX - SAMPLE_DOT_MIN_SPACING_PX));
    if (alpha <= 0) return;
    const dpr = devicePixelRatio;
    const r = Math.min(3, Math.max(1.75, spacing * 0.2)) * dpr;
    const i0 = Math.max(0, vis0 - 1);
    const i1 = Math.min(ts.length - 1, vis1);

    const ctx = u.ctx;
    ctx.globalAlpha = alpha;
    ctx.lineWidth = 1 * dpr;
    ctx.strokeStyle = sampleDotOutline;
    for (let s = 1; s < u.series.length; s++) {
        const ser = u.series[s];
        const col = u.data[s];
        if (!ser.show || !col) continue;
        ctx.beginPath();
        let any = false;
        for (let i = i0; i <= i1; i++) {
            const v = col[i];
            if (!Number.isFinite(v)) continue;     // NaN = gap (see nanGaps)
            const x = u.valToPos(ts[i], 'x', true);
            const y = u.valToPos(v, ser.scale, true);
            ctx.moveTo(x + r, y);
            ctx.arc(x, y, r, 0, 2 * Math.PI);
            any = true;
        }
        if (!any) continue;
        // uPlot normalises series.stroke to a function at init.
        ctx.fillStyle = typeof ser.stroke === 'function' ? ser.stroke(u, s) : ser.stroke;
        ctx.fill();
        ctx.stroke();
    }
    ctx.globalAlpha = 1;
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

/** Active middle-click pan — at most one across all charts.  Singleton so
 *  the global mousemove/mouseup listeners can be installed once at module
 *  load instead of being re-added on every chart rebuild. */
let activePan = null;  // { over, panStart: { px, min, max, width } }
window.addEventListener('mousemove', (ev) => {
    if (!activePan) return;
    const { over, panStart } = activePan;
    const rect = over.getBoundingClientRect();
    const dxPx = (ev.clientX - rect.left) - panStart.px;
    const span = panStart.max - panStart.min;
    const dxT = -(dxPx / panStart.width) * span;
    setXRangeAllCharts(panStart.min + dxT, panStart.max + dxT);
});
window.addEventListener('mouseup', (ev) => {
    if (ev.button === 1) activePan = null;
});

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
 *   0. Sample dots while a chart is hovered (drawSampleDots).
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

    // ---- Sample dots (hover only; under the markers) ----
    drawSampleDots(u);

    // ---- Event markers ----
    // One pass: un-highlighted events drawn faint, the highlighted one drawn
    // thicker + fully opaque on top so it pops against a marker forest.
    // Hidden by the toolbar's "Hide event logs" toggle → no markers, no tag.
    const evs = eventMarkersHidden ? [] : getEventsInRange(xMin, xMax);
    const hlId = getHighlightedEventId();
    let hlEvent = null;
    // A chart-marker hover lives on this chart only.  Re-hit-test here so the
    // tooltip follows markers that live data scrolls under a still cursor.
    const hoverIds = markerHover && markerHover.u === u ? applyMarkerHover() : [];
    ctx.lineWidth = 1 * devicePixelRatio;
    ctx.globalAlpha = 0.55;
    for (const ev of evs) {
        if (ev.id === hlId) { hlEvent = ev; continue; }
        if (hoverIds.includes(ev.id)) continue;
        const xPx = u.valToPos(ev.t, 'x', true);
        ctx.strokeStyle = EVENT_COLORS[ev.type] || '#94a3b8';
        ctx.beginPath();
        ctx.moveTo(xPx, top);
        ctx.lineTo(xPx, top + height);
        ctx.stroke();
    }
    ctx.globalAlpha = 1;

    // Hovered markers: opaque and thicker, but no label tag — the label is
    // in the cursor tooltip.
    ctx.lineWidth = 2 * devicePixelRatio;
    for (const ev of evs) {
        if (ev.id === hlId || !hoverIds.includes(ev.id)) continue;
        const xPx = u.valToPos(ev.t, 'x', true);
        ctx.strokeStyle = EVENT_COLORS[ev.type] || '#94a3b8';
        ctx.beginPath();
        ctx.moveTo(xPx, top);
        ctx.lineTo(xPx, top + height);
        ctx.stroke();
    }

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
            // Δ is in the chart's unit — the position offset (BB pitch's +90°)
            // cancels in a difference, and both samples were converted at
            // ingestion, so no extra handling is needed here.
            const absStr = formatCalloutValue(Math.abs(dv), sig.scale, i);
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
    return new Date().toISOString().replace(/:/g, '-').replace(/\..+$/, ''); // wall-clock: export filename
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

    // Column names carry their unit, per axis: the stores hold converted
    // values, and the position/velocity unit differs BETWEEN axes (leg_0 mm,
    // bb_pitch deg), so a bare `pos_measured` header would be ambiguous in
    // exactly the place a reader can no longer ask the GUI.  No in-repo
    // consumer parses these files, so the rename breaks nothing.
    const header = ['timestamp_s'];
    for (let i = 0; i < MOTOR_COUNT; i++) {
        const motor = CHART_LABELS[i].replace(/\s+/g, '_').toLowerCase();
        for (const sg of SIGNAL_GROUPS) {
            const slug = UNIT_FORMAT[unitFor(i, sg.scale)]?.csv;
            header.push(slug ? `${motor}.${sg.key}_${slug}` : `${motor}.${sg.key}`);
        }
    }

    const rows = [header.join(',')];
    const N = master.length;
    for (let k = 0; k < N; k++) {
        const cells = [master.timestamps[k].toFixed(3)];
        for (let i = 0; i < MOTOR_COUNT; i++) {
            const store = stores[i];
            // Gate on store.length so empty stores (e.g. BB motors when
            // the unit is offline) write blank cells instead of phantom
            // 0.0s from the typed-array zero-fill capacity.
            const inRange = store && k < store.length;
            for (const sg of SIGNAL_GROUPS) {
                const col = inRange ? store.columns[sg.key] : null;
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
 * Step to the next preset window size in direction `dir` (±1).  From a
 * preset that is the neighbouring preset; from an off-preset span it is the
 * nearest preset on that side (so `[` after zooming out to 87 s lands on
 * 60s).  Clamped at the ends; at an end, an off-preset span still snaps to
 * the end preset.  Done through the <select> + change event so persistence
 * and the live/paused resize behaviour live in one place.
 */
function cycleWindowSize(dir) {
    const select = document.getElementById('chart-window-select');
    if (!select) return;
    const presets = windowPresets(select);
    const span = visibleSpanSec();
    const onPreset = matchingPreset(presets, span);
    let target;
    if (dir > 0) {
        target = presets.find(p => p > span + 0.01) ?? presets[presets.length - 1];
    } else {
        target = [...presets].reverse().find(p => p < span - 0.01) ?? presets[0];
    }
    if (onPreset === target) return;  // already at the end preset
    select.value = String(target);
    select.dispatchEvent(new Event('change'));
}

/**
 * Isolate chart `idx`: hide every other chart.  Entering isolation snapshots
 * the visible set, so leaving it returns to the user's own layout rather than
 * to "show all"; isolating a DIFFERENT chart while isolated switches target
 * and keeps the original snapshot; isolating the SAME chart restores.
 *
 * The single entry point for the long-press, Shift-click and 1-9 gestures —
 * any short pill click (or the Show-all button) leaves isolation again.
 */
function isolateChart(idx) {
    if (idx < 0 || idx >= MOTOR_COUNT) return;
    if (isolatedChart === idx) {
        restoreFromIsolate();
        return;
    }
    if (isolatedChart === null) {
        preIsolateVisible = new Set(visibleCharts);
    }
    isolatedChart = idx;
    visibleCharts = new Set([idx]);
    saveChartVisibility();
    syncVisibilityButtonStates();
    applyChartLayout();
}

/**
 * Leave isolation, restoring the snapshot taken when it began (falling back to
 * all charts if the snapshot is missing or empty).  Returns true if it did
 * something, so callers can use it as "handled this click".
 */
function restoreFromIsolate() {
    if (isolatedChart === null) return false;
    const restored = (preIsolateVisible && preIsolateVisible.size > 0)
        ? new Set(preIsolateVisible)
        : new Set(Array.from({ length: MOTOR_COUNT }, (_, i) => i));
    isolatedChart = null;
    preIsolateVisible = null;
    visibleCharts = restored;
    saveChartVisibility();
    syncVisibilityButtonStates();
    applyChartLayout();
    return true;
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
            isolateChart(parseInt(ev.key, 10) - 1);
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
function createCalloutRecord(signalList, chartIdx) {
    const overlay = document.createElement('div');
    overlay.className = 'chart-callouts';

    const pills = signalList.map(sig => {
        const el = document.createElement('div');
        el.className = 'chart-callout';
        el.style.setProperty('--signal-color', sig.color);
        el.innerHTML = '<span class="chart-callout-dot"></span><span class="chart-callout-text"></span>';
        el.style.display = 'none';
        el.title = `${sig.label} \u2014 click to copy the value in ` +
                   `${unitFor(chartIdx, sig.scale) || 'chart units'} (full precision, no unit suffix)`;

        // Click-to-copy: copies the displayed quantity as a bare number — in
        // the chart's current unit (mm / mm-s / deg / deg-s, or rev / rev-s
        // when the units toggle reads "rev"), at full precision and with no
        // unit suffix, so it pastes straight into a script or calculator.
        // A brief .copied class flashes the pill green as confirmation.
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

    return { overlay, pills, signalList, chartIdx };
}

/**
 * Format one value for a callout / Δ pill.  `chartIdx` selects the unit and
 * decimal count, because the position and velocity scales mean different
 * things on different charts (mm on a leg, deg on BB pitch).
 */
function formatCalloutValue(v, scale, chartIdx) {
    const decimals = decimalsFor(chartIdx, scale);
    const unit = unitFor(chartIdx, scale);
    return `${v.toFixed(decimals)}${unit ? ' ' + unit : ''}`;
}

/**
 * Position the per-curve callouts at the current crosshair x.  uPlot fires
 * setCursor on every mouse move (and on cursor sync from peer charts), so
 * this runs at ~60 Hz while the user drags across the grid.
 */
function updateCallouts(u, record) {
    if (!record) return;
    const { overlay, pills, chartIdx } = record;
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
        textNode.textContent = formatCalloutValue(val, sig.scale, chartIdx);
        // Bare numeric (no unit suffix) for click-to-copy, in the chart's
        // display unit.  Kept at full precision so copy-paste into a script
        // preserves everything we have.
        el.dataset.rawValue = String(val);
    }
}

// ---- Per-chart physical units -------------------------------------------

/**
 * Build one immutable unit record.  `decimals` and `padFloor` are keyed by
 * SCALE key so the axis/callout/range code can look them up with the same key
 * it already carries (`sig.scale`).
 */
function makeUnitRecord(posUnit, posScale, posOffset, velUnit, velScale) {
    return Object.freeze({
        posUnit, posScale, posOffset, velUnit, velScale,
        decimals: Object.freeze({
            position: UNIT_FORMAT[posUnit].decimals,
            velocity: UNIT_FORMAT[velUnit].decimals,
        }),
        padFloor: Object.freeze({
            position: UNIT_FORMAT[posUnit].padFloor,
            velocity: UNIT_FORMAT[velUnit].padFloor,
        }),
    });
}

/**
 * Unit record per chart index — built once at module load, frozen, and handed
 * out by reference (ingestion reads it 9× per telemetry frame; allocating a
 * fresh object there would be pointless garbage).
 *
 *   legs 0-5 : mm / mm-s, per-leg gain 1/MM_TO_REV[i] — the six legs do NOT
 *              share a factor (they differ by ~1.3 %), so a single "leg
 *              constant" would bake a per-leg error into every reading.
 *   hand   6 : mm / mm-s via HAND_MM_PER_REV (platform spool).
 *   BB pitch 7: ABSOLUTE barrel degrees, deg = 90 + 360·rev (PitchAxis.h) —
 *              the same number the BB panel shows as pitch_deg and the same
 *              frame as the configured 12-90° range, so the two readouts can
 *              be compared directly.  Velocity has no offset: deg/s = 360·rev/s.
 *   BB hand 8: mm / mm-s via BB_HAND_MM_PER_REV (BB spool ≠ platform spool).
 */
const CHART_UNITS = Object.freeze([
    ...Array.from({ length: 6 }, (_, i) => {
        const mmPerRev = 1 / MM_TO_REV[i];
        return makeUnitRecord('mm', mmPerRev, 0, 'mm/s', mmPerRev);
    }),
    makeUnitRecord('mm', HAND_MM_PER_REV, 0, 'mm/s', HAND_MM_PER_REV),
    makeUnitRecord('deg', BB_PITCH_DEG_PER_REV, BB_PITCH_DEG_OFFSET,
                   'deg/s', BB_PITCH_DEG_PER_REV),
    makeUnitRecord('mm', BB_HAND_MM_PER_REV, 0, 'mm/s', BB_HAND_MM_PER_REV),
]);

/** Identity map: raw motor revs — every chart while the units toggle reads
 *  "rev", and the fallback for an out-of-range index. */
const RAW_REV_UNITS = makeUnitRecord('rev', 1, 0, 'rev/s', 1);

/**
 * Unit conversion + formatting rules for chart `chartIdx`.
 *
 * Position:  physical = rev * posScale + posOffset
 * Velocity:  physical = rev_per_s * velScale        (no offset — a constant
 *            position offset has zero derivative)
 *
 * Applied ONCE, at ingestion (onTelemetryData), so every downstream reader —
 * y-axis, callouts, Δ pills, y-range padding, CSV export — is automatically in
 * the same unit.  Converting later, per consumer, is how measured and
 * commanded end up on different scales and fabricate a constant tracking error.
 */
function axisUnitsFor(chartIdx) {
    if (rawRevMode) return RAW_REV_UNITS;
    return CHART_UNITS[chartIdx] || RAW_REV_UNITS;
}

// ---- Global pos/vel unit toggle (physical ↔ motor revs) -----------------

/** localStorage key for the unit toggle: 'rev' = raw motor revs, anything
 *  else (incl. missing) = physical units (the default). */
const UNITS_STORAGE_KEY = 'jugglebot-chart-units';

/** True while position/velocity are plotted in raw motor revs (rev, rev/s) on
 *  EVERY chart — BB pitch included, so the rev view is the uniform wire-unit
 *  view the charts showed before the physical-unit conversion landed. */
let rawRevMode = false;

/** Stored columns whose values depend on the pos/vel unit.  Every other
 *  column (current, temperature, voltage) is unit-independent. */
const UNIT_DEPENDENT_COLUMNS = [
    ['pos_measured',  'position'],
    ['pos_commanded', 'position'],
    ['vel_measured',  'velocity'],
];

/**
 * Re-express the cached history of one store in a new unit, in place.
 *
 * The stores hold values converted at ingestion (see onTelemetryData), so a
 * unit switch that only affected NEW samples would put a step discontinuity
 * into every curve at the switch time.  Instead the existing samples go back
 * through the inverse affine map to revs and forward through the new one —
 * the same map ingestion applies, so old and new samples share one scale.
 * NaN (absent commanded samples) stays NaN, preserving the nanGaps semantics.
 */
function convertStoreUnits(store, from, to) {
    const n = store.length;
    for (const [key, scale] of UNIT_DEPENDENT_COLUMNS) {
        const col = store.columns[key];
        const [fs, fo, ts, to_] = scale === 'position'
            ? [from.posScale, from.posOffset, to.posScale, to.posOffset]
            : [from.velScale, 0, to.velScale, 0];
        if (fs === ts && fo === to_) continue;
        for (let k = 0; k < n; k++) {
            col[k] = ((col[k] - fo) / fs) * ts + to_;
        }
    }
}

function applyUnitsButtonUI() {
    const btn = document.getElementById('chart-units-btn');
    if (!btn) return;
    btn.textContent = rawRevMode ? 'rev' : 'mm';
    btn.title = rawRevMode
        ? 'Position/velocity shown in raw motor revs (rev, rev/s) on every chart. Click for physical units (mm, mm/s; deg, deg/s on BB Pitch).'
        : 'Position/velocity shown in physical units (mm, mm/s; deg, deg/s on BB Pitch). Click for raw motor revs (rev, rev/s).';
}

/** Switch every chart's pos/vel unit, converting the cached history so the
 *  curves (live or paused) keep their shape across the switch. */
function setRawRevMode(on) {
    on = !!on;
    if (on !== rawRevMode) {
        const before = stores.map((_, i) => axisUnitsFor(i));
        rawRevMode = on;
        if (replayStore) {
            // Replay columns are immutable: re-derive them from the chunks in
            // the new unit instead of converting in place.
            replayStore.invalidate();
        } else {
            for (let i = 0; i < stores.length; i++) {
                convertStoreUnits(stores[i], before[i], axisUnitsFor(i));
            }
        }
        // Rebuild: axis labels, y-range pad floors, callout titles and Δ pills
        // are all baked per build.  Also repaints (copying while paused).
        rebuildAllCharts();
    }
    applyUnitsButtonUI();
    try { localStorage.setItem(UNITS_STORAGE_KEY, rawRevMode ? 'rev' : 'mm'); } catch { /* ignore */ }
}

function loadUnitsSetting() {
    try { rawRevMode = localStorage.getItem(UNITS_STORAGE_KEY) === 'rev'; } catch { /* ignore */ }
}

/** Display unit string for one (chart, scale-group) pair. */
function unitFor(chartIdx, scaleKey) {
    const u = axisUnitsFor(chartIdx);
    if (scaleKey === 'position') return u.posUnit;
    if (scaleKey === 'velocity') return u.velUnit;
    return SCALE_META[scaleKey]?.unit || '';
}

/** Decimal places for one (chart, scale-group) pair. */
function decimalsFor(chartIdx, scaleKey) {
    const u = axisUnitsFor(chartIdx);
    if (scaleKey === 'position' || scaleKey === 'velocity') {
        return u.decimals[scaleKey];
    }
    return SCALE_META[scaleKey]?.decimals ?? 2;
}

/**
 * Y-scale span below which data counts as flat, relative to its magnitude.
 * uPlot's numeric tick loop (numAxisSplits) steps `v = roundDec(v + incr)` until
 * v > max; when the span is a few ulps of a large value (|v| >= ~1e9, e.g. a
 * near-constant raw field) the chosen incr is below half an ulp, v never
 * advances, and the loop grows one array to V8's maximum length: about 1.1 GB
 * and a 6 s stall per draw, then "RangeError: Invalid array length" (measured
 * 2026-10-11, headless Chromium 154, [1.79e9, 1.79e9 + 4.8e-7] and
 * [3e9, 3e9 + 1e-6]). 1e-9 is far above float resolution (2.2e-16) and far
 * below anything a 9-chart grid can show.
 */
export const Y_MIN_REL_SPAN = 1e-9;

/**
 * Y range for one scale: [0, 1] with no (or non-finite) data, the flat-data pad
 * when the span is flat to within Y_MIN_REL_SPAN, otherwise the data plus 5 %.
 * Every y scale of the telemetry charts goes through here, so every range
 * handed to uPlot has a span its tick loop can step across.
 */
export function yRangeFor(dataMin, dataMax, padFloor) {
    if (dataMin == null || dataMax == null) return [0, 1];
    if (!Number.isFinite(dataMin) || !Number.isFinite(dataMax)) return [0, 1];
    const span = dataMax - dataMin;
    if (span <= Math.max(Math.abs(dataMin), Math.abs(dataMax)) * Y_MIN_REL_SPAN) {
        // Flat (or flat to float resolution): a small range around the value
        const v = dataMin === dataMax ? dataMin : (dataMin + dataMax) / 2;
        const pad = Math.max(Math.abs(v) * 0.1, padFloor);
        return [v - pad, v + pad];
    }
    // Add 5% padding above and below
    const pad = span * 0.05;
    return [dataMin - pad, dataMax + pad];
}

/** Flat-data y-range pad floor for one (chart, scale-group) pair. */
function padFloorFor(chartIdx, scaleKey) {
    const u = axisUnitsFor(chartIdx);
    if (scaleKey === 'position' || scaleKey === 'velocity') {
        return u.padFloor[scaleKey];
    }
    return UNIT_FORMAT[SCALE_META[scaleKey]?.unit]?.padFloor ?? 0.5;
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
        // Live mode: every push schedules a repaint, so zero-copy views are
        // refreshed before the store can shift under them.  A forced repaint
        // while paused hands the chart data it will keep — copy it.
        const data = stores[i].getAlignedData(signalKeys, windowStart, paused);
        // In live mode we drive the x-scale every frame; in manual mode the
        // user owns the scale (already set by setXRangeAllCharts) — don't
        // fight them.
        if (viewMode === 'live') {
            charts[i].setScale('x', { min: windowStart, max: windowEnd });
        }
        charts[i].setData(data, false);
    }
}
