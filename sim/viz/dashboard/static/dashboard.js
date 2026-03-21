/**
 * dashboard.js — Real-time telemetry charts for the Jugglebot MuJoCo sim.
 *
 * Connects via SSE to the Python DashboardServer, receives StepRecord
 * data at 50 Hz, and renders grouped charts using uPlot.
 */

// ---- Colour palette (matched to prod GUI: ros_ws/gui/js/telemetry-charts.js)
const C = {
    // Signal-type colours (prod canonical)
    pos:       '#3b82f6',  // position measured — blue
    pos_cmd:   '#60a5fa',  // position commanded — light blue
    vel:       '#22c55e',  // velocity — green
    cur_set:   '#f59e0b',  // current setpoint — amber
    cur_meas:  '#fbbf24',  // current measured — light amber
    temp_fet:  '#ef4444',  // FET temperature — red
    temp_mot:  '#f87171',  // motor temperature — light red
    bus_v:     '#a78bfa',  // bus voltage — purple
    bus_i:     '#c084fc',  // bus current — light purple

    // General semantic colours
    blue:   '#3b82f6',
    green:  '#22c55e',
    amber:  '#f59e0b',
    red:    '#ef4444',
    purple: '#a78bfa',
    pink:   '#ec4899',
    cyan:   '#06b6d4',
};

const RAD2DEG = 180 / Math.PI;

// ---- Section & chart definitions ---------------------------------------

const SECTIONS = [
    {
        title: 'Legs',
        cols: 3,
        charts: Array.from({length: 6}, (_, i) => ({
            id: `leg${i}`, title: `Leg ${i}`,
            signals: [
                { key: `cmd_L${i}`,  label: 'Cmd (mm)',    color: C.pos_cmd, scale: 'pos', extract: d => d.cmd_ext[i] },
                { key: `act_L${i}`,  label: 'Actual (mm)', color: C.pos,     scale: 'pos', extract: d => d.act_ext[i] },
                { key: `vel_L${i}`,  label: 'Vel (mm/s)',  color: C.vel,     scale: 'vel', extract: d => d.leg_vel[i] },
            ],
        })),
    },
    {
        title: 'Platform',
        cols: 3,
        charts: [
            {
                id: 'pos', title: 'Position (mm)', scaleId: 'plat_pos',
                // Per-axis hue spread within the blue family; dim variant for actual
                signals: ['X', 'Y', 'Z'].flatMap((axis, i) => [
                    { key: `ref_${axis}`, label: `Ref ${axis}`, color: ['#2563eb', '#3b82f6', '#60a5fa'][i],
                      extract: d => d.ref[i] },
                    { key: `act_${axis}`, label: `Act ${axis}`, color: ['#93c5fd', '#7dd3fc', '#bae6fd'][i],
                      extract: d => d.act[i] },
                ]),
            },
            {
                id: 'ori', title: 'Orientation (deg)', scaleId: 'plat_ori',
                signals: ['Rx', 'Ry', 'Rz'].flatMap((axis, i) => [
                    { key: `ref_${axis}`, label: `Ref ${axis}`, color: [C.purple, C.pink, C.cyan][i],
                      extract: d => d.ref[i + 3] * RAD2DEG },
                    { key: `act_${axis}`, label: `Act ${axis}`, color: ['#c4b5fd', '#f9a8d4', '#67e8f9'][i],
                      extract: d => d.act[i + 3] * RAD2DEG },
                ]),
            },
            {
                id: 'hand', title: 'Hand (mm)',
                signals: [
                    { key: 'hand_cmd',  label: 'Cmd (mm)',    color: C.pos_cmd, scale: 'hand_pos', extract: d => d.hand.cmd },
                    { key: 'hand_pos',  label: 'Actual (mm)', color: C.pos,     scale: 'hand_pos', extract: d => d.hand.pos },
                    { key: 'hand_vel',  label: 'Vel (mm/s)',  color: C.vel,     scale: 'hand_vel', extract: d => d.hand.vel },
                ],
            },
        ],
    },
    {
        title: 'Diagnostics',
        cols: 2,
        charts: [
            {
                id: 'err', title: 'Tracking Error',
                signals: [
                    { key: 'err_mm',  label: 'Pos (mm)',  color: C.pos,   scale: 'err_mm',  extract: d => d.err.mm },
                    { key: 'err_deg', label: 'Ori (deg)', color: C.amber, scale: 'err_deg', extract: d => d.err.deg },
                ],
            },
            {
                id: 'mpc', title: 'MPC Solver',
                signals: [
                    { key: 'solve_ms', label: 'Solve (ms)',  color: C.green, scale: 'solve_ms', extract: d => d.mpc.solve_ms },
                    { key: 'cost',     label: 'Cost',        color: C.amber, scale: 'cost',     extract: d => d.mpc.cost },
                    { key: 'cv',       label: 'Constr Viol', color: C.red,   scale: 'cv',       extract: d => d.mpc.cv },
                ],
            },
        ],
    },
];

// Flatten charts for data processing
const ALL_CHARTS = SECTIONS.flatMap(s => s.charts);

// ---- Minimum axis ranges ------------------------------------------------
// Maps scale name → minimum visible span.  For non-negative signals (errors,
// solve time, cost, constraint violation) the floor is 0 and the span sets
// the ceiling.  For bipolar signals (position, velocity) the span is centred
// on the data.
const MIN_RANGE = {
    // Legs
    pos:       10,     // mm  — small moves shouldn't collapse to a flat line
    vel:       100,    // mm/s
    // Platform
    plat_pos:  10,     // mm
    plat_ori:  2,      // deg
    // Hand
    hand_pos:  50,     // mm  — stroke is ~355mm, keep visible at idle
    hand_vel:  200,    // mm/s
    // Diagnostics — tracking error
    err_mm:    3,      // mm  (hardware threshold is 2-3 mm)
    err_deg:   1,      // deg
    // Diagnostics — MPC solver
    solve_ms:  25,     // ms  (budget is 20 ms, so show headroom)
    cost:      100,    // cost units
    cv:        0.1,    // constraint violation
};

/** Build a uPlot-compatible range function that enforces a minimum span. */
function rangeForScale(scaleName) {
    const minSpan = MIN_RANGE[scaleName];
    if (minSpan == null) return (u, dMin, dMax) => [dMin, dMax];

    // Non-negative signals: floor at 0, ensure ceiling >= minSpan
    const NON_NEG = new Set(['err_mm', 'err_deg', 'solve_ms', 'cost', 'cv']);
    if (NON_NEG.has(scaleName)) {
        return (_u, dMin, dMax) => {
            const lo = 0;
            const hi = Math.max(dMax ?? minSpan, minSpan);
            return [lo, hi];
        };
    }
    // Bipolar: keep data centred, widen symmetrically if span too small
    return (_u, dMin, dMax) => {
        if (dMin == null || dMax == null) return [-minSpan / 2, minSpan / 2];
        const span = dMax - dMin;
        if (span >= minSpan) return [dMin, dMax];
        const mid = (dMin + dMax) / 2;
        return [mid - minSpan / 2, mid + minSpan / 2];
    };
}

// ---- State -------------------------------------------------------------
let windowSec = 10;
let maxPoints = windowSec * 50;
let paused = false;
let connected = false;

// Per-chart runtime state
const charts = ALL_CHARTS.map(c => ({
    ...c,
    activeKeys: new Set(c.signals.map(s => s.key)),
    bufTime: new Float64Array(maxPoints).fill(NaN),
    bufs: c.signals.map(() => new Float64Array(maxPoints).fill(NaN)),
    writeIdx: 0,
    count: 0,
    uplot: null,
    el: null,
}));

// ---- Persistence -------------------------------------------------------
function loadToggles() {
    try {
        const saved = JSON.parse(localStorage.getItem('simDashToggles3'));
        if (!saved) return;
        for (const c of charts) {
            if (saved[c.id]) c.activeKeys = new Set(saved[c.id]);
        }
    } catch { /* ignore */ }
}

function saveToggles() {
    const obj = {};
    for (const c of charts) obj[c.id] = [...c.activeKeys];
    localStorage.setItem('simDashToggles3', JSON.stringify(obj));
}

// ---- Ring-buffer -------------------------------------------------------
function pushSample(chart, time, values) {
    const idx = chart.writeIdx % maxPoints;
    chart.bufTime[idx] = time;
    for (let i = 0; i < values.length; i++) {
        chart.bufs[i][idx] = values[i];
    }
    chart.writeIdx++;
    chart.count = Math.min(chart.count + 1, maxPoints);
}

function getOrderedData(chart) {
    const n = chart.count;
    if (n === 0) return null;
    const start = (chart.writeIdx - n + maxPoints) % maxPoints;
    const activeIndices = [];
    chart.signals.forEach((s, i) => {
        if (chart.activeKeys.has(s.key)) activeIndices.push(i);
    });
    const times = new Array(n);
    const series = activeIndices.map(() => new Array(n));
    for (let j = 0; j < n; j++) {
        const idx = (start + j) % maxPoints;
        times[j] = chart.bufTime[idx];
        for (let k = 0; k < activeIndices.length; k++) {
            series[k][j] = chart.bufs[activeIndices[k]][idx];
        }
    }
    return [times, ...series];
}

// ---- uPlot chart building ----------------------------------------------
const SYNC_KEY = 'sim-dash';
const CHART_HEIGHT = 210;

const AXIS_STYLE = {
    stroke: '#536179',
    font: '10px Inter, sans-serif',
    gap: 4,
};

function buildOpts(chart, width) {
    const active = chart.signals.filter(s => chart.activeKeys.has(s.key));

    // Build a stable, ordered list of distinct scales from ALL signals (not
    // just active) so that axes don't shift when signals are toggled.
    const scaleOrder = [];
    const scaleSeen = new Set();
    for (const s of chart.signals) {
        const sc = s.scale || 'y';
        if (!scaleSeen.has(sc)) { scaleSeen.add(sc); scaleOrder.push(sc); }
    }

    // Map each scale name to the colour of the first signal that uses it.
    const scaleColor = {};
    for (const s of chart.signals) {
        const sc = s.scale || 'y';
        if (!scaleColor[sc]) scaleColor[sc] = s.color;
    }

    const multiAxis = scaleOrder.length > 1;

    const series = [
        { label: 'Time (s)' },
        ...active.map(s => ({
            label: s.label,
            stroke: s.color,
            width: 1.5,
            scale: s.scale || 'y',
        })),
    ];

    const xAxis = {
        ...AXIS_STYLE,
        label: 'Time (s)',
        labelSize: 12,
        labelFont: '10px Inter, sans-serif',
        grid: { stroke: 'rgba(255,255,255,0.03)', width: 1 },
        ticks: { stroke: 'rgba(255,255,255,0.06)', width: 1 },
    };

    let axes, scales;

    if (multiAxis) {
        // Distribute axes: alternate left (3) / right (1) so they don't
        // pile up on one side.  Only the first axis draws a background grid.
        const nScales = scaleOrder.length;
        const axisSize = nScales > 2 ? 42 : 50;
        axes = [xAxis];
        scales = { x: { time: false } };

        scaleOrder.forEach((sc, i) => {
            const color = scaleColor[sc];
            // Find the label text from the first signal on this scale
            const sig = chart.signals.find(s => (s.scale || 'y') === sc);
            const label = sig ? sig.label : sc;
            axes.push({
                ...AXIS_STYLE,
                stroke: color,
                label: label,
                labelSize: 14,
                labelFont: 'bold 10px Inter, sans-serif',
                labelGap: 2,
                scale: sc,
                side: i % 2 === 0 ? 3 : 1,  // even → left, odd → right
                size: axisSize,
                grid: i === 0
                    ? { stroke: 'rgba(255,255,255,0.03)', width: 1 }
                    : { show: false },
                ticks: { stroke: color + '40', width: 1 },
            });
            scales[sc] = { auto: true, range: rangeForScale(sc) };
        });
    } else {
        // Single shared y-axis.  Use the chart's scaleId (if set) for min-range.
        const scId = chart.scaleId || scaleOrder[0] || 'y';
        axes = [
            xAxis,
            {
                ...AXIS_STYLE,
                grid: { stroke: 'rgba(255,255,255,0.03)', width: 1 },
                ticks: { stroke: 'rgba(255,255,255,0.06)', width: 1 },
                size: 50,
            },
        ];
        scales = { x: { time: false }, y: { auto: true, range: rangeForScale(scId) } };
    }

    return {
        width,
        height: CHART_HEIGHT,
        cursor: {
            sync: { key: SYNC_KEY, setSeries: true },
            points: { size: 4, fill: '#fff' },
        },
        series,
        axes,
        scales,
        padding: [8, 12, 0, 0],
    };
}

function rebuildChart(chart) {
    const body = chart.el;
    if (!body) return;
    body.innerHTML = '';
    const data = getOrderedData(chart) || [[], []];
    const opts = buildOpts(chart, body.clientWidth || 480);
    chart.uplot = new uPlot(opts, data, body);
}

// ---- DOM construction --------------------------------------------------
function buildUI() {
    const content = document.getElementById('content');
    let chartIdx = 0;

    for (const section of SECTIONS) {
        const sec = document.createElement('div');
        sec.className = 'section';

        const header = document.createElement('div');
        header.className = 'section-header';
        header.innerHTML = `<span class="section-title">${section.title}</span><span class="section-line"></span>`;
        sec.appendChild(header);

        const grid = document.createElement('div');
        grid.className = `section-grid cols-${section.cols}`;

        for (const chartDef of section.charts) {
            const chart = charts[chartIdx++];
            const card = document.createElement('div');
            card.className = 'chart-card';

            const hdr = document.createElement('div');
            hdr.className = 'chart-header';

            const title = document.createElement('span');
            title.className = 'chart-title';
            title.textContent = chart.title;
            hdr.appendChild(title);

            const toggles = document.createElement('div');
            toggles.className = 'signal-toggles';
            for (const sig of chart.signals) {
                const btn = document.createElement('span');
                btn.className = 'signal-toggle' + (chart.activeKeys.has(sig.key) ? ' active' : '');
                btn.innerHTML = `<span class="dot" style="background:${sig.color}"></span>${sig.label}`;
                btn.addEventListener('click', () => {
                    if (chart.activeKeys.has(sig.key)) {
                        chart.activeKeys.delete(sig.key);
                        btn.classList.remove('active');
                    } else {
                        chart.activeKeys.add(sig.key);
                        btn.classList.add('active');
                    }
                    saveToggles();
                    rebuildChart(chart);
                });
                toggles.appendChild(btn);
            }
            hdr.appendChild(toggles);
            card.appendChild(hdr);

            const body = document.createElement('div');
            body.className = 'chart-body';
            card.appendChild(body);
            chart.el = body;

            grid.appendChild(card);
        }

        sec.appendChild(grid);
        content.appendChild(sec);
    }
}

// ---- Repaint -----------------------------------------------------------
let repaintScheduled = false;

function scheduleRepaint() {
    if (repaintScheduled || paused) return;
    repaintScheduled = true;
    requestAnimationFrame(() => {
        repaintScheduled = false;
        for (const c of charts) {
            if (!c.uplot) continue;
            const data = getOrderedData(c);
            if (data) {
                c.uplot.setData(data);
                const w = c.el.clientWidth;
                if (w && Math.abs(w - c.uplot.width) > 2) {
                    c.uplot.setSize({ width: w, height: CHART_HEIGHT });
                }
            }
        }
    });
}

// ---- SSE ---------------------------------------------------------------
function connect() {
    const es = new EventSource('/events');

    es.onopen = () => {
        connected = true;
        document.getElementById('status-dot').className = 'status-dot connected';
        document.getElementById('status-text').textContent = 'Connected';
    };

    es.onerror = () => {
        connected = false;
        document.getElementById('status-dot').className = 'status-dot disconnected';
        document.getElementById('status-text').textContent = 'Disconnected';
    };

    es.onmessage = (e) => {
        const d = JSON.parse(e.data);
        for (const c of charts) {
            const vals = c.signals.map(s => s.extract(d));
            pushSample(c, d.t, vals);
        }
        scheduleRepaint();
    };
}

// ---- Resize / realloc --------------------------------------------------
function resizeAll() {
    for (const c of charts) {
        if (!c.uplot || !c.el) continue;
        const w = c.el.clientWidth;
        if (w > 0) c.uplot.setSize({ width: w, height: CHART_HEIGHT });
    }
}

function reallocBuffers() {
    maxPoints = windowSec * 50;
    for (const c of charts) {
        c.bufTime = new Float64Array(maxPoints).fill(NaN);
        c.bufs = c.signals.map(() => new Float64Array(maxPoints).fill(NaN));
        c.writeIdx = 0;
        c.count = 0;
        rebuildChart(c);
    }
}

// ---- Init --------------------------------------------------------------
window.addEventListener('DOMContentLoaded', () => {
    loadToggles();
    buildUI();
    for (const c of charts) rebuildChart(c);

    document.getElementById('time-window').addEventListener('change', (e) => {
        windowSec = parseInt(e.target.value, 10);
        reallocBuffers();
    });

    const btnPause = document.getElementById('btn-pause');
    btnPause.addEventListener('click', () => {
        paused = !paused;
        document.getElementById('pause-label').textContent = paused ? 'Resume' : 'Pause';
        document.getElementById('pause-icon').setAttribute('d', paused
            ? 'M4 2.5l9 5.5-9 5.5z'
            : 'M5 3h2v10H5zM9 3h2v10H9z'
        );
        btnPause.classList.toggle('active', paused);
        if (!paused) scheduleRepaint();
    });

    window.addEventListener('resize', resizeAll);
    connect();
});
