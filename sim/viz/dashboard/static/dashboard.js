/**
 * dashboard.js — Real-time telemetry charts for the Jugglebot MuJoCo sim.
 *
 * Connects via SSE to the Python DashboardServer, receives StepRecord
 * data at 50 Hz, and renders grouped charts using uPlot.
 */

// ---- Colour palette ----------------------------------------------------
const C = {
    blue:   '#3b82f6',
    green:  '#22c55e',
    amber:  '#f59e0b',
    red:    '#ef4444',
    purple: '#a78bfa',
    pink:   '#ec4899',
    cyan:   '#06b6d4',
    lime:   '#84cc16',
};
const LEG_COLORS = [C.blue, C.green, C.amber, C.red, C.purple, C.pink];

// Dimmer variants for "actual" signals (same hue, lower opacity feel)
const LEG_COLORS_DIM = ['#60a5fa', '#4ade80', '#fbbf24', '#f87171', '#c4b5fd', '#f9a8d4'];

const RAD2DEG = 180 / Math.PI;

// ---- Section & chart definitions ---------------------------------------

const SECTIONS = [
    {
        title: 'Legs',
        cols: 3,
        charts: Array.from({length: 6}, (_, i) => ({
            id: `leg${i}`, title: `Leg ${i}`,
            signals: [
                { key: `cmd_L${i}`,  label: 'Cmd (mm)',    color: LEG_COLORS[i],     scale: 'pos', extract: d => d.cmd_ext[i] },
                { key: `act_L${i}`,  label: 'Actual (mm)', color: LEG_COLORS_DIM[i], scale: 'pos', extract: d => d.act_ext[i] },
                { key: `vel_L${i}`,  label: 'Vel (mm/s)',  color: '#64748b',         scale: 'vel', extract: d => d.leg_vel[i] },
            ],
        })),
    },
    {
        title: 'Platform',
        cols: 2,
        charts: [
            {
                id: 'pos', title: 'Position (mm)',
                signals: ['X', 'Y', 'Z'].flatMap((axis, i) => [
                    { key: `ref_${axis}`, label: `Ref ${axis}`, color: [C.blue, C.green, C.amber][i],
                      extract: d => d.ref[i] },
                    { key: `act_${axis}`, label: `Act ${axis}`, color: [C.cyan, C.lime, '#fb923c'][i],
                      extract: d => d.act[i] },
                ]),
            },
            {
                id: 'ori', title: 'Orientation (deg)',
                signals: ['Rx', 'Ry', 'Rz'].flatMap((axis, i) => [
                    { key: `ref_${axis}`, label: `Ref ${axis}`, color: [C.purple, C.pink, C.cyan][i],
                      extract: d => d.ref[i + 3] * RAD2DEG },
                    { key: `act_${axis}`, label: `Act ${axis}`, color: ['#c4b5fd', '#f9a8d4', '#67e8f9'][i],
                      extract: d => d.act[i + 3] * RAD2DEG },
                ]),
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
                    { key: 'err_mm',  label: 'Pos (mm)',  color: C.red,  extract: d => d.err.mm },
                    { key: 'err_deg', label: 'Ori (deg)', color: C.blue, extract: d => d.err.deg },
                ],
            },
            {
                id: 'mpc', title: 'MPC Solver',
                signals: [
                    { key: 'solve_ms', label: 'Solve (ms)',  color: C.green, extract: d => d.mpc.solve_ms },
                    { key: 'cost',     label: 'Cost',        color: C.amber, extract: d => d.mpc.cost },
                    { key: 'cv',       label: 'Constr Viol', color: C.red,   extract: d => d.mpc.cv },
                ],
            },
        ],
    },
];

// Flatten charts for data processing
const ALL_CHARTS = SECTIONS.flatMap(s => s.charts);

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

    // Detect dual-axis from ALL signals the chart defines (not just active),
    // so axes stay stable when signals are toggled on/off.
    const allScaleNames = new Set(chart.signals.map(s => s.scale).filter(Boolean));
    const dualAxis = allScaleNames.size > 1;

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

    if (dualAxis) {
        axes = [
            xAxis,
            {
                ...AXIS_STYLE,
                label: 'mm',
                labelSize: 14,
                labelFont: '10px Inter, sans-serif',
                labelGap: 2,
                scale: 'pos',
                side: 3,
                size: 50,
                grid: { stroke: 'rgba(255,255,255,0.03)', width: 1 },
                ticks: { stroke: 'rgba(255,255,255,0.06)', width: 1 },
            },
            {
                ...AXIS_STYLE,
                label: 'mm/s',
                labelSize: 14,
                labelFont: '10px Inter, sans-serif',
                labelGap: 2,
                scale: 'vel',
                side: 1,
                size: 50,
                grid: { show: false },
                ticks: { stroke: 'rgba(255,255,255,0.06)', width: 1 },
            },
        ];
        scales = {
            x:   { time: false },
            pos: { auto: true },
            vel: { auto: true },
        };
    } else {
        axes = [
            xAxis,
            {
                ...AXIS_STYLE,
                grid: { stroke: 'rgba(255,255,255,0.03)', width: 1 },
                ticks: { stroke: 'rgba(255,255,255,0.06)', width: 1 },
                size: 50,
            },
        ];
        scales = { x: { time: false } };
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
