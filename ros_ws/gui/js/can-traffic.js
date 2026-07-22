/**
 * can-traffic.js — Per-bus CAN traffic panel (#panel-can, left sidebar).
 *
 * Three per-bus rows — CAN1 (Ball Butler), CAN2 (Catching cone), CAN3
 * (Jugglebot core) — each with live msg/s, kbit/s, util% readouts and a
 * BusHealth dot, plus ONE uPlot moving line chart of msg/s per bus with
 * selectable 30/60/120 s windows.
 *
 * Data sources (both diagnostic_msgs/DiagnosticStatus, from teensy_bridge_node):
 *   'profile'      1 Hz — canN_rx / canN_tx frame-count deltas + canN_util_pct
 *   'link_status' 10 Hz — bus1_health / bus2_health (BusHealth enum names),
 *                         plus bridge_link — the bridge's own verdict on the
 *                         can-hub Teensy uplink (UP/LOST/NO_HEARTBEAT), which
 *                         gates everything above (see "Staleness state")
 *
 * Derivations:
 *   msg/s  = (rx + tx) / 1 s.  The firmware counts frames over its ~1 s
 *            PROFILE window (LED-cadence, not exactly 1.000 s), so this is an
 *            approximation carrying the same few-% window jitter the
 *            firmware's own util% computation absorbs exactly.
 *   kbit/s = msg/s × CAN_BITS_PER_FRAME_APPROX / 1000 (same pre-bit-stuffing
 *            approximation the firmware uses, profiling.cpp util_x100).
 *   util%  = the firmware's canN_util_pct string, verbatim.
 *
 * Sample timestamps are client arrival times (the ROS message carries no
 * window timestamp KeyValue); at 1 Hz the arrival jitter is invisible.
 *
 * Theme: series/surface colours are read from CSS custom properties at chart
 * build time (uPlot draws on canvas), so theme.js calls rebuildCanChart() on
 * toggle — same contract as telemetry-charts' rebuildCharts().
 */

import { CAN_BITS_PER_FRAME_APPROX } from './geometry-config.js';
import { nanGaps } from './telemetry-charts.js';

// ---- Bus registry ----
//
// *** WIRE-SLOT → PHYSICAL-BUS MAPPING — do NOT label rows off the KeyValue
// field names. ***  The can-hub PROFILE uplink frame has TWO wire slots for
// THREE physical buses, and the firmware fills them re-mapped
// (Teensy_code_canbridge/profiling.cpp:24-28):
//
//     wire slot can1_*  =  physical CAN3 — Jugglebot core
//                          (6 leg ODrives + hand ODrive + Platform Teensy)
//     wire slot can2_*  =  physical CAN1 — Ball Butler
//     physical CAN2 (catching cone) is NOT on the uplink — acknowledged
//     firmware TODO; needs a 3rd PROFILE slot (protocol v4) + can-hub flash.
//
// 'link_status' mirrors the same slot semantics AT THE SOURCE: the firmware
// heartbeat packs bus1_health = jugglebot/CAN3, bus2_health = bb/CAN1
// (Teensy_code_canbridge.ino:100-101; the wire field NAMES are fixed by
// udp_protocol.h:184-185).  teensy_bridge_node._publish_link_status relays
// both fields verbatim (BusHealth enum names, no re-mapping).
//
// `slot`/`healthKey` of null ⇒ bus not on the uplink: its row renders greyed
// with 'n/a' values and its series toggle is disabled.  Rows and chart series
// are generated from this array, so lighting CAN2 up later is a two-field
// change here (slot: 'can3', healthKey: 'bus3_health' or whatever protocol v4
// names them) — no layout rework.
//
// Series colours come from the theme accent vars (resolved at chart build
// time).  Palette checked with the dataviz validator: all-pairs CVD
// separation passes in both themes; identity is never colour-alone (each
// series has a text-labelled row with live numeric readouts).
const BUSES = [
    {
        id: 'can1', label: 'CAN1', desc: 'Ball Butler',
        slot: 'can2', healthKey: 'bus2_health',
        colorVar: '--accent-amber', colorFallback: '#f59e0b',
        tooltip: 'Ball Butler bus (PROFILE wire slot can2) — click to toggle chart series',
    },
    {
        id: 'can2', label: 'CAN2', desc: 'Catching cone',
        slot: null, healthKey: null,
        colorVar: '--accent-cyan', colorFallback: '#06b6d4',
        tooltip: 'not on uplink — needs protocol v4 + can-hub flash',
    },
    {
        id: 'can3', label: 'CAN3', desc: 'Jugglebot core',
        slot: 'can1', healthKey: 'bus1_health',
        colorVar: '--accent-blue', colorFallback: '#3b82f6',
        tooltip: 'Jugglebot core bus: 6 legs + hand + Platform Teensy (PROFILE wire slot can1) — click to toggle chart series',
    },
];

// ---- Constants ----

const PROFILE_WINDOW_S = 1.0;   // firmware PROFILE window (~1 s, see header)
// Ring capacity: the 120 s preset needs 121 samples at 1 Hz (N samples span
// N-1 intervals), and sample spacing carries the firmware's LED-cadence
// window jitter — at 120 the widest preset spanned only ~119 s (a visibly
// short window).  132 = 121 + ~10 % jitter headroom.
const RING_CAP = 132;
const WINDOW_PRESETS = [30, 60, 120];
const DEFAULT_WINDOW_SEC = 60;
const WINDOW_STORAGE_KEY = 'jugglebot-can-window';
const SERIES_STORAGE_KEY = 'jugglebot-can-series';
// No 'profile' MESSAGE for >3 s ⇒ profileStale.  This watchdog only covers
// bridge-node / rosbridge / subscription death: when the can-hub Teensy
// UPLINK dies, teensy_bridge_node keeps republishing its cached profile at
// 1 Hz, so messages still flow — that outage class is detected via the
// bridge_link KeyValue instead (see canTrafficOnLinkStatus).
const STALE_TIMEOUT_MS = 3000;

/** BusHealth enum name → health-dot CSS class. */
const HEALTH_CLASS = {
    OK: 'ok',
    WARN: 'warn',
    BUS_OFF: 'busoff',
    UNKNOWN: 'unknown',
};

// ---- State ----

/** Ring buffer: sample arrival times (unix seconds), shared x for all buses. */
const times = [];
/** Ring buffer: msg/s per bus id (NaN = missing: cone slot, or stale gap). */
const rates = {};
for (const bus of BUSES) rates[bus.id] = [];

let windowSec = DEFAULT_WINDOW_SEC;
/** Per-bus chart-series visibility (persisted).  Hidden series keep their
 *  numeric row readouts updating — visibility only affects the chart. */
const seriesVisible = {};
for (const bus of BUSES) seriesVisible[bus.id] = true;

let chart = null;
let chartContainer = null;
let profileStaleTimer = null;
let healthStaleTimer = null;
let repaintTimer = null;

// ---- Staleness state ----
//
// Four INDEPENDENT stale causes, composed in one place (applyStaleUI) so
// they never stomp each other:
//
//   profileStale  no 'profile' message for >3 s — the bridge node, rosbridge,
//                 or this page's subscription died (transport liveness).
//   healthStale   no 'link_status' message for >3 s — same transport class.
//   linkDown      the latest 'link_status' carried bridge_link != 'UP': the
//                 bridge reports the can-hub Teensy uplink dead (LOST /
//                 NO_HEARTBEAT).  Messages KEEP FLOWING in this state, but
//                 their profile + bus-health payloads are the bridge's frozen
//                 cache of the last heartbeat (teensy_bridge_node republishes
//                 _latest_profile at 1 Hz and _latest_heartbeat healths at
//                 10 Hz unconditionally) — live-looking lies the panel must
//                 not render as fresh.
//   rosDown       the rosbridge websocket itself is down (setCanTrafficRosLink,
//                 driven from main.js's connection-state router).  Unlike the
//                 three above, this one ALSO freezes the chart's x-window —
//                 see getViewAnchor.
//
// UI derivation (applyStaleUI is the single writer for stale visuals):
//   badge     shown when ANY cause is active; tooltip names the live causes
//   readouts  '--' when profileStale OR linkDown OR rosDown
//   dots      UNKNOWN when linkDown OR healthStale OR rosDown; otherwise the
//             last received BusHealth.  profileStale alone leaves dots to the
//             health source — they have their own topic + watchdog.
const staleState = {
    profileStale: false,
    healthStale: false,
    linkDown: false,
    rosDown: false,
};
/** Human-readable linkDown cause for the badge tooltip (refreshed at 10 Hz
 *  while down, so the heartbeat age reads live). */
let linkDownReason = '';
/** Last BusHealth name RECEIVED per bus id — display is gated by
 *  applyStaleUI (frozen cached values are shown as UNKNOWN, not green). */
const lastHealth = {};
for (const bus of BUSES) lastHealth[bus.id] = 'UNKNOWN';
/** One NaN gap column per link-down episode (idempotent latch, re-armed
 *  when bridge_link returns to 'UP'). */
let linkGapInjected = false;
/** Wall-clock instant the current rosDown freeze began — the x-window's right
 *  edge while frozen with an EMPTY ring (a page that has never connected).
 *  Once the ring has columns the newest one is a better edge; see
 *  getViewAnchor.  Re-stamped on every down edge. */
let freezeAnchor = Date.now() / 1000;

// ---- DOM helpers ----

/** Read a CSS custom property, falling back to a default if unset. */
function cssVar(name, fallback) {
    const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
    return v || fallback;
}

function el(id) {
    return document.getElementById(id);
}

// ---- Panel init ----

export function initCanTrafficPanel() {
    const rowsHost = el('can-bus-rows');
    if (!rowsHost) return;

    // Restore persisted window preset + series visibility.
    const savedWin = parseInt(localStorage.getItem(WINDOW_STORAGE_KEY), 10);
    if (WINDOW_PRESETS.includes(savedWin)) windowSec = savedWin;
    try {
        const savedVis = JSON.parse(localStorage.getItem(SERIES_STORAGE_KEY));
        if (savedVis && typeof savedVis === 'object') {
            for (const bus of BUSES) {
                if (typeof savedVis[bus.id] === 'boolean') {
                    seriesVisible[bus.id] = savedVis[bus.id];
                }
            }
        }
    } catch { /* corrupt storage — keep defaults */ }

    // Header + one row per bus (numeric order).  Shared .can-bus-grid class
    // keeps the columns aligned across header and rows.
    const head = `
        <div class="can-bus-grid can-rows-head" aria-hidden="true">
            <span></span>
            <span>msg/s</span>
            <span>kbit/s</span>
            <span>util%</span>
            <span></span>
        </div>`;
    const rows = BUSES.map(bus => {
        const na = bus.slot === null;
        const naCls = na ? ' can-row-na' : '';
        const offCls = (!na && !seriesVisible[bus.id]) ? ' series-off' : '';
        const initVal = na ? 'n/a' : '--';
        return `
        <div class="can-bus-grid can-bus-row${naCls}${offCls}" id="can-row-${bus.id}"
             ${na ? `title="${bus.tooltip}"` : ''}>
            <button class="can-bus-label" id="can-label-${bus.id}"
                    title="${bus.tooltip}" ${na ? 'disabled' : ''}
                    aria-pressed="${!na && seriesVisible[bus.id]}">
                <span class="can-bus-swatch" style="background: var(${bus.colorVar}, ${bus.colorFallback})"></span>
                <span class="can-bus-name">${bus.label}</span>
                <span class="can-bus-desc">${bus.desc}</span>
            </button>
            <span class="can-bus-val" id="can-msgs-${bus.id}">${initVal}</span>
            <span class="can-bus-val" id="can-kbits-${bus.id}">${initVal}</span>
            <span class="can-bus-val" id="can-util-${bus.id}">${initVal}</span>
            <span class="can-health-dot unknown" id="can-health-${bus.id}"
                  title="${bus.label} health: ${na ? 'n/a (not on uplink)' : 'UNKNOWN'}"></span>
        </div>`;
    }).join('');
    rowsHost.innerHTML = head + rows;

    // Row-label clicks toggle the bus's chart series (disabled for CAN2).
    for (const bus of BUSES) {
        if (bus.slot === null) continue;
        const btn = el(`can-label-${bus.id}`);
        if (btn) btn.addEventListener('click', () => toggleBusSeries(bus));
    }

    // Window preset buttons (segmented control).
    const btnHost = el('can-window-btns');
    if (btnHost) {
        btnHost.innerHTML = WINDOW_PRESETS.map(sec =>
            `<button class="can-window-btn" data-sec="${sec}"
                     title="Show the last ${sec} s">${sec}s</button>`).join('');
        btnHost.querySelectorAll('.can-window-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                windowSec = parseInt(btn.dataset.sec, 10);
                localStorage.setItem(WINDOW_STORAGE_KEY, String(windowSec));
                updateWindowButtonsUI();
                paint();
            });
        });
        updateWindowButtonsUI();
    }

    // Chart + keep it sized to the (draggable) left sidebar.
    chartContainer = el('can-chart');
    buildCanChart();
    if (chartContainer && typeof ResizeObserver !== 'undefined') {
        const ro = new ResizeObserver(() => {
            if (!chartContainer) return;
            const w = chartContainer.clientWidth;
            const h = chartContainer.clientHeight;
            if (w < 10 || h < 10) return;
            if (!chart) buildCanChart();       // was too small at init
            else chart.setSize({ width: w, height: h });
        });
        ro.observe(chartContainer);
    }

    // 1 Hz live-slide: keeps the x-window moving (and outage gaps visibly
    // growing) even when no messages arrive.  Frozen while ROS2 is down —
    // paint() takes its right edge from getViewAnchor, not wall-clock.
    repaintTimer = setInterval(paint, 1000);

    // Arm the staleness watchdogs immediately so a never-connected page shows
    // an honest STALE state 3 s after load (same setTimeout-watchdog pattern
    // as the motion panel / mocap connection).
    armProfileWatchdog();
    armHealthWatchdog();
}

// ---- Message handlers (called from main.js router) ----

/** Parse DiagnosticStatus KeyValue[] into a plain object (string values). */
function kvMap(msg) {
    const kv = {};
    for (const v of (msg.values || [])) kv[v.key] = v.value;
    return kv;
}

/**
 * Handle a 'profile' message (1 Hz): update readouts + append a chart column.
 * @param {object} msg - diagnostic_msgs/DiagnosticStatus
 */
export function canTrafficOnProfile(msg) {
    // Transport liveness: a message arrived, so the bridge→GUI path is up.
    if (staleState.profileStale) {
        staleState.profileStale = false;
        applyStaleUI();
    }
    armProfileWatchdog();

    // Data gate: while the Teensy uplink is down the bridge republishes its
    // CACHED profile at 1 Hz — appending those frozen values would draw a
    // live-looking flat line through the exact outage this panel exists to
    // surface.  The gap column was injected at the link-down transition;
    // drop the payload (readouts already read '--' via applyStaleUI).
    if (staleState.linkDown) return;

    const kv = kvMap(msg);
    const now = Date.now() / 1000;

    times.push(now);
    for (const bus of BUSES) {
        let msgsPerSec = NaN;
        let util = null;
        if (bus.slot !== null) {
            const rx = parseInt(kv[`${bus.slot}_rx`], 10);
            const tx = parseInt(kv[`${bus.slot}_tx`], 10);
            if (Number.isFinite(rx) && Number.isFinite(tx)) {
                msgsPerSec = (rx + tx) / PROFILE_WINDOW_S;
            }
            util = kv[`${bus.slot}_util_pct`];
        }
        rates[bus.id].push(msgsPerSec);
        updateBusReadouts(bus, msgsPerSec, util);
    }
    trimRing();
    paint();
}

/**
 * Handle a 'link_status' message (10 Hz): update the per-bus health dots.
 * @param {object} msg - diagnostic_msgs/DiagnosticStatus
 */
export function canTrafficOnLinkStatus(msg) {
    const kv = kvMap(msg);

    // Record last RECEIVED per-bus health; whether it is displayed or shown
    // as UNKNOWN is decided in applyStaleUI (while the uplink is down these
    // are the bridge's frozen cache of the last heartbeat).
    for (const bus of BUSES) {
        if (bus.healthKey === null) continue;  // cone health not on uplink
        lastHealth[bus.id] = kv[bus.healthKey] || 'UNKNOWN';
    }

    // bridge_link: the bridge's own liveness verdict on the can-hub Teensy
    // uplink (teensy_bridge_node._publish_link_status: UP/LOST/NO_HEARTBEAT).
    // A missing key is treated as down — honest-UNKNOWN beats frozen-green.
    const bridgeLink = kv.bridge_link || 'UNKNOWN';
    if (bridgeLink !== 'UP') {
        const age = kv.heartbeat_age_ms;
        linkDownReason = `Teensy uplink lost (bridge_link=${bridgeLink}`
            + ((age && age !== 'n/a') ? `, heartbeat age ${age} ms` : '') + ')';
        staleState.linkDown = true;
        if (!linkGapInjected) {
            // Exactly one NaN column per episode: the chart line gaps at the
            // moment of loss and stays gapped until data is trustworthy again.
            linkGapInjected = true;
            injectGapColumn();
            paint();
        }
    } else {
        // Uplink back: re-arm the one-gap-per-episode latch.  Readouts stay
        // '--' until the next fresh 'profile' (≤1 s); dots restore now from
        // this message's fresh health values via applyStaleUI.
        staleState.linkDown = false;
        linkGapInjected = false;
        linkDownReason = '';
    }

    staleState.healthStale = false;
    armHealthWatchdog();
    applyStaleUI();
}

/**
 * ROS2 connection edge — called from main.js's connection-state router for
 * every state, not just the down edge.
 *
 * A websocket drop takes the whole ROS graph with it, so the panel has no
 * source of truth at all: readouts blank, dots go UNKNOWN, and the chart's
 * x-window freezes at the last column instead of scrolling the history away
 * (see getViewAnchor).  The three CAN-side outage causes are deliberately NOT
 * treated this way — ROS2 is still up for those, so their gaps keep growing.
 *
 * @param {boolean} isUp - true ONLY for the 'connected' state.  The reconnect
 *   loop oscillates connecting↔disconnected every 2 s while ROS is down; both
 *   of those count as down, so the freeze holds across the whole outage.
 */
export function setCanTrafficRosLink(isUp) {
    if (staleState.rosDown === !isUp) return;   // no edge — idempotent
    staleState.rosDown = !isUp;
    // Stamp the freeze instant on the DOWN edge only: it is the x-window's
    // right edge until the ring has a column to anchor to (getViewAnchor).
    if (!isUp) freezeAnchor = Date.now() / 1000;
    applyStaleUI();
    paint();
}

// ---- Readouts ----

function updateBusReadouts(bus, msgsPerSec, utilStr) {
    if (bus.slot === null) return;  // cone row stays 'n/a'
    const msgsEl = el(`can-msgs-${bus.id}`);
    const kbitsEl = el(`can-kbits-${bus.id}`);
    const utilEl = el(`can-util-${bus.id}`);
    if (Number.isFinite(msgsPerSec)) {
        if (msgsEl) msgsEl.textContent = String(Math.round(msgsPerSec));
        if (kbitsEl) {
            kbitsEl.textContent =
                (msgsPerSec * CAN_BITS_PER_FRAME_APPROX / 1000).toFixed(1);
        }
    } else {
        if (msgsEl) msgsEl.textContent = '--';
        if (kbitsEl) kbitsEl.textContent = '--';
    }
    // util% is the firmware's canN_util_pct string, verbatim.
    if (utilEl) utilEl.textContent = (utilStr != null && utilStr !== '') ? utilStr : '--';
}

function setHealthDot(bus, healthName) {
    const dot = el(`can-health-${bus.id}`);
    if (!dot) return;
    const cls = HEALTH_CLASS[healthName] || 'unknown';
    dot.className = `can-health-dot ${cls}`;
    dot.title = `${bus.label} health: ${healthName}`;
}

// ---- Staleness ----

function armProfileWatchdog() {
    if (profileStaleTimer) clearTimeout(profileStaleTimer);
    profileStaleTimer = setTimeout(onProfileStale, STALE_TIMEOUT_MS);
}

function armHealthWatchdog() {
    if (healthStaleTimer) clearTimeout(healthStaleTimer);
    healthStaleTimer = setTimeout(onHealthStale, STALE_TIMEOUT_MS);
}

/** No 'profile' message for >3 s: readouts '--' and one all-NaN chart column
 *  so the line GAPS across the outage (the nanGaps hook renders NaN runs as
 *  true gaps, never a bridging segment).  Single-shot per outage — re-armed
 *  by the next message.  Dots are NOT touched: health has its own topic and
 *  watchdog, and may still be flowing. */
function onProfileStale() {
    staleState.profileStale = true;
    injectGapColumn();
    applyStaleUI();
    paint();
}

/** No 'link_status' for >3 s: health is unknown, not "last known". */
function onHealthStale() {
    staleState.healthStale = true;
    applyStaleUI();
}

/** Append one all-NaN column so the chart line gaps.  No-op on an empty
 *  ring (nothing to gap from).  Adjacent duplicate NaN columns (e.g.
 *  link-down transition followed by a full bridge death) merge into the
 *  same visual gap — harmless. */
function injectGapColumn() {
    if (times.length === 0) return;
    times.push(Date.now() / 1000);
    for (const bus of BUSES) rates[bus.id].push(NaN);
    trimRing();
}

/** Single writer for all stale visuals — derives badge, readouts, and dots
 *  from staleState so the three causes compose instead of stomping each
 *  other.  Idempotent; called on every state change and on every
 *  link_status message (10 Hz — trivial DOM writes). */
function applyStaleUI() {
    const causes = [];
    if (staleState.rosDown) causes.push('ROS2 disconnected — chart frozen at last sample');
    if (staleState.linkDown) causes.push(linkDownReason);
    if (staleState.profileStale) causes.push("no 'profile' from bridge >3 s");
    if (staleState.healthStale) causes.push("no 'link_status' from bridge >3 s");

    const badge = el('can-stale-badge');
    if (badge) {
        badge.style.display = causes.length ? '' : 'none';
        if (causes.length) badge.title = `Data stale: ${causes.join('; ')}`;
    }

    // Readouts: profile values are trustworthy only when fresh AND the
    // uplink is up (linkDown ⇒ cached/frozen numbers) AND ROS2 is connected
    // (rosDown ⇒ the last values are however old the outage is).
    if (staleState.profileStale || staleState.linkDown || staleState.rosDown) {
        for (const bus of BUSES) updateBusReadouts(bus, NaN, null);
    }

    // Dots: linkDown / healthStale / rosDown ⇒ UNKNOWN (the cached green
    // values are frozen lies); otherwise show the last received BusHealth.
    const dotsUnknown = staleState.linkDown || staleState.healthStale || staleState.rosDown;
    for (const bus of BUSES) {
        if (bus.healthKey === null) continue;
        setHealthDot(bus, dotsUnknown ? 'UNKNOWN' : lastHealth[bus.id]);
    }
}

// ---- Ring buffer ----

function trimRing() {
    while (times.length > RING_CAP) {
        times.shift();
        for (const bus of BUSES) rates[bus.id].shift();
    }
}

// ---- Series visibility ----

function toggleBusSeries(bus) {
    seriesVisible[bus.id] = !seriesVisible[bus.id];
    localStorage.setItem(SERIES_STORAGE_KEY, JSON.stringify(seriesVisible));

    // Reflect the new state in the DOM FIRST: the row label greys via the
    // .series-off CSS rule (readouts stay live — visibility is a chart
    // concern), and this MUST land independently of the chart mutation below.
    // (Previously the chart call threw and aborted the function before these
    // lines ran, so the label never greyed — the operator-visible bug.)
    const row = el(`can-row-${bus.id}`);
    if (row) row.classList.toggle('series-off', !seriesVisible[bus.id]);
    const btn = el(`can-label-${bus.id}`);
    if (btn) btn.setAttribute('aria-pressed', String(seriesVisible[bus.id]));

    // Toggle the chart series LAST.  Set series.show + redraw() rather than
    // chart.setSeries({show}): with cursor.show=false there is no cursor-point
    // element, and setSeries()'s hide branch unconditionally repositions it
    // (yt[i].style) — an undefined deref that throws in uPlot 1.6.31.  redraw()
    // re-renders paths + re-fits the y-scale to the visible series, never
    // touching the cursor layer.
    if (chart) {
        chart.series[BUSES.indexOf(bus) + 1].show = seriesVisible[bus.id];
        chart.redraw();
    }
}

// ---- Window preset UI ----

function updateWindowButtonsUI() {
    const btnHost = el('can-window-btns');
    if (!btnHost) return;
    btnHost.querySelectorAll('.can-window-btn').forEach(btn => {
        btn.classList.toggle('active', parseInt(btn.dataset.sec, 10) === windowSec);
    });
}

// ---- Chart ----

/**
 * uPlot points.filter hook: draw a dot ONLY where a finite sample has no
 * finite neighbour.  An isolated sample between two NaN gap columns (e.g. a
 * single profile message landing between two outages) draws no line segment
 * at all — both adjacent segments are pen-up — so with points hidden it
 * would be invisible.  Returns null (draw no points) when every finite
 * sample has a finite neighbour: the line already shows those.
 *
 * Contract (validated against the vendored uPlot 1.6.31 — see the probe
 * output in the review-fix session): with points.show=false, uPlot still
 * calls filter(u, sidx, show, gaps); a returned idx array is passed to
 * points.paths as filtIdxs, which draws exactly those samples.
 */
function isolatedPointFilter(u, sidx) {
    const ys = u.data[sidx];
    const idxs = [];
    for (let i = 0; i < ys.length; i++) {
        if (!Number.isFinite(ys[i])) continue;
        const prevFinite = i > 0 && Number.isFinite(ys[i - 1]);
        const nextFinite = i + 1 < ys.length && Number.isFinite(ys[i + 1]);
        if (!prevFinite && !nextFinite) idxs.push(i);
    }
    return idxs.length ? idxs : null;
}

function buildCanChart() {
    if (!chartContainer || typeof uPlot === 'undefined') return;
    if (chart) {
        chart.destroy();
        chart = null;
    }
    chartContainer.innerHTML = '';

    const w = chartContainer.clientWidth;
    const h = chartContainer.clientHeight;
    if (w < 10 || h < 10) return;  // ResizeObserver builds once sized

    // Surface colours re-read each build so theme switches pick up.
    const axisStroke = cssVar('--chart-axis-stroke', '#94a3b8');
    const gridStroke = cssVar('--chart-grid-stroke', 'rgba(51,65,85,0.5)');
    const ticksStroke = cssVar('--chart-ticks-stroke', '#334155');

    const series = [{}];
    for (const bus of BUSES) {
        series.push({
            label: bus.label,
            stroke: cssVar(bus.colorVar, bus.colorFallback),
            width: 1.5 * window.devicePixelRatio,
            // Points hidden for dense data; the filter surfaces ONLY
            // isolated samples (see isolatedPointFilter above).
            points: { show: false, size: 5, filter: isolatedPointFilter },
            show: seriesVisible[bus.id],
            // NaN-as-gap: the cone series is all-NaN (draws nothing) and
            // staleness injects NaN columns; without this hook uPlot would
            // bridge an outage with a false straight segment.
            gaps: nanGaps,
        });
    }

    const opts = {
        width: w,
        height: h,
        series,
        scales: {
            x: { time: true },
            y: {
                auto: true,
                // Rate chart: baseline anchored at 0 (magnitude, not deltas).
                range: (u, dataMin, dataMax) => {
                    if (dataMax == null || dataMax <= 0) return [0, 1];
                    return [0, dataMax * 1.05];
                },
            },
        },
        axes: [
            {
                stroke: axisStroke,
                grid: { stroke: gridStroke, width: 1 },
                ticks: { stroke: ticksStroke, width: 1, size: 4 },
                font: '10px JetBrains Mono, monospace',
                size: 24,
                values: (u, splits) => splits.map(v => {
                    const d = new Date(v * 1000);
                    const mm = String(d.getMinutes()).padStart(2, '0');
                    const ss = String(d.getSeconds()).padStart(2, '0');
                    return `${String(d.getHours()).padStart(2, '0')}:${mm}:${ss}`;
                }),
            },
            {
                scale: 'y',
                label: 'msg/s',
                labelSize: 12,
                size: 40,
                stroke: axisStroke,
                grid: { stroke: gridStroke, width: 1 },
                ticks: { stroke: ticksStroke, width: 1 },
                font: '10px JetBrains Mono, monospace',
                labelFont: '10px JetBrains Mono, monospace',
            },
        ],
        // The bus rows are the legend (colour swatch + text label + live
        // values), matching the house legend:{show:false} idiom.
        legend: { show: false },
        cursor: { show: false },
        padding: [8, 8, 0, 0],
    };

    chart = new uPlot(opts, alignedData(), chartContainer);
    paint();
}

function alignedData() {
    return [times, ...BUSES.map(bus => rates[bus.id])];
}

/**
 * Pick the right-hand edge of the x-window.
 *   - ROS2 connected: wall-clock now, so the window slides live and an
 *     in-progress outage's NaN gap visibly GROWS (the whole point of the 1 Hz
 *     repaint timer — a bridge_link drop or a dead bridge node is an outage we
 *     are still actively observing, and its duration is information).
 *   - ROS2 disconnected: the newest column in the ring, or — with nothing in
 *     the ring at all — the instant the freeze began.  Nothing about the CAN
 *     buses is observable while the websocket is down, so wall-clock progress
 *     is not an observation: rendering it as one would scroll the entire
 *     pre-disconnect history off the left edge within `windowSec`, destroying
 *     the data an operator opens this panel to look at after a dropout.
 *
 * The empty-ring case is the never-connected page (the connection router fires
 * on registration, so freezeAnchor is stamped at load).  It matters because
 * this panel drives repaints from an unconditional 1 Hz timer, unlike
 * telemetry-charts, which repaints only when a sample arrives and therefore
 * sits still on its own when no data ever comes.  Falling back to wall-clock
 * here would make a never-connected CAN chart scroll while the actuator charts
 * below it stayed put — same situation, two different behaviours.
 */
function getViewAnchor() {
    if (!staleState.rosDown) return Date.now() / 1000;
    return times.length ? times[times.length - 1] : freezeAnchor;
}

/** Redraw with the window [anchor - windowSec, anchor] (see getViewAnchor). */
function paint() {
    if (!chart) return;
    chart.setData(alignedData(), false);
    const anchor = getViewAnchor();
    chart.setScale('x', { min: anchor - windowSec, max: anchor });
}

/**
 * Rebuild the chart (called by theme.js on theme toggle — canvas strokes are
 * read from CSS vars only at build time, so a var swap alone won't repaint).
 */
export function rebuildCanChart() {
    buildCanChart();
}
