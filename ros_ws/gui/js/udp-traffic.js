/**
 * udp-traffic.js — UDP message-rate view of the topics panel (#panel-topics).
 *
 * The left-sidebar panel has two mutually exclusive views, switched by the
 * ROS/UDP segmented control in its header:
 *
 *   ROS  the original topic monitor (panels.js) — messages THIS BROWSER
 *        received per ROS topic.
 *   UDP  this module — true per-message-type frame rates on the Jetson ↔
 *        can-bridge UDP link (ports 5005/5006), both directions.
 *
 * *** WHY THE COLUMN SAYS msg/s AND NEVER "Hz". ***  The two views measure
 * different things and the units must not look interchangeable:
 *
 *   ROS view   a browser-received rate.  Spy subscriptions are throttled to
 *              200 ms in ros-bridge.js, so its Hz column SATURATES AT 5 for
 *              every spied topic no matter how fast the publisher runs.  It
 *              answers "what is reaching this page".
 *   UDP view   the true wire rate.  teensy_bridge_node counts every frame in
 *              teensy_link's RX/TX paths and publishes CUMULATIVE counters at
 *              1 Hz; this module differences two counter samples over the
 *              selected window.  Throttling the /udp_diag topic would change
 *              how often the counters are sampled, never the rate they encode.
 *              It answers "what is on the wire".
 *
 * Data sources (both diagnostic_msgs/DiagnosticStatus, from teensy_bridge_node):
 *   'udp_diag'     1 Hz — rx_<TYPE>/tx_<TYPE> cumulative counters (MsgType enum
 *                  NAMES, upper case), plus the lower case aggregates
 *                  rx_frames/tx_frames/crc_errors/decode_errors/drain_capped/
 *                  seq_gaps.
 *   'link_status' 10 Hz — bridge_link (UP/LOST/NO_HEARTBEAT), the bridge's own
 *                  verdict on the Teensy uplink, which GATES this whole view
 *                  (see "Staleness state"), and latency_monitor, the alarmed
 *                  latency verdict token rendered in the footer.
 *
 * *** WHY LATENCY IS IN THIS PANEL AT ALL. ***  The worst outage this project
 * has had (the 2026-07-18 → 2026-08-15 uptime-lag arc, closed by can-bridge
 * FW 14) was a pure LATENCY failure: the frames all arrived, just late, and
 * every loss counter read clean through it.  A panel that shows only rates and
 * error totals is blind to that whole failure class, so the bridge's own
 * latency verdict rides in the footer above the loss numbers.
 *
 *   'clock_diag'  ~1 per 30 s — rtt_us, the time-sync anchor's round-trip
 *                  time.  The ONLY live latency in milliseconds this system
 *                  publishes anywhere: 1–3 ms is healthy, and it is the number
 *                  that exonerated the transport during the uptime-lag arc
 *                  (flat 1–3 ms at 63 h uptime while the lag read ~240 ms).
 *                  It is NOT an end-to-end telemetry latency — nothing
 *                  publishes one — so the footer labels it 'anchor rtt'.
 *
 * Derivation:  msg/s = Δcount / Δt between the newest sample and the oldest
 * sample still inside the window, using CLIENT ARRIVAL timestamps (the message
 * carries no window stamp).  Window presets are 5/10/30/60 s: at the 1 Hz
 * publish cadence a 1 s window would hold a single sample and could not
 * produce a rate at all — the lesson the CAN panel's presets already encode.
 *
 * Σ rx_<TYPE> ≤ rx_frames by construction: rx_frames counts every datagram,
 * including those that then failed CRC or structural decode, and any frame of
 * a msg_type this firmware/enum pair does not name.  The aggregate footer
 * shows both so the gap is visible rather than silently absorbed.
 *
 * No imports: this panel is a table, not a chart — it needs neither the
 * geometry constants nor telemetry-charts' uPlot gap hook.
 */

// ---- Key contract with teensy_bridge_node._publish_udp_diag ----
//
// Per-type keys are '<dir>_<MSGTYPE_NAME>' with the enum name in UPPER case;
// aggregates are lower case.  The case split is what lets this consumer work
// from a pattern instead of a hardcoded MsgType inventory — a firmware/enum
// addition shows up as a new row here with no GUI change at all.  Pinned
// producer-side by TestUdpDiagKeyValueContract (tests/ros/test_gui_geometry.py).
//
// 'gap' is deliberately NOT a prefix here: the wire carries ONE shared sequence
// counter per (channel, direction), so a per-type gap tally only counted how
// often another type interleaved, never loss
// (logbook/2026-08-25-udp-gap-column-artifact.md).
const PER_TYPE_KEY_RE = /^(rx|tx)_([A-Z][A-Z0-9_]*)$/;
const AGGREGATE_KEYS = [
    'rx_frames', 'tx_frames', 'crc_errors', 'decode_errors', 'drain_capped',
    'seq_gaps',
];

/** One-shot RUNTIME half of that contract.  The test catches drift inside the
 *  repo; this catches the case the test cannot see — a browser holding this
 *  GUI build against a bridge node running an older one, where the absent rows
 *  would otherwise render as a permanent, unexplained '--'. */
let aggregatesChecked = false;
function checkAggregates(kv) {
    if (aggregatesChecked) return;
    aggregatesChecked = true;
    const missing = AGGREGATE_KEYS.filter(k => !(k in kv));
    if (missing.length) {
        console.warn('udp_diag is missing aggregate KeyValues:', missing,
                     '— the bridge node is older than this GUI build');
    }
}

/** The same one-shot idiom for the footer's two OTHER feeds.
 *
 *  A missing MESSAGE and a missing KEY are different faults with different
 *  fixes — a dead topic or subscription, versus a bridge node older than this
 *  GUI build — and the footer renders '--' for both, so the two have to be told
 *  apart explicitly.  The tooltips below say which one it is; these warnings
 *  put the same distinction in the console, once, for whoever opens it. */
let latencyKeyChecked = false;
function checkLatencyKey(kv) {
    if (latencyKeyChecked) return;
    latencyKeyChecked = true;
    if (!('latency_monitor' in kv)) {
        console.warn("link_status carries no 'latency_monitor' KeyValue — the "
                     + 'bridge node is older than this GUI build (rebuild with '
                     + 'colcon); the footer verdict stays UNKNOWN, not OK');
    }
}

let rttKeyChecked = false;
function checkRttKey(kv) {
    if (rttKeyChecked) return;
    rttKeyChecked = true;
    if (!('rtt_us' in kv)) {
        console.warn("clock_diag carries no 'rtt_us' KeyValue — the bridge "
                     + 'node is older than this GUI build (rebuild with '
                     + "colcon); the footer's anchor rtt stays --");
    }
}

// ---- Latency verdict (link_status 'latency_monitor') ----
//
// The bridge node publishes ONE token, never a number, and publishes it
// unconditionally (teensy_bridge_node.py:4916): the individual figures behind
// it — ring leak, cache-age floor, lead-clamp duty — are already bagged on
// /ring_diag, /cache_diag and /link_status, so repeating them here would be a
// second, drifting copy of a number the bag already has.
//
// The rank order below MIRRORS _LATENCY_MONITOR_RANK in
// teensy_bridge_node.py:304-308 and is pinned against that source text by
// TestUdpPanelLatencyMonitor (tests/ros/test_gui_geometry.py).  It is CAUSAL,
// not severity: the RX-ring leak is the earliest precursor, the cache-age floor
// is the stale-feedback symptom downstream of it, and the clamp duty is the
// last and most visible link in the same chain.  Ranking this way makes
// "worst-in-session" name the most UPSTREAM condition seen, i.e. the one whose
// fix subsumes the others.
const LATENCY_MONITOR_OK = 'OK';
const LATENCY_MONITOR_RANK = {
    OK: 0,
    CLAMP_DUTY: 1,
    CACHE_AGE: 2,
    RING_LEAK: 3,
};
/** Rank an UNRECOGNISED token above every known cause.
 *  A token this page does not know comes from a bridge node NEWER than this GUI
 *  build — a condition added after this file was written, whose place in the
 *  causal chain we cannot know and therefore must not guess downwards.  Ranking
 *  it top means worst-in-session surfaces it instead of hiding it behind a
 *  named cause, and the operator is pointed at the thing this page cannot
 *  explain.  Matches `_LATENCY_MONITOR_UNKNOWN_RANK` in
 *  tests/hardware/tilt_cal_grid.py, the offline harness that reduces the same
 *  tokens — all three copies are pinned together by TestUdpPanelLatencyMonitor. */
const LATENCY_MONITOR_UNKNOWN_RANK = 4;
function latencyRank(token) {
    const r = LATENCY_MONITOR_RANK[token];
    return (r === undefined) ? LATENCY_MONITOR_UNKNOWN_RANK : r;
}

// ---- Nominal wire rates (rate conformance) ----
//
// The msg/s column is only readable if you already know what each flow SHOULD
// run at.  This table supplies that reference for the STEADY flows and nothing
// else: an event-driven type has no nominal, and inventing one would turn a
// quiet link into a permanent false alarm.  Types absent from this table are
// never annotated — RPC_REQUEST/RPC_RESPONSE, CMD_RESULT, HAND_CMD_ECHO,
// CONE_FRAME, PLATFORM_FRAME and DIAGNOSTIC are all event-driven.
//
// Every entry names the FIRMWARE constant it mirrors, and a drift tripwire
// (TestUdpNominalRates, tests/ros/test_gui_geometry.py) compares the numbers
// against those headers — so a firmware retune fails the suite here instead of
// silently making the operator's reference wrong.
const NOMINAL_RATES = {
    // TELEM_RATE_HZ = 100, canbridge_config.h:96.  The 250 two lines above it
    // is the BENCH_SYSID_BUILD override, which the robot's image never takes.
    rx_TELEMETRY: { hz: 100 },
    // Same tick, same frame: telemetry_step() calls send_telemetry(),
    // send_bb_estimates() and send_leg_cmd() back-to-back (telemetry.cpp:619-622)
    // and all three sends are unconditional, so they share TELEM_RATE_HZ.
    rx_BB_AXIS_ESTIMATES: { hz: 100 },
    rx_LEG_CMD: { hz: 100 },
    // JBBallDetect::CHECK_INTERVAL_MS = 20 ms, hardware_config.h:524 (the 50 ms
    // at :512 belongs to BBBallDetect, a different robot).  The uplink sends on
    // a new poll reply with a 1 Hz keepalive floor (HAND_SENSOR_KEEPALIVE_US,
    // telemetry.cpp:249), so a healthy poller is 50 msg/s and a DROP TOWARDS
    // 1 msg/s is itself the diagnosis: the poller stopped getting replies.
    //
    // FW 15 REALITY: 50 is the CONFIGURED rate, and the image on the bench does
    // not reach it — the flashed FW 15 poller measures ~42 msg/s (20 ms p50 /
    // 30 ms p95, ~38 % of cycles taking the slow mode), diagnosed and fixed in
    // FW 16 (canbridge_config.h:44, the 15→16 changelog entry).  Until that
    // flash this row sits LOW in its band by design — ~42 against 50 is inside
    // the ±25 % band, so it does not colour, but an operator comparing against
    // the stated nominal is not seeing a fault.
    rx_HAND_SENSOR: { hz: 50 },
    // JbUdp::HEARTBEAT_HZ = 10, udp_protocol.h:28 — both directions, one
    // constant (canbridge_config.h's HEARTBEAT_RATE_HZ just aliases it).
    rx_HEARTBEAT_T2J: { hz: 10 },
    tx_HEARTBEAT_J2T: { hz: 10 },
    // BIMODAL: 0 while nothing streams, ~40 while trajectory_node does.
    // NEITHER is a fault, and a window straddling a stream's start reads
    // somewhere in between — so only an OVERSHOOT above the band is a
    // deviation here (see rateDeviates).
    tx_SETPOINT: { hz: 40, bimodal: true },
};

/** Conformance band, fraction of nominal.  ±25% is chosen against the two
 *  things that legitimately move these numbers: a window whose edges do not
 *  line up with the 1 Hz counter publishes, and flows that are periodic but
 *  not perfectly so — HAND_SENSOR is ROUND-TRIP PACED, not on-change: it sends
 *  once per poll REPLY (telemetry.cpp:273-275 gates on the reply's wall stamp
 *  changing, plus a flags flip, never on the ball verdict changing value), so
 *  its cadence carries the CAN round-trip jitter of the poll.  It keeps a 100 msg/s
 *  stream quiet from 75 to 125 while still flagging every failure this panel
 *  is for — a halved telemetry rate, a hand poller fallen back to its 1 Hz
 *  keepalive, a heartbeat that stopped. */
const RATE_TOLERANCE = 0.25;

/** True when a rate is outside its flow's nominal band.  No nominal, or no
 *  trustworthy rate, means no verdict — never a deviation by default. */
function rateDeviates(key, rate) {
    const nom = NOMINAL_RATES[key];
    if (!nom || !Number.isFinite(rate)) return false;
    if (rate > nom.hz * (1 + RATE_TOLERANCE)) return true;
    if (nom.bimodal) return false;   // no low side to police
    return rate < nom.hz * (1 - RATE_TOLERANCE);
}

/** The 'expected …' clause for a row's hover, or '' for an exempt type. */
function nominalNote(dir, key) {
    const nom = NOMINAL_RATES[key];
    if (!nom) return '';
    return nom.bimodal
        ? ` — ${dir} expected 0 (idle) or ~${nom.hz} msg/s (streaming)`
        : ` — ${dir} expected ~${nom.hz} msg/s ±${Math.round(RATE_TOLERANCE * 100)}%`;
}

// ---- Constants ----

const WINDOW_PRESETS = [5, 10, 30, 60];
const DEFAULT_WINDOW_SEC = 10;
// Ring capacity: the 60 s preset needs 61 samples at 1 Hz (N samples span N-1
// intervals); +10 for cadence jitter and the odd double-publish.
const RING_CAP = 72;
const WINDOW_STORAGE_KEY = 'jugglebot-udp-window';
/** Which view the shared #panel-topics shows.  Persisted beside
 *  'jugglebot-hidden-topics' (panels.js), same panel, same operator choice. */
const MODE_STORAGE_KEY = 'jugglebot-topics-mode';
const MODE_ROS = 'ros';
const MODE_UDP = 'udp';

// No 'udp_diag' MESSAGE for >3 s ⇒ diagStale.  Same watchdog class as the CAN
// panel's: it covers bridge-node / rosbridge / subscription death only.  A dead
// TEENSY UPLINK does NOT stop these messages — the bridge node keeps publishing
// counters that no longer advance — which is why linkDown exists below.
const STALE_TIMEOUT_MS = 3000;
/** Below this Δt two samples are too close to divide by (a double publish). */
const MIN_DT_S = 0.5;

// ---- State ----

/** Ring of {t: unixSeconds, kv: {key: number}} counter samples. */
const samples = [];
/** Publication order of the per-type names, learned from the newest message —
 *  the producer emits them in MsgType (id) order, which is a stable, meaningful
 *  order (downlink types first).  Never re-sorted by rate: a table whose rows
 *  reshuffle every second is unreadable. */
let typeOrder = [];

/** Latest 'latency_monitor' token, and the worst one seen since this page
 *  loaded.  Page load is the reset, matching the per-session scope of the
 *  offline harness helper this mirrors (worse_latency_monitor,
 *  tests/hardware/tilt_cal_grid.py) — a condition that cleared itself is still
 *  the truth about the session, and the footer says so. '' = none received. */
let latencyToken = '';
let latencyWorst = '';
/** True once ANY 'link_status' message has arrived.  Separates "no message"
 *  from "message without the key" — see checkLatencyKey. */
let linkStatusSeen = false;
/** Previous 'bridge_link' value, for the repaint edge below.  '' = none yet. */
let prevBridgeLink = '';
/** Latest 'clock_diag' rtt_us, or NaN.  Deliberately NOT watchdogged: anchors
 *  are ~30 s apart in steady state, so the 3 s staleness class the two other
 *  feeds use would false-fire permanently on this one. */
let anchorRttUs = NaN;
/** True once ANY 'clock_diag' message has arrived — same distinction as
 *  linkStatusSeen, for the anchor-rtt half of the footer. */
let clockDiagSeen = false;

let windowSec = DEFAULT_WINDOW_SEC;
let mode = MODE_ROS;
let diagStaleTimer = null;
let healthStaleTimer = null;
let repaintTimer = null;

// ---- Staleness state ----
//
// Four INDEPENDENT causes, composed in one place (applyStaleUI) so they never
// stomp each other — the can-traffic.js contract, for the same reasons:
//
//   diagStale    no 'udp_diag' for >3 s — the bridge node, rosbridge or this
//                page's subscription died (transport liveness).
//   healthStale  no 'link_status' for >3 s — same class, different topic.  It
//                matters on its own because bridge_link is what gates the view:
//                without it the last received verdict would be trusted forever.
//   linkDown     the latest 'link_status' carried bridge_link != 'UP'.  THE
//                DECEPTION MODE THIS PANEL EXISTS TO REFUSE: /udp_diag keeps
//                arriving on time from a live bridge node while its counters
//                sit frozen, so every rate differences to a clean 0.0 and the
//                table renders a plausible, entirely idle-looking link.  A dead
//                uplink must read as LINK DOWN, never as a table of zeros.
//   rosDown      the rosbridge websocket itself is down (setUdpTrafficRosLink,
//                from main.js's connection-state router) — no source of truth
//                for anything.
//
// UI derivation (applyStaleUI is the single writer for stale visuals):
//   badge   shown when ANY cause is active; tooltip names the live causes
//   rates   '--' under ANY cause (a rate is only meaningful when the counters
//           behind it are both fresh AND trustworthy)
//   counts  the TABLE carries no cumulative column at all, so nothing in a row
//           can print a live-looking total while its rates read '--' (the row
//           hover blanks with them too).  The only cumulative figures left in
//           the panel are the aggregate error/loss totals in the footer: they
//           are labelled "since node start" in the footer text itself and
//           dimmed via the .udp-stale class, and they deliberately KEEP their
//           last values — a crc/decode/seq-gap total is the truth about a link
//           precisely when that link has just gone down.  The latency verdict
//           and the anchor RTT beside them keep their last values for the same
//           reason, and for the same reason are dimmed rather than blanked.
const staleState = {
    diagStale: false,
    healthStale: false,
    linkDown: false,
    rosDown: false,
};
/** Human-readable linkDown cause for the badge tooltip. */
let linkDownReason = '';

// ---- DOM helpers ----

function el(id) {
    return document.getElementById(id);
}

/** Parse DiagnosticStatus KeyValue[] into a plain object (string values). */
function kvMap(msg) {
    const kv = {};
    for (const v of (msg.values || [])) kv[v.key] = v.value;
    return kv;
}

// ---- Panel init ----

export function initUdpTrafficPanel() {
    const host = el('udp-table-container');
    if (!host) return;

    const savedWin = parseInt(localStorage.getItem(WINDOW_STORAGE_KEY), 10);
    if (WINDOW_PRESETS.includes(savedWin)) windowSec = savedWin;
    const savedMode = localStorage.getItem(MODE_STORAGE_KEY);
    if (savedMode === MODE_UDP || savedMode === MODE_ROS) mode = savedMode;

    host.innerHTML = `
        <div id="udp-window-btns" role="group" aria-label="Rate window"></div>
        <div id="udp-rows"><div class="topic-empty">Waiting for udp_diag…</div></div>
        <div class="udp-aggregates" id="udp-aggregates"></div>`;

    const btnHost = el('udp-window-btns');
    if (btnHost) {
        btnHost.innerHTML = WINDOW_PRESETS.map(sec =>
            `<button type="button" class="can-window-btn udp-window-btn" data-sec="${sec}"
                     title="Average each rate over the last ${sec} s (counters arrive at 1 Hz, so this is ${sec} samples)">${sec}s</button>`
        ).join('');
        btnHost.querySelectorAll('.udp-window-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                windowSec = parseInt(btn.dataset.sec, 10);
                localStorage.setItem(WINDOW_STORAGE_KEY, String(windowSec));
                updateWindowButtonsUI();
                paint();
            });
        });
        updateWindowButtonsUI();
    }

    // Mode switch (ROS ⇄ UDP).  Both halves are <button>s: a button inside a
    // .panel-header is inert to initCollapsiblePanels' interactive-element
    // guard, so clicking one can never double as a collapse gesture.
    const modeHost = el('topic-mode-btns');
    if (modeHost) {
        modeHost.querySelectorAll('button[data-mode]').forEach(btn => {
            btn.addEventListener('click', () => setMode(btn.dataset.mode));
        });
    }
    applyMode();

    // 1 Hz repaint: the window's trailing edge moves even when no message
    // arrives, so a rate that is going stale must visibly change.  paint()
    // early-returns while the ROS view is showing.
    repaintTimer = setInterval(paint, 1000);

    // Arm the watchdogs immediately so a never-connected page reads honestly
    // 3 s after load rather than sitting on "Waiting…" forever.
    armDiagWatchdog();
    armHealthWatchdog();
}

// ---- Mode switching ----

function setMode(next) {
    if (next !== MODE_ROS && next !== MODE_UDP) return;
    if (mode === next) return;
    mode = next;
    localStorage.setItem(MODE_STORAGE_KEY, mode);
    applyMode();
}

/** Show exactly one view.  The ROS monitor keeps running while hidden
 *  (panels.js's 1 Hz updateTopicMonitor writes into a display:none container),
 *  so switching back shows current data immediately, not an empty table. */
function applyMode() {
    const udpOn = (mode === MODE_UDP);

    const topicTable = el('topic-table-container');
    if (topicTable) topicTable.style.display = udpOn ? 'none' : '';
    const udpTable = el('udp-table-container');
    if (udpTable) udpTable.style.display = udpOn ? '' : 'none';

    // The ROS window label is the ROS view's own control; the UDP view has its
    // own presets inside the table container.
    const label = el('topic-window-label');
    if (label) label.style.display = udpOn ? 'none' : '';

    const title = el('topics-panel-title');
    if (title) title.textContent = udpOn ? 'UDP Messages' : 'ROS2 Topics';

    const modeHost = el('topic-mode-btns');
    if (modeHost) {
        modeHost.querySelectorAll('button[data-mode]').forEach(btn => {
            const on = (btn.dataset.mode === mode);
            btn.classList.toggle('active', on);
            btn.setAttribute('aria-pressed', String(on));
        });
    }

    applyStaleUI();   // the badge belongs to the UDP view only
    paint();
}

// ---- Message handlers (called from main.js router) ----

/**
 * Handle a 'udp_diag' message (1 Hz): append a counter sample.
 *
 * NOTE the deliberate divergence from can-traffic.js, which DROPS a profile
 * payload that arrives while the uplink is down.  It has to: its payload is
 * the bridge's frozen CACHE of the last heartbeat, so appending it would draw
 * a flat line of live-looking lies.  A COUNTER is different — it is a total,
 * and a total that stopped advancing is the truth about a link with no
 * traffic.  Keeping those samples is what makes the recovery window honest: a
 * 10 s window spanning a 3 s outage then averages ~70 msg/s on a 100 msg/s
 * stream, which is exactly what happened.  Dropping them would rewrite the
 * outage out of the history and report a clean 100.
 *
 * @param {object} msg - diagnostic_msgs/DiagnosticStatus
 */
export function udpTrafficOnDiag(msg) {
    if (staleState.diagStale) {
        staleState.diagStale = false;
        applyStaleUI();
    }
    armDiagWatchdog();

    const raw = kvMap(msg);
    const kv = {};
    const order = [];
    const seen = new Set();
    for (const key of Object.keys(raw)) {
        const n = parseInt(raw[key], 10);
        if (!Number.isFinite(n)) continue;      // never trust a non-numeric row
        kv[key] = n;
        const m = PER_TYPE_KEY_RE.exec(key);
        if (m && !seen.has(m[2])) {
            seen.add(m[2]);
            order.push(m[2]);
        }
    }
    if (order.length) typeOrder = order;
    checkAggregates(kv);

    const now = Date.now() / 1000;

    // Counter RESET (the bridge node restarted): every count in the ring is
    // from a different process, so differencing across the boundary would
    // produce a large negative — or, worse, a plausible positive on a counter
    // that happened to restart above its old value.  Drop the history and
    // start again from this sample.
    const prev = samples.length ? samples[samples.length - 1] : null;
    if (prev && countersWentBackwards(prev.kv, kv)) samples.length = 0;

    samples.push({ t: now, kv });
    while (samples.length > RING_CAP) samples.shift();
    paint();
}

/** True when any shared counter decreased — only a restart can do that. */
function countersWentBackwards(prevKv, kv) {
    for (const key of Object.keys(kv)) {
        if (key in prevKv && kv[key] < prevKv[key]) return true;
    }
    return false;
}

/**
 * Handle a 'link_status' message (10 Hz): gate the view on bridge_link, and
 * take the latency verdict for the footer.
 * @param {object} msg - diagnostic_msgs/DiagnosticStatus
 */
export function udpTrafficOnLinkStatus(msg) {
    const kv = kvMap(msg);
    linkStatusSeen = true;
    checkLatencyKey(kv);

    // Latency verdict.  The repaint is deferred to the bottom of this handler
    // so it renders THIS message's link state, not the previous one's.
    const token = kv.latency_monitor || '';
    const latencyChanged = Boolean(token) && token !== latencyToken;
    if (latencyChanged) {
        latencyToken = token;
        if (!latencyWorst || latencyRank(token) > latencyRank(latencyWorst)) {
            latencyWorst = token;
        }
    }

    // A missing key is treated as down — honest-unknown beats a frozen table
    // of zeros (see "Staleness state").
    const bridgeLink = kv.bridge_link || 'UNKNOWN';
    if (bridgeLink !== 'UP') {
        const age = kv.heartbeat_age_ms;
        linkDownReason = `Teensy uplink lost (bridge_link=${bridgeLink}`
            + ((age && age !== 'n/a') ? `, heartbeat age ${age} ms` : '')
            + ') — counters are frozen, so every rate below would read 0';
        staleState.linkDown = true;
    } else {
        staleState.linkDown = false;
        linkDownReason = '';
    }

    staleState.healthStale = false;
    armHealthWatchdog();
    applyStaleUI();
    // Only on a CHANGE, but on EITHER change this handler carries: the latency
    // token, or bridge_link.  This topic runs at 10 Hz and both values hold for
    // minutes at a time, so an unconditional repaint would be 10 Hz of DOM
    // writes carrying no new information — while gating on the token alone left
    // a LINK DOWN transition waiting up to a second for the 1 Hz repaint timer,
    // which is the one edge an operator most needs immediately.
    const linkChanged = bridgeLink !== prevBridgeLink;
    prevBridgeLink = bridgeLink;
    if (latencyChanged || linkChanged) paint();
}

/**
 * Handle a 'clock_diag' message (~1 per 30 s): take the anchor RTT.
 *
 * One message per ACCEPTED time-of-day anchor, so this is a series, not a
 * cache — the footer shows the latest sample only, which is all a health
 * glance needs (the series itself is the bag's job).
 *
 * @param {object} msg - diagnostic_msgs/DiagnosticStatus
 */
export function udpTrafficOnClockDiag(msg) {
    const kv = kvMap(msg);
    clockDiagSeen = true;
    checkRttKey(kv);
    const us = parseInt(kv.rtt_us, 10);
    if (!Number.isFinite(us)) return;      // never trust a non-numeric row
    anchorRttUs = us;
    paint();
}

/**
 * ROS2 connection edge — called from main.js's connection-state router for
 * every state, not just the down edge.
 *
 * @param {boolean} isUp - true ONLY for the 'connected' state (the reconnect
 *   loop oscillates connecting↔disconnected every 2 s; both are down).
 */
export function setUdpTrafficRosLink(isUp) {
    if (staleState.rosDown === !isUp) return;   // no edge — idempotent
    staleState.rosDown = !isUp;
    applyStaleUI();
    paint();
}

// ---- Staleness ----

function armDiagWatchdog() {
    if (diagStaleTimer) clearTimeout(diagStaleTimer);
    diagStaleTimer = setTimeout(() => {
        staleState.diagStale = true;
        applyStaleUI();
        paint();
    }, STALE_TIMEOUT_MS);
}

function armHealthWatchdog() {
    if (healthStaleTimer) clearTimeout(healthStaleTimer);
    healthStaleTimer = setTimeout(() => {
        staleState.healthStale = true;
        applyStaleUI();
        paint();
    }, STALE_TIMEOUT_MS);
}

/** Every live stale cause, worst-first, for the badge tooltip. */
function staleCauses() {
    const causes = [];
    if (staleState.rosDown) causes.push('ROS2 disconnected');
    if (staleState.linkDown) causes.push(linkDownReason);
    if (staleState.diagStale) causes.push("no 'udp_diag' from bridge >3 s");
    if (staleState.healthStale) causes.push("no 'link_status' from bridge >3 s");
    return causes;
}

/**
 * Short headline for the banner: the WORST live cause, named for what it is.
 *
 * This is R5's actual requirement — "LINK DOWN" has to be readable AS link
 * down.  A row of '--'s plus a generic STALE chip would leave an operator to
 * infer the difference between "the uplink died" and "this page lost
 * rosbridge", which are opposite diagnoses with opposite next actions.
 */
function staleHeadline() {
    if (staleState.rosDown) return 'ROS2 DISCONNECTED';
    if (staleState.linkDown) return 'LINK DOWN';
    if (staleState.diagStale) return 'NO udp_diag';
    if (staleState.healthStale) return 'NO link_status';
    return '';
}

/** True when no rate on this panel can be trusted. */
function ratesUntrusted() {
    return staleState.rosDown || staleState.linkDown
        || staleState.diagStale || staleState.healthStale;
}

/** Single writer for the stale visuals (badge + the dim class on the table). */
function applyStaleUI() {
    const causes = staleCauses();
    const badge = el('udp-stale-badge');
    if (badge) {
        // The badge lives in the shared panel header — it must not shout about
        // the UDP link while the ROS view is showing.
        badge.style.display = (mode === MODE_UDP && causes.length) ? '' : 'none';
        if (causes.length) badge.title = `UDP data stale: ${causes.join('; ')}`;
    }
    const host = el('udp-table-container');
    if (host) host.classList.toggle('udp-stale', ratesUntrusted());
}

// ---- Window preset UI ----

function updateWindowButtonsUI() {
    const btnHost = el('udp-window-btns');
    if (!btnHost) return;
    btnHost.querySelectorAll('.udp-window-btn').forEach(btn => {
        btn.classList.toggle('active', parseInt(btn.dataset.sec, 10) === windowSec);
    });
}

// ---- Rate derivation ----

/**
 * The pair of samples the current window resolves to, or null.
 *
 * Base = the OLDEST sample still inside [newest.t - windowSec, newest.t].  A
 * window holding one sample (a fresh page, or the first message after an
 * outage longer than the window) yields no pair and therefore no rate — '--',
 * never a fabricated 0.
 */
function windowPair() {
    if (samples.length < 2) return null;
    const newest = samples[samples.length - 1];
    const cutoff = newest.t - windowSec;
    let base = null;
    for (const s of samples) {
        if (s.t >= cutoff) { base = s; break; }
    }
    if (!base || base === newest) return null;
    const dt = newest.t - base.t;
    if (!(dt >= MIN_DT_S)) return null;
    return { base, newest, dt };
}

/** Δcount/Δt for one key, or NaN when the pair cannot support it. */
function rateOf(pair, key) {
    if (!pair) return NaN;
    const a = pair.base.kv[key];
    const b = pair.newest.kv[key];
    if (!Number.isFinite(a) || !Number.isFinite(b)) return NaN;
    return (b - a) / pair.dt;
}

/** Format a rate: '--' when unknown, one decimal under 10 (SETPOINT at 40 and
 *  TELEMETRY at 100 want integers; a 0.1 msg/s trickle wants the decimal). */
function fmtRate(v) {
    if (!Number.isFinite(v)) return '--';
    if (v === 0) return '0';
    return v < 10 ? v.toFixed(1) : String(Math.round(v));
}

// ---- Render ----

function paint() {
    if (mode !== MODE_UDP) return;    // hidden — skip the DOM work entirely
    const rowsHost = el('udp-rows');
    if (!rowsHost) return;

    const causes = staleCauses();
    // The banner is the R5 headline: an outage names ITSELF, above the data,
    // instead of leaving an operator to infer it from a table of '--'.
    const banner = causes.length
        ? `<div class="udp-banner" title="${causes.join('; ')}">`
          + `${staleHeadline()} — rates unavailable</div>`
        : '';

    if (!samples.length) {
        rowsHost.innerHTML =
            banner + '<div class="topic-empty">Waiting for udp_diag…</div>';
        paintAggregates(null, null);
        return;
    }

    const newest = samples[samples.length - 1];
    const pair = ratesUntrusted() ? null : windowPair();

    // No gaps column: per-type gaps are shared wire-seq artifacts (plans/archived/udp-channel-health.md).
    let html = banner + `<table class="topic-table udp-table">
        <thead><tr>
            <th class="col-topic" title="teensy_link MsgType — the wire message type, by enum name">Type</th>
            <th class="col-rate" title="Jetson ← Teensy, true wire rate over the last ${windowSec} s (NOT the ROS view's browser-received Hz). Steady flows with a firmware nominal turn amber outside ±${Math.round(RATE_TOLERANCE * 100)}%; hover a row for its expected rate. Event-driven types have no nominal and are never flagged.">rx (msg/s)</th>
            <th class="col-rate" title="Jetson → Teensy, true wire rate over the last ${windowSec} s (NOT the ROS view's browser-received Hz). Steady flows with a firmware nominal turn amber outside ±${Math.round(RATE_TOLERANCE * 100)}%; hover a row for its expected rate. Event-driven types have no nominal and are never flagged.">tx (msg/s)</th>
        </tr></thead><tbody>`;

    for (const name of typeOrder) {
        const rxKey = `rx_${name}`;
        const txKey = `tx_${name}`;
        const rxRate = rateOf(pair, rxKey);
        const txRate = rateOf(pair, txKey);
        const idle = !(rxRate > 0) && !(txRate > 0);
        // The hover totals are CUMULATIVE, a different unit from the msg/s
        // columns — so they say so, and they blank exactly when the rates do.
        // A frozen total beside a '--' rate is the deception this panel exists
        // to refuse (see "Staleness state").
        const title = pair
            ? `${name}: cumulative since node start — rx ${newest.kv[rxKey] || 0}`
              + ` / tx ${newest.kv[txKey] || 0} frames`
              + nominalNote('rx', rxKey) + nominalNote('tx', txKey)
            : `${name}: ${staleHeadline() || 'not enough samples yet'} — rates and`
              + ` cumulative totals unavailable`;
        // Conformance is only asserted on a TRUSTWORTHY rate: rateDeviates
        // returns false for the NaN a missing window pair produces, so a stale
        // or down link colours nothing and the banner stays the whole story.
        const rxCls = rateDeviates(rxKey, rxRate) ? ' udp-rate-off' : '';
        const txCls = rateDeviates(txKey, txRate) ? ' udp-rate-off' : '';
        html += `<tr class="${idle ? 'udp-row-idle' : ''}" title="${title}">
            <td class="col-topic">${name}</td>
            <td class="col-rate${rxCls}">${fmtRate(rxRate)}</td>
            <td class="col-rate${txCls}">${fmtRate(txRate)}</td>
        </tr>`;
    }
    html += '</tbody></table>';
    // innerHTML replacement can reset scrollTop (engine-dependent), and this
    // table scrolls in the narrow sidebar — repaint must not yank the view.
    const keepScroll = rowsHost.scrollTop;
    rowsHost.innerHTML = html;
    rowsHost.scrollTop = keepScroll;

    paintAggregates(newest, pair);
}

/**
 * The bridge's latency verdict, as one footer span.
 *
 * Two tokens, not one: the CURRENT verdict and the WORST seen since page load.
 * A latency condition that cleared itself — the uptime-lag arc's clamp duty
 * did exactly that between moves — leaves no trace in the current token, and
 * that is precisely the session an operator needs to be told about.
 */
function latencyHtml() {
    if (!latencyToken) {
        // Two different faults render the same '--', so the tooltip names which:
        // nothing is publishing link_status at all, or link_status is arriving
        // without the key (a bridge node older than this GUI build).  The fix
        // differs — chase the topic, or rebuild the workspace.
        const why = linkStatusSeen
            ? '\'link_status\' is arriving but carries no \'latency_monitor\' —'
              + ' the bridge node is older than this GUI build (rebuild with'
              + ' colcon)'
            : 'No \'link_status\' seen yet — the bridge node\'s latency verdict'
              + ' is UNKNOWN, not OK';
        return `<span class="udp-latency" title="${why}">latency --</span>`;
    }
    const title = 'Bridge latency verdict (link_status latency_monitor), and the'
        + ' worst seen since this page loaded. RING_LEAK / CACHE_AGE /'
        + ' CLAMP_DUTY name the most UPSTREAM active condition — the one whose'
        + ' fix subsumes the others. The numbers behind it are on /ring_diag,'
        + ' /cache_diag and /link_status.';
    const cls = (latencyToken === LATENCY_MONITOR_OK)
        ? 'udp-latency-ok' : 'udp-latency-alarm';
    // Show the worst only when it OUTRANKS the current token: repeating an
    // unchanged verdict twice would be noise, and hiding a cleared one would
    // be a lie by omission.
    const worse = latencyRank(latencyWorst) > latencyRank(latencyToken);
    return `<span class="udp-latency" title="${title}">latency `
        + `<span class="${cls}">${latencyToken}</span>`
        + (worse ? ` <span class="udp-latency-alarm">(worst ${latencyWorst})</span>`
                 : '')
        + '</span>';
}

/** The time-sync anchor RTT, in ms — the one live latency NUMBER on this page.
 *  '--' until the first anchor, which is honest: on a cold link the first one
 *  can be tens of seconds away. */
function anchorRttHtml() {
    const base = 'Round-trip time of the last accepted time-sync anchor'
        + ' (clock_diag rtt_us), ~1 per 30 s. 1–3 ms is healthy. This is the'
        + ' TRANSPORT round trip, NOT an end-to-end telemetry latency —'
        + ' nothing publishes one of those.';
    const ok = Number.isFinite(anchorRttUs);
    // Same no-message / no-key split as latencyHtml: on this feed a missing
    // MESSAGE is often innocent (the first anchor can be tens of seconds away
    // on a cold link), while a message without the key never is.
    const why = clockDiagSeen
        ? ' No number: \'clock_diag\' is arriving but carries no numeric'
          + ' \'rtt_us\' — the bridge node is older than this GUI build'
          + ' (rebuild with colcon).'
        : ' No number yet: no \'clock_diag\' message seen at all. On a cold link'
          + ' the first anchor can be tens of seconds away; a permanent -- means'
          + ' the time sync never completed an exchange.';
    const title = ok ? base : base + why;
    const txt = ok ? `${(anchorRttUs / 1000).toFixed(1)} ms` : '--';
    return `<span title="${title}">anchor rtt ${txt}</span>`;
}

/**
 * The aggregate footer.  rx_frames counts EVERY datagram, so it is ≥ the sum
 * of the per-type rows above; the difference is CRC failures, structural
 * decode failures and frames of a type this build cannot name.  Showing the
 * error totals next to it is what makes that difference readable instead of
 * an unexplained discrepancy.
 *
 * The latency verdict leads, and it renders even before the first 'udp_diag':
 * it comes from a DIFFERENT topic at 10 Hz, so blanking it while waiting for
 * counters would hide a live verdict behind an unrelated silence.
 */
function paintAggregates(newest, pair) {
    const host = el('udp-aggregates');
    if (!host) return;
    if (!newest) {
        host.innerHTML = latencyHtml() + anchorRttHtml();
        return;
    }
    const rx = fmtRate(rateOf(pair, 'rx_frames'));
    const tx = fmtRate(rateOf(pair, 'tx_frames'));
    const crc = newest.kv.crc_errors || 0;
    const dec = newest.kv.decode_errors || 0;
    const capped = newest.kv.drain_capped || 0;
    // The ONE honest loss number on this panel: an aggregate over the shared
    // wire sequence counter, tracked per socket (teensy_link's _track_seq).
    // Nonzero means the sequence really broke — unlike the per-type tally that
    // used to sit in the table, which counted type interleaving.  It counts
    // EVENTS, not frames (see the tooltip), so it bounds the loss from below.
    const gaps = newest.kv.seq_gaps || 0;
    const errCls = (crc || dec || gaps) ? ' udp-err' : '';
    host.innerHTML =
        latencyHtml() + anchorRttHtml()
        + `<span title="All frames, both directions, over the last ${windowSec} s">`
        + `frames ${rx} rx / ${tx} tx msg/s</span>`
        + `<span class="udp-agg-errors${errCls}" title="Cumulative since the bridge node started. `
        + `crc + decode failures are counted in rx_frames but in no per-type row, `
        + `which is why the rows can sum to less than the total. drain_capped = RX `
        + `drain hit its per-wakeup frame cap (backlog picked up on the next wakeup). `
        + `seq gaps = times the wire's per-socket sequence counter skipped. DISCONTINUITY `
        + `EVENTS, not a frame count: a skip of N frames scores 1, and a reorder scores 2. `
        + `Its STREAM half is the same quantity the Teensy counts for the downlink and is `
        + `directly comparable; the RPC half has no far-end counterpart at all.">`
        // EVERY number states its unit.  The confusion this panel already
        // caused once was exactly a unit mix — rates and cumulative totals
        // side by side in one table with neither labelled — so a cumulative
        // figure here names both its unit and its epoch.
        + `since node start: crc ${crc} frames · decode ${dec} frames`
        + ` · seq gaps ${gaps} events · capped ${capped} drains</span>`;
}
