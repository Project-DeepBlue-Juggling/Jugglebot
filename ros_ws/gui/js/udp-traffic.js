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
 *                  NAMES, upper case), gap_<TYPE> when nonzero, plus the lower
 *                  case aggregates rx_frames/tx_frames/crc_errors/
 *                  decode_errors/drain_capped.
 *   'link_status' 10 Hz — bridge_link (UP/LOST/NO_HEARTBEAT), the bridge's own
 *                  verdict on the Teensy uplink, which GATES this whole view
 *                  (see "Staleness state").
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
const PER_TYPE_KEY_RE = /^(rx|tx|gap)_([A-Z][A-Z0-9_]*)$/;
const AGGREGATE_KEYS = [
    'rx_frames', 'tx_frames', 'crc_errors', 'decode_errors', 'drain_capped',
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
//           the panel are the aggregate error totals in the footer: they are
//           labelled "cumulative" in the footer text itself and dimmed via the
//           .udp-stale class, and they deliberately KEEP their last values —
//           a crc/decode total is the truth about a link precisely when that
//           link has just gone down.
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
 * Handle a 'link_status' message (10 Hz): gate the view on bridge_link.
 * @param {object} msg - diagnostic_msgs/DiagnosticStatus
 */
export function udpTrafficOnLinkStatus(msg) {
    const kv = kvMap(msg);
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

    // No gaps column: per-type gaps are shared wire-seq artifacts (plans/active/udp-channel-health.md).
    let html = banner + `<table class="topic-table udp-table">
        <thead><tr>
            <th class="col-topic" title="teensy_link MsgType — the wire message type, by enum name">Type</th>
            <th class="col-rate" title="Jetson ← Teensy, true wire rate over the last ${windowSec} s (NOT the ROS view's browser-received Hz)">rx (msg/s)</th>
            <th class="col-rate" title="Jetson → Teensy, true wire rate over the last ${windowSec} s (NOT the ROS view's browser-received Hz)">tx (msg/s)</th>
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
            : `${name}: ${staleHeadline() || 'not enough samples yet'} — rates and`
              + ` cumulative totals unavailable`;
        html += `<tr class="${idle ? 'udp-row-idle' : ''}" title="${title}">
            <td class="col-topic">${name}</td>
            <td class="col-rate">${fmtRate(rxRate)}</td>
            <td class="col-rate">${fmtRate(txRate)}</td>
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
 * The aggregate footer.  rx_frames counts EVERY datagram, so it is ≥ the sum
 * of the per-type rows above; the difference is CRC failures, structural
 * decode failures and frames of a type this build cannot name.  Showing the
 * error totals next to it is what makes that difference readable instead of
 * an unexplained discrepancy.
 */
function paintAggregates(newest, pair) {
    const host = el('udp-aggregates');
    if (!host) return;
    if (!newest) {
        host.textContent = '';
        return;
    }
    const rx = fmtRate(rateOf(pair, 'rx_frames'));
    const tx = fmtRate(rateOf(pair, 'tx_frames'));
    const crc = newest.kv.crc_errors || 0;
    const dec = newest.kv.decode_errors || 0;
    const capped = newest.kv.drain_capped || 0;
    const errCls = (crc || dec) ? ' udp-err' : '';
    host.innerHTML =
        `<span title="All frames, both directions, over the last ${windowSec} s">`
        + `frames ${rx} rx / ${tx} tx msg/s</span>`
        + `<span class="udp-agg-errors${errCls}" title="Cumulative since the bridge node started. `
        + `crc + decode failures are counted in rx_frames but in no per-type row, `
        + `which is why the rows can sum to less than the total. drain_capped = RX `
        + `drain hit its per-wakeup frame cap (backlog picked up on the next wakeup).">`
        + `cumulative: crc ${crc} · decode ${dec} · capped ${capped}</span>`;
}
