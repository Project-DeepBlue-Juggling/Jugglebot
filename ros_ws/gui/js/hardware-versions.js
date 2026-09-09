/**
 * hardware-versions.js — Hardware panel (#panel-hardware, right sidebar,
 * directly below the Event Log).
 *
 * One row per (model, firmware version) pair across every board on the robot:
 * the two Teensys and the nine ODrives.  Devices of the same MODEL collapse
 * into a single row while they agree on firmware; the moment they don't, the
 * minority splits out onto its own row with an ODD badge, which is the whole
 * point of the panel — a half-flashed set of leg drives is invisible in every
 * other surface the GUI has.
 *
 * TWO SOURCES, AND THEY ARE NOT THE SAME KIND OF FACT:
 *
 *   MODEL  — static, from geometry-config.js HARDWARE_MODELS (generated from
 *            hardware_config.yaml -> hardware_models).  Declared, never
 *            verified: it changes only when somebody physically swaps a board.
 *            Neither Teensy reports its own board model on the wire, so there
 *            is nothing to read even if we wanted to (BridgeIdentity 0x8E is
 *            fw_version + protocol_version; the Platform's 0x6E0 reply is
 *            FW_VERSION).  See the YAML section header for the full rationale.
 *
 *   FIRMWARE — live, from 'link_status' (10 Hz, diagnostic_msgs/DiagnosticStatus,
 *            teensy_bridge_node._publish_link_status).  Three KeyValues:
 *              bridge_fw_version    _bridge_fw_version_str()
 *              platform_fw_version  _platform_fw_version_str()
 *              odrive_fw_versions   _odrive_fw_versions_str()
 *            Their exact rendering is pinned producer-side by
 *            TestHardwareVersionKeyValueContract (tests/ros/test_gui_geometry.py).
 *
 * The two Ball Butler ODrives (axes 7-8) have NO firmware version on any wire:
 * the can-bridge's Get_Version sweep covers the Jugglebot axes only
 * (Teensy_code_canbridge/version_check.h — the GET_AXIS_VERSIONS blob is a
 * fixed 7-axis array) and BB's heartbeat carries none.  They are listed anyway,
 * with the version column reading 'n/a' and a tooltip saying why.  Dropping the
 * rows would make "not checked" indistinguishable from "not present", which is
 * the failure mode every never-seen rendering in this codebase exists to avoid.
 *
 * SKEW is surfaced, never judged here.  The bridge and Platform rows carry the
 * node's own verdict text verbatim ('SKEW — expected v19', '0 (PRE-VERSIONING)')
 * because that comparison is against EXPECTED_BRIDGE_FW_VERSION /
 * PLATFORM_FW_VERSION_EXPECTED — two deliberately hand-authored host constants
 * whose whole job is to detect board-vs-tree drift.  Re-deriving the comparison
 * in the GUI would give it a second, silently-diverging opinion.
 */

import { HARDWARE_MODELS } from './geometry-config.js';

// ---- Device registry ----
//
// `key` joins to HARDWARE_MODELS (and so to hardware_config.yaml).  A board
// added to the YAML but not listed here will not appear — the join is
// deliberately explicit, because the panel needs a human label and a firmware
// SOURCE per device and neither can be derived from a model string.
//
// `src` says where the firmware version comes from:
//   'kv'     a whole link_status KeyValue, rendered by the bridge node
//   'axis'   one field of the odrive_fw_versions KeyValue, keyed by `axis`
//   null     nothing reports it (see the BB note in the file header)
const DEVICES = [
    {
        key: 'teensy_can_bridge', label: 'can-bridge', short: 'can-bridge',
        src: 'kv', kv: 'bridge_fw_version',
        tip: 'Can-bridge Teensy — self-reported on the BRIDGE_IDENTITY (0x8E) uplink',
    },
    {
        key: 'teensy_platform', label: 'platform', short: 'platform',
        src: 'kv', kv: 'platform_fw_version',
        tip: 'Platform Teensy — self-reported in the 0x6E0 RobotState relay reply',
    },
    { key: 'odrive_axis_0', label: 'Leg 0', short: 'L0', src: 'axis', axis: 0 },
    { key: 'odrive_axis_1', label: 'Leg 1', short: 'L1', src: 'axis', axis: 1 },
    { key: 'odrive_axis_2', label: 'Leg 2', short: 'L2', src: 'axis', axis: 2 },
    { key: 'odrive_axis_3', label: 'Leg 3', short: 'L3', src: 'axis', axis: 3 },
    { key: 'odrive_axis_4', label: 'Leg 4', short: 'L4', src: 'axis', axis: 4 },
    { key: 'odrive_axis_5', label: 'Leg 5', short: 'L5', src: 'axis', axis: 5 },
    { key: 'odrive_axis_6', label: 'Hand', short: 'Hand', src: 'axis', axis: 6 },
    {
        key: 'odrive_axis_7', label: 'BB pitch', short: 'BB pitch', src: null,
        tip: 'Ball Butler pitch ODrive — no firmware version on any wire: the '
           + "can-bridge's Get_Version sweep covers the Jugglebot axes only "
           + '(version_check.h) and the BB heartbeat carries none',
    },
    {
        key: 'odrive_axis_8', label: 'BB hand', short: 'BB hand', src: null,
        tip: 'Ball Butler hand ODrive — no firmware version on any wire: the '
           + "can-bridge's Get_Version sweep covers the Jugglebot axes only "
           + '(version_check.h) and the BB heartbeat carries none',
    },
];

/** link_status is 10 Hz; 3 s of silence matches can-traffic / udp-traffic. */
const STALE_TIMEOUT_MS = 3000;

// Bucket keys for the two NON-version states.  NUL-prefixed so they can never
// collide with a real version string off the wire (the bridge / platform
// buckets key off a whole KeyValue value, which is arbitrary text).
const BUCKET_UNAVAILABLE = '\u0000unavailable';
const BUCKET_UNKNOWN = '\u0000unknown';

// ---- State ----

/** Latest per-device firmware, keyed by DEVICES[].key.  Each entry is the
 *  parsed shape from parseKv/parseAxis, or null when nothing has arrived. */
let latestFw = {};
/** True once any link_status has been decoded — separates "never seen" from
 *  "seen and empty", exactly as udp-traffic's seenLinkStatus does. */
let seenLinkStatus = false;
let staleTimer = null;

// Two independent stale causes, composed in applyStale() so neither stomps the
// other.  Unlike the traffic panels this one does NOT blank its readouts while
// stale: a firmware version is a CONSTANT, so the last-known value stays the
// best answer available and hiding it would be less informative, not more
// honest.  The badge + dimmed version column say "last known, not live".
const staleState = { msgStale: false, rosDown: false };

// ---- Small helpers ----

function el(id) {
    return document.getElementById(id);
}

/** Escape for innerHTML interpolation.  The version strings are wire-sourced
 *  (they reach us through rosbridge), so they never go in unescaped. */
function esc(s) {
    return String(s)
        .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;')
        .replace(/"/g, '&quot;').replace(/'/g, '&#39;');
}

/** Parse DiagnosticStatus KeyValue[] into a plain object (string values). */
function kvMap(msg) {
    const kv = {};
    for (const v of (msg.values || [])) kv[v.key] = v.value;
    return kv;
}

// ---- Wire parsing ----

/**
 * Parse a whole-KeyValue firmware rendering (bridge / platform).
 *
 * The node emits, verbatim:
 *   bridge   '19 (proto 6)' | '15 (SKEW — expected v19, proto 6)' | 'unknown (never seen)'
 *   platform '6'            | '0 (PRE-VERSIONING)'                | 'unknown'
 *
 * Split into the bare number and the node's own qualifier so the number can be
 * shown large and the qualifier as a badge — WITHOUT re-deriving the verdict.
 * `flag` is set from the node's words, never from a comparison of our own:
 * SKEW because the node says the board disagrees with the tree, PRE-VERSIONING
 * because a board reporting 0 predates the identity block and is un-flashed.
 */
function parseKv(raw) {
    if (raw === undefined || raw === null || raw === '') return null;
    const text = String(raw);
    const m = text.match(/^(\d+)\s*(?:\((.*)\))?\s*$/);
    if (!m) {
        // 'unknown (never seen)', 'unknown', or anything a future node adds.
        return { version: null, note: text, noteShort: text, flag: false, raw: text };
    }
    const note = m[2] || '';
    const flag = /SKEW|PRE-VERSIONING/.test(note);
    return {
        version: m[1],
        note,
        // A flagged note is a whole sentence ('SKEW — expected v19, proto 6');
        // the sidebar column is ~10 characters wide, so the badge carries the
        // verdict WORD and the full sentence stays in the tooltip.
        noteShort: flag ? (/PRE-VERSIONING/.test(note) ? 'UNVERSIONED' : 'SKEW') : note,
        flag,
        raw: text,
    };
}

/**
 * Parse the odrive_fw_versions KeyValue into {axis: rawPerAxis}.
 *
 * Format is space-separated 'axis:major.minor.rev-unreleased', with a bare
 * 'axis:?' for an axis whose Get_Version reply has not been cached yet (a
 * partial rig, or the sweep still running at boot — it is bus-paced, one frame
 * per cold-start tick, so a fresh launch legitimately shows '?' for a moment).
 */
function parseAxisVersions(raw) {
    const out = {};
    if (!raw) return out;
    for (const tok of String(raw).trim().split(/\s+/)) {
        const idx = tok.indexOf(':');
        if (idx <= 0) continue;
        const axis = parseInt(tok.slice(0, idx), 10);
        if (!Number.isInteger(axis)) continue;
        out[axis] = tok.slice(idx + 1);
    }
    return out;
}

/**
 * Render one axis's raw 'major.minor.rev-unreleased' for display.
 *
 * The trailing byte is Get_Version's fw_unreleased: 0 on a release build,
 * non-zero on a build made between releases.  It is dropped from the headline
 * (it is noise on every healthy row) but shown as a 'dev' marker when set, and
 * the raw string always survives in the tooltip.  '?' — the byte was never
 * surfaced — is shown as a marker too rather than silently reading as a
 * release build.
 */
function renderAxisVersion(rawPerAxis) {
    if (!rawPerAxis || rawPerAxis === '?') return null;
    const dash = rawPerAxis.lastIndexOf('-');
    if (dash < 0) {
        return { version: rawPerAxis, note: '', noteShort: '', flag: false, raw: rawPerAxis };
    }
    const version = rawPerAxis.slice(0, dash);
    const unrel = rawPerAxis.slice(dash + 1);
    let note = '';
    if (unrel === '?') note = 'build?';
    else if (unrel !== '0') note = 'dev';
    return { version, note, noteShort: note, flag: false, raw: rawPerAxis };
}

// ---- Grouping ----

/**
 * Collapse a run of consecutive leg short-labels ('L0'..'L5') into 'L0-L5'.
 * Purely cosmetic: the seven-Pro row would otherwise be the widest thing in
 * the sidebar.  The full membership always survives in the row's tooltip.
 */
function collapseMembers(shorts) {
    const legNum = (s) => {
        const m = /^L(\d+)$/.exec(s);
        return m ? Number(m[1]) : null;
    };
    const out = [];
    let i = 0;
    while (i < shorts.length) {
        let j = i;
        if (legNum(shorts[i]) !== null) {
            while (j + 1 < shorts.length
                   && legNum(shorts[j + 1]) === legNum(shorts[j]) + 1) {
                j += 1;
            }
        }
        // Collapse runs of THREE or more only: 'L0, L1' is no longer than
        // 'L0–L1' and reads better.
        const run = j - i >= 2;
        out.push(run ? `${shorts[i]}–${shorts[j]}` : shorts[i]);
        i = run ? j + 1 : i + 1;
    }
    return out;
}

/**
 * Build the display rows: one per (model, firmware) bucket, model groups in
 * registry order.
 *
 * MIXED is decided over the KNOWN buckets only.  An axis whose version has not
 * arrived yet does not DISAGREE with anything — treating absence as a mismatch
 * would raise an ODD badge on every launch while the bus-paced sweep runs, and
 * a badge that cries wolf at boot is a badge nobody reads at the one sitting
 * where it matters.  Unknown and unavailable get their own muted buckets.
 *
 * When a model group's known buckets disagree, the strictly-largest one is the
 * consensus and every other is flagged ODD.  On a TIE there is no consensus, so
 * every known bucket is flagged — an even split is not a case where one half is
 * quietly correct.
 */
function buildRows() {
    // 1. Resolve each device to its model + parsed firmware.
    const resolved = DEVICES.map(d => ({
        dev: d,
        model: HARDWARE_MODELS[d.key] || 'unknown model',
        fw: d.src === null ? null : (latestFw[d.key] || null),
        unavailable: d.src === null,
    }));

    // 2. Group by model, preserving registry order.
    const groups = [];
    const byModel = new Map();
    for (const r of resolved) {
        let g = byModel.get(r.model);
        if (!g) {
            g = { model: r.model, buckets: new Map() };
            byModel.set(r.model, g);
            groups.push(g);
        }
        // 3. Bucket by the RAW version string, so two axes that differ only in
        //    the fw_unreleased byte are correctly seen as different firmware.
        let bucketKey;
        if (r.unavailable) {
            bucketKey = BUCKET_UNAVAILABLE;
        } else if (!r.fw || r.fw.version === null) {
            // Sub-keyed by the node's own words: 'unknown (never seen)' (no
            // BRIDGE_IDENTITY frame has EVER arrived) and a bare 'unknown' (the
            // Platform relay read failed) are different facts, and merging them
            // onto one row would invent an agreement that does not exist.
            bucketKey = BUCKET_UNKNOWN + (r.fw && r.fw.note ? '\u0000' + r.fw.note : '');
        } else {
            bucketKey = r.fw.raw;
        }

        let b = g.buckets.get(bucketKey);
        if (!b) {
            b = { key: bucketKey, fw: r.fw, members: [] };
            g.buckets.set(bucketKey, b);
        }
        b.members.push(r.dev);
    }

    // 4. Per group: find the consensus bucket among the known ones.
    const rows = [];
    for (const g of groups) {
        const isNonVersion = b => b.key === BUCKET_UNAVAILABLE
            || b.key.startsWith(BUCKET_UNKNOWN);
        const known = [...g.buckets.values()].filter(b => !isNonVersion(b));
        let consensus = null;
        if (known.length > 1) {
            const sorted = [...known].sort((a, b) => b.members.length - a.members.length);
            // Strictly larger than the runner-up, or there is no consensus.
            if (sorted[0].members.length > sorted[1].members.length) consensus = sorted[0];
        }
        const mixed = known.length > 1;

        for (const b of g.buckets.values()) {
            const shorts = b.members.map(m => m.short);
            const isUnavailable = b.key === BUCKET_UNAVAILABLE;
            const isUnknown = b.key.startsWith(BUCKET_UNKNOWN);
            rows.push({
                model: g.model,
                members: collapseMembers(shorts).join(', '),
                membersFull: b.members.map(m => m.label).join(', '),
                count: b.members.length,
                fw: b.fw,
                unavailable: isUnavailable,
                unknown: isUnknown,
                // Flagged when this bucket is a minority (or there is no
                // majority at all).  Never for the two non-version buckets.
                odd: mixed && !isUnavailable && !isUnknown && b !== consensus,
                mixed,
                // A single-device group carries that device's own tooltip; a
                // multi-device one names its full membership instead.
                tip: b.members.length === 1
                    ? (b.members[0].tip || '')
                    : `${b.members.length} devices: ${b.members.map(m => m.label).join(', ')}`,
            });
        }
    }
    return rows;
}

// ---- Rendering ----

function versionCell(row) {
    if (row.unavailable) {
        return '<span class="hwver-fw hwver-fw-na" '
             + 'title="No firmware version is reported for this device on any wire">n/a</span>';
    }
    if (row.unknown) {
        if (!seenLinkStatus) {
            return '<span class="hwver-fw hwver-fw-unknown" '
                 + 'title="Waiting for the first link_status message">--</span>';
        }
        // The node's OWN words when it gave any.  'unknown (never seen)' (the
        // bridge has never sent a BRIDGE_IDENTITY frame) and a bare 'unknown'
        // (the Platform relay read failed, which also forces a re-home) are
        // different diagnoses, and the generic sweep explanation below is
        // simply untrue for either — it describes the ODrive Get_Version sweep.
        // Shown as 'unknown' with the verbatim text in the tooltip: the column
        // is a sidebar's width, not a sentence's.
        if (row.fw && row.fw.note) {
            return '<span class="hwver-fw hwver-fw-unknown" '
                 + `title="${esc(row.fw.raw)}">unknown</span>`;
        }
        const tip = "This axis's Get_Version reply has not been cached yet. The "
            + "can-bridge's sweep is bus-paced (one frame per cold-start tick), "
            + 'so a fresh launch clears this within a few seconds; a value that '
            + 'persists means the axis is not heartbeating.';
        return `<span class="hwver-fw hwver-fw-unknown" title="${esc(tip)}">`
             + 'not reported</span>';
    }
    const fw = row.fw;
    const noteCls = fw.flag ? 'hwver-note hwver-note-flag' : 'hwver-note';
    const note = fw.noteShort
        ? ` <span class="${noteCls}" title="${esc(fw.raw)}">${esc(fw.noteShort)}</span>`
        : '';
    return `<span class="hwver-fw" title="${esc(fw.raw)}">${esc(fw.version)}</span>${note}`;
}

function rowHtml(row) {
    // Shares .hwver-grid with the static header row in index.html, so the
    // three columns stay aligned (the .can-bus-grid precedent).
    const cls = ['hwver-grid', 'hwver-row'];
    if (row.odd) cls.push('hwver-row-odd');
    if (row.unavailable || row.unknown) cls.push('hwver-row-muted');
    const count = row.count > 1
        ? `<span class="hwver-count">×${row.count}</span>`
        : '';
    const oddBadge = row.odd
        ? '<span class="hwver-odd-badge" title="This device\'s firmware differs '
          + 'from the rest of its model group">ODD</span>'
        : '';
    return `
        <div class="${cls.join(' ')}" title="${esc(row.tip)}">
            <span class="hwver-model">${esc(row.model)}${count}</span>
            <span class="hwver-members" title="${esc(row.membersFull)}">${esc(row.members)}</span>
            <span class="hwver-version">${versionCell(row)}${oddBadge}</span>
        </div>`;
}

function render() {
    const host = el('hardware-rows');
    if (!host) return;
    host.innerHTML = buildRows().map(rowHtml).join('');
    applyStale();
}

// ---- Staleness ----

function applyStale() {
    const badge = el('hardware-stale-badge');
    const host = el('hardware-rows');
    const causes = [];
    if (staleState.rosDown) causes.push('ROS2 disconnected');
    if (staleState.msgStale) causes.push("no 'link_status' from the bridge >3 s");
    const stale = causes.length > 0;
    if (badge) {
        badge.style.display = stale ? '' : 'none';
        badge.title = stale
            ? `Firmware versions are LAST KNOWN, not live — ${causes.join('; ')}. `
              + 'Models are static config and stay valid.'
            : '';
    }
    if (host) host.classList.toggle('hwver-stale', stale);
}

function armStaleTimer() {
    if (staleTimer) clearTimeout(staleTimer);
    staleTimer = setTimeout(() => {
        staleState.msgStale = true;
        applyStale();
    }, STALE_TIMEOUT_MS);
}

// ---- Public API ----

/** Build the panel.  Renders the static MODEL rows immediately, so the panel
 *  is useful before rosbridge connects (the models are config, not telemetry). */
export function initHardwareVersionsPanel() {
    latestFw = {};
    seenLinkStatus = false;
    render();
}

/** Handle a 'link_status' message (10 Hz) — refresh every firmware version. */
export function hardwareVersionsOnLinkStatus(msg) {
    const kv = kvMap(msg);
    seenLinkStatus = true;

    for (const d of DEVICES) {
        if (d.src === 'kv') {
            latestFw[d.key] = parseKv(kv[d.kv]);
        }
    }
    const perAxis = parseAxisVersions(kv.odrive_fw_versions);
    for (const d of DEVICES) {
        if (d.src === 'axis') {
            latestFw[d.key] = renderAxisVersion(perAxis[d.axis]);
        }
    }

    staleState.msgStale = false;
    armStaleTimer();
    render();
}

/** rosbridge websocket up/down.  Driven for EVERY state from main.js's
 *  connection router, not just the down edge — same contract as the traffic
 *  panels, so a reconnect always clears the badge. */
export function setHardwareVersionsRosLink(isUp) {
    staleState.rosDown = !isUp;
    if (!isUp) {
        // The 3 s watchdog would fire on its own, but not for 3 s — and a
        // dropped websocket is known-stale immediately.
        if (staleTimer) clearTimeout(staleTimer);
        staleState.msgStale = true;
    }
    applyStale();
}
