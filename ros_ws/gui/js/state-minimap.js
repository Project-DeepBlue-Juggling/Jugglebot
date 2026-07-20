/**
 * state-minimap.js — Orchestrator state-machine minimap.
 *
 * A clickable map of the orchestrator state machine (BOOT / HOMING / IDLE /
 * LEVELLING / ACTIVE:{STANDBY,TRAJECTORY,SPACEMOUSE,GUI} / FAULT)
 * that greys out unreachable states with a reason, and — on a hold-to-confirm
 * — runs the SAFE multi-step sequence to reach the target state, honouring
 * the bench-validated ordering rules (tests/hardware/mvp_bench_runbook.md
 * "Sharp edges", esp. #1 disarm-before-leaving-a-streaming-mode and
 * #6 disarm-before-deactivate).
 *
 * Ground-truth anchors (re-checked 2026-07-16, max-deviation attribution commit):
 *   - states / commands:  ros_ws/src/jugglebot/jugglebot/state_machine.py
 *     (commands consumed: IDLE :309-319, ACTIVE :451-457, FAULT :569-575;
 *      commands DISCARDED every tick in BOOT :238 / HOMING :277 / LEVELLING :356)
 *   - forced FAULT:       orchestrator_node.py:143-180 (any robot_state error;
 *     guard latch forces FAULT only from ACTIVE)
 *   - /link_status keys:  teensy_bridge_node.py _publish_link_status
 *     (consumed: fault_state, mpc_active, bridge_link, guard_fault_leg)
 *   - arming preconditions: _arm_setpoint_output (teensy_bridge_node.py:1654)
 *     + _u0_within_encoder_tol (:1786); stream-then-arm
 *   - armed /clear_errors reroutes converge-first: _svc_clear_errors
 *     (teensy_bridge_node.py:3316-3368)
 *   - streaming set:      trajectory_node.py:140-141 (all six submodes stream)
 *
 * Everything actuating goes through the table-driven sequencer in S4/S5 —
 * plans are reviewable DATA, computed at confirm time (never at pointerdown),
 * with per-step preconditions and a hard verify-gate before any deactivate.
 */

import * as ros from './ros-bridge.js';
import { holdToConfirm } from './hold-to-confirm.js';
import { emitEvent, EVENT_TYPES } from './event-store.js';

// ====================================================================
// S1. Constants
// ====================================================================

const HOLD_MS = 800;
const LS_EXPANDED_KEY = 'jugglebot-minimap-expanded';

/** /link_status is 10 Hz; 2 s of silence means teensy_bridge_node is gone. */
const LINK_STALE_MS = 2000;
/** /orchestrator_state is 10 Hz; 2 s of silence means orchestrator_node is gone. */
const ORCH_STALE_MS = 2000;
/** /robot_state is ~20 Hz (same producer node as link_status); 2 s ⇒ stale. */
const ROBOT_STALE_MS = 2000;
/** pump/staleness re-evaluation tick — timeouts fire even if topics stop. */
const TICK_MS = 250;

// Sequencer timeouts (minimap plan §7.2).
// T_MODE / T_DEACT are sized for QUEUED consumption, not next-tick:
// ActiveHandler does not consume commands until the multi-second ACTIVATE
// move completes (state_machine.py:441-449 gates consume_command on
// self._activated), so a publish issued while a previous move is finishing
// sits in the server-side queue (depth 4) and fires seconds late.
// Residual (accepted): if the wait STILL times out, the queued command can
// fire AFTER the operator saw ABORTED — confusing but safe: in the teardown
// the disarm + verify-gate precede the deactivate publish, and a zombie
// submode change is streaming→streaming.
const T_MODE = 15000;           // submode change; may queue behind a finishing move
const T_IDLE_CMD = 15000;       // IDLE may sit on invisible operation_pending
const T_ACTIVATE = 20000;       // ACTIVATE is a multi-second TRAP_TRAJ move
const T_DEACT = 15000;          // instant ONLY once activation completed; may queue
const T_DISARM_VERIFY = 3000;   // next /link_status tick shows mpc_active=0
const T_FAULT_TO_IDLE = 60000;  // BOOT revalidation (+ ~9 s HOMING if unhomed)
const T_AFTER_TEARDOWN = 20000; // queued command fires when the stow completes
const T_RETRY_BOOT = 35000;     // boot re-validation after orchestrator clear

// Service client timeouts (Promise.race)
const T_SVC_SETPOINT = 10000;
const T_SVC_CLEAR = 45000;      // armed reroute blocks through reseed+converge
const T_SVC_GO_HOME = 5000;

// Teardown echo-quiescence ("go_home profile finished" proxy)
const QUIESCE_TOL_REV = 0.005;
const QUIESCE_WINDOW_MS = 500;
const QUIESCE_MIN_SPAN_MS = 400;
const QUIESCE_MIN_SAMPLES = 5;
const QUIESCE_FRESH_MS = 300;
const QUIESCE_TIMEOUT_MS = 8000;
const ECHO_KEEP_MS = 1500;

const TRIGGER = 'std_srvs/srv/Trigger';
const SETBOOL = 'std_srvs/srv/SetBool';

const SUBMODES = ['STANDBY', 'TRAJECTORY', 'SPACEMOUSE', 'GUI'];
const MAIN_STATES = ['BOOT', 'HOMING', 'IDLE', 'LEVELLING', 'ACTIVE', 'FAULT'];
const ALL_NODE_IDS = MAIN_STATES.concat(SUBMODES.map((m) => 'ACTIVE:' + m));

/**
 * Hand-laid SVG geometry, one table per display mode.  The submode chips are
 * built in BOTH modes (so the DOM is stable for probes); CSS hides them while
 * collapsed and the cluster shows the current submode as text instead.
 */
const LAYOUTS = {
    collapsed: {
        viewBox: '0 0 200 212',
        nodeFont: 9, chipFont: 7.5, captionFont: 6.5, edgeLabelFont: 6.5,
        nodes: {
            BOOT:      { x: 12,  y: 8,   w: 70,  h: 20 },
            FAULT:     { x: 120, y: 8,   w: 68,  h: 20, halo: { x: 114, y: 3, w: 80, h: 30 } },
            HOMING:    { x: 12,  y: 52,  w: 70,  h: 20 },
            IDLE:      { x: 12,  y: 96,  w: 70,  h: 20 },
            LEVELLING: { x: 114, y: 96,  w: 74,  h: 20 },
            ACTIVE:    { x: 12,  y: 140, w: 176, h: 64 },
        },
        chips: { x0: 20, y0: 178, w: 80, h: 11, dx: 84, dy: 13 },  // display:none while collapsed
        clusterLabel: { x: 22, y: 156 },
        clusterSub:   { x: 22, y: 172 },
        faultCaption: { x: 154, y: 41 },
        faultLive:    { x: 154, y: 49 },
        edges: [
            { x1: 47, y1: 28,  x2: 47,  y2: 52,  arrow: true },                       // BOOT→HOMING
            { x1: 47, y1: 72,  x2: 47,  y2: 96,  arrow: true, arrowStart: true },     // HOMING→IDLE (auto) ⇄ IDLE→HOMING (commanded)
            { x1: 82, y1: 106, x2: 114, y2: 106, arrow: true, arrowStart: true },     // IDLE⇄LEVELLING
            { x1: 47, y1: 116, x2: 47,  y2: 140, arrow: true, arrowStart: true },     // IDLE→ACTIVE ⇄ ACTIVE→IDLE (teardown)
            { path: 'M 12,18 L 5,18 L 5,106 L 12,106', arrow: true, dashed: true },   // BOOT→IDLE (auto skip-if-homed, state_machine.py:257-258)
            { x1: 114, y1: 18, x2: 84,  y2: 18,  arrow: true, dashed: true },         // FAULT→BOOT (auto)
            { path: 'M 194,23 L 197,23 L 197,163 L 190,163', arrow: true, dashed: true }, // FAULT→ACTIVE (guard-only)
        ],
    },
    expanded: {
        viewBox: '0 0 266 432',
        nodeFont: 11, chipFont: 9.5, captionFont: 8, edgeLabelFont: 9,
        nodes: {
            BOOT:      { x: 20,  y: 16,  w: 100, h: 28 },
            FAULT:     { x: 156, y: 16,  w: 84,  h: 28, halo: { x: 150, y: 10, w: 96, h: 60 } },
            HOMING:    { x: 20,  y: 88,  w: 100, h: 28 },
            IDLE:      { x: 20,  y: 160, w: 100, h: 28 },
            LEVELLING: { x: 150, y: 160, w: 96,  h: 28 },
            ACTIVE:    { x: 14,  y: 232, w: 232, h: 150 },
        },
        chips: { x0: 24, y0: 262, w: 104, h: 30, dx: 110, dy: 38 },
        clusterLabel: { x: 26, y: 252 },
        clusterSub: null,
        faultCaption: { x: 198, y: 55 },
        faultLive:    { x: 198, y: 65 },
        edges: [
            { x1: 70,  y1: 44,  x2: 70,  y2: 88,  arrow: true },                      // BOOT→HOMING
            { x1: 70,  y1: 116, x2: 70,  y2: 160, arrow: true, arrowStart: true },    // HOMING→IDLE (auto) ⇄ IDLE→HOMING (commanded)
            { x1: 120, y1: 174, x2: 150, y2: 174, arrow: true, arrowStart: true },    // IDLE⇄LEVELLING
            { x1: 70,  y1: 188, x2: 70,  y2: 232, arrow: true, arrowStart: true },    // IDLE→ACTIVE ⇄ ACTIVE→IDLE (teardown)
            { path: 'M 20,30 L 10,30 L 10,174 L 20,174', arrow: true, dashed: true,   // BOOT→IDLE (auto skip-if-homed, state_machine.py:257-258)
              label: 'auto if homed', lx: 5, ly: 102, rotate: 90 },
            { x1: 150, y1: 30,  x2: 124, y2: 30,  arrow: true, dashed: true,
              label: 'auto when clear', lx: 133, ly: 12 },
            { path: 'M 246,40 L 256,40 L 256,300 L 246,300', arrow: true, dashed: true,
              label: 'guard-only resume', lx: 261, ly: 170, rotate: 90 },
        ],
    },
};

// ====================================================================
// S2. Snapshot store
// ====================================================================

/**
 * Everything the reachability table, plan table and sequencer read.  Updated
 * ONLY by the four minimapOn* entry points, the leg-echo feed, and the
 * connection listener; consumed by pure functions below.
 */
const snap = {
    conn: 'disconnected',
    orch: { main: null, sub: '', raw: null, lastMs: 0 },
    controlMode: null,
    robot: {
        lastMs: 0,
        is_homed: false,
        firmware_validated: false,
        error: [],
        has_fatal_odrive_error: false,
        has_fatal_can_error: false,
        has_undervoltage: false,
        heartbeatsOk: false,
    },
    link: {
        lastMs: 0,
        faultState: 'UNKNOWN',
        mpcActive: null,      // 1 / 0 / null (never seen OR unparseable = UNVERIFIED)
        bridgeLink: 'UNKNOWN',
        // Culprit leg for an ACTIVE MAX_DEVIATION latch ('' when no such latch).
        // Bridge gates this on fault_state==MAX_DEVIATION (teensy_bridge_node.py
        // guard_fault_leg KeyValue), so it is safe to read as a live attribution.
        guardFaultLeg: '',
    },
};

/** Recent leg_setpoint_echo samples for the teardown quiescence wait. */
const echoSamples = [];  // [{t: ms, v: number[6]}]

function isLinkFresh(s) { return Date.now() - s.link.lastMs < LINK_STALE_MS; }
function isOrchFresh(s) { return Date.now() - s.orch.lastMs < ORCH_STALE_MS; }
function isRobotFresh(s) { return Date.now() - s.robot.lastMs < ROBOT_STALE_MS; }
function guardLatched(s) {
    return s.link.faultState !== 'NONE' && s.link.faultState !== 'UNKNOWN';
}
function robotErrorsClear(s) {
    return s.robot.error.length === 0 &&
        !s.robot.has_fatal_odrive_error && !s.robot.has_fatal_can_error;
}

export function minimapOnOrchestratorState(str) {
    const parts = String(str).split(':');
    snap.orch.raw = String(str);
    snap.orch.main = (parts[0] || '').toUpperCase();
    snap.orch.sub = (parts[1] || '').toUpperCase();
    snap.orch.lastMs = Date.now();
    pump();
    scheduleRender();
}

export function minimapOnControlMode(str) {
    snap.controlMode = String(str || '').toUpperCase();
    pump();
    scheduleRender();
}

export function minimapOnRobotState(msg) {
    const r = snap.robot;
    r.lastMs = Date.now();
    r.is_homed = !!msg.is_homed;
    r.firmware_validated = !!msg.firmware_validated;
    r.error = msg.error || [];
    r.has_fatal_odrive_error = !!msg.has_fatal_odrive_error;
    r.has_fatal_can_error = !!msg.has_fatal_can_error;
    r.has_undervoltage = !!msg.has_undervoltage;
    // ctx.all_heartbeats replica: all 7 Jugglebot axes (legs 0-5 + hand 6)
    // report current_state != 0 (orchestrator_node.py:106-110).
    const motors = msg.motor_states || [];
    let hb = motors.length >= 7;
    for (let i = 0; hb && i < 7; i++) {
        if (!motors[i] || motors[i].current_state === 0) hb = false;
    }
    r.heartbeatsOk = hb;
    pump();
    scheduleRender();
}

export function minimapOnLinkStatus(msg) {
    const kv = {};
    for (const v of (msg.values || [])) kv[v.key] = v.value;
    snap.link.lastMs = Date.now();
    snap.link.faultState = kv.fault_state || 'UNKNOWN';
    // Tri-state parse — anything other than an exact 0/1 is null (UNVERIFIED),
    // never 0: the teardown's disarm verify-gate requires an exact fresh 0
    // (Sharp Edge #6), so a value-format drift must fail the gate (abort
    // before deactivate) rather than silently reading as disarmed.
    const n = parseInt(kv.mpc_active, 10);
    snap.link.mpcActive = n === 1 ? 1 : (n === 0 ? 0 : null);
    snap.link.bridgeLink = kv.bridge_link || 'UNKNOWN';
    snap.link.guardFaultLeg = kv.guard_fault_leg || '';
    pump();
    scheduleRender();
}

/** Feeds ONLY the teardown quiescence predicate (accepted-u0 echo, ~20 Hz). */
export function minimapOnLegSetpointEcho(msg) {
    const now = Date.now();
    echoSamples.push({ t: now, v: Array.prototype.slice.call(msg.data || []) });
    while (echoSamples.length && now - echoSamples[0].t > ECHO_KEEP_MS) {
        echoSamples.shift();
    }
    pump();
}

/**
 * All 6 leg setpoints stable within QUIESCE_TOL_REV over the last
 * QUIESCE_WINDOW_MS, counting ONLY samples observed at/after the anchor
 * `t0` — the "go_home profile finished" proxy.
 *
 * The anchor is load-bearing: while armed at a settled pose the 40 Hz hold
 * stream fills the trailing window with perfectly flat PRE-go_home samples,
 * so an unanchored trailing-window check false-passes the instant the wait
 * starts (before the descent has moved a single sample).  Requiring
 * >= QUIESCE_MIN_SPAN_MS of samples all NEWER than the wait's own start
 * means a real ~2 s descent holds the wait open (each window shows >> tol
 * of motion until it lands) while an already-neutral pose still passes
 * after ~0.5 s of NEW flat samples.  A stalled echo stream (emitter
 * stopped) reads false; the wait step's 8 s timeout then continues — the
 * disarm that follows is the actual safety step.
 */
function echoQuiescentSince(t0) {
    const now = Date.now();
    if (!echoSamples.length) return false;
    if (now - echoSamples[echoSamples.length - 1].t > QUIESCE_FRESH_MS) return false;
    const win = echoSamples.filter((sm) => sm.t >= t0 && now - sm.t <= QUIESCE_WINDOW_MS);
    if (win.length < QUIESCE_MIN_SAMPLES) return false;
    if (now - win[0].t < QUIESCE_MIN_SPAN_MS) return false;
    for (let leg = 0; leg < 6; leg++) {
        let lo = Infinity, hi = -Infinity;
        for (const sm of win) {
            const v = sm.v[leg];
            if (typeof v !== 'number' || Number.isNaN(v)) return false;
            if (v < lo) lo = v;
            if (v > hi) hi = v;
        }
        if (hi - lo > QUIESCE_TOL_REV) return false;
    }
    return true;
}

function onConnChange(state) {
    snap.conn = state;
    if (state !== 'connected') {
        // Drop everything the topics told us: after a reconnect the 10 Hz
        // republish restores it within ~100 ms, and stale reads must never
        // drive reachability or the sequencer's wait predicates.
        snap.orch.main = null; snap.orch.sub = ''; snap.orch.raw = null;
        snap.orch.lastMs = 0;
        snap.controlMode = null;
        snap.link.lastMs = 0; snap.link.faultState = 'UNKNOWN';
        snap.link.mpcActive = null; snap.link.bridgeLink = 'UNKNOWN';
        snap.link.guardFaultLeg = '';
        // Robot flags must not survive a disconnect as trusted values: a
        // reconnect can deliver orchestrator_state before the throttled
        // robot_state, and G_ERRORS/G_HB/G_FW/G_HOMED would otherwise
        // evaluate against the previous session.  Clear AND stamp-invalidate.
        snap.robot.lastMs = 0;
        snap.robot.is_homed = false;
        snap.robot.firmware_validated = false;
        snap.robot.error = [];
        snap.robot.has_fatal_odrive_error = false;
        snap.robot.has_fatal_can_error = false;
        snap.robot.has_undervoltage = false;
        snap.robot.heartbeatsOk = false;
        echoSamples.length = 0;
    }
    pump();  // aborts any in-flight plan ('rosbridge disconnected')
    scheduleRender();
}

// ====================================================================
// S3. Reachability — pure fn snap -> {nodeId: {ok, reason, ...}}
// ====================================================================

/**
 * Guards evaluated in order; the first failure is the greyed-out reason.
 * G_GUARD is deliberately stricter than the orchestrator's BOOT tolerance:
 * for ACTUATION we need a live, provably-clear guard, so UNKNOWN fails too.
 */
const GUARDS = {
    G_CONN: (s) => (s.conn === 'connected' ? null : 'rosbridge disconnected'),
    G_LINKFRESH: (s) => (isLinkFresh(s) ? null
        : 'link_status stale — teensy_bridge_node down?'),
    // The bridge keeps publishing a FRESH link_status after the Teensy uplink
    // dies, with fault_state frozen at the last heartbeat's value — a fresh
    // message is NOT a live guard reading.  bridge_link is the bridge's own
    // liveness verdict (UP / LOST / NO_HEARTBEAT).
    G_BRIDGE: (s) => (s.link.bridgeLink === 'UP' ? null
        : 'Teensy uplink lost (bridge_link=' + s.link.bridgeLink +
          ') — guard/arm state unverifiable'),
    G_GUARD: (s) => {
        if (s.link.faultState === 'NONE') return null;
        if (s.link.faultState === 'UNKNOWN') return 'Teensy link down — guard state unknown';
        return 'Teensy guard latched (' + s.link.faultState + ') — Recover first';
    },
    G_ROBOTFRESH: (s) => (isRobotFresh(s) ? null
        : 'robot_state stale — teensy_bridge_node down?'),
    G_ERRORS: (s) => {
        if (robotErrorsClear(s)) return null;
        const first = s.robot.error[0] ||
            (s.robot.has_fatal_odrive_error ? 'fatal ODrive error' : 'fatal CAN error');
        return 'active errors: ' + first;
    },
    G_HB: (s) => (s.robot.heartbeatsOk ? null : 'motor heartbeats missing'),
    G_FW: (s) => (s.robot.firmware_validated ? null : 'firmware not validated'),
    G_HOMED: (s) => (s.robot.is_homed ? null : 'not homed — run HOMING first'),
};
const GUARDS_BASE = ['G_CONN', 'G_LINKFRESH', 'G_BRIDGE', 'G_GUARD',
    'G_ROBOTFRESH', 'G_ERRORS', 'G_HB', 'G_FW'];
const GUARDS_FULL = GUARDS_BASE.concat(['G_HOMED']);
// FAULT-recovery gate: freshness/uplink only (G_GUARD is latched in F-A by
// definition; errors exist in F-B) — no recovery clicks on frozen link data.
const GUARDS_FRESH = ['G_CONN', 'G_LINKFRESH', 'G_BRIDGE', 'G_ROBOTFRESH'];

function gateFail(s, names) {
    for (const n of names) {
        const r = GUARDS[n](s);
        if (r) return { ok: false, reason: r };
    }
    return null;
}
function okNode(extra) { return Object.assign({ ok: true }, extra || {}); }
function curNode() { return { ok: false, reason: 'current state', isCurrent: true }; }

/** Full per-node reachability with greyed reasons (minimap plan §6). */
function reachability(s) {
    const out = {};
    const setAll = (reason) => {
        for (const id of ALL_NODE_IDS) out[id] = { ok: false, reason };
    };
    const main = s.orch.main;

    if (s.conn !== 'connected') {
        setAll('rosbridge disconnected');
    } else if (main === null) {
        setAll('waiting for /orchestrator_state…');
    } else if (!isOrchFresh(s)) {
        // A frozen orchestrator_state must never drive reachability: with
        // orchestrator_node dead, published commands queue blind (or vanish)
        // and every wait would time out with a misleading reason.
        setAll('orchestrator_state stale — orchestrator down?');
    } else if (main === 'BOOT' || main === 'HOMING' || main === 'LEVELLING') {
        // Commands are DISCARDED every tick in these states
        // (state_machine.py:238, 277, 356) — stricter than the strip here.
        setAll(main + ' in progress — orchestrator discards all commands until it completes');
    } else if (main === 'IDLE') {
        setAll('unreachable from IDLE');
        out.HOMING = gateFail(s, GUARDS_BASE) ||
            okNode({ confirmTip: 'hold to confirm — re-home the legs (auto-returns to IDLE)' });
        out.LEVELLING = gateFail(s, GUARDS_FULL) || okNode({
            confirmTip: 'hold to confirm — LEVELLING raises the platform to full ACTIVE height',
        });
        const actFail = gateFail(s, GUARDS_FULL);
        out.ACTIVE = actFail || okNode({
            confirmTip: 'hold to confirm — activate (multi-second raise, lands in ACTIVE:STANDBY)',
        });
        for (const m of SUBMODES) {
            out['ACTIVE:' + m] = actFail || okNode({
                // STANDBY is automatic on ACTIVE entry (state_machine.py:419) —
                // the plan for it is activateSteps() alone, no switch published.
                confirmTip: m === 'STANDBY'
                    ? 'hold to confirm — activate (multi-second raise, lands in ACTIVE:STANDBY)'
                    : 'hold to confirm — activate, then switch to ' + m,
            });
        }
        out.IDLE = curNode();
    } else if (main === 'ACTIVE') {
        setAll('unreachable from ACTIVE');
        for (const m of SUBMODES) {
            out['ACTIVE:' + m] = (m === s.orch.sub) ? curNode()
                : (gateFail(s, GUARDS_FULL) || okNode({
                    confirmTip: 'hold to confirm — switch to ' + m +
                        ' (streaming→streaming, safe armed; trajectory_node.py:140-141)',
                }));
        }
        out.ACTIVE = curNode();
        // Teardown is the escape hatch: gated on the connection ONLY.  It is
        // deliberately available with a stale/latched link — step 2 assumes
        // worst-case armed and the disarm verify-gate ABORTS before any
        // deactivate if mpc_active cannot be confirmed 0 (Sharp Edge #6).
        out.IDLE = gateFail(s, ['G_CONN']) || okNode({
            confirmTip: 'hold to confirm — SAFE TEARDOWN: standby → go_home → ' +
                'disarm+verify → deactivate (runbook Sharp Edges #1/#6)',
        });
        out.HOMING = gateFail(s, GUARDS_BASE) ||
            okNode({ confirmTip: 'hold to confirm — safe teardown to IDLE, then home' });
        out.LEVELLING = gateFail(s, GUARDS_FULL) || okNode({
            confirmTip: 'hold to confirm — safe teardown to IDLE, then LEVELLING (raises the platform)',
        });
    } else if (main === 'FAULT') {
        // Classify the FAULT flavour client-side (mirror of
        // state_machine.py:503-508; boot_timed_out is unobservable — the
        // opaque flavour is handled conservatively via the button only).
        if (s.robot.has_undervoltage) {
            setAll('undervoltage active — check E-stop / DC power (auto-clears when power returns)');
        } else if (!robotErrorsClear(s)) {
            setAll('in FAULT — clear errors first');
            // F-B gate: freshness + uplink too (GUARDS_FRESH) — recovery must
            // not be clickable on frozen link/robot data.
            out.IDLE = gateFail(s, GUARDS_FRESH) || okNode({
                confirmTip: 'hold to confirm — clear errors, then ride the automatic ' +
                    'BOOT→(HOMING→)IDLE chain to IDLE',
            });
        } else if (guardLatched(s)) {
            // Guard-only fault preserves control_mode — the preserved
            // streaming submode is the ONE resume target (auto-resume edge,
            // no re-arm move; state_machine.py:497-558).
            const mode = SUBMODES.indexOf(s.controlMode) >= 0 ? s.controlMode : null;
            setAll('in FAULT — Recover to resume ' + (mode || 'the previous mode') + ' first');
            if (mode) {
                // F-A gate: freshness + uplink (GUARDS_FRESH) — same rationale
                // as F-B above; the flavour classification itself reads link
                // data that must be live.
                const resume = gateFail(s, GUARDS_FRESH) || okNode({
                    resume: true,
                    confirmTip: 'hold to confirm — Recover (converge-first clear) and ' +
                        'auto-resume ' + mode + ' (no re-arm move)',
                });
                out['ACTIVE:' + mode] = resume;
                out.ACTIVE = resume;  // collapsed view: the cluster IS the resume target
            }
        } else {
            setAll("in FAULT with no visible errors (boot timeout?) — use 'Retry Boot' below");
        }
    } else {
        // Fail-safe for orchestrator evolution: never guess.
        setAll("unknown state '" + main + "' — minimap table out of date");
    }

    // Universal rules always win (minimap plan §7.3 header).
    out.BOOT = (main === 'BOOT') ? curNode() : {
        ok: false,
        reason: 'automatic state — entered on startup and fault recovery, cannot be commanded',
    };
    out.FAULT = (main === 'FAULT') ? curNode() : {
        ok: false,
        reason: 'cannot be commanded — latches on error, auto-exits when conditions clear',
    };
    return out;
}

// ====================================================================
// S4. Plan table (SAFETY-CRITICAL — reviewable data + tiny composers)
// ====================================================================

// Step constructors.  `pre` on publish steps is re-checked immediately
// before publishing (abort on mismatch: closes the FAULT-auto-exit-during-
// hold race and the concurrent-strip-click race).
function pub(data, pre, preDesc, desc) {
    return { kind: 'publish', data, pre, preDesc, desc: desc || "publish '" + data + "'" };
}
function svcStep(name, type, request, clientTimeoutMs, onReject, desc, surfaceMessage) {
    return { kind: 'service', name, type, request, clientTimeoutMs, onReject,
        desc: desc || ('service ' + name), surfaceMessage: !!surfaceMessage };
}
function waitStep(pred, timeoutMs, onTimeout, desc) {
    return { kind: 'wait', pred, timeoutMs, onTimeout, desc };
}
function mkPlan(label, steps, recovery) {
    return { label, steps, recovery: !!recovery };
}

/** IDLE → ACTIVE:STANDBY (activation lands in STANDBY automatically). */
function activateSteps() {
    return [
        pub('activate', (s) => s.orch.main === 'IDLE', 'orchestrator still IDLE',
            "publish 'activate' — folds configure, lands ACTIVE:STANDBY (runbook Sharp Edge #4)"),
        waitStep((s) => s.orch.main === 'ACTIVE' && s.orch.sub === 'STANDBY',
            T_IDLE_CMD, 'abort',
            'orchestrator enters ACTIVE:STANDBY (IDLE may first wait out an invisible deactivation)'),
        waitStep((s) => s.controlMode === 'STANDBY', T_ACTIVATE, 'abort',
            'control_mode confirms STANDBY (set only after the multi-second ACTIVATE move completes)'),
    ];
}

/** ACTIVE submode change — streaming→streaming, safe armed or not. */
function modeSteps(X) {
    return [
        pub(X.toLowerCase(), (s) => s.orch.main === 'ACTIVE', 'orchestrator in ACTIVE',
            "publish '" + X.toLowerCase() + "' (streaming→streaming — Sharp Edge #1 does not apply)"),
        waitStep((s) => s.orch.main === 'ACTIVE' && s.orch.sub === X, T_MODE, 'abort',
            'orchestrator shows ACTIVE:' + X +
            ' — waiting (a previous move may be finishing)'),
        waitStep((s) => s.controlMode === X, T_MODE, 'abort',
            'control_mode confirms ' + X + ' (runbook Sharp Edge #5 — verify before arming)' +
            ' — waiting (a previous move may be finishing)'),
    ];
}

/**
 * THE TEARDOWN PLAN — ACTIVE:X → IDLE.  SAFETY-CRITICAL ORDER.
 *
 * Canonical safe order (tests/hardware/mvp_bench_runbook.md:202-203, and the
 * S1/S2 session teardowns): go_home → disarm → orchestrator deactivate.
 *   Sharp Edge #1 (runbook:79-83): leaving a streaming mode while armed
 *     latches MPC_STALE within 250 ms — only streaming→streaming changes
 *     (step 1's 'standby') are permitted while possibly armed.
 *   Sharp Edge #6 (runbook:123-133): DEACTIVATE while mpc_active=1 latches
 *     MPC_STALE AND leaves the legs un-stowed (firmware rejects it) — so the
 *     disarm is unconditional and step 4 is HARD-GATED on a fresh
 *     /link_status tick proving mpc_active=0.  NEVER reach step 4 unverified.
 */
function teardownSteps(s) {
    const steps = [];
    // Step 1 — iff not already STANDBY: silence pose-mode target sources so
    // go_home is not fought.  streaming→streaming, safe while armed.
    if (s.orch.sub !== 'STANDBY') {
        steps.push(pub('standby', (s2) => s2.orch.main === 'ACTIVE', 'orchestrator in ACTIVE',
            "publish 'standby' (streaming→streaming, safe armed)"));
        steps.push(waitStep(
            (s2) => s2.orch.main === 'ACTIVE' && s2.orch.sub === 'STANDBY' &&
                s2.controlMode === 'STANDBY',
            T_MODE, 'abort', 'STANDBY confirmed on orchestrator_state + control_mode'));
    }
    // Step 2 — iff armed OR the link is stale (assume worst): profiled
    // neutral return.  Best-effort: the comfort step, not the safety step —
    // on unavailable / reject / timeout we log and continue to the disarm.
    if (!isLinkFresh(s) || s.link.mpcActive !== 0) {
        steps.push(svcStep('trajectory/go_home', TRIGGER, {}, T_SVC_GO_HOME, 'continue',
            'trajectory/go_home — profiled neutral return (best-effort)'));
        // Anchor the quiescence window to THIS wait's own activation: the
        // first pump() evaluation stamps t0 and returns false, and
        // echoQuiescentSince then counts only samples newer than t0.
        // Without the anchor, the flat pre-go_home hold stream false-passes
        // the wait instantly and the disarm lands mid-descent (unprofiled
        // freeze of moving legs) — see echoQuiescentSince's doc comment.
        let tQuiesce0 = null;
        steps.push(waitStep(() => {
            if (tQuiesce0 === null) { tQuiesce0 = Date.now(); return false; }
            return echoQuiescentSince(tQuiesce0);
        }, QUIESCE_TIMEOUT_MS, 'continue',
            'leg setpoints quiescent ≤' + QUIESCE_TOL_REV + ' rev over ' +
            (QUIESCE_WINDOW_MS / 1000) + ' s of NEW samples (go_home profile finished)'));
    }
    // Step 3 — ALWAYS disarm, then VERIFY.  Disarm is idempotent and always
    // succeeds (teensy_bridge_node.py:1555).  The verify-gate is the whole
    // point: on failure or timeout we ABORT and never emit step 4.
    steps.push(svcStep('set_setpoint_output', SETBOOL, { data: false },
        T_SVC_SETPOINT, 'abort', 'disarm setpoint output (set_setpoint_output false)'));
    steps.push(waitStep((s2) => isLinkFresh(s2) && s2.link.mpcActive === 0,
        T_DISARM_VERIFY, 'abort',
        'VERIFY mpc_active=0 on a fresh /link_status tick — HARD GATE (Sharp Edge #6)'));
    // Step 4 — only now is DEACTIVATE safe: firmware accepts it and the legs
    // profile-stow.
    steps.push(pub('deactivate', (s2) => s2.orch.main === 'ACTIVE', 'orchestrator in ACTIVE',
        "publish 'deactivate' — profiled stow to IDLE"));
    steps.push(waitStep((s2) => s2.orch.main === 'IDLE', T_DEACT, 'abort',
        'orchestrator reaches IDLE (the stow continues asynchronously)' +
        ' — waiting (a previous move may be finishing)'));
    return steps;
}

/** FAULT (guard-only flavour) → resume the preserved streaming mode. */
function resumePlan(mode) {
    // Recovery entry point decision (minimap plan §0, last row): ALWAYS the
    // bridge /clear_errors Trigger — while armed it self-routes through the
    // converge-first /recover sequence using its authoritative in-process
    // _mpc_active (_svc_clear_errors, teensy_bridge_node.py:3316-3368), immune to a stale GUI
    // link_status read.  Never a GUI-side mpc_active branch to /recover.
    return mkPlan('FAULT → ACTIVE:' + mode + ' (recover + auto-resume)', [
        svcStep('clear_errors', TRIGGER, {}, T_SVC_CLEAR, 'abort',
            'bridge /clear_errors (self-routes converge-first while armed)', true),
        waitStep((s) => s.link.faultState === 'NONE', 5000, 'abort',
            'guard latch cleared on /link_status'),
        waitStep((s) => s.orch.main === 'ACTIVE', 10000, 'abort',
            'orchestrator auto-resumes ACTIVE (no re-arm move)'),
    ], true);
}

/** FAULT (real-fault flavour) → IDLE via the automatic exit chain. */
function faultToIdlePlan() {
    return mkPlan('FAULT → IDLE (clear errors + auto chain)', [
        svcStep('clear_errors', TRIGGER, {}, T_SVC_CLEAR, 'abort',
            'bridge /clear_errors Trigger', true),
        waitStep((s) => robotErrorsClear(s) &&
            (s.link.faultState === 'NONE' || s.link.faultState === 'UNKNOWN'),
            10000, 'abort', 'robot errors + guard latch clear'),
        waitStep((s) => s.orch.main !== 'FAULT', 10000, 'abort',
            'orchestrator leaves FAULT'),
        waitStep((s) => s.orch.main === 'IDLE', T_FAULT_TO_IDLE, 'abort',
            'automatic BOOT→(HOMING→)IDLE chain lands in IDLE'),
    ], true);
}

/**
 * planFor(snap, targetNodeId) — the full transition table (§7.3), computed
 * at CONFIRM time from a fresh snapshot.  Returns a plan or {refuse}.
 */
function planFor(s, targetId) {
    const main = s.orch.main;
    if (targetId === 'BOOT') return { refuse: 'automatic state — cannot be commanded' };
    if (targetId === 'FAULT') return { refuse: 'cannot command a fault' };

    if (main === 'IDLE') {
        if (targetId === 'HOMING') {
            return mkPlan('IDLE → HOMING', [
                pub('home', (s2) => s2.orch.main === 'IDLE', 'orchestrator still IDLE'),
                waitStep((s2) => s2.orch.main === 'HOMING', T_IDLE_CMD, 'abort',
                    'orchestrator enters HOMING (auto-returns to IDLE when done)'),
            ]);
        }
        if (targetId === 'LEVELLING') {
            return mkPlan('IDLE → LEVELLING', [
                pub('level', (s2) => s2.orch.main === 'IDLE', 'orchestrator still IDLE'),
                waitStep((s2) => s2.orch.main === 'LEVELLING', T_IDLE_CMD, 'abort',
                    'orchestrator enters LEVELLING'),
            ]);
        }
        if (targetId === 'ACTIVE' || targetId === 'ACTIVE:STANDBY') {
            return mkPlan('IDLE → ACTIVE:STANDBY', activateSteps());
        }
        const m = targetId.indexOf('ACTIVE:') === 0 ? targetId.slice(7) : null;
        if (m && SUBMODES.indexOf(m) >= 0) {
            // Not armed at this point (arming is a separate explicit button),
            // so the mode change is unconditionally safe; STANDBY is automatic
            // on entry and never published.
            return mkPlan('IDLE → ACTIVE:' + m,
                activateSteps().concat(modeSteps(m)));
        }
    } else if (main === 'ACTIVE') {
        if (targetId.indexOf('ACTIVE:') === 0) {
            const Y = targetId.slice(7);
            if (SUBMODES.indexOf(Y) < 0) return { refuse: 'unknown submode ' + Y };
            if (Y === s.orch.sub) return { refuse: 'already in ' + Y };
            return mkPlan('ACTIVE:' + s.orch.sub + ' → ACTIVE:' + Y, modeSteps(Y));
        }
        if (targetId === 'IDLE') {
            return mkPlan('ACTIVE → IDLE (safe teardown)', teardownSteps(s));
        }
        if (targetId === 'HOMING') {
            return mkPlan('ACTIVE → IDLE → HOMING', teardownSteps(s).concat([
                pub('home', (s2) => s2.orch.main === 'IDLE', 'orchestrator reached IDLE'),
                waitStep((s2) => s2.orch.main === 'HOMING', T_AFTER_TEARDOWN, 'abort',
                    'orchestrator enters HOMING (generous: the stow holds operation_pending; ' +
                    'the queued command fires when it completes)'),
            ]));
        }
        if (targetId === 'LEVELLING') {
            return mkPlan('ACTIVE → IDLE → LEVELLING', teardownSteps(s).concat([
                pub('level', (s2) => s2.orch.main === 'IDLE', 'orchestrator reached IDLE'),
                waitStep((s2) => s2.orch.main === 'LEVELLING', T_AFTER_TEARDOWN, 'abort',
                    'orchestrator enters LEVELLING'),
            ]));
        }
    } else if (main === 'FAULT') {
        if (s.robot.has_undervoltage) {
            return { refuse: 'undervoltage active — check E-stop / DC power' };
        }
        if (!robotErrorsClear(s)) {
            if (targetId === 'IDLE') return faultToIdlePlan();
            return { refuse: 'in FAULT — clear errors first (IDLE is the recovery landing state)' };
        }
        if (guardLatched(s)) {
            const mode = SUBMODES.indexOf(s.controlMode) >= 0 ? s.controlMode : null;
            if (mode && (targetId === 'ACTIVE:' + mode || targetId === 'ACTIVE')) {
                return resumePlan(mode);
            }
            return { refuse: 'in FAULT — Recover to resume ' + (mode || 'the previous mode') + ' first' };
        }
        return { refuse: "boot-timeout/opaque FAULT — use the 'Retry Boot' button" };
    }
    return { refuse: 'no safe plan from ' + (s.orch.raw || 'unknown') + ' to ' + targetId };
}

// ====================================================================
// S5. Sequencer engine
// ====================================================================

function PlanAbort(message) {
    const e = new Error(message);
    e.isPlanAbort = true;
    return e;
}

let cmdPub = null;
let activePlan = null;   // { label, steps, recovery, abortReason }
let pendingWait = null;  // { step, deadline, resolve, reject }

function globalAbortReason(plan) {
    if (snap.conn !== 'connected') return 'rosbridge disconnected';
    if (!plan.recovery && snap.orch.main === 'FAULT') {
        const err = snap.robot.error[0] || '';
        return 'orchestrator FAULT during sequence' + (err ? ' (' + err + ')' : '') +
            ' (guard=' + snap.link.faultState + ')';
    }
    return null;
}

/**
 * Convergence pump: called from every snapshot update, and from the 250 ms
 * ticker so wait timeouts fire even if topics stop arriving.
 */
function pump() {
    if (!activePlan) return;
    const g = activePlan.abortReason || globalAbortReason(activePlan);
    if (g) {
        activePlan.abortReason = g;
        if (pendingWait) {
            const w = pendingWait;
            pendingWait = null;
            w.reject(PlanAbort(g));
        }
        return;
    }
    if (pendingWait) {
        let hit = false;
        try { hit = !!pendingWait.step.pred(snap); } catch (e) { hit = false; }
        if (hit) {
            const w = pendingWait;
            pendingWait = null;
            w.resolve(true);
            return;
        }
        if (Date.now() > pendingWait.deadline) {
            const w = pendingWait;
            pendingWait = null;
            w.resolve(false);
        }
    }
}

function waitFor(step) {
    return new Promise((resolve, reject) => {
        pendingWait = { step, deadline: Date.now() + step.timeoutMs, resolve, reject };
        pump();  // predicate may already hold
    });
}

function withTimeout(promise, ms, name) {
    return new Promise((resolve, reject) => {
        const t = setTimeout(() => reject(new Error(
            name + ' timed out after ' + (ms / 1000) + ' s')), ms);
        promise.then(
            (v) => { clearTimeout(t); resolve(v); },
            (e) => { clearTimeout(t); reject(e); },
        );
    });
}

function throwIfAborted() {
    if (!activePlan) return;
    const r = activePlan.abortReason || globalAbortReason(activePlan);
    if (r) throw PlanAbort(r);
}

async function runPlan(plan) {
    if (activePlan) { setSticky('a sequence is already running', 'warn'); return; }
    activePlan = { label: plan.label, steps: plan.steps, recovery: plan.recovery, abortReason: null };
    stickyStatus = null;
    let surfacedMsg = null;
    emitEvent({
        type: EVENT_TYPES.COMMAND,
        label: 'Minimap: ' + plan.label,
        detail: 'sequence start (' + plan.steps.length + ' steps)',
    });
    scheduleRender();
    try {
        for (let i = 0; i < plan.steps.length; i++) {
            const step = plan.steps[i];
            setProgress('[' + (i + 1) + '/' + plan.steps.length + '] ' + step.desc + '…');
            throwIfAborted();
            if (step.kind === 'publish') {
                // Per-step precondition, re-checked at the last moment.
                if (step.pre && !step.pre(snap)) {
                    throw PlanAbort("state diverged before publishing '" + step.data +
                        "' — external command? (expected: " + (step.preDesc || '') + ')');
                }
                cmdPub.publish({ data: step.data });
                emitEvent({
                    type: EVENT_TYPES.COMMAND,
                    label: 'Minimap: ' + step.data,
                    detail: 'orchestrator_command: ' + step.data,
                });
            } else if (step.kind === 'service') {
                emitEvent({
                    type: EVENT_TYPES.COMMAND,
                    label: 'Minimap: ' + step.name,
                    detail: 'service ' + step.name + ' ' + JSON.stringify(step.request),
                });
                let resp = null;
                let failed = null;
                try {
                    resp = await withTimeout(
                        ros.callService(step.name, step.type, step.request),
                        step.clientTimeoutMs, step.name);
                } catch (e) {
                    failed = e.message || String(e);
                }
                if (failed === null && resp && resp.success === false) {
                    failed = resp.message || '(rejected, no message)';
                }
                if (failed !== null) {
                    if (step.onReject === 'continue') {
                        setProgress(step.name + ': ' + failed + ' — continuing (best-effort step)');
                        continue;
                    }
                    throw PlanAbort(step.name + ' failed: ' + failed);
                }
                if (step.surfaceMessage && resp && resp.message) surfacedMsg = resp.message;
            } else if (step.kind === 'wait') {
                const ok = await waitFor(step);
                if (!ok) {
                    if (step.onTimeout === 'continue') {
                        setProgress('timeout waiting for: ' + step.desc + ' — continuing');
                        continue;
                    }
                    throw PlanAbort('timed out after ' + (step.timeoutMs / 1000) +
                        ' s waiting for: ' + step.desc);
                }
            }
        }
        setSticky('OK — ' + plan.label + (surfacedMsg ? ': ' + surfacedMsg : ' complete'), 'ok');
        emitEvent({
            type: EVENT_TYPES.COMMAND,
            label: 'Minimap: ' + plan.label + ' complete',
            detail: surfacedMsg || '',
        });
    } catch (e) {
        const reason = e && e.isPlanAbort ? e.message : 'internal error: ' + (e && e.message);
        setSticky('ABORTED — ' + reason, 'err');
        emitEvent({
            type: EVENT_TYPES.COMMAND,
            label: 'Minimap: sequence aborted',
            detail: reason,
        });
    } finally {
        activePlan = null;
        pendingWait = null;
        progressStatus = null;
        scheduleRender();
    }
}

/** Node hold-confirm dispatch — belt and braces: reachability AND the plan
 *  are both recomputed here, at confirm time, never at pointerdown; the
 *  confirm-time plan must then MATCH the plan pinned at pointerdown. */
function onNodeConfirm(nodeId, el, heldPlanLabel) {
    if (activePlan) return;
    // A renderGraph() rebuild mid-hold (multi-touch: second finger toggles
    // expand/collapse) detaches the held <g>; it never receives pointerup,
    // so its timer can still fire.  Never confirm from a detached element.
    if (el && !el.isConnected) {
        setSticky('layout changed during the hold — hold again', 'warn');
        return;
    }
    const r = reachability(snap)[nodeId];
    if (!r || !r.ok) {
        setSticky('not available — ' + (r ? r.reason : 'unknown node'), 'warn');
        return;
    }
    const p = planFor(snap, nodeId);
    if (!p || p.refuse) {
        setSticky('refused — ' + (p ? p.refuse : 'no plan'), 'warn');
        return;
    }
    // Hold-identity pin: the plan label encodes source→target semantics
    // (e.g. 'ACTIVE → IDLE (safe teardown)' vs 'FAULT → IDLE (clear errors +
    // auto chain)'), so a mid-hold world change that re-targets the node
    // refuses instead of firing the NEW plan on the OLD gesture.
    if (heldPlanLabel === null || p.label !== heldPlanLabel) {
        setSticky('target changed during the hold (' +
            (heldPlanLabel || 'no plan at hold start') + ' → ' + p.label +
            ') — check and hold again', 'warn');
        return;
    }
    runPlan(p);
}

// ====================================================================
// S6. Contextual "necessary commands" button
// ====================================================================

/** The Disarm action variant — bridge-local, idempotent, always safe. */
function disarmAction(title) {
    return {
        key: 'disarm', visible: true, label: 'Disarm',
        title,
        plan: () => mkPlan('Disarm setpoints', [
            svcStep('set_setpoint_output', SETBOOL, { data: false },
                T_SVC_SETPOINT, 'abort', 'disarm the setpoint downlink', true),
        ], true),
    };
}

/**
 * computeAction(snap) — first match wins (minimap plan §8).  Every actuating
 * variant is hold-to-confirm; response.message is surfaced verbatim.
 *
 * Worst-case defaults: any staleness (link/robot/orchestrator) or a lost
 * Teensy uplink DISABLES the clear/arm rules with an honest reason — a fresh
 * link_status with bridge_link!=UP carries a FROZEN fault_state, and frozen
 * data must never enable actuation.  The one deliberate exception is Disarm:
 * last-known mpc_active=1 keeps Disarm available on ANY stale/lost signal
 * (it is bridge-local, idempotent and always safe — hiding the escape hatch
 * behind a disabled Arm was the optimistic direction).
 */
function computeAction(s) {
    if (s.conn !== 'connected') return { visible: false };
    const inFault = s.orch.main === 'FAULT';
    const orchFresh = isOrchFresh(s);
    // First failing verifiability reason (null = everything live).
    const staleReason =
        !isLinkFresh(s) ? 'link_status stale — teensy_bridge_node down?'
        : s.link.bridgeLink !== 'UP' ? 'Teensy uplink lost (bridge_link=' +
            s.link.bridgeLink + ') — guard/arm state unverifiable'
        : !isRobotFresh(s) ? 'robot_state stale — teensy_bridge_node down?'
        : null;

    // Worst-case armed escape hatch — outranks every other rule: if the
    // last-known mpc_active was 1 and ANY signal is stale/lost, keep Disarm
    // available (annotated) instead of degrading to a disabled Arm.
    if (s.link.mpcActive === 1 && (staleReason || !orchFresh)) {
        return disarmAction('last-known mpc_active=1 with ' +
            (staleReason || 'orchestrator_state stale — orchestrator down?') +
            ' — best-effort disarm (idempotent, always safe)');
    }
    // A frozen orchestrator_state must not label the button (the nodes carry
    // the stale reason; nothing else is safely actionable).
    if (!orchFresh) return { visible: false };

    // Rule 2 — a latched guard in ANY orchestrator state (covers the
    // healthy-looking-IDLE benign prior-session latch; the latch survives ROS
    // relaunches — runbook:63-67).  Blocked on stale/lost data: the latch
    // reading is frozen and /clear_errors could not be verified.
    if (guardLatched(s)) {
        if (staleReason) {
            return { key: 'clear-guard', visible: true, disabled: true,
                label: inFault ? 'Recover' : 'Clear Guard Latch', title: staleReason };
        }
        return {
            key: 'clear-guard', visible: true,
            label: inFault ? 'Recover' : 'Clear Guard Latch',
            title: 'Teensy guard latched (' + s.link.faultState +
                (s.link.guardFaultLeg ? ', leg ' + s.link.guardFaultLeg : '') +
                ') — bridge /clear_errors ' +
                '(self-routes converge-first while armed; _svc_clear_errors, teensy_bridge_node.py:3316-3368)',
            plan: () => mkPlan(inFault ? 'Recover (guard latch)' : 'Clear guard latch', [
                svcStep('clear_errors', TRIGGER, {}, T_SVC_CLEAR, 'abort',
                    'bridge /clear_errors Trigger', true),
            ], true),
        };
    }
    // Rule 3 — undervoltage: nothing to click, explain why.
    if (inFault && s.robot.has_undervoltage) {
        return { key: 'undervoltage', visible: true, disabled: true, label: 'Undervoltage',
            title: 'check E-stop / DC power (auto-clears when power returns)' };
    }
    // Rule 4 — real errors, guard clear.
    if (inFault && !robotErrorsClear(s)) {
        if (staleReason) {
            return { key: 'clear-errors', visible: true, disabled: true,
                label: 'Clear Errors', title: staleReason };
        }
        return {
            key: 'clear-errors', visible: true, label: 'Clear Errors',
            title: 'bridge /clear_errors Trigger — errors: ' + (s.robot.error[0] || 'fatal flags'),
            plan: () => mkPlan('Clear errors', [
                svcStep('clear_errors', TRIGGER, {}, T_SVC_CLEAR, 'abort',
                    'bridge /clear_errors Trigger', true),
            ], true),
        };
    }
    // Rule 5 — opaque/boot-timeout FAULT: the ONLY use of the orchestrator
    // 'clear_errors' command path (consumed only in FAULT — clears
    // boot_timed_out with no Teensy-side clear; state_machine.py:569-575).
    if (inFault) {
        if (staleReason) {
            return { key: 'retry-boot', visible: true, disabled: true,
                label: 'Retry Boot', title: staleReason };
        }
        return {
            key: 'retry-boot', visible: true, label: 'Retry Boot',
            title: "publish 'clear_errors' to the orchestrator (boot-timeout FAULT flavour)",
            plan: () => mkPlan('Retry boot', [
                pub('clear_errors', (s2) => s2.orch.main === 'FAULT', 'orchestrator in FAULT',
                    "publish 'clear_errors' (orchestrator topic — boot-timeout clear)"),
                waitStep((s2) => s2.orch.main !== 'FAULT', T_RETRY_BOOT, 'abort',
                    'orchestrator leaves FAULT'),
            ], true),
        };
    }
    if (s.orch.main === 'ACTIVE') {
        const sub = s.orch.sub;
        // Rule 7 — armed: Disarm.  Checked before the arm rules so an armed
        // transient (mode flip echoing) never hides the disarm escape hatch.
        // (Stale/lost-signal armed cases were caught by the worst-case rule
        // above, so mpc_active=1 here is a fresh, verifiable reading.)
        if (s.link.mpcActive === 1) {
            return disarmAction(
                'set_setpoint_output false — stop forwarding setpoints (always succeeds)');
        }
        if (staleReason) {
            return { key: 'arm', visible: true, disabled: true, label: 'Arm Setpoints',
                title: staleReason };
        }
        // Rules 6/6' — ARM, hard-gated on verification (runbook Sharp Edge #5,
        // :107-121): /control_mode_topic must confirm the submode took effect
        // BEFORE arming — a lost mode publish is silent, and arming into the
        // wrong mode is how the 2026-07-09 session tripped MPC_STALE.
        if (s.controlMode !== sub) {
            return { key: 'arm', visible: true, disabled: true, label: 'Arm Setpoints',
                title: 'waiting for control_mode to confirm ' + sub };
        }
        // Plan §8 rule 6 requires guard state NONE *exactly*: UNKNOWN (bridge
        // up, no Teensy heartbeat yet) must not enable Arm — same policy as
        // G_GUARD's UNKNOWN-fails-for-actuation.
        if (s.link.faultState !== 'NONE') {
            return { key: 'arm', visible: true, disabled: true, label: 'Arm Setpoints',
                title: 'guard state unknown (fault_state=' + s.link.faultState +
                    ') — waiting for a Teensy heartbeat' };
        }
        if (s.link.mpcActive === 0) {
            return {
                key: 'arm', visible: true, label: 'Arm Setpoints',
                title: 'set_setpoint_output true — stream-then-arm; the bridge verifies a ' +
                    'fresh mpccmd frame + u0 within tolerance (_arm_setpoint_output, teensy_bridge_node.py:1654-1810)',
                plan: () => mkPlan('Arm setpoints', [
                    svcStep('set_setpoint_output', SETBOOL, { data: true },
                        T_SVC_SETPOINT, 'abort', 'arm the 40 Hz setpoint downlink', true),
                ], false),  // NOT a recovery plan: a FAULT mid-arm aborts
            };
        }
        return { key: 'arm', visible: true, disabled: true, label: 'Arm Setpoints',
            title: 'mpc_active UNVERIFIED — waiting for a parseable /link_status' };
    }
    return { visible: false };
}

/** Action-button confirm: recompute at confirm time; refuse on any drift. */
function onActionConfirm() {
    if (activePlan) return;
    if (dom && dom.action && !dom.action.isConnected) return;
    const a = computeAction(snap);
    if (!a.visible || a.disabled || !a.plan) {
        setSticky('action no longer applicable — state changed during the hold', 'warn');
        return;
    }
    // Hold-identity pin: compare against the key latched at POINTERDOWN.
    // renderedActionKey itself is rewritten by every render (10-20 Hz), so
    // comparing against it here was vacuous — a hold begun as Disarm could
    // fire as Arm if an external disarm landed mid-hold.  heldActionKey is
    // null when no hold-start was observed ⇒ refuse (fail-safe).
    if (a.key !== heldActionKey) {
        setSticky('action changed during the hold — check and hold again', 'warn');
        return;
    }
    runPlan(a.plan());
}

// ====================================================================
// S7. Rendering
// ====================================================================

const SVG_NS = 'http://www.w3.org/2000/svg';
let dom = null;           // { root, header, svg, status, action, badges, ... }
let nodeEls = new Map();  // nodeId -> { g, rect, title, label }
let expanded = false;
let lastReach = null;
let renderedActionKey = null;
let heldActionKey = null;  // action key latched at POINTERDOWN (hold-identity pin)
let renderQueued = false;

// Status line: hover > sequence progress > sticky result > connection note.
let stickyStatus = null;
let progressStatus = null;
let hoverStatus = null;

function setProgress(text) { progressStatus = { text, cls: '' }; renderStatusLine(); }
function setSticky(text, cls) { stickyStatus = { text, cls }; progressStatus = null; renderStatusLine(); }
function setHover(text, cls) { hoverStatus = text ? { text, cls: cls || '' } : null; renderStatusLine(); }

function renderStatusLine() {
    if (!dom) return;
    let s;
    if (hoverStatus) s = hoverStatus;
    else if (progressStatus) s = progressStatus;
    else if (stickyStatus) s = stickyStatus;
    else if (snap.conn !== 'connected') s = { text: 'rosbridge disconnected — all commands unavailable', cls: 'err' };
    else s = { text: '', cls: '' };
    dom.status.textContent = s.text;
    dom.status.className = s.cls ? 'status-' + s.cls : '';
}

function svgEl(tag, attrs, cls) {
    const e = document.createElementNS(SVG_NS, tag);
    if (attrs) for (const k of Object.keys(attrs)) e.setAttribute(k, attrs[k]);
    if (cls) e.setAttribute('class', cls);
    return e;
}

function addNode(svg, id, x, y, w, h, labelText, fontSize, extraCls, labelPos) {
    const g = svgEl('g', { 'data-node': id }, 'minimap-node' + (extraCls ? ' ' + extraCls : ''));
    g.appendChild(svgEl('rect', { x, y, width: w, height: h, rx: 5 }, 'node-rect'));
    g.appendChild(svgEl('rect', { x, y, width: w, height: h, rx: 5 }, 'node-hold-fill'));
    const t = svgEl('text', labelPos === 'topleft'
        ? { x: x + 8, y: y + 4 + fontSize, 'font-size': fontSize }
        : { x: x + w / 2, y: y + h / 2, 'text-anchor': 'middle',
            'dominant-baseline': 'central', 'font-size': fontSize },
        'node-label');
    t.textContent = labelText;
    g.appendChild(t);
    const title = svgEl('title');
    g.appendChild(title);
    svg.appendChild(g);
    nodeEls.set(id, { g, rect: null, title, label: t });
    // EXPANDO, not an attribute: SVGGElement has no native `disabled`, but
    // hold-to-confirm.js:24,36 gates its pointerdown AND its fire-time
    // re-check on `btn.disabled`.  This property (maintained in
    // applySnapshot, initialised here) is what makes holds on greyed / busy
    // nodes inert.  Do NOT convert to setAttribute.
    g.disabled = true;
    // Hold-identity pin: capture the plan identity at POINTERDOWN so a
    // mid-hold world change (external command, FAULT auto-exit, source-state
    // flip) cannot silently re-target the confirm.  onNodeConfirm compares
    // the confirm-time plan label against this latch.
    let heldPlanLabel = null;
    g.addEventListener('pointerdown', () => {
        const r0 = lastReach && lastReach[id];
        if (r0 && r0.ok) {
            const p0 = planFor(snap, id);
            heldPlanLabel = (p0 && !p0.refuse) ? p0.label : null;
        } else {
            heldPlanLabel = null;
        }
    });
    const clearHeld = () => { heldPlanLabel = null; };
    g.addEventListener('pointerup', clearHeld);
    g.addEventListener('pointerleave', clearHeld);
    g.addEventListener('pointercancel', clearHeld);
    holdToConfirm(g, () => onNodeConfirm(id, g, heldPlanLabel), HOLD_MS);
    g.addEventListener('pointerenter', () => {
        const r = lastReach && lastReach[id];
        if (r && !r.ok && !r.isCurrent) setHover(r.reason, 'warn');
    });
    g.addEventListener('pointerleave', () => setHover(null));
}

function renderGraph() {
    const L = LAYOUTS[expanded ? 'expanded' : 'collapsed'];
    const svg = dom.svg;
    while (svg.firstChild) svg.removeChild(svg.firstChild);
    nodeEls = new Map();
    svg.setAttribute('viewBox', L.viewBox);

    const defs = svgEl('defs');
    const marker = svgEl('marker', {
        id: 'mm-arrow', viewBox: '0 0 8 8', refX: '7', refY: '4',
        markerWidth: '5.5', markerHeight: '5.5', orient: 'auto-start-reverse',
    });
    marker.appendChild(svgEl('path', { d: 'M0,0 L8,4 L0,8 z' }, 'minimap-arrow'));
    defs.appendChild(marker);
    svg.appendChild(defs);

    for (const e of L.edges) {
        const el = e.path
            ? svgEl('path', { d: e.path }, 'minimap-edge' + (e.dashed ? ' dashed' : ''))
            : svgEl('line', { x1: e.x1, y1: e.y1, x2: e.x2, y2: e.y2 },
                'minimap-edge' + (e.dashed ? ' dashed' : ''));
        if (e.arrow) el.setAttribute('marker-end', 'url(#mm-arrow)');
        if (e.arrowStart) el.setAttribute('marker-start', 'url(#mm-arrow)');
        svg.appendChild(el);
        if (e.label) {
            const t = svgEl('text', {
                x: e.lx, y: e.ly, 'text-anchor': 'middle', 'font-size': L.edgeLabelFont,
            }, 'minimap-caption');
            if (e.rotate) t.setAttribute('transform', 'rotate(' + e.rotate + ' ' + e.lx + ' ' + e.ly + ')');
            t.textContent = e.label;
            svg.appendChild(t);
        }
    }

    // FAULT halo + captions.  FAULT is latching-until-condition: the live
    // caption shows the blocking condition, never a click-to-clear affordance.
    const f = L.nodes.FAULT;
    svg.appendChild(svgEl('rect', {
        x: f.halo.x, y: f.halo.y, width: f.halo.w, height: f.halo.h, rx: 6,
    }, 'fault-halo'));
    const cap = svgEl('text', {
        x: L.faultCaption.x, y: L.faultCaption.y, 'text-anchor': 'middle',
        'font-size': L.captionFont,
    }, 'minimap-caption');
    cap.textContent = 'any state → on error';
    svg.appendChild(cap);
    dom.faultLive = svgEl('text', {
        x: L.faultLive.x, y: L.faultLive.y, 'text-anchor': 'middle',
        'font-size': L.captionFont,
    }, 'minimap-caption fault-live');
    svg.appendChild(dom.faultLive);

    for (const id of MAIN_STATES) {
        const n = L.nodes[id];
        if (id === 'ACTIVE') {
            addNode(svg, id, n.x, n.y, n.w, n.h, 'ACTIVE', L.nodeFont, 'cluster', 'topleft');
        } else {
            addNode(svg, id, n.x, n.y, n.w, n.h, id, L.nodeFont);
        }
    }

    // Collapsed-only current-submode text inside the cluster.
    dom.clusterSub = null;
    if (L.clusterSub) {
        const clusterEntry = nodeEls.get('ACTIVE');
        dom.clusterSub = svgEl('text', {
            x: L.clusterSub.x, y: L.clusterSub.y, 'font-size': L.nodeFont,
        }, 'cluster-sub');
        clusterEntry.g.insertBefore(dom.clusterSub, clusterEntry.title);
    }

    // Submode chips (2x3).  Rendered in BOTH display modes for DOM stability;
    // CSS hides them while collapsed.
    const c = L.chips;
    for (let i = 0; i < SUBMODES.length; i++) {
        const m = SUBMODES[i];
        const col = i % 2, row = Math.floor(i / 2);
        addNode(svg, 'ACTIVE:' + m, c.x0 + col * c.dx, c.y0 + row * c.dy,
            c.w, c.h, m, L.chipFont, 'submode-chip');
    }
}

function faultLiveText() {
    if (snap.orch.main !== 'FAULT') return '';
    if (snap.robot.has_undervoltage) return 'undervoltage';
    if (!robotErrorsClear(snap)) {
        const e = snap.robot.error[0] ||
            (snap.robot.has_fatal_odrive_error ? 'ODrive fatal' : 'CAN fatal');
        return e.length > 26 ? e.slice(0, 25) + '…' : e;
    }
    if (guardLatched(snap)) return 'guard: ' + snap.link.faultState;
    return 'no visible errors';
}

function clusterSubText() {
    if (snap.orch.main === 'ACTIVE') return snap.orch.sub || 'STANDBY';
    if (snap.orch.main === 'FAULT' && guardLatched(snap) && robotErrorsClear(snap) &&
        SUBMODES.indexOf(snap.controlMode) >= 0) {
        return 'resume: ' + snap.controlMode;
    }
    return '—';
}

function applySnapshot() {
    if (!dom) return;
    const reach = reachability(snap);
    lastReach = reach;
    const busy = !!activePlan;
    const main = snap.orch.main;
    const sub = snap.orch.sub;
    dom.root.classList.toggle('busy', busy);

    for (const [id, n] of nodeEls) {
        const r = reach[id] || { ok: false, reason: '' };
        const isCur = (id === main) ||
            (id === 'ACTIVE' && main === 'ACTIVE') ||
            (main === 'ACTIVE' && id === 'ACTIVE:' + sub);
        n.g.classList.toggle('current', isCur);
        n.g.classList.toggle('unreachable', !isCur && !r.ok);
        n.g.classList.toggle('resume', !!r.resume);
        n.g.classList.toggle('busy', busy);
        // Expando maintained here ONLY — see the loud comment in addNode().
        n.g.disabled = busy || !r.ok;
        n.title.textContent = r.ok
            ? (r.confirmTip || 'hold ' + HOLD_MS + ' ms to confirm')
            : r.reason;
    }
    if (dom.faultLive) dom.faultLive.textContent = faultLiveText();
    if (dom.clusterSub) dom.clusterSub.textContent = clusterSubText();

    // Header badges (same snapshot as the sequencer reads).
    // Worst-case ARMED display: a stale link must never render as disarmed —
    // if the LAST-KNOWN mpc_active was 1, keep the badge visible as 'ARMED?'
    // (amber) until a fresh tick proves 0.  Hiding it on staleness was the
    // one place staleness mapped to 'looks safe' instead of 'looks unknown'.
    const latched = guardLatched(snap);
    if (snap.link.mpcActive === 1) {
        const staleArm = !isLinkFresh(snap);
        dom.armedBadge.style.display = '';
        dom.armedBadge.textContent = staleArm ? 'ARMED?' : 'ARMED';
        dom.armedBadge.classList.toggle('stale', staleArm);
        dom.armedBadge.title = staleArm
            ? 'last-known mpc_active=1 but /link_status is STALE — assume armed ' +
              'until a fresh tick proves otherwise (Disarm stays available)'
            : 'mpc_active=1 — the bridge is forwarding 40 Hz setpoints to the Teensy';
    } else {
        dom.armedBadge.style.display = 'none';
    }
    dom.guardBadge.style.display = latched ? '' : 'none';
    if (latched) {
        dom.guardBadge.title = 'Teensy guard latched: ' + snap.link.faultState +
            ' — survives ROS relaunches; CLEAR_ERRORS required (runbook:63-67)';
    }

    // Contextual button.
    const a = computeAction(snap);
    renderedActionKey = a.visible ? a.key : null;
    dom.action.style.display = a.visible ? '' : 'none';
    if (a.visible) {
        dom.action.textContent = a.label;
        dom.action.disabled = !!a.disabled || busy;
        dom.action.title = (a.title || '') + (a.disabled || busy ? '' : ' — hold to confirm');
    }
    renderStatusLine();
}

function scheduleRender() {
    if (renderQueued || !dom) return;
    renderQueued = true;
    requestAnimationFrame(() => {
        renderQueued = false;
        applySnapshot();
    });
}

function setExpanded(exp, persist) {
    expanded = !!exp;
    dom.root.classList.toggle('expanded', expanded);
    dom.root.classList.toggle('collapsed', !expanded);
    const pane = document.getElementById('viewer-pane');
    // The pane-level modifier drives the scene push-right + command-strip
    // inset in state-minimap.css.  No CSS transition on any property that
    // changes #viewer-container's layout box — the resize is a snap, and
    // viewer.js's ResizeObserver re-fits the camera once.
    if (pane) pane.classList.toggle('minimap-expanded', expanded);
    dom.chevron.textContent = expanded ? '▾' : '▸';
    renderGraph();
    applySnapshot();
    if (persist !== false) {
        try { localStorage.setItem(LS_EXPANDED_KEY, expanded ? 'true' : 'false'); } catch (e) { /* blocked */ }
    }
}

// ====================================================================
// S8. init
// ====================================================================

function buildDom() {
    const root = document.getElementById('state-minimap');
    if (!root) return false;
    root.innerHTML = '';

    const header = document.createElement('div');
    header.id = 'minimap-header';
    const title = document.createElement('span');
    title.className = 'minimap-title';
    title.textContent = 'State Machine';
    const armed = document.createElement('span');
    armed.id = 'minimap-armed-badge';
    armed.className = 'badge minimap-badge';
    armed.textContent = 'ARMED';
    armed.style.display = 'none';
    armed.title = 'mpc_active=1 — the bridge is forwarding 40 Hz setpoints to the Teensy';
    const guard = document.createElement('span');
    guard.id = 'minimap-guard-badge';
    guard.className = 'badge minimap-badge fault';
    guard.textContent = 'GUARD';
    guard.style.display = 'none';
    const chev = document.createElement('span');
    chev.id = 'minimap-chevron';
    chev.textContent = '▸';
    header.appendChild(title);
    header.appendChild(armed);
    header.appendChild(guard);
    header.appendChild(chev);
    header.addEventListener('click', () => setExpanded(!expanded));

    const svg = svgEl('svg', { id: 'minimap-svg' });
    const status = document.createElement('div');
    status.id = 'minimap-status';
    const action = document.createElement('button');
    action.id = 'minimap-action';
    action.type = 'button';
    action.className = 'cmd-btn hold-fillable';
    action.style.display = 'none';
    holdToConfirm(action, onActionConfirm, HOLD_MS);
    // Hold-identity pin (see onActionConfirm): latch the rendered action key
    // at POINTERDOWN — the confirm-time key must equal this latch, so a hold
    // begun as Disarm can never fire as Arm after a mid-hold relabel.
    action.addEventListener('pointerdown', () => { heldActionKey = renderedActionKey; });
    const clearHeldAction = () => { heldActionKey = null; };
    action.addEventListener('pointerup', clearHeldAction);
    action.addEventListener('pointerleave', clearHeldAction);
    action.addEventListener('pointercancel', clearHeldAction);

    root.appendChild(header);
    root.appendChild(svg);
    root.appendChild(status);
    root.appendChild(action);

    dom = {
        root, header, svg, status, action,
        armedBadge: armed, guardBadge: guard, chevron: chev,
        faultLive: null, clusterSub: null,
    };
    return true;
}

let minimapInitialised = false;

export function initStateMinimap() {
    // Idempotence guard: a second call would stack another ResizeObserver,
    // another permanent connection listener (ros-bridge's stateListeners
    // array only ever grows) and another un-cancellable 250 ms ticker.
    if (minimapInitialised) {
        console.warn('initStateMinimap: already initialised — ignoring repeat call');
        return;
    }
    if (!buildDom()) return;
    minimapInitialised = true;

    // ros.advertise caches by topic name (ros-bridge.js:244-268), so this IS
    // the same publisher handle commands.js uses — long-lived, reconnect-safe
    // via recreatePublishers(), immune to the DDS --once discovery race.
    cmdPub = ros.advertise('orchestrator_command', 'std_msgs/msg/String');

    let exp = false;
    try { exp = localStorage.getItem(LS_EXPANDED_KEY) === 'true'; } catch (e) { /* blocked */ }
    setExpanded(exp, false);

    // Collapsed bottom offset must clear the command strip, whose height
    // changes with the 10-22 px font-size control — measure it, never
    // hardcode (CSS var consumed by state-minimap.css).  The same observer
    // watches #viewer-pane to keep the expanded width PANE-relative: a
    // viewport-relative cap (40vw) can exceed a pane narrowed by wide
    // sidebars, driving #viewer-container's width to zero (viewer.js's
    // ResizeObserver has no zero guard — the 3D scene and the command strip
    // would vanish).  min(420, pane*0.55) never lets the container reach 0.
    const pane = document.getElementById('viewer-pane');
    const strip = document.getElementById('command-overlay');
    if (pane && strip && typeof ResizeObserver !== 'undefined') {
        const update = () => {
            pane.style.setProperty('--command-strip-h', strip.offsetHeight + 'px');
            const w = Math.min(420, Math.round(pane.clientWidth * 0.55));
            pane.style.setProperty('--minimap-width', w + 'px');
        };
        const ro = new ResizeObserver(update);
        ro.observe(strip);
        ro.observe(pane);
        update();
    }

    ros.onConnectionStateChange(onConnChange);  // fires immediately with current state
    setInterval(() => { pump(); scheduleRender(); }, TICK_MS);
}
