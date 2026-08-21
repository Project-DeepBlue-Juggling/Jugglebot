/**
 * panels.js — Right sidebar panel creation + data update functions.
 *
 * Creates DOM elements for all panels and provides update functions
 * that are called from main.js when new ROS data arrives.
 */

import { ODRIVE_STATE, BB_STATE_NAMES, LEG_STROKE_MM, MM_TO_REV,
         CC_DELTA_OK_MS, CC_DELTA_WARN_MS,
         CC_OFFSET_DISPLAY_LIMIT_MS, CC_OFFSET_HISTORY_LEN } from './geometry-config.js';
import { callService } from './ros-bridge.js';
import { onWorkspaceStatus } from './jog-panel.js';

// ---- Motor grid ----

const MOTOR_LABELS = ['Leg 0', 'Leg 1', 'Leg 2', 'Leg 3', 'Leg 4', 'Leg 5', 'Hand'];

export function initMotorGrid() {
    const grid = document.getElementById('motor-grid');
    if (!grid) return;

    for (let i = 0; i < 7; i++) {
        const col = document.createElement('div');
        col.className = 'motor-col';
        col.id = `motor-${i}`;

        col.innerHTML = `
            <span class="motor-label">${MOTOR_LABELS[i]}</span>
            <span class="motor-state-dot undefined" id="motor-dot-${i}"></span>
            <span class="motor-value" id="motor-pos-${i}">--</span>
            <span class="motor-value" id="motor-vel-${i}">--</span>
            <span class="motor-value" id="motor-temp-${i}">--</span>
        `;
        grid.appendChild(col);
    }
}

/**
 * Update motor status from robot_state message.
 * @param {object[]} motorStates - Array of MotorStateSingle objects
 */
export function updateMotorGrid(motorStates) {
    for (let i = 0; i < Math.min(motorStates.length, 7); i++) {
        const m = motorStates[i];

        // State dot
        const dot = document.getElementById(`motor-dot-${i}`);
        if (dot) {
            dot.className = 'motor-state-dot ' + stateClass(m.current_state);
        }

        // Position (show as mm for legs, rev for hand)
        const posEl = document.getElementById(`motor-pos-${i}`);
        if (posEl) {
            if (i < 6) {
                // Convert rev to mm: rev / mm_to_rev (positive = extension)
                const mm = m.pos_estimate / MM_TO_REV[i];
                posEl.textContent = mm.toFixed(1) + ' mm';
            } else {
                posEl.textContent = m.pos_estimate.toFixed(2) + ' rev';
            }
        }

        // Velocity
        const velEl = document.getElementById(`motor-vel-${i}`);
        if (velEl) {
            velEl.textContent = m.vel_estimate.toFixed(1) + ' r/s';
        }

        // FET temperature
        const tempEl = document.getElementById(`motor-temp-${i}`);
        if (tempEl) {
            const temp = m.fet_temp;
            tempEl.textContent = temp.toFixed(0) + '\u00b0C';
            tempEl.className = 'motor-value ' + tempClass(temp);
        }
    }

    // Bus voltage (from first motor)
    if (motorStates.length > 0) {
        const vEl = document.getElementById('bus-voltage-value');
        if (vEl) vEl.textContent = motorStates[0].bus_voltage.toFixed(1);
    }
}

function stateClass(state) {
    switch (state) {
        case ODRIVE_STATE.CLOSED_LOOP: return 'closed-loop';
        case ODRIVE_STATE.IDLE: return 'idle-state';
        case ODRIVE_STATE.UNDEFINED: return 'undefined';
        default:
            // Any error-like state
            return state === 0 ? 'undefined' : 'idle-state';
    }
}

function tempClass(temp) {
    if (temp < 50) return 'temp-ok';
    if (temp < 70) return 'temp-warm';
    return 'temp-hot';
}

// ---- Orchestrator state badge ----

const STATE_CLASSES = {
    'BOOT': 'boot',
    'HOMING': 'homing',
    'LEVELLING': 'homing',  // Blue, like HOMING
    'IDLE': 'idle',
    'ACTIVE': 'active',
    'FAULT': 'fault',
};

/**
 * Update the orchestrator state badge.
 * @param {string} stateStr - e.g. "IDLE", "ACTIVE:SPACEMOUSE"
 */
export function updateOrchestratorState(stateStr) {
    const badge = document.getElementById('state-badge');
    const subMode = document.getElementById('state-sub-mode');
    if (!badge) return;

    const parts = stateStr.split(':');
    const mainState = parts[0].toUpperCase();
    const sub = parts[1] || '';

    badge.textContent = mainState;
    badge.className = 'badge state-badge ' + (STATE_CLASSES[mainState] || 'boot');

    if (mainState === 'FAULT') {
        badge.classList.add('fault-pulse');
    }

    if (subMode) {
        subMode.textContent = sub ? sub.toUpperCase() : '';
    }

    // Store for command enable/disable
    currentOrchestratorState = mainState;
    currentSubMode = sub;
}

export let currentOrchestratorState = 'BOOT';
export let currentSubMode = '';

// ---- System flags ----

// Layout: 2-column grid, ordered left-to-right then top-to-bottom.
// Left column: error-style flags (good=tick, bad=cross)
// Right column: progress-style flags (done=tick, waiting=dot)
// Row 4: Mocap connected (left, updated via setMocapConnected) + aligned (right, via setMocapAligned)
// Row 5: Ball-in-hand pill (left, via updateBallHeld). type 'pill' carries NO
// flag icon — it is driven by /hand_telemetry, not robot_state, and the pill
// itself is the state, so a tick/cross/dot beside it would be dead furniture.
const FLAGS = [
    // Row 1
    { id: 'flag-undervoltage', type: 'error', labelOk: 'Voltage OK',       labelFail: 'Undervoltage' },
    { id: 'flag-encoder',      type: 'progress', labelDone: 'Encoders Found', labelWait: 'Encoder Search' },
    // Row 2
    { id: 'flag-fatal-odrive', type: 'error', labelOk: 'ODrives OK',       labelFail: 'ODrive Error' },
    { id: 'flag-homed',        type: 'progress', labelDone: 'Homed',          labelWait: 'Not Homed' },
    // Row 3
    { id: 'flag-fatal-can',    type: 'error', labelOk: 'CAN OK',           labelFail: 'CAN Error' },
    { id: 'flag-levelled',     type: 'error', labelOk: 'Levelled',         labelFail: 'Not Levelled' },
    // Row 4
    { id: 'flag-mocap',        type: 'error', labelOk: 'Mocap Connected',  labelFail: 'Mocap Disconnected' },
    { id: 'flag-mocap-aligned', type: 'error', labelOk: 'Mocap Aligned',  labelFail: 'Mocap Misaligned' },
    // Row 5
    { id: 'flag-ball-held',    type: 'pill', label: 'Ball in Hand' },
];

export function initFlagsGrid() {
    const grid = document.getElementById('flags-grid');
    if (!grid) return;

    for (const flag of FLAGS) {
        const item = document.createElement('div');
        if (flag.type === 'spacer') {
            item.className = 'flag-item flag-spacer';
            grid.appendChild(item);
            continue;
        }
        if (flag.type === 'pill') {
            // Tri-state pill, no flag icon. The INITIAL DOM STATE IS UNKNOWN and
            // that is load-bearing: /hand_telemetry does not publish at all until
            // the bridge's first Telemetry frame lands (teensy_bridge_node's
            // _publish_hand_telemetry early-returns), so the topic cannot be
            // relied on to drive the pill to UNKNOWN — it has to be born there.
            item.className = 'flag-item';
            item.innerHTML = `
                <span>${flag.label}</span>
                <span class="flag-ball-pill unknown" id="${flag.id}-pill">UNKNOWN</span>
            `;
            grid.appendChild(item);
            continue;
        }
        item.className = 'flag-item';
        const defaultLabel = flag.type === 'error' ? flag.labelOk : flag.labelWait;
        item.innerHTML = `
            <span class="flag-icon wait" id="${flag.id}-icon">&#x25cf;</span>
            <span id="${flag.id}-label">${defaultLabel}</span>
        `;
        if (flag.id === 'flag-levelled') {
            // Measured-tilt pill, populated once levelling completes
            // (replaces the old standalone Levelling panel).
            const pill = document.createElement('span');
            pill.className = 'flag-level-pill';
            pill.id = 'flag-levelled-angles';
            pill.style.display = 'none';
            pill.title = 'Measured platform tilt at levelling — roll (X) / pitch (Y)';
            item.appendChild(pill);
        }
        grid.appendChild(item);
    }
}

/**
 * Update system flags from robot_state.
 */
export function updateFlags(robotState) {
    setFlagProgress('flag-encoder', robotState.encoder_search_complete, 'Encoders Found', 'Encoder Search');
    setFlagProgress('flag-homed', robotState.is_homed, 'Homed', 'Not Homed');
    setFlagError('flag-levelled', !robotState.levelling_complete, 'Not Levelled', 'Levelled');
    updateLevelledAngles(robotState);
    setFlagError('flag-fatal-odrive', robotState.has_fatal_odrive_error, 'ODrive Error', 'ODrives OK');
    setFlagError('flag-fatal-can', robotState.has_fatal_can_error, 'CAN Error', 'CAN OK');
    setFlagError('flag-undervoltage', robotState.has_undervoltage, 'Undervoltage', 'Voltage OK');
}

/** Last-known mocap connection state, from main.js's mocap_data watchdog.
 *  Read by applyBBCalibrateGate() (F4/Q4) — the Calibrate button is useless
 *  without QTM marker data, so it follows this flag. */
let mocapConnected = false;

/** Last-known Ball Butler connection state.
 *
 *  Tracked EXPLICITLY rather than inferred from lastBBState, which is a
 *  last-value cache that setBBDisconnected does not invalidate. Without this
 *  flag the gate had a live re-enable path: BB disconnects (button correctly
 *  disabled, title 'Ball Butler disconnected'), lastBBState still holds 1 from
 *  the final IDLE heartbeat, and then the very next mocap_data frame calls
 *  setMocapConnected(true) -> applyBBCalibrateGate(), which sees bbIdle true
 *  and mocapConnected true and RE-ENABLES Calibrate on a Ball Butler that is
 *  not there. mocap_data runs continuously, so that is not a corner case — it
 *  is what happens within milliseconds of every BB disconnect. */
let bbConnected = false;

/**
 * Update the mocap connected flag.
 * @param {boolean} connected
 */
export function setMocapConnected(connected) {
    mocapConnected = !!connected;
    setFlagError('flag-mocap', !connected, 'Mocap Disconnected', 'Mocap Connected');
    // Re-evaluate the Calibrate gate: this flag flips on the mocap_data
    // watchdog, NOT on a BB heartbeat, so nothing else would re-run the rule.
    applyBBCalibrateGate();
}

/**
 * Update the mocap alignment flag.
 * @param {boolean} aligned
 */
export function setMocapAligned(aligned) {
    setFlagError('flag-mocap-aligned', !aligned, 'Mocap Misaligned', 'Mocap Aligned');
}

function setFlagProgress(id, value, labelDone, labelWait) {
    const icon = document.getElementById(id + '-icon');
    const label = document.getElementById(id + '-label');
    if (!icon) return;
    icon.className = 'flag-icon ' + (value ? 'ok' : 'wait');
    icon.innerHTML = value ? '&#x2713;' : '&#x25cf;';
    if (label) label.textContent = value ? labelDone : labelWait;
}

function setFlagError(id, hasError, labelFail, labelOk) {
    const icon = document.getElementById(id + '-icon');
    const label = document.getElementById(id + '-label');
    if (!icon) return;
    icon.className = 'flag-icon ' + (hasError ? 'fail' : 'ok');
    icon.innerHTML = hasError ? '&#x2717;' : '&#x2713;';
    if (label) label.textContent = hasError ? labelFail : labelOk;
}

/**
 * Show/refresh the measured levelling angles next to the "Levelled" flag.
 * Visible only while levelling_complete \u2014 an un-levelled (or re-levelling)
 * platform shows just the flag, no stale numbers.
 * @param {object} robotState - RobotState message
 */
function updateLevelledAngles(robotState) {
    const pill = document.getElementById('flag-levelled-angles');
    if (!pill) return;
    const offset = robotState.pose_offset_rad || [];
    if (robotState.levelling_complete && offset.length >= 2) {
        const rollDeg = (offset[0] * 180 / Math.PI).toFixed(2);
        const pitchDeg = (offset[1] * 180 / Math.PI).toFixed(2);
        pill.textContent = `X ${rollDeg}\u00b0 Y ${pitchDeg}\u00b0`;
        pill.style.display = '';
    } else {
        pill.style.display = 'none';
    }
}

// ---- Ball-in-hand pill (hand ball-present sensor) ----

/** Last raw-vs-debounced disagreement, ms epoch. 0 = no marker latched. */
let ballFlickerAtMs = 0;
/** Last DEBOUNCED verdict on a valid sample; null = none yet / validity gap. */
let ballLastHeld = null;
/** How long a single disagreeing sample keeps the "~" marker lit. */
const BALL_FLICKER_HOLD_MS = 1000;

/**
 * Update the ball-in-hand pill from a HandTelemetryMessage.
 *
 * Four visual states, from the four message fields:
 *   HELD    (filled)  ball_held_valid && ball_held
 *   EMPTY   (hollow)  ball_held_valid && !ball_held
 *   UNKNOWN (greyed)  !ball_held_valid, or no message at all
 *   ~ suffix          ball_held_raw disagrees with ball_held on a VALID reading
 *
 * UNKNOWN is NEVER rendered as EMPTY: boot, staleness and an un-anchored bridge
 * clock all mean "we don't know", not "no ball" (plans/archived/hand-ball-sensor.md
 * § Architecture, normative).
 *
 * FLICKER SEMANTICS — the marker means "spotty contact", NOT "an edge happened".
 * Every verdict flip is ACCOMPANIED by ~100 ms of raw-vs-debounced disagreement,
 * because the debounce needs MAX_MISSING_SAMPLES (5 at a 50 Hz poll) before the
 * verdict follows the raw bit. Latching that would light the marker on every
 * ordinary press and release — the debounce doing exactly its job, reported as a
 * fault. So the latch is CLEARED whenever the debounced verdict changes value,
 * and SET only on a valid sample whose raw bit disagrees with the (unchanged)
 * verdict. With that clear in place, a `~` that persists past an edge is
 * unambiguous evidence of GENUINE mid-carry flicker — which is the operator
 * observability the pill exists for. The latch is also cleared on any invalid
 * sample, so a sub-second UNKNOWN gap cannot relight a pre-gap marker on the
 * far side of it. (Note the one case that still latches across a gap: the FIRST
 * valid sample after a validity gap. ballLastHeld was reset to null, so that
 * sample takes the flip-clear branch — but there was no prior verdict for it to
 * be "unchanged" from; the verdict is newly DEFINED rather than unchanged, and
 * the set below re-latches if raw disagrees. Deliberate: a raw-vs-debounced
 * disagreement on the first reading back from a stale window is worth
 * surfacing, not suppressing.)
 *
 * The marker is held for BALL_FLICKER_HOLD_MS because the sensor is polled at
 * 50 Hz while this subscription is throttled to 10 Hz — an instantaneous marker
 * would blink for one frame and be unseeable. It is an event marker, not a live
 * state; the quantitative surface is a rosbag of /hand_telemetry (the ball-soak
 * step — tests/hardware/session_hand_ball_sensor.md step 6 / plan Phase 7 step 5
 * — counts raw dropouts per distinct ball_held_stamp).
 *
 * The marker is SINGLE-encoded as a text suffix (no colour/ring channel): the
 * plan asks for "a flicker marker", singular, and text is colourblind-safe and
 * assertable from a DOM string.
 *
 * @param {object|null} msg - HandTelemetryMessage, or null for "no data"
 *                            (boot, watchdog timeout, websocket down).
 */
export function updateBallHeld(msg) {
    const pill = document.getElementById('flag-ball-held-pill');
    if (!pill) return;
    const valid = !!(msg && msg.ball_held_valid);
    const held = valid && !!msg.ball_held;

    if (!valid) {
        ballFlickerAtMs = 0;
        ballLastHeld = null;
    } else {
        if (held !== ballLastHeld) ballFlickerAtMs = 0;   // verdict flip: not flicker
        ballLastHeld = held;
        if (!!msg.ball_held_raw !== held) ballFlickerAtMs = Date.now();
    }
    const flicker = ballFlickerAtMs !== 0
        && (Date.now() - ballFlickerAtMs) < BALL_FLICKER_HOLD_MS;

    let klass, text;
    if (!valid)     { klass = 'unknown'; text = 'UNKNOWN'; }
    else if (held)  { klass = 'held';    text = 'HELD'; }
    else            { klass = 'empty';   text = 'EMPTY'; }
    pill.className = 'flag-ball-pill ' + klass;
    pill.textContent = flicker ? text + ' ~' : text;
}

// ---- Collapsible sidebar panels ----
//
// A general, minimal mechanism: clicking a collapsible .panel-header toggles
// .collapsed on its .panel; CSS (`.panel.collapsed > *:not(.panel-header)`)
// hides every non-header child, so it works for heterogeneous panel bodies
// without per-panel wiring.  #panel-history is deliberately EXCLUDED here — it
// owns its own collapse handler in command-history.js; the shared CSS rule is
// compatible with it, we just must not bind a second click handler.
//
// Sidebar collapse state is intentionally NOT persisted to localStorage: the
// connection contract (BB / Catching Cone auto expand-on-connect,
// collapse-on-disconnect) plus fresh defaults should compute cleanly on every
// load.  Persisting would let a stale "collapsed while connected" (or vice
// versa) state fight the connection edges.
const COLLAPSIBLE_PANEL_IDS = [
    'panel-orchestrator', 'panel-flags', 'panel-motion',
    'panel-bb', 'panel-catching-cone', 'panel-tracking',
];

/** Panels that DEFAULT collapsed — the two device panels that start
 *  disconnected and auto-expand on their connect edge. */
const DEFAULT_COLLAPSED_PANEL_IDS = ['panel-bb', 'panel-catching-cone'];

/** Collapse/expand a panel by id (helper for the connection-edge drivers). */
function setPanelCollapsed(id, collapsed) {
    const panel = document.getElementById(id);
    if (panel) panel.classList.toggle('collapsed', collapsed);
}

export function initCollapsiblePanels() {
    for (const id of COLLAPSIBLE_PANEL_IDS) {
        const panel = document.getElementById(id);
        if (!panel) continue;
        const header = panel.querySelector('.panel-header');
        if (!header) continue;

        header.classList.add('panel-header-collapsible');

        // Chevron affordance (rotates via CSS when the panel is collapsed).
        const chevron = document.createElement('span');
        chevron.className = 'panel-collapse-chevron';
        chevron.innerHTML = '&#x25BC;';  // ▼
        header.appendChild(chevron);

        header.addEventListener('click', (e) => {
            // Don't toggle when the click landed on an interactive control
            // inside the header (button/select/input/link) — those have their
            // own behaviour and must not double as a collapse gesture.
            if (e.target.closest('button, select, input, textarea, a')) return;
            panel.classList.toggle('collapsed');
        });

        if (DEFAULT_COLLAPSED_PANEL_IDS.includes(id)) {
            panel.classList.add('collapsed');
        }
    }
}

// ---- Ball Butler panel ----

/** Last known BB state code (from heartbeat). */
let lastBBState = -1;

/** Last-known BB connected bool — drives auto expand/collapse of #panel-bb on
 *  the connect/disconnect EDGE only (manual header clicks between edges are
 *  honoured).  null until the first heartbeat / disconnect is processed. */
let lastBBConnected = null;

function driveBBConnectionEdge(connected) {
    if (lastBBConnected === connected) return;
    setPanelCollapsed('panel-bb', !connected);  // connect → expand, disconnect → collapse
    lastBBConnected = connected;
}

function onBBCalibrateClick() {
    const btn = document.getElementById('bb-calibrate-btn');
    if (!btn || btn.disabled) return;

    btn.disabled = true;
    btn.textContent = 'Calibrating...';

    callService('bb/calibrate', 'std_srvs/srv/Trigger')
        .then((result) => {
            if (result.success) {
                btn.textContent = 'Done';
            } else {
                btn.textContent = 'Failed';
                console.warn('BB calibrate failed:', result.message);
            }
            // Revert label after 2s; state update will re-enable if still IDLE
            setTimeout(() => { btn.textContent = 'Calibrate'; }, 2000);
        })
        .catch((err) => {
            btn.textContent = 'Error';
            console.error('BB calibrate service error:', err);
            setTimeout(() => { btn.textContent = 'Calibrate'; }, 2000);
        });
}

function onBBResetClick() {
    const btn = document.getElementById('bb-calibrate-btn');
    if (!btn || btn.disabled) return;

    btn.disabled = true;
    btn.textContent = 'Resetting...';

    callService('bb/reset', 'std_srvs/srv/Trigger')
        .then((result) => {
            if (!result.success) {
                console.warn('BB reset failed:', result.message);
            }
            // State update will swap button back once out of ERROR
        })
        .catch((err) => {
            btn.textContent = 'RESET';
            btn.disabled = false;
            console.error('BB reset service error:', err);
        });
}

export function initBBPanel() {
    const content = document.getElementById('bb-content');
    if (!content) return;

    content.innerHTML = `
        <div class="bb-calibrate-row">
            <button class="bb-calibrate-btn" id="bb-calibrate-btn" disabled>Calibrate</button>
        </div>
        <div style="display:flex; align-items:center; gap:8px; margin-bottom:4px;">
            <span class="bb-ball-indicator">
                <span class="bb-ball-dot no-ball" id="bb-ball-dot"></span>
                <span id="bb-ball-text">No ball</span>
            </span>
            <span class="bb-calib-indicator" id="bb-calib-indicator">
                <span class="bb-calib-dot uncalibrated" id="bb-calib-dot"></span>
                <span id="bb-calib-text">Not Calibrated</span>
            </span>
        </div>
        <div class="bb-readouts">
            <div class="bb-readout">
                <div class="label">Yaw</div>
                <div class="value" id="bb-yaw">--</div>
            </div>
            <div class="bb-readout">
                <div class="label">Pitch</div>
                <div class="value" id="bb-pitch">--</div>
            </div>
            <div class="bb-readout">
                <div class="label">Hand</div>
                <div class="value" id="bb-hand">--</div>
            </div>
        </div>
    `;

    // Wire up calibrate button
    const btn = document.getElementById('bb-calibrate-btn');
    btn.addEventListener('click', onBBCalibrateClick);
}

// Map BB state codes to CSS badge classes (matches orchestrator state colours)
const BB_STATE_CLASSES = {
    0: 'boot',          // BOOT — grey
    1: 'idle',          // IDLE — amber
    2: 'active',        // TRACKING — green
    3: 'active',        // THROWING — green
    4: 'homing',        // RELOADING — blue
    5: 'homing',        // CALIBRATING — blue
    6: 'homing',        // CHECKING_BALL — blue
    127: 'fault',       // ERROR — red pulsing
};

/**
 * Update Ball Butler panel from heartbeat.
 * @param {object} hb - BallButlerHeartbeat message
 */
export function updateBBPanel(hb) {
    // Handle disconnected state — show placeholder values and bail early
    // (setBBDisconnected drives the collapse edge).
    if (!hb.connected) {
        setBBDisconnected();
        return;
    }

    // Connected — auto-expand the panel on the connect edge.
    driveBBConnectionEdge(true);
    bbConnected = true;

    const badge = document.getElementById('bb-state-badge');
    if (badge) {
        const name = BB_STATE_NAMES[hb.state] || 'UNKNOWN';
        badge.textContent = name;
        const cls = BB_STATE_CLASSES[hb.state] || 'boot';
        badge.className = 'badge bb-state-badge state-badge ' + cls;
        if (hb.state === 127) badge.classList.add('fault-pulse');
    }

    const ballDot = document.getElementById('bb-ball-dot');
    const ballText = document.getElementById('bb-ball-text');
    if (ballDot) {
        ballDot.className = 'bb-ball-dot ' + (hb.ball_in_hand ? 'has-ball' : 'no-ball');
    }
    if (ballText) {
        ballText.textContent = hb.ball_in_hand ? 'Ball in hand' : 'No ball';
    }

    const yaw = document.getElementById('bb-yaw');
    const pitch = document.getElementById('bb-pitch');
    const hand = document.getElementById('bb-hand');
    if (yaw) yaw.textContent = hb.yaw_deg.toFixed(1) + '\u00b0';
    if (pitch) pitch.textContent = hb.pitch_deg.toFixed(1) + '\u00b0';
    if (hand) hand.textContent = hb.hand_pos_mm.toFixed(0) + ' mm';

    // Swap button between Calibrate (IDLE) and RESET (ERROR)
    lastBBState = hb.state;
    const calBtn = document.getElementById('bb-calibrate-btn');
    if (calBtn) {
        const isError = hb.state === 127;
        const wasReset = calBtn.classList.contains('bb-reset-mode');

        if (isError && !wasReset) {
            // Switch to RESET button
            calBtn.textContent = 'RESET';
            calBtn.classList.add('bb-reset-mode');
            calBtn.removeEventListener('click', onBBCalibrateClick);
            calBtn.addEventListener('click', onBBResetClick);
            calBtn.disabled = false;
            calBtn.title = '';
        } else if (!isError && wasReset) {
            // Switch back to Calibrate button
            calBtn.textContent = 'Calibrate';
            calBtn.classList.remove('bb-reset-mode');
            calBtn.removeEventListener('click', onBBResetClick);
            calBtn.addEventListener('click', onBBCalibrateClick);
            applyBBCalibrateGate();
        } else if (!isError && calBtn.textContent === 'Calibrate') {
            applyBBCalibrateGate();
        }
    }
}

/**
 * Apply the Calibrate button's enable rule (F4/Q4).
 *
 * THREE preconditions, checked in that order. BB must be CONNECTED at all —
 * the gate is re-run from the mocap_data watchdog, which keeps ticking after
 * Ball Butler goes away, so without an explicit connection flag the first
 * mocap frame after a BB disconnect re-enabled the button off a stale
 * lastBBState (see the bbConnected declaration). BB must be IDLE (state 1) —
 * it cannot start a sweep from any other state — AND mocap must be connected,
 * because the calibrate COMMAND path (this button -> bb/calibrate -> the BB
 * firmware sweeps) and the calibrate DATA path (QTM -> mocap_node -> the
 * circle fit) share no edge. With QTM down the press used to move the machine
 * through a full sweep that could produce nothing at all, or — worse, with a
 * partly-visible constellation — a plausible BB pose that every subsequent
 * throw is then aimed with. teensy_bridge_node now refuses such a call outright
 * (QTM_STALE / BB_MARKERS_NOT_VISIBLE, no RPC dispatched); this rule is the
 * cheaper half of the same gate — the operator sees WHY before pressing rather
 * than after.
 *
 * The tooltip names the blocking reason: a greyed button with no explanation
 * is the failure mode this is meant to prevent, not a milder version of it.
 *
 * Deliberately does NOT touch the button in RESET mode — the BB-ERROR reset
 * has nothing to do with mocap and must stay pressable with QTM down.
 */
function applyBBCalibrateGate() {
    const calBtn = document.getElementById('bb-calibrate-btn');
    if (!calBtn) return;
    if (calBtn.classList.contains('bb-reset-mode')) return;
    if (calBtn.textContent !== 'Calibrate') return;  // mid-flight label

    const bbIdle = lastBBState === 1;
    if (!bbConnected) {
        calBtn.disabled = true;
        calBtn.title = 'Ball Butler disconnected';
    } else if (!bbIdle) {
        calBtn.disabled = true;
        calBtn.title = 'Ball Butler must be IDLE to calibrate';
    } else if (!mocapConnected) {
        calBtn.disabled = true;
        calBtn.title = 'Mocap disconnected — calibration needs QTM marker data';
    } else {
        calBtn.disabled = false;
        calBtn.title = '';
    }
}

export function setBBDisconnected() {
    // Auto-collapse the panel on the disconnect edge (fires for both a
    // BB-reported disconnected heartbeat and a full GUI↔rosbridge drop, which
    // calls this from main.js — mirrors how Catching Cone is handled).
    driveBBConnectionEdge(false);
    // Invalidate BOTH pieces of state the Calibrate gate reads. lastBBState is
    // a last-value cache, and leaving it at its final IDLE value let the next
    // mocap_data frame's setMocapConnected() -> applyBBCalibrateGate() re-enable
    // the button on an absent Ball Butler; bbConnected is the primary guard and
    // resetting lastBBState makes the stale value unusable either way.
    bbConnected = false;
    lastBBState = -1;

    const badge = document.getElementById('bb-state-badge');
    if (badge) {
        badge.textContent = 'Disconnected';
        badge.className = 'badge';
    }

    // Blank readouts — stale data is unreliable (axes can be backdriven)
    const yaw = document.getElementById('bb-yaw');
    const pitch = document.getElementById('bb-pitch');
    const hand = document.getElementById('bb-hand');
    if (yaw) yaw.textContent = '--';
    if (pitch) pitch.textContent = '--';
    if (hand) hand.textContent = '--';

    // Reset ball indicator
    const ballDot = document.getElementById('bb-ball-dot');
    const ballText = document.getElementById('bb-ball-text');
    if (ballDot) ballDot.className = 'bb-ball-dot no-ball';
    if (ballText) ballText.textContent = '--';

    // Reset button to Calibrate (disabled) on disconnect
    const calBtn = document.getElementById('bb-calibrate-btn');
    if (calBtn) {
        if (calBtn.classList.contains('bb-reset-mode')) {
            calBtn.textContent = 'Calibrate';
            calBtn.classList.remove('bb-reset-mode');
            calBtn.removeEventListener('click', onBBResetClick);
            calBtn.addEventListener('click', onBBCalibrateClick);
        }
        calBtn.disabled = true;
        calBtn.title = 'Ball Butler disconnected';
    }
}

// ---- BB calibration status ----

/**
 * Update Ball Butler calibration status from bb/calibration_result.
 * @param {object} msg - BallButlerCalibrationResult message
 */
export function updateBBCalibration(msg) {
    const dot = document.getElementById('bb-calib-dot');
    const text = document.getElementById('bb-calib-text');
    if (!dot) return;

    if (msg.success) {
        dot.className = 'bb-calib-dot calibrated';
        if (text) text.textContent = 'Calibrated';
    } else {
        dot.className = 'bb-calib-dot uncalibrated';
        if (text) text.textContent = 'Not Calibrated';
    }
}

/**
 * Reset the Ball Butler calibration indicator to its safe "unknown" default
 * ("Not Calibrated"). The Calibrated state is driven by a *latched event*
 * (bb/calibration_result, published by mocap_node only when a calibration is
 * performed) — nothing re-publishes it to reflect that a session ended. In a
 * long-lived GUI tab that outlives ROS relaunches, the last session's
 * "Calibrated" therefore persists into a fresh, uncalibrated session (the new
 * mocap_node has no latched sample, so updateBBCalibration is never called).
 * Calling this on the GUI↔rosbridge drop clears that stale latch: when the
 * websocket drops the whole ROS graph — including the mocap_node that owns the
 * calibration — is gone, so the calibration state is genuinely unknown. A
 * same-session network blip self-heals: on reconnect resubscribeAll() re-reads
 * the still-latched success=true and restores "Calibrated".
 */
export function resetBBCalibration() {
    const dot = document.getElementById('bb-calib-dot');
    const text = document.getElementById('bb-calib-text');
    if (dot) dot.className = 'bb-calib-dot uncalibrated';
    if (text) text.textContent = 'Not Calibrated';
}

// ---- Motion planner panel ----

let motionTimeout = null;
const MOTION_TIMEOUT_MS = 3000;

/**
 * Update motion planner panel from DiagnosticStatus message.
 * @param {object} msg - diagnostic_msgs/DiagnosticStatus
 */
export function updateMotionPanel(msg) {
    const panel = document.getElementById('panel-motion');
    if (!panel) return;

    // Reset timeout — show disconnected state if no data for 3s
    if (motionTimeout) clearTimeout(motionTimeout);
    motionTimeout = setTimeout(() => { setMotionDisconnected(); }, MOTION_TIMEOUT_MS);

    // Parse key-value pairs into a map
    const kv = {};
    for (const v of (msg.values || [])) {
        kv[v.key] = v.value;
    }

    // Status badge (level: 0=OK, 1=WARN, 2=ERROR)
    const badge = document.getElementById('motion-status-badge');
    if (badge) {
        const level = msg.level;
        if (level === 2) {
            badge.textContent = 'ERR';
            badge.className = 'badge motion-badge motion-err';
        } else if (level === 1) {
            badge.textContent = 'WARN';
            badge.className = 'badge motion-badge motion-warn';
        } else {
            badge.textContent = 'OK';
            badge.className = 'badge motion-badge motion-ok';
        }
    }

    // Trajectory state + progress
    const trajLabel = document.getElementById('motion-traj-label');
    const progressTrack = document.getElementById('motion-progress-track');
    const progressFill = document.getElementById('motion-progress-fill');
    const trajState = kv['traj_state'] || 'idle';
    if (trajLabel) trajLabel.textContent = trajState;

    const progress = parseFloat(kv['traj_progress']);
    const showProgress = (trajState === 'executing' || trajState === 'returning') && !isNaN(progress);
    if (progressTrack) progressTrack.style.display = showProgress ? '' : 'none';
    if (progressFill && showProgress) progressFill.style.width = (progress * 100) + '%';

    // Readouts
    const wsEl = document.getElementById('motion-ws');
    const condEl = document.getElementById('motion-cond');
    const scaleEl = document.getElementById('motion-scale');
    const scaleReadout = document.getElementById('motion-scale-readout');

    const ws = kv['workspace_status'] || '--';
    if (wsEl) {
        wsEl.textContent = ws;
        wsEl.style.color = ws === 'ok' ? 'var(--accent-green)'
                         : ws === 'soft' ? 'var(--accent-amber)'
                         : ws === 'hard' ? 'var(--accent-red)'
                         : 'var(--text-secondary)';
    }
    // Notify jog panel so it can clamp jogTarget at workspace boundaries
    if (ws === 'ok' || ws === 'soft' || ws === 'hard') {
        onWorkspaceStatus(ws);
    }
    if (condEl) condEl.textContent = kv['cond_number'] || '--';

    const scale = parseFloat(kv['workspace_speed_scale']);
    if (scaleReadout) scaleReadout.style.display = (!isNaN(scale) && scale < 1.0) ? '' : 'none';
    if (scaleEl && !isNaN(scale)) scaleEl.textContent = scale.toFixed(2);
}

export function setMotionDisconnected() {
    const badge = document.getElementById('motion-status-badge');
    if (badge) {
        badge.textContent = 'DISABLED';
        badge.className = 'badge motion-badge motion-off';
    }
    const trajLabel = document.getElementById('motion-traj-label');
    if (trajLabel) trajLabel.textContent = '';
    const progressTrack = document.getElementById('motion-progress-track');
    if (progressTrack) progressTrack.style.display = 'none';
}

// ---- Tracking error panel ----

/** Per-axis ring buffer of recent error magnitudes for the inline
 *  sparkline.  Sized for ~3 s of history at the 20 Hz robot_state rate
 *  the panel is fed at. */
const TRACKING_HIST_LEN = 60;
const trackingHistory = [];
const trackingSparklineCtx = [];

export function initTrackingGrid() {
    const grid = document.getElementById('tracking-grid');
    if (!grid) return;

    const labels = ['Leg 0', 'Leg 1', 'Leg 2', 'Leg 3', 'Leg 4', 'Leg 5', 'Hand'];
    for (let i = 0; i < 7; i++) {
        const col = document.createElement('div');
        col.className = 'tracking-col';
        col.innerHTML = `
            <span class="motor-label">${labels[i]}</span>
            <div class="tracking-bar">
                <div class="tracking-bar-fill" id="track-bar-${i}"></div>
            </div>
            <span class="motor-value" id="track-val-${i}">--</span>
            <canvas class="tracking-sparkline" id="track-spark-${i}"></canvas>
        `;
        grid.appendChild(col);

        trackingHistory.push(new Float32Array(TRACKING_HIST_LEN));
        trackingSparklineCtx.push(null);  // canvas ctx populated lazily
    }
}

/** Lazy canvas-context fetch + DPI-correct sizing (called from
 *  updateTrackingError on every tick — so a panel resize naturally
 *  reflows the sparkline without a separate ResizeObserver). */
function getTrackingSparklineCtx(i) {
    const canvas = document.getElementById(`track-spark-${i}`);
    if (!canvas) return null;
    const dpr = window.devicePixelRatio || 1;
    const cssW = canvas.clientWidth;
    const cssH = canvas.clientHeight;
    if (cssW === 0 || cssH === 0) return null;
    const targetW = Math.round(cssW * dpr);
    const targetH = Math.round(cssH * dpr);
    if (canvas.width !== targetW || canvas.height !== targetH) {
        canvas.width = targetW;
        canvas.height = targetH;
    }
    return canvas.getContext('2d');
}

/** Push the latest error magnitude onto the i-th history buffer and
 *  redraw the sparkline.  Threshold-aware colouring matches the bar/
 *  numeric value above it. */
function drawTrackingSparkline(i, err, threshold) {
    const buf = trackingHistory[i];
    if (!buf) return;
    // Shift left by 1 (cheap memcpy on a typed array), append the new
    // value at the tail.
    buf.copyWithin(0, 1);
    buf[TRACKING_HIST_LEN - 1] = err;

    const ctx = getTrackingSparklineCtx(i);
    if (!ctx) return;
    const w = ctx.canvas.width;
    const h = ctx.canvas.height;
    ctx.clearRect(0, 0, w, h);

    // Y-scale: clamp to 2× warn-threshold so a single huge spike doesn't
    // flatten everything else into the baseline.
    const maxErr = threshold.warn * 2;

    // Draw threshold lines (faint, behind the trace) so the user can
    // tell at a glance how close to ok/warn the recent history has been.
    ctx.strokeStyle = 'rgba(148, 163, 184, 0.35)';
    ctx.lineWidth = 1;
    const okY = h - (Math.min(threshold.ok, maxErr) / maxErr) * h * 0.92;
    ctx.beginPath();
    ctx.moveTo(0, okY);
    ctx.lineTo(w, okY);
    ctx.stroke();

    // Trace itself.  Colour follows the *current* error so the line tip
    // matches the numeric value's classification.
    let stroke = 'rgb(34, 197, 94)';   // accent-green
    if (err >= threshold.warn) stroke = 'rgb(239, 68, 68)';   // accent-red
    else if (err >= threshold.ok) stroke = 'rgb(245, 158, 11)';  // accent-amber
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 1.5 * (window.devicePixelRatio || 1);
    ctx.beginPath();
    // NaN samples (unknown ticks — commanded source stale/absent) lift the
    // pen: the trace breaks over them instead of bridging, so a stale window
    // reads as a visible hole in the history, mirroring the chart gaps.
    let penDown = false;
    for (let k = 0; k < TRACKING_HIST_LEN; k++) {
        if (!Number.isFinite(buf[k])) {
            penDown = false;
            continue;
        }
        const x = (k / (TRACKING_HIST_LEN - 1)) * w;
        const v = Math.min(buf[k], maxErr);
        const y = h - (v / maxErr) * h * 0.92;
        if (!penDown) { ctx.moveTo(x, y); penDown = true; }
        else ctx.lineTo(x, y);
    }
    ctx.stroke();
}

/**
 * Update tracking error display.
 * @param {(number|null)[]} errors - error per axis (mm for legs, rev for
 *   hand; sign ignored).  null = UNKNOWN — the commanded-side source for
 *   that axis is stale or absent (leg echo stopped, no hand telemetry).
 *   Unknown renders as the panel's '--' placeholder with an empty bar and a
 *   pen-up sparkline hole — never as 0/green, which would fake perfect
 *   tracking for an axis nobody is commanding.
 */
export function updateTrackingError(errors) {
    // Thresholds (mm for legs, rev for hand)
    const thresholds = [
        { ok: 0.5, warn: 2.0 }, // legs
        { ok: 0.05, warn: 0.2 }, // hand (rev)
    ];

    for (let i = 0; i < Math.min(errors.length, 7); i++) {
        const th = i < 6 ? thresholds[0] : thresholds[1];

        const bar = document.getElementById(`track-bar-${i}`);
        const val = document.getElementById(`track-val-${i}`);

        if (errors[i] == null || !Number.isFinite(errors[i])) {
            // Unknown — same placeholder idiom as the panel's initial state.
            // Bar collapses to 0% (colour irrelevant at zero height).
            if (bar) bar.style.height = '0%';
            if (val) {
                val.textContent = '--';
                val.className = 'motor-value';
            }
            drawTrackingSparkline(i, NaN, th);
            continue;
        }

        const err = Math.abs(errors[i]);
        const maxErr = th.warn * 2;

        if (bar) {
            const pct = Math.min(100, (err / maxErr) * 100);
            bar.style.height = pct + '%';

            if (err < th.ok) bar.style.background = 'var(--accent-green)';
            else if (err < th.warn) bar.style.background = 'var(--accent-amber)';
            else bar.style.background = 'var(--accent-red)';
        }

        if (val) {
            if (i < 6) {
                val.textContent = err.toFixed(1) + ' mm';
            } else {
                val.textContent = err.toFixed(3) + ' rev';
            }

            if (err < th.ok) val.className = 'motor-value tracking-error-ok';
            else if (err < th.warn) val.className = 'motor-value tracking-error-warn';
            else val.className = 'motor-value tracking-error-bad';
        }

        drawTrackingSparkline(i, err, th);
    }
}

// ---- Topic monitor panel ----

/**
 * Per-topic tracking data.
 * Key: topic name, Value: { type, timestamps: number[], lastTime: number }
 */
const topicData = new Map();

/** Window durations to cycle through (ms) */
const WINDOW_OPTIONS = [1000, 5000, 10000, 30000];
let currentWindowIndex = 1; // Start at 5s

/** Hidden topics — persisted to localStorage */
const HIDDEN_TOPICS_KEY = 'jugglebot-hidden-topics';
let hiddenTopics = new Set();
let showHidden = false;

function loadHiddenTopics() {
    try {
        const saved = localStorage.getItem(HIDDEN_TOPICS_KEY);
        if (saved) hiddenTopics = new Set(JSON.parse(saved));
    } catch { /* ignore */ }
}

function saveHiddenTopics() {
    localStorage.setItem(HIDDEN_TOPICS_KEY, JSON.stringify([...hiddenTopics]));
}

/** @returns {number} Current window in ms */
export function getTopicWindowMs() {
    return WINDOW_OPTIONS[currentWindowIndex];
}

export function initTopicMonitor() {
    const container = document.getElementById('topic-table-container');
    if (!container) return;

    loadHiddenTopics();
    container.innerHTML = '<div class="topic-empty">No topics</div>';
    // Single delegated click listener — innerHTML rebuilds in
    // updateTopicMonitor() swap children, not the container itself, so
    // one persistent listener handles every future row click.
    container.addEventListener('click', onTopicTableClick);

    // Click to cycle window
    const label = document.getElementById('topic-window-label');
    if (label) {
        label.addEventListener('click', () => {
            currentWindowIndex = (currentWindowIndex + 1) % WINDOW_OPTIONS.length;
            const sec = WINDOW_OPTIONS[currentWindowIndex] / 1000;
            label.textContent = sec + 's window';
        });
    }
}

/**
 * Record a message received on a topic (just the timestamp).
 * @param {string} topicName
 */
export function recordTopicMessage(topicName) {
    let entry = topicData.get(topicName);
    if (!entry) {
        entry = { type: '', timestamps: [], lastTime: 0 };
        topicData.set(topicName, entry);
    }
    const now = Date.now();
    entry.timestamps.push(now);
    entry.lastTime = now;
}

/**
 * Register a topic (from discovery) without requiring a message.
 * @param {string} topicName
 * @param {string} topicType
 */
export function registerTopic(topicName, topicType) {
    if (!topicData.has(topicName)) {
        topicData.set(topicName, { type: topicType, timestamps: [], lastTime: 0 });
    } else {
        topicData.get(topicName).type = topicType;
    }
}

/**
 * Compute rate for a topic entry.
 * @param {object} entry
 * @param {number} windowSec
 * @returns {number}
 */
function topicRate(entry, windowSec) {
    return windowSec > 0 ? entry.timestamps.length / windowSec : 0;
}

/**
 * Update the topic monitor table. Called on a 1-second timer.
 */
export function updateTopicMonitor() {
    const container = document.getElementById('topic-table-container');
    if (!container) return;

    const windowMs = WINDOW_OPTIONS[currentWindowIndex];
    const now = Date.now();

    // Prune old timestamps
    for (const entry of topicData.values()) {
        entry.timestamps = entry.timestamps.filter(t => now - t < windowMs);
    }

    if (topicData.size === 0) {
        container.innerHTML = '<div class="topic-empty">No topics</div>';
        return;
    }

    const windowSec = windowMs / 1000;

    // Separate visible and hidden topics
    const visible = [];
    const hidden = [];
    for (const [name, entry] of topicData) {
        if (hiddenTopics.has(name)) {
            hidden.push([name, entry]);
        } else {
            visible.push([name, entry]);
        }
    }

    // Sort visible by rate descending, then alphabetically for ties
    visible.sort((a, b) => {
        const rateA = topicRate(a[1], windowSec);
        const rateB = topicRate(b[1], windowSec);
        if (rateB !== rateA) return rateB - rateA;
        return a[0].localeCompare(b[0]);
    });

    // Sort hidden alphabetically
    hidden.sort((a, b) => a[0].localeCompare(b[0]));

    // Build table
    let html = `<table class="topic-table">
        <thead><tr>
            <th class="col-topic">Topic</th>
            <th class="col-last">Last</th>
            <th class="col-rate">Hz</th>
        </tr></thead><tbody>`;

    for (const [name, entry] of visible) {
        html += buildTopicRow(name, entry, windowSec, now, false);
    }

    html += '</tbody></table>';

    // Hidden topics footer
    if (hidden.length > 0) {
        html += `<div class="topic-hidden-toggle" id="topic-hidden-toggle">${hidden.length} hidden</div>`;

        if (showHidden) {
            html += `<table class="topic-table topic-table-hidden">
                <tbody>`;
            for (const [name, entry] of hidden) {
                html += buildTopicRow(name, entry, windowSec, now, true);
            }
            html += '</tbody></table>';
        }
    }

    container.innerHTML = html;
    // Click handler is registered once in initTopicMonitor — no per-tick
    // re-registration needed.
}

/**
 * Build a single topic table row.
 */
function buildTopicRow(name, entry, windowSec, now, isHidden) {
    // Last message time
    let lastStr;
    if (entry.lastTime === 0) {
        lastStr = '--';
    } else {
        const ago = (now - entry.lastTime) / 1000;
        if (ago < 10) lastStr = ago.toFixed(1) + 's';
        else if (ago < 60) lastStr = Math.round(ago) + 's';
        else if (ago < 3600) lastStr = Math.round(ago / 60) + 'm';
        else lastStr = Math.round(ago / 3600) + 'h';
    }

    // Rate
    const count = entry.timestamps.length;
    const rate = topicRate(entry, windowSec);
    let rateStr;
    let rateClass;
    if (count === 0) {
        rateStr = '--';
        rateClass = 'topic-rate-stale';
    } else if (rate >= 100) {
        rateStr = Math.round(rate).toString();
        rateClass = 'topic-rate-high';
    } else if (rate >= 10) {
        rateStr = rate.toFixed(0);
        rateClass = 'topic-rate-active';
    } else {
        rateStr = rate.toFixed(1);
        rateClass = 'topic-rate-active';
    }

    const displayName = name.startsWith('/') ? name.substring(1) : name;
    const action = isHidden ? 'unhide' : 'hide';
    const actionTitle = isHidden ? 'Click to unhide' : 'Click to hide';
    const rowClass = isHidden ? ' class="topic-row-hidden"' : '';

    return `<tr${rowClass} data-topic="${name}" data-action="${action}" title="${actionTitle}: ${name} [${entry.type}]">
        <td class="col-topic">${displayName}</td>
        <td class="col-last">${lastStr}</td>
        <td class="col-rate ${rateClass}">${rateStr}</td>
    </tr>`;
}

/**
 * Handle clicks on the topic table for hide/unhide.
 */
function onTopicTableClick(e) {
    // Toggle hidden section visibility
    const toggle = e.target.closest('#topic-hidden-toggle');
    if (toggle) {
        showHidden = !showHidden;
        return;
    }

    // Hide/unhide a topic row
    const row = e.target.closest('tr[data-topic]');
    if (row) {
        const topic = row.dataset.topic;
        const action = row.dataset.action;
        if (action === 'hide') {
            hiddenTopics.add(topic);
        } else {
            hiddenTopics.delete(topic);
        }
        saveHiddenTopics();
    }
}

/**
 * Clear all topic data (e.g. on disconnect).
 */
export function clearTopicData() {
    topicData.clear();
}

// ---- Init all panels ----

// ════════════════════════════════════════════════════════════════
// Catching Cone panel
//   cone/heartbeat      (CatchingConeHeartbeat) -> updateConeHeartbeat
//   cone/timing_result  (CatchTimingResult)     -> updateConeTimingResult
// ════════════════════════════════════════════════════════════════

// Recent matched timing offsets (ms), oldest first — drives the sound bar.
let coneDeltaHistory = [];

// Per-catch heartbeat feedback state (independent of the correlation pipeline).
// coneLastCatchSeq stays null until the first connected heartbeat is processed,
// so a pre-existing catch count (GUI connected mid-session) baselines silently
// instead of flashing a phantom 'catch' on panel init / reconnect.
let coneLastCatchSeq = null;
let coneHadAnyCatch = false;
let coneCatchFlashTimer = null;

/** Last-known Catching-Cone connected bool — drives auto expand/collapse of
 *  #panel-catching-cone on the connect/disconnect EDGE only (manual header
 *  clicks between edges are honoured). null until the first heartbeat /
 *  disconnect is processed. */
let lastCCConnected = null;

function driveCCConnectionEdge(connected) {
    if (lastCCConnected === connected) return;
    setPanelCollapsed('panel-catching-cone', !connected);  // connect → expand, disconnect → collapse
    lastCCConnected = connected;
}

function coneDeltaClass(ms) {
    const a = Math.abs(ms);
    if (a < CC_DELTA_OK_MS) return 'cc-delta-ok';
    if (a < CC_DELTA_WARN_MS) return 'cc-delta-warn';
    return 'cc-delta-bad';
}

function coneZoneColor(ms, css) {
    const a = Math.abs(ms);
    return css.getPropertyValue(
        a < CC_DELTA_OK_MS ? '--accent-green'
        : a < CC_DELTA_WARN_MS ? '--accent-amber' : '--accent-red').trim();
}

/** Format a builtin_interfaces/Time {sec, nanosec} as a HH:MM:SS.mmm clock string. */
function fmtConeClock(t) {
    if (!t || t.sec === undefined) return '--';
    const ms = t.sec * 1000 + t.nanosec / 1e6;
    const hms = new Date(ms).toLocaleTimeString('en-GB', { hour12: false });
    return hms + '.' + String(Math.floor(((ms % 1000) + 1000) % 1000)).padStart(3, '0');
}

export function initCatchingConePanel() {
    const content = document.getElementById('cc-content');
    if (!content) return;
    content.innerHTML = `
        <div class="cc-sync-row">
            <span class="cc-sync-dot offline" id="cc-sync-dot"></span>
            <span id="cc-sync-text">No time-sync</span>
            <span class="cc-sync-jitter" id="cc-sync-jitter">-- µs</span>
        </div>
        <div class="cc-footer">
            <span id="cc-last-catch">no catches yet</span>
        </div>
        <div class="cc-readouts">
            <div class="cc-readout">
                <div class="label">Predicted</div>
                <div class="value" id="cc-predicted">--</div>
            </div>
            <div class="cc-readout">
                <div class="label">Actual</div>
                <div class="value" id="cc-actual">--</div>
            </div>
            <div class="cc-readout cc-delta" id="cc-delta-cell">
                <div class="label">Delta</div>
                <div class="value" id="cc-delta">--</div>
            </div>
        </div>
        <div class="cc-soundbar-caption">Recent timing offsets</div>
        <canvas class="cc-soundbar" id="cc-soundbar"></canvas>
        <div class="cc-footer">
            <span id="cc-foot-left">no catches yet</span>
            <span id="cc-foot-right"></span>
        </div>
    `;
    window.addEventListener('resize', drawConeSoundBar);
    drawConeSoundBar();
}

export function setCatchingConeDisconnected() {
    // Auto-collapse on the disconnect edge (fires for both a cone-reported
    // offline heartbeat and a full GUI↔rosbridge drop via main.js).
    driveCCConnectionEdge(false);

    const badge = document.getElementById('cc-state-badge');
    if (badge) { badge.textContent = 'Disconnected'; badge.className = 'badge cc-state-badge cc-disconnected'; }
    const dot = document.getElementById('cc-sync-dot');
    if (dot) dot.className = 'cc-sync-dot offline';
    const text = document.getElementById('cc-sync-text');
    if (text) text.textContent = 'No time-sync';
    const jit = document.getElementById('cc-sync-jitter');
    if (jit) jit.textContent = '-- µs';
    const foot = document.getElementById('cc-foot-left');
    if (foot) foot.textContent = 'cone offline';
    // Re-baseline the heartbeat catch counter so the first heartbeat after
    // reconnect doesn't flash for catches that happened while disconnected.
    coneLastCatchSeq = null;
    coneHadAnyCatch = false;
    const lastCatch = document.getElementById('cc-last-catch');
    if (lastCatch) { lastCatch.textContent = '—'; lastCatch.style.color = ''; }
    clearTimeout(coneCatchFlashTimer);
}

/** Update connection badge + time-sync line from a CatchingConeHeartbeat. */
export function updateConeHeartbeat(hb) {
    if (!hb.connected) { setCatchingConeDisconnected(); return; }
    // Connected (synced or unsynced) — auto-expand on the connect edge.
    driveCCConnectionEdge(true);
    const badge = document.getElementById('cc-state-badge');
    const dot = document.getElementById('cc-sync-dot');
    const text = document.getElementById('cc-sync-text');
    const jit = document.getElementById('cc-sync-jitter');
    if (hb.time_synced) {
        if (badge) { badge.textContent = 'Connected'; badge.className = 'badge cc-state-badge cc-connected'; }
        if (dot) dot.className = 'cc-sync-dot synced';
        if (text) text.textContent = 'Time-synced';
        if (jit) jit.textContent = hb.sync_rms_us + ' µs';
    } else {
        if (badge) { badge.textContent = 'Unsynced'; badge.className = 'badge cc-state-badge cc-unsynced'; }
        if (dot) dot.className = 'cc-sync-dot unsynced';
        if (text) text.textContent = 'Acquiring sync…';
        if (jit) jit.textContent = '-- µs';
    }

    // Per-catch feedback straight from the heartbeat — independent of the
    // correlation pipeline (cone/timing_result). This surfaces a piezo hit even
    // when catch_correlation_node has no matching throw, and is what the operator
    // instinctively looks for on a tap.  Updates every heartbeat (10 Hz).
    const lastCatchEl = document.getElementById('cc-last-catch');
    if (lastCatchEl) {
        if (hb.have_any_catch) {
            const secs = Math.max(0, Math.round((hb.ms_since_last_catch || 0) / 1000));
            lastCatchEl.textContent = `last catch #${hb.last_catch_seq} · ${secs}s ago`;
        } else {
            lastCatchEl.textContent = 'no catches yet';
        }
        // Flash green on a genuinely NEW catch: either the first catch this
        // session (have_any_catch just went false→true) or the sequence number
        // advanced.  coneLastCatchSeq === null means this is the first heartbeat
        // after (re)connect — baseline only, never flash (guards against a
        // phantom 'catch' on panel init / for catches seen while disconnected).
        // The baseline is reset by setCatchingConeDisconnected(), which fires on
        // BOTH axes of "disconnected": a cone-reported-offline heartbeat
        // (updateConeHeartbeat above) AND a GUI↔rosbridge websocket drop
        // (main.js's connection-state 'disconnected' branch). A reconnect on
        // either axis therefore re-baselines silently — no flash for catches
        // that landed while either link was down.
        const baselined = coneLastCatchSeq !== null;
        const firstEverCatch = baselined && hb.have_any_catch && !coneHadAnyCatch;
        const seqAdvanced = baselined && hb.have_any_catch && coneHadAnyCatch
            && hb.last_catch_seq !== coneLastCatchSeq;
        if (firstEverCatch || seqAdvanced) {
            const green = getComputedStyle(document.documentElement)
                .getPropertyValue('--accent-green').trim();
            lastCatchEl.style.color = green;
            clearTimeout(coneCatchFlashTimer);
            coneCatchFlashTimer = setTimeout(() => { lastCatchEl.style.color = ''; }, 800);
        }
        coneLastCatchSeq = hb.last_catch_seq;
        coneHadAnyCatch = hb.have_any_catch;
    }
}

/** Update predicted/actual/delta readouts + sound bar from a CatchTimingResult. */
export function updateConeTimingResult(res) {
    const predEl = document.getElementById('cc-predicted');
    const actEl = document.getElementById('cc-actual');
    const deltaEl = document.getElementById('cc-delta');
    const cell = document.getElementById('cc-delta-cell');
    const footL = document.getElementById('cc-foot-left');
    const footR = document.getElementById('cc-foot-right');
    if (!deltaEl) return;

    if (actEl) actEl.textContent = fmtConeClock(res.actual_catch_time);

    if (res.matched) {
        if (predEl) predEl.textContent = fmtConeClock(res.predicted_landing_time);
        const d = res.timing_error_ms;
        const sign = d >= 0 ? '+' : '−';
        deltaEl.textContent = sign + Math.abs(d).toFixed(1) + ' ms';
        if (cell) cell.className = 'cc-readout cc-delta ' + coneDeltaClass(d);
        coneDeltaHistory.push(d);
        if (coneDeltaHistory.length > CC_OFFSET_HISTORY_LEN) coneDeltaHistory.shift();
        if (footL) footL.textContent = 'last: ' + (res.thrower_name || '?');
    } else {
        // Unmatched catch — we know when, but not which throw.
        if (predEl) predEl.textContent = '—';
        deltaEl.textContent = '—';
        if (cell) cell.className = 'cc-readout cc-delta';
        if (footL) footL.textContent = 'unmatched catch';
    }
    if (footR) footR.textContent = coneDeltaHistory.length + ' recent';
    drawConeSoundBar();
}

/** Draw recent offsets as vertical bars on a fixed -limit..0..+limit axis;
 *  newest at full opacity, older bars fading out, coloured by threshold zone. */
function drawConeSoundBar() {
    const canvas = document.getElementById('cc-soundbar');
    if (!canvas) return;
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (w === 0 || h === 0) return;
    canvas.width = w * dpr;
    canvas.height = h * dpr;
    const ctx = canvas.getContext('2d');
    ctx.scale(dpr, dpr);
    ctx.clearRect(0, 0, w, h);

    const css = getComputedStyle(document.documentElement);
    const cBorder = css.getPropertyValue('--border-color').trim();
    const cMuted = css.getPropertyValue('--text-muted').trim();
    const cGreen = css.getPropertyValue('--accent-green').trim();
    const cAmber = css.getPropertyValue('--accent-amber').trim();
    const fontSans = css.getPropertyValue('--font-sans');

    const limit = CC_OFFSET_DISPLAY_LIMIT_MS;
    const axisY = h - 20;
    const barTop = 8;
    const barBot = axisY - 3;
    const pad = 10;
    const xOf = (ms) => {
        const c = Math.max(-limit, Math.min(limit, ms));
        return w / 2 + (c / limit) * (w / 2 - pad);
    };

    // Threshold zones (faint bands behind the bars)
    ctx.fillStyle = cAmber + '14';
    ctx.fillRect(xOf(-CC_DELTA_WARN_MS), barTop,
                 xOf(CC_DELTA_WARN_MS) - xOf(-CC_DELTA_WARN_MS), barBot - barTop);
    ctx.fillStyle = cGreen + '24';
    ctx.fillRect(xOf(-CC_DELTA_OK_MS), barTop,
                 xOf(CC_DELTA_OK_MS) - xOf(-CC_DELTA_OK_MS), barBot - barTop);

    // Zero line
    ctx.strokeStyle = cMuted;
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(w / 2, barTop); ctx.lineTo(w / 2, axisY); ctx.stroke();

    // Axis line + tick labels
    ctx.strokeStyle = cBorder;
    ctx.beginPath(); ctx.moveTo(0, axisY); ctx.lineTo(w, axisY); ctx.stroke();
    ctx.fillStyle = cMuted;
    ctx.font = '10px ' + fontSans;
    ctx.textBaseline = 'top';
    ctx.textAlign = 'left';   ctx.fillText('−' + limit + ' ms', 0, axisY + 5);
    ctx.textAlign = 'center'; ctx.fillText('0', w / 2, axisY + 5);
    ctx.textAlign = 'right';  ctx.fillText('+' + limit + ' ms', w, axisY + 5);
    ctx.fillStyle = cBorder;
    ctx.font = '9px ' + fontSans;
    ctx.textAlign = 'left';   ctx.fillText('◀ early', 1, barTop - 2);
    ctx.textAlign = 'right';  ctx.fillText('late ▶', w - 1, barTop - 2);

    if (coneDeltaHistory.length === 0) {
        ctx.fillStyle = cMuted;
        ctx.font = '11px ' + fontSans;
        ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
        ctx.fillText('awaiting catch events…', w / 2, (barTop + barBot) / 2);
        return;
    }

    const n = coneDeltaHistory.length;
    coneDeltaHistory.forEach((d, i) => {
        const newest = i === n - 1;
        const recency = (i + 1) / n;
        const x = xOf(d);
        const bw = newest ? 6 : 4;
        ctx.globalAlpha = 0.16 + 0.84 * recency;
        ctx.fillStyle = coneZoneColor(d, css);
        ctx.fillRect(x - bw / 2, barTop, bw, barBot - barTop);

        // Chevron when an offset is past the display limit
        if (Math.abs(d) > limit) {
            const dir = d > 0 ? 1 : -1, cy = (barTop + barBot) / 2;
            ctx.beginPath();
            ctx.moveTo(x + dir * 5, cy - 4);
            ctx.lineTo(x + dir * 9, cy);
            ctx.lineTo(x + dir * 5, cy + 4);
            ctx.fill();
        }
        // Marker triangle above the newest bar
        if (newest) {
            ctx.beginPath();
            ctx.moveTo(x - 4, barTop - 6);
            ctx.lineTo(x + 4, barTop - 6);
            ctx.lineTo(x, barTop - 1);
            ctx.fill();
        }
    });
    ctx.globalAlpha = 1;
}

// ─────────────────────────────────────────────────────────────────────
// Throw Director panel — aim BB at a named target + throw
// ─────────────────────────────────────────────────────────────────────

// Candidate targets shown in the dropdown.  Add entries here when new
// QTM rigid bodies become throw targets.  The string must match the
// rigid_body name published by mocap_node.
const BB_THROW_TARGETS = ['Catching_Cone'];

function onBBThrowClick() {
    const sel = document.getElementById('bb-throw-target-select');
    const btn = document.getElementById('bb-throw-btn');
    const status = document.getElementById('bb-throw-status');
    if (!sel || !btn || !status) return;
    if (btn.disabled) return;

    const targetName = sel.value;
    btn.disabled = true;
    btn.textContent = 'Throwing…';
    status.textContent = `Calling bb/throw_at_target(${targetName})…`;
    status.className = 'bb-throw-status bb-throw-status-pending';

    callService('bb/throw_at_target',
                'jugglebot_interfaces/srv/BallButlerThrow',
                { target_name: targetName, throw_delay_s: 0.0 })
        .then((result) => {
            if (result.success) {
                status.textContent = result.message;
                status.className = 'bb-throw-status bb-throw-status-ok';
            } else {
                status.textContent = result.message || 'Throw failed';
                status.className = 'bb-throw-status bb-throw-status-err';
            }
        })
        .catch((err) => {
            status.textContent = 'Service error: ' + err.message;
            status.className = 'bb-throw-status bb-throw-status-err';
        })
        .finally(() => {
            setTimeout(() => {
                btn.disabled = false;
                btn.textContent = 'Throw';
            }, 800);
        });
}

export function initBBThrowPanel() {
    const content = document.getElementById('bb-throw-content');
    if (!content) return;
    const options = BB_THROW_TARGETS
        .map(t => `<option value="${t}">${t}</option>`).join('');
    content.innerHTML = `
        <div class="bb-throw-row">
            <label class="bb-throw-label" for="bb-throw-target-select">Throw target</label>
            <select class="bb-throw-select" id="bb-throw-target-select">${options}</select>
            <button class="bb-calibrate-btn" id="bb-throw-btn">Throw</button>
        </div>
        <div class="bb-throw-status" id="bb-throw-status">Ready.</div>
    `;
    const btn = document.getElementById('bb-throw-btn');
    if (btn) btn.addEventListener('click', onBBThrowClick);
}


export function initAllPanels() {
    initFlagsGrid();
    initBBPanel();
    initCatchingConePanel();
    initBBThrowPanel();
    initTrackingGrid();
    initTopicMonitor();
}
