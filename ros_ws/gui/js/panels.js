/**
 * panels.js — Right sidebar panel creation + data update functions.
 *
 * Creates DOM elements for all panels and provides update functions
 * that are called from main.js when new ROS data arrives.
 */

import { ODRIVE_STATE, BB_STATE_NAMES, LEG_STROKE_MM, MM_TO_REV,
         CAN_BAUD_RATE, CAN_BITS_PER_FRAME_APPROX } from './geometry-config.js';
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
        item.className = 'flag-item';
        const defaultLabel = flag.type === 'error' ? flag.labelOk : flag.labelWait;
        item.innerHTML = `
            <span class="flag-icon wait" id="${flag.id}-icon">&#x25cf;</span>
            <span id="${flag.id}-label">${defaultLabel}</span>
        `;
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
    setFlagError('flag-fatal-odrive', robotState.has_fatal_odrive_error, 'ODrive Error', 'ODrives OK');
    setFlagError('flag-fatal-can', robotState.has_fatal_can_error, 'CAN Error', 'CAN OK');
    setFlagError('flag-undervoltage', robotState.has_undervoltage, 'Undervoltage', 'Voltage OK');
}

/**
 * Update the mocap connected flag.
 * @param {boolean} connected
 */
export function setMocapConnected(connected) {
    setFlagError('flag-mocap', !connected, 'Mocap Disconnected', 'Mocap Connected');
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

// ---- Levelling panel ----

export function initLevellingPanel() {
    // DOM is pre-built in index.html (inline layout)
}

/**
 * Update levelling panel from robot_state.
 * @param {object} robotState - RobotState message
 */
export function updateLevellingPanel(robotState) {
    const badge = document.getElementById('level-status-badge');
    const rollEl = document.getElementById('level-roll');
    const pitchEl = document.getElementById('level-pitch');

    const complete = robotState.levelling_complete;
    const offset = robotState.pose_offset_rad || [];

    if (badge) {
        if (complete) {
            badge.textContent = 'Levelled';
            badge.className = 'badge level-badge level-ok';
        } else {
            badge.textContent = 'Not Levelled';
            badge.className = 'badge level-badge level-pending';
        }
    }

    if (offset.length >= 2) {
        const rollDeg = (offset[0] * 180 / Math.PI).toFixed(2);
        const pitchDeg = (offset[1] * 180 / Math.PI).toFixed(2);
        if (rollEl) rollEl.textContent = rollDeg + '\u00b0';
        if (pitchEl) pitchEl.textContent = pitchDeg + '\u00b0';
    }
}

// ---- Ball Butler panel ----

/** Last known BB state code (from heartbeat). */
let lastBBState = -1;

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
    if (!hb.connected) {
        setBBDisconnected();
        return;
    }

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
        } else if (!isError && wasReset) {
            // Switch back to Calibrate button
            calBtn.textContent = 'Calibrate';
            calBtn.classList.remove('bb-reset-mode');
            calBtn.removeEventListener('click', onBBResetClick);
            calBtn.addEventListener('click', onBBCalibrateClick);
            calBtn.disabled = hb.state !== 1;
        } else if (!isError && calBtn.textContent === 'Calibrate') {
            calBtn.disabled = hb.state !== 1;
        }
    }
}

export function setBBDisconnected() {
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

// ---- CAN traffic panel ----

const CAN_HISTORY_LEN = 20; // 10 seconds at ~2Hz reporting
const canHistory = [];
let canSparklineCtx = null;

export function initCANPanel() {
    const canvas = document.getElementById('can-sparkline');
    if (canvas) {
        canSparklineCtx = canvas.getContext('2d');
    }
}

/**
 * Update CAN traffic panel.
 * @param {object} msg - CanTrafficReportMessage
 */
export function updateCANTraffic(msg) {
    const rate = msg.report_interval > 0
        ? (msg.received_count / (msg.report_interval / 1000))
        : 0;

    const rateEl = document.getElementById('can-rate-value');
    if (rateEl) rateEl.textContent = Math.round(rate);

    // Bits/s and bus utilization
    const bitsPerSec = rate * CAN_BITS_PER_FRAME_APPROX;
    const kbitsPerSec = bitsPerSec / 1000;
    const utilPct = (bitsPerSec / CAN_BAUD_RATE) * 100;

    const bitsEl = document.getElementById('can-bits-value');
    if (bitsEl) bitsEl.textContent = kbitsPerSec.toFixed(1);

    const utilEl = document.getElementById('can-util-value');
    if (utilEl) utilEl.textContent = `(${utilPct.toFixed(1)}%)`;

    canHistory.push(rate);
    if (canHistory.length > CAN_HISTORY_LEN) canHistory.shift();

    drawSparkline();
}

function drawSparkline() {
    if (!canSparklineCtx) return;
    const canvas = canSparklineCtx.canvas;
    const w = canvas.width = canvas.clientWidth * window.devicePixelRatio;
    const h = canvas.height = canvas.clientHeight * window.devicePixelRatio;
    canSparklineCtx.clearRect(0, 0, w, h);

    if (canHistory.length < 2) return;

    const maxRate = Math.max(1, ...canHistory);
    canSparklineCtx.beginPath();
    canSparklineCtx.strokeStyle = '#3b82f6';
    canSparklineCtx.lineWidth = 2 * window.devicePixelRatio;

    for (let i = 0; i < canHistory.length; i++) {
        const x = (i / (CAN_HISTORY_LEN - 1)) * w;
        const y = h - (canHistory[i] / maxRate) * h * 0.9;
        if (i === 0) canSparklineCtx.moveTo(x, y);
        else canSparklineCtx.lineTo(x, y);
    }
    canSparklineCtx.stroke();
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
    for (let k = 0; k < TRACKING_HIST_LEN; k++) {
        const x = (k / (TRACKING_HIST_LEN - 1)) * w;
        const v = Math.min(buf[k], maxErr);
        const y = h - (v / maxErr) * h * 0.92;
        if (k === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    }
    ctx.stroke();
}

/**
 * Update tracking error display.
 * @param {number[]} errors - absolute error per axis (mm for legs, rev for hand)
 */
export function updateTrackingError(errors) {
    // Thresholds (mm for legs, rev for hand)
    const thresholds = [
        { ok: 0.5, warn: 2.0 }, // legs
        { ok: 0.05, warn: 0.2 }, // hand (rev)
    ];

    for (let i = 0; i < Math.min(errors.length, 7); i++) {
        const err = Math.abs(errors[i]);
        const th = i < 6 ? thresholds[0] : thresholds[1];
        const maxErr = th.warn * 2;

        const bar = document.getElementById(`track-bar-${i}`);
        const val = document.getElementById(`track-val-${i}`);

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

    // Wire up hide/unhide click handlers (event delegation)
    container.addEventListener('click', onTopicTableClick, { once: true });
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

export function initAllPanels() {
    initFlagsGrid();
    initLevellingPanel();
    initBBPanel();
    initCANPanel();
    initTrackingGrid();
    initTopicMonitor();
}
