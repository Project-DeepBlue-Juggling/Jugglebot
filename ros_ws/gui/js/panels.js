/**
 * panels.js — Right sidebar panel creation + data update functions.
 *
 * Creates DOM elements for all panels and provides update functions
 * that are called from main.js when new ROS data arrives.
 */

import { ODRIVE_STATE, BB_STATE_NAMES, LEG_STROKE_MM, MM_TO_REV } from './geometry-config.js';

// ---- Motor grid ----

const MOTOR_LABELS = ['L0', 'L1', 'L2', 'L3', 'L4', 'L5', 'Hand'];

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

const FLAGS = [
    { id: 'flag-encoder', label: 'Encoder Search' },
    { id: 'flag-homed', label: 'Homed' },
    { id: 'flag-levelled', label: 'Levelled' },
    { id: 'flag-fatal-odrive', label: 'Fatal ODrive' },
    { id: 'flag-fatal-can', label: 'Fatal CAN' },
    { id: 'flag-undervoltage', label: 'Undervoltage' },
];

export function initFlagsGrid() {
    const grid = document.getElementById('flags-grid');
    if (!grid) return;

    for (const flag of FLAGS) {
        const item = document.createElement('div');
        item.className = 'flag-item';
        item.innerHTML = `
            <span class="flag-icon wait" id="${flag.id}-icon">&#x25cf;</span>
            <span>${flag.label}</span>
        `;
        grid.appendChild(item);
    }
}

/**
 * Update system flags from robot_state.
 */
export function updateFlags(robotState) {
    setFlag('flag-encoder', robotState.encoder_search_complete);
    setFlag('flag-homed', robotState.is_homed);
    setFlag('flag-levelled', robotState.levelling_complete);
    setFlagError('flag-fatal-odrive', robotState.has_fatal_odrive_error);
    setFlagError('flag-fatal-can', robotState.has_fatal_can_error);
    setFlagError('flag-undervoltage', robotState.has_undervoltage);
}

function setFlag(id, value) {
    const icon = document.getElementById(id + '-icon');
    if (!icon) return;
    icon.className = 'flag-icon ' + (value ? 'ok' : 'wait');
    icon.innerHTML = value ? '&#x2713;' : '&#x25cf;';
}

function setFlagError(id, value) {
    const icon = document.getElementById(id + '-icon');
    if (!icon) return;
    icon.className = 'flag-icon ' + (value ? 'fail' : 'ok');
    icon.innerHTML = value ? '&#x2717;' : '&#x2713;';
}

// ---- Ball Butler panel ----

export function initBBPanel() {
    const content = document.getElementById('bb-content');
    if (!content) return;

    content.innerHTML = `
        <div style="display:flex; align-items:center; gap:8px; margin-bottom:4px;">
            <span class="bb-ball-indicator">
                <span class="bb-ball-dot no-ball" id="bb-ball-dot"></span>
                <span id="bb-ball-text">No ball</span>
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
}

export function setBBDisconnected() {
    const badge = document.getElementById('bb-state-badge');
    if (badge) {
        badge.textContent = 'Disconnected';
        badge.className = 'badge';
    }
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

export function initTrackingGrid() {
    const grid = document.getElementById('tracking-grid');
    if (!grid) return;

    const labels = ['L0', 'L1', 'L2', 'L3', 'L4', 'L5', 'Hand'];
    for (let i = 0; i < 7; i++) {
        const col = document.createElement('div');
        col.className = 'tracking-col';
        col.innerHTML = `
            <span class="motor-label">${labels[i]}</span>
            <div class="tracking-bar">
                <div class="tracking-bar-fill" id="track-bar-${i}"></div>
            </div>
            <span class="motor-value" id="track-val-${i}">--</span>
        `;
        grid.appendChild(col);
    }
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
    }
}

// ---- Init all panels ----

export function initAllPanels() {
    initMotorGrid();
    initFlagsGrid();
    initBBPanel();
    initCANPanel();
    initTrackingGrid();
}
