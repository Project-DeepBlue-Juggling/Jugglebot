/**
 * jog-panel.js — CNC-style jogging control panel for manual platform positioning.
 *
 * Provides per-axis rows of fixed-increment buttons (-10, -5, -1, -0.5, 0,
 * +0.5, +1, +5, +10) for cumulative jogging.  The centre "0" button resets
 * that axis to zero.
 *
 * Shown only when the control mode is 'GUI'; hidden otherwise.
 *
 * Pose commands are published to platform_pose_topic with publisher='GUI'.
 * The motion_bridge_node forwards these to the control loop's stream
 * smoother, which generates C2-continuous quintic profiles.
 */

import * as ros from './ros-bridge.js';
import { DEFAULT_ACTIVE_Z_MM } from './geometry-config.js';

// ---- Axis definitions ----

const AXES = [
    { id: 'x',  label: 'X',  idx: 0, type: 'trans' },
    { id: 'y',  label: 'Y',  idx: 1, type: 'trans' },
    { id: 'z',  label: 'Z',  idx: 2, type: 'trans' },
    { id: 'rx', label: 'Rx', idx: 3, type: 'rot' },
    { id: 'ry', label: 'Ry', idx: 4, type: 'rot' },
];

// Step sizes for CNC-style buttons
const TRANS_STEPS = [10, 5, 1, 0.5];       // mm (shown as -10 … -0.5, 0, +0.5 … +10)
const ROT_STEPS   = [5, 2, 1, 0.5];        // degrees

// ---- State ----

/** Current jog target pose: [x, y, z, rx, ry, rz] in mm and radians */
let jogTarget = [0, 0, 0, 0, 0, 0];

/** Whether the jog panel is currently visible (used to detect transitions). */
let jogPanelVisible = false;

/** Last jogTarget known to be within workspace limits (not hard-clamped). */
let lastSafeJogTarget = [0, 0, 0, 0, 0, 0];

/** @type {{ publish: function } | null} */
let posePublisher = null;

// ---- Rotation utilities ----

/**
 * Convert rotation vector to quaternion [w, x, y, z].
 * @param {number} rx - rotation about X (radians)
 * @param {number} ry - rotation about Y (radians)
 * @param {number} rz - rotation about Z (radians)
 * @returns {number[]} [w, x, y, z]
 */
function rotvecToQuat(rx, ry, rz) {
    const angle = Math.sqrt(rx * rx + ry * ry + rz * rz);
    if (angle < 1e-12) {
        return [1, 0, 0, 0];
    }
    const halfAngle = angle / 2;
    const s = Math.sin(halfAngle) / angle;
    return [Math.cos(halfAngle), rx * s, ry * s, rz * s];
}

// ---- Panel creation ----

/**
 * Format a step value for display on a button.
 * @param {number} value - step size
 * @param {string} type - 'trans' or 'rot'
 * @returns {string}
 */
function formatStep(value, type) {
    if (Number.isInteger(value)) return value.toString();
    return value.toFixed(1);
}

/**
 * Build one axis row: [-max … -min, LABEL, +min … +max]
 * @param {object} axis - axis definition
 * @returns {HTMLElement}
 */
function buildAxisRow(axis) {
    const row = document.createElement('div');
    row.className = 'jog-axis-row';

    const steps = axis.type === 'trans' ? TRANS_STEPS : ROT_STEPS;

    // Negative buttons (large to small: -10, -5, -1, -0.5)
    for (const step of steps) {
        const btn = document.createElement('button');
        btn.className = 'jog-btn jog-btn-neg';
        btn.textContent = '\u2212' + formatStep(step, axis.type);
        btn.addEventListener('click', () => onJogClick(axis, -step));
        row.appendChild(btn);
    }

    // Centre zero/label button — resets this axis to 0
    const zeroBtn = document.createElement('button');
    zeroBtn.className = 'jog-btn jog-btn-zero';
    zeroBtn.textContent = axis.label;
    zeroBtn.title = `Reset ${axis.label} to 0`;
    zeroBtn.addEventListener('click', () => {
        jogTarget[axis.idx] = 0;
        publishPose();
        updateReadout();
    });
    row.appendChild(zeroBtn);

    // Positive buttons (small to large: +0.5, +1, +5, +10)
    for (const step of [...steps].reverse()) {
        const btn = document.createElement('button');
        btn.className = 'jog-btn jog-btn-pos';
        btn.textContent = '+' + formatStep(step, axis.type);
        btn.addEventListener('click', () => onJogClick(axis, +step));
        row.appendChild(btn);
    }

    return row;
}

/**
 * Initialise the jog panel DOM and publisher.
 */
export function initJogPanel() {
    const panel = document.getElementById('panel-jog');
    if (!panel) return;

    // Create publisher
    posePublisher = ros.advertise(
        'platform_pose_topic',
        'jugglebot_interfaces/msg/PlatformPoseCommand');

    // Build DOM
    panel.innerHTML = `
        <div class="panel-header">
            <span class="panel-title">Jog</span>
            <button class="jog-home-btn" id="jog-home" title="Reset all axes to home">Home</button>
        </div>
        <div class="jog-grid" id="jog-grid"></div>
        <div class="jog-readout" id="jog-readout"></div>
    `;

    // Build per-axis rows
    const grid = document.getElementById('jog-grid');
    for (const axis of AXES) {
        grid.appendChild(buildAxisRow(axis));
    }

    // Home button
    document.getElementById('jog-home').addEventListener('click', () => {
        jogTarget = [0, 0, 0, 0, 0, 0];
        publishPose();
        updateReadout();
    });

    updateReadout();
}

// ---- Jog logic ----

function onJogClick(axis, step) {
    // step is in mm for translation, degrees for rotation
    const delta = axis.type === 'trans'
        ? step
        : step * Math.PI / 180;  // convert deg to rad

    jogTarget[axis.idx] += delta;
    publishPose();
    updateReadout();
}

function publishPose() {
    if (!posePublisher) return;

    const [x, y, z, rx, ry, rz] = jogTarget;
    const [qw, qx, qy, qz] = rotvecToQuat(rx, ry, rz);

    const now = Date.now();
    const secs = Math.floor(now / 1000);
    const nsecs = (now % 1000) * 1e6;

    posePublisher.publish({
        pose_stamped: {
            header: {
                stamp: { sec: secs, nanosec: nsecs },
                frame_id: 'platform_start',
            },
            pose: {
                position: { x, y, z: z + DEFAULT_ACTIVE_Z_MM },
                orientation: { w: qw, x: qx, y: qy, z: qz },
            },
        },
        publisher: 'GUI',
    });
}

function updateReadout() {
    const el = document.getElementById('jog-readout');
    if (!el) return;

    const [x, y, z, rx, ry, rz] = jogTarget;
    const rxDeg = (rx * 180 / Math.PI).toFixed(1);
    const ryDeg = (ry * 180 / Math.PI).toFixed(1);

    el.innerHTML = `
        <span title="X offset">${x.toFixed(1)}</span>
        <span title="Y offset">${y.toFixed(1)}</span>
        <span title="Z offset">${z.toFixed(1)}</span>
        <span title="Roll">${rxDeg}&deg;</span>
        <span title="Pitch">${ryDeg}&deg;</span>
    `;
}

// ---- Visibility ----

/**
 * Show or hide the jog panel.
 * @param {boolean} visible
 */
export function setJogPanelVisible(visible) {
    const panel = document.getElementById('panel-jog');
    if (panel) {
        panel.style.display = visible ? '' : 'none';
    }

    // Reset jog target only on transition into GUI mode (not on every
    // redundant control_mode_topic tick from the orchestrator at 10 Hz).
    if (visible && !jogPanelVisible) {
        jogTarget = [0, 0, 0, 0, 0, 0];
        lastSafeJogTarget = [0, 0, 0, 0, 0, 0];
        updateReadout();
    }
    jogPanelVisible = visible;
}

// ---- Workspace feedback ----

/**
 * Called from the motion diagnostics handler with the current workspace status.
 * When the control loop reports a hard limit, snaps jogTarget back to the last
 * known-good value so that one reverse step immediately starts recovery.
 * @param {string} status - 'ok', 'soft', or 'hard'
 */
export function onWorkspaceStatus(status) {
    if (status === 'hard') {
        // Revert to last safe target — further clicks in the same direction
        // will keep snapping back, but one reverse click moves immediately.
        jogTarget = [...lastSafeJogTarget];
        updateReadout();
    } else {
        lastSafeJogTarget = [...jogTarget];
    }
}

// ---- Speed limits panel ----

// Default limits matching hardware_config.py (both GUI and SpaceMouse use same defaults)
const DEFAULT_VEL_LIMIT_RPS = 5.0;
const DEFAULT_ACCEL_LIMIT_RPS2 = 6.0;

/** @type {{ publish: function } | null} */
let limitsPublisher = null;

/**
 * Initialise the speed limits panel: wire slider events and create publisher.
 */
export function initSpeedLimitsPanel() {
    limitsPublisher = ros.advertise(
        'smoother_limits', 'std_msgs/msg/Float64MultiArray');

    const velSlider = document.getElementById('speed-vel-slider');
    const accelSlider = document.getElementById('speed-accel-slider');
    const velValue = document.getElementById('speed-vel-value');
    const accelValue = document.getElementById('speed-accel-value');

    if (velSlider) {
        velSlider.value = DEFAULT_VEL_LIMIT_RPS;
        velValue.textContent = DEFAULT_VEL_LIMIT_RPS.toFixed(1);
        velSlider.addEventListener('input', () => {
            velValue.textContent = parseFloat(velSlider.value).toFixed(1);
            publishLimits();
        });
    }

    if (accelSlider) {
        accelSlider.value = DEFAULT_ACCEL_LIMIT_RPS2;
        accelValue.textContent = DEFAULT_ACCEL_LIMIT_RPS2.toFixed(1);
        accelSlider.addEventListener('input', () => {
            accelValue.textContent = parseFloat(accelSlider.value).toFixed(1);
            publishLimits();
        });
    }
}

function publishLimits() {
    if (!limitsPublisher) return;
    const vel = parseFloat(document.getElementById('speed-vel-slider')?.value || DEFAULT_VEL_LIMIT_RPS);
    const accel = parseFloat(document.getElementById('speed-accel-slider')?.value || DEFAULT_ACCEL_LIMIT_RPS2);
    limitsPublisher.publish({ data: [vel, accel] });
}

/**
 * Show or hide the speed limits panel.
 * @param {boolean} visible
 */
export function setSpeedLimitsPanelVisible(visible) {
    const panel = document.getElementById('panel-speed-limits');
    if (panel) {
        panel.style.display = visible ? '' : 'none';
    }
}

/**
 * Reset sliders to defaults for a given mode.
 * @param {string} mode - 'gui' or 'spacemouse'
 */
export function resetSpeedLimitsForMode(mode) {
    // Both modes currently share the same defaults
    const velSlider = document.getElementById('speed-vel-slider');
    const accelSlider = document.getElementById('speed-accel-slider');
    const velValue = document.getElementById('speed-vel-value');
    const accelValue = document.getElementById('speed-accel-value');

    if (velSlider) {
        velSlider.value = DEFAULT_VEL_LIMIT_RPS;
        if (velValue) velValue.textContent = DEFAULT_VEL_LIMIT_RPS.toFixed(1);
    }
    if (accelSlider) {
        accelSlider.value = DEFAULT_ACCEL_LIMIT_RPS2;
        if (accelValue) accelValue.textContent = DEFAULT_ACCEL_LIMIT_RPS2.toFixed(1);
    }
}
