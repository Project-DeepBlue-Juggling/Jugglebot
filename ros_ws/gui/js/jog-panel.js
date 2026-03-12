/**
 * jog-panel.js — Jogging control panel for manual platform positioning.
 *
 * Provides +/- buttons for 5 DoF (X, Y, Z, Roll, Pitch) with adjustable
 * step sizes. Publishes PlatformPoseCommand messages via ROSLIB.
 *
 * Shown only when the control mode is 'GUI'; hidden otherwise.
 *
 * Pose commands are published to platform_pose_topic with publisher='GUI'.
 * The motion_bridge_node forwards these to the control loop's stream
 * smoother, which generates C2-continuous quintic profiles.  sp_ik.py is
 * gated to ignore GUI mode, preventing dual-publisher conflicts.
 */

import * as ros from './ros-bridge.js';
import { DEFAULT_ACTIVE_Z_MM } from './geometry-config.js';

// ---- Configuration ----

const TRANS_STEP_MIN = 1;    // mm
const TRANS_STEP_MAX = 50;   // mm
const TRANS_STEP_DEFAULT = 5;

const ROT_STEP_MIN = 0.1;   // degrees
const ROT_STEP_MAX = 5.0;   // degrees
const ROT_STEP_DEFAULT = 0.5;

const STORAGE_KEY_TRANS = 'jugglebot-jog-trans-step';
const STORAGE_KEY_ROT = 'jugglebot-jog-rot-step';

// ---- Axis definitions ----

const AXES = [
    { id: 'x',  label: 'X',  idx: 0, type: 'trans' },
    { id: 'y',  label: 'Y',  idx: 1, type: 'trans' },
    { id: 'z',  label: 'Z',  idx: 2, type: 'trans' },
    { id: 'rx', label: 'Rx', idx: 3, type: 'rot' },
    { id: 'ry', label: 'Ry', idx: 4, type: 'rot' },
];

// ---- State ----

/** Current jog target pose: [x, y, z, rx, ry, rz] in mm and radians */
let jogTarget = [0, 0, 0, 0, 0, 0];

let transStep = TRANS_STEP_DEFAULT;
let rotStep = ROT_STEP_DEFAULT;

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
 * Initialise the jog panel DOM and publisher.
 */
export function initJogPanel() {
    const panel = document.getElementById('panel-jog');
    if (!panel) return;

    // Restore saved step sizes
    const savedTrans = localStorage.getItem(STORAGE_KEY_TRANS);
    if (savedTrans) transStep = parseFloat(savedTrans) || TRANS_STEP_DEFAULT;
    const savedRot = localStorage.getItem(STORAGE_KEY_ROT);
    if (savedRot) rotStep = parseFloat(savedRot) || ROT_STEP_DEFAULT;

    // Create publisher
    posePublisher = ros.advertise(
        'platform_pose_topic',
        'jugglebot_interfaces/msg/PlatformPoseCommand');

    // Build DOM
    panel.innerHTML = `
        <div class="panel-header">
            <span class="panel-title">Jog</span>
            <button class="jog-home-btn" id="jog-home" title="Reset jog target to home">Home</button>
        </div>
        <div class="jog-sliders">
            <div class="jog-slider-row">
                <label class="jog-slider-label">Translation</label>
                <input type="range" class="jog-range" id="jog-trans-slider"
                    min="${TRANS_STEP_MIN}" max="${TRANS_STEP_MAX}" step="1"
                    value="${transStep}">
                <span class="jog-step-value" id="jog-trans-value">${transStep.toFixed(0)} mm</span>
            </div>
            <div class="jog-slider-row">
                <label class="jog-slider-label">Rotation</label>
                <input type="range" class="jog-range" id="jog-rot-slider"
                    min="${ROT_STEP_MIN}" max="${ROT_STEP_MAX}" step="0.1"
                    value="${rotStep}">
                <span class="jog-step-value" id="jog-rot-value">${rotStep.toFixed(1)}&deg;</span>
            </div>
        </div>
        <div class="jog-buttons" id="jog-buttons"></div>
        <div class="jog-readout" id="jog-readout"></div>
    `;

    // Build jog buttons
    const btnContainer = document.getElementById('jog-buttons');
    for (const sign of [+1, -1]) {
        for (const axis of AXES) {
            const btn = document.createElement('button');
            btn.className = 'jog-btn';
            btn.textContent = (sign > 0 ? '+' : '\u2212') + axis.label;
            btn.dataset.axis = axis.id;
            btn.dataset.sign = sign;
            btn.addEventListener('click', () => onJogClick(axis, sign));
            btnContainer.appendChild(btn);
        }
    }

    // Slider events
    const transSlider = document.getElementById('jog-trans-slider');
    const rotSlider = document.getElementById('jog-rot-slider');
    const transValue = document.getElementById('jog-trans-value');
    const rotValue = document.getElementById('jog-rot-value');

    transSlider.addEventListener('input', () => {
        transStep = parseFloat(transSlider.value);
        transValue.textContent = transStep.toFixed(0) + ' mm';
        localStorage.setItem(STORAGE_KEY_TRANS, transStep);
    });

    rotSlider.addEventListener('input', () => {
        rotStep = parseFloat(rotSlider.value);
        rotValue.textContent = rotStep.toFixed(1) + '\u00b0';
        localStorage.setItem(STORAGE_KEY_ROT, rotStep);
    });

    // Home button
    document.getElementById('jog-home').addEventListener('click', () => {
        jogTarget = [0, 0, 0, 0, 0, 0];
        publishPose();
        updateReadout();
    });

    updateReadout();
}

// ---- Jog logic ----

function onJogClick(axis, sign) {
    const step = axis.type === 'trans'
        ? transStep * sign
        : (rotStep * Math.PI / 180) * sign;  // convert deg to rad

    jogTarget[axis.idx] += step;
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

    // Reset jog target when panel becomes visible (entering GUI mode)
    if (visible) {
        jogTarget = [0, 0, 0, 0, 0, 0];
        updateReadout();
    }
}
