/**
 * main.js — Entry point: init ROS, viewer, panels, wire callbacks.
 *
 * Wires together ros-bridge, the 3D viewer, status panels, and command controls.
 * Subscribes to all ROS topics and routes data to the appropriate modules.
 */

import * as ros from './ros-bridge.js';
import { initViewer, sceneGroups, registerPickables, onMeshPick } from './viewer.js';
import {
    initStewartModel, updateStewartPose, setLegFault, setHandFault,
    getStewartPickables,
} from './stewart-model.js';
import { legLengthsToPose } from './stewart-fk.js';
import { initMocapMarkers, updateMocapMarkers, updateRigidBodyAxes } from './mocap-markers.js';
import {
    initBallButlerModel, updateBallButler, updateBallButlerPose,
    setBBPitchFault, setBBHandFault, getBallButlerPickables,
} from './ball-butler-model.js';
import {
    initAllPanels, updateMotorGrid, updateOrchestratorState,
    updateFlags, updateLevellingPanel, updateBBPanel, updateBBCalibration,
    updateCANTraffic, updateTrackingError, updateMotionPanel,
    recordTopicMessage, registerTopic, updateTopicMonitor, clearTopicData,
    setMocapConnected, setMocapAligned,
    updateConeHeartbeat, updateConeTimingResult,
} from './panels.js';
import { initCommands, updateCommandStates, onModeButtonClick } from './commands.js';
import { INITIAL_HEIGHT_MM, MM_TO_REV } from './geometry-config.js';
import {
    initTelemetryCharts, onTelemetryData, rebuildCharts, flashChart,
} from './telemetry-charts.js';
import { initJogPanel, setJogPanelVisible,
         initSpeedLimitsPanel, setSpeedLimitsPanelVisible, resetSpeedLimitsForMode,
} from './jog-panel.js';
import { initTheme } from './theme.js';
import { emitEvent, EVENT_TYPES } from './event-store.js';
import { initCommandHistory } from './command-history.js';
import { initCameraPresets } from './camera-presets.js';

// ---- Latest data stores ----
let latestCommandedLegs = null;  // Float64MultiArray data (revs)

/** Human-readable labels for the fault flags we surface as event markers. */
const FAULT_LABELS = {
    has_fatal_odrive_error: 'ODrive Error',
    has_fatal_can_error:    'CAN Error',
    has_undervoltage:       'Undervoltage',
};

// ---- Initialisation ----

function init() {
    // 1. Init 3D viewer
    const container = document.getElementById('viewer-container');
    initViewer(container);
    initStewartModel();
    initBallButlerModel();
    initMocapMarkers();

    // Register pickable meshes + wire the pick-to-flash bridge.  Models
    // init before this so their pickable lists are populated.
    registerPickables(getStewartPickables());
    registerPickables(getBallButlerPickables());
    onMeshPick((chartIdx) => flashChart(chartIdx));

    // 2. Init panels
    initAllPanels();

    // 3. Init commands
    initCommands();
    onModeButtonClick((mode) => {
        setJogPanelVisible(mode === 'gui');
        setSpeedLimitsPanelVisible(mode === 'gui' || mode === 'spacemouse');
        if (mode === 'gui' || mode === 'spacemouse') {
            resetSpeedLimitsForMode(mode);
        }
    });

    // 4. Init scene menu
    initSceneMenu();

    // 5. Init resize handles + font size
    initLeftResizeHandle();
    initResizeHandle();
    initChartResizeHandle();
    initFontSize();

    // 6. Init telemetry charts
    initTelemetryCharts();

    // 6a. Init jog panel + speed limits
    initJogPanel();
    initSpeedLimitsPanel();

    // 6b. Init theme toggle (runs after charts so the first rebuild picks
    //     up the saved palette without an unnecessary re-render).
    initTheme();

    // 6c. Init command / event history panel.
    initCommandHistory();

    // 7. Init ROS connection
    ros.onConnectionStateChange(onConnectionStateChange);
    ros.init();

    // 8. Subscribe to topics
    subscribeAll();

    // 9. Start topic monitor timers
    setInterval(updateTopicMonitor, 1000);

    // 10. Connection-quality indicator: 4 Hz tick on the robot_state age
    setInterval(updateConnectionQuality, 250);
}

// ---- Connection state UI ----

/**
 * 4 Hz update of the connection-quality badge.  The badge surfaces how
 * recently a robot_state landed — a useful proxy for end-to-end link
 * health (rosbridge → ROS2 → CAN node → us).  Thresholds:
 *   <  300 ms → "OK"     (green) — tracking the 100 Hz publisher fine
 *   <  1.0 s  → "Slow"   (amber) — stalls / GC pauses / WiFi blip
 *   ≥  1.0 s  → "Stale"  (red)   — link is effectively dead
 */
function updateConnectionQuality() {
    const el = document.getElementById('conn-quality');
    if (!el) return;
    if (lastRobotStateMs === 0) {
        // Before any robot_state has arrived — hide the badge so we don't
        // claim "Stale" on a clean connect just because the publisher is
        // ramping up.
        el.textContent = '';
        el.classList.remove('ok', 'slow', 'stale');
        return;
    }
    const ageMs = Date.now() - lastRobotStateMs;
    let label, klass;
    if (ageMs < 300)        { label = 'OK';    klass = 'ok'; }
    else if (ageMs < 1000)  { label = 'Slow';  klass = 'slow'; }
    else                    { label = 'Stale'; klass = 'stale'; }
    // Bake the age into the visible text — the parent #connection-status
    // has pointer-events: none, which can swallow the native title tooltip
    // depending on browser, so we don't rely on hover for the number.
    const ageStr = ageMs < 1000
        ? `${ageMs} ms`
        : `${(ageMs / 1000).toFixed(1)} s`;
    el.textContent = `${label} \u00b7 ${ageStr}`;
    el.classList.remove('ok', 'slow', 'stale');
    el.classList.add(klass);
}

function onConnectionStateChange(state) {
    const dot = document.getElementById('conn-dot');
    const text = document.getElementById('conn-text');
    if (!dot || !text) return;

    switch (state) {
        case 'connected':
            dot.className = 'status-dot connected';
            text.textContent = 'Connected';
            startTopicDiscovery();
            break;
        case 'connecting':
            dot.className = 'status-dot disconnected';
            text.textContent = 'Connecting...';
            break;
        case 'disconnected':
            dot.className = 'status-dot disconnected';
            text.textContent = 'Disconnected';
            // Drop the freshness latch so we don't claim "Stale" against
            // the *previous* session's clock when reconnect happens.
            lastRobotStateMs = 0;
            // Drop the state-transition latch too — any state change that
            // happened during disconnect shouldn't be emitted as a fake
            // event with the reconnect timestamp.
            lastOrchestratorState = null;
            setJogPanelVisible(false);
            setSpeedLimitsPanelVisible(false);
            stopTopicDiscovery();
            break;
    }
}

// ---- ROS subscriptions ----

function subscribeAll() {
    // Robot state (100Hz -> throttle to 20Hz = 50ms)
    ros.subscribe('robot_state', 'jugglebot_interfaces/msg/RobotState', onRobotState, 50);

    // BB heartbeat (10Hz, no throttle)
    ros.subscribe('bb/heartbeat', 'jugglebot_interfaces/msg/BallButlerHeartbeat', onBBHeartbeat, 0);

    // Catching cone heartbeat (10Hz) + timing results (per catch)
    ros.subscribe('cone/heartbeat', 'jugglebot_interfaces/msg/CatchingConeHeartbeat', onConeHeartbeat, 0);
    ros.subscribe('cone/timing_result', 'jugglebot_interfaces/msg/CatchTimingResult', onConeTimingResult, 0);

    // Orchestrator state (on change)
    ros.subscribe('orchestrator_state', 'std_msgs/msg/String', onOrchestratorState, 0);

    // CAN traffic (~2Hz)
    ros.subscribe('can_traffic', 'jugglebot_interfaces/msg/CanTrafficReportMessage', onCANTraffic, 0);

    // Hand telemetry (500Hz -> throttle to 10Hz = 100ms)
    ros.subscribe('hand_telemetry', 'jugglebot_interfaces/msg/HandTelemetryMessage', onHandTelemetry, 100);

    // Mocap data — markers + connection/alignment status (200Hz -> throttle to 20Hz = 50ms)
    ros.subscribe('mocap_data', 'jugglebot_interfaces/msg/MocapDataMulti', onMocapData, 50);

    // Rigid body poses — for rendering coordinate axes (200Hz -> throttle to 20Hz = 50ms)
    ros.subscribe('rigid_body_poses', 'jugglebot_interfaces/msg/RigidBodyPoses', onRigidBodyPoses, 50);

    // Commanded leg lengths (500Hz -> throttle to 20Hz = 50ms)
    ros.subscribe('leg_lengths_topic', 'std_msgs/msg/Float64MultiArray', onLegLengths, 50);

    // Control mode (on change) — used to show/hide jog panel
    ros.subscribe('control_mode_topic', 'std_msgs/msg/String', onControlMode, 0);

    // Motion planner diagnostics (500Hz -> throttle to 5Hz = 200ms)
    ros.subscribe('motion/diagnostics', 'diagnostic_msgs/msg/DiagnosticStatus', onMotionDiagnostics, 200);

    // BB calibration result (latched — last value available to late subscribers)
    ros.subscribe('bb/calibration_result', 'jugglebot_interfaces/msg/BallButlerCalibrationResult', onBBCalibrationResult, 0);
}

// ---- Topic handlers ----

// Mocap connection status
let mocapConnTimeout = null;
const MOCAP_CONN_TIMEOUT_MS = 2000;

/** Wall-clock ms of the most recent robot_state we received.  Drives the
 *  connection-quality indicator next to the connection-status dot. */
let lastRobotStateMs = 0;

/** Last-seen fault flags — we emit a fault event only on false→true edges. */
const lastFaultFlags = {
    has_fatal_odrive_error: null,
    has_fatal_can_error: null,
    has_undervoltage: null,
};

function onRobotState(msg) {
    recordTopicMessage('robot_state');
    lastRobotStateMs = Date.now();
    const motors = msg.motor_states || [];

    // Update panels
    updateMotorGrid(motors);
    updateFlags(msg);
    updateLevellingPanel(msg);

    // Fault-transition events (false → true only; we don't log clears as
    // "events" to keep the marker forest readable, though we do reset the
    // latch on true → false so the next fault re-triggers an event).
    for (const key of Object.keys(lastFaultFlags)) {
        const now = !!msg[key];
        const prev = lastFaultFlags[key];
        if (prev === false && now === true) {
            emitEvent({
                type: EVENT_TYPES.FAULT,
                label: FAULT_LABELS[key] || key,
                detail: `Fault rising edge on ${key}`,
            });
        }
        lastFaultFlags[key] = now;
    }

    // Per-motor fault viz — red pulse on rising edge, steady red while the
    // fault persists.  A motor is faulted when either the ODrive errors or
    // the disarm reason bitfields are non-zero.
    for (let i = 0; i < Math.min(motors.length, 9); i++) {
        const m = motors[i];
        const faulted = (m.active_errors !== 0) || (m.disarm_reason !== 0);
        if (i <= 5)      setLegFault(i, faulted);
        else if (i === 6) setHandFault(faulted);
        else if (i === 7) setBBPitchFault(faulted);
        else if (i === 8) setBBHandFault(faulted);
    }

    // Feed telemetry charts
    onTelemetryData(motors, latestCommandedLegs, latestHandTelemetry);

    // Update 3D model via FK from motor feedback (always)
    if (motors.length >= 6) {
        const motorRevs = motors.slice(0, 6).map(m => m.pos_estimate);
        const result = legLengthsToPose(motorRevs);

        if (result.converged) {
            const platCentre = [
                result.pos[0],
                result.pos[1],
                result.pos[2] + INITIAL_HEIGHT_MM,
            ];

            // Hand extension: convert motor rev to mm
            let handExtMM = 0;
            if (motors.length >= 7) {
                handExtMM = motors[6].pos_estimate / MM_TO_REV[0]; // approximate
            }

            updateStewartPose(result.platNodes, platCentre, handExtMM);
        }
    }

    // Update tracking error if we have both commanded and measured
    if (latestCommandedLegs && motors.length >= 6) {
        const errors = [];
        for (let i = 0; i < 6; i++) {
            const cmdRev = latestCommandedLegs[i] || 0;
            const measRev = motors[i].pos_estimate;
            const errorRev = cmdRev - measRev;
            const errorMM = errorRev / MM_TO_REV[i];
            errors.push(errorMM);
        }

        // Hand error in revs
        if (motors.length >= 7 && latestCommandedLegs.length >= 7) {
            errors.push((latestCommandedLegs[6] || 0) - motors[6].pos_estimate);
        } else {
            errors.push(0);
        }
        updateTrackingError(errors);
    }
}

function onBBHeartbeat(msg) {
    recordTopicMessage('bb/heartbeat');
    updateBBPanel(msg);
    if (msg.connected) {
        updateBallButler(msg.yaw_deg, msg.pitch_deg, msg.hand_pos_mm);
    }
}

function onConeHeartbeat(msg) {
    recordTopicMessage('cone/heartbeat');
    updateConeHeartbeat(msg);
}

function onConeTimingResult(msg) {
    recordTopicMessage('cone/timing_result');
    updateConeTimingResult(msg);
}

/** Previous orchestrator_state payload — used to detect transitions so we
 *  only emit an event on change (the topic re-publishes continuously). */
let lastOrchestratorState = null;

function onOrchestratorState(msg) {
    recordTopicMessage('orchestrator_state');
    updateOrchestratorState(msg.data);
    updateCommandStates();

    if (msg.data !== lastOrchestratorState) {
        if (lastOrchestratorState !== null) {
            emitEvent({
                type: EVENT_TYPES.STATE,
                label: msg.data,
                detail: `State: ${lastOrchestratorState} \u2192 ${msg.data}`,
            });
        }
        lastOrchestratorState = msg.data;
    }

    // Show/hide jog + speed limits panels based on sub-mode (backup for control_mode_topic)
    const parts = msg.data.split(':');
    const sub = (parts[1] || '').toUpperCase();
    setJogPanelVisible(sub === 'GUI');
    setSpeedLimitsPanelVisible(sub === 'GUI' || sub === 'SPACEMOUSE');
}

function onCANTraffic(msg) {
    recordTopicMessage('can_traffic');
    updateCANTraffic(msg);
}

let latestHandTelemetry = null;

function onHandTelemetry(msg) {
    recordTopicMessage('hand_telemetry');
    latestHandTelemetry = msg;
}

function onMocapData(msg) {
    recordTopicMessage('mocap_data');
    // mocap_data arriving means QTM connection is live
    setMocapConnected(true);
    if (mocapConnTimeout) clearTimeout(mocapConnTimeout);
    mocapConnTimeout = setTimeout(() => {
        setMocapConnected(false);
        setMocapAligned(false);
        updateMocapMarkers([]); // clear markers when disconnected
    }, MOCAP_CONN_TIMEOUT_MS);

    // Alignment flag from the message
    setMocapAligned(!!msg.aligned);

    // Render markers in 3D viewer
    const markers = msg.markers || [];
    updateMocapMarkers(markers);
}

function onRigidBodyPoses(msg) {
    recordTopicMessage('rigid_body_poses');
    const bodies = msg.bodies || [];
    updateRigidBodyAxes(bodies);

    // Align Ball Butler 3D model with its mocap rigid body pose
    const bbBody = bodies.find(b => b.name === 'Ball_Butler');
    if (bbBody && bbBody.pose) {
        const poseStamped = bbBody.pose;
        const pose = poseStamped.pose || poseStamped;
        if (pose.position && pose.orientation) {
            updateBallButlerPose(pose.position, pose.orientation);
        }
    }
}

function onLegLengths(msg) {
    recordTopicMessage('leg_lengths_topic');
    latestCommandedLegs = msg.data;
}

function onControlMode(msg) {
    recordTopicMessage('control_mode_topic');
    const mode = msg.data;
    setJogPanelVisible(mode === 'GUI');
    setSpeedLimitsPanelVisible(mode === 'GUI' || mode === 'SPACEMOUSE');
}

function onMotionDiagnostics(msg) {
    recordTopicMessage('motion/diagnostics');
    updateMotionPanel(msg);
}

/** bb/calibration_result is latched — the first message that lands in a
 *  short window after GUI init is the stale history record and is dropped
 *  so reloading doesn't plant a phantom marker.  Any message after the
 *  window is a genuine live publish and is emitted as an event.  Message
 *  type has no header.stamp, so we can't gate on message age directly. */
const BB_CALIBRATION_STALE_WINDOW_MS = 1000;
const guiInitTimeMs = Date.now();
let bbCalibrationInitialSkipped = false;

function onBBCalibrationResult(msg) {
    recordTopicMessage('bb/calibration_result');
    updateBBCalibration(msg);
    // Skip the latched-stale message during the wall-clock init window OR
    // while no real telemetry has arrived yet — `lastRobotStateMs === 0`
    // catches slow-handshake cases where the latched message lands after
    // the 1 s window but before the link is genuinely live.
    if (!bbCalibrationInitialSkipped &&
        ((Date.now() - guiInitTimeMs) < BB_CALIBRATION_STALE_WINDOW_MS ||
         lastRobotStateMs === 0)) {
        bbCalibrationInitialSkipped = true;
        return;
    }
    bbCalibrationInitialSkipped = true;
    const success = msg && msg.success !== false;
    emitEvent({
        type: EVENT_TYPES.CALIBRATION,
        label: success ? 'BB calibrated' : 'BB calibration failed',
        detail: msg && msg.message ? String(msg.message) : '',
    });
}

// ---- Scene menu ----

function initSceneMenu() {
    const toggle = document.getElementById('scene-menu-toggle');
    const dropdown = document.getElementById('scene-menu-dropdown');
    if (!toggle || !dropdown) return;

    toggle.addEventListener('click', () => {
        dropdown.classList.toggle('open');
    });

    // Close on click outside
    document.addEventListener('click', (e) => {
        if (!e.target.closest('#scene-menu')) {
            dropdown.classList.remove('open');
        }
    });

    // Populate after a short delay to let groups register
    setTimeout(() => {
        dropdown.innerHTML = '';
        for (const [name, obj] of Object.entries(sceneGroups)) {
            const label = document.createElement('label');
            label.className = 'scene-toggle';

            const checkbox = document.createElement('input');
            checkbox.type = 'checkbox';
            checkbox.checked = obj.visible;
            checkbox.addEventListener('change', () => {
                obj.visible = checkbox.checked;
            });

            label.appendChild(checkbox);
            label.appendChild(document.createTextNode(name));
            dropdown.appendChild(label);
        }
        // Camera presets append below the scene-group toggles — run here
        // so they aren't wiped by the `dropdown.innerHTML = ''` above.
        initCameraPresets();
    }, 100);
}

// ---- Left sidebar resize handle ----

const LEFT_SIDEBAR_MIN = 200;
const LEFT_SIDEBAR_MAX_PCT = 0.3;
const LEFT_SIDEBAR_STORAGE_KEY = 'jugglebot-left-sidebar-width';

function initLeftResizeHandle() {
    const handle = document.getElementById('left-resize-handle');
    const app = document.getElementById('app');
    if (!handle || !app) return;

    // Restore saved width
    const saved = localStorage.getItem(LEFT_SIDEBAR_STORAGE_KEY);
    if (saved) {
        const w = parseInt(saved, 10);
        if (w >= LEFT_SIDEBAR_MIN) {
            app.style.setProperty('--left-sidebar-width', w + 'px');
        }
    }

    let dragging = false;

    function onPointerDown(e) {
        dragging = true;
        handle.classList.add('active');
        handle.setPointerCapture(e.pointerId);
        document.body.style.cursor = 'col-resize';
        document.body.style.userSelect = 'none';
        e.preventDefault();
    }

    function onPointerMove(e) {
        if (!dragging) return;
        const maxWidth = Math.floor(window.innerWidth * LEFT_SIDEBAR_MAX_PCT);
        let newWidth = e.clientX;
        newWidth = Math.max(LEFT_SIDEBAR_MIN, Math.min(maxWidth, newWidth));
        app.style.setProperty('--left-sidebar-width', newWidth + 'px');
    }

    function onPointerUp() {
        if (!dragging) return;
        dragging = false;
        handle.classList.remove('active');
        document.body.style.cursor = '';
        document.body.style.userSelect = '';

        // Persist
        const current = getComputedStyle(app).getPropertyValue('--left-sidebar-width').trim();
        localStorage.setItem(LEFT_SIDEBAR_STORAGE_KEY, parseInt(current, 10));
    }

    handle.addEventListener('pointerdown', onPointerDown);
    document.addEventListener('pointermove', onPointerMove);
    document.addEventListener('pointerup', onPointerUp);
}

// ---- Right sidebar resize handle ----

const SIDEBAR_MIN = 280;
const SIDEBAR_STORAGE_KEY = 'jugglebot-sidebar-width';

function initResizeHandle() {
    const handle = document.getElementById('resize-handle');
    const app = document.getElementById('app');
    if (!handle || !app) return;

    // Restore saved width
    const saved = localStorage.getItem(SIDEBAR_STORAGE_KEY);
    if (saved) {
        const w = parseInt(saved, 10);
        if (w >= SIDEBAR_MIN) {
            app.style.setProperty('--sidebar-width', w + 'px');
        }
    }

    let dragging = false;

    function onPointerDown(e) {
        dragging = true;
        handle.classList.add('active');
        handle.setPointerCapture(e.pointerId);
        document.body.style.cursor = 'col-resize';
        document.body.style.userSelect = 'none';
        e.preventDefault();
    }

    function onPointerMove(e) {
        if (!dragging) return;
        const maxWidth = Math.floor(window.innerWidth * 0.5);
        let newWidth = window.innerWidth - e.clientX;
        newWidth = Math.max(SIDEBAR_MIN, Math.min(maxWidth, newWidth));
        app.style.setProperty('--sidebar-width', newWidth + 'px');
    }

    function onPointerUp() {
        if (!dragging) return;
        dragging = false;
        handle.classList.remove('active');
        document.body.style.cursor = '';
        document.body.style.userSelect = '';

        // Persist
        const current = getComputedStyle(app).getPropertyValue('--sidebar-width').trim();
        localStorage.setItem(SIDEBAR_STORAGE_KEY, parseInt(current, 10));
    }

    handle.addEventListener('pointerdown', onPointerDown);
    document.addEventListener('pointermove', onPointerMove);
    document.addEventListener('pointerup', onPointerUp);
}

// ---- Chart panel resize handle ----

const CHART_HEIGHT_MIN = 150;
const CHART_HEIGHT_MAX_PCT = 0.6;
const CHART_HEIGHT_STORAGE_KEY = 'jugglebot-chart-height';
const CHART_COLLAPSED_STORAGE_KEY = 'jugglebot-chart-collapsed';

function initChartResizeHandle() {
    const handle = document.getElementById('chart-resize-handle');
    const panel = document.getElementById('chart-panel');
    const app = document.getElementById('app');
    const chevron = document.getElementById('chart-toggle-chevron');
    if (!handle || !panel || !app) return;

    // Restore saved state
    const savedCollapsed = localStorage.getItem(CHART_COLLAPSED_STORAGE_KEY);
    if (savedCollapsed === 'true') {
        panel.classList.add('collapsed');
        if (chevron) chevron.classList.add('collapsed');
    } else {
        const savedHeight = localStorage.getItem(CHART_HEIGHT_STORAGE_KEY);
        if (savedHeight) {
            const h = parseInt(savedHeight, 10);
            if (h >= CHART_HEIGHT_MIN) {
                app.style.setProperty('--chart-panel-height', h + 'px');
            }
        }
    }

    // Click on label/chevron toggles collapse
    const labelEl = document.getElementById('chart-toggle-label');
    [labelEl, chevron].forEach(el => {
        if (el) el.addEventListener('click', (e) => {
            e.stopPropagation();
            const isCollapsed = panel.classList.toggle('collapsed');
            if (chevron) chevron.classList.toggle('collapsed', isCollapsed);
            localStorage.setItem(CHART_COLLAPSED_STORAGE_KEY, isCollapsed);
            // Rebuild charts after un-collapsing (layout needs a frame to settle)
            if (!isCollapsed) {
                requestAnimationFrame(() => rebuildCharts());
            }
        });
    });

    // Drag to resize
    let dragging = false;

    handle.addEventListener('pointerdown', (e) => {
        // Don't start drag on label/chevron click
        if (e.target === labelEl || e.target === chevron) return;
        dragging = true;
        handle.classList.add('active');
        handle.setPointerCapture(e.pointerId);
        document.body.style.cursor = 'row-resize';
        document.body.style.userSelect = 'none';
        e.preventDefault();
    });

    document.addEventListener('pointermove', (e) => {
        if (!dragging) return;
        const maxHeight = Math.floor(window.innerHeight * CHART_HEIGHT_MAX_PCT);
        let newHeight = window.innerHeight - e.clientY;
        newHeight = Math.max(CHART_HEIGHT_MIN, Math.min(maxHeight, newHeight));
        app.style.setProperty('--chart-panel-height', newHeight + 'px');
        panel.classList.remove('collapsed');
        if (chevron) chevron.classList.remove('collapsed');
    });

    document.addEventListener('pointerup', () => {
        if (!dragging) return;
        dragging = false;
        handle.classList.remove('active');
        document.body.style.cursor = '';
        document.body.style.userSelect = '';
        const current = getComputedStyle(app).getPropertyValue('--chart-panel-height').trim();
        localStorage.setItem(CHART_HEIGHT_STORAGE_KEY, parseInt(current, 10));
        localStorage.setItem(CHART_COLLAPSED_STORAGE_KEY, 'false');
    });
}

// ---- Font size control ----

const FONT_SIZE_STORAGE_KEY = 'jugglebot-font-size';
const FONT_SIZE_MIN = 10;
const FONT_SIZE_MAX = 22;
const FONT_SIZE_STEP = 1;
const FONT_SIZE_DEFAULT = 14;

function initFontSize() {
    const saved = localStorage.getItem(FONT_SIZE_STORAGE_KEY);
    const size = saved ? parseInt(saved, 10) : FONT_SIZE_DEFAULT;
    applyFontSize(size);

    const decreaseBtn = document.getElementById('font-decrease');
    const increaseBtn = document.getElementById('font-increase');
    if (decreaseBtn) {
        decreaseBtn.addEventListener('click', () => {
            const current = getCurrentFontSize();
            if (current > FONT_SIZE_MIN) applyFontSize(current - FONT_SIZE_STEP);
        });
    }
    if (increaseBtn) {
        increaseBtn.addEventListener('click', () => {
            const current = getCurrentFontSize();
            if (current < FONT_SIZE_MAX) applyFontSize(current + FONT_SIZE_STEP);
        });
    }
}

function getCurrentFontSize() {
    return parseInt(document.documentElement.style.fontSize, 10) || FONT_SIZE_DEFAULT;
}

function applyFontSize(size) {
    size = Math.max(FONT_SIZE_MIN, Math.min(FONT_SIZE_MAX, size));
    document.documentElement.style.fontSize = size + 'px';
    localStorage.setItem(FONT_SIZE_STORAGE_KEY, size);
    const label = document.getElementById('font-size-label');
    if (label) label.textContent = size + 'px';
}

// ---- Topic discovery ----

/** Topics we subscribe to for data processing (not just monitoring) */
const GUI_SUBSCRIBED_TOPICS = new Set([
    'robot_state', 'bb/heartbeat', 'orchestrator_state',
    'can_traffic', 'hand_telemetry', 'mocap_data', 'rigid_body_poses',
    'leg_lengths_topic', 'control_mode_topic', 'motion/diagnostics',
    'bb/calibration_result', 'cone/heartbeat', 'cone/timing_result',
]);

/** Active spy subscriptions: Map<topicName, ROSLIB.Topic> */
const spySubscriptions = new Map();
let discoveryTimer = null;

function startTopicDiscovery() {
    // Run immediately, then every 3 seconds
    runTopicDiscovery();
    if (!discoveryTimer) {
        discoveryTimer = setInterval(runTopicDiscovery, 3000);
    }
}

function stopTopicDiscovery() {
    if (discoveryTimer) {
        clearInterval(discoveryTimer);
        discoveryTimer = null;
    }
    // Clean up spy subscriptions
    for (const topic of spySubscriptions.values()) {
        ros.unsubscribeSpy(topic);
    }
    spySubscriptions.clear();
}

function runTopicDiscovery() {
    ros.discoverTopics((result) => {
        if (!result || !result.topics) return;

        const topics = result.topics;
        const types = result.types || [];

        for (let i = 0; i < topics.length; i++) {
            const name = topics[i].startsWith('/') ? topics[i].substring(1) : topics[i];
            const type = types[i] || '';

            // Register for display
            registerTopic(name, type);

            // Skip topics we already have full subscriptions for
            if (GUI_SUBSCRIBED_TOPICS.has(name)) continue;

            // Skip if we already have a spy subscription
            if (spySubscriptions.has(name)) continue;

            // Create a lightweight spy subscription
            const spyTopic = ros.subscribeSpy(name, type, () => {
                recordTopicMessage(name);
            }, 200);

            if (spyTopic) {
                spySubscriptions.set(name, spyTopic);
            }
        }
    });
}

// ---- Start ----

// Wait for ROSLIB to be available (loaded via script tag in HTML)
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
} else {
    init();
}
