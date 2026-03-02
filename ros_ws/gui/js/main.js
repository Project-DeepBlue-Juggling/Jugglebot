/**
 * main.js — Entry point: init ROS, viewer, panels, wire callbacks.
 *
 * Wires together ros-bridge, the 3D viewer, status panels, and command controls.
 * Subscribes to all ROS topics and routes data to the appropriate modules.
 */

import * as ros from './ros-bridge.js';
import { initViewer, sceneGroups } from './viewer.js';
import {
    initStewartModel, updateStewartPose,
    initGhostOverlay, updateGhostOverlay,
} from './stewart-model.js';
import {
    legLengthsToPose, solveFK, resetFKState,
    quatToRotMatrix, poseToPlatNodes, poseToLegLengths, rotvecToRotMatrix,
} from './stewart-fk.js';
import { initBallButlerModel, updateBallButler } from './ball-butler-model.js';
import {
    initAllPanels, updateMotorGrid, updateOrchestratorState,
    updateFlags, updateLevellingPanel, updateBBPanel, setBBDisconnected,
    updateCANTraffic, updateTrackingError,
    recordTopicMessage, registerTopic, updateTopicMonitor, clearTopicData,
} from './panels.js';
import { initCommands, updateCommandStates } from './commands.js';
import { INITIAL_HEIGHT_MM, MM_TO_REV } from './geometry-config.js';
import { initTelemetryCharts, onTelemetryData, rebuildCharts } from './telemetry-charts.js';

// ---- Latest data stores ----
let latestMotorStates = null;
let latestCommandedLegs = null;  // Float64MultiArray data (revs)
let receivedOrchestratorState = false;  // True once an explicit orchestrator_state msg arrives

// ---- Initialisation ----

function init() {
    // 1. Init 3D viewer
    const container = document.getElementById('viewer-container');
    initViewer(container);
    initStewartModel();
    initGhostOverlay();
    initBallButlerModel();

    // 2. Init panels
    initAllPanels();

    // 3. Init commands
    initCommands();

    // 4. Init scene menu
    initSceneMenu();

    // 5. Init resize handles + font size
    initLeftResizeHandle();
    initResizeHandle();
    initChartResizeHandle();
    initFontSize();

    // 6. Init telemetry charts
    initTelemetryCharts();

    // 7. Init ROS connection
    ros.onConnectionStateChange(onConnectionStateChange);
    ros.init();

    // 8. Subscribe to topics
    subscribeAll();

    // 9. Start topic monitor timers
    setInterval(updateTopicMonitor, 1000);
}

// ---- Connection state UI ----

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
            receivedOrchestratorState = false;
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

    // Orchestrator state (on change)
    ros.subscribe('orchestrator_state', 'std_msgs/msg/String', onOrchestratorState, 0);

    // CAN traffic (~2Hz)
    ros.subscribe('can_traffic', 'jugglebot_interfaces/msg/CanTrafficReportMessage', onCANTraffic, 0);

    // Hand telemetry (500Hz -> throttle to 10Hz = 100ms)
    ros.subscribe('hand_telemetry', 'jugglebot_interfaces/msg/HandTelemetryMessage', onHandTelemetry, 100);

    // Rigid body poses (200Hz -> throttle to 20Hz = 50ms) -- preferred pose source
    ros.subscribe('rigid_body_poses', 'jugglebot_interfaces/msg/RigidBodyPoses', onRigidBodyPoses, 50);

    // Commanded leg lengths (500Hz -> throttle to 20Hz = 50ms)
    ros.subscribe('leg_lengths_topic', 'std_msgs/msg/Float64MultiArray', onLegLengths, 50);
}

// ---- Topic handlers ----

let useMocapPose = false;
let mocapTimeout = null;
const MOCAP_TIMEOUT_MS = 1000;

function onRobotState(msg) {
    recordTopicMessage('robot_state');
    const motors = msg.motor_states || [];
    latestMotorStates = motors;

    // Update panels
    updateMotorGrid(motors);
    updateFlags(msg);
    updateLevellingPanel(msg);

    // Feed telemetry charts
    onTelemetryData(motors, latestCommandedLegs, latestHandTelemetry);

    // Infer orchestrator state from robot_state flags when no explicit
    // orchestrator_state message has been received yet. This handles the
    // case where the GUI connects after the orchestrator has already
    // settled into IDLE (it only publishes state on change).
    if (!receivedOrchestratorState && msg.is_homed) {
        updateOrchestratorState('IDLE');
        updateCommandStates();
    }

    // Update 3D model via FK (if mocap not available)
    if (!useMocapPose && motors.length >= 6) {
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

    // Update tracking error + ghost overlay if we have both commanded and measured
    if (latestCommandedLegs && motors.length >= 6) {
        const errors = [];
        let rmsError = 0;
        for (let i = 0; i < 6; i++) {
            // Both are in revs; convert error to mm
            const cmdRev = latestCommandedLegs[i] || 0;
            const measRev = motors[i].pos_estimate;
            const errorRev = cmdRev - measRev;
            const errorMM = errorRev / MM_TO_REV[i];
            errors.push(errorMM);
            rmsError += errorMM * errorMM;
        }
        rmsError = Math.sqrt(rmsError / 6);

        // Hand error in revs
        if (motors.length >= 7 && latestCommandedLegs.length >= 7) {
            errors.push((latestCommandedLegs[6] || 0) - motors[6].pos_estimate);
        } else {
            errors.push(0);
        }
        updateTrackingError(errors);

        // Ghost overlay: compute commanded platform pose from commanded leg lengths
        if (latestCommandedLegs.length >= 6) {
            const cmdExtensions = [];
            for (let i = 0; i < 6; i++) {
                cmdExtensions.push(latestCommandedLegs[i] / MM_TO_REV[i]);
            }
            const cmdResult = solveFK(cmdExtensions);
            if (cmdResult.converged) {
                updateGhostOverlay(cmdResult.platNodes, rmsError);
            }
        }
    }
}

let bbTimeout = null;
const BB_TIMEOUT_MS = 5000;

function onBBHeartbeat(msg) {
    recordTopicMessage('bb/heartbeat');
    updateBBPanel(msg);
    updateBallButler(msg.yaw_deg, msg.pitch_deg, msg.hand_pos_mm);

    if (bbTimeout) clearTimeout(bbTimeout);
    bbTimeout = setTimeout(() => { setBBDisconnected(); }, BB_TIMEOUT_MS);
}

function onOrchestratorState(msg) {
    recordTopicMessage('orchestrator_state');
    receivedOrchestratorState = true;
    updateOrchestratorState(msg.data);
    updateCommandStates();
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

function onRigidBodyPoses(msg) {
    recordTopicMessage('rigid_body_poses');
    // Mark mocap as available; reset timeout
    useMocapPose = true;
    if (mocapTimeout) clearTimeout(mocapTimeout);
    mocapTimeout = setTimeout(() => { useMocapPose = false; }, MOCAP_TIMEOUT_MS);

    // Find the platform rigid body
    // RigidBodyPoses contains: bodies[] where each body has { name, pose: PoseStamped }
    const bodies = msg.bodies || [];
    if (bodies.length === 0) return;

    // Find the platform body by name, or fall back to the first body
    let platformBody = bodies.find(b => b.name && b.name.toLowerCase().includes('platform'));
    if (!platformBody) platformBody = bodies[0];
    if (!platformBody || !platformBody.pose) return;

    // PoseStamped: { header, pose: { position: {x,y,z}, orientation: {x,y,z,w} } }
    const pose = platformBody.pose.pose || platformBody.pose;
    const p = pose.position;
    const q = pose.orientation;

    if (!p || !q) return;

    // Position is in mm, global frame (mocap reports absolute position)
    const globalPos = [p.x, p.y, p.z];

    // Convert quaternion to rotation matrix
    const R = quatToRotMatrix(q.w, q.x, q.y, q.z);

    // Compute platform node world positions
    const platNodes = poseToPlatNodes(globalPos, R);

    // Platform centre is the measured position
    const platCentre = globalPos;

    // Hand extension from motor state (if available)
    let handExtMM = 0;
    if (latestMotorStates && latestMotorStates.length >= 7) {
        handExtMM = latestMotorStates[6].pos_estimate / MM_TO_REV[0]; // approximate
    }

    updateStewartPose(platNodes, platCentre, handExtMM);
}

function onLegLengths(msg) {
    recordTopicMessage('leg_lengths_topic');
    latestCommandedLegs = msg.data;
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
    'can_traffic', 'hand_telemetry', 'rigid_body_poses', 'leg_lengths_topic',
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
            const name = topics[i];
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
