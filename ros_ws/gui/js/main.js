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
    updateFlags, updateBBPanel, setBBDisconnected,
    updateCANTraffic, updateTrackingError,
} from './panels.js';
import { initCommands, updateCommandStates } from './commands.js';
import { INITIAL_HEIGHT_MM, MM_TO_REV } from './geometry-config.js';

// ---- Latest data stores ----
let latestMotorStates = null;
let latestCommandedLegs = null;  // Float64MultiArray data (revs)

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

    // 5. Init ROS connection
    ros.onConnectionStateChange(onConnectionStateChange);
    ros.init();

    // 6. Subscribe to topics
    subscribeAll();
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
            break;
        case 'connecting':
            dot.className = 'status-dot disconnected';
            text.textContent = 'Connecting...';
            break;
        case 'disconnected':
            dot.className = 'status-dot disconnected';
            text.textContent = 'Disconnected';
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
    const motors = msg.motor_states || [];
    latestMotorStates = motors;

    // Update panels
    updateMotorGrid(motors);
    updateFlags(msg);

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

function onBBHeartbeat(msg) {
    updateBBPanel(msg);
    updateBallButler(msg.yaw_deg, msg.pitch_deg, msg.hand_pos_mm);
}

function onOrchestratorState(msg) {
    updateOrchestratorState(msg.data);
    updateCommandStates();
}

function onCANTraffic(msg) {
    updateCANTraffic(msg);
}

let latestHandTelemetry = null;

function onHandTelemetry(msg) {
    latestHandTelemetry = msg;
}

function onRigidBodyPoses(msg) {
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

// ---- Start ----

// Wait for ROSLIB to be available (loaded via script tag in HTML)
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
} else {
    init();
}
