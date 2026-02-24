/**
 * ball-butler-model.js — Ball Butler 3D geometry.
 *
 * Creates a simple line-art model: pedestal, yaw turntable, pitch arm, hand slider.
 * Updates from bb/heartbeat topic data.
 */

import * as THREE from 'three';
import { scene, sceneGroups, robotToThreeScaled } from './viewer.js';
import { BB_POSITION_MM } from './geometry-config.js';

let bbGroup;
let pedestalMesh;
let yawGroup;    // rotates around Z (robot) = Y (Three.js)
let pitchGroup;  // tilts from turntable
let handMarker;

// Dimensions (approximate, for visualisation)
const PEDESTAL_HEIGHT = 120;  // mm
const PEDESTAL_RADIUS = 30;   // mm
const ARM_LENGTH = 200;       // mm
const HAND_MARKER_SIZE = 10;  // mm

const BB_COLOR = 0xa78bfa; // purple accent

/**
 * Initialise the Ball Butler 3D model.
 */
export function initBallButlerModel() {
    bbGroup = new THREE.Group();
    bbGroup.name = 'ball-butler';

    // Position at configured offset from base centre
    const pos = robotToThreeScaled(BB_POSITION_MM[0], BB_POSITION_MM[1], BB_POSITION_MM[2]);
    bbGroup.position.copy(pos);

    // ---- Pedestal: small cylinder ----
    const pedGeom = new THREE.CylinderGeometry(
        PEDESTAL_RADIUS * 0.001, PEDESTAL_RADIUS * 0.001,
        PEDESTAL_HEIGHT * 0.001, 12
    );
    const pedMat = new THREE.MeshStandardMaterial({ color: BB_COLOR, transparent: true, opacity: 0.6 });
    pedestalMesh = new THREE.Mesh(pedGeom, pedMat);
    // In Three.js Y-up: pedestal stands on Y axis
    pedestalMesh.position.y = PEDESTAL_HEIGHT * 0.5 * 0.001;
    bbGroup.add(pedestalMesh);

    // ---- Yaw group: rotates around Y (Three.js) ----
    yawGroup = new THREE.Group();
    yawGroup.position.y = PEDESTAL_HEIGHT * 0.001;
    bbGroup.add(yawGroup);

    // Turntable ring
    const ringGeom = new THREE.TorusGeometry(PEDESTAL_RADIUS * 0.8 * 0.001, 0.003, 8, 24);
    const ringMat = new THREE.MeshStandardMaterial({ color: BB_COLOR });
    const ring = new THREE.Mesh(ringGeom, ringMat);
    ring.rotation.x = Math.PI / 2; // lay flat
    yawGroup.add(ring);

    // ---- Pitch group: tilts from turntable ----
    pitchGroup = new THREE.Group();
    yawGroup.add(pitchGroup);

    // Arm (cylinder along local Y in Three.js)
    const armGeom = new THREE.CylinderGeometry(0.003, 0.003, ARM_LENGTH * 0.001, 8);
    const armMat = new THREE.MeshStandardMaterial({ color: BB_COLOR, transparent: true, opacity: 0.7 });
    const armMesh = new THREE.Mesh(armGeom, armMat);
    armMesh.position.y = ARM_LENGTH * 0.5 * 0.001;
    pitchGroup.add(armMesh);

    // ---- Hand marker: small sphere sliding along arm ----
    const handGeom = new THREE.SphereGeometry(HAND_MARKER_SIZE * 0.001, 8, 8);
    const handMat = new THREE.MeshStandardMaterial({ color: 0xf59e0b });
    handMarker = new THREE.Mesh(handGeom, handMat);
    handMarker.position.y = 0;
    pitchGroup.add(handMarker);

    scene.add(bbGroup);
    sceneGroups['Ball Butler'] = bbGroup;
}

/**
 * Update the Ball Butler model from heartbeat data.
 * @param {number} yawDeg - Current yaw position in degrees
 * @param {number} pitchDeg - Current pitch position in degrees
 * @param {number} handPosMM - Current hand position in mm along the arm
 */
export function updateBallButler(yawDeg, pitchDeg, handPosMM) {
    if (!yawGroup) return;

    // Yaw: rotate around Three.js Y axis
    yawGroup.rotation.y = yawDeg * Math.PI / 180;

    // Pitch: tilt around Three.js Z axis (robot X axis after yaw rotation)
    pitchGroup.rotation.z = pitchDeg * Math.PI / 180;

    // Hand: slide along arm (local Y in pitch group)
    handMarker.position.y = Math.max(0, handPosMM) * 0.001;
}

/**
 * Show/hide the Ball Butler model.
 * @param {boolean} visible
 */
export function setBallButlerVisible(visible) {
    if (bbGroup) bbGroup.visible = visible;
}
