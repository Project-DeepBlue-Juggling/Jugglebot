/** Ball Butler CAD. Local robot frame is X right, Y forward, Z up. */
import * as THREE from 'three';
import { scene, sceneGroups, robotToThreeScaled } from './viewer.js';
import { BB_POSITION_MM, BB_YAW_S_OFFSET_MM, BB_PITCH_D_OFFSET_MM, BB_PITCH_Z_OFFSET_MM } from './geometry-config.js';
import { cadMesh, setAxisFault, setAxisHighlight } from './robot-meshes.js';

let bbGroup, yawGroup, pitchGroup, hand, pitch;
const DEG2RAD = Math.PI / 180;

export function initBallButlerModel() {
    bbGroup = new THREE.Group();
    bbGroup.name = 'ball-butler';
    bbGroup.position.copy(robotToThreeScaled(...BB_POSITION_MM));
    bbGroup.add(cadMesh('bb_base', 0x63758a));
    yawGroup = new THREE.Group();
    yawGroup.add(cadMesh('bb_yaw', 0x8c9bab));
    bbGroup.add(yawGroup);
    pitchGroup = new THREE.Group();
    // CAD pitch pivot lies on x=0; the hand's lateral offset is separate.
    pitchGroup.position.copy(robotToThreeScaled(0, -BB_PITCH_D_OFFSET_MM, BB_PITCH_Z_OFFSET_MM));
    pitch = cadMesh('bb_pitch', 0xb8c2cd, 7);
    hand = cadMesh('bb_hand', 0xe4ae53, 8);
    pitchGroup.add(pitch, hand);
    yawGroup.add(pitchGroup);
    updateBallButler(0, 90, 0);
    scene.add(bbGroup);
    sceneGroups['Ball Butler'] = bbGroup;
}

export function getBallButlerPickables() { return pitch ? [pitch, hand] : []; }

export function updateBallButler(yawDeg, pitchDeg, handPosMM) {
    if (!yawGroup || ![yawDeg, pitchDeg, handPosMM].every(Number.isFinite)) return;
    yawGroup.rotation.y = yawDeg * DEG2RAD;
    // CAD rail is vertical at rest. Positive pitch aims toward robot +Y.
    pitchGroup.rotation.x = (pitchDeg - 90) * DEG2RAD;
    hand.position.set(BB_YAW_S_OFFSET_MM * .001, handPosMM * .001, 0);
}

export function updateBallButlerPose(position, orientation) {
    if (!bbGroup) return;
    bbGroup.position.copy(robotToThreeScaled(position.x, position.y, position.z));
    bbGroup.quaternion.set(orientation.x, orientation.z, -orientation.y, orientation.w).normalize();
}
export function setBallButlerVisible(visible) { if (bbGroup) bbGroup.visible = visible; }
export function setBallButlerHighlight(target) {
    setAxisHighlight(7, target === 'pitch');
    setAxisHighlight(8, target === 'hand');
}
export function setBBPitchFault(faulted) { setAxisFault(7, faulted); }
export function setBBHandFault(faulted) { setAxisFault(8, faulted); }
