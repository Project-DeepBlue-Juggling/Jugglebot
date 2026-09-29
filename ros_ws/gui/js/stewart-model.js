/** Articulated CAD in metres; homed GLB mesh registrations are baked offline. */
import * as THREE from 'three';
import { scene, sceneGroups, robotToThreeScaled } from './viewer.js';
import { BASE_NODES_MM, INIT_PLAT_NODES_MM, INITIAL_HEIGHT_MM } from './geometry-config.js';
import { cadMesh, setAxisFault, setAxisHighlight } from './robot-meshes.js';

let platformGroup, movingPlatform, hand;
const legs = [];
const pickables = [];
const up = new THREE.Vector3(0, 1, 0);
const direction = new THREE.Vector3();
const target = new THREE.Vector3();
const matrix = new THREE.Matrix4();
const identity = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];

export function initStewartModel() {
    platformGroup = new THREE.Group();
    platformGroup.name = 'stewart-platform';
    platformGroup.add(cadMesh('jb_base', 0x718096));
    movingPlatform = new THREE.Group();
    movingPlatform.add(cadMesh('jb_platform', 0xb9c2cc));
    hand = cadMesh('jb_hand', 0xe4ae53, 6);
    movingPlatform.add(hand);
    platformGroup.add(movingPlatform);
    for (let i = 0; i < 6; i++) {
        const group = new THREE.Group();
        const outer = cadMesh('jb_outer', 0x8997a5, i);
        const inner = cadMesh('jb_inner', 0xd7dee5, i);
        group.add(outer, inner);
        legs.push({ group, inner });
        pickables.push(outer, inner);
        platformGroup.add(group);
    }
    pickables.push(hand);
    updateStewartPose(INIT_PLAT_NODES_MM.map(n => [n[0], n[1], n[2] + INITIAL_HEIGHT_MM]),
        [0, 0, INITIAL_HEIGHT_MM], 0, identity);
    scene.add(platformGroup);
    sceneGroups['Jugglebot'] = platformGroup;
}

export function getStewartPickables() { return pickables; }

/** FK supplies the complete rotation, preserving yaw as well as platform tilt. */
export function updateStewartPose(platNodes, platCentre, handExtensionMM, R = identity) {
    if (!movingPlatform || !platCentre.every(Number.isFinite) || !Number.isFinite(handExtensionMM)) return;
    movingPlatform.position.copy(robotToThreeScaled(...platCentre));
    // C R C^-1 for C(x,y,z)=(x,z,-y).
    matrix.set(R[0][0], R[0][2], -R[0][1], 0,
        R[2][0], R[2][2], -R[2][1], 0,
        -R[1][0], -R[1][2], R[1][1], 0, 0, 0, 0, 1);
    movingPlatform.quaternion.setFromRotationMatrix(matrix);
    hand.position.y = handExtensionMM * .001;
    for (let i = 0; i < 6; i++) {
        const { group, inner } = legs[i];
        group.position.copy(robotToThreeScaled(...BASE_NODES_MM[i]));
        target.copy(robotToThreeScaled(...platNodes[i]));
        direction.subVectors(target, group.position);
        const length = direction.length();
        if (length > 1e-9) group.quaternion.setFromUnitVectors(up, direction.multiplyScalar(1 / length));
        // Inner tube translates relative to outer; neither mesh is stretched.
        inner.position.y = length;
    }
}

export function setStewartHighlight(target) {
    for (let i = 0; i < 6; i++) setAxisHighlight(i, target === `leg${i}`);
    setAxisHighlight(6, target === 'hand');
}
export function setLegFault(index, faulted) { if (index >= 0 && index < 6) setAxisFault(index, faulted); }
export function setHandFault(faulted) { setAxisFault(6, faulted); }
