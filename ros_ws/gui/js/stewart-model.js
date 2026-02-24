/**
 * stewart-model.js — Stewart platform 3D geometry.
 *
 * Creates base hexagon, 6 legs, platform hexagon, and hand axis.
 * Updates from pose data (either FK result or direct mocap).
 */

import * as THREE from 'three';
import { scene, sceneGroups, robotToThreeScaled } from './viewer.js';
import {
    BASE_NODES_MM, INIT_PLAT_NODES_MM, INIT_LEG_LENGTHS_MM,
    INITIAL_HEIGHT_MM, LEG_STROKE_MM,
    ARM_HEIGHT_FROM_PLATFORM_MM, HAND_DEPTH_OFFSET_MM,
} from './geometry-config.js';

/** Three.js group containing all Stewart platform objects */
let platformGroup;

/** Individual mesh references */
let baseMesh;
let platformMesh;
const legMeshes = [];
let handLine;

/** Current platform node positions (world frame, mm) */
let currentPlatNodes = null;
/** Current platform centre (world frame, mm) */
let currentPlatCentre = null;

// Colours
const BASE_COLOR = 0x3b82f6;
const PLAT_COLOR = 0xd1d5db;
const HAND_COLOR = 0xf59e0b;

function lerp(a, b, t) { return a + (b - a) * t; }

/**
 * Initialise the Stewart platform geometry at home position.
 */
export function initStewartModel() {
    platformGroup = new THREE.Group();
    platformGroup.name = 'stewart-platform';

    // ---- Base hexagon ----
    baseMesh = createHexMesh(BASE_NODES_MM, BASE_COLOR, 0.15);
    platformGroup.add(baseMesh);

    // ---- Platform hexagon (at home position) ----
    const homePlatNodes = INIT_PLAT_NODES_MM.map(n => [n[0], n[1], n[2] + INITIAL_HEIGHT_MM]);
    platformMesh = createHexMesh(homePlatNodes, PLAT_COLOR, 0.2);
    platformGroup.add(platformMesh);

    // ---- Legs (cylinders between base and platform nodes) ----
    for (let i = 0; i < 6; i++) {
        const leg = createLegCylinder();
        legMeshes.push(leg);
        platformGroup.add(leg);
    }

    // Position legs at home
    currentPlatNodes = homePlatNodes;
    currentPlatCentre = [0, 0, INITIAL_HEIGHT_MM];
    updateLegPositions();

    // ---- Hand axis (line from platform centre, perpendicular to surface) ----
    const handGeom = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(0, 0, 0),
        new THREE.Vector3(0, 0, 0),
    ]);
    handLine = new THREE.Line(
        handGeom,
        new THREE.LineBasicMaterial({ color: HAND_COLOR, linewidth: 2 })
    );
    platformGroup.add(handLine);
    updateHandAxis(0); // home position, no extension

    scene.add(platformGroup);
    sceneGroups['Platform'] = platformGroup;
}

/**
 * Create a hexagonal mesh (edges + semi-transparent fill) from 6 node positions.
 */
function createHexMesh(nodes, color, opacity) {
    const group = new THREE.Group();

    // Edge lines
    const linePoints = [];
    for (let i = 0; i < 6; i++) {
        const n = nodes[i];
        linePoints.push(robotToThreeScaled(n[0], n[1], n[2]));
    }
    linePoints.push(linePoints[0].clone()); // close the loop

    const lineGeom = new THREE.BufferGeometry().setFromPoints(linePoints);
    const line = new THREE.Line(lineGeom, new THREE.LineBasicMaterial({ color }));
    group.add(line);

    // Semi-transparent fill (triangle fan from centroid)
    const cx = nodes.reduce((s, n) => s + n[0], 0) / 6;
    const cy = nodes.reduce((s, n) => s + n[1], 0) / 6;
    const cz = nodes.reduce((s, n) => s + n[2], 0) / 6;
    const centre = robotToThreeScaled(cx, cy, cz);

    const fillGeom = new THREE.BufferGeometry();
    const verts = [];
    for (let i = 0; i < 6; i++) {
        const n0 = nodes[i];
        const n1 = nodes[(i + 1) % 6];
        verts.push(
            centre.x, centre.y, centre.z,
            ...robotToThreeScaled(n0[0], n0[1], n0[2]).toArray(),
            ...robotToThreeScaled(n1[0], n1[1], n1[2]).toArray(),
        );
    }
    fillGeom.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3));
    fillGeom.computeVertexNormals();

    const fillMat = new THREE.MeshBasicMaterial({
        color,
        transparent: true,
        opacity,
        side: THREE.DoubleSide,
        depthWrite: false,
    });
    group.add(new THREE.Mesh(fillGeom, fillMat));

    return group;
}

/**
 * Create a leg cylinder mesh (will be positioned later).
 */
function createLegCylinder() {
    // Thin cylinder, repositioned each frame
    const geom = new THREE.CylinderGeometry(0.004, 0.004, 1, 8);
    const mat = new THREE.MeshStandardMaterial({ color: 0x22c55e });
    const mesh = new THREE.Mesh(geom, mat);
    return mesh;
}

/**
 * Position a cylinder mesh between two points (in Three.js coords).
 */
function positionCylinder(mesh, p1, p2) {
    const mid = new THREE.Vector3().addVectors(p1, p2).multiplyScalar(0.5);
    mesh.position.copy(mid);

    const dir = new THREE.Vector3().subVectors(p2, p1);
    const len = dir.length();
    mesh.scale.set(1, len, 1);

    // Orient cylinder (default Y axis) to point from p1 to p2
    if (len > 1e-10) {
        const axis = new THREE.Vector3(0, 1, 0);
        const quat = new THREE.Quaternion().setFromUnitVectors(axis, dir.normalize());
        mesh.quaternion.copy(quat);
    }
}

/**
 * Update leg cylinder positions and colours based on current platform nodes.
 */
function updateLegPositions() {
    if (!currentPlatNodes) return;

    for (let i = 0; i < 6; i++) {
        const base = BASE_NODES_MM[i];
        const plat = currentPlatNodes[i];

        const p1 = robotToThreeScaled(base[0], base[1], base[2]);
        const p2 = robotToThreeScaled(plat[0], plat[1], plat[2]);
        positionCylinder(legMeshes[i], p1, p2);

        // Color by extension ratio
        const dx = plat[0] - base[0];
        const dy = plat[1] - base[1];
        const dz = plat[2] - base[2];
        const absLen = Math.sqrt(dx * dx + dy * dy + dz * dz);
        const ext = absLen - INIT_LEG_LENGTHS_MM[i];
        const ratio = Math.max(0, Math.min(1, ext / LEG_STROKE_MM));

        const color = extensionColor(ratio);
        legMeshes[i].material.color.setHex(color);
    }
}

/**
 * Return a color for a leg extension ratio (0=retracted → 1=fully extended).
 * Green → Amber → Red.
 */
function extensionColor(ratio) {
    if (ratio < 0.5) {
        // Green to Amber
        const t = ratio * 2;
        const r = Math.round(lerp(0x22, 0xf5, t));
        const g = Math.round(lerp(0xc5, 0x9e, t));
        const b = Math.round(lerp(0x5e, 0x0b, t));
        return (r << 16) | (g << 8) | b;
    } else {
        // Amber to Red
        const t = (ratio - 0.5) * 2;
        const r = Math.round(lerp(0xf5, 0xef, t));
        const g = Math.round(lerp(0x9e, 0x44, t));
        const b = Math.round(lerp(0x0b, 0x44, t));
        return (r << 16) | (g << 8) | b;
    }
}

/**
 * Update the hand axis line.
 * @param {number} extensionMM - hand extension in mm (0 = fully retracted)
 */
function updateHandAxis(extensionMM) {
    if (!currentPlatCentre || !currentPlatNodes) return;

    // Compute platform normal (cross product of two edges)
    const p0 = currentPlatNodes[0];
    const p1 = currentPlatNodes[1];
    const pc = currentPlatCentre;

    const v1 = [p0[0] - pc[0], p0[1] - pc[1], p0[2] - pc[2]];
    const v2 = [p1[0] - pc[0], p1[1] - pc[1], p1[2] - pc[2]];
    const normal = [
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0],
    ];
    const nLen = Math.sqrt(normal[0] ** 2 + normal[1] ** 2 + normal[2] ** 2);
    if (nLen < 1e-10) return;
    normal[0] /= nLen; normal[1] /= nLen; normal[2] /= nLen;

    // Ensure normal points upward (positive Z in robot frame)
    if (normal[2] < 0) {
        normal[0] *= -1; normal[1] *= -1; normal[2] *= -1;
    }

    // Hand base: platform centre + arm height offset along normal
    const handBase = [
        pc[0] + normal[0] * ARM_HEIGHT_FROM_PLATFORM_MM,
        pc[1] + normal[1] * ARM_HEIGHT_FROM_PLATFORM_MM,
        pc[2] + normal[2] * ARM_HEIGHT_FROM_PLATFORM_MM,
    ];

    // Hand tip: extend along normal by current extension
    const handTip = [
        handBase[0] + normal[0] * extensionMM,
        handBase[1] + normal[1] * extensionMM,
        handBase[2] + normal[2] * extensionMM,
    ];

    const positions = handLine.geometry.attributes.position;
    const base3 = robotToThreeScaled(handBase[0], handBase[1], handBase[2]);
    const tip3 = robotToThreeScaled(handTip[0], handTip[1], handTip[2]);
    positions.setXYZ(0, base3.x, base3.y, base3.z);
    positions.setXYZ(1, tip3.x, tip3.y, tip3.z);
    positions.needsUpdate = true;
}

/**
 * Update platform hexagon mesh from new node positions.
 */
function updatePlatformMesh() {
    if (!currentPlatNodes) return;

    // Update edge line
    const line = platformMesh.children[0];
    const linePos = line.geometry.attributes.position;
    for (let i = 0; i < 6; i++) {
        const n = currentPlatNodes[i];
        const p = robotToThreeScaled(n[0], n[1], n[2]);
        linePos.setXYZ(i, p.x, p.y, p.z);
    }
    // Close loop
    const n0 = currentPlatNodes[0];
    const p0 = robotToThreeScaled(n0[0], n0[1], n0[2]);
    linePos.setXYZ(6, p0.x, p0.y, p0.z);
    linePos.needsUpdate = true;

    // Update fill triangles
    const fillMesh = platformMesh.children[1];
    const fillPos = fillMesh.geometry.attributes.position;

    const cx = currentPlatNodes.reduce((s, n) => s + n[0], 0) / 6;
    const cy = currentPlatNodes.reduce((s, n) => s + n[1], 0) / 6;
    const cz = currentPlatNodes.reduce((s, n) => s + n[2], 0) / 6;
    const centre = robotToThreeScaled(cx, cy, cz);

    let vi = 0;
    for (let i = 0; i < 6; i++) {
        const na = currentPlatNodes[i];
        const nb = currentPlatNodes[(i + 1) % 6];
        const pa = robotToThreeScaled(na[0], na[1], na[2]);
        const pb = robotToThreeScaled(nb[0], nb[1], nb[2]);

        fillPos.setXYZ(vi++, centre.x, centre.y, centre.z);
        fillPos.setXYZ(vi++, pa.x, pa.y, pa.z);
        fillPos.setXYZ(vi++, pb.x, pb.y, pb.z);
    }
    fillPos.needsUpdate = true;
    fillMesh.geometry.computeVertexNormals();
}

// ---- Ghost overlay (commanded pose vs measured) ----

let ghostGroup = null;
let ghostLines = null;
let ghostVisible = false;
let ghostNodes = null; // commanded platform nodes

function initGhostOverlay() {
    ghostGroup = new THREE.Group();
    ghostGroup.name = 'ghost-overlay';

    // Ghost hexagon: wireframe only, red-tinted, semi-transparent
    const linePoints = [];
    for (let i = 0; i <= 6; i++) {
        linePoints.push(new THREE.Vector3(0, 0, 0));
    }
    const lineGeom = new THREE.BufferGeometry().setFromPoints(linePoints);
    ghostLines = new THREE.Line(
        lineGeom,
        new THREE.LineBasicMaterial({
            color: 0xef4444,
            transparent: true,
            opacity: 0.0, // starts invisible, fades in with error magnitude
            linewidth: 2,
        })
    );
    ghostGroup.add(ghostLines);

    ghostGroup.visible = false;
    scene.add(ghostGroup);
    sceneGroups['Ghost Overlay'] = ghostGroup;
}

/**
 * Update the ghost overlay to show the commanded pose.
 * Opacity scales with tracking error magnitude.
 * @param {number[][]} commandedPlatNodes - 6 commanded platform node positions (mm)
 * @param {number} errorMagnitude - RMS tracking error (mm)
 */
export function updateGhostOverlay(commandedPlatNodes, errorMagnitude) {
    if (!ghostLines) return;

    ghostNodes = commandedPlatNodes;

    // Update ghost hexagon vertices
    const linePos = ghostLines.geometry.attributes.position;
    for (let i = 0; i < 6; i++) {
        const n = commandedPlatNodes[i];
        const p = robotToThreeScaled(n[0], n[1], n[2]);
        linePos.setXYZ(i, p.x, p.y, p.z);
    }
    // Close loop
    const n0 = commandedPlatNodes[0];
    const p0 = robotToThreeScaled(n0[0], n0[1], n0[2]);
    linePos.setXYZ(6, p0.x, p0.y, p0.z);
    linePos.needsUpdate = true;

    // Opacity: 0 when error < 0.5mm, full at error > 5mm
    const minErr = 0.5;
    const maxErr = 5.0;
    const t = Math.max(0, Math.min(1, (errorMagnitude - minErr) / (maxErr - minErr)));
    ghostLines.material.opacity = t * 0.8;

    ghostGroup.visible = ghostVisible && t > 0.01;
}

/**
 * Toggle ghost overlay visibility.
 * @param {boolean} visible
 */
export function setGhostVisible(visible) {
    ghostVisible = visible;
    if (ghostGroup) ghostGroup.visible = visible && ghostLines.material.opacity > 0.01;
}

// ---- Public update API ----

/**
 * Update the full Stewart platform from a pose result.
 * @param {number[][]} platNodes - 6 platform node world positions [x,y,z] in mm
 * @param {number[]} platCentre - platform centre [x,y,z] in mm
 * @param {number} handExtensionMM - hand extension in mm
 */
export function updateStewartPose(platNodes, platCentre, handExtensionMM) {
    currentPlatNodes = platNodes;
    currentPlatCentre = platCentre;

    updatePlatformMesh();
    updateLegPositions();
    updateHandAxis(handExtensionMM);
}

export { initGhostOverlay };
