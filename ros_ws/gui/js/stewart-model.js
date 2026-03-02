/**
 * stewart-model.js — Stewart platform 3D geometry.
 *
 * Creates base hexagon, 6 legs, platform hexagon, and hand axis.
 * Updates from pose data (either FK result or direct mocap).
 *
 * Supports two rendering modes:
 *  - Procedural: wireframe hexagons + cylinders (always available)
 *  - CAD: GLB models loaded from models/ directory (optional, graceful fallback)
 */

import * as THREE from 'three';
import { scene, sceneGroups, robotToThreeScaled } from './viewer.js';
import {
    BASE_NODES_MM, INIT_PLAT_NODES_MM, INIT_LEG_LENGTHS_MM,
    INITIAL_HEIGHT_MM, LEG_STROKE_MM,
    ARM_HEIGHT_FROM_PLATFORM_MM, HAND_DEPTH_OFFSET_MM,
} from './geometry-config.js';
import { loadAllModels } from './cad-loader.js';

/** Three.js group containing all Stewart platform objects */
let platformGroup;

/** Individual mesh references (procedural geometry) */
let baseMesh;
let platformMesh;
const legMeshes = [];
let handLine;

/** Group containing all procedural geometry */
let proceduralGroup;

// ---- CAD model state ----

/** Group containing all CAD meshes */
let cadGroup = null;

/** CAD mesh references */
let cadBase = null;
let cadPlatform = null;
const cadLegLowers = [];
const cadLegUppers = [];
let cadHand = null;

/** Whether CAD models were successfully loaded */
let cadAvailable = false;

/** Whether user wants CAD mode (persisted to localStorage) */
const CAD_MODE_STORAGE_KEY = 'jugglebot-cad-mode';
let cadModeEnabled = localStorage.getItem(CAD_MODE_STORAGE_KEY) !== 'false'; // default true

/**
 * The local axis of the leg CAD model that points along the leg direction.
 * Adjust this if the exported GLB has a different convention.
 * Default: Y-up (matches glTF convention where Y is the default "up" axis).
 */
const LEG_LOCAL_AXIS = new THREE.Vector3(0, 1, 0);

/** Scale factor: mm → Three.js units (must match viewer.js SCALE) */
const SCALE = 0.001;

/** Current platform node positions (world frame, mm) */
let currentPlatNodes = null;
/** Current platform centre (world frame, mm) */
let currentPlatCentre = null;
/** Current platform rotation matrix (3x3 row-major, robot frame) */
let currentRotMatrix = null;

// Colours
const BASE_COLOR = 0x3b82f6;
const PLAT_COLOR = 0xd1d5db;
const HAND_COLOR = 0xf59e0b;

// Compute the longest platform edge (mm) for hand line sizing
function computeLongestPlatEdge() {
    let maxLen = 0;
    for (let i = 0; i < 6; i++) {
        const n0 = INIT_PLAT_NODES_MM[i];
        const n1 = INIT_PLAT_NODES_MM[(i + 1) % 6];
        const dx = n1[0] - n0[0], dy = n1[1] - n0[1], dz = n1[2] - n0[2];
        const len = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (len > maxLen) maxLen = len;
    }
    return maxLen;
}

/** Hand line base length: 2/3 of the longest platform strut */
const HAND_LINE_BASE_LENGTH_MM = computeLongestPlatEdge() * (2 / 3);

function lerp(a, b, t) { return a + (b - a) * t; }

/**
 * Initialise the Stewart platform geometry at home position.
 */
export function initStewartModel() {
    platformGroup = new THREE.Group();
    platformGroup.name = 'stewart-platform';

    // ---- Procedural geometry (always created as fallback) ----
    proceduralGroup = new THREE.Group();
    proceduralGroup.name = 'procedural';

    // Base hexagon
    baseMesh = createHexMesh(BASE_NODES_MM, BASE_COLOR, 0.15);
    proceduralGroup.add(baseMesh);

    // Platform hexagon (at home position)
    const homePlatNodes = INIT_PLAT_NODES_MM.map(n => [n[0], n[1], n[2] + INITIAL_HEIGHT_MM]);
    platformMesh = createHexMesh(homePlatNodes, PLAT_COLOR, 0.2);
    proceduralGroup.add(platformMesh);

    // Legs (cylinders between base and platform nodes)
    for (let i = 0; i < 6; i++) {
        const leg = createLegCylinder();
        legMeshes.push(leg);
        proceduralGroup.add(leg);
    }

    // Hand axis (cylinder from platform centre along normal)
    const handGeom = new THREE.CylinderGeometry(0.006, 0.006, 1, 8);
    handLine = new THREE.Mesh(
        handGeom,
        new THREE.MeshStandardMaterial({ color: HAND_COLOR, emissive: HAND_COLOR, emissiveIntensity: 0.3 })
    );
    proceduralGroup.add(handLine);

    platformGroup.add(proceduralGroup);

    // Position at home
    currentPlatNodes = homePlatNodes;
    currentPlatCentre = [0, 0, INITIAL_HEIGHT_MM];
    currentRotMatrix = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    updateLegPositions();
    updateHandAxis(0);

    scene.add(platformGroup);
    sceneGroups['Platform'] = platformGroup;

    // ---- Async CAD model loading ----
    initCADModels();
}

/**
 * Attempt to load CAD models. On success, adds them to the scene and
 * hides procedural geometry. On failure, procedural geometry stays visible.
 */
async function initCADModels() {
    const models = await loadAllModels();
    if (!models.hasAnyModel) return;

    cadAvailable = true;
    cadGroup = new THREE.Group();
    cadGroup.name = 'cad-models';

    // Base (static at origin)
    if (models.base) {
        cadBase = models.base;
        cadGroup.add(cadBase);
    }

    // Platform
    if (models.platform) {
        cadPlatform = models.platform;
        cadGroup.add(cadPlatform);
    }

    // Legs
    for (let i = 0; i < 6; i++) {
        const lower = models.legLowers[i];
        const upper = models.legUppers[i];
        if (lower) {
            cadLegLowers.push(lower);
            cadGroup.add(lower);
        } else {
            cadLegLowers.push(null);
        }
        if (upper) {
            cadLegUppers.push(upper);
            cadGroup.add(upper);
        } else {
            cadLegUppers.push(null);
        }
    }

    // Hand
    if (models.hand) {
        cadHand = models.hand;
        cadGroup.add(cadHand);
    }

    platformGroup.add(cadGroup);

    // Position CAD parts at home pose
    updateCADParts();

    // Apply initial view mode
    applyViewMode();
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
 * Update the hand axis cylinder.
 * The hand line extends from the platform centre along the platform normal.
 * Its length is always at least HAND_LINE_BASE_LENGTH_MM (2/3 of the longest
 * platform strut), plus any additional hand motor extension.
 *
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

    // Hand starts at platform centre, extends along normal for base length + motor extension
    const totalLength = HAND_LINE_BASE_LENGTH_MM + extensionMM;
    const handBase = [pc[0], pc[1], pc[2]];
    const handTip = [
        handBase[0] + normal[0] * totalLength,
        handBase[1] + normal[1] * totalLength,
        handBase[2] + normal[2] * totalLength,
    ];

    // Position the cylinder between handBase and handTip
    const base3 = robotToThreeScaled(handBase[0], handBase[1], handBase[2]);
    const tip3 = robotToThreeScaled(handTip[0], handTip[1], handTip[2]);
    positionCylinder(handLine, base3, tip3);
}

// ---- CAD model per-frame updates ----

/**
 * Convert a 3x3 rotation matrix (robot frame, row-major) to a Three.js Matrix4.
 * Includes the robot→Three.js coordinate swap: robot(x,y,z) → Three(x,z,-y).
 *
 * The coordinate swap matrix S maps robot-frame points to Three.js:
 *   Three.x = Robot.x,  Three.y = Robot.z,  Three.z = -Robot.y
 *   S = [[1,0,0],[0,0,1],[0,-1,0]],  S^T = S^-1 = [[1,0,0],[0,0,-1],[0,1,0]]
 *
 * Three.js rotation = S * R * S^T
 */
function rotMatrixToThreeMatrix4(R) {
    // Step 1: R * S^T  (S^T col0=[1,0,0], col1=[0,0,1], col2=[0,-1,0])
    const RSt = [
        [R[0][0], R[0][2], -R[0][1]],
        [R[1][0], R[1][2], -R[1][1]],
        [R[2][0], R[2][2], -R[2][1]],
    ];
    // Step 2: S * (R * S^T)  (S rows: [1,0,0], [0,0,1], [0,-1,0])
    // row 0 = RSt[0], row 1 = RSt[2], row 2 = -RSt[1]
    const Rthree = [
        [RSt[0][0], RSt[0][1], RSt[0][2]],
        [RSt[2][0], RSt[2][1], RSt[2][2]],
        [-RSt[1][0], -RSt[1][1], -RSt[1][2]],
    ];

    const m = new THREE.Matrix4();
    m.set(
        Rthree[0][0], Rthree[0][1], Rthree[0][2], 0,
        Rthree[1][0], Rthree[1][1], Rthree[1][2], 0,
        Rthree[2][0], Rthree[2][1], Rthree[2][2], 0,
        0, 0, 0, 1,
    );
    return m;
}

/**
 * Update all CAD model positions and orientations from current pose.
 */
function updateCADParts() {
    if (!cadAvailable || !cadGroup || !currentPlatNodes || !currentPlatCentre) return;

    // Platform: position + rotation
    if (cadPlatform && currentRotMatrix) {
        const pc = currentPlatCentre;
        cadPlatform.position.copy(robotToThreeScaled(pc[0], pc[1], pc[2]));
        const rotMat = rotMatrixToThreeMatrix4(currentRotMatrix);
        cadPlatform.quaternion.setFromRotationMatrix(rotMat);
    }

    // Legs: each half oriented along the leg axis
    for (let i = 0; i < 6; i++) {
        const base = BASE_NODES_MM[i];
        const plat = currentPlatNodes[i];

        const basePos = robotToThreeScaled(base[0], base[1], base[2]);
        const platPos = robotToThreeScaled(plat[0], plat[1], plat[2]);

        // Lower leg: anchored at base node, points toward platform node
        if (cadLegLowers[i]) {
            cadLegLowers[i].position.copy(basePos);
            const dir = new THREE.Vector3().subVectors(platPos, basePos).normalize();
            const quat = new THREE.Quaternion().setFromUnitVectors(LEG_LOCAL_AXIS, dir);
            cadLegLowers[i].quaternion.copy(quat);
        }

        // Upper leg: anchored at platform node, points toward base node
        if (cadLegUppers[i]) {
            cadLegUppers[i].position.copy(platPos);
            const dir = new THREE.Vector3().subVectors(basePos, platPos).normalize();
            const quat = new THREE.Quaternion().setFromUnitVectors(LEG_LOCAL_AXIS, dir);
            cadLegUppers[i].quaternion.copy(quat);
        }
    }

    // Hand: position at platform centre, oriented along platform normal
    if (cadHand && currentRotMatrix) {
        const pc = currentPlatCentre;
        cadHand.position.copy(robotToThreeScaled(pc[0], pc[1], pc[2]));
        const rotMat = rotMatrixToThreeMatrix4(currentRotMatrix);
        cadHand.quaternion.setFromRotationMatrix(rotMat);
    }
}

// ---- View mode (CAD vs procedural) ----

/**
 * Apply the current view mode: show CAD or procedural geometry.
 */
function applyViewMode() {
    const showCAD = cadAvailable && cadModeEnabled;
    if (proceduralGroup) proceduralGroup.visible = !showCAD;
    if (cadGroup) cadGroup.visible = showCAD;
}

/**
 * Toggle CAD rendering mode on/off. Falls back to procedural if CAD not available.
 * @param {boolean} enabled
 */
export function setCADMode(enabled) {
    cadModeEnabled = enabled;
    localStorage.setItem(CAD_MODE_STORAGE_KEY, enabled ? 'true' : 'false');
    applyViewMode();
}

/**
 * @returns {boolean} Whether CAD models are available
 */
export function isCADAvailable() {
    return cadAvailable;
}

/**
 * @returns {boolean} Whether CAD mode is currently enabled
 */
export function isCADModeEnabled() {
    return cadModeEnabled;
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
 * @param {number[][]|null} [rotMatrix] - 3x3 rotation matrix (row-major, robot frame). Required for CAD mode.
 */
export function updateStewartPose(platNodes, platCentre, handExtensionMM, rotMatrix) {
    currentPlatNodes = platNodes;
    currentPlatCentre = platCentre;
    if (rotMatrix) currentRotMatrix = rotMatrix;

    // Always update procedural geometry (even if hidden — keeps state consistent for toggle)
    updatePlatformMesh();
    updateLegPositions();
    updateHandAxis(handExtensionMM);

    // Update CAD models if available
    updateCADParts();
}

export { initGhostOverlay, setCADMode, isCADAvailable, isCADModeEnabled };
