/**
 * stewart-model.js — Stewart platform 3D geometry.
 *
 * Creates base hexagon, 6 legs, platform hexagon, and hand axis.
 * Always updated from FK (motor feedback).
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
/** Last-applied hand extension, cached so highlight updates can re-apply geometry. */
let currentHandExtensionMM = 0;

/** Per-leg radial scale multiplier (1 normally, >1 while highlighted). */
const legRadialScale = [1, 1, 1, 1, 1, 1];
/** Hand axis radial scale multiplier. */
let handRadialScale = 1;

/** Per-leg fault phase: null | 'new' (pulsing, ~2 s) | 'persistent' (steady).
 *  `handFault` mirrors the same state for the hand axis (motor 6). */
const legFaultState = [null, null, null, null, null, null];
const legFaultStartMs = [0, 0, 0, 0, 0, 0];
let handFaultState = null;
let handFaultStartMs = 0;
/** rAF handle for the fault-pulse animation loop — null when idle. */
let faultPulseRAF = null;

const FAULT_COLOR_HEX = 0xef4444;
const FAULT_NEW_DURATION_MS = 2000;

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

    // ---- Base hexagon ----
    baseMesh = createHexMesh(BASE_NODES_MM, BASE_COLOR, 0.15);
    platformGroup.add(baseMesh);

    // ---- Platform hexagon (at home position) ----
    const homePlatNodes = INIT_PLAT_NODES_MM.map(n => [n[0], n[1], n[2] + INITIAL_HEIGHT_MM]);
    platformMesh = createHexMesh(homePlatNodes, PLAT_COLOR, 0.2);
    platformGroup.add(platformMesh);

    // ---- Legs (cylinders between base and platform nodes) ----
    // Each leg gets its own material so it can be colour-cycled by extension
    // *and* toggled into a highlighted (emissive) state independently.
    for (let i = 0; i < 6; i++) {
        const leg = createLegCylinder();
        // chartIdx links the mesh back to its chart-panel cell for the
        // click-in-3D-to-highlight-chart feature.
        leg.userData.chartIdx = i;
        legMeshes.push(leg);
        platformGroup.add(leg);
    }

    // Position legs at home
    currentPlatNodes = homePlatNodes;
    currentPlatCentre = [0, 0, INITIAL_HEIGHT_MM];
    updateLegPositions();

    // ---- Hand axis (cylinder from platform centre along normal) ----
    // Use a cylinder for visibility (WebGL Line linewidth is always 1px)
    const handGeom = new THREE.CylinderGeometry(0.006, 0.006, 1, 8);
    handLine = new THREE.Mesh(
        handGeom,
        new THREE.MeshStandardMaterial({ color: HAND_COLOR, emissive: HAND_COLOR, emissiveIntensity: 0.3 })
    );
    handLine.userData.chartIdx = 6;  // Hand motor
    platformGroup.add(handLine);
    updateHandAxis(0); // home position

    scene.add(platformGroup);
    sceneGroups['Platform'] = platformGroup;
}

/**
 * Return the set of meshes that should be pickable for the
 * click-in-3D-to-highlight-chart feature.  Each mesh carries
 * `userData.chartIdx` pointing back to its chart cell index.
 */
export function getStewartPickables() {
    return handLine ? [...legMeshes, handLine] : [...legMeshes];
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
 * radialScale multiplies the cross-section radius (x/z scale) so a caller can
 * visually fatten a cylinder — used by the highlight API without needing a
 * separate mesh path.
 */
function positionCylinder(mesh, p1, p2, radialScale = 1) {
    const mid = new THREE.Vector3().addVectors(p1, p2).multiplyScalar(0.5);
    mesh.position.copy(mid);

    const dir = new THREE.Vector3().subVectors(p2, p1);
    const len = dir.length();
    mesh.scale.set(radialScale, len, radialScale);

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
        positionCylinder(legMeshes[i], p1, p2, legRadialScale[i]);

        // Color by extension ratio — unless the leg is faulted, in which
        // case the fault-pulse loop owns the colour and we skip here.
        if (!legFaultState[i]) {
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
    positionCylinder(handLine, base3, tip3, handRadialScale);
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
    currentHandExtensionMM = handExtensionMM;

    updatePlatformMesh();
    updateLegPositions();
    updateHandAxis(handExtensionMM);
}

/**
 * Highlight a single Stewart-platform element (leg 0..5 or the hand axis) so
 * it pops visually in the 3D scene.  Pass `null` to clear.
 *
 * @param {null | 'leg0' | 'leg1' | 'leg2' | 'leg3' | 'leg4' | 'leg5' | 'hand'} target
 */
export function setStewartHighlight(target) {
    for (let i = 0; i < 6; i++) {
        const isHi = target === `leg${i}`;
        legRadialScale[i] = isHi ? 2.4 : 1;
        // Faulted legs own their own emissive/colour via the pulse loop —
        // leave those properties alone so highlight doesn't stomp the red.
        if (legFaultState[i]) continue;
        const mat = legMeshes[i]?.material;
        if (mat) {
            mat.emissive.setHex(isHi ? 0xffffff : 0x000000);
            mat.emissiveIntensity = isHi ? 0.6 : 0;
        }
    }
    const isHandHi = target === 'hand';
    handRadialScale = isHandHi ? 2.4 : 1;
    if (handLine && handFaultState == null) {
        handLine.material.emissive.setHex(HAND_COLOR);
        handLine.material.emissiveIntensity = isHandHi ? 1.2 : 0.3;
    }

    // Re-run positioning so the new radial scale is applied immediately,
    // even if no telemetry frame arrives for a while.
    if (currentPlatNodes) {
        updateLegPositions();
        updateHandAxis(currentHandExtensionMM);
    }
}

// ---- Fault visualisation ------------------------------------------------

/**
 * Animation loop for fault pulses.  A "new" fault oscillates the emissive
 * intensity for FAULT_NEW_DURATION_MS before falling back to a steady red
 * glow while the fault persists.  The loop self-exits when no leg/hand has
 * an active fault.
 */
function runFaultPulseLoop() {
    if (faultPulseRAF != null) return;
    const tick = () => {
        const now = performance.now();
        let anyActive = false;

        for (let i = 0; i < 6; i++) {
            if (!legFaultState[i] || !legMeshes[i]) continue;
            anyActive = true;
            const mat = legMeshes[i].material;
            if (legFaultState[i] === 'new') {
                const elapsed = now - legFaultStartMs[i];
                if (elapsed > FAULT_NEW_DURATION_MS) {
                    legFaultState[i] = 'persistent';
                    mat.emissiveIntensity = 0.7;
                } else {
                    const pulse = 0.5 + 0.5 * Math.sin(elapsed / 125);  // ~1.3 Hz
                    mat.emissiveIntensity = 0.35 + pulse * 0.9;
                }
            } else {
                mat.emissiveIntensity = 0.7;
            }
        }

        if (handFaultState && handLine) {
            anyActive = true;
            const mat = handLine.material;
            if (handFaultState === 'new') {
                const elapsed = now - handFaultStartMs;
                if (elapsed > FAULT_NEW_DURATION_MS) {
                    handFaultState = 'persistent';
                    mat.emissiveIntensity = 0.9;
                } else {
                    const pulse = 0.5 + 0.5 * Math.sin(elapsed / 125);
                    mat.emissiveIntensity = 0.45 + pulse * 1.1;
                }
            } else {
                mat.emissiveIntensity = 0.9;
            }
        }

        faultPulseRAF = anyActive ? requestAnimationFrame(tick) : null;
    };
    faultPulseRAF = requestAnimationFrame(tick);
}

/**
 * Set the fault state for a single Stewart leg (0..5).  Rising edge starts
 * a 2 s "new" pulse; clearing (`faulted=false`) restores normal colouring
 * on the next telemetry frame.
 *
 * @param {number} legIdx   – 0..5
 * @param {boolean} faulted – whether the motor has active_errors or
 *                            disarm_reason non-zero
 */
export function setLegFault(legIdx, faulted) {
    if (legIdx < 0 || legIdx >= 6) return;
    const prev = legFaultState[legIdx];
    if (faulted && !prev) {
        legFaultState[legIdx] = 'new';
        legFaultStartMs[legIdx] = performance.now();
        const mesh = legMeshes[legIdx];
        if (mesh) {
            mesh.material.color.setHex(FAULT_COLOR_HEX);
            mesh.material.emissive.setHex(FAULT_COLOR_HEX);
            mesh.material.emissiveIntensity = 0.35;
        }
        runFaultPulseLoop();
    } else if (!faulted && prev) {
        legFaultState[legIdx] = null;
        const mesh = legMeshes[legIdx];
        if (mesh) {
            mesh.material.emissive.setHex(0x000000);
            mesh.material.emissiveIntensity = 0;
        }
        // Leg color will be restored to the extension-ratio hue on the next
        // updateLegPositions() call (triggered by the next telemetry frame).
    }
}

/** Fault API for the Stewart hand axis (motor 6).  Same semantics as
 *  setLegFault — rising edge pulses, persistent state is a steady glow. */
export function setHandFault(faulted) {
    const prev = handFaultState;
    if (faulted && !prev) {
        handFaultState = 'new';
        handFaultStartMs = performance.now();
        if (handLine) {
            handLine.material.color.setHex(FAULT_COLOR_HEX);
            handLine.material.emissive.setHex(FAULT_COLOR_HEX);
            handLine.material.emissiveIntensity = 0.45;
        }
        runFaultPulseLoop();
    } else if (!faulted && prev) {
        handFaultState = null;
        if (handLine) {
            handLine.material.color.setHex(HAND_COLOR);
            handLine.material.emissive.setHex(HAND_COLOR);
            handLine.material.emissiveIntensity = 0.3;
        }
    }
}

