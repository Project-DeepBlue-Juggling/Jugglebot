/**
 * ball-butler-model.js — Ball Butler 3D geometry.
 *
 * Renders the Ball Butler with:
 *   - Yaw arc:    range indicator in the horizontal (robot XY) plane, +/-30 deg from Y
 *   - Pitch arc:  range indicator in the vertical (robot YZ) plane, ~5 deg to 100 deg
 *   - Throw axis: tube through the pitch pivot, angled by current pitch
 *   - Hand marker: sphere sliding along the throw axis
 *
 * All three axes are solid white; pedestal stays grey.
 * Coordinate convention (inside bbGroup):
 *   Robot frame: X = right, Y = forward, Z = up
 *   Three.js:    X = right, Y = up,      Z = towards viewer
 *   Mapping:     robot(x, y, z) -> Three(x, z, -y)
 *
 * Updates from bb/heartbeat topic data.
 */

import * as THREE from 'three';
import { scene, sceneGroups } from './viewer.js';
import {
    BB_POSITION_MM,
    BB_YAW_S_OFFSET_MM,
    BB_PITCH_D_OFFSET_MM,
    BB_HAND_STROKE_MM,
} from './geometry-config.js';

const DEG2RAD = Math.PI / 180;
const S = 0.001; // mm -> metres (Three.js scene units)

/** Convert robot-frame mm to Three.js local coords (metres). */
function r2t(rx, ry, rz) {
    return new THREE.Vector3(rx * S, rz * S, -ry * S);
}

// ---- Scene objects ----
let bbGroup;
let yawGroup;
let pitchGroup;
let throwTube;
let handSphere;

// ---- Visual parameters (mm) ----
const PEDESTAL_HEIGHT   = 66.5;
const PEDESTAL_RADIUS   = 125;
const YAW_ARC_RADIUS    = PEDESTAL_RADIUS + 5;  // 130 mm
const PITCH_ARC_RADIUS  = 150;
const ARC_SEGMENTS      = 48;
const ARC_TUBE_R        = 2.5;  // tube cross-section radius
const THROW_TUBE_R      = 2.0;  // slightly thinner for the linear axis
const HAND_SPHERE_R     = 8;

// ---- Colours ----
const COL_PEDESTAL = 0xb0b0b0;  // light grey
const COL_AXIS     = 0xffffff;  // white for all three axes
const COL_HAND     = 0xf59e0b;  // amber

// ---- Pitch pivot in robot coords (inside yawGroup) ----
// s active negatively in local x, d active negatively in local y
const PIVOT_X =  BB_YAW_S_OFFSET_MM;   // -105.65
const PIVOT_Y = -BB_PITCH_D_OFFSET_MM; // -41.0
const PIVOT_Z =  0;

// Throw axis line is offset from hand sphere in local x
const THROW_LINE_X_OFFSET = 80;  // mm

// ---- Tube helpers ----
const RADIAL_SEGS = 8;  // cross-section segments for all tubes

/**
 * Build an arc as an array of THREE.Vector3 (in Three.js local metres).
 *
 * angle convention: 0 deg = along robot +Y (forward), increasing toward +X (xy) or +Z (yz).
 */
function arcPts(r, aDeg, bDeg, n, plane, off) {
    const pts = [];
    for (let i = 0; i <= n; i++) {
        const a = (aDeg + (bDeg - aDeg) * (i / n)) * DEG2RAD;
        let p;
        if (plane === 'xy') {
            p = r2t(r * Math.sin(a), r * Math.cos(a), 0);
        } else {
            p = r2t(0, r * Math.cos(a), r * Math.sin(a));
        }
        if (off) p.add(off);
        pts.push(p);
    }
    return pts;
}

/** Shared material for all axis tubes. */
const axisMat = new THREE.MeshStandardMaterial({
    color: COL_AXIS,
    transparent: true,
    opacity: 0.7,
});

/** Create a solid tube mesh following an arc (CatmullRomCurve3). */
function arcTube(pts, tubeR) {
    const curve = new THREE.CatmullRomCurve3(pts, false, 'catmullrom', 0.0);
    const geom = new THREE.TubeGeometry(curve, pts.length * 2, tubeR * S, RADIAL_SEGS, false);
    return new THREE.Mesh(geom, axisMat);
}

/** Create a solid tube mesh along a straight line (LineCurve3). */
function lineTube(from, to, tubeR, segs) {
    const curve = new THREE.LineCurve3(from, to);
    const geom = new THREE.TubeGeometry(curve, segs, tubeR * S, RADIAL_SEGS, false);
    return new THREE.Mesh(geom, axisMat);
}

// ====================================================================
// Public API
// ====================================================================

/**
 * Initialise the Ball Butler 3D model and add it to the scene.
 */
export function initBallButlerModel() {
    bbGroup = new THREE.Group();
    bbGroup.name = 'ball-butler';

    // Position at configured world offset from base centre
    bbGroup.position.copy(r2t(BB_POSITION_MM[0], BB_POSITION_MM[1], BB_POSITION_MM[2]));

    // ---- Pedestal cylinder ----
    const pedGeom = new THREE.CylinderGeometry(
        PEDESTAL_RADIUS * S, PEDESTAL_RADIUS * S,
        PEDESTAL_HEIGHT * S, 24,
    );
    const pedMat = new THREE.MeshStandardMaterial({
        color: COL_PEDESTAL, transparent: true, opacity: 0.35,
    });
    const pedestal = new THREE.Mesh(pedGeom, pedMat);
    pedestal.position.y = PEDESTAL_HEIGHT * 0.5 * S;
    bbGroup.add(pedestal);

    // Everything above the pedestal lives at pedestal-top height
    const topY = PEDESTAL_HEIGHT * S;

    // ---- Yaw group (rotates around Three.js Y = robot Z) ----
    yawGroup = new THREE.Group();
    yawGroup.position.y = topY;
    bbGroup.add(yawGroup);

    // ---- Yaw arc: +/-30 deg from robot +Y, in robot XY plane ----
    const yawArc = arcTube(
        arcPts(YAW_ARC_RADIUS, -30, 30, ARC_SEGMENTS, 'xy'),
        ARC_TUBE_R,
    );
    yawGroup.add(yawArc);

    // ---- Pitch arc: 5 deg to 100 deg, in robot YZ plane ----
    // Centred on BB's local Y axis (robot x = 0)
    const pivotThree = r2t(PIVOT_X, PIVOT_Y, PIVOT_Z);
    const pitchArcOffset = pivotThree.clone();
    pitchArcOffset.x = 0;

    const pitchArc = arcTube(
        arcPts(PITCH_ARC_RADIUS, 5, 100, ARC_SEGMENTS, 'yz', pitchArcOffset),
        ARC_TUBE_R,
    );
    yawGroup.add(pitchArc);

    // ---- Pitch group: origin at pivot, rotates around Three.js X ----
    pitchGroup = new THREE.Group();
    pitchGroup.position.copy(pivotThree);
    yawGroup.add(pitchGroup);

    // ---- Throw axis tube ----
    // In pitchGroup local frame the throw direction is -Z (= robot +Y = forward).
    // Pitch rotation around X tilts it upward.
    const throwLen = BB_HAND_STROKE_MM * S;
    const throwX = THROW_LINE_X_OFFSET * S;
    throwTube = lineTube(
        new THREE.Vector3(throwX, 0, 0),
        new THREE.Vector3(throwX, 0, -throwLen),
        THROW_TUBE_R, 32,
    );
    pitchGroup.add(throwTube);

    // ---- Hand sphere ----
    const handGeom = new THREE.SphereGeometry(HAND_SPHERE_R * S, 12, 12);
    const handMat = new THREE.MeshStandardMaterial({
        color: COL_HAND,
        emissive: COL_HAND,
        emissiveIntensity: 0.35,
    });
    handSphere = new THREE.Mesh(handGeom, handMat);
    pitchGroup.add(handSphere);

    scene.add(bbGroup);
    sceneGroups['Ball Butler'] = bbGroup;
}

/**
 * Update the Ball Butler model from heartbeat data.
 * @param {number} yawDeg    - Current yaw angle (degrees)
 * @param {number} pitchDeg  - Current pitch angle (degrees, 0 = horizontal, 90 = vertical)
 * @param {number} handPosMM - Hand position along throw axis (mm)
 */
export function updateBallButler(yawDeg, pitchDeg, handPosMM) {
    if (!yawGroup) return;

    // Yaw: rotate around Three.js Y axis
    yawGroup.rotation.y = yawDeg * DEG2RAD;

    // Pitch: rotate around Three.js X axis
    pitchGroup.rotation.x = pitchDeg * DEG2RAD;

    // Hand: slide along local -Z (throw direction) in pitchGroup
    const dist = Math.max(0, handPosMM) * S;
    handSphere.position.set(0, 0, -dist);
}

/**
 * Show/hide the Ball Butler model.
 * @param {boolean} visible
 */
export function setBallButlerVisible(visible) {
    if (bbGroup) bbGroup.visible = visible;
}
