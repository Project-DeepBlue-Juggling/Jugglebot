/** Ball Butler CAD. Local robot frame is X right, Y forward, Z up.
 *
 * WHERE BB stands comes from the bb/calibration_result message, NOT from the
 * QTM rigid body: mocap_node uses QTM only to label which markers belong to BB,
 * and locates BB by the calibration procedure. Until a successful calibration
 * is known (initial, websocket drop, success:false) the model is a translucent
 * "ghost" that tumbles around Jugglebot so an uncalibrated BB is unmistakable;
 * a successful result glides it to the calibrated pose and makes it solid.
 * The joints (yaw / pitch / hand) are driven independently by the heartbeat.
 */
import * as THREE from 'three';
import { scene, sceneGroups, robotToThreeScaled, onFrame } from './viewer.js';
import { BB_POSITION_MM, BB_YAW_S_OFFSET_MM, BB_PITCH_D_OFFSET_MM, BB_PITCH_Z_OFFSET_MM } from './geometry-config.js';
import { cadMesh, setAxisFault, setAxisHighlight } from './robot-meshes.js';

let bbGroup, yawGroup, pitchGroup, hand, pitch;
const DEG2RAD = Math.PI / 180;

// ---- Calibrated placement / ghost state ----
// Frame math (derived from throw_ballistics.bb_release_state; verified
// numerically): the CAD throws along three/robot +Y when yawGroup.rotation.y
// is 0, i.e. azimuth 90 deg in the bbGroup frame, and the FK throws along
// (yaw + yaw_offset_rad) CCW from world +X, so the group heading is
// yaw_offset - 90 deg. position_mm is the point on the yaw axis at PITCH-axis
// height, while the CAD origin sits BB_PITCH_Z_OFFSET_MM below the pitch axis.
// NOTE: the CAD hand mesh sits |s| to the LEFT of the throw direction, as the
// real hand does (owner, 2026-10-05); throw_ballistics' FK puts it on the RIGHT.
// That is an open question for the aim model, not this file - see logbook
// 2026-10-05-gui-juggle-panel-bb-calibrated-ghost-chart-dots.
const GHOST_ORBIT_PERIOD_S = 25;      // one lap around the robot
const GHOST_ORBIT_R_MM = 700;         // roughly where BB really lives
const GHOST_HOVER_MM = 300;
const GHOST_OPACITY = 0.35;
const GLIDE_S = 1.25;                 // ghost <-> calibrated transition

const bbMaterials = [];               // BB's own materials only (never the Stewart's)
let calibrated = false;
let blend = 0;                        // 0 = full ghost, 1 = full calibrated
let calibPos = new THREE.Vector3();   // current (possibly gliding) calibrated pose
let calibQuat = new THREE.Quaternion();
let glideFromPos = new THREE.Vector3(), glideFromQuat = new THREE.Quaternion();
let glideToPos = new THREE.Vector3(), glideToQuat = new THREE.Quaternion();
let glideT = 1;                       // 1 = calibPos is at its target
let lastFrameMs = 0;
let settled = false;                  // solid + still: the frame hook does no work
const ghostPos = new THREE.Vector3(), ghostQuat = new THREE.Quaternion();
const ghostEuler = new THREE.Euler(0, 0, 0, 'YXZ');
const smooth = t => t * t * (3 - 2 * t);

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
    bbGroup.traverse(o => { if (o.isMesh) bbMaterials.push(o.material); });
    scene.add(bbGroup);
    sceneGroups['Ball Butler'] = bbGroup;
    onFrame(tickPlacement);
}

/** Ghost trajectory: orbit + bob + multi-frequency tumble. Incommensurate
 * frequencies so it never visibly repeats; all slow and C-infinity smooth. */
function ghostPoseAt(t) {
    const ang = t * 2 * Math.PI / GHOST_ORBIT_PERIOD_S;
    const r = GHOST_ORBIT_R_MM + 60 * Math.sin(t * 0.37);
    const z = GHOST_HOVER_MM + 100 * Math.sin(t * 0.9 + 1);
    ghostPos.copy(robotToThreeScaled(r * Math.cos(ang), r * Math.sin(ang), z));
    ghostEuler.set(
        0.5 * Math.sin(t * 0.7) + 0.3 * Math.sin(t * 1.3 + 2),
        t * 0.6 + 0.4 * Math.sin(t * 0.41),
        0.5 * Math.sin(t * 0.53 + 1) + 0.25 * Math.sin(t * 1.1));
    ghostQuat.setFromEuler(ghostEuler);
}

/** Opacity/transparency composes with axis highlight/fault: updateRobotMaterials
 * only writes colour + emissive, never opacity, so the two never fight. */
function setOpacity(op) {
    const solid = op >= 1;
    for (const m of bbMaterials) {
        if (m.transparent === solid || m.opacity !== op) {
            m.transparent = !solid;
            m.depthWrite = solid;
            m.opacity = op;
        }
    }
}

function tickPlacement(nowMs) {
    if (!bbGroup || !bbGroup.visible) { lastFrameMs = 0; return; }
    const dt = lastFrameMs ? Math.min(0.5, (nowMs - lastFrameMs) / 1000) : 0;
    lastFrameMs = nowMs;
    const target = calibrated ? 1 : 0;
    if (blend !== target) {
        // Step toward the target, never past it (target is 0 or 1).
        blend = Math.min(1, Math.max(0, blend + Math.sign(target - blend) * dt / GLIDE_S));
        settled = false;
    }
    if (glideT < 1) {
        glideT = Math.min(1, glideT + dt / GLIDE_S);
        const e = smooth(glideT);
        calibPos.lerpVectors(glideFromPos, glideToPos, e);
        calibQuat.slerpQuaternions(glideFromQuat, glideToQuat, e);
        settled = false;
    }
    if (settled) return;
    const e = smooth(blend);
    if (e < 1) {
        const t = nowMs * 0.001;
        ghostPoseAt(t);
        if (e > 0) {
            bbGroup.position.lerpVectors(ghostPos, calibPos, e);
            bbGroup.quaternion.slerpQuaternions(ghostQuat, calibQuat, e);
        } else {
            bbGroup.position.copy(ghostPos);
            bbGroup.quaternion.copy(ghostQuat);
        }
        const pulse = GHOST_OPACITY + 0.06 * Math.sin(t * 2 * Math.PI / 3.5);
        setOpacity(pulse + (1 - pulse) * e);
    } else {
        bbGroup.position.copy(calibPos);
        bbGroup.quaternion.copy(calibQuat);
        setOpacity(1);
        if (glideT >= 1) settled = true;
    }
}

/** Feed every bb/calibration_result (including the latched one) or null on
 * disconnect. Only success:true places the model; anything else ghosts it.
 * `glide:false` places it solid at once, with no transition frames — for the
 * synchronous self-test page (test_robot_models.html), not the live GUI. */
export function setBallButlerCalibration(msg, { glide = true } = {}) {
    const p = msg && msg.success === true ? msg.position_mm : null;
    const off = msg ? msg.yaw_offset_rad : NaN;
    if (!p || ![p.x, p.y, p.z, off].every(Number.isFinite)) {
        calibrated = false;
        settled = false;
        return;
    }
    const pos = robotToThreeScaled(p.x, p.y, p.z - BB_PITCH_Z_OFFSET_MM);
    const quat = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), off - Math.PI / 2);
    if (calibrated && blend >= 1) {
        // Already placed: glide from where it is now to the new pose.
        glideFromPos.copy(calibPos); glideFromQuat.copy(calibQuat);
        glideToPos.copy(pos); glideToQuat.copy(quat);
        glideT = 0;
    } else {
        calibPos.copy(pos); calibQuat.copy(quat);
        glideToPos.copy(pos); glideToQuat.copy(quat);
        glideT = 1;
    }
    calibrated = true;
    settled = false;
    if (!glide && bbGroup) {
        blend = 1;
        glideT = 1;
        calibPos.copy(pos); calibQuat.copy(quat);
        bbGroup.position.copy(pos);
        bbGroup.quaternion.copy(quat);
        setOpacity(1);
        settled = true;
    }
}

export function getBallButlerPickables() { return pitch ? [pitch, hand] : []; }

export function updateBallButler(yawDeg, pitchDeg, handPosMM) {
    if (!yawGroup || ![yawDeg, pitchDeg, handPosMM].every(Number.isFinite)) return;
    yawGroup.rotation.y = yawDeg * DEG2RAD;
    // CAD rail is vertical at rest. Positive pitch aims toward robot +Y.
    pitchGroup.rotation.x = (pitchDeg - 90) * DEG2RAD;
    hand.position.set(BB_YAW_S_OFFSET_MM * .001, handPosMM * .001, 0);
}

export function setBallButlerVisible(visible) { if (bbGroup) bbGroup.visible = visible; }
export function setBallButlerHighlight(target) {
    setAxisHighlight(7, target === 'pitch');
    setAxisHighlight(8, target === 'hand');
}
export function setBBPitchFault(faulted) { setAxisFault(7, faulted); }
export function setBBHandFault(faulted) { setAxisFault(8, faulted); }
