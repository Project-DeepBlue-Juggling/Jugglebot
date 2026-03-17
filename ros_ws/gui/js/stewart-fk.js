/**
 * stewart-fk.js — Forward Kinematics solver for the Stewart platform.
 *
 * Ports the IK from motion/ik_solver.py and adds a Newton-Raphson FK solver.
 * All geometry in mm, rotation as rotation vectors (Rodrigues).
 */

import {
    BASE_NODES_MM, INIT_PLAT_NODES_MM, INIT_LEG_LENGTHS_MM,
    INITIAL_HEIGHT_MM, MM_TO_REV, LEG_STROKE_MM,
} from './geometry-config.js';

// ---- Rotation utilities ----

/**
 * Rodrigues formula: rotation vector → 3x3 rotation matrix.
 * @param {number[]} rv - [rx, ry, rz] rotation vector (axis * angle)
 * @returns {number[][]} 3x3 rotation matrix (row-major)
 */
export function rotvecToRotMatrix(rv) {
    const angle = Math.sqrt(rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2]);
    if (angle < 1e-12) {
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    }
    const k = [rv[0] / angle, rv[1] / angle, rv[2] / angle];
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    const t = 1 - c;

    return [
        [t * k[0] * k[0] + c,        t * k[0] * k[1] - s * k[2], t * k[0] * k[2] + s * k[1]],
        [t * k[0] * k[1] + s * k[2], t * k[1] * k[1] + c,        t * k[1] * k[2] - s * k[0]],
        [t * k[0] * k[2] - s * k[1], t * k[1] * k[2] + s * k[0], t * k[2] * k[2] + c       ],
    ];
}

/**
 * Rotation matrix → rotation vector (inverse Rodrigues).
 * @param {number[][]} R - 3x3 rotation matrix
 * @returns {number[]} [rx, ry, rz]
 */
export function rotMatrixToRotvec(R) {
    const cosAngle = (R[0][0] + R[1][1] + R[2][2] - 1) / 2;
    const clampedCos = Math.max(-1, Math.min(1, cosAngle));
    const angle = Math.acos(clampedCos);

    if (angle < 1e-12) {
        return [0, 0, 0];
    }

    const s = 2 * Math.sin(angle);
    return [
        (R[2][1] - R[1][2]) / s * angle,
        (R[0][2] - R[2][0]) / s * angle,
        (R[1][0] - R[0][1]) / s * angle,
    ];
}

// ---- Linear algebra helpers ----

function matVec3(M, v) {
    return [
        M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
        M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
        M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2],
    ];
}

function vecSub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function vecAdd(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
function vecNorm(v) { return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

/**
 * Solve a 6x6 linear system Ax = b using Gaussian elimination with partial pivoting.
 * Modifies A and b in place. Returns x.
 */
function solve6x6(A, b) {
    const n = 6;
    // Forward elimination
    for (let col = 0; col < n; col++) {
        // Pivot
        let maxRow = col;
        let maxVal = Math.abs(A[col][col]);
        for (let row = col + 1; row < n; row++) {
            if (Math.abs(A[row][col]) > maxVal) {
                maxVal = Math.abs(A[row][col]);
                maxRow = row;
            }
        }
        if (maxRow !== col) {
            [A[col], A[maxRow]] = [A[maxRow], A[col]];
            [b[col], b[maxRow]] = [b[maxRow], b[col]];
        }
        if (Math.abs(A[col][col]) < 1e-14) continue; // singular

        for (let row = col + 1; row < n; row++) {
            const factor = A[row][col] / A[col][col];
            for (let j = col; j < n; j++) {
                A[row][j] -= factor * A[col][j];
            }
            b[row] -= factor * b[col];
        }
    }
    // Back substitution
    const x = new Array(n).fill(0);
    for (let row = n - 1; row >= 0; row--) {
        let sum = b[row];
        for (let j = row + 1; j < n; j++) {
            sum -= A[row][j] * x[j];
        }
        x[row] = Math.abs(A[row][row]) > 1e-14 ? sum / A[row][row] : 0;
    }
    return x;
}

// ---- Inverse Kinematics ----

/**
 * IK: given a platform pose (pos relative to home, rotation matrix), compute leg extensions (mm).
 * @param {number[]} pos - [x, y, z] platform translation from home (mm)
 * @param {number[][]} R - 3x3 rotation matrix
 * @returns {{ extensions: number[], platNodes: number[][] }} leg extensions and world-frame platform nodes
 */
export function poseToLegLengths(pos, R) {
    const newPos = [pos[0], pos[1], pos[2] + INITIAL_HEIGHT_MM];
    const platNodes = [];
    const extensions = [];

    for (let i = 0; i < 6; i++) {
        const rotated = matVec3(R, INIT_PLAT_NODES_MM[i]);
        const worldNode = vecAdd(newPos, rotated);
        platNodes.push(worldNode);

        const legVec = vecSub(worldNode, BASE_NODES_MM[i]);
        const absLen = vecNorm(legVec);
        extensions.push(absLen - INIT_LEG_LENGTHS_MM[i]);
    }

    return { extensions, platNodes };
}

// ---- Jacobian ----

/**
 * Compute the 6x6 Jacobian: d(leg_lengths) / d(state), where state = [px, py, pz, rx, ry, rz].
 * Uses finite differences.
 */
function computeJacobian(pos, rotvec) {
    const eps = 1e-6;
    const R0 = rotvecToRotMatrix(rotvec);
    const { extensions: L0 } = poseToLegLengths(pos, R0);

    const J = Array.from({ length: 6 }, () => new Array(6));

    // Partials w.r.t. position (columns 0-2)
    for (let j = 0; j < 3; j++) {
        const posPlus = [...pos];
        posPlus[j] += eps;
        const { extensions: Lp } = poseToLegLengths(posPlus, R0);
        for (let i = 0; i < 6; i++) {
            J[i][j] = (Lp[i] - L0[i]) / eps;
        }
    }

    // Partials w.r.t. rotation (columns 3-5)
    for (let j = 0; j < 3; j++) {
        const rvPlus = [...rotvec];
        rvPlus[j] += eps;
        const Rp = rotvecToRotMatrix(rvPlus);
        const { extensions: Lp } = poseToLegLengths(pos, Rp);
        for (let i = 0; i < 6; i++) {
            J[i][j + 3] = (Lp[i] - L0[i]) / eps;
        }
    }

    return J;
}

// ---- Forward Kinematics (Newton-Raphson) ----

/** Previous solution for warm-starting */
let prevState = [0, 0, 0, 0, 0, 0];

/**
 * FK: given motor positions (revolutions), solve for platform pose.
 * Uses Newton-Raphson iteration with warm-start from previous solution.
 *
 * Motor convention: positive revs = extension (motion_bridge_node publishes
 * positive revs via leg_lengths_topic, and ODrive reports positive pos_estimate
 * for extended legs after homing sets leg_abs_pos_rev = 0.1).
 *
 * @param {number[]} motorRevs - 6 motor positions in revolutions (positive = extension)
 * @returns {{ pos: number[], R: number[][], platNodes: number[][], converged: boolean }}
 */
export function legLengthsToPose(motorRevs) {
    // Convert motor revs to extensions in mm: revs / mm_to_rev
    const targetExtensions = new Array(6);
    for (let i = 0; i < 6; i++) {
        targetExtensions[i] = motorRevs[i] / MM_TO_REV[i];
    }

    return solveFK(targetExtensions);
}

/**
 * FK from leg extensions in mm.
 * @param {number[]} targetExtensions - 6 leg extensions in mm
 * @returns {{ pos: number[], R: number[][], platNodes: number[][], converged: boolean }}
 */
export function solveFK(targetExtensions) {
    const maxIter = 15;
    const tol = 1e-6;

    // State: [px, py, pz, rx, ry, rz]
    let state = [...prevState];

    let converged = false;
    for (let iter = 0; iter < maxIter; iter++) {
        const pos = [state[0], state[1], state[2]];
        const rv = [state[3], state[4], state[5]];
        const R = rotvecToRotMatrix(rv);
        const { extensions } = poseToLegLengths(pos, R);

        // Residual = current - target
        const residual = new Array(6);
        let maxResidual = 0;
        for (let i = 0; i < 6; i++) {
            residual[i] = extensions[i] - targetExtensions[i];
            maxResidual = Math.max(maxResidual, Math.abs(residual[i]));
        }

        if (maxResidual < tol) {
            converged = true;
            break;
        }

        // Compute Jacobian and solve J * dx = -residual
        const J = computeJacobian(pos, rv);
        const negResidual = residual.map(r => -r);
        const dx = solve6x6(J, negResidual);

        // Update state
        for (let i = 0; i < 6; i++) {
            state[i] += dx[i];
        }
    }

    // Store for warm-start
    if (converged) {
        prevState = [...state];
    }

    const pos = [state[0], state[1], state[2]];
    const rv = [state[3], state[4], state[5]];
    const R = rotvecToRotMatrix(rv);
    const { platNodes } = poseToLegLengths(pos, R);

    return { pos, R, platNodes, converged };
}

/**
 * Reset the warm-start state (e.g., when the robot is rehomed).
 */
export function resetFKState() {
    prevState = [0, 0, 0, 0, 0, 0];
}

// ---- Quaternion utilities (for mocap data) ----

/**
 * Convert a quaternion (w, x, y, z) to a 3x3 rotation matrix.
 * @param {number} w
 * @param {number} x
 * @param {number} y
 * @param {number} z
 * @returns {number[][]} 3x3 rotation matrix
 */
export function quatToRotMatrix(w, x, y, z) {
    const len = Math.sqrt(w * w + x * x + y * y + z * z);
    if (len < 1e-12) return [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    const nw = w / len, nx = x / len, ny = y / len, nz = z / len;

    return [
        [1 - 2 * (ny * ny + nz * nz), 2 * (nx * ny - nw * nz),     2 * (nx * nz + nw * ny)    ],
        [2 * (nx * ny + nw * nz),     1 - 2 * (nx * nx + nz * nz), 2 * (ny * nz - nw * nx)    ],
        [2 * (nx * nz - nw * ny),     2 * (ny * nz + nw * nx),     1 - 2 * (nx * nx + ny * ny)],
    ];
}

/**
 * Compute platform node positions from a global pose (position + rotation matrix).
 * Used for mocap-driven updates where pose is measured directly.
 *
 * @param {number[]} globalPos - [x, y, z] global position of platform centre (mm)
 * @param {number[][]} R - 3x3 rotation matrix (global orientation)
 * @returns {number[][]} 6 platform node positions in global frame
 */
export function poseToPlatNodes(globalPos, R) {
    const platNodes = [];
    for (let i = 0; i < 6; i++) {
        const rotated = matVec3(R, INIT_PLAT_NODES_MM[i]);
        platNodes.push(vecAdd(globalPos, rotated));
    }
    return platNodes;
}
