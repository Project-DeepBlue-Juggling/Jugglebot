/**
 * fk_harness.js — runs ros_ws/gui/js/stewart-fk.js against the Python golden
 * vectors and prints the results as JSON on stdout.
 *
 * Driven by tests/ros/test_gui_fk_golden.py, which copies this file next to
 * verbatim copies of `stewart-fk.js` and `geometry-config.js` in a temp dir
 * (plus a `package.json` marking the dir as ESM — the GUI sources are ES
 * modules but live in a directory with no package.json, so node would
 * otherwise parse them as CommonJS and choke on `export`).
 *
 * Usage:  node fk_harness.js <golden.json>
 *
 * Nothing is asserted here; all comparison happens in Python so failures read
 * as pytest diffs.
 *
 * Scope, stated so nobody over-reads a green run:
 *   - `resetFKState()` runs before every vector, so only the COLD-start FK path
 *     is pinned. The live GUI runs warm-started (`prevState` is module-level in
 *     stewart-fk.js); a warm-start-specific divergence is out of scope here.
 *     The reset is deliberate — without it each vector's answer would depend on
 *     which vector ran before it, and the golden set would not be reproducible.
 *   - `quatToRotMatrix()` and `poseToPlatNodes()` — the mocap-driven path in the
 *     same module — are exported but NOT exercised by these vectors.
 */

import { readFileSync } from 'node:fs';
import {
    rotvecToRotMatrix,
    rotMatrixToRotvec,
    poseToLegLengths,
    solveFK,
    legLengthsToPose,
    resetFKState,
} from './stewart-fk.js';

function main() {
    const goldenPath = process.argv[2];
    if (!goldenPath) {
        process.stderr.write('usage: node fk_harness.js <golden.json>\n');
        process.exit(2);
    }
    const doc = JSON.parse(readFileSync(goldenPath, 'utf8'));

    const results = [];
    for (const v of doc.vectors) {
        // --- IK: pose -> leg extensions (the hand-ported half) ---
        const R = rotvecToRotMatrix(v.rotvec_rad);
        const ik = poseToLegLengths(v.pos_mm, R);

        // --- FK: extensions -> pose.  Reset the warm-start every time so each
        // vector is solved from the same cold guess and the result cannot
        // depend on the order of the sweep. ---
        resetFKState();
        const fkExt = solveFK(v.extensions_mm);

        // --- FK through the revs entry point (exercises the MM_TO_REV
        // division, i.e. the unit convention most likely to invert). ---
        resetFKState();
        const fkRev = legLengthsToPose(v.motor_revs);

        results.push({
            label: v.label,
            ik_extensions_mm: ik.extensions,
            rot_matrix: R,
            fk_from_extensions: {
                pos_mm: fkExt.pos,
                rot_matrix: fkExt.R,
                rotvec_rad: rotMatrixToRotvec(fkExt.R),
                converged: fkExt.converged,
            },
            fk_from_revs: {
                pos_mm: fkRev.pos,
                rot_matrix: fkRev.R,
                rotvec_rad: rotMatrixToRotvec(fkRev.R),
                converged: fkRev.converged,
            },
        });
    }

    process.stdout.write(JSON.stringify({
        node_version: process.version,
        results,
    }));
}

main();
