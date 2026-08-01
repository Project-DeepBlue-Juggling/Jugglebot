#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate the JS-vs-Python kinematics golden vectors for the GUI FK solver.

``ros_ws/gui/js/stewart-fk.js`` is the repo's only *duplicate* kinematics
implementation — a hand-port of ``jugglebot.motion.ik_solver``'s IK plus its own
Newton-Raphson FK.  Until now it was pinned only by a manual browser page
(``ros_ws/gui/test_fk.html``), so a divergence would surface as a wrong 3D
platform pose on the dashboard with nothing red anywhere.

**Python is ground truth.**  This script evaluates the production Python IK over
a deterministic sweep of poses and writes the results to
``tests/ros/gui_fk_golden.json``; ``tests/ros/test_gui_fk_golden.py`` replays the
same vectors through the JS implementation under node and compares.

Units, both sides (verified against both sources before the vectors were built):

* ``pos``  — platform-centre offset from the STOW pose, mm.  The absolute
  centre is ``pos + [0, 0, INITIAL_HEIGHT_MM]``; ``pos = 0`` gives zero leg
  extension.
* ``rotvec`` — Rodrigues rotation vector, radians, platform -> base.
* ``extensions_mm`` — per-leg extension, 0 = retracted, ``leg_stroke_mm`` = full.
* ``motor_revs`` — ``extensions_mm * mm_to_rev``, per leg (positive = extension).
  The JS ``legLengthsToPose`` divides by ``MM_TO_REV``, so this leg of the
  round trip is what would catch a rev/mm inversion.

Usage::

    python tools/gen_gui_fk_golden.py            # write the committed JSON
    python tools/gen_gui_fk_golden.py --stdout   # print, write nothing
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, List, Tuple

import numpy as np

_TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_TOOLS_DIR)

for _path in (os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot'),
              os.path.join(_REPO_ROOT, 'config', 'generated'),
              _REPO_ROOT):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from jugglebot.motion.geometry import StewartGeometry          # noqa: E402
from jugglebot.motion.ik_solver import (                       # noqa: E402
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
)

DEFAULT_OUT = os.path.join(_REPO_ROOT, 'tests', 'ros', 'gui_fk_golden.json')

#: Extension margin kept away from both hard stops (mm).  Vectors that would sit
#: outside it are dropped: the FK Newton solve is only meaningful inside the
#: reachable set, and a golden vector at a stop would pin behaviour the robot
#: can never be in.
_EXT_MARGIN_MM = 8.0

#: Deterministic pose sweep: (label, x_mm, y_mm, z_mm, rx, ry, rz).
#: z is the offset above STOW, so 170 mm is the nominal active pose.
#: Rotations are small (<= 0.12 rad ~ 6.9 deg) — the platform's real envelope.
_POSE_SWEEP: Tuple[Tuple[str, float, float, float, float, float, float], ...] = (
    ('stow',                 0.0,    0.0,    0.0,   0.0,   0.0,    0.0),
    ('low-z',                0.0,    0.0,   40.0,   0.0,   0.0,    0.0),
    ('active-home',          0.0,    0.0,  170.0,   0.0,   0.0,    0.0),
    ('high-z',               0.0,    0.0,  250.0,   0.0,   0.0,    0.0),
    ('x-plus',              80.0,    0.0,  170.0,   0.0,   0.0,    0.0),
    ('x-minus',            -80.0,    0.0,  170.0,   0.0,   0.0,    0.0),
    ('y-plus',               0.0,   80.0,  170.0,   0.0,   0.0,    0.0),
    ('y-minus',              0.0,  -80.0,  170.0,   0.0,   0.0,    0.0),
    ('xy-diag',             55.0,   55.0,  170.0,   0.0,   0.0,    0.0),
    ('xy-anti-diag',       -55.0,   55.0,  120.0,   0.0,   0.0,    0.0),
    ('roll-plus',            0.0,    0.0,  170.0,   0.10,  0.0,    0.0),
    ('roll-minus',           0.0,    0.0,  170.0,  -0.10,  0.0,    0.0),
    ('pitch-plus',           0.0,    0.0,  170.0,   0.0,   0.10,   0.0),
    ('pitch-minus',          0.0,    0.0,  170.0,   0.0,  -0.10,   0.0),
    ('yaw-plus',             0.0,    0.0,  170.0,   0.0,   0.0,    0.12),
    ('yaw-minus',            0.0,    0.0,  170.0,   0.0,   0.0,   -0.12),
    ('roll-pitch',           0.0,    0.0,  170.0,   0.07,  0.07,   0.0),
    ('roll-yaw',             0.0,    0.0,  170.0,   0.07,  0.0,    0.07),
    ('pitch-yaw',            0.0,    0.0,  170.0,   0.0,   0.07,   0.07),
    ('full-tilt-low',        0.0,    0.0,   60.0,   0.06,  0.06,   0.06),
    ('full-tilt-high',       0.0,    0.0,  240.0,  -0.06,  0.06,  -0.06),
    ('translate-tilt-a',    60.0,   30.0,  140.0,   0.05, -0.05,   0.03),
    ('translate-tilt-b',   -60.0,  -30.0,  200.0,  -0.05,  0.05,  -0.03),
    ('translate-tilt-c',    40.0,  -70.0,  100.0,   0.08,  0.02,  -0.09),
    ('near-stow-tilt',       0.0,    0.0,   20.0,   0.03, -0.03,   0.0),
    ('near-top-tilt',        0.0,    0.0,  262.0,   0.02,  0.02,   0.0),
)


def build_vectors(geom: StewartGeometry = None) -> List[Dict[str, Any]]:
    """Evaluate the production Python IK over the pose sweep."""
    if geom is None:
        geom = StewartGeometry()

    lower = _EXT_MARGIN_MM
    upper = geom.leg_stroke_mm - _EXT_MARGIN_MM

    vectors: List[Dict[str, Any]] = []
    for label, x, y, z, rx, ry, rz in _POSE_SWEEP:
        pos = np.array([x, y, z], dtype=np.float64)
        rotvec = np.array([rx, ry, rz], dtype=np.float64)
        rot = rotvec_to_rot_matrix(rotvec)
        extensions = pose_to_leg_lengths(pos, rot, geom)

        if extensions.min() < lower or extensions.max() > upper:
            # Out of the usable stroke — excluded rather than silently pinned.
            continue

        vectors.append({
            'label': label,
            'pos_mm': [float(v) for v in pos],
            'rotvec_rad': [float(v) for v in rotvec],
            'rot_matrix': [[float(v) for v in row] for row in rot],
            'extensions_mm': [float(v) for v in extensions],
            'motor_revs': [float(v) for v in extensions * geom.mm_to_rev],
        })

    return vectors


def build_document(geom: StewartGeometry = None) -> Dict[str, Any]:
    if geom is None:
        geom = StewartGeometry()
    return {
        'generator': 'tools/gen_gui_fk_golden.py',
        'ground_truth': 'jugglebot.motion.ik_solver.pose_to_leg_lengths',
        'units': {
            'pos_mm': 'platform-centre offset from STOW, mm',
            'rotvec_rad': 'Rodrigues rotation vector, radians (platform -> base)',
            'extensions_mm': 'per-leg extension, 0 = retracted',
            'motor_revs': 'extensions_mm * mm_to_rev, positive = extension',
        },
        'geometry': {
            'initial_height_mm': geom.init_height_mm,
            'leg_stroke_mm': geom.leg_stroke_mm,
            'base_nodes_mm': geom.base_nodes.tolist(),
            'init_plat_nodes_mm': geom.plat_nodes.tolist(),
            'init_leg_lengths_mm': geom.init_leg_lengths_mm.tolist(),
            'mm_to_rev': geom.mm_to_rev.tolist(),
        },
        'vectors': build_vectors(geom),
    }


def render(geom: StewartGeometry = None) -> str:
    return json.dumps(build_document(geom), indent=2, sort_keys=True) + '\n'


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out', default=DEFAULT_OUT)
    parser.add_argument('--stdout', action='store_true')
    args = parser.parse_args(argv)

    text = render()
    if args.stdout:
        sys.stdout.write(text)
        return 0
    with open(args.out, 'w', encoding='utf-8') as handle:
        handle.write(text)
    sys.stdout.write('Wrote %s (%d vectors)\n'
                     % (args.out, len(json.loads(text)['vectors'])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
