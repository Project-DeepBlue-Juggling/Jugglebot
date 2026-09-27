"""The CAD (pre-calibration) Stewart geometry, frozen for characterisation tests.

The kinematic calibration applied on 2026-09-27
(``plans/active/kinematic-calibration.md`` § 6 step 5) replaced the CAD node
circles, the single derived leg zero length and the measured ``mm_to_rev`` in
``config/hardware_config.yaml`` with a per-leg fit to a mocap sweep. Two kinds
of test were written against the CAD geometry and are NOT statements about the
machine's geometry:

* **solver characterisations** — ``tests/motion/test_fk_convergence.py``
  reproduces recorded hardware extension vectors and an empirically found
  Newton-iteration recipe. Their residual floors, iteration counts and the
  "still raises under the historical criterion" guard are properties of
  (solver × geometry); moving the geometry silently re-characterises them;
* **wire-format ground truth** — ``tests/motion/data/v5_emitter_frame_fixtures.json``
  is a byte capture whose provenance notes say never to regenerate it; the
  bytes encode revolutions, i.e. ``mm × mm_to_rev`` at the capture checkout.

Both keep proving what they were written to prove only on the geometry they
were recorded under, so they take THIS object instead of ``StewartGeometry()``.
Anything that asserts the live machine's behaviour keeps the live geometry.

The values are the retired ones recorded in the ``jugglebot_geometry`` comment
block of ``config/hardware_config.yaml`` (and in git history at the apply
commit's parent).
"""
from __future__ import annotations

import numpy as np

from jugglebot.motion.geometry import StewartGeometry

CAD_INITIAL_HEIGHT_MM = 574.3
CAD_BASE_NODES_MM = (
    (-385.274, -140.228, 0.0),
    (-314.078, -263.543, 0.0),
    (314.078, -263.543, 0.0),
    (385.274, -140.228, 0.0),
    (71.196, 403.771, 0.0),
    (-71.196, 403.771, 0.0),
)
CAD_INIT_PLAT_NODES_MM = (
    (-197.405, 95.000, 0.0),
    (-16.431, -218.458, 0.0),
    (16.431, -218.458, 0.0),
    (197.405, 95.000, 0.0),
    (180.975, 123.458, 0.0),
    (-180.975, 123.458, 0.0),
)
#: One derived constant for all six: |plat_node_world - base_node| at 574.3 mm.
CAD_INIT_LEG_LENGTHS_MM = (648.419,) * 6
#: sp_ik.py line 228, measured experimentally (pre-calibration).
CAD_MM_TO_REV = (0.01418332, 0.01419076, 0.01408956,
                 0.01418684, 0.01426801, 0.01424951)


def cad_geometry() -> StewartGeometry:
    """A ``StewartGeometry`` carrying the 2026-09-27-retired CAD constants.

    Built from the live class so every derived attribute and method is the
    live one; only the five calibrated fields are overridden. ``spool_radius_mm``
    is re-derived from ``mm_to_rev`` the way ``__init__`` does it.
    """
    g = StewartGeometry()
    g.init_height_mm = CAD_INITIAL_HEIGHT_MM
    g.base_nodes = np.array(CAD_BASE_NODES_MM, dtype=np.float64)
    g.plat_nodes = np.array(CAD_INIT_PLAT_NODES_MM, dtype=np.float64)
    g.init_leg_lengths_mm = np.array(CAD_INIT_LEG_LENGTHS_MM, dtype=np.float64)
    g.leg_lengths_with_offset_mm = g.init_leg_lengths_mm.copy()
    g.mm_to_rev = np.array(CAD_MM_TO_REV, dtype=np.float64)
    g.spool_radius_mm = 1.0 / (2.0 * np.pi * g.mm_to_rev)
    return g
