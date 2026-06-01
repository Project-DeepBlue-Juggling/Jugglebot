"""Run the Hermite/Taylor interpolator cross-check inside the test suite.

Drives the validated Python port (teensy_interp.py — the C++ leg_interp.cpp
translation target) and the real motor_guard through identical inputs and asserts
< 1e-6 rev divergence (Phase 7 "done when"). Also asserts the firmware's embedded
per-leg stroke-clamp bounds (legbridge_config.h) match the live MotorGuard, so the
constants the firmware actually ships can't silently drift from the Python the
xref validates against.

The interpolator math is pure (numpy + the jugglebot motion package); no ROS2,
no hardware.
"""

from __future__ import annotations

import os
import re
import sys

import numpy as np
import pytest

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_XREF_DIR = os.path.join(_REPO, "tools", "probes", "teensy_link_profiling", "hermite_xref")
sys.path.insert(0, _XREF_DIR)

xref = pytest.importorskip("xref")  # imports motor_guard + teensy_interp
_LEGBRIDGE_CFG = os.path.join(
    _REPO, "ros_ws", "src", "jugglebot", "Teensy_code_legbridge", "legbridge_config.h")


def test_synthetic_all_modes_match_motor_guard():
    max_dpos, max_dvel = xref.run_synthetic_xref()
    assert max_dpos < 1e-6, f"synthetic Δpos {max_dpos:.3e} >= 1e-6"
    assert max_dvel < 1e-6, f"synthetic Δvel {max_dvel:.3e} >= 1e-6"


def test_recorded_stream_matches_motor_guard():
    csv = xref._newest_csv()
    if not csv:
        pytest.skip("no temp/logs/mpc_*.csv recorded data available")
    max_dpos, max_dvel, n = xref.run_recorded_xref(csv)
    assert n >= 3
    assert max_dpos < 1e-6, f"recorded Δpos {max_dpos:.3e} >= 1e-6 ({csv})"
    assert max_dvel < 1e-6, f"recorded Δvel {max_dvel:.3e} >= 1e-6 ({csv})"


def _parse_float_array(header_text, name):
    m = re.search(rf"{name}\[\w+\]\s*=\s*\{{([^}}]*)\}}", header_text)
    assert m, f"{name} not found in legbridge_config.h"
    return [float(x.strip().rstrip("f")) for x in m.group(1).split(",") if x.strip()]


def test_firmware_stroke_bounds_match_motor_guard():
    """legbridge_config.h STROKE_{MIN,MAX}_REV == live MotorGuard bounds."""
    from jugglebot.motion.geometry import StewartGeometry
    import jugglebot.motion.motor_guard as mg
    guard = mg.MotorGuard(geom=StewartGeometry(), ipc=xref._DummyIPC())

    text = open(_LEGBRIDGE_CFG).read()
    fw_min = _parse_float_array(text, "STROKE_MIN_REV")
    fw_max = _parse_float_array(text, "STROKE_MAX_REV")
    assert len(fw_min) == 6 and len(fw_max) == 6
    # Header rounds to 6 decimals; allow that tolerance.
    assert np.allclose(fw_min, np.array(guard._stroke_min_rev), atol=1e-6), \
        (fw_min, list(np.array(guard._stroke_min_rev)))
    assert np.allclose(fw_max, np.array(guard._stroke_max_rev), atol=1e-6), \
        (fw_max, list(np.array(guard._stroke_max_rev)))
