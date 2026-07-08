"""Pure-Python catch_coordinator receive-tilt tests (Phase 7 audit fix 6).

The exhaustive catch-policy tests live in the package tree
(``ros_ws/src/jugglebot/jugglebot/tracking/tests/test_coordinator.py``); this file pins
the one behaviour that changed in the Phase-7 audit and must be covered by the collected
suite: the receive tilt is CLAMPED to ``tilt_geometry.MAX_TILT_DEG`` (12°) rather than
REJECTED above the old 30° catch-angle limit. Real BB arrivals are 18–40° off vertical,
so the old None-reject silently dropped every real reload catch.

ROS 2 is mocked by ``tests/ros/conftest.py`` (CatchCoordinator is pure Python — imported
directly).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from jugglebot.catch_coordinator import CatchCoordinator
from jugglebot.motion.trajectory.tilt_geometry import MAX_TILT_DEG


def _arrival_velocity(angle_deg: float, speed_mmps: float = 5000.0) -> np.ndarray:
    """A descending arrival velocity tilted ``angle_deg`` from vertical in the +X plane."""
    a = math.radians(angle_deg)
    return np.array([speed_mmps * math.sin(a), 0.0, -speed_mmps * math.cos(a)])


def _tilt_magnitude_deg(quat: np.ndarray) -> float:
    """From-vertical tilt magnitude (deg) of a [w, x, y, z] orientation."""
    return math.degrees(2.0 * math.acos(min(abs(float(quat[0])), 1.0)))


def test_steep_arrival_clamped_not_rejected():
    """A 25° arrival (past the 12° ceiling AND past the old 30°-adjacent envelope) yields
    a CLAMPED 12° orientation, not None."""
    coord = CatchCoordinator(catch_angle_limit_deg=30.0)
    quat = coord.compute_catch_orientation(_arrival_velocity(25.0))
    assert quat is not None
    assert _tilt_magnitude_deg(quat) == pytest.approx(MAX_TILT_DEG, abs=0.05)


def test_very_steep_arrival_still_clamped_and_emitted():
    """A 40° arrival (that the old code rejected as > 30°) is now clamped, not dropped —
    so the catch command is still produced downstream."""
    coord = CatchCoordinator(catch_angle_limit_deg=30.0)
    quat = coord.compute_catch_orientation(_arrival_velocity(40.0))
    assert quat is not None
    assert _tilt_magnitude_deg(quat) == pytest.approx(MAX_TILT_DEG, abs=0.05)


def test_sub_ceiling_arrival_is_exact():
    """An arrival within the ceiling is NOT clamped — the tilt matches the arrival."""
    coord = CatchCoordinator(catch_angle_limit_deg=30.0)
    quat = coord.compute_catch_orientation(_arrival_velocity(8.0))
    assert quat is not None
    assert _tilt_magnitude_deg(quat) == pytest.approx(8.0, abs=0.05)


def test_steep_arrival_produces_catch_command():
    """End-to-end through update(): a 25° arrival now produces a catch command (the
    None-reject is gone)."""
    from jugglebot.tracking.ball import Ball, BallStatus, TrackingConfidence
    coord = CatchCoordinator(initial_height_mm=574.3)
    ball = Ball(
        id=1, status=BallStatus.IN_FLIGHT, tracking=TrackingConfidence.CONFIRMED,
        source="ball_butler", destination="jugglebot",
        position=np.array([0.0, 0.0, 900.0]), velocity=np.array([0.0, 0.0, -2000.0]),
        landing_position=np.array([100.0, 0.0, 734.3]),
        landing_velocity=_arrival_velocity(25.0), landing_time=10.0)
    cmd = coord.update([ball], current_time=5.0)
    assert cmd is not None
    assert _tilt_magnitude_deg(cmd.target_quat) == pytest.approx(MAX_TILT_DEG, abs=0.05)
