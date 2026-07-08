"""Ballistic boundary-condition tests (Phase 6).

Covers the touchdown quadratic + launch/arrival velocity BCs the catch path
consumes, and pins the *identical-form* overlap with ``controller/ballistics.py``
(the copy is faithful) — a copied formula that silently drifts from its source is
the failure this guards.
"""

from __future__ import annotations

import numpy as np
import pytest

from jugglebot.motion.trajectory import ballistics_bc as bb


G = bb.GRAVITY_MMS2


# ── launch / arrival velocity round-trips ──

def test_launch_velocity_lands_on_target():
    """A ball launched with launch_velocity() reaches the target after T."""
    rel = np.array([-800.0, -600.0, 1400.0])
    tgt = np.array([30.0, -10.0, 840.0])
    T = 0.65
    v0 = bb.launch_velocity(rel, tgt, T)
    landed = bb.position_at(rel, v0, T)
    assert np.allclose(landed, tgt, atol=1e-6)


def test_arrival_velocity_matches_ballistic_integration():
    v0 = np.array([1200.0, 900.0, 3000.0])
    T = 0.5
    assert np.allclose(bb.arrival_velocity(v0, T),
                       v0 + np.array([0.0, 0.0, -G * T]))


def test_launch_velocity_zero_time_raises():
    with pytest.raises(ValueError):
        bb.launch_velocity(np.zeros(3), np.ones(3), 0.0)


# ── touchdown quadratic ──

def test_touchdown_time_descending_root():
    """A ball thrown up from below the catch height crosses it twice; the catch
    (descending) takes the later root."""
    rel = np.array([0.0, 0.0, 500.0])
    v0 = np.array([0.0, 0.0, 3000.0])       # straight up
    z_catch = 800.0
    t_desc = bb.touchdown_time(rel, v0, z_catch, descending=True)
    t_asc = bb.touchdown_time(rel, v0, z_catch, descending=False)
    assert t_desc > t_asc > 0.0
    # Both roots land at the catch height.
    assert bb.position_at(rel, v0, t_desc)[2] == pytest.approx(z_catch, abs=1e-6)
    assert bb.position_at(rel, v0, t_asc)[2] == pytest.approx(z_catch, abs=1e-6)
    # The descending crossing is moving downward.
    assert bb.velocity_at(v0, t_desc)[2] < 0.0


def test_touchdown_apex_below_catch_raises():
    """A throw whose apex is below the catch height never reaches it — loud reject."""
    rel = np.array([0.0, 0.0, 500.0])
    v0 = np.array([0.0, 0.0, 500.0])        # apex ~ 500 + 12.7 mm ≪ 800
    with pytest.raises(ValueError):
        bb.touchdown_time(rel, v0, 800.0)


def test_arrival_state_at_z_consistent():
    """arrival_state_at_z ties position/velocity/time together consistently."""
    rel = np.array([-870.0, -630.0, 1430.0])
    tgt = np.array([0.0, 0.0, 840.0])
    T_flight = 0.7
    v0 = bb.launch_velocity(rel, tgt, T_flight)
    pos, vel, t = bb.arrival_state_at_z(rel, v0, 840.0)
    assert t == pytest.approx(T_flight, abs=1e-6)
    assert np.allclose(pos, tgt, atol=1e-6)
    assert np.allclose(vel, bb.arrival_velocity(v0, T_flight))
    assert vel[2] < 0.0                     # descending catch


# ── overlap with controller/ballistics.py is faithful (identical forms) ──

def test_matches_controller_ballistics_forms():
    """launch/arrival velocity are the exact forms controller/ballistics uses."""
    from controller import ballistics as cb
    assert bb.GRAVITY_MMS2 == cb.GRAVITY_MMS2
    rel = np.array([-800.0, -600.0, 1400.0])
    tgt = np.array([30.0, -10.0, 840.0])
    T = 0.6
    assert np.allclose(bb.launch_velocity(rel, tgt, T),
                       cb.compute_launch_velocity(rel, tgt, T))
    v0 = bb.launch_velocity(rel, tgt, T)
    assert np.allclose(bb.arrival_velocity(v0, T),
                       cb.compute_arrival_velocity(v0, T))
