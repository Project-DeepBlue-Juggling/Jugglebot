"""Tests for the hand-generator characterisation probe.

These tests pin the Phase 1 characterisation findings of
``plans/active/hand-trajectory-generator-overhaul.md``. They assert the
numbers ``tools/probes/hand_profile_probe.py`` measures, and — critically —
that the probe drives the *real* ``sim/hand/trajectory.py`` generator rather
than a re-implementation of it.

The headline findings locked here:

* The current throw/catch generator has piecewise-CONSTANT acceleration, so
  acceleration *steps* discontinuously at every segment boundary (unbounded
  jerk).
* Peak acceleration scales as ``v²`` and motion duration as ``1/v`` — the
  "backwards" scaling the overhaul fixes.
* ``HandSmoothMove`` (already quintic) is jerk- and acceleration-bounded —
  the design reference for Phase 2.
* The ``catch_vel_ratio`` port/firmware reconciliation (both 0.6 since
  2026-05-23; the probe still records both side-by-side as a drift guard).

Logbook: logbook/2026-05-22-hand-generator-phase1-characterisation.md
"""

from __future__ import annotations

import os
import sys

import pytest

# Make the committed probe importable (tests/conftest adds repo root + sim).
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_PROBES_DIR = os.path.join(_REPO_ROOT, "tools", "probes")
if _PROBES_DIR not in sys.path:
    sys.path.insert(0, _PROBES_DIR)

import hand_profile_probe as probe  # noqa: E402


# ── Robustness: the probe must exercise the real generator ───────────────

def test_probe_drives_the_real_generator():
    """The probe must import the real port classes, not re-implement them."""
    import hand.trajectory as real

    assert probe.HandThrowTrajectory is real.HandThrowTrajectory
    assert probe.HandCatchTrajectory is real.HandCatchTrajectory
    assert probe.HandSmoothMove is real.HandSmoothMove


# ── Headline finding: acceleration is discontinuous at segment boundaries ─

def test_throw_acceleration_is_discontinuous():
    """Every throw segment boundary has a non-zero acceleration step."""
    m = probe.characterise_throw(3.0)
    steps = [abs(s["step_mps2"]) for s in m["accel_steps"]]

    # All four boundaries (rest→accel, accel→vel, vel→decel, decel→rest)
    # carry a real step. A jerk-limited generator would have steps ≈ 0.
    assert len(steps) == 4
    assert all(s > 1.0 for s in steps), f"expected discontinuous steps, got {steps}"
    # The largest step equals the analytic peak acceleration.
    assert m["max_accel_step_mps2"] == pytest.approx(m["analytic_peak_accel_mps2"])


def test_catch_acceleration_is_discontinuous():
    """Every catch segment boundary has a non-zero acceleration step."""
    m = probe.characterise_catch(3.0)
    steps = [abs(s["step_mps2"]) for s in m["accel_steps"]]

    assert len(steps) == 4
    assert all(s > 1.0 for s in steps), f"expected discontinuous steps, got {steps}"
    assert m["max_accel_step_mps2"] == pytest.approx(m["analytic_peak_accel_mps2"])


# ── Backwards scaling: peak accel ∝ v², motion duration ∝ 1/v ────────────

def test_throw_peak_accel_scales_as_v_squared():
    """Throw peak acceleration is an exact v² power law across the sweep."""
    summary = probe.run_probe()
    fit = summary["throw"]["peak_accel_v2_fit"]
    assert fit["exponent"] == 2.0
    assert fit["max_rel_residual"] < 1e-12, (
        f"peak accel does not follow v²: residual {fit['max_rel_residual']:.2e}")


def test_throw_motion_duration_scales_as_inverse_v():
    """Throw motion duration is an exact 1/v power law across the sweep."""
    summary = probe.run_probe()
    fit = summary["throw"]["motion_duration_inv_v_fit"]
    assert fit["exponent"] == -1.0
    assert fit["max_rel_residual"] < 1e-12, (
        f"duration does not follow 1/v: residual {fit['max_rel_residual']:.2e}")


def test_catch_motion_duration_scales_as_inverse_v():
    """Catch motion duration (excluding the end-hold tail) follows 1/v."""
    summary = probe.run_probe()
    fit = summary["catch"]["motion_duration_inv_v_fit"]
    assert fit["max_rel_residual"] < 1e-12


def test_total_catch_duration_is_not_pure_inverse_v():
    """Catch *total* duration carries a fixed END_PROFILE_HOLD tail.

    Documents why the probe fits motion_duration (not total duration): the
    constant hold breaks the pure 1/v law.
    """
    velocities = probe.sweep_velocities()
    catches = [probe.characterise_catch(v) for v in velocities]
    bad_fit = probe.fit_power_law(
        velocities, [c["duration_s"] for c in catches], -1.0)
    assert bad_fit["max_rel_residual"] > 0.01, (
        "total catch duration unexpectedly fit a pure 1/v law")


# ── Reference: HandSmoothMove is jerk- and acceleration-bounded ──────────

def test_smooth_move_is_acceleration_bounded():
    """The quintic smooth move respects its design acceleration limit."""
    sm = probe.characterise_smooth_move()
    limit = sm["accel_limit_revps2"]
    # FD slightly under the analytic peak; never materially above the limit.
    assert sm["fd_peak_accel_revps2"] <= limit * 1.01
    assert sm["fd_peak_accel_revps2"] >= limit * 0.90


def test_throw_jerk_dwarfs_the_smooth_move_reference():
    """The unbounded-jerk throw is orders of magnitude worse than the ref."""
    summary = probe.run_probe()
    throw_jerk = summary["throw"]["fd_peak_jerk_mps3_at_v_max"]
    ref_jerk = summary["smooth_move_reference"]["fd_peak_jerk_mps3"]
    assert throw_jerk > 100.0 * ref_jerk


def test_throw_peak_accel_exceeds_smooth_move_limit():
    """At v_max the throw peak accel is many times the smooth-move limit."""
    summary = probe.run_probe()
    ratio = summary["throw"]["peak_accel_over_smooth_limit_at_v_max"]
    assert ratio > 10.0


# ── Determinism ──────────────────────────────────────────────────────────

def test_probe_measurements_are_deterministic():
    """Re-running the probe produces identical measurements (no flakiness)."""
    assert probe.run_probe() == probe.run_probe()


# ── catch_vel_ratio: port reconciled to firmware (0.6) ──────────────────

def test_catch_vel_ratio_matches_firmware():
    """The port matches the firmware-authoritative value (both 0.6).

    Phase 1 found the port had drifted to a hardcoded 0.9; it was reconciled
    to 0.6 on 2026-05-23 so port and firmware agree. This test fails (and
    surfaces the regression) if anyone re-introduces a drift.
    """
    cvr = probe.run_probe()["catch_vel_ratio"]
    assert cvr["firmware_value"] == 0.6
    assert cvr["port_value"] == cvr["firmware_value"]
    assert cvr["divergent"] is False
