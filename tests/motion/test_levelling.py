"""Unit tests for the gravity-levelling transform (`motion/levelling.py`).

Contract: `ros_ws/docs/levelling_frame.md` (C-LEVEL-1).

These pin the two halves the contract's *shared implementation* owns — the sign
convention and the composition order — plus the worked example from the
2026-07-25 self-toss investigation that motivated the whole plan. No ROS mocking:
`motion/` is pure Python by design.

The composition-order test deliberately uses a **non-identity** target. With an
identity target `R_gravity @ R_target` and `R_target @ R_gravity` are the same
matrix, so an identity-target test cannot distinguish the correct order from its
plausible transposition — it would pass against a silently reversed
implementation. Verified empirically before writing (probe, 2026-07-25): for the
session offset and a `[0.15, -0.08, 0]` target the two orders differ by
`1.268e-3 rad` (0.0727°), entirely in `rz`.
"""

from __future__ import annotations

import numpy as np
import pytest

from jugglebot.motion import levelling
from jugglebot.motion.ik_solver import rot_matrix_to_rotvec, rotvec_to_rot_matrix

# The levelling offset published during the 2026-07-25 15:17:48 self-toss session
# — the measurement the whole investigation is anchored on.
_SESSION_OFFSET = (0.013592347421588673, 0.001207157476773584)


# ── identity_correction ──────────────────────────────────────────


def test_identity_correction_is_identity_and_not_shared():
    a = levelling.identity_correction()
    b = levelling.identity_correction()
    assert np.array_equal(a, np.eye(3))
    # A shared mutable module constant would let one node's in-place write corrupt
    # every other node's levelling frame in the same process.
    assert a is not b
    a[0, 0] = 99.0
    assert np.array_equal(levelling.identity_correction(), np.eye(3))


def test_identity_correction_is_a_no_op():
    R = levelling.identity_correction()
    for rotvec in (np.zeros(3), np.array([0.15, -0.08, 0.02]),
                   np.array([1.2, -0.7, 0.4])):
        out = levelling.apply_gravity_correction(rotvec, R)
        assert np.allclose(out, rotvec, atol=1e-12)


# ── correction_from_offset — the sign convention ─────────────────


def test_correction_counter_tilts_the_measured_error():
    """A +tilt_x measured error must produce a −rx command.

    This is the whole point of the correction: the platform *has* +tilt_x while
    commanded level, so commanding level must counter-tilt by −tilt_x to put it
    level against gravity. A sign flip here would DOUBLE the tilt error rather
    than null it, and would look plausible in the code.
    """
    R = levelling.correction_from_offset(0.05, 0.03)
    out = levelling.apply_gravity_correction(np.zeros(3), R)
    assert out[0] < 0.0, "a +tilt_x offset must command −rx"
    assert out[1] < 0.0, "a +tilt_y offset must command −ry"
    assert out[2] == pytest.approx(0.0, abs=1e-15)


def test_correction_round_trip_reproduces_the_negated_offset():
    """correction_from_offset → apply(identity target) → [-tilt_x, -tilt_y, 0]."""
    for tilt_x, tilt_y in ((0.0, 0.0), _SESSION_OFFSET, (0.05, -0.03),
                           (-0.2, 0.11)):
        R = levelling.correction_from_offset(tilt_x, tilt_y)
        out = levelling.apply_gravity_correction(np.zeros(3), R)
        assert np.allclose(out, [-tilt_x, -tilt_y, 0.0], atol=1e-15)
        # ...and the stored form round-trips through the matrix representation.
        assert np.allclose(rot_matrix_to_rotvec(R), out, atol=1e-15)


def test_correction_rz_is_always_zero():
    """Levelling corrects the two tilt axes; yaw has no gravity reference."""
    for tilt_x, tilt_y in ((0.4, -0.4), (_SESSION_OFFSET), (1e-9, 1e-9)):
        R = levelling.correction_from_offset(tilt_x, tilt_y)
        assert rot_matrix_to_rotvec(R)[2] == pytest.approx(0.0, abs=1e-15)


# ── composition order (non-commutative — the load-bearing test) ──


def test_composition_order_is_R_gravity_at_R_target():
    """Pin ``R_corrected = R_gravity @ R_target`` against a NON-IDENTITY target.

    The reversed form is a plausible transcription error and is a genuinely
    different rotation — for this target the orders differ by 1.268e-3 rad, all
    of it in ``rz``, which is invisible in leg-space telemetry.
    """
    R = levelling.correction_from_offset(*_SESSION_OFFSET)
    target = np.array([0.15, -0.08, 0.0])

    expected = rot_matrix_to_rotvec(R @ rotvec_to_rot_matrix(target))
    reversed_order = rot_matrix_to_rotvec(rotvec_to_rot_matrix(target) @ R)

    out = levelling.apply_gravity_correction(target, R)
    assert np.allclose(out, expected, atol=1e-15)

    # The test is only meaningful if the two orders are actually distinguishable
    # for this target — assert that, so a future edit to the target value cannot
    # silently reduce this to a tautology.
    delta = float(np.linalg.norm(expected - reversed_order))
    assert delta == pytest.approx(1.268256e-03, rel=1e-4), (
        "the pinned target must keep the two composition orders distinguishable")
    assert not np.allclose(out, reversed_order, atol=1e-9)


def test_identity_target_cannot_distinguish_the_two_orders():
    """Documents WHY the order test above needs a non-identity target."""
    R = levelling.correction_from_offset(*_SESSION_OFFSET)
    I = np.eye(3)
    assert np.allclose(R @ I, I @ R, atol=1e-15)


# ── the pinned worked example (regression) ───────────────────────


def test_pinned_2026_07_25_worked_example():
    """The session offset with an identity target ⇒ (−0.77878414°, −0.06916503°, 0°).

    This is the exact tilt the toss's pre-tilt ``catch/dynamic_target`` asked for
    while the positioning ``go_to_pose`` parked the platform at plan-frame
    ``rx = 0`` — the reach between the two frames IS the anomaly this contract
    closes. Confirmed to 8 dp by probe before being pinned here.
    """
    R = levelling.correction_from_offset(*_SESSION_OFFSET)
    deg = np.degrees(levelling.apply_gravity_correction(np.zeros(3), R))

    assert deg[0] == pytest.approx(-0.77878414, abs=5e-9)
    assert deg[1] == pytest.approx(-0.06916503, abs=5e-9)
    assert deg[2] == pytest.approx(0.0, abs=1e-12)
    # Total tilt, and the ry/rx axis ratio that identified the cause in the bag.
    assert float(np.hypot(deg[0], deg[1])) == pytest.approx(0.78184944, abs=5e-9)
    assert deg[1] / deg[0] == pytest.approx(0.0888, abs=5e-5)


# ── correct_pose — rotation only, never in place ─────────────────


def test_correct_pose_rewrites_rotation_only():
    """Position must pass through EXACTLY — bit-identical, not merely close.

    The catch reach-envelope check compares position-only 3-vectors on both
    sides; if the correction could move position by even a float ULP, the
    envelope's centre and its targets would drift into different frames.
    """
    R = levelling.correction_from_offset(*_SESSION_OFFSET)
    for pose in (np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0]),
                 np.array([11.88, 2.87, 171.16, 0.19345, 0.0227, 0.0]),
                 np.array([-40.0, 33.0, 190.0, 0.02, -0.05, 0.01])):
        out = levelling.correct_pose(pose, R)
        assert np.array_equal(out[:3], pose[:3])
        assert np.allclose(
            out[3:6], levelling.apply_gravity_correction(pose[3:6], R), atol=1e-15)


def test_correct_pose_does_not_mutate_its_input():
    """The neutral constant is corrected AT USE, so `correct_pose` is called on a
    stored array every time. Mutating it would double-apply on the next read —
    exactly the failure the "correct at use, not at construction" rule prevents.
    """
    R = levelling.correction_from_offset(*_SESSION_OFFSET)
    neutral = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    before = neutral.copy()
    first = levelling.correct_pose(neutral, R)
    second = levelling.correct_pose(neutral, R)
    assert np.array_equal(neutral, before), "input pose was mutated"
    assert np.array_equal(first, second), "repeated correction is not idempotent"
    assert not np.shares_memory(first, neutral)


def test_correct_pose_accepts_a_list():
    R = levelling.correction_from_offset(*_SESSION_OFFSET)
    out = levelling.correct_pose([1.0, 2.0, 170.0, 0.0, 0.0, 0.0], R)
    assert out.dtype == np.float64
    assert np.array_equal(out[:3], [1.0, 2.0, 170.0])


def test_apply_gravity_correction_does_not_mutate_its_input():
    R = levelling.correction_from_offset(*_SESSION_OFFSET)
    rotvec = np.array([0.15, -0.08, 0.0])
    before = rotvec.copy()
    levelling.apply_gravity_correction(rotvec, R)
    assert np.array_equal(rotvec, before)


# ── the inverse (C-LEVEL-1's observability corollary, 2026-08-23) ─────────────

def test_uncorrect_pose_inverts_correct_pose_exactly():
    """Round trip, on a NON-IDENTITY target and a non-trivial correction.

    Both halves matter. An identity target cannot tell `Rᵀ @ R_corrected` from
    `R_corrected @ Rᵀ` (the same trap the composition-order test names), and a
    zero correction makes the inverse the identity function, which would pass
    against a `uncorrect_pose = correct_pose` typo."""
    correction = levelling.correction_from_offset(*_SESSION_OFFSET)
    for intent in ([0.0, 0.0, 170.0, 0.15, -0.08, 0.0],
                   [12.0, -30.0, 190.0, 0.004, -0.006, 0.0],
                   [0.0, 0.0, 170.0, 0.0, 0.0, 0.0]):
        pose = np.array(intent, dtype=float)
        back = levelling.uncorrect_pose(
            levelling.correct_pose(pose, correction), correction)
        assert back == pytest.approx(pose, abs=1e-12)

    # …and it really is doing work: the corrected rotation differs from the
    # intent by the correction, so a no-op inverse would fail the round trip.
    tilted = np.array([0.0, 0.0, 170.0, 0.15, -0.08, 0.0])
    corrected = levelling.correct_pose(tilted, correction)
    assert not np.allclose(corrected[3:6], tilted[3:6], atol=1e-6)


def test_uncorrect_pose_leaves_position_untouched_and_does_not_mutate():
    """Position is frame-invariant here (``correct_pose`` never touches it), so
    the inverse must not either — and the input is a caller's array."""
    correction = levelling.correction_from_offset(*_SESSION_OFFSET)
    pose = np.array([12.0, -30.0, 190.0, 0.15, -0.08, 0.0])
    before = pose.copy()
    out = levelling.uncorrect_pose(pose, correction)
    assert out[:3] == pytest.approx(pose[:3], abs=0.0)
    assert pose == pytest.approx(before, abs=0.0)
    assert out is not pose


def test_uncorrect_pose_is_the_transpose_not_the_negated_offset():
    """``Rᵀ`` and ``R(−offset)`` agree for THIS correction (a single Rodrigues of
    a negated rotvec), and that agreement is worth pinning rather than assuming:
    it is what makes the inverse exact instead of first-order, and it is the
    property a future pose-dependent correction must preserve or the inverse
    silently becomes an approximation."""
    correction = levelling.correction_from_offset(*_SESSION_OFFSET)
    negated = rotvec_to_rot_matrix(
        -rot_matrix_to_rotvec(correction))
    assert correction.T == pytest.approx(negated, abs=1e-12)
