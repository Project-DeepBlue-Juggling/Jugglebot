"""Unified 7-DoF planner, Phase 1 — the ``CyclePlan`` 7-channel plan object.

Plan: plans/active/unified-7dof-planner.md § 4 Phase 1.

Two things have to hold or Phase 2/4 cannot reuse the existing machinery:

  1. **``state_at`` is a ``TrajectoryPlan``.**  Same return shape, same terminal
     hold, same pre-start boundary conditions, same ``kind``/``total_duration``/
     ``final_pose`` surface the emitter and ``trajectory_node._install`` read.  The
     emitter conformance test drives the REAL ``SetpointEmitter`` over a cycle plan
     — a shape the plan class merely *claims* is worth much less than one the
     actual consumer swallows.
  2. **The interpolation is the firmware's.**  Piecewise cubic Hermite from knot
     values + knot velocities, so the host's idea of the curve between knots is the
     one the 500 Hz lane reconstructs from ``(u0, v0, u1, v1)``.  Exactness on a
     known cubic is the direct check; continuity at every knot join is the
     structural one.
"""

from __future__ import annotations

import numpy as np
import pytest

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory.cycle_plan import CyclePlan
from jugglebot.motion.trajectory.emitter import KnotEmitter
from jugglebot.motion.trajectory.plan import TrajectoryPlan

DT = 0.025
N = 17


# ── fixtures ─────────────────────────────────────────────────────────────────

def _cubic_channel(t, coeffs):
    """Value and exact derivative of ``a + b·t + c·t² + d·t³`` at ``t``."""
    a, b, c, d = coeffs
    return (a + b * t + c * t ** 2 + d * t ** 3,
            b + 2.0 * c * t + 3.0 * d * t ** 2)


def _make_plan(n: int = N, dt: float = DT) -> CyclePlan:
    """A cycle plan whose every channel is a genuine cubic in ``t``.

    Each pose axis and the hand get their own cubic, so a per-channel mix-up shows
    up as a wrong value rather than cancelling out.  Poses are kept near the active
    neutral pose so the emitter's IK is well-conditioned.
    """
    t = np.arange(n, dtype=float) * dt
    pose_coeffs = [
        (0.0, 40.0, -30.0, 8.0),        # x  (mm)
        (0.0, -25.0, 18.0, -5.0),       # y
        (170.0, 6.0, -4.0, 1.0),        # z
        (0.0, 0.05, -0.04, 0.01),       # rx (rad)
        (0.0, -0.03, 0.02, -0.006),     # ry
        (0.0, 0.0, 0.0, 0.0),           # rz
    ]
    hand_coeffs = (5.0, 3.0, -2.0, 0.5)

    pose = np.empty((n, 6))
    pose_vel = np.empty((n, 6))
    for i, c in enumerate(pose_coeffs):
        pose[:, i], pose_vel[:, i] = _cubic_channel(t, c)
    hand_rev, hand_vel = _cubic_channel(t, hand_coeffs)
    return CyclePlan(pose, pose_vel, hand_rev, hand_vel, dt, catch_k=7)


def _pose_cubic(t):
    """Ground truth for the fixture's pose channels at arbitrary ``t``."""
    coeffs = [(0.0, 40.0, -30.0, 8.0), (0.0, -25.0, 18.0, -5.0),
              (170.0, 6.0, -4.0, 1.0), (0.0, 0.05, -0.04, 0.01),
              (0.0, -0.03, 0.02, -0.006), (0.0, 0.0, 0.0, 0.0)]
    return np.array([_cubic_channel(t, c)[0] for c in coeffs])


# ── TrajectoryPlan contract conformance ──────────────────────────────────────

def test_is_a_trajectory_plan_with_the_expected_surface():
    plan = _make_plan()
    assert isinstance(plan, TrajectoryPlan)
    assert plan.segments == ()
    assert plan.kind == 'move'          # NOT the base class's 'hold' inference
    assert plan.total_duration == pytest.approx((N - 1) * DT)
    assert plan.final_pose.shape == (6,)
    np.testing.assert_array_equal(plan.final_pose, plan.pose[-1])


def test_state_at_returns_three_six_vectors():
    plan = _make_plan()
    state = plan.state_at(0.11)
    assert len(state) == 3
    for arr in state:
        assert isinstance(arr, np.ndarray)
        assert arr.shape == (6,)
        assert np.all(np.isfinite(arr))


def test_state_at_knots_returns_the_knot_state_exactly():
    plan = _make_plan()
    for k in range(plan.n_knots - 1):
        pose, twist, _ = plan.state_at(k * DT)
        np.testing.assert_allclose(pose, plan.pose[k], rtol=0.0, atol=1e-12)
        np.testing.assert_allclose(twist, plan.pose_vel[k], rtol=0.0, atol=1e-12)


def test_terminal_hold_matches_the_base_contract():
    plan = _make_plan()
    for t in (plan.total_duration, plan.total_duration + 1e-9,
              plan.total_duration + 5.0):
        pose, twist, accel = plan.state_at(t)
        np.testing.assert_array_equal(pose, plan.final_pose)
        np.testing.assert_array_equal(twist, np.zeros(6))
        np.testing.assert_array_equal(accel, np.zeros(6))
    # ...and the returned pose is a copy, so a caller cannot poison the plan.
    pose, _, _ = plan.state_at(plan.total_duration)
    pose[0] = 1234.0
    assert plan.final_pose[0] != 1234.0


def test_before_start_returns_the_first_knot_boundary_conditions():
    """``TrajectoryPlan`` hands back the first segment's START BCs for t <= 0 —
    pose AND its (possibly nonzero) velocity.  A cycle plan does the same."""
    plan = _make_plan()
    for t in (0.0, -1e-6, -3.0):
        pose, twist, _ = plan.state_at(t)
        np.testing.assert_allclose(pose, plan.pose[0], rtol=0.0, atol=1e-12)
        np.testing.assert_allclose(twist, plan.pose_vel[0], rtol=0.0, atol=1e-12)
    assert np.any(plan.pose_vel[0] != 0.0), "fixture starts at rest — test vacuous"


def test_state_at_is_continuous_across_every_knot_join():
    """Pose and twist must not step at a knot: the cubic Hermite shares both with
    its neighbour by construction, and a broken ``_locate`` shows up here."""
    plan = _make_plan()
    eps = 1e-9
    for k in range(1, plan.n_knots - 1):
        tk = k * DT
        before = plan.state_at(tk - eps)
        after = plan.state_at(tk + eps)
        np.testing.assert_allclose(before[0], after[0], rtol=0.0, atol=1e-6)
        np.testing.assert_allclose(before[1], after[1], rtol=0.0, atol=1e-4)


def test_state_at_reproduces_the_source_cubic_between_knots():
    """The fixture's channels ARE cubics with exact knot derivatives, so the
    Hermite must reproduce them everywhere, not merely at the knots."""
    plan = _make_plan()
    for t in np.linspace(0.0, plan.total_duration, 401)[:-1]:
        np.testing.assert_allclose(plan.state_at(float(t))[0], _pose_cubic(t),
                                   rtol=0.0, atol=1e-9)


# ── the 7th channel ──────────────────────────────────────────────────────────

def test_hand_at_is_exact_on_a_known_cubic():
    plan = _make_plan()
    for t in np.linspace(0.0, plan.total_duration, 401)[:-1]:
        want_p, want_v = _cubic_channel(float(t), (5.0, 3.0, -2.0, 0.5))
        got_p, got_v = plan.hand_at(float(t))
        assert got_p == pytest.approx(want_p, abs=1e-10)
        assert got_v == pytest.approx(want_v, abs=1e-8)


def test_hand_at_returns_plain_floats():
    plan = _make_plan()
    rev, rev_s = plan.hand_at(0.07)
    assert isinstance(rev, float) and isinstance(rev_s, float)


def test_hand_at_knots_and_continuity():
    plan = _make_plan()
    for k in range(plan.n_knots - 1):
        rev, rev_s = plan.hand_at(k * DT)
        assert rev == pytest.approx(plan.hand_rev[k], abs=1e-12)
        assert rev_s == pytest.approx(plan.hand_vel_rps[k], abs=1e-10)
    eps = 1e-9
    for k in range(1, plan.n_knots - 1):
        b = plan.hand_at(k * DT - eps)
        a = plan.hand_at(k * DT + eps)
        assert b[0] == pytest.approx(a[0], abs=1e-6)
        assert b[1] == pytest.approx(a[1], abs=1e-4)


def test_hand_at_clamps_at_both_window_ends():
    plan = _make_plan()
    for t in (plan.total_duration, plan.total_duration + 2.0):
        rev, rev_s = plan.hand_at(t)
        assert rev == pytest.approx(float(plan.hand_rev[-1]), abs=1e-12)
        assert rev_s == 0.0
    for t in (0.0, -5.0):
        rev, rev_s = plan.hand_at(t)
        assert rev == pytest.approx(float(plan.hand_rev[0]), abs=1e-12)
        assert rev_s == pytest.approx(float(plan.hand_vel_rps[0]), abs=1e-12)


def test_pose_and_hand_share_one_clock():
    """The whole point of the class: both channels answer the same ``t`` with the
    same interval, so 'the hand strokes while the platform is here' is structural."""
    plan = _make_plan()
    k = 5
    t = k * DT
    pose, _, _ = plan.state_at(t)
    rev, _ = plan.hand_at(t)
    np.testing.assert_allclose(pose, plan.pose[k], rtol=0.0, atol=1e-12)
    assert rev == pytest.approx(plan.hand_rev[k], abs=1e-12)


# ── the real consumer swallows it ────────────────────────────────────────────

def test_the_production_emitter_samples_a_cycle_plan_unchanged():
    """``KnotEmitter.frame`` is the 40 Hz consumer; Phase 2 only ADDS hand keys to
    it, so a cycle plan must already work through it untouched today."""
    plan = _make_plan()
    emitter = KnotEmitter(StewartGeometry())
    frames = [emitter.frame(plan, float(tau), seq)
              for seq, tau in enumerate(np.arange(0.0, plan.total_duration, DT))]
    assert len(frames) >= 8
    for f in frames:
        for key in ('ext_mm', 'motor_rev', 'vel_mm_s', 'acc_mm_s2',
                    'cmd_next_mm', 'cmd_next2_mm'):
            arr = np.asarray(f[key], dtype=float)
            assert arr.shape == (6,), f"{key} shape {arr.shape}"
            assert np.all(np.isfinite(arr)), f"{key} not finite"
        np.testing.assert_allclose(np.asarray(f['pose_6dof'], dtype=float),
                                   plan.state_at(f['seq'] * DT)[0])


def test_from_realized_duck_types_the_realized_cycle_contract():
    from jugglebot.motion.trajectory import cup_realize as cr

    class _Realized:
        pass

    r = _Realized()
    src = _make_plan()
    r.pose = src.pose
    r.pose_vel = src.pose_vel
    r.slider_rev = src.hand_rev
    r.slider_vel_rev_s = src.hand_vel_rps
    r.dt = DT
    r.catch_k = 4

    plan = CyclePlan.from_realized(r)
    assert plan.catch_k == 4
    np.testing.assert_array_equal(plan.pose, src.pose)
    np.testing.assert_array_equal(plan.hand_rev, src.hand_rev)
    # And the real RealizedCycle satisfies the same contract.
    assert all(hasattr(cr.RealizedCycle, '__dataclass_fields__') and
               f in cr.RealizedCycle.__dataclass_fields__
               for f in ('pose', 'pose_vel', 'slider_rev', 'slider_vel_rev_s',
                         'dt', 'catch_k'))


# ── construction guards ──────────────────────────────────────────────────────

@pytest.mark.parametrize("mutate,match", [
    (lambda kw: kw.update(pose=np.zeros((5, 3))), r"pose must be"),
    (lambda kw: kw.update(pose=np.zeros((1, 6)), pose_vel=np.zeros((1, 6)),
                          hand_rev=np.zeros(1), hand_vel_rps=np.zeros(1)),
     r"at least 2 knots"),
    (lambda kw: kw.update(pose_vel=np.zeros((4, 6))), r"pose_vel must match"),
    (lambda kw: kw.update(hand_rev=np.zeros(3)), r"hand_rev/hand_vel_rps"),
    (lambda kw: kw.update(dt=0.0), r"dt must be > 0"),
])
def test_construction_rejects_malformed_input(mutate, match):
    n = 5
    kw = dict(pose=np.zeros((n, 6)), pose_vel=np.zeros((n, 6)),
              hand_rev=np.zeros(n), hand_vel_rps=np.zeros(n), dt=DT)
    mutate(kw)
    with pytest.raises(ValueError, match=match):
        CyclePlan(**kw)


def test_construction_rejects_non_finite_values():
    n = 5
    pose = np.zeros((n, 6))
    pose[2, 1] = np.nan
    with pytest.raises(ValueError, match=r"non-finite"):
        CyclePlan(pose, np.zeros((n, 6)), np.zeros(n), np.zeros(n), DT)
