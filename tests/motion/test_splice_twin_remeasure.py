"""BUG-149: a splice's work must not grow with the record's length.

``extend`` used to re-measure ``stroke_clear_s`` / ``arm_lead_s`` of EVERY
release and catch on the joined plan, each a bisection over the hand
trajectory, so a CATCH install cost ~1 ms more per 100 record Knots (measured
2026-11-28 offline: ``splice_at`` 11.1 ms at 29 Knots, 24.1 ms at 1279; the
solve and the matching check stayed flat).  Only marks whose measurement can
read a knot at or beyond the seam can change, so only those are re-measured.

* (a) determinism: after every install of a long oval-style chain, every mark's
  twin equals the full old-path re-measure on the joined plan, bit for bit,
  and the plan arrays the emitter streams are unchanged below the seam.
* (b) scaling: the number of twin measurements per splice is flat in the chain
  depth (counted, not timed, so the test is parallel-safe).
"""
from __future__ import annotations

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import sites as si
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory.limits import TrajectoryLimits

FLIGHT_S = 0.857
LAUNCH_S = 0.4
DWELL_S = 0.30
T0 = 100.0
N_INSTALLS = 14
LAND_VEL = np.array([0.0, 0.0, -FLIGHT_S / 2.0 * 9806.0])


def _rest_state(cup_mm) -> uc.CycleState:
    cfg = cr.RealizeConfig()
    rev = ((float(cup_mm[2]) - cfg.cup_z_base_mm - cfg.slider_rev_zero_mm)
           / 1000.0 * cr.HAND_REV_PER_M)
    pose = np.array([cup_mm[0], cup_mm[1], cfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, cfg)


@pytest.fixture(scope='module')
def chain():
    """THROW, then ``N_INSTALLS`` cross-site CATCH+throw splices (an oval).

    Returns a list of ``(old_record, new_record, stroke_calls, arm_calls,
    splice_k)``, the call counts being the twin measurements that ONE install
    made and ``splice_k`` the splice knot ``install_segment`` reported."""
    geom = StewartGeometry()
    limits = TrajectoryLimits.from_config(hw).with_sitting_limits(
        leg_vel_mmps=350.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0,
        hand_acc_rps2=float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2))
    p1, p2 = si.columns_sites(100.0)
    rec, res, _ = ex.install_segment(
        None, _rest_state(p1.rest_site_mm()), sg.THROW,
        sg.ThrowTerminal(site_mm=p1.throw_site_mm(),
                         target_mm=p1.catch_site_mm(), flight_s=FLIGHT_S,
                         t_release_s=T0 + LAUNCH_S),
        T0, limits=limits, geom=geom)
    assert res.accepted, res.message

    counts = {'stroke': 0, 'arm': 0}
    orig_s, orig_a = uc.plan_stroke_clear_s, uc.plan_arm_lead_s

    def count_s(*a, **k):
        counts['stroke'] += 1
        return orig_s(*a, **k)

    def count_a(*a, **k):
        counts['arm'] += 1
        return orig_a(*a, **k)

    out = []
    sites = [p1, p2]
    t_rel = T0 + LAUNCH_S
    uc.plan_stroke_clear_s, uc.plan_arm_lead_s = count_s, count_a
    try:
        for i in range(N_INSTALLS):
            a, b = sites[(i + 1) % 2], sites[i % 2]
            t_land = t_rel + FLIGHT_S
            term = sg.CatchTerminal(
                landing_mm=a.catch_site_mm(), landing_vel_mm_s=LAND_VEL,
                t_land_s=t_land, rest_site_mm=a.rest_site_mm(),
                then_throw=sg.ThrowAfterCatch(
                    t_release_s=t_land + DWELL_S, site_mm=a.throw_site_mm(),
                    target_mm=b.catch_site_mm(), flight_s=FLIGHT_S))
            counts['stroke'] = counts['arm'] = 0
            new, res, _ = ex.install_segment(
                rec, None, sg.CATCH, term, t_rel - ex.LEAD_S,
                limits=limits, geom=geom)
            assert res.accepted, res.message
            out.append((rec, new, counts['stroke'], counts['arm'], res.splice_k))
            rec, t_rel = new, t_land + DWELL_S
    finally:
        uc.plan_stroke_clear_s, uc.plan_arm_lead_s = orig_s, orig_a
    return out


def test_every_mark_twin_equals_the_full_remeasure_on_the_joined_plan(chain):
    """(a) Determinism: the OLD path (re-measure every mark on the joined plan)
    is recomputed here as the reference; the new path must match it exactly,
    for every mark, after every install of the chain."""
    n_marks = 0
    for _old, new, _s, _a, _k in chain:
        for m in new.meta.releases:
            assert m.stroke_clear_s == uc.plan_stroke_clear_s(new.plan, m.t_s)
            n_marks += 1
        for m in new.meta.catches:
            assert m.arm_lead_s == uc.plan_arm_lead_s(new.plan, m.t_s)
            n_marks += 1
    assert n_marks > 100          # the chain really is long


def test_the_streamed_head_is_bit_identical_below_the_seam(chain):
    """(a) The emitted Knots are untouched: below the SPLICE knot every per-knot
    array the twins (and the wire) read equals the old record's, bit for
    bit, and the joined plan has a non-zero record origin (``new.t0_s``).  At the splice knot itself the position is
    pinned and only the velocity is re-derived."""
    n_truncated = 0
    for old, new, _s, _a, k_s in chain:
        assert k_s >= 1 and new.t0_s != 0.0
        n_old = old.plan.n_knots
        assert k_s <= n_old - 1
        if k_s < n_old - 1:
            n_truncated += 1      # the head really was cut, not just extended
        o, w = old.plan, new.plan
        channels = (('pose', o.pose, w.pose),
                    ('pose_vel', o.pose_vel, w.pose_vel),
                    ('hand_rev', o.hand_rev, w.hand_rev),
                    ('hand_vel_rps', o.hand_vel_rps, w.hand_vel_rps))
        for name, a, b in channels:
            below = np.nonzero(np.any(
                (a[:k_s] != b[:k_s]).reshape(k_s, -1), axis=1))[0]
            assert below.size == 0, (name, int(below[0]), k_s)
            assert np.array_equal(a[:k_s], b[:k_s]), name
        # The seam knot's POSITION is pinned (velocity may be re-derived).
        assert np.array_equal(o.pose[k_s], w.pose[k_s])
        assert o.hand_rev[k_s] == w.hand_rev[k_s]
        assert w.dt == o.dt
    # Every install of this chain cut the head short (CATCH splices mid-record),
    # so the identity above is not the trivial extend-at-the-end case.
    assert n_truncated == len(chain), (n_truncated, len(chain))


# ---- direct boundary tests of the two predicates --------------------------------

DT = 0.025
MARGIN = uc._ARM_SUPPRESS_MARGIN_S
N_A = 200


def _clear_at(n_stop_dt: float) -> float:
    """A stroke_clear_s whose ``old - margin`` equals ``n_stop_dt * DT``."""
    return n_stop_dt * DT + MARGIN


def test_release_predicate_none_is_always_remeasured():
    for n_a in (2, 3, 50, 1280):
        assert uc._release_twin_needs_remeasure(None, n_a, DT) is True


def test_release_predicate_boundary_at_n_a_minus_3():
    """``old - margin <= (n_a - 3) dt`` is the skip side; 1 microsecond above flips."""
    n_a = N_A
    assert not uc._release_twin_needs_remeasure(_clear_at(n_a - 3), n_a, DT)
    assert uc._release_twin_needs_remeasure(_clear_at(n_a - 3) + 1e-6, n_a, DT)
    # stops (clear = stop knot k * dt + margin) at n_a-2, n_a-1, n_a: remeasure.
    for k in (n_a - 2, n_a - 1, n_a):
        assert uc._release_twin_needs_remeasure(_clear_at(k), n_a, DT), k
    # a stop at n_a-4 and far below: skip.
    for k in (n_a - 4, n_a - 10, 5):
        assert not uc._release_twin_needs_remeasure(_clear_at(k), n_a, DT), k


def test_release_exactly_at_the_seam_is_remeasured():
    """A release at the seam knot has its stop at or beyond it by construction."""
    n_a = N_A
    seam_t = (n_a - 1) * DT
    assert uc._release_twin_needs_remeasure(seam_t + MARGIN, n_a, DT)
    assert uc._release_twin_needs_remeasure(seam_t, n_a, DT)


def test_catch_predicate_boundary():
    """int(t/dt) = n_a-1 and n_a-2 are re-measured (the catch knot is at or one
    short of the seam knot n_a-1); n_a-3 and below keep their carried value."""
    n_a = N_A
    # t a hair above the knot so int() is exact
    def t_of(k):
        return (k + 0.25) * DT
    assert uc._catch_twin_needs_remeasure(t_of(n_a - 1), n_a, DT)
    assert uc._catch_twin_needs_remeasure(t_of(n_a - 2), n_a, DT)
    assert not uc._catch_twin_needs_remeasure(t_of(n_a - 3), n_a, DT)
    assert not uc._catch_twin_needs_remeasure(t_of(n_a - 4), n_a, DT)
    assert not uc._catch_twin_needs_remeasure(t_of(10), n_a, DT)


def test_predicates_skip_only_when_a_full_remeasure_is_bit_equal(chain):
    """Seeded property check on real installs: for a random head mark of the OLD
    record (below the splice knot, as ``extend`` sees it), whenever a predicate
    says skip, the twin carried from the old record equals a FULL re-measure on
    the NEW joined plan, bit for bit.  ``n_a = k_s + 1`` is the head length."""
    rng = np.random.RandomState(20261129)
    n_skips = 0
    for _ in range(50):
        old, new, _s, _a, k_s = chain[int(rng.randint(len(chain)))]
        assert old.plan.dt == DT and new.plan.dt == DT   # the helpers' clock is the plan's
        n_a = k_s + 1
        if rng.randint(2) == 0:
            marks = list(old.meta.releases)
            if not marks:
                continue
            m = marks[int(rng.randint(len(marks)))]
            if int(round(m.t_s / DT)) > k_s:
                continue          # a discarded-tail mark: not a head mark
            if not uc._release_twin_needs_remeasure(m.stroke_clear_s, n_a, DT):
                n_skips += 1
                assert m.stroke_clear_s == uc.plan_stroke_clear_s(
                    new.plan, m.t_s)
        else:
            marks = list(old.meta.catches)
            if not marks:
                continue
            m = marks[int(rng.randint(len(marks)))]
            if int(m.t_s / DT) > k_s:
                continue
            if not uc._catch_twin_needs_remeasure(m.t_s, n_a, DT):
                n_skips += 1
                assert m.arm_lead_s == uc.plan_arm_lead_s(new.plan, m.t_s)
    assert n_skips >= 10, n_skips


def test_twin_measurements_per_splice_do_not_grow_with_record_length(chain):
    """(b) Scaling: counted calls of ``plan_stroke_clear_s`` + ``plan_arm_lead_s``
    during one install.  Before the fix this was one per mark of the whole
    record, so growing by about 2 per install (the logbook's fail-before run
    measured 9, 11, 13, 15, 17, 19, ... with a max of 35); now it is the new
    segment's own marks plus the few head marks near the seam."""
    totals = [s + a for _o, _n, s, a, _k in chain]
    deep = chain[-1][1].plan.n_knots
    assert deep > 500
    assert max(totals) <= 8, totals
    # Flat: the deepest installs cost no more than the early ones did.
    assert max(totals[-4:]) <= max(totals[2:6]), totals
