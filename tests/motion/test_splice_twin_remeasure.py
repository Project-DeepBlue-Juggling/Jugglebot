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

    Returns a list of ``(old_record, new_record, stroke_calls, arm_calls)``,
    the call counts being the twin measurements that ONE install made."""
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
            out.append((rec, new, counts['stroke'], counts['arm']))
            rec, t_rel = new, t_land + DWELL_S
    finally:
        uc.plan_stroke_clear_s, uc.plan_arm_lead_s = orig_s, orig_a
    return out


def test_every_mark_twin_equals_the_full_remeasure_on_the_joined_plan(chain):
    """(a) Determinism: the OLD path (re-measure every mark on the joined plan)
    is recomputed here as the reference; the new path must match it exactly,
    for every mark, after every install of the chain."""
    n_marks = 0
    for _old, new, _s, _a in chain:
        for m in new.meta.releases:
            assert m.stroke_clear_s == uc.plan_stroke_clear_s(new.plan, m.t_s)
            n_marks += 1
        for m in new.meta.catches:
            assert m.arm_lead_s == uc.plan_arm_lead_s(new.plan, m.t_s)
            n_marks += 1
    assert n_marks > 100          # the chain really is long


def test_the_streamed_head_is_bit_identical_below_the_seam(chain):
    """(a) The emitted Knots are untouched: below the splice knot every array of
    the new plan equals the old record's, and the origin did not move."""
    for old, new, _s, _a in chain:
        k_s = new.plan.n_knots - 1
        n = min(old.plan.n_knots, new.plan.n_knots)
        # The seam is >= WIRE_READ_KNOTS ahead of the emitter; everything the
        # wire could have read is at least this far below the shorter plan.
        # Compare the common prefix up to the first differing knot: it must
        # be a long run (the whole head) and never start earlier than the head.
        diff = np.nonzero(np.any(old.plan.pose[:n] != new.plan.pose[:n],
                                 axis=1))[0]
        first = int(diff[0]) if diff.size else n
        assert first >= 1 and new.t0_s == old.t0_s
        assert np.array_equal(old.plan.pose[:first], new.plan.pose[:first])
        assert np.array_equal(old.plan.hand_rev[:first],
                              new.plan.hand_rev[:first])
        assert k_s >= first


def test_twin_measurements_per_splice_do_not_grow_with_record_length(chain):
    """(b) Scaling: counted calls of ``plan_stroke_clear_s`` + ``plan_arm_lead_s``
    during one install.  Before the fix this was one per mark of the whole
    record (~2 per install of depth; the logbook's fail-before run measured
    9, 11, 13, 15, 17, 19, ... with a max of 35); now it is the new
    segment's own marks plus the few head marks near the seam."""
    totals = [s + a for _o, _n, s, a in chain]
    deep = chain[-1][1].plan.n_knots
    assert deep > 500
    assert max(totals) <= 8, totals
    # Flat: the deepest installs cost no more than the early ones did.
    assert max(totals[-4:]) <= max(totals[2:6]), totals
