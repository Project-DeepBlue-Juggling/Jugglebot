"""``unified_cycle.state_at_knot`` / ``splice_at`` — the interior-knot seam.

WHAT THESE TESTS DEFEND
-----------------------
R1 could only ever chain onto a plan's LAST knot (``extend``) or re-aim its
catch (``replan_tail``).  R2 dispatches skills on a wall clock, so a segment
must be able to join a plan that is *already streaming*, at an interior knot.
That seam is the six-leg continuity boundary: the head below it has been sent to
the can-bridge already, and anything that rewrites it is a step command on the
wire at 500 Hz interpolation.  Four claims, one test class each:

* **The head is untouchable.**  ``splice_at`` keeps ``pose[:k_s+1]`` bit for bit
  on all four channels — ``np.array_equal``, not ``allclose``.
* **There is ONE splice, not two.**  ``splice_at`` at the terminal knot IS
  ``extend``, bit for bit, so the module has a single concatenation and a single
  gate range rather than a second copy that can drift.
* **There is ONE splice SEED, not two.**  ``state_at_knot`` is exactly the state
  ``replan_tail`` has always built inline; the refactor is pinned by comparing
  against the former expression, written out here by hand.
* **The refusals name a physical fact.**  A seam that is merely close, a splice
  inside a ball's detach cone, a splice with no head.

Operating point: the owner's R2 point (2026-09-12, ``brief_common.md``) — apex
0.9 m (``flight_s`` 0.857 s), separation 100 mm, leg 300/5000/200000, hand acc
3500 rev/s².  The 250/3000/150000 point ``test_unified_cycle.py`` uses REFUSES
every interior splice tried here with ``LIMIT_JERK``/``LIMIT_ACC`` (measured
2026-09-12, scratchpad probe, run twice with identical output: k_s ∈ {28, 32,
40} on the 1.6 s launch+landing chain at 250/3000/150000 → 154 510 mm/s³ against
the 150 000 cap and worse) — the envelope ``replan_tail``'s docstring already
documents, not a defect here.

Unmarked and parallel-safe: pure planner code, no filesystem, no ports.

Plan: ``plans/active/two-ball-skill-stack.md`` § 2.4 (R2 unit D1).
"""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import sites as si
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory.cycle_plan import CyclePlan
from jugglebot.motion.trajectory.limits import TrajectoryLimits

#: The R2 operating point, § 0 of the plan.
FLIGHT_S = 0.857
LAUNCH_S = 0.4
SEPARATION_MM = 100.0
SESSION_LEG_VEL = 300.0
SESSION_LEG_ACC = 5000.0
SESSION_LEG_JERK = 200000.0
SESSION_HAND_ACC = 3500.0

#: A splice knot past the launch's release (knot 16 at ``LAUNCH_S``) and past
#: its two detach knots — the first knot at which a new window may open.
K_SPLICE = 20


@pytest.fixture(scope='module')
def geom():
    return StewartGeometry()


@pytest.fixture(scope='module')
def limits():
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=SESSION_LEG_VEL, leg_acc_mmps2=SESSION_LEG_ACC,
        leg_jerk_mmps3=SESSION_LEG_JERK, hand_acc_rps2=SESSION_HAND_ACC)


@pytest.fixture(scope='module')
def sites():
    return si.columns_sites(SEPARATION_MM)


def _rest_state(cup_mm) -> uc.CycleState:
    """A resting state whose cup opening sits at ``cup_mm`` (the
    ``test_unified_cycle._rest_state`` fixture, same construction)."""
    cfg = cr.RealizeConfig()
    slider_mm = float(cup_mm[2]) - cfg.cup_z_base_mm
    rev = (slider_mm - cfg.slider_rev_zero_mm) / 1000.0 * cr.HAND_REV_PER_M
    pose = np.array([cup_mm[0], cup_mm[1], cfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, cfg)


@pytest.fixture(scope='module')
def halves(limits, geom, sites):
    """A LAUNCH and the SETTLE chained off its release — the two windows a
    THROW segment is made of, kept apart so ``extend`` and ``splice_at`` can be
    run over the SAME pair."""
    p1, _p2 = sites
    rest_mm = np.array([p1.cup_mm[0], p1.cup_mm[1], uc.SETTLE_CUP_Z_MM])
    goals_a = uc.CycleGoals(period_s=LAUNCH_S,
                            throw_site_mm=p1.throw_site_mm(),
                            throw_target_mm=p1.catch_site_mm(),
                            flight_s=FLIGHT_S, settle_site_mm=rest_mm)
    plan_a, meta_a = uc.plan_launch(goals_a, _rest_state(p1.rest_site_mm()),
                                    limits, geom)
    seed_b = uc.release_state_from_meta(meta_a, plan_a)
    goals_b = uc.CycleGoals(period_s=sg.REST_TAIL_S, settle_site_mm=rest_mm)
    plan_b, meta_b = uc.plan_settle(goals_b, seed_b, limits, geom)
    return plan_a, meta_a, plan_b, meta_b


@pytest.fixture(scope='module')
def throw(halves, limits, geom):
    """The rest-terminal THROW the interior splices are made against."""
    plan_a, meta_a, plan_b, meta_b = halves
    return uc.extend(plan_a, meta_a, plan_b, meta_b, limits, geom)


@pytest.fixture(scope='module')
def catch_segment(throw, limits, geom, sites):
    """A CATCH planned from knot :data:`K_SPLICE` of the THROW — what the
    executor would install a transit later."""
    plan, meta = throw
    _p1, p2 = sites
    seed = uc.state_at_knot(plan, meta, K_SPLICE)
    t_land = LAUNCH_S + FLIGHT_S - K_SPLICE * float(plan.dt)
    terminal = sg.CatchTerminal(
        landing_mm=p2.catch_site_mm(),
        landing_vel_mm_s=np.array([0.0, 0.0, -FLIGHT_S / 2.0 * 9806.0]),
        t_land_s=t_land, rest_site_mm=p2.rest_site_mm())
    return sg.plan_segment(sg.CATCH, seed, terminal, sg.SegmentConfig(),
                           limits, geom)


# ---------------------------------------------------------------------------
# ONE splice seed
# ---------------------------------------------------------------------------

def test_release_state_at_the_terminal_knot_is_release_state_from_meta(halves):
    """One path, pinned field for field.

    ``release_state_from_meta`` is the chain's handoff at a window's LAST knot;
    ``release_state_at_knot`` is the same handoff at an interior release knot,
    which is what a ring's splice lands on.  Two spellings of "the state just
    after a ball left the cup" would be two places to forget the detach axis or
    the ``g`` acceleration — and the cone those pin is the only thing keeping a
    thrown ball from being shoved off the lip.  So the interior one IS the
    implementation and this test pins the terminal case to it.
    """
    plan_a, meta_a, _plan_b, _meta_b = halves
    want = uc.release_state_from_meta(meta_a, plan_a)
    got = uc.release_state_at_knot(plan_a, meta_a, int(plan_a.n_knots) - 1)
    for field in dataclasses.fields(uc.CycleState):
        a = getattr(want, field.name)
        b = getattr(got, field.name)
        if isinstance(a, np.ndarray) or isinstance(b, np.ndarray):
            assert np.array_equal(np.asarray(a), np.asarray(b)), field.name
        else:
            assert a == b, field.name
    assert got.post_release is True
    assert got.detach_axis is not None
    assert np.array_equal(got.cup_accel_mm_s2, uc._G_MM_S2)


def test_release_state_at_knot_refuses_a_knot_with_no_release(throw):
    """A caller asking for a post-release seed where no ball left has mislaid a
    mark — and the window it would build carries a detach cone for a throw that
    never happened."""
    plan, meta = throw
    with pytest.raises(ValueError, match='carries no release'):
        uc.release_state_at_knot(plan, meta, K_SPLICE)


def test_splicing_AT_a_release_knot_keeps_the_release_in_the_head(
        throw, limits, geom, sites):
    """The ring's own handoff: a segment seeded post-release joins AT the
    release knot, which is the SEAM and not part of the discarded tail — so the
    mark survives in the spliced meta and the ball that just left the cup is
    still recorded as thrown.

    The detach refusal still stands for ``k_rel < k_s <= k_rel + n_detach``
    (the test below); the equal case is ``extend``'s own contract, which every
    ordinary LAUNCH+SETTLE join already is.
    """
    plan, meta = throw
    _p1, p2 = sites
    dt = float(plan.dt)
    k_rel = int(round(float(meta.releases[0].t_s) / dt))
    seed = uc.release_state_at_knot(plan, meta, k_rel)
    assert seed.post_release is True
    t_land = LAUNCH_S + FLIGHT_S - k_rel * dt
    terminal = sg.CatchTerminal(
        landing_mm=p2.catch_site_mm(),
        landing_vel_mm_s=np.array([0.0, 0.0, -FLIGHT_S / 2.0 * 9806.0]),
        t_land_s=t_land, rest_site_mm=p2.rest_site_mm())
    seg = sg.plan_segment(sg.CATCH, seed, terminal, sg.SegmentConfig(),
                          limits, geom)

    spliced, new_meta = uc.splice_at(plan, meta, k_rel, seg.plan, seg.meta,
                                     limits, geom)

    assert np.array_equal(spliced.pose[:k_rel + 1], plan.pose[:k_rel + 1])
    assert len(new_meta.releases) == 1
    assert new_meta.releases[0].t_s == pytest.approx(k_rel * dt, abs=0.5 * dt)
    assert new_meta.report.ok, new_meta.report.reasons


def test_state_at_knot_is_replan_tails_former_tail_state(throw):
    """The extracted seed is the inline one, field for field and bit for bit.

    ``replan_tail`` built this expression inline until R2; the refactor is only
    safe if the extracted function is the SAME state, so the former expression
    is written out here by hand and compared exactly.  The one addition is
    ``levelling_correction``, which the inline version left at ``None``: it is
    read only by ``plan_cycle`` (the splice path's new consumer) and never by
    ``to_cup_state`` (``replan_tail``'s only consumer), so carrying it changes
    nothing on the re-plan path and is what makes the splice path inherit the
    frame its predecessor was built in — row E8's in-flight rule.
    """
    plan, meta = throw
    k = K_SPLICE
    cup0 = meta.cup_plan
    former = uc.CycleState(
        pose=plan.pose[k].copy(), pose_vel=plan.pose_vel[k].copy(),
        pose_accel=np.zeros(6), hand_rev=float(plan.hand_rev[k]),
        hand_vel_rps=float(plan.hand_vel_rps[k]),
        detach_axis=None, post_release=False,
        cup_pos_mm=cup0.pos[k] * 1000.0,
        cup_vel_mm_s=cup0.vel[k] * 1000.0,
        cup_accel_mm_s2=cup0.acc[k] * 1000.0)

    got = uc.state_at_knot(plan, meta, k)

    assert np.array_equal(got.pose, former.pose)
    assert np.array_equal(got.pose_vel, former.pose_vel)
    assert np.array_equal(got.pose_accel, former.pose_accel)
    assert got.hand_rev == former.hand_rev
    assert got.hand_vel_rps == former.hand_vel_rps
    assert got.detach_axis is None and got.post_release is False
    assert np.array_equal(got.cup_pos_mm, former.cup_pos_mm)
    assert np.array_equal(got.cup_vel_mm_s, former.cup_vel_mm_s)
    assert np.array_equal(got.cup_accel_mm_s2, former.cup_accel_mm_s2)
    assert got.levelling_correction is meta.levelling_correction
    # And the cup state the QP actually sees is identical too.
    a, b = got.to_cup_state(), former.to_cup_state()
    assert np.array_equal(a.pos, b.pos) and np.array_equal(a.vel, b.vel)
    assert np.array_equal(a.acc, b.acc)


def test_state_at_knot_refuses_a_meta_with_no_cup_track(throw):
    """Without the source cup track there is no exact acceleration to seed from,
    and inferring one would put finite-difference noise into the detach rows."""
    plan, meta = throw
    with pytest.raises(uc.CycleInfeasible) as excinfo:
        uc.state_at_knot(plan, dataclasses.replace(meta, cup_plan=None), 4)
    assert excinfo.value.code == uc.REPLAN_WINDOW


# ---------------------------------------------------------------------------
# ONE splice
# ---------------------------------------------------------------------------

def test_splice_at_the_terminal_knot_is_extend_bit_for_bit(halves, throw,
                                                           limits, geom):
    """``k_s == n_knots - 1`` reduces to today's ``extend``, exactly.

    This is what makes "no second concatenation, no second gate range" a
    checkable claim rather than a comment: if ``splice_at`` ever grew its own
    join, this test would be the first thing to notice.
    """
    plan_a, meta_a, plan_b, meta_b = halves
    want_plan, want_meta = throw

    got_plan, got_meta = uc.splice_at(plan_a, meta_a, plan_a.n_knots - 1,
                                      plan_b, meta_b, limits, geom)

    assert np.array_equal(got_plan.pose, want_plan.pose)
    assert np.array_equal(got_plan.pose_vel, want_plan.pose_vel)
    assert np.array_equal(got_plan.hand_rev, want_plan.hand_rev)
    assert np.array_equal(got_plan.hand_vel_rps, want_plan.hand_vel_rps)
    assert got_plan.catch_k == want_plan.catch_k
    assert got_plan.n_knots == want_plan.n_knots
    assert np.array_equal(got_meta.tilts, want_meta.tilts)
    assert [m.t_s for m in got_meta.releases] == [m.t_s for m in
                                                  want_meta.releases]
    assert got_meta.duration_s == want_meta.duration_s
    # The ONE deliberate difference: the kind, and the solve-cost fields (a
    # splice reports ITS OWN install cost, not the chain's cumulative one).
    assert got_meta.kind == uc.SPLICED
    assert want_meta.kind == uc.JOINED
    assert got_meta.plan_wall_s < want_meta.plan_wall_s


def test_splice_at_an_interior_knot_keeps_the_head_and_re_bases_the_marks(
        throw, catch_segment, limits, geom):
    """The head is bit-identical, the duplicate knot is dropped, and the
    segment's own catch arrives on the JOINT clock, shifted by ``k_s·dt``.

    The head claim is the safety one: knots ``0..k_s`` are at or before
    ``t0 + k_s·dt``, which is what the emitter has already handed the wire.
    """
    plan, meta = throw
    seg = catch_segment
    k_s = K_SPLICE
    dt = float(plan.dt)

    spliced, new_meta = uc.splice_at(plan, meta, k_s, seg.plan, seg.meta,
                                     limits, geom)

    assert np.array_equal(spliced.pose[:k_s + 1], plan.pose[:k_s + 1])
    assert np.array_equal(spliced.pose_vel[:k_s + 1], plan.pose_vel[:k_s + 1])
    assert np.array_equal(spliced.hand_rev[:k_s + 1], plan.hand_rev[:k_s + 1])
    assert np.array_equal(spliced.hand_vel_rps[:k_s + 1],
                          plan.hand_vel_rps[:k_s + 1])
    # One knot dropped at the seam, not two and not none.
    assert spliced.n_knots == k_s + seg.plan.n_knots
    assert new_meta.n_knots == spliced.n_knots
    assert new_meta.duration_s == pytest.approx((spliced.n_knots - 1) * dt)
    assert new_meta.kind == uc.SPLICED
    assert new_meta.report.ok, new_meta.report.reasons

    # The launch's release survives in the head; the segment's catch is re-based.
    assert len(new_meta.releases) == 1
    assert new_meta.releases[0].t_s == pytest.approx(LAUNCH_S)
    assert len(new_meta.catches) == 1
    assert new_meta.catches[0].t_s == pytest.approx(
        k_s * dt + seg.meta.catches[0].t_s)
    assert new_meta.catches[0].knot == seg.meta.catches[0].knot + k_s
    # A spliced plan is itself splice-able: it carries a joint cup track.
    assert new_meta.cup_plan is not None
    assert new_meta.cup_plan.pos.shape[0] == spliced.n_knots
    assert new_meta.cup_plan.jerk.shape[0] == spliced.n_knots - 1
    # ...and the frame is the HEAD's, not re-read from anywhere (row E8).
    assert new_meta.levelling_correction is meta.levelling_correction


def test_splice_at_the_seam_knot_matches_the_segments_knot_zero(throw,
                                                                catch_segment):
    """The segment planned from ``state_at_knot`` really does open ON the head's
    knot ``k_s`` — all seven channels, to the seam tolerance ``_seam_check``
    applies.  If this drifts, every splice becomes a step on six legs."""
    plan, _meta = throw
    seg = catch_segment
    k_s = K_SPLICE
    assert np.max(np.abs(seg.plan.pose[0] - plan.pose[k_s])) < 1e-9
    assert abs(float(seg.plan.hand_rev[0]) - float(plan.hand_rev[k_s])) < 1e-9


# ---------------------------------------------------------------------------
# The refusals
# ---------------------------------------------------------------------------

def test_splice_at_refuses_a_seam_that_does_not_share_the_knot(
        throw, catch_segment, limits, geom):
    """A millimetre of pose gap at the seam is a millimetre step on six legs
    inside one 25 ms knot — the one thing this stack never emits."""
    plan, meta = throw
    seg = catch_segment
    bad_pose = seg.plan.pose.copy()
    bad_pose[0, 0] += 1.0
    bad = CyclePlan(pose=bad_pose, pose_vel=seg.plan.pose_vel,
                    hand_rev=seg.plan.hand_rev,
                    hand_vel_rps=seg.plan.hand_vel_rps, dt=seg.plan.dt,
                    catch_k=seg.plan.catch_k)
    with pytest.raises(uc.CycleInfeasible) as excinfo:
        uc.splice_at(plan, meta, K_SPLICE, bad, seg.meta, limits, geom)
    assert excinfo.value.code == uc.CHAIN_DISCONTINUITY


@pytest.mark.parametrize('k_s', [17, 18])
def test_splice_at_refuses_inside_a_releases_detach_cone(throw, catch_segment,
                                                          limits, geom, k_s):
    """Knots ``k_rel+1 .. k_rel+n_detach`` pin the cup's acceleration DIRECTION
    to the axis the ball left along.  A window opened there is solved with
    ``post_release=False`` and loses those rows — a lateral shove delivered to a
    ball already in the air, and ``validate_cycle`` cannot see it (what is left
    is a perfectly smooth track).

    The release is at knot 16 (``LAUNCH_S`` / dt) and ``n_detach`` is 2, so 17
    and 18 are the cone and 19 is the first legal knot.  The refusal is reached
    before the seam check, so the segment's own seam is irrelevant here.
    """
    plan, meta = throw
    with pytest.raises(uc.CycleInfeasible) as excinfo:
        uc.splice_at(plan, meta, k_s, catch_segment.plan, catch_segment.meta,
                     limits, geom)
    assert excinfo.value.code == uc.REPLAN_WINDOW
    assert 'detach' in ' '.join(excinfo.value.reasons)


@pytest.mark.parametrize('k_s,needle', [(0, 'head'), (10 ** 6, 'knot')])
def test_splice_at_refuses_a_knot_that_is_not_on_the_plan(throw, catch_segment,
                                                          limits, geom,
                                                          k_s, needle):
    """Knot 0 would leave no head to keep; past the terminal knot there is
    nothing to splice onto."""
    plan, meta = throw
    with pytest.raises(uc.CycleInfeasible) as excinfo:
        uc.splice_at(plan, meta, k_s, catch_segment.plan, catch_segment.meta,
                     limits, geom)
    assert excinfo.value.code == uc.REPLAN_WINDOW
    assert needle in ' '.join(excinfo.value.reasons)
