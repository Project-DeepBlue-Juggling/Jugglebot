"""The unified LAUNCH's two accept-time preconditions: the hand FLOOR, and AIM.

Both landed 2026-09-07 out of the 23:55 sitting the night before, in which four
`TossContinuous` goals were refused without a ball leaving the cup. The two are
independent findings that arrived in the same bag, and they are tested together
because they are the two gates a unified goal now has to pass before anything is
armed, tilted or solved for.

**THE FLOOR.** `unified_cycle.SETTLE_CUP_Z_MM` (689.6 mm) is the planner's usable
cup floor — the cup box's bottom, 679.6 mm, plus a 10 mm inset.
`cup_cycle._assemble` writes the z-box rows for knots 1..n (knot 0 is the SEED
and is exempt) and `_seed_relaxed_z_box`'s carve-out applies only to REST-terminal
windows, so a LAUNCH — release-terminal — takes the configured box. A LAUNCH
seeded below the floor is therefore not refused for being low: the QP is required
to put the cup inside the box inside ONE 25 ms knot, and the hand acceleration
that takes is

    |a| = 6·(floor_rev − seed)/dt²,  floor_rev = 0.3161715 at the sitting
    (0.3071 since R1's measured gain), dt = 0.025 s

which is a family, not a threshold. MEASURED (sitting 2026-09-06 23:55, hand at
−0.1087 rev): 4077.2 / 4076.9 / 4063.2 rev/s² against the 3500 cap, and inverting
the closed form recovers the seed from each refusal to four decimals.

⚠ **The half that does not refuse is the half this file cares about most.** The
cap is only crossed at seed −0.0485 rev (11.53 mm low). Every seed between there
and the floor is ACCEPTED and flies a ~0.35 rev knot-1 step at ~14 rev/s with the
launch cup arcing PAST its 860 mm release site (probe 2026-09-07: seed −0.038 ⇒
3400 rev/s², peak cup 875.7 mm). The park the machine sat at twelve hours earlier
was 0.33 mm on the passing side of that line. A guard that only caught the loud
half would leave the machine flying the quiet one, so the rule here is the FLOOR
and not the cap.

**THE AIM.** The unified launch throws vertically by construction
(`_unified_cycle_request` pins `throw_target_mm = throw_site_mm`), and the plan
owns the whole traverse — there is no A→B reach on this path. The legacy displaced
preamble, however, keys on `tier` and not on the planner, so until 2026-09-07 it
ran unbranched: goal 4 of the same sitting asked for 2.384°, mocap held +2.27° at
POSITIONING, the plan's own first knots tilted the machine straight back, and the
ball went vertically up with nothing in any channel saying the aim had been
discarded.
"""

from __future__ import annotations

import time
import types

import pytest

from geometry_msgs.msg import Quaternion
from jugglebot_interfaces.srv import PlanCycle

from jugglebot import reload_coordinator_node as rcn
from jugglebot.motion import unified_cycle as uc
from jugglebot.toss_sequencer import TossSequencer

from tests.ros.test_toss_continuous_node import (
    _Clock,
    _ContGoalHandle,
    _ready_node,
    _stub_cycles,
)
from jugglebot.toss_sequencer import TossResult


# ── The seeds the sitting actually produced ──────────────────────────────────
#: Where the hand rested at 23:55 on 2026-09-06. Cup 676.16 mm — 13.44 mm below
#: the floor — and the seed every one of the four refusals inverts back to.
#: The floor the SITTING's code computed (pre-R1 gain 31.617 rev/m = 1.035 /
#: (2π·0.00521)): the bag's printed HAND_LIMIT_ACC numbers were made with it,
#: so the inversion below uses it and NOT the live floor. Frozen provenance,
#: 2026-09-08 sitting; superseded for planning by hand_mm_per_rev (R1).
FLOOR_REV_SITTING_2026_09_08 = 0.3161715
SEED_SITTING_REV = -0.1087
#: The 12:05 park the same day: 11.20 mm low, |a| 3400 rev/s², which the cap does
#: NOT catch. THE SILENT BAND, and the reason this is a floor rule.
SEED_SILENT_REV = -0.038
#: A hand parked INSIDE the box — the bench carries had left it here the day the
#: same goals were accepted. 5.39 mm of clearance.
SEED_IN_BOX_REV = 0.4865

FLOOR_REV = uc.hand_rev_for_cup_z(uc.SETTLE_CUP_Z_MM)


# ── Harness ──────────────────────────────────────────────────────────────────

def _node(seed_rev, *, commanded_pos=(0.0, 0.0, 170.0)):
    """A toss-ready node with the hand parked at ``seed_rev`` and telemetry fresh.

    `_ready_node` parks the hand at 0.0 rev — the homed zero, which is the cup
    box's BOTTOM and 10.00 mm below the planner's floor, i.e. inside the silent
    band. That is a faithful default for the legacy path and a below-floor seed
    for this one, so every test here states its own seed rather than inheriting
    a number whose meaning changed.
    """
    node = _ready_node(_Clock(), commanded_pos=commanded_pos)
    # `_ready_node` stamps the freshness caches at the FAKE clock's t0 (1000.0).
    # These tests run on the REAL `time` — they drive one method at a time rather
    # than a session, so there is no loop to pace — and a cache stamped 1000.0
    # against a live `perf_counter` reads as decades stale. Re-stamp on the clock
    # the code under test will actually consult.
    now = time.perf_counter()
    with node._lock:
        node._commanded_pos_mono = now
        node._traj_status_mono = now
        node._mocap_mono = now
        node._balls_mono = now
        node._hb_mono = now
    _seed_hand(node, seed_rev)
    return node


def _seed_hand(node, seed_rev):
    with node._lock:
        node._hand_pos_meas = float(seed_rev)
        node._hand_telemetry_mono = time.perf_counter()


def _seq_state(node):
    """A started unified `TossSequencer` and the node's committed cycle slot."""
    seq = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                        flight_time_s=0.8, throw_delay_s=5.0, unified=True)
    seq.start(time.perf_counter())
    seq._prepare_dispatched = True
    node._toss_unified_live = True
    return seq, node._toss_committed


def _accepted_response(**kw):
    resp = PlanCycle.Response()
    resp.accepted = True
    resp.code = 'OK'
    resp.message = 'installed'
    resp.t0_mono = time.perf_counter()
    resp.duration_s = 0.0            # already spent: the wait returns at once
    resp.t_release_mono = time.perf_counter() + 10.0
    resp.release_vel_mm_s = [0.0, 0.0, 2900.0]
    resp.plan_wall_ms = 120.0
    for k, v in kw.items():
        setattr(resp, k, v)
    return resp


def _spy_plan_cycle(node, monkeypatch, *, lands=True, accepted=True):
    """Record every `PlanCycle.Request` and, for a SETTLE that ACCEPTS, move the
    hand where that settle would have put it.

    ``lands=False`` is the settle that installs and does not move the machine —
    the shape sitting 1 taught us to distrust, where a plan running to its
    terminal hold over a de-energised axis is indistinguishable from a pass on
    every channel except the encoder.
    """
    sent = []

    def fake(req, **kw):
        sent.append(req)
        if not accepted:
            resp = _accepted_response()
            resp.accepted = False
            resp.code = 'REJECTED_CYCLE_INFEASIBLE'
            resp.message = ('REJECTED_CYCLE_INFEASIBLE(SETTLE_SITE: settle site '
                            'z 0.6796 m is outside the cup box)')
            return resp, True
        if req.kind == PlanCycle.Request.KIND_SETTLE and lands:
            _seed_hand(node, uc.hand_rev_for_cup_z(req.settle_site_mm[2]))
        return _accepted_response(), True

    monkeypatch.setattr(node, '_call_plan_cycle', fake)
    return sent


def _settles(sent):
    return [r for r in sent if r.kind == PlanCycle.Request.KIND_SETTLE]


def _launches(sent):
    return [r for r in sent if r.kind == PlanCycle.Request.KIND_LAUNCH]


# ═════════════════════════════════════════════════════════════════════════════
# F1 — the lift
# ═════════════════════════════════════════════════════════════════════════════

def test_the_closed_form_reproduces_the_sittings_three_refusals():
    """The arithmetic the whole fix rests on, pinned against the bag.

    Not a tautology: it says the four `HAND_LIMIT_ACC` numbers are ONE fact —
    where the hand was — rather than four goals that happened to be infeasible.
    If this ever stops holding, the lift is fixing the wrong thing.
    """
    assert uc.SETTLE_CUP_Z_MM == pytest.approx(689.6)
    # 10 mm above the hand zero at the MEASURED gain (R1: hand_mm_per_rev).
    assert FLOOR_REV == pytest.approx(0.010 * float(rcn.hw.HAND_REV_PER_M),
                                      abs=1e-6)
    dt = float(uc.cc.CupCycleConfig.dt)
    # The sitting's seed, forward, under the LIVE floor: 3991.3 rev/s², an
    # ABSOLUTE computed once by hand on 2026-09-11 from FLOOR_REV 0.30706 and
    # dt 0.025 (it read 4078.8 on the sitting itself, whose floor was
    # FLOOR_REV_SITTING_2026_09_08).  A literal, not the formula re-run.
    assert rcn.ReloadCoordinatorNode._unified_floor_slam_rps2(
        SEED_SITTING_REV) == pytest.approx(3991.3, abs=0.5)
    # ...and the three refusals, inverted back to it.
    for measured in (4077.2, 4076.9, 4063.2):
        seed = FLOOR_REV_SITTING_2026_09_08 - measured * dt * dt / 6.0
        assert seed == pytest.approx(-0.108, abs=0.002)
    # THE SILENT BAND: the cap is crossed only at -0.0485 rev, so a seed 11.2 mm
    # low is ACCEPTED. This is the number that makes the rule a floor.
    assert rcn.ReloadCoordinatorNode._unified_floor_slam_rps2(
        SEED_SILENT_REV) == pytest.approx(3312.6, abs=1.0)   # 3400.0 pre-R1
    assert (rcn.ReloadCoordinatorNode._unified_floor_slam_rps2(SEED_SILENT_REV)
            < float(rcn.hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2))


def test_a_sitting_seed_is_lifted_by_exactly_one_settle_before_the_launch(
        monkeypatch):
    """F1: −0.1087 rev ⇒ ONE `MODE_NEW`/`KIND_SETTLE` to (xy, 689.6) at 1.0 s,
    and the LAUNCH that follows is planned normally.

    The settle xy is the LIVE commanded platform xy, so the lift is a pure z
    move: a lift that traversed would be a platform motion nobody asked for,
    taken with a ball seated in the cup.
    """
    node = _node(SEED_SITTING_REV, commanded_pos=(20.0, -10.0, 170.0))
    sent = _spy_plan_cycle(node, monkeypatch)

    assert node._unified_floor_lift('test') == ''
    settles = _settles(sent)
    assert len(settles) == 1, sent
    req = settles[0]
    assert req.mode == PlanCycle.Request.MODE_NEW
    assert req.period_s == pytest.approx(rcn._UNIFIED_FLOOR_LIFT_S) == 1.0
    assert req.settle_site_mm[0] == pytest.approx(20.0)
    assert req.settle_site_mm[1] == pytest.approx(-10.0)
    assert req.settle_site_mm[2] == pytest.approx(uc.SETTLE_CUP_Z_MM) == 689.6
    assert req.banking_enabled is True

    # ...and the launch now goes out, because the seed is in the box.
    seq, state = _seq_state(node)
    node._tick_unified_launch(
        seq, state, seq.t_release - rcn._UNIFIED_LAUNCH_LEAD_S + 0.01)
    assert len(_launches(sent)) == 1
    assert state.unified_reject == ''


def test_a_hand_parked_ON_the_floor_is_not_refused_by_noise(monkeypatch):
    """The knife edge the belt would otherwise sit on, and the reason for the
    tolerance.

    **The settle site IS the floor** — the lift's, and every chained LANDING's —
    so a hand that has just done exactly what it was told sits at 0.3162 rev with
    the measurement straddling the line. An exact `<` test would refuse roughly
    every other cycle of a perfectly healthy session in the name of a hazard that
    is not there. The tolerance is 0.01 rev: ~8x the MEASURED 0.0013 rev hold
    error (600 s hold, 2026-09-04) and worth 96 rev/s² of knot-1 acceleration,
    **2.7 % of the cap**, against the 4078 the sitting hit.
    """
    # 0.01 rev expressed in mm at the measured gain: 0.3257 mm (0.3163 pre-R1).
    assert rcn._UNIFIED_FLOOR_TOL_MM == pytest.approx(0.3257, abs=1e-3)
    dt = float(uc.cc.CupCycleConfig.dt)
    slam = 6.0 * rcn._UNIFIED_FLOOR_TOL_REV / (dt * dt)
    assert slam == pytest.approx(96.0, abs=0.5)
    assert slam < 0.03 * float(rcn.hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2)

    # A hand one hold-error BELOW the floor: not a lift, and not a refusal.
    node = _node(FLOOR_REV - 0.0013)
    sent = _spy_plan_cycle(node, monkeypatch)
    assert node._unified_floor_lift('test') == ''
    assert sent == []
    seq, state = _seq_state(node)
    node._tick_unified_launch(
        seq, state, seq.t_release - rcn._UNIFIED_LAUNCH_LEAD_S + 0.01)
    assert len(_launches(sent)) == 1
    assert state.unified_reject == ''

    # ...and one just past the tolerance IS.
    node2 = _node(FLOOR_REV - 2.0 * rcn._UNIFIED_FLOOR_TOL_REV)
    sent2 = _spy_plan_cycle(node2, monkeypatch)
    assert node2._unified_floor_lift('test') == ''
    assert len(_settles(sent2)) == 1


def test_a_hand_already_in_the_box_commands_nothing(monkeypatch):
    """F1's other half: the lift is a NO-OP on a healthy chain.

    Every unified cycle's chained LANDING settles the cup at exactly the floor,
    so cycle N+1 starts where cycle N stopped. A lift that commanded a move
    anyway would spend 1.0 s of every turnaround travelling zero millimetres —
    the census-B1 mistake, one axis over.
    """
    node = _node(SEED_IN_BOX_REV)
    sent = _spy_plan_cycle(node, monkeypatch)
    assert node._unified_floor_lift('test') == ''
    assert sent == []
    assert node._unified_floor_deficit_mm(SEED_IN_BOX_REV) < 0.0


def test_the_silent_band_is_lifted_and_not_launched(monkeypatch):
    """F2's headline: a seed the 3500 cap does NOT catch is still refused a
    launch until it is lifted.

    At −0.038 rev the launch is FEASIBLE — 3400 rev/s², under the cap — and it
    is exactly the throw nobody would have questioned: a 0.354 rev knot-1 step
    and a cup peaking 15.7 mm past its release site. The rule is the floor.
    """
    node = _node(SEED_SILENT_REV)
    seq, state = _seq_state(node)
    sent = _spy_plan_cycle(node, monkeypatch)

    # Un-lifted, the launch is refused before any solve.
    node._tick_unified_launch(
        seq, state, seq.t_release - rcn._UNIFIED_LAUNCH_LEAD_S + 0.01)
    assert _launches(sent) == []
    assert state.unified_reject.startswith(
        '{}({}:'.format(rcn._OUTCOME_CYCLE_PLAN, rcn._UNIFIED_BELOW_FLOOR))
    assert '11.2 mm low' in state.unified_reject
    assert '3313' in state.unified_reject     # 6·(0.30706+0.038)/dt², was '3400'

    # Lifted, the same cycle plans.
    state.unified_reject = ''
    state.unified_launch_pending = True
    assert node._unified_floor_lift('test') == ''
    assert len(_settles(sent)) == 1
    node._tick_unified_launch(
        seq, state, seq.t_release - rcn._UNIFIED_LAUNCH_LEAD_S + 0.01)
    assert len(_launches(sent)) == 1
    assert state.unified_reject == ''


# ═════════════════════════════════════════════════════════════════════════════
# F2 — the belt
# ═════════════════════════════════════════════════════════════════════════════

def test_a_refused_lift_refuses_the_launch_and_says_why(monkeypatch):
    """A lift the planner REFUSES must not become a launch that is attempted.

    The refusal carries the lift's own verdict, because "the launch was refused"
    and "the fix for it did not land, and here is why" are one finding — split
    across two log lines, the second half is the one that gets lost.
    """
    node = _node(SEED_SITTING_REV)
    sent = _spy_plan_cycle(node, monkeypatch, accepted=False)
    detail = node._unified_floor_lift('test')
    assert 'REFUSED' in detail and 'SETTLE_SITE' in detail
    node._unified_lift_detail = detail

    seq, state = _seq_state(node)
    node._tick_unified_launch(
        seq, state, seq.t_release - rcn._UNIFIED_LAUNCH_LEAD_S + 0.01)
    assert _launches(sent) == [], 'a launch was requested from a below-floor seed'
    assert rcn.base_outcome(state.unified_reject) == rcn._OUTCOME_CYCLE_PLAN
    assert rcn.outcome_subcode(state.unified_reject) == rcn._UNIFIED_BELOW_FLOOR
    assert '13.5 mm low' in state.unified_reject   # (0.30706+0.1087) rev at 30.706 rev/m; '13.4' pre-R1
    assert 'SETTLE_SITE' in state.unified_reject       # the lift's verdict rides along


def test_a_lift_that_installs_but_does_not_move_the_hand_is_not_a_pass(
        monkeypatch):
    """VERIFY on the hand, never on the plan.

    Sitting 1 (2026-09-04) cost a whole bench evening to the converse: a stage
    that ran to its terminal hold over an IDLE axis reads as a pass on every
    channel except the encoder. So the lift asks the hand whether it moved, and
    a settle that installed cleanly over a hand that did not is a FAILED lift.
    """
    node = _node(SEED_SITTING_REV)
    sent = _spy_plan_cycle(node, monkeypatch, lands=False)
    detail = node._unified_floor_lift('test')
    assert len(_settles(sent)) == 1
    assert 'STILL 13.5 mm below the floor' in detail   # '13.4' pre-R1
    assert 'did not move' in detail


def test_an_unknown_hand_position_refuses_the_launch_rather_than_guessing(
        monkeypatch):
    """Stale `/hand_telemetry` ⇒ the seed is UNKNOWN ⇒ no launch.

    Absence is not "at the park". A node that has not been told where the hand
    is has no basis for either the lift or the launch, and guessing a park is
    precisely what authorises the throw this refuses.
    """
    node = _node(SEED_IN_BOX_REV)
    with node._lock:
        node._hand_telemetry_mono = 0.0        # never heard
    sent = _spy_plan_cycle(node, monkeypatch)
    assert node._unified_hand_seed_rev() is None
    assert 'UNKNOWN' in node._unified_floor_lift('test')

    seq, state = _seq_state(node)
    node._tick_unified_launch(
        seq, state, seq.t_release - rcn._UNIFIED_LAUNCH_LEAD_S + 0.01)
    assert _launches(sent) == []
    assert rcn.outcome_subcode(state.unified_reject) == rcn._UNIFIED_BELOW_FLOOR
    assert 'UNKNOWN' in state.unified_reject


def test_the_session_lifts_once_at_start_before_any_cycle_is_built(monkeypatch):
    """WHERE the lift sits, as an order: after the planner warm-up (R1: there
    is no more hand-source latch to sit after — owner decision 4), BEFORE the
    first cycle exists.

    It is 1.0 s of motion plus a service round trip, and the only place that is
    free is here — the LAUNCH trigger fires `_UNIFIED_LAUNCH_LEAD_S` (1.80 s)
    before the FSM's scheduled release and every millisecond of that is already
    spoken for by the solve. Spending it inside the lead would push the release
    past `TOSS_RELEASE_GRACE_S` and abort the cycle with the ball in the air.
    """
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    monkeypatch.setattr(rcn.hw, 'JB_OP_UNIFIED_CYCLE_ENABLED', True,
                        raising=False)
    node = _ready_node(clock)
    _seed_hand(node, SEED_SITTING_REV)
    order = []
    monkeypatch.setattr(node, '_unified_warm_planner',
                        lambda: order.append('warm') or 0.0)
    monkeypatch.setattr(
        node, '_unified_prelevel',
        lambda why: (order.append(('prelevel', why)), '')[1])
    monkeypatch.setattr(
        node, '_unified_floor_lift',
        lambda why: (order.append(('lift', why)),
                     _seed_hand(node, FLOOR_REV), '')[2])
    real_build = node._build_toss_cycle
    monkeypatch.setattr(
        node, '_build_toss_cycle',
        lambda *a, **k: (order.append('build'), real_build(*a, **k))[1])
    _stub_cycles(node, monkeypatch, clock,
                 [TossResult(True, 'CAUGHT', 2.0, 0.8)])

    goal = _ContGoalHandle(num_throws=1)
    goal.request.unified_cycle = True
    result = node._execute_toss_continuous(goal)

    assert result.outcome == 'COMPLETED', result.outcome
    assert order[0] == 'warm'
    # The platform is pre-levelled to gravity-level BEFORE the session-start
    # lift, so that lift's banking KIND_SETTLE seeds from the gravity frame it is
    # built in rather than from the un-positioned STANDBY hold (2026-09-11).
    assert order[1] == ('prelevel', 'session start')
    assert order[2] == ('lift', 'session start')
    # ...and once more per cycle, still before the cycle is BUILT. The per-cycle
    # POSITIONING move already carries the pre-level for cycles 2..n, so there is
    # no second prelevel here.
    assert order[3] == ('lift', 'cycle 1')
    assert order[4] == 'build'


def _fake_go_to_pose(node, monkeypatch, *, accepted=True, available=True,
                     answers=True, planned_s=0.3):
    """Capture the go_to_pose request and return a canned response.

    Mirrors the go_to_pose faking in ``test_the_legacy_pretilt_runs_*`` — the
    request is captured for its ORIENTATION and POSITION, and the wait is stubbed
    so the test drives one method without a live service.
    """
    captured = {}
    monkeypatch.setattr(node._go_to_pose_cli, 'wait_for_service',
                        lambda timeout_sec=None: available)
    monkeypatch.setattr(node._go_to_pose_cli, 'call_async',
                        lambda req: captured.__setitem__('req', req))
    resp = None if not answers else types.SimpleNamespace(
        accepted=accepted, code='OK' if accepted else 'LIMIT_JERK',
        planned_duration_s=planned_s,
        message='planned OK' if accepted else 'peak leg jerk 152455 > 30000')
    monkeypatch.setattr(node, '_wait_future',
                        lambda fut, timeout_s=2.0: resp)
    return captured


def test_prelevel_commands_a_pure_attitude_move_with_a_level_intent(monkeypatch):
    """The pre-level move holds the LIVE xy/z and sends an IDENTITY orientation.

    Identity is what the node's E3 ingest corrects into the gravity-level
    counter-tilt, so the launch is seeded from the frame its banking schedule is
    built in. Position is left where the machine stands — a pure attitude move,
    the same primitive `_position_platform_for_toss` sends under unified.
    """
    node = _node(SEED_IN_BOX_REV, commanded_pos=(42.0, -7.0, 170.0))
    captured = _fake_go_to_pose(node, monkeypatch)

    assert node._unified_prelevel('session start') == ''

    req = captured['req']
    assert (req.pose.position.x, req.pose.position.y, req.pose.position.z) == \
        pytest.approx((42.0, -7.0, 170.0))
    q = req.pose.orientation
    level = Quaternion()
    assert (q.x, q.y, q.z, q.w) == (level.x, level.y, level.z, level.w)


def test_prelevel_reports_a_refusal_and_is_non_fatal(monkeypatch):
    """A refused pre-level returns the reason (the caller only WARNs on it).

    The session must not die on a pre-level it could not make: the floor lift and
    launch that follow refuse loudly by name, exactly as they did before this
    move existed, so the pre-level's contract is to REPORT, never to abort.
    """
    node = _node(SEED_IN_BOX_REV)
    _fake_go_to_pose(node, monkeypatch, accepted=False)
    detail = node._unified_prelevel('session start')
    assert 'REFUSED' in detail and 'LIMIT_JERK' in detail


def test_prelevel_without_a_live_pose_refuses_rather_than_guessing(monkeypatch):
    """A stale `commanded_position` refuses — a guessed xy would MOVE the platform."""
    node = _node(SEED_IN_BOX_REV)
    with node._lock:
        node._commanded_pos_mono = 0.0          # never heard
    _fake_go_to_pose(node, monkeypatch)
    detail = node._unified_prelevel('session start')
    assert 'stale' in detail and 'guessed' in detail


def test_prelevel_unavailable_service_is_reported(monkeypatch):
    """No go_to_pose service ⇒ a reason, not an exception."""
    node = _node(SEED_IN_BOX_REV)
    _fake_go_to_pose(node, monkeypatch, available=False)
    detail = node._unified_prelevel('session start')
    assert 'unavailable' in detail


# ═════════════════════════════════════════════════════════════════════════════
# F3 — aim honesty
# ═════════════════════════════════════════════════════════════════════════════

def _unified_goal(**kw):
    gh = _ContGoalHandle(**kw)
    gh.request.unified_cycle = True
    return gh


def test_a_displaced_unified_goal_is_refused_before_anything_is_tilted(
        monkeypatch):
    """F3(a): |B − A| past the co-location tolerance ⇒ refused at ACCEPTANCE.

    Before the warm-up, before the FSM's own displacement gate — and above
    all before the legacy preamble physically tilts the platform to an aim
    the unified plan will discard. R1 (owner decision 4): there is no more
    hand-source latch to check before either.
    """
    monkeypatch.setattr(rcn.hw, 'JB_OP_UNIFIED_CYCLE_ENABLED', True,
                        raising=False)
    node = _node(SEED_IN_BOX_REV, commanded_pos=(0.0, 0.0, 170.0))
    monkeypatch.setattr(
        node, '_position_platform_for_toss',
        lambda *a, **k: pytest.fail('the platform was positioned for a goal '
                                    'that must be refused at acceptance'))
    monkeypatch.setattr(
        node, '_build_toss_cycle',
        lambda *a, **k: pytest.fail('a cycle was built for a refused goal'))

    result = node._execute_toss_continuous(_unified_goal(x=100.0, num_throws=2))

    assert result.success is False
    assert rcn.base_outcome(result.outcome) == rcn._OUTCOME_UNIFIED_AIM
    assert '|B-A| = 100.0 mm' in result.outcome
    assert 'catch_position' in result.outcome
    assert node._goal_claimed is False


def test_a_co_located_unified_goal_still_runs(monkeypatch):
    """The gate is a co-location test, not a ban on unified sessions.

    The shipped goal — `catch_position` at the pre-throw xy — is unchanged, and
    the 5 mm tolerance is far below the scale at which a displacement could
    express an aim (8 mm/s of take-off at the 0.6 s window).
    """
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    monkeypatch.setattr(rcn.hw, 'JB_OP_UNIFIED_CYCLE_ENABLED', True,
                        raising=False)
    node = _ready_node(clock, commanded_pos=(30.0, 0.0, 170.0))
    _seed_hand(node, FLOOR_REV)
    monkeypatch.setattr(node, '_unified_warm_planner', lambda: 0.0)
    monkeypatch.setattr(node, '_unified_floor_lift', lambda why: '')
    built = _stub_cycles(node, monkeypatch, clock,
                         [TossResult(True, 'CAUGHT', 2.0, 0.8)])
    # B within the tolerance of the live A (30, 0): 3 mm out, not an aim.
    result = node._execute_toss_continuous(_unified_goal(x=33.0, num_throws=1))
    assert result.outcome == 'COMPLETED', result.outcome
    assert len(built) == 1


def test_an_unknown_platform_pose_does_not_mint_a_second_pose_unknown(
        monkeypatch):
    """A stale `trajectory/commanded_position` is NOT refused here.

    The FSM's own `REJECTED_POSE_UNKNOWN` already owns that case on its first
    step, and a second spelling of one fact is how a guard and its twin drift
    apart. The physical hazard is closed anyway — the pre-tilt is suppressed for
    every unified cycle, displaced or not (see the POSITIONING test below).
    """
    monkeypatch.setattr(rcn.hw, 'JB_OP_UNIFIED_CYCLE_ENABLED', True,
                        raising=False)
    node = _node(SEED_IN_BOX_REV)
    with node._lock:
        node._commanded_pos_mono = 0.0          # never heard
    result = node._execute_toss_continuous(_unified_goal(x=100.0, num_throws=1))
    # It got PAST the aim gate (no second refusal minted there) — the FSM's
    # own REJECTED_POSE_UNKNOWN, wrapped as a cycle abort, is what stops it.
    # R1: there is no hand_source latch to stop at any more (owner decision 4).
    assert rcn.base_outcome(result.outcome) == 'ABORTED_CYCLE_REJECTED_POSE_UNKNOWN'


@pytest.mark.parametrize('unified,expect_tilt', [(False, True), (True, False)])
def test_the_legacy_pretilt_runs_for_legacy_and_never_for_unified(
        monkeypatch, unified, expect_tilt):
    """F3(b): the POSITIONING move's ORIENTATION, both ways.

    The legacy displaced/aimed toss must still pre-tilt — that is the whole of
    Tier 8b's aim, and nothing here is allowed to weaken it. A unified cycle must
    NOT: `_unified_cycle_request` pins a vertical self-toss, so
    `unified_cycle._throw_tilt_for` returns level and the plan's own first knots
    take the tilt straight back out. Commanding it anyway seeds the window from a
    banked machine, which is the knot-0 hazard `_cycle_start_state` documents.
    """
    node = _node(SEED_IN_BOX_REV)
    node._toss_unified_live = bool(unified)
    seq = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0), flight_time_s=0.8,
                        throw_delay_s=5.0, event_vel_mps=2.0)
    state = node._toss_committed
    with node._lock:
        state.release_cmd = types.SimpleNamespace(
            tilt_rx=0.04, tilt_ry=-0.02,
            pretilt_pose_stow=(0.0, 0.0, 170.0, 0.04, -0.02, 0.0))
        state.positioning_move = True
    captured = {}
    monkeypatch.setattr(node._go_to_pose_cli, 'call_async',
                        lambda req: captured.__setitem__('req', req))
    monkeypatch.setattr(node._go_to_pose_cli, 'wait_for_service',
                        lambda timeout_sec=None: True)
    monkeypatch.setattr(
        node, '_wait_future',
        lambda fut, timeout_s=2.0: types.SimpleNamespace(
            accepted=True, code='OK', planned_duration_s=0.3,
            message='planned OK'))

    node._position_platform_for_toss(seq, state)

    q = captured['req'].pose.orientation
    level = Quaternion()
    tilted = (q.x, q.y, q.z, q.w) != (level.x, level.y, level.z, level.w)
    assert tilted is expect_tilt


def test_the_unified_throw_site_is_where_the_platform_IS(monkeypatch):
    """F3(c): the launch site is read from the LIVE pose, not from the nomination.

    2026-09-06 goal 3 returned the goal's B while the platform stood at A, so the
    LAUNCH was planned and ANNOUNCED at B and the 100 mm of traverse was absorbed
    by the window with nothing measuring it. The aim gate makes B == A upstream;
    this reads the live pose anyway, because a structural guarantee that depends
    on an upstream gate staying in place is not one.
    """
    node = _node(SEED_IN_BOX_REV, commanded_pos=(42.0, -7.0, 170.0))
    seq, _state = _seq_state(node)                 # nominated B is (0, 0)
    assert node._toss_unified_throw_xy(seq) == pytest.approx((42.0, -7.0))
    # ...and the LAUNCH request carries it.
    sent = _spy_plan_cycle(node, monkeypatch)
    node._tick_unified_launch(
        seq, node._toss_committed,
        seq.t_release - rcn._UNIFIED_LAUNCH_LEAD_S + 0.01)
    req = _launches(sent)[0]
    assert req.throw_site_mm[:2] == pytest.approx([42.0, -7.0])
    assert req.throw_site_mm[2] == pytest.approx(rcn._UNIFIED_THROW_CUP_Z_MM)
