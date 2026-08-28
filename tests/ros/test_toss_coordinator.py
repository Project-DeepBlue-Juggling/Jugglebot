"""Toss-side tests for the ball-ops coordinator (reload_coordinator_node).

The toss FSM itself is exhaustively covered by test_toss_sequencer.py; here we test
the node's seams: the wiring surface (second action server, new clients/publishers),
toss observation assembly (hand-evidence chain, possession latch, the config-keyed
mocap arrival cross-check — DISABLED by default, release-evidence latching gated on
OUR dispatch), the goal-numerics gate (REJECTED_BAD_GOAL before anything runs), the
cross-action busy gate (goal claim under _lock at ACCEPT — both directions), the
PREPARE choreography's load-bearing order (prime_hold raised ALONE one tick before
the bundle; in-bundle gains → arm → vel_scale → prime_dispatched stamp → armed →
snapshot, with the announcement on a LATER tick still), the single-shot tri-state
throw dispatch (classification pinned against the REAL teensy_bridge validation
path — no copied strings), the S6 SESSION-scoped arming (one raise and one lower
per contiguous chained run, catch/armed still per-cycle) with its reach-centre
drift guard, the S7 drain-before-go_home, the terminal orderings (prime_hold released LAST), the
trace-only ball-evidence waiver, the per-phase cancel deferral, and the node-level
early exits (cancel/timeout/shutdown/exception all safe before terminalising).

Service calls are monkeypatch-seamed (MockServiceClient futures never resolve) and
time.sleep is monkeypatched where a ladder would otherwise wait — the
test_reload_coordinator_node.py pattern throughout.

ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

import dataclasses
import math
import threading
import time
import types
from unittest import mock

import numpy as np
import pytest

import jugglebot.hardware_config as hw
import jugglebot.reload_coordinator_node as rcn
from jugglebot.motion.trajectory import hand_stroke
# THE production helper: assertions here compare the CODE, and must strip the
# parenthetical the same way the node's own guards do.
from jugglebot.outcome_detail import base_outcome
from jugglebot.reload_coordinator_node import (
    ReloadCoordinatorNode,
    _TOSS_SESSION_REACH_DRIFT_TOL_MM,
    _TOSS_SOFT_CATCH_GAINS,
    _TOSS_WAIVER_PARAM,
    _toss_deadline_s,
)
from jugglebot.reload_sequencer import ReloadSequencer
from jugglebot.toss_sequencer import (
    ACTION_ANNOUNCE,
    ACTION_DISPATCH_THROW,
    ACTION_NONE,
    ACTION_POSITION_PLATFORM,
    ACTION_PREPARE_CATCH,
    ACTION_REACH_CATCH,
    DEFAULT_TOSS_THROW_DELAY_S,
    FLOOR_REPRESENTATION_SLACK_S,
    PHASE_BALL_IN_FLIGHT,
    PHASE_CATCHING,
    PHASE_CHECKING,
    PHASE_POSITIONING,
    PHASE_PREPARING,
    PHASE_SETTLING,
    PHASE_THROWING,
    THROW_DISPATCH_AMBIGUOUS,
    THROW_DISPATCH_OK,
    THROW_DISPATCH_REJECTED,
    TIER_8A,
    TIER_8B,
    TOSS_CONTROL_MODE,
    TossDecision,
    TossResult,
    TossSequencer,
    pre_dispatch_budget_s,
)
from jugglebot.motion import toss_cal
from jugglebot.motion.ik_solver import (
    quat_to_rot_matrix,
    rot_matrix_to_rotvec,
)
from jugglebot.motion.trajectory.toss_release import (
    ThrowTiltInfeasible,
    aim_target_offset_mm,
    build_announcement_fields,
    compute_release_state,
    compute_release_state_tilted,
)


# ── Helpers ────────────────────────────────────────────────────────────────────

class _Pos:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Ball:
    def __init__(self, status, destination='jugglebot', x=0.0, y=0.0, z=809.08,
                 id=5, tracking=0, time_at_land=None):
        self.id = id
        self.status = status
        self.tracking = tracking
        self.destination = destination
        self.position = _Pos(x, y, z)
        self.time_at_land = time_at_land


class _TossGoalHandle:
    def __init__(self, x=0.0, y=0.0, z=170.0, throw_height=0.0, delay=0.0,
                 vel_scale=0.0):
        self.request = types.SimpleNamespace(
            catch_position=types.SimpleNamespace(x=x, y=y, z=z),
            throw_height_m=throw_height, throw_delay_s=delay,
            catch_vel_scale=vel_scale)
        self.is_cancel_requested = False
        self.feedbacks = []
        self.terminal = None

    def publish_feedback(self, fb):
        self.feedbacks.append(fb.phase)

    def succeed(self):
        self.terminal = 'succeed'

    def abort(self):
        self.terminal = 'abort'

    def canceled(self):
        self.terminal = 'canceled'


def _toss_ready_node(now, commanded_pos=(0.0, 0.0, 170.0),
                     commanded_rotvec=(0.0, 0.0, 0.0)):
    """A node with every toss precondition satisfied and caches FRESH at `now`:
    TRAJECTORY streaming, mocap fresh, a fresh trajectory/status affirming a
    loaded gravity correction, a fresh trajectory/commanded_position (the
    displaced toss's throw site A — Phase E; absent it the goal is
    REJECTED_POSE_UNKNOWN), a fresh trajectory/commanded_pose (the SAME sample
    with its INTENT-frame orientation — the census-B1 positioning skip reads it
    and refuses to skip without it), hand telemetry fresh AT the bottom park
    band, ball possession latched, a SEATED hand ball sensor, no live tracks."""
    node = ReloadCoordinatorNode()
    with node._lock:
        node._control_mode = TOSS_CONTROL_MODE
        node._streaming = True
        node._traj_status_mono = now
        node._gravity_correction_loaded = True
        node._commanded_pos_mm = tuple(float(v) for v in commanded_pos)
        node._commanded_pos_mono = now
        node._commanded_pose = (tuple(float(v) for v in commanded_pos)
                                + tuple(float(v) for v in commanded_rotvec))
        node._commanded_pose_mono = now
        node._mocap_mono = now
        node._balls = []
        node._balls_mono = now
        node._hand_pos_meas = 0.0
        node._hand_vel_meas = 0.0
        node._hand_telemetry_mono = now
        node._ball_possession = True
    _feed_ball_sensor(node, now, held=True)
    return node


def _feed_ball_sensor(node, t, held=True, valid=True):
    """Drive the node's hand ball sensor to a definite state at synthetic time `t`.

    TWO samples on purpose. These tests jump the clock in whole seconds, so a
    single sample always arrives after a gap longer than the source's staleness
    bound and is (correctly) treated as the far side of a blind window — which
    reads UNKNOWN, not SEATED. The pair re-establishes the link exactly the way a
    reconnecting node does: the first sample closes the blind span, the second
    lands 10 ms later and re-seeds the verdict. Deliberately produces NO edge, so
    it can never manufacture an arrival — a test that wants one must say so with
    an explicit empty→held pair (see `_feed_sensor_catch`)."""
    node._ball_sensor.note_sample(float(t) - 0.01, held=bool(held), valid=bool(valid))
    node._ball_sensor.note_sample(float(t), held=bool(held), valid=bool(valid))


def _telem(node, *, held=True, raw=None, valid=True):
    """One ``/hand_telemetry`` message through the REAL subscription callback.

    Unlike :func:`_feed_ball_sensor` it runs on the node module's own perf clock
    (the callback stamps with it), which is what the possession LATCH needs: since
    D1 (2026-08-26) the latch is maintained there, off the live ``evidence`` read,
    not from a tracker CAUGHT on ``/balls``."""
    node._on_hand_telemetry(types.SimpleNamespace(
        pos_meas=0.0, vel_meas=0.0, ball_held=bool(held),
        ball_held_raw=bool(held if raw is None else raw),
        ball_held_valid=bool(valid)))


def _feed_sensor_catch(node, t_release, t_catch):
    """Simulate one real throw as the SENSOR sees it: the ball leaves the cup at
    the release stroke and re-enters at the catch. The empty→held edge is what
    C-POSSESS-1 § 3.2 requires before the sensor will confirm an arrival, so a
    node test that drives a tracker CAUGHT without this is asserting a merged
    verdict the sensor vetoes — which is the whole point of the merge."""
    _feed_ball_sensor(node, float(t_release), held=False)
    _feed_ball_sensor(node, float(t_catch), held=False)
    node._ball_sensor.note_sample(float(t_catch) + 0.01, held=True, valid=True)


def _stamp_fresh(node, t, held=True, valid=True):
    """Re-stamp the freshness clocks at synthetic time `t` (the observation
    builder compares them against the step's `now`), including the ball sensor."""
    with node._lock:
        node._mocap_mono = t
        node._balls_mono = t
        node._hand_telemetry_mono = t
        node._traj_status_mono = t
        node._commanded_pos_mono = t
    _feed_ball_sensor(node, t, held=held, valid=valid)


def _install_toss_goal(node, pose=(0.0, 0.0, 170.0), flight=0.8, vel_scale=0.0):
    """Install the per-goal toss state exactly as _execute_toss does (for tests
    that drive _step_toss_sequence / executors directly): goal-start phantom
    snapshot from the current balls cache, platform target kept STOW-relative
    UNCONVERTED (the platform_start-frame comparison)."""
    release = compute_release_state(pose, flight)
    with node._lock:
        node._announced_ball_id = None
        node._announced_id_untagged = False
        node._preexisting_flight_ids = {
            int(b.id) for b in node._balls if int(b.status) == 1}
        node._catch_vel_scale = (vel_scale if vel_scale > 0.0
                                 else float(hw.JB_OP_CATCH_VEL_SCALE_DEFAULT))
        node._toss_committed.release_state = release
        # …and the COMMANDED release, which `_build_toss_cycle` sets to the same
        # object whenever the aim is zero (`release_cmd = release`). It was left
        # None here until 2026-08-28, which was a fixture gap rather than a
        # modelled state: a level release and a None one behave identically in
        # `_release_is_tilted` / `_toss_positioning_xyz` / `_toss_reach_quat`, so
        # nothing noticed — until the staged commit gate started asking the
        # cycle what orientation it nominated, and a None answered "I don't
        # know" (fail-closed) on a machine that plainly did.
        node._toss_committed.release_cmd = release
        node._toss_committed.landing_global_mm = tuple(
            float(v) for v in release.catch_point_global_mm)
        node._toss_committed.platform_target_mm = tuple(float(v) for v in pose)
        node._toss_committed.waiver = False
        node._toss_committed.prepare_pending = False
        node._toss_committed.throw_dispatched = False
        node._toss_committed.stroke_seen = False
        node._toss_committed.track_confirmed = False
    return release


def _fresh_seq(node, pose=(0.0, 0.0, 170.0), flight=0.8, delay=5.0, start=100.0):
    release = _install_toss_goal(node, pose=pose, flight=flight)
    seq = TossSequencer(catch_pose_stow_mm=pose, flight_time_s=flight,
                        throw_delay_s=delay,
                        event_vel_mps=float(release.event_vel_mps))
    seq.start(start)
    # `_execute_toss` latches this before it ticks, and since D1 (2026-08-26) it
    # is load-bearing rather than bookkeeping: `_expected_landing_perf` reads it
    # to give the cup sensor a window, and without it EVERY tick answers
    # ARRIVAL_UNKNOWN/SENSOR_NO_LANDING — i.e. blind, which refuses.
    with node._lock:
        node._active_seq = seq
    return seq


# ── Wiring surface ─────────────────────────────────────────────────────────────

def test_toss_wiring_surface():
    """The merged ball-ops node: BOTH action servers, the toss's new clients
    (go_to_pose / set_hand_traj_cmd / set_hand_gains) and publishers
    (throw_announcements / catch/prime_hold) on the same node as the reload
    surface — the one-process mutual-exclusion premise."""
    node = ReloadCoordinatorNode()
    assert 'jugglebot/reload' in node._action_servers
    assert 'jugglebot/toss' in node._action_servers
    assert 'trajectory/go_to_pose' in node._clients
    assert 'set_hand_traj_cmd' in node._clients
    assert 'set_hand_gains' in node._clients
    assert 'throw_announcements' in node._publishers
    assert 'catch/prime_hold' in node._publishers
    # Both servers share the reentrant group (blocking execute + live caches).
    assert (node._action_servers['jugglebot/toss']._callback_group
            is node._action_servers['jugglebot/reload']._callback_group)


def test_toss_soft_gains_mirror_catch_coordinator():
    """DRIFT GUARD (D3): prime_hold suppresses catch_coordinator's auto-prime —
    the only place soft catch gains are normally set — so the toss sets the SAME
    values itself at PREPARE. A silent divergence would change catch compliance
    only on tosses, invisibly."""
    from jugglebot.catch_coordinator_node import CatchCoordinatorNode
    ccn = CatchCoordinatorNode()
    assert _TOSS_SOFT_CATCH_GAINS['pos_gain'] == pytest.approx(
        ccn._catch_hand_gains['pos_gain'])
    assert _TOSS_SOFT_CATCH_GAINS['vel_gain'] == pytest.approx(
        ccn._catch_hand_gains['vel_gain'])
    assert _TOSS_SOFT_CATCH_GAINS['vel_integrator_gain'] == pytest.approx(
        ccn._catch_hand_gains['vel_int_gain'])


def test_toss_deadline_never_lands_inside_the_flight_window():
    """The per-goal ceiling mirrors reload's doctrine: never below the fixed
    floor and never inside a legitimate sequence window (the timeout path
    SAFE_ABORTs — retract under an airborne ball)."""
    from jugglebot.reload_coordinator_node import _MAX_SEQUENCE_S
    short = TossSequencer(catch_pose_stow_mm=(0, 0, 170.0), flight_time_s=0.8,
                          throw_delay_s=5.0)
    assert _toss_deadline_s(short) == _MAX_SEQUENCE_S
    long = TossSequencer(catch_pose_stow_mm=(0, 0, 170.0), flight_time_s=1.1,
                         throw_delay_s=25.0)
    ceiling = _toss_deadline_s(long)
    assert ceiling > 6.0 + 25.0 + 1.1 + 0.5 + 0.7
    assert ceiling >= _MAX_SEQUENCE_S


# ── Goal accept/reject enumeration (through the real execute path) ─────────────

@pytest.mark.parametrize('breakage,expected', [
    ('tier', 'REJECTED_TIER'),
    ('mode', 'REJECTED_WRONG_MODE'),
    ('mocap', 'REJECTED_MOCAP_STALE'),
    ('streaming', 'REJECTED_NOT_STREAMING'),
    ('not_levelled', 'REJECTED_NOT_LEVELLED'),
    ('traj_status_stale', 'REJECTED_NOT_LEVELLED'),
    ('hand_stale', 'REJECTED_HAND_STALE'),
    ('hand_not_parked', 'REJECTED_HAND_NOT_PARKED'),
    ('no_ball', 'REJECTED_NO_BALL'),
    ('ball_unknown', 'REJECTED_BALL_UNKNOWN'),
    ('track_active', 'REJECTED_TRACK_ACTIVE'),
    ('delay_floor', 'REJECTED_CANT_MAKE_LEAD'),
    ('flight_floor', 'REJECTED_THROW_ENVELOPE'),
    ('flight_ceiling', 'REJECTED_THROW_ENVELOPE'),
    ('displacement', 'REJECTED_DISPLACEMENT'),
    ('workspace', 'REJECTED_WORKSPACE'),
])
def test_toss_goal_rejections_via_execute(breakage, expected, monkeypatch):
    """Every CHECKING reject surfaces through _execute_toss as a loud outcome +
    goal abort. TIER is driven through the GENERATED config gate (the node reads
    hw.JB_OP_TOSS_TIER at goal time — the serviceable set is {8a, 8b}, so an
    UNIMPLEMENTED tier like '9z' is REJECTED_TIER); HAND_STALE / HAND_NOT_PARKED
    / TRACK_ACTIVE / NOT_LEVELLED are the four preconditions the toss adds over
    reload (blind release verification / kind-0 absolute-position stroke hazard
    / F7 phantom-track correlation hole / an un-levelled launch drifting 43 mm
    against a ~35 mm cup). NOT_LEVELLED has two wires — the applier says it
    holds no correction, or the applier stopped saying anything — and both must
    refuse.

    TIER. Every row but the last two runs at whatever tier the config ships
    ('8a' since the operator's 2026-08-10 flip), because the gates they drive
    are tier-agnostic. The two envelope rows PIN 8b at the seam the node reads
    (`hw.JB_OP_TOSS_TIER`, resolved per goal in `_build_toss_cycle`): 8b is a
    CAPABILITY under test here, not the shipped default, and the |B − A| cap
    they are about exists only under it. Before the flip these rows inherited
    8b ambiently, so `displacement` quietly became REJECTED_WORKSPACE the
    moment the YAML changed — the pin is what makes them mean the same thing
    at either shipped default.

    GATE ORDER, and why the two envelope rows read the way they do. Under 8b
    the displaced-throw gates (toss_sequencer's CHECKING block: the |B − A|
    cap, then the closed-form reach bound) run BEFORE the workspace box —
    documented-in-code and intended, because a cap-rejected goal has no valid
    tilted release state and so no meaningful event_vel to check. Consequences,
    measured through this very path on 2026-07-28 (a throw-away probe drove
    _execute_toss over a coordinate sweep; results are the ground truth quoted
    here, not inferred from reading the gate). NOTE the sweep predates Phase E:
    it was taken at the then-70 mm cap, and the cap is
    `hw.JB_OP_TOSS_MAX_DISPLACEMENT_MM` = 150 mm today, so the three middle
    rows are HISTORY, not live expectations. Only the first and last survive
    unchanged at 150 mm, and they are the two the parametrisation drives:

      x=200 → REJECTED_DISPLACEMENT   (200 > 150; this row reads WORKSPACE at 8a)
      x=80  → REJECTED_DISPLACEMENT   x=71 → REJECTED_DISPLACEMENT   [at cap 70]
      x=70  → passes CHECKING         (the cap is `>`, so a goal AT it is legal)
      z=221 → REJECTED_WORKSPACE      z=220 → passes (the ±50 band is `>` too)

    So under 8b with a throw site A = (0, 0), the workspace box's
    |x|,|y| ≤ 150 mm half is STRUCTURALLY UNREACHABLE through this path: the
    displacement cap is never looser than the 150 mm box, so any goal that
    could violate the box laterally is rejected as DISPLACEMENT first. The
    z band is the only reachable WORKSPACE branch, and the row is built as
    x=60 (a LIVE displacement, inside the cap and the 256 mm reach bound
    at T=0.8 s, so the 8b gates genuinely run and pass) plus z=300 (|z − 170| =
    130 mm, past the ±50 mm band). A zero-displacement variant would reach the
    same branch while proving less — it would still pass if the cap collapsed
    to zero and took every real displaced goal with it. The lateral half of the
    box keeps its coverage at FSM level, tier-agnostic, in
    test_toss_sequencer.py::test_workspace_precheck_rejected, and the SHIPPED
    8a reading of the x=200 goal is pinned by
    test_8a_has_no_displacement_cap_so_a_far_goal_reads_workspace below."""
    now = time.perf_counter()
    node = _toss_ready_node(now)
    gh = _TossGoalHandle()
    if breakage == 'tier':
        monkeypatch.setattr(hw, 'JB_OP_TOSS_TIER', '9z')
    elif breakage == 'mode':
        node._control_mode = 'STANDBY'
    elif breakage == 'mocap':
        node._mocap_mono = now - 1.0
    elif breakage == 'streaming':
        node._streaming = False
    elif breakage == 'not_levelled':
        # trajectory_node is alive and talking, and says it holds no correction
        # (it restarted after the `level`, or none was ever pushed).
        node._gravity_correction_loaded = False
    elif breakage == 'traj_status_stale':
        # trajectory_node stopped talking: the cached affirmation is a dead
        # process's answer, so it must not be trusted.
        node._traj_status_mono = now - 5.0
    elif breakage == 'hand_stale':
        node._hand_telemetry_mono = 0.0
    elif breakage == 'hand_not_parked':
        node._hand_pos_meas = 5.0            # mid-stroke, outside the bottom band
    elif breakage == 'no_ball':
        # The sensor positively reads an EMPTY cup. Since 2026-08-10 CHECKING is
        # a LIVE sensor read, so clearing the latch alone no longer produces this
        # code (and must not — that is the stale-belief gate C-POSSESS-1 § 3.3
        # replaced). The gate default is now true, so nothing is monkeypatched.
        node._ball_possession = False
        _feed_ball_sensor(node, now, held=False)
    elif breakage == 'ball_unknown':
        # The other half of the same gate: the sensor cannot answer (a dead
        # poller, boot before the first TxSdo reply). It REFUSES, with its OWN
        # code — a fail-open sensor gate is the BallButler defect this project
        # declined to copy.
        node._ball_possession = True
        _feed_ball_sensor(node, now, held=True, valid=False)
    elif breakage == 'track_active':
        node._balls = [_Ball(status=1, destination='jugglebot', id=9)]
    elif breakage == 'delay_floor':
        # Under the DERIVED :642 dispatch budget (0.281 s at the 0.80 s default
        # flight), not under a flat 3.5 s — that constant retired 2026-08-22
        # (census A1). 2.0 s is now a perfectly legal delay and this row would
        # sail through to REJECTED_POSITION(NO_RESPONSE) if it still asked for it.
        gh = _TossGoalHandle(delay=0.20)
    elif breakage == 'flight_floor':
        # 0.2 m → 0.404 s. Below the DERIVED floor (C-HAND-3): the catch-arm
        # window is −96 ms there, i.e. the arm cannot be placed after the throw
        # stroke clears and still meet the Teensy's :533 budget.
        gh = _TossGoalHandle(throw_height=0.2)
    elif breakage == 'flight_ceiling':
        # 1.8 m → 1.212 s → 5.947 m/s: past the DECEL_FF_HEADROOM ceiling
        # (5.637) but still inside the 7.0 m/s bridge band, so it is the
        # ENVELOPE that refuses and not the wire copy. (1.2 m was this row's
        # driver until 2026-08-20; the measured post-fix coast ladder admits it.)
        gh = _TossGoalHandle(throw_height=1.8)
    elif breakage == 'displacement':
        # 8b is the capability under test, not the shipped default (operator
        # flipped the shipped tier to '8a' on 2026-08-10) — the |B − A| cap is
        # an 8b gate, so the tier is pinned at the seam the node reads.
        monkeypatch.setattr(hw, 'JB_OP_TOSS_TIER', TIER_8B)
        # 200 mm from the live throw site A = (0, 0): past the 150 mm cap.
        gh = _TossGoalHandle(x=200.0)
    elif breakage == 'workspace':
        monkeypatch.setattr(hw, 'JB_OP_TOSS_TIER', TIER_8B)
        # Displacement 60 mm PASSES the 8b gates (cap 150 mm; reach bound
        # 256 mm at T = 0.8 s), then the ±50 mm z band rejects at
        # |z − 170| = 130 mm — which is what makes this the WORKSPACE row and
        # not a second DISPLACEMENT row.
        gh = _TossGoalHandle(x=60.0, z=300.0)
    result = node._execute_toss(gh)
    assert result.success is False
    # REJECTED_THROW_ENVELOPE carries a parenthesised `BOUND:numbers` payload —
    # the REJECTED_POSITION(NO_RESPONSE) shape — so the row pins the CODE and
    # tests/motion/test_throw_envelope.py pins what the payload must contain.
    assert (result.outcome == expected
            or result.outcome.startswith(expected + '(')), result.outcome
    assert gh.terminal == 'abort'
    assert math.isnan(result.catch_error_mm)
    assert math.isnan(result.achieved_flight_s)


def test_8a_has_no_displacement_cap_so_a_far_goal_reads_workspace(monkeypatch):
    """The 8a half of the envelope pair above, pinned rather than inherited.

    Tier 8a pre-positions LEVEL at the nominated catch site and throws from
    there, so throw site == catch site and there is no |B − A| to cap: the
    SAME x=200 goal that is REJECTED_DISPLACEMENT under 8b falls through to the
    workspace box and reads REJECTED_WORKSPACE. Both readings are correct; which
    one the machine gives depends on the shipped tier, which is an operator
    decision that has now moved twice (8a → 8b 2026-07-28, 8b → 8a 2026-08-10).

    Pinning both directions explicitly is the point. While 8b shipped, this
    reading had no test at all and the 8b reading was covered only by accident
    of the config — so the flip turned a config edit into a test failure that
    looked like a code regression. Neither row can do that again."""
    monkeypatch.setattr(hw, 'JB_OP_TOSS_TIER', TIER_8A)
    monkeypatch.setattr(rcn, 'compute_release_state_tilted',
                        lambda *a, **k: pytest.fail(
                            'the tilted 8b aim must not run for an 8a goal'))
    node = _toss_ready_node(time.perf_counter())
    gh = _TossGoalHandle(x=200.0)
    result = node._execute_toss(gh)
    assert result.success is False
    assert base_outcome(result.outcome) == 'REJECTED_WORKSPACE'
    # …and it is the LATERAL box that refused, not the z band — which is what
    # the enriched message makes checkable at the node seam rather than
    # inferable from the goal.
    assert '|B.x| = 200.0 mm' in result.outcome, result.outcome
    assert gh.terminal == 'abort'


@pytest.mark.parametrize('kwargs,field', [
    (dict(throw_height=float('nan')), 'throw_height_m'),
    (dict(delay=float('nan')), 'throw_delay_s'),
    (dict(vel_scale=float('nan')), 'catch_vel_scale'),
    (dict(x=float('nan')), 'catch_position.x'),
    (dict(z=float('inf')), 'catch_position.z'),
    (dict(throw_height=-0.8), 'throw_height_m'),
    (dict(delay=-5.0), 'throw_delay_s'),
    (dict(vel_scale=-0.8), 'catch_vel_scale'),
])
def test_bad_goal_numerics_rejected_before_anything_runs(kwargs, field,
                                                         monkeypatch):
    """The goal-numerics gate (ball_butler_node's non-finite guard pattern): a
    NaN/inf anywhere in the six goal numerics — which would flow through the
    release-state ballistics into a NaN announcement / event_vel — and any
    NEGATIVE tunable (a sign typo; 0.0 is the only "use the default" sentinel;
    coercing would silently run a physically different toss) reject as
    REJECTED_BAD_GOAL(<field>) BEFORE anything runs: no FSM tick, no per-goal
    install, claim released."""
    node = _toss_ready_node(time.perf_counter())
    monkeypatch.setattr(
        node, '_step_toss_sequence',
        lambda seq, now, gh=None, state=None: pytest.fail(
            'FSM ran on a bad goal'))
    gh = _TossGoalHandle(**kwargs)
    result = node._execute_toss(gh)
    assert result.success is False
    assert base_outcome(result.outcome) == 'REJECTED_BAD_GOAL'
    # The FIELD is still named — that is the code's whole purpose — and since
    # 2026-08-29 the offending VALUE rides with it. Ten near-identical goals in
    # a goal storm produced ten identical results before, and the value is the
    # entire diagnosis (a minus sign vs a divide-by-zero).
    assert result.outcome.startswith(
        'REJECTED_BAD_GOAL({} = '.format(field)), result.outcome
    assert ('not finite' in result.outcome
            or 'negative' in result.outcome), result.outcome
    assert gh.terminal == 'abort'
    assert math.isnan(result.catch_error_mm)
    with node._lock:
        assert node._active_seq is None          # nothing was ever installed
        assert node._goal_claimed is False       # claim released on the reject


def test_toss_execute_exception_safes_logs_and_reraises(monkeypatch):
    """An unexpected exception inside the execute loop must not strand a
    half-armed robot or a silent goal: safing runs FIRST (the early-exit path —
    a no-op only when nothing was armed), the one-line-per-goal outcome log
    fires with ABORTED_EXCEPTION, the goal handle is aborted, and the exception
    RE-RAISES so the executor's own error path still sees the fault."""
    node = _toss_ready_node(time.perf_counter())
    order = []
    monkeypatch.setattr(node, '_safe_toss_on_early_exit',
                        lambda seq, state=None: order.append('safed'))
    logged = []
    monkeypatch.setattr(node, '_log_toss_outcome',
                        lambda r: logged.append(str(r.outcome)))
    monkeypatch.setattr(
        node, '_step_toss_sequence',
        lambda seq, now, gh=None, state=None: (
            _ for _ in ()).throw(RuntimeError('boom')))
    gh = _TossGoalHandle()
    orig_abort = gh.abort
    gh.abort = lambda: (order.append('aborted'), orig_abort())
    with pytest.raises(RuntimeError, match='boom'):
        node._execute_toss(gh)
    assert order == ['safed', 'aborted']         # safing strictly before abort
    assert logged == ['ABORTED_EXCEPTION']       # the outcome line fired
    assert gh.terminal == 'abort'
    with node._lock:
        assert node._goal_claimed is False       # finally still released it


# ── Cross-action busy gate (goal claim under _lock at ACCEPT) ──────────────────

def test_goal_claim_taken_at_accept_closes_install_race():
    """The claim is taken IN the goal callback, under _lock — not at execute
    entry — so a second goal racing the accept→install window is REJECTED even
    though _active_seq is still None (the RCN:462-474-class window, now a real
    cross-action race with two servers)."""
    from rclpy.action import GoalResponse
    node = ReloadCoordinatorNode()
    assert node._goal_callback(object()) == GoalResponse.ACCEPT
    assert node._goal_claimed is True            # claimed synchronously at ACCEPT
    with node._lock:
        assert node._active_seq is None          # install has NOT happened yet
    assert node._goal_callback(object()) == GoalResponse.REJECT


@pytest.mark.parametrize('active', ['reload', 'toss'])
def test_busy_cross_action_both_directions(active):
    """A Toss goal while a Reload runs (and vice versa) is REJECTED_BUSY: both
    servers share the one _lock-guarded gate, so two live sequences can never
    double-own the hand on the last-writer-wins queue or fight over the
    catch/armed latch."""
    from rclpy.action import GoalResponse
    node = ReloadCoordinatorNode()
    with node._lock:
        if active == 'reload':
            node._active_seq = ReloadSequencer(catch_point_mm=(0, 0, 809.08))
        else:
            node._active_seq = TossSequencer(catch_pose_stow_mm=(0, 0, 170.0))
    assert node._goal_callback(object()) == GoalResponse.REJECT


def test_goal_claim_released_after_toss_execute():
    """The claim is released in _execute_toss's finally (with _active_seq), so a
    finished toss immediately frees the gate for the next ball-op."""
    from rclpy.action import GoalResponse
    node = _toss_ready_node(time.perf_counter())
    node._control_mode = 'STANDBY'               # fast first-tick reject
    assert node._goal_callback(object()) == GoalResponse.ACCEPT
    node._execute_toss(_TossGoalHandle())
    assert node._goal_claimed is False
    node._control_mode = TOSS_CONTROL_MODE
    assert node._goal_callback(object()) == GoalResponse.ACCEPT


def test_goal_claim_released_after_reload_execute(monkeypatch):
    """Same release discipline on the reload side (its finally also clears the
    claim) — otherwise one reload would wedge the toss server forever."""
    from rclpy.action import GoalResponse
    from jugglebot.reload_sequencer import ReloadDecision, ReloadResult
    node = ReloadCoordinatorNode()
    assert node._goal_callback(object()) == GoalResponse.ACCEPT
    done = ReloadDecision('SETTLING', 'none', True,
                          ReloadResult(False, 'MISSED', float('nan')))
    monkeypatch.setattr(node, '_step_sequence', lambda seq, now, gh=None: done)
    gh = types.SimpleNamespace(
        request=types.SimpleNamespace(throw_delay_s=3.0, catch_vel_scale=0.0),
        is_cancel_requested=False,
        publish_feedback=lambda fb: None,
        succeed=lambda: None, abort=lambda: None)
    node._execute_reload(gh)
    assert node._goal_claimed is False


# ── Trace-only ball-evidence waiver (D4d) ──────────────────────────────────────

def test_waiver_waives_possession_only(monkeypatch):
    """The waiver parameter waives ONLY the possession latch (REJECTED_NO_BALL):
    with it set and no possession the goal proceeds past CHECKING (to the
    positioning reject we script) — but hand freshness and the parked band stay
    HARD, because they gate a physical dispatch hazard the bench can still
    exhibit."""
    monkeypatch.setattr(hw, 'JB_OP_TOSS_REQUIRE_BALL_EVIDENCE', True)
    now = time.perf_counter()
    node = _toss_ready_node(now)
    node._params[_TOSS_WAIVER_PARAM] = True
    node._ball_possession = False
    monkeypatch.setattr(
        node, '_position_platform_for_toss',
        lambda seq, state=None: seq.note_position_result(
            time.perf_counter(), False, 0.0, 'WORKSPACE'))
    result = node._execute_toss(_TossGoalHandle())
    assert result.outcome == 'REJECTED_POSITION(WORKSPACE)'   # past NO_BALL

    # hand_fresh stays hard under the waiver…
    node2 = _toss_ready_node(time.perf_counter())
    node2._params[_TOSS_WAIVER_PARAM] = True
    node2._ball_possession = False
    node2._hand_telemetry_mono = 0.0
    assert node2._execute_toss(_TossGoalHandle()).outcome == 'REJECTED_HAND_STALE'

    # …and so does the parked band.
    node3 = _toss_ready_node(time.perf_counter())
    node3._params[_TOSS_WAIVER_PARAM] = True
    node3._ball_possession = False
    node3._hand_pos_meas = 5.0
    assert (node3._execute_toss(_TossGoalHandle()).outcome
            == 'REJECTED_HAND_NOT_PARKED')


def test_the_ball_evidence_gate_ships_enabled(monkeypatch):
    """The 2026-08-10 flip, pinned at the generated constant.

    The `false` default shipped 2026-07-25 for one stated reason — *"there is NO
    ball-in-cup sensor"*, so a throw could only be gated on an unreliable
    tracker-derived belief and a physically-loaded ball would be refused. That
    condition no longer holds: the G02 switch is wired, polled and validated in
    situ, and CHECKING reads it LIVE. Pinning the constant (not just the
    behaviour) is what makes a silent revert visible."""
    assert bool(hw.JB_OP_TOSS_REQUIRE_BALL_EVIDENCE) is True


def test_the_config_escape_hatch_restores_the_unconditional_pass(monkeypatch):
    """`toss_require_ball_evidence: false` is the operator's escape hatch, and it
    must remain a TOTAL bypass: with it off, a positively-EMPTY cup gets past
    CHECKING exactly as it did before 2026-08-10. Without this the operator has
    no way to run the machine when the sensor is unavailable, and a sensor fault
    would strand a whole sitting.

    The goal proceeds to a scripted positioning reject — hand-parked and the
    other physical-hazard gates stay hard, which the waiver test covers."""
    monkeypatch.setattr(hw, 'JB_OP_TOSS_REQUIRE_BALL_EVIDENCE', False)
    now = time.perf_counter()
    node = _toss_ready_node(now)
    node._ball_possession = False
    _feed_ball_sensor(node, now, held=False)     # sensor says EMPTY, and is right
    monkeypatch.setattr(
        node, '_position_platform_for_toss',
        lambda seq, state=None: seq.note_position_result(
            time.perf_counter(), False, 0.0, 'WORKSPACE'))
    result = node._execute_toss(_TossGoalHandle())
    assert result.outcome == 'REJECTED_POSITION(WORKSPACE)'     # got PAST NO_BALL


def test_an_unknown_sensor_refuses_rather_than_passing(monkeypatch):
    """The safety asymmetry, at the node. BallButler boots `ball_in_hand_ = true`
    and a dead ODrive republishes the stale value forever; for a *throw* gate that
    is fail-OPEN, and this project recorded it as one of three BallButler
    properties deliberately not copied. An UNKNOWN sensor therefore refuses — and
    with a code of its own, because `NO_BALL` would send the operator hunting for
    a ball when the fault is the sensor."""
    now = time.perf_counter()
    node = _toss_ready_node(now)
    node._ball_possession = True                 # a stale belief must not rescue it
    _feed_ball_sensor(node, now, held=True, valid=False)
    assert node._execute_toss(_TossGoalHandle()).outcome == 'REJECTED_BALL_UNKNOWN'
    # …and staleness is the same answer as invalidity: the node stopped hearing
    # the sensor, so it does not know, so it refuses.
    node2 = _toss_ready_node(now)
    _feed_ball_sensor(node2, now - 5.0, held=True)
    node2._hand_telemetry_mono = now             # hand link "fresh", sensor is not
    assert node2._execute_toss(_TossGoalHandle()).outcome == 'REJECTED_BALL_UNKNOWN'


# ── Possession latch (D4a — no ball-in-cup sensor exists) ──────────────────────

def test_the_possession_latch_follows_the_cup_not_the_tracker():
    """**D1, 2026-08-26.** ``_ball_possession`` is "do we have a ball", and the cup
    is what knows. It used to latch on any plausible destination-tagged tracker
    ``CAUGHT`` on ``/balls``, which fails in BOTH directions: it cannot latch a
    catch the tracker never confirmed (12 of 23 on bag 2026-08-26_14-25-16) and it
    cannot un-latch a ball that bounced out — the false positive that comment
    accepted "until the ball-held-sensor era"."""
    node = ReloadCoordinatorNode()
    assert node._ball_possession is False
    # A perfectly plausible tracker CAUGHT, and nothing in the cup: no latch.
    node._on_balls(types.SimpleNamespace(
        balls=[_Ball(status=2, x=10.0, y=-5.0, z=805.0)]))
    assert node._ball_possession is False
    # The cup says SEATED, and no track exists at all: latched. Two samples
    # because the FIRST one after construction has no predecessor and opens a
    # blind span (ball_possession.note_sample's WARM-UP rule).
    _telem(node, held=True, raw=True)
    _telem(node, held=True, raw=True)
    assert node._ball_possession is True
    # The cup empties — a bounce-out during a dwell, the case the old latch was
    # structurally blind to: un-latched, with no release evidence required.
    _telem(node, held=True, raw=False)
    assert node._ball_possession is False


def test_the_possession_latch_ignores_a_blind_cup():
    """UNKNOWN touches neither branch. Blindness is not evidence in either
    direction, so a dead sensor must neither mint possession nor destroy a belief
    that was true when it was last observable."""
    node = ReloadCoordinatorNode()
    _telem(node, held=True, raw=True)
    _telem(node, held=True, raw=True)
    assert node._ball_possession is True
    _telem(node, held=False, raw=False, valid=False)
    assert node._ball_possession is True


def test_a_tracker_caught_on_balls_no_longer_latches_possession():
    """The negative that closes the D1 class rather than one instance: NO track,
    at ANY position and ANY destination, moves the latch — the split-track
    corruption case, the plausible-but-wrong case and the other-robot case alike."""
    for ball in (_Ball(status=2, x=-539.0, y=-323.0, z=-532.0),   # corrupt
                 _Ball(status=2, x=0.0, y=0.0, z=805.0),          # plausible
                 _Ball(status=2, destination='someone_else')):
        node = ReloadCoordinatorNode()
        node._on_balls(types.SimpleNamespace(balls=[ball]))
        assert node._ball_possession is False


def test_possession_survives_safe_abort(monkeypatch):
    """A pre-release SAFE_ABORT retracts the cup WITH the ball (3.16 m/s² ≪ g) —
    possession must survive so the aborted toss is retryable without a Reload."""
    node = ReloadCoordinatorNode()
    node._ball_possession = True
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    monkeypatch.setattr(node, '_smooth_move_hand', lambda p: True)
    monkeypatch.setattr(node, '_arm_catch', lambda a: True)
    monkeypatch.setattr(node, '_go_home', lambda: True)
    node._toss_safe_abort()
    assert node._ball_possession is True


def test_release_evidence_clears_possession_but_the_sensor_overrules_it(monkeypatch):
    """OUR release evidence (the throw-stroke telemetry signature after the
    dispatch) clears possession — but since 2026-08-10 a LIVE sensor read has the
    last word, and that ordering is deliberate rather than incidental.

    The stroke signature fires during the ASCENT (|vel| ≥ 40 rev/s, off the park
    band), and the ball does not leave the cup until the release point near the
    top. So at the instant the release-evidence branch fires, a healthy sensor is
    still reading HELD — and it is *right*. The release clear is a prediction
    ("we saw the stroke, so the ball must be gone"); the sensor is the
    observation. Letting the prediction win would put the machine back to
    believing a model over the cup, which is the whole failure C-POSSESS-1 § 3.2
    made the sensor primary to end.

    The stroke watch itself is untouched — `throw_stroke_seen` is still latched,
    because it is release evidence for the FSM, not a possession claim."""
    now = 100.0
    node = _toss_ready_node(now)                 # sensor SEATED
    _install_toss_goal(node)
    with node._lock:
        node._toss_committed.throw_dispatched = True
        node._hand_vel_meas = 50.0           # ≫ the 40 rev/s stroke threshold
        node._hand_pos_meas = 3.0            # clear of the bottom park band
    obs = node._build_toss_observations(now)
    assert obs.throw_stroke_seen is True         # release evidence still latches
    assert node._ball_possession is True         # the cup still holds it — sensor wins
    assert obs.ball_seated is True

    # Once the ball IS gone, the sensor says so and the latch follows — with no
    # release evidence needed at all (C-POSSESS-1 § 3.3 edit 2: the latch-clear
    # path that a TossContinuous dwell bounce-out previously had no way to take).
    _feed_ball_sensor(node, now + 0.1, held=False)
    obs2 = node._build_toss_observations(now + 0.1)
    assert node._ball_possession is False
    assert obs2.ball_seated is False
    assert obs2.ball_evidence == 'EMPTY'


# ── Phantom-track poisoning (FIX-2: goal-start snapshot + dispatch gating) ─────

def test_preexisting_confirmed_track_does_not_poison_goal(monkeypatch):
    """An untagged IN_FLIGHT + CONFIRMED track present from tick 1 (split-track
    debris, a stale flight) must not poison the goal: the GOAL-START snapshot
    excludes it from the announced-ball latch, its CONFIRMED tracking is not
    release evidence before OUR dispatch, possession survives, and CHECKING
    passes — REJECTED_NO_BALL must NOT fire. (Terminates via a scripted
    positioning reject; the point is that the goal got PAST CHECKING with its
    possession intact.)"""
    now = time.perf_counter()
    node = _toss_ready_node(now)
    with node._lock:
        node._balls = [_Ball(status=1, destination='', id=42, tracking=1)]
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    monkeypatch.setattr(
        node, '_position_platform_for_toss',
        lambda seq, state=None: seq.note_position_result(
            time.perf_counter(), False, 0.0, 'WORKSPACE'))
    result = node._execute_toss(_TossGoalHandle())
    assert result.outcome == 'REJECTED_POSITION(WORKSPACE)'   # past CHECKING
    with node._lock:
        assert node._ball_possession is True          # possession survived
        # no false release evidence
        assert node._toss_committed.track_confirmed is False


def test_track_confirmed_not_latched_before_dispatch(monkeypatch):
    """FIX-2b: the tracker-CONFIRMED release-evidence latch is gated on OUR
    dispatch (mirroring the stroke watch) — before it, a CONFIRMED track can
    only be a phantom, and latching it would clear possession and fake the
    release."""
    monkeypatch.setattr(hw, 'JB_OP_TOSS_REQUIRE_BALL_EVIDENCE', True)
    now = 100.0
    node = _toss_ready_node(now)
    _install_toss_goal(node)
    with node._lock:
        node._balls = [_Ball(status=1, id=5, tracking=1)]   # post-install: latched
    obs = node._build_toss_observations(now)
    assert obs.ball_track_confirmed is False          # pre-dispatch: not evidence
    assert obs.ball_seated is True                    # possession untouched
    with node._lock:
        node._toss_committed.throw_dispatched = True
    obs = node._build_toss_observations(now)
    assert obs.ball_track_confirmed is True           # post-dispatch: latches


def test_prepare_snapshot_adds_and_resets_latch(monkeypatch):
    """FIX-2a/c: the PREPARE-tick snapshot only ADDS ids that went IN_FLIGHT
    since the goal-start snapshot (never clears the baseline), and resets the
    announced-ball latch so nothing latched against a pre-snapshot phantom
    survives into the flight."""
    node = _toss_ready_node(100.0)
    _install_toss_goal(node)
    with node._lock:
        node._preexisting_flight_ids = {7}            # goal-start baseline
        node._announced_ball_id = 42                  # phantom latched pre-PREPARE
        node._announced_id_untagged = True
        node._balls = [_Ball(status=1, destination='', id=42)]
    monkeypatch.setattr(node, '_set_soft_catch_gains', lambda: True)
    monkeypatch.setattr(node, '_arm_catch', lambda a: True)
    assert node._prepare_toss_catch() is True
    with node._lock:
        assert node._preexisting_flight_ids == {7, 42}    # union, baseline kept
        assert node._announced_ball_id is None            # latch reset
        assert node._announced_id_untagged is False


# ── Toss observation assembly ──────────────────────────────────────────────────

# ── platform_levelled (C-LEVEL-1's observability half → REJECTED_NOT_LEVELLED) ──
# The gate observes the node that APPLIES the correction, on a FRESH message,
# never RobotState.levelling_complete. That flag is Teensy-persisted per BOOT: it
# stays True across a relaunch that empties trajectory_node's in-memory
# correction, so a gate wired to it would pass in exactly the state it exists to
# refuse — false assurance, which is strictly worse than today's no gate at all.

def _traj_status(loaded, streaming=True):
    from jugglebot_interfaces.msg import TrajectoryStatus
    msg = TrajectoryStatus()
    msg.streaming = streaming
    msg.gravity_correction_loaded = bool(loaded)
    return msg


def _real_trajectory_node(offset=None):
    """A REAL trajectory_node, seeded and streaming in TRAJECTORY, optionally
    holding `offset` (rad) as its levelling correction. Used so the status
    message the coordinator consumes is built by the producer rather than
    fabricated here — a fabricated one would prove only that the coordinator
    reads a field it was handed."""
    from jugglebot_interfaces.msg import MotorStateSingle, RobotState
    from std_msgs.msg import Float64MultiArray, String
    from jugglebot.trajectory_node import TrajectoryNode

    node = TrajectoryNode(start_emitter=False)
    rs = RobotState()
    rs.motor_states = [
        MotorStateSingle(pos_estimate=float(v))
        for v in hw.JB_OP_ACTIVATE_POSITION_REVS] + [MotorStateSingle()]
    rs.is_homed = True
    node._on_robot_state(rs)
    node._on_control_mode(String(data='TRAJECTORY'))
    if offset is not None:
        node._on_gravity_offset(Float64MultiArray(data=list(offset)))
    node._publish_status()
    return node


@pytest.mark.parametrize('loaded,age_s,expected', [
    (True, 0.0, True),        # fresh affirmation
    (True, 0.5, True),        # inside the window (a 5 Hz topic's worst observed
                              #   inter-arrival on the reference bags is 0.509 s)
    (True, 2.0, False),       # the applier went quiet — a dead process's answer
    (False, 0.0, False),      # alive and says it holds nothing
    (False, 2.0, False),
])
def test_platform_levelled_needs_a_fresh_affirmation_from_the_applier(
        loaded, age_s, expected):
    """`platform_levelled` is an AFFIRMATION with an expiry, not a sticky bool.
    The freshness half is what closes the restart window: between
    trajectory_node dying and its replacement's first status there is no message
    to flip the cache, so without an expiry the coordinator would keep answering
    with the dead process's True while the live one holds identity."""
    now = 100.0
    node = _toss_ready_node(now)
    _install_toss_goal(node)
    with node._lock:
        node._gravity_correction_loaded = loaded
        node._traj_status_mono = now - age_s
    assert node._build_toss_observations(now).platform_levelled is expected


def test_platform_levelled_false_before_any_status_has_ever_arrived():
    """Fail closed on silence. A coordinator that has never heard from
    trajectory_node has no basis to assert the commanded frame is the gravity
    frame, and `now - 0.0` must not be mistaken for a fresh stamp."""
    now = 100.0
    node = ReloadCoordinatorNode()
    _install_toss_goal(node)
    assert node._traj_status_mono == 0.0
    assert node._build_toss_observations(now).platform_levelled is False


def test_on_traj_status_caches_both_the_flag_and_its_arrival():
    now = 100.0
    node = _toss_ready_node(now)
    node._on_traj_status(_traj_status(loaded=False))
    with node._lock:
        assert node._gravity_correction_loaded is False
        assert node._traj_status_mono > 0.0
    node._on_traj_status(_traj_status(loaded=True))
    with node._lock:
        assert node._gravity_correction_loaded is True


def test_live_limits_ride_traj_status_and_expire_with_it():
    """The LIVE session limits (2026-08-14) are cached off trajectory/status and
    fed to the toss FSM ONLY while that status is FRESH — the same window as
    platform_levelled, and for the same reason: a stale cache could carry a
    triple the node has since ramped away from, and the reach bound must judge
    against what the feasibility gate enforces NOW. Stale or never-heard ⇒ 0.0
    ⇒ the sequencer's YAML-default fallback (the pre-field behaviour), never a
    refusal — the bound is a loud-early convenience, the planner stays the
    truth. An old publisher (field-less message) degrades the same way via the
    dataclass/getattr default rather than taking the coordinator down."""
    now = 100.0
    node = _toss_ready_node(now)
    _install_toss_goal(node)
    msg = _traj_status(loaded=True)
    msg.leg_vel_limit_mmps = 1000.0
    msg.leg_acc_limit_mmps2 = 5000.0
    msg.leg_jerk_limit_mmps3 = 15000.0
    node._on_traj_status(msg)
    with node._lock:
        assert node._leg_limits_live == (1000.0, 5000.0, 15000.0)
        node._traj_status_mono = now          # fresh (the real stamp is perf)
    obs = node._build_toss_observations(now)
    assert obs.leg_vel_limit_mmps == pytest.approx(1000.0)
    assert obs.leg_acc_limit_mmps2 == pytest.approx(5000.0)
    assert obs.leg_jerk_limit_mmps3 == pytest.approx(15000.0)
    with node._lock:
        node._traj_status_mono = now - 2.0    # the applier went quiet
    stale = node._build_toss_observations(now)
    assert stale.leg_vel_limit_mmps == 0.0
    assert stale.leg_acc_limit_mmps2 == 0.0
    assert stale.leg_jerk_limit_mmps3 == 0.0
    # Field-less status (pre-2026-08-14 publisher): a bespoke object WITHOUT
    # the limit attributes, so the real getattr-absence path is exercised (the
    # conftest TrajectoryStatus mock now always carries the fields, defaulted
    # 0.0, and cannot test absence). The cache degrades to the absent sentinel
    # rather than keeping the previous triple.
    class _PreLiveLimitsStatus:
        streaming = True
        gravity_correction_loaded = True
    node._on_traj_status(_PreLiveLimitsStatus())
    with node._lock:
        assert node._leg_limits_live == (0.0, 0.0, 0.0)


def test_the_persisted_startup_push_alone_satisfies_the_gate():
    """The NEGATIVE half the plan demands: the gate must NOT fire when only the
    orchestrator's persisted auto-push has run (first IDLE after boot, gated on
    the Teensy's levelling_complete) — that is a legitimate way to have a
    correction loaded, and refusing it would make the gate a re-run-`level`
    ritual rather than a statement about the frame.

    Driven through the REAL producer: the offset the orchestrator would publish
    goes into trajectory_node._on_gravity_offset, trajectory_node's own
    _publish_status builds the message, and THAT message is fed to the
    coordinator's callback. Fabricating the status here instead would prove only
    that the coordinator reads a field it was handed."""
    # Exactly what orchestrator_node's first-IDLE push sends: the tilt persisted
    # on the Teensy, published once, with no `level` run this session.
    traj = _real_trajectory_node(
        offset=[0.013592347421588673, 0.001207157476773584])
    status = traj.status_pub.published[-1]
    assert status.gravity_correction_loaded is True

    now = 100.0
    node = _toss_ready_node(now)
    with node._lock:                       # nothing known yet — as after a boot
        node._gravity_correction_loaded = False
        node._traj_status_mono = 0.0
    node._on_traj_status(status)
    with node._lock:                       # re-stamp onto the synthetic clock
        node._traj_status_mono = now
    _install_toss_goal(node)
    assert node._build_toss_observations(now).platform_levelled is True

    seq = _fresh_seq(node, start=now)
    d = seq.step(now, node._build_toss_observations(now))
    assert d.phase == PHASE_POSITIONING and d.action == ACTION_POSITION_PLATFORM


def test_trajectory_node_restart_flips_the_gate_to_refuse():
    """THE RESTART CASE, end to end. A correction is loaded and the toss is
    serviceable; trajectory_node is then replaced (crash, or the `colcon build`
    + relaunch this contract's own deployment mandates). Nothing republishes
    /gravity_offset — it is VOLATILE and its startup push is latched per
    orchestrator boot — and RobotState.levelling_complete still reads True
    because it lives on the Teensy. The replacement's first status says False
    and the goal must come back REJECTED_NOT_LEVELLED."""
    now = time.perf_counter()
    node = _toss_ready_node(now)

    old = _real_trajectory_node(offset=[0.05, -0.03])
    node._on_traj_status(old.status_pub.published[-1])
    _install_toss_goal(node)
    assert node._build_toss_observations(time.perf_counter()).platform_levelled

    fresh = _real_trajectory_node()                # the post-relaunch process
    node._on_traj_status(fresh.status_pub.published[-1])

    # Re-stamp the SIBLING windows onto the wall clock immediately before the
    # terminal call. This test runs on real perf_counter (the restart it models
    # is about two live processes, not a synthetic clock) and builds two whole
    # TrajectoryNodes in between — measured 0.04 s against the 0.5 s
    # _MOCAP_STALE_S window, i.e. 12x headroom, but a loaded full-suite run that
    # spent that budget would report REJECTED_MOCAP_STALE and look like a gate
    # ordering bug rather than a timing artefact. _stamp_fresh deliberately
    # touches only the caches this test is NOT about: _gravity_correction_loaded
    # stays False (set by the replacement node's status above), so the asserted
    # outcome is unchanged.
    _stamp_fresh(node, time.perf_counter())

    result = node._execute_toss(_TossGoalHandle())
    assert result.success is False
    assert result.outcome == 'REJECTED_NOT_LEVELLED'


def test_hand_parked_band_and_freshness():
    """The hand-evidence chain: parked requires BOTH freshness and the bottom
    park band (|pos − retract| ≤ the ladder's near-band) — a kind-0 stroke
    commands absolute positions from 0 rev."""
    now = 100.0
    node = _toss_ready_node(now)
    _install_toss_goal(node)
    assert node._build_toss_observations(now).hand_parked is True
    with node._lock:
        node._hand_pos_meas = 0.6            # just outside the 0.5 near-band
    assert node._build_toss_observations(now).hand_parked is False
    with node._lock:
        node._hand_pos_meas = 0.0
        node._hand_telemetry_mono = now - 1.0
    obs = node._build_toss_observations(now)
    assert obs.hand_fresh is False
    assert obs.hand_parked is False


def test_track_active_counts_live_destined_tracks_only():
    """TRACK_ACTIVE (F7): live TO_BE_THROWN / IN_FLIGHT tracks destined for us
    refuse the goal; the seated ball's own CAUGHT track (and other robots'
    tracks) do not."""
    now = 100.0
    node = _toss_ready_node(now)
    _install_toss_goal(node)
    with node._lock:
        node._balls = [_Ball(status=2, id=4),                        # our CAUGHT
                       _Ball(status=1, destination='someone_else', id=5)]
    assert node._build_toss_observations(now).track_active is False
    with node._lock:
        node._balls = [_Ball(status=0, destination='jugglebot', id=6)]
    assert node._build_toss_observations(now).track_active is True
    with node._lock:
        node._balls = [_Ball(status=1, destination='jugglebot', id=7)]
    assert node._build_toss_observations(now).track_active is True


def test_mocap_cross_check_disabled_by_default():
    """D7-REVISED: no QTM body for the platform has ever been validated live
    (known bodies: Base, Ball_Butler, Catching_Cone), so the arrival
    cross-check ships DISABLED (toss_mocap_body defaults to ''): the node
    feeds platform_at_target True unconditionally — arrival rests on the
    go_to_pose accept + planned-duration wait — and _on_mocap caches nothing.
    Enabling requires naming a bench-resolved body (Phase-3 dry trace / T0)."""
    now = 100.0
    node = _toss_ready_node(now)
    _install_toss_goal(node)
    assert node._toss_mocap_body == ''
    with node._lock:
        node._platform_pos_mm = None          # no platform body ever seen
    assert node._build_toss_observations(now).platform_at_target is True
    body = types.SimpleNamespace(
        name='Base', pose=types.SimpleNamespace(pose=types.SimpleNamespace(
            position=_Pos(1.0, 2.0, 3.0))))
    node._on_mocap(types.SimpleNamespace(bodies=[body]))
    assert node._platform_pos_mm is None      # nothing cached while disabled


def test_platform_at_target_cross_check_configured_body():
    """The mocap arrival cross-check WHEN a body is configured: measured
    position within the reach-envelope radius of the nominated PLATFORM pose,
    compared in the STOW/platform_start frame (non-base bodies publish z
    shifted by the base→platform transform, so ACTIVE reads z ≈ 170 — never
    converted to global, which was the review-caught double-add). The silent
    no-op class (disarmed wire, guard latch) fails it."""
    now = 100.0
    node = _toss_ready_node(now)
    node._toss_mocap_body = 'Platform'
    _install_toss_goal(node, pose=(60.0, -60.0, 170.0))
    with node._lock:
        node._platform_pos_mm = (60.0, -60.0, 170.0)     # at the nominated pose
    assert node._build_toss_observations(now).platform_at_target is True
    with node._lock:
        node._platform_pos_mm = (0.0, 0.0, 170.0)        # stayed home: 84.9 mm off
    assert node._build_toss_observations(now).platform_at_target is False
    with node._lock:
        node._platform_pos_mm = None                     # body never seen
    assert node._build_toss_observations(now).platform_at_target is False
    with node._lock:
        node._platform_pos_mm = (60.0, -60.0, 170.0)
        node._mocap_mono = now - 1.0                     # stale mocap can't verify
    assert node._build_toss_observations(now).platform_at_target is False


def test_platform_position_cached_for_configured_body_only():
    """_on_mocap caches the CONFIG-NAMED body's position AS PUBLISHED (the
    platform_start frame for non-base bodies) — and only that body's."""
    node = ReloadCoordinatorNode()
    node._toss_mocap_body = 'Platform'
    body = types.SimpleNamespace(
        name='Platform',
        pose=types.SimpleNamespace(pose=types.SimpleNamespace(
            position=_Pos(30.0, -40.0, 170.0))))
    other = types.SimpleNamespace(
        name='Base', pose=types.SimpleNamespace(pose=types.SimpleNamespace(
            position=_Pos(1.0, 2.0, 3.0))))
    node._on_mocap(types.SimpleNamespace(bodies=[other, body]))
    assert node._platform_pos_mm == (30.0, -40.0, 170.0)


def test_stroke_watch_threshold_clears_smooth_move_prelude():
    """FIX-5a: the release-stroke threshold sits ABOVE the kind-1 smooth-move
    prelude's peak ascent and well BELOW the ≈85 rev/s minimum release-band
    stroke speed (2.7 m/s sweep floor). Behavioural pin: a prelude-speed ascent
    does NOT latch, a release-band ascent does. T0 re-tunes against measured
    stroke telemetry.

    **Two peak figures, and the distinction is load-bearing.** ``√(a·Δx)`` =
    31.56 rev/s is the peak a BANG-BANG profile would reach; the firmware ships
    a QUINTIC, whose commanded peak is ``|Δ|·1.875/T`` = 24.63 rev/s — 22 %
    lower. The guard clearing the bound is the conservative thing for a guard to
    do and that inequality is the one pinned first below, but the bound is not
    what the hand does, so the commanded peak is pinned too and it is the figure
    the HAND-3 bench row is scored against. Until 2026-07-26 this docstring, the
    ``_THROW_STROKE_VEL_RPS`` comment and the bench row all called the bound
    "the peak"; scoring a capture that way accepts a 28 % overspeed as nominal,
    and overspeed on a smooth move is the re-seeded/clobbered-profile signature.

    The 31.4 → 31.56 restatement is the 2026-07-26 prime move (9.858 → the
    derived stroke top 9.9594 rev), which lengthens the full-stroke prime ascent
    and so raises both figures. The TOLERANCE and both inequalities below are
    unchanged; the guard's margin against the 40 rev/s threshold went 8.60 →
    8.44 rev/s on the bound (15.50 → 15.37 rev/s on the commanded peak)."""
    from jugglebot.reload_coordinator_node import _THROW_STROKE_VEL_RPS
    prelude_peak = math.sqrt(100.0 * float(hw.JB_OP_HAND_CATCH_PRIME_REV))
    assert prelude_peak == pytest.approx(31.56, abs=0.1)
    assert _THROW_STROKE_VEL_RPS - prelude_peak == pytest.approx(8.44, abs=0.01)
    assert _THROW_STROKE_VEL_RPS > prelude_peak
    assert _THROW_STROKE_VEL_RPS < 85.0
    # The bound must BE a bound on the commanded peak, and the guard must clear
    # the commanded peak by more than it clears the bound.
    commanded_peak = hand_stroke.smooth_move_peak_vel_rps(
        float(hw.JB_OP_HAND_CATCH_PRIME_REV))
    assert commanded_peak == pytest.approx(24.63, abs=0.01)
    assert commanded_peak < prelude_peak
    assert prelude_peak == pytest.approx(
        hand_stroke.smooth_move_peak_vel_bound_rps(
            float(hw.JB_OP_HAND_CATCH_PRIME_REV)), rel=1e-9)
    assert _THROW_STROKE_VEL_RPS - commanded_peak == pytest.approx(15.37,
                                                                   abs=0.01)
    now = 100.0
    node = _toss_ready_node(now)
    _install_toss_goal(node)
    with node._lock:
        node._toss_committed.throw_dispatched = True
        node._hand_pos_meas = 3.0            # clear of the bottom park band
        node._hand_vel_meas = 35.0           # prelude-speed ascent: NOT a throw
    assert node._build_toss_observations(now).throw_stroke_seen is False
    with node._lock:
        node._hand_vel_meas = 45.0           # above threshold: throw underway
    assert node._build_toss_observations(now).throw_stroke_seen is True


def test_ball_time_at_land_crossed_to_perf():
    """The tracker's landing-plane crossing is ROS→perf crossed exactly like the
    announcement landing (perf = ros + (now_perf − now_ros); MockClock's ros now
    is 0). The dispatch flag is set — the CONFIRMED latch is dispatch-gated
    (FIX-2b)."""
    now = 100.0
    node = _toss_ready_node(now)
    _install_toss_goal(node)
    ball = _Ball(status=1, id=5, tracking=1,
                 time_at_land=types.SimpleNamespace(sec=12, nanosec=500000000))
    with node._lock:
        node._balls = [ball]
        node._toss_committed.throw_dispatched = True
    obs = node._build_toss_observations(now)
    assert obs.ball_track_confirmed is True
    assert obs.ball_time_at_land_perf == pytest.approx(12.5 + 100.0)


def test_the_reported_catch_error_comes_from_the_latched_id_vs_nominated_point():
    """``catch_error_mm`` is the LATCHED announced ball's lateral miss against the
    NOMINATED landing point (not reload's fixed catch point).

    Since D1 (2026-08-26) the tracker no longer decides WHETHER a catch happened,
    so this test's subject narrowed to what it reports: the cup mints the verdict
    on every branch below, and only the reported number moves. A stray id
    contributes nothing (NaN — the honest reading, and the same one a catch with
    no track at all now produces)."""
    now = 100.0
    node = _toss_ready_node(now)
    pose = (30.0, -40.0, 170.0)
    seq = _fresh_seq(node, pose=pose, delay=5.0, start=now)   # landing 105.8
    with node._lock:
        node._active_seq = seq
        node._balls = [_Ball(status=1, id=5)]            # latch id 5
    node._build_toss_observations(now)
    # The cup sees the ball arrive +0.11 s past the scheduled landing.
    _stamp_fresh(node, now + 5.9, held=False)
    node._ball_sensor.note_sample(now + 5.91, held=True, valid=True, raw=True)
    t = now + 5.92
    with node._lock:
        node._balls = [_Ball(status=2, id=99, x=30.0, y=-40.0),   # stray id
                       _Ball(status=1, id=5)]
    obs = node._build_toss_observations(t)
    assert obs.ball_caught is True                       # the CUP saw it
    assert obs.catch_error_mm != obs.catch_error_mm      # NaN: not OUR track
    with node._lock:
        node._balls = [_Ball(status=2, id=5, x=45.0, y=-40.0, z=805.0)]
    obs = node._build_toss_observations(t)
    assert obs.ball_caught is True
    assert obs.catch_error_mm == pytest.approx(15.0)     # vs (30, −40), not (0, 0)


# ── PREPARE bundle order + the announce gap ────────────────────────────────────

def _wire_prepare_recorder(node, order, monkeypatch, arm_ok=True):
    monkeypatch.setattr(node, '_set_soft_catch_gains',
                        lambda: order.append('gains') or True)
    monkeypatch.setattr(node, '_publish_prime_hold',
                        lambda hold: order.append(('prime_hold', hold)))
    monkeypatch.setattr(node, '_arm_catch',
                        lambda a: order.append(('arm', a)) or arm_ok)
    vel_pub = node._publishers['catch/vel_scale']
    monkeypatch.setattr(
        vel_pub, 'publish',
        lambda msg: order.append(('vel_scale', float(msg.data))))
    monkeypatch.setattr(
        node._prime_dispatched_pub, 'publish',
        lambda msg: order.append(('prime_dispatched', bool(msg.data))))
    monkeypatch.setattr(node, '_publish_catch_armed',
                        lambda a: order.append(('armed', a)))


def test_prepare_bundle_order_pinned(monkeypatch):
    """THE in-bundle PREPARE ordering (each step's position is load-bearing):
    gains first (the standing prime_hold suppresses the auto-prime that
    normally sets them) → latch raise+confirm → vel_scale BEFORE armed (the
    coordinator must hold this goal's scale before any hand-arm) → the ONE
    prime_dispatched stamp immediately before armed (the belt: it holds CCN's
    1.2 s prime-inflight window across the armed edge if the hold was
    lost/reordered) → armed → phantom-flight snapshot. prime_hold is NOT in
    the bundle — the node raised it one FSM tick earlier (pinned by
    test_prime_hold_raised_tick_before_prepare_bundle) — and the announcement
    is NOT in the bundle either (separate later tick)."""
    node = _toss_ready_node(100.0)
    _install_toss_goal(node, vel_scale=0.75)
    with node._lock:
        node._balls = [_Ball(status=1, id=77, destination='')]   # phantom flight
    order = []
    monkeypatch.setattr(
        node._publishers['throw_announcements'], 'publish',
        lambda msg: order.append('announce'))
    _wire_prepare_recorder(node, order, monkeypatch)
    assert node._prepare_toss_catch() is True
    assert order == ['gains', ('arm', True), ('vel_scale', 0.75),
                     ('prime_dispatched', True), ('armed', True)]
    with node._lock:
        assert node._preexisting_flight_ids == {77}      # snapshot refreshed at PREPARE


def test_prepare_arm_failure_stops_bundle(monkeypatch):
    """A failed latch raise returns False (→ ABORTED_PREPARE_FAILED upstream)
    and must publish NEITHER vel_scale NOR prime_dispatched NOR catch/armed —
    arming a sequence that is being aborted is the exact hazard the ordering
    exists to prevent. The prime_hold raised on the previous tick stays True
    here; the SAFE_ABORT terminal releases it."""
    node = _toss_ready_node(100.0)
    _install_toss_goal(node)
    order = []
    _wire_prepare_recorder(node, order, monkeypatch, arm_ok=False)
    assert node._prepare_toss_catch() is False
    assert order == ['gains', ('arm', True)]


def test_prime_hold_raised_tick_before_prepare_bundle(monkeypatch):
    """FIX-4: the hold is published ALONE on the verified-arrival
    (ACTION_PREPARE_CATCH) tick and the bundle runs on the NEXT tick — the
    hold and catch/armed have no cross-topic ordering guarantee, and callbacks
    landing in the SAME CCN wait-set cycle have no intra-cycle ordering
    either, so a same-tick hold could lose to the armed edge and auto-prime
    the ball-laden hand. Pins: hold tick STRICTLY before armed tick;
    prime_dispatched immediately before armed within the bundle tick."""
    node = _toss_ready_node(100.0)
    seq = _fresh_seq(node, start=100.0)
    events = []                                  # (tick_index, event)
    tick = {'i': 0}
    monkeypatch.setattr(
        node, '_position_platform_for_toss',
        lambda s, state=None: s.note_position_result(
            100.0, True, 0.3))                               # arrival 100.5
    monkeypatch.setattr(node, '_set_soft_catch_gains', lambda: True)
    monkeypatch.setattr(node, '_arm_catch', lambda a: True)
    monkeypatch.setattr(
        node, '_publish_prime_hold',
        lambda h: events.append((tick['i'], ('prime_hold', h))))
    monkeypatch.setattr(
        node._prime_dispatched_pub, 'publish',
        lambda msg: events.append(
            (tick['i'], ('prime_dispatched', bool(msg.data)))))
    monkeypatch.setattr(
        node, '_publish_catch_armed',
        lambda a: events.append((tick['i'], ('armed', a))))
    for t in (100.0, 100.6, 100.7):
        tick['i'] += 1
        _stamp_fresh(node, t)
        node._step_toss_sequence(seq, t)
    hold_tick = next(i for i, e in events if e == ('prime_hold', True))
    armed_tick = next(i for i, e in events if e == ('armed', True))
    assert hold_tick == 2 and armed_tick == 3    # strictly earlier FSM tick
    flat = [e for _, e in events]
    assert flat[flat.index(('prime_dispatched', True)) + 1] == ('armed', True)


def test_reach_centre_declared_a_tick_before_the_arm_raise(monkeypatch):
    """C-REACH-1: catch/reach_center carries the goal's NOMINATED catch B and is
    published on the ACTION_PREPARE_CATCH tick — one full FSM tick (50 ms >>
    topic latency) BEFORE the bundle that calls trajectory/arm_catch, which is
    where trajectory_node consumes it.

    Same cross-topic-ordering reason that split prime_hold off the bundle: the
    declaration and the service call travel on different transports with no
    ordering guarantee. Losing the race degrades to the pre-contract behaviour —
    envelope at A, the A→B reach rejected WORKSPACE mid-flight — which is a
    missed ball, not a hazard, and is why the declaration needs no ack."""
    pose = (120.0, -30.0, 170.0)
    node = _toss_ready_node(100.0)
    seq = _fresh_seq(node, pose=pose, start=100.0)
    events = []
    tick = {'i': 0}
    monkeypatch.setattr(
        node, '_position_platform_for_toss',
        lambda s, state=None: s.note_position_result(
            100.0, True, 0.3))                               # arrival 100.5
    monkeypatch.setattr(node, '_set_soft_catch_gains', lambda: True)
    monkeypatch.setattr(
        node, '_arm_catch',
        lambda a: events.append((tick['i'], ('arm', a))) or True)
    monkeypatch.setattr(node, '_publish_catch_armed', lambda a: None)
    monkeypatch.setattr(node, '_publish_prime_hold', lambda h: None)
    monkeypatch.setattr(
        node._reach_center_pub, 'publish',
        lambda msg: events.append(
            (tick['i'], ('centre', (msg.x, msg.y, msg.z)))))
    for t in (100.0, 100.6, 100.7):
        tick['i'] += 1
        _stamp_fresh(node, t)
        node._step_toss_sequence(seq, t)
    centre = [(i, e) for i, e in events if e[0] == 'centre']
    assert len(centre) == 1                       # declared ONCE per goal
    assert centre[0][1][1] == pose                # the NOMINATED B, verbatim
    arm_tick = next(i for i, e in events if e == ('arm', True))
    assert centre[0][0] == 2 and arm_tick == 3    # strictly earlier FSM tick


def test_reach_centre_declared_for_tier_8b_too(monkeypatch):
    """The 8b goal declares B while the platform is pre-tilted at A — the case
    the contract exists for. The declared centre is B, NOT the pre-tilt pose the
    platform is actually holding."""
    node = _toss_ready_node(100.0, commanded_pos=(0.0, 0.0, 170.0))
    seq, _rel = _fresh_seq_8b(node, pose=(140.0, 0.0, 170.0),
                              throw_site=(0.0, 0.0), start=100.0)
    published = []
    monkeypatch.setattr(
        node, '_position_platform_for_toss',
        lambda s, state=None: s.note_position_result(100.0, True, 0.3))
    monkeypatch.setattr(node, '_publish_prime_hold', lambda h: None)
    monkeypatch.setattr(node, '_publish_pretilt_hold', lambda h: None)
    monkeypatch.setattr(node._reach_center_pub, 'publish',
                        lambda msg: published.append((msg.x, msg.y, msg.z)))
    for t in (100.0, 100.6):
        _stamp_fresh(node, t)
        node._step_toss_sequence(seq, t)
    assert published == [(140.0, 0.0, 170.0)]


# ── S6/S7: the SESSION-scoped catch arming ────────────────────────────────────
# plans/active/toss-pipelined-preamble.md § 2.3 (S6, S7) and § 4 (Phase B3).
# A contiguous run of chained cycles raises trajectory/arm_catch, the two
# catch_coordinator holds and the ONE catch/reach_center declaration EXACTLY
# once, and lowers them exactly once — so there is no per-cycle re-raise left to
# land inside a go_home. catch/armed stays PER-CYCLE (the S6 amendment).

def _wire_session_recorder(node, order, monkeypatch, arm_ok=True):
    """Record every publish/service the session arming touches, in order.

    A superset of :func:`_wire_prepare_recorder` — it adds the reach-centre
    declaration and the pretilt hold, because under S6 those are session-scoped
    too and "how many times" is the whole subject."""
    _wire_prepare_recorder(node, order, monkeypatch, arm_ok=arm_ok)
    monkeypatch.setattr(node, '_publish_pretilt_hold',
                        lambda hold: order.append(('pretilt_hold', hold)))
    monkeypatch.setattr(
        node._reach_center_pub, 'publish',
        lambda msg: order.append(('centre', (msg.x, msg.y, msg.z))))


def _walk_chained_cycle(node, monkeypatch, order, pose, start, tick):
    """Drive ONE chained cycle from CHECKING to its PREPARE bundle.

    Three FSM ticks, exactly as `test_prime_hold_raised_tick_before_prepare_
    bundle` walks them: positioning (answered synchronously), verified arrival
    (the declaration tick), and the deferred bundle. Returns the sequencer."""
    seq = _fresh_seq(node, pose=pose, start=start)
    monkeypatch.setattr(
        node, '_position_platform_for_toss',
        lambda s, state=None: s.note_position_result(start, True, 0.3))
    for t in (start, start + 0.6, start + 0.7):
        tick['i'] += 1
        _stamp_fresh(node, t)
        node._step_toss_sequence(seq, t)
    return seq


def test_a_chained_session_declares_and_arms_exactly_once(monkeypatch):
    """S6, the headline: across TWO chained cycles the reach-centre declaration,
    both holds, the soft gains, the arm_catch raise and the vel_scale relay each
    happen ONCE — while catch/prime_dispatched and catch/armed happen once PER
    CYCLE (the S6 amendment: catch/armed installs no graceful stop, and the bench
    trace recorder segments every CS check off its edges).

    That count IS the invariant. A resource that is never re-raised cannot be
    re-raised at the wrong moment, which is how S6 closes the arm-mid-move seam
    by construction instead of by the DEFAULT_SESSION_MISS_CLEANUP_S fence."""
    pose = (30.0, -40.0, 170.0)
    node = _toss_ready_node(100.0, commanded_pos=pose)
    node._toss_session_live = True                 # what _execute_toss_continuous sets
    order = []
    tick = {'i': 0}
    _wire_session_recorder(node, order, monkeypatch)
    _walk_chained_cycle(node, monkeypatch, order, pose, 100.0, tick)
    _walk_chained_cycle(node, monkeypatch, order, pose, 110.0, tick)
    assert order.count(('centre', pose)) == 1
    assert order.count(('prime_hold', True)) == 1
    assert order.count(('pretilt_hold', True)) == 1
    assert order.count('gains') == 1
    assert order.count(('arm', True)) == 1
    assert len([e for e in order if e[0] == 'vel_scale']) == 1
    # …and the per-cycle remainder, twice: three topic publishes and no service.
    assert order.count(('prime_dispatched', True)) == 2
    assert order.count(('armed', True)) == 2


def test_the_session_arm_preserves_the_in_bundle_relative_order(monkeypatch):
    """Hoisting three steps to session scope must preserve their RELATIVE order,
    which is the property each of them was placed for: gains before the raise
    (the standing prime_hold suppresses the auto-prime that normally sets them),
    the raise before vel_scale, vel_scale before any armed edge, and the ONE
    prime_dispatched stamp immediately before armed.

    It also pins the two cross-transport gaps S6 must NOT collapse: the
    catch/reach_center declaration lands a full FSM tick before the arm_catch
    service call that consumes it (C-REACH-1 — and under S6 losing that race
    would mis-centre the envelope for the WHOLE RUN, not for one cycle), and
    catch/prime_hold lands a tick before the armed edge."""
    pose = (30.0, -40.0, 170.0)
    node = _toss_ready_node(100.0, commanded_pos=pose)
    node._toss_session_live = True
    order = []
    tick = {'i': 0}
    ticked = []
    _wire_session_recorder(node, order, monkeypatch)
    # Re-wrap the two ordering-critical events with their FSM tick index.
    monkeypatch.setattr(
        node, '_publish_prime_hold',
        lambda hold: (order.append(('prime_hold', hold)),
                      ticked.append((tick['i'], ('prime_hold', hold))))[0])
    monkeypatch.setattr(
        node._reach_center_pub, 'publish',
        lambda msg: (order.append(('centre', (msg.x, msg.y, msg.z))),
                     ticked.append((tick['i'], 'centre')))[0])
    monkeypatch.setattr(
        node, '_arm_catch',
        lambda a: (order.append(('arm', a)),
                   ticked.append((tick['i'], ('arm', a))))[0] or True)
    monkeypatch.setattr(
        node, '_publish_catch_armed',
        lambda a: (order.append(('armed', a)),
                   ticked.append((tick['i'], ('armed', a))))[0])
    _walk_chained_cycle(node, monkeypatch, order, pose, 100.0, tick)
    assert order == [
        # the declaration tick (verified arrival)
        ('prime_hold', True), ('centre', pose), ('pretilt_hold', True),
        # the bundle tick, one FSM tick later
        'gains', ('arm', True),
        ('vel_scale', float(hw.JB_OP_CATCH_VEL_SCALE_DEFAULT)),
        ('prime_dispatched', True), ('armed', True)]
    centre_tick = next(i for i, e in ticked if e == 'centre')
    hold_tick = next(i for i, e in ticked if e == ('prime_hold', True))
    arm_tick = next(i for i, e in ticked if e == ('arm', True))
    armed_tick = next(i for i, e in ticked if e == ('armed', True))
    assert centre_tick < arm_tick                  # C-REACH-1's transport gap
    assert hold_tick < armed_tick                  # FIX-4's wait-set gap


def test_a_chained_stay_keeps_the_latch_and_the_holds_standing(monkeypatch):
    """STAY is the CHAINING terminal and the one terminal S6 deliberately does
    NOT disarm: it issues no go_home, so there is no move for a latch to land
    inside, and lowering the latch here would force the next cycle to re-raise
    it — which is the seam. What still happens per cycle is catch/armed False.

    Releasing prime_hold here would be the live hazard: the NEXT cycle's armed
    edge would then meet a released hold and auto-prime the hand with the caught
    ball resting in the cup."""
    node = _toss_ready_node(100.0)
    node._toss_session_live = True
    node._toss_session_center_mm = (0.0, 0.0, 170.0)
    node._toss_session_armed = True
    order = []
    _wire_session_recorder(node, order, monkeypatch)
    node._toss_stay()
    assert order == [('armed', False)]
    assert node._toss_session_armed is True
    assert node._toss_session_center_mm == (0.0, 0.0, 170.0)


def test_the_session_disarm_lowers_armed_before_the_holds(monkeypatch):
    """S6's mirror, and the ORDER inside it is the part that does not change:
    catch/armed False must precede the prime_hold release, because a released
    hold meeting a still-armed catch_coordinator re-opens the auto-prime exactly
    while a ball rests in the cup — the ascent would launch it. pretilt_hold
    goes last of all (a stale one only DEGRADES a later reload's pre-tilt).

    Session scope moved WHEN the teardown runs, never the order inside it."""
    node = _toss_ready_node(100.0)
    node._toss_session_live = True
    node._toss_session_center_mm = (0.0, 0.0, 170.0)
    node._toss_session_armed = True
    order = []
    _wire_session_recorder(node, order, monkeypatch)
    node._disarm_session()
    assert order == [('armed', False), ('arm', False),
                     ('prime_hold', False), ('pretilt_hold', False)]
    assert node._toss_session_armed is False
    assert node._toss_session_center_mm is None
    # Idempotent: a second call (the session terminal after a cycle ladder
    # already drained) publishes nothing at all.
    order.clear()
    node._disarm_session()
    assert order == []


def test_a_chained_safe_abort_drains_before_the_go_home(monkeypatch):
    """S7: every path that dispatches go_home drains and disarms FIRST, so no
    go_home is ever installed under a catch the machine still thinks is live —
    the arm-mid-move seam, closed from the other side.

    Also pins that the drain's first act is catch/armed False, which is exactly
    what _safe_abort's own ladder needs before the retract (catch_coordinator's
    prime-retry tick must stand down before a kind-3 descent is dispatched)."""
    node = _toss_ready_node(100.0)
    node._toss_session_live = True
    node._toss_session_center_mm = (0.0, 0.0, 170.0)
    node._toss_session_armed = True
    order = []
    _wire_session_recorder(node, order, monkeypatch)
    monkeypatch.setattr(node, '_retract_hand_with_retries',
                        lambda: order.append('retract') or True)
    monkeypatch.setattr(node, '_go_home',
                        lambda: order.append('go_home') or True)
    node._toss_safe_abort()
    # The whole drain — armed False FIRST (so catch_coordinator's prime-retry
    # tick stands down), then the latch, then the holds — precedes the safing
    # ladder, and the go_home is last of everything.
    assert order == [('armed', False), ('arm', False), ('prime_hold', False),
                     ('pretilt_hold', False),
                     # _safe_abort's own ladder, unchanged and shared verbatim
                     # with the RELOAD path: its armed/latch publishes are now
                     # idempotent repeats of the drain's.
                     ('armed', False), 'retract', ('arm', False), 'go_home']
    # The holds are released EXACTLY ONCE: the per-cycle tail no-ops for every
    # cycle of a session, drain included, so nothing re-publishes them.
    assert order.count(('prime_hold', False)) == 1
    assert order.count(('pretilt_hold', False)) == 1


def test_the_single_toss_keeps_its_per_cycle_arming(monkeypatch):
    """The single Toss action is NOT session-scoped and its arming is unchanged:
    every cycle runs the full bundle, and the session flags stay clear so the
    STAY/RECENTER/SAFE_ABORT ladders keep lowering the latch per goal.

    This is the control for the two counts above — the same walk, the same
    recorder, `_toss_session_live` False."""
    pose = (30.0, -40.0, 170.0)
    node = _toss_ready_node(100.0, commanded_pos=pose)
    assert node._toss_session_live is False        # the shipped default
    order = []
    tick = {'i': 0}
    _wire_session_recorder(node, order, monkeypatch)
    _walk_chained_cycle(node, monkeypatch, order, pose, 100.0, tick)
    _walk_chained_cycle(node, monkeypatch, order, pose, 110.0, tick)
    assert order.count(('centre', pose)) == 2
    assert order.count(('prime_hold', True)) == 2
    assert order.count(('pretilt_hold', True)) == 2
    assert order.count('gains') == 2
    assert order.count(('arm', True)) == 2
    assert order.count(('armed', True)) == 2
    assert node._toss_session_armed is False
    assert node._toss_session_center_mm is None


# ── S6's reach-centre DRIFT GUARD ─────────────────────────────────────────────

def test_the_drift_bound_is_the_envelope_minus_the_worst_case_swing(monkeypatch):
    """The bound is DERIVED, not chosen, and this pins the derivation rather
    than the number: the C-REACH-1 envelope radius minus the systematic swing
    shift the reach itself carries (hand_catch_offset · sin(MAX_TILT_DEG), which
    SATURATES on every real arrival — the same subtraction
    _RELOAD_CENTERED_TOL_MM makes one path over, for the same reason: a gate
    that says yes to a band the envelope will then say no to mid-flight)."""
    expected = (float(hw.JB_TRAJ_CATCH_REACH_ENVELOPE_MM)
                - float(hw.HAND_CATCH_OFFSET_MM)
                * math.sin(math.radians(rcn.MAX_TILT_DEG)))
    assert _TOSS_SESSION_REACH_DRIFT_TOL_MM == pytest.approx(expected)
    assert _TOSS_SESSION_REACH_DRIFT_TOL_MM < float(
        hw.JB_TRAJ_CATCH_REACH_ENVELOPE_MM)


def test_the_drift_guard_refuses_a_cycle_that_left_the_session_envelope(
        monkeypatch):
    """The foreclosed case fails LOUDLY and EARLY. Under S6 trajectory_node's
    envelope centre is frozen at the session's ONE declaration (`_svc_arm_catch`
    read-and-clears the pending centre BEFORE its idempotent early return), so a
    cycle nominating a different B would be judged against an envelope centred
    somewhere else — and the refusal would arrive MID-FLIGHT as a WORKSPACE
    reject of the A→B reach, with the ball already airborne.

    So the cycle is refused here instead, with NOTHING published, and it is
    refused with a REJECTED code rather than a generic PREPARE abort."""
    node = _toss_ready_node(100.0)
    node._toss_session_live = True
    node._toss_session_center_mm = (0.0, 0.0, 170.0)
    node._toss_session_armed = True
    order = []
    _wire_session_recorder(node, order, monkeypatch)
    far = (0.0, _TOSS_SESSION_REACH_DRIFT_TOL_MM + 1.0, 170.0)
    seq = TossSequencer(catch_pose_stow_mm=far)
    assert node._prepare_toss_catch(seq) is False
    assert node._toss_prepare_reject == 'REACH_CENTER_DRIFT'
    assert order == []                              # nothing armed, nothing stamped


def test_the_drift_guard_admits_a_cycle_inside_the_margin(monkeypatch):
    """The complement, so the guard is a BOUND and not a blanket refusal: a B
    just inside the tolerance still lands its own requested reach inside the
    80 mm envelope, so it flies. (Every session in scope sits at drift 0 —
    catch_position is one goal field, constant for the whole session — so this
    is the guard's inactive side, which is the side it must stay on.)"""
    node = _toss_ready_node(100.0)
    node._toss_session_live = True
    node._toss_session_center_mm = (0.0, 0.0, 170.0)
    node._toss_session_armed = True
    order = []
    _wire_session_recorder(node, order, monkeypatch)
    near = (0.0, _TOSS_SESSION_REACH_DRIFT_TOL_MM - 1.0, 170.0)
    seq = TossSequencer(catch_pose_stow_mm=near)
    assert node._prepare_toss_catch(seq) is True
    assert node._toss_prepare_reject == ''
    assert order == [('prime_dispatched', True), ('armed', True)]


def test_a_drifted_cycle_terminalises_rejected_reach_center_drift(monkeypatch):
    """End to end at the FSM seam: the refusal is carried to the terminal as
    REJECTED_REACH_CENTER_DRIFT (an operator-visible refusal that names the
    forward path) rather than laundered into ABORTED_PREPARE_FAILED (which reads
    as a plant fault). The cleanup is still SAFE_ABORT — the declaration and the
    holds are out, so the machine is not pristine."""
    pose = (0.0, 0.0, 170.0)
    node = _toss_ready_node(100.0, commanded_pos=pose)
    node._toss_session_live = True
    node._toss_session_center_mm = pose
    node._toss_session_armed = True
    order = []
    tick = {'i': 0}
    _wire_session_recorder(node, order, monkeypatch)
    safed = []
    monkeypatch.setattr(node, '_toss_safe_abort',
                        lambda state=None: safed.append(1))
    far = (0.0, _TOSS_SESSION_REACH_DRIFT_TOL_MM + 20.0, 170.0)
    seq = _fresh_seq(node, pose=far, start=100.0)
    monkeypatch.setattr(
        node, '_position_platform_for_toss',
        lambda s, state=None: s.note_position_result(100.0, True, 0.3))
    d = None
    for t in (100.0, 100.6, 100.7, 100.8):
        tick['i'] += 1
        _stamp_fresh(node, t)
        d = node._step_toss_sequence(seq, t)
        if d.done:
            break
    assert d.done and base_outcome(
        d.result.outcome) == 'REJECTED_REACH_CENTER_DRIFT'
    # The three numbers that make this refusal actionable — the B this cycle
    # nominated, the centre the SESSION captured cycles ago, and the distance
    # against the tolerance. The operator chose none of them but B.
    msg = d.result.outcome
    assert 'B (0.0, 86.5, 170.0) mm is 86.5 mm' in msg, msg
    assert 'tolerance 66.5 mm' in msg, msg
    assert 'reach centre (0.0, 0.0, 170.0)' in msg, msg
    assert 're-arm at the new B' in msg, msg
    assert safed == [1]


def test_announcement_content_and_frames():
    """The self-announcement: target_id AND thrower_name are this robot
    (target_id is the load-bearing one — the tracker tags destination from it),
    all physics fields from the single tested release-state module (landing =
    the nominated catch state in GLOBAL mm — the one conversion point), and
    landing_time − throw_time = the flight time (the one ROS↔perf crossing)."""
    node = _toss_ready_node(100.0)
    _install_toss_goal(node, pose=(30.0, -40.0, 170.0), flight=0.8)
    seq = TossSequencer(catch_pose_stow_mm=(30.0, -40.0, 170.0),
                        flight_time_s=0.8, throw_delay_s=5.0)
    seq.start(time.perf_counter())
    # note_announcement is gated on the PREPARE dispatch (pinned in
    # test_toss_sequencer); satisfy the gate — this test's subject is the
    # announcement packing, not the FSM walk.
    seq._prepare_dispatched = True
    node._announce_toss(seq)
    ann = node._publishers['throw_announcements'].published[-1]
    assert ann.thrower_name == 'jugglebot'
    assert ann.target_id == 'jugglebot'
    assert (ann.landing_position.x, ann.landing_position.y) == (30.0, -40.0)
    assert ann.landing_position.z == pytest.approx(809.08)
    assert ann.initial_position.z == pytest.approx(802.344)
    assert ann.initial_velocity.x == pytest.approx(0.0)
    assert ann.initial_velocity.z == pytest.approx(3930.82, abs=0.01)
    assert ann.landing_velocity.z == pytest.approx(-3913.98, abs=0.01)
    assert ann.predicted_tof_sec == pytest.approx(0.8)
    # Mock clock: ros now = 0 → the stamps are the perf deltas in ns; the
    # landing stamp is release + ToF on the same clock.
    assert (ann.landing_time - ann.throw_time) * 1e-9 == pytest.approx(
        0.8, abs=1e-6)
    assert ann.throw_time * 1e-9 == pytest.approx(5.0, abs=0.2)
    assert seq._announced is True                        # note_announcement fed


def test_own_announcement_echo_not_double_fed():
    """The node subscribes to throw_announcements AND publishes on it: the echo
    of our own toss announcement must be dropped for a TossSequencer (its FSM
    was already notified synchronously; ReloadSequencer's note_announcement
    signature would not even fit)."""
    node = ReloadCoordinatorNode()
    seq = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0))
    seq.start(0.0)
    with node._lock:
        node._active_seq = seq
    ann = types.SimpleNamespace(target_id='jugglebot', landing_time=None,
                                predicted_tof_sec=0.8)
    node._on_announcement(ann)                           # must not raise
    assert seq._announced is False                       # and must not feed


# ── Full choreography walk (position → prepare → gap → announce → dispatch) ────

def test_toss_choreography_full_walk(monkeypatch):
    """The whole goal in order, at the node level: positioning dispatched once →
    timed arrival (cross-check disabled — the shipped default, so no mocap body
    is fed) → prime_hold ALONE on the arrival tick → the PREPARE bundle one
    tick later → ≥1-tick gap → announcement (target_id ours) → the single throw
    dispatch → telemetry release evidence → flight → correlated plausible
    CAUGHT → RECENTER. Pins that the hold precedes the bundle tick, the
    announce happens on a LATER tick than the armed publish, and the dispatch
    after the announce confirm."""
    t0 = 100.0
    node = _toss_ready_node(t0)
    pose = (30.0, -40.0, 170.0)
    seq = _fresh_seq(node, pose=pose, delay=5.0, start=t0)   # t_release 105.0
    order = []
    monkeypatch.setattr(
        node, '_position_platform_for_toss',
        lambda s, state=None: order.append('position') or s.note_position_result(
            t0, True, 0.3))                              # arrival t0 + 0.5
    _wire_prepare_recorder(node, order, monkeypatch)
    ann_pub = node._publishers['throw_announcements']
    orig_ann_publish = ann_pub.publish

    def _rec_announce(msg):
        order.append('announce')
        orig_ann_publish(msg)
    monkeypatch.setattr(ann_pub, 'publish', _rec_announce)

    def _fake_dispatch(s, state=None):
        order.append('dispatch')
        with node._lock:
            node._toss_committed.throw_dispatched = True
        return THROW_DISPATCH_OK, ''
    monkeypatch.setattr(node, '_dispatch_toss_throw', _fake_dispatch)
    # The CAUGHT terminal is _toss_stay since 2026-07-29 (Phase E). Let the REAL
    # ladder run through the already-wired _arm_catch / _publish_catch_armed /
    # _publish_prime_hold recorders so the ORDER is pinned, and record go_home
    # separately so "no go_home on CAUGHT" is asserted rather than assumed —
    # _toss_recenter is monkeypatched to fail loudly if it is ever reached.
    monkeypatch.setattr(node, '_go_home',
                        lambda: order.append('go_home') or True)
    monkeypatch.setattr(node, '_toss_recenter',
                        lambda state=None: pytest.fail(
                            'CAUGHT must STAY, not RECENTER, at the shipped '
                            'toss_stay_at_pose_on_caught default'))

    gh = _TossGoalHandle()
    d = node._step_toss_sequence(seq, t0, gh)
    assert d.action == ACTION_POSITION_PLATFORM
    _stamp_fresh(node, t0 + 0.5)
    d = node._step_toss_sequence(seq, t0 + 0.5, gh)      # verified arrival: hold
    assert d.action == ACTION_PREPARE_CATCH
    _stamp_fresh(node, t0 + 0.55)
    d = node._step_toss_sequence(seq, t0 + 0.55, gh)     # the deferred bundle
    assert d.action == ACTION_NONE and d.phase == PHASE_PREPARING
    _stamp_fresh(node, t0 + 0.6)
    d = node._step_toss_sequence(seq, t0 + 0.6, gh)      # the ≥1-tick gap
    assert d.action == ACTION_ANNOUNCE
    _stamp_fresh(node, t0 + 0.65)
    d = node._step_toss_sequence(seq, t0 + 0.65, gh)
    assert d.action == ACTION_DISPATCH_THROW
    # Release evidence: the ascending-stroke telemetry signature.
    with node._lock:
        node._hand_vel_meas = 50.0
        node._hand_pos_meas = 3.0
    _stamp_fresh(node, t0 + 0.7)
    d = node._step_toss_sequence(seq, t0 + 0.7, gh)
    assert d.phase == PHASE_BALL_IN_FLIGHT
    # Our ball appears in flight (latch), then a plausible CAUGHT near the
    # NOMINATED point confirms.
    with node._lock:
        node._balls = [_Ball(status=1, id=5)]
    _stamp_fresh(node, t0 + 5.0)
    node._step_toss_sequence(seq, t0 + 5.0, gh)
    with node._lock:
        node._balls = [_Ball(status=2, id=5, x=30.0, y=-40.0, z=805.0)]
    # THE CATCH, as the CUP sees it (D1, 2026-08-26): the verdict is the cup's
    # alone, so this walk has to put a ball back in the cup — a tracker CAUGHT no
    # longer mints one. Scheduled landing is t_release + flight = t0 + 5.8, and
    # the empty→held edge lands +0.11 s past it, inside the measured arrival band.
    _stamp_fresh(node, t0 + 5.9, held=False)
    node._ball_sensor.note_sample(t0 + 5.91, held=True, valid=True, raw=True)
    d = node._step_toss_sequence(seq, t0 + 5.92, gh)
    assert d.done and d.result.outcome == 'CAUGHT'
    assert d.result.catch_error_mm == pytest.approx(0.0)
    assert d.result.catch_event_dt_s == pytest.approx(0.11, abs=1e-6)
    # vel_scale is the CONFIG default relayed verbatim, not a number this test
    # owns: the goal leaves catch_vel_scale at 0, so `_install_toss_goal`
    # resolves JB_OP_CATCH_VEL_SCALE_DEFAULT exactly as `_execute_toss` does.
    # Reading it from `hw` keeps this an ORDER test — which is its whole
    # subject — instead of a second, silent pin on the operator's catch-speed
    # knob (which moved 0.8 → 0.9 on 2026-08-10 and broke the literal).
    assert order == ['position', ('prime_hold', True), 'gains', ('arm', True),
                     ('vel_scale', float(hw.JB_OP_CATCH_VEL_SCALE_DEFAULT)),
                     ('prime_dispatched', True),
                     ('armed', True), 'announce', 'dispatch',
                     # the STAY ladder: latch down, armed False, THEN the hold
                     # released last (a released hold meeting a still-armed
                     # catch_coordinator re-opens the auto-prime with the caught
                     # ball in the cup) — and NO go_home, which is what leaves
                     # the platform at B for the next toss to throw from.
                     ('arm', False), ('armed', False), ('prime_hold', False)]
    assert 'go_home' not in order
    ann = ann_pub.published[-1]
    assert ann.target_id == 'jugglebot'
    assert gh.feedbacks[0] == PHASE_POSITIONING
    assert PHASE_PREPARING in gh.feedbacks and PHASE_THROWING in gh.feedbacks


def test_position_unverified_arrival_aborts(monkeypatch):
    """Mocap-verify FAIL branch (cross-check ENABLED via the body parameter):
    an accepted move whose arrival mocap never corroborates (the silent no-op
    class — disarmed wire, guard latch) aborts ABORTED_POSITION_FAILED at the
    positioning deadline instead of arming the catch envelope at the wrong
    pose. The accepted move means SAFE_ABORT (the go_home leg)."""
    t0 = 100.0
    node = _toss_ready_node(t0)
    node._toss_mocap_body = 'Platform'
    seq = _fresh_seq(node, pose=(60.0, -60.0, 170.0), start=t0)
    with node._lock:
        node._platform_pos_mm = (0.0, 0.0, 170.0)        # never moved (84.9 mm off)
    monkeypatch.setattr(
        node, '_position_platform_for_toss',
        lambda s, state=None: s.note_position_result(t0, True, 0.3))
    safed = []
    monkeypatch.setattr(node, '_toss_safe_abort',
                        lambda state=None: safed.append(1))
    node._step_toss_sequence(seq, t0)
    _stamp_fresh(node, t0 + 0.6)
    d = node._step_toss_sequence(seq, t0 + 0.6)
    assert not d.done                                    # verify window open
    _stamp_fresh(node, t0 + 6.0)
    d = node._step_toss_sequence(seq, t0 + 6.0)
    assert d.done and d.result.outcome == 'ABORTED_POSITION_FAILED'
    assert safed == [1]


def test_go_to_pose_wire_disarmed_marker_maps_to_reject(monkeypatch):
    """The '[wire DISARMED' marker on a go_to_pose response maps to
    REJECTED_POSITION(WIRE_DISARMED) — the plan-installed-but-not-actuating case
    (string-fragile by design; this test is the drift alarm)."""
    node = _toss_ready_node(100.0)
    seq = _fresh_seq(node, start=100.0)
    resp = types.SimpleNamespace(
        accepted=True, code='OK', planned_duration_s=1.0,
        message='planned [wire DISARMED — setpoints not reaching the legs]')
    monkeypatch.setattr(node, '_wait_future',
                        lambda fut, timeout_s=2.0: resp)
    node._step_toss_sequence(seq, 100.0)                 # dispatches the move
    _stamp_fresh(node, 100.1)
    d = node._step_toss_sequence(seq, 100.1)
    assert d.done
    # The subcode still LEADS the parenthetical (the FSM's BUSY re-poll and the
    # zombie-superseder guard both key on it), and trajectory_node's own
    # message now follows it instead of being discarded.
    assert d.result.outcome.startswith(
        'REJECTED_POSITION(WIRE_DISARMED'), d.result.outcome
    assert 'wire DISARMED' in d.result.outcome, d.result.outcome


def test_position_no_response_dispatches_best_effort_go_home(monkeypatch):
    """FIX-5c: NO_RESPONSE leaves the platform's motion state UNKNOWN — the
    go_to_pose service returns at plan-INSTALL, so an ack that timed out may
    still be executing the move. The FSM's terminal action is ACTION_NONE
    (nothing verifiably accepted), so the node dispatches a best-effort
    go_home to supersede any zombie plan (go_home replaces the installed
    trajectory by design)."""
    node = _toss_ready_node(100.0)
    seq = _fresh_seq(node, start=100.0)
    monkeypatch.setattr(
        node, '_position_platform_for_toss',
        lambda s, state=None: s.note_position_result(
            100.0, False, 0.0, 'NO_RESPONSE'))
    homed = []
    monkeypatch.setattr(node, '_go_home', lambda: homed.append(1) or True)
    node._step_toss_sequence(seq, 100.0)
    _stamp_fresh(node, 100.1)
    d = node._step_toss_sequence(seq, 100.1)
    assert d.done and d.result.outcome == 'REJECTED_POSITION(NO_RESPONSE)'
    assert homed == [1]


def test_the_zombie_superseder_survives_an_enriched_position_refusal(monkeypatch):
    """THE guard the 2026-08-29 enrichment could most easily have disarmed, and
    it would have disarmed it SILENTLY.

    ``_TOSS_POSITION_UNKNOWN_TERMINALS`` used to hold the literal string
    ``'REJECTED_POSITION(NO_RESPONSE)'``. Now that ``go_to_pose``'s message
    rides the outcome, that literal stops matching the moment trajectory_node
    has anything to say about a timed-out ack — and a membership test that
    stops matching does nothing at all: no go_home, no superseder, and a plan
    that may still be executing left to run under a machine that believes
    nothing was accepted. The guard therefore matches (CODE, SUBCODE), which
    both forms share, and this row drives the enriched form specifically."""
    node = _toss_ready_node(100.0)
    seq = _fresh_seq(node, start=100.0)
    monkeypatch.setattr(
        node, '_position_platform_for_toss',
        lambda s, state=None: s.note_position_result(
            100.0, False, 0.0, 'NO_RESPONSE',
            'service unavailable after 2.0 s'))
    homed = []
    monkeypatch.setattr(node, '_go_home', lambda: homed.append(1) or True)
    node._step_toss_sequence(seq, 100.0)
    _stamp_fresh(node, 100.1)
    d = node._step_toss_sequence(seq, 100.1)
    assert d.done
    assert d.result.outcome.startswith('REJECTED_POSITION(NO_RESPONSE: ')
    assert homed == [1], 'the zombie-move superseder did not run'


def test_position_timeout_dispatches_best_effort_go_home(monkeypatch):
    """FIX-5c, the never-noted sibling: a positioning that times out without
    ANY response is the same unknown-motion state — best-effort go_home on
    ABORTED_POSITION_TIMEOUT too."""
    node = _toss_ready_node(100.0)
    seq = _fresh_seq(node, start=100.0)
    monkeypatch.setattr(node, '_position_platform_for_toss',
                        lambda s, state=None: None)
    homed = []
    monkeypatch.setattr(node, '_go_home', lambda: homed.append(1) or True)
    node._step_toss_sequence(seq, 100.0)
    _stamp_fresh(node, 106.1)
    d = node._step_toss_sequence(seq, 106.1)
    assert d.done and d.result.outcome == 'ABORTED_POSITION_TIMEOUT'
    assert homed == [1]


# ── Throw dispatch: single-shot tri-state (D8) ─────────────────────────────────

def _real_bridge_reject_message(event_delay=2.0, event_vel=9.0, traj_type=0):
    """Drive the REAL teensy_bridge validation path (no copied strings): the
    handler raises its ValueError before touching self for invalid inputs, so an
    attribute-less stub self suffices."""
    from jugglebot.teensy_bridge_node import TeensyBridgeNode
    from jugglebot_interfaces.srv import SetHandTrajCmd
    req = SetHandTrajCmd.Request()
    req.event_delay = event_delay
    req.event_vel = event_vel
    req.traj_type = traj_type
    res = TeensyBridgeNode._svc_set_hand_traj(
        types.SimpleNamespace(), req, SetHandTrajCmd.Response())
    assert res.success is False
    return res.message


@pytest.mark.parametrize('kwargs', [
    dict(event_vel=9.0),                  # above the 7.0 ceiling
    dict(event_vel=0.1),                  # below the 0.3 floor
    dict(event_delay=-1.0, event_vel=3.0),
    dict(event_vel=3.0, traj_type=5),
])
def test_definitive_reject_classification_vs_real_bridge_validation(kwargs):
    """D8: the definitive-reject classifier is pinned against the messages the
    bridge's REAL validation path emits (raised BEFORE any CAN frame — a
    guaranteed no-arm). If the bridge's literals ever drift, this test breaks
    loudly instead of the classifier silently downgrading rejects to ambiguous
    (which would stall every invalid dispatch to ABORTED_NO_RELEASE)."""
    message = _real_bridge_reject_message(**kwargs)
    assert (ReloadCoordinatorNode._classify_hand_traj_reject(message)
            == THROW_DISPATCH_REJECTED)


def test_transport_failure_classifies_ambiguous_via_real_handler():
    """An ERR_TIMEOUT-class failure REACHES the CAN path (the frame may have
    transmitted) — through the real handler with a stubbed transport it must
    classify ambiguous, never rejected: an ambiguous ack forbids both retry and
    retract-free abort."""
    from jugglebot.teensy_bridge_node import TeensyBridgeNode
    from jugglebot_interfaces.srv import SetHandTrajCmd
    req = SetHandTrajCmd.Request()
    req.event_delay = 2.0
    req.event_vel = 3.0
    req.traj_type = 0
    stub = types.SimpleNamespace(
        teensy_hand_traj_cmd=lambda args: (False, 'ERR_TIMEOUT: no reply', None))
    res = TeensyBridgeNode._svc_set_hand_traj(
        stub, req, SetHandTrajCmd.Response())
    assert res.success is False
    assert (ReloadCoordinatorNode._classify_hand_traj_reject(res.message)
            == THROW_DISPATCH_AMBIGUOUS)


def test_dispatch_request_fields_and_ok(monkeypatch):
    """The dispatch: traj_type=0, event_vel from the release state, event_delay
    recomputed from the ABSOLUTE scheduled release at dispatch time (minus the
    reserved release-latency slot, which ships 0.0) — and the telemetry watch is
    armed BEFORE the ack wait (the frame may fire while we wait)."""
    node = _toss_ready_node(100.0)
    t0 = time.perf_counter()
    seq = _fresh_seq(node, delay=5.0, start=t0)
    captured = []
    monkeypatch.setattr(node._hand_traj_cli, 'call_async',
                        lambda req: captured.append(req) or object())

    def _resp(fut, timeout_s=2.0):
        assert node._toss_committed.throw_dispatched is True       # armed pre-ack
        return types.SimpleNamespace(success=True, message='Hand trajectory set.')
    monkeypatch.setattr(node, '_wait_future', _resp)
    outcome, _ = node._dispatch_toss_throw(seq)
    assert outcome == THROW_DISPATCH_OK
    req = captured[0]
    assert req.traj_type == 0
    assert req.event_vel == pytest.approx(seq.event_vel_mps)
    assert req.event_delay == pytest.approx(seq.t_release - t0, abs=0.2)
    # The latency slot is RESERVED at 0.0 until Phase-5 T0 measures it — a
    # non-zero value landing without a T0 session should fail here first.
    assert hw.JB_OP_TOSS_RELEASE_LATENCY_MS == 0.0


def test_dispatch_service_unavailable_is_rejected(monkeypatch):
    """No service ⇒ no CAN frame can exist ⇒ definitive reject (safe to abort
    with the retract clearing any half-state)."""
    node = _toss_ready_node(100.0)
    seq = _fresh_seq(node, start=time.perf_counter())
    node._hand_traj_cli._ready = False
    outcome, message = node._dispatch_toss_throw(seq)
    assert outcome == THROW_DISPATCH_REJECTED
    assert 'unavailable' in message


def test_dispatch_ack_timeout_is_ambiguous(monkeypatch):
    node = _toss_ready_node(100.0)
    seq = _fresh_seq(node, start=time.perf_counter())
    monkeypatch.setattr(node, '_wait_future', lambda fut, timeout_s=2.0: None)
    outcome, _ = node._dispatch_toss_throw(seq)
    assert outcome == THROW_DISPATCH_AMBIGUOUS


# ── Terminal orderings (prime_hold released LAST) ──────────────────────────────

def test_toss_recenter_releases_prime_hold_last(monkeypatch):
    """RECENTER: the reload teardown (latch lower, armed False, go_home; hand
    keeps the ball) with prime_hold False LAST — the hold and catch/armed travel
    on different topics with no ordering guarantee, and a released hold meeting
    a still-armed catch_coordinator would auto-prime with the caught ball in the
    cup (the ascent launches it). A stale hold costs nothing."""
    node = ReloadCoordinatorNode()
    order = []
    monkeypatch.setattr(node, '_arm_catch',
                        lambda a: order.append(('arm', a)) or True)
    monkeypatch.setattr(node, '_publish_catch_armed',
                        lambda a: order.append(('armed', a)))
    monkeypatch.setattr(node, '_go_home', lambda: order.append('home') or True)
    monkeypatch.setattr(node, '_publish_prime_hold',
                        lambda h: order.append(('prime_hold', h)))
    node._toss_recenter()
    assert order == [('arm', False), ('armed', False), 'home',
                     ('prime_hold', False)]


def test_toss_stay_is_the_recenter_ladder_minus_go_home(monkeypatch):
    """STAY (the CAUGHT terminal since 2026-07-29): the RECENTER ordering
    verbatim — latch lower, catch/armed False, prime_hold False LAST — with
    **no go_home**, and no retract, and no other command of any kind.

    "Not calling go_home" is the entire mechanism: the emitter's terminal hold
    already keeps the platform where the catch left it, which is what makes the
    NEXT toss's throw site A be B. The ordering is load-bearing for the
    unchanged reason: a released prime_hold meeting a still-armed
    catch_coordinator re-opens the auto-prime with the caught ball in the cup.
    """
    node = ReloadCoordinatorNode()
    order = []
    monkeypatch.setattr(node, '_arm_catch',
                        lambda a: order.append(('arm', a)) or True)
    monkeypatch.setattr(node, '_publish_catch_armed',
                        lambda a: order.append(('armed', a)))
    monkeypatch.setattr(node, '_go_home', lambda: order.append('home') or True)
    monkeypatch.setattr(node, '_smooth_move_hand',
                        lambda p: order.append(('hand', p)) or True)
    monkeypatch.setattr(node, '_publish_prime_hold',
                        lambda h: order.append(('prime_hold', h)))
    node._toss_stay()
    assert order == [('arm', False), ('armed', False), ('prime_hold', False)]


def test_toss_stay_releases_pretilt_hold_last_when_raised(monkeypatch):
    """STAY releases catch/pretilt_hold LAST of all, iff this goal raised it —
    identical to RECENTER/SAFE_ABORT. A stale pretilt_hold only DEGRADES (a
    later reload announcement loses its platform pre-tilt), never a hazard."""
    node = ReloadCoordinatorNode()
    order = []
    monkeypatch.setattr(node, '_arm_catch', lambda a: True)
    monkeypatch.setattr(node, '_publish_catch_armed', lambda a: None)
    monkeypatch.setattr(node, '_publish_prime_hold',
                        lambda h: order.append(('prime_hold', h)))
    monkeypatch.setattr(node, '_publish_pretilt_hold',
                        lambda h: order.append(('pretilt_hold', h)))
    node._toss_committed.pretilt_hold_raised = False
    node._toss_stay()
    assert order == [('prime_hold', False)]              # 8a: never touched
    order.clear()
    node._toss_committed.pretilt_hold_raised = True
    node._toss_stay()
    assert order == [('prime_hold', False), ('pretilt_hold', False)]


def test_toss_safe_abort_ordering_with_prime_hold_last(monkeypatch):
    """SAFE_ABORT: the reload safing ladder verbatim — catch/armed False FIRST
    (retry-tick stand-down), telemetry-verified retract (which deliberately also
    clears an armed throw stroke on the last-writer-wins queue), latch lower,
    go_home — and prime_hold False LAST, outliving the whole teardown for the
    same cross-topic-ordering reason as RECENTER (the ball is still seated on a
    pre-release abort)."""
    node = ReloadCoordinatorNode()
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    order = []
    monkeypatch.setattr(node, '_publish_catch_armed',
                        lambda a: order.append(('armed', a)))
    monkeypatch.setattr(node, '_smooth_move_hand',
                        lambda p: order.append(('retract', p)) or True)
    monkeypatch.setattr(node, '_arm_catch',
                        lambda a: order.append(('arm', a)) or True)
    monkeypatch.setattr(node, '_go_home', lambda: order.append('home') or True)
    monkeypatch.setattr(node, '_publish_prime_hold',
                        lambda h: order.append(('prime_hold', h)))
    node._toss_safe_abort()
    assert order == [('armed', False), ('retract', hw.JB_OP_HAND_RETRACT_REV),
                     ('arm', False), 'home', ('prime_hold', False)]


# ── Cancellation (per-phase deferral — plan § Choreography 7) ──────────────────

def test_cancel_deferral_phase_table():
    """The per-phase table: honoured through PREPARING and early THROWING (the
    retract clears the armed stroke; nothing airborne), DEFERRED from the
    cutoff before release onward — aborting a catch mid-flight drops a ball on
    the robot (the reload hazard this deliberately deviates from)."""
    node = ReloadCoordinatorNode()
    mk = lambda phase: types.SimpleNamespace(phase=phase, t_release=105.0)
    for phase in (PHASE_CHECKING, PHASE_POSITIONING, PHASE_PREPARING):
        assert node._toss_cancel_deferred(mk(phase), 104.9) is False
    assert node._toss_cancel_deferred(mk(PHASE_THROWING), 104.7) is False
    assert node._toss_cancel_deferred(mk(PHASE_THROWING), 104.8) is True
    for phase in (PHASE_BALL_IN_FLIGHT, PHASE_CATCHING, PHASE_SETTLING):
        assert node._toss_cancel_deferred(mk(phase), 104.0) is True


def test_cancel_honoured_early_returns_cancelled():
    """A cancel in CHECKING is honoured immediately: ABORTED_CANCELLED,
    canceled() (not abort), and no safing (nothing armed yet)."""
    node = _toss_ready_node(time.perf_counter())
    gh = _TossGoalHandle()
    gh.is_cancel_requested = True
    result = node._execute_toss(gh)
    assert result.outcome == 'ABORTED_CANCELLED'
    assert gh.terminal == 'canceled'


def test_cancel_deferred_resolves_with_fsm_outcome(monkeypatch):
    """A DEFERRED cancel keeps ticking the FSM to its own terminal, then
    resolves canceled() with the FSM's REAL outcome — the catch attempt ran."""
    node = _toss_ready_node(time.perf_counter())
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    monkeypatch.setattr(node, '_toss_cancel_deferred',
                        lambda seq, now: True)
    decisions = iter([
        TossDecision(PHASE_BALL_IN_FLIGHT, 'none', False, None),
        TossDecision(PHASE_SETTLING, 'none', True,
                     TossResult(False, 'MISSED')),
    ])
    monkeypatch.setattr(node, '_step_toss_sequence',
                        lambda seq, now, gh=None, state=None: next(decisions))
    gh = _TossGoalHandle()
    gh.is_cancel_requested = True
    result = node._execute_toss(gh)
    assert result.outcome == 'MISSED'                    # the FSM's real verdict
    assert gh.terminal == 'canceled'


# ── Node-level early exits safe the robot first (FIX-6) ────────────────────────

def _script_position_accept(node, monkeypatch, side_effect=None):
    """Script positioning to ACCEPT (seq becomes prepared — the go_home leg of
    SAFE_ABORT is owed from that point), optionally firing a side effect."""
    def _accept(seq, state=None):
        seq.note_position_result(time.perf_counter(), True, 0.3)
        if side_effect is not None:
            side_effect()
    monkeypatch.setattr(node, '_position_platform_for_toss', _accept)


def test_cancel_prepared_safes_before_canceled(monkeypatch):
    """A cancel honoured pre-cutoff with prepared=True (the positioning move
    was ACCEPTED — the platform may have moved) runs the full toss safing
    STRICTLY BEFORE canceled(): terminalising the goal first would let a new
    goal race the teardown for the hand/latch."""
    node = _toss_ready_node(time.perf_counter())
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    gh = _TossGoalHandle()
    order = []
    _script_position_accept(
        node, monkeypatch,
        side_effect=lambda: setattr(gh, 'is_cancel_requested', True))
    monkeypatch.setattr(node, '_toss_safe_abort',
                        lambda state=None: order.append('safed'))
    orig_canceled = gh.canceled
    gh.canceled = lambda: (order.append('canceled'), orig_canceled())
    result = node._execute_toss(gh)
    assert result.outcome == 'ABORTED_CANCELLED'
    assert gh.terminal == 'canceled'
    assert order == ['safed', 'canceled']        # safing strictly first


def test_sequence_ceiling_timeout_aborts_and_safes(monkeypatch):
    """The per-goal ceiling: a wedged sequence terminates ABORTED_TIMEOUT and
    the early-exit safing runs (prepared — the move was accepted)."""
    node = _toss_ready_node(time.perf_counter())
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    monkeypatch.setattr(
        'jugglebot.reload_coordinator_node._toss_deadline_s', lambda seq: 0.0)
    _script_position_accept(node, monkeypatch)
    safed = []
    monkeypatch.setattr(node, '_toss_safe_abort',
                        lambda state=None: safed.append(1))
    gh = _TossGoalHandle()
    result = node._execute_toss(gh)
    assert result.outcome == 'ABORTED_TIMEOUT'
    assert gh.terminal == 'abort'
    assert safed == [1]


def test_rclpy_shutdown_aborts_and_safes(monkeypatch):
    """rclpy going down mid-sequence exits ABORTED_SHUTDOWN with the same
    early-exit safing — a shutdown must not strand the latch raised or the
    hand parted from its park band.

    It must ALSO leave the goal handle untouched: a status transition on a
    dying executor can itself raise, and the execute callback's except would
    then overwrite the ABORTED_SHUTDOWN line already logged with a spurious
    ABORTED_EXCEPTION and re-raise a fault trace out of a clean shutdown. That
    was unpinned until 2026-07-29, and the Phase-F _run_toss_cycle extraction
    silently regressed it (the shutdown exit fell through to the common
    abort() branch) — TossContinuous asserted parity with a single Toss that
    had stopped behaving that way."""
    import rclpy
    node = _toss_ready_node(time.perf_counter())
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    _script_position_accept(node, monkeypatch)
    safed = []
    monkeypatch.setattr(node, '_toss_safe_abort',
                        lambda state=None: safed.append(1))
    calls = {'n': 0}

    def _ok():
        calls['n'] += 1
        return calls['n'] < 2                    # one loop pass, then shutdown
    monkeypatch.setattr(rclpy, 'ok', _ok)
    gh = _TossGoalHandle()
    result = node._execute_toss(gh)
    assert result.outcome == 'ABORTED_SHUTDOWN'
    assert safed == [1]
    assert gh.terminal is None


# ── Tier 8b (Phase 4): pretilt_hold choreography + deferred A→B reach ──────────

def _fresh_seq_8b(node, pose=(50.0, 0.0, 170.0), throw_site=(0.0, 0.0),
                  flight=0.8, delay=5.0, start=100.0):
    """Install per-goal 8b state (via compute_release_state_tilted, the tilted
    branch _execute_toss runs) + a Tier-8b sequencer, for node-level FSM-tick
    tests that bypass the go_to_pose service."""
    release = compute_release_state_tilted(pose, flight,
                                           throw_site_xy_mm=throw_site)
    with node._lock:
        node._announced_ball_id = None
        node._announced_id_untagged = False
        node._preexisting_flight_ids = {
            int(b.id) for b in node._balls if int(b.status) == 1}
        node._catch_vel_scale = float(hw.JB_OP_CATCH_VEL_SCALE_DEFAULT)
        node._toss_committed.release_state = release
        # Zero aim map in these fixtures ⇒ the commanded release IS the
        # announcement release, the same object (C-TOSS-CAL-1's disabled path).
        node._toss_committed.release_cmd = release
        node._toss_committed.aim = None
        node._toss_committed.landing_global_mm = tuple(
            float(v) for v in release.catch_point_global_mm)
        # Mirror _execute_toss: the tilted cross-check target is the commanded A
        # pose (single source _toss_positioning_xyz), NOT the nominated catch B.
        node._toss_committed.platform_target_mm = (
            ReloadCoordinatorNode._toss_positioning_xyz(pose, release))
        node._toss_committed.waiver = False
        node._toss_committed.prepare_pending = False
        node._toss_committed.throw_dispatched = False
        node._toss_committed.stroke_seen = False
        node._toss_committed.track_confirmed = False
        node._toss_committed.pretilt_hold_raised = False
        node._toss_committed.announced_reach = None
    seq = TossSequencer(catch_pose_stow_mm=pose, flight_time_s=flight,
                        throw_delay_s=delay, tier=TIER_8B,
                        throw_site_xy_mm=throw_site, throw_site_known=True,
                        event_vel_mps=float(release.event_vel_mps))
    seq.start(start)
    return seq, release


def test_pretilt_hold_raised_at_prepare_for_8b(monkeypatch):
    """Tier 8b raises catch/pretilt_hold on the ACTION_PREPARE_CATCH tick
    (alongside prime_hold, >=2 FSM ticks before our announcement can reach
    catch_coordinator) and records the cycle state's ``pretilt_hold_raised``."""
    t0 = 100.0
    node = _toss_ready_node(t0)
    seq, _ = _fresh_seq_8b(node, start=t0)
    calls = []
    monkeypatch.setattr(node, '_position_platform_for_toss',
                        lambda s, state=None: s.note_position_result(
                            t0, True, 0.3))
    monkeypatch.setattr(node, '_publish_prime_hold',
                        lambda h: calls.append(('prime_hold', h)))
    monkeypatch.setattr(node, '_publish_pretilt_hold',
                        lambda h: calls.append(('pretilt_hold', h)))
    node._step_toss_sequence(seq, t0)                    # POSITION
    _stamp_fresh(node, t0 + 0.5)
    d = node._step_toss_sequence(seq, t0 + 0.5)          # verified arrival: PREPARE
    assert d.action == ACTION_PREPARE_CATCH
    assert ('prime_hold', True) in calls
    assert ('pretilt_hold', True) in calls
    assert node._toss_committed.pretilt_hold_raised is True


def test_pretilt_hold_is_raised_for_8a_too_and_prime_hold_still_leads(monkeypatch):
    """⚠ **SUPERSEDED PIN, kept re-pointed rather than deleted.**

    This was ``test_pretilt_hold_never_published_for_8a``: THE 8a byte-identity
    pin, asserting that an 8a goal never touches ``catch/pretilt_hold`` so the
    publish sequence stayed bit-identical to Phase 1. Census E5 retires that
    identity DELIBERATELY (2026-08-22) — at the cadence rungs CCN's pre-tilt
    arrival clamps to the landing itself, so even a level 8a would command a
    reach arriving AT CONTACT, and at a sub-second dwell the next cycle's
    announcement lands inside the previous ball's settle-hold window.

    What the test still guards is the half that did NOT change and must not: the
    PREPARE tick raises ``prime_hold`` and the ORDER is unchanged. The
    prime_hold-before-armed ordering is the load-bearing one (an armed-edge
    auto-prime would ascend with the seated ball); pretilt_hold rides alongside
    it on the same tick, which is where it already rode for 8b."""
    t0 = 100.0
    node = _toss_ready_node(t0)
    seq = _fresh_seq(node, start=t0)                     # tier 8a (default)
    calls = []
    monkeypatch.setattr(node, '_position_platform_for_toss',
                        lambda s, state=None: s.note_position_result(
                            t0, True, 0.3))
    monkeypatch.setattr(node, '_publish_prime_hold',
                        lambda h: calls.append(('prime_hold', h)))
    monkeypatch.setattr(node, '_publish_pretilt_hold',
                        lambda h: calls.append(('pretilt_hold', h)))
    node._step_toss_sequence(seq, t0)
    _stamp_fresh(node, t0 + 0.5)
    d = node._step_toss_sequence(seq, t0 + 0.5)
    assert d.action == ACTION_PREPARE_CATCH
    assert calls == [('prime_hold', True), ('pretilt_hold', True)]
    assert node._toss_committed.pretilt_hold_raised is True


def _capture_go_to_pose(node, monkeypatch, accepted=True):
    """Seam the go_to_pose client so a test can drive the REAL
    _position_platform_for_toss and read the request it builds: call_async
    captures the request, _wait_future returns a canned accept (bypassing the
    never-resolving MockFuture). Returns the captured-request dict."""
    captured = {}
    monkeypatch.setattr(node._go_to_pose_cli, 'call_async',
                        lambda req: captured.__setitem__('req', req))
    resp = types.SimpleNamespace(accepted=bool(accepted), code='OK',
                                 planned_duration_s=0.3, message='planned OK')
    monkeypatch.setattr(node, '_wait_future', lambda fut, timeout_s=2.0: resp)
    return captured


def test_8b_positioning_commands_tilted_pretilt_pose_at_A(monkeypatch):
    """FIX-1: an 8b toss pre-positions at the swing-compensated PRE-TILT pose at
    the throw site A with a NON-identity tilted orientation — NOT level at B. This
    drives the REAL _position_platform_for_toss (no monkeypatch of it — the 8b
    pretilt-hold tests all stub it, so none cover the pose it actually sends) and
    captures the go_to_pose request. Without this the platform would pre-position
    level at B and throw straight up, orphaning the computed pre-tilt aim."""
    node = _toss_ready_node(100.0)
    seq, release = _fresh_seq_8b(node, pose=(50.0, 0.0, 170.0),
                                 throw_site=(0.0, 0.0), start=100.0)
    captured = _capture_go_to_pose(node, monkeypatch)
    node._position_platform_for_toss(seq)
    req = captured['req']
    # POSITION = A (swing-compensated pretilt pose), NOT the level catch pose B.
    pre = np.asarray(release.pretilt_pose_stow, dtype=float)
    assert req.pose.position.x == pytest.approx(float(pre[0]))
    assert req.pose.position.y == pytest.approx(float(pre[1]))
    assert req.pose.position.z == pytest.approx(float(pre[2]))
    assert req.pose.position.x != pytest.approx(50.0)         # NOT B's x
    # ORIENTATION is the throw tilt — NON-identity — and round-trips (through the
    # SAME decode go_to_pose runs) back to the (tilt_rx, tilt_ry, 0) rotvec.
    q = req.pose.orientation
    assert not (q.w == pytest.approx(1.0) and q.x == pytest.approx(0.0)
                and q.y == pytest.approx(0.0) and q.z == pytest.approx(0.0))
    from jugglebot.motion.ik_solver import (
        quat_to_rot_matrix, rot_matrix_to_rotvec)
    rotvec = rot_matrix_to_rotvec(
        quat_to_rot_matrix(float(q.w), float(q.x), float(q.y), float(q.z)))
    assert rotvec[0] == pytest.approx(release.tilt_rx, abs=1e-9)
    assert rotvec[1] == pytest.approx(release.tilt_ry, abs=1e-9)
    assert rotvec[2] == pytest.approx(0.0, abs=1e-9)
    assert abs(release.tilt_rx) + abs(release.tilt_ry) > 1e-6  # genuinely tilted


def test_8a_positioning_commands_level_pose_at_B(monkeypatch):
    """The 8a byte-identity pin: an 8a toss pre-positions LEVEL (identity
    orientation) at the nominated catch pose B — the tilted 8b branch must not
    perturb it. Drives the REAL _position_platform_for_toss and reads the pose."""
    node = _toss_ready_node(100.0)
    seq = _fresh_seq(node, pose=(30.0, -40.0, 170.0), start=100.0)
    captured = _capture_go_to_pose(node, monkeypatch)
    node._position_platform_for_toss(seq)
    req = captured['req']
    assert (req.pose.position.x, req.pose.position.y, req.pose.position.z) == \
        pytest.approx((30.0, -40.0, 170.0))                  # level pose at B
    q = req.pose.orientation
    assert (q.w, q.x, q.y, q.z) == pytest.approx((1.0, 0.0, 0.0, 0.0))  # identity
    # 8a cross-check target stays B, equal to what go_to_pose is commanded.
    assert node._toss_committed.platform_target_mm == pytest.approx(
        (30.0, -40.0, 170.0))


def test_8b_cross_check_target_equals_commanded_A_pose(monkeypatch):
    """FIX-1 follow-up: the POSITIONING mocap arrival cross-check target
    (the cycle state's ``platform_target_mm``) must equal the pose go_to_pose is
    COMMANDED — A (the swing-compensated pre-tilt pose) for 8b, NOT the nominated
    catch B. Single source (_toss_positioning_xyz) so command and verification can
    never diverge; else an operator who configures a platform body for an 8b
    sitting gets measured-A vs target-B → a spurious ABORTED_POSITION_FAILED. The
    cross-check itself stays disabled by default; this pins target == commanded."""
    node = _toss_ready_node(100.0)
    seq, release = _fresh_seq_8b(node, pose=(50.0, 0.0, 170.0),
                                 throw_site=(0.0, 0.0), start=100.0)
    captured = _capture_go_to_pose(node, monkeypatch)
    node._position_platform_for_toss(seq)          # the REAL command path
    req = captured['req']
    commanded = (req.pose.position.x, req.pose.position.y, req.pose.position.z)
    pre = np.asarray(release.pretilt_pose_stow, dtype=float)
    # Verification target == the commanded A pose == the pre-tilt pose, NOT B.
    assert node._toss_committed.platform_target_mm == pytest.approx(commanded)
    assert node._toss_committed.platform_target_mm == pytest.approx(
        (float(pre[0]), float(pre[1]), float(pre[2])))
    # NOT B's x
    assert node._toss_committed.platform_target_mm[0] != pytest.approx(50.0)


class _FakePerfClock:
    """The node module's ``time`` surface (``perf_counter`` + ``sleep`` are the
    only two members ``reload_coordinator_node`` uses).

    Needed wherever a test drives the REAL ``_position_platform_for_toss``
    through more than one tick: the method anchors the timed arrival on
    ``time.perf_counter()``, so on the real clock the arrival lands ~1e5 s away
    from the synthetic ``now`` the FSM is being ticked at and the ladder never
    advances past POSITIONING."""

    def __init__(self, t0):
        self.t = float(t0)

    def perf_counter(self):
        return self.t

    def sleep(self, dt):
        self.t += float(dt)


def _capture_go_to_pose_ladder(node, monkeypatch, responses):
    """:func:`_capture_go_to_pose` for a SEQUENCE of responses: every request is
    captured in order and ``_wait_future`` serves the next canned response (the
    last one repeats), so a test can drive the BUSY re-poll through the REAL
    ``_position_platform_for_toss`` and read every request it built."""
    reqs = []
    monkeypatch.setattr(node._go_to_pose_cli, 'call_async',
                        lambda req: reqs.append(req))
    queue = list(responses)

    def serve(fut, timeout_s=2.0):
        return queue.pop(0) if len(queue) > 1 else queue[0]

    monkeypatch.setattr(node, '_wait_future', serve)
    return reqs


def test_position_busy_redispatches_the_go_to_pose(monkeypatch):
    """The node half of the BUSY re-poll (bag 2026-08-28_23-53-25): the FSM
    re-emits ACTION_POSITION_PLATFORM and `_step_toss_sequence` re-runs
    `_position_platform_for_toss` VERBATIM — same cached
    ``state.positioning_move``, same ``_toss_positioning_xyz``, so the retried
    request is the SAME pose. A retry that re-derived the pose would be a second
    source for the commanded site, which is the drift `_build_toss_cycle`'s cache
    exists to prevent.

    Then the cycle rejoins the ordinary ladder and reaches PREPARING, which is
    the whole point: pre-fix the first BUSY terminalised the cycle and
    toss_session escalated it to ABORTED_CYCLE_REJECTED_POSITION(BUSY)."""
    t0 = 100.0
    clock = _FakePerfClock(t0)
    monkeypatch.setattr(rcn, 'time', clock)
    node = _toss_ready_node(t0)
    seq = _fresh_seq(node, pose=(30.0, -40.0, 170.0), start=t0)
    busy = types.SimpleNamespace(
        accepted=False, code='BUSY', planned_duration_s=0.0,
        message='a move is in flight — Phase 2 accepts moves only from hold')
    ok = types.SimpleNamespace(accepted=True, code='OK', planned_duration_s=0.0,
                               message='planned OK')
    reqs = _capture_go_to_pose_ladder(node, monkeypatch, [busy, ok])

    d = node._step_toss_sequence(seq, t0)
    assert d.action == ACTION_POSITION_PLATFORM
    assert len(reqs) == 1                       # the BUSY-refused first attempt

    prepared = None
    for i in range(1, 120):
        t = t0 + i * 0.02
        clock.t = t
        _stamp_fresh(node, t)
        d = node._step_toss_sequence(seq, t)
        assert not d.done, d.result.outcome
        if d.action == ACTION_PREPARE_CATCH:
            prepared = d
            break
    assert len(reqs) == 2, 'the BUSY was never re-polled'
    # The retried request is byte-identical to the refused one.
    first, second = reqs
    assert (second.pose.position.x, second.pose.position.y,
            second.pose.position.z) == pytest.approx(
        (first.pose.position.x, first.pose.position.y, first.pose.position.z))
    q1, q2 = first.pose.orientation, second.pose.orientation
    assert (q2.w, q2.x, q2.y, q2.z) == pytest.approx((q1.w, q1.x, q1.y, q1.z))
    # …and the cycle really did get past POSITIONING on the retry.
    assert prepared is not None and prepared.phase == PHASE_PREPARING
    assert seq.position_busy_polls == 1
    assert seq.position_busy_wait_s > 0.0


def test_step_dispatches_reach_catch_to_publish_toss_reach(monkeypatch):
    """The ACTION_REACH_CATCH decision routes to _publish_toss_reach — the
    node's ONLY platform publish for an 8b flight."""
    node = ReloadCoordinatorNode()
    called = []
    monkeypatch.setattr(node, '_publish_toss_reach',
                        lambda state=None: called.append(1))
    monkeypatch.setattr(node, '_build_toss_observations',
                        lambda now, state=None: None)
    stub = types.SimpleNamespace(
        tier=TIER_8B,
        step=lambda now, obs: TossDecision(
            PHASE_BALL_IN_FLIGHT, ACTION_REACH_CATCH, False, None))
    node._step_toss_sequence(stub, 105.0)
    assert called == [1]


def test_publish_toss_reach_publishes_one_target_for_B():
    """The deferred A→B reach: _publish_toss_reach publishes ONE
    catch/dynamic_target from the stashed announced landing via
    predicted_catch_command (the SAME policy CCN uses — single-sourced pose),
    arrival = the announced landing crossed to the perf clock (MockClock ros
    now = 0 ⇒ arrival = landing_time_ros + now_perf; lead = the flight by
    construction)."""
    node = ReloadCoordinatorNode()
    landing_pos = np.array([50.0, 0.0, 809.08])          # B's cup point (global mm)
    landing_vel = np.array([100.0, 0.0, -3900.0])
    landing_time_ros = 105.8
    with node._lock:
        node._toss_committed.announced_reach = (
            landing_pos, landing_vel, landing_time_ros)
    n0 = len(node._dyn_target_pub.published)
    before = time.perf_counter()
    node._publish_toss_reach()
    after = time.perf_counter()
    assert len(node._dyn_target_pub.published) == n0 + 1
    out = node._dyn_target_pub.published[-1]
    # arrival = landing_time_ros + (now_perf − 0) ∈ [before, after] + landing.
    assert before + landing_time_ros <= out.arrival_time <= after + landing_time_ros
    # Pose == predicted_catch_command's (no drift-prone second copy of the math).
    cmd = node._toss_catch_policy.predicted_catch_command(
        landing_pos, landing_vel, landing_time_ros)
    assert out.target_pos.x == pytest.approx(cmd.target_pos[0])
    assert out.target_pos.y == pytest.approx(cmd.target_pos[1])
    assert out.target_pos.z == pytest.approx(cmd.target_pos[2])
    assert out.target_quat.w == pytest.approx(cmd.target_quat[0])
    assert out.target_quat.x == pytest.approx(cmd.target_quat[1])


def test_publish_toss_reach_no_stash_publishes_nothing():
    """A missing announced-landing stash (reach requested before announce, or a
    crashed goal) publishes NO platform target — the FSM's ABORTED_NO_RELEASE /
    MISSED path still cleans up."""
    node = ReloadCoordinatorNode()
    with node._lock:
        node._toss_committed.announced_reach = None
    n0 = len(node._dyn_target_pub.published)
    node._publish_toss_reach()
    assert len(node._dyn_target_pub.published) == n0


def test_publish_toss_reach_policy_reject_publishes_nothing(monkeypatch):
    """A predicted_catch_command reject (should not happen post-CHECKING, but
    guarded) publishes nothing."""
    node = ReloadCoordinatorNode()
    with node._lock:
        node._toss_committed.announced_reach = (np.zeros(3), np.zeros(3), 105.8)
    monkeypatch.setattr(node._toss_catch_policy, 'predicted_catch_command',
                        lambda p, v, t: None)
    n0 = len(node._dyn_target_pub.published)
    node._publish_toss_reach()
    assert len(node._dyn_target_pub.published) == n0


# ── P-4: the deferred reach holds the throw's PRE-TILT when the two agree ─────
#
# `tests/hardware/session_cadence_ladder.md` carried finding 2, closed
# 2026-08-27. The cycle's TERMINAL orientation was chosen by the catch policy
# (level the cup to receive) and the NEXT cycle's INITIAL orientation by the
# throw aim, with nothing reconciling them — so an armed aim was taken back out
# of the platform every cycle, the B1 skip honestly declined, and no chained
# aimed cycle could ever STAGE. Five tests: the zero-aim path is untouched
# (twice — synthetically, against a non-identity policy answer, and end to end),
# the aimed co-located path closes the chain (the closure itself, with the
# pre-fix counterfactual beside it), the displaced path is left alone, and the
# bound that separates the two regimes is derived rather than picked.
# The pipeline CONSEQUENCE — an aimed chain now stages — is pinned in
# tests/ros/test_toss_continuous_node.py, where the two-slot harness lives.


def _armed_aim(node, monkeypatch, rx, ry):
    """Arm a layer-1-shaped aim of exactly ``(rx, ry)`` rad on this node.

    Wraps the REAL ``_toss_aim_for_goal`` rather than substituting a hand-built
    dict, so every other key of the block stays the production default and
    ``_build_toss_cycle`` runs its real aim arm — the virtual-target
    ``compute_release_state_tilted`` call that turns an aim into a commanded
    pre-tilt. ``offset_mm`` is computed by the same ``aim_target_offset_mm``
    production uses, so the resulting tilt IS the armed aim."""
    real = node._toss_aim_for_goal

    def aimed(catch_pose, flight):
        block = real(catch_pose, flight)
        off = aim_target_offset_mm(float(rx), float(ry), float(flight),
                                   float(catch_pose[2]))
        block['aim_rad'] = (float(rx), float(ry))
        block['offset_mm'] = (float(off[0]), float(off[1]))
        return block

    monkeypatch.setattr(node, '_toss_aim_for_goal', aimed)


def _announce_and_reach(node, seq):
    """The cycle's REAL announcement (which stashes ``announced_reach``) followed
    by its REAL deferred reach. Returns the published DynamicTargetCommand."""
    seq._prepare_dispatched = True            # note_announcement's FSM gate
    node._announce_toss(seq)
    node._publish_toss_reach()
    return node._dyn_target_pub.published[-1]


def _policy_answer(node, state):
    """What the catch policy alone would have commanded for this cycle — i.e.
    the pre-fix reach, recomputed from the same stash."""
    with node._lock:
        stash = state.announced_reach
    return node._toss_catch_policy.predicted_catch_command(*stash)


def _park(node, now, pos, quat):
    """Park the platform at a commanded pose, decoding the wire orientation
    EXACTLY as trajectory_node's ``_catch_target_from_msg`` does (quat → matrix →
    rotvec) — the INTENT frame ``trajectory/commanded_pose`` republishes and the
    B1 skip compares against."""
    rotvec = rot_matrix_to_rotvec(quat_to_rot_matrix(
        float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])))
    pos = tuple(float(v) for v in pos)
    with node._lock:
        node._commanded_pos_mm = pos
        node._commanded_pos_mono = now
        node._commanded_pose = pos + tuple(float(v) for v in rotvec)
        node._commanded_pose_mono = now


def _wire_quat(out):
    return (out.target_quat.w, out.target_quat.x,
            out.target_quat.y, out.target_quat.z)


def test_the_reach_quat_is_the_policys_verbatim_when_the_release_is_LEVEL():
    """P-4, the byte-identity half: a LEVEL commanded release never reaches one
    float of the substitution.

    Driven with a landing velocity that is NOT vertical (25.6 mrad off), so the
    policy's quaternion is a real receive tilt rather than an identity any
    implementation would reproduce — and compared with ``==``, not ``approx``."""
    node = ReloadCoordinatorNode()
    landing_pos = np.array([50.0, 0.0, 809.08])
    landing_vel = np.array([100.0, 0.0, -3900.0])
    for release in (None, compute_release_state((50.0, 0.0, 170.0), 0.8)):
        with node._lock:
            node._toss_committed.announced_reach = (landing_pos, landing_vel,
                                                    105.8)
            node._toss_committed.release_cmd = release
        node._publish_toss_reach()
        out = node._dyn_target_pub.published[-1]
        cmd = node._toss_catch_policy.predicted_catch_command(
            landing_pos, landing_vel, 105.8)
        assert _wire_quat(out) == tuple(float(v) for v in cmd.target_quat)
        # …and the policy's answer was a REAL tilt, so the compare had something
        # to fail on.
        assert cmd.target_quat[0] != 1.0


def test_a_zero_aim_chain_publishes_the_policy_quat_unchanged():
    """The same byte-identity, end to end through the real builder: with NO aim
    armed the commanded release is the announcement release (C-TOSS-CAL-1's
    disabled path), the release is level, and the wire carries the policy's own
    quaternion component for component."""
    now = time.perf_counter()
    node = _toss_ready_node(now, commanded_pos=(0.0, 0.0, 170.0))
    seq, state = node._build_toss_cycle((0.0, 0.0, 170.0), 0.8, 5.0, 0.0)
    assert node._release_is_tilted(state.release_cmd) is False
    out = _announce_and_reach(node, seq)
    cmd = _policy_answer(node, state)
    assert _wire_quat(out) == tuple(float(v) for v in cmd.target_quat)
    # A self-toss lands vertically, so that IS identity — asserted so the
    # zero-aim premise of the whole fix is on the record.
    assert _wire_quat(out) == (1.0, 0.0, 0.0, 0.0)


def test_an_aimed_colocated_reach_holds_the_pretilt_and_closes_the_chain(
        monkeypatch):
    """**THE P-4 closure**, end to end: an aimed co-located cycle publishes its
    deferred reach at the throw's PRE-TILT, so the platform never un-tilts and
    the NEXT cycle's census-B1 skip fires.

    The counterfactual is in the test: parked at the policy's own answer — which
    is EXACTLY level, because the announcement is built from the uncorrected
    release, so the receive tilt has no aim in it — the same next cycle declines
    the skip and commands the 0.520 s move. That decline is carried finding 2."""
    now = time.perf_counter()
    node = _toss_ready_node(now, commanded_pos=(0.0, 0.0, 170.0))
    aim = (math.radians(0.8), math.radians(-0.5))          # |aim| = 0.943°
    _armed_aim(node, monkeypatch, *aim)
    seq, state = node._build_toss_cycle((0.0, 0.0, 170.0), 0.8, 5.0, 0.0)
    rel = state.release_cmd
    assert node._release_is_tilted(rel) is True
    assert (rel.tilt_rx, rel.tilt_ry) == pytest.approx(aim, abs=1e-6)

    out = _announce_and_reach(node, seq)
    cmd = _policy_answer(node, state)
    # The policy's answer was LEVEL — the re-tilt this fix removes was real.
    assert tuple(float(v) for v in cmd.target_quat) == (1.0, 0.0, 0.0, 0.0)
    pre = ReloadCoordinatorNode._tilt_quaternion(rel.tilt_rx, rel.tilt_ry)
    assert _wire_quat(out) == (pre.w, pre.x, pre.y, pre.z)
    # The POSITION is untouched — which is what keeps _predicted_chain_site_mm
    # a prediction of what is commanded.
    assert (out.target_pos.x, out.target_pos.y, out.target_pos.z) == (
        pytest.approx(float(cmd.target_pos[0])),
        pytest.approx(float(cmd.target_pos[1])),
        pytest.approx(float(cmd.target_pos[2])))

    # ── the closure: park where the reach commanded, and the next cycle skips ──
    _park(node, time.perf_counter(), (out.target_pos.x, out.target_pos.y,
                                      out.target_pos.z), _wire_quat(out))
    seq2, state2 = node._build_toss_cycle((0.0, 0.0, 170.0), 0.8, 5.0, 0.0)
    px, py, pz = ReloadCoordinatorNode._toss_positioning_xyz(
        (0.0, 0.0, 170.0), state2.release_cmd)
    assert node._toss_already_positioned(px, py, pz, state2.release_cmd) is True
    assert state2.positioning_move is False
    assert seq2.positioning_move_expected is False
    # The position half never broke: the residual is the cup swing alone.
    live = node._live_commanded_pose(time.perf_counter())
    assert math.hypot(live[0] - px, live[1] - py) < 1.2      # mm, vs 17.5 tol

    # ── the counterfactual: the pre-fix reach, i.e. the finding itself ──
    _park(node, time.perf_counter(), (out.target_pos.x, out.target_pos.y,
                                      out.target_pos.z), cmd.target_quat)
    seq3, state3 = node._build_toss_cycle((0.0, 0.0, 170.0), 0.8, 5.0, 0.0)
    assert node._toss_already_positioned(px, py, pz, state3.release_cmd) is False
    assert state3.positioning_move is True
    assert seq3.positioning_move_expected is True


@pytest.mark.parametrize('aim_deg,axis', [(0.0, 'rx'), (1.0, 'rx'),
                                          (-1.0, 'rx'), (1.0, 'ry'),
                                          (-1.0, 'ry')])
def test_a_displaced_reach_keeps_the_policys_receive_tilt(monkeypatch, aim_deg,
                                                          axis):
    """A DISPLACED cycle (A ≠ B) is left alone, with or without a saturated aim,
    **on either aim axis**.

    There the announced landing carries the throw's own lateral velocity, so the
    receive tilt is the MIRROR of the throw tilt and the delta is ~2θ — far
    outside the ±1° bound. The branch IS entered (the release is tilted) and
    declines on the bound, which is what makes this test non-vacuous.

    The AXIS is parametrised because it is not a free variation: this harness
    displaces along x, and ``aim_target_offset_mm`` maps rx onto -y and ry onto
    +x, so an ``rx`` aim is ORTHOGONAL to the throw tilt (it adds in quadrature)
    while an ``ry`` aim is CO-AXIAL with it and one sign of it SUBTRACTS. Only
    the co-axial-opposing case can push the delta back toward the bound, so an
    rx-only parametrisation was testing the easy direction three times. At this
    harness's 60 mm all five still decline (the co-axial-opposing worst case is
    79.3 mrad, still 4.5× the bound); the crossover that case DOES set is pinned
    in ``test_the_displaced_regime_is_separated_by_more_than_the_bound``."""
    now = time.perf_counter()
    node = _toss_ready_node(now, commanded_pos=(-60.0, 0.0, 170.0))
    if aim_deg:
        rad = math.radians(aim_deg)
        _armed_aim(node, monkeypatch, *((rad, 0.0) if axis == 'rx'
                                        else (0.0, rad)))
    seq, state = node._build_toss_cycle((0.0, 0.0, 170.0), 0.8, 5.0, 0.0)
    assert node._release_is_tilted(state.release_cmd) is True
    out = _announce_and_reach(node, seq)
    cmd = _policy_answer(node, state)
    assert _wire_quat(out) == tuple(float(v) for v in cmd.target_quat)
    assert cmd.target_quat[0] != 1.0                     # a real receive tilt
    _, _, _, _, held = ReloadCoordinatorNode._toss_reach_quat(
        cmd.target_quat, state.release_cmd)
    assert held is False


def test_the_displaced_regime_is_separated_by_more_than_the_bound():
    """The bound is ``toss_cal.TOTAL_MAX_RAD`` — the ±1° aim authority — and it
    is the RIGHT number because the delta it bounds IS the seat re-aim the
    substitution costs. Re-derived from the live modules, not transcribed:

    * aimed co-located ⇒ delta is EXACTLY the aim magnitude, which
      ``clamp_total_aim`` has already bounded by the same constant, so the rule
      fires by construction and the seat tilt is the aim's own ≤1°;
    * displaced ⇒ delta ~2θ: 32.3 mrad at 20 mm, 240.8 mrad at the 150 mm cap
      (flight 0.5029 s) — 1.85× to 13.8× the bound;
    * the crossover is ~10.8 mm of displacement at that flight time with NO aim
      armed, and ~21.6 mm in the worst case an aim can produce — a SATURATED
      CO-AXIAL OPPOSING aim, which subtracts from the throw tilt rather than
      adding to it. Below the crossover the rule DOES fire on a displaced goal,
      and that is the bound working: the seat re-aim it buys is inside the same
      ≤1°, and the lateral cup shift is inside the 1.131 mm that
      ``TOTAL_MAX_RAD``'s own derivation names."""
    node = ReloadCoordinatorNode()
    flight, B = 0.5029, (0.0, 0.0, float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM))

    def delta(release_ann, release_cmd):
        fields = build_announcement_fields(release_ann, throw_time_s=0.0)
        cmd = node._toss_catch_policy.predicted_catch_command(
            np.asarray(fields['landing_position'], dtype=float),
            np.asarray(fields['landing_velocity'], dtype=float), flight)
        recv = rot_matrix_to_rotvec(quat_to_rot_matrix(
            *(float(v) for v in cmd.target_quat)))
        target = np.array([float(release_cmd.tilt_rx),
                           float(release_cmd.tilt_ry), 0.0])
        return float(np.linalg.norm(np.asarray(recv, dtype=float) - target))

    level = compute_release_state_tilted(B, flight, throw_site_xy_mm=(0.0, 0.0))
    for aim_rad in (math.radians(0.155), math.radians(0.5), toss_cal.TOTAL_MAX_RAD):
        off = aim_target_offset_mm(aim_rad, 0.0, flight, B[2])
        aimed = compute_release_state_tilted(
            (B[0] + off[0], B[1] + off[1], B[2]), flight,
            throw_site_xy_mm=(0.0, 0.0))
        d = delta(level, aimed)
        assert d == pytest.approx(aim_rad, abs=1e-9)       # EXACTLY the aim
        assert d <= toss_cal.TOTAL_MAX_RAD                 # so the rule fires

    for disp_mm, ratio in ((20.0, 1.8), (50.0, 4.6), (150.0, 13.7)):
        rel = compute_release_state_tilted(B, flight,
                                           throw_site_xy_mm=(-disp_mm, 0.0))
        d = delta(rel, rel)
        assert d > toss_cal.TOTAL_MAX_RAD
        assert d / toss_cal.TOTAL_MAX_RAD > ratio
    # …and the crossover, stated so a future flight-time change is visible: the
    # bound is on the ANGLE, so the displacement it corresponds to scales as g·T².
    below = compute_release_state_tilted(B, flight, throw_site_xy_mm=(-10.0, 0.0))
    above = compute_release_state_tilted(B, flight, throw_site_xy_mm=(-11.5, 0.0))
    assert delta(below, below) <= toss_cal.TOTAL_MAX_RAD
    assert delta(above, above) > toss_cal.TOTAL_MAX_RAD

    # ── the crossover an ARMED AIM can move it to, which is the one that
    # bounds the real system. An aim does NOT simply add: aim_target_offset_mm
    # maps rx → −y and ry → +x, so against an x-displacement the ry component
    # is CO-AXIAL with the throw tilt and one of its signs SUBTRACTS. That
    # co-axial-opposing saturated aim is the worst case, and it pushes the
    # crossover out from ~10.8 mm to ~21.6 mm. Asserted as a fires/declines
    # PAIR either side of it, the same shape as the zero-aim pair above.
    A = toss_cal.TOTAL_MAX_RAD

    def coaxial_opposing_delta(disp_mm):
        """delta for a displaced throw carrying a saturated aim that opposes
        its own throw tilt — announcement from the UNCORRECTED release
        (C-TOSS-CAL-1 D4), commanded release from the aim-offset target."""
        base = compute_release_state_tilted(B, flight,
                                            throw_site_xy_mm=(-disp_mm, 0.0))
        off = aim_target_offset_mm(0.0, -A, flight, B[2])
        cmd_rel = compute_release_state_tilted(
            (B[0] + off[0], B[1] + off[1], B[2]), flight,
            throw_site_xy_mm=(-disp_mm, 0.0))
        return delta(base, cmd_rel)

    assert coaxial_opposing_delta(21.5) <= A          # still fires
    assert coaxial_opposing_delta(21.75) > A          # declines
    # …and that the AIM is what moved it: at the same 21.5 mm the unaimed delta
    # is far outside the bound, so this pair is not re-testing the one above.
    at_21_5 = compute_release_state_tilted(B, flight,
                                           throw_site_xy_mm=(-21.5, 0.0))
    assert delta(at_21_5, at_21_5) > A

    # The ORTHOGONAL axis is the easy direction — it adds in quadrature, so it
    # can only ever move the crossover IN. Pinned so the asymmetry is on the
    # record rather than rediscovered.
    off_rx = aim_target_offset_mm(A, 0.0, flight, B[2])
    base20 = compute_release_state_tilted(B, flight,
                                          throw_site_xy_mm=(-20.0, 0.0))
    orth20 = delta(base20, compute_release_state_tilted(
        (B[0] + off_rx[0], B[1] + off_rx[1], B[2]), flight,
        throw_site_xy_mm=(-20.0, 0.0)))
    assert orth20 == pytest.approx(math.hypot(delta(base20, base20), A),
                                   rel=1e-4)
    assert orth20 > delta(base20, base20) > A
    # …while the co-axial-opposing aim at the same 20 mm brings it INSIDE.
    assert coaxial_opposing_delta(20.0) < A

    # THE ANSWERED SUB-QUESTION of `session_cadence_ladder.md`'s finding 2: the
    # reach's POSITION never broke the 17.5 mm half of the B1 test. At a full ±1°
    # aim the next cycle's swing-compensated pre-tilt pose sits 1.013 mm from the
    # centroid the catch parks at — the cup swing alone, 17x inside the bound.
    off = aim_target_offset_mm(toss_cal.TOTAL_MAX_RAD, 0.0, flight, B[2])
    saturated = compute_release_state_tilted(
        (B[0] + off[0], B[1] + off[1], B[2]), flight, throw_site_xy_mm=(0.0, 0.0))
    fields = build_announcement_fields(level, throw_time_s=0.0)
    parked = node._toss_catch_policy.predicted_catch_command(
        np.asarray(fields['landing_position'], dtype=float),
        np.asarray(fields['landing_velocity'], dtype=float), flight).target_pos
    pre = np.asarray(saturated.pretilt_pose_stow, dtype=float)
    residual_mm = math.hypot(pre[0] - float(parked[0]), pre[1] - float(parked[1]))
    assert residual_mm == pytest.approx(1.013, abs=0.005)
    assert residual_mm < rcn._TOSS_ALREADY_THERE_TOL_MM / 10.0


def test_toss_recenter_releases_pretilt_hold_last_when_raised(monkeypatch):
    """Tier 8b RECENTER: catch/pretilt_hold released LAST of all (after
    prime_hold), iff it was raised this goal — same cross-topic-ordering
    rationale; a stale pretilt_hold only DEGRADES a later reload, never a
    hazard."""
    node = ReloadCoordinatorNode()
    node._toss_committed.pretilt_hold_raised = True
    order = []
    monkeypatch.setattr(node, '_arm_catch',
                        lambda a: order.append(('arm', a)) or True)
    monkeypatch.setattr(node, '_publish_catch_armed',
                        lambda a: order.append(('armed', a)))
    monkeypatch.setattr(node, '_go_home', lambda: order.append('home') or True)
    monkeypatch.setattr(node, '_publish_prime_hold',
                        lambda h: order.append(('prime_hold', h)))
    monkeypatch.setattr(node, '_publish_pretilt_hold',
                        lambda h: order.append(('pretilt_hold', h)))
    node._toss_recenter()
    assert order == [('arm', False), ('armed', False), 'home',
                     ('prime_hold', False), ('pretilt_hold', False)]


def test_toss_safe_abort_releases_pretilt_hold_last_when_raised(monkeypatch):
    """Tier 8b SAFE_ABORT: catch/pretilt_hold released LAST of all, iff raised —
    it outlives the whole teardown (same reason as prime_hold)."""
    node = ReloadCoordinatorNode()
    node._toss_committed.pretilt_hold_raised = True
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    order = []
    monkeypatch.setattr(node, '_publish_catch_armed',
                        lambda a: order.append(('armed', a)))
    monkeypatch.setattr(node, '_smooth_move_hand',
                        lambda p: order.append(('retract', p)) or True)
    monkeypatch.setattr(node, '_arm_catch',
                        lambda a: order.append(('arm', a)) or True)
    monkeypatch.setattr(node, '_go_home', lambda: order.append('home') or True)
    monkeypatch.setattr(node, '_publish_prime_hold',
                        lambda h: order.append(('prime_hold', h)))
    monkeypatch.setattr(node, '_publish_pretilt_hold',
                        lambda h: order.append(('pretilt_hold', h)))
    node._toss_safe_abort()
    assert order == [('armed', False), ('retract', hw.JB_OP_HAND_RETRACT_REV),
                     ('arm', False), 'home', ('prime_hold', False),
                     ('pretilt_hold', False)]


def test_toss_terminals_skip_pretilt_release_when_not_raised(monkeypatch):
    """8a (or an 8b goal that never reached PREPARE): ``pretilt_hold_raised``
    stays False, so the terminals NEVER publish on catch/pretilt_hold — the 8a
    terminal publish sequence is byte-identical to Phase 1."""
    node = ReloadCoordinatorNode()
    assert node._toss_committed.pretilt_hold_raised is False
    pretilt = []
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    monkeypatch.setattr(node, '_arm_catch', lambda a: True)
    monkeypatch.setattr(node, '_publish_catch_armed', lambda a: None)
    monkeypatch.setattr(node, '_go_home', lambda: True)
    monkeypatch.setattr(node, '_smooth_move_hand', lambda p: True)
    monkeypatch.setattr(node, '_publish_prime_hold', lambda h: None)
    monkeypatch.setattr(node, '_publish_pretilt_hold', lambda h: pretilt.append(h))
    node._toss_recenter()
    node._toss_safe_abort()
    assert pretilt == []


@pytest.mark.parametrize('live_xy', [(0.0, 0.0), (150.0, 0.0), (-90.0, 40.0)])
def test_8b_throw_site_is_the_live_commanded_pose(monkeypatch, live_xy):
    """Tier 8b routes _execute_toss through compute_release_state_tilted at the
    throw site A = the platform's LIVE commanded xy (Phase E, 2026-07-29) — NOT
    a config site and NOT the 8a compute_release_state.

    Re-pointed from test_8b_uses_tilted_release_with_config_throw_site, which
    pinned A == hw.JB_OP_TOSS_THROW_SITE_MM. That key is retired: a config site
    is a phantom the aim is solved for while POSITIONING obediently translates
    the platform to it, so a chained session's second toss would fly back to
    (0, 0) before throwing. Parametrised over three live poses precisely so the
    live read cannot be satisfied by a constant."""
    calls = {}
    real = rcn.compute_release_state_tilted

    def _spy(catch_pose, flight, *, throw_site_xy_mm):
        calls['pose'] = tuple(catch_pose)
        calls['site'] = tuple(float(v) for v in throw_site_xy_mm)
        return real(catch_pose, flight, throw_site_xy_mm=throw_site_xy_mm)
    monkeypatch.setattr(rcn, 'compute_release_state_tilted', _spy)
    monkeypatch.setattr(rcn, 'compute_release_state',
                        lambda *a, **k: pytest.fail('8a path used for an 8b goal'))
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    node = _toss_ready_node(time.perf_counter(),
                            commanded_pos=(live_xy[0], live_xy[1], 170.0))
    monkeypatch.setattr(hw, 'JB_OP_TOSS_TIER', '8b')
    # B is 50 mm +x of wherever the platform is, so every leg is a legal
    # displaced goal regardless of the live pose.
    gh = _TossGoalHandle(x=live_xy[0] + 50.0, y=live_xy[1], z=170.0,
                         throw_height=0.8, delay=5.0)
    node._execute_toss(gh)               # runs to REJECTED_POSITION (go_to_pose n/a)
    assert calls['pose'] == (live_xy[0] + 50.0, live_xy[1], 170.0)
    assert calls['site'] == live_xy


def test_8b_without_a_fresh_commanded_pose_is_rejected_pose_unknown(monkeypatch):
    """No fresh trajectory/commanded_position ⇒ REJECTED_POSE_UNKNOWN, and the
    release state is never computed.

    Fail-closed is the whole point: the alternative to refusing is guessing A,
    and a guessed A is not merely a wrong number — POSITIONING translates the
    platform to the pre-tilt pose derived from it, so the guess becomes
    commanded motion nobody asked for."""
    monkeypatch.setattr(rcn, 'compute_release_state_tilted',
                        lambda *a, **k: pytest.fail(
                            'no release state may be computed without a site'))
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    node = _toss_ready_node(time.perf_counter())
    with node._lock:
        node._commanded_pos_mm = None
        node._commanded_pos_mono = 0.0
    monkeypatch.setattr(hw, 'JB_OP_TOSS_TIER', '8b')
    gh = _TossGoalHandle(x=50.0, y=0.0, z=170.0, throw_height=0.8, delay=5.0)
    result = node._execute_toss(gh)
    assert result.success is False
    assert result.outcome == 'REJECTED_POSE_UNKNOWN'
    assert gh.terminal == 'abort'


def test_8b_stale_commanded_pose_is_rejected_pose_unknown(monkeypatch):
    """A commanded position older than _TRAJ_STATUS_STALE_S is UNKNOWN, not
    usable: trajectory_node publishes it only while seeded+streaming, so silence
    means "no commanded pose exists" — the exact state where guessing A is
    worst."""
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    now = time.perf_counter()
    node = _toss_ready_node(now)
    with node._lock:
        node._commanded_pos_mono = now - (rcn._TRAJ_STATUS_STALE_S + 1.0)
    monkeypatch.setattr(hw, 'JB_OP_TOSS_TIER', '8b')
    gh = _TossGoalHandle(x=50.0, y=0.0, z=170.0, throw_height=0.8, delay=5.0)
    result = node._execute_toss(gh)
    assert result.outcome == 'REJECTED_POSE_UNKNOWN'


def test_non_finite_commanded_position_is_discarded(monkeypatch):
    """A NaN/inf commanded position is DROPPED, not cached: it would poison the
    throw-site aim and make the reload's centred test read False-or-True by
    accident. The previous good value survives."""
    node = _toss_ready_node(100.0, commanded_pos=(12.0, -5.0, 170.0))
    node._on_commanded_position(
        types.SimpleNamespace(x=float('nan'), y=0.0, z=170.0))
    with node._lock:
        assert node._commanded_pos_mm == (12.0, -5.0, 170.0)


def test_8b_degenerate_b_equals_live_pose_is_a_vertical_toss(monkeypatch):
    """B == the live commanded pose ⇒ zero displacement ⇒ the aim is exactly
    level and the release state is BITWISE the 8a vertical toss.

    This is the operator's "8b subsumes 8a" expectation and the case that has
    NEVER flown on hardware (all 11 validated T4 throws were displaced): under
    the retired config throw site a centred goal from an off-centre platform was
    a >70 mm DISPLACED throw, so the degenerate case was unreachable from
    anywhere but the origin."""
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    node = _toss_ready_node(time.perf_counter(),
                            commanded_pos=(150.0, -150.0, 170.0))
    monkeypatch.setattr(hw, 'JB_OP_TOSS_TIER', '8b')
    captured = {}
    real = rcn.compute_release_state_tilted

    def _spy(catch_pose, flight, *, throw_site_xy_mm):
        rs = real(catch_pose, flight, throw_site_xy_mm=throw_site_xy_mm)
        captured['rs'] = rs
        return rs
    monkeypatch.setattr(rcn, 'compute_release_state_tilted', _spy)
    gh = _TossGoalHandle(x=150.0, y=-150.0, z=170.0, throw_height=0.8, delay=5.0)
    node._execute_toss(gh)
    rs = captured['rs']
    assert rs.displacement_mm == 0.0
    assert (rs.tilt_rx, rs.tilt_ry) == (0.0, 0.0)
    ref = compute_release_state((150.0, -150.0, 170.0), rs.flight_time_s)
    assert np.array_equal(rs.launch_vel_mms, ref.launch_vel_mms)
    assert np.array_equal(rs.release_pos_global_mm, ref.release_pos_global_mm)


def test_8b_tilt_clamp_maps_to_rejected_tilt_clamp(monkeypatch):
    """Tier 8b: compute_release_state_tilted's ThrowTiltInfeasible raise maps
    onto the FSM's tilt_clamp_exceeded flag → REJECTED_TILT_CLAMP (in gate
    order, before EVENT_VEL) — no drift-prone second copy of the aim math in
    the node."""
    def _raise(*a, **k):
        raise ThrowTiltInfeasible(20.0, 12.0)
    monkeypatch.setattr(rcn, 'compute_release_state_tilted', _raise)
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    node = _toss_ready_node(time.perf_counter())
    monkeypatch.setattr(hw, 'JB_OP_TOSS_TIER', '8b')
    gh = _TossGoalHandle(x=50.0, y=0.0, z=170.0, throw_height=0.8, delay=5.0)
    result = node._execute_toss(gh)
    assert result.success is False
    assert base_outcome(result.outcome) == 'REJECTED_TILT_CLAMP'
    # The raise CARRIES its two numbers and the node forwards them, so the
    # refusal says by how much the ceiling was broken rather than only that it
    # was — 12.1° against 12° and 20° against 12° are different situations.
    assert 'aim 20.00 deg > ceiling 12.00 deg [MAX_TILT_DEG]' in result.outcome
    assert gh.terminal == 'abort'


# ══ The cadence fixes the census orders BEFORE rung R3 ═══════════════════════
#
# D3 (the raw bit reaches the live query), D6 (our own ball is not a phantom)
# and the C-POSSESS-1 § 3.4 window clamp, at the NODE seam. The pure-module half
# is in tests/ros/test_ball_possession.py.

def test_the_node_feeds_both_sensor_bits_not_just_the_debounced_one():
    """CENSUS D3 at the wire. ``/hand_telemetry`` has carried ``ball_held_raw``
    since Phase 5 and the node ignored it, so the live ``evidence`` query — the
    ball-evidence PRECONDITION — answered from a bit whose measured fall lag is
    ~241 ms. At the cadence ladder's lower rungs that gate would pass an EMPTY
    cup, which is the fail-OPEN inversion of C-POSSESS-1's whole posture.

    The debounced bit still owns the EDGES: this test drives them apart and
    asserts the live read follows the raw one."""
    node = _toss_ready_node(time.perf_counter())

    def telem(held, raw, valid=True):
        return types.SimpleNamespace(pos_meas=0.0, vel_meas=0.0, ball_held=held,
                                     ball_held_raw=raw, ball_held_valid=valid)

    # The subscription stamps with the node module's own perf clock, so these run
    # on real time — microseconds apart, i.e. well inside the staleness bound.
    node._on_hand_telemetry(telem(True, True))
    node._on_hand_telemetry(telem(True, True))
    assert node._ball_sensor.evidence(time.perf_counter()) == 'SEATED'
    # The ball leaves: raw falls at once, the debounced verdict lags ~241 ms.
    node._on_hand_telemetry(telem(True, False))
    assert node._ball_sensor.evidence(time.perf_counter()) == 'EMPTY'
    assert node._ball_sensor.evidence_settled(time.perf_counter()) == 'UNKNOWN'


def test_a_message_without_the_raw_bit_degrades_to_the_debounced_one():
    """A bag cut before Phase 5, or an older interface build, has no
    ``ball_held_raw``. The node passes None — never a defaulted False, which
    would make a missing FIELD indistinguishable from an empty CUP."""
    node = _toss_ready_node(time.perf_counter())
    msg = types.SimpleNamespace(pos_meas=0.0, vel_meas=0.0, ball_held=True,
                                ball_held_valid=True)      # no ball_held_raw
    node._on_hand_telemetry(msg)
    node._on_hand_telemetry(msg)
    assert node._ball_sensor.evidence(time.perf_counter()) == 'SEATED'


def test_our_own_previous_cycles_ball_is_not_a_phantom_track():
    """CENSUS D6 — the real bug, and the docstring in
    ``_build_toss_observations`` already claimed the fix ("NOT the seated ball's
    own pruned/CAUGHT track").

    ``track_active`` refuses a live destination='jugglebot' track at CHECKING,
    because a phantom would correlate against OUR announcement. The ball this
    session just caught is not a phantom: its track only leaves IN_FLIGHT when
    the tracker mints CAUGHT, at landing +0.202..+0.442 s. At any dwell short
    enough for CHECKING to run inside that window the gate hard-rejects a
    healthy cycle — and it does so from an angle completely independent of the
    handoff floor, so lowering the dwell alone cannot dodge it."""
    now = 100.0
    node = _toss_ready_node(now)
    ours = _Ball(status=1, destination='jugglebot', id=9)   # still IN_FLIGHT
    with node._lock:
        node._balls = [ours]
        node._balls_mono = now
        node._prev_announced_ball_id = None
    assert node._build_toss_observations(now).track_active is True   # the defect
    with node._lock:
        node._prev_announced_ball_id = 9        # ... it is OUR previous cycle's
    assert node._build_toss_observations(now).track_active is False
    # A DIFFERENT live track is still a phantom and still refuses — the exclusion
    # is one id, not a disabled gate.
    with node._lock:
        node._balls = [ours, _Ball(status=1, destination='jugglebot', id=11)]
    assert node._build_toss_observations(now).track_active is True


def test_the_own_ball_exclusion_spans_cycles_but_never_goals():
    """The latch is rolled forward at cycle start and cleared at goal ACCEPT. A
    stale id surviving into a later goal would silently un-guard one track for a
    session that never threw it."""
    now = 100.0
    node = _toss_ready_node(now)
    with node._lock:
        node._announced_ball_id = 9
    node._build_toss_cycle((0.0, 0.0, 170.0), 0.8, 5.0, 0.0)
    with node._lock:
        assert node._prev_announced_ball_id == 9   # rolled forward for cycle N+1
        assert node._announced_ball_id is None     # and this cycle starts clean
        node._goal_claimed = False
        node._active_seq = None
    node._goal_callback(types.SimpleNamespace())
    with node._lock:
        assert node._prev_announced_ball_id is None


def test_a_single_toss_leaves_the_sensor_windows_unclamped():
    """C-POSSESS-1 § 3.4 defaults. A single ``Toss`` has no next scheduled
    release, so the honest horizon is the shipped fixed window and the behaviour
    is the pre-2026-08-21 behaviour, byte for byte."""
    now = 100.0
    node = _toss_ready_node(now)
    _install_toss_goal(node)
    assert node._expected_next_cycle_perf() == (None, None)


def test_a_session_cycle_clamps_the_windows_to_its_own_next_release():
    """... and a SESSION cycle carries the clamp, derived from the session's own
    dwell rather than from a constant. The last intended cycle is deliberately
    left unclamped: there is no next release for a departure to belong to, and
    clamping one cycle too FEW is the cheap error (§ 3.4's sized residual)."""
    import jugglebot.toss_session as ts
    now = 100.0
    node = _toss_ready_node(now)
    session = ts.TossSessionSequencer(num_throws=3, dwell_time_s=1.5,
                                      throw_delay_s=5.0)
    session.start(now)
    seq, _ = node._build_toss_cycle((0.0, 0.0, 170.0), 0.8, 5.0, 0.0)
    node._set_toss_next_cycle_perf(seq, session)
    landing = seq.t_release + seq.flight_time_s
    rel, land = node._expected_next_cycle_perf()
    assert rel == pytest.approx(landing + 1.5)
    assert land == pytest.approx(landing + 1.5 + 0.8)
    # Tearing the cycle down drops the schedule — a horizon must never outlive
    # the cycle that owns it (the arm/disarm lifecycle bug class).
    node._clear_toss_cycle_state()
    assert node._expected_next_cycle_perf() == (None, None)


def test_the_cadence_clamp_reads_the_sessions_beat_rather_than_copying_it():
    """…and it reads it from ``TossSessionSequencer.next_release_at``, the
    session's single beat derivation, instead of holding a second copy of
    ``landing + dwell``.

    Pinned by substitution: move the beat and the clamp must move with it. The
    two numbers must name the SAME instant, because the clamp is what closes the
    hand sensor's retention window "where the next toss's departure search
    opens" (C-POSSESS-1 § 3.4) — a clamp sized off a beat the session does not
    run on closes that window against a release that never comes. Phase C moves
    the beat by replacing that one method body (plan § 2.6), and this is the
    node-side half of the promise that nothing else has to be told."""
    import jugglebot.toss_session as ts
    now = 100.0
    node = _toss_ready_node(now)
    session = ts.TossSessionSequencer(num_throws=3, dwell_time_s=1.5,
                                      throw_delay_s=5.0)
    session.start(now)
    session.next_release_at = lambda landing: float(landing) + 9.0
    seq, _ = node._build_toss_cycle((0.0, 0.0, 170.0), 0.8, 5.0, 0.0)
    node._set_toss_next_cycle_perf(seq, session)
    landing = seq.t_release + seq.flight_time_s
    rel, land = node._expected_next_cycle_perf()
    assert rel == pytest.approx(landing + 9.0)          # the beat, not the dwell
    assert land == pytest.approx(landing + 9.0 + 0.8)


def test_the_cycle_builder_passes_an_absolute_release_straight_through():
    """``_build_toss_cycle``'s ``release_at_perf`` reaches the FSM unmodified,
    and its 0.0 default is the pre-B2 arithmetic bit for bit.

    The cycle builder is shared verbatim by the single ``Toss`` and by every
    session cycle, so this parameter is the ONE place a beat clock enters the
    cycle machinery — and the builder stays ignorant of where the beat came
    from (plan § 2.6). Both shipped call sites take the default today; B4's
    pipeline is what supplies a number."""
    node = _toss_ready_node(100.0)
    # The builder starts the FSM off the real perf clock, so the derived twin is
    # pinned by its LEAD (accept -> release) rather than by an absolute instant.
    derived, _ = node._build_toss_cycle((0.0, 0.0, 170.0), 0.8, 5.0, 0.0)
    assert derived.release_at_perf == 0.0
    assert derived.scheduled_lead_s == pytest.approx(5.0)
    beat = time.perf_counter() + 7.5
    scheduled, _ = node._build_toss_cycle((0.0, 0.0, 170.0), 0.8, 5.0, 0.0,
                                          release_at_perf=beat)
    assert scheduled.release_at_perf == beat
    assert scheduled.t_release == beat                   # the input, verbatim
    assert scheduled.landing_perf == pytest.approx(beat + 0.8)
    # The lead is now the SCHEDULE's, not the delay field's — the number the
    # per-goal ceiling has to be sized on.
    assert scheduled.scheduled_lead_s == pytest.approx(7.5, abs=0.5)


def test_the_toss_ceiling_covers_an_absolutely_scheduled_release():
    """The per-goal ceiling is sized on the lead the cycle ACTUALLY has
    (``scheduled_lead_s``), not on ``throw_delay_s``.

    Same doctrine as the sibling test above, applied to the schedule the release
    became an input to: size the ceiling off the delay while the cycle runs on a
    release 25 s out, and the ceiling lands INSIDE a legitimate window. The exit
    path there is a SAFE_ABORT — hand retracted, latch dropped, go_home — for a
    goal doing exactly what it was asked to do, and under an airborne ball the
    retract happens beneath it."""
    from jugglebot.reload_coordinator_node import _MAX_SEQUENCE_S
    now = 100.0
    seq = TossSequencer(catch_pose_stow_mm=(0, 0, 170.0), flight_time_s=1.1,
                        throw_delay_s=5.0, release_at_perf=now + 25.0)
    seq.start(now)
    assert seq.scheduled_lead_s == pytest.approx(25.0)
    ceiling = _toss_deadline_s(seq)
    assert ceiling > 25.0 + 1.1 + 0.5 + 0.7
    assert ceiling >= _MAX_SEQUENCE_S
    # …and the derived path is unchanged: an UNSTARTED sequencer (the throwaway
    # one the session ceiling budgets from) still answers off the delay.
    unstarted = TossSequencer(catch_pose_stow_mm=(0, 0, 170.0),
                              flight_time_s=1.1, throw_delay_s=25.0)
    assert unstarted.scheduled_lead_s == pytest.approx(25.0)
    assert _toss_deadline_s(unstarted) == pytest.approx(ceiling)


#: Phase B1's field inventory: the node attribute that WAS, and the
#: :class:`TossCycleState` field it became. Kept as data rather than as prose so
#: the structural test below can enumerate it — a field that quietly comes back
#: to the node is a field two coexisting cycles would silently share.
_MOVED_ONTO_THE_CYCLE_OBJECT = {
    '_toss_release_state': 'release_state',
    '_toss_release_cmd': 'release_cmd',
    '_toss_aim': 'aim',
    '_toss_landing_global_mm': 'landing_global_mm',
    '_toss_platform_target_mm': 'platform_target_mm',
    '_toss_positioning_move': 'positioning_move',
    '_toss_waiver': 'waiver',
    '_toss_prepare_pending': 'prepare_pending',
    '_toss_throw_dispatched': 'throw_dispatched',
    '_toss_stroke_seen': 'stroke_seen',
    '_toss_track_confirmed': 'track_confirmed',
    '_toss_pretilt_hold_raised': 'pretilt_hold_raised',
    '_toss_announced_reach': 'announced_reach',
    '_toss_next_release_perf': 'next_release_perf',
    '_toss_next_landing_perf': 'next_landing_perf',
    '_toss_record_announce': 'record_announce',
}

#: The node attributes that DELIBERATELY did not move, each with the reason it
#: cannot: the first four span cycles by design, and the last three are shared
#: verbatim with the RELOAD path, which has no toss cycle to carry them.
_STAYS_ON_THE_NODE = (
    '_prev_announced_ball_id',      # census D6 — spans cycles by design
    '_toss_prev_landing_perf',      # the arrival boundary's cross-cycle latch,
    '_toss_cycle_landing_perf',     #   reset per SESSION, never per cycle
    '_ball_possession',             # the latch survives across cycles
    '_announced_ball_id',           # shared with the reload observation builder
    '_preexisting_flight_ids',      #   ... and written by the reload paths
    '_catch_vel_scale',             # a ball-op-wide knob, not a toss one
)


def test_per_cycle_state_lives_on_the_cycle_object_not_the_node():
    """Phase B1's whole point, made mechanical.

    Every field in ``_MOVED_ONTO_THE_CYCLE_OBJECT`` used to be a node global
    written once by ``_build_toss_cycle``. Two coexisting cycles — the two-slot
    pipeline this phase clears the way for — would silently SHARE every one of
    them: cycle k+1's build would overwrite the release state cycle k is still
    flying on, and the sticky release-evidence latches would cross-contaminate
    in both directions. So the invariant is structural rather than a matter of
    discipline: after a cycle has been built AND after it has been torn down, no
    per-cycle name may exist on the node at all.

    ``hasattr`` is the mechanism because it catches the failure that matters —
    an assignment somewhere re-creating the attribute — which a class-dict
    inspection would miss (these were always instance attributes) and which
    checking only the dataclass would miss too."""
    now = 100.0
    node = _toss_ready_node(now)
    fields = {f.name for f in dataclasses.fields(rcn.TossCycleState)}

    # Before any cycle: the node carries an EMPTY committed slot, never None, so
    # nothing downstream needs a None branch it did not need before.
    assert isinstance(node._toss_committed, rcn.TossCycleState)

    seq, state = node._build_toss_cycle((0.0, 0.0, 170.0), 0.8, 5.0, 0.0)
    assert isinstance(state, rcn.TossCycleState)
    assert node._toss_committed is state       # ... and it IS the committed slot
    assert seq is node._active_seq

    for old, new in sorted(_MOVED_ONTO_THE_CYCLE_OBJECT.items()):
        assert new in fields, f'{old} moved to a field TossCycleState lacks'
        assert not hasattr(node, old), (
            f'{old} is back on the node — two live cycles would share it')

    node._clear_toss_cycle_state()
    for old in sorted(_MOVED_ONTO_THE_CYCLE_OBJECT):
        assert not hasattr(node, old), f'{old} reappeared at teardown'

    # The exclusions are as deliberate as the moves: each of these is read
    # ACROSS a cycle boundary (or by the reload path, which has no cycle), so a
    # per-cycle home would delete exactly the number its next reader needs.
    for kept in _STAYS_ON_THE_NODE:
        assert hasattr(node, kept), f'{kept} must stay on the node'


def test_the_previous_cycles_landing_is_the_number_that_cycle_judged_against():
    """C-POSSESS-1 § 3.4 clause C.1's other boundary end (2026-08-23).

    Cycle N closes its arrival window at ``arrival_boundary_t(prev, N)`` and
    cycle N+1 must OPEN at the identical call, or the two stop abutting and one
    seat edge can be claimed twice. That identity survives only if N+1 is handed
    the number N was judged against — ``seq.landing_perf`` — rather than a second
    derivation of it, which is why the latch is rolled here and not recomputed at
    the query. The FIRST cycle has no predecessor and must read ``None``: the
    boundary can only move an opening LATER, so its absence is the shipped
    window, never a widened one."""
    import jugglebot.toss_session as ts
    now = 100.0
    node = _toss_ready_node(now)
    session = ts.TossSessionSequencer(num_throws=3, dwell_time_s=1.5,
                                      throw_delay_s=5.0)
    session.start(now)

    assert node._expected_prev_landing_perf() is None       # nothing landed yet
    first, _ = node._build_toss_cycle((0.0, 0.0, 170.0), 0.8, 5.0, 0.0)
    node._set_toss_next_cycle_perf(first, session)
    assert node._expected_prev_landing_perf() is None       # ... still the first
    first_landing = first.landing_perf

    second, _ = node._build_toss_cycle((0.0, 0.0, 170.0), 0.8, 5.0, 0.0)
    node._set_toss_next_cycle_perf(second, session)
    # The IDENTITY, not merely the value: `landing_perf` is what `observe` was
    # given as `landing_t` for cycle 1.
    assert node._expected_prev_landing_perf() == pytest.approx(first_landing)
    # Tearing the CYCLE down must NOT drop it — the boundary spans cycles, which
    # is the whole difference between this latch and the next-release pair above.
    node._clear_toss_cycle_state()
    assert node._expected_prev_landing_perf() == pytest.approx(first_landing)
    # A landing carried in from a previous GOAL would clamp the next session's
    # first window against an instant minutes old, so the per-SESSION reset drops
    # it — and that reset is a named method precisely so this lifecycle is one
    # grep rather than two assignments buried in a goal handler.
    node._reset_toss_arrival_boundary()
    assert node._expected_prev_landing_perf() is None
    third, _ = node._build_toss_cycle((0.0, 0.0, 170.0), 0.8, 5.0, 0.0)
    node._set_toss_next_cycle_perf(third, session)
    assert node._expected_prev_landing_perf() is None, (
        'a reset session starts its first cycle with no predecessor')


def test_the_last_intended_cycle_is_not_clamped():
    """`intends_another_cycle` is keyed on THROWS, like the session's own
    completion test, so a REJECTED_NO_BALL cycle does not consume one."""
    import jugglebot.toss_session as ts
    session = ts.TossSessionSequencer(num_throws=1, dwell_time_s=1.5,
                                      throw_delay_s=5.0)
    session.start(100.0)
    assert session.intends_another_cycle is False
    more = ts.TossSessionSequencer(num_throws=2, dwell_time_s=1.5,
                                   throw_delay_s=5.0)
    more.start(100.0)
    assert more.intends_another_cycle is True
    now = 100.0
    node = _toss_ready_node(now)
    seq, _ = node._build_toss_cycle((0.0, 0.0, 170.0), 0.8, 5.0, 0.0)
    node._set_toss_next_cycle_perf(seq, session)
    assert node._expected_next_cycle_perf() == (None, None)


# ══ The per-cycle budget fixes (census B1, B6, E5) ═══════════════════════════
#
# B1 skips a move that traverses zero millimetres; B6 gets the blocking I/O off
# the cycle thread; E5 stops CCN commanding the platform during a toss at all.
# All three are cadence work, and all three are ways the machine does LESS —
# which is why each one's test is mostly about the conditions under which it
# must decline to.

def test_a_colocated_chain_skips_the_no_op_positioning_move():
    """CENSUS B1. After a CAUGHT toss with ``stay_at_pose_on_caught`` the
    platform is ALREADY at B — the terminal hold never let it go — so cycle N+1's
    ``go_to_pose`` is a 0 mm move costing ``min_move_duration_s`` (0.20) +
    ``TOSS_POSITION_SETTLE_PAD_S`` (0.20) + a service round trip, every cycle, out
    of a turnaround whose whole physical floor is 0.49 s.

    The skip is asserted at BOTH ends: no service call is made, and the FSM is
    told it has arrived NOW rather than after a settle pad that pads nothing."""
    now = time.perf_counter()
    node = _toss_ready_node(now, commanded_pos=(0.0, 0.0, 170.0))
    calls = []
    node._go_to_pose_cli.wait_for_service = lambda timeout_sec=None: (
        calls.append('service') or True)
    seq, _ = node._build_toss_cycle((0.0, 0.0, 170.0), 0.8, 5.0, 0.0)
    seq._position_dispatched = True
    node._position_platform_for_toss(seq)
    assert calls == []                                  # nothing was commanded
    accepted, planned, code = seq._position_result
    assert accepted is True and planned == 0.0 and code == 'ALREADY_THERE'
    assert seq._positioned is True
    # No settle pad: there is no plan to grant status granularity to and no
    # terminal hold to enter — the platform is already in one.
    assert seq._position_arrival_time <= time.perf_counter()


def test_the_positioning_decision_is_taken_once_and_reused():
    """**ONE decision, one place** (2026-08-23). The B1 predicate is now TWO
    things at once — the branch POSITIONING takes, and the pre-dispatch budget
    the cycle's CHECKING delay gate charges — so it is evaluated in
    ``_build_toss_cycle`` and CACHED, never re-derived a tick later.

    Re-deriving it is the failure this closes: the commanded pose is republished
    at 5 Hz and the platform can be commanded by other nodes between the two
    evaluations, so a second read could take the cheap branch under an expensive
    budget (harmless) or the expensive branch under a cheap one — a cycle that
    was ACCEPTED against the 0.080 s sequence and then spends 0.460 s of its
    lead, i.e. exactly the accept-vs-runtime gap the 2026-08-22 audit found.

    Driven by MOVING the platform between the two calls: the cached answer must
    win."""
    now = time.perf_counter()
    node = _toss_ready_node(now, commanded_pos=(0.0, 0.0, 170.0))
    calls = []
    node._go_to_pose_cli.wait_for_service = lambda timeout_sec=None: (
        calls.append('service') or True)
    seq, _ = node._build_toss_cycle((0.0, 0.0, 170.0), 0.8, 5.0, 0.0)
    assert seq.positioning_move_expected is False        # the skip is expected
    with node._lock:
        assert node._toss_committed.positioning_move is False
    # The platform moves after the decision was taken. A re-derivation here
    # would command the move under a budget that assumed the skip.
    with node._lock:
        node._commanded_pose = (500.0, 0.0, 170.0, 0.0, 0.0, 0.0)
    seq._position_dispatched = True
    node._position_platform_for_toss(seq)
    assert calls == []
    assert seq._position_result[2] == 'ALREADY_THERE'


def test_a_cycle_that_must_move_charges_the_moving_budget_at_CHECKING(monkeypatch):
    """The cycle FSM is told which sequence it will run, and its delay gate
    charges that sequence — the loud+early half of the fix.

    A cycle that cannot prove the platform is already positioned pays
    ``min_move_duration_s + settle pad + ticks``; before 2026-08-23 the gate
    charged the kind-0 dispatch budget alone and the shortfall surfaced as
    ``ABORTED_CANT_MAKE_RELEASE`` at ``cycle_start + 0.06 s``, with the hand
    retracting under a seated ball."""
    # Tier 8a, pinned: an 8b throw sites A at the LIVE commanded pose, so
    # displacing the platform to make the skip decline would also change the aim
    # and with it the release speed — and the delay floor is a function of that
    # speed. 8a keeps the two cycles' dispatch budgets identical so the only
    # thing this measures is the pre-dispatch sequence.
    monkeypatch.setattr(hw, 'JB_OP_TOSS_TIER', '8a')
    now = time.perf_counter()
    node = _toss_ready_node(now, commanded_pos=(0.0, 60.0, 170.0))
    seq, _ = node._build_toss_cycle((0.0, 0.0, 170.0), 0.8, 5.0, 0.0)
    assert seq.positioning_move_expected is True
    assert seq.min_throw_delay_for_cycle_s == pytest.approx(
        seq.min_event_delay_for_throw_s + pre_dispatch_budget_s(True)
        + FLOOR_REPRESENTATION_SLACK_S, abs=1e-12)
    # …and the same goal from the pose it throws from charges the skip budget.
    here = _toss_ready_node(now, commanded_pos=(0.0, 0.0, 170.0))
    chained, _ = here._build_toss_cycle((0.0, 0.0, 170.0), 0.8, 5.0, 0.0)
    assert chained.min_throw_delay_for_cycle_s == pytest.approx(
        chained.min_event_delay_for_throw_s + pre_dispatch_budget_s(False)
        + FLOOR_REPRESENTATION_SLACK_S, abs=1e-12)
    assert (seq.min_throw_delay_for_cycle_s
            - chained.min_throw_delay_for_cycle_s) == pytest.approx(
                pre_dispatch_budget_s(True) - pre_dispatch_budget_s(False))
    # 0.38 s until 2026-08-26; the D3 unit change (sleep -> measured loop period)
    # re-quantises the go_to_pose arrival wait onto a coarser grid, so the SPLIT
    # narrows to 0.36 s even though both budgets grew.
    assert (pre_dispatch_budget_s(True)
            - pre_dispatch_budget_s(False)) == pytest.approx(0.36)


def test_a_session_cycle_that_must_move_is_GRANTED_the_lead_a_single_toss_is_refused(
        monkeypatch):
    """The one thing the two callers do differently, and why.

    A single ``Toss``'s ``throw_delay`` is an APPOINTMENT the operator set: too
    short for the sequence this cycle must run is a loud
    ``REJECTED_CANT_MAKE_LEAD``, because stretching it answers a request nobody
    made. A ``TossContinuous`` cycle's delay is a CADENCE parameter the session's
    own scheduler consumes (``next_at = landing + dwell − throw_delay``), and the
    session's dwell floor is sized on the CHAINED steady state — so the one cycle
    that must first command its pre-positioning move is granted the extra lead
    instead of killing the sitting on cycle 1.

    Without the grant, every ILC-primary sitting dies on its first cycle: the
    platform is level at goal accept, the aim makes the release tilted, and the
    ladder's delay is the chained one."""
    monkeypatch.setattr(hw, 'JB_OP_TOSS_TIER', '8a')
    now = time.perf_counter()
    delay = 0.45
    away = _toss_ready_node(now, commanded_pos=(0.0, 60.0, 170.0))
    single, _ = away._build_toss_cycle((0.0, 0.0, 170.0), 0.8, delay, 0.0)
    assert single.throw_delay_s == pytest.approx(delay)     # untouched
    now2 = time.perf_counter()
    single.start(now2)
    d = single.step(now2, away._build_toss_observations(now2))
    assert d.done and d.result.outcome.startswith('REJECTED_CANT_MAKE_LEAD')

    away2 = _toss_ready_node(now, commanded_pos=(0.0, 60.0, 170.0))
    chained, _ = away2._build_toss_cycle((0.0, 0.0, 170.0), 0.8, delay, 0.0,
                                      delay_is_cadence=True)
    assert chained.throw_delay_s > delay
    assert chained.throw_delay_s == pytest.approx(
        chained.min_throw_delay_for_cycle_s)

    # A session cycle that DOES take the skip keeps the operator's number
    # exactly — the grant is for the move, not a blanket raise.
    here = _toss_ready_node(now, commanded_pos=(0.0, 0.0, 170.0))
    kept, _ = here._build_toss_cycle((0.0, 0.0, 170.0), 0.8, delay, 0.0,
                                  delay_is_cadence=True)
    assert kept.throw_delay_s == pytest.approx(delay)


@pytest.mark.parametrize('delay', [0.0, -1.0])
def test_the_lead_grant_never_launders_an_unset_or_negative_delay(delay,
                                                                 monkeypatch):
    """0.0 is the goal's "unset" sentinel (``__post_init__`` substitutes the
    5.0 s default) and a negative delay is the sign typo ``__post_init__``
    deliberately PRESERVES so CHECKING rejects it loudly. Raising either would
    turn an operator error into a number the machine chose."""
    monkeypatch.setattr(hw, 'JB_OP_TOSS_TIER', '8a')
    now = time.perf_counter()
    node = _toss_ready_node(now, commanded_pos=(0.0, 60.0, 170.0))
    seq, _ = node._build_toss_cycle((0.0, 0.0, 170.0), 0.8, delay, 0.0,
                                 delay_is_cadence=True)
    expected = (DEFAULT_TOSS_THROW_DELAY_S if delay == 0.0 else delay)
    assert seq.throw_delay_s == pytest.approx(expected)


@pytest.mark.parametrize('commanded,why', [
    ((0.0, 0.0, 170.0 + rcn._TOSS_ALREADY_THERE_TOL_MM + 1.0), 'z is off'),
    ((rcn._TOSS_ALREADY_THERE_TOL_MM + 1.0, 0.0, 170.0), 'x is off'),
    ((0.0, rcn._TOSS_ALREADY_THERE_TOL_MM + 1.0, 170.0), 'y is off'),
    (None, 'the live commanded pose is UNKNOWN'),
])
def test_the_no_op_skip_declines_whenever_it_cannot_prove_the_pose(commanded, why):
    """The skip's failure mode is a throw fired from a pose nobody solved for, so
    every uncertainty resolves to "command the move".

    ``None`` is the load-bearing row: an absent or stale
    ``trajectory/commanded_pose`` reads as NOT-there, never as centre — the
    same fail-closed doctrine as ``throw_site_known``, where a node that was
    never told is not entitled to assume. It is also the version-skew case: a
    trajectory_node that predates the topic leaves the skip permanently off,
    which costs cadence and nothing else."""
    now = time.perf_counter()
    node = _toss_ready_node(now)
    with node._lock:
        node._commanded_pose = (tuple(commanded) + (0.0, 0.0, 0.0)
                                if commanded is not None else None)
        node._commanded_pose_mono = now if commanded is not None else 0.0
    release = node._toss_commanded_release() if node._toss_record_ctx else None
    assert node._toss_already_positioned(0.0, 0.0, 170.0, release) is False, why


def test_a_stale_commanded_pose_is_not_a_matching_one():
    """Freshness is a separate axis from agreement. A pose that MATCHES but is
    older than the 5 Hz channel's staleness window is a pose from a machine that
    may since have moved, and reading it as "already there" is how a stopped
    publisher becomes a silent aim error."""
    now = time.perf_counter()
    node = _toss_ready_node(now)
    with node._lock:
        node._commanded_pose = (0.0, 0.0, 170.0, 0.0, 0.0, 0.0)
        node._commanded_pose_mono = now - rcn._TRAJ_STATUS_STALE_S - 1.0
    assert node._toss_already_positioned(0.0, 0.0, 170.0, None) is False


def test_a_tilted_release_skips_only_when_the_platform_HOLDS_that_tilt():
    """**The census-B1 skip on an AIMED chain** (2026-08-23).

    Until then this asserted the opposite — ``a tilted release ALWAYS commands
    the move`` — because ``trajectory/commanded_position`` carried position and
    nothing else, so a pre-tilt pose had an orientation the node could not
    verify. Correct, and it cost 0.38 s of throw delay on EVERY cycle of every
    aimed sitting, which is every ILC-primary sitting there is.

    The fix published the orientation rather than weakening the check, so the
    check got STRICTER, not looser. Three rows, and the middle one is the whole
    point:

    * platform LEVEL, release TILTED  ⇒ move (the aim is not applied yet);
    * platform AT the tilt, release TILTED ⇒ **skip** — the chained cycle;
    * platform at a DIFFERENT tilt ⇒ move, even a tilt one ILC step away.

    The third row is what stops the skip from laundering a stale aim: an ILC
    correction that changed between sittings must be commanded, and the measured
    per-cell |aim| is 9.1-10.5 mrad against a 2.7 mrad tolerance, so it always
    is.

    ``AIM_RAD`` is that measured per-cell magnitude, not a round number — a test
    driven at a tilt UNDER the tolerance would assert the opposite of what it
    reads (a sub-tolerance aim is deliberately admitted; see the last block)."""
    now = time.perf_counter()
    AIM_RAD = 0.010

    class _Tilted:
        tilt_rx, tilt_ry = AIM_RAD, 0.0
        pretilt_pose_stow = (0.0, 0.0, 170.0, 0.0, 0.0, 0.0)

    assert AIM_RAD > 3.0 * rcn._TOSS_ALREADY_THERE_TOL_RAD

    level = _toss_ready_node(now)
    assert level._release_is_tilted(_Tilted()) is True
    assert level._toss_already_positioned(0.0, 0.0, 170.0, _Tilted()) is False

    holding = _toss_ready_node(now, commanded_rotvec=(AIM_RAD, 0.0, 0.0))
    assert holding._toss_already_positioned(0.0, 0.0, 170.0, _Tilted()) is True
    # …and a LEVEL release from that same tilted platform is refused, which is
    # the symmetric half: the old check would have skipped it on position alone.
    assert holding._toss_already_positioned(0.0, 0.0, 170.0, None) is False

    other = _toss_ready_node(
        now, commanded_rotvec=(AIM_RAD + 2 * rcn._TOSS_ALREADY_THERE_TOL_RAD,
                               0.0, 0.0))
    assert other._toss_already_positioned(0.0, 0.0, 170.0, _Tilted()) is False

    # A sub-tolerance aim IS admitted, deliberately and symmetrically with the
    # position half: the landing drift it costs is inside the same budget the
    # position tolerance already spends, so refusing it would buy nothing and
    # cost 0.38 s. Asserted so a future reader does not read the tolerance as a
    # bug.
    class _Tiny:
        tilt_rx = rcn._TOSS_ALREADY_THERE_TOL_RAD / 2.0
        tilt_ry = 0.0
        pretilt_pose_stow = (0.0, 0.0, 170.0, 0.0, 0.0, 0.0)

    assert level._toss_already_positioned(0.0, 0.0, 170.0, _Tiny()) is True


def test_the_angular_tolerance_is_derived_from_the_position_one():
    """Both halves of the skip admit the SAME physical error.

    A residual platform tilt δθ tilts the release velocity by δθ and displaces
    the landing by ``4·h·sin(δθ)`` — the identical expression the CHECKING
    levelling gate is sized on. Setting that equal to the position tolerance at
    the LARGEST apex the C-HAND-3 band admits (fail-closed: the drift is linear
    in apex) is where the angular bound comes from. Pinned as a derivation, not
    as a literal, so re-deriving the cup radius or the flight band moves it.

    And pinned against the ILC's own aim authority, because that ratio is what
    makes the skip safe on an aimed chain: an armed ±1.0° aim can never be
    mistaken for a level platform."""
    apex_max = rcn.apex_height_from_flight_time(
        float(rcn.TOSS_FLIGHT_TIME_MAX_S))
    assert rcn._TOSS_ALREADY_THERE_TOL_RAD == pytest.approx(
        (rcn._TOSS_ALREADY_THERE_TOL_MM / 1000.0) / (4.0 * apex_max))
    # The landing drift it admits IS the position budget, at the worst apex.
    drift_mm = 4.0 * apex_max * rcn._TOSS_ALREADY_THERE_TOL_RAD * 1000.0
    assert drift_mm == pytest.approx(rcn._TOSS_ALREADY_THERE_TOL_MM)
    # …and it is a small fraction of the aim authority it must discriminate.
    assert rcn._TOSS_ALREADY_THERE_TOL_RAD < math.radians(1.0) / 5.0


def test_the_tolerance_is_a_fraction_of_the_cup_not_of_the_workspace():
    """The bound is HALF the cup radius, and the choice is load-bearing.

    For a co-located Tier-8a toss a positioning residual is aim-NEUTRAL — throw
    site == catch site, so the ball comes back down into the cup wherever the cup
    is. What it is NOT neutral about is everything told the ball will be at the
    NOMINATED B: the declared reach centre, the announcement the tracker
    correlates against, the mocap cross-check target. Reusing the recentre
    tolerance (66.53 mm, an order of magnitude looser) would let the skip fire on
    a platform two cup-radii off B."""
    assert rcn._TOSS_ALREADY_THERE_TOL_MM == pytest.approx(
        float(hw.GEOM_HAND_RADIUS_MM) / 2.0)
    assert rcn._TOSS_ALREADY_THERE_TOL_MM < rcn._RELOAD_CENTERED_TOL_MM
    assert rcn._TOSS_ALREADY_THERE_TOL_MM < rcn._TOSS_POSITION_TOL_MM


def test_pretilt_hold_is_raised_on_every_cycle_including_a_level_one():
    """CENSUS E5. The hold was 8b-only, then any-commanded-tilt, and is now
    unconditional.

    CCN's pre-tilt arrival is ``min(landing, max(landing - 1.5, now + 1.0))``. At
    the cadence rungs ``landing - 1.5`` is in the PAST, so the max() picks
    ``now + 1.0`` and the min() clamps it to the landing itself — an arrival
    scheduled AT CONTACT, the exact degeneracy ``_pretilt_arrival_perf``'s own
    docstring exists to avoid. For a level 8a the target is
    ``predicted_catch_command``'s output rather than literally the held pose, so
    any receive-tilt or landing-prediction residual becomes a commanded reach
    arriving at contact — and at a sub-second dwell the NEXT cycle's announcement
    lands inside the previous ball's settle-hold window, where any platform
    motion is a dropped catch."""
    from jugglebot.toss_sequencer import ACTION_PREPARE_CATCH, TossDecision
    now = 100.0
    node = _toss_ready_node(now)
    seq, _ = node._build_toss_cycle((0.0, 0.0, 170.0), 0.8, 5.0, 0.0)
    assert node._release_is_tilted(node._toss_commanded_release()) is False
    node._build_toss_observations = lambda _now, _state=None: None
    seq.step = lambda _now, _obs: TossDecision(
        done=False, phase='PREPARING', action=ACTION_PREPARE_CATCH, result=None)
    node._step_toss_sequence(seq, now)
    published = [m.data for m in node._publishers['catch/pretilt_hold'].published]
    assert published == [True]
    assert node._toss_committed.pretilt_hold_raised is True


def test_the_unconditional_pretilt_hold_is_still_released_by_every_teardown():
    """Raising it always is only safe because every terminal lowers it. The flag
    is what makes that true, and it is keyed the same way for all three ladders
    (STAY, RECENTER, SAFE_ABORT) — so a hold raised for the cadence reason is
    released by exactly the code that released one raised for the aim reason."""
    now = 100.0
    for teardown in ('_toss_stay', '_toss_recenter', '_toss_safe_abort'):
        node = _toss_ready_node(now)
        node._toss_committed.pretilt_hold_raised = True
        getattr(node, teardown)()
        published = [m.data for m
                     in node._publishers['catch/pretilt_hold'].published]
        assert published[-1] is False, teardown


def test_the_belt_write_and_trim_update_leave_the_cycle_thread():
    """CENSUS B6. Both ran synchronously inside ``_run_toss_cycle`` before it
    returned — squarely in the landing -> next-cycle handoff — and both can BLOCK
    (an ``open(..., 'a')`` on the SD card, a numpy estimator update). A full disk
    was a cadence fault; now it is a lost measurement.

    The ROS publish deliberately STAYS on the cycle thread: it is a non-blocking
    hand-off to the middleware and it is the canonical sink."""
    now = 100.0
    node = _toss_ready_node(now)
    ran_on = []
    node._belt_toss_record = lambda payload: ran_on.append(
        threading.current_thread().name)
    node._open_toss_record(action='toss', goal_id='g', cycle_index=1,
                           catch_pose=(0.0, 0.0, 170.0), throw_delay=5.0,
                           vel_scale=0.9, raw_goal={}, flight=0.8)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    assert node._toss_records_drain()
    assert ran_on == ['toss_records']


def test_the_trim_context_is_snapshotted_at_submit_not_at_execute():
    """The one real hazard in moving the trim update off-thread, closed by
    construction rather than by timing.

    ``_toss_trim_observe`` reads the cycle's pose, flight and aim.
    ``_build_toss_cycle`` overwrites all three for the NEXT cycle. A worker that
    read them at execute time would therefore attribute cycle N's toss to cycle
    N+1's pose and aim — a silently wrong corpus, which is the failure class the
    whole record layer exists to avoid."""
    now = 100.0
    node = _toss_ready_node(now)
    node._open_toss_record(action='toss', goal_id='g', cycle_index=1,
                           catch_pose=(11.0, 22.0, 170.0), throw_delay=5.0,
                           vel_scale=0.9, raw_goal={}, flight=0.8)
    snap = node._toss_trim_snapshot()
    assert snap['pose'] == (11.0, 22.0, 170.0)
    # The next cycle moves on; the snapshot does not.
    node._open_toss_record(action='toss', goal_id='g', cycle_index=2,
                           catch_pose=(99.0, 88.0, 170.0), throw_delay=5.0,
                           vel_scale=0.9, raw_goal={}, flight=0.8)
    assert snap['pose'] == (11.0, 22.0, 170.0)
    assert node._toss_trim_snapshot()['pose'] == (99.0, 88.0, 170.0)


def test_the_worker_is_drained_before_the_trim_proposal_is_written():
    """The drain is the CONTRACT: "asynchronous" must not come to mean
    "sometimes missing". The proposal is built from the trim's accumulated
    state, and the last cycle or two of a session are still queued when the goal
    terminates — a proposal that omitted them would be a silently short session,
    which is worse than a slow one."""
    import ast
    src = ast.parse(open(rcn.__file__).read())
    fn = next(n for n in ast.walk(src)
              if isinstance(n, ast.FunctionDef) and n.name == '_toss_trim_end')
    calls = [n.func.attr for n in ast.walk(fn)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
    assert '_toss_records_drain' in calls
    assert calls.index('_toss_records_drain') < calls.index('_toss_ilc_session_end')


def test_a_worker_that_cannot_start_falls_back_to_running_inline():
    """Degrading to the OLD behaviour is the right failure here; dropping the
    measurement is not. A node whose thread creation failed still produces a
    corpus and still converges a trim — just on the cycle thread, as before."""
    now = 100.0
    node = _toss_ready_node(now)
    node._toss_records_worker_start = lambda: (_ for _ in ()).throw(
        RuntimeError('cannot spawn'))
    warned, ran = [], []
    node.get_logger().warning = warned.append
    node._belt_toss_record = lambda payload: ran.append(
        threading.current_thread().name)
    node._open_toss_record(action='toss', goal_id='g', cycle_index=1,
                           catch_pose=(0.0, 0.0, 170.0), throw_delay=5.0,
                           vel_scale=0.9, raw_goal={}, flight=0.8)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    assert ran == [threading.current_thread().name]        # inline, not deferred
    assert any('INLINE' in w for w in warned)


def test_a_failed_thread_start_does_not_latch_a_dead_worker_handle():
    """CENSUS B6, audit fix 2026-08-22. ``_toss_records_worker_start`` assigns the
    handle INSIDE the lock and calls ``start()`` outside it; without a rollback a
    single ``RuntimeError`` from ``Thread.start()`` (thread exhaustion on a loaded
    Jetson) latched the handle forever.

    The caller's inline fallback then wrote THAT record correctly — so the failure
    looked transient — while every later ``_toss_records_submit`` returned early
    at the ``is not None`` check and ``put_nowait``'d into a queue no thread was
    servicing. Silent, permanent loss of the sitting's toss corpus, plus a full
    5 s drain timeout at every goal end, announced as one warning.
    """
    now = 100.0
    node = _toss_ready_node(now)
    real_thread = threading.Thread
    attempts = []

    class _FailFirst(real_thread):
        def start(self):
            attempts.append(self.name)
            if len(attempts) == 1:
                raise RuntimeError('cannot spawn')
            return real_thread.start(self)

    ran = []
    node._belt_toss_record = lambda payload: ran.append(
        threading.current_thread().name)
    node.get_logger().warning = lambda *_a, **_k: None

    with mock.patch.object(threading, 'Thread', _FailFirst):
        node._open_toss_record(action='toss', goal_id='g', cycle_index=1,
                               catch_pose=(0.0, 0.0, 170.0), throw_delay=5.0,
                               vel_scale=0.9, raw_goal={}, flight=0.8)
        node._log_toss_outcome(TossResult(True, 'CAUGHT'))
        # Cycle 1: start() raised, so the record ran INLINE and the handle was
        # rolled back rather than latched.
        assert ran == [threading.current_thread().name]
        assert node._toss_records_thread is None

        # Cycle 2: the retry is possible BECAUSE of the rollback, and it lands
        # on the worker.
        node._open_toss_record(action='toss', goal_id='g', cycle_index=2,
                               catch_pose=(0.0, 0.0, 170.0), throw_delay=5.0,
                               vel_scale=0.9, raw_goal={}, flight=0.8)
        node._log_toss_outcome(TossResult(True, 'CAUGHT'))

    assert node._toss_records_drain()
    assert len(attempts) == 2
    assert ran[1] == 'toss_records'


def test_the_drain_waits_for_the_WORK_not_merely_for_the_queue_to_empty():
    """``Queue.empty()`` goes True the instant the worker ``get``s the last item —
    BEFORE the belt write and the trim update have run. Polling it returned while
    the final cycle was still being processed, which is the exact opposite of what
    the drain promises, and it is why ``_toss_trim_end`` could write a proposal
    with the last cycle missing (audit fix, 2026-08-22).

    Driven by making the worker's own work slow enough that ``empty()`` and
    ``unfinished_tasks == 0`` are separated by a real interval — no sleep in the
    test thread, and no dependence on which thread wins a race."""
    now = 100.0
    node = _toss_ready_node(now)
    started, finished = threading.Event(), []

    def _slow(payload):
        started.set()
        time.sleep(0.25)
        finished.append(payload)

    node._belt_toss_record = _slow
    node._open_toss_record(action='toss', goal_id='g', cycle_index=1,
                           catch_pose=(0.0, 0.0, 170.0), throw_delay=5.0,
                           vel_scale=0.9, raw_goal={}, flight=0.8)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))

    # The worker has the item in hand: the queue reads EMPTY and the work is not
    # done. This is precisely the window the old drain returned in.
    assert started.wait(timeout=5.0)
    assert node._toss_records_q.empty()
    assert finished == []

    assert node._toss_records_drain(timeout_s=5.0)
    assert len(finished) == 1                    # the drain outlasted the work


def test_the_drain_reports_failure_rather_than_hanging_on_a_wedged_worker():
    """Bounded, and False on timeout: a wedged worker must never hold a session's
    terminal open. The WARN names how many records are outstanding, which is the
    number an operator needs to know how short the corpus is."""
    now = 100.0
    node = _toss_ready_node(now)
    release = threading.Event()
    node._belt_toss_record = lambda payload: release.wait(timeout=10.0)
    warned = []
    node.get_logger().warning = warned.append
    node._open_toss_record(action='toss', goal_id='g', cycle_index=1,
                           catch_pose=(0.0, 0.0, 170.0), throw_delay=5.0,
                           vel_scale=0.9, raw_goal={}, flight=0.8)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    try:
        t0 = time.perf_counter()
        assert node._toss_records_drain(timeout_s=0.2) is False
        assert time.perf_counter() - t0 < 5.0     # returned, did not hang
        assert any('did not drain' in w for w in warned)
    finally:
        release.set()
        node._toss_records_drain(timeout_s=5.0)


# ── The tick-loop census is actually wired (INSTRUMENT ONLY) ─────────────────

def test_the_cycle_loop_feeds_the_census_and_drops_only_the_terminal(monkeypatch):
    """THE wiring pin. Every other census test injects a pre-filled census or
    exercises the class in isolation, so all of them stay green if
    ``_run_toss_cycle`` stops calling it — and a whole sitting would then report
    a null timing block that reads exactly like "no cycle ran".

    Also pins the lag-by-one boundary against the real control flow: four
    decisions, the fourth terminal, must commit THREE iterations. The terminal
    tick returns from the middle of the loop and never reaches its sleep, so
    charging it a period would report time the loop did not spend."""
    node = _toss_ready_node(time.perf_counter())
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    script = [
        TossDecision(PHASE_PREPARING),
        TossDecision(PHASE_PREPARING),
        TossDecision(PHASE_BALL_IN_FLIGHT),
        TossDecision(PHASE_BALL_IN_FLIGHT, done=True,
                     result=TossResult(True, 'CAUGHT')),
    ]
    monkeypatch.setattr(node, '_step_toss_sequence',
                        lambda seq, now, gh, state=None: script.pop(0))
    seen = {}
    monkeypatch.setattr(
        node, '_log_toss_outcome',
        lambda r: seen.update(summary=node._toss_loop_census.summary()))
    result, kind = node._run_toss_cycle(
        object(), deadline_s=10.0, cancel_now_fn=lambda now: False,
        feedback_fn=None)

    assert kind == 'fsm' and result.outcome == 'CAUGHT'
    s = seen['summary']
    assert s['loop_n_pre'] == 2                   # the two PREPARING ticks
    assert s['loop_n_post'] == 1                  # the first BALL_IN_FLIGHT tick
    assert s['loop_period_max_pre_s'] > 0.0       # a real measurement, not a zero
    assert s['loop_n_over_pre'] == 0              # sleep is stubbed out here


def test_the_census_is_consumed_by_the_outcome_line():
    """``_log_toss_outcome`` clears it, so a later terminal that ran no loop
    (REJECTED_BAD_GOAL) cannot inherit these timings. Pinned at the node seam
    because the clear lives there, not in the census."""
    from jugglebot.toss_sequencer import LoopPeriodCensus
    node = _toss_ready_node(time.perf_counter())
    node._toss_loop_census = LoopPeriodCensus()
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    assert node._toss_loop_census is None


def test_the_loop_census_on_the_record_is_the_one_the_cycle_actually_fed(
        monkeypatch):
    """The census is on the SLOT since B4, and there must be exactly ONE per
    cycle — the one `_run_toss_cycle` (or `_tick_toss_pipeline`) feeds and the
    one `_toss_record_fields` reads.

    Two would be the worst kind of instrument bug: the loop would feed one, the
    record would declare the other, and every timing field would read null while
    the loop was measured perfectly. Nothing would go red — a null is "not
    measured", which is a legal value — so a session's whole timing census would
    vanish silently. That is precisely what the census exists to prevent one
    level up."""
    node = _toss_ready_node(100.0)
    _install_toss_goal(node)
    seq = _fresh_seq(node)
    state = node._toss_committed
    from jugglebot.toss_sequencer import LoopPeriodCensus, TossDecision
    built = LoopPeriodCensus()
    state.census = built
    monkeypatch.setattr(node, '_step_toss_sequence',
                        lambda *a, **k: TossDecision(
                            'CHECKING', 'none', True,
                            TossResult(False, 'REJECTED_TEST')))
    monkeypatch.setattr(node, '_log_toss_outcome', lambda r: None)
    node._run_toss_cycle(seq, deadline_s=1.0, cancel_now_fn=lambda n: False,
                         feedback_fn=None, state=state)
    assert node._toss_loop_census is built, (
        'the loop fed a census the record cannot see')
    assert state.census is built


# ══ B5 lever 1 — absolute-schedule tick pacing ════════════════════════════════
#
# What is under test here is the SCHEDULE, so these drive the module function
# directly against a fake clock rather than through a node: a node would only add
# ways for the schedule to be right and the test to be wrong. The two loops' USE
# of it is pinned STRUCTURALLY at the bottom of this section, because a loop that
# quietly went back to `time.sleep(_TICK_S)` would pass every behavioural test
# above it.
#
# The recipe behind the numbers: /tmp/probe_pace_rescore.py re-scored the B0/P1
# corpus (73 chained cycles) as `max(PERIOD, work)` and /tmp/probe_sleep_
# granularity.py measured this Jetson's `time.sleep` overshoot at p50 0.1 ms /
# max 0.9 ms idle, both 2026-08-27. Both are one-offs; the durable numbers live
# in the constants' own comments.

_PERIOD = float(rcn._PACE_PERIOD_S)
_SLOP = float(rcn._PACE_SLOP_S)


class _PacedClock:
    """Stand-in for the node module's ``time`` — ``perf_counter`` + ``sleep``.

    ``sleep`` advances the clock by the requested interval PLUS ``overshoot``,
    which is what a real ``time.sleep`` does and the entire reason the early-fire
    band exists. Every interval the pacer asks for is recorded, so a test can
    assert it never asked for a negative one and never asked for none at all."""

    def __init__(self, t0=1000.0, overshoot=0.0):
        self.t = float(t0)
        self.overshoot = float(overshoot)
        self.sleeps = []

    def perf_counter(self):
        return self.t

    def sleep(self, dt):
        self.sleeps.append(float(dt))
        self.t += float(dt) + self.overshoot

    def work(self, dt):
        """Burn ``dt`` of clock inside a loop body."""
        self.t += float(dt)


def _paced_tops(clock, works, monkeypatch):
    """Run the pacer over a list of per-iteration WORK durations and return the
    ``now`` each loop body would have seen. One extra top is returned — the one
    the iteration AFTER the last would start at — because the interesting
    quantity is the interval, not the iteration."""
    monkeypatch.setattr(rcn, 'time', clock)
    next_due = clock.perf_counter()
    tops = []
    for w in works:
        tops.append(clock.perf_counter())
        clock.work(w)
        next_due = rcn._pace_to_next_tick(next_due)
    tops.append(clock.perf_counter())
    return tops


def test_the_paced_period_is_the_budget_denominator_and_not_the_sleep():
    """The drift guard, and the whole B5 decision in three asserts.

    Every budget in this stack counts `NODE_LOOP_PERIOD_S`, so that is what the
    loops are paced to; `_TICK_S` is half of it and pacing there would degenerate
    to no sleep at all on the majority of measured pre-dispatch ticks (chained
    `loop_work_max_pre_s` p50 0.0237 against a 0.020 target, B0/P1). The alias is
    an alias so the two CANNOT drift."""
    assert rcn._PACE_PERIOD_S == rcn.TOSS_LOOP_PERIOD_S
    assert rcn._PACE_PERIOD_S != rcn._TICK_S
    # The band absorbs the WAKE granularity, not half the period — see the
    # poller transposition at `_PACE_SLOP_S`. Half a period is the ceiling.
    assert 0.0 < rcn._PACE_SLOP_S < rcn._PACE_PERIOD_S / 2.0


def test_the_paced_loop_holds_an_absolute_grid_and_never_accumulates_drift(
        monkeypatch):
    """THE property. Due instants come from ``next_due += PERIOD``, never from
    ``now``, so a per-iteration lateness (the sleep's own overshoot, the band it
    is slept short by) is a CONSTANT offset from the grid rather than a term that
    compounds. Twenty iterations of varying work, and the offset at the last is
    the offset at the first."""
    clock = _PacedClock(overshoot=0.0005)
    t0 = clock.perf_counter()
    works = [0.005, 0.030, 0.001, 0.020, 0.012] * 4
    tops = _paced_tops(clock, works, monkeypatch)
    offsets = [tops[k] - (t0 + k * _PERIOD) for k in range(1, len(tops))]
    assert offsets[0] == pytest.approx(-_SLOP + clock.overshoot, abs=1e-9)
    assert offsets[-1] == pytest.approx(offsets[0], abs=1e-9)
    for k in range(2, len(tops)):
        assert tops[k] - tops[k - 1] == pytest.approx(_PERIOD, abs=1e-9)


def test_the_early_fire_band_fires_a_wake_inside_it_without_sleeping(
        monkeypatch):
    """The band's second face: a due instant already inside the band is not worth
    a scheduler round trip. Sleeping the residual would cost a whole wake
    latency (~1.5 ms under the live executor) to remove less than that — so the
    pacer fires, up to ``_PACE_SLOP_S`` early, and leaves the GRID untouched.
    That is the poller's "round the due instant to the nearest tick", and it is
    why the band can never be sized to half the PERIOD."""
    clock = _PacedClock()
    t0 = clock.perf_counter()
    # One tick of work that lands inside the band, then a normal one.
    tops = _paced_tops(clock, [_PERIOD - _SLOP / 2.0, 0.005], monkeypatch)
    assert len(clock.sleeps) == 1                # the FIRST tick slept not at all
    assert tops[1] - tops[0] == pytest.approx(_PERIOD - _SLOP / 2.0, abs=1e-9)
    # …and the grid is unmoved: the next top is still on `t0 + 2 * PERIOD`.
    assert tops[2] == pytest.approx(t0 + 2 * _PERIOD - _SLOP, abs=1e-9)


def test_a_mild_overrun_stays_on_the_grid_so_four_ticks_still_cost_four_periods(
        monkeypatch):
    """``pre_dispatch_budget_s`` charges the SUM of four ticks, not four
    individual ticks, and a preserved grid is what makes the sum exact even when
    one member of it overran. The catch-up this implies is bounded to ONE period
    by construction — the next tick is short by exactly the overrun and no
    more."""
    clock = _PacedClock()
    over = 0.010
    tops = _paced_tops(clock, [0.010, _PERIOD + over, 0.010, 0.010], monkeypatch)
    assert tops[2] - tops[1] == pytest.approx(_PERIOD + over, abs=1e-9)
    assert tops[3] - tops[2] == pytest.approx(_PERIOD - over, abs=1e-9)
    assert tops[4] - tops[0] == pytest.approx(4 * _PERIOD - _SLOP, abs=1e-9)


def test_a_loop_more_than_one_period_behind_reanchors_instead_of_bursting(
        monkeypatch):
    """RECOVERY, and it is the decision this phase had to take rather than
    inherit. The loop is a periodic SAMPLER — every FSM guard is level-triggered
    on ``now`` — so a grid slot not taken has no backlog, and replaying the
    missed ones would fire iterations at almost the same ``now`` doing no useful
    work, charge the census several near-zero periods on exactly the cycle that
    just overran, and starve the executor of a yield straight after a heavy tick.

    The trigger is measured, not hypothetical: the one B0/P1 cycle that commanded
    a positioning move spent 0.3022 s in a single iteration, which is 7.5 periods
    of backlog. A catch-up pacer would answer that with seven zero-length sleeps;
    this one answers with one full period and a re-based grid."""
    clock = _PacedClock()
    tops = _paced_tops(clock, [0.3022, 0.010, 0.010], monkeypatch)
    assert len(clock.sleeps) == 3                    # one per iteration
    assert all(s > 0.0 for s in clock.sleeps)        # …and NOT a burst of zeros
    assert clock.sleeps[0] == pytest.approx(_PERIOD - _SLOP, abs=1e-9)
    # The grid is re-based on the overrun's END, so the very next interval is a
    # whole period rather than whatever was left of an abandoned schedule.
    assert tops[2] - tops[1] == pytest.approx(_PERIOD, abs=1e-9)
    assert tops[3] - tops[2] == pytest.approx(_PERIOD, abs=1e-9)


def test_the_pacer_never_asks_for_a_negative_sleep(monkeypatch):
    """``max(0, .)`` lives in the ``if``, and a negative interval would be a
    ``ValueError`` from the real ``time.sleep`` — i.e. an exception thrown from
    the bottom of a loop holding an armed catch latch. Swept across work
    durations that straddle the band, the period and the re-anchor threshold."""
    clock = _PacedClock()
    works = [i * _PERIOD / 8.0 for i in range(0, 25)]
    _paced_tops(clock, works, monkeypatch)
    assert clock.sleeps, 'the sweep must exercise the sleeping branch too'
    assert all(s > 0.0 for s in clock.sleeps)


def test_both_toss_loops_pace_to_the_grid_rather_than_sleeping_a_fixed_tick():
    """STRUCTURAL, and it is the one that would catch a regression the
    behavioural tests above cannot see: a loop that went back to
    ``time.sleep(_TICK_S)`` still ticks, still terminalises, still censuses — it
    just silently un-denominates every budget again. The two loops are named
    explicitly because the pipelined one is the session loop, NOT
    ``_tick_toss_pipeline`` (which has no loop of its own), and that is exactly
    the seam a future reader would get wrong."""
    import ast
    src = ast.parse(open(rcn.__file__).read())
    for name in ('_run_toss_cycle', '_execute_toss_continuous'):
        fn = next(n for n in ast.walk(src)
                  if isinstance(n, ast.FunctionDef) and n.name == name)
        called = [n.func.id for n in ast.walk(fn)
                  if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)]
        assert '_pace_to_next_tick' in called, name
        slept = [n for n in ast.walk(fn)
                 if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Attribute)
                 and n.func.attr == 'sleep']
        assert not slept, '%s still sleeps a fixed interval' % name


def test_the_census_measures_the_paced_period_and_the_headroom_left(monkeypatch):
    """The instrument stays truthful across the change of what the WAIT is.
    ``loop_sleep_max_pre_s`` is still ``next_now - t_pre_sleep`` and still
    measured — but under pacing it reports HEADROOM (``period - work``) rather
    than a fixed sleep's overshoot, which is what its docstring now says. The
    period itself is the constant, which is the whole point of the phase."""
    clock = _PacedClock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _toss_ready_node(clock.perf_counter())
    work = 0.011
    script = [
        TossDecision(PHASE_PREPARING),
        TossDecision(PHASE_PREPARING),
        TossDecision(PHASE_PREPARING),
        TossDecision(PHASE_BALL_IN_FLIGHT, done=True,
                     result=TossResult(True, 'CAUGHT')),
    ]

    def _step(seq, now, gh=None, state=None, obs=None):
        clock.work(work)
        return script.pop(0)

    monkeypatch.setattr(node, '_step_toss_sequence', _step)
    seen = {}
    monkeypatch.setattr(
        node, '_log_toss_outcome',
        lambda r: seen.update(summary=node._toss_loop_census.summary()))
    node._run_toss_cycle(object(), deadline_s=10.0,
                         cancel_now_fn=lambda now: False, feedback_fn=None)
    s = seen['summary']
    assert s['loop_n_pre'] == 3        # lag-by-one: the terminal tick is dropped
    assert s['loop_period_max_pre_s'] == pytest.approx(_PERIOD, abs=1e-9)
    assert s['loop_work_max_pre_s'] == pytest.approx(work, abs=1e-9)
    # HEADROOM, exactly: in the steady state the band cancels (an iteration both
    # starts and ends `_PACE_SLOP_S` before its grid instant), so the wait is
    # `period - work` on the nose. Only the FIRST iteration is short by the band,
    # which is why this is the MAX and not the min.
    assert s['loop_sleep_max_pre_s'] == pytest.approx(_PERIOD - work, abs=1e-9)
    # The signal B5 buys: with the fixed sleep gone, over-threshold now means
    # "this tick's own work outran the budget denominator". On the real box a
    # wake-latency spike can still trip it by a millisecond or two — see the
    # caveat at `_PACE_PERIOD_S` — which is exactly why this asserts against a
    # clock with zero overshoot rather than pretending the box has none.
    assert s['loop_n_over_pre'] == 0
