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
path — no copied strings), the terminal orderings (prime_hold released LAST), the
trace-only ball-evidence waiver, the per-phase cancel deferral, and the node-level
early exits (cancel/timeout/shutdown/exception all safe before terminalising).

Service calls are monkeypatch-seamed (MockServiceClient futures never resolve) and
time.sleep is monkeypatched where a ladder would otherwise wait — the
test_reload_coordinator_node.py pattern throughout.

ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

import math
import time
import types

import numpy as np
import pytest

import jugglebot.hardware_config as hw
import jugglebot.reload_coordinator_node as rcn
from jugglebot.motion.trajectory import hand_stroke
from jugglebot.reload_coordinator_node import (
    ReloadCoordinatorNode,
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
    TIER_8B,
    TOSS_CONTROL_MODE,
    TossDecision,
    TossResult,
    TossSequencer,
)
from jugglebot.motion.trajectory.toss_release import (
    ThrowTiltInfeasible,
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


def _toss_ready_node(now, commanded_pos=(0.0, 0.0, 170.0)):
    """A node with every toss precondition satisfied and caches FRESH at `now`:
    TRAJECTORY streaming, mocap fresh, a fresh trajectory/status affirming a
    loaded gravity correction, a fresh trajectory/commanded_position (the
    displaced toss's throw site A — Phase E; absent it the goal is
    REJECTED_POSE_UNKNOWN), hand telemetry fresh AT the bottom park band,
    ball possession latched, no live tracks."""
    node = ReloadCoordinatorNode()
    with node._lock:
        node._control_mode = TOSS_CONTROL_MODE
        node._streaming = True
        node._traj_status_mono = now
        node._gravity_correction_loaded = True
        node._commanded_pos_mm = tuple(float(v) for v in commanded_pos)
        node._commanded_pos_mono = now
        node._mocap_mono = now
        node._balls = []
        node._balls_mono = now
        node._hand_pos_meas = 0.0
        node._hand_vel_meas = 0.0
        node._hand_telemetry_mono = now
        node._ball_possession = True
    return node


def _stamp_fresh(node, t):
    """Re-stamp the freshness clocks at synthetic time `t` (the observation
    builder compares them against the step's `now`)."""
    with node._lock:
        node._mocap_mono = t
        node._balls_mono = t
        node._hand_telemetry_mono = t
        node._traj_status_mono = t
        node._commanded_pos_mono = t


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
        node._toss_release_state = release
        node._toss_landing_global_mm = tuple(
            float(v) for v in release.catch_point_global_mm)
        node._toss_platform_target_mm = tuple(float(v) for v in pose)
        node._toss_waiver = False
        node._toss_prepare_pending = False
        node._toss_throw_dispatched = False
        node._toss_stroke_seen = False
        node._toss_track_confirmed = False
    return release


def _fresh_seq(node, pose=(0.0, 0.0, 170.0), flight=0.8, delay=5.0, start=100.0):
    release = _install_toss_goal(node, pose=pose, flight=flight)
    seq = TossSequencer(catch_pose_stow_mm=pose, flight_time_s=flight,
                        throw_delay_s=delay,
                        event_vel_mps=float(release.event_vel_mps))
    seq.start(start)
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
    ('track_active', 'REJECTED_TRACK_ACTIVE'),
    ('delay_floor', 'REJECTED_CANT_MAKE_LEAD'),
    ('flight_band', 'REJECTED_FLIGHT_TIME'),
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

    GATE ORDER, and why the two envelope rows read the way they do. This
    enumeration runs at the SHIPPED config tier, which became '8b' on
    2026-07-28. Under 8b the displaced-throw gates (toss_sequencer's CHECKING
    block: the |B − A| cap, then the closed-form reach bound) run BEFORE the
    workspace box — documented-in-code and intended, because a cap-rejected
    goal has no valid tilted release state and so no meaningful event_vel to
    check. Consequences, measured through this very path on 2026-07-28 (a
    throw-away probe drove _execute_toss over a coordinate sweep; results are
    the ground truth quoted here, not inferred from reading the gate):

      x=200 → REJECTED_DISPLACEMENT   (this row read WORKSPACE under 8a)
      x=80  → REJECTED_DISPLACEMENT   x=71 → REJECTED_DISPLACEMENT
      x=70  → passes CHECKING         (the 70 mm cap is `>`, so 70 is legal)
      z=221 → REJECTED_WORKSPACE      z=220 → passes (the ±50 band is `>` too)

    So under 8b with the shipped throw site A = (0, 0), the workspace box's
    |x|,|y| ≤ 150 mm half is STRUCTURALLY UNREACHABLE through this path: the
    70 mm displacement cap is strictly tighter than the 150 mm box, so any goal
    that could violate the box laterally is rejected as DISPLACEMENT first. The
    z band is the only reachable WORKSPACE branch, and the row is built as
    x=60 (a LIVE displacement, inside the 70 mm cap and the 256 mm reach bound
    at T=0.8 s, so the 8b gates genuinely run and pass) plus z=300 (|z − 170| =
    130 mm, past the ±50 mm band). A zero-displacement variant would reach the
    same branch while proving less — it would still pass if the cap collapsed
    to zero and took every real displaced goal with it. The lateral half of the
    box keeps its coverage at FSM level, tier-agnostic, in
    test_toss_sequencer.py::test_workspace_precheck_rejected."""
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
        monkeypatch.setattr(hw, 'JB_OP_TOSS_REQUIRE_BALL_EVIDENCE', True)
        node._ball_possession = False
    elif breakage == 'track_active':
        node._balls = [_Ball(status=1, destination='jugglebot', id=9)]
    elif breakage == 'delay_floor':
        gh = _TossGoalHandle(delay=2.0)
    elif breakage == 'flight_band':
        gh = _TossGoalHandle(throw_height=0.2)   # →0.404 s, below the flight band
    elif breakage == 'displacement':
        # 200 mm from the config throw site A = (0, 0): past the 70 mm cap.
        gh = _TossGoalHandle(x=200.0)
    elif breakage == 'workspace':
        # Displacement 60 mm PASSES the 8b gates (cap 70 mm; reach bound 256 mm
        # at T = 0.8 s), then the ±50 mm z band rejects at |z − 170| = 130 mm.
        gh = _TossGoalHandle(x=60.0, z=300.0)
    result = node._execute_toss(gh)
    assert result.success is False
    assert result.outcome == expected
    assert gh.terminal == 'abort'
    assert math.isnan(result.catch_error_mm)
    assert math.isnan(result.achieved_flight_s)


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
        lambda seq, now, gh=None: pytest.fail('FSM ran on a bad goal'))
    gh = _TossGoalHandle(**kwargs)
    result = node._execute_toss(gh)
    assert result.success is False
    assert result.outcome == 'REJECTED_BAD_GOAL({})'.format(field)
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
                        lambda seq: order.append('safed'))
    logged = []
    monkeypatch.setattr(node, '_log_toss_outcome',
                        lambda r: logged.append(str(r.outcome)))
    monkeypatch.setattr(
        node, '_step_toss_sequence',
        lambda seq, now, gh=None: (_ for _ in ()).throw(RuntimeError('boom')))
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
        lambda seq: seq.note_position_result(
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


def test_no_ball_does_not_reject_by_default(monkeypatch):
    """Change B (single-ball-toss Phase 5): with the SHIPPED default
    toss_require_ball_evidence=false, an empty-cup toss does NOT
    REJECTED_NO_BALL — there is no ball-in-cup sensor, "possession" is only an
    unreliable tracker belief, and the operator guarantees the ball, so the
    software belief must not block a legitimate throw. The goal proceeds past
    CHECKING to a scripted positioning reject (hand-parked stays hard — the
    waiver test covers that the physical-hazard gates are untouched)."""
    assert bool(hw.JB_OP_TOSS_REQUIRE_BALL_EVIDENCE) is False   # shipped default
    now = time.perf_counter()
    node = _toss_ready_node(now)
    node._ball_possession = False                # empty cup, no waiver
    monkeypatch.setattr(
        node, '_position_platform_for_toss',
        lambda seq: seq.note_position_result(
            time.perf_counter(), False, 0.0, 'WORKSPACE'))
    result = node._execute_toss(_TossGoalHandle())
    assert result.outcome == 'REJECTED_POSITION(WORKSPACE)'     # got PAST NO_BALL


# ── Possession latch (D4a — no ball-in-cup sensor exists) ──────────────────────

def test_possession_latched_from_plausible_caught_on_balls():
    node = ReloadCoordinatorNode()
    assert node._ball_possession is False
    node._on_balls(types.SimpleNamespace(
        balls=[_Ball(status=2, x=10.0, y=-5.0, z=805.0)]))
    assert node._ball_possession is True


def test_possession_not_latched_from_implausible_caught():
    """The split-track corruption class: a CAUGHT coasting below the floor near
    BB must not fake possession (the same plausibility gates as the reload
    verdict)."""
    node = ReloadCoordinatorNode()
    node._on_balls(types.SimpleNamespace(
        balls=[_Ball(status=2, x=-539.0, y=-323.0, z=-532.0)]))
    assert node._ball_possession is False


def test_possession_not_latched_from_other_destination():
    node = ReloadCoordinatorNode()
    node._on_balls(types.SimpleNamespace(
        balls=[_Ball(status=2, destination='someone_else')]))
    assert node._ball_possession is False


def test_possession_plausibility_uses_nominated_point_during_toss():
    """While a toss goal is live the CAUGHT plausibility reference is the
    NOMINATED landing point, not the fixed ACTIVE catch point — a corner-pose
    toss's own catch must latch possession, and a far-away corrupt CAUGHT that
    happens to sit near the DEFAULT point must not."""
    node = ReloadCoordinatorNode()
    _install_toss_goal(node, pose=(140.0, 140.0, 170.0), flight=0.8)
    # Near the nominated cup point → latches.
    node._on_balls(types.SimpleNamespace(
        balls=[_Ball(status=2, x=140.0, y=140.0, z=805.0)]))
    assert node._ball_possession is True
    # Far from the nominated point (339 mm off) but near the DEFAULT catch
    # point — with the nominated reference this is implausible → no latch.
    node._ball_possession = False
    node._on_balls(types.SimpleNamespace(
        balls=[_Ball(status=2, x=-100.0, y=-100.0, z=805.0, id=6)]))
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


def test_possession_cleared_on_release_evidence(monkeypatch):
    """OUR release evidence (the throw-stroke telemetry signature after the
    dispatch) clears possession — the ball left the cup. A missed toss stays
    cleared (next toss is honestly REJECTED_NO_BALL); a caught one re-latches
    via the CAUGHT path (chainable Toss → Toss)."""
    monkeypatch.setattr(hw, 'JB_OP_TOSS_REQUIRE_BALL_EVIDENCE', True)
    now = 100.0
    node = _toss_ready_node(now)
    _install_toss_goal(node)
    with node._lock:
        node._toss_throw_dispatched = True
        node._hand_vel_meas = 50.0           # ≫ the 40 rev/s stroke threshold
        node._hand_pos_meas = 3.0            # clear of the bottom park band
    obs = node._build_toss_observations(now)
    assert obs.throw_stroke_seen is True
    assert node._ball_possession is False
    assert obs.ball_seated is False


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
        lambda seq: seq.note_position_result(
            time.perf_counter(), False, 0.0, 'WORKSPACE'))
    result = node._execute_toss(_TossGoalHandle())
    assert result.outcome == 'REJECTED_POSITION(WORKSPACE)'   # past CHECKING
    with node._lock:
        assert node._ball_possession is True          # possession survived
        assert node._toss_track_confirmed is False    # no false release evidence


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
        node._toss_throw_dispatched = True
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
        node._toss_throw_dispatched = True
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
        node._toss_throw_dispatched = True
    obs = node._build_toss_observations(now)
    assert obs.ball_track_confirmed is True
    assert obs.ball_time_at_land_perf == pytest.approx(12.5 + 100.0)


def test_caught_correlates_to_latched_id_vs_nominated_point():
    """CAUGHT confirms only for the latched announced-ball id, plausibility- and
    error-judged against the NOMINATED landing point (not reload's fixed catch
    point)."""
    now = 100.0
    node = _toss_ready_node(now)
    _install_toss_goal(node, pose=(30.0, -40.0, 170.0))
    with node._lock:
        node._balls = [_Ball(status=1, id=5)]            # latch id 5
    node._build_toss_observations(now)
    with node._lock:
        node._balls = [_Ball(status=2, id=99, x=30.0, y=-40.0),   # stray id
                       _Ball(status=1, id=5)]
    assert node._build_toss_observations(now).ball_caught is False
    with node._lock:
        node._balls = [_Ball(status=2, id=5, x=45.0, y=-40.0, z=805.0)]
    obs = node._build_toss_observations(now)
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
        lambda s: s.note_position_result(100.0, True, 0.3))   # arrival 100.5
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
        lambda s: s.note_position_result(100.0, True, 0.3))   # arrival 100.5
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
        lambda s: s.note_position_result(100.0, True, 0.3))
    monkeypatch.setattr(node, '_publish_prime_hold', lambda h: None)
    monkeypatch.setattr(node, '_publish_pretilt_hold', lambda h: None)
    monkeypatch.setattr(node._reach_center_pub, 'publish',
                        lambda msg: published.append((msg.x, msg.y, msg.z)))
    for t in (100.0, 100.6):
        _stamp_fresh(node, t)
        node._step_toss_sequence(seq, t)
    assert published == [(140.0, 0.0, 170.0)]


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
        lambda s: order.append('position') or s.note_position_result(
            t0, True, 0.3))                              # arrival t0 + 0.5
    _wire_prepare_recorder(node, order, monkeypatch)
    ann_pub = node._publishers['throw_announcements']
    orig_ann_publish = ann_pub.publish

    def _rec_announce(msg):
        order.append('announce')
        orig_ann_publish(msg)
    monkeypatch.setattr(ann_pub, 'publish', _rec_announce)

    def _fake_dispatch(s):
        order.append('dispatch')
        with node._lock:
            node._toss_throw_dispatched = True
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
                        lambda: pytest.fail(
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
    _stamp_fresh(node, t0 + 5.9)
    d = node._step_toss_sequence(seq, t0 + 5.9, gh)
    assert d.done and d.result.outcome == 'CAUGHT'
    assert d.result.catch_error_mm == pytest.approx(0.0)
    assert order == ['position', ('prime_hold', True), 'gains', ('arm', True),
                     ('vel_scale', 0.8), ('prime_dispatched', True),
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
        lambda s: s.note_position_result(t0, True, 0.3))
    safed = []
    monkeypatch.setattr(node, '_toss_safe_abort', lambda: safed.append(1))
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
    assert d.result.outcome == 'REJECTED_POSITION(WIRE_DISARMED)'


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
        lambda s: s.note_position_result(100.0, False, 0.0, 'NO_RESPONSE'))
    homed = []
    monkeypatch.setattr(node, '_go_home', lambda: homed.append(1) or True)
    node._step_toss_sequence(seq, 100.0)
    _stamp_fresh(node, 100.1)
    d = node._step_toss_sequence(seq, 100.1)
    assert d.done and d.result.outcome == 'REJECTED_POSITION(NO_RESPONSE)'
    assert homed == [1]


def test_position_timeout_dispatches_best_effort_go_home(monkeypatch):
    """FIX-5c, the never-noted sibling: a positioning that times out without
    ANY response is the same unknown-motion state — best-effort go_home on
    ABORTED_POSITION_TIMEOUT too."""
    node = _toss_ready_node(100.0)
    seq = _fresh_seq(node, start=100.0)
    monkeypatch.setattr(node, '_position_platform_for_toss', lambda s: None)
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
        assert node._toss_throw_dispatched is True       # armed pre-ack
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
    node._toss_pretilt_hold_raised = False
    node._toss_stay()
    assert order == [('prime_hold', False)]              # 8a: never touched
    order.clear()
    node._toss_pretilt_hold_raised = True
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
                        lambda seq, now, gh=None: next(decisions))
    gh = _TossGoalHandle()
    gh.is_cancel_requested = True
    result = node._execute_toss(gh)
    assert result.outcome == 'MISSED'                    # the FSM's real verdict
    assert gh.terminal == 'canceled'


# ── Node-level early exits safe the robot first (FIX-6) ────────────────────────

def _script_position_accept(node, monkeypatch, side_effect=None):
    """Script positioning to ACCEPT (seq becomes prepared — the go_home leg of
    SAFE_ABORT is owed from that point), optionally firing a side effect."""
    def _accept(seq):
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
    monkeypatch.setattr(node, '_toss_safe_abort', lambda: order.append('safed'))
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
    monkeypatch.setattr(node, '_toss_safe_abort', lambda: safed.append(1))
    gh = _TossGoalHandle()
    result = node._execute_toss(gh)
    assert result.outcome == 'ABORTED_TIMEOUT'
    assert gh.terminal == 'abort'
    assert safed == [1]


def test_rclpy_shutdown_aborts_and_safes(monkeypatch):
    """rclpy going down mid-sequence exits ABORTED_SHUTDOWN with the same
    early-exit safing — a shutdown must not strand the latch raised or the
    hand parted from its park band."""
    import rclpy
    node = _toss_ready_node(time.perf_counter())
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    _script_position_accept(node, monkeypatch)
    safed = []
    monkeypatch.setattr(node, '_toss_safe_abort', lambda: safed.append(1))
    calls = {'n': 0}

    def _ok():
        calls['n'] += 1
        return calls['n'] < 2                    # one loop pass, then shutdown
    monkeypatch.setattr(rclpy, 'ok', _ok)
    gh = _TossGoalHandle()
    result = node._execute_toss(gh)
    assert result.outcome == 'ABORTED_SHUTDOWN'
    assert safed == [1]


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
        node._toss_release_state = release
        node._toss_landing_global_mm = tuple(
            float(v) for v in release.catch_point_global_mm)
        # Mirror _execute_toss: the 8b cross-check target is the commanded A pose
        # (single source _toss_positioning_xyz), NOT the nominated catch B.
        node._toss_platform_target_mm = ReloadCoordinatorNode._toss_positioning_xyz(
            TIER_8B, pose, release)
        node._toss_waiver = False
        node._toss_prepare_pending = False
        node._toss_throw_dispatched = False
        node._toss_stroke_seen = False
        node._toss_track_confirmed = False
        node._toss_pretilt_hold_raised = False
        node._toss_announced_reach = None
    seq = TossSequencer(catch_pose_stow_mm=pose, flight_time_s=flight,
                        throw_delay_s=delay, tier=TIER_8B,
                        throw_site_xy_mm=throw_site, throw_site_known=True,
                        event_vel_mps=float(release.event_vel_mps))
    seq.start(start)
    return seq, release


def test_pretilt_hold_raised_at_prepare_for_8b(monkeypatch):
    """Tier 8b raises catch/pretilt_hold on the ACTION_PREPARE_CATCH tick
    (alongside prime_hold, >=2 FSM ticks before our announcement can reach
    catch_coordinator) and records _toss_pretilt_hold_raised."""
    t0 = 100.0
    node = _toss_ready_node(t0)
    seq, _ = _fresh_seq_8b(node, start=t0)
    calls = []
    monkeypatch.setattr(node, '_position_platform_for_toss',
                        lambda s: s.note_position_result(t0, True, 0.3))
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
    assert node._toss_pretilt_hold_raised is True


def test_pretilt_hold_never_published_for_8a(monkeypatch):
    """THE 8a byte-identity pin: an 8a goal NEVER touches catch/pretilt_hold —
    the PREPARE tick raises prime_hold alone and _toss_pretilt_hold_raised stays
    False, so the publish sequence is bit-identical to Phase 1."""
    t0 = 100.0
    node = _toss_ready_node(t0)
    seq = _fresh_seq(node, start=t0)                     # tier 8a (default)
    calls = []
    monkeypatch.setattr(node, '_position_platform_for_toss',
                        lambda s: s.note_position_result(t0, True, 0.3))
    monkeypatch.setattr(node, '_publish_prime_hold',
                        lambda h: calls.append(('prime_hold', h)))
    monkeypatch.setattr(node, '_publish_pretilt_hold',
                        lambda h: calls.append(('pretilt_hold', h)))
    node._step_toss_sequence(seq, t0)
    _stamp_fresh(node, t0 + 0.5)
    d = node._step_toss_sequence(seq, t0 + 0.5)
    assert d.action == ACTION_PREPARE_CATCH
    assert ('prime_hold', True) in calls
    assert all(c[0] != 'pretilt_hold' for c in calls)    # topic NEVER touched
    assert node._toss_pretilt_hold_raised is False


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
    assert node._toss_platform_target_mm == pytest.approx((30.0, -40.0, 170.0))


def test_8b_cross_check_target_equals_commanded_A_pose(monkeypatch):
    """FIX-1 follow-up: the POSITIONING mocap arrival cross-check target
    (_toss_platform_target_mm) must equal the pose go_to_pose is actually
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
    assert node._toss_platform_target_mm == pytest.approx(commanded)
    assert node._toss_platform_target_mm == pytest.approx(
        (float(pre[0]), float(pre[1]), float(pre[2])))
    assert node._toss_platform_target_mm[0] != pytest.approx(50.0)   # NOT B's x


def test_step_dispatches_reach_catch_to_publish_toss_reach(monkeypatch):
    """The ACTION_REACH_CATCH decision routes to _publish_toss_reach — the
    node's ONLY platform publish for an 8b flight."""
    node = ReloadCoordinatorNode()
    called = []
    monkeypatch.setattr(node, '_publish_toss_reach', lambda: called.append(1))
    monkeypatch.setattr(node, '_build_toss_observations', lambda now: None)
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
        node._toss_announced_reach = (landing_pos, landing_vel, landing_time_ros)
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
        node._toss_announced_reach = None
    n0 = len(node._dyn_target_pub.published)
    node._publish_toss_reach()
    assert len(node._dyn_target_pub.published) == n0


def test_publish_toss_reach_policy_reject_publishes_nothing(monkeypatch):
    """A predicted_catch_command reject (should not happen post-CHECKING, but
    guarded) publishes nothing."""
    node = ReloadCoordinatorNode()
    with node._lock:
        node._toss_announced_reach = (np.zeros(3), np.zeros(3), 105.8)
    monkeypatch.setattr(node._toss_catch_policy, 'predicted_catch_command',
                        lambda p, v, t: None)
    n0 = len(node._dyn_target_pub.published)
    node._publish_toss_reach()
    assert len(node._dyn_target_pub.published) == n0


def test_toss_recenter_releases_pretilt_hold_last_when_raised(monkeypatch):
    """Tier 8b RECENTER: catch/pretilt_hold released LAST of all (after
    prime_hold), iff it was raised this goal — same cross-topic-ordering
    rationale; a stale pretilt_hold only DEGRADES a later reload, never a
    hazard."""
    node = ReloadCoordinatorNode()
    node._toss_pretilt_hold_raised = True
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
    node._toss_pretilt_hold_raised = True
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
    """8a (or an 8b goal that never reached PREPARE): _toss_pretilt_hold_raised
    stays False, so the terminals NEVER publish on catch/pretilt_hold — the 8a
    terminal publish sequence is byte-identical to Phase 1."""
    node = ReloadCoordinatorNode()
    assert node._toss_pretilt_hold_raised is False
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
    assert result.outcome == 'REJECTED_TILT_CLAMP'
    assert gh.terminal == 'abort'
