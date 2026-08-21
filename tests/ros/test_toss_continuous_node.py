"""Node-side tests for the continuous self-toss session (TossContinuous.action).

The session FSM itself is covered by test_toss_session.py; here we test the
coordinator's seams: the third action server on the SAME one-ball-op claim, the
goal-numerics gate, the session-level rejects through the real execute path, the
chain-reachability pre-check (Phase E's KNOWN LIMITATION made checkable before
anything moves), that every cycle is built and ticked by the SAME two methods the
single Toss uses, feedback content, and the node-level exits (cancel between
cycles / cancel inside a cycle / timeout / exception).

Two invariants that only exist at this level are pinned here:

  S1 — one live cycle. The session holds ONE busy claim, so a Reload or a Toss
       dispatched mid-session is REJECTED_BUSY.
  S2 — the session commands NO motion of its own. Every go_to_pose / go_home /
       hand dispatch in a session belongs to a cycle; a cancel or timeout
       BETWEEN cycles issues nothing at all.

The fake clock is a namespace swapped in for the node module's ``time``, which
uses exactly ``perf_counter`` and ``sleep`` — so a multi-second dwell costs
microseconds and the scheduling arithmetic is exact rather than approximate.

ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

import math
import types
from pathlib import Path

import pytest

import jugglebot.hardware_config as hw
import jugglebot.reload_coordinator_node as rcn
from jugglebot.reload_coordinator_node import (
    ReloadCoordinatorNode,
    _toss_session_deadline_s,
)
from jugglebot.reload_sequencer import ReloadSequencer
from jugglebot.toss_sequencer import (
    TIER_8A,
    TIER_8B,
    TOSS_CONTROL_MODE,
    TOSS_XY_LIMIT_MM,
    TossResult,
    TossSequencer,
)
from jugglebot.toss_session import TossSessionSequencer

DWELL = 8.0
DELAY = 5.0
FLIGHT = 0.8


class _Clock:
    """Stand-in for the node module's ``time`` (perf_counter + sleep only).

    ``attach`` makes every simulated tick re-stamp the node's freshness caches,
    reproducing what the real graph does during a dwell: ``trajectory/status``,
    ``rigid_body_poses``, ``hand_telemetry`` and ``trajectory/commanded_position``
    keep arriving on the ReentrantCallbackGroup while the execute callback
    sleeps. Without it the fake clock silently starves the caches and every
    cycle after the first reads REJECTED_POSE_UNKNOWN — an artefact of the
    harness, not of the node."""

    def __init__(self, t0=1000.0):
        self.t = float(t0)
        self._node = None
        # When set, every simulated tick also re-feeds the hand ball sensor at
        # the new time — /hand_telemetry keeps arriving at 100 Hz through a
        # dwell, so a sensor that goes UNKNOWN purely because the fake clock
        # jumped is a harness artefact. None leaves the sensor untouched (the
        # pre-2d behaviour of every test in this file).
        self.sensor_held = None
        self.sensor_valid = True

    def attach(self, node):
        self._node = node

    def detach(self):
        """Stop re-stamping the freshness caches — the harness equivalent of a
        publisher going silent, which is how a test drives an UNKNOWN read."""
        self._node = None

    def perf_counter(self):
        return self.t

    def sleep(self, dt):
        self.t += float(dt)
        if self._node is not None:
            _stamp(self._node, self.t)
            if self.sensor_held is not None:
                _feed_sensor(self._node, self.t, held=self.sensor_held,
                             valid=self.sensor_valid)


class _ContGoalHandle:
    def __init__(self, x=0.0, y=0.0, z=170.0, throw_height=0.0, num_throws=3,
                 dwell=DWELL, delay=DELAY, vel_scale=0.0, stop_on_miss=True):
        self.request = types.SimpleNamespace(
            catch_position=types.SimpleNamespace(x=x, y=y, z=z),
            throw_height_m=throw_height, num_throws=num_throws,
            dwell_time_s=dwell, throw_delay_s=delay,
            catch_vel_scale=vel_scale, stop_on_miss=stop_on_miss)
        self.is_cancel_requested = False
        self.feedbacks = []
        self.terminal = None

    def publish_feedback(self, fb):
        self.feedbacks.append((fb.cycle_index, fb.phase, fb.catches_confirmed))

    def succeed(self):
        self.terminal = 'succeed'

    def abort(self):
        self.terminal = 'abort'

    def canceled(self):
        self.terminal = 'canceled'


def _stamp(node, t):
    with node._lock:
        node._mocap_mono = t
        node._balls_mono = t
        node._hand_telemetry_mono = t
        node._hb_mono = t          # bb/heartbeat keeps arriving through a dwell
        node._traj_status_mono = t
        node._commanded_pos_mono = t


def _ready_node(clock, commanded_pos=(0.0, 0.0, 170.0)):
    """Every toss precondition satisfied, caches fresh at the fake clock."""
    node = ReloadCoordinatorNode()
    with node._lock:
        node._control_mode = TOSS_CONTROL_MODE
        node._streaming = True
        node._gravity_correction_loaded = True
        node._commanded_pos_mm = tuple(float(v) for v in commanded_pos)
        node._balls = []
        node._hand_pos_meas = 0.0
        node._hand_vel_meas = 0.0
        node._ball_possession = True
    _stamp(node, clock.t)
    clock.attach(node)
    return node


def _stub_cycles(node, monkeypatch, clock, results, *, verdict_latency=0.30):
    """Replace _run_toss_cycle with a scripted sequence of TossResults, advancing
    the fake clock to the cycle's scheduled landing + a CAUGHT-verdict latency
    (0.30 s sits inside the measured 0.202-0.442 s band). Records the sequencers
    the session built so the caller can assert on them."""
    built = []
    pending = list(results)

    def fake_run(seq, *, deadline_s, cancel_now_fn, feedback_fn):
        built.append(seq)
        if feedback_fn is not None:
            feedback_fn('THROWING')
        clock.t = seq.t_release + float(seq.flight_time_s) + verdict_latency
        _stamp(node, clock.t)
        return pending.pop(0), 'fsm'

    monkeypatch.setattr(node, '_run_toss_cycle', fake_run)
    return built


# ── Wiring surface ────────────────────────────────────────────────────────────

def test_toss_continuous_action_server_registered():
    """A THIRD action server on the same node, so the one-ball-op claim covers
    it: a cross-node session server would reintroduce the publish-latency TOCTOU
    window in which two sequences double-own the hand."""
    from jugglebot_interfaces.action import TossContinuous
    node = ReloadCoordinatorNode()
    assert 'jugglebot/toss_continuous' in node._action_servers
    srv = node._action_servers['jugglebot/toss_continuous']
    assert srv.action_type is TossContinuous
    assert set(node._action_servers) == {
        'jugglebot/reload', 'jugglebot/toss', 'jugglebot/toss_continuous'}


def test_session_shares_the_one_ball_op_claim():
    """S1 at node level: the session claims once for the WHOLE session, so a
    Reload or Toss dispatched mid-session is REJECTED_BUSY. The one-ball-op rule
    extends rather than gaining a session-shaped exception."""
    from rclpy.action import GoalResponse
    node = ReloadCoordinatorNode()
    assert node._goal_callback(object()) == GoalResponse.ACCEPT   # the session
    assert node._goal_callback(object()) == GoalResponse.REJECT   # a Toss
    assert node._goal_callback(object()) == GoalResponse.REJECT   # a Reload


@pytest.mark.parametrize('active', ['reload', 'toss'])
def test_session_rejected_while_another_ball_op_runs(active):
    from rclpy.action import GoalResponse
    node = ReloadCoordinatorNode()
    with node._lock:
        node._active_seq = (ReloadSequencer(catch_point_mm=(0, 0, 809.08))
                            if active == 'reload'
                            else TossSequencer(catch_pose_stow_mm=(0, 0, 170.0)))
    assert node._goal_callback(object()) == GoalResponse.REJECT


def test_claim_released_after_the_session(monkeypatch):
    from rclpy.action import GoalResponse
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    assert node._goal_callback(object()) == GoalResponse.ACCEPT
    _stub_cycles(node, monkeypatch, clock, [TossResult(True, 'CAUGHT', 2.0, .8)])
    node._execute_toss_continuous(_ContGoalHandle(num_throws=1))
    assert node._goal_claimed is False
    assert node._goal_callback(object()) == GoalResponse.ACCEPT


# ── Goal-numerics + session-level rejects (nothing runs) ──────────────────────

@pytest.mark.parametrize('kwargs,field', [
    (dict(throw_height=float('nan')), 'throw_height_m'),
    (dict(delay=float('nan')), 'throw_delay_s'),
    (dict(vel_scale=-0.8), 'catch_vel_scale'),
    (dict(x=float('inf')), 'catch_position.x'),
    (dict(dwell=float('nan')), 'dwell_time_s'),
    (dict(dwell=-1.0), 'dwell_time_s'),
])
def test_bad_goal_numerics_rejected_before_anything_runs(kwargs, field,
                                                         monkeypatch):
    """The six shared numerics route through the SAME gate the single Toss uses
    (so the two actions cannot disagree about a valid toss goal), and
    dwell_time_s gets the same treatment — 0.0 is the only 'use the default'
    sentinel, so a negative is a sign typo."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    monkeypatch.setattr(
        node, '_build_toss_cycle',
        lambda *a, **k: pytest.fail('a cycle was built on a bad goal'))
    gh = _ContGoalHandle(**kwargs)
    result = node._execute_toss_continuous(gh)
    assert result.success is False
    assert result.outcome == 'REJECTED_BAD_GOAL({})'.format(field)
    assert gh.terminal == 'abort'
    assert result.throws_completed == 0
    assert list(result.per_cycle_outcomes) == []
    with node._lock:
        assert node._active_seq is None
        assert node._goal_claimed is False


@pytest.mark.parametrize('kwargs,expected', [
    (dict(num_throws=0), 'REJECTED_NUM_THROWS'),
    (dict(num_throws=-3), 'REJECTED_NUM_THROWS'),
    (dict(num_throws=int(hw.JB_OP_TOSS_SESSION_MAX_THROWS) + 1),
     'REJECTED_NUM_THROWS'),
    (dict(dwell=2.0), 'REJECTED_DWELL'),
    (dict(dwell=DELAY + float(hw.JB_OP_TOSS_SESSION_DWELL_MARGIN_S) - 0.01),
     'REJECTED_DWELL'),
    # A sub-floor throw_delay is refused HERE, not one cycle later as
    # ABORTED_CYCLE_REJECTED_CANT_MAKE_LEAD with a whole cycle's per-goal state
    # already installed. The dwell is legal for the delay, so only the delay
    # gate can catch it.
    (dict(delay=2.0, dwell=4.0), 'REJECTED_THROW_DELAY'),
])
def test_session_checking_rejects_via_execute(kwargs, expected, monkeypatch):
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    monkeypatch.setattr(
        node, '_build_toss_cycle',
        lambda *a, **k: pytest.fail('a cycle was built on a rejected session'))
    gh = _ContGoalHandle(**kwargs)
    result = node._execute_toss_continuous(gh)
    assert result.outcome == expected
    assert result.success is False
    assert gh.terminal == 'abort'


def test_the_session_checking_phase_is_observable_on_the_wire(monkeypatch):
    """SESSION_CHECKING is a documented feedback.phase value, and the FSM leaves
    CHECKING inside the same step() that enters it — so without an explicit
    publish the string exists in the .action, in the module and nowhere on the
    wire, and a GUI waiting for it waits forever. It must appear even on a
    REJECTED session, which is exactly when an operator wants to know the goal
    was received and adjudicated rather than dropped."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    gh = _ContGoalHandle(num_throws=0)
    node._execute_toss_continuous(gh)
    assert (0, 'SESSION_CHECKING', 0) in gh.feedbacks


def test_a_rejected_session_never_reports_success(monkeypatch):
    """Regression: ``success`` was computed from counts alone, so a
    ``num_throws = 0`` goal satisfied ``0 == 0 and 0 == 0`` VACUOUSLY and the
    node called ``goal_handle.succeed()`` on a REJECTED goal. Success now also
    requires the COMPLETED terminal, which closes the class rather than the
    one case."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    gh = _ContGoalHandle(num_throws=0)
    result = node._execute_toss_continuous(gh)
    assert result.outcome == 'REJECTED_NUM_THROWS'
    assert result.success is False
    assert gh.terminal == 'abort'


# ── Chain reachability (Phase E's KNOWN LIMITATION, caught pre-throw) ─────────


@pytest.fixture
def tier_8b(monkeypatch):
    """Pin tier 8b at the seam production reads it.

    `_predicted_chain_site_mm` and `_build_toss_cycle` both resolve
    `hw.JB_OP_TOSS_TIER` per call, and the chain-reachability gate below EXISTS
    only under 8b — at 8a the platform pre-positions LEVEL at B every cycle and
    never reads a throw site, so there is no chain to be unreachable and the
    predictor returns None by design (`test_chain_check_is_skipped_on_tier_8a`).

    8b is a CAPABILITY under test, not the shipped default: the operator flipped
    the shipped tier back to '8a' on 2026-08-10. These tests used to inherit 8b
    from the config, which meant a YAML edit turned all nine of them into
    `assert None is not None` — a config decision wearing a code regression's
    clothes. The pin is what decouples them."""
    monkeypatch.setattr(hw, 'JB_OP_TOSS_TIER', TIER_8B)


@pytest.mark.parametrize('bx,expect_x', [
    (0.0, 0.000), (70.0, 71.448), (140.0, 142.894),
    (146.0, 149.017), (147.0, 150.038), (150.0, 153.100)])
def test_predicted_chain_site_matches_the_production_policy(bx, expect_x,
                                                            monkeypatch,
                                                            tier_8b):
    """The prediction is single-sourced through the SAME
    ``predicted_catch_command`` the deferred A->B reach publishes from, so it
    cannot drift from where the machine will actually be commanded. These values
    are the measured ground truth (probe, 2026-07-29) and reproduce Phase E's
    own -153.10 at the 150 mm cap."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    site = node._predicted_chain_site_mm((bx, 0.0, 170.0), FLIGHT)
    assert site is not None
    assert site[0] == pytest.approx(expect_x, abs=0.01)
    assert site[1] == pytest.approx(0.0, abs=1e-6)


def test_chain_frontier_is_between_146_and_147_mm(monkeypatch, tier_8b):
    """The predictor's frontier is sharp, and against a 150 box (the box == cap
    configuration the 2026-07-29 frontier was MEASURED at) the binding gate is
    the planning box on A, NOT the 150 mm displacement cap (the residual |B-A|
    never exceeds 3.1 mm). The shipped box is now toss_workspace_xy_mm = 160
    (> cap × 1.03), which moves the operative frontier out past the cap — the
    predictor math pinned here is box-independent."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    inside = node._predicted_chain_site_mm((146.5, 0.0, 170.0), FLIGHT)
    outside = node._predicted_chain_site_mm((147.0, 0.0, 170.0), FLIGHT)
    assert abs(inside[0]) <= TOSS_XY_LIMIT_MM
    assert abs(outside[0]) > TOSS_XY_LIMIT_MM
    for bx, site in ((146.5, inside), (147.0, outside)):
        assert math.hypot(bx - site[0], site[1]) < 3.2   # never the cap


def test_chain_unreachable_refuses_before_a_ball_flies(monkeypatch, tier_8b):
    """Without this the session throws ONE ball, catches it, then refuses cycle 2
    REJECTED_WORKSPACE with the platform parked outside the planning box and the
    ball in the cup — actuation for nothing. The refusal moves nothing.

    The box is pinned to 150 (box == cap) for this arm because the 146.5/147.0
    frontier was measured there; at the SHIPPED 160 box the same B = 147 goal
    chains — asserted as the second arm, because admitting exactly this class
    of goal is what the box/cap separation (2026-08-14) exists to do."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    monkeypatch.setattr(hw, 'JB_OP_TOSS_WORKSPACE_XY_MM', 150.0)
    node = _ready_node(clock)
    monkeypatch.setattr(
        node, '_build_toss_cycle',
        lambda *a, **k: pytest.fail('a cycle was built on an unchainable goal'))
    gh = _ContGoalHandle(x=147.0, num_throws=3)
    result = node._execute_toss_continuous(gh)
    assert result.outcome == 'REJECTED_CHAIN_UNREACHABLE'
    assert gh.terminal == 'abort'


def test_chain_at_147_is_admitted_at_the_shipped_box(monkeypatch, tier_8b):
    """The resolution arm of the former known limitation: with the shipped
    toss_workspace_xy_mm (160 > cap × 1.03) the predicted chain site at
    B = 147 (~150.1 mm, 2.07 % centroid divergence) sits INSIDE the box, so
    the session proceeds past the chain gate instead of refusing — chaining at
    the cap edge is the Phase-E working range the wider box exists to admit."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    # num_throws = 2 so the chain gate RUNS (it is skipped for a single cycle)
    # — the point is that it consults the shipped box and finds the site inside.
    _stub_cycles(node, monkeypatch, clock,
                 [TossResult(True, 'CAUGHT', 2.0, .8),
                  TossResult(True, 'CAUGHT', 2.0, .8)])
    result = node._execute_toss_continuous(
        _ContGoalHandle(x=147.0, num_throws=2))
    assert result.outcome == 'COMPLETED'
    site = node._predicted_chain_site_mm((147.0, 0.0, 170.0), FLIGHT)
    assert abs(site[0]) <= float(hw.JB_OP_TOSS_WORKSPACE_XY_MM)


def test_chain_gate_does_not_fire_for_a_single_cycle_session(monkeypatch):
    """num_throws = 1 has no chain: refusing it would deny the operator the very
    throw Phase E validated at the cap."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    _stub_cycles(node, monkeypatch, clock, [TossResult(True, 'CAUGHT', 2.0, .8)])
    result = node._execute_toss_continuous(
        _ContGoalHandle(x=150.0, num_throws=1))
    assert result.outcome == 'COMPLETED'


def test_chain_check_is_skipped_when_the_pose_is_unknown(monkeypatch,
                                                         tier_8b):
    """An unknown live pose is already REJECTED_POSE_UNKNOWN on cycle 1 —
    rejecting CHAIN_UNREACHABLE here instead would send the operator to the wrong
    subsystem. The check declines rather than guessing."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    with node._lock:
        node._commanded_pos_mm = None
    assert node._predicted_chain_site_mm((147.0, 0.0, 170.0), FLIGHT) is None
    result = node._execute_toss_continuous(_ContGoalHandle(x=147.0,
                                                           num_throws=2))
    assert result.outcome == 'ABORTED_CYCLE_REJECTED_POSE_UNKNOWN'


def test_chain_check_is_skipped_on_tier_8a(monkeypatch):
    """Tier 8a pre-positions LEVEL at B every cycle and never reads a throw
    site, so there is no chain to be unreachable."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    monkeypatch.setattr(hw, 'JB_OP_TOSS_TIER', TIER_8A)
    node = _ready_node(clock)
    assert node._predicted_chain_site_mm((147.0, 0.0, 170.0), FLIGHT) is None


# ── The session drives the REAL cycle machinery ──────────────────────────────

def test_every_cycle_goes_through_the_shared_builder(monkeypatch):
    """Cycles are built by _build_toss_cycle — the SAME method the single Toss
    uses — with the session's resolved throw_delay. A second copy of the cycle
    construction is how a session would silently drift from the toss the
    hardware ladder validated."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    calls = []
    real_build = node._build_toss_cycle

    def spy(catch_pose, flight, throw_delay, vel_scale):
        calls.append((catch_pose, flight, throw_delay, vel_scale))
        return real_build(catch_pose, flight, throw_delay, vel_scale)

    monkeypatch.setattr(node, '_build_toss_cycle', spy)
    _stub_cycles(node, monkeypatch, clock,
                 [TossResult(True, 'CAUGHT', 2.0, 0.81)] * 2)
    node._execute_toss_continuous(
        _ContGoalHandle(num_throws=2, delay=DELAY, vel_scale=0.9))
    assert len(calls) == 2
    for pose, flight, delay, scale in calls:
        assert pose == (0.0, 0.0, 170.0)
        assert flight == pytest.approx(float(hw.JB_OP_TOSS_FLIGHT_TIME_DEFAULT_S))
        assert delay == pytest.approx(DELAY)
        assert scale == pytest.approx(0.9)


def test_a_real_cycle_reject_terminates_the_session(monkeypatch):
    """End to end through the REAL _build_toss_cycle AND the REAL
    _run_toss_cycle: a cycle that faults carries its own verdict verbatim into
    the session outcome, so a failure routes exactly where a single Toss would
    have routed it."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    node._control_mode = 'STANDBY'          # first-tick REJECTED_WRONG_MODE
    gh = _ContGoalHandle(num_throws=4)
    result = node._execute_toss_continuous(gh)
    assert result.outcome == 'ABORTED_CYCLE_REJECTED_WRONG_MODE'
    assert result.success is False
    assert list(result.per_cycle_outcomes) == ['REJECTED_WRONG_MODE']
    assert result.throws_completed == 0     # nothing flew
    assert gh.terminal == 'abort'


def test_cycle_state_is_cleared_between_cycles_but_possession_survives(
        monkeypatch):
    """Per-cycle node state must not leak into the next cycle (a stale landing
    reference would judge the next catch against the wrong point), but the
    possession latch MUST survive — the caught ball is still in the cup, and
    clearing it would make cycle N+1 refuse NO_BALL under a ball-evidence gate."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    seen = []

    def fake_run(seq, *, deadline_s, cancel_now_fn, feedback_fn):
        with node._lock:
            seen.append((node._toss_release_state is not None,
                         node._toss_landing_global_mm is not None))
        clock.t = seq.t_release + float(seq.flight_time_s) + 0.3
        _stamp(node, clock.t)
        return TossResult(True, 'CAUGHT', 2.0, 0.81), 'fsm'

    monkeypatch.setattr(node, '_run_toss_cycle', fake_run)
    node._execute_toss_continuous(_ContGoalHandle(num_throws=2))
    assert seen == [(True, True), (True, True)]   # installed for every cycle
    with node._lock:
        assert node._toss_release_state is None       # …and torn down after
        assert node._toss_landing_global_mm is None
        assert node._active_seq is None
        assert node._ball_possession is True          # the ball is still there


# ── Scheduling + accounting through the node ─────────────────────────────────

def test_session_completes_and_reports_per_cycle_evidence(monkeypatch):
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    built = _stub_cycles(node, monkeypatch, clock, [
        TossResult(True, 'CAUGHT', 3.0, 0.805),
        TossResult(True, 'CAUGHT', 5.0, 0.812),
        TossResult(True, 'CAUGHT', 4.0, 0.799)])
    gh = _ContGoalHandle(num_throws=3, dwell=DWELL, delay=DELAY)
    result = node._execute_toss_continuous(gh)
    assert result.outcome == 'COMPLETED' and result.success is True
    assert result.throws_completed == 3 and result.catches_confirmed == 3
    assert list(result.per_cycle_outcomes) == ['CAUGHT'] * 3
    assert list(result.per_cycle_catch_error_mm) == pytest.approx([3.0, 5.0, 4.0])
    assert list(result.per_cycle_flight_s) == pytest.approx(
        [0.805, 0.812, 0.799])
    assert math.isnan(result.per_cycle_dwell_s[0])
    # The achieved dwell lands on the request within one node tick: the fake
    # clock only advances in _TICK_S steps, so the wait overshoots by < 50 ms.
    for achieved in result.per_cycle_dwell_s[1:]:
        assert DWELL <= achieved < DWELL + rcn._TICK_S
    assert len(built) == 3
    assert gh.terminal == 'succeed'


def test_stop_on_miss_default_true_stops_after_the_first_miss(monkeypatch):
    """Operator decision (c): an omitted stop_on_miss must STOP. The goal handle
    here carries the wire default."""
    from jugglebot_interfaces.action import TossContinuous
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    built = _stub_cycles(node, monkeypatch, clock, [
        TossResult(True, 'CAUGHT', 3.0, 0.805),
        TossResult(False, 'MISSED', float('nan'), float('nan')),
        TossResult(True, 'CAUGHT', 3.0, 0.805)])
    gh = _ContGoalHandle(num_throws=3)
    gh.request.stop_on_miss = TossContinuous.Goal().stop_on_miss
    result = node._execute_toss_continuous(gh)
    assert result.outcome == 'STOPPED_ON_MISS'
    assert result.success is False
    assert len(built) == 2                       # the third never started
    assert result.throws_completed == 2 and result.catches_confirmed == 1
    assert gh.terminal == 'abort'


def test_stop_on_miss_false_runs_every_cycle(monkeypatch):
    """The bench affordance the no-ball dry trace needs: with a ball-evidence
    gate off, every cycle fires an empty stroke and MISSES, so reaching cycle 3
    requires this flag."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    built = _stub_cycles(node, monkeypatch, clock, [
        TossResult(False, 'MISSED', float('nan'), float('nan'))] * 3)
    result = node._execute_toss_continuous(
        _ContGoalHandle(num_throws=3, stop_on_miss=False))
    assert result.outcome == 'COMPLETED'
    assert result.success is False
    assert result.throws_completed == 3 and result.catches_confirmed == 0
    assert len(built) == 3


def test_feedback_carries_cycle_index_phase_and_running_catches(monkeypatch):
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    _stub_cycles(node, monkeypatch, clock,
                 [TossResult(True, 'CAUGHT', 2.0, 0.8)] * 2)
    gh = _ContGoalHandle(num_throws=2)
    node._execute_toss_continuous(gh)
    # The cycle's own Toss phase is reported verbatim while a cycle runs…
    assert (1, 'THROWING', 0) in gh.feedbacks
    assert (2, 'THROWING', 1) in gh.feedbacks     # running count is live
    # …and the session's own phase between cycles.
    assert any(phase == 'DWELL' for _idx, phase, _n in gh.feedbacks)


# ── Node-level exits ─────────────────────────────────────────────────────────

def test_cancel_between_cycles_is_honoured_and_commands_nothing(monkeypatch):
    """S2 + S4: a cancel during DWELL is honoured immediately — nothing is armed
    and nothing is airborne, the previous cycle's own terminal already left the
    machine where it belongs — and the session issues NO motion of its own."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    moved = []
    for name in ('_go_home', '_safe_abort', '_recenter', '_toss_safe_abort',
                 '_toss_stay', '_toss_recenter', '_retract_hand_with_retries',
                 '_prime_hand_with_retries', '_position_platform_for_toss'):
        monkeypatch.setattr(node, name,
                            lambda *a, _n=name, **k: moved.append(_n))
    gh = _ContGoalHandle(num_throws=3)

    def fake_run(seq, *, deadline_s, cancel_now_fn, feedback_fn):
        clock.t = seq.t_release + float(seq.flight_time_s) + 0.3
        _stamp(node, clock.t)
        gh.is_cancel_requested = True          # cancel arrives after cycle 1
        return TossResult(True, 'CAUGHT', 2.0, 0.81), 'fsm'

    monkeypatch.setattr(node, '_run_toss_cycle', fake_run)
    result = node._execute_toss_continuous(gh)
    assert result.outcome == 'ABORTED_CANCELLED'
    assert gh.terminal == 'canceled'
    assert result.throws_completed == 1 and result.catches_confirmed == 1
    assert list(result.per_cycle_outcomes) == ['CAUGHT']   # evidence preserved
    assert moved == []                                     # S2: nothing commanded


def test_cancel_inside_a_cycle_ends_the_session_with_its_accounting(monkeypatch):
    """A cancel adjudicated INSIDE a cycle (per the single-toss phase rules,
    which this action does not modify) ends the session too — with everything
    scored so far preserved rather than an empty result."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    outcomes = [(TossResult(True, 'CAUGHT', 2.0, 0.81), 'fsm'),
                (TossResult(False, 'ABORTED_CANCELLED'), 'cancel')]

    def fake_run(seq, *, deadline_s, cancel_now_fn, feedback_fn):
        clock.t = seq.t_release + float(seq.flight_time_s) + 0.3
        _stamp(node, clock.t)
        return outcomes.pop(0)

    monkeypatch.setattr(node, '_run_toss_cycle', fake_run)
    gh = _ContGoalHandle(num_throws=4)
    result = node._execute_toss_continuous(gh)
    assert result.outcome == 'ABORTED_CANCELLED'
    assert gh.terminal == 'canceled'
    assert result.throws_completed == 1 and result.catches_confirmed == 1
    assert list(result.per_cycle_outcomes) == ['CAUGHT', 'ABORTED_CANCELLED']


@pytest.mark.parametrize('exit_kind,outcome,terminal', [
    ('timeout', 'ABORTED_TIMEOUT', 'abort'),
    # SHUTDOWN terminalises NOTHING on the handle — the single Toss does the
    # same. rclpy is tearing down, and a goal-status transition on a dying
    # executor can itself raise, replacing a clean shutdown with a spurious
    # ABORTED_EXCEPTION.
    ('shutdown', 'ABORTED_SHUTDOWN', None)])
def test_cycle_level_node_exits_end_the_session(exit_kind, outcome, terminal,
                                                monkeypatch):
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)

    def fake_run(seq, *, deadline_s, cancel_now_fn, feedback_fn):
        clock.t = seq.t_release + float(seq.flight_time_s) + 0.3
        _stamp(node, clock.t)
        return TossResult(False, 'ABORTED_' + exit_kind.upper()), exit_kind

    monkeypatch.setattr(node, '_run_toss_cycle', fake_run)
    gh = _ContGoalHandle(num_throws=3)
    result = node._execute_toss_continuous(gh)
    assert result.outcome == outcome
    assert gh.terminal == terminal


def test_session_exception_preserves_accounting_and_reraises(monkeypatch):
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    calls = {'n': 0}

    def fake_run(seq, *, deadline_s, cancel_now_fn, feedback_fn):
        calls['n'] += 1
        clock.t = seq.t_release + float(seq.flight_time_s) + 0.3
        _stamp(node, clock.t)
        if calls['n'] == 2:
            raise RuntimeError('boom')
        return TossResult(True, 'CAUGHT', 2.0, 0.81), 'fsm'

    monkeypatch.setattr(node, '_run_toss_cycle', fake_run)
    logged = []
    monkeypatch.setattr(node, '_log_toss_session_outcome',
                        lambda r: logged.append(str(r.outcome)))
    gh = _ContGoalHandle(num_throws=3)
    with pytest.raises(RuntimeError, match='boom'):
        node._execute_toss_continuous(gh)
    assert logged == ['ABORTED_EXCEPTION']
    assert gh.terminal == 'abort'
    with node._lock:
        assert node._goal_claimed is False
        assert node._active_seq is None


# ── Deadlines ────────────────────────────────────────────────────────────────

def test_session_deadline_never_lands_inside_a_legitimate_session():
    """The session timeout path is only reachable BETWEEN cycles (each cycle
    carries its own ceiling), but it must still never fire inside a healthy
    session: a premature terminal would abandon a machine mid-session."""
    session = TossSessionSequencer(num_throws=5, dwell_time_s=8.0,
                                   throw_delay_s=5.0, flight_time_s=0.8)
    per_cycle = 30.0
    ceiling = _toss_session_deadline_s(session, per_cycle)
    honest_worst = 5 * (per_cycle + 8.0)
    assert ceiling >= honest_worst
    assert ceiling >= rcn._MAX_SEQUENCE_S


def test_session_deadline_is_at_least_the_single_sequence_floor():
    session = TossSessionSequencer(num_throws=1, dwell_time_s=6.0,
                                   throw_delay_s=5.0)
    assert _toss_session_deadline_s(session, 1.0) >= rcn._MAX_SEQUENCE_S


# ══ 2d — the auto-reload interlude, Layer 1.5, and the reopened retry ═════════
#
# The FSM half (budget/floor counters, the on_empty_cup whitelist, the
# consecutive-NO_RELEASE gauge) is in test_toss_session.py. Here we test the
# node's seams onto the graph: the observation-driven gate rungs, the VERIFIED
# recentre, the reload attempt loop with its targeted BB retry, and the
# structural confinement of the Layer-1.5 reads to the quiescent dwell.

import jugglebot.toss_session as ts
from jugglebot.reload_sequencer import (
    BB_STATE_ERROR,
    BB_STATE_IDLE,
    BB_STATE_RELOADING,
    ReloadResult,
)


class _Hb:
    def __init__(self, connected=True, state=BB_STATE_IDLE, ball_in_hand=True):
        self.connected = connected
        self.state = state
        self.ball_in_hand = ball_in_hand


def _feed_sensor(node, t, held=False, valid=True):
    """Drive the hand ball sensor to a definite state at synthetic time ``t``.

    TWO samples, for the reason test_toss_coordinator._feed_ball_sensor
    documents: these tests jump the clock, so one sample lands on the far side of
    a blind window and reads UNKNOWN. Deliberately produces no edge."""
    node._ball_sensor.note_sample(float(t) - 0.01, held=bool(held),
                                  valid=bool(valid))
    node._ball_sensor.note_sample(float(t), held=bool(held), valid=bool(valid))


def _reload_ready_node(clock, *, commanded_pos=(0.0, 0.0, 170.0),
                       hb=None, cup_held=False, bb_verified=True):
    """A node with every INTERLUDE precondition satisfied, so each test can break
    exactly one rung and assert the code that names it."""
    node = _ready_node(clock, commanded_pos=commanded_pos)
    with node._lock:
        node._hb = hb if hb is not None else _Hb()
        node._hb_mono = clock.t
        node._bb_ball_in_hand_observed_false = bool(bb_verified)
    clock.sensor_held = bool(cup_held)
    _feed_sensor(node, clock.t, held=cup_held, valid=True)
    return node


def _session(**kw):
    params = dict(num_throws=5, dwell_time_s=DWELL, throw_delay_s=DELAY,
                  stop_on_miss=False, on_empty_cup=ts.ON_EMPTY_CUP_RELOAD,
                  max_reloads=3)
    params.update(kw)
    s = TossSessionSequencer(**params)
    s.start(0.0)
    return s


def _no_ball():
    return TossResult(False, 'REJECTED_NO_BALL', float('nan'), float('nan'))


def _no_release():
    return TossResult(False, 'ABORTED_NO_RELEASE', float('nan'), float('nan'))


# ── The precondition gate: one code per rung, and NOTHING moves ──────────────

def test_gate_passes_when_every_rung_holds(monkeypatch):
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    assert node._reload_interlude_gate(_session()) is None


def test_gate_refuses_when_ball_evidence_is_disabled(monkeypatch):
    """The RUNTIME prerequisite, checked live rather than assumed from the
    landed config. With toss_require_ball_evidence false, CHECKING passes on an
    empty cup, a drop produces a silent empty dry stroke, and the session would
    be 'reloading' around tosses that never happened — so the operator's
    total-bypass escape hatch must not silently re-open that path."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    monkeypatch.setattr(rcn.hw, 'JB_OP_TOSS_REQUIRE_BALL_EVIDENCE', False)
    node = _reload_ready_node(clock)
    assert node._reload_interlude_gate(_session()) \
        == 'STOPPED_BALL_EVIDENCE_DISABLED'


@pytest.mark.parametrize('hb', [
    _Hb(connected=False),
    _Hb(state=BB_STATE_RELOADING),
    _Hb(state=BB_STATE_ERROR),
    None,                       # no heartbeat at all
])
def test_gate_refuses_when_bb_is_not_ready(hb, monkeypatch):
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    with node._lock:
        node._hb = hb
    assert node._reload_interlude_gate(_session()) == 'STOPPED_BB_NOT_READY'


def test_gate_refuses_a_stale_heartbeat_as_not_ready(monkeypatch):
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    with node._lock:
        node._hb_mono = clock.t - 10.0
    assert node._reload_interlude_gate(_session()) == 'STOPPED_BB_NOT_READY'


def test_gate_refuses_until_bb_ball_in_hand_has_been_seen_false(monkeypatch):
    """The CONSUMER-side fence on BallButler's fail-open boot default. A
    freshly-rebooted BB heartbeats ball_in_hand TRUE before its first GPIO read,
    which makes the reload FSM SKIP ACTION_CALL_RELOAD: it primes the hand,
    raises the latch, throws at an empty BB and dies ABORTED_NO_ANNOUNCEMENT
    having armed everything for nothing. Inside an autonomous session nobody is
    watching the heartbeat."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock, bb_verified=False)
    assert node._reload_interlude_gate(_session()) == 'STOPPED_BB_UNVERIFIED'


def test_a_false_ball_in_hand_heartbeat_latches_the_fence_open(monkeypatch):
    """It answers 'has this BB's GPIO ever spoken?', which is a property of the
    BB process — so it latches on the first false and never clears."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock, bb_verified=False)
    node._on_heartbeat(_Hb(ball_in_hand=False))
    assert node._bb_ball_in_hand_observed_false is True
    node._on_heartbeat(_Hb(ball_in_hand=True))
    assert node._bb_ball_in_hand_observed_false is True


def test_gate_refuses_an_unknown_cup(monkeypatch):
    """A dead sensor must not license an autonomous BB throw at a cup nobody can
    see into — the fail-open default this project declined to copy."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    _feed_sensor(node, clock.t, held=False, valid=False)
    assert node._reload_interlude_gate(_session()) == 'STOPPED_SENSOR_UNKNOWN'


def test_gate_refuses_a_cup_that_is_not_empty(monkeypatch):
    """The interlude was entered because the cup read EMPTY a moment ago, so
    SEATED here is a CONTRADICTION — and reloading onto a loaded cup throws a
    real ball at a hand that already holds one. § 3.9 names one sensor code;
    splitting it is deliberate and is in the fail-closed direction."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock, cup_held=True)
    assert node._reload_interlude_gate(_session()) == 'STOPPED_CUP_NOT_EMPTY'


def test_a_refused_gate_moves_nothing(monkeypatch):
    """Every refusal happens BEFORE the recentre and before a reload sequencer
    exists, so the machine is left exactly where the REJECTED_NO_BALL cycle left
    it — which is quiescent, because that terminal's action is ACTION_NONE."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock, bb_verified=False)
    for name in ('_go_home', '_call_reload', '_prime_hand_with_retries',
                 '_arm_catch', '_send_throw', '_smooth_move_hand'):
        monkeypatch.setattr(node, name,
                            lambda *a, **k: pytest.fail(
                                '{} ran on a refused gate'.format(name)))
    session = _session()
    ok, code, attempts = node._run_reload_interlude(session)
    assert (ok, code, attempts) == (False, 'STOPPED_BB_UNVERIFIED', 0)


# ── Rung 2: go_home + VERIFIED arrival ───────────────────────────────────────

def test_the_interlude_cannot_be_entered_from_an_off_centre_park(monkeypatch):
    """THE gate this rung exists for. reload_sequencer refuses an off-centre park
    (REJECTED_NOT_CENTERED) because the reload never pre-positions: from off
    centre it arms a reach envelope centred off (0,0) and rejects the incoming BB
    ball MID-FLIGHT, unsavable. With toss_stay_at_pose_on_caught true, 'parked
    150 mm off centre' is the ROUTINE state after a CAUGHT cycle — so a session
    that skipped the verification would meet that failure on almost every drop.

    Here go_home is ACKed but the platform never arrives: the verification must
    time out into STOPPED_RECENTRE_FAILED and must NOT attempt a reload."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock, commanded_pos=(150.0, 0.0, 170.0))
    monkeypatch.setattr(node, '_go_home', lambda: True)
    monkeypatch.setattr(
        node, '_run_one_reload_attempt',
        lambda **k: pytest.fail('a reload was attempted from an unverified pose'))
    ok, code, attempts = node._run_reload_interlude(_session())
    assert (ok, code, attempts) == (False, 'STOPPED_RECENTRE_FAILED', 0)


def test_a_failed_go_home_dispatch_never_becomes_a_reload_attempt(monkeypatch):
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    monkeypatch.setattr(node, '_go_home', lambda: False)
    monkeypatch.setattr(
        node, '_run_one_reload_attempt',
        lambda **k: pytest.fail('a reload was attempted after a failed go_home'))
    assert node._run_reload_interlude(_session())[1] == 'STOPPED_RECENTRE_FAILED'


def test_the_recentre_waits_the_whole_profile_before_believing_the_position(
        monkeypatch):
    """_go_home() returns on the service ACK at plan-INSTALL, not on arrival — the
    same trap the MISS-cleanup floor documents. A platform that is ALREADY at
    centre must therefore still not short-circuit the wait, because 'centred' one
    tick after the install is the pose it is about to leave."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    monkeypatch.setattr(node, '_go_home', lambda: True)
    t0 = clock.t
    assert node._recentre_for_reload() is True
    assert clock.t - t0 >= rcn.GO_HOME_DURATION_S


def test_a_stale_commanded_position_reads_as_not_centred(monkeypatch):
    """UNKNOWN is not centre. The freshness gate fails closed, so a dead
    trajectory link times the verification out instead of licensing a throw at a
    platform whose pose nobody knows."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    monkeypatch.setattr(node, '_go_home', lambda: True)
    clock.detach()      # stop re-stamping the caches, so they go stale
    assert node._recentre_for_reload() is False


# ── Rung 3: the reload attempts, and the ONE targeted BB retry ───────────────

def _stub_attempts(node, monkeypatch, outcomes, *, bb_codes=None):
    """Script _run_one_reload_attempt with a sequence of ReloadResults, and
    optionally the BB throw outcome text each attempt leaves behind."""
    calls = []
    pending = list(outcomes)
    codes = list(bb_codes or [])

    def fake(**kw):
        idx = len(calls)
        calls.append(kw)
        if idx < len(codes) and codes[idx]:
            node._on_bb_throw_outcome(types.SimpleNamespace(data=codes[idx]))
        return pending.pop(0)

    monkeypatch.setattr(node, '_run_one_reload_attempt', fake)
    monkeypatch.setattr(node, '_go_home', lambda: True)
    return calls


def test_a_caught_reload_succeeds_on_the_first_attempt(monkeypatch):
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    calls = _stub_attempts(node, monkeypatch, [ReloadResult(True, 'CAUGHT')])
    ok, code, attempts = node._run_reload_interlude(_session())
    assert (ok, code, attempts) == (True, None, 1)
    assert len(calls) == 1


def test_the_bb_not_settled_abort_is_retried_within_budget(monkeypatch):
    """The ONE identifiable BB-side defect: BallButler was not positioned in
    time, so no ball ever left it. bb/throw_at_target is fire-and-forget, so this
    code reaches us only on bb/throw_outcome — and without it the failure looks
    exactly like an ordinary MISSED."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    calls = _stub_attempts(
        node, monkeypatch,
        [ReloadResult(False, 'MISSED'), ReloadResult(True, 'CAUGHT')],
        bb_codes=['THROW_ABORTED_NOT_SETTLED (axis=YAW, detail1=0)', ''])
    ok, code, attempts = node._run_reload_interlude(_session(max_reloads=3))
    assert (ok, code, attempts) == (True, None, 2)
    assert len(calls) == 2


def test_the_retry_is_targeted_at_that_code_and_nothing_else(monkeypatch):
    """A blanket 'retry any failed reload' would swallow the BB fail-open boot
    bug and every real BB fault, and would keep asking an unwell machine to throw
    real balls. Every other terminal stops the session by name."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    for bb_code in ('', 'THROW_REJECTED_PV_STALE (axis=YAW, detail1=0)',
                    'TIMEOUT (axis=n/a, detail1=0)'):
        node = _reload_ready_node(clock)
        calls = _stub_attempts(node, monkeypatch,
                               [ReloadResult(False, 'MISSED')],
                               bb_codes=[bb_code])
        ok, code, attempts = node._run_reload_interlude(_session(max_reloads=3))
        assert ok is False and attempts == 1, bb_code
        assert code == 'STOPPED_RELOAD_MISSED', bb_code
        assert len(calls) == 1, bb_code


def test_a_not_settled_run_that_exhausts_the_budget_stops_with_the_budget_code(
        monkeypatch):
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    _stub_attempts(
        node, monkeypatch,
        [ReloadResult(False, 'MISSED'), ReloadResult(False, 'MISSED')],
        bb_codes=['THROW_ABORTED_NOT_SETTLED (axis=YAW, detail1=0)',
                  'THROW_ABORTED_NOT_SETTLED (axis=YAW, detail1=0)'])
    ok, code, attempts = node._run_reload_interlude(_session(max_reloads=2))
    assert (ok, code, attempts) == (False, rcn.OUTCOME_STOPPED_RELOAD_BUDGET, 2)


def test_a_stale_bb_outcome_cannot_license_a_retry(monkeypatch):
    """A NOT_SETTLED from a PREVIOUS attempt must not authorise retrying a reload
    that failed for some other reason — so the cache is only consulted for
    outcomes stamped after this attempt began."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    node._on_bb_throw_outcome(
        types.SimpleNamespace(data='THROW_ABORTED_NOT_SETTLED (axis=YAW)'))
    clock.sleep(1.0)                       # the attempt starts AFTER that stamp
    _stub_attempts(node, monkeypatch, [ReloadResult(False, 'MISSED')])
    ok, code, _ = node._run_reload_interlude(_session(max_reloads=3))
    assert (ok, code) == (False, 'STOPPED_RELOAD_MISSED')


def test_the_retried_attempts_are_charged_to_the_session_budget(monkeypatch):
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    _stub_attempts(
        node, monkeypatch,
        [ReloadResult(False, 'MISSED'), ReloadResult(True, 'CAUGHT')],
        bb_codes=['THROW_ABORTED_NOT_SETTLED (axis=YAW)', ''])
    session = _session(max_reloads=3)
    session.step(0.0)
    session.note_cycle_result(_no_ball(), DELAY, DELAY + FLIGHT)
    session.step(DELAY)                    # -> SESSION_ACTION_RELOAD
    ok, code, attempts = node._run_reload_interlude(session)
    session.note_reload_result(ok, attempts=attempts, stop_code=code)
    assert session.reloads_used == 2 and session.reload_budget_remaining == 1


def test_the_reload_attempt_reuses_the_shipping_fsm_and_step(monkeypatch):
    """The interlude invents no motion primitive: the attempt drives the SAME
    ReloadSequencer through the SAME _step_sequence the shipping Reload action
    drives, so a rung added to the reload ladder lands in one place."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    seen = []

    def fake_step(seq, now, goal_handle=None):
        seen.append((type(seq).__name__, goal_handle))
        return types.SimpleNamespace(
            done=True, result=ReloadResult(True, 'CAUGHT'), action='none',
            phase='SETTLING')

    monkeypatch.setattr(node, '_step_sequence', fake_step)
    result = node._run_one_reload_attempt()
    assert result.outcome == 'CAUGHT'
    assert seen and seen[0][0] == 'ReloadSequencer'
    assert seen[0][1] is None            # a session's interlude owns no handle
    with node._lock:
        assert node._active_seq is None  # always torn down


def test_the_reload_attempt_settles_before_handing_back(monkeypatch):
    """Rung 4. The reload's terminal action dispatches on SERVICE ACKS, so it
    returns while the go_home profile is still traversing — the same trap the
    MISS path already carries a floor for, and the same constant."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    monkeypatch.setattr(
        node, '_step_sequence',
        lambda seq, now, gh=None: types.SimpleNamespace(
            done=True, result=ReloadResult(True, 'CAUGHT'), action='none',
            phase='SETTLING'))
    t0 = clock.t
    node._run_one_reload_attempt()
    assert clock.t - t0 >= rcn.DEFAULT_SESSION_MISS_CLEANUP_S


# ── End to end through the real execute path ─────────────────────────────────

def test_an_omitted_on_empty_cup_stops_the_session(monkeypatch):
    """The pre-2026-08-11 behaviour, unchanged: a goal that did not ask for
    reloads stops on REJECTED_NO_BALL verbatim and never runs an interlude."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    _stub_cycles(node, monkeypatch, clock, [_no_ball()])
    monkeypatch.setattr(
        node, '_run_reload_interlude',
        lambda *a, **k: pytest.fail('an interlude ran on a STOP session'))
    gh = _ContGoalHandle(num_throws=3, stop_on_miss=False)
    assert not hasattr(gh.request, 'on_empty_cup')     # the field is OMITTED
    result = node._execute_toss_continuous(gh)
    assert result.outcome == 'ABORTED_CYCLE_REJECTED_NO_BALL'
    assert result.reloads_used == 0


def test_a_reload_session_runs_the_interlude_and_reports_the_budget(monkeypatch):
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    _stub_cycles(node, monkeypatch, clock,
                 [_no_ball(), TossResult(True, 'CAUGHT', 3.0, 0.8)])
    monkeypatch.setattr(node, '_run_reload_interlude',
                        lambda *a, **k: (True, None, 1))
    gh = _ContGoalHandle(num_throws=1, stop_on_miss=False)
    gh.request.on_empty_cup = 'RELOAD'
    gh.request.max_reloads = 0
    result = node._execute_toss_continuous(gh)
    assert result.outcome == 'COMPLETED'
    assert result.success is True
    assert result.reloads_used == 1
    assert list(result.per_cycle_outcomes) == ['REJECTED_NO_BALL', 'CAUGHT']


def test_a_refused_interlude_terminalises_the_session_with_its_code(monkeypatch):
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    _stub_cycles(node, monkeypatch, clock, [_no_ball()])
    monkeypatch.setattr(node, '_run_reload_interlude',
                        lambda *a, **k: (False, 'STOPPED_BB_NOT_READY', 0))
    gh = _ContGoalHandle(num_throws=3, stop_on_miss=False)
    gh.request.on_empty_cup = 'RELOAD'
    result = node._execute_toss_continuous(gh)
    assert result.outcome == 'STOPPED_BB_NOT_READY'
    assert gh.terminal == 'abort'
    assert node._goal_claimed is False


def test_the_reload_phase_is_published_before_the_interlude_runs(monkeypatch):
    """An operator watching feedback must be able to tell 'the session is
    reloading' from 'the session has stalled in a dwell'."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    _stub_cycles(node, monkeypatch, clock, [_no_ball()])
    monkeypatch.setattr(node, '_run_reload_interlude',
                        lambda *a, **k: (False, 'STOPPED_BB_NOT_READY', 0))
    gh = _ContGoalHandle(num_throws=3, stop_on_miss=False)
    gh.request.on_empty_cup = 'RELOAD'
    node._execute_toss_continuous(gh)
    assert any(phase == ts.SESSION_PHASE_RELOAD
               for _idx, phase, _c in gh.feedbacks)


@pytest.mark.parametrize('max_reloads', [-1, -3])
def test_a_negative_max_reloads_is_refused_by_name(max_reloads, monkeypatch):
    """0 is the only 'use the config default' sentinel, so a negative is a sign
    typo. Coercing it would silently substitute a 3-ball budget for something the
    operator asked for and the machine cannot represent."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    monkeypatch.setattr(
        node, '_build_toss_cycle',
        lambda *a, **k: pytest.fail('a cycle was built on a bad goal'))
    gh = _ContGoalHandle(num_throws=3)
    gh.request.max_reloads = max_reloads
    result = node._execute_toss_continuous(gh)
    assert result.outcome == 'REJECTED_BAD_GOAL(max_reloads)'


def test_the_session_ceiling_makes_room_for_the_reload_budget():
    """The session ceiling's exit path is an ABORT, so a session that actually
    spends the budget it was given must not trip it for doing what it was
    asked."""
    stop = TossSessionSequencer(num_throws=5, dwell_time_s=DWELL,
                                throw_delay_s=DELAY)
    reload_ = TossSessionSequencer(num_throws=5, dwell_time_s=DWELL,
                                   throw_delay_s=DELAY,
                                   on_empty_cup=ts.ON_EMPTY_CUP_RELOAD,
                                   max_reloads=3)
    assert rcn._reload_interlude_budget_s(stop) == 0.0
    assert rcn._reload_interlude_budget_s(reload_) > 0.0
    base = _toss_session_deadline_s(stop, 30.0)
    assert _toss_session_deadline_s(
        reload_, 30.0,
        reload_budget_s=rcn._reload_interlude_budget_s(reload_)) > base


# ── The reopened ABORTED_NO_RELEASE retry, through the node ──────────────────

@pytest.mark.parametrize('held,valid,retried', [
    (True, True, True),         # valid HELD  -> the ball is demonstrably in the cup
    (False, True, False),       # valid EMPTY -> it went somewhere nobody watched
    (True, False, False),       # UNKNOWN     -> blindness is not evidence
])
def test_no_release_retry_is_gated_on_the_live_cup_read(held, valid, retried,
                                                        monkeypatch):
    """The node reads the cup at the CYCLE'S TERMINAL and passes it in, so the
    decision is made on the cup the retry would stroke over rather than on an
    earlier belief."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    built = _stub_cycles(node, monkeypatch, clock,
                         [_no_release(), TossResult(True, 'CAUGHT', 2.0, 0.8)])

    orig = node._run_toss_cycle

    def run_then_set_sensor(seq, **kw):
        out = orig(seq, **kw)
        _feed_sensor(node, clock.t, held=held, valid=valid)
        return out

    monkeypatch.setattr(node, '_run_toss_cycle', run_then_set_sensor)
    # num_throws = 1: completion is keyed on THROWS, so the retried cycle's
    # CAUGHT is the session's one and only data point.
    gh = _ContGoalHandle(num_throws=1, stop_on_miss=False)
    result = node._execute_toss_continuous(gh)
    if retried:
        assert len(built) == 2
        assert list(result.per_cycle_outcomes) == ['ABORTED_NO_RELEASE',
                                                   'CAUGHT']
    else:
        assert len(built) == 1
        assert result.outcome == 'ABORTED_CYCLE_ABORTED_NO_RELEASE'


def test_a_retried_cycle_names_what_it_retried(monkeypatch):
    """Guard G11: the retried cycle carries a back-reference to the uid it
    retried, so a fit can exclude the pair rather than silently double-count the
    same intended toss."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    _stub_cycles(node, monkeypatch, clock,
                 [_no_release(), TossResult(True, 'CAUGHT', 2.0, 0.8)])
    orig = node._run_toss_cycle

    def run_then_hold(seq, **kw):
        out = orig(seq, **kw)
        _feed_sensor(node, clock.t, held=True, valid=True)
        return out

    monkeypatch.setattr(node, '_run_toss_cycle', run_then_hold)
    node._execute_toss_continuous(_ContGoalHandle(num_throws=1,
                                                  stop_on_miss=False))
    ctx = node._toss_record_ctx
    assert ctx['retry_of'] is not None
    assert ctx['retry_of'].endswith('-1')      # cycle 1's uid
    assert ctx['uid'].endswith('-2')


def test_the_cycle_after_a_reload_is_flagged_for_exclusion(monkeypatch):
    """Guard G10: a just-recentred platform holding a just-delivered ball is not
    the steady state the map is fitted from."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    _stub_cycles(node, monkeypatch, clock,
                 [_no_ball(), TossResult(True, 'CAUGHT', 3.0, 0.8)])
    monkeypatch.setattr(node, '_run_reload_interlude',
                        lambda *a, **k: (True, None, 1))
    gh = _ContGoalHandle(num_throws=1, stop_on_miss=False)
    gh.request.on_empty_cup = 'RELOAD'
    node._execute_toss_continuous(gh)
    assert node._toss_record_ctx['reload_settle'] is True


# ── Layer 1.5 — the dwell inclinometer covariate ─────────────────────────────

def test_dwell_tilt_reads_have_exactly_one_call_site_and_it_is_the_dwell():
    """THE structural gate (§ 3.10 rule 1: reads never overlap PREPARE→THROW).

    It is structural rather than a runtime check: _run_toss_cycle BLOCKS the
    session loop for a cycle's whole life, so no iteration — and therefore no
    read — can happen between PREPARE and THROW. This test pins that there is
    exactly ONE call site and that it sits in the outer loop's quiescent branch,
    because a second call site added inside the cycle path would be invisible to
    every behavioural test."""
    src = (Path(rcn.__file__).with_suffix('.py')).read_text()
    assert src.count('self._maybe_read_dwell_tilt(') == 1
    call_at = src.index('self._maybe_read_dwell_tilt(')
    guard = src[max(0, call_at - 500):call_at]
    assert 'SESSION_PHASE_DWELL' in guard
    assert 'not session.cycle_live' in guard
    # and the READ itself is never reachable from the toss FSM's own tick
    body_start = src.index('def _step_toss_sequence(')
    body_end = src.index('def _position_platform_for_toss(')
    assert '_maybe_read_dwell_tilt' not in src[body_start:body_end]


def test_dwell_reads_accumulate_during_a_quiescent_dwell(monkeypatch):
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    monkeypatch.setattr(node, '_read_platform_tilt', lambda: (0.001, -0.002))
    session = _session(num_throws=3)
    session._next_cycle_at = clock.t + 10.0
    for _ in range(200):
        node._maybe_read_dwell_tilt(clock.t, session)
        clock.sleep(0.05)
    n = int(hw.JB_OP_TOSS_SESSION_DWELL_TILT_READS)
    with node._lock:
        assert len(node._dwell_tilt_reads) == n
        assert node._dwell_tilt_degraded is False


def test_a_tight_dwell_degrades_the_read_count_never_the_throw(monkeypatch):
    """§ 3.10 rule 2, and it BITES at the shipped defaults: the QUIESCENT dwell is
    dwell_time_s - throw_delay_s minus the CAUGHT-verdict latency (~0.7 s at
    dwell 6.0 / delay 5.0), because the rest of the nominal dwell is the next
    cycle's own throw countdown. The reads must shrink; the cadence must not."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    monkeypatch.setattr(node, '_read_platform_tilt', lambda: (0.001, -0.002))
    session = _session()
    session._next_cycle_at = clock.t + 0.3       # less than the reserve
    node._maybe_read_dwell_tilt(clock.t, session)
    with node._lock:
        assert node._dwell_tilt_reads == []
        assert node._dwell_tilt_degraded is True


def test_a_failed_read_is_a_lost_data_point_not_a_retry_storm(monkeypatch):
    """The service BLOCKS the Platform-Teensy loop that streams hand moves, so a
    dead inclinometer must not turn the dwell into a hammering loop against it."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    calls = []
    monkeypatch.setattr(node, '_read_platform_tilt',
                        lambda: calls.append(clock.t))
    session = _session()
    session._next_cycle_at = clock.t + 10.0
    for _ in range(60):
        node._maybe_read_dwell_tilt(clock.t, session)
        clock.sleep(0.05)
    gaps = [b - a for a, b in zip(calls, calls[1:])]
    assert calls, 'the read was never attempted'
    assert all(g >= float(hw.JB_OP_TOSS_SESSION_DWELL_TILT_GAP_S) - 1e-9
               for g in gaps), gaps
    with node._lock:
        assert node._dwell_tilt_degraded is True


def test_a_nan_reading_never_reaches_the_record(monkeypatch):
    """NaN is the service's documented failure shape (the bridge returns
    [nan, nan] when the relay read fails). A NaN covariate in the corpus is
    indistinguishable from a real reading of zero in any fit that forgot to
    check."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    monkeypatch.setattr(
        node._tilt_cli, 'call_async',
        lambda req: types.SimpleNamespace(done=lambda: True))
    monkeypatch.setattr(
        node, '_wait_future',
        lambda fut, timeout_s=2.0: types.SimpleNamespace(
            tilt_xy=[float('nan'), float('nan')]))
    assert node._read_platform_tilt() is None


def test_the_dwell_tilt_block_reaches_the_record(monkeypatch):
    """Mean, sd, n, span and the LAST-READ-TO-RELEASE gap — the last of which is
    what lets a corpus AUDIT rule 1 rather than trust a docstring: it is positive
    iff the last read finished before the release."""
    reads = [(100.0, 0.001, -0.002), (100.15, 0.003, -0.004),
             (100.30, 0.002, -0.003)]
    fields = ReloadCoordinatorNode._dwell_tilt_fields(reads, 101.0)
    assert fields['dwell_tilt_n'] == 3
    assert fields['dwell_tilt_rad'] == pytest.approx([0.002, -0.003])
    assert fields['dwell_tilt_sd_rad'][0] == pytest.approx(
        math.sqrt(((0.001 - 0.002) ** 2 + (0.003 - 0.002) ** 2
                   + (0.002 - 0.002) ** 2) / 3.0))
    assert fields['dwell_tilt_span_s'] == pytest.approx(0.30)
    assert fields['dwell_tilt_last_read_to_release_s'] == pytest.approx(0.70)
    assert fields['dwell_tilt_last_read_to_release_s'] > 0.0


def test_no_dwell_reads_is_a_legal_block():
    fields = ReloadCoordinatorNode._dwell_tilt_fields([], 101.0)
    assert fields == {'dwell_tilt_n': 0}


def test_dwell_reads_belong_to_exactly_one_cycle(monkeypatch):
    """Snapshotted and CLEARED when the record is opened, so a read can never be
    attributed to two cycles' releases."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    with node._lock:
        node._dwell_tilt_reads = [(1.0, 0.001, 0.002)]
        node._dwell_tilt_degraded = True
    node._open_toss_record(action='toss_continuous', goal_id='abc', flight=0.8,
                           cycle_index=2, catch_pose=(0.0, 0.0, 170.0),
                           throw_delay=DELAY, vel_scale=0.8, raw_goal={})
    assert node._toss_record_ctx['dwell_tilt'] == [(1.0, 0.001, 0.002)]
    assert node._toss_record_ctx['dwell_tilt_degraded'] is True
    with node._lock:
        assert node._dwell_tilt_reads == []
        assert node._dwell_tilt_degraded is False


def test_a_cancel_during_the_interlude_safes_and_does_not_wait_out_the_settle(
        monkeypatch):
    """The settle floor protects the NEXT cycle; a cancelled session has none, so
    waiting it out would only make the operator's cancel look 2.8 s slower than
    it is. The SAFING still runs — that is the reload's own early-exit ladder,
    unchanged."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    safed = []
    monkeypatch.setattr(node, '_safe_on_early_exit',
                        lambda seq: safed.append(seq))
    monkeypatch.setattr(
        node, '_step_sequence',
        lambda seq, now, gh=None: types.SimpleNamespace(
            done=False, result=None, action='none', phase='CHECKING'))
    t0 = clock.t
    assert node._run_one_reload_attempt(cancel_now_fn=lambda: True) is None
    assert safed, 'the cancel path must still safe the machine'
    assert clock.t - t0 < rcn.DEFAULT_SESSION_MISS_CLEANUP_S


def test_a_cancel_inside_the_interlude_terminalises_the_session_as_cancelled(
        monkeypatch):
    """The stop code the interlude hands back is superseded at the next loop
    top by the session's own cancel check, so the operator sees ABORTED_CANCELLED
    and a CANCELED goal — not a STOPPED_* that reads like a machine fault."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    gh = _ContGoalHandle(num_throws=3, stop_on_miss=False)
    gh.request.on_empty_cup = 'RELOAD'
    _stub_cycles(node, monkeypatch, clock, [_no_ball()])

    def fake_interlude(session, *, cancel_now_fn=None):
        gh.is_cancel_requested = True
        return False, 'STOPPED_RELOAD_CANCELLED', 1

    monkeypatch.setattr(node, '_run_reload_interlude', fake_interlude)
    result = node._execute_toss_continuous(gh)
    assert result.outcome == 'ABORTED_CANCELLED'
    assert gh.terminal == 'canceled'


# ══ The interlude fixes the census orders BEFORE rung R3 (D4 / F3 + the HIGH) ══

ARRIVAL_BAND_S = float(rcn.hw.JB_BD_ARRIVAL_WINDOW_S)
_TICK = float(rcn._TICK_S)


def _caught(flight=FLIGHT):
    return TossResult(True, 'CAUGHT', 3.0, float(flight))


def _session_after_a_catch(landing, **kw):
    """A session that has completed ONE cycle, CAUGHT at ``landing``.

    That is the state the interlude is actually entered from, and the anchor the
    seat-edge band needs. Note the session does NOT advance ``_last_landing`` on
    the REJECTED_NO_BALL branch (it returns early), so at the interlude
    ``last_landing_perf`` is the previous CAUGHT cycle's landing — the instant the
    seat edge belongs to. That early return is load-bearing here: anchoring the
    band on the REJECTED cycle instead would anchor it on a landing that never
    happened, which is in the future and would over-wait."""
    s = _session(**kw)
    assert s.step(0.0).action == ts.SESSION_ACTION_START_CYCLE
    s.note_cycle_result(_caught(), 0.0, float(landing))
    assert s.last_landing_perf == pytest.approx(float(landing))
    return s


class _SeatingClock(_Clock):
    """A fake clock whose cup fills part-way through the wait — a REAL catch
    whose seat edge lands inside the measured +137…+798 ms band, after a CHECKING
    tick that already read the cup and called it empty."""

    def __init__(self, t0, seat_at):
        super().__init__(t0)
        self.seat_at = float(seat_at)
        self.sensor_held = False

    def sleep(self, dt):
        super().sleep(dt)
        if self.t >= self.seat_at:
            self.sensor_held = True


def test_a_good_catch_mislabelled_no_ball_never_licenses_a_bb_throw(monkeypatch):
    """CENSUS D4 + F3 — the "single most dangerous change" the cadence census
    names, closed.

    ``REJECTED_NO_BALL`` is minted at CHECKING, which runs at
    ``landing + (dwell - throw_delay)``. The physical seat edge lands
    **+137…+798 ms** after the announced landing. Below roughly a 1.2 s dwell
    those cross, so a CHECKING tick reads the cup BEFORE the ball has finished
    arriving and a GOOD catch mints ``REJECTED_NO_BALL``. With
    ``on_empty_cup: RELOAD`` that route asks BallButler to throw a SECOND ball
    at a cup that is about to be — and by the time BB throws, already is — full.

    Both halves are asserted. Read at the instant the interlude is entered the
    cup is EMPTY and the gate PASSES (the defect). Read after the seat-edge band
    it is SEATED and the gate refuses, naming the contradiction."""
    landing = 1000.0
    clock = _SeatingClock(landing, seat_at=landing + 0.30)   # inside the band
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock, cup_held=False)
    session = _session_after_a_catch(landing)
    # THE DEFECT: at the instant the interlude is entered the cup reads EMPTY.
    assert node._ball_sensor.evidence_settled(clock.t) == 'EMPTY'
    # THE FIX: the gate takes its own read, after the band.
    assert node._reload_interlude_gate(session) == 'STOPPED_CUP_NOT_EMPTY'
    assert clock.t >= landing + ARRIVAL_BAND_S, (
        'the read must be taken after the seat-edge band, not before it')


def test_the_band_wait_commands_nothing_and_is_derived_from_the_arrival_window(
        monkeypatch):
    """Nothing is armed and nothing is airborne while it waits — the interlude is
    only ever entered from ``REJECTED_NO_BALL``, the one toss terminal whose own
    terminal action was ACTION_NONE. And the wait is DERIVED from
    ``JB_BD_ARRIVAL_WINDOW_S``, the constant that already means "how long after
    the predicted landing a seat edge may still arrive", so the pending post-FW14
    band re-measure shrinks this wait too."""
    landing = 1000.0
    clock = _Clock(landing)
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock, cup_held=False)
    moved = []
    for name in ('_go_home', '_arm_catch', '_smooth_move_hand', '_safe_abort'):
        monkeypatch.setattr(node, name,
                            lambda *a, _n=name, **k: moved.append(_n) or True)
    assert node._reload_interlude_gate(_session_after_a_catch(landing)) is None
    assert clock.t == pytest.approx(landing + ARRIVAL_BAND_S, abs=_TICK)
    assert moved == []


def test_a_session_with_no_landing_yet_does_not_wait(monkeypatch):
    """Before any cycle has landed there is no seat edge to wait for, and a
    session that never reports one must not stall the interlude for a band that
    is anchored on nothing. NaN is the honest "no anchor"."""
    clock = _Clock(1000.0)
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock, cup_held=False)
    t0 = clock.t
    assert node._reload_interlude_gate(_session()) is None
    assert clock.t == pytest.approx(t0)


def test_the_gate_reads_the_settled_query_so_a_flicker_cannot_license_a_throw(
        monkeypatch):
    """C-POSSESS-1 § 3.5/§ 3.6. The live ``evidence`` query reads the RAW bit,
    which is right everywhere a wrong answer REFUSES — and wrong here, where a
    carry-flicker over a seated ball would COMMAND an autonomous BB throw into a
    full cup. The gate uses ``evidence_settled``: the two bits must agree, and a
    disagreement is UNKNOWN, which this gate already refuses on without moving
    anything."""
    landing = 1000.0
    clock = _Clock(landing)
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock, cup_held=True)
    clock.detach()          # stop the harness re-feeding, so the flicker stands
    # A single raw sample flickers EMPTY under a seated ball (five misses are
    # needed before the DEBOUNCED verdict would follow).
    node._ball_sensor.note_sample(landing + 0.01, held=True, valid=True,
                                  raw=False)
    clock.t = landing + 0.01
    assert node._ball_sensor.evidence(clock.t) == 'EMPTY'        # raw, fail-closed
    assert node._reload_interlude_gate(_session()) == 'STOPPED_SENSOR_UNKNOWN'


# ── The deferred-cancel discipline, transposed onto the interlude ─────────────

def _reload_seq_in(node, phase):
    """A ReloadSequencer parked in ``phase`` — the interlude builds its own, so
    the deferral predicate is exercised on the real object."""
    seq = ReloadSequencer(catch_point_mm=node._catch_point_mm, throw_delay_s=0.0)
    seq.start(0.0)
    seq._phase = phase
    return seq


@pytest.mark.parametrize('phase,deferred', [
    ('CHECKING', False),        # nothing commanded to BB — nothing can be falling
    ('PREPARING', False),       # armed, but the throw has not been sent
    ('AIMING', True),           # bb/throw_at_target dispatched: a ball may leave
    ('THROW_PENDING', True),
    ('BALL_IN_FLIGHT', True),
    ('CATCHING', True),
    ('SETTLING', True),
])
def test_a_cancel_is_deferred_once_the_throw_is_committed_to_bb(phase, deferred):
    """THE UNFIXED HIGH from the Phase-2 audit, closed by transposing
    ``_toss_cancel_deferred``.

    The toss path has always refused to honour a cancel mid-flight, for a reason
    that was never toss-specific: aborting a catch mid-flight drops a ball on the
    robot, and the honoured path runs ``_safe_on_early_exit`` -> ``_safe_abort``,
    i.e. it RETRACTS THE HAND out from under the incoming ball. The interlude's
    ball is thrown by BallButler and is exactly as airborne, and until 2026-08-21
    a session cancel during an interlude was honoured on the very next tick, at
    any phase.

    The boundary is the DISPATCH, not a cutoff around a release instant: AIMING
    is entered by invoking ``bb/throw_at_target``, BB's countdown cannot be
    aborted (``_enter_preparing``'s own docstring says so), and the interlude —
    unlike the toss, which is its own announcer — has no release instant to
    compare against until BB's announcement lands."""
    node = ReloadCoordinatorNode()
    assert node._reload_cancel_deferred(_reload_seq_in(node, phase), 0.0) \
        is deferred


def test_a_deferred_cancel_keeps_ticking_and_never_retracts_mid_flight(
        monkeypatch):
    """The behavioural half: with a ball committed, the attempt does NOT safe and
    does NOT return None — it ticks the FSM to its own terminal, exactly as
    ``_run_toss_cycle`` does under ``_toss_cancel_deferred``, and the session's
    own loop-top check terminalises the goal as cancelled afterwards."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    safed = []
    monkeypatch.setattr(node, '_safe_on_early_exit',
                        lambda seq: safed.append(seq))
    ticks = []
    cancelling = []

    def fake_step(seq, now, gh=None):
        ticks.append(now)
        # Tick 1 commits the throw to BB; the operator's cancel arrives after it.
        seq._phase = 'BALL_IN_FLIGHT'
        cancelling.append(True)
        if len(ticks) < 3:
            return types.SimpleNamespace(done=False, result=None,
                                         action='none', phase='CATCHING')
        return types.SimpleNamespace(
            done=True, result=ReloadResult(False, 'MISSED'),
            action='safe_abort', phase='SETTLING')

    monkeypatch.setattr(node, '_step_sequence', fake_step)
    result = node._run_one_reload_attempt(cancel_now_fn=lambda: bool(cancelling))
    assert result is not None and result.outcome == 'MISSED'
    assert safed == [], 'a deferred cancel must never retract under a live ball'
    assert len(ticks) == 3, 'the FSM keeps ticking to its own terminal'


def test_a_deferred_cancel_still_skips_the_settle_floor(monkeypatch):
    """The floor protects the NEXT cycle and a cancelled session has none —
    whether the cancel was honoured or deferred. Charging the operator's stop
    both the deferral AND the floor would make it look 2.9 s slower than it is."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    monkeypatch.setattr(node, '_safe_on_early_exit', lambda seq: None)

    cancelling = []

    def fake_step(seq, now, gh=None):
        # Tick 1 commits the throw and the cancel lands after it; tick 2 sees the
        # cancel (deferred, ball in the air) and then reaches the FSM terminal.
        seq._phase = 'BALL_IN_FLIGHT'
        if not cancelling:
            cancelling.append(True)
            return types.SimpleNamespace(done=False, result=None,
                                         action='none', phase='CATCHING')
        return types.SimpleNamespace(
            done=True, result=ReloadResult(False, 'MISSED'),
            action='safe_abort', phase='SETTLING')

    monkeypatch.setattr(node, '_step_sequence', fake_step)
    t0 = clock.t
    node._run_one_reload_attempt(cancel_now_fn=lambda: bool(cancelling))
    assert clock.t - t0 < rcn.DEFAULT_SESSION_MISS_CLEANUP_S


def test_a_cancel_during_the_band_wait_is_honoured_before_anything_is_committed(
        monkeypatch):
    """The band wait can run for ~1.5 s and it precedes every commitment, so a
    cancel there must be honoured immediately — the deferral exists for a
    committed throw, not for a wait."""
    landing = 1000.0
    clock = _Clock(landing)
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock, cup_held=False)
    session = _session_after_a_catch(landing)
    stop = node._reload_interlude_gate(session, cancel_now_fn=lambda: True)
    assert stop == 'STOPPED_RELOAD_CANCELLED'
    assert clock.t < landing + ARRIVAL_BAND_S, 'it must not wait the band out'
