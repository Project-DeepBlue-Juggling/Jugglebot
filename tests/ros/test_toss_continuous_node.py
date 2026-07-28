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

import pytest

import jugglebot.hardware_config as hw
import jugglebot.reload_coordinator_node as rcn
from jugglebot.reload_coordinator_node import (
    ReloadCoordinatorNode,
    _toss_session_deadline_s,
)
from jugglebot.reload_sequencer import ReloadSequencer
from jugglebot.toss_sequencer import (
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

    def attach(self, node):
        self._node = node

    def perf_counter(self):
        return self.t

    def sleep(self, dt):
        self.t += float(dt)
        if self._node is not None:
            _stamp(self._node, self.t)


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

@pytest.mark.parametrize('bx,expect_x', [
    (0.0, 0.000), (70.0, 71.448), (140.0, 142.894),
    (146.0, 149.017), (147.0, 150.038), (150.0, 153.100)])
def test_predicted_chain_site_matches_the_production_policy(bx, expect_x,
                                                            monkeypatch):
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


def test_chain_frontier_is_between_146_and_147_mm(monkeypatch):
    """The frontier is sharp and the binding gate is the +-150 mm planning box on
    A, NOT the 150 mm displacement cap (the residual |B-A| never exceeds
    3.1 mm)."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    inside = node._predicted_chain_site_mm((146.5, 0.0, 170.0), FLIGHT)
    outside = node._predicted_chain_site_mm((147.0, 0.0, 170.0), FLIGHT)
    assert abs(inside[0]) <= TOSS_XY_LIMIT_MM
    assert abs(outside[0]) > TOSS_XY_LIMIT_MM
    for bx, site in ((146.5, inside), (147.0, outside)):
        assert math.hypot(bx - site[0], site[1]) < 3.2   # never the cap


def test_chain_unreachable_refuses_before_a_ball_flies(monkeypatch):
    """Without this the session throws ONE ball, catches it, then refuses cycle 2
    REJECTED_WORKSPACE with the platform parked outside the planning box and the
    ball in the cup — actuation for nothing. The refusal moves nothing."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    monkeypatch.setattr(
        node, '_build_toss_cycle',
        lambda *a, **k: pytest.fail('a cycle was built on an unchainable goal'))
    gh = _ContGoalHandle(x=147.0, num_throws=3)
    result = node._execute_toss_continuous(gh)
    assert result.outcome == 'REJECTED_CHAIN_UNREACHABLE'
    assert gh.terminal == 'abort'


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


def test_chain_check_is_skipped_when_the_pose_is_unknown(monkeypatch):
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
    monkeypatch.setattr(hw, 'JB_OP_TOSS_TIER', '8a')
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
