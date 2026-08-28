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
  S6 — the catch latch and the two catch_coordinator holds are SESSION-scoped:
       one raise inside the execute call, one lower on every exit path.
  S7 — every go_home is fronted by _drain_pipeline_and_disarm, pinned
       STRUCTURALLY (an added call site is the failure, and no behavioural test
       would see it).

The fake clock is a namespace swapped in for the node module's ``time``, which
uses exactly ``perf_counter`` and ``sleep`` — so a multi-second dwell costs
microseconds and the scheduling arithmetic is exact rather than approximate.

ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

import ast
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
from jugglebot.motion.ik_solver import (
    quat_to_rot_matrix,
    rot_matrix_to_rotvec,
)
from jugglebot.motion.trajectory.toss_release import aim_target_offset_mm
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


@pytest.fixture(autouse=True)
def _pin_the_pipeline_flag(monkeypatch):
    """**Every test in this file states which pipeline it drives, and none of
    them inherits it from the shipped config** (2026-08-28).

    Until the operator turned ``toss_pipeline_enabled`` on, the flag shipped
    FALSE and the serial half of this file got the serial ladder for free. That
    was an implicit precondition: the session-semantics tests below
    (``_stub_cycles`` scripts ``_run_toss_cycle``, the SERIAL blocking wrapper —
    a method the pipelined loop never calls) assert stop_on_miss, cancel,
    retry, teardown and reload-interlude behaviour, and the flip silently
    re-pointed all 28 of them at a path their harness does not stub. They went
    red without a line of production code changing, which is the tell: a test
    whose subject is chosen by a YAML key is a test a config edit can retarget.

    So the default is pinned HERE, explicitly, and the pipelined half overrides
    it in ``_pipelined_node`` (its ``monkeypatch.setattr`` runs later and wins).
    No assertion in this file changed; what changed is that the ones written for
    the serial ladder now SAY so.

    ⚠ The honest consequence, and it is a real coverage gap rather than a
    bookkeeping one: the session-accounting semantics below have NO pipelined
    twin. They never did — under the old default they could not have — but the
    machine now ships pipelined, so the gap is live rather than latent. It is
    recorded in ``logbook/2026-08-28-pipeline-first-contact-deadlock.md``
    § Follow-ups.

    **The instance that mattered most, named** (2026-08-28 audit): both of F3's
    non-firing tests — ``test_the_watchdog_never_fires_on_a_healthy_chained_session``
    and ``test_the_watchdog_never_fires_across_a_reload_interlude`` — FAIL when
    this fixture is flipped to True (verified by flipping it), so the watchdog
    that ships ON was pinned against FALSE FIRING only on the serial ladder,
    which is the machine the operator no longer runs. That specific hole is now
    closed by ``test_the_watchdog_never_fires_on_a_healthy_PIPELINED_session``;
    the rest of the accounting suite (stop_on_miss, cancel, the retry ladder,
    the reload interlude, the S6/S7 teardown) is still serial-only."""
    monkeypatch.setattr(rcn.hw, 'JB_OP_TOSS_PIPELINE_ENABLED', False,
                        raising=False)


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

    def fake_run(seq, *, deadline_s, cancel_now_fn, feedback_fn, state=None):
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
    # gate can catch it. The floor is the DERIVED :642 dispatch budget since
    # 2026-08-22 (census A1) — 0.337 s at the band floor this session is judged
    # at — so the row asks for 0.20 s; 2.0 s is now legal.
    (dict(delay=0.20, dwell=4.0), 'REJECTED_THROW_DELAY'),
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

    def spy(catch_pose, flight, throw_delay, vel_scale, **kw):
        calls.append((catch_pose, flight, throw_delay, vel_scale, kw))
        return real_build(catch_pose, flight, throw_delay, vel_scale, **kw)

    monkeypatch.setattr(node, '_build_toss_cycle', spy)
    _stub_cycles(node, monkeypatch, clock,
                 [TossResult(True, 'CAUGHT', 2.0, 0.81)] * 2)
    node._execute_toss_continuous(
        _ContGoalHandle(num_throws=2, delay=DELAY, vel_scale=0.9))
    assert len(calls) == 2
    for pose, flight, delay, scale, kw in calls:
        assert pose == (0.0, 0.0, 170.0)
        assert flight == pytest.approx(float(hw.JB_OP_TOSS_FLIGHT_TIME_DEFAULT_S))
        assert delay == pytest.approx(DELAY)
        assert scale == pytest.approx(0.9)
        # …and the SESSION path declares its delay a CADENCE parameter, which is
        # what lets the one cycle that must command its pre-positioning move be
        # granted the extra lead instead of killing the sitting (2026-08-23). A
        # single Toss passes no such flag and is refused instead.
        assert kw == {'delay_is_cadence': True}


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

    def fake_run(seq, *, deadline_s, cancel_now_fn, feedback_fn, state=None):
        with node._lock:
            seen.append((node._toss_committed.release_state is not None,
                         node._toss_committed.landing_global_mm is not None))
        clock.t = seq.t_release + float(seq.flight_time_s) + 0.3
        _stamp(node, clock.t)
        return TossResult(True, 'CAUGHT', 2.0, 0.81), 'fsm'

    monkeypatch.setattr(node, '_run_toss_cycle', fake_run)
    node._execute_toss_continuous(_ContGoalHandle(num_throws=2))
    assert seen == [(True, True), (True, True)]   # installed for every cycle
    with node._lock:
        assert node._toss_committed.release_state is None       # …and torn down after
        assert node._toss_committed.landing_global_mm is None
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
    # The achieved dwell lands on the request within ONE LOOP PERIOD. The bound
    # was `_TICK_S` until 2026-08-27, when B5 replaced the session loop's fixed
    # `time.sleep(_TICK_S)` with absolute-schedule pacing at `_PACE_PERIOD_S`:
    # the fake clock now advances in paced steps, so the START_CYCLE poll can be
    # up to a whole period late instead of up to a whole sleep late. That is the
    # cost the pacing constant's comment names in so many words, and it is the
    # quantity `DEFAULT_SESSION_MISS_CLEANUP_S` has charged at
    # `2 x NODE_LOOP_PERIOD_S` since D3 — so the bound is now denominated in the
    # same unit the session's own budget is.
    for achieved in result.per_cycle_dwell_s[1:]:
        assert DWELL <= achieved < DWELL + rcn._PACE_PERIOD_S
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

    def fake_run(seq, *, deadline_s, cancel_now_fn, feedback_fn, state=None):
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

    def fake_run(seq, *, deadline_s, cancel_now_fn, feedback_fn, state=None):
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

    def fake_run(seq, *, deadline_s, cancel_now_fn, feedback_fn, state=None):
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

    def fake_run(seq, *, deadline_s, cancel_now_fn, feedback_fn, state=None):
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


# ── S6/S7: session-scoped arming and drain-before-go_home ────────────────────
# plans/active/toss-pipelined-preamble.md § 2.3. The per-cycle choreography and
# the drift guard are pinned in test_toss_coordinator.py; here we pin the
# SESSION lifecycle (raised inside the execute call, lowered on every exit) and
# the structural invariant S7 rests on.

def _method_sources(module):
    """Every top-level-or-nested ``def`` in a module, by name, as source text.

    AST rather than string-splitting so a method's extent is exact — the point
    of the structural test below is "in the SAME function", and a regex over
    ``\\n    def `` would silently merge a method into its neighbour the first
    time someone nests a helper."""
    text = Path(module.__file__).with_suffix('.py').read_text()
    lines = text.splitlines()
    out = {}
    for node in ast.walk(ast.parse(text)):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            out[node.name] = '\n'.join(lines[node.lineno - 1:node.end_lineno])
    return out


#: The two go_home dispatchers that are SHARED VERBATIM with the reload path
#: (`_step_sequence`'s RECENTER / SAFE_ABORT actions), mapped to the toss-side
#: wrapper that drains for them. They are not given the drain themselves because
#: the drain is a toss concept and these two are the reload's ladders too —
#: keeping them byte-identical for the reload is what makes "the toss teardown
#: is the reload teardown verbatim" still true.
_GO_HOME_DRAINED_BY_WRAPPER = {'_recenter': '_toss_recenter',
                               '_safe_abort': '_toss_safe_abort'}


def test_go_home_has_no_call_site_that_is_not_drained_first():
    """**THE structural gate for S7** (the test_dwell_tilt_reads_have_exactly_one
    _call_site idiom): every function that dispatches ``trajectory/go_home``
    drains the pipeline and lowers the session-scoped arming FIRST, in the same
    function — or is one of the two reload-shared ladders whose toss-side
    wrapper does it for them.

    Structural rather than behavioural because the failure it prevents is an
    ADDED call site: a new teardown that dispatches go_home under a standing
    catch latch reproduces the arm-mid-move seam (10 of 16 post-MISS cycles on
    bag 2026-08-26_14-25-16) and no behavioural test would notice, because every
    existing path would still be green."""
    src = _method_sources(rcn)
    callers = sorted(n for n, s in src.items()
                     if n != '_go_home' and 'self._go_home(' in s)
    assert callers, 'no go_home call sites found — the test would pass vacuously'
    for name in callers:
        body = src[name]
        if 'self._drain_pipeline_and_disarm(' in body:
            assert (body.index('self._drain_pipeline_and_disarm(')
                    < body.index('self._go_home(')), (
                f'{name} drains AFTER its go_home')
            continue
        wrapper = _GO_HOME_DRAINED_BY_WRAPPER.get(name)
        assert wrapper is not None, (
            f'{name} dispatches go_home without draining first (S7)')
        w = src[wrapper]
        assert (w.index('self._drain_pipeline_and_disarm(')
                < w.index('self.%s(' % name)), (
            f'{wrapper} must drain before calling {name}')


def test_every_named_teardown_routes_through_the_drain():
    """The five paths the plan names, plus the position-unknown zombie
    superseder — the one go_home call site the FSM reaches with ACTION_NONE, so
    no terminal ladder runs and nothing else would have drained it."""
    src = _method_sources(rcn)
    for name in ('_toss_safe_abort', '_toss_recenter', '_recentre_for_reload',
                 '_safe_toss_on_early_exit', '_step_toss_sequence',
                 '_execute_toss_continuous'):
        assert 'self._drain_pipeline_and_disarm(' in src[name], name
    # …and STAY deliberately does NOT: it issues no go_home, and draining there
    # would force the next chained cycle to re-raise the latch — the seam.
    assert 'self._drain_pipeline_and_disarm(' not in src['_toss_stay']


def _arming_cycles(node, monkeypatch, clock, results, *, verdict_latency=0.30):
    """`_stub_cycles`, but the stubbed cycle also performs the SESSION ARM the
    real first cycle's PREPARE would — so the teardown paths under test have
    something to lower."""
    monkeypatch.setattr(node, '_set_soft_catch_gains', lambda: True)
    arm_calls = []
    monkeypatch.setattr(node, '_arm_catch',
                        lambda a: arm_calls.append(bool(a)) or True)
    pending = list(results)

    def fake_run(seq, *, deadline_s, cancel_now_fn, feedback_fn, state=None):
        node._arm_session_declare(seq)
        assert node._arm_session(seq) is True
        clock.t = seq.t_release + float(seq.flight_time_s) + verdict_latency
        _stamp(node, clock.t)
        return pending.pop(0), 'fsm'

    monkeypatch.setattr(node, '_run_toss_cycle', fake_run)
    return arm_calls


def test_a_chained_session_raises_the_latch_once_and_lowers_it_once(monkeypatch):
    """S6 at session scope: three chained CAUGHT cycles, ONE arm_catch raise and
    ONE lower. The lower happens at the session terminal — the CAUGHT terminal
    (STAY) deliberately leaves the latch up, which is what makes the run
    contiguous."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    arm_calls = _arming_cycles(
        node, monkeypatch, clock,
        [TossResult(True, 'CAUGHT', 2.0, 0.81)] * 3)
    gh = _ContGoalHandle(num_throws=3, stop_on_miss=False)
    result = node._execute_toss_continuous(gh)
    assert result.outcome == 'COMPLETED'
    assert arm_calls == [True, False]
    assert node._toss_session_armed is False
    assert node._toss_session_center_mm is None
    assert node._toss_session_live is False


@pytest.mark.parametrize('exit_kind,outcome', [
    ('fsm', 'COMPLETED'),
    ('cancel', 'ABORTED_CANCELLED'),
    ('timeout', 'ABORTED_TIMEOUT'),
    ('shutdown', 'ABORTED_SHUTDOWN')])
def test_the_session_terminal_disarms_on_every_exit(exit_kind, outcome,
                                                    monkeypatch):
    """S7 at the session terminal, and it is in the ``finally`` for exactly this
    reason: a session that ends with the latch standing is a machine left armed
    with a ball in the cup and nobody ticking it. Every node-level exit —
    completion, a cancel or timeout adjudicated inside a cycle, an rclpy
    shutdown — routes through the same one call site."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    monkeypatch.setattr(node, '_set_soft_catch_gains', lambda: True)
    arm_calls = []
    monkeypatch.setattr(node, '_arm_catch',
                        lambda a: arm_calls.append(bool(a)) or True)

    def fake_run(seq, *, deadline_s, cancel_now_fn, feedback_fn, state=None):
        node._arm_session_declare(seq)
        node._arm_session(seq)
        clock.t = seq.t_release + float(seq.flight_time_s) + 0.3
        _stamp(node, clock.t)
        if exit_kind == 'fsm':
            return TossResult(True, 'CAUGHT', 2.0, 0.81), 'fsm'
        return TossResult(False, 'ABORTED_' + exit_kind.upper()), exit_kind

    monkeypatch.setattr(node, '_run_toss_cycle', fake_run)
    node._execute_toss_continuous(_ContGoalHandle(num_throws=1,
                                                  stop_on_miss=False))
    assert arm_calls == [True, False]
    assert node._toss_session_armed is False
    assert node._toss_session_live is False


def test_the_session_disarm_survives_a_raising_cycle(monkeypatch):
    """The ``finally`` is what makes the disarm unconditional: a cycle that
    RAISES must not leave the latch up on the way out. Same argument as the
    goal-claim release beside it, and the reason both live there."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    monkeypatch.setattr(node, '_set_soft_catch_gains', lambda: True)
    arm_calls = []
    monkeypatch.setattr(node, '_arm_catch',
                        lambda a: arm_calls.append(bool(a)) or True)

    def fake_run(seq, *, deadline_s, cancel_now_fn, feedback_fn, state=None):
        node._arm_session_declare(seq)
        node._arm_session(seq)
        raise RuntimeError('boom')

    monkeypatch.setattr(node, '_run_toss_cycle', fake_run)
    with pytest.raises(RuntimeError, match='boom'):
        node._execute_toss_continuous(_ContGoalHandle(num_throws=2))
    assert arm_calls == [True, False]
    assert node._toss_session_armed is False
    assert node._toss_session_live is False


def test_the_single_toss_action_is_never_session_scoped(monkeypatch):
    """The flag is set inside `_execute_toss_continuous`'s try and cleared in
    its finally, so a single `Toss` on the same node — before, after, or
    interleaved with a session — always takes the per-cycle arming path."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    assert node._toss_session_live is False
    seen = []
    _stub_cycles(node, monkeypatch, clock,
                 [TossResult(True, 'CAUGHT', 2.0, 0.81)])
    orig = node._run_toss_cycle

    def watch(seq, **kw):
        seen.append(node._toss_session_live)
        return orig(seq, **kw)

    monkeypatch.setattr(node, '_run_toss_cycle', watch)
    node._execute_toss_continuous(_ContGoalHandle(num_throws=1,
                                                  stop_on_miss=False))
    assert seen == [True]                 # live for the cycle…
    assert node._toss_session_live is False   # …and clear again after


def test_the_reload_interlude_drains_before_its_recentre(monkeypatch):
    """S7 on the interlude, and this is the seam that used to fire loudest: the
    interlude traverses the whole workspace on a go_home, and before S6 the NEXT
    cycle's PREPARE re-raised the latch into that traverse. The interlude ends
    the contiguous run; the cycle after it re-arms with a fresh declaration."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    node._toss_session_live = True
    node._toss_session_center_mm = (0.0, 0.0, 170.0)
    node._toss_session_armed = True
    order = []
    monkeypatch.setattr(node, '_arm_catch',
                        lambda a: order.append(('arm', a)) or True)
    monkeypatch.setattr(node, '_publish_catch_armed',
                        lambda a: order.append(('armed', a)))
    monkeypatch.setattr(node, '_publish_prime_hold',
                        lambda h: order.append(('prime_hold', h)))
    monkeypatch.setattr(node, '_publish_pretilt_hold',
                        lambda h: order.append(('pretilt_hold', h)))
    monkeypatch.setattr(node, '_go_home',
                        lambda: order.append('go_home') or False)
    assert node._recentre_for_reload() is False        # go_home refused
    assert order == [('armed', False), ('arm', False), ('prime_hold', False),
                     ('pretilt_hold', False), 'go_home']
    assert node._toss_session_armed is False


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


def test_any_failed_attempt_is_retried_within_budget(monkeypatch):
    """**Owner decision D2, 2026-08-26.** Every way a reload can fail draws on
    ``max_reloads``, not just BB's ``THROW_ABORTED_NOT_SETTLED``.

    Before D2 this test asserted the opposite — that only that ONE code retried,
    because 'a blanket retry would swallow the BB fail-open boot bug and every
    real BB fault, and would keep asking an unwell machine to throw real balls'.
    The argument was right about the hazard and wrong about the remedy: the
    fail-open boot fence is ``STOPPED_BB_UNVERIFIED`` and the unwell-machine fence
    is ``STOPPED_BB_NOT_READY``, both rungs of the interlude GATE — and the gate
    ran once, outside the retry loop, so keying on the code was doing the fences'
    job badly instead of running the fences. D2 runs the whole ladder per attempt
    (``test_every_refusal_rung_gates_every_retry``), which is what makes the
    widening safe.

    The operator cost of the old rule: a DELIVERED reload that missed the cup
    stopped the sitting on its first miss."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    for bb_code in ('', 'THROW_REJECTED_PV_STALE (axis=YAW, detail1=0)',
                    'TIMEOUT (axis=n/a, detail1=0)',
                    'THROW_ABORTED_NOT_SETTLED (axis=YAW, detail1=0)'):
        node = _reload_ready_node(clock)
        calls = _stub_attempts(
            node, monkeypatch,
            [ReloadResult(False, 'MISSED'), ReloadResult(True, 'CAUGHT')],
            bb_codes=[bb_code, ''])
        ok, code, attempts = node._run_reload_interlude(_session(max_reloads=3))
        assert (ok, code, attempts) == (True, None, 2), bb_code
        assert len(calls) == 2, bb_code


def test_a_delivered_but_missed_reload_draws_on_the_budget(monkeypatch):
    """The D2 case the old rule could not serve: BB threw, the ball flew, the cup
    stayed empty. Nothing is wrong with the machine — the throw missed — so the
    budget is exactly the right fence, and it is spent one ball at a time until it
    runs out."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    calls = _stub_attempts(
        node, monkeypatch,
        [ReloadResult(False, 'MISSED'), ReloadResult(False, 'MISSED'),
         ReloadResult(False, 'MISSED')],
        bb_codes=['', '', ''])            # BB reported nothing: a real delivery
    ok, code, attempts = node._run_reload_interlude(_session(max_reloads=3))
    assert (ok, code, attempts) == (False, rcn.OUTCOME_STOPPED_RELOAD_BUDGET, 3)
    assert len(calls) == 3


@pytest.mark.parametrize('break_rung,want_code', [
    ('cup_not_empty', 'STOPPED_CUP_NOT_EMPTY'),
    ('sensor_unknown', 'STOPPED_SENSOR_UNKNOWN'),
    ('bb_not_ready', 'STOPPED_BB_NOT_READY'),
    ('bb_unverified', 'STOPPED_BB_UNVERIFIED'),
    ('recentre', 'STOPPED_RECENTRE_FAILED'),
])
def test_every_refusal_rung_gates_every_retry(monkeypatch, break_rung, want_code):
    """**The safety half of D2.** A retry is a repeat of the whole LADDER, not of
    the throw: rungs 1 and 2 run again before every attempt.

    Driven the only way that proves it — let attempt 1 run and FAIL, then break
    one rung and assert the RETRY refuses with that rung's own code rather than
    throwing a second ball. Before D2 the gate ran once, outside the loop, so the
    retry could not see any of these.

    ``STOPPED_CUP_NOT_EMPTY`` is the load-bearing one: the interlude answers an
    empty cup by asking BallButler to throw at it, so a retry that skipped this
    rung after the first ball actually landed would put a second ball into a full
    cup."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    calls = _stub_attempts(
        node, monkeypatch,
        [ReloadResult(False, 'MISSED'), ReloadResult(True, 'CAUGHT')],
        bb_codes=['', ''])

    real_attempt = node._run_one_reload_attempt

    def attempt_then_break(**kw):
        out = real_attempt(**kw)
        if break_rung == 'cup_not_empty':
            clock.sensor_held = True
            _feed_sensor(node, clock.t, held=True, valid=True)
        elif break_rung == 'sensor_unknown':
            _feed_sensor(node, clock.t, held=False, valid=False)
        elif break_rung == 'bb_not_ready':
            with node._lock:
                node._hb = _Hb(connected=False)
        elif break_rung == 'bb_unverified':
            with node._lock:
                node._bb_ball_in_hand_observed_false = False
        elif break_rung == 'recentre':
            monkeypatch.setattr(node, '_go_home', lambda: False)
        return out

    monkeypatch.setattr(node, '_run_one_reload_attempt', attempt_then_break)
    ok, code, attempts = node._run_reload_interlude(_session(max_reloads=3))
    assert ok is False
    assert code == want_code
    # ONE ball was thrown, not two: the refusal landed BEFORE the retry.
    assert attempts == 1
    assert len(calls) == 1


@pytest.mark.parametrize('outcome', [
    'REJECTED_WRONG_MODE', 'REJECTED_MOCAP_STALE', 'REJECTED_NOT_STREAMING',
    'REJECTED_NO_BALL', 'REJECTED_BB_BUSY', 'REJECTED_NOT_CENTERED',
    'REJECTED_BB(solver: no solution)',
    'ABORTED_BB_ERROR', 'ABORTED_TIMEOUT', 'ABORTED_SHUTDOWN',
])
def test_a_precondition_failure_stops_by_name_and_draws_no_further_budget(
        monkeypatch, outcome):
    """**D2 widens the budget to failed THROWS, not to precondition failures**
    (audit finding W6, 2026-08-26).

    ``max_reloads`` is a BALL budget — it counts balls the interlude put on the
    floor. As first written the D2 loop retried on ANY non-success
    ``ReloadResult``, which spends that budget on failures where no ball ever
    left BallButler, and then collapses all of them into
    ``STOPPED_RELOAD_BUDGET``.

    That collapse is the part with teeth. ``tests/hardware/toss_cal_grid.py``
    reads ``STOPPED_RELOAD_BUDGET`` as *"the node exhausted its reloads — skip
    this node"*, so a BallButler with a stale mocap feed or a hard fault would
    have burned three balls per cell and then let the capture tool complete a
    THIN GRID SILENTLY, with no operator-visible fault anywhere. A calibration
    that quietly measured less than it says it did is the one failure a capture
    tool must never have.

    So each of these stops the session by its OWN name, on the first attempt,
    before any further budget is drawn and before the recentre is re-dispatched.
    The operator reads what to fix."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    calls = _stub_attempts(
        node, monkeypatch,
        [ReloadResult(False, outcome), ReloadResult(True, 'CAUGHT')])
    recentres = []
    real_recentre = node._recentre_for_reload
    monkeypatch.setattr(node, '_recentre_for_reload',
                        lambda: (recentres.append(1), real_recentre())[1])
    ok, code, attempts = node._run_reload_interlude(_session(max_reloads=3))
    assert ok is False
    assert code == 'STOPPED_RELOAD_{}'.format(outcome)
    assert code != rcn.OUTCOME_STOPPED_RELOAD_BUDGET
    # ONE attempt, ONE recentre: the second ReloadResult above is never reached.
    assert attempts == 1
    assert len(calls) == 1
    assert len(recentres) == 1


def test_a_delivered_MISS_is_still_a_failed_throw_and_still_retries(monkeypatch):
    """The other side of W6, so the narrowing cannot quietly become "stop on any
    failure". A ``MISSED`` is a ball that flew and did not land in the cup — the
    exact case D2 widened the budget FOR — and it must still draw an attempt and
    retry. Ditto the three ABORTED codes deliberately left on the retry side."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    for outcome in ('MISSED', 'ABORTED_MODE_CHANGED', 'ABORTED_PRIME_FAILED',
                    'ABORTED_PREPARE_FAILED'):
        node = _reload_ready_node(clock)
        calls = _stub_attempts(
            node, monkeypatch,
            [ReloadResult(False, outcome), ReloadResult(True, 'CAUGHT')])
        ok, code, attempts = node._run_reload_interlude(_session(max_reloads=3))
        assert (ok, code, attempts) == (True, None, 2), outcome
        assert len(calls) == 2, outcome


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


def test_a_stale_bb_outcome_is_not_attributed_to_this_attempt(monkeypatch):
    """A BB code from a PREVIOUS attempt must not be reported as this attempt's —
    so the cache is only consulted for outcomes stamped after this attempt began.

    Until D2 (2026-08-26) this was a SAFETY rule: a stale NOT_SETTLED would have
    licensed a retry of a reload that failed for another reason. Since D2 every
    failure retries anyway, so what the freshness rule protects is the LOG LINE —
    a mis-attributed BB code routes the operator at the wrong machine. Kept, and
    re-stated for what it now does; the assertion is on the reported code."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    node._on_bb_throw_outcome(
        types.SimpleNamespace(data='THROW_ABORTED_NOT_SETTLED (axis=YAW)'))
    clock.sleep(1.0)                       # the attempt starts AFTER that stamp
    assert node._bb_throw_outcome_since(clock.perf_counter()) is None
    _stub_attempts(node, monkeypatch,
                   [ReloadResult(False, 'MISSED')] * 3)
    ok, code, attempts = node._run_reload_interlude(_session(max_reloads=3))
    assert (ok, code, attempts) == (False, rcn.OUTCOME_STOPPED_RELOAD_BUDGET, 3)


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
    # Every wait the interlude can legitimately spend must be IN the budget. The
    # seat-edge band (C-POSSESS-1 § 3.6) is the newest one and it is the easiest
    # to forget, because it is spent in the GATE — before the attempt loop the
    # rest of this arithmetic walks. Charged per attempt, which over-counts on
    # purpose: this ceiling ABORTS.
    per_attempt = rcn._reload_interlude_budget_s(reload_) / 3.0
    assert per_attempt > (rcn.GO_HOME_DURATION_S + rcn._RECENTRE_VERIFY_PAD_S
                          + rcn.DEFAULT_SESSION_MISS_CLEANUP_S
                          + float(rcn.hw.JB_BD_ARRIVAL_WINDOW_S))


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
    the predicted landing a seat edge may still arrive".

    ⚠ **That is a YAML knob, not ``ball_possession.ARRIVAL_BAND_MAX_S``**, and
    the docstring here used to claim the pending post-FW14 band re-measure would
    shrink this wait too. It did not: the re-measure (2026-08-24) moved the band
    constant 0.80 -> 0.56 and left ``JB_BD_ARRIVAL_WINDOW_S`` at 1.50 s, widening
    this wait's margin over the ceiling from 1.9x to 2.7x. This test pins the
    derivation that actually exists."""
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


@pytest.mark.parametrize('fault_code', [
    'REJECTED_WRONG_MODE', 'ABORTED_BB_ERROR', 'ABORTED_TIMEOUT'])
def test_a_precondition_fault_stops_the_session_by_name(monkeypatch, fault_code):
    """**The narrowing half of D2 (audit W6).** The budget is a BALL-SUPPLY
    fence: it is drawn on by failed THROWS (BB's NOT_SETTLED abort, or a
    delivered ball the cup did not catch) and by nothing else. A precondition
    fault — wrong mode, stale mocap, a BB error — is not a ball: retrying it
    spends the budget on a failure no reload can fix, and collapsing its
    terminal into ``STOPPED_RELOAD_BUDGET`` makes ``toss_cal_grid`` read a
    faulted BallButler as "node exhausted, skip" and complete a thin grid
    instead of aborting. So the session stops on the FIRST such attempt, with
    the fault's own name on the terminal."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _reload_ready_node(clock)
    calls = _stub_attempts(
        node, monkeypatch,
        [ReloadResult(False, fault_code), ReloadResult(True, 'CAUGHT')],
        bb_codes=['', ''])
    ok, code, attempts = node._run_reload_interlude(_session(max_reloads=3))
    assert (ok, code, attempts) == (
        False, 'STOPPED_RELOAD_{}'.format(fault_code), 1)
    assert len(calls) == 1, 'a precondition fault must never be retried'


# ── B4: the two-slot pipeline, at node level ─────────────────────────────────
#
# Everything below runs behind `toss_pipeline_enabled`, which SHIPS FALSE — so
# every test above this line describes the machine as it ships and every test
# below it describes the machine behind the flag. The pipelined tests drive the
# REAL FSMs through the REAL `_execute_toss_continuous`; only the service round
# trips (arm_catch, gains, go_home, the hand dispatch) and the PLANT are
# simulated, because those are the things a bench sitting supplies.

_SEAT_EDGE_S = 0.1839        # B0/P2's measured median seat edge


class _PipelineClock(_Clock):
    """``_Clock`` plus a PLANT: every simulated tick re-drives the hand
    telemetry and the cup sensor from where the committed cycle's ball actually
    is.

    Without it a pipelined session is unrunnable in a test for reasons that have
    nothing to do with the pipeline: the FSM waits on release evidence
    (`throw_stroke_seen`) that only hand telemetry can supply, and the COMMIT
    gate waits on a cup edge that only an empty→held transition can supply. The
    plant is the cheapest thing that makes both true at the right instants."""

    def __init__(self, t0=1000.0, seat_edge_s=_SEAT_EDGE_S):
        super().__init__(t0)
        self.seat_edge_s = float(seat_edge_s)

    def sleep(self, dt):
        self.t += float(dt)
        if self._node is None:
            return
        _stamp(self._node, self.t)
        self._plant(self.t)

    def _plant(self, t):
        node = self._node
        # `trajectory/commanded_pose` — the SIX-component sample, with the
        # INTENT-frame orientation. It is not optional scenery: the census-B1
        # positioning skip refuses to skip without it (a tilted pre-tilt pose is
        # unverifiable from position alone), and a cycle that must COMMAND its
        # move cannot stage. Without this the pipeline is correctly INERT and
        # every test below silently measures the serial path.
        with node._lock:
            node._commanded_pose = tuple(node._commanded_pos_mm) + (0.0, 0.0, 0.0)
            node._commanded_pose_mono = t
        seq = node._active_seq
        held = True
        pos, vel = 0.0, 0.0
        if seq is not None and seq.t_release > 0.0:
            rel = float(seq.t_release)
            land = rel + float(seq.flight_time_s)
            if rel <= t < land + self.seat_edge_s:
                held = False              # the ball is out of the cup
            if rel <= t < rel + 0.15:
                # the ascending release stroke — channel 1 of the release
                # evidence the FSM will not leave THROWING without
                pos, vel = 3.0, 60.0
        with node._lock:
            node._hand_pos_meas = pos
            node._hand_vel_meas = vel
        node._ball_sensor.note_sample(t, held=held, valid=True, raw=held)


def _pipelined_node(clock, monkeypatch, *, arm_ok=True):
    """A node with every precondition satisfied, the flag ON, and every service
    round trip stubbed. Returns ``(node, calls)`` where ``calls`` is the ordered
    log the structural/ordering assertions read."""
    monkeypatch.setattr(rcn.hw, 'JB_OP_TOSS_PIPELINE_ENABLED', True,
                        raising=False)
    node = _ready_node(clock)
    calls = []
    monkeypatch.setattr(node, '_set_soft_catch_gains',
                        lambda: calls.append('gains') or True)
    monkeypatch.setattr(node, '_arm_catch',
                        lambda a: calls.append(('arm', bool(a))) or arm_ok)
    monkeypatch.setattr(node, '_go_home',
                        lambda *a, **k: calls.append('go_home') or True)
    monkeypatch.setattr(node, '_safe_abort',
                        lambda *a, **k: calls.append('safe_abort'))
    monkeypatch.setattr(node, '_recenter', lambda *a, **k: calls.append('recenter'))
    monkeypatch.setattr(node, '_publish_catch_armed',
                        lambda a: calls.append(('armed', bool(a))))
    monkeypatch.setattr(node, '_publish_prime_hold',
                        lambda h: calls.append(('prime_hold', bool(h))))
    monkeypatch.setattr(node, '_publish_pretilt_hold',
                        lambda h: calls.append(('pretilt_hold', bool(h))))
    monkeypatch.setattr(node, '_publish_reach_center',
                        lambda c: calls.append(('centre', tuple(float(v)
                                                                for v in c))))
    monkeypatch.setattr(node._prime_dispatched_pub, 'publish',
                        lambda msg: calls.append(('prime_dispatched',
                                                  bool(msg.data))))
    monkeypatch.setattr(node._vel_scale_pub, 'publish',
                        lambda msg: calls.append(('vel_scale', float(msg.data))))
    monkeypatch.setattr(node, '_announce_toss',
                        lambda seq, state=None: (
                            calls.append(('announce', round(seq.t_release, 4))),
                            seq.note_announcement())[0])
    def _dispatch(seq, state=None):
        calls.append(('dispatch', round(seq.t_release, 4)))
        # The REAL method sets this before the ack, and it is not bookkeeping:
        # it is the gate on BOTH release-evidence channels (a CONFIRMED track or
        # a stroke signature before our own dispatch can only be a phantom). A
        # stub that skips it leaves the FSM waiting for evidence it has
        # forbidden itself to see, and every cycle dies ABORTED_NO_RELEASE.
        state = node._toss_committed if state is None else state
        with node._lock:
            state.throw_dispatched = True
        return 'ok', ''

    monkeypatch.setattr(node, '_dispatch_toss_throw', _dispatch)
    monkeypatch.setattr(node, '_position_platform_for_toss',
                        lambda seq, state=None: (
                            calls.append('position'),
                            seq.note_position_noop(clock.perf_counter()))[0])
    clock.attach(node)
    # Seed the plant BEFORE the first tick: cycle 1's CHECKING runs before any
    # sleep, and an unseeded cup reads UNKNOWN (which is the shipped
    # fail-closed refusal, not a harness convenience worth working around —
    # the real graph has been publishing /hand_telemetry for seconds by then).
    clock._plant(clock.t - 0.02)
    clock._plant(clock.t)
    return node, calls


def _sample_both_slots(node, monkeypatch):
    """Sample the two slots on every tick where BOTH are live, through the real
    `_tick_toss_pipeline`.

    Sampling at the PROMOTION would sample nothing, and that is S1′ working
    rather than a harness problem: the committed slot is freed on the same tick
    the staged slot commits (its terminal is resolved FIRST, by construction),
    so at the promotion instant there is exactly one cycle again. The interval
    where two exist is the previous cycle's FLIGHT, which is the whole point."""
    seen = []
    real_tick = node._tick_toss_pipeline

    def spy(now, session, **kw):
        with node._lock:
            c, s = node._active_seq, node._toss_staged_seq
        if c is not None and s is not None:
            seen.append({
                'now': now,
                'landing_read': node._expected_landing_perf(),
                'next_read': node._expected_next_cycle_perf(),
                'prev_read': node._expected_prev_landing_perf(),
                'committed_landing': float(c.t_release) + float(c.flight_time_s),
                'staged_release': float(s.t_release),
                'staged_landing': float(s.t_release) + float(s.flight_time_s),
                'staged_at': float(s.staged_at),
            })
        return real_tick(now, session, **kw)

    monkeypatch.setattr(node, '_tick_toss_pipeline', spy)
    return seen


def _run_pipelined(monkeypatch, *, num_throws=3, dwell=0.55, height=1.30,
                   delay=DELAY, seat_edge_s=_SEAT_EDGE_S):
    clock = _PipelineClock(seat_edge_s=seat_edge_s)
    monkeypatch.setattr(rcn, 'time', clock)
    node, calls = _pipelined_node(clock, monkeypatch)
    gh = _ContGoalHandle(num_throws=num_throws, dwell=dwell, delay=delay,
                         throw_height=height, vel_scale=0.9)
    result = node._execute_toss_continuous(gh)
    return node, calls, result, gh


def test_the_shipped_default_is_the_two_slot_pipeline():
    """T-U13 / the rollback, RE-AIMED at the value that actually ships.

    **The operator turned the key on 2026-08-28** after the first pipelined
    sitting, and it rides as the committed default — so the assertion that used
    to read ``is False`` reads ``is True``, and it is the ONE assertion in this
    file the flip is allowed to move. Its intent is unchanged and is the reason
    it exists: `plans/active/toss-pipelined-preamble.md` § 9.5 level 1 says one
    YAML key plus a colcon build selects the machine, and this is what proves
    the key is really what selects rather than a second switch somewhere.

    It reads the GENERATED module's SOURCE rather than the imported attribute,
    because the autouse fixture above pins that attribute per test: an assertion
    about what SHIPS must not be readable from a value a fixture is allowed to
    move. ``hw.__file__`` is the generated artifact
    (`config/generate_config.py` writes it), so this also fails if the codegen
    stops emitting the key at all."""
    source = Path(hw.__file__).read_text()
    tree = ast.parse(source)
    shipped = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if getattr(target, 'id', None) == 'JB_OP_TOSS_PIPELINE_ENABLED':
                shipped['value'] = ast.literal_eval(node.value)
    assert 'value' in shipped, 'the codegen no longer emits the pipeline key'
    assert shipped['value'] is True


def test_the_session_reads_the_flag_and_the_single_toss_never_does(monkeypatch):
    """T-G3: the flag is a SESSION property, resolved once per goal, and the
    single `Toss` action does not read it at all — it has no previous cycle to
    pipeline behind, so its arithmetic is unchanged whatever the key says."""
    import inspect
    src = inspect.getsource(rcn.ReloadCoordinatorNode._execute_toss)
    assert 'PIPELINE_ENABLED' not in src
    assert 'staged' not in src
    sess_src = inspect.getsource(
        rcn.ReloadCoordinatorNode._execute_toss_continuous)
    assert sess_src.count('JB_OP_TOSS_PIPELINE_ENABLED') == 1


def test_the_committed_slot_is_ticked_before_the_staged_slot():
    """T-U15 — STRUCTURAL, on `_tick_toss_pipeline`'s AST (the
    `test_the_worker_is_drained_before…` idiom).

    Order is not cosmetic: the committed slot owns the hand and the airborne
    ball, so its terminal must be resolved before the staged slot is offered a
    commit. That ordering is what makes S1′ true WITHIN a tick rather than
    merely between ticks, and it is what lets the staged slot's gate consume the
    very verdict the committed slot just minted, from the same read."""
    src = (Path(rcn.__file__).with_suffix('.py')).read_text()
    fn = next(n for n in ast.walk(ast.parse(src))
              if isinstance(n, ast.FunctionDef)
              and n.name == '_tick_toss_pipeline')
    # The two `_step_toss_sequence` call sites, in SOURCE ORDER, each tagged by
    # the sequencer it steps. Reading the AST rather than the text is what makes
    # this survive a re-wrap of the argument list — the thing being pinned is
    # the ORDER, not the formatting.
    stepped = []
    for node_ in ast.walk(fn):
        if (isinstance(node_, ast.Call)
                and isinstance(node_.func, ast.Attribute)
                and node_.func.attr == '_step_toss_sequence'):
            first = node_.args[0]
            assert isinstance(first, ast.Name), ast.dump(first)
            stepped.append((node_.lineno, first.id))
    assert len(stepped) == 2, 'exactly two slots are stepped'
    stepped.sort()
    assert [name for _, name in stepped] == ['seq_c', 'seq_s'], stepped
    body = ast.get_source_segment(src, fn) or ''
    assert body.index('THE COMMITTED SLOT') < body.index('THE STAGED SLOT')


def test_a_pipelined_session_stages_every_cycle_after_the_first(monkeypatch):
    """THE headline, end to end through the real FSMs: cycle 1 runs serially
    (nothing to pipeline behind, and it is the cycle that ARMS the session) and
    every later cycle's preamble runs inside the previous cycle's FLIGHT.

    The evidence is the record's own `staged_at_s`: it is null for cycle 1 and,
    for every later cycle, sits BEFORE the previous cycle's scheduled landing —
    i.e. off the critical path, which is the whole claim."""
    node, calls, result, gh = _run_pipelined(monkeypatch, num_throws=3)
    assert result.outcome == 'COMPLETED', result.outcome
    assert result.catches_confirmed == 3
    # three announcements, three dispatches, in that interleaved order
    seq_calls = [c for c in calls if c[0] in ('announce', 'dispatch')]
    assert [c[0] for c in seq_calls] == ['announce', 'dispatch'] * 3
    # …and each pair names the SAME release instant: the announcement cannot be
    # a tick stale, because both went out in one tick.
    for i in range(0, 6, 2):
        assert seq_calls[i][1] == seq_calls[i + 1][1]


def test_the_staged_preamble_runs_inside_the_previous_flight(monkeypatch):
    """§ 2.2's claim, measured rather than asserted: the staged cycle reaches
    PHASE_STAGED before the previous cycle's ball has landed.

    That is the definition of "off the critical path" — everything the serial
    ladder charged to the dwell is spent while a ball is in the air."""
    clock = _PipelineClock()
    monkeypatch.setattr(rcn, 'time', clock)
    node, calls = _pipelined_node(clock, monkeypatch)
    both = _sample_both_slots(node, monkeypatch)
    node._execute_toss_continuous(
        _ContGoalHandle(num_throws=3, dwell=0.55, delay=DELAY,
                        throw_height=1.30, vel_scale=0.9))
    assert both, 'no tick had both slots live — nothing pipelined'
    staged_ready = [s for s in both if s['staged_at'] > 0.0]
    assert staged_ready, 'no staged cycle ever reached PHASE_STAGED'
    for s in staged_ready:
        # THE claim: the staged preamble finished before the ball the previous
        # cycle threw had landed. Everything the serial ladder charged to the
        # dwell was spent while a ball was in the air.
        assert s['staged_at'] < s['committed_landing'], s


def test_the_sensor_is_told_the_committed_slots_landing(monkeypatch):
    """T-U8 — § 2.5(b), the slot-naming rule, with BOTH slots live and holding
    DIFFERENT landings.

    Getting this wrong evaluates cycle k's arrival verdict against cycle k+1's
    landing. The three reads and their slots:
      `_expected_landing_perf`      -> the COMMITTED slot;
      `_expected_next_cycle_perf`   -> the STAGED slot (its ACTUALS, § 2.5a);
      `_expected_prev_landing_perf` -> the previously-committed slot."""
    clock = _PipelineClock()
    monkeypatch.setattr(rcn, 'time', clock)
    node, calls = _pipelined_node(clock, monkeypatch)
    both = _sample_both_slots(node, monkeypatch)
    node._execute_toss_continuous(
        _ContGoalHandle(num_throws=3, dwell=0.55, delay=DELAY,
                        throw_height=1.30, vel_scale=0.9))
    assert both, 'no tick had both slots live'
    for s in both:
        assert s['committed_landing'] != s['staged_landing'], s
        assert s['landing_read'] == pytest.approx(
            s['committed_landing']), 'the COMMITTED slot'
        assert s['next_read'][0] == pytest.approx(
            s['staged_release']), 'the STAGED slot, its ACTUAL'
        assert s['next_read'][1] == pytest.approx(
            s['staged_landing']), 'the STAGED slot, its ACTUAL'
        assert (s['prev_read'] is None
                or s['prev_read'] < s['committed_landing']), s


def test_the_clamp_reads_the_staged_actual_not_the_prediction(monkeypatch):
    """§ 2.5(a): under the pipeline the clamp stops being a PREDICTION.

    Today's `_set_toss_next_cycle_perf` latches `landing + dwell` a cycle early;
    C-POSSESS-1.C's own text names that as the weakness of the rule it
    superseded. With a staged slot live the clamp reads that slot's real
    `t_release` — slip included — so a release that ran late no longer pulls the
    two window ends apart."""
    clock = _PipelineClock()
    monkeypatch.setattr(rcn, 'time', clock)
    node, calls = _pipelined_node(clock, monkeypatch)
    node._toss_session_live = True
    committed = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                              flight_time_s=0.8, release_at_perf=100.0)
    committed.start(90.0)
    staged = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                           flight_time_s=0.8, release_at_perf=101.4)
    staged.start(100.0)
    with node._lock:
        node._active_seq = committed
        node._toss_committed.next_release_perf = 999.0   # the stale PREDICTION
        node._toss_committed.next_landing_perf = 999.8
        node._toss_staged_seq = staged
    assert node._expected_next_cycle_perf() == (101.4, pytest.approx(102.2))
    # …a SLIP moves the release, and the clamp follows it for free.
    staged._t_release = 101.46
    assert node._expected_next_cycle_perf()[0] == pytest.approx(101.46)
    # …and with no staged slot the latched prediction is the honest fallback.
    with node._lock:
        node._toss_staged_seq = None
    assert node._expected_next_cycle_perf() == (999.0, 999.8)


def test_the_unwind_discards_the_staged_slot_before_any_go_home(monkeypatch):
    """T-U7 / S7 — the § 2.4.3 unwind, ordered.

    The staged slot is discarded, then the session arming comes down, and only
    THEN is a profile installed. A `go_home` traversing under a slot the machine
    still believes in is the arm-mid-move seam with the two events in the other
    order."""
    clock = _PipelineClock()
    monkeypatch.setattr(rcn, 'time', clock)
    node, calls = _pipelined_node(clock, monkeypatch)
    node._toss_session_live = True
    node._toss_session_armed = True
    node._toss_session_center_mm = (0.0, 0.0, 170.0)
    staged = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                           flight_time_s=0.8, release_at_perf=101.4)
    staged.start(100.0)
    with node._lock:
        node._toss_staged_seq = staged
        node._toss_staged = rcn.TossCycleState(staged=True)
    order = []
    monkeypatch.setattr(node, '_discard_toss_staged',
                        lambda reason: order.append(('discard', reason)))
    monkeypatch.setattr(node, '_disarm_session', lambda: order.append('disarm'))
    node._toss_safe_abort(node._toss_committed)
    assert order[0][0] == 'discard'
    assert order[1] == 'disarm'
    assert 'safe_abort' in calls
    assert order.index(('discard', 'DRAINED')) == 0
    # …and the discard really does precede the ladder, not merely the disarm.
    assert calls.index('safe_abort') >= 0


def test_every_go_home_path_drains_the_staged_slot(monkeypatch):
    """S7's structural half, extended to B4: the drain is the ONE place the
    staged slot is dropped on a teardown, and every `go_home` path is already
    pinned to be fronted by it (the B3 structural test). This adds the other
    end — that the drain actually discards, and does so BEFORE the disarm."""
    src = (Path(rcn.__file__).with_suffix('.py')).read_text()
    body = src.split('def _drain_pipeline_and_disarm(')[1].split('\n    def ')[0]
    assert body.index('_discard_toss_staged(') < body.index('_disarm_session()')


def test_a_discarded_slot_closes_its_own_record(monkeypatch):
    """§ 4 B4: "the staged slot is discarded, its record is closed with
    `staged_discarded_reason`". A discard that left no row would make a dropped
    cycle an ABSENCE in the census, which is exactly the shape of a defect
    nobody counts."""
    clock = _PipelineClock()
    monkeypatch.setattr(rcn, 'time', clock)
    node, calls = _pipelined_node(clock, monkeypatch)
    published = []
    monkeypatch.setattr(node, '_publish_toss_record',
                        lambda result, ctx=None, cycle_state=None, seq=None:
                        published.append((result.outcome, cycle_state)))
    staged = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                           flight_time_s=0.8, release_at_perf=101.4)
    staged.start(100.0)
    state = rcn.TossCycleState(staged=True)
    with node._lock:
        node._toss_staged_seq = staged
        node._toss_staged = state
    dropped = node._discard_toss_staged('DRAINED')
    assert dropped is state
    assert state.discarded_reason == 'DRAINED'
    assert published == [('DISCARDED_DRAINED', state)]
    with node._lock:
        assert node._toss_staged is None and node._toss_staged_seq is None
    # …and it is idempotent: a second drain has nothing to drop and says so.
    assert node._discard_toss_staged('DRAINED') is None


def test_the_stay_terminal_holds_catch_armed_high_for_a_staged_successor(
        monkeypatch):
    """The armed→announce gap, re-argued for the pipeline.

    `catch_coordinator._on_throw_announcement` DROPS an announcement that
    arrives while `catch/armed` is False, and dropping it loses the C-HAND-1
    stroke-busy latch. Under the pipeline the previous cycle's STAY and the next
    cycle's announcement land in the SAME tick, so a per-cycle armed False/True
    toggle would put two edges and an announcement into one wait-set with no
    cross-topic ordering guarantee. STAY installs no `go_home`, so a standing
    latch has no move to land inside — it comes down at `_disarm_session`."""
    clock = _PipelineClock()
    monkeypatch.setattr(rcn, 'time', clock)
    node, calls = _pipelined_node(clock, monkeypatch)
    node._toss_session_live = True
    node._toss_session_armed = True
    # (a) a staged successor is waiting -> armed stays HIGH
    with node._lock:
        node._toss_staged = rcn.TossCycleState(staged=True)
    node._toss_stay(node._toss_committed)
    assert ('armed', False) not in calls
    # (b) no successor (the last cycle) -> armed comes down, as it always did
    with node._lock:
        node._toss_staged = None
    node._toss_stay(node._toss_committed)
    assert ('armed', False) in calls


def test_a_cycle_that_must_move_does_not_stage(monkeypatch):
    """T-U5 — asserted on the DECISION STREAM, not on a flag.

    A cycle whose POSITIONING must COMMAND a move cannot stage: the move would
    traverse the platform during the previous cycle's flight, under a ball the
    catch is armed for. It falls back to the serial path, which is correct and
    is not a regression — it simply does not get faster."""
    clock = _PipelineClock()
    monkeypatch.setattr(rcn, 'time', clock)
    node, calls = _pipelined_node(clock, monkeypatch)
    node._toss_session_live = True
    session = TossSessionSequencer(num_throws=3, dwell_time_s=0.55,
                                   throw_delay_s=DELAY, flight_time_s=0.8,
                                   pipelined=True)
    session.start(clock.t)
    node._toss_session_ref = session
    session.step(clock.t)                          # cycle 1 -> the staging slot
    committed = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                              flight_time_s=0.8, release_at_perf=clock.t + 1.0)
    committed.start(clock.t)
    with node._lock:
        node._active_seq = committed
    session.note_cycle_committed()
    session.step(clock.t)                          # cycle 2 -> stage attempt
    # The platform is nowhere near the nominated B, so POSITIONING must move.
    node._start_pipelined_cycle(
        _ContGoalHandle(), session, (120.0, -90.0, 170.0), 0.8, 0.9,
        raw_goal={})
    with node._lock:
        assert node._toss_staged is None, 'a mover must not occupy the slot'
    assert session.cycle_live is False
    assert session._stage_declined is True


class _AimedPipelineClock(_PipelineClock):
    """``_PipelineClock`` plus the ORIENTATION half of the plant.

    The base plant republishes ``trajectory/commanded_pose`` at a LEVEL
    orientation forever, which is only true of a zero-aim session. On an AIMED
    chain the pose the platform ends a cycle holding is the one Tier 8b's
    deferred A→B reach commanded, so this plant reads that reach back off the
    wire and decodes it EXACTLY as ``trajectory_node._catch_target_from_msg``
    does (quat → matrix → rotvec, the intent frame). Without it the harness
    would answer the question this test asks — "what is the platform holding
    when the next cycle decides whether to stage?" — with a constant."""

    def _plant(self, t):
        super()._plant(t)
        published = self._node._dyn_target_pub.published
        if not published:
            return
        q = published[-1].target_quat
        rotvec = rot_matrix_to_rotvec(quat_to_rot_matrix(
            float(q.w), float(q.x), float(q.y), float(q.z)))
        with self._node._lock:
            self._node._commanded_pose = (
                tuple(self._node._commanded_pose[:3])
                + tuple(float(v) for v in rotvec))


def _arm_aim(node, monkeypatch, rx, ry):
    """Arm an aim of ``(rx, ry)`` rad AND restore the real announcement.

    Both halves are needed and neither is scenery: the aim is what makes the
    commanded release tilted, and the real ``_announce_toss`` is what stashes the
    landing the deferred reach is built from (`_pipelined_node`'s stub records
    the call and stashes nothing, so the reach would publish nothing at all and
    the plant above would have nothing to read)."""
    real_aim = node._toss_aim_for_goal
    real_announce = rcn.ReloadCoordinatorNode._announce_toss

    def aimed(catch_pose, flight):
        block = real_aim(catch_pose, flight)
        off = aim_target_offset_mm(float(rx), float(ry), float(flight),
                                   float(catch_pose[2]))
        block['aim_rad'] = (float(rx), float(ry))
        block['offset_mm'] = (float(off[0]), float(off[1]))
        return block

    monkeypatch.setattr(node, '_toss_aim_for_goal', aimed)
    monkeypatch.setattr(node, '_announce_toss',
                        lambda seq, state=None: real_announce(node, seq, state))


def test_an_aimed_colocated_chain_stages_now_that_the_reach_holds_the_pretilt(
        monkeypatch):
    """**P-4** — `tests/hardware/session_cadence_ladder.md` carried finding 2, at
    the level the finding was written about: an AIMED chain on the SHIPPED tier
    (8b) STAGES.

    It could not, and the reason was mechanical rather than a threshold: the
    deferred A→B reach published the catch policy's receive tilt (level for a
    self-toss), so every cycle ended with the aim taken back OUT of the platform
    orientation, the next cycle's census-B1 skip honestly declined, and § 2.4.1's
    rule — *a cycle stages only if its positioning decision is SKIP* — kept the
    pipeline inert on the tier that ships. The fix is in
    ``_publish_toss_reach``/``_toss_reach_quat``; this is its consequence, driven
    end to end through the real FSMs.

    The inertness test above (`test_a_cycle_that_must_move_does_not_stage`) is
    unchanged and still right: a cycle that genuinely must traverse still refuses
    to stage. What changed is that an armed aim is no longer such a cycle."""
    clock = _AimedPipelineClock()
    monkeypatch.setattr(rcn, 'time', clock)
    node, calls = _pipelined_node(clock, monkeypatch)
    _arm_aim(node, monkeypatch, math.radians(0.8), math.radians(-0.5))
    both = _sample_both_slots(node, monkeypatch)
    reaches = []                      # (commanded release, published target)
    real_reach = node._publish_toss_reach

    def spy_reach(state=None):
        real_reach(state)
        st = node._toss_committed if state is None else state
        reaches.append((st.release_cmd, node._dyn_target_pub.published[-1]))

    monkeypatch.setattr(node, '_publish_toss_reach', spy_reach)
    result = node._execute_toss_continuous(
        _ContGoalHandle(num_throws=3, dwell=0.55, delay=DELAY,
                        throw_height=1.30, vel_scale=0.9))
    assert result.outcome == 'COMPLETED', result.outcome
    # The premise: every cycle's aim really was armed and really tilted the
    # release, and the reach really held that pre-tilt rather than the level
    # receive the policy computes for a self-toss.
    assert len(reaches) == 3
    for rel, out in reaches:
        assert node._release_is_tilted(rel) is True
        pre = rcn.ReloadCoordinatorNode._tilt_quaternion(rel.tilt_rx,
                                                         rel.tilt_ry)
        assert (out.target_quat.w, out.target_quat.x, out.target_quat.y,
                out.target_quat.z) == (pre.w, pre.x, pre.y, pre.z)
        assert out.target_quat.w != 1.0
    # THE claim: cycles staged, and each staged preamble ran inside the previous
    # cycle's flight. Before the fix `both` was empty — the chain never staged.
    assert both, 'no tick had both slots live — the aimed chain is still inert'
    staged_ready = [s for s in both if s['staged_at'] > 0.0]
    assert staged_ready, 'no staged cycle ever reached PHASE_STAGED'
    for s in staged_ready:
        assert s['staged_at'] < s['committed_landing'], s


# ── the DISPLACED-chain stale-site defect (bag 2026-08-28_14-48-38) ─────────


class _DisplacedPipelineClock(_PipelineClock):
    """``_PipelineClock`` plus the two things that actually MOVE the platform on
    a displaced chain, which the base plant models as a constant:

    * the POSITIONING ``go_to_pose`` — the platform ends at the pre-tilt pose at
      the throw site A;
    * **the deferred A->B reach**, published at ``t_release`` and arriving by the
      landing, which is what puts the platform at B *during the previous cycle's
      flight* — i.e. inside the window between a staged cycle's nomination and
      its commit. That is the whole mechanism of the bag, and a harness whose
      ``trajectory/commanded_position`` never moves cannot see it.

    Commands are queued with the instant they arrive and applied on the tick
    that reaches it, so the ORDER a real session sees is preserved."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self._queue = []                       # (arrival_t, xyz, rotvec)
        self._pose = ((0.0, 0.0, 170.0), (0.0, 0.0, 0.0))
        self._reaches_seen = 0
        self.commands = []                     # (kind, arrival_t, xyz)

    def command(self, arrival_t, pos, rotvec, kind):
        self._queue.append((float(arrival_t), tuple(float(v) for v in pos),
                            tuple(float(v) for v in rotvec)))
        self.commands.append((kind, round(float(arrival_t), 3),
                              tuple(round(float(v), 3) for v in pos)))

    def _plant(self, t):
        super()._plant(t)                      # re-drives hand + cup
        node = self._node
        published = node._dyn_target_pub.published
        if len(published) > self._reaches_seen:
            self._reaches_seen = len(published)
            out = published[-1]
            q = out.target_quat
            rot = rot_matrix_to_rotvec(quat_to_rot_matrix(
                float(q.w), float(q.x), float(q.y), float(q.z)))
            seq = node._active_seq
            # the reach carries `arrival = the announced landing` by
            # construction, so that is when the platform is at B
            land = (float(seq.t_release) + float(seq.flight_time_s)
                    if seq is not None else t)
            self.command(land, (float(out.target_pos.x),
                                float(out.target_pos.y), 170.0), rot, 'reach')
        while self._queue and self._queue[0][0] <= t:
            _, pos, rot = self._queue.pop(0)
            self._pose = (pos, rot)
        with node._lock:
            node._commanded_pos_mm = self._pose[0]
            node._commanded_pose = self._pose[0] + self._pose[1]


def _displaced_pipelined_node(clock, monkeypatch):
    """`_pipelined_node` with the three things a DISPLACED chain needs and a
    co-located one does not: a go_to_pose that accepts (POSITIONING really
    commands a move here), the REAL `_announce_toss` (it stashes the landing the
    deferred reach is built from — the stub records the call and stashes
    nothing, so the reach would publish nothing at all), and the plant hook that
    walks the platform to each commanded pose."""
    node, calls = _pipelined_node(clock, monkeypatch)
    real_announce = rcn.ReloadCoordinatorNode._announce_toss
    monkeypatch.setattr(node, '_announce_toss',
                        lambda seq, state=None: real_announce(node, seq, state))
    monkeypatch.setattr(
        node, '_wait_future',
        lambda fut, timeout_s=2.0: types.SimpleNamespace(
            accepted=True, planned_duration_s=0.30, code='OK', message=''))
    real_position = node._position_platform_for_toss

    def positioning(seq, state=None):
        st = node._toss_committed if state is None else state
        real_position(seq, state)
        if st.positioning_move and st.platform_target_mm is not None:
            rel = node._toss_commanded_release(st)
            rot = ((float(rel.tilt_rx), float(rel.tilt_ry), 0.0)
                   if node._release_is_tilted(rel) else (0.0, 0.0, 0.0))
            clock.command(clock.t, st.platform_target_mm, rot, 'position')
    monkeypatch.setattr(node, '_position_platform_for_toss', positioning)
    return node, calls


@pytest.mark.parametrize('stale_nomination', [False, True])
def test_a_displaced_chain_never_throws_from_a_site_it_did_not_nominate(
        monkeypatch, stale_nomination):
    """**THE bag-2026-08-28_14-48-38 regression**, end to end through the real
    FSMs, the real session and a plant that actually moves.

    The defect: on a DISPLACED chain (catch pose B != the throw site A) the
    first staged cycle read its throw site LIVE at stage time — during the
    previous cycle's flight, with the platform still at A — and cached a
    census-B1 "already positioned" that was honestly true at that instant. The
    previous cycle's deferred A->B reach then moved the platform ~71 mm and
    mirrored the tilt; nothing re-read; the staged cycle released from B with a
    release state solved for A. Measured landing: x=+3.98 mm against a +70 mm
    target — the ball came back to HOME — and because only the first chained
    cycle after each un-staged one carries the defect, the sitting alternated
    caught/missed deterministically.

    **The invariant asserted here is the one the operator can see**: every
    dispatch's nominated throw site is the site the platform is actually at. It
    holds under BOTH of the fix's layers, which is why they are parametrised
    rather than tested apart:

    * ``stale_nomination=False`` — the shipped path. The staged cycle nominates
      the PREDICTED chain site (~B), so its positioning pose is at B while the
      platform is still at A, ``positioning_move`` is honestly True, and the
      skip-only rule declines to stage it at all. It runs serially, one cycle
      later, from a live read;
    * ``stale_nomination=True`` — the pre-fix nomination, restored, so the
      staged cycle really does cache the stale site. The COMMIT gate's
      re-validation then refuses it ``REJECTED_SITE_MOVED`` with nothing
      announced and nothing armed, and the session rebuilds it serially. This
      is the belt: the class stays closed even if the nomination regresses.
    """
    clock = _DisplacedPipelineClock()
    monkeypatch.setattr(rcn, 'time', clock)
    node, calls = _displaced_pipelined_node(clock, monkeypatch)
    B = (70.0, 0.0, 170.0)
    if stale_nomination:
        monkeypatch.setattr(
            node, '_predicted_chain_site_mm',
            lambda catch_pose, flight: (
                (lambda p: None if p is None else (float(p[0]), float(p[1])))(
                    node._live_commanded_position(clock.t))))
    dispatches = []
    real_dispatch = node._dispatch_toss_throw

    def spy_dispatch(seq, state=None):
        dispatches.append((tuple(float(v) for v in seq.throw_site_xy_mm),
                           node._live_commanded_position(clock.t)))
        return real_dispatch(seq, state)
    monkeypatch.setattr(node, '_dispatch_toss_throw', spy_dispatch)
    abandoned = []
    real_abandon = TossSessionSequencer.note_stage_abandoned
    monkeypatch.setattr(
        TossSessionSequencer, 'note_stage_abandoned',
        lambda self, reason: (abandoned.append(str(reason)),
                              real_abandon(self, reason))[1])

    result = node._execute_toss_continuous(
        _ContGoalHandle(num_throws=3, dwell=0.55, delay=DELAY,
                        throw_height=1.30, vel_scale=0.9,
                        x=B[0], y=B[1], z=B[2]))
    assert result.outcome == 'COMPLETED', result.outcome
    # The premise: the plant really did traverse, and the reach really is what
    # moved it (a harness that never moved would pass this test vacuously).
    kinds = [c[0] for c in clock.commands]
    assert 'reach' in kinds, clock.commands
    xs = [c[2][0] for c in clock.commands]
    assert max(xs) - min(xs) > 60.0, clock.commands
    # ── THE assertion ──
    assert len(dispatches) == 3
    for site, live in dispatches:
        assert live is not None
        err = math.hypot(site[0] - float(live[0]), site[1] - float(live[1]))
        assert err <= rcn._TOSS_ALREADY_THERE_TOL_MM, (site, live, err)
    # …and the first chained cycle really did decline, by the layer under test.
    if stale_nomination:
        assert 'REJECTED_SITE_MOVED' in abandoned, abandoned
    else:
        assert abandoned and abandoned[0] == 'POSITIONING_MOVE', abandoned
        assert 'REJECTED_SITE_MOVED' not in abandoned, (
            'the honest nomination should never have to be caught by the belt')


def test_the_serial_rebuild_of_a_declined_cycle_commands_the_move(monkeypatch):
    """The other half of the fallback: a cycle that declines to stage is not
    merely dropped — it is REBUILT serially and its POSITIONING really does
    COMMAND the move that makes its nomination true.

    That is what turns the honest refusal into a correct throw rather than a
    lost cycle, and it is why the displaced chain still completes 3/3."""
    clock = _DisplacedPipelineClock()
    monkeypatch.setattr(rcn, 'time', clock)
    node, calls = _displaced_pipelined_node(clock, monkeypatch)
    moves = []
    real_position = node._position_platform_for_toss

    def spy(seq, state=None):
        st = node._toss_committed if state is None else state
        moves.append((bool(st.positioning_move),
                      tuple(float(v) for v in (st.platform_target_mm or ())),
                      tuple(float(v) for v in seq.throw_site_xy_mm)))
        return real_position(seq, state)
    monkeypatch.setattr(node, '_position_platform_for_toss', spy)

    result = node._execute_toss_continuous(
        _ContGoalHandle(num_throws=3, dwell=0.55, delay=DELAY,
                        throw_height=1.30, vel_scale=0.9,
                        x=70.0, y=0.0, z=170.0))
    assert result.outcome == 'COMPLETED', result.outcome
    assert len(moves) == 3, moves
    # cycle 2 is the declined one: rebuilt serially, and it MOVES — from the A
    # it threw cycle 1 from to the B it caught at.
    assert moves[1][0] is True, moves
    assert moves[1][1][0] == pytest.approx(70.0, abs=2.0), moves
    # …and by cycle 3 the chain is co-located and takes the census-B1 skip
    # again, which is the pipeline re-engaging one cycle after the displacement.
    assert moves[2][0] is False, moves


def test_a_staged_cycle_nominates_the_predicted_chain_site_not_the_live_read(
        monkeypatch):
    """The nomination itself, isolated from the ladder.

    A SERIAL cycle nominates the site from the live commanded pose, and that is
    self-consistent because its POSITIONING then COMMANDS the platform there. A
    STAGED cycle's POSITIONING is a skip by construction, so the live read is a
    claim about an instant a whole flight before the throw — it nominates the
    prediction of where it WILL be instead, through the same single
    `_predicted_chain_site_mm` derivation the accept-time
    REJECTED_CHAIN_UNREACHABLE check consults."""
    clock = _PipelineClock()
    monkeypatch.setattr(rcn, 'time', clock)
    node, calls = _pipelined_node(clock, monkeypatch)
    B = (70.0, 0.0, 170.0)
    flight = node._resolve_toss_flight_s(1.30)
    live = node._live_commanded_position(clock.t)
    predicted = node._predicted_chain_site_mm(B, flight)
    assert predicted is not None
    # The premise: on a displaced goal the two answers are ~70 mm apart, which
    # is the whole defect and is four times the census-B1 tolerance.
    assert abs(predicted[0] - float(live[0])) > 60.0
    staged_seq, _ = node._build_toss_cycle(
        B, flight, 5.0, 0.9, delay_is_cadence=True,
        release_at_perf=clock.t + 3.0, staged=True)
    assert staged_seq.throw_site_xy_mm == pytest.approx(predicted)
    # …and it therefore refuses to stage, by the SKIP-ONLY rule and not by a
    # second displaced-specific test: its positioning pose is at B and the
    # platform is at A.
    assert staged_seq.positioning_move_expected is True
    assert staged_seq.staged is False
    serial_seq, _ = node._build_toss_cycle(B, flight, 5.0, 0.9,
                                           delay_is_cadence=True)
    assert serial_seq.throw_site_xy_mm == pytest.approx(
        (float(live[0]), float(live[1])))


@pytest.mark.parametrize('pose', [(0.0, 0.0, 170.0), (30.0, -20.0, 170.0)])
def test_on_a_colocated_chain_the_prediction_is_the_live_read_bit_for_bit(
        monkeypatch, pose):
    """**The no-regression half, and it is an IDENTITY rather than a bound.**

    The shipped steady state of a chained session is CO-LOCATED: the platform
    parks at B and throws from B. There the parked-centroid prediction is the
    live read's own fixed point — a self-toss lands with a vertical velocity, so
    the receive tilt is identity, the cup swing is zero, and the catch policy
    parks the centroid exactly on B. So the 2026-08-28 nomination change cannot
    move a single co-located decision, because it does not move a single
    co-located NUMBER.

    Asserted as `==` on purpose. A tolerance here would hide the day this stops
    being an identity — at which point the co-located chain has acquired a
    residual and somebody needs to know why."""
    clock = _PipelineClock()
    monkeypatch.setattr(rcn, 'time', clock)
    node, calls = _pipelined_node(clock, monkeypatch)
    with node._lock:
        node._commanded_pos_mm = tuple(float(v) for v in pose)
        node._commanded_pose = tuple(float(v) for v in pose) + (0.0, 0.0, 0.0)
        node._commanded_pos_mono = clock.t
        node._commanded_pose_mono = clock.t
    flight = node._resolve_toss_flight_s(1.30)
    live = node._live_commanded_position(clock.t)
    predicted = node._predicted_chain_site_mm(pose, flight)
    assert predicted == (float(live[0]), float(live[1]))


def test_the_honest_cache_check_re_reads_the_live_pose_and_re_derives_nothing():
    """STRUCTURAL — the re-validation's two properties, pinned where they live.

    1. it is the SAME predicate against the SAME cached target the BUILD used
       (`_toss_already_positioned` fed `state.platform_target_mm`). A second
       derivation here would be free to disagree with the one the pre-dispatch
       budget was charged for — the exact accept-vs-runtime gap the 2026-08-23
       single-decision rule closed;
    2. what IS fresh is the live pose, and it is read on the tick the
       observation snapshot is built — the commit tick — rather than cached at
       stage time."""
    import inspect
    # The COMPILED identifier set, not the source text — the
    # `test_the_census_never_feeds_a_budget` idiom, and for the same reason: the
    # docstring below names `_toss_positioning_xyz` precisely to say it is NOT
    # called here, and a source grep would read that sentence as the call.
    names = (set(rcn.ReloadCoordinatorNode._staged_site_ok.__code__.co_names)
             | set(rcn.ReloadCoordinatorNode._staged_site_ok
                   .__code__.co_varnames))
    assert '_toss_already_positioned' in names
    assert 'platform_target_mm' in names
    assert '_toss_positioning_xyz' not in names, 'no second derivation'
    predicate = inspect.getsource(
        rcn.ReloadCoordinatorNode._toss_already_positioned)
    assert '_live_commanded_pose(' in predicate
    # `_staged_observations` may source the field from exactly two places, and
    # BOTH are the current tick's own read (audit fix, 2026-08-28): it calls
    # `_staged_site_ok` itself, or — when `_tick_toss_pipeline` tells it the
    # tick's one snapshot was already built FROM the staged state — it carries
    # `obs.staged_site_ok`, which `_build_toss_observations` produced from
    # `_staged_site_ok` on that same tick. That is § 2.4.4's one-read doctrine
    # applied to this field; the second live read it replaces was two reads for
    # one decision, free to disagree across the gap.
    staged_obs_names = set(
        rcn.ReloadCoordinatorNode._staged_observations.__code__.co_names)
    assert '_staged_site_ok' in staged_obs_names
    assert 'staged_site_ok' in staged_obs_names
    builder_names = set(
        rcn.ReloadCoordinatorNode._build_toss_observations.__code__.co_names)
    assert '_staged_site_ok' in builder_names, (
        'the other producer must still take the live read itself, or the '
        'carried value would have no fresh source')
    staged_obs = inspect.getsource(
        rcn.ReloadCoordinatorNode._staged_observations)
    assert 'state.staged_site_ok' not in staged_obs, (
        'the re-validation may never be read off the cycle STATE — a value '
        'cached there IS the stage-time latch this contract forbids')


def test_the_pipelined_builder_passes_the_beat_and_the_stage_flag(monkeypatch):
    """The B2 seam finally carrying a value, pinned as a kwarg set — the
    pipelined twin of `test_every_cycle_goes_through_the_shared_builder`.

    That test pins the SERIAL kwargs and is deliberately unchanged: the flip is
    confined to the pipelined path, which is what "zero existing assertion
    changes on the flag-false path" means. Cycle 1 is serial and gets the
    derived release (0.0); every later cycle is TOLD its beat."""
    clock = _PipelineClock()
    monkeypatch.setattr(rcn, 'time', clock)
    node, calls = _pipelined_node(clock, monkeypatch)
    seen = []
    real_build = node._build_toss_cycle

    def spy(catch_pose, flight, throw_delay, vel_scale, **kw):
        seen.append(kw)
        return real_build(catch_pose, flight, throw_delay, vel_scale, **kw)

    monkeypatch.setattr(node, '_build_toss_cycle', spy)
    node._execute_toss_continuous(
        _ContGoalHandle(num_throws=3, dwell=0.55, delay=DELAY,
                        throw_height=1.30, vel_scale=0.9))
    assert len(seen) == 3
    assert set(seen[0]) == {'delay_is_cadence', 'release_at_perf', 'staged'}
    assert seen[0]['delay_is_cadence'] is True
    assert seen[0]['staged'] is False and seen[0]['release_at_perf'] == 0.0
    for kw in seen[1:]:
        assert kw['staged'] is True
        assert kw['release_at_perf'] > 0.0


def test_the_record_carries_the_slip_and_the_commit_instant(monkeypatch):
    """The runbook's PIPE-1 row, sourced. `commit_slip_s` is the number the
    operator scores the § 1.4 prediction from, and a slip RISING across a
    session is a loop-cost regression — neither is readable if the record does
    not carry it.

    A zero slip is RECORDED, not nulled: a 0.000 is a measurement (the commit
    fired on its scheduled tick) and a null is "this cycle had no commit gate".
    A distribution that silently drops its zeros is not the one the prediction
    was made about.

    `commit_slips` rides next to it (2026-08-28): the slip says HOW LATE, the
    count says HOW MANY ITERATIONS, and one late tick on a healthy loop is a
    different finding from a loop chronically over period even when the two
    produce the same lateness. That distinction is the stated input to the
    deferred `NODE_LOOP_PERIOD_S` decision, so it has to reach the corpus."""
    clock = _PipelineClock()
    monkeypatch.setattr(rcn, 'time', clock)
    node, calls = _pipelined_node(clock, monkeypatch)
    rows = []
    monkeypatch.setattr(node, '_publish_toss_record',
                        lambda result, ctx=None, cycle_state=None, seq=None:
                        rows.append(node._toss_record_fields(
                            result, ctx, cycle_state, seq)))
    node._execute_toss_continuous(
        _ContGoalHandle(num_throws=3, dwell=0.55, delay=DELAY,
                        throw_height=1.30, vel_scale=0.9))
    assert len(rows) == 3
    assert rows[0]['staged_at_s'] is None, 'cycle 1 ran serially'
    assert rows[0]['commit_slip_s'] is None
    assert rows[0]['commit_slips'] is None
    for row in rows[1:]:
        assert row['staged_at_s'] is not None
        assert row['commit_at_s'] is not None
        assert row['commit_slip_s'] is not None and row['commit_slip_s'] >= 0.0
        assert row['commit_slips'] is not None and row['commit_slips'] >= 0
        assert row['staged_discarded_reason'] is None


def test_the_two_additive_phases_actually_reach_the_wire(monkeypatch):
    """The `.action`'s two new phase strings are a WIRE CONTRACT, and a contract
    nobody can observe is documentation.

    `STAGED` is published as an EDGE (one tick, when the staged cycle finishes
    its preamble) so it cannot shadow the flight phases the runbook watches;
    `COMMITTING` takes the slot for the arm point and for every SLIP tick, which
    is how a commit running away becomes visible live rather than only in the
    record's `commit_slip_s`."""
    from jugglebot.toss_session import (SESSION_PHASE_COMMITTING,
                                        SESSION_PHASE_STAGED)
    clock = _PipelineClock()
    monkeypatch.setattr(rcn, 'time', clock)
    node, calls = _pipelined_node(clock, monkeypatch)
    gh = _ContGoalHandle(num_throws=3, dwell=0.55, delay=DELAY,
                         throw_height=1.30, vel_scale=0.9)
    node._execute_toss_continuous(gh)
    phases = [p for _idx, p, _c in gh.feedbacks]
    assert SESSION_PHASE_STAGED in phases
    assert SESSION_PHASE_COMMITTING in phases
    # At least the two edges (one per staged cycle), plus the handful of ticks
    # where the committed slot has terminalised and the staged one is the only
    # cycle there is to report — the third of the three documented cases. What
    # must NOT happen is STAGED becoming a LEVEL that swamps the flight, so it
    # is asserted as a small minority of the stream rather than as an exact
    # count (an exact count would pin the length of the verdict wait, which is
    # a plant property, not a feedback policy).
    assert phases.count(SESSION_PHASE_STAGED) >= 2
    assert phases.count(SESSION_PHASE_STAGED) < 0.15 * len(phases), (
        phases.count(SESSION_PHASE_STAGED), len(phases))
    # …and the flight phases the runbook scores are still reported.
    assert 'BALL_IN_FLIGHT' in phases
    assert phases.count('BALL_IN_FLIGHT') > phases.count(SESSION_PHASE_STAGED)


def test_a_pipelined_cycle_censuses_its_own_commit_tick(monkeypatch):
    """The loop census must SEE the pipelined pre-dispatch ticks — above all the
    COMMIT tick, which is the single iteration `commit_budget_s` charges its one
    loop period for.

    It is the easiest one to lose: the commit and the UPSTREAM CYCLE'S TERMINAL
    land on the same tick by construction (§ 2.4.4), so a census-end gated on
    "the committed slot did not terminalise" skips exactly the tick that
    matters, and the pipeline's whole pre-dispatch census reads as the empty
    set. A null is a legal value here ("not measured"), so nothing would go red
    — which is why this is asserted rather than assumed."""
    clock = _PipelineClock()
    monkeypatch.setattr(rcn, 'time', clock)
    node, calls = _pipelined_node(clock, monkeypatch)
    rows = []
    monkeypatch.setattr(node, '_publish_toss_record',
                        lambda result, ctx=None, cycle_state=None, seq=None:
                        rows.append(node._toss_record_fields(
                            result, ctx, cycle_state, seq)))
    node._execute_toss_continuous(
        _ContGoalHandle(num_throws=3, dwell=0.55, delay=DELAY,
                        throw_height=1.30, vel_scale=0.9))
    assert len(rows) == 3
    for i, row in enumerate(rows):
        assert row['loop_n_pre'] is not None, i
        assert row['loop_n_pre'] > 0, i
        assert row['loop_period_max_pre_s'] is not None, i
        assert row['loop_n_post'] is not None and row['loop_n_post'] > 0, i
    # The STAGED cycles reach their commit through PHASE_COMMITTING, which is in
    # PRE_DISPATCH_PHASES — so their pre-dispatch count covers the ladder AND
    # the arm point, not just the ladder.
    for row in rows[1:]:
        assert row['loop_n_pre'] >= 4, row['loop_n_pre']


# ── F3: the SESSION no-progress watchdog (2026-08-28) ────────────────────────
#
# WHY IT EXISTS, and why the session ceiling was not enough. The pre-fix
# `_stage_declined` deadlock left the session answering DWELL / ACTION_NONE /
# done=False with both slots empty; `_toss_session_deadline_s` DID eventually
# terminalise it, but that ceiling is a backstop sized for a whole sitting —
# 270.9 s for the 2026-08-28 sitting's goals (num_throws 5, dwell 0.45-0.50 s,
# on_empty_cup RELOAD with max_reloads 3): 5 x (30.0 + 0.50) + 113.4 + 5.0,
# where the 113.4 is `_reload_interlude_budget_s`, the term a "157.5 s" reading
# of this ceiling forgets and every goal of that sitting carried. It names the
# failure ABORTED_TIMEOUT, which reads as "the session ran
# long". For the whole of that window the node's cross-action `_goal_claimed` is
# held, so every Reload / Toss / TossContinuous the operator sends is
# REJECTED_BUSY. F1 removes the one deadlock we found; this catches the class,
# in seconds, by name.

def _wedged_session_class():
    """A session that STOPS ADVANCING — the observable signature of the class
    F3 guards, expressed without depending on the defect that produced it.

    Everything except `step` is the real FSM (the ceiling arithmetic, the
    accounting, `force_terminal`), so what this exercises is the node's loop
    against a session that will never emit another action: both slots empty,
    `next_cycle_at` in the past, nothing pending. That is exactly the state the
    four hung goals of the first pipelined sitting sat in."""
    from jugglebot.toss_session import (SESSION_ACTION_NONE,
                                        SESSION_PHASE_DWELL,
                                        TossSessionDecision)

    class _Wedged(TossSessionSequencer):
        def step(self, now):
            return TossSessionDecision(SESSION_PHASE_DWELL,
                                       SESSION_ACTION_NONE,
                                       self.cycle_index, False, None)

    return _Wedged


def test_a_wedged_session_aborts_STALLED_in_seconds_and_releases_the_claim(
        monkeypatch):
    """THE watchdog, end to end through the real execute callback.

    Three things have to be true together, and only the first is about the
    verdict string:

      * it terminalises ABORTED_STALLED — a name that says the session stopped
        advancing, not that it ran long;
      * it does so at the DERIVED bound (`_SESSION_STALL_S`), which is orders of
        magnitude inside the session ceiling — asserted against BOTH so a future
        edit to either cannot silently make this the ceiling again;
      * and it goes through `_finish_session`, so the execute callback's
        `finally` runs: the pipeline drains, the session arming comes down and
        `_goal_claimed` is released. That last one is the whole point — a wedge
        must cost ONE goal, not every ball-op for the rest of the process."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    monkeypatch.setattr(rcn, 'TossSessionSequencer', _wedged_session_class())
    moved = []
    for name in ('_go_home', '_safe_abort', '_recenter', '_toss_safe_abort',
                 '_toss_stay', '_toss_recenter', '_retract_hand_with_retries',
                 '_prime_hand_with_retries', '_position_platform_for_toss'):
        monkeypatch.setattr(node, name,
                            lambda *a, _n=name, **k: moved.append(_n))
    gh = _ContGoalHandle(num_throws=5, dwell=DWELL, delay=DELAY)
    with node._lock:
        node._goal_claimed = True          # what `_goal_callback` takes at accept
    t0 = clock.t
    result = node._execute_toss_continuous(gh)
    assert result.outcome == 'ABORTED_STALLED'
    assert result.success is False
    assert gh.terminal == 'abort'
    with node._lock:
        assert node._goal_claimed is False, (
            'the wedge must cost ONE goal — a held claim makes every later '
            'ball-op REJECTED_BUSY')
    elapsed = clock.t - t0
    assert elapsed >= rcn._SESSION_STALL_S
    assert elapsed < rcn._SESSION_STALL_S + 10 * rcn._PACE_PERIOD_S, elapsed
    # …and it is a real tightening of the ceiling, not a second copy of it.
    budget_seq = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                               flight_time_s=FLIGHT, throw_delay_s=DELAY,
                               event_vel_mps=1.0)
    ceiling = _toss_session_deadline_s(
        rcn.TossSessionSequencer(num_throws=5, dwell_time_s=DWELL,
                                 throw_delay_s=DELAY, flight_time_s=FLIGHT),
        rcn._toss_deadline_s(budget_seq))
    assert elapsed < 0.25 * ceiling, (elapsed, ceiling)
    # S2 still holds on the way out: the watchdog commands nothing of its own
    # (both slots are empty by its own precondition, so there is nothing to safe).
    assert moved == [], moved


def test_the_watchdog_never_fires_on_a_healthy_chained_session(monkeypatch):
    """The other half, and the one that matters more: a watchdog that fires on a
    good sitting is worse than no watchdog.

    A full three-cycle session runs to COMPLETED and the ABORTED_STALLED
    terminal never appears — over a QUIESCENT WAIT that is itself longer than
    the stall bound, which is the case a naive "no progress for N seconds"
    watchdog would have failed. It passes because the progress clock does not
    run while the session is inside its own scheduled wait (clause 4 of
    ``_toss_session_progressing``: ``now < next_cycle_at``).

    ⚠ **The guard used to be on the DWELL, and that was the wrong quantity**
    (2026-08-28 audit). On the serial ladder the cycle itself consumes
    ``throw_delay_s`` before its release, so the between-cycle wait the progress
    clock would run over is ``dwell − throw_delay``, not ``dwell``. At the
    module fixture's 8.0 s dwell against a 5.0 s delay that wait is 3.0 s, well
    INSIDE the 7.8 s bound — and the no-progress gap this session actually
    reaches, measured, is **0.0000 s**. The test was green for a reason that had
    nothing to do with clause 4. The local dwell below is 14.0 s so the wait is
    a real 9.0 s, past the bound, and removing clause 4 makes this test RED."""
    dwell, delay = 14.0, 5.0
    assert dwell - delay > rcn._SESSION_STALL_S, (
        'the QUIESCENT wait is dwell - throw_delay, not dwell — that is what '
        'the progress clock would run over if clause 4 were removed')
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    _stub_cycles(node, monkeypatch, clock, [
        TossResult(True, 'CAUGHT', 3.0, 0.805),
        TossResult(True, 'CAUGHT', 5.0, 0.812),
        TossResult(True, 'CAUGHT', 4.0, 0.799)])
    gh = _ContGoalHandle(num_throws=3, dwell=dwell, delay=delay)
    result = node._execute_toss_continuous(gh)
    assert result.outcome == 'COMPLETED' and result.success is True
    assert 'STALLED' not in str(result.outcome)


def test_the_watchdog_never_fires_across_a_reload_interlude(monkeypatch):
    """The interlude is a BLOCKING call whose legitimate wall time
    (`_reload_interlude_budget_s`) is an order of magnitude past the stall
    bound: a BB delivery, a 2.0 s recentre, a seat-edge band wait and a whole
    reload sequence ceiling, per attempt, times `max_reloads`.

    It cannot trip the watchdog for two independent reasons and this test would
    catch the loss of either: the interlude branch `continue`s before the check
    is ever reached, AND the loop re-anchors the progress clock on the instant
    the interlude actually returned rather than on the `now` its iteration
    started with. Drop the re-anchor and the FOLLOWING tick reads the whole
    interlude as a stall."""
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    _stub_cycles(node, monkeypatch, clock, [
        TossResult(False, 'REJECTED_NO_BALL', float('nan'), float('nan')),
        TossResult(True, 'CAUGHT', 3.0, 0.805)])

    ran = []

    def fake_interlude(session, *, cancel_now_fn):
        # Far longer than `_SESSION_STALL_S`, which is the point.
        ran.append(clock.t)
        clock.t += 10 * rcn._SESSION_STALL_S
        _stamp(node, clock.t)
        return True, None, 1

    monkeypatch.setattr(node, '_run_reload_interlude', fake_interlude)
    gh = _ContGoalHandle(num_throws=1, dwell=DWELL, delay=DELAY)
    gh.request.on_empty_cup = 'RELOAD'
    gh.request.max_reloads = 2
    result = node._execute_toss_continuous(gh)
    assert ran, 'the interlude never ran — this test would be vacuous'
    assert 'STALLED' not in str(result.outcome), result.outcome
    assert result.outcome == 'COMPLETED', result.outcome
    assert result.reloads_used == 1


def test_the_watchdog_never_fires_on_a_healthy_PIPELINED_session(monkeypatch):
    """**The pipelined twin, and it is the one that covers the shipped machine.**

    The two non-firing tests above drive the SERIAL ladder — `_stub_cycles`
    scripts `_run_toss_cycle`, the blocking wrapper the pipelined loop never
    calls — and they are pinned there by this file's autouse fixture. Under the
    pipelined flag they do not merely lose their subject, they FAIL: the harness
    stubs a method that is no longer on the path. So without this test the F3
    watchdog's non-firing guarantee was pinned against false firing on a machine
    the operator no longer runs.

    The pipelined loop is where a false firing would actually cost something, and
    its progress signature is a different one: the committed and staged slots are
    occupied for almost the whole session (clause 2), and the quiescent gap
    clause 4 covers is a fraction of a serial one because the next cycle's
    preamble already ran inside the previous flight. This runs a full three-cycle
    pipelined session through the real FSMs and the real `_tick_toss_pipeline`,
    and asserts the terminal is COMPLETED with no ABORTED_STALLED anywhere."""
    node, calls, result, gh = _run_pipelined(monkeypatch, num_throws=3)
    assert 'STALLED' not in str(result.outcome), result.outcome
    assert result.outcome == 'COMPLETED', result.outcome
    assert result.catches_confirmed == 3
    # …and it really was the PIPELINED path: cycles 2 and 3 staged, which is the
    # precondition that makes this a twin rather than a second serial test.
    assert [c[0] for c in calls if c[0] == 'dispatch'].count('dispatch') == 3
    assert gh.terminal == 'succeed', gh.terminal
    assert not [f for f in gh.feedbacks if 'STALLED' in str(f[1])], gh.feedbacks


def test_the_progress_predicate_is_a_pure_function_of_the_four_things():
    """F3's POLICY, unit-tested away from the loop — because the failure it
    guards is "the session answers the same thing forever", and a policy only
    reachable through a live loop would be tested the same way that loop was.

    The four ways to be progressing, and the two that keep the bound honest:
    a terminal on this tick, an occupied slot, an emitted action, or being
    inside the session's OWN scheduled wait. Only the last one is subtle, and it
    is what lets a dwell be arbitrarily long without the watchdog knowing
    anything about cadence arithmetic."""
    from jugglebot.toss_session import (SESSION_ACTION_NONE,
                                        SESSION_ACTION_RELOAD,
                                        SESSION_ACTION_START_CYCLE,
                                        SESSION_PHASE_DWELL,
                                        TossSessionDecision)
    progressing = rcn.ReloadCoordinatorNode._toss_session_progressing
    idle = TossSessionDecision(SESSION_PHASE_DWELL, SESSION_ACTION_NONE,
                               1, False, None)

    def _sess(*, cycle=False, committed=False, next_at=0.0):
        return types.SimpleNamespace(cycle_live=cycle, committed_live=committed,
                                     next_cycle_at=next_at)

    # THE STALL: nothing live, nothing emitted, past the scheduled instant.
    assert progressing(100.0, _sess(next_at=50.0), idle, None) is False
    # 1. a cycle terminalised on this tick
    assert progressing(100.0, _sess(next_at=50.0), idle,
                       TossResult(True, 'CAUGHT', 1.0, 0.8)) is True
    # 2. either slot occupied — a flight, a settle, a staged preamble
    assert progressing(100.0, _sess(cycle=True, next_at=50.0), idle,
                       None) is True
    assert progressing(100.0, _sess(committed=True, next_at=50.0), idle,
                       None) is True
    # 3. the session emitted an action
    for action in (SESSION_ACTION_START_CYCLE, SESSION_ACTION_RELOAD):
        busy = TossSessionDecision(SESSION_PHASE_DWELL, action, 1, False, None)
        assert progressing(100.0, _sess(next_at=50.0), busy, None) is True
    # 4. inside the session's OWN scheduled quiescent wait, however long
    assert progressing(100.0, _sess(next_at=100.0 + 86400.0), idle,
                       None) is True
    # …and the boundary is the instant itself, not a window around it.
    assert progressing(100.0, _sess(next_at=100.0), idle, None) is False


def test_the_stall_bound_is_derived_and_can_never_reach_a_legitimate_wait():
    """`_SESSION_STALL_S` is arithmetic over two constants that already exist,
    and the assertion is on the DERIVATION rather than on the number — a literal
    here would go stale the first time either term is re-measured (both have
    been, twice, in the last four days).

    The two orderings that make it safe are asserted too: it must sit ABOVE the
    longest scheduled between-cycle wait a session can hold with both slots
    empty (the MISS cleanup floor) and BELOW the session ceiling it tightens."""
    assert rcn._SESSION_STALL_S == pytest.approx(
        rcn.DEFAULT_SESSION_MISS_CLEANUP_S + rcn._SEQUENCE_CEILING_MARGIN_S)
    assert rcn._SESSION_STALL_S > rcn.DEFAULT_SESSION_MISS_CLEANUP_S
    assert rcn._SESSION_STALL_S < rcn._MAX_SEQUENCE_S, (
        'the session ceiling is never below _MAX_SEQUENCE_S, so staying under '
        'it is what makes this a tightening rather than a second ceiling')
