"""End-to-end cold-start integration: the LOCKED orchestrator state machine driven
against the REAL bridge conduit handlers.

The drift-guard (``test_orchestrator_conduit_contract.py``) proves the (name, type)
interfaces line up; the behaviour suite (``test_teensy_bridge_node_conduit.py``)
proves each handler in isolation. This test proves they COMPOSE: it wires the
orchestrator's mock service/action clients + the set_level_state publisher to the
bridge's actual handler methods (an in-process stand-in for DDS — no executor spin,
no discovery) and drives the orchestrator's 10 Hz tick loop through a full cold
start, asserting it reaches IDLE by driving the conduit, and a levelling cycle
persists through the conduit's set_level_state → STATE_WRITE path.

The bridge's heavy verbs (_run_encoder_search / _run_home / _run_configure /
_run_activate / _run_deactivate / relay_read_tilt) are stubbed to succeed — their
internals are covered by their own suites; here the conduit wrappers + the
orchestrator wiring are the system under test.
"""

from __future__ import annotations

import types

from jugglebot.state_machine import RobotState

from teensy_link import RpcMethod, RpcStatus

import jugglebot.hardware_config as hw

from tests.ros.conftest import (
    MotorStateSingle, RobotState as RobotStateMsg, MockFuture, Trigger,
    ActivateOrDeactivate, GetTiltReadingService, ODriveCommandService,
)

from tests.ros._bridge_harness import _build_paired_node, _teardown


def _cold_start_msg(*, is_homed=False, encoder_search_complete=False):
    """A healthy 7-axis robot_state: all heartbeats + firmware validated."""
    msg = RobotStateMsg()
    for i in range(9):
        ms = MotorStateSingle()
        if i < 7:
            ms.current_state = 1
        msg.motor_states.append(ms)
    msg.all_heartbeats = True
    msg.firmware_validated = True
    msg.is_homed = is_homed
    msg.encoder_search_complete = encoder_search_complete
    msg.error = []
    return msg


def _done_future(result):
    f = MockFuture()
    f.set_result(result)
    return f


def _wire_conduit(orch, bridge):
    """Route the orchestrator's mock clients / action / set_level_state publisher to
    the REAL bridge handlers, returning already-resolved futures — the in-process
    stand-in for DDS delivery."""
    orch._encoder_search_client.call_async = lambda req: _done_future(
        bridge._svc_encoder_search(req, Trigger.Response()))
    orch._activate_client.call_async = lambda req: _done_future(
        bridge._svc_activate_or_deactivate(req, ActivateOrDeactivate.Response()))
    orch._odrive_cmd_client.call_async = lambda req: _done_future(
        bridge._svc_odrive_command(req, ODriveCommandService.Response()))
    orch._bb_calibrate_client.call_async = lambda req: _done_future(
        bridge._svc_bb_calibrate(req, Trigger.Response()))
    orch._tilt_client.call_async = lambda req: _done_future(
        bridge._svc_get_platform_tilt(req, GetTiltReadingService.Response()))

    def _send_goal_async(goal):
        # Two-phase action: goal accepted → result. Drive the real conduit execute.
        result = bridge._home_action_execute(
            types.SimpleNamespace(request=types.SimpleNamespace(),
                                  succeed=lambda: None, abort=lambda: None))
        gh = types.SimpleNamespace(
            accepted=True,
            get_result_async=lambda: _done_future(
                types.SimpleNamespace(result=result)))
        return _done_future(gh)
    orch._home_client.send_goal_async = _send_goal_async
    # set_level_state publisher → bridge subscriber (real STATE_WRITE path).
    orch._level_state_pub.publish = lambda msg: bridge._sub_set_level_state(msg)
    # /link_status: bridge publisher → orchestrator subscriber (the A5 conduit —
    # BOOT's stale-latch pre-flight waits for the bridge's first report, and the
    # guard_latched / wire_armed decodes ride the REAL KeyValues the bridge builds).
    bridge.link_status_pub.publish = lambda msg: orch._on_link_status(msg)


def _run_until(orch, msg, target_state, max_ticks=60):
    """Feed robot_state + link_status + tick until the SM reaches target_state.

    ``orch._bridge_for_link_status`` (set by the tests after _wire_conduit) lets
    the loop drive the bridge's REAL 10 Hz link-status publish each tick, exactly
    as the production timer would.
    """
    bridge = getattr(orch, '_bridge_for_link_status', None)
    for _ in range(max_ticks):
        orch._on_robot_state(msg)
        if bridge is not None:
            bridge._publish_link_status()
        orch._tick()
        if orch.sm.state == target_state:
            return True
    return orch.sm.state == target_state


def test_full_cold_start_boot_home_level_to_idle_via_conduit():
    from jugglebot.orchestrator_node import OrchestratorNode
    orch = OrchestratorNode()
    teensy, client, bridge = _build_paired_node(boot_state_read=False)
    try:
        # Stub heavy verbs (covered elsewhere) so the conduit + wiring are the SUT.
        bridge._run_encoder_search = lambda axes, **kw: (True, "searched")
        bridge._run_home = lambda axes, **kw: (True, "homed")
        bridge._run_configure = lambda axes, **kw: (True, "configured")
        bridge._run_activate = lambda axes, **kw: (True, "activated")
        bridge._run_deactivate = lambda axes, **kw: (True, "deactivated")
        bridge.relay_read_tilt_ex = lambda timeout=None: (True, "OK", (0.02, -0.01), None)
        teensy.on_rpc(int(RpcMethod.STATE_WRITE), lambda rid, a: (int(RpcStatus.OK), b""))
        # Make the levelling settle instant (test-only tweak to the handler instance).
        orch.sm._handlers[RobotState.LEVELLING]._settle_s = 0.0
        _wire_conduit(orch, bridge)
        orch._bridge_for_link_status = bridge

        msg = _cold_start_msg(is_homed=False, encoder_search_complete=False)
        # ARMING_CONTRACT A6: Homing waits until the six Legs have been Idle for
        # 2 s; back-date the run start instead of sleeping.
        orch._on_robot_state(msg)
        orch.ctx.legs_idle_since -= 10.0

        # ── BOOT → HOMING (encoder_search → home → bb_calibrate) → IDLE ──
        assert _run_until(orch, msg, RobotState.IDLE), (
            f"cold start did not reach IDLE (stuck at {orch.sm.state})")

        # ── level command: IDLE → LEVELLING → (activate/tilt/persist/deactivate) → IDLE ──
        orch.ctx.enqueue_command('level')
        assert _run_until(orch, msg, RobotState.LEVELLING, max_ticks=5)
        assert _run_until(orch, msg, RobotState.IDLE), (
            f"levelling did not return to IDLE (stuck at {orch.sm.state})")
        # The levelling result persisted through the conduit (set_level_state →
        # STATE_WRITE); the orchestrator marked levelling_complete on persist.
        assert orch.ctx.levelling_complete is True
        # And the tilt the conduit returned flowed into the accumulated offset.
        assert orch.ctx.pose_offset_rad != [0.0, 0.0]

        # Never faulted anywhere in the automated cold start.
        assert orch.sm.state == RobotState.IDLE
    finally:
        _teardown(teensy, client, bridge)


def test_cold_start_skips_homing_when_already_homed():
    """If robot_state reports is_homed (persisted from a prior session via the Platform-Teensy cold-start
    relay), BOOT skips straight to IDLE — the skip-if-homed path a powered bench sitting
    showed already works, now reachable without a transient FAULT."""
    from jugglebot.orchestrator_node import OrchestratorNode
    orch = OrchestratorNode()
    teensy, client, bridge = _build_paired_node(boot_state_read=False)
    try:
        _wire_conduit(orch, bridge)
        orch._bridge_for_link_status = bridge
        msg = _cold_start_msg(is_homed=True, encoder_search_complete=True)
        assert _run_until(orch, msg, RobotState.IDLE, max_ticks=10)
        assert orch.sm.state == RobotState.IDLE
    finally:
        _teardown(teensy, client, bridge)


# ── BUG-130 / C-LEVEL-1.A: the operator's ACTIVATE lands on the Levelled Active pose ──
#
# Four nodes in-process: the orchestrator, the REAL trajectory_node handlers
# (/gravity_offset ingest, trajectory/status, trajectory/levelled_activate_targets),
# and the bridge conduit, whose _run_activate is captured (its ACTIVATE_TO path is
# covered by tests/ros/test_teensy_bridge_node_activate.py).

def _wire_trajectory(orch, traj):
    from tests.ros.conftest import LevelledActivateTargets
    orch._gravity_offset_pub.publish = lambda msg: traj._on_gravity_offset(msg)
    traj.status_pub.publish = lambda msg: orch._on_trajectory_status(msg)
    orch._levelled_targets_client.call_async = lambda req: _done_future(
        traj._svc_levelled_activate_targets(req, LevelledActivateTargets.Response()))


def _run_until_with_traj(orch, traj, msg, target_state, max_ticks=60):
    bridge = getattr(orch, '_bridge_for_link_status', None)
    for _ in range(max_ticks):
        orch._on_robot_state(msg)
        if bridge is not None:
            bridge._publish_link_status()
        traj._publish_status()
        orch._tick()
        if orch.sm.state == target_state:
            return True
    return orch.sm.state == target_state


def _four_nodes(activations):
    from jugglebot.orchestrator_node import OrchestratorNode
    from jugglebot.trajectory_node import TrajectoryNode
    orch = OrchestratorNode()
    traj = TrajectoryNode(start_emitter=False)
    teensy, client, bridge = _build_paired_node(boot_state_read=False)
    bridge._run_encoder_search = lambda axes, **kw: (True, "searched")
    bridge._run_home = lambda axes, **kw: (True, "homed")
    bridge._run_configure = lambda axes, **kw: (True, "configured")
    bridge._run_activate = lambda axes, **kw: (
        activations.append(kw.get('leg_targets_rev')), (True, "activated"))[1]
    bridge._run_deactivate = lambda axes, **kw: (True, "deactivated")
    # A small level reading (rad), well inside the ACTIVATE_TO band.
    bridge.relay_read_tilt_ex = lambda timeout=None: (True, "OK", (0.004, -0.006), None)
    teensy.on_rpc(int(RpcMethod.STATE_WRITE), lambda rid, a: (int(RpcStatus.OK), b""))
    orch.sm._handlers[RobotState.LEVELLING]._settle_s = 0.0
    _wire_conduit(orch, bridge)
    _wire_trajectory(orch, traj)
    orch._bridge_for_link_status = bridge
    return orch, traj, teensy, client, bridge


def _expected_levelled(traj):
    import numpy as np
    return [float(v) for v in traj._pose_to_motor_rev(traj._corrected_neutral_pose())]


def test_boot_home_level_activate_lands_on_the_levelled_active_pose():
    """The operator-visible sequence Boot > Home > Level > Activate, end to end:
    activate before level is refused in IDLE (nothing sent); level's own rise is
    the CONSTANT pose; the operator's activate carries the levelled targets."""
    import numpy as np
    activations = []
    orch, traj, teensy, client, bridge = _four_nodes(activations)
    try:
        msg = _cold_start_msg(is_homed=False, encoder_search_complete=False)
        # ARMING_CONTRACT A6 holds BOOT until the six Legs have been Idle for
        # 2 s (composes with C-LEVEL-1.A below: fw-integrate 2026-11-25).
        _run_until_with_traj(orch, traj, msg, None, max_ticks=3)
        assert orch.sm.state == RobotState.BOOT
        orch.ctx.legs_idle_since -= 10.0      # back-date the run, no sleep
        assert _run_until_with_traj(orch, traj, msg, RobotState.IDLE)

        # Activate before level: refused in IDLE, nothing reaches the bridge.
        orch.ctx.enqueue_command('activate')
        for _ in range(5):
            _run_until_with_traj(orch, traj, msg, RobotState.ACTIVE, max_ticks=1)
        assert orch.sm.state == RobotState.IDLE
        assert activations == []

        orch.ctx.enqueue_command('level')
        assert _run_until_with_traj(orch, traj, msg, RobotState.LEVELLING, max_ticks=5)
        assert _run_until_with_traj(orch, traj, msg, RobotState.IDLE)
        assert activations == [None]          # level_activate: the constant pose
        assert traj._gravity_correction_loaded is True

        orch.ctx.enqueue_command('activate')
        assert _run_until_with_traj(orch, traj, msg, RobotState.ACTIVE, max_ticks=5)
        _run_until_with_traj(orch, traj, msg, None, max_ticks=3)   # dispatch the rise
        assert len(activations) == 2 and activations[1] is not None
        want = _expected_levelled(traj)
        assert np.allclose(activations[1], want, atol=1e-12)
        # ...and it is not the constant pose (the level moved it).
        assert not np.allclose(activations[1], hw.JB_OP_ACTIVATE_POSITION_REVS, atol=1e-3)
    finally:
        _teardown(teensy, client, bridge)


def test_relaunch_restored_by_the_auto_push_activates_levelled_without_level():
    """A relaunch: the Platform Teensy still holds levelling_complete + the offset,
    the orchestrator's persisted auto-push on the first IDLE restores the
    correction in a fresh trajectory_node, and activate lands levelled with no
    new `level`."""
    import numpy as np
    activations = []
    orch, traj, teensy, client, bridge = _four_nodes(activations)
    try:
        msg = _cold_start_msg(is_homed=True, encoder_search_complete=True)
        msg.levelling_complete = True
        msg.pose_offset_rad = [0.004, -0.006]
        assert traj._gravity_correction_loaded is False
        assert _run_until_with_traj(orch, traj, msg, RobotState.IDLE, max_ticks=10)
        _run_until_with_traj(orch, traj, msg, None, max_ticks=2)
        assert traj._gravity_correction_loaded is True      # restored, not levelled
        orch.ctx.enqueue_command('activate')
        assert _run_until_with_traj(orch, traj, msg, RobotState.ACTIVE, max_ticks=5)
        _run_until_with_traj(orch, traj, msg, None, max_ticks=3)
        assert activations and activations[-1] is not None
        assert np.allclose(activations[-1], _expected_levelled(traj), atol=1e-12)
    finally:
        _teardown(teensy, client, bridge)
