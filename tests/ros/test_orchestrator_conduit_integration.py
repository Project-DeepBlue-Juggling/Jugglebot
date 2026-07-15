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

from controller.teensy_link import RpcMethod, RpcStatus

from tests.ros.conftest import (
    MotorStateSingle, RobotState as RobotStateMsg, MockFuture, Trigger,
    ActivateOrDeactivate, GetTiltReadingService, ODriveCommandService,
)

from tests.ros.test_teensy_bridge_node_read import _build_paired_node


def _teardown(teensy, client, node):
    node.on_shutdown()
    client.stop()
    teensy.stop()


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
        bridge.relay_read_tilt = lambda timeout=None: (True, "OK", (0.02, -0.01))
        teensy.on_rpc(int(RpcMethod.STATE_WRITE), lambda rid, a: (int(RpcStatus.OK), b""))
        # Make the levelling settle instant (test-only tweak to the handler instance).
        orch.sm._handlers[RobotState.LEVELLING]._settle_s = 0.0
        _wire_conduit(orch, bridge)
        orch._bridge_for_link_status = bridge

        msg = _cold_start_msg(is_homed=False, encoder_search_complete=False)

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
