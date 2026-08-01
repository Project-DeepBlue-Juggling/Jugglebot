"""Read-side tests for teensy_bridge_node (the UDP→ROS read mirror).

Exercises the UDP→ROS mirror end-to-end on loopback: a ``FakeTeensy`` (reused
from ``tests/teensy_link/conftest.py``) injects T→J frames, the real
``TeensyLinkClient`` RX thread decodes + dispatches them to the node's
callbacks, and we assert the node publishes the right (production-named) messages.

ROS 2 is mocked by ``tests/ros/conftest.py`` (so ``create_publisher`` returns a
recording ``MockPublisher`` and timers don't auto-fire — we call the publish
methods directly). ``diagnostic_msgs`` is the real package (installed on the
Jetson; used by the production ``motion_bridge_node`` too).

The single most important assertion here is ``test_heartbeat_mpc_active_zero``:
the bridge MUST send ``mpc_active=0`` on every startup path.
"""

from __future__ import annotations

import time

import pytest

from teensy_link import protocol as p
from teensy_link import (
    MsgType, LinkState, BusHealth, FaultState,
    HeartbeatJ2T, HeartbeatT2J, Telemetry, Diagnostic, Profile,
)

# Shared loopback harness (node construction, polling, teardown, builders).
from tests.ros._bridge_harness import (
    _build_paired_node, _messages, _teardown, _wait_until,
)


@pytest.fixture
def bridge():
    """Yield (teensy, node) wired on loopback; tear everything down on exit."""
    teensy, client, node = _build_paired_node()
    yield teensy, node
    _teardown(teensy, client, node)   # node → client → FakeTeensy, one order


def _telem(pos=None, vel=None):
    if pos is None:
        pos = tuple(float(i) for i in range(p.NUM_AXES))
    if vel is None:
        vel = tuple(0.1 * i for i in range(p.NUM_AXES))
    return Telemetry(t_teensy_us=123, pos_rev=tuple(pos), vel_rps=tuple(vel))


# ── Safety-critical: mpc_active=0 ──────────────────────────────

def test_heartbeat_mpc_active_zero(bridge):
    """The bridge MUST emit J→T heartbeats with mpc_active (flags bit0) = 0."""
    teensy, node = bridge
    got = teensy.wait_for(int(MsgType.HEARTBEAT_J2T), count=1, timeout=2.0)
    assert got, "no J→T heartbeat received"
    hb = HeartbeatJ2T.unpack(got[0].payload)
    assert hb.flags == 0, f"mpc_active flag set on startup! flags={hb.flags:#x}"
    # And the node's own mpc_active state is False.
    assert node._mpc_active is False


def test_enable_setpoint_output_defaults_false(bridge):
    """The setpoint-output parameter defaults to false (no setpoint stream)."""
    _, node = bridge
    assert bool(node.get_parameter('enable_setpoint_output').value) is False


# ── Topic namespace discipline ─────────────────────────────────

# The leg/hand cutover: the side-by-side ``/teensy/*`` namespace
# (the interim dual-publisher handoff) is fully retired. ``can_node`` is out of the production launch,
# so the dual-publisher risk it prevented is moot and the bridge owns the
# PRODUCTION topic/service names directly — which is what reconnects the GUI /
# orchestrator / consumers (they subscribe to the bare names). BB + cone were
# promoted in the Ball Butler cutover; the legs/hand follow here.


def test_no_publisher_under_teensy_namespace(bridge):
    """The rename is complete: NO publisher remains under the ``/teensy/*``
    namespace, and the leg/hand production names the GUI subscribes to are
    present."""
    _, node = bridge
    topics = list(node._publishers.keys())
    assert topics, "node created no publishers"
    for t in topics:
        assert not t.startswith('/teensy/') and not t.startswith('teensy/'), \
            f"leftover /teensy/* topic (rename incomplete): {t}"
    # The leg/hand production names must be published (GUI subscribes to these).
    for expected in ('robot_state', 'hand_telemetry', 'link_status', 'profile'):
        assert expected in topics, f"missing production topic: {expected}"


def test_leg_hand_services_use_production_names(bridge):
    """The leg/hand RPC services were promoted off /teensy/* to
    the production names the orchestrator's service clients depend on
    (encoder_search, odrive_command, set_motor_vel_curr_limits) plus the
    bridge-new ops (clear_errors, reboot_odrives, home). A typo here would
    silently disconnect the orchestrator, so pin the exact names."""
    _, node = bridge
    svcs = set(node._services.keys())
    subs = set(node._subscriptions.keys())
    expected_svcs = {'clear_errors', 'reboot_odrives', 'encoder_search',
                     'home', 'odrive_command'}
    assert expected_svcs <= svcs, f"missing services: {expected_svcs - svcs}"
    assert 'set_motor_vel_curr_limits' in subs
    for name in svcs | subs:
        assert not name.startswith('/teensy/') and not name.startswith('teensy/'), \
            f"leftover /teensy/* service/subscription: {name}"


# ── robot_state ────────────────────────────────────────────────

def test_robot_state_suppressed_before_telemetry(bridge):
    """No robot_state is published until the first Telemetry frame arrives."""
    _, node = bridge
    node._publish_robot_state()
    assert node.robot_state_pub.published == []


def test_robot_state_from_telemetry_and_diagnostic(bridge):
    teensy, node = bridge
    teensy.send_telemetry(pos_rev=_telem().pos_rev, vel_rps=_telem().vel_rps)
    # Diagnostic for leg axis 2: CLOSED_LOOP, an active error, temps.
    diag = Diagnostic(axis_id=2, axis_state=8, active_errors=0x40,
                      disarm_reason=0, iq_measured=1.5,
                      temp_fet=30.0, temp_motor=25.0, bus_voltage=48.0,
                      bus_current=2.5)
    teensy.send_to_jetson(int(MsgType.DIAGNOSTIC), diag.pack())

    assert _wait_until(lambda: node._latest_telemetry is not None
                       and 2 in node._latest_diag)
    node._publish_robot_state()
    assert len(node.robot_state_pub.published) == 1
    msg = node.robot_state_pub.published[0]
    assert len(msg.motor_states) == p.NUM_AXES
    # pos/vel mirror the telemetry, sign preserved (Jugglebot convention).
    assert msg.motor_states[2].pos_estimate == pytest.approx(2.0)
    assert msg.motor_states[2].vel_estimate == pytest.approx(0.2)
    # Diagnostic-sourced fields.
    assert msg.motor_states[2].current_state == 8
    assert msg.motor_states[2].active_errors == 0x40
    assert msg.motor_states[2].iq_measured == pytest.approx(1.5)
    assert msg.motor_states[2].bus_voltage == pytest.approx(48.0)
    assert msg.motor_states[2].bus_current == pytest.approx(2.5)


def test_has_fatal_can_error_from_bus_off(bridge):
    teensy, node = bridge
    teensy.send_telemetry()
    # bus1_health = Jugglebot core bus (CAN3: legs + hand) after the three-bus
    # remap (ADR-0013 / firmware HANDOFF D4); its BUS_OFF is the fatal condition.
    # (Pre-remap this test set bus2_health, which is now Ball Butler, not legs.)
    hb = HeartbeatT2J(t_teensy_us=1, link_state=int(LinkState.UP),
                      bus1_health=int(BusHealth.BUS_OFF),
                      bus2_health=int(BusHealth.OK),
                      fault_state=int(FaultState.NONE), flags=0, uptime_ms=10)
    teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), hb.pack())
    assert _wait_until(lambda: node._latest_heartbeat is not None
                       and node._latest_telemetry is not None)
    node._publish_robot_state()
    msg = node.robot_state_pub.published[-1]
    assert msg.has_fatal_can_error is True
    assert "Fatal CAN bus issue." in msg.error


def test_bb_bus_off_alone_is_not_leg_fatal(bridge):
    # Contract (review #3): a BUS_OFF on bus2_health = Ball Butler (CAN1), with the
    # Jugglebot core bus (bus1_health) healthy and no CAN_BUS_DOWN fault, must NOT
    # raise has_fatal_can_error -- BB faults are isolated from the legs (ADR-0013).
    teensy, node = bridge
    teensy.send_telemetry()
    hb = HeartbeatT2J(t_teensy_us=1, link_state=int(LinkState.UP),
                      bus1_health=int(BusHealth.OK),
                      bus2_health=int(BusHealth.BUS_OFF),
                      fault_state=int(FaultState.NONE), flags=0, uptime_ms=10)
    teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), hb.pack())
    assert _wait_until(lambda: node._latest_heartbeat is not None
                       and node._latest_telemetry is not None)
    node._publish_robot_state()
    assert node.robot_state_pub.published[-1].has_fatal_can_error is False


def test_has_fatal_can_error_from_fault_state(bridge):
    teensy, node = bridge
    teensy.send_telemetry()
    hb = HeartbeatT2J(t_teensy_us=1, link_state=int(LinkState.UP),
                      bus1_health=int(BusHealth.OK), bus2_health=int(BusHealth.OK),
                      fault_state=int(FaultState.CAN_BUS_DOWN), flags=0, uptime_ms=10)
    teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), hb.pack())
    assert _wait_until(lambda: node._latest_heartbeat is not None
                       and node._latest_telemetry is not None)
    node._publish_robot_state()
    assert node.robot_state_pub.published[-1].has_fatal_can_error is True


def test_has_fatal_can_error_from_link_loss(bridge):
    """A dead Jetson↔Teensy UDP link raises has_fatal_can_error with a
    DISTINCT operator-facing string, even when the Teensy's own bus/fault flags
    look healthy. The orchestrator's ONLY health input is robot_state; without
    this OR-term a frozen link leaves it consuming stale-but-fresh-looking motor
    states forever (can_node coupled 2 s silence → fatal_can, dropped in the port)."""
    teensy, node = bridge
    teensy.send_telemetry()
    hb = HeartbeatT2J(t_teensy_us=1, link_state=int(LinkState.UP),
                      bus1_health=int(BusHealth.OK), bus2_health=int(BusHealth.OK),
                      fault_state=int(FaultState.NONE), flags=0, uptime_ms=10)
    teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), hb.pack())
    assert _wait_until(lambda: node._latest_heartbeat is not None
                       and node._latest_telemetry is not None)
    # Force the link-loss latch as the 1 Hz watchdog would on heartbeat silence.
    node._link_latch.link_lost = True
    node._publish_robot_state()
    msg = node.robot_state_pub.published[-1]
    assert msg.has_fatal_can_error is True
    # The link-loss cause gets its OWN string, distinct from a CAN3 bus fault.
    assert "Teensy link lost (UDP) — telemetry is stale." in msg.error
    assert "Fatal CAN bus issue." not in msg.error


def test_fatal_odrive_not_masked_by_concurrent_fault(bridge):
    """A higher-priority single-valued Teensy fault (CAN_BUS_DOWN) must NOT mask a
    concurrent per-leg ODrive fault — the bridge ORs the raw per-leg signals in."""
    teensy, node = bridge
    teensy.send_telemetry()
    hb = HeartbeatT2J(t_teensy_us=1, link_state=int(LinkState.UP),
                      bus1_health=int(BusHealth.OK), bus2_health=int(BusHealth.OK),
                      fault_state=int(FaultState.CAN_BUS_DOWN), flags=0, uptime_ms=1)
    teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), hb.pack())
    diag = Diagnostic(axis_id=1, axis_state=8, active_errors=0x40, disarm_reason=0)
    teensy.send_to_jetson(int(MsgType.DIAGNOSTIC), diag.pack())
    assert _wait_until(lambda: node._latest_heartbeat is not None
                       and 1 in node._latest_diag
                       and node._latest_telemetry is not None)
    node._publish_robot_state()
    msg = node.robot_state_pub.published[-1]
    assert msg.has_fatal_odrive_error is True   # not masked by CAN_BUS_DOWN
    assert msg.has_fatal_can_error is True


def test_fatal_odrive_from_disarm_while_closed_loop(bridge):
    teensy, node = bridge
    teensy.send_telemetry()
    diag = Diagnostic(axis_id=2, axis_state=8, active_errors=0,
                      disarm_reason=0x40)  # disarm while CLOSED_LOOP → fatal
    teensy.send_to_jetson(int(MsgType.DIAGNOSTIC), diag.pack())
    assert _wait_until(lambda: 2 in node._latest_diag
                       and node._latest_telemetry is not None)
    node._publish_robot_state()
    assert node.robot_state_pub.published[-1].has_fatal_odrive_error is True


def test_no_fatal_odrive_when_legs_clean(bridge):
    teensy, node = bridge
    teensy.send_telemetry()
    diag = Diagnostic(axis_id=0, axis_state=8, active_errors=0, disarm_reason=0)
    teensy.send_to_jetson(int(MsgType.DIAGNOSTIC), diag.pack())
    assert _wait_until(lambda: 0 in node._latest_diag
                       and node._latest_telemetry is not None)
    node._publish_robot_state()
    assert node.robot_state_pub.published[-1].has_fatal_odrive_error is False


def test_cross_axis_disarm_while_other_leg_closed_loop_is_fatal(bridge):
    """ANY leg disarmed while ANY (other) leg still holds CLOSED_LOOP is a
    fatal ODrive condition (fault_logic.py `any_disarmed and any_cl` parity). The
    prior per-leg conjunction (the SAME leg disarmed AND CLOSED_LOOP) essentially
    never fired — a disarmed ODrive leaves CLOSED_LOOP almost immediately — so the
    common real event (one leg drops torque while the others keep holding) slipped
    through exactly the OR-term meant to un-mask it."""
    teensy, node = bridge
    teensy.send_telemetry()
    # Leg 2: disarmed but already OUT of CLOSED_LOOP (IDLE=1) — the old per-leg AND
    # would see disarm-but-not-CL here and miss it.
    d2 = Diagnostic(axis_id=2, axis_state=1, active_errors=0, disarm_reason=0x40)
    # Leg 3: clean, still holding CLOSED_LOOP (8).
    d3 = Diagnostic(axis_id=3, axis_state=8, active_errors=0, disarm_reason=0)
    teensy.send_to_jetson(int(MsgType.DIAGNOSTIC), d2.pack())
    teensy.send_to_jetson(int(MsgType.DIAGNOSTIC), d3.pack())
    assert _wait_until(lambda: 2 in node._latest_diag and 3 in node._latest_diag
                       and node._latest_telemetry is not None)
    node._publish_robot_state()
    assert node.robot_state_pub.published[-1].has_fatal_odrive_error is True


def test_disarm_with_no_leg_closed_loop_is_not_fatal(bridge):
    """Contrast: a leg disarmed with NO leg in CLOSED_LOOP (a clean stow /
    powered-down state) is NOT fatal — the `any_cl` term keeps a deliberate
    deactivate from reading as a fault."""
    teensy, node = bridge
    teensy.send_telemetry()
    d2 = Diagnostic(axis_id=2, axis_state=1, active_errors=0, disarm_reason=0x40)
    d3 = Diagnostic(axis_id=3, axis_state=1, active_errors=0, disarm_reason=0)
    teensy.send_to_jetson(int(MsgType.DIAGNOSTIC), d2.pack())
    teensy.send_to_jetson(int(MsgType.DIAGNOSTIC), d3.pack())
    assert _wait_until(lambda: 2 in node._latest_diag and 3 in node._latest_diag
                       and node._latest_telemetry is not None)
    node._publish_robot_state()
    assert node.robot_state_pub.published[-1].has_fatal_odrive_error is False


def test_undervoltage_flag_from_active_error_bitwise(bridge):
    """UV is asserted from a BITWISE test on active_errors (mirrors can_node:436),
    and is robust to a UV bit combined with other error bits."""
    teensy, node = bridge
    teensy.send_telemetry()
    diag = Diagnostic(axis_id=0, axis_state=1,
                      active_errors=512 | 0x40,  # UV bit + another bit
                      disarm_reason=0)
    teensy.send_to_jetson(int(MsgType.DIAGNOSTIC), diag.pack())
    assert _wait_until(lambda: 0 in node._latest_diag
                       and node._latest_telemetry is not None)
    node._publish_robot_state()
    msg = node.robot_state_pub.published[-1]
    assert msg.has_undervoltage is True
    assert "Undervoltage detected. Was the E-stop hit?" in msg.error


def test_undervoltage_not_asserted_from_disarm_only(bridge):
    """disarm_reason==UV alone must NOT assert has_undervoltage — can_node sets
    the flag only from active_errors (disarm==UV is its clear predicate). This
    keeps robot_state faithful to can_node's semantics for the recovered state."""
    teensy, node = bridge
    teensy.send_telemetry()
    diag = Diagnostic(axis_id=0, axis_state=1, active_errors=0, disarm_reason=512)
    teensy.send_to_jetson(int(MsgType.DIAGNOSTIC), diag.pack())
    assert _wait_until(lambda: 0 in node._latest_diag
                       and node._latest_telemetry is not None)
    node._publish_robot_state()
    assert node.robot_state_pub.published[-1].has_undervoltage is False


def test_firmware_validated_conservative_false(bridge):
    """firmware_validated defaults to the CONSERVATIVE False until the
    GET_AXIS_VERSIONS handshake validates the ODrive versions (no version pull has
    happened here). It is no longer a hardcoded handoff gap — see the full
    handshake (match / mismatch / cannot-validate) in
    tests/ros/test_teensy_bridge_node_version.py."""
    teensy, node = bridge
    teensy.send_telemetry()
    assert _wait_until(lambda: node._latest_telemetry is not None)
    node._publish_robot_state()
    assert node.robot_state_pub.published[-1].firmware_validated is False


# ── hand_telemetry ─────────────────────────────────────────────

def test_hand_telemetry_axis6(bridge):
    teensy, node = bridge
    pos = [0.0] * p.NUM_AXES
    vel = [0.0] * p.NUM_AXES
    pos[p.NUM_LEGS] = 3.14   # hand = axis 6
    vel[p.NUM_LEGS] = -0.5
    teensy.send_telemetry(pos_rev=pos, vel_rps=vel)
    diag = Diagnostic(axis_id=p.NUM_LEGS, axis_state=8, iq_measured=2.2)
    teensy.send_to_jetson(int(MsgType.DIAGNOSTIC), diag.pack())
    assert _wait_until(lambda: node._latest_telemetry is not None
                       and p.NUM_LEGS in node._latest_diag)
    node._publish_hand_telemetry()
    assert len(node.hand_telemetry_pub.published) == 1
    msg = node.hand_telemetry_pub.published[0]
    assert msg.pos_meas == pytest.approx(3.14)
    assert msg.vel_meas == pytest.approx(-0.5)
    assert msg.iq_meas == pytest.approx(2.2)
    # No hand command path here yet → command fields are zero.
    assert msg.pos_cmd == 0.0 and msg.vel_ff_cmd == 0.0 and msg.tor_ff_cmd == 0.0


# ── link_status ────────────────────────────────────────────────

def test_link_status_reflects_heartbeat_and_mpc_active(bridge):
    teensy, node = bridge
    hb = HeartbeatT2J(t_teensy_us=1, link_state=int(LinkState.UP),
                      bus1_health=int(BusHealth.OK), bus2_health=int(BusHealth.OK),
                      fault_state=int(FaultState.NONE),
                      flags=0x1, uptime_ms=12345)  # bit0 = time_synced
    teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), hb.pack())
    assert _wait_until(lambda: node._latest_heartbeat is not None)
    node._publish_link_status()
    assert len(node.link_status_pub.published) == 1
    msg = node.link_status_pub.published[0]
    kv = {v.key: v.value for v in msg.values}
    assert kv['mpc_active'] == '0'          # surfaced for operator confirmation
    assert kv['teensy_link'] == 'UP'
    assert kv['time_synced'] == '1'
    assert kv['uptime_ms'] == '12345'


def test_link_status_surfaces_guard_deviation_diagnostics(bridge):
    """The 2026-07-10 leg guard-deviation diagnostics (per-leg live deviation,
    lead-clamp bitmask, and the MAX_DEVIATION latch-event snapshot) surface as
    /link_status KeyValues so a future stutter/latch rosbag is self-diagnosing."""
    teensy, node = bridge
    # torque_clamp_mask rides HeartbeatT2J.flags bits 8-13 (the 2026-07-14
    # firmware ingest clamp — no new payload field). legs 1 and 3 clamped.
    tq_flags = 0b001010 << p.HEARTBEAT_TORQUE_CLAMP_SHIFT
    hb = HeartbeatT2J(t_teensy_us=1, link_state=int(LinkState.UP),
                      bus1_health=int(BusHealth.OK), bus2_health=int(BusHealth.OK),
                      fault_state=int(FaultState.MAX_DEVIATION),
                      flags=tq_flags, uptime_ms=1,
                      live_deviation=(0.12, -0.03, 0.55, 0.0, 0.0, 0.0),
                      lead_clamp_mask=0b000101,   # legs 0 and 2 clamped
                      max_dev_leg=2, max_dev_value=0.55,
                      max_dev_u0=2.80, max_dev_enc=2.25)
    teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), hb.pack())
    assert _wait_until(lambda: node._latest_heartbeat is not None)
    node._publish_link_status()
    kv = {v.key: v.value for v in node.link_status_pub.published[-1].values}
    assert kv['lead_clamp_mask'] == '5'
    assert kv['torque_clamp_mask'] == '10'   # 0b001010 — legs 1 and 3
    assert kv['live_deviation'].startswith('0.1200,-0.0300,0.5500')
    assert kv['max_dev_leg'] == '2'
    assert kv['max_dev_value'] == '0.5500'
    assert kv['max_dev_u0'] == '2.8000'
    assert kv['max_dev_enc'] == '2.2500'
    # An ACTIVE MAX_DEVIATION latch → guard_fault_leg mirrors the frozen leg.
    assert kv['guard_fault_leg'] == '2'


def test_link_status_max_dev_leg_none_when_unset(bridge):
    """max_dev_leg == 0xFF (no MAX_DEVIATION latch since boot) renders as 'none'."""
    teensy, node = bridge
    hb = HeartbeatT2J(t_teensy_us=1, link_state=int(LinkState.UP),
                      bus1_health=int(BusHealth.OK), bus2_health=int(BusHealth.OK),
                      fault_state=int(FaultState.NONE), flags=0, uptime_ms=1,
                      max_dev_leg=0xFF)
    teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), hb.pack())
    assert _wait_until(lambda: node._latest_heartbeat is not None)
    node._publish_link_status()
    kv = {v.key: v.value for v in node.link_status_pub.published[-1].values}
    assert kv['max_dev_leg'] == 'none'


def test_link_status_error_on_fault(bridge):
    teensy, node = bridge
    hb = HeartbeatT2J(t_teensy_us=1, link_state=int(LinkState.UP),
                      bus1_health=int(BusHealth.OK), bus2_health=int(BusHealth.OK),
                      fault_state=int(FaultState.MOTOR_OVERSPEED),
                      flags=0, uptime_ms=1)
    teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), hb.pack())
    assert _wait_until(lambda: node._latest_heartbeat is not None)
    node._publish_link_status()
    from diagnostic_msgs.msg import DiagnosticStatus
    assert node.link_status_pub.published[-1].level == DiagnosticStatus.ERROR


# ── guard-latch LOUD logging (edge + persistent reminder + clear) ──


def _messages(mock_method):
    """All positional first-arg strings across a MagicMock log method's calls."""
    return [c.args[0] for c in mock_method.call_args_list if c.args]


def test_guard_fault_edge_names_reason_and_leg(bridge):
    """The fault EDGE logs an unthrottled ERROR naming the guard REASON and,
    when identifiable from the per-axis diagnostics, the offending LEG."""
    from unittest.mock import MagicMock
    teensy, node = bridge
    # Leg 3 carries an active ODrive error → the leg hint should name it.
    diag = Diagnostic(axis_id=3, active_errors=1)
    teensy.send_to_jetson(int(MsgType.DIAGNOSTIC), diag.pack())
    assert _wait_until(lambda: 3 in node._latest_diag)

    node._logger = MagicMock()
    hb = HeartbeatT2J(t_teensy_us=1, link_state=int(LinkState.UP),
                      bus1_health=int(BusHealth.OK), bus2_health=int(BusHealth.OK),
                      fault_state=int(FaultState.ODRIVE_FATAL),
                      flags=0, uptime_ms=1)
    teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), hb.pack())
    assert _wait_until(lambda: node._latest_heartbeat is not None
                       and int(node._latest_heartbeat.fault_state)
                       == int(FaultState.ODRIVE_FATAL))
    node._publish_link_status()

    edge = [m for m in _messages(node._logger.error) if 'FAULT LATCHED' in m]
    assert len(edge) == 1, f"expected one edge ERROR, got {_messages(node._logger.error)}"
    assert 'ODRIVE_FATAL' in edge[0]
    assert 'leg 3' in edge[0]


def test_guard_fault_leg_hint_absent_for_bus_fault(bridge):
    """A bus-level fault with no leg-specific diagnostic names no leg (honest)."""
    from unittest.mock import MagicMock
    teensy, node = bridge
    node._logger = MagicMock()
    hb = HeartbeatT2J(t_teensy_us=1, link_state=int(LinkState.UP),
                      bus1_health=int(BusHealth.OK), bus2_health=int(BusHealth.OK),
                      fault_state=int(FaultState.CAN_BUS_DOWN),
                      flags=0, uptime_ms=1)
    teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), hb.pack())
    assert _wait_until(lambda: node._latest_heartbeat is not None)
    node._publish_link_status()
    edge = [m for m in _messages(node._logger.error) if 'FAULT LATCHED' in m]
    assert len(edge) == 1
    assert 'leg' not in edge[0].split('—')[0]  # no leg hint in the reason clause


def test_guard_fault_repeats_every_5s_rate_exact(bridge):
    """While latched, a persistent 'TEENSY GUARD LATCHED' ERROR repeats every
    _GUARD_LATCH_REPEAT_S — rate-exact off the last-log timestamp, not throttle
    magic. Between repeats the reminder is silent."""
    from unittest.mock import MagicMock
    from jugglebot.teensy_bridge_node import _GUARD_LATCH_REPEAT_S
    teensy, node = bridge
    # max_dev_leg=1 (no ODrive diag sent → empty _latest_diag) so the reminder's
    # leg hint comes from the frozen MAX_DEVIATION snapshot fallback.
    hb = HeartbeatT2J(t_teensy_us=1, link_state=int(LinkState.UP),
                      bus1_health=int(BusHealth.OK), bus2_health=int(BusHealth.OK),
                      fault_state=int(FaultState.MAX_DEVIATION),
                      flags=0, uptime_ms=1,
                      max_dev_leg=1, max_dev_value=-0.552)
    teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), hb.pack())
    assert _wait_until(lambda: node._latest_heartbeat is not None)

    node._logger = MagicMock()
    node._publish_link_status()            # edge fires + anchors the reminder
    assert any('FAULT LATCHED' in m for m in _messages(node._logger.error))

    # A second publish within the interval must NOT emit a reminder.
    node._logger = MagicMock()
    node._publish_link_status()
    assert not any('TEENSY GUARD LATCHED' in m for m in _messages(node._logger.error))

    # Backdate the anchor past the interval → the reminder must fire once.
    node._logger = MagicMock()
    node._last_guard_latch_log_t = time.monotonic() - (_GUARD_LATCH_REPEAT_S + 1.0)
    node._publish_link_status()
    reminders = [m for m in _messages(node._logger.error)
                 if 'TEENSY GUARD LATCHED' in m]
    assert len(reminders) == 1
    assert 'MAX_DEVIATION' in reminders[0]
    assert 'CLEAR_ERRORS required' in reminders[0]
    # The persistent reminder now carries the culprit-leg hint too (it had NONE
    # before, even for ODRIVE_FATAL): reuse of _guard_fault_leg_hint() names the
    # leg from the frozen snapshot when the ODrive-diag cache is empty.
    assert 'leg 1' in reminders[0]


def test_max_deviation_latch_attributed_from_frozen_snapshot(bridge):
    """The EXACT 2026-07-16 S4 shape: a MAX_DEVIATION guard latch with ZERO
    ODrive errors, so _latest_diag is empty and the primary per-axis diag scan
    finds no culprit — the old _guard_fault_leg_hint() therefore printed nothing.

    The edge log must now (a) name the offending leg + trip deviation from the
    firmware's frozen max_dev_leg/max_dev_value snapshot, and (b) append the raw
    live-deviation vector for multi-leg context; /link_status must carry
    guard_fault_leg as a direct, already-gated field. Previously uncovered."""
    from unittest.mock import MagicMock
    teensy, node = bridge
    node._logger = MagicMock()
    # NO Diagnostic frames sent → _latest_diag stays empty (all three real
    # 2026-07-16 latches had active_errors==0). Leg 1 crossed first (leg-1-first,
    # matching the forensics) at dev=-0.552 rev.
    hb = HeartbeatT2J(t_teensy_us=1, link_state=int(LinkState.UP),
                      bus1_health=int(BusHealth.OK), bus2_health=int(BusHealth.OK),
                      fault_state=int(FaultState.MAX_DEVIATION),
                      flags=0, uptime_ms=1,
                      live_deviation=(-0.55, -0.31, 0.34, 0.33, -0.12, 0.41),
                      max_dev_leg=1, max_dev_value=-0.552,
                      max_dev_u0=0.52, max_dev_enc=1.07)
    teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), hb.pack())
    assert _wait_until(lambda: node._latest_heartbeat is not None
                       and int(node._latest_heartbeat.fault_state)
                       == int(FaultState.MAX_DEVIATION))
    assert not node._latest_diag, "precondition: diag cache empty (zero ODrive errors)"
    node._publish_link_status()

    edge = [m for m in _messages(node._logger.error) if 'FAULT LATCHED' in m]
    assert len(edge) == 1, f"expected one edge ERROR, got {_messages(node._logger.error)}"
    assert 'MAX_DEVIATION' in edge[0]
    assert 'leg 1' in edge[0]                       # frozen-snapshot attribution
    assert 'dev=-0.552 rev at trip' in edge[0]      # trip deviation
    assert 'live_dev=[' in edge[0]                  # multi-leg context vector
    assert '-0.55' in edge[0] and '+0.41' in edge[0]

    kv = {v.key: v.value for v in node.link_status_pub.published[-1].values}
    assert kv['guard_fault_leg'] == '1'             # direct gated field
    assert kv['max_dev_leg'] == '1'                 # raw frozen snapshot present


def test_guard_fault_leg_empty_when_cleared_but_snapshot_persists(bridge):
    """max_dev_leg is 'last latch since boot' and PERSISTS after /clear_errors,
    so a consumer asking 'is a leg AT FAULT right now?' must not read it raw.
    guard_fault_leg gates on an ACTIVE MAX_DEVIATION latch: with fault_state==NONE
    but max_dev_leg still 1 from a prior latch, guard_fault_leg is '' while the
    raw max_dev_leg snapshot stays visible."""
    teensy, node = bridge
    hb = HeartbeatT2J(t_teensy_us=1, link_state=int(LinkState.UP),
                      bus1_health=int(BusHealth.OK), bus2_health=int(BusHealth.OK),
                      fault_state=int(FaultState.NONE), flags=0, uptime_ms=1,
                      max_dev_leg=1, max_dev_value=-0.552)
    teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), hb.pack())
    assert _wait_until(lambda: node._latest_heartbeat is not None)
    node._publish_link_status()
    kv = {v.key: v.value for v in node.link_status_pub.published[-1].values}
    assert kv['guard_fault_leg'] == ''              # gated off — fault cleared
    assert kv['max_dev_leg'] == '1'                 # raw snapshot still visible


def test_guard_fault_clear_logs_info_and_resets_anchor(bridge):
    """Clearing the fault (fault_state→NONE) logs an INFO and resets the
    persistent-reminder anchor so a future latch starts fresh."""
    from unittest.mock import MagicMock
    teensy, node = bridge
    faulted = HeartbeatT2J(t_teensy_us=1, link_state=int(LinkState.UP),
                           bus1_health=int(BusHealth.OK), bus2_health=int(BusHealth.OK),
                           fault_state=int(FaultState.ODRIVE_FATAL),
                           flags=0, uptime_ms=1)
    teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), faulted.pack())
    assert _wait_until(lambda: node._latest_heartbeat is not None
                       and int(node._latest_heartbeat.fault_state)
                       == int(FaultState.ODRIVE_FATAL))
    node._publish_link_status()            # latch
    assert node._last_guard_latch_log_t is not None

    cleared = HeartbeatT2J(t_teensy_us=2, link_state=int(LinkState.UP),
                           bus1_health=int(BusHealth.OK), bus2_health=int(BusHealth.OK),
                           fault_state=int(FaultState.NONE),
                           flags=0, uptime_ms=2)
    teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), cleared.pack())
    assert _wait_until(lambda: node._latest_heartbeat is not None
                       and int(node._latest_heartbeat.fault_state)
                       == int(FaultState.NONE))
    node._logger = MagicMock()
    node._publish_link_status()
    assert any('cleared' in m for m in _messages(node._logger.info))
    assert node._last_guard_latch_log_t is None


# ── profile ────────────────────────────────────────────────────

def test_profile_published(bridge):
    teensy, node = bridge
    prof = Profile(t_teensy_us=1, free_heap_bytes=40000,
                   interp_deadline_misses=0, udp_rtt_us=250)
    teensy.send_to_jetson(int(MsgType.PROFILE), prof.pack())
    assert _wait_until(lambda: node._latest_profile is not None)
    node._publish_profile()
    assert len(node.profile_pub.published) == 1
    msg = node.profile_pub.published[0]
    kv = {v.key: v.value for v in msg.values}
    assert kv['free_heap_bytes'] == '40000'
    assert kv['interp_deadline_misses'] == '0'


# ── TIME_OF_DAY responder (inbound RPC, ADR-0008) ─────────────

def test_time_of_day_responder(bridge):
    """The Teensy-initiated TIME_OF_DAY_QUERY gets an OK response with 8 bytes."""
    import struct
    from teensy_link import RpcMethod, RpcStatus, RpcResponse

    teensy, node = bridge
    teensy.send_rpc_request(int(RpcMethod.TIME_OF_DAY_QUERY), req_id=77)
    got = teensy.wait_for(int(MsgType.RPC_RESPONSE), count=1, timeout=2.0)
    assert got, "no RPC_RESPONSE to TIME_OF_DAY_QUERY"
    resp = RpcResponse.unpack(got[0].payload[:p.RPC_RESPONSE_SIZE])
    assert resp.req_id == 77
    assert resp.status == int(RpcStatus.OK)
    assert resp.res_len == 8
    blob = got[0].payload[p.RPC_RESPONSE_SIZE:p.RPC_RESPONSE_SIZE + resp.res_len]
    (wall_us,) = struct.unpack('<Q', blob)
    assert wall_us > 0


# ── ARMING_CONTRACT (A1 / A5) ──────────────────────────────────

def test_enable_setpoint_output_true_is_inert_no_boot_arm():
    """A1 — the boot-arm path is REMOVED: constructing the node with
    enable_setpoint_output=true must NOT arm. Boot-arming ran ZERO
    preconditions (nothing can legitimately stream at __init__), so it armed
    onto a silent :5557 and the firmware's MPC-staleness watchdog latched
    within one guard tick — reconfirmed live 2026-07-15 21:32. The parameter
    survives only so stale launch invocations fail loud (an ERROR log), not
    weird (an instant latch)."""
    from unittest.mock import patch
    from jugglebot.teensy_bridge_node import TeensyBridgeNode

    orig = TeensyBridgeNode.declare_parameter

    def declare(self, name, default_value):
        if name == 'enable_setpoint_output':
            default_value = True
        return orig(self, name, default_value)

    with patch.object(TeensyBridgeNode, 'declare_parameter', declare):
        teensy, client, node = _build_paired_node()
    try:
        assert bool(node.get_parameter('enable_setpoint_output').value) is True
        assert node._mpc_active is False          # NOT armed
        assert node._sp_thread is None            # no ingest thread
        assert node._sp_source is None            # no :5557 subscription
    finally:
        _teardown(teensy, client, node)


def test_link_status_surfaces_firmware_arm_took_bit(bridge):
    """A5 — HeartbeatT2J bit3 (the firmware's own mpc_active, put on the wire
    precisely so 'a setpoint source can verify its arm actually took') surfaces
    as 'teensy_mpc_active' on /link_status. A persistent split between it and
    the host 'mpc_active' KeyValue means an arm/disarm never took on the wire."""
    teensy, node = bridge
    hb = HeartbeatT2J(t_teensy_us=1, link_state=int(LinkState.UP),
                      bus1_health=int(BusHealth.OK), bus2_health=int(BusHealth.OK),
                      fault_state=int(FaultState.NONE),
                      flags=0x1 | 0x8, uptime_ms=1)  # bit0 time_synced, bit3 MPC_ACTIVE
    teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), hb.pack())
    assert _wait_until(lambda: node._latest_heartbeat is not None)
    node._publish_link_status()
    kv = {v.key: v.value for v in node.link_status_pub.published[-1].values}
    assert kv['teensy_mpc_active'] == '1'   # firmware says armed
    assert kv['mpc_active'] == '0'          # host disarmed — the split is visible
