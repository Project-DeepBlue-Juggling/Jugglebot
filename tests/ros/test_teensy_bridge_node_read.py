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

from controller.teensy_link import TeensyLinkClient
from controller.teensy_link import protocol as p
from controller.teensy_link import (
    MsgType, LinkState, BusHealth, FaultState,
    HeartbeatJ2T, HeartbeatT2J, Telemetry, Diagnostic, Profile,
)

# Reuse the FakeTeensy loopback peer.
from tests.teensy_link.conftest import FakeTeensy


# ── Helpers ────────────────────────────────────────────────────

def _wait_until(predicate, timeout=2.0, interval=0.005):
    """Poll ``predicate`` until true or timeout. Returns the final bool."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


def _build_paired_node(*, boot_state_read=False):
    """Build a (FakeTeensy, client, node) triple wired on loopback.

    Mirrors the pairing in tests/teensy_link/conftest.py::fake_teensy_and_client
    but injects the started client into a TeensyBridgeNode.

    ``boot_state_read`` defaults False so the cold-start boot read does NOT
    fire during construction (these tests don't wire a Platform-Teensy STATE_READ
    responder before __init__, and exercise the cold-start path explicitly via
    tests/ros/test_teensy_bridge_node_coldstart.py). Pass True to exercise the boot
    read (the cache then ends conservative unless a responder is pre-wired).
    """
    from jugglebot.teensy_bridge_node import TeensyBridgeNode

    teensy = FakeTeensy(bind_host="127.0.0.1", stream_port=0, rpc_port=0)
    teensy.start()
    client = TeensyLinkClient(
        teensy_addr=("127.0.0.1", teensy.stream_port),
        rpc_port=teensy.rpc_port,
        local_bind_stream=0,
        local_bind_rpc=0,
        bind_host="127.0.0.1",
    )
    # stow_on_shutdown=False: on_shutdown() must not fire a profiled DEACTIVATE
    # against the FakeTeensy (which never completes the descent → DeactivateMonitor
    # would hang the test). Production defaults True; the stow path is exercised
    # explicitly in test_teensy_bridge_node_shutdown_stow.py.
    node = TeensyBridgeNode(client=client, boot_state_read=boot_state_read,
                            stow_on_shutdown=False)  # __init__ starts the client

    jetson_stream_port = client._stream_sock.getsockname()[1]
    jetson_rpc_port = client._rpc_sock.getsockname()[1]
    teensy.set_jetson_addr(("127.0.0.1", jetson_stream_port))

    def _send(msg_type, payload, port="stream"):
        host, _ = teensy._jetson_addr
        if port == "stream":
            seq = teensy._tx_seq_stream
            teensy._tx_seq_stream = (teensy._tx_seq_stream + 1) & 0xFFFF
            dest = (host, jetson_stream_port)
            sock = teensy.stream_sock
        else:
            seq = teensy._tx_seq_rpc
            teensy._tx_seq_rpc = (teensy._tx_seq_rpc + 1) & 0xFFFF
            dest = (host, jetson_rpc_port)
            sock = teensy.rpc_sock
        frame = p.encode_frame(msg_type, seq, payload)
        sock.sendto(frame, dest)
        return seq

    teensy.send_to_jetson = _send  # type: ignore[assignment]
    return teensy, client, node


@pytest.fixture
def bridge():
    """Yield (teensy, node) wired on loopback; tear everything down on exit."""
    teensy, client, node = _build_paired_node()
    yield teensy, node
    node.on_shutdown()       # closes RPC server/client/TOD; injected client untouched
    client.stop()
    teensy.stop()


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
                      temp_fet=30.0, temp_motor=25.0, bus_voltage=48.0)
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
    from controller.teensy_link import RpcMethod, RpcStatus, RpcResponse

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
