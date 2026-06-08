"""Teensy can-bridge ROS 2 node — UDP-sourced mirror of ``can_node.py``.

This is the Phase-10b side-by-side bridge: it exposes the same observable
surface as :mod:`jugglebot.can_node` (robot state, hand telemetry, link/fault
health) but sources everything from the can-bridge Teensy over the dedicated
UDP link (``controller/teensy_link``) instead of socketcan. It runs *alongside*
``can_node`` during the migration — **every topic it owns lives under
``/teensy/*``; it never publishes to a production topic name** and never
modifies any production code path.

Safety invariants this node upholds (non-negotiable):

* **``mpc_active`` defaults to 0.** The J→T heartbeat carries ``flags=0``
  (``mpc_active`` clear) on every startup path. Setpoint output is gated behind
  the ``~enable_setpoint_output`` parameter (default ``false``) and is wired in
  Commit 3. While disabled, no ``Setpoint`` frame is ever sent and the Teensy
  will not enable leg output. There is no code path through ``__init__`` that
  sets ``mpc_active`` without an explicit operator opt-in.
* **Never command a dead link.** The link health monitor (Commit 2) mirrors
  ``can_node._watchdog_check``'s deferred-stow latch
  (``logbook/2026-05-19-can-loss-fault-response-safety-inversion.md``): it does
  not command the Teensy while the link is down; a stow is deferred to confirmed
  reconnect.

Threading model: the :class:`TeensyLinkClient` RX thread decodes frames and
invokes our callbacks. Those callbacks are kept short — they only stash the
latest decoded frame under a lock. All ROS publishing happens on the executor
thread via timers, mirroring ``can_node``'s "poll on one thread, publish on a
timer" split. This respects the teensy_link callback contract ("keep them fast;
enqueue heavy work").

Build order: Commit 1 adds the read side (this file). Commit 2 adds the link
watchdog + deferred-stow latch. Commit 3 adds the gated setpoint downlink.
Commit 4 adds the RPC service surface. Each is a separate, test-gated commit.
"""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.node import Node

from jugglebot_interfaces.msg import (
    BallButlerHeartbeat,
    HandTelemetryMessage,
    MotorStateSingle,
    RobotState,
    SetMotorVelCurrLimitsMessage,
)
from jugglebot_interfaces.srv import ODriveCommandService, SendBallButlerCommand
from geometry_msgs.msg import Quaternion
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from std_srvs.srv import Trigger

from controller.teensy_link import (
    TeensyLinkClient,
    RpcClient,
    RpcServer,
    TimeOfDayServer,
    RpcMethod,
    RpcError,
    MsgType,
    LinkState,
    BusHealth,
    FaultState,
    HeartbeatT2J,
    Telemetry,
    Diagnostic,
    Profile,
)
from controller.teensy_link import protocol as p
from controller.teensy_link import rpc_args
from controller.teensy_link.fault_logic import LinkLossLatch
from controller.teensy_link.setpoint_pump import SetpointPump
import jugglebot.hardware_config as hw


# ── Constants ──────────────────────────────────────────────────
# Mirror of can_node._HEARTBEAT_TIMEOUT_S: the can-bridge link is declared lost
# after this long without a T→J heartbeat. 2 s matches the CAN watchdog so the
# two nodes agree on what "lost" means during the side-by-side window.
_HEARTBEAT_TIMEOUT_S = 2.0

# HeartbeatJ2T.flags bit0 = mpc_active (guard ENABLED). Sending this set tells
# the Teensy the Jetson is driving setpoints. It MUST stay 0 unless the operator
# has explicitly enabled setpoint output (Commit 3).
_FLAG_MPC_ACTIVE = 0x1

# HeartbeatT2J.flags bits (T→J), per the protocol SPEC.
_T2J_FLAG_TIME_SYNCED = 0x1
_T2J_FLAG_STOW_PENDING = 0x2
_T2J_FLAG_ALL_AXIS_HB_OK = 0x4

# HeartbeatT2J.bb_flags bits (T→J, Phase A BB cutover).
_T2J_BB_FLAG_BALL_IN_HAND  = 0x1
_T2J_BB_FLAG_HEARTBEAT_SEEN  = 0x2
_T2J_BB_FLAG_HEARTBEAT_STALE = 0x4

# Axis layout (from the generated protocol): legs 0..5, hand = 6, NUM_AXES = 7.
_NUM_LEGS = p.NUM_LEGS
_HAND_AXIS = p.NUM_LEGS         # 6
_NUM_AXES = p.NUM_AXES          # 7

# ODrive undervoltage error/disarm bit (matches odrive.ERR_DC_BUS_UNDER_VOLTAGE).
_ERR_DC_BUS_UNDER_VOLTAGE = 512

# ODrive CLOSED_LOOP axis state (matches odrive.AXIS_STATES['CLOSED_LOOP']).
_AXIS_STATE_CLOSED_LOOP = 8


def _enum_name(enum_cls, value: int) -> str:
    """Best-effort enum-member name for a wire value (falls back to ``v<n>``)."""
    try:
        return enum_cls(value).name
    except ValueError:
        return f"v{value}"


class _MotorGuardSetpointSource:
    """SUB-only connection to motor_guard's telemetry PUB (tcp://127.0.0.1:5556).

    Deliberately does NOT reuse ``jugglebot.motion.ipc.BridgeIPC``: that also
    BINDS the command PUB on :5555, which the production ``motion_bridge_node``
    already binds — two binds on one address fail. The Teensy bridge only needs
    to *read* the 500 Hz setpoint telemetry, never to command motor_guard.

    Mirrors BridgeIPC's SUB tuning (RCVHWM=2 + drain-to-latest) and the msgpack
    wire format (``[topic, msgpack(dict)]``). zmq/msgpack are imported lazily so
    a read-only / setpoint-disabled bridge has no hard dependency on them.
    """

    def __init__(self, addr: str = 'tcp://127.0.0.1:5556'):
        import zmq  # lazy — only when setpoint output is enabled
        self._zmq = zmq
        import msgpack
        self._msgpack = msgpack
        self._ctx = zmq.Context()
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.setsockopt(zmq.RECONNECT_IVL, 100)       # match BridgeIPC
        self._sub.setsockopt(zmq.RECONNECT_IVL_MAX, 200)
        self._sub.setsockopt(zmq.RCVHWM, 2)                # bounded queue
        self._sub.connect(addr)
        self._sub.setsockopt(zmq.SUBSCRIBE, b'')
        self._sub.setsockopt(zmq.RCVTIMEO, 0)              # non-blocking

    def recv_latest(self) -> dict | None:
        """Drain the SUB and return the latest telemetry dict, or None."""
        latest = None
        while True:
            try:
                frames = self._sub.recv_multipart(flags=self._zmq.NOBLOCK)
                latest = self._msgpack.unpackb(frames[1], raw=False)
            except self._zmq.Again:
                break
        return latest

    def close(self) -> None:
        try:
            self._sub.close()
            self._ctx.term()
        except Exception:  # noqa: BLE001
            pass


class TeensyBridgeNode(Node):
    """ROS 2 node bridging the can-bridge Teensy UDP link to ``/teensy/*``.

    Args:
        client: An optional already-constructed :class:`TeensyLinkClient`. When
            ``None`` (production), one is built from the node's parameters and
            started. Tests inject a loopback client paired with ``FakeTeensy``.
            When injected, the node does NOT take ownership — it will not stop
            the client on shutdown (the test fixture owns its lifecycle).
    """

    def __init__(self, client: TeensyLinkClient | None = None,
                 setpoint_source=None):
        super().__init__('teensy_bridge_node')

        # ── Parameters ─────────────────────────────────────────
        self.declare_parameter('teensy_ip', p.TEENSY_IP)
        # SAFETY-CRITICAL: setpoint output is OFF by default. Wired in Commit 3.
        self.declare_parameter('enable_setpoint_output', False)
        self.declare_parameter('heartbeat_timeout_s', _HEARTBEAT_TIMEOUT_S)

        self._heartbeat_timeout_s = float(
            self.get_parameter('heartbeat_timeout_s').value)

        # ── Transport client ───────────────────────────────────
        if client is None:
            teensy_ip = str(self.get_parameter('teensy_ip').value)
            self._client = TeensyLinkClient(
                teensy_addr=(teensy_ip, p.PORT_STREAM))
            self._own_client = True
        else:
            self._client = client
            self._own_client = False
        # start() is idempotent — safe whether the client is fresh or injected.
        self._client.start()

        # Inbound RPC server + the wall-clock anchor (ADR-0008). The Teensy is
        # the time-sync master; we answer its TIME_OF_DAY_QUERY with our
        # CLOCK_REALTIME. Registered unconditionally — it is read-only and safe.
        self._rpc_server = RpcServer(self._client)
        self._tod = TimeOfDayServer(self._rpc_server)

        # Outbound RPC client (used by the service surface in Commit 4).
        self._rpc = RpcClient(self._client)

        # ── Latest-frame state (written on RX thread, read on ROS thread) ──
        self._lock = threading.Lock()
        self._latest_telemetry: Telemetry | None = None
        self._latest_heartbeat: HeartbeatT2J | None = None
        self._latest_profile: Profile | None = None
        # Per-axis latest Diagnostic (one axis per frame on the wire).
        self._latest_diag: dict[int, Diagnostic] = {}

        # Link-loss deferred-stow latch (the bridge's UDP-link watchdog — the
        # Jetson↔Teensy analog of can_node._watchdog_check; the CAN-side latch is
        # owned by the Teensy firmware). See controller/teensy_link/fault_logic.py.
        self._link_latch = LinkLossLatch()
        self._last_link_lost = False  # edge detector for logging

        # ── Heartbeat: ALWAYS mpc_active=0 at startup ──────────
        # flags=0 ⇒ mpc_active clear. set_heartbeat_flags(0) AFTER start_heartbeat
        # makes the pin STRUCTURAL: start_heartbeat is a no-op against an already-
        # running heartbeat thread (an injected, pre-started client), so without
        # this explicit clear a stale flags=1 could survive construction. The
        # defensive 0-write closes that — mpc_active=0 on every startup path.
        self._mpc_active = False
        self._client.start_heartbeat(hz=float(p.HEARTBEAT_HZ), flags=0)
        self._client.set_heartbeat_flags(0)

        # ── Subscriptions to T→J streams (RX-thread callbacks) ──
        self._client.subscribe(int(MsgType.HEARTBEAT_T2J), self._on_heartbeat_t2j)
        self._client.subscribe(int(MsgType.TELEMETRY), self._on_telemetry)
        self._client.subscribe(int(MsgType.DIAGNOSTIC), self._on_diagnostic)
        self._client.subscribe(int(MsgType.PROFILE), self._on_profile)

        # ── Publishers (all under /teensy/*) ───────────────────
        self.robot_state_pub = self.create_publisher(
            RobotState, '/teensy/robot_state', 10)
        self.hand_telemetry_pub = self.create_publisher(
            HandTelemetryMessage, '/teensy/hand_telemetry', 10)
        self.link_status_pub = self.create_publisher(
            DiagnosticStatus, '/teensy/link_status', 10)
        self.profile_pub = self.create_publisher(
            DiagnosticStatus, '/teensy/profile', 10)

        # ── Ball Butler (Phase A cutover, production names) ────
        # Intentional naming deviation from D1's "all under /teensy/*"
        # convention: with USB-CAN removed, the dual-publisher risk D1 was
        # preventing is moot (can_node is gone for BB). The bridge inherits
        # the production names so the GUI / orchestrator / mocap_node /
        # throw_director see no name change across the cutover. Leg/hand
        # services keep /teensy/* until phase C of the cutover.
        self.bb_heartbeat_pub = self.create_publisher(
            BallButlerHeartbeat, 'bb/heartbeat', 10)

        # ── RPC service surface (Commit 4) — all under /teensy/* ──
        # ODrive control issued over the can-bridge link via RpcClient. The
        # can-bridge owns legs 0-5 only — the hand is the platform Teensy's, and
        # the firmware rejects hand-axis RPCs — so these target legs/broadcast.
        # Services using EXISTING ROS types are wired here; the arg-bearing
        # per-axis ops (set_axis_state, set_controller_mode, per-axis gains,
        # set_absolute_position, sdo_read/write) are tested node methods pending
        # new jugglebot_interfaces .srv types (handoff D10).
        self.create_service(Trigger, '/teensy/clear_errors', self._svc_clear_errors)
        self.create_service(Trigger, '/teensy/reboot_odrives', self._svc_reboot_odrives)
        self.create_service(Trigger, '/teensy/encoder_search', self._svc_encoder_search)
        self.create_service(Trigger, '/teensy/home', self._svc_home)
        self.create_service(ODriveCommandService, '/teensy/odrive_command',
                            self._svc_odrive_command)
        self.create_subscription(
            SetMotorVelCurrLimitsMessage, '/teensy/set_motor_vel_curr_limits',
            self._sub_vel_curr_limits, 10)

        # ── Ball Butler services (production names, Phase A cutover) ────
        # The four bb/* services formerly served by can_node. The firmware
        # gates each TX on bb_present(); we translate the ERR_BUS_DOWN that
        # gate produces into a silent-success for bb/calibrate to preserve
        # can_node._svc_bb_calibrate's HOMING semantics (allows homing to
        # complete without BB attached). The other three propagate the
        # error so an operator sees a failed throw/reload/reset.
        self.create_service(SendBallButlerCommand, 'bb/send_throw_command',
                            self._svc_bb_throw)
        self.create_service(Trigger, 'bb/reload',    self._svc_bb_reload)
        self.create_service(Trigger, 'bb/reset',     self._svc_bb_reset)
        self.create_service(Trigger, 'bb/calibrate', self._svc_bb_calibrate)

        # ── Timers (publish on the executor thread) ────────────
        self.create_timer(0.01, self._publish_robot_state)     # 100 Hz (telem rate)
        self.create_timer(0.01, self._publish_hand_telemetry)  # 100 Hz
        self.create_timer(0.1, self._publish_link_status)      # 10 Hz (heartbeat rate)
        self.create_timer(0.1, self._publish_bb_heartbeat)     # 10 Hz BB (matches CAN1 rate)
        self.create_timer(1.0, self._publish_profile)          # 1 Hz (profile rate)
        self.create_timer(1.0, self._health_check)             # 1 Hz link watchdog

        # ── Setpoint downlink (Commit 3) — DEFAULT DISABLED ────
        # The 40/500 Hz hot path. The SetpointPump (pure packing + per-step
        # safety gate) is always constructed (cheap), but the ZMQ source and the
        # ingest thread are created ONLY when ~enable_setpoint_output is true.
        # While disabled there is NO setpoint thread and the heartbeat keeps
        # mpc_active=0, so the Teensy will not enable leg output. The operator
        # flips the parameter (and restarts the node) only after bench validation.
        self._sp_pump = SetpointPump(
            num_legs=p.NUM_LEGS, max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV)
        self._sp_source = None
        self._sp_thread = None
        self._sp_stop = threading.Event()
        enable_sp = bool(self.get_parameter('enable_setpoint_output').value)
        if enable_sp:
            self._start_setpoint_output(setpoint_source)

        peer = (self.get_parameter('teensy_ip').value
                if client is None else 'injected')
        enable_sp = bool(self.get_parameter('enable_setpoint_output').value)
        self.get_logger().info(
            f"TeensyBridgeNode up — peer={peer} stream={p.PORT_STREAM} "
            f"rpc={p.PORT_RPC}, enable_setpoint_output={enable_sp} "
            f"(mpc_active pinned to 0)")

    # ═══════════════════════════════════════════════════════════
    # RX-thread frame callbacks — keep these short (stash + return)
    # ═══════════════════════════════════════════════════════════

    def _on_heartbeat_t2j(self, msg_type, seq, payload, addr):
        try:
            hb = HeartbeatT2J.unpack(payload)
        except Exception:  # noqa: BLE001 — never let a bad frame kill the RX thread
            return
        with self._lock:
            self._latest_heartbeat = hb

    def _on_telemetry(self, msg_type, seq, payload, addr):
        try:
            tm = Telemetry.unpack(payload)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._latest_telemetry = tm

    def _on_diagnostic(self, msg_type, seq, payload, addr):
        try:
            dg = Diagnostic.unpack(payload)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._latest_diag[int(dg.axis_id)] = dg

    def _on_profile(self, msg_type, seq, payload, addr):
        try:
            pr = Profile.unpack(payload)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._latest_profile = pr

    # ═══════════════════════════════════════════════════════════
    # Read-side publishers (mirror can_node field-by-field)
    # ═══════════════════════════════════════════════════════════

    def _build_motor_states(self, telem: Telemetry,
                            diag: dict[int, Diagnostic]) -> list:
        """Build a NUM_AXES list of MotorStateSingle from telemetry + diagnostics.

        pos/vel come from the 100 Hz Telemetry frame; per-axis state, errors,
        currents, temps, and bus voltage come from the on-change Diagnostic
        frame for that axis. Fields the can-bridge link does not carry
        (procedure_result, trajectory_done, bus_current) are left at their
        MotorStateSingle defaults — documented in the handoff. Mirrors the
        construction can_node feeds into RobotState.motor_states.
        """
        states = []
        for axis in range(_NUM_AXES):
            s = MotorStateSingle()
            # pos/vel are already in Jugglebot convention (positive = extension)
            # — the Teensy sign-corrects from ODrive convention before sending.
            s.pos_estimate = float(telem.pos_rev[axis])
            s.vel_estimate = float(telem.vel_rps[axis])
            d = diag.get(axis)
            if d is not None:
                s.current_state = int(d.axis_state)
                s.active_errors = int(d.active_errors)
                s.disarm_reason = int(d.disarm_reason)
                s.iq_setpoint = float(d.iq_setpoint)
                s.iq_measured = float(d.iq_measured)
                s.fet_temp = float(d.temp_fet)
                s.motor_temp = float(d.temp_motor)
                s.bus_voltage = float(d.bus_voltage)
            states.append(s)
        return states

    def _publish_robot_state(self):
        """Publish /teensy/robot_state, mirroring can_node._publish_robot_state.

        Suppressed until the first Telemetry frame arrives so the topic never
        carries a misleading all-zero / all-IDLE snapshot before the link is up.
        """
        try:
            with self._lock:
                telem = self._latest_telemetry
                hb = self._latest_heartbeat
                diag = dict(self._latest_diag)
            if telem is None:
                return  # No real data yet — don't publish a phantom snapshot.

            states = self._build_motor_states(telem, diag)

            msg = RobotState()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.motor_states = states

            # Typed fault flags. The Teensy owns the fault state machine, so the
            # headline determination is its single-valued HeartbeatT2J.fault_state.
            # But fault_state is single-valued, so a higher-priority fault (e.g.
            # CAN_BUS_DOWN) can MASK a concurrent ODrive fault — which would make
            # /teensy/robot_state disagree with /robot_state for the same hardware
            # state (defeating the side-by-side comparison). So we also OR in the
            # raw per-leg fatal conditions can_node uses (active error on any leg,
            # or disarm-while-CLOSED_LOOP — can_node._handle_error:416-421), keeping
            # the comparison faithful WITHOUT re-running the Teensy's stateful
            # soft-reset machine on the Jetson (which fault_state already reports).
            fault_state = int(hb.fault_state) if hb is not None else 0
            # After the three-bus remap (ADR-0013 / firmware HANDOFF D4) the on-wire
            # bus1_health slot carries the Jugglebot CORE bus (CAN3: 6 legs + hand) --
            # the bus whose BUS_OFF is fatal for the legs. (bus2_health is now Ball
            # Butler; the cone bus health is not yet on the uplink -- TODO phase-10b.)
            core_bus_health = int(hb.bus1_health) if hb is not None else 0
            legs = states[:_NUM_LEGS]  # the can-bridge owns legs 0-5 (hand = platform Teensy)
            any_leg_active_err = any(s.active_errors != 0 for s in legs)
            any_leg_disarm_in_cl = any(
                s.disarm_reason != 0 and s.current_state == _AXIS_STATE_CLOSED_LOOP
                for s in legs)
            msg.has_fatal_odrive_error = (
                fault_state == int(FaultState.ODRIVE_FATAL)
                or any_leg_active_err or any_leg_disarm_in_cl)
            msg.has_fatal_can_error = (
                fault_state == int(FaultState.CAN_BUS_DOWN)
                or core_bus_health == int(BusHealth.BUS_OFF))
            # Undervoltage: matches can_node, which sets undervoltage_error ONLY
            # from a BITWISE test on active_errors (can_node._handle_error:436);
            # disarm_reason==UV is used by can_node solely in its clear predicate,
            # never to ASSERT UV. active_errors/disarm are ODrive bitfields, so use
            # bitwise & (not ==), active_errors only.
            msg.has_undervoltage = any(
                s.active_errors & _ERR_DC_BUS_UNDER_VOLTAGE for s in legs)
            # firmware_validated: not carried on the can-bridge link in 10b
            # (the Teensy validates ODrive versions internally — Phase 5 — but
            # does not yet expose the result). Conservatively False; handoff gap.
            msg.firmware_validated = False

            # Human-readable error strings (logging/rosbag parity with can_node).
            errors = []
            if msg.has_undervoltage:
                errors.append("Undervoltage detected. Was the E-stop hit?")
            if msg.has_fatal_odrive_error:
                errors.append(
                    f"Fatal ODrive issue (Teensy fault_state="
                    f"{_enum_name(FaultState, fault_state)}).")
            if msg.has_fatal_can_error:
                errors.append("Fatal CAN bus issue.")
            msg.error = errors

            # Teensy-persisted cold-start state is not on the link in 10b
            # (Phase 9 RPC). Conservative defaults; documented in the handoff.
            msg.encoder_search_complete = False
            msg.is_homed = False
            msg.levelling_complete = False
            msg.pose_offset_rad = [0.0, 0.0]
            msg.pose_offset_quat = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)

            self.robot_state_pub.publish(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Robot state publish error: {e}",
                                    throttle_duration_sec=5.0)

    def _publish_hand_telemetry(self):
        """Publish /teensy/hand_telemetry from axis 6 of the Telemetry frame.

        Mirrors can_node._publish_hand_telemetry's measured side. The command
        fields (pos_cmd/vel_ff_cmd/tor_ff_cmd) are not echoed on the can-bridge
        link in 10b (the hand setpoint path is out of scope), so they are 0.
        """
        try:
            with self._lock:
                telem = self._latest_telemetry
                diag = self._latest_diag.get(_HAND_AXIS)
            if telem is None:
                return
            msg = HandTelemetryMessage()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.pos_cmd = 0.0
            msg.vel_ff_cmd = 0.0
            msg.tor_ff_cmd = 0.0
            msg.pos_meas = float(telem.pos_rev[_HAND_AXIS])
            msg.vel_meas = float(telem.vel_rps[_HAND_AXIS])
            msg.iq_meas = float(diag.iq_measured) if diag is not None else 0.0
            self.hand_telemetry_pub.publish(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Hand telemetry error: {e}",
                                    throttle_duration_sec=5.0)

    def _link_age_us(self) -> int | None:
        """Microseconds since the last T→J heartbeat, or None if never seen."""
        return self._client.time_since_last_t2j_heartbeat_us()

    # ═══════════════════════════════════════════════════════════
    # Link-loss watchdog + deferred-stow latch (Commit 2)
    # ═══════════════════════════════════════════════════════════

    def _health_check(self):
        """1 Hz link-loss watchdog — the Jetson↔Teensy analog of can_node's
        _watchdog_check. Drives the deferred-stow latch.

        Invariant (ported from logbook/2026-05-19-can-loss-fault-response-
        safety-inversion.md): on confirmed link loss, ARM the deferred-stow
        latch and do NOT command the Teensy (the firmware's own fault machine
        holds the legs safely while the link is down — and frames wouldn't be
        delivered anyway). On confirmed reconnect, the latch stays ``stow_pending``
        and the bridge SURFACES it on /teensy/link_status for the operator /
        orchestrator. The bridge does NOT auto-execute a stow: there is no stow
        RPC until firmware Phase 9, and the Teensy already owns the profiled
        CAN-side stow (decision D12). This is the "always stow on confirmed
        reconnect, never command a dead link" invariant, minus the executor the
        bridge does not have.
        """
        try:
            age_us = self._link_age_us()
            seen = age_us is not None
            stale = seen and age_us > self._heartbeat_timeout_s * 1e6
            self._link_latch.update(stale=bool(stale), seen=bool(seen))

            if self._link_latch.link_lost and not self._last_link_lost:
                self.get_logger().error(
                    "Teensy link LOST — deferred stow armed; NOT commanding "
                    "the Teensy while the link is down (firmware holds the legs).")
            elif not self._link_latch.link_lost and self._last_link_lost:
                if self._link_latch.stow_pending:
                    self.get_logger().error(
                        "Teensy link RESTORED — a mid-run loss occurred. "
                        "STOW PENDING: no stow RPC exists yet (firmware Phase 9); "
                        "operator/orchestrator must stow the platform. Surfaced "
                        "on /teensy/link_status (bridge_stow_pending=1).")
                else:
                    self.get_logger().info("Teensy link RESTORED.")
            self._last_link_lost = self._link_latch.link_lost
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Health check error: {e}",
                                    throttle_duration_sec=5.0)

    # ═══════════════════════════════════════════════════════════
    # Setpoint downlink (Commit 3) — gated, default disabled
    # ═══════════════════════════════════════════════════════════

    def _set_mpc_active(self, active: bool):
        """Set the mpc_active state AND the J→T heartbeat flag together.

        This is the ONLY place mpc_active becomes 1. The heartbeat flag tells the
        Teensy the Jetson is driving setpoints (the firmware's guard-ENABLE
        precondition). Pinned to 0 unless ~enable_setpoint_output is true.
        """
        self._mpc_active = bool(active)
        self._client.set_heartbeat_flags(_FLAG_MPC_ACTIVE if active else 0)
        # Enable is the safety-relevant transition (warning); disable is benign.
        log = self.get_logger().warning if active else self.get_logger().info
        log(f"mpc_active set to {int(self._mpc_active)} — setpoint output "
            f"{'ENABLED' if active else 'disabled'}.")

    def _start_setpoint_output(self, setpoint_source=None):
        """Bring up the setpoint source + ingest thread and set mpc_active=1.

        Called from __init__ only when ~enable_setpoint_output is true.
        ``setpoint_source`` may be injected (tests); otherwise a real ZMQ SUB on
        motor_guard's :5556 is created.
        """
        if setpoint_source is not None:
            self._sp_source = setpoint_source
        else:
            self._sp_source = _MotorGuardSetpointSource()
        # Set mpc_active BEFORE starting the thread so the first tick it drains
        # is not silently dropped by the mpc_active gate (and so the Teensy is
        # told we are driving before any setpoint can flow).
        self._set_mpc_active(True)
        self._sp_stop.clear()
        self._sp_thread = threading.Thread(
            target=self._setpoint_loop, name="teensy_bridge_setpoint", daemon=True)
        self._sp_thread.start()

    def _setpoint_loop(self):
        """Dedicated thread: drain motor_guard telemetry → pack → gate → send."""
        while not self._sp_stop.is_set():
            try:
                telem = self._sp_source.recv_latest()
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"Setpoint source error: {e}",
                                        throttle_duration_sec=5.0)
                telem = None
            if telem is not None:
                self._process_setpoint(telem)
            else:
                # ~1 kHz idle poll (motor_guard publishes at 500 Hz).
                self._sp_stop.wait(0.001)

    def _process_setpoint(self, telem: dict):
        """Pack one telemetry tick into a Setpoint frame, gate it, and send.

        SAFETY gates, in order:
          1. mpc_active must be set (operator opt-in). Belt-and-suspenders — the
             ingest thread only runs when enabled, but a direct caller is gated
             here too.
          2. Never command a dead link (the deferred-stow latch's command gate).
          3. The SetpointPump's per-step clamp + NaN/short-vector rejection.
        Only a clean, accepted frame is transmitted.
        """
        if not self._mpc_active:
            return
        if not self._link_latch.command_allowed():
            return  # never command a dead link
        t_origin_us = int(time.time() * 1_000_000)
        sp, reason = self._sp_pump.build(telem, t_origin_us)
        if reason is not None:
            self.get_logger().error(
                f"Setpoint REJECTED (not sent): {reason}",
                throttle_duration_sec=1.0)
            return
        if sp is None:
            return  # feedback-only telemetry — nothing to send
        self._client.send_stream(int(MsgType.SETPOINT), sp.pack())

    def _publish_link_status(self):
        """Publish /teensy/link_status as a DiagnosticStatus.

        Surfaces the Teensy's reported link/fault/bus health AND the bridge's
        own view of link liveness (heartbeat age) plus — critically — the
        ``mpc_active`` flag we are sending, so an operator can confirm at a
        glance that setpoint output is disabled.
        """
        try:
            with self._lock:
                hb = self._latest_heartbeat
            age_us = self._link_age_us()

            msg = DiagnosticStatus()
            msg.name = 'teensy/link'
            msg.hardware_id = 'can_bridge_teensy'

            # Bridge-side liveness view (independent of the Teensy's self-report).
            if age_us is None:
                bridge_link = 'NO_HEARTBEAT'
            elif age_us > self._heartbeat_timeout_s * 1e6:
                bridge_link = 'LOST'
            else:
                bridge_link = 'UP'

            teensy_link = (_enum_name(LinkState, hb.link_state)
                           if hb is not None else 'UNKNOWN')
            teensy_fault = (_enum_name(FaultState, hb.fault_state)
                            if hb is not None else 'UNKNOWN')

            # Level: ERROR on lost link or any non-NONE Teensy fault; OK otherwise.
            fault_active = hb is not None and int(hb.fault_state) != int(FaultState.NONE)
            if bridge_link in ('LOST', 'NO_HEARTBEAT') or fault_active:
                msg.level = DiagnosticStatus.ERROR
                msg.message = f'link={bridge_link} fault={teensy_fault}'
            elif teensy_link == 'DEGRADED':
                msg.level = DiagnosticStatus.WARN
                msg.message = 'link degraded'
            else:
                msg.level = DiagnosticStatus.OK
                msg.message = 'OK'

            # Surface the bridge's own deferred-stow latch (Commit 2). When
            # bridge_stow_pending=1 after a reconnect, a mid-run link loss
            # happened and the platform needs stowing (no auto-stow RPC yet).
            if self._link_latch.stow_pending:
                msg.level = DiagnosticStatus.ERROR
                if msg.message == 'OK':
                    msg.message = 'stow pending (mid-run link loss)'

            stats = self._client.stats
            values = [
                KeyValue(key='bridge_link', value=bridge_link),
                KeyValue(key='teensy_link', value=teensy_link),
                KeyValue(key='fault_state', value=teensy_fault),
                KeyValue(key='mpc_active', value=str(int(self._mpc_active))),
                KeyValue(key='bridge_link_lost',
                         value=str(int(self._link_latch.link_lost))),
                KeyValue(key='bridge_stow_pending',
                         value=str(int(self._link_latch.stow_pending))),
                KeyValue(key='setpoints_sent',
                         value=str(self._sp_pump.frames_built)),
                KeyValue(key='setpoints_rejected',
                         value=str(self._sp_pump.frames_rejected)),
                KeyValue(key='heartbeat_age_ms',
                         value=('n/a' if age_us is None else f'{age_us / 1000.0:.0f}')),
                KeyValue(key='rx_frames', value=str(stats.rx_frames)),
                KeyValue(key='tx_frames', value=str(stats.tx_frames)),
                KeyValue(key='crc_errors', value=str(stats.crc_errors)),
                KeyValue(key='decode_errors', value=str(stats.decode_errors)),
            ]
            if hb is not None:
                values += [
                    KeyValue(key='bus1_health',
                             value=_enum_name(BusHealth, hb.bus1_health)),
                    KeyValue(key='bus2_health',
                             value=_enum_name(BusHealth, hb.bus2_health)),
                    KeyValue(key='uptime_ms', value=str(int(hb.uptime_ms))),
                    KeyValue(key='time_synced',
                             value=str(int(bool(hb.flags & _T2J_FLAG_TIME_SYNCED)))),
                    KeyValue(key='teensy_stow_pending',
                             value=str(int(bool(hb.flags & _T2J_FLAG_STOW_PENDING)))),
                ]
            msg.values = values
            self.link_status_pub.publish(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Link status publish error: {e}",
                                    throttle_duration_sec=5.0)

    def _publish_profile(self):
        """Publish /teensy/profile (firmware instrumentation) as DiagnosticStatus."""
        try:
            with self._lock:
                pr = self._latest_profile
            if pr is None:
                return
            msg = DiagnosticStatus()
            msg.name = 'teensy/profile'
            msg.hardware_id = 'can_bridge_teensy'
            # interp deadline misses are the headline health signal.
            msg.level = (DiagnosticStatus.WARN
                         if pr.interp_deadline_misses > 0
                         else DiagnosticStatus.OK)
            msg.message = (f'interp_misses={pr.interp_deadline_misses} '
                           f'heap={pr.free_heap_bytes}B')
            msg.values = [
                KeyValue(key='free_heap_bytes', value=str(pr.free_heap_bytes)),
                KeyValue(key='interp_deadline_misses',
                         value=str(pr.interp_deadline_misses)),
                KeyValue(key='interp_max_jitter_us',
                         value=str(pr.interp_max_jitter_us)),
                KeyValue(key='udp_rtt_us', value=str(pr.udp_rtt_us)),
                KeyValue(key='udp_jitter_us', value=str(pr.udp_jitter_us)),
                KeyValue(key='can1_util_pct', value=f'{pr.can1_util_x100 / 100.0:.1f}'),
                KeyValue(key='can2_util_pct', value=f'{pr.can2_util_x100 / 100.0:.1f}'),
                KeyValue(key='can1_rx', value=str(pr.can1_rx)),
                KeyValue(key='can1_tx', value=str(pr.can1_tx)),
                KeyValue(key='can2_rx', value=str(pr.can2_rx)),
                KeyValue(key='can2_tx', value=str(pr.can2_tx)),
                KeyValue(key='cpu_pct_x100',
                         value=','.join(str(c) for c in pr.cpu_pct_x100)),
            ]
            self.profile_pub.publish(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Profile publish error: {e}",
                                    throttle_duration_sec=5.0)

    # ═══════════════════════════════════════════════════════════
    # RPC service surface (Commit 4)
    # ═══════════════════════════════════════════════════════════

    def _call_rpc(self, method, args=b"", *, timeout=None, retries=None):
        """Issue an RPC; return (success, message, result_blob).

        Blocks on the calling thread until the response arrives (decoded on the
        RX thread) or the timeout × retries budget expires. RpcError/RpcTimeout
        are caught and reported as (False, message).
        """
        try:
            result = self._rpc.call(int(method), args,
                                    timeout=timeout, retries=retries)
            return True, 'OK', result
        except RpcError as e:
            return False, str(e), b""

    # ── Tested node methods (one per RpcMethod) — the reusable surface ──
    # Arg encoding (rpc_args, codegen-hoisted) + RpcClient call. ROS service
    # wrappers for the arg-bearing per-axis ops await new .srv types (D10); the
    # encodings are fully covered by tests/ros/test_teensy_bridge_node_rpc.py.

    def teensy_set_axis_state(self, axis, state):
        return self._call_rpc(RpcMethod.SET_AXIS_STATE,
                              rpc_args.encode_set_axis_state(axis, state))

    def teensy_set_controller_mode(self, axis, ctrl, input_mode):
        return self._call_rpc(RpcMethod.SET_CONTROLLER_MODE,
                              rpc_args.encode_set_controller_mode(axis, ctrl, input_mode))

    def teensy_set_vel_curr_limits(self, axis, vel_limit, curr_limit, **kw):
        return self._call_rpc(RpcMethod.SET_VEL_CURR_LIMITS,
                              rpc_args.encode_set_vel_curr_limits(axis, vel_limit, curr_limit),
                              **kw)

    def teensy_set_pos_gain(self, axis, pos_gain):
        return self._call_rpc(RpcMethod.SET_POS_GAIN,
                              rpc_args.encode_set_pos_gain(axis, pos_gain))

    def teensy_set_vel_gains(self, axis, vel_gain, vel_int_gain):
        return self._call_rpc(RpcMethod.SET_VEL_GAINS,
                              rpc_args.encode_set_vel_gains(axis, vel_gain, vel_int_gain))

    def teensy_set_absolute_position(self, axis, position):
        return self._call_rpc(RpcMethod.SET_ABSOLUTE_POSITION,
                              rpc_args.encode_set_absolute_position(axis, position))

    def teensy_clear_errors(self, axis=rpc_args.AXIS_ALL):
        return self._call_rpc(RpcMethod.CLEAR_ERRORS,
                              rpc_args.encode_clear_errors(axis))

    def teensy_reboot(self, axis=rpc_args.AXIS_ALL):
        return self._call_rpc(RpcMethod.REBOOT_ODRIVES,
                              rpc_args.encode_reboot(axis))

    def teensy_encoder_search(self, axis=rpc_args.AXIS_ALL):
        return self._call_rpc(RpcMethod.ENCODER_SEARCH,
                              rpc_args.encode_encoder_search(axis))

    def teensy_home(self, axis=rpc_args.AXIS_ALL):
        return self._call_rpc(RpcMethod.HOME, rpc_args.encode_home(axis))

    def teensy_sdo_read(self, axis, endpoint):
        return self._call_rpc(RpcMethod.SDO_READ,
                              rpc_args.encode_sdo_read(axis, endpoint))

    def teensy_sdo_write(self, axis, endpoint, value):
        return self._call_rpc(RpcMethod.SDO_WRITE,
                              rpc_args.encode_sdo_write(axis, endpoint, value))

    # ── ROS service handlers (existing-type subset) ───────────

    def _svc_clear_errors(self, req, res):
        ok, msg, _ = self.teensy_clear_errors()
        res.success = ok
        res.message = msg
        return res

    def _svc_reboot_odrives(self, req, res):
        ok, msg, _ = self.teensy_reboot()
        res.success = ok
        res.message = msg
        return res

    def _svc_encoder_search(self, req, res):
        ok, msg, _ = self.teensy_encoder_search()
        res.success = ok
        res.message = msg if ok else f'{msg} (encoder_search needs firmware Phase 9)'
        return res

    def _svc_home(self, req, res):
        ok, msg, _ = self.teensy_home()
        res.success = ok
        res.message = msg if ok else f'{msg} (home needs firmware Phase 9)'
        return res

    def _svc_odrive_command(self, req, res):
        cmd = req.command
        if cmd == 'clear_errors':
            ok, msg, _ = self.teensy_clear_errors()
        elif cmd == 'reboot_odrives':
            ok, msg, _ = self.teensy_reboot()
        else:
            ok, msg = False, f'Unknown command: {cmd}'
        res.success = ok
        res.message = msg
        return res

    # ═══════════════════════════════════════════════════════════
    # Ball Butler (Phase A cutover — replaces can_node bb/*)
    # ═══════════════════════════════════════════════════════════

    def _publish_bb_heartbeat(self):
        """Publish bb/heartbeat (10 Hz) from the BB fields on HeartbeatT2J.

        Mirror of can_node._publish_bb_heartbeat: same ROS message, same field
        semantics. ``connected`` is the firmware's bb_present() predicate
        (heartbeat_seen && !heartbeat_stale), so a stale BB shows connected=
        False without the bridge needing its own staleness clock.

        Suppressed until the first HeartbeatT2J arrives so the topic never
        carries a misleading all-zero / state=BOOT snapshot before the link
        is up (mirroring _publish_robot_state's "no phantom snapshot" rule).
        """
        try:
            with self._lock:
                hb = self._latest_heartbeat
            if hb is None:
                return
            seen   = bool(hb.bb_flags & _T2J_BB_FLAG_HEARTBEAT_SEEN)
            stale  = bool(hb.bb_flags & _T2J_BB_FLAG_HEARTBEAT_STALE)
            msg = BallButlerHeartbeat()
            msg.connected    = bool(seen and not stale)
            msg.ball_in_hand = bool(hb.bb_flags & _T2J_BB_FLAG_BALL_IN_HAND)
            msg.state        = int(hb.bb_state)
            msg.state_data   = int(hb.bb_state_data)
            msg.yaw_deg      = float(hb.bb_yaw_deg)
            msg.pitch_deg    = float(hb.bb_pitch_deg)
            msg.hand_pos_mm  = float(hb.bb_hand_mm)
            self.bb_heartbeat_pub.publish(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"BB heartbeat publish error: {e}",
                                    throttle_duration_sec=5.0)

    def _bb_present(self) -> bool:
        """Local view of the firmware's bb_present() — used by bb/calibrate's
        skip-if-absent semantics. False before the first HeartbeatT2J or
        whenever BB has gone stale."""
        with self._lock:
            hb = self._latest_heartbeat
        if hb is None:
            return False
        return (bool(hb.bb_flags & _T2J_BB_FLAG_HEARTBEAT_SEEN)
                and not bool(hb.bb_flags & _T2J_BB_FLAG_HEARTBEAT_STALE))

    def _svc_bb_throw(self, req, res):
        """bb/send_throw_command — typed throw via BB_THROW RPC.

        D3 / HANDOFF: the ThrowAnnouncement publisher does NOT live here.
        throw_director_node is the sole publisher of throw_announcements; it
        has the solver's actual target z and serial-chain ToF, so its
        announcement is the only accurate one. ``req.suppress_announcement``
        is dead protocol on this service — retained on the .srv for interface
        stability during the cutover; can be removed in a follow-up.
        """
        try:
            args = rpc_args.encode_bb_throw(
                req.yaw_angle_rad, req.pitch_angle_rad,
                req.throw_speed, req.throw_time)
        except Exception as e:  # noqa: BLE001 (Python-side range check failure)
            res.success = False
            res.message = f"Throw arg encode failed: {e}"
            return res
        ok, m, _ = self._call_rpc(RpcMethod.BB_THROW, args)
        res.success = ok
        res.message = "Throw command sent." if ok else f"Throw failed: {m}"
        return res

    def _svc_bb_reload(self, req, res):
        """bb/reload — BB_RELOAD RPC. ERR_BUS_DOWN surfaces as failure (the
        operator wants to know if BB isn't there for a reload attempt)."""
        ok, m, _ = self._call_rpc(RpcMethod.BB_RELOAD, b"")
        res.success = ok
        res.message = "Reload command sent." if ok else f"Reload failed: {m}"
        return res

    def _svc_bb_reset(self, req, res):
        """bb/reset — BB_RESET RPC. ERR_BUS_DOWN surfaces as failure."""
        ok, m, _ = self._call_rpc(RpcMethod.BB_RESET, b"")
        res.success = ok
        res.message = "Reset command sent." if ok else f"Reset failed: {m}"
        return res

    def _svc_bb_calibrate(self, req, res):
        """bb/calibrate — BB_CALIBRATE_LOC RPC. If BB is not present, return
        success=True silently (mirror can_node._svc_bb_calibrate). The state
        machine's HOMING phase uses this service; the silent-skip lets
        homing complete without BB attached (the original can_node
        behaviour). When BB IS present we forward the RPC's status.
        """
        if not self._bb_present():
            res.success = True
            res.message = "No BB heartbeat received — calibration skipped"
            return res
        ok, m, _ = self._call_rpc(RpcMethod.BB_CALIBRATE_LOC, b"")
        res.success = ok
        res.message = "Calibrate command sent." if ok else f"Calibrate failed: {m}"
        return res

    def _sub_vel_curr_limits(self, msg):
        """Apply leg vel/current limits over the can-bridge link.

        Mirrors the legs portion of can_node._sub_vel_curr_limits. The hand
        limits are ignored — the platform Teensy owns the hand (the firmware
        rejects hand-axis RPCs). Short-timeout RPCs keep a topic callback from
        stalling the executor on a dead link.
        """
        if msg.legs_vel_limit > 0 and msg.legs_curr_limit > 0:
            for axis in range(p.NUM_LEGS):
                ok, m, _ = self.teensy_set_vel_curr_limits(
                    axis, msg.legs_vel_limit, msg.legs_curr_limit,
                    timeout=0.2, retries=0)
                if not ok:
                    self.get_logger().error(
                        f"set_vel_curr_limits leg {axis} failed: {m}",
                        throttle_duration_sec=2.0)
                    break

    # ═══════════════════════════════════════════════════════════
    # Shutdown
    # ═══════════════════════════════════════════════════════════

    def on_shutdown(self):
        """Tear down the transport. Only stops the client if we created it."""
        self.get_logger().info("Shutting down TeensyBridgeNode...")
        # Stop the setpoint thread first (so no frame is sent mid-teardown), then
        # drop mpc_active on the WIRE (not just the local flag) BEFORE stopping the
        # client, so the final J→T heartbeat carries flags=0 — even for an injected
        # client whose heartbeat thread keeps running after on_shutdown. Using
        # _set_mpc_active (the sole flag writer) clears client._heartbeat_flags.
        self._sp_stop.set()
        if self._sp_thread is not None and self._sp_thread.is_alive():
            self._sp_thread.join(timeout=1.0)
        try:
            self._set_mpc_active(False)
        except Exception:  # noqa: BLE001 — best-effort during teardown
            self._mpc_active = False
        if self._sp_source is not None:
            try:
                self._sp_source.close()
            except Exception:  # noqa: BLE001
                pass
        try:
            self._tod.close()
            self._rpc_server.close()
            self._rpc.close()
        except Exception:  # noqa: BLE001
            pass
        if self._own_client:
            try:
                self._client.stop()
            except Exception:  # noqa: BLE001
                pass


def main(args=None):
    rclpy.init(args=args)
    node = TeensyBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
