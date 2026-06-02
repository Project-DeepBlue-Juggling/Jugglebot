"""Teensy leg-bridge ROS 2 node — UDP-sourced mirror of ``can_node.py``.

This is the Phase-10b side-by-side bridge: it exposes the same observable
surface as :mod:`jugglebot.can_node` (robot state, hand telemetry, link/fault
health) but sources everything from the leg-bridge Teensy over the dedicated
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
    HandTelemetryMessage,
    MotorStateSingle,
    RobotState,
)
from geometry_msgs.msg import Quaternion
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from controller.teensy_link import (
    TeensyLinkClient,
    RpcClient,
    RpcServer,
    TimeOfDayServer,
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
from controller.teensy_link.fault_logic import LinkLossLatch


# ── Constants ──────────────────────────────────────────────────
# Mirror of can_node._HEARTBEAT_TIMEOUT_S: the leg-bridge link is declared lost
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


class TeensyBridgeNode(Node):
    """ROS 2 node bridging the leg-bridge Teensy UDP link to ``/teensy/*``.

    Args:
        client: An optional already-constructed :class:`TeensyLinkClient`. When
            ``None`` (production), one is built from the node's parameters and
            started. Tests inject a loopback client paired with ``FakeTeensy``.
            When injected, the node does NOT take ownership — it will not stop
            the client on shutdown (the test fixture owns its lifecycle).
    """

    def __init__(self, client: TeensyLinkClient | None = None):
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
        # This is the single startup heartbeat path. flags=0 ⇒ mpc_active clear.
        # Commit 3 introduces _mpc_active + _update_heartbeat_flags(); until then
        # the flag is structurally pinned to 0.
        self._mpc_active = False
        self._client.start_heartbeat(hz=float(p.HEARTBEAT_HZ), flags=0)

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

        # ── Timers (publish on the executor thread) ────────────
        self.create_timer(0.01, self._publish_robot_state)     # 100 Hz (telem rate)
        self.create_timer(0.01, self._publish_hand_telemetry)  # 100 Hz
        self.create_timer(0.1, self._publish_link_status)      # 10 Hz (heartbeat rate)
        self.create_timer(1.0, self._publish_profile)          # 1 Hz (profile rate)
        self.create_timer(1.0, self._health_check)             # 1 Hz link watchdog

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
        frame for that axis. Fields the leg-bridge link does not carry
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

            # Typed fault flags. The Teensy owns the fault state machine now, so
            # these are derived from its HeartbeatT2J.fault_state / bus health
            # plus the per-axis diagnostics (undervoltage). Mirrors the meaning
            # of can_node's motors.fatal_error / fatal_can_error / undervoltage.
            fault_state = int(hb.fault_state) if hb is not None else 0
            bus2_health = int(hb.bus2_health) if hb is not None else 0
            msg.has_fatal_odrive_error = (fault_state == int(FaultState.ODRIVE_FATAL))
            msg.has_fatal_can_error = (
                fault_state == int(FaultState.CAN_BUS_DOWN)
                or bus2_health == int(BusHealth.BUS_OFF))
            # Undervoltage: any axis reports the UV bit in active errors or as
            # its disarm reason (matches can_node's undervoltage tracking).
            msg.has_undervoltage = any(
                (s.active_errors & _ERR_DC_BUS_UNDER_VOLTAGE)
                or (s.disarm_reason == _ERR_DC_BUS_UNDER_VOLTAGE)
                for s in states)
            # firmware_validated: not carried on the leg-bridge link in 10b
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
        fields (pos_cmd/vel_ff_cmd/tor_ff_cmd) are not echoed on the leg-bridge
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
            msg.hardware_id = 'leg_bridge_teensy'

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
            msg.hardware_id = 'leg_bridge_teensy'
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
    # Shutdown
    # ═══════════════════════════════════════════════════════════

    def on_shutdown(self):
        """Tear down the transport. Only stops the client if we created it."""
        self.get_logger().info("Shutting down TeensyBridgeNode...")
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
