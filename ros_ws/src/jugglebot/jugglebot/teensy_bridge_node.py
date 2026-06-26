"""Teensy can-bridge ROS 2 node — UDP-sourced mirror of ``can_node.py``.

This is the can-bridge successor to :mod:`jugglebot.can_node`: it exposes the
same observable surface (robot state, hand telemetry, link/fault health) but
sources everything from the can-bridge Teensy over the dedicated UDP link
(``controller/teensy_link``) instead of socketcan. ``can_node`` is out of the
production launch (its USB-CAN path is dead), so the bridge now owns the
**production topic/service names** directly (legs/hand promoted off the
side-by-side ``/teensy/*`` namespace in Phase 11 / U4; BB + cone in Phase A).

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

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from jugglebot_interfaces.msg import (
    BallButlerHeartbeat,
    CatchEvent,
    CatchingConeHeartbeat,
    HandTelemetryMessage,
    MotorStateSingle,
    RobotState,
    SetMotorVelCurrLimitsMessage,
)
from jugglebot_interfaces.srv import ODriveCommandService
from jugglebot_interfaces.action import BallButlerThrowCmd
from geometry_msgs.msg import Quaternion
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
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
    ConeFrame,
    CmdResultFrame,
    BbAxisEstimates,
)
from controller.teensy_link import protocol as p
from controller.teensy_link import rpc_args
from controller.teensy_link.fault_logic import LinkLossLatch
from controller.teensy_link.setpoint_pump import SetpointPump
from controller.teensy_link.encoder_search import (
    EncoderSearch, AxisStatus, AXIS_STATE_ENCODER_INDEX_SEARCH,
)
from controller.teensy_link.homing import (
    HomingMonitor, AxisStatus as HomingAxisStatus,
)
from controller.teensy_link.activate import (
    ActivateMonitor, AxisStatus as ActivateAxisStatus,
)
from jugglebot.can import catching_cone
import jugglebot.hardware_config as hw
import jugglebot.protocol_config as proto


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


class _MpcCommandSetpointSource:
    """SUB-only connection to the MPC command PUB (tcp://127.0.0.1:5557).

    **Phase 11 / U4 — the α→β switch.** Previously this read motor_guard's
    *already-interpolated* 500 Hz telemetry on :5556 (the α relay); it now reads
    the **40 Hz MPC command stream** on :5557 (``ipc.MPC_COMMAND_ADDR``,
    ``ipc.TOPIC_MPC_CMD``) — the same ``make_mpc_command`` dicts motor_guard
    consumes — so the Teensy does the 500 Hz interpolation (β path). motor_guard
    leaves the leg path; its :5556 output simply goes unconsumed.

    Subscribes to the ``mpccmd`` topic specifically (not ``b''``): :5557 also
    carries the HardwarePlant fallback-enable message, which is not a setpoint.

    Deliberately does NOT reuse ``jugglebot.motion.ipc.BridgeIPC`` (that BINDS the
    command PUB on :5555, which ``motion_bridge_node`` already binds — two binds
    on one address fail) and does NOT import ``ipc`` at module scope (``ipc``
    imports zmq/msgpack eagerly; the address/topic literals below mirror
    ``ipc.MPC_COMMAND_ADDR`` / ``ipc.TOPIC_MPC_CMD``). Mirrors BridgeIPC's SUB
    tuning (RCVHWM=2 + drain-to-latest) and the msgpack wire format
    (``[topic, msgpack(dict)]``). zmq/msgpack are imported lazily so a read-only
    / setpoint-disabled bridge has no hard dependency on them.
    """

    def __init__(self, addr: str = 'tcp://127.0.0.1:5557',
                 topic: bytes = b'mpccmd'):   # = ipc.MPC_COMMAND_ADDR / TOPIC_MPC_CMD
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
        self._sub.setsockopt(zmq.SUBSCRIBE, topic)         # mpc_cmd frames only
        self._sub.setsockopt(zmq.RCVTIMEO, 0)              # non-blocking

    def recv_latest(self) -> dict | None:
        """Drain the SUB and return the latest MPC command dict, or None."""
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
    """ROS 2 node bridging the can-bridge Teensy UDP link to production topics.

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
        # Phase 9a: which axes the encoder_search service runs index
        # search on. Default = all legs; set to e.g. [0] for the standalone-leg
        # bench rig (node 0 = axis 0).
        self.declare_parameter('encoder_search_axes', list(range(p.NUM_LEGS)))
        # Phase 9b: which axes the home service homes (sequentially — the
        # firmware homes one axis at a time). Default = all legs; set to e.g. [0]
        # for the standalone-leg bench rig (node 0 = axis 0).
        self.declare_parameter('home_axes', list(range(p.NUM_LEGS)))
        # Phase 11 U5: which axes the configure + activate services act on.
        # configure applies gains + vel/curr limits + POSITION/PASSTHROUGH (the
        # β-path _setup_odrives); activate fires the TRAP_TRAJ move to the active
        # pose. Default = all legs; set to e.g. [0] for the standalone-leg bench rig.
        self.declare_parameter('configure_axes', list(range(p.NUM_LEGS)))
        self.declare_parameter('activate_axes', list(range(p.NUM_LEGS)))

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

        # Catching cone state (phase-10b cone uplink). Catch events are
        # DISCRETE — every one is queued and published exactly once (the
        # stash-latest pattern above would drop impacts that arrive between
        # timer ticks); heartbeats are state — latest wins. The connected
        # gate mirrors can_node: a cone heartbeat within the CAN timeout.
        self._cone_catch_queue: list = []   # [(decoded CatchEvent, host_arrival_us)]
        self._latest_cone_hb = catching_cone.ConeHeartbeat()
        self._cone_hb_received = False
        self._cone_last_hb_mono = 0.0
        self._cone_hb_timeout_s = proto.CC_HEARTBEAT_TIMEOUT_MS / 1000.0

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

        # RX-thread queues MUST exist before any subscribe() below: the client's
        # RX thread is already live, so a frame arriving between subscribe() and
        # a later __init__ line would hit an unset attribute (startup race).
        self._bb_est_queue = []   # list of BbAxisEstimates, drained by timer

        # ── BB loud command-outcome channel (Phase 2: CMD_RESULT relay) ──
        # One outstanding throw at a time (firmware is serialized → no correlation
        # token). The bb/throw action's execute_callback waits on _bb_throw_event;
        # the RX-thread _on_cmd_result handler stores the firmware outcome and sets
        # it. _bb_throw_active gates a second concurrent goal (rejected in
        # goal_callback). Guarded by _bb_throw_lock; the event lives outside it.
        self._bb_throw_lock = threading.Lock()
        self._bb_throw_event = threading.Event()
        self._bb_throw_active = False
        self._bb_throw_result = None   # (outcome:int, detail0:int, detail1:int)

        # ── Subscriptions to T→J streams (RX-thread callbacks) ──
        self._client.subscribe(int(MsgType.HEARTBEAT_T2J), self._on_heartbeat_t2j)
        self._client.subscribe(int(MsgType.TELEMETRY), self._on_telemetry)
        self._client.subscribe(int(MsgType.DIAGNOSTIC), self._on_diagnostic)
        self._client.subscribe(int(MsgType.PROFILE), self._on_profile)
        self._client.subscribe(int(MsgType.CONE_FRAME), self._on_cone_frame)
        self._client.subscribe(int(MsgType.BB_AXIS_ESTIMATES), self._on_bb_estimates)
        self._client.subscribe(int(MsgType.CMD_RESULT), self._on_cmd_result)

        # ── Publishers (production names — Phase 11 / U4 leg-side cutover) ──
        # Promoted from the /teensy/* namespace to the production topic names
        # can_node used: with USB-CAN gone (can_node out of the launch), the
        # dual-publisher risk that drove the /teensy/* namespacing (handoff D1)
        # is moot, and the GUI / orchestrator / consumers subscribe to these
        # production names — so the rename RECONNECTS them to the bridge.
        self.robot_state_pub = self.create_publisher(
            RobotState, 'robot_state', 10)
        self.hand_telemetry_pub = self.create_publisher(
            HandTelemetryMessage, 'hand_telemetry', 10)
        self.link_status_pub = self.create_publisher(
            DiagnosticStatus, 'link_status', 10)
        self.profile_pub = self.create_publisher(
            DiagnosticStatus, 'profile', 10)

        # ── Ball Butler (Phase A cutover, production names) ────
        # Intentional naming deviation from D1's "all under /teensy/*"
        # convention: with USB-CAN removed, the dual-publisher risk D1 was
        # preventing is moot (can_node is gone for BB). The bridge inherits
        # the production names so the GUI / orchestrator / mocap_node /
        # throw_director see no name change across the cutover. (The leg/hand
        # topics + services followed onto production names in Phase 11 / U4.)
        self.bb_heartbeat_pub = self.create_publisher(
            BallButlerHeartbeat, 'bb/heartbeat', 10)

        # ── Catching cone (phase-10b cone uplink, production names) ────
        # Same naming rationale as bb/*: can_node's cone path is dead — the
        # cone lives on the can-bridge's CAN2, which USB-CAN never sees — so
        # the bridge inherits the production names and the existing consumers
        # (catch_correlation_node, analysis tooling) see no change. CONE_FRAME
        # relays carry the raw CAN payloads; decode reuses the same
        # jugglebot.can.catching_cone helpers can_node used.
        self.catch_event_pub = self.create_publisher(
            CatchEvent, 'cone/catch_event', 10)
        self.cone_heartbeat_pub = self.create_publisher(
            CatchingConeHeartbeat, 'cone/heartbeat', 10)

        # Ball Butler ODrive diagnostics (CAN1 nodes 7=pitch, 8=hand). The bridge
        # emits these as DIAGNOSTIC frames with axis_id 7/8, stashed in
        # self._latest_diag[7|8] by _on_diagnostic; republished as a flat array.
        # Layout: [pitch_fet, pitch_motor, pitch_iq, pitch_state,
        #          hand_fet,  hand_motor,  hand_iq,  hand_state]  (degC, degC, A, enum).
        self.bb_odrive_pub = self.create_publisher(
            Float32MultiArray, 'bb/odrive_diag', 10)

        # Ball Butler high-rate pitch/hand encoder estimates (BB_AXIS_ESTIMATES,
        # CAN1 nodes 7/8) for during-throw diagnostics: launch angle vs commanded
        # pitch, hand launch speed vs commanded. Published as JointState
        # (name=[bb_pitch, bb_hand], position=rev, velocity=rev/s), stamped with
        # the bridge wall-clock at sample time. Every sample is queued in
        # _on_bb_estimates and drained on the executor thread by
        # _publish_bb_axis_estimates (publishing off the RX thread, per the
        # other RX callbacks' contract).
        self.bb_estimates_pub = self.create_publisher(
            JointState, 'bb/axis_estimates', 50)
        # (_bb_est_queue is initialized above, before the subscribe block, to
        # avoid a startup race with the already-live RX thread.)

        # ── RPC service surface (production names — Phase 11 / U4) ──
        # ODrive control issued over the can-bridge link via RpcClient. The
        # can-bridge owns legs 0-5 only — the hand is the platform Teensy's, and
        # the firmware rejects hand-axis RPCs — so these target legs/broadcast.
        # Promoted from /teensy/* to the production names can_node served
        # (encoder_search, odrive_command, set_motor_vel_curr_limits) so the
        # orchestrator's existing service clients reach the bridge; clear_errors,
        # reboot_odrives and home are new bridge ops, named bare for consistency.
        # Services using EXISTING ROS types are wired here; the arg-bearing
        # per-axis ops (set_axis_state, set_controller_mode, per-axis gains,
        # set_absolute_position, sdo_read/write) are tested node methods pending
        # new jugglebot_interfaces .srv types (handoff D10).
        self.create_service(Trigger, 'clear_errors', self._svc_clear_errors)
        self.create_service(Trigger, 'reboot_odrives', self._svc_reboot_odrives)
        self.create_service(Trigger, 'encoder_search', self._svc_encoder_search)
        self.create_service(Trigger, 'home', self._svc_home)
        self.create_service(Trigger, 'configure', self._svc_configure)
        self.create_service(Trigger, 'activate', self._svc_activate)
        self.create_service(ODriveCommandService, 'odrive_command',
                            self._svc_odrive_command)
        self.create_subscription(
            SetMotorVelCurrLimitsMessage, 'set_motor_vel_curr_limits',
            self._sub_vel_curr_limits, 10)

        # ── Ball Butler services (production names, Phase A cutover) ────
        # The bb/* services formerly served by can_node. The firmware
        # gates each TX on bb_present(); we translate the ERR_BUS_DOWN that
        # gate produces into a silent-success for bb/calibrate to preserve
        # can_node._svc_bb_calibrate's HOMING semantics (allows homing to
        # complete without BB attached). The other two propagate the
        # error so an operator sees a failed reload/reset.
        self.create_service(Trigger, 'bb/reload',    self._svc_bb_reload)
        self.create_service(Trigger, 'bb/reset',     self._svc_bb_reset)
        self.create_service(Trigger, 'bb/calibrate', self._svc_bb_calibrate)

        # ── Ball Butler throw ACTION (Phase 2 — replaces bb/send_throw_command) ──
        # The action awaits the firmware's terminal outcome (relayed back as a
        # CMD_RESULT CAN frame) instead of the bridge-side "frame queued" ack the
        # service returned. Its callbacks share a ReentrantCallbackGroup so a
        # result-wait in execute_callback never stalls the 100 Hz telemetry timers
        # (which run in the default group on other MultiThreadedExecutor threads),
        # and a second goal's goal_callback can run (and be rejected) while a throw
        # is still outstanding.
        self._bb_throw_cbgroup = ReentrantCallbackGroup()
        self._bb_throw_action = ActionServer(
            self, BallButlerThrowCmd, 'bb/throw',
            execute_callback=self._bb_throw_execute,
            goal_callback=self._bb_throw_goal,
            cancel_callback=self._bb_throw_cancel,
            callback_group=self._bb_throw_cbgroup)

        # ── Timers (publish on the executor thread) ────────────
        self.create_timer(0.01, self._publish_robot_state)     # 100 Hz (telem rate)
        self.create_timer(0.01, self._publish_hand_telemetry)  # 100 Hz
        self.create_timer(0.1, self._publish_link_status)      # 10 Hz (heartbeat rate)
        self.create_timer(0.1, self._publish_bb_heartbeat)     # 10 Hz BB (matches CAN1 rate)
        self.create_timer(0.01, self._drain_cone_catch_events) # 100 Hz cone events (cheap when idle)
        self.create_timer(0.1, self._publish_cone_heartbeat)   # 10 Hz cone (matches CAN2 rate)
        self.create_timer(0.01, self._publish_bb_axis_estimates)  # 100 Hz drain (BB pitch/hand est.)
        self.create_timer(0.5, self._publish_bb_odrive)        # 2 Hz BB ODrive diag (CAN1 ~1 Hz src)
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
            mm_to_rev=hw.GEOM_MM_TO_REV,
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

    def _on_cone_frame(self, msg_type, seq, payload, addr):
        # Decode the relayed CAN payload here (cheap struct unpack) so the
        # drain timer publishes ready-made events; anything malformed or
        # unrecognised (future cone frame types) is dropped silently, matching
        # the other RX callbacks' never-kill-the-RX-thread contract.
        try:
            cf = ConeFrame.unpack(payload)
            data = bytes(cf.data[:cf.dlc])
            if cf.can_id == catching_cone.CATCH_EVENT_ID:
                evt = catching_cone.CatchEvent.from_can_frame(data)
                arrival_us = int(time.time() * 1e6)
                with self._lock:
                    self._cone_catch_queue.append((evt, arrival_us))
            elif cf.can_id == catching_cone.HEARTBEAT_ID:
                hb = catching_cone.ConeHeartbeat.from_can_frame(data)
                with self._lock:
                    self._latest_cone_hb = hb
                    self._cone_hb_received = True
                    self._cone_last_hb_mono = time.monotonic()
        except Exception:  # noqa: BLE001
            return

    def _on_bb_estimates(self, msg_type, seq, payload, addr):
        # RX-thread callback: decode + queue every sample. The drain timer
        # publishes on the executor thread (never publish from the RX thread,
        # matching the other RX callbacks' contract). Malformed frames dropped.
        try:
            e = BbAxisEstimates.unpack(payload)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            q = self._bb_est_queue
            q.append(e)
            if len(q) > 4000:        # ~40 s at 100 Hz — bound if the drain stalls
                del q[:len(q) - 4000]

    def _on_cmd_result(self, msg_type, seq, payload, addr):
        # RX-thread callback (Phase-2 loud channel). Decode the relayed CMD_RESULT
        # CAN frame; if it's a THROW outcome and a throw goal is outstanding, hand
        # the (outcome, detail0, detail1) to the waiting bb/throw execute_callback
        # and wake it. Never publish / never block here (RX-thread contract); any
        # malformed or unsolicited frame is dropped silently.
        try:
            cf = CmdResultFrame.unpack(payload)
            data = bytes(cf.data[:cf.dlc])
            if len(data) < 6:
                return
            cmd_type = data[0]
            outcome = data[1]
            detail0 = int.from_bytes(data[2:4], 'little', signed=True)
            detail1 = int.from_bytes(data[4:6], 'little', signed=True)
        except Exception:  # noqa: BLE001
            return
        if cmd_type != int(proto.BallButlerCommandType.THROW):
            return  # only the throw consumes CMD_RESULT today
        with self._bb_throw_lock:
            if not self._bb_throw_active:
                return  # no waiter — stale/duplicate frame, drop
            self._bb_throw_result = (outcome, detail0, detail1)
        self._bb_throw_event.set()

    def _publish_bb_axis_estimates(self):
        """Drain queued BB pitch/hand estimates → /bb/axis_estimates (JointState).

        Each sample is stamped with the bridge wall-clock at sample time
        (e.t_bridge_us, time-synced to the Jetson), so per-sample timing is
        correct regardless of when this drain runs. position=rev, velocity=rev/s;
        name=[bb_pitch, bb_hand]. Pitch deg = 90 + 360*pos_rev (PitchAxis.h);
        ball speed = vel_rps * 2*pi*HAND_SPOOL_RADIUS_M.
        """
        with self._lock:
            batch = self._bb_est_queue
            self._bb_est_queue = []
        for e in batch:
            js = JointState()
            t_us = int(e.t_bridge_us)
            js.header.stamp.sec = t_us // 1_000_000
            js.header.stamp.nanosec = (t_us % 1_000_000) * 1000
            js.name = ['bb_pitch', 'bb_hand']
            js.position = [float(e.pitch_pos_rev), float(e.hand_pos_rev)]
            js.velocity = [float(e.pitch_vel_rps), float(e.hand_vel_rps)]
            self.bb_estimates_pub.publish(js)

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
        """Publish robot_state, mirroring can_node._publish_robot_state.

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
            # robot_state under-report a real per-leg fault for the same hardware
            # state. So we also OR in the
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
        """Publish hand_telemetry from axis 6 of the Telemetry frame.

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
        and the bridge SURFACES it on link_status for the operator /
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
                        "on link_status (bridge_stow_pending=1).")
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
            self._sp_source = _MpcCommandSetpointSource()
        # Set mpc_active BEFORE starting the thread so the first tick it drains
        # is not silently dropped by the mpc_active gate (and so the Teensy is
        # told we are driving before any setpoint can flow).
        self._set_mpc_active(True)
        self._sp_stop.clear()
        self._sp_thread = threading.Thread(
            target=self._setpoint_loop, name="teensy_bridge_setpoint", daemon=True)
        self._sp_thread.start()

    def _setpoint_loop(self):
        """Dedicated thread: drain the 40 Hz MPC command → pack β knots → gate → send."""
        while not self._sp_stop.is_set():
            try:
                cmd = self._sp_source.recv_latest()
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"Setpoint source error: {e}",
                                        throttle_duration_sec=5.0)
                cmd = None
            if cmd is not None:
                self._process_setpoint(cmd)
            else:
                # ~1 kHz idle poll (the MPC command stream arrives at 40 Hz; the
                # Teensy interpolates to 500 Hz from the β knots).
                self._sp_stop.wait(0.001)

    def _process_setpoint(self, cmd: dict):
        """Pack one 40 Hz MPC command into a β-knot Setpoint frame, gate it, send.

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
        sp, reason = self._sp_pump.build(cmd, t_origin_us)
        if reason is not None:
            self.get_logger().error(
                f"Setpoint REJECTED (not sent): {reason}",
                throttle_duration_sec=1.0)
            return
        if sp is None:
            return  # feedback-only telemetry — nothing to send
        self._client.send_stream(int(MsgType.SETPOINT), sp.pack())

    def _publish_link_status(self):
        """Publish link_status as a DiagnosticStatus.

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

    def _publish_bb_odrive(self):
        """Publish /bb/odrive_diag: BB pitch(7)/hand(8) ODrive temps, current, state.

        The can-bridge sends these on CAN1 as DIAGNOSTIC frames with axis_id 7/8,
        which _on_diagnostic stashes in self._latest_diag. Flat-array layout (8
        floats): [pitch_fet, pitch_motor, pitch_iq_meas, pitch_state,
                  hand_fet,  hand_motor,  hand_iq_meas,  hand_state]. Suppressed
        until at least one BB ODrive frame has arrived (avoids phantom zeros).
        """
        with self._lock:
            dp = self._latest_diag.get(7)
            dh = self._latest_diag.get(8)
        if dp is None and dh is None:
            return
        def quad(d):
            if d is None:
                return [float('nan')] * 4
            return [float(d.temp_fet), float(d.temp_motor),
                    float(d.iq_measured), float(d.axis_state)]
        msg = Float32MultiArray()
        msg.data = quad(dp) + quad(dh)
        self.bb_odrive_pub.publish(msg)

    def _publish_profile(self):
        """Publish profile (firmware instrumentation) as DiagnosticStatus."""
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

    def teensy_activate(self, axis=rpc_args.AXIS_ALL):
        return self._call_rpc(RpcMethod.ACTIVATE, rpc_args.encode_activate(axis))

    def teensy_sdo_read(self, axis, endpoint):
        return self._call_rpc(RpcMethod.SDO_READ,
                              rpc_args.encode_sdo_read(axis, endpoint))

    def teensy_sdo_write(self, axis, endpoint, value):
        return self._call_rpc(RpcMethod.SDO_WRITE,
                              rpc_args.encode_sdo_write(axis, endpoint, value))

    # ── Encoder index search (Phase 9a, Jetson-side orchestration) ──
    # The firmware ENCODER_SEARCH RPC is stubbed (ERR_NOT_IMPL); encoder index
    # search is an ODrive-autonomous axis state, so we orchestrate it from here
    # over the implemented SET_AXIS_STATE primitive + the telemetry/diagnostic
    # cache. The pure sequencing lives in controller/teensy_link/encoder_search.py
    # (unit-tested); this method is the I/O loop around it. Homing (Phase 9b) is
    # the part that must live in firmware (no per-leg motion RPC).

    def _encoder_axis_status(self, axes):
        """Build per-axis ``AxisStatus`` for the search state machine from the
        latest telemetry (pos → finite) + diagnostic (axis_state, active_errors)
        cache. An axis with no diagnostic yet is omitted — the state machine
        treats a missing axis as "no fresh status", aging it toward its timeout.
        """
        out = {}
        with self._lock:
            telem = self._latest_telemetry
            diag = dict(self._latest_diag)
        if telem is None:
            return out
        for axis in axes:
            d = diag.get(int(axis))
            if d is None:
                continue
            out[int(axis)] = AxisStatus(
                axis_state=int(d.axis_state),
                pos_finite=math.isfinite(float(telem.pos_rev[int(axis)])),
                active_errors=int(d.active_errors))
        return out

    def _run_encoder_search(self, axes, *, poll_dt=0.05):
        """Drive encoder index search on ``axes`` over the can-bridge link.

        Synchronous: blocks the calling (executor) thread until the search
        finishes — acceptable for a deliberate cold-start op; the RX + heartbeat
        threads keep the link alive and the telemetry cache fresh throughout, and
        the heartbeat stays ``mpc_active=0`` (no setpoint output). Returns
        ``(ok, message)``.
        """
        axes = [int(a) for a in axes]
        if not axes:
            return False, "no axes configured for encoder search"
        es = EncoderSearch(axes, timeout_s=2.0 * hw.JB_OP_ENCODER_SEARCH_TIMEOUT_S)
        # Belt-and-suspenders wall-clock bound; the state machine self-terminates
        # via its own per-axis timeout × (retries + 1) well inside this.
        hard_deadline = time.monotonic() + es.timeout_s * (es.max_retries + 1) + 5.0
        self.get_logger().info(f"encoder search: starting on axes {axes}")
        while True:
            now = time.monotonic()
            res = es.step(now, self._encoder_axis_status(axes))
            for axis in res.clear_errors:
                self.teensy_clear_errors(axis)
            for axis in res.set_search:
                self.teensy_set_axis_state(axis, AXIS_STATE_ENCODER_INDEX_SEARCH)
            if res.done:
                break
            if now > hard_deadline:
                self.get_logger().error("encoder search: hard deadline exceeded")
                break
            time.sleep(poll_dt)
        if es.done and not es.failed:
            msg = f"encoder search complete on axes {es.succeeded}"
            self.get_logger().info(msg)
            return True, msg
        parts = [f"axis {a}: {r}" for a, r in es.failed.items()]
        msg = "encoder search FAILED — " + "; ".join(parts)
        if es.succeeded:
            msg += f" (succeeded: {es.succeeded})"
        self.get_logger().error(msg)
        return False, msg

    # ── Homing (Phase 9b — firmware move, Jetson-side observation) ──
    # Unlike encoder search, the homing *move* runs autonomously in the can-bridge
    # HOME handler (no per-leg motion RPC; the velocity-limited move-to-hardstop
    # must live in firmware). The Jetson fires HOME (fire-and-monitor) and watches
    # the telemetry + diagnostic cache for completion via HomingMonitor (pure,
    # unit-tested). The firmware homes one axis at a time, so axes are homed
    # sequentially.

    def _homing_axis_status(self, axis):
        """Build the :class:`HomingAxisStatus` for ``axis`` from the latest
        telemetry (pos) + diagnostic (axis_state, active_errors) cache, or None
        if no diagnostic has arrived yet."""
        with self._lock:
            telem = self._latest_telemetry
            d = self._latest_diag.get(int(axis))
        if telem is None or d is None:
            return None
        return HomingAxisStatus(
            axis_state=int(d.axis_state),
            pos_rev=float(telem.pos_rev[int(axis)]),
            active_errors=int(d.active_errors))

    def _run_home(self, axes, *, poll_dt=0.05):
        """Home ``axes`` over the can-bridge link, one axis at a time.

        Synchronous: blocks the calling (executor) thread until homing finishes —
        acceptable for a deliberate cold-start op; the RX + heartbeat threads keep
        the link alive and the telemetry cache fresh throughout, and the heartbeat
        stays ``mpc_active=0`` (no setpoint output). Returns ``(ok, message)``.
        """
        axes = [int(a) for a in axes]
        if not axes:
            return False, "no axes configured for homing"
        home_ref = abs(float(hw.HOMING_LEG_ABS_POS_REV))
        # Firmware per-axis hard timeout (Homing::MOTOR_TIMEOUT_S) + margin.
        timeout_s = float(hw.HOMING_MOTOR_TIMEOUT_S) + 5.0
        succeeded, failed = [], {}
        self.get_logger().info(f"homing: starting on axes {axes} (sequential)")
        for axis in axes:
            mon = HomingMonitor([axis], home_ref_rev=home_ref, timeout_s=timeout_s)
            hard_deadline = time.monotonic() + timeout_s + 5.0
            while True:
                now = time.monotonic()
                st = self._homing_axis_status(axis)
                res = mon.step(now, {axis: st} if st is not None else {})
                for ax in res.set_home:
                    ok, msg, _ = self.teensy_home(ax)
                    if not ok:
                        # Firmware rejected the move (busy / bus down / bad axis).
                        failed[axis] = f"HOME rejected: {msg}"
                        mon = None
                        break
                if mon is None or res.done:
                    break
                if now > hard_deadline:
                    self.get_logger().error(f"homing: hard deadline on axis {axis}")
                    failed[axis] = "hard deadline exceeded"
                    mon = None
                    break
                time.sleep(poll_dt)
            if mon is not None:
                succeeded.extend(mon.succeeded)
                failed.update(mon.failed)
            if axis in failed:
                # Abort the sequence on the first failure (matches can_node
                # _home_robot, which returns False on any motor's failure).
                break
        if failed:
            parts = [f"axis {a}: {r}" for a, r in failed.items()]
            msg = "homing FAILED — " + "; ".join(parts)
            if succeeded:
                msg += f" (succeeded: {succeeded})"
            self.get_logger().error(msg)
            return False, msg
        msg = f"homing complete on axes {succeeded}"
        self.get_logger().info(msg)
        return True, msg

    # ── Configure + activate (Phase 11 U5 — β-path cold-start) ──
    # The β path has only one leg-motion path (the gated 40 Hz setpoint stream),
    # so the can_node `_setup_odrives_steps` (gains/limits/mode) and
    # `_gentle_move_steps` (move to active pose) have no equivalent until here.
    # `_run_configure` is the pure-config _setup_odrives analogue; `_run_activate`
    # fires the firmware TRAP_TRAJ activate op + observes it. `/home` calls
    # `_run_configure` at completion (the operator's "set after every homing").

    def _run_configure(self, axes):
        """Apply the β-path cold-start config to ``axes`` (the _setup_odrives
        analogue): per-leg position/velocity gains + vel/curr limits +
        POSITION/PASSTHROUGH controller mode. Idempotent and motion-free — it does
        NOT change axis_state or command a position (that is /activate's job), so it
        is safe to run after homing (legs IDLE at the hardstop) AND again after
        activate (legs CLOSED_LOOP holding the active pose, switching TRAP_TRAJ →
        PASSTHROUGH for the interp). Returns ``(ok, message)``.
        """
        axes = [int(a) for a in axes]
        if not axes:
            return False, "no axes configured for configure"
        ctrl = proto.ODRIVE_CONTROL_MODES['POSITION']
        inp = proto.ODRIVE_INPUT_MODES['PASSTHROUGH']
        failed = []
        self.get_logger().info(f"configure: applying gains/limits/PASSTHROUGH on axes {axes}")
        for axis in axes:
            for name, (ok, m, _) in (
                ("pos_gain",
                 self.teensy_set_pos_gain(axis, hw.ODRIVE_LEG_POS_GAINS[axis])),
                ("vel_gains",
                 self.teensy_set_vel_gains(axis, hw.ODRIVE_LEG_VEL_GAINS[axis],
                                           hw.ODRIVE_LEG_VEL_INT_GAINS[axis])),
                ("vel_curr_limits",
                 self.teensy_set_vel_curr_limits(axis, hw.ODRIVE_LEG_VEL_LIMIT_RPS,
                                                 hw.ODRIVE_LEG_CURR_LIMIT_A)),
                ("controller_mode",
                 self.teensy_set_controller_mode(axis, ctrl, inp)),
            ):
                if not ok:
                    failed.append(f"axis {axis} {name}: {m}")
        if failed:
            msg = "configure FAILED — " + "; ".join(failed)
            self.get_logger().error(msg)
            return False, msg
        msg = f"configure complete on axes {axes}"
        self.get_logger().info(msg)
        return True, msg

    def _activate_axis_status(self, axes):
        """Build per-axis ``ActivateAxisStatus`` for the activate observer from the
        latest telemetry (pos/vel) + diagnostic (axis_state, active_errors) cache.
        An axis with no diagnostic yet is omitted (aged toward its timeout)."""
        out = {}
        with self._lock:
            telem = self._latest_telemetry
            diag = dict(self._latest_diag)
        if telem is None:
            return out
        for axis in axes:
            d = diag.get(int(axis))
            if d is None:
                continue
            out[int(axis)] = ActivateAxisStatus(
                axis_state=int(d.axis_state),
                pos_rev=float(telem.pos_rev[int(axis)]),
                vel_rps=float(telem.vel_rps[int(axis)]),
                active_errors=int(d.active_errors))
        return out

    def _run_activate(self, axes, *, poll_dt=0.05):
        """Fire the firmware ACTIVATE (TRAP_TRAJ move to the active pose) and
        observe it to completion. A single configured axis fires that leg; any
        larger set fires ``AXIS_ALL`` (every PRESENT leg, parallel even-rise). The
        ActivateMonitor watches each leg reach its active-pose target + settle.

        Synchronous: blocks the calling (executor) thread for the ~seconds the move
        takes; the RX + heartbeat threads keep the link + telemetry cache alive and
        the heartbeat stays ``mpc_active=0`` (no setpoint output). Returns
        ``(ok, message)``.

        Precondition: a prior ``/configure`` (gains/limits/mode). Without it the
        legs run on flash-default gains and the move fails safe (never reaches the
        target → ActivateMonitor timeout → FAILED).
        """
        axes = [int(a) for a in axes]
        if not axes:
            return False, "no axes configured for activate"
        targets = {a: float(hw.JB_OP_ACTIVATE_POSITION_REVS[a]) for a in axes}
        # Single configured axis → that leg only; otherwise AXIS_ALL (all present
        # legs in parallel — the platform rises straight up, no tilt).
        fire_axis = axes[0] if len(axes) == 1 else rpc_args.AXIS_ALL
        # Footgun guard: a partial multi-leg subset fires AXIS_ALL (every PRESENT
        # leg), so legs NOT in activate_axes are still driven but unobserved. Only
        # [single leg] or the full leg set are coherent; warn on anything else.
        if len(axes) > 1 and set(axes) != set(range(p.NUM_LEGS)):
            self.get_logger().warning(
                f"activate_axes={axes} is a partial subset — ACTIVATE(AXIS_ALL) "
                f"will move ALL present legs, but only {axes} are observed. Use a "
                f"single leg or the full leg set.")
        ok, msg, _ = self.teensy_activate(fire_axis)
        if not ok:
            return False, f"ACTIVATE rejected: {msg}"
        mon = ActivateMonitor(axes, targets)
        hard_deadline = time.monotonic() + mon.timeout_s + 5.0
        self.get_logger().info(f"activate: TRAP_TRAJ move to active pose on axes {axes}")
        while True:
            now = time.monotonic()
            res = mon.step(now, self._activate_axis_status(axes))
            if res.done:
                break
            if now > hard_deadline:
                self.get_logger().error("activate: hard deadline exceeded")
                break
            time.sleep(poll_dt)
        if mon.failed:
            parts = [f"axis {a}: {r}" for a, r in mon.failed.items()]
            msg = "activate FAILED — " + "; ".join(parts)
            if mon.succeeded:
                msg += f" (succeeded: {mon.succeeded})"
            self.get_logger().error(msg)
            return False, msg
        msg = f"activate complete on axes {mon.succeeded}"
        self.get_logger().info(msg)
        return True, msg

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
        # Phase 9a: Jetson-side orchestration over SET_AXIS_STATE (the firmware
        # ENCODER_SEARCH RPC remains stubbed). Scope via the encoder_search_axes
        # parameter (default all legs; [0] for the standalone-leg bench rig).
        axes = list(self.get_parameter('encoder_search_axes').value or [])
        ok, msg = self._run_encoder_search(axes)
        res.success = ok
        res.message = msg
        return res

    def _svc_home(self, req, res):
        # Phase 9b: the firmware HOME handler runs the velocity-limited
        # move-to-hardstop autonomously; this drives + observes it to completion.
        # Scope via the home_axes parameter (default all legs; [0] for the
        # standalone-leg bench rig).
        axes = list(self.get_parameter('home_axes').value or [])
        ok, msg = self._run_home(axes)
        # Phase 11 U5: apply the cold-start config after every successful homing
        # (gains/limits may have been changed in/since a prior session). Scope it
        # to the homed axes. A configure failure surfaces but does not undo homing.
        if ok:
            cfg_ok, cfg_msg = self._run_configure(axes)
            ok = ok and cfg_ok
            msg = f"{msg}; {cfg_msg}"
        res.success = ok
        res.message = msg
        return res

    def _svc_configure(self, req, res):
        # Phase 11 U5: apply the β-path cold-start config (gains + vel/curr limits
        # + POSITION/PASSTHROUGH) to the configure_axes. Run after homing (auto via
        # /home) and again before arming / after activate (TRAP_TRAJ → PASSTHROUGH).
        axes = list(self.get_parameter('configure_axes').value or [])
        ok, msg = self._run_configure(axes)
        res.success = ok
        res.message = msg
        return res

    def _svc_activate(self, req, res):
        # Phase 11 U5: fire the firmware TRAP_TRAJ move to the active pose +
        # observe completion. Scope via activate_axes (default all legs; [0] for
        # the standalone-leg bench rig). Precondition: a prior /configure.
        axes = list(self.get_parameter('activate_axes').value or [])
        ok, msg = self._run_activate(axes)
        res.success = ok
        res.message = msg
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

    # ═══════════════════════════════════════════════════════════
    # Catching cone (phase-10b cone uplink — mirrors can_node's
    # _handle_catch_event / _publish_cone_heartbeat field-by-field)
    # ═══════════════════════════════════════════════════════════

    def _drain_cone_catch_events(self):
        """Publish every queued catch event on cone/catch_event (100 Hz drain).

        Timestamp semantics mirror can_node._handle_catch_event: when the cone
        is time-synced, the frame's low-32 µs timestamp (latched in the cone's
        piezo ISR) is reconstructed into a full wall-clock instant against the
        host's clock at UDP arrival; when NOT synced the cone's counter is
        local-only and not comparable to wall time — fall back to host time so
        header.stamp / catch_time are at least a valid recent instant.
        Consumers must still check time_synced before treating catch_time as
        impact-truth.
        """
        with self._lock:
            if not self._cone_catch_queue:
                return
            queued = self._cone_catch_queue
            self._cone_catch_queue = []
        for evt, arrival_us in queued:
            try:
                msg = CatchEvent()
                msg.header.frame_id = 'catching_cone'
                if evt.time_synced:
                    catch_us = catching_cone.reconstruct_catch_time_us(
                        evt.catch_time_us_low32, arrival_us)
                    # Defensive: a host-clock jump (NTP step, manual `date`)
                    # between the last time-sync and this decode can place the
                    # reconstructed high bits in the wrong wrap window. Catches
                    # are necessarily within ms of UDP arrival; flag anything
                    # beyond ~1 s as a likely artefact.
                    delta_us = abs(catch_us - arrival_us)
                    if delta_us > 1_000_000:
                        self.get_logger().warning(
                            f"Catch seq {evt.sequence}: reconstructed time "
                            f"{delta_us / 1e6:.2f} s from host now — possible "
                            "host clock jump or stale time-sync.",
                            throttle_duration_sec=5.0)
                    stamp = rclpy.time.Time(nanoseconds=catch_us * 1000).to_msg()
                    msg.header.stamp = stamp
                    msg.catch_time = stamp
                else:
                    host_stamp = self.get_clock().now().to_msg()
                    msg.header.stamp = host_stamp
                    msg.catch_time = host_stamp
                msg.sequence = evt.sequence
                msg.time_synced = evt.time_synced
                msg.retrigger_suppressed = evt.retrigger_suppressed
                self.catch_event_pub.publish(msg)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"Catch event publish error: {e}")

    def _publish_cone_heartbeat(self):
        """Publish cone/heartbeat (10 Hz) from the latest relayed CONE_HEARTBEAT.

        Mirror of can_node._publish_cone_heartbeat: ``connected`` requires a
        cone heartbeat within CC_HEARTBEAT_TIMEOUT_MS, measured on the host
        monotonic clock from UDP arrival. Publishes connected=False defaults
        before the first heartbeat (matching can_node, whose consumers rely on
        the topic being alive to display "cone disconnected").
        """
        try:
            with self._lock:
                hb = self._latest_cone_hb
                received = self._cone_hb_received
                last_mono = self._cone_last_hb_mono
            connected = (received
                         and (time.monotonic() - last_mono) < self._cone_hb_timeout_s)
            msg = CatchingConeHeartbeat()
            msg.connected = connected
            msg.state = int(hb.state)
            msg.state_data = hb.state_data
            msg.sync_rms_us = hb.sync_rms_us
            msg.last_catch_seq = hb.last_catch_seq
            msg.ms_since_last_catch = hb.ms_since_last_catch
            msg.time_synced = hb.time_synced
            msg.have_any_catch = hb.have_any_catch
            self.cone_heartbeat_pub.publish(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Cone heartbeat publish error: {e}")

    # Extra wait (s) past throw_time before the action times out a result. Covers
    # the wind-up + release + the streamer completing and the firmware emitting OK.
    _BB_THROW_RESULT_MARGIN_S = 5.0

    @staticmethod
    def _bb_outcome_text(outcome, detail0, detail1):
        """Human-readable CMD_RESULT outcome (name + decoded details)."""
        try:
            name = proto.BallButlerCommandOutcome(int(outcome)).name
        except ValueError:
            name = f"UNKNOWN(0x{int(outcome):02x})"
        axis = {0: 'YAW', 1: 'PITCH', 2: 'BOTH'}.get(int(detail0), 'n/a')
        return f"{name} (axis={axis}, detail1={int(detail1)})"

    def _bb_throw_goal(self, goal_request):
        """Accept a throw goal only if none is outstanding (firmware is serialized).

        A speed==0 goal is an aim/track command (the firmware routes it to
        requestTracking — no throw, no CMD_RESULT); it does NOT consume the
        single-throw slot, so it is always accepted and completes immediately on
        dispatch (see execute_callback). Only real throws (speed>0) are gated.

        Runs in the action's ReentrantCallbackGroup, so it can reject a second
        throw promptly while execute_callback is still awaiting the first result.
        """
        if float(goal_request.throw_speed) == 0.0:
            return GoalResponse.ACCEPT
        with self._bb_throw_lock:
            if self._bb_throw_active:
                self.get_logger().warn(
                    'bb/throw: rejecting goal — a throw is already outstanding')
                return GoalResponse.REJECT
            self._bb_throw_active = True
            self._bb_throw_result = None
            self._bb_throw_event.clear()
        return GoalResponse.ACCEPT

    def _bb_throw_cancel(self, goal_handle):
        """Reject cancellation — a throw in flight has no firmware abort path."""
        return CancelResponse.REJECT

    def _bb_throw_execute(self, goal_handle):
        """Dispatch the throw and await the firmware's terminal CMD_RESULT.

        The BB_THROW RPC acks at the can-bridge (frame queued to CAN1), NOT at BB,
        so success/reject is learned only from the relayed CMD_RESULT — set on the
        RX thread by _on_cmd_result, which wakes the _bb_throw_event we wait on here.
        Times out (rather than hanging) if BB never answers.
        """
        req = goal_handle.request
        result = BallButlerThrowCmd.Result()

        # Aim/track command (speed 0): the firmware routes it to requestTracking —
        # no throw, no CMD_RESULT to await — so dispatch and complete immediately,
        # matching the retired service's fire-and-forget tracking behaviour. Does
        # not touch the single-throw slot (see goal_callback).
        if float(req.throw_speed) == 0.0:
            try:
                args = rpc_args.encode_bb_throw(
                    req.yaw_angle_rad, req.pitch_angle_rad, 0.0, req.throw_time)
            except Exception as e:  # noqa: BLE001
                result.success = False
                result.outcome = int(proto.BallButlerCommandOutcome.REJECTED)
                result.message = f"Aim arg encode failed: {e}"
                goal_handle.abort()
                return result
            ok, m, _ = self._call_rpc(RpcMethod.BB_THROW, args)
            result.success = ok
            result.outcome = int(proto.BallButlerCommandOutcome.OK if ok
                                 else proto.BallButlerCommandOutcome.REJECTED)
            result.message = ("Aim/track command dispatched (no throw)." if ok
                              else f"Aim dispatch failed (BB unreachable?): {m}")
            if ok:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result

        try:
            try:
                args = rpc_args.encode_bb_throw(
                    req.yaw_angle_rad, req.pitch_angle_rad,
                    req.throw_speed, req.throw_time)
            except Exception as e:  # noqa: BLE001 (Python-side range check failure)
                result.success = False
                result.outcome = int(proto.BallButlerCommandOutcome.REJECTED)
                result.message = f"Throw arg encode failed: {e}"
                goal_handle.abort()
                return result

            # Dispatch. A failed RPC means the bridge couldn't queue the frame
            # (BB not present) — no CMD_RESULT will ever come, so abort now.
            ok, m, _ = self._call_rpc(RpcMethod.BB_THROW, args)
            if not ok:
                result.success = False
                result.outcome = int(proto.BallButlerCommandOutcome.REJECTED)
                result.message = f"Throw dispatch failed (BB unreachable?): {m}"
                goal_handle.abort()
                return result

            # Await the firmware's terminal outcome (set on the RX thread).
            timeout_s = max(float(req.throw_time), 0.0) + self._BB_THROW_RESULT_MARGIN_S
            if not self._bb_throw_event.wait(timeout=timeout_s):
                result.success = False
                result.outcome = int(proto.BallButlerCommandOutcome.TIMEOUT)
                result.message = (
                    f"No CMD_RESULT within {timeout_s:.1f}s — firmware silent "
                    f"(BB detached, or a terminal point missed an emit).")
                self.get_logger().warn(f'bb/throw: {result.message}')
                goal_handle.abort()
                return result

            with self._bb_throw_lock:
                outcome, detail0, detail1 = self._bb_throw_result
            result.outcome = int(outcome)
            result.detail0 = int(detail0)
            result.detail1 = int(detail1)
            result.message = self._bb_outcome_text(outcome, detail0, detail1)
            if int(outcome) == int(proto.BallButlerCommandOutcome.OK):
                result.success = True
                self.get_logger().info(f'bb/throw: {result.message}')
                goal_handle.succeed()
            else:
                result.success = False
                self.get_logger().warn(f'bb/throw: {result.message}')
                goal_handle.abort()
            return result
        finally:
            with self._bb_throw_lock:
                self._bb_throw_active = False

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
    # MultiThreadedExecutor so the bb/throw action's result-wait (which can block
    # an executor thread for up to throw_time + margin in execute_callback) never
    # stalls the 100 Hz telemetry / heartbeat timers — they run on other threads.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
