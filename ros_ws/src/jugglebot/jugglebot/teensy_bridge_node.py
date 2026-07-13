"""Teensy can-bridge ROS 2 node — UDP-sourced mirror of ``can_node.py``.

This is the can-bridge successor to ``can_node.py``: it exposes the
same observable surface (robot state, hand telemetry, link/fault health) but
sources everything from the can-bridge Teensy over the dedicated UDP link
(``controller/teensy_link``) instead of socketcan. ``can_node`` was DELETED in
the SocketCAN decommission (2026-07-06; see
logbook/2026-07-06-phase13-socketcan-decommission.md) — the many
``can_node.py:NNN`` parity
citations throughout this file refer to the last pre-deletion revision in git
history (the capability mapping lives in
``ros_ws/docs/can-node-teensy-parity.md``). The bridge owns the
**production topic/service names** directly (legs/hand promoted off the
side-by-side ``/teensy/*`` namespace during the leg/hand cutover; BB + cone in a later cutover).

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
import struct
import threading
import time
from collections import namedtuple

import quaternion  # numpy-quaternion; same library can_node uses for tilt→quat

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
from jugglebot_interfaces.srv import (
    ODriveCommandService,
    SetHandTrajCmd,
    SetHandGains,
    SetFloat,
    SetString,
    ActivateOrDeactivate,
    GetTiltReadingService,
)
from jugglebot_interfaces.action import BallButlerThrowCmd, HomeMotors
from geometry_msgs.msg import Quaternion
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, SetBool

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
    PlatformFrame,
    HandCmdEcho,
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
from controller.teensy_link.deactivate import (
    DeactivateMonitor, AxisStatus as DeactivateAxisStatus,
)
from jugglebot.can import catching_cone
from jugglebot.can import odrive
from jugglebot.can.motor_state import MotorStateTracker
import jugglebot.hardware_config as hw
import jugglebot.protocol_config as proto


# ── Constants ──────────────────────────────────────────────────
# Mirror of can_node._HEARTBEAT_TIMEOUT_S: the can-bridge link is declared lost
# after this long without a T→J heartbeat. 2 s matches the CAN watchdog so the
# two nodes agree on what "lost" means during the side-by-side window.
_HEARTBEAT_TIMEOUT_S = 2.0

# Cadence of the persistent "TEENSY GUARD LATCHED" reminder while a guard fault
# stays latched. The fault EDGE logs once (loud, detailed); this repeats every
# _GUARD_LATCH_REPEAT_S so a latched E-STOP stays unmissable in a long log
# instead of scrolling out of sight after a single line. Enforced off a
# monotonic last-log timestamp (rate-EXACT) rather than rclpy
# throttle_duration_sec, whose first-call bookkeeping can silently drop the
# initial repeat.
_GUARD_LATCH_REPEAT_S = 5.0

# HeartbeatJ2T.flags bit0 = mpc_active (guard ENABLED). Sending this set tells
# the Teensy the Jetson is driving setpoints. It MUST stay 0 unless the operator
# has explicitly enabled setpoint output (Commit 3).
_FLAG_MPC_ACTIVE = 0x1

# Staleness window for the leg_setpoint_echo GUI topic: if no setpoint frame has
# been ACCEPTED by the pump within this window, the echo publishes NOTHING — so a
# stopped stream reads as silence downstream (the GUI gaps the dashed Pos (cmd)
# series out), never a stale flatline at the last commanded value.
_SETPOINT_ECHO_STALE_S = 0.5

# HeartbeatT2J.flags bits (T→J), per the protocol SPEC.
_T2J_FLAG_TIME_SYNCED = 0x1
_T2J_FLAG_STOW_PENDING = 0x2
_T2J_FLAG_ALL_AXIS_HB_OK = 0x4

# HeartbeatT2J.bb_flags bits (T→J, Ball Butler cutover).
_T2J_BB_FLAG_BALL_IN_HAND  = 0x1
_T2J_BB_FLAG_HEARTBEAT_SEEN  = 0x2
_T2J_BB_FLAG_HEARTBEAT_STALE = 0x4

# Axis layout (from the generated protocol): legs 0..5, hand = 6, NUM_AXES = 7.
_NUM_LEGS = p.NUM_LEGS
_HAND_AXIS = p.NUM_LEGS         # 6
_NUM_AXES = p.NUM_AXES          # 7

# "Already at STOW" band for the deactivate no-op guard: a leg is considered stowed
# when it is IDLE and within this of the STOW pose (DeactivateMonitor stow_rev = 0.0,
# so |pos| <= this). 0.2 rev sits just above the monitor's 0.15 arrival tolerance to
# allow the small foam-relaxed drift a leg takes the instant it IDLEs.
_STOW_POS_MAX_REV = 0.2

# ODrive undervoltage error/disarm bit (matches odrive.ERR_DC_BUS_UNDER_VOLTAGE).
_ERR_DC_BUS_UNDER_VOLTAGE = 512

# ODrive CLOSED_LOOP axis state (matches odrive.AXIS_STATES['CLOSED_LOOP']).
_AXIS_STATE_CLOSED_LOOP = 8

# ── Safe-shutdown sequencing (Task 3.3: Ctrl-C must ALWAYS profiled-stow) ──
# on_shutdown runs an ordered, bounded, best-effort sequence: DISARM (mpc_active→0)
# → SETTLE → profiled DEACTIVATE. The settle exists because set_heartbeat_flags(0)
# only takes effect on the heartbeat thread's NEXT tick (≤ 1/HEARTBEAT_HZ away): the
# firmware REJECTS DEACTIVATE while mpc_active=1 (leg_deactivate.cpp guard), so
# firing the DEACTIVATE RPC immediately would race ahead of the flags=0 J→T
# heartbeat and get bounced with ERR_BUS_DOWN — leaving the legs standing. Waiting a
# couple of heartbeat periods lets the disarm land first. Bounded — never hangs.
_SHUTDOWN_DISARM_SETTLE_S = 2.0 / float(p.HEARTBEAT_HZ)   # ≈ 0.2 s at 10 Hz
# Cap the shutdown DEACTIVATE so a stalled descent can't blow the ~8 s teardown
# budget. A healthy profiled stow reaches STOW+IDLE in ~2-4 s; this only bounds the
# stall case (DeactivateMonitor declares FAILED at timeout_s, the poll loop breaks).
_SHUTDOWN_DEACTIVATE_TIMEOUT_S = 6.0

# ── One-call guard recovery (/recover, FIX 3) ──
# Recovery convergence gate: after trajectory_node reseeds its hold at the measured
# encoder, /recover (and the rerouted armed /clear_errors) clears the guard ONLY once
# the streamed u0 is within this of every leg — so the clear cannot re-trip the latch.
# Tightened 0.25 → 0.03 rev (2026-07-11 clear-errors jolt forensics): the old 0.25 was
# 2.5× the firmware 0.10 rev lead clamp, so a "converged" clear could still leave u0 up
# to 0.25 rev off the drifted encoder → the lead clamp SATURATED at re-enable and
# injected pos_gain × 0.10 = 40 × 0.10 = a 4 rev/s velocity step to the −10 A current
# rail on every leg (measured on both clear events). 0.03 rev is well under the lead
# clamp, so the firmware re-enable slew barely engages — the two fixes COMPOSE (Jetson
# minimises the residual; firmware bounds the worst case at the single choke point).
_RECOVER_U0_TOL_REV = 0.03
# Arming pre-check margin — a SEPARATE, deliberately looser gate than the recovery
# convergence one above. Kept at 0.25 rev: arming seeds trajectory_node's hold at the
# measured pose so u0 already sits ≈ the encoder, AND the firmware re-enable slew now
# bounds any residual at the arm edge too, so tightening this would only risk spurious
# arm rejections without a safety gain. (Half the firmware 0.5 rev MAX_DEVIATION
# backstop.) The recovery path does NOT reuse this — a diverged command cleared onto a
# latched guard is exactly what the tight 0.03 gate above must catch.
_ARM_U0_TOL_REV = 0.25
# Bounded block on trajectory_node's reseed reply. The bridge runs a
# MultiThreadedExecutor, so a bounded wait here (in a ReentrantCallbackGroup) is safe.
_RECOVER_RESEED_TIMEOUT_S = 2.0
# Window /recover WAITS for trajectory_node's PROFILED DESCENT to walk the streamed
# u0 down onto the frozen encoder (u0 collapses over ~0.5-3 s at the session limits,
# each 40 Hz knot inside the pump gate — NOT the old instant one-frame reseed). Bounds
# the wait so a descent that never converges (froze in place) refuses rather than
# blocking the executor thread indefinitely.
_RECOVER_VERIFY_TIMEOUT_S = 4.0
# Bounded re-descend retries: the profiled descent targets the encoder SAMPLED AT
# INSTALL time, but during suppression the leg keeps drifting (~0.1 rev, closing the
# frozen lead offset — the Event-1 −0.102 rev plateau). If the descent converges u0
# onto that stale snapshot while the LIVE encoder has moved on, the convergence verify
# (which compares u0 vs live telemetry) plateaus above tol. Each retry re-samples the
# now-current encoder and re-installs the descent, chasing the settling leg; refuse
# after this many so a genuinely stuck leg does not block the executor thread forever.
_RECOVER_MAX_RESEED_ATTEMPTS = 3
# Manual-recovery hint appended to every armed-recovery REFUSAL (F5, 2026-07-11). When
# converge-first cannot complete (reseed timed out / refused / the descent never
# converged onto a stuck leg), the guard is left latched — so the refusal must tell the
# operator the always-available escape: DISARM (set_setpoint_output false → mpc_active=0,
# which has no external dependency and cannot deadlock) and then the plain /clear_errors
# clears straight through. Without this, an operator facing a stuck leg is left guessing.
_MANUAL_RECOVERY_HINT = ("manual recovery: disarm with set_setpoint_output=false, then "
                         "/clear_errors clears directly")

# Decoded Platform-Teensy RobotState (relay read). Fields mirror
# Teensy_code.ino RobotState (is_homed / levelling_complete / pose offset, rad).
RelayRobotState = namedtuple(
    "RelayRobotState",
    ["is_homed", "levelling_complete", "pose_offset_tiltX", "pose_offset_tiltY"])


def _decode_relay_robot_state(data: bytes) -> RelayRobotState:
    """Decode a 0x6E0 RobotState reply exactly as Teensy_code.ino
    decodeStateCANMessage packs it: byte0 flags (bit0 is_homed, bit1 levelling),
    int16 LE pose*1000 about X (bytes 1-2) and Y (bytes 3-4)."""
    if len(data) < 5:
        raise ValueError(f"RobotState reply too short: {len(data)} bytes")
    flags = data[0]
    x = int.from_bytes(data[1:3], "little", signed=True)
    y = int.from_bytes(data[3:5], "little", signed=True)
    return RelayRobotState(
        is_homed=bool(flags & 0x1),
        levelling_complete=bool(flags & 0x2),
        pose_offset_tiltX=x / 1000.0,
        pose_offset_tiltY=y / 1000.0)


def _tilt_to_quat(tilt_x: float, tilt_y: float) -> Quaternion:
    """Convert inclinometer tilt readings to a ROS Quaternion — byte-identical to
    ``can_node._tilt_to_quat`` (can_node.py:1667), so ``robot_state.pose_offset_quat``
    carries the SAME value can_node published.

    ``tilt_x`` / ``tilt_y`` are absolute tilt angles (radians), not deltas. The
    conversion is stateless (no accumulation drift)."""
    q_roll = quaternion.from_rotation_vector([-tilt_x, 0, 0])
    q_pitch = quaternion.from_rotation_vector([0, -tilt_y, 0])
    result = (q_roll * q_pitch).normalized()
    quat = Quaternion()
    quat.x = result.x
    quat.y = result.y
    quat.z = result.z
    quat.w = result.w
    return quat


def _enum_name(enum_cls, value: int) -> str:
    """Best-effort enum-member name for a wire value (falls back to ``v<n>``)."""
    try:
        return enum_cls(value).name
    except ValueError:
        return f"v{value}"


class _MpcCommandSetpointSource:
    """SUB-only connection to the MPC command PUB (tcp://127.0.0.1:5557).

    **The Jetson-relay→Teensy-side switch.** Previously this read motor_guard's
    *already-interpolated* 500 Hz telemetry on :5556 (the Jetson-relay path); it now reads
    the **40 Hz MPC command stream** on :5557 (``ipc.MPC_COMMAND_ADDR``,
    ``ipc.TOPIC_MPC_CMD``) — the same ``make_mpc_command`` dicts motor_guard
    consumes — so the Teensy does the 500 Hz interpolation (Teensy-side path). motor_guard
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
                 setpoint_source=None, boot_state_read: bool = True,
                 stow_on_shutdown: bool = True):
        super().__init__('teensy_bridge_node')

        # On a clean shutdown (Ctrl-C of the launch), profiled-stow the platform
        # before transport teardown — the Teensy-side analogue of can_node.on_shutdown's
        # _gently_move_to_setpoint(0.0, deactivating=True). Default True in
        # production; unit tests (which call on_shutdown against a FakeTeensy that
        # never completes the descent) pass False via _build_paired_node.
        self._stow_on_shutdown = bool(stow_on_shutdown)

        # ── Parameters ─────────────────────────────────────────
        self.declare_parameter('teensy_ip', p.TEENSY_IP)
        # SAFETY-CRITICAL: setpoint output is OFF by default. Wired in Commit 3.
        self.declare_parameter('enable_setpoint_output', False)
        self.declare_parameter('heartbeat_timeout_s', _HEARTBEAT_TIMEOUT_S)
        # which axes the encoder_search service runs index
        # search on. Default = all legs; set to e.g. [0] for the standalone-leg
        # bench rig (node 0 = axis 0).
        self.declare_parameter('encoder_search_axes', list(range(p.NUM_LEGS)))
        # which axes the home service homes (sequentially — the
        # firmware homes one axis at a time). Default = all legs + the HAND (axis 6),
        # matching can_node._home_robot (JUGGLEBOT_AXES = legs + hand). The hand homes
        # with Homing::HAND_* params after its gains are applied host-side; a leg-only
        # bench rig sets e.g. [0], which then excludes the hand automatically.
        self.declare_parameter('home_axes', list(range(p.NUM_LEGS)) + [_HAND_AXIS])
        # which axes the configure + activate services act on.
        # configure applies gains + vel/curr limits + POSITION/PASSTHROUGH (the
        # Teensy-side _setup_odrives, for the hand too — closes the "configure the hand"
        # parity gap). activate fires the TRAP_TRAJ move to the active pose (legs
        # only; ACTIVATE is rejected on the hand). Default = all legs + the hand;
        # set to e.g. [0] for the standalone-leg bench rig.
        self.declare_parameter('configure_axes', list(range(p.NUM_LEGS)) + [_HAND_AXIS])
        self.declare_parameter('activate_axes', list(range(p.NUM_LEGS)))
        self.declare_parameter('deactivate_axes', list(range(p.NUM_LEGS)))

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

        # ── Hand config state (byte-identical to can_node) ────────────
        # The gains/limits the bridge applies to the HAND ODrive (axis 6). Seeded
        # from config defaults (odrive.DEFAULT_*), updated by set_hand_gains. Used by
        # the cold-start hand configure/home path (refuse-flash-defaults) + the
        # set_hand_gains service. Mirrors can_node.py:135-137.
        self._hand_gains = dict(odrive.DEFAULT_HAND_GAINS)
        self._hand_vel_limit = odrive.DEFAULT_VEL_CURR['hand_vel']
        self._hand_curr_limit = odrive.DEFAULT_VEL_CURR['hand_curr']
        # Latest sniffed hand command (HAND_CMD_ECHO) → hand_telemetry cmd fields.
        # Mirrors can_node._last_hand_cmd (can_node.py:159).
        self._last_hand_cmd = {'pos': 0.0, 'vel': 0.0, 'tor': 0.0}

        # ── Latest-frame state (written on RX thread, read on ROS thread) ──
        self._lock = threading.Lock()
        self._latest_telemetry: Telemetry | None = None
        self._latest_heartbeat: HeartbeatT2J | None = None
        # Last fault_state seen on the T→J heartbeat, for edge-triggered logging in
        # _publish_link_status. None = nothing seen yet (so the first NONE is silent).
        self._last_fault_state: int | None = None
        # Monotonic timestamp of the last persistent "GUARD LATCHED" reminder (see
        # _GUARD_LATCH_REPEAT_S). None while no fault is latched; anchored to the
        # fault edge so the first repeat lands _GUARD_LATCH_REPEAT_S after it.
        self._last_guard_latch_log_t: float | None = None
        self._latest_profile: Profile | None = None
        # Per-axis latest Diagnostic (one axis per frame on the wire).
        self._latest_diag: dict[int, Diagnostic] = {}

        # Catching cone state (cone uplink). Catch events are
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
        # CAN3 (Jugglebot core) bus-health edge detector for the firmware-validation cold-start
        # reconnect re-trigger. None until the first heartbeat; only a recovery from
        # a DEGRADED state (WARN/BUS_OFF → OK) fires the conservative re-read — the
        # boot UNKNOWN→OK first-connection is excluded (the boot read already ran).
        self._last_bus1_health = None
        # The CAN3-reconnect conservative re-read does bounded retries + sleeps
        # (~1.9 s worst case), so it runs on a short-lived daemon thread and never
        # stalls the 100 Hz publish (which shares _health_check's callback group;
        # audit 2026-06-29 MEDIUM). This guard keeps at most one re-read in flight
        # (the recovery edge is one-shot). See _read_cold_start_state_conservative_async.
        self._cold_start_reread_inflight = False

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

        # ── BB loud command-outcome channel (CMD_RESULT relay) ──
        # One outstanding throw at a time (firmware is serialized → no correlation
        # token). The bb/throw action's execute_callback waits on _bb_throw_event;
        # the RX-thread _on_cmd_result handler stores the firmware outcome and sets
        # it. _bb_throw_active gates a second concurrent goal (rejected in
        # goal_callback). Guarded by _bb_throw_lock; the event lives outside it.
        self._bb_throw_lock = threading.Lock()
        self._bb_throw_event = threading.Event()
        self._bb_throw_active = False
        self._bb_throw_result = None   # (outcome:int, detail0:int, detail1:int)

        # ── Platform-Teensy relay replies (PLATFORM_FRAME) ──
        # The relay READ RPCs (TILT_READ / STATE_READ) only TRIGGER a Platform
        # reply; the reply arrives async as a PLATFORM_FRAME on the RX thread. A
        # relay read clears the latched reply for its can_id, sends the RPC, then
        # blocks on this condition for a fresh reply. Correlation is by
        # (can_id, dlc) — sound only if CAN3 SRX_DIS suppresses the bridge's own
        # 0x6E0 write echo (bench-probe gate; see _await_platform_reply).
        self._platform_reply_cv = threading.Condition()
        self._platform_replies = {}    # can_id -> (data: bytes, dlc: int)
        # Concurrency hardening: serialize whole relay round-trips. Two concurrent
        # relay ops (e.g. a boot/reconnect STATE_READ racing a home's STATE_WRITE)
        # would otherwise (a) clear + await the same can_id's latched reply and
        # cross-correlate, and (b) lose a read-modify-write update
        # (cache-vs-Platform-Teensy divergence). One lock around each op closes both.
        # Held only for the rare (~ms) relay round-trip, never on the 100 Hz path.
        # RLock (re-entrant) so the read-modify-write helpers (_write_is_homed /
        # _write_level_state / _clear_cold_start_state_on_reboot) can hold it across
        # their whole cache-read → wire-write → cache-update sequence — which itself
        # calls relay_write_robot_state (also lock-guarded) — fully closing the
        # lost-update, not just serializing the two wire writes.
        self._relay_lock = threading.RLock()

        # ── Cold-start state cache (relay-sourced is_homed/level/pose) ──
        # The Platform Teensy OWNS the persisted cold-start state; it shares the
        # ODrive supply so it "forgets when they forget" (there is no can-bridge
        # store and no invalidation rule). The bridge READS that state
        # via the relay STATE_READ at boot + on each confirmed CAN3 (UDP-link)
        # reconnect, CACHES it here, and surfaces is_homed / levelling_complete /
        # pose_offset on robot_state FROM THE CACHE — the 100 Hz publish path NEVER
        # does a CAN3 round-trip. The bridge is the SOLE writer and does
        # read-modify-write THROUGH this cache (a homing write preserves levelling +
        # pose, and vice versa). Guarded by self._lock (same as the telemetry cache;
        # written off the publish thread, read on it under a MultiThreadedExecutor).
        # The default is the CONSERVATIVE cold value (is_homed=False) so that, until
        # an authoritative read lands (or if every boot read fails), robot_state
        # forces a re-home — wasteful but SAFE; NEVER the reverse (which could skip
        # homing on an unhomed robot).
        self._cold_start_state = RelayRobotState(
            is_homed=False, levelling_complete=False,
            pose_offset_tiltX=0.0, pose_offset_tiltY=0.0)
        # True once an authoritative relay read (or a bridge-issued write) has set
        # the cache; surfaced on link_status for the operator (the powered bring-up sitting).
        self._cold_start_authoritative = False
        # encoder_search_complete is DERIVED (no wire field): is_homed OR a search
        # that completed THIS process. This in-session bit tracks the OR term; it is
        # sticky-True for the process once set (exact can_node parity — can_node only
        # ever sets encoder_search_complete True, never clears it mid-run).
        self._encoder_search_done_session = False
        # pose_offset_quat memo: _tilt_to_quat runs the (numpy-allocating) quaternion
        # composition only when the cached tilt CHANGES (it changes on a relay read,
        # not per publish) — can_node likewise computed it on-change, not per publish.
        # Touched only on the publish thread (the timer's MutuallyExclusiveCallbackGroup
        # serializes it), so no lock needed.
        self._pose_quat_key = None
        self._pose_quat_xyzw = (0.0, 0.0, 0.0, 1.0)

        # ── Firmware version validation ──────────────
        # Restores can_node's Get_Version handshake (_handle_get_version:474-495).
        # The firmware sweeps Get_Version + caches the raw versions; the bridge
        # pulls them via GET_AXIS_VERSIONS (a cheap UDP RPC reading a bridge-LOCAL
        # cache — NO CAN3 round-trip) and runs the EXISTING tested
        # MotorStateTracker.validate_group against EXPECTED_HW_VERSIONS, latching:
        #   firmware_validated=True  → orchestrator BOOT proceeds (un-gates the
        #                              is_homed skip, state_machine.py:228-235),
        #   mismatch string latched   → kept firmware_validated=False AND surfaced
        #                              on robot_state.error → orchestrator force-
        #                              FAULT (exact can_node:489-492 / 1085 parity).
        # Until validation lands, firmware_validated stays False (BOOT waits, then
        # FAULTs on BOOT_TIMEOUT_S — can_node's behaviour when versions never come).
        # _versions is reused ONLY for record_version + validate_group; the latched
        # bools are read on the publish path under self._lock. The tracker itself is
        # only touched on the version-poll timer thread (no lock needed for it).
        self._versions = MotorStateTracker()
        self._firmware_validated = False
        self._firmware_mismatch_error = None     # str once a mismatch is latched (sticky)

        # ── Subscriptions to T→J streams (RX-thread callbacks) ──
        self._client.subscribe(int(MsgType.HEARTBEAT_T2J), self._on_heartbeat_t2j)
        self._client.subscribe(int(MsgType.TELEMETRY), self._on_telemetry)
        self._client.subscribe(int(MsgType.DIAGNOSTIC), self._on_diagnostic)
        self._client.subscribe(int(MsgType.PROFILE), self._on_profile)
        self._client.subscribe(int(MsgType.CONE_FRAME), self._on_cone_frame)
        self._client.subscribe(int(MsgType.BB_AXIS_ESTIMATES), self._on_bb_estimates)
        self._client.subscribe(int(MsgType.CMD_RESULT), self._on_cmd_result)
        self._client.subscribe(int(MsgType.PLATFORM_FRAME), self._on_platform_frame)
        self._client.subscribe(int(MsgType.HAND_CMD_ECHO), self._on_hand_cmd_echo)  # hand conduit

        # ── Publishers (production names — leg-side cutover) ──
        # Promoted from the /teensy/* namespace to the production topic names
        # can_node used: with USB-CAN gone (can_node out of the launch), the
        # dual-publisher risk that drove the /teensy/* namespacing
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
        # GUI observability: the last ACCEPTED leg setpoint u0 (6 floats, motor
        # revs), source-agnostic — trajectory_node and run_mpc.py both feed the
        # :5557 funnel this echoes. No ROS topic otherwise carries commanded leg
        # positions on the Teensy-side path (the GUI's old datasource,
        # leg_lengths_topic, only publishes while the dormant run_mpc/motor_guard
        # stack streams). See _publish_leg_setpoint_echo for the freshness gate.
        self.leg_setpoint_echo_pub = self.create_publisher(
            Float64MultiArray, 'leg_setpoint_echo', 10)

        # ── Ball Butler (cutover, production names) ────
        # Intentional naming deviation from the earlier "all under /teensy/*"
        # convention: with USB-CAN removed, the dual-publisher risk that namespacing was
        # preventing is moot (can_node is gone for BB). The bridge inherits
        # the production names so the GUI / orchestrator / mocap_node /
        # throw_director see no name change across the cutover. (The leg/hand
        # topics + services followed onto production names in the leg/hand cutover.)
        self.bb_heartbeat_pub = self.create_publisher(
            BallButlerHeartbeat, 'bb/heartbeat', 10)

        # ── Catching cone (cone uplink, production names) ────
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

        # ── RPC service surface (production names — leg/hand cutover) ──
        # ODrive control issued over the can-bridge link via RpcClient. The
        # can-bridge owns CAN3 — legs 0-5 AND the hand ODrive (axis 6). Leg ops
        # target legs/broadcast; the hand surface is registered below.
        # Promoted from /teensy/* to the production names can_node served
        # (encoder_search, odrive_command, set_motor_vel_curr_limits) so the
        # orchestrator's existing service clients reach the bridge; clear_errors,
        # reboot_odrives and home are new bridge ops, named bare for consistency.
        #
        # ── Cold-start verb callback group ────────────────────────
        # Every BLOCKING cold-start verb — the manual encoder_search / home /
        # configure / activate / deactivate services (each a multi-second _run_*
        # poll loop) AND the orchestrator-facing home_motors action /
        # activate_or_deactivate / get_platform_tilt / set_level_state below — runs
        # in ONE ReentrantCallbackGroup so a multi-second move never starves the
        # 100 Hz robot_state publish + heartbeat (those stay in the node's default
        # MutuallyExclusiveCallbackGroup, dispatched on OTHER MultiThreadedExecutor
        # threads). Without this, a blocking verb in the default group serializes with
        # — and stalls — the telemetry timers for the whole move. Reentrancy is safe
        # here: the firmware busy-rejects a second concurrent move and the orchestrator
        # drives strictly sequentially; the one remaining race (two home_motors goals)
        # is closed by _home_action_goal's in-progress guard. reboot_odrives /
        # odrive_command stay in the default group — they are quick single RPCs, not
        # multi-second moves. clear_errors is created LATER, in the /recover
        # ReentrantCallbackGroup, because the armed reroute (F1) blocks multi-second.
        self._coldstart_cbgroup = ReentrantCallbackGroup()

        self.create_service(Trigger, 'reboot_odrives', self._svc_reboot_odrives)
        self.create_service(Trigger, 'encoder_search', self._svc_encoder_search,
                            callback_group=self._coldstart_cbgroup)
        self.create_service(Trigger, 'home', self._svc_home,
                            callback_group=self._coldstart_cbgroup)
        self.create_service(Trigger, 'configure', self._svc_configure,
                            callback_group=self._coldstart_cbgroup)
        self.create_service(Trigger, 'activate', self._svc_activate,
                            callback_group=self._coldstart_cbgroup)
        self.create_service(Trigger, 'deactivate', self._svc_deactivate,
                            callback_group=self._coldstart_cbgroup)
        self.create_service(ODriveCommandService, 'odrive_command',
                            self._svc_odrive_command)
        self.create_subscription(
            SetMotorVelCurrLimitsMessage, 'set_motor_vel_curr_limits',
            self._sub_vel_curr_limits, 10)

        # ── Orchestrator-facing cold-start conduit ────────────────
        # Makes the bridge a drop-in for the retired can_node from the LOCKED
        # orchestrator_node's view — ZERO edits to orchestrator_node / state_machine
        # (behaviour parity, not orchestrator churn). Each of the
        # four wrappers registers the exact (name, type) interface the orchestrator
        # drives cold-start through and delegates to the bridge's existing verbs:
        #   home_motors (HomeMotors action)      → _do_home (homes legs + hand)
        #   activate_or_deactivate (srv)         → _run_activate + _run_configure /
        #                                          _run_deactivate
        #   get_platform_tilt (srv)              → relay_read_tilt (NaN-on-failure)
        #   set_level_state (Float64MultiArray)  → _write_level_state (STATE_WRITE)
        # Registered directly on the bridge (not a separate conduit node/module): the
        # bridge owns CAN3 + the RPC client + these verbs; there is no existing
        # actions/services module to join, and grouping four verbs into one only
        # because they land together would be cohesion-by-implementation-moment (a
        # future teensy_bridge_node split should cut along the relay / hand /
        # cold-start seams instead). The drift-guard test
        # (tests/ros/test_orchestrator_conduit_contract.py) pins this (name, type) set
        # to the orchestrator's declared clients so a future rename fails the suite,
        # not the bench.
        self._home_lock = threading.Lock()
        self._home_in_progress = False   # guards concurrent home_motors goals
        self._home_action = ActionServer(
            self, HomeMotors, 'home_motors',
            execute_callback=self._home_action_execute,
            goal_callback=self._home_action_goal,
            callback_group=self._coldstart_cbgroup)
        self.create_service(
            ActivateOrDeactivate, 'activate_or_deactivate',
            self._svc_activate_or_deactivate,
            callback_group=self._coldstart_cbgroup)
        self.create_service(
            GetTiltReadingService, 'get_platform_tilt',
            self._svc_get_platform_tilt,
            callback_group=self._coldstart_cbgroup)
        self.create_subscription(
            Float64MultiArray, 'set_level_state', self._sub_set_level_state, 10,
            callback_group=self._coldstart_cbgroup)

        # ── Hand command surface ──────────────────────────────────
        # The can-bridge owns CAN3, which carries the hand ODrive (axis 6), so it
        # restores the hand conduit can_node held (silently no-op'd against the
        # bridge until now — catch_coordinator's hand services were GAPs). Same ROS
        # names + srv types can_node registered (can_node.py:198-201), so
        # catch_coordinator_node reaches the bridge UNCHANGED: set_hand_state/gains
        # ride the relay-seam axis-6 allow-table; set_hand_traj_cmd/smooth_move_hand
        # ride the HAND_TRAJ_CMD RPC (byte-identical 0x6D0 → Platform Teensy).
        self.create_service(SetString, 'set_hand_state', self._svc_set_hand_state)
        self.create_service(SetHandTrajCmd, 'set_hand_traj_cmd', self._svc_set_hand_traj)
        self.create_service(SetFloat, 'smooth_move_hand', self._svc_smooth_move_hand)
        self.create_service(SetHandGains, 'set_hand_gains', self._svc_set_hand_gains)

        # ── Ball Butler services (production names, cutover) ────
        # The bb/* services formerly served by can_node. The firmware
        # gates each TX on bb_present(); we translate the ERR_BUS_DOWN that
        # gate produces into a silent-success for bb/calibrate to preserve
        # can_node._svc_bb_calibrate's HOMING semantics (allows homing to
        # complete without BB attached). The other two propagate the
        # error so an operator sees a failed reload/reset.
        self.create_service(Trigger, 'bb/reload',    self._svc_bb_reload)
        self.create_service(Trigger, 'bb/reset',     self._svc_bb_reset)
        self.create_service(Trigger, 'bb/calibrate', self._svc_bb_calibrate)

        # ── Ball Butler throw ACTION (replaces bb/send_throw_command) ──
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
        self.create_timer(1.0, self._version_check_poll)       # 1 Hz firmware-version handshake (no-op once resolved)

        # ── Setpoint downlink (Commit 3) — DEFAULT DISABLED ────
        # The 40/500 Hz hot path. The SetpointPump (pure packing + per-step
        # safety gate) is always constructed (cheap), but the ZMQ source and the
        # ingest thread are created ONLY when ~enable_setpoint_output is true.
        # While disabled there is NO setpoint thread and the heartbeat keeps
        # mpc_active=0, so the Teensy will not enable leg output. The operator
        # flips the parameter (and restarts the node) only after bench validation.
        #
        # LEG TORQUE FEEDFORWARD — also default-disabled, independently
        # (dynamics.torque_ff_enabled, shipped false). The pump is the SINGLE wire
        # enforcement point for it: clamp (TRUE Nm) → ramp → Kt wire scale. With the
        # flag off it packs torque_ff = zeros and never reads cmd['torque_Nm'], so
        # the frame is byte-identical to the pre-feature one. The ramp is expressed
        # in ACCEPTED FRAMES (deterministic, and a dropped frame lengthens it — the
        # safe direction), derived here from the ramp seconds and the fixed 40 Hz
        # knot spacing.
        _ff_ramp_frames = int(math.ceil(
            float(hw.DYNAMICS_TORQUE_FF_RAMP_S) / float(hw.JB_TRAJ_KNOT_DT_S)))
        self._sp_pump = SetpointPump(
            mm_to_rev=hw.GEOM_MM_TO_REV,
            num_legs=p.NUM_LEGS, max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV,
            torque_ff_enabled=bool(hw.DYNAMICS_TORQUE_FF_ENABLED),
            torque_ff_max_nm=float(hw.DYNAMICS_TORQUE_FF_MAX_NM),
            torque_wire_scale=float(hw.ODRIVE_LEG_TORQUE_WIRE_SCALE),
            torque_ff_ramp_frames=_ff_ramp_frames)
        if hw.DYNAMICS_TORQUE_FF_ENABLED:
            self.get_logger().warn(
                'LEG TORQUE FEEDFORWARD IS ENABLED '
                f'(clamp ±{hw.DYNAMICS_TORQUE_FF_MAX_NM} Nm true, wire scale '
                f'{hw.ODRIVE_LEG_TORQUE_WIRE_SCALE:.6f}, ramp {_ff_ramp_frames} frames '
                f'≈ {hw.DYNAMICS_TORQUE_FF_RAMP_S} s). '
                'Set dynamics.torque_ff_enabled=false + regenerate + colcon build to disable.')
        # ── leg_setpoint_echo stash (GUI observability) ────────
        # Written by _process_setpoint on the SETPOINT THREAD (the production
        # leg hot path) — the write is the absolute minimum under self._lock:
        # one tuple ref + one float + one int. NO message construction and NO
        # publishing happen on that thread; _publish_leg_setpoint_echo (100 Hz
        # robot_state timer, executor thread) does both, gated on freshness
        # (_SETPOINT_ECHO_STALE_S) and deduped by seq so each accepted 40 Hz
        # frame is echoed at most once.
        self._last_accepted_u0 = None        # tuple[float x6], motor revs
        self._last_accepted_u0_mono = 0.0    # time.monotonic() at accept
        self._last_accepted_u0_seq = 0       # bumps once per ACCEPTED frame
        # Timer-thread-only (the timer's MutuallyExclusiveCallbackGroup
        # serializes it — same argument as _pose_quat_key): no lock needed.
        self._echo_last_pub_seq = 0
        self._sp_source = None
        self._sp_thread = None
        self._sp_stop = threading.Event()
        enable_sp = bool(self.get_parameter('enable_setpoint_output').value)
        if enable_sp:
            self._start_setpoint_output(setpoint_source)
        # Injected source retained so runtime arming (set_setpoint_output) can reuse
        # it in tests instead of opening a real ZMQ SUB on :5557.
        self._injected_setpoint_source = setpoint_source

        # ── Runtime arming service (fixes the arm-before-stream trap) ──
        # The documented PRODUCTION arming flow (the launch parameter stays for
        # bench use). On true: require (a) Teensy link up + fresh heartbeat, (b) a
        # fresh mpccmd frame on :5557 within 0.5 s, (c) that frame's u0 within
        # _ARM_U0_TOL_REV (0.25 rev, half the firmware 0.5 rev MAX_DEVIATION backstop) of
        # every leg's live pos_estimate — THEN stream-then-arm (start the thread, set
        # mpc_active=1). This is the pattern validated in
        # tests/hardware/teensy_guard_validation.py: never arm before a matching
        # stream is confirmed flowing, so the Teensy never E-STOPs at the arm edge.
        self.create_service(
            SetBool, 'set_setpoint_output', self._svc_set_setpoint_output)

        # ── One-call guard recovery (/recover, FIX 3) ──
        # The 2026-07-10 runaway left the streamed u0 diverged ~0.5+ rev from the
        # frozen encoder, so every bare /clear_errors re-latched MAX_DEVIATION within
        # one 10 Hz fault tick. /recover does the recovery in the ONLY order that
        # survives: reseed trajectory_node's hold at the measured encoder → VERIFY the
        # streamed u0 has collapsed to within _RECOVER_U0_TOL_REV (0.03 rev) of every
        # encoder → THEN CLEAR_ERRORS. Hosted HERE (not trajectory_node) because the bridge runs a
        # MultiThreadedExecutor and already blocks executor threads for multi-second
        # cold-start verbs, so the bounded block on the reseed future is safe;
        # trajectory_node runs a single-threaded rclpy.spin() executor where blocking
        # a callback on a cross-process future would deadlock its sole thread. The
        # service + the cross-process reseed client share a ReentrantCallbackGroup so
        # the reseed reply is dispatched on another executor thread while /recover
        # blocks on the future.
        self._recover_cbgroup = ReentrantCallbackGroup()
        self._reseed_client = self.create_client(
            Trigger, 'trajectory/reseed_from_measured',
            callback_group=self._recover_cbgroup)
        self.create_service(Trigger, 'recover', self._svc_recover,
                            callback_group=self._recover_cbgroup)
        # clear_errors shares the /recover ReentrantCallbackGroup (F1, 2026-07-11).
        # WHY not the node-default MutuallyExclusiveCallbackGroup: the armed bare
        # /clear_errors reroutes INLINE through _svc_recover, which blocks up to
        # _RECOVER_MAX_RESEED_ATTEMPTS × (reseed_timeout + verify_timeout) ≈ 18 s. In the
        # default group that block would SERIALIZE with — and starve — the 100 Hz
        # telemetry timers AND the set_setpoint_output disarm service, making the disarm
        # escape hatch unreachable for the whole block. In the reentrant group the reseed
        # reply dispatches on another executor thread and the disarm/telemetry callbacks
        # keep running, so the operator can always disarm out.
        self.create_service(Trigger, 'clear_errors', self._svc_clear_errors,
                            callback_group=self._recover_cbgroup)
        # Instance-level so tests can shorten the descent-convergence wait (a
        # never-converging descent otherwise blocks the test for the full timeout).
        self._recover_verify_timeout_s = _RECOVER_VERIFY_TIMEOUT_S

        # ── Cold-start boot read ─────────────────────
        # Read the Platform Teensy's persisted RobotState BEFORE construction
        # returns — i.e. before the executor spins and the first robot_state is
        # published — so the orchestrator never acts on a wrong is_homed. Bounded
        # (a few retries × _RELAY_READ_TIMEOUT_S); on total failure the cache stays
        # at the CONSERVATIVE default (is_homed=False → forces a re-home, SAFE).
        # Disabled in unit tests (boot_state_read=False) that don't wire a Platform
        # responder — they exercise _refresh_cold_start_state directly instead.
        if boot_state_read:
            self._boot_read_cold_start_state()

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
                    q = self._cone_catch_queue
                    q.append((evt, arrival_us))
                    # Concurrency hardening: bound the queue if the 100 Hz drain stalls, same
                    # as the BB-estimates sibling (drop-oldest). Catches are discrete
                    # + rare, so 4000 (~unbounded in practice) only guards a stuck drain.
                    if len(q) > 4000:
                        del q[:len(q) - 4000]
            elif cf.can_id == catching_cone.HEARTBEAT_ID:
                hb = catching_cone.ConeHeartbeat.from_can_frame(data)
                with self._lock:
                    self._latest_cone_hb = hb
                    self._cone_hb_received = True
                    self._cone_last_hb_mono = time.monotonic()
        except Exception:  # noqa: BLE001
            return

    def _on_platform_frame(self, msg_type, seq, payload, addr):
        # RX-thread callback: a Platform-Teensy relay reply (CAN3 0x6E0 RobotState
        # / 0x7DE tilt) the bridge forwarded verbatim. Latch the latest reply by
        # can_id and wake any relay read blocked on it. Malformed frames dropped
        # (never kill the RX thread). Decode lives in the relay read methods so the
        # bridge stays decoupled from the Platform-Teensy byte layout.
        try:
            pf = PlatformFrame.unpack(payload)
        except Exception:  # noqa: BLE001
            return
        data = bytes(pf.data[:pf.dlc])
        with self._platform_reply_cv:
            self._platform_replies[int(pf.can_id)] = (data, int(pf.dlc))
            self._platform_reply_cv.notify_all()

    def _on_hand_cmd_echo(self, msg_type, seq, payload, addr):
        # RX-thread callback (hand conduit): the bridge sniffed the Platform Teensy's
        # Set_Input_Pos to the hand ODrive (axis 6) on CAN3. Decode the commanded
        # pos/vel_ff/tor_ff and stash them so _publish_hand_telemetry can echo them
        # (byte-identical to can_node._handle_hand_input_pos: <f h h> then vel/tor
        # ÷ INPUT_SCALE_HAND_VEL/_TOR). Malformed frames dropped (never kill the RX
        # thread). Stash under the shared lock; the publish timer reads it.
        try:
            ce = HandCmdEcho.unpack(payload)
            pos, vel, tor = rpc_args.decode_hand_cmd_echo(
                bytes(ce.data),
                proto.INPUT_SCALE_HAND_VEL, proto.INPUT_SCALE_HAND_TOR)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._last_hand_cmd = {'pos': pos, 'vel': vel, 'tor': tor}

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
        # RX-thread callback (loud command-outcome channel). Decode the relayed CMD_RESULT
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

    def _publish_leg_setpoint_echo(self):
        """Echo the last ACCEPTED leg setpoint u0 (6 floats, motor revs) for the GUI.

        Rides the 100 Hz robot_state timer (executor thread) — the setpoint
        thread only stashes (see _process_setpoint); it NEVER publishes. Two
        gates keep the topic honest:
          * freshness — no accepted frame within _SETPOINT_ECHO_STALE_S ⇒
            publish nothing, so downstream sees silence (chart gap), not a
            stale flatline;
          * dedup — each accepted frame (seq) is published at most once, so
            the topic carries the real ~40 Hz stream, not 100 Hz repeats.
        """
        try:
            with self._lock:
                u0 = self._last_accepted_u0
                mono = self._last_accepted_u0_mono
                seq = self._last_accepted_u0_seq
            if u0 is None or seq == self._echo_last_pub_seq:
                return  # nothing accepted yet / this frame already echoed
            if (time.monotonic() - mono) > _SETPOINT_ECHO_STALE_S:
                return  # stream stopped — silence, never a stale flatline
            msg = Float64MultiArray()
            msg.data = [float(v) for v in u0]
            self.leg_setpoint_echo_pub.publish(msg)
            self._echo_last_pub_seq = seq
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Setpoint echo publish error: {e}",
                                    throttle_duration_sec=5.0)

    def _publish_robot_state(self):
        """Publish robot_state, mirroring can_node._publish_robot_state.

        Suppressed until the first Telemetry frame arrives so the topic never
        carries a misleading all-zero / all-IDLE snapshot before the link is up.

        Also drives the leg_setpoint_echo publish off this same 100 Hz timer
        (see _publish_leg_setpoint_echo) — deliberately BEFORE the telemetry
        suppression gate, since the echo has its own freshness gate.
        """
        try:
            # Echo first (before the telemetry gate below): the echo has its
            # own freshness gate and must publish even while robot_state is
            # suppressed pre-telemetry.  Inside this try as containment —
            # _publish_leg_setpoint_echo catches its own exceptions, but a
            # pathological escape (e.g. the attribute itself broken) must not
            # take robot_state publishing down with it.
            self._publish_leg_setpoint_echo()
            with self._lock:
                telem = self._latest_telemetry
                hb = self._latest_heartbeat
                diag = dict(self._latest_diag)
                cold = self._cold_start_state            # immutable namedtuple
                search_session = self._encoder_search_done_session
                fw_validated = self._firmware_validated
                fw_mismatch = self._firmware_mismatch_error
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
            # After the three-bus remap (ADR-0013) the on-wire
            # bus1_health slot carries the Jugglebot CORE bus (CAN3: 6 legs + hand) --
            # the bus whose BUS_OFF is fatal for the legs. (bus2_health is now Ball
            # Butler; the cone bus health is not yet on the uplink -- TODO (cone-uplink work).)
            core_bus_health = int(hb.bus1_health) if hb is not None else 0
            legs = states[:_NUM_LEGS]  # the can-bridge owns legs 0-5 (hand = platform Teensy)
            any_leg_active_err = any(s.active_errors != 0 for s in legs)
            # CROSS-AXIS disarm-while-CLOSED_LOOP (fault-parity hardening; exact
            # parity with fault_logic.py:117 `any_disarmed and any_cl` + can_node):
            # ANY leg disarmed while ANY leg still holds CLOSED_LOOP is fatal. The
            # previous per-leg conjunction (the SAME leg disarmed AND CLOSED_LOOP)
            # essentially never fired — a disarmed ODrive leaves CLOSED_LOOP almost
            # immediately — so the common real event (one leg drops torque while the
            # others keep holding) was missed by exactly the OR-term that exists to
            # un-mask a fault while fault_state carries a higher-priority code.
            any_leg_disarmed = any(s.disarm_reason != 0 for s in legs)
            any_leg_in_cl = any(
                s.current_state == _AXIS_STATE_CLOSED_LOOP for s in legs)
            disarm_while_cl = any_leg_disarmed and any_leg_in_cl
            # A firmware-version MISMATCH is a fatal ODrive condition (can_node set
            # fatal_error=True → has_fatal_odrive_error=True, can_node:491/1080). It
            # is a latched OR-term recomputed each publish (sticky for the session).
            # NOTE (audit 2026-06-29 LOW, intentional): because the latch is sticky,
            # this stays True even after a CLEAR_ERRORS, where can_node's
            # has_fatal_odrive_error (= fatal_error) would drop to False. The
            # divergence is deliberate — a wrong-firmware ODrive IS a hard config
            # error that must stay fatal until the firmware is fixed + the bridge
            # restarted (the plan's "latched OR-term" instruction); both behaviours
            # still force-FAULT via the sticky error string in robot_state.error.
            fw_mismatch_present = fw_mismatch is not None
            msg.has_fatal_odrive_error = (
                fault_state == int(FaultState.ODRIVE_FATAL)
                or any_leg_active_err or disarm_while_cl
                or fw_mismatch_present)
            # Safety hardening: surface a dead Jetson↔Teensy UDP link as a fatal
            # CAN error. The orchestrator's ONLY health input is robot_state; without
            # this, a lost link leaves it consuming FROZEN motor states stamped with
            # fresh clock times + healthy flags indefinitely (can_node coupled 2 s
            # silence → fatal_can_error; that coupling drove the 2026-05-19 stow work
            # and was dropped in the port). The LinkLossLatch never arms before the
            # first heartbeat, so this OR is boot-safe. Read outside self._lock (the
            # latch is a separate object, updated by the 1 Hz _health_check).
            link_lost = bool(self._link_latch.link_lost)
            msg.has_fatal_can_error = (
                fault_state == int(FaultState.CAN_BUS_DOWN)
                or core_bus_health == int(BusHealth.BUS_OFF)
                or link_lost)
            # Undervoltage: matches can_node, which sets undervoltage_error ONLY
            # from a BITWISE test on active_errors (can_node._handle_error:436);
            # disarm_reason==UV is used by can_node solely in its clear predicate,
            # never to ASSERT UV. active_errors/disarm are ODrive bitfields, so use
            # bitwise & (not ==), active_errors only.
            msg.has_undervoltage = any(
                s.active_errors & _ERR_DC_BUS_UNDER_VOLTAGE for s in legs)
            # firmware_validated (version handshake): latched by the GET_AXIS_VERSIONS
            # handshake (_version_check_poll → validate_group). False until the
            # bridge has pulled + validated the ODrive versions; un-gates the
            # orchestrator's is_homed skip (state_machine.py:228-235) when True.
            msg.firmware_validated = fw_validated

            # Human-readable error strings (logging/rosbag parity with can_node).
            errors = []
            # Firmware-version mismatch FIRST (can_node:1085-1086 order) — its
            # presence in robot_state.error force-FAULTs the orchestrator
            # (orchestrator_node.py:137-139), the parity stop for a wrong-firmware
            # ODrive. Recomputed each publish from the sticky latch.
            if fw_mismatch_present:
                errors.append(fw_mismatch)
            if msg.has_undervoltage:
                errors.append("Undervoltage detected. Was the E-stop hit?")
            if msg.has_fatal_odrive_error:
                errors.append(
                    f"Fatal ODrive issue (Teensy fault_state="
                    f"{_enum_name(FaultState, fault_state)}).")
            if msg.has_fatal_can_error:
                # Distinguish the UDP-link-loss cause (safety hardening) from a CAN3
                # bus fault, so an operator sees WHY robot_state went fatal.
                errors.append(
                    "Teensy link lost (UDP) — telemetry is stale."
                    if link_lost else "Fatal CAN bus issue.")
            msg.error = errors

            # Teensy-persisted cold-start state — sourced from the
            # Platform Teensy's RobotState via the relay STATE_READ (cached at boot
            # + on CAN3 reconnect; the publish path is non-blocking — it reads only
            # the cache, never a CAN3 round-trip). Replaces the hardcoded
            # conservative defaults of the earlier side-by-side bring-up.
            is_homed = bool(cold.is_homed)
            msg.is_homed = is_homed
            msg.levelling_complete = bool(cold.levelling_complete)
            # encoder_search_complete is DERIVED, not independently persisted —
            # exact can_node parity (can_node.py:549-550 sets it True whenever
            # is_homed; homing requires a prior search, so is_homed ⇒ search done)
            # plus the in-session bit set on a successful /encoder_search.
            msg.encoder_search_complete = is_homed or search_session
            tilt_x = float(cold.pose_offset_tiltX)
            tilt_y = float(cold.pose_offset_tiltY)
            msg.pose_offset_rad = [tilt_x, tilt_y]
            # Recompute the quat only when the tilt changes (memo — see __init__);
            # the publish-thread serialization makes the bare reads/writes safe.
            key = (tilt_x, tilt_y)
            if key != self._pose_quat_key:
                q = _tilt_to_quat(tilt_x, tilt_y)
                self._pose_quat_key = key
                self._pose_quat_xyzw = (q.x, q.y, q.z, q.w)
            qx, qy, qz, qw = self._pose_quat_xyzw
            msg.pose_offset_quat = Quaternion(x=qx, y=qy, z=qz, w=qw)

            self.robot_state_pub.publish(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Robot state publish error: {e}",
                                    throttle_duration_sec=5.0)

    def _publish_hand_telemetry(self):
        """Publish hand_telemetry from axis 6 of the Telemetry frame.

        Mirrors can_node._publish_hand_telemetry: the MEASURED side from the
        Telemetry/Diagnostic cache, and (hand conduit) the COMMAND side (pos_cmd/
        vel_ff_cmd/tor_ff_cmd) from the last sniffed HAND_CMD_ECHO — the hand's
        commanded-vs-measured tracking-error diagnostic (catch-tuning) that was
        dropped to 0 on the bridge until now.
        """
        try:
            with self._lock:
                telem = self._latest_telemetry
                diag = self._latest_diag.get(_HAND_AXIS)
                cmd = dict(self._last_hand_cmd)
            if telem is None:
                return
            msg = HandTelemetryMessage()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.pos_cmd = float(cmd['pos'])
            msg.vel_ff_cmd = float(cmd['vel'])
            msg.tor_ff_cmd = float(cmd['tor'])
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
        RPC until a future firmware release, and the Teensy already owns the profiled
        CAN-side stow. This is the "always stow on confirmed
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
                        "STOW PENDING: no stow RPC exists yet (future firmware); "
                        "operator/orchestrator must stow the platform. Surfaced "
                        "on link_status (bridge_stow_pending=1).")
                else:
                    self.get_logger().info("Teensy link RESTORED.")
                # Safety hardening: forget the pre-loss setpoint so the per-step
                # gate can't wedge after a firmware stow. During the loss the firmware
                # may have stowed the legs to ~0 rev, but the pump's _prev_pos still
                # holds the pre-loss active pose (~2.2 rev) — so the first
                # post-reconnect MPC frame would fail the 0.3 rev step gate FOREVER
                # (reject loop until a node restart). reset() drops _prev_pos so the
                # next frame is accepted; the firmware MAX_DEVIATION gate remains the
                # complementary command-vs-encoder layer (setpoint_pump's documented
                # two-layer design). SAFE — it only forgets the prior frame, never
                # relaxes a bound.
                self._sp_pump.reset()
                # Cold-start refresh: on a confirmed UDP-link reconnect, re-read the Platform
                # Teensy's cold-start state (it is the authoritative store; a
                # reconnect only triggers a re-read, never INFERS reference state).
                # Refresh off the publish path; KEEP the cached value if the read
                # fails — a Jetson↔Teensy link blip leaves the Platform Teensy (and
                # its references) powered, so a re-home would be wrong (can_node
                # passive last-known parity). Dispatched to a daemon thread
                # so the relay round-trip can't stall the 100 Hz publish (shared
                # callback group). Best-effort: if a conservative CAN3-recovery
                # re-read already holds the shared guard, this keep-stale read is
                # simply skipped — the conservative read is the stronger refresh.
                self._dispatch_cold_start_reread(
                    self._refresh_cold_start_state, 'reconnect', 'cs-udp-reread')
            self._last_link_lost = self._link_latch.link_lost

            # Firmware-validation precondition (audit 2026-06-29): CAN3-bus-health reconnect
            # re-trigger. The UDP-watchdog edge above does NOT fire for a CAN3-only
            # drop — if Jugglebot is disconnected while the Jetson + can-bridge stay
            # powered, the UDP link never drops, so the UDP-reconnect re-read
            # never runs and the cache could hold a STALE is_homed=True against a
            # de-referenced robot (the hole that goes LIVE once firmware validation ungates the
            # orchestrator's is_homed skip). bus1_health (CAN3, the Jugglebot core
            # bus) recovering to OK from a DEGRADED state (WARN/BUS_OFF) signals the
            # Platform Teensy may have lost + regained power (it shares the ODrive
            # supply), so re-read CONSERVATIVELY (retry,
            # then is_homed=False on failure — NOT keep-stale). UNKNOWN→OK (the boot
            # first-connection) is excluded: it is not a loss, and the __init__ boot
            # read already covered it (re-reading there could needlessly clobber a
            # good boot value with the conservative fallback).
            with self._lock:
                hb = self._latest_heartbeat
            cur_bus1 = int(hb.bus1_health) if hb is not None else None
            _degraded = (int(BusHealth.WARN), int(BusHealth.BUS_OFF))
            if (cur_bus1 == int(BusHealth.OK)
                    and self._last_bus1_health in _degraded):
                self.get_logger().warning(
                    "CAN3 bus health recovered to OK (was "
                    f"{_enum_name(BusHealth, self._last_bus1_health)}) — Jugglebot "
                    "may have power-cycled; conservative cold-start re-read.")
                # OFF the 1 Hz timer thread — the re-read can take ~1.9 s and must
                # not stall the 100 Hz publish (shared callback group; audit MEDIUM).
                dispatched = self._dispatch_cold_start_reread(
                    self._read_cold_start_state_conservative,
                    'can3_reconnect', 'cs-can3-reread')
                # Consume the recovery edge ONLY once the conservative re-read is
                # under way. If a same-tick UDP-reconnect keep-stale read won the
                # shared guard (dispatched=False), leave _last_bus1_health DEGRADED
                # so the edge re-fires next tick — the conservative read (is_homed=
                # False on failure) must never be permanently preempted by the
                # weaker keep-stale path.
                if dispatched:
                    self._last_bus1_health = cur_bus1
            elif cur_bus1 is not None:
                self._last_bus1_health = cur_bus1
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Health check error: {e}",
                                    throttle_duration_sec=5.0)

    # ═══════════════════════════════════════════════════════════
    # Firmware-version handshake — Get_Version → validate_group
    # ═══════════════════════════════════════════════════════════

    def _version_check_poll(self):
        """Pull the bridge's cached ODrive versions (GET_AXIS_VERSIONS) and run the
        tested validate_group, latching firmware_validated / the mismatch string.

        No-op once resolved (validated, or a mismatch latched). Runs OFF the publish
        path: GET_AXIS_VERSIONS reads a bridge-LOCAL cache (no CAN3 round-trip), so
        the RPC is a cheap UDP round-trip. A failed pull — old firmware
        (ERR_UNKNOWN_METHOD / ERR_NOT_IMPL) or the firmware sweep not yet complete —
        just retries next tick; the orchestrator's BOOT_TIMEOUT_S governs the
        give-up, exactly as can_node validated asynchronously as Get_Version replies
        arrived (can_node._handle_get_version:487-495)."""
        with self._lock:
            if self._firmware_validated or self._firmware_mismatch_error:
                return
        try:
            # GET_AXIS_VERSIONS is a bridge-LOCAL cache read (sub-ms RTT normally),
            # but this poll shares _publish_robot_state's callback group, so bound
            # the worst-case block on a degraded link (audit 2026-06-29 LOW): one
            # short timeout + one retry rather than the default budget. A miss just
            # retries next 1 Hz tick (BOOT_TIMEOUT governs the give-up).
            ok, msg, blob = self._call_rpc(
                RpcMethod.GET_AXIS_VERSIONS, timeout=0.3, retries=1)
        except Exception as e:  # noqa: BLE001 — never let a timer die
            self.get_logger().error(f"version check pull errored: {e}",
                                    throttle_duration_sec=5.0)
            return
        if not ok or not blob:
            # Old firmware (ERR_UNKNOWN_METHOD/ERR_NOT_IMPL) or sweep not done →
            # cannot validate yet; keep firmware_validated=False and retry.
            self.get_logger().warning(
                f"firmware version pull unavailable ({msg}) — cannot validate yet",
                throttle_duration_sec=10.0)
            return
        try:
            per_axis = rpc_args.decode_axis_versions_result(blob)
            for axis, raw in per_axis.items():
                (_proto, hw_product, hw_ver, hw_variant,
                 fw_major, fw_minor, fw_rev, _unrel) = odrive.decode_get_version(raw)
                self._versions.record_version(
                    axis, (fw_major, fw_minor, fw_rev),
                    (hw_product, hw_ver, hw_variant))
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"version decode errored: {e}",
                                    throttle_duration_sec=5.0)
            return
        if not self._versions.all_jugglebot_versions_received():
            return  # firmware still sweeping — try again next tick
        # All present-and-expected Jugglebot axes reported — make a single
        # determination (can_node._handle_get_version:487-495 parity).
        error = self._versions.validate_group(odrive.JUGGLEBOT_AXES, "Jugglebot")
        with self._lock:
            if error:
                self._firmware_mismatch_error = error   # sticky; forces FAULT
            else:
                self._firmware_validated = True
        if error:
            self.get_logger().error(f"Jugglebot firmware check FAILED: {error}")
        else:
            self.get_logger().info(
                "Jugglebot firmware check PASSED — all axes match expected versions")

    # ═══════════════════════════════════════════════════════════
    # Setpoint downlink (Commit 3) — gated, default disabled
    # ═══════════════════════════════════════════════════════════

    def _set_mpc_active(self, active: bool):
        """Set the mpc_active state AND the J→T heartbeat flag together.

        This is the ONLY place mpc_active becomes 1. The heartbeat flag tells the
        Teensy the Jetson is driving setpoints (the firmware's guard-ENABLE
        precondition). Pinned to 0 unless ~enable_setpoint_output is true.
        """
        if active and not self._mpc_active:
            # Safety hardening: on the 0→1 re-enable edge, forget any stale
            # prior setpoint so the per-step gate starts fresh (the pump may hold a
            # _prev_pos from a prior session/pose). Same rationale as the reconnect
            # reset; the firmware MAX_DEVIATION gate is the complementary layer.
            self._sp_pump.reset()
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

    def _stop_setpoint_output(self):
        """Stop the setpoint thread, drop mpc_active on the wire, close the source.

        The disarm half of the runtime arming service. Mirrors on_shutdown's
        setpoint teardown (thread first, then flags=0 on the wire), but leaves the
        node otherwise live (no stow, no transport teardown) so the operator can
        re-arm later in the same session.
        """
        self._sp_stop.set()
        if self._sp_thread is not None and self._sp_thread.is_alive():
            self._sp_thread.join(timeout=1.0)
        self._sp_thread = None
        try:
            self._set_mpc_active(False)
        except Exception:  # noqa: BLE001
            self._mpc_active = False
        if self._sp_source is not None:
            # Ownership: an injected setpoint source belongs to the test fixture
            # (same convention as the injected client — see the __init__ docstring
            # around client ownership), so disarm must NOT close it; only close a
            # source the node itself opened.
            if self._sp_source is not self._injected_setpoint_source:
                try:
                    self._sp_source.close()
                except Exception:  # noqa: BLE001
                    pass
            self._sp_source = None

    def _svc_set_setpoint_output(self, request, response):
        """SetBool: arm (true) / disarm (false) the 40 Hz setpoint downlink."""
        if bool(request.data):
            ok, msg = self._arm_setpoint_output()
            response.success = ok
            response.message = msg
        else:
            self._stop_setpoint_output()
            response.success = True
            response.message = 'setpoint output disabled (mpc_active=0)'
        return response

    def _arm_setpoint_output(self) -> tuple:
        """Run the arming preconditions; on all-pass, stream-then-arm.

        Returns ``(ok, message)``. The message carries the reject reason on
        failure (surfaced in the service response AND logged) so an operator sees
        exactly which precondition blocked the arm.
        """
        if self._mpc_active:
            return True, 'setpoint output already enabled'

        # (a) Teensy link up + a fresh heartbeat (never arm onto a dead/silent link).
        age_us = self._link_age_us()
        if age_us is None or age_us > self._heartbeat_timeout_s * 1e6:
            return False, ('Teensy link down / no fresh heartbeat — refuse to arm '
                           '(never command a dead link)')
        if not self._link_latch.command_allowed():
            return False, 'Teensy link latched lost — refuse to arm'

        # (a2) The Teensy guard must not already be faulted. A latched E-STOP
        # silently DISCARDS leg output, so without this check the arm reports
        # "setpoint output ENABLED (armed)" onto a robot that cannot move — the
        # 40 Hz stream flows, setpoints_sent climbs, setpoints_rejected stays 0,
        # and nothing happens. That exact false-success cost a bench session on
        # 2026-07-09. The other preconditions cannot catch it: they check link
        # freshness, stream freshness, and u0-vs-encoder, none of which depend on
        # the guard latch.
        with self._lock:
            hb = self._latest_heartbeat
        if hb is not None and int(hb.fault_state) != int(FaultState.NONE):
            name = _enum_name(FaultState, int(hb.fault_state))
            return False, (
                f'Teensy guard fault latched (fault_state={name}) — refuse to arm; '
                f'leg output is suppressed until it is cleared. Recover with: '
                f'ros2 service call /clear_errors std_srvs/srv/Trigger')

        # (b) a fresh mpccmd frame on :5557 within 0.5 s (is trajectory_node up?).
        source, created_here = self._acquire_setpoint_source()
        if source is None:
            return False, 'could not open the :5557 setpoint source'
        try:
            frame = None
            deadline = time.time() + 0.5
            while time.time() < deadline:
                frame = source.recv_latest()
                if frame is not None:
                    break
                time.sleep(0.02)
            if frame is None:
                if created_here:
                    source.close()
                return False, ('no mpccmd frame on :5557 within 0.5 s — is '
                               'trajectory_node running and streaming?')

            # Derive u0 exactly as the production pump would (rejects a malformed
            # frame, and guarantees the frame we arm on is itself pump-acceptable).
            # torque_ff_enabled is mirrored so the ACCEPTANCE decision matches the
            # production pump's — with the FF on, a malformed torque_Nm is a reject,
            # and we must not arm on a frame the production pump would refuse.
            probe = SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV, num_legs=p.NUM_LEGS,
                                 torque_ff_enabled=bool(hw.DYNAMICS_TORQUE_FF_ENABLED))
            sp, reason = probe.build(frame, 0)
            if sp is None:
                if created_here:
                    source.close()
                return False, (f'mpccmd frame not pump-acceptable: '
                               f'{reason or "no position command"}')
            u0 = sp.u0

            # (c) u0 within _ARM_U0_TOL_REV of every leg's live pos_estimate.
            with self._lock:
                telem = self._latest_telemetry
            if telem is None:
                if created_here:
                    source.close()
                return False, 'no telemetry yet — cannot verify u0 vs encoder'
            ok, why = self._u0_within_encoder_tol(u0, telem.pos_rev,
                                                  _ARM_U0_TOL_REV)
            if not ok:
                if created_here:
                    source.close()
                return False, (f'{why} — non-finite or mismatched encoder — refuse '
                               'to arm (would risk MAX_DEVIATION E-STOP)')
        except Exception as e:  # noqa: BLE001 — arming must never crash the node
            if created_here:
                try:
                    source.close()
                except Exception:  # noqa: BLE001
                    pass
            return False, f'arming check error: {e}'

        # All preconditions met — stream-then-arm using the SAME source the check
        # observed (so the ingest thread continues from the confirmed stream).
        self._start_setpoint_output(source)
        return True, ('setpoint output ENABLED (armed): fresh stream confirmed, '
                      f'u0 within {_ARM_U0_TOL_REV} rev of every encoder')

    def _u0_within_encoder_tol(self, u0, pos_rev, tol) -> tuple:
        """Return ``(ok, reason)``: every leg's ``u0`` within ``tol`` rev of its
        encoder ``pos_rev``. NaN-safe — ``not (d <= tol)`` rejects a non-finite
        encoder (``d > tol`` would PASS a NaN, since every NaN comparison is False).
        Tolerance-agnostic, so it is shared by the arming pre-check (``_ARM_U0_TOL_REV``,
        0.25 rev) and /recover's convergence verify (``_RECOVER_U0_TOL_REV``, the tighter
        0.03 rev) — each passes its own ``tol`` (FIX 3)."""
        for i in range(p.NUM_LEGS):
            d = abs(float(u0[i]) - float(pos_rev[i]))
            if not (d <= tol):
                return False, (f'leg {i} u0 {float(u0[i]):.3f} rev vs encoder '
                               f'{float(pos_rev[i]):.3f} rev = {d:.3f} rev '
                               f'(> {tol} rev)')
        return True, ''

    def _verify_streamed_u0_converged(self, tol, timeout_s) -> tuple:
        """Poll until the streamed u0 is within ``tol`` rev of every live encoder, or
        ``timeout_s`` elapses. Returns ``(ok, reason)`` (FIX 3).

        This WAITS for trajectory_node's profiled descent to converge — u0 walks down
        onto the encoder over ~0.5-3 s (each 40 Hz knot inside the pump gate), so a
        single sample would race the still-descending command; the poll re-reads until
        it lands. Sources u0 from ``self._sp_pump._prev_pos`` — the exact LAST frame
        the bridge sent to the Teensy, which is what the firmware MAX_DEVIATION guard
        compares against — rather than re-reading the :5557 stream, so it never races
        the live setpoint-ingest thread during armed recovery. Reuses the same NaN-safe
        per-leg comparison as the arming pre-check (``_u0_within_encoder_tol``).
        """
        deadline = time.monotonic() + timeout_s
        last = 'no setpoint frame built yet'
        while True:
            u0 = self._sp_pump._prev_pos      # last accepted u0 (None until first send)
            with self._lock:
                telem = self._latest_telemetry
            if telem is None:
                last = 'no telemetry — cannot verify u0 vs encoder'
            elif u0 is None:
                last = 'no setpoint frame built yet (is the stream armed + flowing?)'
            else:
                ok, why = self._u0_within_encoder_tol(u0, telem.pos_rev, tol)
                if ok:
                    return True, ''
                last = why
            if time.monotonic() >= deadline:
                return False, last
            time.sleep(0.02)

    def _svc_recover(self, req, res):
        """``/recover`` (Trigger, FIX 3): one-call recovery from a latched Teensy guard.

        The 2026-07-10 stuttering-stroke runaway left the streamed u0 diverged
        ~0.5+ rev from the frozen encoder, so every bare /clear_errors re-latched
        MAX_DEVIATION within one 10 Hz fault tick. This does the recovery in the ONLY
        order that survives, WITHOUT disarming (mpc_active stays 1 — stopping the
        stream would trip MPC_STALE at 250 ms):
          1. ask trajectory_node to install a PROFILED DESCENT that walks the
             commanded u0 down onto the frozen encoder
             (``trajectory/reseed_from_measured``);
          2. WAIT (bounded) for that descent to converge — poll the pump's last-built
             u0 until it is within ``_RECOVER_U0_TOL_REV`` (0.03 rev) of every live
             encoder — so the clear cannot re-trip OR jolt at re-enable;
          3. if it plateaus above tol (the descent converged onto a STALE encoder
             snapshot while the leg drifted on during suppression — the Event-1
             −0.102 rev plateau), RE-INSTALL onto the now-current encoder and re-wait,
             up to ``_RECOVER_MAX_RESEED_ATTEMPTS`` times;
          4. only THEN fire CLEAR_ERRORS, and report precisely.
        If the reseed client is UNAVAILABLE (trajectory_node down), converge-first is
        IMPOSSIBLE — there is no node to install the descent, and with the setpoint
        source gone no fresh stream can jolt at re-enable — so it CLEARS DIRECTLY as an
        explicit escape hatch (with a loud WARN) rather than stranding the operator with
        a latched guard (F5). It still REFUSES (leaving the guard latched) when a live
        trajectory_node refuses/times out or the descent never converges — it NEVER
        clears onto a diverged command; those refusals state the manual disarm→clear
        recovery. Also the shared converge-first sequence the armed bare ``/clear_errors``
        reroutes through.
        """
        last_why = 'descent not yet attempted'
        for _attempt in range(_RECOVER_MAX_RESEED_ATTEMPTS):
            # 1. Ask trajectory_node to (re-)install a profiled descent onto the CURRENT
            #    measured encoder. Re-sampling each attempt is load-bearing: a descent
            #    onto a stale snapshot leaves u0 off the live (drifted) encoder; a fresh
            #    reseed chases the settling leg (see _RECOVER_MAX_RESEED_ATTEMPTS).
            if not self._reseed_client.service_is_ready():
                # trajectory_node (the setpoint source) is DOWN — converge-first cannot
                # help (nothing to install the descent, and no live stream to jolt at
                # re-enable). Refusing here would strand the operator with a latched guard
                # and no reachable converge path, so CLEAR DIRECTLY as the escape hatch,
                # with a loud WARN (F5). This is the raw-clear behaviour the pre-reroute
                # bare /clear_errors always had, now scoped to exactly the case where
                # converge-first is impossible.
                self.get_logger().warning(
                    'trajectory/reseed_from_measured UNAVAILABLE (trajectory_node down) '
                    '— converge-first impossible; clearing errors DIRECTLY (escape hatch)')
                cok, cmsg, _ = self.teensy_clear_errors()
                res.success = cok
                res.message = (
                    ('reseed unavailable (trajectory_node down) — cleared DIRECTLY '
                     f'(escape hatch): {cmsg}') if cok else
                    ('reseed unavailable AND direct CLEAR_ERRORS failed: '
                     f'{cmsg} — {_MANUAL_RECOVERY_HINT}'))
                return res
            try:
                future = self._reseed_client.call_async(Trigger.Request())
                deadline = time.monotonic() + _RECOVER_RESEED_TIMEOUT_S
                while not future.done() and time.monotonic() < deadline:
                    time.sleep(0.02)
                if not future.done():
                    res.success = False
                    res.message = ('trajectory_node reseed timed out '
                                   f'(> {_RECOVER_RESEED_TIMEOUT_S:.0f} s) — guard left '
                                   f'latched; {_MANUAL_RECOVERY_HINT}')
                    return res
                reseed = future.result()
            except Exception as e:  # noqa: BLE001 — recovery must never crash the node
                res.success = False
                res.message = (f'reseed call error: {e} — guard left latched; '
                               f'{_MANUAL_RECOVERY_HINT}')
                return res
            if not getattr(reseed, 'success', False):
                # A reseed refusal (stale telemetry / not streaming) will not be cured
                # by retrying — refuse now rather than burning the attempt budget.
                res.success = False
                res.message = (f'trajectory_node reseed refused: '
                               f'{getattr(reseed, "message", "unknown")} — guard left '
                               f'latched; {_MANUAL_RECOVERY_HINT}')
                return res

            # 2. WAIT for the profiled descent to walk u0 onto every LIVE encoder BEFORE
            #    clearing (it collapses over ~0.5-3 s, not in one frame).
            ok, last_why = self._verify_streamed_u0_converged(
                _RECOVER_U0_TOL_REV, self._recover_verify_timeout_s)
            if ok:
                break
            # 3. Plateaued off the live encoder — the leg drifted during the descent.
            #    Loop to re-descend onto the now-current encoder (bounded retries).
        else:
            res.success = False
            res.message = (
                f'reseed done but the profiled descent did not converge u0 onto the '
                f'encoder within {self._recover_verify_timeout_s:.1f}s × '
                f'{_RECOVER_MAX_RESEED_ATTEMPTS} attempts ({last_why}) — refusing to '
                f'clear (a clear now would re-latch); guard left latched; '
                f'{_MANUAL_RECOVERY_HINT}')
            return res

        # 4. Safe to clear now: u0 is within tol of every encoder.
        cok, cmsg, _ = self.teensy_clear_errors()
        if not cok:
            res.success = False
            res.message = f'reseed+converge OK but CLEAR_ERRORS failed: {cmsg}'
            return res
        res.success = True
        res.message = ('recovered: reseeded hold at measured, streamed u0 within '
                       f'{_RECOVER_U0_TOL_REV} rev of every encoder, CLEAR_ERRORS '
                       'fired')
        return res

    def _acquire_setpoint_source(self) -> tuple:
        """Return ``(source, created_here)`` for the arming check.

        Reuses an injected source (tests) or an already-open one; otherwise opens a
        fresh ``_MpcCommandSetpointSource`` (SUB on :5557), flagged so a failed arm
        closes it (no leak). A source opened here becomes the production ingest
        source only on a successful arm (``_start_setpoint_output`` adopts it).
        """
        if self._sp_source is not None:
            return self._sp_source, False
        if self._injected_setpoint_source is not None:
            return self._injected_setpoint_source, False
        try:
            return _MpcCommandSetpointSource(), True
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"could not open :5557 setpoint source: {e}")
            return None, False

    def _setpoint_loop(self):
        """Dedicated thread: drain the 40 Hz MPC command → pack Teensy-side knots → gate → send."""
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
                # Teensy interpolates to 500 Hz from the knots).
                self._sp_stop.wait(0.001)

    def _process_setpoint(self, cmd: dict):
        """Pack one 40 Hz MPC command into a Teensy-side knot Setpoint frame, gate it, send.

        SAFETY gates, in order:
          1. mpc_active must be set (operator opt-in). Belt-and-suspenders — the
             ingest thread only runs when enabled, but a direct caller is gated
             here too.
          2. Never command a dead link (the deferred-stow latch's command gate).
          3. The SetpointPump's per-step clamp + NaN/short-vector rejection.
        Only a clean, accepted frame is transmitted.

        FULLY EXCEPTION-CONTAINED (safety hardening): this runs on the dedicated
        'teensy_bridge_setpoint' daemon thread, so one malformed command or a
        transient send error (e.g. ENETUNREACH during the pre-latch window) must
        NOT kill the thread while mpc_active stays 1 — that would turn a one-frame
        problem into a restart-only outage of the production leg path with
        misleading telemetry. build() already REJECTS (not raises) malformed input;
        the send is OSError-guarded (mirroring the heartbeat thread); the outer
        guard is the backstop so no unexpected error escapes to the loop.
        """
        try:
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
            # Stash the ACCEPTED u0 for the leg_setpoint_echo GUI topic.
            # "Accepted" = passed the pump's validation + per-step gate (the
            # pump REJECTS, it never clamps — an accepted u0 is the exact
            # motor-rev vector packed into the frame). Stashed before the send
            # so a transient send error doesn't retract the echo (the link
            # watchdog owns real outages). HOT-PATH MINIMUM: three plain
            # assignments under the lock — message construction + publishing
            # happen on the executor timer (_publish_leg_setpoint_echo), never
            # on this thread.
            with self._lock:
                self._last_accepted_u0 = sp.u0
                self._last_accepted_u0_mono = time.monotonic()
                self._last_accepted_u0_seq += 1
            try:
                self._client.send_stream(int(MsgType.SETPOINT), sp.pack())
            except OSError as e:
                # Transient link error (ENETUNREACH/EHOSTUNREACH before the peer
                # is up, or a mid-run drop). The link watchdog owns the stow; here
                # we just drop this frame and keep the thread alive.
                self.get_logger().error(
                    f"Setpoint send failed (link down?): {e}",
                    throttle_duration_sec=1.0)
        except Exception as e:  # noqa: BLE001 — never let one frame kill the setpoint thread
            self.get_logger().error(
                f"Setpoint processing error (contained): {e}",
                throttle_duration_sec=1.0)

    def _guard_fault_leg_hint(self) -> str:
        """Best-effort ' (leg N)' / ' (legs N,M)' suffix naming the offending leg.

        HeartbeatT2J.fault_state carries the guard REASON but no axis, so the
        edge log names the leg from the latest per-axis Diagnostic cache: a leg
        (0..5) with a non-zero active error or disarm reason is the likely
        culprit for an ODRIVE_FATAL/MAX_DEVIATION/OVERSPEED latch. Returns ''
        when no leg-specific cause is identifiable (e.g. a bus-level
        CAN_BUS_DOWN or an MPC-staleness fault) so the message stays honest
        rather than guessing a leg.
        """
        with self._lock:
            diag = dict(self._latest_diag)
        culprits = []
        for axis in range(_NUM_LEGS):
            d = diag.get(axis)
            if d is not None and (int(d.active_errors) != 0
                                  or int(d.disarm_reason) != 0):
                culprits.append(axis)
        if not culprits:
            return ''
        if len(culprits) == 1:
            return f' (leg {culprits[0]})'
        return f' (legs {",".join(str(a) for a in culprits)})'

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

            # Announce guard fault-state EDGES on the node log. Until this landed the
            # firmware could latch an E-STOP in total silence: /link_status carried
            # fault_state but was not recorded in the rosbag, and nothing was logged,
            # so an operator saw only "the robot stopped responding" and DEACTIVATE
            # coming back ERR_BUS_DOWN. Edge-triggered (not per-tick) so a latched
            # fault does not spam the 5 Hz timer.
            if hb is not None:
                fs = int(hb.fault_state)
                if fs != self._last_fault_state:
                    if fs != int(FaultState.NONE):
                        leg_hint = self._guard_fault_leg_hint()
                        self.get_logger().error(
                            f'Teensy guard FAULT LATCHED: fault_state={teensy_fault}'
                            f'{leg_hint} — leg output is now SUPPRESSED and every '
                            f'leg command (incl. DEACTIVATE, which returns '
                            f'ERR_BUS_DOWN) will be refused. Recover with: '
                            f'ros2 service call /clear_errors std_srvs/srv/Trigger')
                        # Anchor the persistent-reminder cadence to this edge so the
                        # first repeat lands _GUARD_LATCH_REPEAT_S later, not on the
                        # very next 10 Hz tick.
                        self._last_guard_latch_log_t = time.monotonic()
                    elif self._last_fault_state is not None:
                        self.get_logger().info(
                            'Teensy guard fault cleared (fault_state=NONE) — leg '
                            'output re-enabled.')
                        self._last_guard_latch_log_t = None
                    self._last_fault_state = fs

            # Persistent reminder while the latch PERSISTS: the edge log above fires
            # once, so without this a latched E-STOP goes silent after one line. Emit
            # a short unmissable ERROR every _GUARD_LATCH_REPEAT_S until CLEAR_ERRORS.
            # Rate-EXACT off a monotonic timestamp (never throttle_duration_sec).
            if fault_active:
                now = time.monotonic()
                if (self._last_guard_latch_log_t is None
                        or now - self._last_guard_latch_log_t >= _GUARD_LATCH_REPEAT_S):
                    self._last_guard_latch_log_t = now
                    self.get_logger().error(
                        f'TEENSY GUARD LATCHED ({teensy_fault}) — leg output '
                        f'suppressed; CLEAR_ERRORS required')
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
                # cold-start cache visibility (the powered bring-up sitting
                # confirms the boot read landed and the right is_homed surfaced).
                KeyValue(key='cold_start_is_homed',
                         value=str(int(self._cold_start_state.is_homed))),
                KeyValue(key='cold_start_authoritative',
                         value=str(int(self._cold_start_authoritative))),
                KeyValue(key='setpoints_sent',
                         value=str(self._sp_pump.frames_built)),
                KeyValue(key='setpoints_rejected',
                         value=str(self._sp_pump.frames_rejected)),
                # Leg torque FF (default-off). torque_ff_ramp is the live 0→1 ramp
                # multiplier — the operator watches this climb during the first arming
                # (tests/hardware/session_torque_ff.md, step S3). It reads 0 whenever
                # the feature is off, so a 0 here on an armed robot means "no FF", full
                # stop, with no ambiguity.
                KeyValue(key='torque_ff_enabled',
                         value=str(int(self._sp_pump.torque_ff_enabled))),
                KeyValue(key='torque_ff_ramp',
                         value=f'{self._sp_pump.torque_ff_ramp_scale():.3f}'),
                KeyValue(key='setpoints_without_ff',
                         value=str(self._sp_pump.frames_without_ff)),
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
                    # Leg guard-deviation diagnostics (2026-07-10 forensics). Per-leg
                    # live deviation (u0-encoder, the MAX_DEVIATION guard quantity) +
                    # the lead-clamp bitmask, plus the frozen latch-event snapshot
                    # (which leg crossed + dev/u0/encoder at the trip). Recorded on
                    # /link_status so a future stutter/latch bag is self-diagnosing.
                    KeyValue(key='lead_clamp_mask',
                             value=str(int(hb.lead_clamp_mask))),
                    KeyValue(key='live_deviation',
                             value=','.join(f'{d:.4f}' for d in hb.live_deviation)),
                    KeyValue(key='max_dev_leg',
                             value=('none' if int(hb.max_dev_leg) == 0xFF
                                    else str(int(hb.max_dev_leg)))),
                    KeyValue(key='max_dev_value', value=f'{hb.max_dev_value:.4f}'),
                    KeyValue(key='max_dev_u0', value=f'{hb.max_dev_u0:.4f}'),
                    KeyValue(key='max_dev_enc', value=f'{hb.max_dev_enc:.4f}'),
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
            return False, self._annotate_rpc_error(str(e)), b""

    def _annotate_rpc_error(self, message: str) -> str:
        """Disambiguate ERR_BUS_DOWN, which the firmware overloads three ways.

        ``leg_deactivate.cpp:deactivate_allowed`` (and the activate/relay/hand
        analogues) return ERR_BUS_DOWN for a WARN/BUS_OFF bus, for
        ``fault_can_bus_down()``, AND for ``fault_guard_mode() == ESTOP`` — a
        latched guard on a perfectly healthy bus. The bare status therefore sends
        an operator hunting a CAN fault that does not exist (2026-07-09: a
        MAX_DEVIATION E-STOP surfaced only as "DEACTIVATE rejected:
        ERR_BUS_DOWN"). Append the live fault_state so the real cause is named at
        the point of failure.
        """
        if 'ERR_BUS_DOWN' not in message:
            return message
        with self._lock:
            hb = self._latest_heartbeat
        if hb is None:
            return message
        fs = int(hb.fault_state)
        if fs != int(FaultState.NONE):
            return (f'{message} — NOTE: ERR_BUS_DOWN also covers a latched guard '
                    f'E-STOP, and fault_state={_enum_name(FaultState, fs)} is '
                    f'currently latched. The bus is likely fine. Clear it with: '
                    f'ros2 service call /clear_errors std_srvs/srv/Trigger')
        return (f'{message} — guard is not faulted (fault_state=NONE), so this is a '
                f'genuine bus condition (CAN3 WARN/BUS_OFF or down)')

    # ── Tested node methods (one per RpcMethod) — the reusable surface ──
    # Arg encoding (rpc_args, codegen-hoisted) + RpcClient call. ROS service
    # wrappers for the arg-bearing per-axis ops await new .srv types; the
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

    def teensy_deactivate(self, axis=rpc_args.AXIS_ALL):
        return self._call_rpc(RpcMethod.DEACTIVATE, rpc_args.encode_deactivate(axis))

    def teensy_hand_traj_cmd(self, args: bytes):
        """HAND_TRAJ_CMD: forward a pre-built 8-byte 0x6D0 payload (from
        rpc_args.encode_hand_traj_cmd / encode_smooth_move_hand). The firmware sends
        the CLOSED_LOOP + POSITION/PASSTHROUGH preamble then forwards it on the
        firmware-owned 0x6D0 id (aborting if a preamble send fails)."""
        return self._call_rpc(RpcMethod.HAND_TRAJ_CMD, args)

    def teensy_sdo_read(self, axis, endpoint):
        return self._call_rpc(RpcMethod.SDO_READ,
                              rpc_args.encode_sdo_read(axis, endpoint))

    def teensy_sdo_write(self, axis, endpoint, value):
        return self._call_rpc(RpcMethod.SDO_WRITE,
                              rpc_args.encode_sdo_write(axis, endpoint, value))

    # ── Platform-Teensy relay ──
    # The relay reads TRIGGER a Platform-Teensy reply over CAN3 (0x7DE tilt /
    # 0x6E0 RobotState); the firmware forwards the reply verbatim as a
    # PLATFORM_FRAME the bridge correlates by (can_id, dlc). These methods are the
    # tested mechanism; the cold-start cache + orchestrator conduit source robot_state.is_homed/levelling/pose +
    # get_platform_tilt from them. NOTE(bench): the (can_id, dlc) discriminator is
    # only sound if CAN3 SRX_DIS is set so the bridge's own 0x6E0 STATE_WRITE is
    # not looped back as a reply, and the await timeout must exceed the measured
    # Platform reply latency — both gated on a bench probe before hardware trust.
    _RELAY_READ_TIMEOUT_S = 0.5

    # Cold-start boot read: a few bounded retries so a momentarily
    # not-yet-synced CAN3 / link gets a second chance before the conservative
    # is_homed=False fallback. Worst-case construction delay when the Platform is
    # unresponsive ≈ 3 × _RELAY_READ_TIMEOUT_S + 2 × _BOOT_STATE_READ_RETRY_S ≈
    # 1.9 s (each attempt's STATE_READ is ACKed fast but no PLATFORM_FRAME reply
    # arrives, so the await runs to timeout), and more if the UDP link itself is
    # down (each attempt then also hits the RpcClient timeout × retries). This runs
    # synchronously in __init__ before the executor spins; the bound is acceptable
    # for a deliberate cold-start step (the legs are not yet moving).
    _BOOT_STATE_READ_ATTEMPTS = 3
    _BOOT_STATE_READ_RETRY_S = 0.2
    # Reboot cold-start clear: the Platform Teensy stays powered through
    # an ODrive reboot, so a dropped clear would leave a STALE is_homed=True against
    # rebooted (de-referenced) ODrives — the dangerous direction. Retry the clear a
    # few times to harden that durability.
    _REBOOT_CLEAR_ATTEMPTS = 3
    _REBOOT_CLEAR_RETRY_S = 0.2

    def _await_platform_reply(self, can_id, expected_dlc, timeout):
        """Block (calling thread) for a fresh PLATFORM_FRAME on ``can_id`` with the
        expected dlc. Returns the raw reply bytes, or None on timeout. The caller
        clears the latch BEFORE sending the trigger RPC so a stale reply can't
        satisfy a new read."""
        deadline = time.monotonic() + timeout
        with self._platform_reply_cv:
            while True:
                entry = self._platform_replies.get(int(can_id))
                if entry is not None and entry[1] == expected_dlc:
                    return entry[0]
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._platform_reply_cv.wait(remaining)

    def relay_read_tilt(self, timeout=None):
        """TILT_READ: read the Platform-Teensy inclinometer. Returns
        (ok, message, (tiltX, tiltY)) in radians; tilt is None on failure."""
        timeout = self._RELAY_READ_TIMEOUT_S if timeout is None else timeout
        can_id = proto.CAN_ID_PLATFORM_TILT_READING
        with self._relay_lock:   # serialize the whole relay round-trip
            with self._platform_reply_cv:
                self._platform_replies.pop(int(can_id), None)
            ok, msg, _ = self._call_rpc(RpcMethod.TILT_READ)
            if not ok:
                return False, msg, None        # fail-fast (e.g. ERR_BUS_DOWN)
            data = self._await_platform_reply(can_id, expected_dlc=8, timeout=timeout)
            if data is None:
                return False, 'relay tilt read: no Platform reply within timeout', None
            tiltX, tiltY = struct.unpack('<ff', data[:8])
            return True, 'OK', (tiltX, tiltY)

    def relay_read_robot_state(self, timeout=None):
        """STATE_READ: read the Platform-Teensy RobotState. Returns
        (ok, message, RelayRobotState | None). Decodes the 0x6E0 reply exactly as
        Teensy_code.ino decodeStateCANMessage packs it."""
        timeout = self._RELAY_READ_TIMEOUT_S if timeout is None else timeout
        can_id = proto.CAN_ID_PLATFORM_STATE_UPDATE
        with self._relay_lock:   # serialize the whole relay round-trip
            with self._platform_reply_cv:
                self._platform_replies.pop(int(can_id), None)
            ok, msg, _ = self._call_rpc(RpcMethod.STATE_READ)
            if not ok:
                return False, msg, None        # fail-fast (e.g. ERR_BUS_DOWN)
            data = self._await_platform_reply(can_id, expected_dlc=8, timeout=timeout)
            if data is None:
                return False, 'relay state read: no Platform reply within timeout', None
            return True, 'OK', _decode_relay_robot_state(data)

    def relay_write_robot_state(self, is_homed, levelling_complete,
                                pose_offset_tiltX=0.0, pose_offset_tiltY=0.0):
        """STATE_WRITE: write the whole Platform-Teensy RobotState (the bridge is
        the sole writer; the firmware re-encodes the 0x6E0 frame). No reply —
        returns (ok, message) from the synchronous RPC ack. Serialized with the relay
        reads via _relay_lock so the STATE_WRITE read-modify-write
        cannot interleave with a concurrent STATE_READ (lost update / cache skew)."""
        with self._relay_lock:
            ok, msg, _ = self._call_rpc(
                RpcMethod.STATE_WRITE,
                rpc_args.encode_state_write(is_homed, levelling_complete,
                                            pose_offset_tiltX, pose_offset_tiltY))
            return ok, msg

    # ── Cold-start state cache: read at boot + on reconnect, write on home/reboot ──
    # The Platform Teensy owns the persisted cold-start state. These methods keep
    # self._cold_start_state in sync with it: a relay
    # read REFRESHES the cache (off the publish path); a relay write does
    # read-modify-write THROUGH the cache so a homing write preserves levelling +
    # pose, and vice versa. The 100 Hz publish path only reads the cache.

    def _refresh_cold_start_state(self, reason: str) -> bool:
        """Read the Platform Teensy RobotState via the relay and refresh the cache.

        Runs the relay read WITHOUT holding self._lock (it blocks for the CAN3
        round-trip), then swaps the new value into the cache under the lock. On
        success the cache + the authoritative flag are updated; on FAILURE the
        cache is LEFT UNCHANGED (keep the last authoritative value — can_node's
        passive "last-known-state until a fresh frame" semantics; never downgrade a
        good read on a transient hiccup). Returns True iff a fresh state was read.

        NOT for the boot path's first read — see _boot_read_cold_start_state, which
        applies the conservative is_homed=False fallback when there is no prior
        authoritative value to keep.
        """
        try:
            ok, msg, state = self.relay_read_robot_state()
        except Exception as e:  # noqa: BLE001 — never let a relay read crash a timer
            self.get_logger().error(
                f"cold-start read ({reason}) errored: {e}",
                throttle_duration_sec=5.0)
            return False
        if not ok or state is None:
            self.get_logger().warning(
                f"cold-start read ({reason}) failed: {msg} — keeping cached state",
                throttle_duration_sec=5.0)
            return False
        with self._lock:
            self._cold_start_state = state
            self._cold_start_authoritative = True
            if state.is_homed:
                # can_node.py:549-550 parity: homing requires a prior search, so a
                # read showing is_homed LATCHES encoder_search_complete True. Because
                # this bit is sticky (never cleared, incl. on reboot — can_node
                # leaves encoder_search_complete set), the DERIVED
                # encoder_search_complete = is_homed OR _encoder_search_done_session
                # is monotonic-True once homed/searched, exactly as can_node's field.
                self._encoder_search_done_session = True
        self.get_logger().info(
            f"cold-start state ({reason}): is_homed={int(state.is_homed)} "
            f"levelling={int(state.levelling_complete)} "
            f"pose=({state.pose_offset_tiltX:.4f},{state.pose_offset_tiltY:.4f})")
        return True

    def _boot_read_cold_start_state(self) -> bool:
        """Boot read: refresh the cold-start cache before the first publish, with a
        few bounded retries and the CONSERVATIVE-on-total-failure fallback. Returns
        True iff an authoritative read landed."""
        return self._read_cold_start_state_conservative('boot')

    def _read_cold_start_state_conservative(self, reason: str) -> bool:
        """Refresh the cold-start cache with bounded retries; on TOTAL failure fall
        back to the CONSERVATIVE cold value (is_homed=False / levelling=False /
        pose=0). Forcing a re-home is wasteful but SAFE; the reverse (a stale
        is_homed=True after the references may have been lost) could skip homing on
        an unhomed / de-referenced robot. Returns True iff an authoritative read
        landed.

        Used by the boot read (reason='boot') AND the CAN3-bus-health reconnect
        re-read (reason='can3_reconnect', firmware-validation precondition). NOTE the deliberate
        asymmetry vs _refresh_cold_start_state (the UDP-watchdog reconnect path),
        which KEEPS the stale cache on a failed read: a Jetson↔Teensy link blip does
        NOT imply the Platform Teensy lost power (its references are intact), so a
        re-home there would be wrong — can_node passive last-known parity. A CAN3
        bus-health recovery (WARN/BUS_OFF→OK) DOES imply the Jugglebot supply may
        have cycled (the Platform Teensy shares the ODrive supply), so a failed
        re-read must NOT keep a possibly-stale is_homed=True."""
        for attempt in range(self._BOOT_STATE_READ_ATTEMPTS):
            if self._refresh_cold_start_state(reason):
                return True
            if attempt + 1 < self._BOOT_STATE_READ_ATTEMPTS:
                time.sleep(self._BOOT_STATE_READ_RETRY_S)
        with self._lock:
            self._cold_start_state = RelayRobotState(
                is_homed=False, levelling_complete=False,
                pose_offset_tiltX=0.0, pose_offset_tiltY=0.0)
            self._cold_start_authoritative = False
        self.get_logger().warning(
            f"cold-start {reason} read failed after "
            f"{self._BOOT_STATE_READ_ATTEMPTS} attempts — defaulting to "
            "is_homed=False (conservative; forces a re-home).")
        return False

    def _dispatch_cold_start_reread(self, fn, reason: str, thread_name: str) -> bool:
        """Run a cold-start re-read (``fn(reason)``) OFF the calling timer thread.

        BOTH cold-start re-read triggers fire from the 1 Hz _health_check timer,
        which shares the node's default MutuallyExclusiveCallbackGroup with the
        100 Hz _publish_robot_state — so a synchronous relay round-trip would stall
        the robot_state stream (the CAN3 conservative re-read up to ~1.9 s: audit
        2026-06-29 MEDIUM; the same off-thread fix extends to the
        UDP-watchdog reconnect re-read, ~0.5 s single round-trip). Dispatch to a
        short-lived daemon thread: it touches only self._cold_start_state /
        _cold_start_authoritative / _encoder_search_done_session under self._lock,
        and the relay RPC + _await_platform_reply primitives are thread-safe (the
        RX thread already feeds them).

        A SINGLE _cold_start_reread_inflight guard keeps at most one re-read of
        EITHER kind running at once — this deliberately prevents the CONSERVATIVE
        (CAN3-recovery) and the KEEP-STALE (UDP-reconnect) reads from racing the
        same cache, where the weaker keep-stale read could land its is_homed=True
        write LAST and resurrect a stale reference after a power-cycle. Returns
        True iff a re-read was started; False iff one was already in flight. The
        CAN3 caller uses the return to AVOID consuming its recovery edge when it
        loses the shared guard, so the conservative re-read re-fires next tick
        rather than being permanently preempted by a keep-stale read."""
        with self._lock:
            if self._cold_start_reread_inflight:
                return False
            self._cold_start_reread_inflight = True

        def _run():
            try:
                fn(reason)
            except Exception as e:  # noqa: BLE001 — never let the worker thread die silently
                self.get_logger().error(
                    f"async cold-start re-read ({reason}) errored: {e}",
                    throttle_duration_sec=5.0)
            finally:
                with self._lock:
                    self._cold_start_reread_inflight = False

        threading.Thread(target=_run, daemon=True, name=thread_name).start()
        return True

    def _write_is_homed(self, is_homed: bool):
        """Persist is_homed via STATE_WRITE, read-modify-write THROUGH the cache so
        levelling_complete + pose_offset are preserved (the bridge is the sole
        writer — a homing write must not clobber a prior levelling result, and vice
        versa). Updates the cache on success. Returns (ok, message).

        The whole read→wire-write→cache-update runs under _relay_lock so a
        concurrent _write_level_state cannot read the same stale cache
        and land its wire write last, clobbering is_homed on the Platform Teensy —
        the RMW is atomic across writers, not just the two wire writes serialized."""
        with self._relay_lock:
            with self._lock:
                cs = self._cold_start_state
            ok, msg = self.relay_write_robot_state(
                is_homed=bool(is_homed),
                levelling_complete=cs.levelling_complete,
                pose_offset_tiltX=cs.pose_offset_tiltX,
                pose_offset_tiltY=cs.pose_offset_tiltY)
            if ok:
                with self._lock:
                    # _replace on the CURRENT cache (re-read) preserves any concurrent
                    # levelling update; only is_homed is changed here.
                    self._cold_start_state = self._cold_start_state._replace(
                        is_homed=bool(is_homed))
                    self._cold_start_authoritative = True
        return ok, msg

    def _write_level_state(self, levelling_complete, tiltX, tiltY):
        """Persist the levelling result (levelling_complete + pose_offset) via
        STATE_WRITE, read-modify-write THROUGH the cache so is_homed is preserved (the
        bridge is the sole writer — a levelling write must not clobber a prior homing
        result, and vice versa). The orchestrator set_level_state subscriber's persist
        path; byte-parity with can_node._sub_set_level_state → _update_teensy_state
        (which merged into last_known_state, so is_homed carried through). Updates the
        cache on success. Returns ``(ok, message)``.

        Whole RMW under _relay_lock — see _write_is_homed; the
        symmetric case is a concurrent homing write being clobbered here."""
        with self._relay_lock:
            with self._lock:
                cs = self._cold_start_state
            ok, msg = self.relay_write_robot_state(
                is_homed=cs.is_homed,                 # preserve (read-modify-write)
                levelling_complete=bool(levelling_complete),
                pose_offset_tiltX=float(tiltX),
                pose_offset_tiltY=float(tiltY))
            if ok:
                with self._lock:
                    # _replace on the CURRENT cache (re-read) preserves any concurrent
                    # is_homed update; only levelling + pose are changed here.
                    self._cold_start_state = self._cold_start_state._replace(
                        levelling_complete=bool(levelling_complete),
                        pose_offset_tiltX=float(tiltX),
                        pose_offset_tiltY=float(tiltY))
                    self._cold_start_authoritative = True
        return ok, msg

    def _clear_cold_start_state_on_reboot(self):
        """REBOOT_ODRIVES shared-hook step 2 (cold-start clear): clear is_homed +
        levelling_complete + pose_offset on the Platform Teensy — the ODrives lose
        their references on reboot, so all three are cleared together (mirrors
        can_node.py:1559-1565). Retried (the Platform Teensy stays powered through
        the reboot, so a dropped clear would leave a dangerous stale is_homed=True).
        The cache is set to the cleared value regardless (the safe local view).
        Returns (ok, message).

        The retried wire clears + the cache reset run under _relay_lock so a
        concurrent homing/levelling write cannot land between a
        successful clear and the cache reset and resurrect a stale is_homed=True."""
        ok, msg = False, 'no attempt'
        with self._relay_lock:
            for attempt in range(self._REBOOT_CLEAR_ATTEMPTS):
                ok, msg = self.relay_write_robot_state(
                    is_homed=False, levelling_complete=False,
                    pose_offset_tiltX=0.0, pose_offset_tiltY=0.0)
                if ok:
                    break
                if attempt + 1 < self._REBOOT_CLEAR_ATTEMPTS:
                    time.sleep(self._REBOOT_CLEAR_RETRY_S)
            with self._lock:
                self._cold_start_state = RelayRobotState(
                    is_homed=False, levelling_complete=False,
                    pose_offset_tiltX=0.0, pose_offset_tiltY=0.0)
                self._cold_start_authoritative = True
                # Clear the in-session encoder-search bit too: a reboot resets the ODrive
                # MCUs, which lose their INCREMENTAL-ENCODER INDEX (not pre-calibrated to
                # flash — operator-confirmed 2026-07-02), so a re-encoder-search is
                # REQUIRED before the next home. The derived
                #   encoder_search_complete = is_homed OR _encoder_search_done_session
                # must therefore go False after a reboot (is_homed is already cleared
                # above). can_node.py:1552-1566 did NOT clear this — a LATENT BUG that only
                # bit once the orchestrator drove homing AUTOMATICALLY after a reboot
                # (orchestrator-driven auto-home): HomingHandler skipped encoder-search on the stale True and
                # homed on an un-indexed encoder → ODRIVE_FATAL → FAULT→BOOT→HOMING loop
                # (hardware, 2026-07-02; see logbook 2026-07-02-canbridge-reboot-encoder-
                # search-clear). Clearing it re-runs encoder-search on the next cold-start,
                # exactly as a fresh launch does. Deliberate divergence from can_node's
                # literal behaviour: the reboot must clear EVERYTHING the ODrive loses —
                # references (is_homed/levelling/pose) AND the encoder index.
                self._encoder_search_done_session = False
        return ok, msg

    # ── Encoder index search (Jetson-side orchestration) ──
    # The firmware ENCODER_SEARCH RPC is stubbed (ERR_NOT_IMPL); encoder index
    # search is an ODrive-autonomous axis state, so we orchestrate it from here
    # over the implemented SET_AXIS_STATE primitive + the telemetry/diagnostic
    # cache. The pure sequencing lives in controller/teensy_link/encoder_search.py
    # (unit-tested); this method is the I/O loop around it. Homing (the firmware move) is
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

    # ── Homing (firmware move, Jetson-side observation) ──
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
            active_errors=int(d.active_errors),
            homing_result=int(d.homing_result))   # authoritative outcome from the HomingResult uplink

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
        # Per-axis observer timeout BACKSTOP. MUST sit BELOW the firmware per-axis
        # hard timeout (Homing::MOTOR_TIMEOUT_S = 30 s). Since the HomingResult uplink
        # landed (see logbook/2026-07-05-canhub-hardening-18a-homing-result-uplink.md),
        # the observer trusts the firmware's uplinked HomingResult, so a firmware abort —
        # including its own 30 s timeout — is reported as HOMING_FAILED directly; this
        # host timeout only catches a leg that never resolves at all (e.g. a lost
        # RUNNING→terminal transition). A real home completes in a few seconds, so 20 s
        # is generous and still below the firmware's 30 s.
        timeout_s = min(20.0, float(hw.HOMING_MOTOR_TIMEOUT_S) - 5.0)
        succeeded, failed = [], {}
        # The firmware briefly rejects a HOME (ERR_REJECTED, busy) while it finishes
        # the PREVIOUS axis: axis_state reads IDLE during STOP_SETTLE — before
        # set_absolute_position completes and s_phase returns to IDLE (~10-20 ms) —
        # and the observer (no longer gating on position; the legs relax off a foam
        # stop, 2026-06-26) can fire the next axis's HOME inside that window. Retry
        # across ticks until the firmware accepts it (the principled form of the
        # post-TX delays can_node used). Bounded so a genuinely-stuck axis fails.
        _MAX_REJECT_RETRIES = 20   # × poll_dt ≈ 1 s; busy window is ~10-20 ms
        self.get_logger().info(f"homing: starting on axes {axes} (sequential)")
        for axis in axes:
            is_hand = (axis == _HAND_AXIS)
            home_ref = abs(float(
                hw.HOMING_HAND_ABS_POS_REV if is_hand else hw.HOMING_LEG_ABS_POS_REV))
            # The hand (axis 6) homes with the SAME firmware move-to-hardstop
            # (Homing::HAND_* params) but its PID gains must be applied FIRST —
            # byte-identical to can_node._home_robot_steps' HAND branch
            # (_set_hand_gains() then the move). Refuse-flash-defaults: abort the
            # sequence if a gain write fails rather than drive the hand into a
            # hardstop on flash-default gains (can_node._set_hand_gains raised).
            if is_hand:
                gok, gmsg = self._apply_hand_gains()
                if not gok:
                    failed[axis] = gmsg
                    self.get_logger().error(f"homing: hand gain apply failed — {gmsg}")
                    break
            mon = HomingMonitor([axis], home_ref_rev=home_ref, timeout_s=timeout_s)
            hard_deadline = time.monotonic() + timeout_s + 5.0
            home_accepted = False     # firmware has ACCEPTED the HOME for this axis
            reject_retries = 0
            while True:
                now = time.monotonic()
                st = self._homing_axis_status(axis)
                res = mon.step(now, {axis: st} if st is not None else {})
                # Fire on the monitor's request, then keep re-firing until accepted.
                if res.set_home or not home_accepted:
                    ok, msg, _ = self.teensy_home(axis)
                    if ok:
                        home_accepted = True
                    elif 'REJECTED' in msg.upper() and reject_retries < _MAX_REJECT_RETRIES:
                        reject_retries += 1   # transient busy — retry next tick
                    else:
                        # Non-transient failure (bus down / bad axis), or retries
                        # exhausted (firmware stuck busy).
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

    def _apply_hand_gains(self):
        """Apply the hand PID gains to axis 6 (before HOME(6) and in _run_configure).
        Byte-identical to can_node._set_hand_gains: SET_POS_GAIN then SET_VEL_GAINS
        from self._hand_gains. Refuse-flash-defaults — returns ``(False, reason)`` if
        a gain RPC fails, so the caller aborts rather than home/configure the hand on
        flash-default gains (the safety can_node held; _set_hand_gains raised there).
        Returns ``(ok, message)``."""
        g = self._hand_gains
        ok1, m1, _ = self.teensy_set_pos_gain(_HAND_AXIS, g['pos_gain'])
        ok2, m2, _ = self.teensy_set_vel_gains(_HAND_AXIS, g['vel_gain'], g['vel_int_gain'])
        if ok1 and ok2:
            return True, "hand gains applied"
        failed = [nm for ok, nm in ((ok1, 'hand.pos_gain'), (ok2, 'hand.vel_gains')) if not ok]
        return False, ("hand gain write failed on CAN: " + ", ".join(failed)
                       + " — hand may be on flash defaults")

    # ── Configure + activate (Teensy-side cold-start) ──
    # The Teensy-side path has only one leg-motion path (the gated 40 Hz setpoint stream),
    # so the can_node `_setup_odrives_steps` (gains/limits/mode) and
    # `_gentle_move_steps` (move to active pose) have no equivalent until here.
    # `_run_configure` is the pure-config _setup_odrives analogue; `_run_activate`
    # fires the firmware TRAP_TRAJ activate op + observes it. `/home` calls
    # `_run_configure` at completion (the operator's "set after every homing").

    def _run_configure(self, axes):
        """Apply the Teensy-side cold-start config to ``axes`` (the _setup_odrives
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
        leg_axes = [a for a in axes if a < p.NUM_LEGS]
        do_hand = _HAND_AXIS in axes
        ctrl = proto.ODRIVE_CONTROL_MODES['POSITION']
        inp = proto.ODRIVE_INPUT_MODES['PASSTHROUGH']
        failed = []
        self.get_logger().info(f"configure: applying gains/limits/PASSTHROUGH on axes {axes}")
        for axis in leg_axes:
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
        if do_hand:
            # Configure the HAND (axis 6) — the hand half of can_node's cold-start
            # _setup_odrives (per the can_node<->Teensy parity audit): hand PID gains
            # (refuse-flash-defaults) + hand vel/curr limits + POSITION/PASSTHROUGH. Gains + limits
            # come from self._hand_gains / self._hand_vel_limit / self._hand_curr_limit
            # (config defaults, updatable via set_hand_gains / set_motor_vel_curr_limits).
            gok, gmsg = self._apply_hand_gains()
            if not gok:
                failed.append(f"axis {_HAND_AXIS} {gmsg}")
            for name, (ok, m, _) in (
                ("vel_curr_limits",
                 self.teensy_set_vel_curr_limits(_HAND_AXIS, self._hand_vel_limit,
                                                 self._hand_curr_limit)),
                ("controller_mode",
                 self.teensy_set_controller_mode(_HAND_AXIS, ctrl, inp)),
            ):
                if not ok:
                    failed.append(f"axis {_HAND_AXIS} {name}: {m}")
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

    def _deactivate_axis_status(self, axes):
        """Build per-axis ``DeactivateAxisStatus`` for the deactivate observer from
        the latest telemetry (pos/vel) + diagnostic (axis_state, active_errors)
        cache. An axis with no diagnostic yet is omitted (aged toward its timeout)."""
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
            out[int(axis)] = DeactivateAxisStatus(
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

    def _legs_already_stowed(self, axes):
        """True iff every leg in ``axes`` is already IDLE and within
        ``_STOW_POS_MAX_REV`` of the STOW pose (``|pos| <= that``; STOW = 0.0 rev, the
        DeactivateMonitor convention) — i.e. the platform is already stowed, so a
        DEACTIVATE would be a redundant re-arm+lower. Fail-SAFE: a missing telemetry
        or per-axis diagnostic (cannot confirm) returns False, so the deactivate
        proceeds normally rather than silently skipping a real lower."""
        with self._lock:
            telem = self._latest_telemetry
            diag = dict(self._latest_diag)
        if telem is None:
            return False
        for axis in axes:
            d = diag.get(int(axis))
            if d is None:
                return False                                          # no diag → can't confirm
            if int(d.axis_state) != proto.ODRIVE_STATES['IDLE']:
                return False                                          # a leg is not IDLE
            pos = float(telem.pos_rev[int(axis)])
            # NaN-safe: pos_rev is NaN until the encoder is ready / after an encoder
            # fault (a faulted active leg drops to IDLE reporting NaN). `abs(nan) > x`
            # is False, so a bare `> _STOW_POS_MAX_REV` would MISCLASSIFY a NaN leg as
            # stowed and skip a needed lower — leaving the platform UP. Treat non-finite
            # as "cannot confirm" → not stowed (fail-safe), mirroring _encoder_search_axis_status.
            if not math.isfinite(pos) or abs(pos) > _STOW_POS_MAX_REV:
                return False                                          # extended or unknown
        return True

    def _run_deactivate(self, axes, *, poll_dt=0.05, timeout_s=None):
        """Fire the firmware DEACTIVATE (TRAP_TRAJ controlled lower to STOW, then
        IDLE) and observe it to completion. A single configured axis fires that
        leg; any larger set fires ``AXIS_ALL`` (every PRESENT leg, parallel
        even-descent). The DeactivateMonitor watches each leg reach IDLE (the
        firmware's clean completion — not the foam-unreliable resting position).

        Synchronous: blocks the calling (executor) thread for the ~seconds the
        descent takes; the RX + heartbeat threads keep the link + telemetry cache
        alive and the heartbeat stays ``mpc_active=0`` (no setpoint output).
        Returns ``(ok, message)``.

        ``timeout_s`` overrides the DeactivateMonitor per-run budget (default None →
        the monitor's own default). The shutdown-stow path passes a tighter budget
        so a stalled descent can't blow the ~8 s teardown window.

        Precondition: the legs are holding the active pose in CLOSED_LOOP (a prior
        /activate). Without it the move fails safe (an errored or absent leg is
        rejected by the firmware; a stalled leg → DeactivateMonitor timeout).
        """
        axes = [int(a) for a in axes]
        if not axes:
            return False, "no axes configured for deactivate"
        # Already-at-STOW no-op guard: if every target leg is already IDLE and within
        # _STOW_POS_MAX_REV of STOW, the platform is stowed — a DEACTIVATE would
        # needlessly re-arm CLOSED_LOOP (a jolt) only to lower nothing, then IDLE. Skip
        # it and report success. (This also short-circuits a redundant stow-on-shutdown
        # when the platform was already brought down.)
        if self._legs_already_stowed(axes):
            self.get_logger().info(
                f"deactivate: axes {axes} already at STOW (IDLE, |pos| <= "
                f"{_STOW_POS_MAX_REV} rev) — skipping the leg move")
            # Only the redundant LEG re-arm+lower is skipped; DEACTIVATE still
            # de-energises the HAND (axis 6) to preserve can_node's "idle ALL
            # JUGGLEBOT_AXES" parity (so e.g. a stow-on-shutdown with legs already down
            # but the hand armed still drops the hand). Best-effort, as in the full path.
            hok, hmsg, _ = self.teensy_set_axis_state(_HAND_AXIS, proto.ODRIVE_STATES['IDLE'])
            if not hok:
                self.get_logger().warning(
                    f"deactivate (already stowed): hand IDLE failed — {hmsg}")
            return True, f"already at STOW (axes {axes})"
        # Single configured axis → that leg only; otherwise AXIS_ALL (all present
        # legs in parallel — the platform lowers straight down, no tilt).
        fire_axis = axes[0] if len(axes) == 1 else rpc_args.AXIS_ALL
        # Footgun guard: a partial multi-leg subset fires AXIS_ALL (every PRESENT
        # leg), so legs NOT in deactivate_axes are still driven but unobserved.
        if len(axes) > 1 and set(axes) != set(range(p.NUM_LEGS)):
            self.get_logger().warning(
                f"deactivate_axes={axes} is a partial subset — DEACTIVATE(AXIS_ALL) "
                f"will move ALL present legs, but only {axes} are observed. Use a "
                f"single leg or the full leg set.")
        ok, msg, _ = self.teensy_deactivate(fire_axis)
        if not ok:
            return False, f"DEACTIVATE rejected: {msg}"
        mon = (DeactivateMonitor(axes, timeout_s=float(timeout_s))
               if timeout_s is not None else DeactivateMonitor(axes))
        hard_deadline = time.monotonic() + mon.timeout_s + 5.0
        self.get_logger().info(
            f"deactivate: TRAP_TRAJ lower to STOW + IDLE on axes {axes}")
        while True:
            now = time.monotonic()
            res = mon.step(now, self._deactivate_axis_status(axes))
            if res.done:
                break
            if now > hard_deadline:
                self.get_logger().error("deactivate: hard deadline exceeded")
                break
            time.sleep(poll_dt)
        # Idle the HAND (axis 6) too: can_node's deactivate de-energised ALL
        # JUGGLEBOT_AXES (legs via the TRAP_TRAJ→IDLE above; hand via
        # SET_AXIS_STATE(IDLE), can_node.py:1541-1543). ACTIVATE/DEACTIVATE are
        # rejected on axis 6 (leg-specific cold-start moves), so the hand is idled
        # explicitly here. Best-effort — a failure is logged but does not fail the
        # leg deactivate result (per the can_node<->Teensy parity audit).
        hok, hmsg, _ = self.teensy_set_axis_state(_HAND_AXIS, proto.ODRIVE_STATES['IDLE'])
        if not hok:
            self.get_logger().warning(f"deactivate: hand IDLE failed — {hmsg}")
        if mon.failed:
            parts = [f"axis {a}: {r}" for a, r in mon.failed.items()]
            msg = "deactivate FAILED — " + "; ".join(parts)
            if mon.succeeded:
                msg += f" (succeeded: {mon.succeeded})"
            self.get_logger().error(msg)
            return False, msg
        msg = f"deactivate complete on axes {mon.succeeded}"
        self.get_logger().info(msg)
        return True, msg

    # ── ROS service handlers (existing-type subset) ───────────

    def _svc_clear_errors(self, req, res):
        # 2026-07-11 clear-errors jolt: WHILE ARMED (mpc_active=1) the guard is actively
        # gating leg output, so a bare clear onto a diverged command re-enables output
        # with u0 off the drifted encoder → the pos_gain × lead velocity kick to the
        # current rail. Route the armed bare /clear_errors through the SAME converge-first
        # sequence as /recover (reseed → verify u0 on the encoder → clear) so no clear can
        # re-trip or jolt — there is no raw armed escape hatch (operator decision).
        # WHEN NOT ARMED (mpc_active=0 — e.g. the benign boot-time MPC_STALE latch: output
        # is not being evaluated, no setpoint is streaming), there is nothing to converge
        # and no jolt is possible, so clear DIRECTLY as before (a reseed would refuse —
        # not streaming — and needlessly block the clear).
        if self._mpc_active:
            return self._svc_recover(req, res)
        ok, msg, _ = self.teensy_clear_errors()
        res.success = ok
        res.message = msg
        return res

    def _reboot_odrives(self):
        """REBOOT_ODRIVES shared ordered hook (mirrors can_node._reboot_odrives_
        steps). Step 1 — the bounded watchdog-suppression latch — is owned elsewhere
        (see logbook/2026-06-30-canbridge-phase6-reboot-latch.md) and out of scope
        here. Step 2 (cold-start clear): after firing the reboot, clear the
        persisted cold-start state on the Platform Teensy (the ODrives lose their
        references on reboot). Returns (ok, message)."""
        ok, msg, _ = self.teensy_reboot()
        # Step 2: clear is_homed/levelling/pose together. Unconditional on the
        # reboot result (parity with can_node, which always cleared) — clearing is
        # the SAFE direction (a re-home), never the reverse.
        cs_ok, cs_msg = self._clear_cold_start_state_on_reboot()
        if not cs_ok:
            self.get_logger().error(
                f"reboot: clearing cold-start state failed: {cs_msg}")
            msg = f"{msg}; WARNING cold-start clear failed: {cs_msg}"
        return ok, msg

    def _svc_reboot_odrives(self, req, res):
        ok, msg = self._reboot_odrives()
        res.success = ok
        res.message = msg
        return res

    def _svc_encoder_search(self, req, res):
        # Jetson-side orchestration over SET_AXIS_STATE (the firmware
        # ENCODER_SEARCH RPC remains stubbed). Scope via the encoder_search_axes
        # parameter (default all legs; [0] for the standalone-leg bench rig).
        axes = list(self.get_parameter('encoder_search_axes').value or [])
        ok, msg = self._run_encoder_search(axes)
        if ok:
            # Cold-start state: track the in-session search-done bit so the DERIVED
            # encoder_search_complete = is_homed OR within-session-search-done
            # (can_node parity — see _publish_robot_state). Sticky-True for the run.
            with self._lock:
                self._encoder_search_done_session = True
        res.success = ok
        res.message = msg
        return res

    def _do_home(self):
        """Home the configured axes (legs + hand), persist the is_homed RESULT, and
        (on success) apply the Teensy-side cold-start config — the shared body behind BOTH
        the manual ``/home`` Trigger service AND the orchestrator-facing
        ``home_motors`` action. One definition, so the two entry points can
        never drift. Returns ``(ok, message)``.

        Scope via the ``home_axes`` parameter (default all legs + the hand; ``[0]``
        for the standalone-leg bench rig). The firmware HOME handler runs the
        velocity-limited move-to-hardstop autonomously; ``_run_home`` drives +
        observes it (applying the hand gains before HOME(6)).

        Persisting is_homed = the home RESULT (read-modify-write through the cache so
        levelling/pose are preserved) is the safety crux, EXACT can_node.py:847 parity
        (``_update_teensy_state({'is_homed': success})`` writes ``success``, i.e.
        False on a FAILED home too): a failed re-home of an already-homed robot MUST
        clear is_homed, or a stale is_homed=True would let the orchestrator skip
        homing on an unhomed robot (state_machine.py:238-239). A failed persist
        surfaces a warning but does not change the home result (a missed persist →
        next boot re-homes, the SAFE direction). On success, re-apply the cold-start
        config (gains/limits may have been changed in/since a prior session), scoped
        to the homed axes; a configure failure surfaces but does not undo homing.

        SINGLE-FLIGHT GUARDED HERE (the real enforcement point): the cold-start
        ReentrantCallbackGroup lets the /home Trigger service AND the home_motors
        action reach this concurrently, and two _run_home sequences would interleave
        HOME RPCs + race _write_is_homed. Guarding _do_home itself (not just
        _home_action_goal) covers action-vs-action, action-vs-/home, AND /home-vs-
        /home; a concurrent caller gets (False, "home already in progress").
        """
        with self._home_lock:
            if self._home_in_progress:
                return False, "home already in progress"
            self._home_in_progress = True
        try:
            axes = list(self.get_parameter('home_axes').value or [])
            ok, msg = self._run_home(axes)
            w_ok, w_msg = self._write_is_homed(ok)
            if not w_ok:
                self.get_logger().error(
                    f"home: persisting is_homed={int(ok)} failed: {w_msg}")
                msg = f"{msg}; WARNING persist is_homed failed: {w_msg}"
            if ok:
                cfg_ok, cfg_msg = self._run_configure(axes)
                ok = ok and cfg_ok
                msg = f"{msg}; {cfg_msg}"
            return ok, msg
        finally:
            with self._home_lock:
                self._home_in_progress = False

    def _svc_home(self, req, res):
        # the manual /home Trigger service shares _do_home() with
        # the orchestrator's home_motors action — one home+persist+configure body.
        ok, msg = self._do_home()
        res.success = ok
        res.message = msg
        return res

    def _svc_configure(self, req, res):
        # apply the Teensy-side cold-start config (gains + vel/curr limits
        # + POSITION/PASSTHROUGH) to the configure_axes. Run after homing (auto via
        # /home) and again before arming / after activate (TRAP_TRAJ → PASSTHROUGH).
        axes = list(self.get_parameter('configure_axes').value or [])
        ok, msg = self._run_configure(axes)
        res.success = ok
        res.message = msg
        return res

    def _svc_activate(self, req, res):
        # fire the firmware TRAP_TRAJ move to the active pose +
        # observe completion. Scope via activate_axes (default all legs; [0] for
        # the standalone-leg bench rig). Precondition: a prior /configure.
        axes = list(self.get_parameter('activate_axes').value or [])
        ok, msg = self._run_activate(axes)
        res.success = ok
        res.message = msg
        return res

    def _svc_deactivate(self, req, res):
        # fire the firmware TRAP_TRAJ controlled lower to the STOW
        # pose + IDLE on arrival (the Teensy-side analogue of can_node deactivate),
        # then observe completion. Scope via deactivate_axes (default all legs;
        # [0] for the standalone-leg bench rig). Precondition: legs at the active
        # pose (a prior /activate).
        axes = list(self.get_parameter('deactivate_axes').value or [])
        ok, msg = self._run_deactivate(axes)
        res.success = ok
        res.message = msg
        return res

    # ── Hand command surface ──────────────────────────────────────
    # Byte-identical ports of can_node's hand services (can_node.py:782-829,
    # 1626-1661). Jetson-side validation matches can_node exactly (reject invalid
    # before a byte hits CAN3); the RPCs ride the relay-seam axis-6 allow-table
    # (state/gains) or the HAND_TRAJ_CMD 0x6D0 conduit (traj/smooth-move).

    def _svc_set_hand_state(self, req, res):
        # Set the hand ODrive axis state (IDLE / CLOSED_LOOP) directly on axis 6.
        # Reject an UNKNOWN state string Jetson-side: can_node passed the string to
        # encode_set_state (which KeyErrors on a bad key → caught as failure); we do
        # the same map + reject explicitly. SET_AXIS_STATE on axis 6 rides the
        # relay-seam allow-table.
        try:
            state = str(req.data)
            if state not in proto.ODRIVE_STATES:
                res.success = False
                res.message = f"Unknown hand state '{state}'"
                return res
            ok, m, _ = self.teensy_set_axis_state(_HAND_AXIS, proto.ODRIVE_STATES[state])
            res.success = ok
            res.message = f"Hand state set to {state}" if ok else m
        except Exception as e:  # noqa: BLE001
            res.success = False
            res.message = str(e)
        return res

    def _svc_set_hand_traj(self, req, res):
        # Arm a timed catch trajectory (SetHandTrajCmd: event_delay, event_vel,
        # traj_type). Validate exactly as can_node._send_hand_traj_cmd, compute the
        # ABSOLUTE wall_time_ms Jetson-side (now + event_delay), forward the 8-byte
        # 0x6D0 payload via HAND_TRAJ_CMD (the firmware sends the CLOSED_LOOP +
        # POSITION/PASSTHROUGH preamble then forwards it, aborting on a preamble
        # failure). The firmware does NOT re-stamp the deadline — an absolute
        # deadline is immune to Jetson→bridge→CAN3 transit jitter.
        try:
            event_delay = float(req.event_delay)
            event_vel = float(req.event_vel)
            traj_type = int(req.traj_type)
            if event_delay <= 0:
                raise ValueError(f"Invalid event delay: {event_delay}")
            if (event_vel < hw.TEENSY_TRAJ_MIN_EVENT_VEL_MPS
                    or event_vel > hw.TEENSY_TRAJ_MAX_EVENT_VEL_MPS):
                raise ValueError(f"Invalid event velocity: {event_vel}")
            if traj_type not in (0, 1, 2):
                raise ValueError(f"Invalid trajectory type: {traj_type}")
            wall_time_ms = int(time.time() * 1000) + int(event_delay * 1000)
            args = rpc_args.encode_hand_traj_cmd(traj_type, event_vel, wall_time_ms)
            ok, m, _ = self.teensy_hand_traj_cmd(args)
            res.success = ok
            res.message = "Hand trajectory set." if ok else m
        except Exception as e:  # noqa: BLE001
            res.success = False
            res.message = str(e)
        return res

    def _svc_smooth_move_hand(self, req, res):
        # Prime the hand to a target rev (SetFloat: data). Validate range exactly as
        # can_node._smooth_move_hand, forward the discriminator-3 0x6D0 payload.
        try:
            target_rev = float(req.data)
            if target_rev < 0 or target_rev > odrive.HAND_MOTOR_MAX_POSITION:
                raise ValueError(f"Invalid target: {target_rev:.3f} rev")
            args = rpc_args.encode_smooth_move_hand(target_rev)
            ok, m, _ = self.teensy_hand_traj_cmd(args)
            res.success = ok
            res.message = f"Smooth-move hand to {target_rev:.3f} rev" if ok else m
        except Exception as e:  # noqa: BLE001
            res.success = False
            res.message = str(e)
        return res

    def _svc_set_hand_gains(self, req, res):
        # Set hand PID gains (SetHandGains: pos_gain, vel_gain, vel_integrator_gain)
        # + remember them (the cold-start hand config/home path reapplies them).
        # Mirrors can_node._svc_set_hand_gains (SET_POS_GAIN then SET_VEL_GAINS on
        # axis 6, both riding the relay-seam allow-table), with ONE intentional
        # hardening: the remembered self._hand_gains is updated only on a successful
        # write (can_node cached unconditionally). This never caches gains that
        # failed to reach the ODrive — otherwise _apply_hand_gains would later
        # re-send stale values as if applied.
        try:
            pos_gain = float(req.pos_gain)
            vel_gain = float(req.vel_gain)
            vel_int_gain = float(req.vel_integrator_gain)
            ok1, m1, _ = self.teensy_set_pos_gain(_HAND_AXIS, pos_gain)
            ok2, m2, _ = self.teensy_set_vel_gains(_HAND_AXIS, vel_gain, vel_int_gain)
            if ok1 and ok2:
                self._hand_gains = {'pos_gain': pos_gain, 'vel_gain': vel_gain,
                                    'vel_int_gain': vel_int_gain}
                res.success = True
                res.message = (f"Hand gains set: pos={pos_gain}, vel={vel_gain}, "
                               f"vel_int={vel_int_gain}")
            else:
                res.success = False
                res.message = "; ".join(m for ok, m in ((ok1, m1), (ok2, m2)) if not ok)
        except Exception as e:  # noqa: BLE001
            res.success = False
            res.message = str(e)
        return res

    def _svc_odrive_command(self, req, res):
        cmd = req.command
        if cmd == 'clear_errors':
            ok, msg, _ = self.teensy_clear_errors()
        elif cmd == 'reboot_odrives':
            # Route through the shared hook so this path ALSO clears the cold-start
            # state (cold-start clear) — the orchestrator reboots via odrive_command.
            ok, msg = self._reboot_odrives()
        else:
            ok, msg = False, f'Unknown command: {cmd}'
        res.success = ok
        res.message = msg
        return res

    # ═══════════════════════════════════════════════════════════
    # Orchestrator-facing cold-start conduit
    # ═══════════════════════════════════════════════════════════
    # Thin wrappers exposing the exact (name, type) interfaces the LOCKED
    # orchestrator drives cold-start through, delegating to the bridge's existing
    # verbs. Zero edits to orchestrator_node / state_machine.

    def _home_action_goal(self, goal_request):
        """Fast-reject a concurrent home_motors goal for a clean action REJECT. The
        AUTHORITATIVE single-flight guard lives in _do_home() (so it also covers the
        /home Trigger path); this is only the action-side optimization so a redundant
        goal is rejected up front rather than accepted-then-aborted. PEEK ONLY — it
        does not claim the guard (that happens in _do_home), so a goal that slips past
        this peek is still safely rejected by _do_home. Mirrors bb/throw's guard."""
        with self._home_lock:
            if self._home_in_progress:
                self.get_logger().warn(
                    'home_motors: rejecting goal — a home is already in progress')
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _home_action_execute(self, goal_handle):
        """home_motors action: the orchestrator's HomingHandler drives this
        (ActionClient(HomeMotors, 'home_motors'), orchestrator_node.py:61). Wraps the
        shared _do_home() (single-flight guarded; home legs + hand → persist is_homed →
        configure) and returns HomeMotors.Result(success) — byte-parity with
        can_node._action_home (can_node.py:843-861: home → persist → succeed/abort →
        Result(success)). The firmware HOME-axis-6 support + hand gains landed with
        the hand conduit, so this homes the hand too (parity with _home_robot_steps). A
        concurrent home (guard held) returns (False, …) → the goal aborts."""
        result = HomeMotors.Result()
        try:
            ok, msg = self._do_home()
            result.success = ok
            if ok:
                self.get_logger().info(f"home_motors: {msg}")
                goal_handle.succeed()
            else:
                self.get_logger().error(f"home_motors FAILED: {msg}")
                goal_handle.abort()
            return result
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"home_motors error: {e}")
            result.success = False
            goal_handle.abort()
            return result

    def _svc_activate_or_deactivate(self, req, res):
        """activate_or_deactivate: the orchestrator's ActiveHandler / LevellingHandler
        drive this (ActivateOrDeactivate client, orchestrator_node.py:51-52; command=
        'activate'|'deactivate'). Dispatches to _run_activate / _run_deactivate on the
        configured axes — the analogue of can_node._svc_activate_or_deactivate
        (can_node.py:765-780).

        ACTIVATE folds a _run_configure AFTER the move (parity requirement per the
        can_node<->Teensy parity audit): _run_activate ends the legs in TRAP_TRAJ
        holding the active pose;
        _run_configure switches them to POSITION/PASSTHROUGH so run_mpc.py's 40 Hz
        interp is the sole setpoint source (can_node ended PASSTHROUGH; the bridge's
        _run_activate alone ends TRAP_TRAJ). Order is activate-then-configure —
        configure is safe post-move (legs CLOSED_LOOP holding pose, motion-free; see
        _run_configure). A configure failure fails the result (legs at pose but not
        interp-ready is a real failure for run_mpc), mirroring _svc_home's fold. The
        configure scopes to activate_axes (the legs that moved; the hand is not part of
        ACTIVATE — it is rejected on axis 6)."""
        cmd = str(req.command)
        if cmd == 'activate':
            axes = list(self.get_parameter('activate_axes').value or [])
            ok, msg = self._run_activate(axes)
            if ok:
                cfg_ok, cfg_msg = self._run_configure(axes)
                ok = ok and cfg_ok
                msg = f"{msg}; {cfg_msg}"
            res.success = ok
            res.message = f"Platform activated. {msg}" if ok else msg
        elif cmd == 'deactivate':
            axes = list(self.get_parameter('deactivate_axes').value or [])
            ok, msg = self._run_deactivate(axes)
            res.success = ok
            res.message = f"Platform deactivated. {msg}" if ok else msg
        else:
            res.success = False
            res.message = f"Invalid command: {cmd}"
        return res

    # Tilt-read retry (orchestrator conduit): bounded retries on a failed/out-of-range relay read,
    # porting can_node's tilt robustness (can_node._tilt_reading_steps: resend on
    # timeout, retry on out-of-range, NaN after exhaustion — per the can_node<->Teensy parity audit).
    _TILT_READ_ATTEMPTS = 3

    def _svc_get_platform_tilt(self, req, res):
        """get_platform_tilt: the orchestrator's LevellingHandler drives this
        (GetTiltReadingService client, orchestrator_node.py:57-58; the level_get_tilt
        phase). Reads the Platform-Teensy inclinometer via the relay TILT_READ (relay-seam
        relay_read_tilt) with bounded retry + a validity bound, and returns the
        NaN-on-failure shape the orchestrator already consumes (orchestrator_node.py:
        194-206: NaN in tilt_xy → operation_result=False → LevellingHandler → FAULT).
        Byte-parity with can_node._svc_get_tilt (can_node.py:749-763) + the
        _tilt_reading_steps validity bound (|tilt| ≤ JB_OP_MAX_VALID_TILT_RAD; see the can_node<->Teensy parity audit)."""
        try:
            tx = ty = None
            for _ in range(self._TILT_READ_ATTEMPTS):
                ok, msg, tilt = self.relay_read_tilt()
                if ok and tilt is not None:
                    cand_x, cand_y = float(tilt[0]), float(tilt[1])
                    if (abs(cand_x) <= hw.JB_OP_MAX_VALID_TILT_RAD
                            and abs(cand_y) <= hw.JB_OP_MAX_VALID_TILT_RAD):
                        tx, ty = cand_x, cand_y
                        break
                    self.get_logger().warning(
                        f"tilt read out of range ([{cand_x:.3f}, {cand_y:.3f}] rad) "
                        "— retrying")
                else:
                    self.get_logger().warning(f"tilt read failed: {msg} — retrying")
            if tx is None:
                # Signal failure with NaN so the orchestrator doesn't mistake it for
                # real data (can_node._svc_get_tilt parity).
                res.tilt_xy = [float('nan'), float('nan')]
                res.tilt_quat = Quaternion(w=float('nan'))
                self.get_logger().error("get_platform_tilt: read failed (NaN returned)")
            else:
                res.tilt_xy = [tx, ty]
                res.tilt_quat = _tilt_to_quat(tx, ty)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"get_platform_tilt error: {e}")
            res.tilt_xy = [float('nan'), float('nan')]
            res.tilt_quat = Quaternion(w=float('nan'))
        return res

    def _sub_set_level_state(self, msg):
        """set_level_state: the orchestrator's LevellingHandler publishes
        [levelling_complete_flag, tiltX, tiltY] on the level_persist_state phase
        (orchestrator_node.py:76-77, 323-330). Persist it to the Platform Teensy via
        _write_level_state (relay STATE_WRITE, read-modify-write preserving is_homed).
        Byte-parity with can_node._sub_set_level_state (can_node.py:1052-1064)."""
        try:
            data = list(msg.data)
            if len(data) < 3:
                self.get_logger().warning(
                    f"set_level_state: expected [flag, tiltX, tiltY], got {data}")
                return
            levelling_complete = bool(data[0])
            tilt_x, tilt_y = float(data[1]), float(data[2])
            ok, wmsg = self._write_level_state(levelling_complete, tilt_x, tilt_y)
            if ok:
                self.get_logger().info(
                    f"Level state persisted: complete={levelling_complete}, "
                    f"offset=[{tilt_x:.4f}, {tilt_y:.4f}] rad")
            else:
                self.get_logger().error(f"set_level_state persist failed: {wmsg}")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"set_level_state error: {e}")

    # ═══════════════════════════════════════════════════════════
    # Ball Butler (cutover — replaces can_node bb/*)
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
    # Catching cone (cone uplink — mirrors can_node's
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
        """Apply leg AND hand vel/current limits over the can-bridge link.

        Mirrors can_node._sub_vel_curr_limits. The can-bridge owns CAN3 — legs 0-5
        AND the hand ODrive (axis 6) — so it applies the HAND limits too (parity
        hardening): the earlier "hand limits ignored" note was a can_node parity
        regression, false since the relay-seam axis-6 allow-table (the node already sends
        SET_VEL_CURR_LIMITS to axis 6 in _run_configure). On a successful hand write we
        also update the cached self._hand_vel_limit/_hand_curr_limit so a later
        _run_configure re-applies the operator's update, not the config default
        (parity with the self._hand_gains cache). Short-timeout RPCs keep a topic
        callback from stalling the executor on a dead link.
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
        # Hand (axis 6): the can-bridge owns it on CAN3 (relay-seam allow-table).
        if msg.hand_vel_limit > 0 and msg.hand_curr_limit > 0:
            ok, m, _ = self.teensy_set_vel_curr_limits(
                _HAND_AXIS, msg.hand_vel_limit, msg.hand_curr_limit,
                timeout=0.2, retries=0)
            if ok:
                self._hand_vel_limit = float(msg.hand_vel_limit)
                self._hand_curr_limit = float(msg.hand_curr_limit)
            else:
                self.get_logger().error(
                    f"set_vel_curr_limits hand failed: {m}",
                    throttle_duration_sec=2.0)

    # ═══════════════════════════════════════════════════════════
    # Shutdown
    # ═══════════════════════════════════════════════════════════

    def _shutdown_stow(self):
        """Profiled controlled-lower to STOW + IDLE on a clean shutdown — the Teensy-side
        analogue of can_node.on_shutdown's ``_gently_move_to_setpoint(0.0,
        deactivating=True)`` (can_node.py:1693-1706). Best-effort and self-bounded:
        never raises and never hangs teardown. Guard *shape* follows can_node (skip
        when the bus is undrivable, else stow), scoped to the CAN3 bus-off /
        no-telemetry / stow-pending subset — a broader fatal-CAN (link-lost /
        CAN_BUS_DOWN) only makes the DEACTIVATE RPC fail fast (bounded by its
        timeout×retry budget, never a hang):
          * CAN3 core bus down / no telemetry → skip (can't drive a dead bus; the
            firmware's CAN3-loss deferred stow safes the platform on reconnect);
          * a deferred stow already pending → skip (the firmware completes it);
          * otherwise DEACTIVATE the configured legs (TRAP_TRAJ lower to STOW then
            IDLE; ``_run_deactivate`` idles the hand too), bounded by
            DeactivateMonitor's internal hard deadline so teardown can't hang.
        A leg that was never activated (not in CLOSED_LOOP) is rejected immediately
        by the firmware → clean no-op, no descent, no hang.
        """
        try:
            hb = self._latest_heartbeat
            core_bus = int(hb.bus1_health) if hb is not None else None   # CAN3
            if hb is None or core_bus == int(BusHealth.BUS_OFF):
                self.get_logger().warning(
                    "shutdown stow skipped — CAN3 core bus down/unseen; leaving the "
                    "firmware deferred-stow to safe the platform on reconnect.")
                return
            if int(hb.flags) & _T2J_FLAG_STOW_PENDING:
                self.get_logger().info(
                    "shutdown stow skipped — a deferred stow is already pending; the "
                    "firmware will complete it.")
                return
            axes = [int(a) for a in self.get_parameter('deactivate_axes').value]
            self.get_logger().info(
                f"shutdown: profiled stow (DEACTIVATE) on legs {axes} before teardown")
            ok, msg = self._run_deactivate(
                axes, timeout_s=_SHUTDOWN_DEACTIVATE_TIMEOUT_S)
            if ok:
                self.get_logger().info(f"shutdown stow complete — {msg}")
            else:
                self.get_logger().error(f"shutdown stow did not complete — {msg}")
                # If a guard E-STOP is latched, DEACTIVATE is IMPOSSIBLE (the firmware
                # bounces it ERR_BUS_DOWN) — leave the operator a loud, unmissable
                # final message rather than a quiet warning that scrolls away.
                self._warn_disarmed_but_standing_if_latched()
        except Exception as e:  # noqa: BLE001 — teardown must never raise
            self.get_logger().error(f"shutdown stow error (continuing teardown): {e}")

    def _warn_disarmed_but_standing_if_latched(self):
        """Loud final message when a latched guard made the shutdown DEACTIVATE
        impossible: the robot is DISARMED (mpc_active=0, no setpoint output — it will
        not move) but STILL STANDING at the active pose because the profiled stow
        could not run. A latched guard only clears with CLEAR_ERRORS, so tell the
        operator exactly that. No-op when no guard is latched (fault_state=NONE) — a
        non-guard stow failure (e.g. a stalled leg) gets the generic error above."""
        hb = self._latest_heartbeat
        fault = int(hb.fault_state) if hb is not None else int(FaultState.NONE)
        if fault == int(FaultState.NONE):
            return
        name = _enum_name(FaultState, fault)
        self.get_logger().error(
            "SHUTDOWN: ROBOT DISARMED BUT STILL STANDING — a latched Teensy guard "
            f"(fault_state={name}) made the profiled DEACTIVATE impossible. "
            "mpc_active is 0 (no setpoint output) so the platform will NOT move, but "
            "it is NOT stowed. CLEAR_ERRORS is required to release the guard: run "
            "'ros2 service call /clear_errors std_srvs/srv/Trigger' then re-run "
            "deactivate, OR lower the legs manually before removing power.")

    def on_shutdown(self):
        """Ordered, bounded, best-effort safe-shutdown, THEN transport teardown.

        Task 3.3: a ``ros2 launch`` Ctrl-C must ALWAYS profiled-stow the robot — no
        matter what mode it was in (armed / streaming / already idle).

        WHY THIS SEQUENCE LIVES HERE (not the orchestrator, not a launch
        OnShutdown): on a launch Ctrl-C, launch broadcasts SIGINT to EVERY node
        process at once and each runs its own ``main()`` finally. A cross-process
        orchestration (orchestrator → bridge service call) would race THIS node's
        executor teardown — an rclpy service call from a shutdown hook on a dying
        executor is the classic deadlock — and a launch OnShutdown handler runs in
        the launch process, which has no link to the robot at all. The bridge is the
        ONE process that owns BOTH the disarm (``_stop_setpoint_output``) and the
        profiled stow (``_run_deactivate``) as IN-PROCESS methods, and its RX +
        heartbeat daemon threads keep the UDP link alive through this teardown — so
        the sequence runs deterministically with NO dependency on any other
        process's executor. Guarantee: it runs to completion (or its bounded
        timeout) as long as this process reaches its ``finally``. Limit: if launch
        SIGKILLs the process before the DEACTIVATE completes, the firmware's own
        CAN3-loss deferred stow is the backstop (the descent is a firmware-side
        TRAP_TRAJ move that continues autonomously even if the link dies mid-way).

        Sequence — each step best-effort (loud on failure, continue), total ≲ 8 s:
          1. DISARM — ``_stop_setpoint_output`` stops the 40 Hz setpoint thread THEN
             drops mpc_active=0 on the WIRE (the in-process 'set_setpoint_output
             false'). Done FIRST so mpc_active reaches 0 before the emitter-stop
             stream silence can latch MPC_STALE (trajectory_node Sharp Edge #1).
          2. SETTLE — a bounded couple of heartbeat periods so the flags=0 J→T
             heartbeat is transmitted + registered before the DEACTIVATE RPC (the
             firmware rejects DEACTIVATE while mpc_active=1).
          3. STOW — profiled TRAP_TRAJ lower to STOW + IDLE (bounded), guarded for a
             dead bus / already-pending deferred stow; a latched guard makes this
             impossible → loud 'disarmed but standing, CLEAR_ERRORS required'.
          4. Transport teardown (only stops the client if we created it).
        """
        self.get_logger().info("Shutting down TeensyBridgeNode...")
        # 1. DISARM. _stop_setpoint_output stops the setpoint thread then drops
        # mpc_active on the WIRE (not just the local flag) — even for an injected
        # client whose heartbeat thread keeps running after on_shutdown. It is fully
        # internally guarded; wrap it so teardown never raises, and fall back to the
        # raw flag drop so mpc_active=0 is guaranteed on any path.
        try:
            self._stop_setpoint_output()
        except Exception as e:  # noqa: BLE001 — best-effort during teardown
            self.get_logger().error(
                f"shutdown disarm error (continuing to stow): {e}")
            try:
                self._set_mpc_active(False)
            except Exception:  # noqa: BLE001
                self._mpc_active = False
        # 2. SETTLE + 3. STOW over the still-live link (RX + heartbeat threads keep
        # the telemetry cache + DEACTIVATE RPC alive; MPC is quiesced above so no
        # setpoint fights the descent). The settle is gated on actually stowing so a
        # stow-disabled node (unit tests) tears down instantly.
        if self._stow_on_shutdown:
            time.sleep(_SHUTDOWN_DISARM_SETTLE_S)
            self._shutdown_stow()
        # 4. Transport teardown.
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
