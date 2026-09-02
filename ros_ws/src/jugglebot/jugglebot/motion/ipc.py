"""ZeroMQ-based IPC layer for the motion control system.

Provides communication between the motor guard process, the ROS2 bridge
nodes, and the MPC process.  Messages are serialised with msgpack for speed.

Architecture::

    ROS2 Motion Bridge                   Motor Guard (500 Hz)
    ──────────────────                  ────────────────────
    BridgeIPC                           MotorGuardIPC
      PUB  ─── tcp://localhost:5555 ──►  SUB   (mode cmds, motor feedback)
      SUB  ◄── tcp://localhost:5556 ───  PUB   (telemetry)

    HardwarePlant (MPC process)  — REMOVED 2026-09-01, tag mpc-final
      PUB  ─── tcp://localhost:5557 ──►  SUB   (mpc commands + fallback enable)
      SUB  ◄── tcp://localhost:5556 ───        (telemetry, shared)
      # The live :5557 publisher is now trajectory_node (MpcCommandPub); the
      # :5556 producer (motor_guard) has no launcher.

    ROS2 MPC Bridge                      MPC Process
    ───────────────                     ───────────
    MpcBridgeIPC                        MpcTargetIPC
      PUB  ─── tcp://localhost:5558 ──►  SUB   (target poses, mode)
      # Both ends removed 2026-09-01 (tag mpc-final): mpc_bridge_node is
      # deleted, so :5558 has no producer.  controller/zmq_target.py's
      # decode/clamp path is retained and independently tested.

    MPC Process                          Catch Coordinator Node
    ───────────                         ──────────────────────
    TargetFeedbackPub                   TargetFeedbackSub
      PUB  ─── tcp://localhost:5559 ──►  SUB   (target accept/reject)
      # DORMANT (MPC stack): the catch coordinator now consumes
      # trajectory/target_feedback (trajectory_node, Phase 5), not this :5559 SUB.

    # Tombstone: the :5560 session-metadata channel (SessionMetadataPush /
    # SessionMetadataPull / SESSION_ADDR / make_session_start) was deleted
    # 2026-09-01 with the MPC chain — both endpoints (sim/main.py's MPC half
    # and mpc_bridge_node) are gone.  At git tag mpc-final.

Message types:
  - ModeCommand:   enable, disable, estop, fault, etc.
  - Telemetry:     motor guard output (leg states, timing, torques)
  - MotorFeedback: encoder positions/velocities/currents from CAN node
  - MPCCommand:    pre-computed motor commands from external MPC controller
  - MPCTarget:     target pose for the MPC to track (from any input source)
  - MPCMode:       active input mode forwarded to the MPC process
  - TargetFeedback: accept/reject decision for catch targets (dormant MPC stack only;
                    the coordinator now consumes trajectory/target_feedback, Phase 5)

Lifecycle authority (historical — motion_bridge_node and HardwarePlant were both
removed 2026-09-01, tag mpc-final; motor_guard now has neither feeder nor
lifecycle owner):
  - The ROS2 motion bridge on :5555 was the primary authority for motor guard
    enable/disable/estop.  It sent disable+enable on every mode transition.
  - HardwarePlant on :5557 sent a fallback enable for direct hardware mode
    (no ROS2).  When the bridge was running, this resolved to a no-op at the
    motor guard (enable-when-already-ENABLED is idempotent).

No ROS2 dependency.  Requires: pyzmq, msgpack.
"""

from __future__ import annotations

import logging
import time
from typing import Any

import msgpack
import zmq

logger = logging.getLogger(__name__)

# Default IPC addresses
COMMAND_ADDR = 'tcp://127.0.0.1:5555'       # bridge PUB → motor guard SUB
TELEMETRY_ADDR = 'tcp://127.0.0.1:5556'     # motor guard PUB → bridge SUB
MPC_COMMAND_ADDR = 'tcp://127.0.0.1:5557'   # trajectory_node PUB → teensy_bridge_node SUB
                                            # (was HardwarePlant PUB → motor guard SUB,
                                            #  removed 2026-09-01, tag mpc-final)
MPC_TARGET_ADDR = 'tcp://127.0.0.1:5558'    # was mpc_bridge PUB → MPC process SUB
                                            # (producer removed 2026-09-01, tag mpc-final)
MPC_FEEDBACK_ADDR = 'tcp://127.0.0.1:5559'  # was MPC process PUB → catch coordinator SUB
# SESSION_ADDR (:5560) removed 2026-09-01 with the MPC chain — both endpoints gone.

# Topic prefixes for ZMQ PUB/SUB filtering
TOPIC_MODE = b'mode'
TOPIC_TELEMETRY = b'telem'
TOPIC_MOTOR_FB = b'motorfb'
TOPIC_MPC_CMD = b'mpccmd'       # pre-computed motor commands from MPC
TOPIC_MPC_TARGET = b'mpctgt'    # target pose for MPC to track
TOPIC_MPC_MODE = b'mpcmode'     # active input mode for MPC process
TOPIC_TARGET_FB = b'tgtfb'      # target accept/reject feedback from MPC


# ---------------------------------------------------------------------------
# Message constructors
# ---------------------------------------------------------------------------

def make_mode_command(command: str, **params) -> dict:
    """Create a ModeCommand message.

    Parameters
    ----------
    command : 'enable', 'disable', 'estop', 'set_gains', etc.
    **params : command-specific parameters (e.g. kp=10.0)
    """
    return {
        'type': 'mode',
        'cmd': command,
        'params': params,
    }


def make_telemetry(leg_positions: list | tuple,
                   leg_velocities: list | tuple,
                   leg_torques: list | tuple,
                   loop_dt_s: float,
                   cond_number: float | None = None,
                   workspace_status: str | None = None,
                   workspace_speed_scale: float | None = None,
                   tracking_error_mm: list | tuple | None = None,
                   fault_state: str | None = None,
                   motor_pos: list | tuple | None = None,
                   motor_vel: list | tuple | None = None,
                   motor_cur: list | tuple | None = None,
                   timestamp: float | None = None) -> dict:
    """Create a Telemetry message from the motor guard.

    Parameters
    ----------
    leg_torques : per-motor feedforward torques (Nm)
    cond_number : Jacobian condition number at current pose, or None
    workspace_status : 'ok', 'soft', or 'hard', or None
    workspace_speed_scale : workspace-imposed speed scale 0.0-1.0, or None
    tracking_error_mm : per-leg position tracking error (mm), or None
    fault_state : fault description string, or None
    motor_pos : 6 actual motor positions (rev) from encoder feedback, or None
    motor_vel : 6 actual motor velocities (rev/s) from encoder feedback, or None
    motor_cur : 6 actual motor currents (A) from encoder feedback, or None
    timestamp : time.perf_counter() at message creation, or None
    """
    msg = {
        'type': 'telemetry',
        'leg_pos': list(leg_positions),
        'leg_vel': list(leg_velocities),
        'leg_torques': list(leg_torques),
        'dt': loop_dt_s,
    }
    if cond_number is not None:
        msg['cond_number'] = cond_number
    if workspace_status is not None:
        msg['workspace_status'] = workspace_status
    if workspace_speed_scale is not None:
        msg['workspace_speed_scale'] = workspace_speed_scale
    if tracking_error_mm is not None:
        msg['tracking_error_mm'] = list(tracking_error_mm)
    if fault_state is not None:
        msg['fault_state'] = fault_state
    if motor_pos is not None:
        msg['motor_pos'] = list(motor_pos)
    if motor_vel is not None:
        msg['motor_vel'] = list(motor_vel)
    if motor_cur is not None:
        msg['motor_cur'] = list(motor_cur)
    msg['timestamp'] = timestamp
    return msg


def make_motor_feedback(positions: list | tuple,
                        velocities: list | tuple,
                        currents: list | tuple) -> dict:
    """Create a MotorFeedback message from the bridge (CAN node data)."""
    return {
        'type': 'motor_feedback',
        'pos': list(positions),
        'vel': list(velocities),
        'cur': list(currents),
    }


def make_mpc_command(ext_mm: list | tuple,
                     pose_6dof: list | tuple,
                     motor_rev: list | tuple | None = None,
                     vel_mm_s: list | tuple | None = None,
                     acc_mm_s2: list | tuple | None = None,
                     torque_Nm: list | tuple | None = None,
                     seq: int = 0,
                     cmd_next_mm: list | tuple | None = None,
                     cmd_next2_mm: list | tuple | None = None,
                     *,
                     vel_next_mm_s: list | tuple | None = None,
                     hand_rev: float | None = None,
                     hand_vel_rps: float | None = None,
                     hand_next_rev: float | None = None,
                     hand_next2_rev: float | None = None,
                     hand_next_vel_rps: float | None = None,
                     out: dict | None = None) -> dict:
    """Create an MPC command message.

    Sent by the external MPC controller (HardwarePlant) directly to the
    motor guard.  The motor guard uses motor_rev directly as commanded
    positions (matching ODrive encoder convention where 0 = STOW), and
    applies the full safety pipeline.

    Parameters
    ----------
    ext_mm : 6 IK-convention leg extensions (mm) -- for workspace checks
    pose_6dof : [x, y, z, rx, ry, rz] in mm, rad -- Cartesian pose for
        condition-number checks (from MPC predicted_poses[0])
    motor_rev : 6 absolute motor positions (rev) in ODrive convention
        (0 = STOW).  Includes the stow offset.  If None, the motor guard
        falls back to ``extensions_mm_to_revs(ext_mm)`` (no stow offset --
        only correct for unit tests / sim).
    vel_mm_s : 6 leg velocities (mm/s), or None for zeros
    acc_mm_s2 : 6 leg accelerations (mm/s²), or None for zeros.
        Used by the motor guard for cubic interpolation between 40 Hz
        MPC commands (eliminates position step at command boundaries).
    torque_Nm : 6 motor torques (Nm), or None for zeros
    seq : monotonic sequence number (for debugging / drop detection)
    cmd_next_mm : 6 leg extensions (mm) for the MPC's predicted next
        step (u[1]).  When present, the motor guard uses cubic Hermite
        interpolation between ext_mm and cmd_next_mm instead of Taylor
        extrapolation from ext_mm alone.  None falls back to extrapolation.
    cmd_next2_mm : 6 leg extensions (mm) for the MPC's predicted step
        after next (u[2]).  When present with cmd_next_mm, the motor guard
        uses (u[2] - u[1]) / T as the endpoint velocity of the current
        Hermite segment, producing C1-continuous position across segment
        boundaries (eliminates 40 Hz velocity discontinuity).
    vel_next_mm_s : 6 leg extension RATES (mm/s) at the u[1] knot, or None.
        Fills the v6 Setpoint's exact ``v1[0:6]`` (behind ``HAS_V1``); absent
        keeps the firmware forward-difference fallback (the flown path).
    hand_rev, hand_vel_rps, hand_next_rev, hand_next2_rev, hand_next_vel_rps :
        the 7th (hand) channel — ODrive-convention absolute rev / rev/s at
        the u0, u0, u1, u2 and u1 knots respectively.  The pump maps them to
        Setpoint index 6 behind ``HAS_HAND`` (and ``v1[6]`` behind
        ``HAS_V1``).  All-or-nothing at the pump: see
        ``teensy_link/setpoint_pump.py``.

    NOTE — the six v6 keys (vel_next_mm_s + the five hand keys) have
    ABSENT-when-None semantics even under ``out=`` reuse: a key a previous
    tick wrote is DELETED when this tick passes None.  This deliberately
    differs from the legacy fields' write-None-under-out behaviour, and it
    is load-bearing: with ``out=`` reuse, a hand VALUE stashed by a previous
    (hand-carrying) tick would otherwise survive into a later legacy frame
    and silently arm ``HAS_HAND`` with a stale hand target — the exact
    stale-key leak the emitter's byte-identity regression (T-U8/T-R1) pins
    against.
    """
    # Values are passed through as-is (ndarrays, lists, etc.).
    # The _pack() serialiser handles ndarray → list conversion via its
    # default handler, avoiding redundant .tolist() calls on the caller side.
    #
    # When ``out`` is provided (hot-loop path from HardwarePlant.command),
    # the dict is mutated in place instead of freshly allocated.  Every
    # key is always written so stale fields from a previous tick cannot
    # leak into a send where the current tick wanted them omitted.
    msg = out if out is not None else {}
    msg['type'] = 'mpc_cmd'
    msg['ext_mm'] = ext_mm
    msg['pose_6dof'] = pose_6dof
    msg['seq'] = seq
    # Optional fields: always write when provided; when None AND this is
    # a fresh dict, leave the key absent (legacy behaviour).  When ``out``
    # is provided we ALWAYS set — None explicitly clears the field, which
    # matches the "stable schema" intent of the pre-alloc path.
    if motor_rev is not None or out is not None:
        msg['motor_rev'] = motor_rev
    elif 'motor_rev' in msg:
        del msg['motor_rev']
    if vel_mm_s is not None or out is not None:
        msg['vel_mm_s'] = vel_mm_s
    elif 'vel_mm_s' in msg:
        del msg['vel_mm_s']
    if acc_mm_s2 is not None or out is not None:
        msg['acc_mm_s2'] = acc_mm_s2
    elif 'acc_mm_s2' in msg:
        del msg['acc_mm_s2']
    if torque_Nm is not None or out is not None:
        msg['torque_Nm'] = torque_Nm
    elif 'torque_Nm' in msg:
        del msg['torque_Nm']
    if cmd_next_mm is not None or out is not None:
        msg['cmd_next_mm'] = cmd_next_mm
    elif 'cmd_next_mm' in msg:
        del msg['cmd_next_mm']
    if cmd_next2_mm is not None or out is not None:
        msg['cmd_next2_mm'] = cmd_next2_mm
    elif 'cmd_next2_mm' in msg:
        del msg['cmd_next2_mm']
    # v6 keys (2026-09-01, unified-7dof-planner Phase 2): absent-when-None
    # EVEN under out= reuse — see the docstring NOTE for why a previously
    # written key must be deleted rather than overwritten with None.
    for key, val in (('vel_next_mm_s', vel_next_mm_s),
                     ('hand_rev', hand_rev),
                     ('hand_vel_rps', hand_vel_rps),
                     ('hand_next_rev', hand_next_rev),
                     ('hand_next2_rev', hand_next2_rev),
                     ('hand_next_vel_rps', hand_next_vel_rps)):
        if val is not None:
            msg[key] = val
        elif key in msg:
            del msg[key]
    return msg


def make_mpc_target(target_pose: list | tuple,
                    arrival_time: float | None = None,
                    target_twist: list | tuple | None = None,
                    source: str = '') -> dict:
    """Create an MPC target message.

    Sent by the MPC bridge node to the MPC process.  Contains the target
    pose that the MPC should track, optionally with an arrival deadline
    and a desired twist at arrival.

    Parameters
    ----------
    target_pose : [x, y, z, rx, ry, rz] in mm / rad (rotation vector)
    arrival_time : absolute time (perf_counter) to arrive, or None = ASAP
    target_twist : [vx, vy, vz, wx, wy, wz] in mm/s / rad/s, or None = hold
    source : input source identifier ('spacemouse', 'gui', 'catch')
    """
    msg = {
        'type': 'mpc_target',
        'target_pose': list(target_pose),
        'source': source,
    }
    if arrival_time is not None:
        msg['arrival_time'] = arrival_time
    if target_twist is not None:
        msg['target_twist'] = list(target_twist)
    return msg


# make_session_start() removed 2026-09-01 with the MPC chain (tag mpc-final) —
# its only caller was SessionMetadataPush, itself removed below.


def make_mpc_mode(mode: str) -> dict:
    """Create an MPC mode message.

    Sent by the MPC bridge node to inform the MPC process of the current
    active input mode.

    Parameters
    ----------
    mode : 'spacemouse', 'gui', 'catch', or 'disabled'
    """
    return {'type': 'mpc_mode', 'mode': mode}


def make_target_feedback(arrival_time: float,
                         accepted: bool,
                         source: str = '',
                         violations: list | None = None,
                         *,
                         feedback_type: str = 'accepted',
                         reason: str | None = None,
                         stretch_ms: float | None = None) -> dict:
    """Create a target feedback message.

    Published by the MPC process after evaluating a target.  Consumed by
    the catch coordinator node (and future consumers).

    Three ``feedback_type`` values (W11 of the K1–K6 reference-feasibility
    cycle — see ``controller/REFERENCE_LAYER_CONTRACT.md``):

    * ``'accepted'`` — legacy/default tag for the original accept/reject
      protocol.  ``accepted`` is True when the solver converged and the
      target is feasible; ``accepted`` MAY be False when the solver failed
      (in which case ``violations`` carries the failure reasons).  This is
      the backward-compatible default for non-catch callers and for the
      pre-K1–K6 catch-feedback path in ``run_mpc.py`` and ``sim/main.py``.
      The K1–K6 feedback types (``'rejected_infeasible'`` /
      ``'stretch_warning'``) carry stronger guarantees on the ``accepted``
      field.
    * ``'rejected_infeasible'`` — the reference-feasibility layer refused to
      stretch the requested trajectory (catch-path only; ``no_stretch=True``
      in ``make_feasible_events``).  ``accepted`` MUST be False, ``reason``
      carries a short diagnostic string (e.g.
      ``"peak_exceeds_no_stretch_allowed (segment 0: vr=1.49 ar=0.47)"``).
      The platform continues the last feasible trajectory — see
      ``ZmqTargetSource.update()``.
    * ``'stretch_warning'`` — the reference-feasibility layer stretched the
      trajectory's arrival_time to satisfy K2/K3.  ``accepted`` MUST be True
      (the target will still be reached, just later than requested).
      ``stretch_ms`` is the delay in milliseconds.  Intended for downstream
      consumers that might want to re-plan around the delay; the existing
      catch coordinator ignores this case (``accepted=True`` routes to
      ``report_acceptance``, which is correct).

    Parameters
    ----------
    arrival_time : float
        Original ``arrival_time`` for correlation.
    accepted : bool
        Required relationship with ``feedback_type``:
          - ``'accepted'``: True on solver success, False on solver failure
            (legacy callers).  ``violations`` carries the failure reasons
            in the False case.
          - ``'stretch_warning'``: MUST be True (the target will still be
            reached, just later than requested).
          - ``'rejected_infeasible'``: MUST be False.
    source : str
        Input source identifier (e.g. 'catch').
    violations : list of str or None
        Legacy field — human-readable solver-failure reasons when present.
        Backward compat for the original accept/reject protocol.  Non-None
        only on solver-failure rejections; K1–K6 rejections use ``reason``
        instead.
    feedback_type : str, default 'accepted'
        One of 'accepted' / 'rejected_infeasible' / 'stretch_warning'.
    reason : str or None
        Machine-readable rejection reason from ``make_feasible_events``.
        Only present when ``feedback_type == 'rejected_infeasible'``.
    stretch_ms : float or None
        Arrival-time delay in milliseconds.  Only present when
        ``feedback_type == 'stretch_warning'``.
    """
    msg = {
        'type': 'target_feedback',
        'arrival_time': arrival_time,
        'accepted': accepted,
        'source': source,
        'feedback_type': feedback_type,
    }
    if violations:
        msg['violations'] = violations
    if reason is not None:
        msg['reason'] = reason
    if stretch_ms is not None:
        msg['stretch_ms'] = stretch_ms
    return msg


# ---------------------------------------------------------------------------
# Serialisation
# ---------------------------------------------------------------------------

def _ndarray_default(obj):
    """msgpack default handler: convert numpy arrays to lists."""
    if hasattr(obj, 'tolist'):
        return obj.tolist()
    raise TypeError(f"Unknown type: {type(obj)}")


def _pack(topic: bytes, msg: dict) -> list[bytes]:
    """Serialise a message for ZMQ multipart send."""
    return [topic, msgpack.packb(msg, use_bin_type=True,
                                 default=_ndarray_default)]


def _unpack(frames: list[bytes]) -> tuple[bytes, dict]:
    """Deserialise a ZMQ multipart message."""
    topic = frames[0]
    msg = msgpack.unpackb(frames[1], raw=False)
    return topic, msg


# ---------------------------------------------------------------------------
# MVP trajectory generator side — publishes MPC commands on :5557
# ---------------------------------------------------------------------------

class MpcCommandPub:
    """PUB endpoint binding MPC_COMMAND_ADDR (:5557), topic ``mpccmd``.

    Used by ``trajectory_node`` to stream ``make_mpc_command`` dicts on the
    exact seam the bridge's ``_MpcCommandSetpointSource`` consumes, through the
    shared ``_pack`` (msgpack + ndarray default handler) on ``TOPIC_MPC_CMD``.
    (The name is historical: the MPC's ``HardwarePlant`` was the original
    binder, removed 2026-09-01 — git tag ``mpc-final``. The wire format is
    unchanged.)

    **Sole-binder interlock.** ZMQ does not allow two PUB ``bind()``s on one
    address, so a second binder raises ``zmq.ZMQError`` rather than silently
    interleaving two command streams onto the legs. ``trajectory_node`` is the
    only binder today; the caller is still expected to translate a bind failure
    into a loud, fatal startup error rather than continuing.
    """

    def __init__(self, addr: str = MPC_COMMAND_ADDR):
        self._ctx = zmq.Context()
        self._pub = self._ctx.socket(zmq.PUB)
        # bind() raises zmq.ZMQError (EADDRINUSE) if another PUB already holds
        # the address — the sole-binder interlock. Let it propagate.
        self._pub.bind(addr)
        self._addr = addr

    def send(self, msg: dict) -> None:
        """Publish an MPC command dict (byte-compatible with HardwarePlant)."""
        self._pub.send_multipart(_pack(TOPIC_MPC_CMD, msg), flags=zmq.NOBLOCK)

    def close(self) -> None:
        try:
            self._pub.close()
            self._ctx.term()
        except Exception:  # noqa: BLE001
            pass


# ---------------------------------------------------------------------------
# Motor Guard (standalone process) side
# ---------------------------------------------------------------------------

class MotorGuardIPC:
    """IPC endpoints for the motor guard process.

    Uses four SUB sockets to isolate message types:
      - ``_sub_mode``: mode commands from the bridge (no CONFLATE)
      - ``_sub_motor_fb``: motor feedback from CAN node (CONFLATE=1)
      - ``_sub_mpc_cmd``: MPC commands (CONFLATE=1, separate port)
      - ``_sub_mpc_mode``: mode commands from HardwarePlant (no CONFLATE,
        separate port)

    Motor feedback gets its own CONFLATE socket so that high-frequency
    feedback (100 Hz) cannot overwrite mode messages and vice-versa.
    Without this separation, ZMQ CONFLATE keeps only the single latest
    message *regardless of topic prefix*, silently dropping whichever
    message arrived first.

    Mode commands from HardwarePlant (enable/disable/estop) get their own
    non-CONFLATE socket on the MPC port for the same reason — mixing them
    with MPC commands on a single CONFLATE socket would silently drop
    whichever message arrived first in the same poll window.

    The MPC command socket listens on a separate port (MPC_COMMAND_ADDR)
    because the bridge already ``bind()``s PUB on COMMAND_ADDR -- ZMQ does
    not support two PUB ``bind()``s on the same address.
    """

    def __init__(self,
                 command_addr: str = COMMAND_ADDR,
                 telemetry_addr: str = TELEMETRY_ADDR,
                 mpc_command_addr: str = MPC_COMMAND_ADDR):
        self._ctx = zmq.Context()

        # SUB socket for motor feedback -- no CONFLATE (ZMQ 4.3.5 drops
        # all messages when CONFLATE + topic filter are combined on a
        # late-connecting SUB).  recv_all() drains to latest instead.
        self._sub_motor_fb = self._ctx.socket(zmq.SUB)
        self._sub_motor_fb.connect(command_addr)
        self._sub_motor_fb.setsockopt(zmq.SUBSCRIBE, TOPIC_MOTOR_FB)
        self._sub_motor_fb.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking
        self._sub_motor_fb.setsockopt(zmq.RCVHWM, 2)    # small buffer, drain to latest

        # SUB socket for mode commands -- no CONFLATE, every command delivered
        self._sub_mode = self._ctx.socket(zmq.SUB)
        self._sub_mode.connect(command_addr)
        self._sub_mode.setsockopt(zmq.SUBSCRIBE, TOPIC_MODE)
        self._sub_mode.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking
        self._sub_mode.setsockopt(zmq.RCVHWM, 64)   # bound queue size

        # SUB socket for MPC commands -- separate port.
        # No CONFLATE: ZMQ 4.3.5 drops all messages when CONFLATE + topic
        # filter are combined on a late-connecting SUB.  Instead, recv_all()
        # drains all pending messages and the caller uses the latest.
        # Cap reconnect interval: the MPC process (HardwarePlant) binds PUB
        # on this port minutes after the motor guard starts.
        self._sub_mpc_cmd = self._ctx.socket(zmq.SUB)
        self._sub_mpc_cmd.setsockopt(zmq.RECONNECT_IVL, 100)      # 100ms base
        self._sub_mpc_cmd.setsockopt(zmq.RECONNECT_IVL_MAX, 200)  # 200ms cap
        self._sub_mpc_cmd.connect(mpc_command_addr)
        self._sub_mpc_cmd.setsockopt(zmq.SUBSCRIBE, TOPIC_MPC_CMD)
        self._sub_mpc_cmd.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking
        self._sub_mpc_cmd.setsockopt(zmq.RCVHWM, 2)    # small buffer, drain to latest

        # SUB socket for mode commands from HardwarePlant -- no CONFLATE,
        # every enable/disable/estop is delivered.  Separate from _sub_mpc_cmd
        # because CONFLATE keeps only the single latest message regardless of
        # topic prefix — mixing mode + MPC on one CONFLATE socket silently
        # drops whichever arrives first in the same poll window.
        self._sub_mpc_mode = self._ctx.socket(zmq.SUB)
        self._sub_mpc_mode.setsockopt(zmq.RECONNECT_IVL, 100)      # 100ms base
        self._sub_mpc_mode.setsockopt(zmq.RECONNECT_IVL_MAX, 200)  # 200ms cap
        self._sub_mpc_mode.connect(mpc_command_addr)
        self._sub_mpc_mode.setsockopt(zmq.SUBSCRIBE, TOPIC_MODE)
        self._sub_mpc_mode.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking
        self._sub_mpc_mode.setsockopt(zmq.RCVHWM, 64)   # bound queue size

        # PUB socket -- publishes telemetry to bridge
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.bind(telemetry_addr)

        self._last_recv_time = time.monotonic()

    def recv_all(self) -> list[tuple[bytes, dict]]:
        """Non-blocking: receive all pending messages.

        Drains mode commands first (critical), then MPC commands, then
        motor feedback.  Returns list of (topic, message_dict) tuples.
        Empty list if nothing available.
        """
        messages = []
        # Drain mode commands first -- these are rare but critical.
        # Both bridge mode (_sub_mode) and HardwarePlant mode (_sub_mpc_mode)
        # are drained before MPC commands.
        for sub in (self._sub_mode, self._sub_mpc_mode, self._sub_mpc_cmd,
                    self._sub_motor_fb):
            while True:
                try:
                    frames = sub.recv_multipart(flags=zmq.NOBLOCK)
                    topic, msg = _unpack(frames)
                    messages.append((topic, msg))
                    self._last_recv_time = time.monotonic()
                except zmq.Again:
                    break
        return messages

    def send_telemetry(self, msg: dict) -> None:
        """Publish telemetry to the bridge."""
        self._pub.send_multipart(_pack(TOPIC_TELEMETRY, msg), flags=zmq.NOBLOCK)

    @property
    def seconds_since_last_recv(self) -> float:
        """Seconds since any message was last received from the bridge."""
        return time.monotonic() - self._last_recv_time

    def close(self) -> None:
        self._sub_motor_fb.close()
        self._sub_mode.close()
        self._sub_mpc_cmd.close()
        self._sub_mpc_mode.close()
        self._pub.close()
        self._ctx.term()



# ---------------------------------------------------------------------------
# Bridge (ROS2) side
# ---------------------------------------------------------------------------

class BridgeIPC:
    """IPC endpoints for the ROS2 bridge node."""

    def __init__(self,
                 command_addr: str = COMMAND_ADDR,
                 telemetry_addr: str = TELEMETRY_ADDR):
        self._ctx = zmq.Context()

        # PUB socket -- sends mode commands and motor feedback to motor guard
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.bind(command_addr)

        # SUB socket -- receives telemetry from motor guard.
        # Use SUBSCRIBE=b'' instead of TOPIC_TELEMETRY filter — ZMQ 4.3.5
        # drops all messages when CONFLATE + topic filter are combined on a
        # late-connecting SUB.  Safe: motor guard PUB only publishes telemetry.
        # Cap reconnect interval to match MotorGuardIPC pattern — prevents
        # ZMQ exponential backoff (100ms → 30s) from freezing telemetry.
        # RCVHWM=2 + drain-in-loop in recv_telemetry() is used in lieu of
        # CONFLATE: CONFLATE must be set *before* connect() to take effect,
        # and even when correctly ordered it silently ignores multi-part
        # messages on some libzmq versions.  Explicit draining is robust.
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.setsockopt(zmq.RECONNECT_IVL, 100)      # 100ms base
        self._sub.setsockopt(zmq.RECONNECT_IVL_MAX, 200)   # 200ms cap
        self._sub.setsockopt(zmq.RCVHWM, 2)                # bounded queue
        self._sub.connect(telemetry_addr)
        self._sub.setsockopt(zmq.SUBSCRIBE, b'')
        self._sub.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking

    def send_mode_command(self, msg: dict) -> None:
        """Send a ModeCommand message to the motor guard."""
        self._pub.send_multipart(_pack(TOPIC_MODE, msg), flags=zmq.NOBLOCK)

    def send_motor_feedback(self, msg: dict) -> None:
        """Send MotorFeedback to the motor guard."""
        self._pub.send_multipart(_pack(TOPIC_MOTOR_FB, msg), flags=zmq.NOBLOCK)

    def recv_telemetry(self) -> dict | None:
        """Non-blocking: drain the SUB and return the latest telemetry, or None.

        Drains all pending messages (not just one) because the motor guard
        publishes at 500 Hz and a slow caller would otherwise accumulate
        stale messages.  CONFLATE is not used — see the SUB construction
        comment.
        """
        latest: dict | None = None
        while True:
            try:
                frames = self._sub.recv_multipart(flags=zmq.NOBLOCK)
                _, latest = _unpack(frames)
            except zmq.Again:
                break
        return latest

    def close(self) -> None:
        self._pub.close()
        self._sub.close()
        self._ctx.term()


# ---------------------------------------------------------------------------
# MPC Bridge (ROS2 node) side — publishes targets to MPC process
# ---------------------------------------------------------------------------

class MpcBridgeIPC:
    """IPC endpoint for the MPC bridge node (PUB side).

    Publishes target poses and mode changes to the MPC process on
    MPC_TARGET_ADDR (:5558).
    """

    def __init__(self, target_addr: str = MPC_TARGET_ADDR):
        self._ctx = zmq.Context()
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.bind(target_addr)

    def send_target(self, msg: dict) -> None:
        """Publish an MPC target command."""
        self._pub.send_multipart(
            _pack(TOPIC_MPC_TARGET, msg), flags=zmq.NOBLOCK)

    def send_mode(self, msg: dict) -> None:
        """Publish an MPC mode change."""
        self._pub.send_multipart(
            _pack(TOPIC_MPC_MODE, msg), flags=zmq.NOBLOCK)

    def close(self) -> None:
        self._pub.close()
        self._ctx.term()


# ---------------------------------------------------------------------------
# MPC Process (standalone) side — receives targets from bridge
# ---------------------------------------------------------------------------

class MpcTargetIPC:
    """IPC endpoint for the MPC process (SUB side).

    Uses two SUB sockets on MPC_TARGET_ADDR (:5558):
      - ``_sub_target``: target poses (CONFLATE=1, latest wins)
      - ``_sub_mode``: mode changes (no CONFLATE, every message delivered)

    Separate sockets prevent CONFLATE from silently dropping mode messages
    when a target arrives in the same inter-poll window (the same pattern
    used by MotorGuardIPC).
    """

    def __init__(self, target_addr: str = MPC_TARGET_ADDR):
        self._ctx = zmq.Context()

        # CONFLATE SUB for targets — only the latest matters
        self._sub_target = self._ctx.socket(zmq.SUB)
        self._sub_target.connect(target_addr)
        self._sub_target.setsockopt(zmq.SUBSCRIBE, TOPIC_MPC_TARGET)
        self._sub_target.setsockopt(zmq.RCVTIMEO, 0)   # non-blocking
        self._sub_target.setsockopt(zmq.CONFLATE, 1)    # keep only latest

        # Non-CONFLATE SUB for mode — every transition matters
        self._sub_mode = self._ctx.socket(zmq.SUB)
        self._sub_mode.connect(target_addr)
        self._sub_mode.setsockopt(zmq.SUBSCRIBE, TOPIC_MPC_MODE)
        self._sub_mode.setsockopt(zmq.RCVTIMEO, 0)     # non-blocking
        self._sub_mode.setsockopt(zmq.RCVHWM, 64)      # bound queue size

    def recv_all(self) -> list[tuple[bytes, dict]]:
        """Non-blocking: receive all pending messages.

        Drains mode messages first (critical), then the latest target.
        Returns list of (topic, message_dict) tuples.

        Corrupt frames (truncated, byte-flipped, malformed msgpack) are
        logged at WARNING and dropped — the MPC loop continues with the
        next frame.  See Plan 2 Phase 6 bugfix
        (logbook 2026-05-11-tier2c-zmq-recv-resilience-bugfix.md):
        msgpack 1.0.7 raises ``ValueError`` on truncated input and
        ``UnicodeDecodeError`` on flipped utf-8 string bytes, neither
        of which inherits from ``msgpack.UnpackException``.
        """
        messages = []
        for sub in (self._sub_mode, self._sub_target):
            while True:
                try:
                    frames = sub.recv_multipart(flags=zmq.NOBLOCK)
                except zmq.Again:
                    break
                try:
                    topic, msg = _unpack(frames)
                except (msgpack.UnpackException, ValueError,
                        UnicodeDecodeError) as exc:
                    logger.warning(
                        "MpcTargetIPC: dropped corrupt frame "
                        "(%s: %s)", type(exc).__name__, exc)
                    continue
                messages.append((topic, msg))
        return messages

    def close(self) -> None:
        self._sub_target.close()
        self._sub_mode.close()
        self._ctx.term()


# ---------------------------------------------------------------------------
# Target Feedback (MPC process → catch coordinator)
# ---------------------------------------------------------------------------

class TargetFeedbackPub:
    """PUB endpoint for target accept/reject feedback.

    Used by the MPC process to inform the catch coordinator whether a
    catch target was accepted (solver converged, target reachable by
    arrival_time) or rejected (solver failed / unreachable).

    Binds on MPC_FEEDBACK_ADDR (:5559).
    """

    def __init__(self, feedback_addr: str = MPC_FEEDBACK_ADDR):
        self._ctx = zmq.Context()
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.bind(feedback_addr)

    def send(self, msg: dict) -> None:
        """Publish a target feedback message."""
        self._pub.send_multipart(
            _pack(TOPIC_TARGET_FB, msg), flags=zmq.NOBLOCK)

    def close(self) -> None:
        self._pub.close()
        self._ctx.term()


class TargetFeedbackSub:
    """SUB endpoint for target accept/reject feedback.

    DORMANT (MPC stack only): historically the catch coordinator received
    accept/reject decisions from the MPC process here (:5559). Since Phase 5 the
    coordinator consumes the ``trajectory/target_feedback`` ROS topic
    (trajectory_node's feasibility gate) instead; this SUB is retained only for the
    dormant MPC path.  Connects to MPC_FEEDBACK_ADDR (:5559).
    """

    def __init__(self, feedback_addr: str = MPC_FEEDBACK_ADDR):
        self._ctx = zmq.Context()
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.connect(feedback_addr)
        self._sub.setsockopt(zmq.SUBSCRIBE, TOPIC_TARGET_FB)
        self._sub.setsockopt(zmq.RCVTIMEO, 0)   # non-blocking
        self._sub.setsockopt(zmq.RCVHWM, 64)

    def recv(self) -> dict | None:
        """Non-blocking: receive the next feedback message, or None."""
        try:
            frames = self._sub.recv_multipart(flags=zmq.NOBLOCK)
            _, msg = _unpack(frames)
            return msg
        except zmq.Again:
            return None

    def close(self) -> None:
        self._sub.close()
        self._ctx.term()


# ---------------------------------------------------------------------------
# Session metadata — REMOVED 2026-09-01 with the MPC chain (git tag mpc-final).
# SessionMetadataPush (sim/main.py --mpc) and SessionMetadataPull (mpc_bridge_node)
# carried the telemetry-CSV filename over :5560 for ROS2 log correlation; both
# endpoints are deleted, so the channel had no producer and no consumer.
# See logbook/2026-09-01-mpc-chain-removed.md.
# ---------------------------------------------------------------------------
