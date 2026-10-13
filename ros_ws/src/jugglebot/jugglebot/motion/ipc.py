"""ZeroMQ-based IPC layer for the leg path and the motor guard.

Messages are serialised with msgpack.  The live leg path uses one channel from
this module::

    trajectory_node (40 Hz emitter)        teensy_bridge_node
      SetpointCommandPub  PUB (binds)  ─── tcp://127.0.0.1:5557 ──►  SUB  topic ``setpoint``
      (``make_setpoint_command`` dicts: leg/hand setpoints, mode commands)

(Renamed 2026-10-14 from the ``mpc``-prefixed names the removed MPC chain,
git tag ``mpc-final``, first used: topic ``mpccmd`` -> ``setpoint``, wire type
``mpc_cmd`` -> ``setpoint_cmd``.  Publisher and subscribers restart together.)

The remaining endpoints serve the parked ``motor_guard`` fallback::

    BridgeIPC                           MotorGuardIPC
      PUB  ─── tcp://localhost:5555 ──►  SUB   (mode cmds, motor feedback)
      SUB  ◄── tcp://localhost:5556 ───  PUB   (telemetry)
      # motor_guard has no launcher (parked); :5557 is also what
      # MotorGuardIPC subscribes to for commands.

Tombstones: the :5558 target, :5559 target-feedback and :5560 session-metadata
channels were deleted with the MPC chain (2026-09-01, tag ``mpc-final``); their
last dead constructors/endpoints were removed 2026-10-14.

Message types:
  - ModeCommand:   enable, disable, estop, fault, etc.
  - Telemetry:     motor guard output (leg states, timing, torques)
  - MotorFeedback: encoder positions/velocities/currents
  - Command:       per-cycle leg/hand setpoints from ``trajectory_node``
                   (built by ``make_setpoint_command``)

No ROS2 dependency.  Requires: pyzmq, msgpack.
"""

from __future__ import annotations

import time

import msgpack
import zmq

# Default IPC addresses
COMMAND_ADDR = 'tcp://127.0.0.1:5555'       # bridge PUB → motor guard SUB
TELEMETRY_ADDR = 'tcp://127.0.0.1:5556'     # motor guard PUB → bridge SUB
SETPOINT_COMMAND_ADDR = 'tcp://127.0.0.1:5557'   # trajectory_node PUB → teensy_bridge_node SUB
# (:5558 / :5559 / :5560 retired with the MPC chain, tag mpc-final.)

# Topic prefixes for ZMQ PUB/SUB filtering
TOPIC_MODE = b'mode'
TOPIC_TELEMETRY = b'telem'
TOPIC_MOTOR_FB = b'motorfb'
TOPIC_SETPOINT_CMD = b'setpoint'       # per-cycle setpoint commands from trajectory_node


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


def make_setpoint_command(ext_mm: list | tuple,
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
                     vel_next2_mm_s: list | tuple | None = None,
                     hand_rev: float | None = None,
                     hand_vel_rps: float | None = None,
                     hand_next_rev: float | None = None,
                     hand_next2_rev: float | None = None,
                     hand_next_vel_rps: float | None = None,
                     hand_next2_vel_rps: float | None = None,
                     hand_acc_rps2: float | None = None,
                     knot_epoch_us: int | None = None,
                     out: dict | None = None) -> dict:
    """Create a leg/hand setpoint command message (wire type ``setpoint_cmd``).

    Built by ``trajectory_node``'s 40 Hz emitter and published on :5557;
    ``teensy_bridge_node`` (or, on the parked fallback, the motor guard)
    consumes it.  ``motor_rev`` is used directly as commanded positions
    (matching ODrive encoder convention where 0 = STOW).

    Parameters
    ----------
    ext_mm : 6 IK-convention leg extensions (mm) -- for workspace checks
    pose_6dof : [x, y, z, rx, ry, rz] in mm, rad -- Cartesian pose for
        condition-number checks (the knot-0 pose)
    motor_rev : 6 absolute motor positions (rev) in ODrive convention
        (0 = STOW).  Includes the stow offset.  If None, the motor guard
        falls back to ``extensions_mm_to_revs(ext_mm)`` (no stow offset --
        only correct for unit tests / sim).
    vel_mm_s : 6 leg velocities (mm/s), or None for zeros
    acc_mm_s2 : 6 leg accelerations (mm/s²), or None for zeros.
        Used by the motor guard for cubic interpolation between 40 Hz
        commands (eliminates position step at command boundaries).
    torque_Nm : 6 motor torques (Nm), or None for zeros
    seq : monotonic sequence number (for debugging / drop detection)
    cmd_next_mm : 6 leg extensions (mm) for the next
        knot (u[1]).  When present, the motor guard uses cubic Hermite
        interpolation between ext_mm and cmd_next_mm instead of Taylor
        extrapolation from ext_mm alone.  None falls back to extrapolation.
    cmd_next2_mm : 6 leg extensions (mm) for the knot
        after next (u[2]).  When present with cmd_next_mm, the motor guard
        uses (u[2] - u[1]) / T as the endpoint velocity of the current
        Hermite segment, producing C1-continuous position across segment
        boundaries (eliminates 40 Hz velocity discontinuity).
    vel_next_mm_s : 6 leg extension RATES (mm/s) at the u[1] knot, or None.
        Fills the v6 Setpoint's exact ``v1[0:6]`` (behind ``HAS_V1``); absent
        keeps the firmware forward-difference fallback (the flown path).
    vel_next2_mm_s : 6 leg extension RATES (mm/s) at the u[2] knot, or None
        (C2FF spec, 2026-09-14). Fills the v8 Setpoint's ``v2[0:6]`` (behind
        ``HAS_V2``, which requires ``HAS_U2`` and ``HAS_V1``).
    hand_rev, hand_vel_rps, hand_next_rev, hand_next2_rev, hand_next_vel_rps,
    hand_next2_vel_rps, hand_acc_rps2 :
        the 7th (hand) channel — ODrive-convention absolute rev / rev/s at
        the u0, u0, u1, u2, u1 and u2 knots respectively, plus the plan's
        hand ACCELERATION (rev/s²) at u0 (``hand_acc_rps2``, C2FF spec,
        observability/cross-check only — torque is computed in firmware from
        the emitted curve, not from this field).  The pump maps the rev/vel
        keys to Setpoint index 6 behind ``HAS_HAND`` (``v1[6]`` behind
        ``HAS_V1``, ``v2[6]`` behind ``HAS_V2``) and ``hand_acc_rps2`` to
        ``accel[6]``.  All-or-nothing at the pump: see
        ``teensy_link/setpoint_pump.py``.
    knot_epoch_us : CLOCK_REALTIME microsecond stamp of this frame's own knot
        time (C2FF spec), or None. Present on EVERY frame with a value — hand
        track or not — since it names when the frame plays, not a hand fact.
        The pump maps it to the v8 Setpoint's ``t_origin_us`` behind
        ``HAS_SCHED``.

    NOTE — the nine v6/v8 keys (vel_next_mm_s, vel_next2_mm_s, the five
    v6-era hand keys, hand_next2_vel_rps and hand_acc_rps2) have
    ABSENT-when-None semantics even under ``out=`` reuse: a key a previous
    tick wrote is DELETED when this tick passes None.  This deliberately
    differs from the legacy fields' write-None-under-out behaviour, and it
    is load-bearing: with ``out=`` reuse, a hand VALUE stashed by a previous
    (hand-carrying) tick would otherwise survive into a later legacy frame
    and silently arm ``HAS_HAND`` with a stale hand target — the exact
    stale-key leak the emitter's byte-identity regression (T-U8/T-R1) pins
    against. ``knot_epoch_us`` is NOT in this group — it follows the LEGACY
    optional-field rule below (write when not-None-or-out-given, else absent
    on a fresh dict) because it is a per-frame timing fact independent of
    whether the frame carries a hand track, not a hand-track-gated value that
    must vanish the instant a plan stops carrying one.
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
    msg['type'] = 'setpoint_cmd'
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
    # knot_epoch_us (C2FF spec, 2026-09-14): the LEGACY optional-field rule —
    # every frame carries it (hand track or not), so it behaves like
    # cmd_next2_mm above, not like the hand-gated group below.
    if knot_epoch_us is not None or out is not None:
        msg['knot_epoch_us'] = knot_epoch_us
    elif 'knot_epoch_us' in msg:
        del msg['knot_epoch_us']
    # v6/v8 keys (2026-09-01 Phase 2 + 2026-09-14 C2FF spec): absent-when-None
    # EVEN under out= reuse — see the docstring NOTE for why a previously
    # written key must be deleted rather than overwritten with None.
    for key, val in (('vel_next_mm_s', vel_next_mm_s),
                     ('vel_next2_mm_s', vel_next2_mm_s),
                     ('hand_rev', hand_rev),
                     ('hand_vel_rps', hand_vel_rps),
                     ('hand_next_rev', hand_next_rev),
                     ('hand_next2_rev', hand_next2_rev),
                     ('hand_next_vel_rps', hand_next_vel_rps),
                     ('hand_next2_vel_rps', hand_next2_vel_rps),
                     ('hand_acc_rps2', hand_acc_rps2)):
        if val is not None:
            msg[key] = val
        elif key in msg:
            del msg[key]
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
# MVP trajectory generator side — publishes setpoint commands on :5557
# ---------------------------------------------------------------------------

class SetpointCommandPub:
    """PUB endpoint binding SETPOINT_COMMAND_ADDR (:5557), topic ``setpoint``.

    Used by ``trajectory_node`` to stream ``make_setpoint_command`` dicts on the
    exact seam the bridge's ``_SetpointCommandSource`` consumes, through the
    shared ``_pack`` (msgpack + ndarray default handler) on ``TOPIC_SETPOINT_CMD``.
    (Formerly ``MpcCommandPub``: the removed MPC chain's ``HardwarePlant`` was
    the original binder, git tag ``mpc-final``.)

    **Sole-binder interlock.** ZMQ does not allow two PUB ``bind()``s on one
    address, so a second binder raises ``zmq.ZMQError`` rather than silently
    interleaving two command streams onto the legs. ``trajectory_node`` is the
    only binder today; the caller is still expected to translate a bind failure
    into a loud, fatal startup error rather than continuing.
    """

    def __init__(self, addr: str = SETPOINT_COMMAND_ADDR):
        self._ctx = zmq.Context()
        self._pub = self._ctx.socket(zmq.PUB)
        # bind() raises zmq.ZMQError (EADDRINUSE) if another PUB already holds
        # the address — the sole-binder interlock. Let it propagate.
        self._pub.bind(addr)
        self._addr = addr

    def send(self, msg: dict) -> None:
        """Publish a setpoint command dict (see ``make_setpoint_command``)."""
        self._pub.send_multipart(_pack(TOPIC_SETPOINT_CMD, msg), flags=zmq.NOBLOCK)

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
      - ``_sub_setpoint_cmd``: setpoint commands (no CONFLATE, RCVHWM=2; separate port)
      - ``_sub_setpoint_mode``: mode commands on the command port (no CONFLATE,
        separate socket)

    Motor feedback gets its own CONFLATE socket so that high-frequency
    feedback (100 Hz) cannot overwrite mode messages and vice-versa.
    Without this separation, ZMQ CONFLATE keeps only the single latest
    message *regardless of topic prefix*, silently dropping whichever
    message arrived first.

    Mode commands on the command port (enable/disable/estop) get their own
    non-CONFLATE socket for the same reason — mixing them
    with setpoint commands on a single CONFLATE socket would silently drop
    whichever message arrived first in the same poll window.

    The command socket listens on a separate port (SETPOINT_COMMAND_ADDR)
    because the bridge already ``bind()``s PUB on COMMAND_ADDR -- ZMQ does
    not support two PUB ``bind()``s on the same address.
    """

    def __init__(self,
                 command_addr: str = COMMAND_ADDR,
                 telemetry_addr: str = TELEMETRY_ADDR,
                 setpoint_command_addr: str = SETPOINT_COMMAND_ADDR):
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

        # SUB socket for setpoint commands -- separate port.
        # No CONFLATE: ZMQ 4.3.5 drops all messages when CONFLATE + topic
        # filter are combined on a late-connecting SUB.  Instead, recv_all()
        # drains all pending messages and the caller uses the latest.
        # Cap reconnect interval: the command publisher (trajectory_node) binds PUB
        # on this port minutes after the motor guard starts.
        self._sub_setpoint_cmd = self._ctx.socket(zmq.SUB)
        self._sub_setpoint_cmd.setsockopt(zmq.RECONNECT_IVL, 100)      # 100ms base
        self._sub_setpoint_cmd.setsockopt(zmq.RECONNECT_IVL_MAX, 200)  # 200ms cap
        self._sub_setpoint_cmd.connect(setpoint_command_addr)
        self._sub_setpoint_cmd.setsockopt(zmq.SUBSCRIBE, TOPIC_SETPOINT_CMD)
        self._sub_setpoint_cmd.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking
        self._sub_setpoint_cmd.setsockopt(zmq.RCVHWM, 2)    # small buffer, drain to latest

        # SUB socket for mode commands on the command port -- no CONFLATE,
        # every enable/disable/estop is delivered.  Separate from _sub_setpoint_cmd
        # because CONFLATE keeps only the single latest message regardless of
        # topic prefix — mixing mode + setpoints on one CONFLATE socket silently
        # drops whichever arrives first in the same poll window.
        self._sub_setpoint_mode = self._ctx.socket(zmq.SUB)
        self._sub_setpoint_mode.setsockopt(zmq.RECONNECT_IVL, 100)      # 100ms base
        self._sub_setpoint_mode.setsockopt(zmq.RECONNECT_IVL_MAX, 200)  # 200ms cap
        self._sub_setpoint_mode.connect(setpoint_command_addr)
        self._sub_setpoint_mode.setsockopt(zmq.SUBSCRIBE, TOPIC_MODE)
        self._sub_setpoint_mode.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking
        self._sub_setpoint_mode.setsockopt(zmq.RCVHWM, 64)   # bound queue size

        # PUB socket -- publishes telemetry to bridge
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.bind(telemetry_addr)

        self._last_recv_time = time.monotonic()

    def recv_all(self) -> list[tuple[bytes, dict]]:
        """Non-blocking: receive all pending messages.

        Drains mode commands first (critical), then setpoint commands, then
        motor feedback.  Returns list of (topic, message_dict) tuples.
        Empty list if nothing available.
        """
        messages = []
        # Drain mode commands first -- these are rare but critical.
        # Both bridge mode (_sub_mode) and command-port mode (_sub_setpoint_mode)
        # are drained before setpoint commands.
        for sub in (self._sub_mode, self._sub_setpoint_mode, self._sub_setpoint_cmd,
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
        self._sub_setpoint_cmd.close()
        self._sub_setpoint_mode.close()
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
# Session metadata — REMOVED 2026-09-01 with the MPC chain (git tag mpc-final).
# SessionMetadataPush (sim/main.py --mpc) and SessionMetadataPull (mpc_bridge_node)
# carried the telemetry-CSV filename over :5560 for ROS2 log correlation; both
# endpoints are deleted, so the channel had no producer and no consumer.
# See logbook/2026-09-01-mpc-chain-removed.md.
# ---------------------------------------------------------------------------
