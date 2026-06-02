"""Pack motor_guard telemetry ticks into Setpoint frames, with a per-step gate.

Pure logic — no ROS, no ZMQ, no UDP. The bridge node owns the transport (ZMQ
ingress from motor_guard on :5556, UDP egress to the Teensy, the dedicated
ingest thread); :class:`SetpointPump` owns the **safety-critical** packing +
per-step clamp so they can be unit-tested in isolation.

Frame mapping (motor_guard ``_publish_telemetry`` dict → ``Setpoint``):

  * ``leg_pos``     → ``u0``        (motor-rev positions, Jugglebot convention:
                                     positive = extension — the Teensy
                                     odrive_protocol flips the sign, ADR-0012)
  * ``leg_vel``     → ``v0``        (velocity feedforward, rev/s)
  * ``leg_torques`` → ``torque_ff`` (Nm). NOTE: motor_guard already SUMS
    gravity+inertia base torque AND its per-leg Stribeck friction FF into
    ``_commanded_torque_ff_Nm`` (motor_guard.py:997-999, friction_ff.enabled
    default true), so the forwarded torque INCLUDES friction FF — it is computed
    Jetson-side, not Teensy-side. The Teensy applies it verbatim and computes no
    friction itself (cf. ADR-0012/firmware-D9: the Teensy does not *compute*
    friction FF; that does not mean the delivered torque excludes it).
  * ``u1``/``u2``/``accel`` = 0; ``flags`` = 0 (no u1/u2 lookahead — motor_guard
    already ran the 500 Hz interpolator, so each tick is a single dense waypoint;
    per firmware handoff D4 absence is signalled by clear flag bits, NOT NaN).

Per-step safety gate — a port of ``can_node._sub_leg_lengths``'s
``JB_OP_MAX_POSITION_STEP_REV`` clamp (can_node.py:1046-1056). Reject any frame
whose commanded leg position jumps more than ``max_step_rev`` from the PRIOR
ACCEPTED frame. The first frame has no prior and is accepted; the firmware's
own ``MAX_DEVIATION`` gate (commanded-vs-encoder, fault_machine.cpp
``evaluate_guard``) is the complementary defence-in-depth layer that catches a
bad *first* command. Two independent layers: the bridge gates command-stream
discontinuity (prior frame); the Teensy gates command-vs-encoder divergence.

Whether the resulting frame is actually transmitted is gated separately by the
node's ``mpc_active`` / ``enable_setpoint_output`` (default off) — this module
only decides *what* a well-formed frame would be and *whether it is safe to
send*.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

from . import protocol as p
from .protocol import Setpoint

# Default per-step clamp (rev). The bridge passes the authoritative
# hardware_config.JB_OP_MAX_POSITION_STEP_REV; this default mirrors it so the
# module is usable standalone in tests.
DEFAULT_MAX_STEP_REV = 0.3


class SetpointPump:
    """Stateful packer + per-step gate for the 40/500 Hz setpoint downlink.

    Args:
        num_legs: Number of leg axes (Setpoint carries 6).
        max_step_rev: Per-leg per-frame position-step clamp (rev).
    """

    def __init__(self, num_legs: int = p.NUM_LEGS,
                 max_step_rev: float = DEFAULT_MAX_STEP_REV):
        self.n = int(num_legs)
        self.max_step_rev = float(max_step_rev)
        self._prev_pos: Optional[list] = None
        self.frames_built = 0
        self.frames_skipped = 0      # feedback-only / no-command ticks (not a fault)
        self.frames_rejected = 0     # SAFETY rejects (NaN, short, step violation)
        self.last_reject_reason = ''

    def reset(self) -> None:
        """Forget the prior frame (e.g. after a link loss / re-enable)."""
        self._prev_pos = None

    def build(self, telem: dict, t_origin_us: int
              ) -> Tuple[Optional[Setpoint], Optional[str]]:
        """Validate + pack one telemetry tick.

        Returns ``(setpoint, reject_reason)``:
          * ``(Setpoint, None)`` — a safe frame to send.
          * ``(None, None)``     — nothing to send (feedback-only / fault
            telemetry with no motor command); NOT a fault.
          * ``(None, reason)``   — a SAFETY reject (do not send, surface a fault).
        """
        pos = telem.get('leg_pos')
        vel = telem.get('leg_vel')
        tor = telem.get('leg_torques')

        # Feedback-only / ESTOP telemetry carries None commands. motor_guard's
        # _publish_fault_telemetry / _publish_feedback_telemetry set these to
        # None precisely so a consumer won't command. Skip silently.
        if pos is None or vel is None or tor is None:
            self.frames_skipped += 1
            return None, None

        if len(pos) < self.n or len(vel) < self.n or len(tor) < self.n:
            self.frames_rejected += 1
            self.last_reject_reason = (
                f'short command vector (pos={len(pos)}, vel={len(vel)}, '
                f'tor={len(tor)}; need {self.n})')
            return None, self.last_reject_reason

        # NaN/Inf reject (mirror can_node's "Leg command contains NaN or Inf").
        for name, seq in (('pos', pos), ('vel', vel), ('tor', tor)):
            for i in range(self.n):
                if not math.isfinite(seq[i]):
                    self.frames_rejected += 1
                    self.last_reject_reason = f'non-finite {name}[{i}]'
                    return None, self.last_reject_reason

        # Per-step gate vs the prior ACCEPTED frame (NOT vs rejected frames, so a
        # burst of bad frames is all measured against the last good command).
        if self._prev_pos is not None:
            for i in range(self.n):
                step = abs(float(pos[i]) - self._prev_pos[i])
                if step > self.max_step_rev:
                    self.frames_rejected += 1
                    self.last_reject_reason = (
                        f'leg {i} step {step:.4f} rev > {self.max_step_rev} '
                        f'limit (cmd={float(pos[i]):.4f}, '
                        f'prev={self._prev_pos[i]:.4f})')
                    return None, self.last_reject_reason

        sp = Setpoint(
            u0=tuple(float(pos[i]) for i in range(self.n)),
            u1=(0.0,) * self.n,
            u2=(0.0,) * self.n,
            v0=tuple(float(vel[i]) for i in range(self.n)),
            accel=(0.0,) * self.n,
            torque_ff=tuple(float(tor[i]) for i in range(self.n)),
            flags=0,  # no u1/u2 lookahead present
            t_origin_us=int(t_origin_us),
        )
        self._prev_pos = [float(pos[i]) for i in range(self.n)]
        self.frames_built += 1
        return sp, None
