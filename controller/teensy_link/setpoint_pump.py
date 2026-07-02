"""Pack 40 Hz MPC commands into β-knot Setpoint frames, with a per-step gate.

Pure logic — no ROS, no ZMQ, no UDP. The bridge node owns the transport (ZMQ
ingress from the MPC on :5557, UDP egress to the Teensy, the dedicated ingest
thread); :class:`SetpointPump` owns the **safety-critical** packing + per-step
clamp so they can be unit-tested in isolation.

**Phase 11 / U4 — the α→β switch.** Historically this pump consumed
``motor_guard``'s *already-interpolated* 500 Hz telemetry (:5556, ``flags=0``,
the **α relay**) and the Teensy ran in Mode-2 pass-through while motor_guard's
500 Hz Hermite stayed on the Jetson. U4 re-points it at the **40 Hz MPC command
stream** (:5557, ``TOPIC_MPC_CMD``, ``make_mpc_command``) and emits **β knots**
(``u0``/``u1``/``u2`` + ``v0``, ``flags`` carrying ``HAS_U1``/``HAS_U2``) so the
Teensy's 500 Hz cubic-Hermite interpolator (``leg_interp.cpp`` Mode 1) does the
interpolation — the migration's headline benefit, finally realised. motor_guard
leaves the leg path entirely; leg safety becomes **MPC (coupled workspace) + the
Teensy fault machine** (per-leg stroke / deviation / MPC-staleness E-STOP /
deferred-stow — all U3-validated). See
``logbook/2026-06-25-phase11-u4-production-cutover.md``.

**Bumplessness — the load-bearing invariant.** The β knots are derived to
reproduce ``MotorGuard._on_mpc_command``'s EXACT knot latch
(``motor_guard.py:541–603``), so the Teensy Hermite — cross-checked bit-for-bit
against motor_guard's interpolator in
``tools/probes/teensy_link_profiling/hermite_xref/xref.py`` (< 1e-6 rev) — emits
the same trajectory the α path would have. The derivation MIXES conventions
exactly as motor_guard does (verified empirically, U4 probe):

  * ``u0`` = ``cmd['motor_rev']`` (ODrive convention, 0 = STOW, **includes** any
    stow offset) if present, else ``cmd['ext_mm'] × mm_to_rev`` (the unit-test /
    sim fallback, no offset). Using ``motor_rev`` verbatim when present is what
    keeps the switch bumpless — falling back to ``ext × mm_to_rev`` would jump
    the leg by the stow offset the instant the source flips.
  * ``u1`` = ``cmd['cmd_next_mm'] × mm_to_rev``; ``u2`` =
    ``cmd['cmd_next2_mm'] × mm_to_rev`` (extension convention, no offset). Absent
    / non-finite / wrong-length ⇒ the corresponding ``HAS_U1`` / ``HAS_U2`` flag
    is **cleared** — never a NaN sentinel (firmware handoff D4). ``u2`` is only
    meaningful with ``u1`` (it sets the Hermite endpoint velocity
    ``v1 = (u2 − u1)/T``), so ``HAS_U2`` is gated on ``HAS_U1``.
  * ``v0`` = ``cmd['vel_mm_s'] × mm_to_rev`` (=
    ``conversions.leg_velocities_to_motor_velocities``; the conversion is a pure
    elementwise scale, so the pump needs only the ``mm_to_rev`` vector — no
    ``jugglebot.motion`` import, preserving purity).
  * ``accel`` = 0 — the firmware Mode-1 Hermite does NOT consume ``accel`` (only
    Mode-2 Taylor does, when ``HAS_U1`` is clear; production always supplies
    ``cmd_next_mm`` so Mode 1 always runs). Matches the bench β sources
    (``synthetic_setpoint`` / ``replay_setpoint``).
  * ``torque_ff`` = 0 — the **friction-FF drop** (decision D9). The α source
    carried motor_guard's Stribeck friction-FF in ``leg_torques``; the β MPC
    stream carries ``torque_Nm = zeros``. U3-iv measured the on-hardware penalty
    as null (the smooth gate suppresses FF at v≈0, so its loss is free at
    breakaway) and the float32 interp residual as 5.5e-7 rev — both within the
    pre-registered D9 criteria.

Per-step safety gate — a port of ``can_node._sub_leg_lengths``'s
``JB_OP_MAX_POSITION_STEP_REV`` clamp (can_node.py:1046-1056). Reject any frame
whose commanded ``u0`` jumps more than ``max_step_rev`` from the PRIOR ACCEPTED
frame. The first frame has no prior and is accepted; the firmware's own
``MAX_DEVIATION`` gate (commanded-vs-encoder, fault_machine.cpp
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
from typing import Optional, Sequence, Tuple

from . import protocol as p
from .protocol import Setpoint

# Setpoint flag bits — u1/u2 lookahead presence (firmware D4: bits, not NaN).
# Mirrors synthetic_setpoint.py / replay_setpoint.py (the bench β sources).
FLAG_HAS_U1 = 0x1
FLAG_HAS_U2 = 0x2

# Default per-step clamp (rev). The bridge passes the authoritative
# hardware_config.JB_OP_MAX_POSITION_STEP_REV; this default mirrors it so the
# module is usable standalone in tests.
DEFAULT_MAX_STEP_REV = 0.3


class SetpointPump:
    """Stateful β-knot packer + per-step gate for the 40/500 Hz setpoint downlink.

    Args:
        mm_to_rev: per-leg mm→motor-rev scale (``hardware_config.GEOM_MM_TO_REV``).
            Injected because the pump now derives motor-rev knots from the MPC's
            mm-space command fields (``ext_mm``/``cmd_next_mm``/``vel_mm_s``);
            kept as a constructor arg (not a per-build param or a
            ``jugglebot.motion`` import) so the module stays pure.
        num_legs: number of leg axes (Setpoint carries 6).
        max_step_rev: per-leg per-frame ``u0`` step clamp (rev).
    """

    def __init__(self, mm_to_rev: Sequence[float],
                 num_legs: int = p.NUM_LEGS,
                 max_step_rev: float = DEFAULT_MAX_STEP_REV):
        self.n = int(num_legs)
        self.mm_to_rev = tuple(float(x) for x in mm_to_rev)
        if len(self.mm_to_rev) < self.n:
            raise ValueError(
                f"mm_to_rev needs >= {self.n} entries, got {len(self.mm_to_rev)}")
        self.max_step_rev = float(max_step_rev)
        self._prev_pos: Optional[list] = None
        self.frames_built = 0
        self.frames_skipped = 0      # no-command ticks (not a fault)
        self.frames_rejected = 0     # SAFETY rejects (NaN, short, step violation)
        self.last_reject_reason = ''

    def reset(self) -> None:
        """Forget the prior frame (e.g. after a link loss / re-enable)."""
        self._prev_pos = None

    def _finite_vec(self, seq, name: str):
        """Validate a 6-vector is present, the EXACT length, and finite.

        Exact-length (not just ``>= n``) mirrors motor_guard's ``shape == (6,)``
        gate (motor_guard.py:445, 585) — a wrong-length command vector is
        malformed, so reject rather than silently take the first ``n``.

        Robust to a malformed message: a scalar / non-sequence ``seq`` or a
        ``None`` / non-numeric element returns ``(None, reason)`` rather than
        raising, so one bad MPC frame is a REJECT (surfaced + counted), never a
        crash of the setpoint thread.

        Returns ``(values_list, None)`` on success or ``(None, reason)``.
        """
        try:
            length = len(seq)
        except TypeError:
            return None, f'{name} is not a sequence ({type(seq).__name__})'
        if length != self.n:
            return None, f'wrong-length {name} vector (len={length}, need {self.n})'
        out = []
        for i in range(self.n):
            try:
                v = float(seq[i])
            except (TypeError, ValueError):
                return None, f'non-numeric {name}[{i}] ({seq[i]!r})'
            if not math.isfinite(v):
                return None, f'non-finite {name}[{i}]'
            out.append(v)
        return out, None

    def build(self, cmd: dict, t_origin_us: int
              ) -> Tuple[Optional[Setpoint], Optional[str]]:
        """Validate + pack one 40 Hz MPC command (:5557 ``mpc_cmd`` dict).

        Returns ``(setpoint, reject_reason)``:
          * ``(Setpoint, None)`` — a safe β-knot frame to send.
          * ``(None, None)``     — nothing to send (no position command in the
            message); NOT a fault.
          * ``(None, reason)``   — a SAFETY reject (do not send, surface a fault).

        REJECT-NOT-RAISE contract: this is called on the production setpoint
        thread, so ANY malformed command (a scalar/None/wrong-type field that slips
        the checks below) must surface as a counted reject, never an exception that
        kills the thread while ``mpc_active`` stays 1. The explicit ``_finite_vec``
        checks catch the known-malformed shapes; this outer guard is the
        belt-and-suspenders backstop for anything they miss.
        """
        try:
            return self._build(cmd, t_origin_us)
        except Exception as e:  # noqa: BLE001 — one bad frame must not crash the thread
            self.frames_rejected += 1
            self.last_reject_reason = f'malformed cmd: {e!r}'
            return None, self.last_reject_reason

    def _build(self, cmd: dict, t_origin_us: int
               ) -> Tuple[Optional[Setpoint], Optional[str]]:
        mr = self.mm_to_rev

        # ── u0: motor_rev (ODrive conv) if present, else ext_mm × mm_to_rev ──
        motor_rev = cmd.get('motor_rev')
        ext = cmd.get('ext_mm')
        if motor_rev is not None:
            u0, reason = self._finite_vec(motor_rev, 'motor_rev')
            if reason is not None:
                self.frames_rejected += 1
                self.last_reject_reason = reason
                return None, reason
        elif ext is not None:
            ext_vals, reason = self._finite_vec(ext, 'ext_mm')
            if reason is not None:
                self.frames_rejected += 1
                self.last_reject_reason = reason
                return None, reason
            u0 = [ext_vals[i] * mr[i] for i in range(self.n)]
        else:
            # No position command in this message (e.g. a non-mpc_cmd frame that
            # slipped the topic filter). Nothing to command — skip, not a fault.
            self.frames_skipped += 1
            return None, None

        # ── v0: vel_mm_s × mm_to_rev (motor_guard's leg_velocities_to_motor_…) ──
        # Production HardwarePlant always supplies vel_mm_s. motor_guard would
        # finite-difference a missing/bad velocity, but the pump has no per-tick
        # ext history to do so and a wrong feedforward velocity is unsafe, so a
        # missing/non-finite velocity is a SAFETY reject here (never silently 0).
        vel = cmd.get('vel_mm_s')
        if vel is None:
            self.frames_rejected += 1
            self.last_reject_reason = 'missing vel_mm_s'
            return None, self.last_reject_reason
        vel_vals, reason = self._finite_vec(vel, 'vel_mm_s')
        if reason is not None:
            self.frames_rejected += 1
            self.last_reject_reason = reason
            return None, reason
        v0 = [vel_vals[i] * mr[i] for i in range(self.n)]

        # ── u1 / u2: cmd_next(_2)_mm × mm_to_rev; absent/bad ⇒ clear the flag ──
        # Mirrors motor_guard: a non-finite / wrong-length lookahead clears the
        # waypoint rather than rejecting the frame (firmware D4: bits, not NaN).
        # Exact-length (== n, not >= n) mirrors motor_guard's ``shape == (6,)``
        # gate (motor_guard.py:585, 597) so a malformed >6-element lookahead
        # clears the flag (Taylor fallback) instead of silently taking the
        # first 6 and emitting a divergent-but-accepted β frame.
        # Absent/malformed lookahead CLEARS the flag (never rejects the frame,
        # never raises) — routed through the hardened _finite_vec so a None/
        # non-numeric/wrong-length cmd_next just falls back to the Taylor path.
        flags = 0
        u1 = [0.0] * self.n
        u2 = [0.0] * self.n
        cmd_next = cmd.get('cmd_next_mm')
        if cmd_next is not None:
            cand, _reason = self._finite_vec(cmd_next, 'cmd_next_mm')
            if cand is not None:
                u1 = [cand[i] * mr[i] for i in range(self.n)]
                flags |= FLAG_HAS_U1
        # u2 only meaningful with u1 (sets the Hermite endpoint velocity).
        cmd_next2 = cmd.get('cmd_next2_mm')
        if (flags & FLAG_HAS_U1) and cmd_next2 is not None:
            cand, _reason = self._finite_vec(cmd_next2, 'cmd_next2_mm')
            if cand is not None:
                u2 = [cand[i] * mr[i] for i in range(self.n)]
                flags |= FLAG_HAS_U2

        # ── Per-step gate vs the prior ACCEPTED frame (NOT vs rejected frames) ──
        if self._prev_pos is not None:
            for i in range(self.n):
                step = abs(u0[i] - self._prev_pos[i])
                if step > self.max_step_rev:
                    self.frames_rejected += 1
                    self.last_reject_reason = (
                        f'leg {i} step {step:.4f} rev > {self.max_step_rev} '
                        f'limit (cmd={u0[i]:.4f}, prev={self._prev_pos[i]:.4f})')
                    return None, self.last_reject_reason

        sp = Setpoint(
            u0=tuple(u0),
            u1=tuple(u1),
            u2=tuple(u2),
            v0=tuple(v0),
            accel=(0.0,) * self.n,          # Mode-1 Hermite ignores accel
            torque_ff=(0.0,) * self.n,      # friction-FF drop (D9)
            flags=flags,
            t_origin_us=int(t_origin_us),
        )
        self._prev_pos = list(u0)
        self.frames_built += 1
        return sp, None
