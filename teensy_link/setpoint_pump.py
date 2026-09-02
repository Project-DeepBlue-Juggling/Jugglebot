"""Pack 40 Hz MPC commands into Teensy-side knot Setpoint frames, with a per-step gate.

Pure logic — no ROS, no ZMQ, no UDP. The bridge node owns the transport (ZMQ
ingress from the MPC on :5557, UDP egress to the Teensy, the dedicated ingest
thread); :class:`SetpointPump` owns the **safety-critical** packing + per-step
clamp so they can be unit-tested in isolation.

**The Jetson-relay→Teensy-side switch.** Historically this pump consumed
``motor_guard``'s *already-interpolated* 500 Hz telemetry (:5556, ``flags=0``,
the **Jetson-relay path**) and the Teensy ran in Mode-2 pass-through while motor_guard's
500 Hz Hermite stayed on the Jetson. This pump re-points it at the **40 Hz MPC command
stream** (:5557, ``TOPIC_MPC_CMD``, ``make_mpc_command``) and emits **Teensy-side knots**
(``u0``/``u1``/``u2`` + ``v0``, ``flags`` carrying ``HAS_U1``/``HAS_U2``) so the
Teensy's 500 Hz cubic-Hermite interpolator (``leg_interp.cpp`` Mode 1) does the
interpolation — the migration's headline benefit, finally realised. motor_guard
leaves the leg path entirely; leg safety becomes **MPC (coupled workspace) + the
Teensy fault machine** (per-leg stroke / deviation / MPC-staleness E-STOP /
deferred-stow — all bench-validated). See
``logbook/2026-06-25-phase11-u4-production-cutover.md``.

The 7-channel v6 frame (2026-09-01, unified-7dof-planner Phase 2)
-----------------------------------------------------------------
The wire ``Setpoint`` carries **7 lanes** (legs 0..5 + hand at index 6) and a
new ``v1`` f32[7] array — the exact velocity at the ``u1`` knot — behind flag
bit 3 ``HAS_V1``. This pump fills them from six new OPTIONAL ``mpc_cmd`` keys:

  * ``hand_rev`` / ``hand_vel_rps`` / ``hand_next_rev`` / ``hand_next2_rev``
    → ``u0[6]`` / ``v0[6]`` / ``u1[6]`` / ``u2[6]``. **ODrive-convention
    absolute rev, NO sign flip and NO host-side scaling** — the firmware's
    ``encode_leg_setpoint`` already applies the hand's 100/100 wire scales
    (vs the legs' negate + 1000/10000) for axis 6, so the host puts RAW rev
    values on the wire.
  * ``vel_next_mm_s`` (6-wide leg extension rates at the u1 knot, mm/s)
    → ``v1[0:6]`` × mm_to_rev; ``hand_next_vel_rps`` → ``v1[6]``.

Presence semantics (each documented rule is pinned by a test in
``tests/teensy_link/test_setpoint_pump.py``):

  * **Hand keys are ALL-OR-NOTHING.** If ANY ``hand_*`` key is present
    (``hand_next_vel_rps`` included), then ``hand_rev`` + ``hand_vel_rps`` +
    ``hand_next_rev`` + ``hand_next2_rev`` must ALL be present and finite —
    anything else is a loud SAFETY REJECT. **NaN/non-finite in ANY hand key
    REJECTS the frame** (never a flag-clear fallback: the hand is a
    221 rev/s axis, and a silently-degraded hand lane is exactly the class
    of failure the legs' deliberate u1/u2 flag-clear fallback — which stays
    — is too gentle for).
  * All four hand keys absent ⇒ ``HAS_HAND`` clear, index 6 of every array
    packed 0.0 — the leg lanes are **byte-identical** to the pre-hand frame
    (regression-pinned against the captured v5 fixtures).
  * **Hand streaming requires the full Mode-1 knot set**: hand keys present
    while the leg ``cmd_next_mm`` / ``cmd_next2_mm`` lookahead is absent (or
    malformed ⇒ flag-cleared) is an incoherent producer state and REJECTS
    the frame — the firmware interp cannot run a 7th Hermite lane against a
    leg Taylor fallback, and a producer that knows the hand's next two knots
    but not the legs' is broken, not degraded. The degenerate case rejects
    the same way: hand keys aboard a frame with NO leg position command at
    all (``motor_rev`` and ``ext_mm`` both absent) are a loud SAFETY reject,
    never the silent ``frames_skipped`` exit a legs-only no-command tick
    takes.
  * **``HAS_V1`` requires EVERY active channel's v1**: ``vel_next_mm_s``
    always, plus ``hand_next_vel_rps`` whenever the hand keys are present. A
    partial set (either one without the other it needs) REJECTS the frame
    loudly — counted and reasoned, per the NaN-reject precedent. Both v1
    keys absent ⇒ ``HAS_V1`` clear and the firmware's ``(u2−u1)/SEG_T``
    forward-difference fallback stays the flown path. A present-but-
    non-finite/wrong-length ``vel_next_mm_s`` is a REJECT (the v0
    precedent: a wrong velocity feedforward is unsafe, never silently
    dropped to the fallback). ``HAS_V1`` also requires the full leg Mode-1
    knot set (``HAS_U1`` AND ``HAS_U2``): v1 is the velocity AT the u1
    knot, so ``vel_next_mm_s`` aboard a frame whose leg lookahead is
    absent or flag-cleared describes a knot the frame does not carry —
    incoherent, loud REJECT (counted + reasoned, per the hand coherence
    rule above).
  * ``torque_ff[6] = 0`` **always** in Phase 2 (no hand feedforward), and
    ``accel[6] = 0`` (Mode-1 ignores accel on every lane).

**Bumplessness — the load-bearing invariant.** The Teensy-side knots are derived to
reproduce ``MotorGuard._on_mpc_command``'s EXACT knot latch
(``motor_guard.py:541–603``), so the Teensy Hermite — cross-checked bit-for-bit
against motor_guard's interpolator in
``tools/probes/teensy_link_profiling/hermite_xref/xref.py`` (< 1e-6 rev) — emits
the same trajectory the Jetson-relay path would have. The derivation MIXES conventions
exactly as motor_guard does (verified empirically):

  * ``u0`` = ``cmd['motor_rev']`` (ODrive convention, 0 = STOW, **includes** any
    stow offset) if present, else ``cmd['ext_mm'] × mm_to_rev`` (the unit-test /
    sim fallback, no offset). Using ``motor_rev`` verbatim when present is what
    keeps the switch bumpless — falling back to ``ext × mm_to_rev`` would jump
    the leg by the stow offset the instant the source flips.
  * ``u1`` = ``cmd['cmd_next_mm'] × mm_to_rev``; ``u2`` =
    ``cmd['cmd_next2_mm'] × mm_to_rev`` (extension convention, no offset). Absent
    / non-finite / wrong-length ⇒ the corresponding ``HAS_U1`` / ``HAS_U2`` flag
    is **cleared** — never a NaN sentinel (a firmware wire-format decision). ``u2`` is only
    meaningful with ``u1`` (it sets the Hermite endpoint velocity
    ``v1 = (u2 − u1)/T``), so ``HAS_U2`` is gated on ``HAS_U1``.
  * ``v0`` = ``cmd['vel_mm_s'] × mm_to_rev`` (=
    ``conversions.leg_velocities_to_motor_velocities``; the conversion is a pure
    elementwise scale, so the pump needs only the ``mm_to_rev`` vector — no
    ``jugglebot.motion`` import, preserving purity).
  * ``accel`` = 0 — the firmware Mode-1 Hermite does NOT consume ``accel`` (only
    Mode-2 Taylor does, when ``HAS_U1`` is clear; production always supplies
    ``cmd_next_mm`` so Mode 1 always runs). Matches the bench Teensy-side sources
    (``synthetic_setpoint`` / ``replay_setpoint``).
  * ``torque_ff`` = the leg torque feedforward — **default 0**, see below.

The leg torque feedforward — THE SINGLE WIRE ENFORCEMENT POINT
--------------------------------------------------------------
Historically this was hard-zeroed (the *friction-FF drop* at the 2026-06-25 Teensy
cutover). It is now a plumbed, live path (shipped ON since the 2026-07-16
arming session): the producer
(``jugglebot.motion.torque_ff.LegTorqueFeedforward``, via the trajectory emitter;
``HardwarePlant`` was a second producer until it was removed 2026-09-01, tag
mpc-final) publishes ``torque_Nm`` in **TRUE Nm, extension-positive**, and this
pump is the ONE place that turns a physical torque into an ODrive wire value. Three
transformations happen here and nowhere else, because this is the sole production
producer of the leg ``Setpoint`` frame:

1. **Clamp** to ``±torque_ff_max_nm`` (TRUE Nm), per leg. This is the FIRST-binding
   layer of a three-layer chain (see ``hardware_config.yaml`` →
   ``torque_ff_firmware_clamp_wire_nm``): (1) this pump clamp (0.15 true ⇒ 0.1451
   wire-Nm at the 0.9673 scale); (2) the can-bridge firmware ingest backstop (±0.25 wire-Nm,
   ``leg_interp.cpp`` — landed 2026-07-14, ACTIVE ONLY AFTER A REFLASH; on older
   firmware the only downstream bound is int16 saturation at ``±3.2767`` ODrive-Nm
   ≈ **59 A** of demand at ``LEG_TOR_SCALE`` = 10000); (3) the ODrive current clamp.
   The ODrive **adds** ``input_torque`` to the velocity loop's output *before* the
   torque limit, so a feedforward above ~0.55 wire-Nm (= ``current_soft_max`` 10 A ×
   ``torque_constant`` 0.055133) saturates the current clamp and the **position loop
   loses all authority** — the leg then runs open-loop at full current until
   ``MAX_DEVIATION`` E-STOPs it mid-motion. This pump clamp is the primary guard
   standing between a mis-modelled FF and that failure mode.
2. **Kt wire scale** (``× ODRIVE_LEG_TORQUE_WIRE_SCALE`` = ``Kt_odrive_config/Kt_measured``
   = 0.055133/0.0570 = 0.9673 since the 2026-07-15 weighed-mass confirmation; the
   pre-measurement value 0.8835 assumed the stiction-biased Kt = 0.0624). The drives
   are flashed with ODrive's nameplate ``torque_constant = 8.27/Kv = 0.055133`` and
   compute ``iq = input_torque / torque_constant``, while the motor's measured Kt is
   0.0570 ± 0.0008 Nm/A (two pooled friction-cancelling traverse sessions). Pre-scaling here makes the **delivered current** physically correct —
   the only thing a torque FF actually needs — without touching the flashed
   ``torque_constant``, which would silently detune the bench-validated velocity loop
   (its authority is ``vel_gain / torque_constant`` amps per rev/s). See the WHY block
   at ``config/hardware_config.yaml:dynamics.motor_kt_odrive_config_nm_per_a``.
3. **Ramp-in** over ``torque_ff_ramp_frames`` accepted frames, restarted by
   :meth:`reset`. When streaming begins, the leg is already held at the activate pose
   and the ODrive's velocity **integrator has already wound up to carry gravity**;
   applying the gravity FF as a step would briefly command ~2× gravity until that
   integrator unwinds. Ramping on *accepted frames* (not wall time) makes it
   deterministic and means a dropped frame lengthens the ramp — the safe direction.

Sign: the wire value stays **extension-positive**. The firmware's ``encode_leg_setpoint``
(``odrive_protocol.h:166-179``) applies ``leg_sign`` — a negation for leg axes — to
position, vel_ff and torque_ff alike, so a positive wire torque becomes a negative ODrive
torque, which drives ODrive position negative, which is leg extension, which is
platform-up. **Do not pre-negate here.**

With ``torque_ff_enabled=False`` (the constructor default; the shipped config
sets it true since 2026-07-16) this whole path is inert:
``torque_ff = (0.0,)*n`` and ``torque_Nm`` is not even read — byte-identical to the
pre-feature frame, including when the field is absent or malformed.

Per-step safety gate — a port of ``can_node._sub_leg_lengths``'s
``JB_OP_MAX_POSITION_STEP_REV`` clamp (can_node.py:1046-1056), now PER-CHANNEL.
Reject any frame whose commanded ``u0`` jumps more than ``max_step_rev`` (legs,
0.3 rev) — or whose hand ``u0[6]`` jumps more than ``max_step_hand_rev``
(5.0 rev at the shipped config) — from the PRIOR ACCEPTED frame's same channel.
The two gates reject independently; a leg violation does not implicate the hand
and vice versa. The first frame has no prior and is accepted — and the hand
mirrors that rule per-channel: the first hand-carrying frame after construction
/ :meth:`reset` / any accepted hand-absent frame has no hand prior and is
accepted (a hand-absent gap CLEARS the hand baseline — comparing across a gap
would gate against a position the plan legitimately left long ago). The
firmware's own ``MAX_DEVIATION`` guards (commanded-vs-encoder,
fault_machine.cpp ``evaluate_guard``; ``MAX_DEVIATION_HAND_REV`` joins in
Phase 3) are the complementary defence-in-depth layer that catches a bad
*first* command. Two independent layers: the bridge gates command-stream
discontinuity (prior frame); the Teensy gates command-vs-encoder divergence.

Whether the resulting frame is actually transmitted is gated separately by the
node's ``mpc_active`` — runtime-armed via ``/set_setpoint_output``'s
stream-then-arm pre-check, automatic on ACTIVE entry since the 2026-07-15
arming contract (``ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md``; the
old ``enable_setpoint_output`` boot-arm is inert) — this module only decides
*what* a well-formed frame would be and *whether it is safe to send*.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

from . import protocol as p
from .protocol import Setpoint

# Setpoint flag bits — u1/u2 lookahead + hand-lane + exact-v1 presence
# (firmware convention: bits, not NaN).
# Mirrors synthetic_setpoint.py / replay_setpoint.py (the bench Teensy-side sources).
FLAG_HAS_U1 = 0x1
FLAG_HAS_U2 = 0x2
FLAG_HAS_HAND = 0x4   # v6: Setpoint index 6 (hand lane) is live
FLAG_HAS_V1 = 0x8     # v6: v1[] carries the exact u1-knot velocity

# Default per-step clamp (rev). The bridge passes the authoritative
# hardware_config.JB_OP_MAX_POSITION_STEP_REV; this default mirrors it so the
# module is usable standalone in tests.
DEFAULT_MAX_STEP_REV = 0.3

# Hand per-step gate (rev per 25 ms knot). ONE derivation chain — never a second
# invented number: hand_vel_limit_rps × knot_dt = 200 × 0.025 = 5.0 rev at the
# shipped config (JB_TRAJ_HAND_VEL_LIMIT_RPS × JB_TRAJ_KNOT_DT_S; the 200 rev/s
# session cap is Phase 0 Decision 4, owner-signed 2026-08-30 — +12 % over the
# C-HAND-3 certified 178.23 rev/s peak). feasibility.py's per-knot hand bound
# (STEP_BOUND_MARGIN 0.80 × hand_vel_limit_rps × dt = 4.0 rev) then sits at
# exactly the same 20 % margin below this gate that the legs use (validate
# 0.8 × 0.3 = 0.24 vs pump 0.3), so a pump reject can't happen on a validated
# plan. Pinned against the generated config AND the 0.80 relation by
# tests/teensy_link/test_setpoint_pump.py (the DEFAULT_TORQUE_WIRE_SCALE
# mirrored-constant pattern).
DEFAULT_MAX_STEP_HAND_REV = 5.0

# The all-or-nothing hand key set (u0/v0/u1/u2 for the hand lane).
# 'hand_next_vel_rps' (the hand v1) is a hand_* key too for PRESENCE-triggering
# purposes, but belongs to the HAS_V1 rule — see _build().
_HAND_KEYS = ('hand_rev', 'hand_vel_rps', 'hand_next_rev', 'hand_next2_rev')

# ── Leg torque-FF defaults (mirror the generated hardware_config constants so the
# module stays pure + standalone-usable; pinned against them by
# tests/motion/test_leg_torque_ff.py::test_pump_defaults_match_generated_config) ──

# TRUE Nm → ODrive-Nm. = DYNAMICS_MOTOR_KT_ODRIVE_CONFIG_NM_PER_A / DYNAMICS_MOTOR_KT_NM_PER_A
#                      = 0.055133331567049 / 0.0570 (Kt measured 2026-07-14/15, pooled).
# hardware_config.ODRIVE_LEG_TORQUE_WIRE_SCALE is the authority; the bridge passes it in.
DEFAULT_TORQUE_WIRE_SCALE = 0.9672514310008596

# Per-leg |torque_ff| clamp in TRUE Nm (hardware_config.DYNAMICS_TORQUE_FF_MAX_NM).
DEFAULT_TORQUE_FF_MAX_NM = 0.15


class SetpointPump:
    """Stateful Teensy-side knot packer + per-step gate for the 40/500 Hz setpoint downlink.

    Args:
        mm_to_rev: per-leg mm→motor-rev scale (``hardware_config.GEOM_MM_TO_REV``).
            Injected because the pump now derives motor-rev knots from the MPC's
            mm-space command fields (``ext_mm``/``cmd_next_mm``/``vel_mm_s``);
            kept as a constructor arg (not a per-build param or a
            ``jugglebot.motion`` import) so the module stays pure.
        num_legs: number of leg axes (the wire ``Setpoint`` carries 7 lanes —
            ``num_legs`` leg lanes plus the hand at index 6).
        max_step_rev: per-leg per-frame ``u0`` step clamp (rev).
        max_step_hand_rev: per-frame hand ``u0[6]`` step clamp (rev). Derived
            from the hand session velocity cap — see the
            ``DEFAULT_MAX_STEP_HAND_REV`` derivation comment above.
        torque_ff_enabled: master switch for the leg torque feedforward
            (``hardware_config.DYNAMICS_TORQUE_FF_ENABLED``). **Default False** — the
            pump then emits ``torque_ff = (0.0,)*n`` and never even reads
            ``cmd['torque_Nm']``, so the frame is byte-identical to the pre-feature one.
        torque_ff_max_nm: per-leg clamp on the feedforward in TRUE Nm, applied before
            the wire scale. See the module docstring for why this clamp is
            load-bearing (it binds first; the firmware ingest backstop at 0.25
            wire-Nm only exists after a can-bridge reflash).
        torque_wire_scale: TRUE Nm → ODrive-Nm (``ODRIVE_LEG_TORQUE_WIRE_SCALE``).
        torque_ff_ramp_frames: number of ACCEPTED frames over which the feedforward
            ramps 0 → 1 after construction / :meth:`reset`. 0 disables the ramp
            (full FF on the first frame — only for tests). The bridge derives this
            from ``DYNAMICS_TORQUE_FF_RAMP_S / JB_TRAJ_KNOT_DT_S``.
    """

    def __init__(self, mm_to_rev: Sequence[float],
                 num_legs: int = p.NUM_LEGS,
                 max_step_rev: float = DEFAULT_MAX_STEP_REV,
                 max_step_hand_rev: float = DEFAULT_MAX_STEP_HAND_REV,
                 torque_ff_enabled: bool = False,
                 torque_ff_max_nm: float = DEFAULT_TORQUE_FF_MAX_NM,
                 torque_wire_scale: float = DEFAULT_TORQUE_WIRE_SCALE,
                 torque_ff_ramp_frames: int = 0):
        self.n = int(num_legs)
        self.mm_to_rev = tuple(float(x) for x in mm_to_rev)
        if len(self.mm_to_rev) < self.n:
            raise ValueError(
                f"mm_to_rev needs >= {self.n} entries, got {len(self.mm_to_rev)}")
        self.max_step_rev = float(max_step_rev)
        self.max_step_hand_rev = float(max_step_hand_rev)

        # ── Leg torque-FF configuration (see the module docstring) ──
        self.torque_ff_enabled = bool(torque_ff_enabled)
        self.torque_ff_max_nm = abs(float(torque_ff_max_nm))
        self.torque_wire_scale = float(torque_wire_scale)
        self.torque_ff_ramp_frames = max(0, int(torque_ff_ramp_frames))
        if not math.isfinite(self.torque_ff_max_nm) or self.torque_ff_max_nm <= 0.0:
            raise ValueError(
                f"torque_ff_max_nm must be finite and > 0, got {torque_ff_max_nm}")
        # CEILING (adversarial review 2026-07-14): this clamp is the FIRST-binding
        # one in the chain (the firmware ingest backstop at 0.25 wire-Nm is wider
        # and reflash-gated), so it must itself be bounded.  The ODrive adds
        # input_torque to the velocity loop's output BEFORE the torque limit;
        # a feedforward near ~0.55 wire-Nm (= current_soft_max 10 A x the
        # configured torque_constant 0.055133) consumes the whole current
        # budget and the position loop loses ALL authority.  0.30 TRUE Nm
        # (~0.265 wire-Nm ~= 4.8 A) leaves >5 A of loop authority and is
        # already 7x the largest gravity torque in the workspace — any config
        # asking for more is a mistake, not a use case.
        if self.torque_ff_max_nm > 0.30:
            raise ValueError(
                f"torque_ff_max_nm={torque_ff_max_nm} exceeds the 0.30 Nm "
                f"ceiling (position-loop authority: the ODrive adds torque_ff "
                f"before its torque limit, and ~0.62 true-Nm saturates the "
                f"10 A current clamp entirely)")
        if not math.isfinite(self.torque_wire_scale) or self.torque_wire_scale <= 0.0:
            raise ValueError(
                f"torque_wire_scale must be finite and > 0, got {torque_wire_scale}")

        self._prev_pos: Optional[list] = None
        self._prev_hand: Optional[float] = None   # prior ACCEPTED hand u0 (None = no baseline)
        self._ff_frames = 0          # accepted frames since reset — drives the FF ramp
        self.frames_built = 0
        self.frames_skipped = 0      # no-command ticks (not a fault)
        self.frames_rejected = 0     # SAFETY rejects (NaN, short, step violation)
        self.frames_without_ff = 0   # FF on, but the producer sent no torque_Nm
        self.last_reject_reason = ''

    def reset(self) -> None:
        """Forget the prior frame (e.g. after a link loss / re-enable).

        Also RESTARTS the torque-FF ramp. That is the point: after a link loss the
        ODrive's velocity integrator has re-absorbed the gravity load on its own, so
        re-applying the full feedforward as a step would double-count it exactly as it
        would at first arm. The hand step-gate baseline clears too — same reasoning
        as the legs', per channel.
        """
        self._prev_pos = None
        self._prev_hand = None
        self._ff_frames = 0

    def restart_torque_ramp(self) -> None:
        """Restart ONLY the torque-FF ramp, leaving the per-step position gate's
        baseline intact.

        For the guard-latch -> ``/recover`` path (adversarial review 2026-07-14):
        ``/recover`` deliberately keeps ``mpc_active = 1``, so neither
        link-restore nor the disarm->arm edge fires and :meth:`reset` is never
        called — yet the firmware zeroes ``cmd_tor`` during its recovery slew
        while the ODrive integrator re-winds gravity, and on hand-back the held
        feedforward would return as a full-magnitude single-tick step (the exact
        transient the ramp exists to prevent, on the exact path the jolt-free
        clear work engineered to be step-free).  The bridge calls this from the
        recover path so the FF re-ramps over the configured window instead.

        Distinct from :meth:`reset` on purpose: clearing ``_prev_pos`` would
        also waive the per-frame position-step gate for one frame, which the
        recover path neither needs nor wants — position streaming continues
        uninterrupted throughout a recovery.
        """
        self._ff_frames = 0

    def torque_ff_ramp_scale(self) -> float:
        """Current FF ramp multiplier in [0, 1].

        0 on the first feedforward-carrying frame after construction / :meth:`reset` /
        any accepted frame that carried no ``torque_Nm``. See :meth:`_build` for why
        each of those restarts the ramp.
        """
        if not self.torque_ff_enabled:
            return 0.0
        if self.torque_ff_ramp_frames <= 0:
            return 1.0
        return min(1.0, self._ff_frames / float(self.torque_ff_ramp_frames))

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

    @staticmethod
    def _finite_scalar(val, name: str):
        """Validate a scalar hand-lane value is numeric and finite.

        The scalar sibling of :meth:`_finite_vec`, with the same
        return-don't-raise robustness contract: a non-numeric / non-finite
        value returns ``(None, reason)`` so one bad frame is a counted REJECT,
        never a crash of the setpoint thread. NaN here is a REJECT by design —
        the hand has no flag-clear fallback (see the module docstring).
        """
        try:
            v = float(val)
        except (TypeError, ValueError):
            return None, f'non-numeric {name} ({val!r})'
        if not math.isfinite(v):
            return None, f'non-finite {name}'
        return v, None

    def build(self, cmd: dict, t_origin_us: int
              ) -> Tuple[Optional[Setpoint], Optional[str]]:
        """Validate + pack one 40 Hz MPC command (:5557 ``mpc_cmd`` dict).

        Returns ``(setpoint, reject_reason)``:
          * ``(Setpoint, None)`` — a safe Teensy-side knot frame to send.
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
            # A hand-carrying frame with NO leg position command is an
            # incoherent producer state (the docstring's Mode-1 coherence
            # rule), not an idle tick — loud SAFETY reject, mirroring the
            # hand/Mode-1 knot-set reject below.
            if (cmd.get('hand_next_vel_rps') is not None
                    or any(cmd.get(k) is not None for k in _HAND_KEYS)):
                self.frames_rejected += 1
                self.last_reject_reason = (
                    'hand keys without a position command '
                    '(motor_rev/ext_mm both absent)')
                return None, self.last_reject_reason
            # No position command in this message (e.g. a non-mpc_cmd frame that
            # slipped the topic filter). Nothing to command — skip, not a fault.
            self.frames_skipped += 1
            return None, None

        # ── v0: vel_mm_s × mm_to_rev (motor_guard's leg_velocities_to_motor_…) ──
        # The production producer (trajectory_node's emitter) always supplies
        # vel_mm_s, as HardwarePlant did before it was removed 2026-09-01
        # (tag mpc-final). motor_guard would
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

        # ── torque_Nm (TRUE Nm, extension-positive) — only read when FF is ON ──
        # When the feature is OFF we do not even look at the field, so a producer
        # sending a malformed / absent / NaN torque_Nm behaves EXACTLY as it did
        # before this feature existed (torque_ff = zeros, frame accepted).
        #
        # When it is ON, a malformed torque vector is a SAFETY REJECT, not a silent
        # zero. Two reasons: (a) it signals an upstream bug we must surface, and
        # (b) silently dropping a live feedforward to zero manufactures exactly the
        # discontinuous-FF transient that bootstrapped the 2026-05-08 5 Hz platform
        # limit cycle. Rejecting instead leaves the firmware holding the last good
        # setpoint AND its torque (continuous), and if the condition persists the
        # 250 ms MPC-staleness E-STOP fires — loud and safe.
        tq_true = None
        if self.torque_ff_enabled:
            tq = cmd.get('torque_Nm')
            if tq is None:
                # A producer that simply doesn't do feedforward (e.g. a bench source).
                # Not a fault — just no FF this frame.
                self.frames_without_ff += 1
            else:
                tq_true, reason = self._finite_vec(tq, 'torque_Nm')
                if reason is not None:
                    self.frames_rejected += 1
                    self.last_reject_reason = reason
                    return None, reason

        # ── u1 / u2: cmd_next(_2)_mm × mm_to_rev; absent/bad ⇒ clear the flag ──
        # Mirrors motor_guard: a non-finite / wrong-length lookahead clears the
        # waypoint rather than rejecting the frame (firmware convention: bits, not NaN).
        # Exact-length (== n, not >= n) mirrors motor_guard's ``shape == (6,)``
        # gate (motor_guard.py:585, 597) so a malformed >6-element lookahead
        # clears the flag (Taylor fallback) instead of silently taking the
        # first 6 and emitting a divergent-but-accepted Teensy-side frame.
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

        # ── Hand lane (index 6): ALL-OR-NOTHING key set, NaN ⇒ reject ──
        # Any hand_* key present (hand_next_vel_rps included) commits the
        # producer to the full u0/v0/u1/u2 hand set. See the module docstring
        # for why the hand never gets the legs' flag-clear fallback.
        hand_vals = None            # (u0h, v0h, u1h, u2h) when the hand lane is live
        hand_v1_raw = cmd.get('hand_next_vel_rps')
        if hand_v1_raw is not None or any(cmd.get(k) is not None for k in _HAND_KEYS):
            missing = [k for k in _HAND_KEYS if cmd.get(k) is None]
            if missing:
                self.frames_rejected += 1
                self.last_reject_reason = (
                    'hand keys are all-or-nothing: missing '
                    + '/'.join(missing))
                return None, self.last_reject_reason
            parsed = []
            for k in _HAND_KEYS:
                v, reason = self._finite_scalar(cmd.get(k), k)
                if reason is not None:
                    self.frames_rejected += 1
                    self.last_reject_reason = reason
                    return None, reason
                parsed.append(v)
            hand_vals = tuple(parsed)
            # Coherence: hand streaming requires the FULL leg Mode-1 knot set
            # (u1 AND u2). A producer that knows the hand's next two knots but
            # not the legs' is broken, not degraded — reject, never silent-drop.
            if (flags & (FLAG_HAS_U1 | FLAG_HAS_U2)) != (FLAG_HAS_U1 | FLAG_HAS_U2):
                self.frames_rejected += 1
                self.last_reject_reason = (
                    'hand keys present without the full leg Mode-1 knot set '
                    '(cmd_next_mm/cmd_next2_mm absent or malformed)')
                return None, self.last_reject_reason

        # ── v1 (exact u1-knot velocity): every active channel or none ──
        # vel_next_mm_s (legs, always required for HAS_V1) + hand_next_vel_rps
        # (required exactly when the hand keys are present). Both absent ⇒
        # HAS_V1 clear ⇒ the firmware forward-difference fallback (the flown
        # path). A partial set rejects loudly; a present-but-bad vel_next is a
        # reject (the v0 precedent), never a fallback.
        vel_next = cmd.get('vel_next_mm_s')
        v1_legs = [0.0] * self.n
        v1_hand = 0.0
        has_v1 = False
        if vel_next is None and hand_v1_raw is None:
            pass                     # no v1 anywhere — legal, flag stays clear
        elif vel_next is None:
            # hand_v1 present alone: with hand keys this is a partial v1 set;
            # without them the all-or-nothing gate above already rejected.
            self.frames_rejected += 1
            self.last_reject_reason = (
                'partial v1 set: hand_next_vel_rps without vel_next_mm_s')
            return None, self.last_reject_reason
        else:
            if hand_vals is not None and hand_v1_raw is None:
                self.frames_rejected += 1
                self.last_reject_reason = (
                    'partial v1 set: vel_next_mm_s with hand keys but no '
                    'hand_next_vel_rps')
                return None, self.last_reject_reason
            # Coherence: v1 is the velocity AT the u1 knot, so HAS_V1 requires
            # the full leg Mode-1 knot set (u1 AND u2) — vel_next_mm_s aboard
            # a frame without the knot it describes is incoherent. Reject,
            # never silent-drop (mirrors the hand/Mode-1 rule above; with hand
            # keys aboard that gate has already fired, so this catches the
            # hand-absent case).
            if (flags & (FLAG_HAS_U1 | FLAG_HAS_U2)) != (
                    FLAG_HAS_U1 | FLAG_HAS_U2):
                self.frames_rejected += 1
                self.last_reject_reason = (
                    'vel_next_mm_s present without the full leg Mode-1 knot '
                    'set (cmd_next_mm/cmd_next2_mm absent or malformed)')
                return None, self.last_reject_reason
            vn_vals, reason = self._finite_vec(vel_next, 'vel_next_mm_s')
            if reason is not None:
                self.frames_rejected += 1
                self.last_reject_reason = reason
                return None, reason
            v1_legs = [vn_vals[i] * mr[i] for i in range(self.n)]
            if hand_v1_raw is not None:
                # hand_next_vel_rps is not in _HAND_KEYS (it belongs to the
                # HAS_V1 rule, not the u0/v0/u1/u2 set) — validate it here.
                v1_hand, reason = self._finite_scalar(hand_v1_raw,
                                                      'hand_next_vel_rps')
                if reason is not None:
                    self.frames_rejected += 1
                    self.last_reject_reason = reason
                    return None, reason
            has_v1 = True

        # ── Per-step gates vs the prior ACCEPTED frame (NOT vs rejected frames) ──
        # Per-CHANNEL: the leg gate and the hand gate reject independently,
        # each against its own prior and its own bound.
        if self._prev_pos is not None:
            for i in range(self.n):
                step = abs(u0[i] - self._prev_pos[i])
                if step > self.max_step_rev:
                    self.frames_rejected += 1
                    self.last_reject_reason = (
                        f'leg {i} step {step:.4f} rev > {self.max_step_rev} '
                        f'limit (cmd={u0[i]:.4f}, prev={self._prev_pos[i]:.4f})')
                    return None, self.last_reject_reason
        if hand_vals is not None and self._prev_hand is not None:
            hand_step = abs(hand_vals[0] - self._prev_hand)
            if hand_step > self.max_step_hand_rev:
                self.frames_rejected += 1
                self.last_reject_reason = (
                    f'hand step {hand_step:.4f} rev > {self.max_step_hand_rev} '
                    f'limit (cmd={hand_vals[0]:.4f}, prev={self._prev_hand:.4f})')
                return None, self.last_reject_reason

        # ── torque_ff: clamp (TRUE Nm) → ramp → wire scale (ODrive-Nm) ──────────
        # Order matters. The clamp is a bound on the PHYSICAL torque we are willing to
        # inject, so it is applied in true Nm, where an operator can compare it against
        # the 0.013–0.041 Nm/leg gravity load. The ramp then attenuates that bounded
        # torque, and the Kt wire scale converts it into the units the drive divides by.
        # (Clamp-then-scale also means the clamp bound is exactly the number in the YAML,
        # not that number times the wire scale.)
        #
        # The ramp counts ACCEPTED FRAMES THAT ACTUALLY CARRIED A FEEDFORWARD
        # (``_ff_frames``, 0 on the first such frame after a reset), so the first frame
        # of any feedforward carries ZERO and the FF grows from nothing — never a step
        # into a velocity integrator that is already holding gravity.
        #
        # Two things that counter deliberately does NOT count, each closing a hole:
        #   * REJECTED frames — they command nothing, so they must not buy ramp credit;
        #     otherwise a burst of rejects would let the FF snap to full on the next
        #     good frame.
        #   * Accepted frames with NO torque_Nm — and, more than that, such a frame
        #     RESETS the counter. Consider a producer that streams position-only for a
        #     few seconds and then starts sending feedforward (a restarted emitter, a
        #     source swap). The integrator has spent that time winding up to carry
        #     gravity by itself, so this is exactly the first-arm situation again and it
        #     needs the ramp again. Without the reset, the FF would snap straight to
        #     full — the very transient the ramp exists to prevent, reintroduced through
        #     the back door.
        if tq_true is None:
            torque_ff = (0.0,) * self.n
            self._ff_frames = 0
        else:
            lim = self.torque_ff_max_nm
            gain = self.torque_wire_scale * self.torque_ff_ramp_scale()
            torque_ff = tuple(
                max(-lim, min(lim, tq_true[i])) * gain for i in range(self.n))
            self._ff_frames += 1

        # ── Assemble the 7-lane v6 frame: legs 0..(n-1) + hand at index n ──
        # Hand absent ⇒ index 6 packed 0.0 in every array with HAS_HAND clear
        # (the firmware ignores the lane) — leg lanes byte-identical to the
        # pre-hand frame. torque_ff[6] = 0 ALWAYS in Phase 2.
        if hand_vals is not None:
            flags |= FLAG_HAS_HAND
            u0h, v0h, u1h, u2h = hand_vals
        else:
            u0h = v0h = u1h = u2h = 0.0
        if has_v1:
            flags |= FLAG_HAS_V1

        sp = Setpoint(
            u0=tuple(u0) + (u0h,),
            u1=tuple(u1) + (u1h,),
            u2=tuple(u2) + (u2h,),
            v0=tuple(v0) + (v0h,),
            accel=(0.0,) * (self.n + 1),        # Mode-1 Hermite ignores accel
            torque_ff=torque_ff + (0.0,),       # zeros unless torque_ff_enabled; [6] always 0
            v1=tuple(v1_legs) + (v1_hand,),     # zeros unless HAS_V1
            flags=flags,
            t_origin_us=int(t_origin_us),
        )
        self._prev_pos = list(u0)
        # A hand-absent accepted frame CLEARS the hand baseline (a gap ends the
        # stream; the next hand-carrying frame is a first frame again, with the
        # firmware deviation guard as the complementary layer).
        self._prev_hand = u0h if hand_vals is not None else None
        self.frames_built += 1
        return sp, None
