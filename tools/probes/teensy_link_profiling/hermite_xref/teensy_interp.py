"""Pure-Python port of the Teensy 500 Hz interpolator — the C++ translation target.

This module is a deliberately C++-shaped, scalar/per-leg-loop transcription of
the can-bridge firmware's `leg_interp.cpp`. It exists so the interpolator math
can be validated against the real `motor_guard.py` offline (see xref.py): the
C++ `leg_interp.cpp` mirrors THIS file line-for-line, so proving this matches
`motor_guard` proves the firmware port is correct (within float precision).

Ported from motor_guard.py's `_interpolate_and_send` method (the ~870-1049
method; the ladder proper is ~894-1048): the Hermite → Taylor → velocity-decay
ladder + lead-clamp + stroke-clamp. No numpy: lists + explicit per-leg loops,
exactly as the C++ does. Keep the two in sync. (Reference the method name, not
the line numbers, when they drift.)

NOTE: friction feedforward is intentionally NOT ported here — it is a separate
additive torque term (motor_guard `_compute_friction_ff_Nm`) that does not affect
the commanded *position* or *velocity*, which is what this ladder produces and
what the xref compares. Porting friction FF to the Teensy is a documented
follow-on (see handoff).

THE 7TH (HAND) LANE — added 2026-09-04, unified-7dof-planner Phase 4
--------------------------------------------------------------------
FW 17 grew a 7th interpolation lane for the hand (axis 6). It is mirrored here
so `sim/unified_gate.py` can drive the production chain
(planner → emitter → SetpointPump → wire → **this** → plant) without a second,
divergent transcription of the ladder living under `sim/`.

Two things about it are deliberate and must not be "tidied":

* **The hand is a SEPARATE block, never a widened leg loop.** That is how
  `leg_interp.cpp` writes it, and for the reason its comment gives: the hand
  carries its OWN guard constants (`MAX_LEAD_HAND_REV` 2.0 /
  `HAND_VELFF_LIMIT_RPS` 300 / clip `[0, HAND_MOTOR_MAX_POSITION]`) and its OWN
  knot clock, and the leg constants must NEVER reach axis 6. Separate loops
  make that structural instead of a comment. Concretely: :meth:`TeensyLegInterp.tick`
  still returns three `list[6]`s and is byte-identical to the pre-hand version;
  the hand runs through :meth:`latch_hand` / :meth:`tick_hand`.
* **The leg block still mirrors `motor_guard`; the hand block mirrors
  `leg_interp.cpp`.** There is no motor_guard hand lane to be the twin of, and
  the firmware's LEG lead clamp deliberately diverges from motor_guard's
  (0.10 rev + a bounded feedforward, versus 0.15 rev + a zeroed one — see
  `leg_interp.cpp`'s clamp comment). Porting the firmware's leg clamp here would
  break the xref this file exists to serve, so it is left alone and the hand
  clamp is written against the firmware directly.

Also added: the **transmitted-v1** Mode-1 rule (`HAS_V1`, v6 wire). Pass
``v1=`` to :meth:`latch_setpoint` / :meth:`latch_hand` and the Hermite endpoint
velocity is the transmitted one (exact reconstruction for knot-aligned cubics);
omit it and the flown `(u2−u1)/T` forward difference — the pre-v6 path the xref
pins — is used unchanged.
"""

from __future__ import annotations

import math          # isfinite only — the C++ block's <cmath> backstop

NUM_LEGS = 6
NUM_AXES = 7
HAND_AXIS = 6

# Constants ported 1:1 from motor_guard.py / canbridge_config.h.
SEGMENT_T_S = 0.025
MAX_EXTRAP_DT_S = 0.05
EXTRAP_DECAY_DT_S = 0.06
JERK_EMA_ALPHA = 0.3
MAX_LEAD_REV = 0.15

# ── Hand-lane guard constants (canbridge_config.h, FW 17) ────────────────────
# These are the firmware's, NOT the legs': applying MAX_LEAD_REV (0.10) or
# LEAD_CLAMP_VELFF_LIMIT_RPS (3.5) to axis 6 would be a 51x feedforward cut on a
# 200 rev/s axis.  Pinned against the header by
# tests/firmware/test_hermite_xref.py::test_hand_lane_constants_match_the_firmware.
MAX_LEAD_HAND_REV = 2.0
HAND_VELFF_LIMIT_RPS = 300.0
HAND_MOTOR_MAX_POSITION = 10.8
MAX_DEVIATION_HAND_REV = 2.5
# Age cap on the lead clamp's freshness-aware anchor (canbridge_config.h
# MOTOR_FB_STALENESS_US = 150000 us; leg_interp.cpp:773-774). Past the cap the
# anchor stops following a possibly-dead extrapolation, the frame STILL
# transmits, and the tick is counted (`hand_stale_holds`) so the telemetry gap
# is loud rather than silent.
MOTOR_FB_STALENESS_S = 0.150


class TeensyLegInterp:
    """Scalar, per-leg port of the motor_guard interpolation ladder.

    The constructor takes the per-leg stroke clamp bounds (rev); these mirror the
    firmware's STROKE_MIN_REV / STROKE_MAX_REV and motor_guard's
    _stroke_min_rev / _stroke_max_rev.
    """

    def __init__(self, stroke_min_rev, stroke_max_rev):
        self.stroke_min = list(stroke_min_rev)
        self.stroke_max = list(stroke_max_rev)

        # Latched base state (per leg).
        self.base_pos = [0.0] * NUM_LEGS
        self.base_vel = [0.0] * NUM_LEGS
        self.base_accel = [0.0] * NUM_LEGS
        self.base_torque = [0.0] * NUM_LEGS
        self.jerk = [0.0] * NUM_LEGS
        self.base_timestamp = 0.0

        self.next_pos = None   # list[6] or None
        self.next2_pos = None  # list[6] or None

        # Jerk-estimation history.
        self.prev_accel = None  # list[6] or None
        self.prev_timestamp = 0.0

        # Outputs.
        self.cmd_pos = [0.0] * NUM_LEGS
        self.cmd_vel = [0.0] * NUM_LEGS
        self.cmd_torque = [0.0] * NUM_LEGS
        # RAW ladder output, before the lead + stroke clamps.  Recorded, never
        # fed back: the clamps are an encoder-dependent SAFETY layer, so a
        # "did the interpolator reconstruct the plan?" score has to read the
        # ladder, not the guard.  ``sim/unified_gate.py`` scores these.
        self.raw_pos = [0.0] * NUM_LEGS
        self.raw_vel = [0.0] * NUM_LEGS
        self.lead_clamp_ticks = 0
        self.stroke_clamp_ticks = 0

        # ── Transmitted-v1 (HAS_V1, v6 wire).  None ⇒ the forward-difference
        # fallback, i.e. the pre-v6 behaviour this file's xref pins. ──
        self.v1 = None          # list[6] or None
        # ── Hand lane (axis 6) — see the module docstring. ──
        self.hand_active = False
        self.hand_base_pos = 0.0
        self.hand_base_vel = 0.0
        self.hand_base_accel = 0.0
        self.hand_jerk = 0.0
        self.hand_ts = 0.0
        self.hand_next_pos = None
        self.hand_next2_pos = None
        self.hand_v1 = None
        self.hand_prev_accel = None
        self.hand_prev_ts = 0.0
        self.hand_cmd_pos = 0.0
        self.hand_cmd_vel = 0.0
        self.hand_raw_pos = 0.0
        self.hand_raw_vel = 0.0
        self.hand_dev_max = 0.0
        self.hand_dev_over_ticks = 0
        self.hand_lead_clamp_ticks = 0
        self.hand_clip_ticks = 0
        self.hand_unseen_skips = 0
        # Ticks whose clamp-anchor age hit MOTOR_FB_STALENESS_S
        # (leg_interp.cpp's s_hand_stale_holds).
        self.hand_stale_holds = 0
        # Ticks whose ladder output was non-finite and was replaced by the
        # encoder backstop (leg_interp.cpp:781-784).
        self.hand_nonfinite_ticks = 0
        #: Which rung of the ladder the LAST :meth:`tick_hand` took — 1 Hermite,
        #: 2 Taylor, 3 velocity decay, 0 never ticked.  Diagnostic only: the
        #: firmware has no such variable (its mode is implicit in the branch),
        #: and nothing in this file reads it.  It exists so a harness can say
        #: WHEN the lane left Mode 1 instead of inferring it from the age it
        #: already used to decide what to look for — see
        #: ``sim/unified_gate.hand_decay_probe``.
        self.hand_mode = 0

    # ── Latch a new MPC setpoint (port of the relevant parts of _on_mpc_command) ──
    def latch_setpoint(self, u0, v0, accel, torque, t_latch,
                       u1=None, u2=None, v1=None):
        """u0/u1/u2 in motor-rev, v0 in rev/s, accel in rev/s^2, torque in Nm.

        u1=None → no forward waypoint (Taylor fallback). u2 only meaningful with u1.
        ``v1`` is the TRANSMITTED u1-knot velocity (rev/s, the v6 wire's
        ``HAS_V1``); ``None`` keeps the flown ``(u2−u1)/T`` forward difference.
        """
        # Jerk EMA from consecutive accelerations (motor_guard._on_mpc_command).
        if self.prev_accel is not None:
            dt_mpc = t_latch - self.prev_timestamp
            if dt_mpc > 1e-6:
                for i in range(NUM_LEGS):
                    raw = (accel[i] - self.prev_accel[i]) / dt_mpc
                    self.jerk[i] = JERK_EMA_ALPHA * raw + (1.0 - JERK_EMA_ALPHA) * self.jerk[i]
        else:
            self.jerk = [0.0] * NUM_LEGS

        for i in range(NUM_LEGS):
            self.base_pos[i] = u0[i]
            self.base_vel[i] = v0[i]
            self.base_accel[i] = accel[i]
            self.base_torque[i] = torque[i]
        self.base_timestamp = t_latch

        self.next_pos = list(u1) if u1 is not None else None
        self.next2_pos = list(u2) if (u2 is not None and u1 is not None) else None
        self.v1 = list(v1) if v1 is not None else None

        self.prev_accel = list(accel)
        self.prev_timestamp = t_latch

    # ── 500 Hz tick (port of _interpolate_and_send) ───────────────────────────
    def tick(self, t_now, fb_pos):
        """Compute the per-leg commanded position/velocity for the current tick.

        fb_pos[6] is the latest motor feedback (rev), used by the lead clamp.
        Returns (cmd_pos, cmd_vel, cmd_torque), each a list[6].
        """
        dt = t_now - self.base_timestamp

        if self.next_pos is not None:
            # ── Mode 1: cubic Hermite between u0 and u1 ──
            T = SEGMENT_T_S
            s = dt / T
            if s > 1.0:
                s = 1.0
            s2 = s * s
            s3 = s2 * s
            h00 = 2.0 * s3 - 3.0 * s2 + 1.0
            h10 = s3 - 2.0 * s2 + s
            h01 = -2.0 * s3 + 3.0 * s2
            h11 = s3 - s2
            inv_T = 1.0 / T
            dh00 = (6.0 * s2 - 6.0 * s) * inv_T
            dh10 = 3.0 * s2 - 4.0 * s + 1.0
            dh01 = (-6.0 * s2 + 6.0 * s) * inv_T
            dh11 = 3.0 * s2 - 2.0 * s
            for i in range(NUM_LEGS):
                p0 = self.base_pos[i]
                p1 = self.next_pos[i]
                v0 = self.base_vel[i]
                # u1-knot endpoint velocity: the TRANSMITTED v1 when the frame
                # carried it (HAS_V1, v6 wire — exact reconstruction for
                # knot-aligned cubics); else the flown (u2−u1)/T forward
                # difference, with (u1−u0)/T when there is no u2 either.
                # leg_interp.cpp writes the same three-way ladder.
                if self.v1 is not None:
                    v1 = self.v1[i]
                elif self.next2_pos is not None:
                    v1 = (self.next2_pos[i] - p1) / T
                else:
                    v1 = (p1 - p0) / T
                self.cmd_pos[i] = h00 * p0 + h10 * (T * v0) + h01 * p1 + h11 * (T * v1)
                self.cmd_vel[i] = dh00 * p0 + dh10 * v0 + dh01 * p1 + dh11 * v1

        elif dt <= MAX_EXTRAP_DT_S:
            # ── Mode 2: cubic Taylor extrapolation ──
            dt2 = dt * dt
            for i in range(NUM_LEGS):
                self.cmd_pos[i] = (self.base_pos[i]
                                   + self.base_vel[i] * dt
                                   + 0.5 * self.base_accel[i] * dt2
                                   + (1.0 / 6.0) * self.jerk[i] * (dt2 * dt))
                self.cmd_vel[i] = (self.base_vel[i]
                                   + self.base_accel[i] * dt
                                   + 0.5 * self.jerk[i] * dt2)

        else:
            # ── Mode 3: velocity decay to zero ──
            dt_b2 = MAX_EXTRAP_DT_S * MAX_EXTRAP_DT_S
            dt_over = dt - MAX_EXTRAP_DT_S
            decay_frac = 1.0 - dt_over / EXTRAP_DECAY_DT_S
            if decay_frac < 0.0:
                decay_frac = 0.0
            for i in range(NUM_LEGS):
                vel_b = (self.base_vel[i]
                         + self.base_accel[i] * MAX_EXTRAP_DT_S
                         + 0.5 * self.jerk[i] * dt_b2)
                pos_b = (self.base_pos[i]
                         + self.base_vel[i] * MAX_EXTRAP_DT_S
                         + 0.5 * self.base_accel[i] * dt_b2
                         + (1.0 / 6.0) * self.jerk[i] * (dt_b2 * MAX_EXTRAP_DT_S))
                if dt_over >= EXTRAP_DECAY_DT_S:
                    extra = vel_b * (EXTRAP_DECAY_DT_S * 0.5)
                else:
                    extra = vel_b * dt_over * (1.0 - dt_over / (2.0 * EXTRAP_DECAY_DT_S))
                self.cmd_pos[i] = pos_b + extra
                self.cmd_vel[i] = vel_b * decay_frac

        # torque_ff passes through (friction FF not ported — see module docstring).
        for i in range(NUM_LEGS):
            self.cmd_torque[i] = self.base_torque[i]

        # The ladder's own answer, before either guard touches it.
        self.raw_pos = list(self.cmd_pos)
        self.raw_vel = list(self.cmd_vel)

        # ── Lead clamp: never run more than MAX_LEAD_REV ahead of encoder ──
        for i in range(NUM_LEGS):
            pre = self.cmd_pos[i]
            dev = self.cmd_pos[i] - fb_pos[i]
            if dev > MAX_LEAD_REV:
                dev = MAX_LEAD_REV
            elif dev < -MAX_LEAD_REV:
                dev = -MAX_LEAD_REV
            self.cmd_pos[i] = fb_pos[i] + dev
            if self.cmd_pos[i] != pre:        # leg was lead-clamped → zero vel_ff
                self.cmd_vel[i] = 0.0
                self.lead_clamp_ticks += 1

        # ── Stroke clamp: backstop against extrapolation past physical limits ──
        for i in range(NUM_LEGS):
            pre = self.cmd_pos[i]
            if self.cmd_pos[i] < self.stroke_min[i]:
                self.cmd_pos[i] = self.stroke_min[i]
            elif self.cmd_pos[i] > self.stroke_max[i]:
                self.cmd_pos[i] = self.stroke_max[i]
            if self.cmd_pos[i] != pre:        # clamped → zero vel_ff and torque
                self.cmd_vel[i] = 0.0
                self.cmd_torque[i] = 0.0
                self.stroke_clamp_ticks += 1

        return (list(self.cmd_pos), list(self.cmd_vel), list(self.cmd_torque))

    # ── Hand lane (axis 6) — port of leg_interp.cpp's hand block ─────────────
    def latch_hand(self, u0, v0, t_latch, u1=None, u2=None, v1=None,
                   accel=0.0):
        """Latch a HAS_HAND frame's index-6 knots on the hand's OWN clock.

        ``t_latch`` stamps ``s_hand_ts_us``: the hand's trajectory phase is the
        age of the last **hand-bearing** frame, not of the last leg frame. That
        separation is the whole of the NORMATIVE falling-edge rule — when
        HAS_HAND falls while leg frames keep arriving, this clock keeps running,
        so the lane finishes its Hermite segment, Taylor-extrapolates from the
        segment ENDPOINT, and decays the velocity to zero, instead of holding
        the endpoint (and its up-to-200 rev/s feedforward) forever.

        ``accel`` mirrors the wire's ``accel[6]``, which ``SetpointPump`` packs
        as an exact 0.0 today; it is carried (with the same jerk EMA the legs
        use) so this stays a port rather than a simplification.
        """
        if self.hand_prev_accel is not None:
            dt_knot = t_latch - self.hand_prev_ts
            if dt_knot > 1e-6:
                raw = (accel - self.hand_prev_accel) / dt_knot
                self.hand_jerk = (JERK_EMA_ALPHA * raw
                                  + (1.0 - JERK_EMA_ALPHA) * self.hand_jerk)
        else:
            self.hand_jerk = 0.0

        self.hand_base_pos = float(u0)
        self.hand_base_vel = float(v0)
        self.hand_base_accel = float(accel)
        self.hand_ts = float(t_latch)
        self.hand_next_pos = None if u1 is None else float(u1)
        self.hand_next2_pos = (None if (u2 is None or u1 is None)
                               else float(u2))
        self.hand_v1 = None if v1 is None else float(v1)
        self.hand_prev_accel = float(accel)
        self.hand_prev_ts = float(t_latch)
        self.hand_active = True

    def tick_hand(self, t_now, fb_rev=None, fb_vel_rps=0.0, age_s=0.0):
        """One 500 Hz hand tick.  ``(cmd_pos, cmd_vel)`` in rev / rev-per-second.

        Returns ``None`` when nothing is transmitted — either the lane has never
        latched a HAS_HAND frame (``hand_active`` false, the firmware's
        ``s_hand_active`` gate) or axis 6 has never reported an encoder position
        (``fb_rev is None``, the firmware's ``hts == 0`` unseen-skip: 0.0 rev is
        a real, reachable, WRONG position, so commanding anything is a guess).

        ``fb_vel_rps``/``age_s`` build the firmware's freshness-aware anchor
        ``fb + vel·age``; a caller with a same-tick encoder passes ``age_s = 0``
        and the anchor is the raw feedback.  ``age_s`` is CAPPED at
        :data:`MOTOR_FB_STALENESS_S` and the cap is COUNTED
        (``hand_stale_holds``): past it the anchor stops following a
        possibly-dead extrapolation, the frame still transmits, and the counter
        makes the telemetry gap loud (``leg_interp.cpp:773-774``).

        A non-finite ladder output is replaced by the encoder (0.0 rev only if
        the encoder is itself non-finite) with the feedforward zeroed and the
        tick counted (``hand_nonfinite_ticks``) — the firmware's NaN/Inf
        backstop at ``leg_interp.cpp:781-784``.  It is not optional: NaN
        compares false to everything, so it would sail through both clamps
        below and reach the wire.
        """
        if not self.hand_active:
            return None

        hdt = float(t_now) - self.hand_ts
        p0, v0 = self.hand_base_pos, self.hand_base_vel
        a0, jk = self.hand_base_accel, self.hand_jerk
        T = SEGMENT_T_S
        h_pos = 0.0
        h_vel = 0.0
        endp = 0.0
        endv = 0.0
        over = hdt                       # < 0 ⇒ still inside the Mode-1 segment

        if self.hand_next_pos is not None:
            p1 = self.hand_next_pos
            if self.hand_v1 is not None:
                v1 = self.hand_v1
            elif self.hand_next2_pos is not None:
                v1 = (self.hand_next2_pos - p1) / T
            else:
                v1 = (p1 - p0) / T
            if hdt <= T:
                # Mode 1 — the same cubic Hermite as the legs.
                s = hdt / T
                s2 = s * s
                s3 = s2 * s
                inv_T = 1.0 / T
                h_pos = ((2.0 * s3 - 3.0 * s2 + 1.0) * p0
                         + (s3 - 2.0 * s2 + s) * (T * v0)
                         + (-2.0 * s3 + 3.0 * s2) * p1
                         + (s3 - s2) * (T * v1))
                h_vel = ((6.0 * s2 - 6.0 * s) * inv_T * p0
                         + (3.0 * s2 - 4.0 * s + 1.0) * v0
                         + (-6.0 * s2 + 6.0 * s) * inv_T * p1
                         + (3.0 * s2 - 2.0 * s) * v1)
                over = -1.0
                self.hand_mode = 1
            else:
                endp, endv, over = p1, v1, hdt - T   # segment complete
        else:
            endp, endv, over = p0, v0, hdt           # no u1 knot — a Mode-2 base

        if over >= 0.0:
            self.hand_mode = 2 if over <= MAX_EXTRAP_DT_S else 3
            if over <= MAX_EXTRAP_DT_S:
                # Mode 2 — cubic Taylor from the endpoint state (continuous with
                # Mode 1 at over = 0 by construction).
                o2 = over * over
                h_pos = (endp + endv * over + 0.5 * a0 * o2
                         + (1.0 / 6.0) * jk * (o2 * over))
                h_vel = endv + a0 * over + 0.5 * jk * o2
            else:
                # Mode 3 — velocity decay to zero (the leg formula, endpoint-based).
                dt_b2 = MAX_EXTRAP_DT_S * MAX_EXTRAP_DT_S
                dt_over = over - MAX_EXTRAP_DT_S
                decay = 1.0 - dt_over / EXTRAP_DECAY_DT_S
                if decay < 0.0:
                    decay = 0.0
                vel_b = endv + a0 * MAX_EXTRAP_DT_S + 0.5 * jk * dt_b2
                pos_b = (endp + endv * MAX_EXTRAP_DT_S + 0.5 * a0 * dt_b2
                         + (1.0 / 6.0) * jk * (dt_b2 * MAX_EXTRAP_DT_S))
                if dt_over >= EXTRAP_DECAY_DT_S:
                    extra = vel_b * (EXTRAP_DECAY_DT_S * 0.5)
                else:
                    extra = vel_b * dt_over * (
                        1.0 - dt_over / (2.0 * EXTRAP_DECAY_DT_S))
                h_pos = pos_b + extra
                h_vel = vel_b * decay

        self.hand_raw_pos = h_pos
        self.hand_raw_vel = h_vel

        if fb_rev is None:
            # The firmware's unseen-skip: no target publish, no TX, counted.
            self.hand_unseen_skips += 1
            return None

        # Age CAP on the extrapolated anchor, counted (leg_interp.cpp:773-774).
        age = float(age_s)
        if age > MOTOR_FB_STALENESS_S:
            age = MOTOR_FB_STALENESS_S
            self.hand_stale_holds += 1
        fb = float(fb_rev)
        fb_ex = fb + float(fb_vel_rps) * age

        # NaN/Inf backstop (leg_interp.cpp:781-784, mirroring the leg stroke-
        # clamp backstop at :659-661): a non-finite command fails EVERY clamp
        # comparison below (NaN compares false to everything), so without this it
        # would pass the lead clamp and the stroke clip untouched and reach the
        # wire.  Hold at the encoder; 0.0 rev (the retract stop) only when the
        # encoder is itself non-finite, as a last resort.
        if not math.isfinite(h_pos):
            h_pos = fb if math.isfinite(fb) else 0.0
            h_vel = 0.0
            self.hand_nonfinite_ticks += 1

        dev = h_pos - fb_ex
        if abs(dev) > abs(self.hand_dev_max):
            self.hand_dev_max = dev
        if abs(dev) > MAX_DEVIATION_HAND_REV:
            self.hand_dev_over_ticks += 1

        d = dev
        if d > MAX_LEAD_HAND_REV:
            d = MAX_LEAD_HAND_REV
            self.hand_lead_clamp_ticks += 1
        elif d < -MAX_LEAD_HAND_REV:
            d = -MAX_LEAD_HAND_REV
            self.hand_lead_clamp_ticks += 1
        h_pos = fb_ex + d
        # vel_ff bound: HAND_VELFF_LIMIT_RPS (300) — NEVER the legs' 3.5.
        if h_vel > HAND_VELFF_LIMIT_RPS:
            h_vel = HAND_VELFF_LIMIT_RPS
        elif h_vel < -HAND_VELFF_LIMIT_RPS:
            h_vel = -HAND_VELFF_LIMIT_RPS
        pre = h_pos
        if h_pos < 0.0:
            h_pos = 0.0
        elif h_pos > HAND_MOTOR_MAX_POSITION:
            h_pos = HAND_MOTOR_MAX_POSITION
        if h_pos != pre:
            h_vel = 0.0
            self.hand_clip_ticks += 1

        self.hand_cmd_pos = h_pos
        self.hand_cmd_vel = h_vel
        return (h_pos, h_vel)
