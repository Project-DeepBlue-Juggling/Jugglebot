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
so `sim/skills_gate.py` (via `sim/stream_chain.py`) can drive the production chain
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
pins the xref pins — is used unchanged.

SCHEDULED PLAYBACK — added 2026-09-14 (can-bridge FW 22, hand C2 spec)
----------------------------------------------------------------------
A HAS_SCHED frame is played on its stamp, not on arrival: :meth:`sched_on_setpoint`
queues it by ``t_start_us`` and every tick (:meth:`tick` / :meth:`tick_hand`)
first runs :meth:`sched_step` — advance both lane groups (cover exhaustion →
C2 stop → hold), then promote the newest due frame at its true phase.  This is
a line-for-line mirror of ``leg_interp.cpp``'s ``sched_*`` block (queue, 2T
cover, stop polynomial + duration bisection, latched-hold resume rule,
promotion-continuity maxima), in integer microseconds for the clock and float64
for the math.  Legacy latches (:meth:`latch_setpoint` / :meth:`latch_hand`)
deactivate the groups exactly as the firmware's legacy latch does, so a stream
that never sends a stamped frame is the pre-FW-22 twin, bit for bit.
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
# = Geometry::HAND_MOTOR_HARD_STOP_REVS (10.701) - Geometry::HAND_CLIP_MARGIN_REV
# (0.2). FW 18 stood the clip off the metal; through FW 17 it was a ZERO-margin
# alias of the hard stop, i.e. a clip AT the stop, where neither hand guard can
# see a stall (the deviation guard's command and encoder AGREE once the slider
# is jammed there, and the lead clamp anchors the setpoint to that encoder).
HAND_MOTOR_MAX_POSITION = 10.501
MAX_DEVIATION_HAND_REV = 2.5
# Age cap on the lead clamp's freshness-aware anchor (canbridge_config.h
# MOTOR_FB_STALENESS_US = 150000 us; leg_interp.cpp:773-774). Past the cap the
# anchor stops following a possibly-dead extrapolation, the frame STILL
# transmits, and the tick is counted (`hand_stale_holds`) so the telemetry gap
# is loud rather than silent.
MOTOR_FB_STALENESS_S = 0.150

# ── FW 22 scheduled playback (canbridge_config.h SCHED_*, pinned by
# tests/firmware/test_sched_c2_twin.py::test_sched_constants_match_the_firmware) ──
SCHED_SEG_US = 25000
SCHED_COVER_US = 2 * SCHED_SEG_US
SCHED_QUEUE_LEN = 4
SCHED_MAX_FUTURE_US = 250000
SCHED_PLAY, SCHED_STOP, SCHED_HOLD = 0, 1, 2
SCHED_PROMO_TOL_LEG = (0.002, 0.1, 10.0)     # (rev, rev/s, rev/s^2)
SCHED_PROMO_TOL_HAND = (0.005, 0.5, 50.0)
SCHED_RESUME_TOL_LEG = (0.005, 0.3, 60.0)    # accept/refuse out of a stop/hold while armed
SCHED_RESUME_TOL_HAND = (0.05, 8.0, 1000.0)
SCHED_GRACE_S = 0.020                        # span-2 cubic continues this long before the stop
SCHED_GRACE_US = 20000
SCHED_EXHAUST_US = SCHED_COVER_US + SCHED_GRACE_US
SCHED_S_MAX = 1.0 + SCHED_GRACE_S / SEGMENT_T_S
STREAM_STOP_LEG_ACCEL_RPS2 = 250.0           # hardware_config.yaml trajectory_op.stream_stop_*
STREAM_STOP_LEG_JERK_RPS3 = 25000.0
STREAM_STOP_HAND_ACCEL_RPS2 = 3500.0
STREAM_STOP_HAND_JERK_RPS3 = 350000.0
# FW 22 hand acceleration torque feedforward (U2b) — canbridge_config.h / trajectory_op.stream_hand_*
HAND_TORQUE_FF_CLAMP_NM = 0.234
HAND_FF_GAIN_SLEW_PER_S = 5.0
HAND_TORQUE_BIAS_CLAMP_NM = 0.05
HAND_TORQUE_BIAS_RATE_NM_PER_S = 0.5
HAND_TORQUE_FADE_PER_S = 50.0
HAND_MEASURED_REFLECTED_INERTIA_KGM2 = 1.05e-5   # hand_envelope.measured_reflected_inertia_kgm2
HAND_TORQUE_FF_J_2PI = HAND_MEASURED_REFLECTED_INERTIA_KGM2 * 6.2831853
HAND_TORQUE_TICK_S = 0.002                       # INTERP_PERIOD_US
SCHED_STOP_BRACKET_S = 0.004
SCHED_STOP_MAX_S = 0.5
SCHED_STOP_BISECT_ITERS = 20
SCHED_STOP_REL_EPS = 1e-4
HAND_FF_GAIN_MAX = 1.5


def span_eval(P0, V0, P1, V1, s):
    """One Hermite span in difference form -> (p, v, a).  leg_interp.cpp span_eval."""
    T = SEGMENT_T_S
    inv_T = 1.0 / SEGMENT_T_S
    d = P1 - P0
    s2 = s * s
    s3 = s2 * s
    p = (P0 + (s3 - 2.0 * s2 + s) * (T * V0) + (-2.0 * s3 + 3.0 * s2) * d
         + (s3 - s2) * (T * V1))
    v = ((-6.0 * s2 + 6.0 * s) * (d * inv_T) + (3.0 * s2 - 4.0 * s + 1.0) * V0
         + (3.0 * s2 - 2.0 * s) * V1)
    a = ((6.0 - 12.0 * s) * (d * inv_T) + (6.0 * s - 4.0) * V0
         + (6.0 * s - 2.0) * V1) * inv_T
    return p, v, a


def play_eval(t0_us, P0, V0, P1, V1, P2, V2, now_us):
    """A frame's 2T cover at ``now_us``.  leg_interp.cpp play_eval."""
    t = (now_us - t0_us) * 1e-6
    if t < 0.0:
        t = 0.0
    if t < SEGMENT_T_S:
        return span_eval(P0, V0, P1, V1, t / SEGMENT_T_S)
    s = (t - SEGMENT_T_S) / SEGMENT_T_S
    if s > SCHED_S_MAX:
        s = SCHED_S_MAX                      # the grace continuation of the same cubic
    return span_eval(P1, V1, P2, V2, s)


def stop_peaks(v0, a0, D):
    """Exact peak |a| and |jerk| of the C2 stop of duration D.  leg_interp.cpp stop_peaks."""
    x = v0 / D
    c0 = a0
    c1 = -6.0 * x - 4.0 * a0
    c2 = 6.0 * x + 3.0 * a0
    apk = abs(c0)
    if c2 != 0.0:
        ts = -c1 / (2.0 * c2)
        if 0.0 < ts < 1.0:
            av = abs(c0 - c1 * c1 / (4.0 * c2))
            if av > apk:
                apk = av
    j0 = abs(c1)
    j1 = abs(2.0 * c2 + c1)
    return apk, (j0 if j0 > j1 else j1) / D


def stop_feasible(evs, eas, D, A, J):
    for v0, a0 in zip(evs, eas):
        apk, jpk = stop_peaks(v0, a0, D)
        ae = abs(a0)
        alim = (A if A > ae else ae) * (1.0 + SCHED_STOP_REL_EPS)
        if not (apk <= alim) or not (jpk <= J):
            return False
    return True


def stop_duration(evs, eas, A, J):
    """Smallest feasible D (bisection) -> (D, capped).  leg_interp.cpp stop_duration."""
    if not any(v != 0.0 or a != 0.0 for v, a in zip(evs, eas)):
        return 0.0, False
    hi = SCHED_STOP_BRACKET_S
    while not stop_feasible(evs, eas, hi, A, J):
        hi *= 2.0
        if hi >= SCHED_STOP_MAX_S:
            return SCHED_STOP_MAX_S, not stop_feasible(evs, eas, SCHED_STOP_MAX_S, A, J)
    lo = 0.0
    for _ in range(SCHED_STOP_BISECT_ITERS):
        mid = 0.5 * (lo + hi)
        if stop_feasible(evs, eas, mid, A, J):
            hi = mid
        else:
            lo = mid
    return hi, False


def stop_eval(P, V, A, D, t):
    """The C2 stop from (P, V, A) over D, at t -> (p, v, a).  leg_interp.cpp stop_eval."""
    if D <= 0.0 or t >= D:
        return P + D * (V * 0.5 + A * D / 12.0), 0.0, 0.0
    if t < 0.0:
        t = 0.0
    u = t / D
    u2 = u * u
    u3 = u2 * u
    u4 = u3 * u
    v = V * (1.0 - 3.0 * u2 + 2.0 * u3) + A * D * (u - 2.0 * u2 + u3)
    a = V * (6.0 * u2 - 6.0 * u) / D + A * (1.0 - 4.0 * u + 3.0 * u2)
    p = P + D * (V * (u - u3 + 0.5 * u4)
                 + A * D * (0.5 * u2 - (2.0 / 3.0) * u3 + 0.25 * u4))
    return p, v, a


class SchedGroup:
    """leg_interp.cpp ``struct SchedGroup`` (first/last = the group's axis range)."""

    def __init__(self, first, last, accel_limit, jerk_limit, tol, rtol):
        self.first, self.last = first, last
        self.accel_limit, self.jerk_limit, self.tol = accel_limit, jerk_limit, tol
        self.rtol = rtol
        self.active = False
        self.phase = SCHED_PLAY
        self.latched = False
        self.restart = False
        self.t0_us = 0
        self.frame_t_start_us = 0
        self.stop_D = 0.0
        z = [0.0] * NUM_AXES
        self.p0, self.v0, self.p1, self.v1, self.p2, self.v2 = (list(z) for _ in range(6))
        self.ep, self.ev, self.ea = list(z), list(z), list(z)
        self.hand_ff_gain = 0.0
        self.dp_max = self.dv_max = self.da_max = 0.0
        self.promo_over = 0


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
        # ladder, not the guard.  ``sim/skills_gate.py`` scores these.
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
        #: ``sim/stream_chain.hand_decay_probe`` (``sim/skills_gate.py``'s caller).
        self.hand_mode = 0
        # ── FW 22 scheduled playback state ──
        self._sq = []                         # pending stamped frames, ordered by t_start_us
        self._sg = [SchedGroup(0, NUM_LEGS, STREAM_STOP_LEG_ACCEL_RPS2,
                               STREAM_STOP_LEG_JERK_RPS3, SCHED_PROMO_TOL_LEG,
                               SCHED_RESUME_TOL_LEG),
                    SchedGroup(HAND_AXIS, NUM_AXES, STREAM_STOP_HAND_ACCEL_RPS2,
                               STREAM_STOP_HAND_JERK_RPS3, SCHED_PROMO_TOL_HAND,
                               SCHED_RESUME_TOL_HAND)]
        self._sched_last_step_us = None
        #: The firmware's output-enable and cold-start gates, as the twin cannot
        #: observe them: ``out_en`` drives the latched-hold refusal, and
        #: ``out_en and not coldstart`` gates the promotion maxima/counters.
        self.sched_out_en = True
        self.sched_coldstart = False
        self.sched_frames = self.sched_demoted = self.sched_future_drops = 0
        self.sched_q_overflow = self.sched_expired = self.sched_superseded = 0
        self.sched_stale = self.sched_stops = self.sched_stop_capped = 0
        self.sched_refused = self.sched_resumes = self.sched_slew_rearms = 0
        self.cmd_acc = [0.0] * NUM_LEGS
        self.hand_raw_acc = 0.0
        self.hand_cmd_acc = 0.0
        # FW 22 hand torque feedforward mirror (leg_interp.cpp U2b block)
        self.hand_base_torque = 0.0      # scheduled frame's torque_ff[6] (the bias target)
        self.hand_recover_slewing = False   # caller-driven: the firmware's s_hand_recover_slewing
        self.hand_ff_ks = 0.0
        self.hand_ff_fade = 0.0
        self.hand_ff_bias = 0.0
        self.hand_cmd_tau = 0.0
        self.hand_tau_clamp_ticks = 0
        self.hand_tau_fade_ticks = 0

    # ── Latch a new MPC setpoint (port of the relevant parts of _on_mpc_command) ──
    def latch_setpoint(self, u0, v0, accel, torque, t_latch,
                       u1=None, u2=None, v1=None):
        """u0/u1/u2 in motor-rev, v0 in rev/s, accel in rev/s^2, torque in Nm.

        u1=None → no forward waypoint (Taylor fallback). u2 only meaningful with u1.
        ``v1`` is the TRANSMITTED u1-knot velocity (rev/s, the v6 wire's
        ``HAS_V1``); ``None`` keeps the flown ``(u2−u1)/T`` forward difference.
        """
        # FW 22: a legacy frame takes the leg lanes back from a scheduled stream.
        g = self._sg[0]
        g.active = g.latched = g.restart = False
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
        now_us = int(round(t_now * 1e6))
        self.sched_step(now_us)
        self.cmd_acc = [0.0] * NUM_LEGS

        if self._sg[0].active:
            # FW 22 scheduled playback — the frame's stamped clock.
            for i in range(NUM_LEGS):
                (self.cmd_pos[i], self.cmd_vel[i],
                 self.cmd_acc[i]) = self._sched_eval(self._sg[0], i, now_us)
        elif self.next_pos is not None:
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
        g = self._sg[1]
        g.active = g.latched = g.restart = False
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
        now_us = int(round(float(t_now) * 1e6))
        self.sched_step(now_us)          # a promotion can activate the lane
        if not self.hand_active:
            self._hand_torque_step(False)
            return None

        hdt = float(t_now) - self.hand_ts
        p0, v0 = self.hand_base_pos, self.hand_base_vel
        a0, jk = self.hand_base_accel, self.hand_jerk
        T = SEGMENT_T_S
        h_pos = 0.0
        h_vel = 0.0
        h_acc = 0.0
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
                h_acc = ((6.0 - 12.0 * s) * (p1 - p0) * inv_T + (6.0 * s - 4.0) * v0
                         + (6.0 * s - 2.0) * v1) * inv_T
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
                h_acc = a0 + jk * over
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
                h_acc = -vel_b / EXTRAP_DECAY_DT_S if dt_over < EXTRAP_DECAY_DT_S else 0.0

        if self._sg[1].active:
            # FW 22: the scheduled group supersedes the legacy ladder above.
            h_pos, h_vel, h_acc = self._sched_eval(self._sg[1], HAND_AXIS, now_us)
        self.hand_raw_pos = h_pos
        self.hand_raw_acc = h_acc
        self.hand_raw_vel = h_vel

        if fb_rev is None:
            # The firmware's unseen-skip: no target publish, no TX, counted.
            self.hand_unseen_skips += 1
            self._hand_torque_step(False)
            return None

        # Age CAP on the extrapolated anchor, counted (leg_interp.cpp:773-774).
        age = float(age_s)
        if age > MOTOR_FB_STALENESS_S:
            age = MOTOR_FB_STALENESS_S
            self.hand_stale_holds += 1
        fb = float(fb_rev)
        fb_ex = fb + float(fb_vel_rps) * age
        lead = clipped = nonfinite = False

        # NaN/Inf backstop (leg_interp.cpp:781-784, mirroring the leg stroke-
        # clamp backstop at :659-661): a non-finite command fails EVERY clamp
        # comparison below (NaN compares false to everything), so without this it
        # would pass the lead clamp and the stroke clip untouched and reach the
        # wire.  Hold at the encoder; 0.0 rev (the retract stop) only when the
        # encoder is itself non-finite, as a last resort.
        if not math.isfinite(h_pos):
            h_pos = fb if math.isfinite(fb) else 0.0
            h_vel = 0.0
            h_acc = 0.0
            self.hand_nonfinite_ticks += 1
            nonfinite = True

        dev_cmd = h_pos - fb_ex
        # FW 22 latched hold: the guard reads the refused incoming command.
        if self._sg[1].active and self._sg[1].latched:
            dev = self.hand_base_pos - fb_ex
        else:
            dev = dev_cmd
        if abs(dev) > abs(self.hand_dev_max):
            self.hand_dev_max = dev
        if abs(dev) > MAX_DEVIATION_HAND_REV:
            self.hand_dev_over_ticks += 1

        d = dev_cmd
        if d > MAX_LEAD_HAND_REV:
            d = MAX_LEAD_HAND_REV
            self.hand_lead_clamp_ticks += 1
            lead = True
        elif d < -MAX_LEAD_HAND_REV:
            d = -MAX_LEAD_HAND_REV
            self.hand_lead_clamp_ticks += 1
            lead = True
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
            clipped = True

        self.hand_cmd_pos = h_pos
        self.hand_cmd_vel = h_vel
        self.hand_cmd_acc = h_acc
        self._hand_torque_step(self.sched_out_en, lead, clipped, nonfinite, h_acc)
        return (h_pos, h_vel)

    def _hand_torque_step(self, on_wire, lead=False, clipped=False, nonfinite=False, a_cmd=0.0):
        """Mirror of leg_interp.cpp's FW 22 hand torque feedforward block:
        ``tau = fade * sat(Ks * J * 2pi * a_cmd + bias)``, every term rate-bounded,
        state reset to 0 off the wire (output disabled / lane idle / unseen).
        Returns and stores ``hand_cmd_tau`` (N m)."""
        if not on_wire:
            self.hand_ff_ks = self.hand_ff_fade = self.hand_ff_bias = 0.0
            self.hand_cmd_tau = 0.0
            return 0.0
        dt = HAND_TORQUE_TICK_S
        g = self._sg[1]
        kstep = HAND_FF_GAIN_SLEW_PER_S * dt
        dk = (g.hand_ff_gain if g.active else 0.0) - self.hand_ff_ks
        self.hand_ff_ks += min(max(dk, -kstep), kstep)
        bt = self.hand_base_torque if g.active else 0.0
        if not math.isfinite(bt):
            bt = 0.0
        bt = min(max(bt, -HAND_TORQUE_BIAS_CLAMP_NM), HAND_TORQUE_BIAS_CLAMP_NM)
        bstep = HAND_TORQUE_BIAS_RATE_NM_PER_S * dt
        self.hand_ff_bias += min(max(bt - self.hand_ff_bias, -bstep), bstep)
        fstep = HAND_TORQUE_FADE_PER_S * dt
        if lead or clipped or nonfinite or self.hand_recover_slewing:
            self.hand_ff_fade = max(0.0, self.hand_ff_fade - fstep)
            self.hand_tau_fade_ticks += 1
        else:
            self.hand_ff_fade = min(1.0, self.hand_ff_fade + fstep)
        raw = self.hand_ff_ks * HAND_TORQUE_FF_J_2PI * a_cmd + self.hand_ff_bias
        if not math.isfinite(raw):
            raw = 0.0
        sat = abs(raw) > HAND_TORQUE_FF_CLAMP_NM
        raw = min(max(raw, -HAND_TORQUE_FF_CLAMP_NM), HAND_TORQUE_FF_CLAMP_NM)
        tau = self.hand_ff_fade * raw
        if sat and self.hand_ff_fade > 0.0:
            self.hand_tau_clamp_ticks += 1
        self.hand_cmd_tau = tau
        return tau

    # ── FW 22 scheduled playback — mirror of leg_interp.cpp sched_* ─────────
    def sched_on_setpoint(self, t_origin_us, recv_us, u0, u1, u2, v0, v1, v2,
                          has_hand=False, torque=None, accel=None,
                          hand_ff_gain=0.0, time_synced=True, full=True,
                          wall_offset_us=0):
        """Ingest one HAS_SCHED frame (7-wide lists).  Returns the verdict:
        ``'sched'`` (queued), ``'demoted'`` (the caller must latch it as legacy),
        or ``'future'`` (refused).  ``full`` = HAS_U1|HAS_U2|HAS_V1|HAS_V2 all set;
        ``wall_offset_us`` = the bridge's wall − mono offset."""
        if not (full and time_synced):
            self.sched_demoted += 1
            return 'demoted'
        t_start = int(t_origin_us) - int(wall_offset_us)
        if t_start - int(recv_us) > SCHED_MAX_FUTURE_US:
            self.sched_future_drops += 1
            return 'future'
        k = float(hand_ff_gain)
        k = 0.0 if k < 0.0 else (HAND_FF_GAIN_MAX if k > HAND_FF_GAIN_MAX else k)
        f = {'t_start_us': t_start,
             'p0': [float(x) for x in u0], 'p1': [float(x) for x in u1],
             'p2': [float(x) for x in u2], 'v0': [float(x) for x in v0],
             'v1': [float(x) for x in v1], 'v2': [float(x) for x in v2],
             'torque': [float(x) for x in torque] if torque is not None else [0.0] * NUM_AXES,
             'accel': [float(x) for x in accel] if accel is not None else [0.0] * NUM_AXES,
             'hand_ff_gain': k, 'has_hand': bool(has_hand)}
        q = self._sq
        for i, e in enumerate(q):
            if e['t_start_us'] == t_start:
                q[i] = f
                self.sched_frames += 1
                return 'sched'
        drop_incoming = False
        if len(q) == SCHED_QUEUE_LEN:
            self.sched_q_overflow += 1
            if t_start < q[0]['t_start_us']:
                drop_incoming = True
            else:
                del q[0]
        if not drop_incoming:
            pos = len(q)
            while pos > 0 and q[pos - 1]['t_start_us'] > t_start:
                pos -= 1
            q.insert(pos, f)
        self.sched_frames += 1
        return 'sched'

    def arm_edge(self):
        """The firmware's output-enable false→true edge, as it touches the lanes."""
        self.hand_active = False
        self.hand_next_pos = self.hand_next2_pos = self.hand_v1 = None
        self.hand_prev_accel = None
        gh, gl = self._sg[1], self._sg[0]
        gh.active = gh.latched = gh.restart = False
        if gl.active and gl.phase != SCHED_PLAY:
            gl.restart = True
        gl.latched = False
        for g in self._sg:
            g.dp_max = g.dv_max = g.da_max = 0.0

    def _sched_eval(self, g, i, now_us):
        if g.phase == SCHED_PLAY:
            return play_eval(g.t0_us, g.p0[i], g.v0[i], g.p1[i], g.v1[i],
                             g.p2[i], g.v2[i], now_us)
        if g.phase == SCHED_STOP:
            return stop_eval(g.ep[i], g.ev[i], g.ea[i], g.stop_D,
                             (now_us - g.t0_us) * 1e-6)
        return g.ep[i], 0.0, 0.0

    def _sched_advance(self, g, now_us):
        if g.phase == SCHED_PLAY and (now_us - g.t0_us) >= SCHED_EXHAUST_US:
            for i in range(g.first, g.last):
                g.ep[i], g.ev[i], g.ea[i] = span_eval(g.p1[i], g.v1[i],
                                                      g.p2[i], g.v2[i], SCHED_S_MAX)
            g.stop_D, capped = stop_duration(g.ev[g.first:g.last], g.ea[g.first:g.last],
                                             g.accel_limit, g.jerk_limit)
            if capped:
                self.sched_stop_capped += 1
            g.t0_us += SCHED_EXHAUST_US
            g.phase = SCHED_STOP
            self.sched_stops += 1
        if g.phase == SCHED_STOP and (now_us - g.t0_us) * 1e-6 >= g.stop_D:
            for i in range(g.first, g.last):
                p, _, _ = stop_eval(g.ep[i], g.ev[i], g.ea[i], g.stop_D, g.stop_D)
                g.ep[i], g.ev[i], g.ea[i] = p, 0.0, 0.0
            g.phase = SCHED_HOLD

    def _sched_apply(self, g, f, now_us, out_en, counts):
        if g.active and f['t_start_us'] < g.frame_t_start_us:
            self.sched_stale += 1
            return
        if g.active and not g.restart:
            # Handover instant: a playing group whose cover ended inside this tick
            # has not been advanced yet (promotion runs first) — compare at the
            # cover end (leg_interp.cpp sched_apply t_cmp).
            t_cmp = (g.t0_us + SCHED_EXHAUST_US
                     if g.phase == SCHED_PLAY and (now_us - g.t0_us) > SCHED_EXHAUST_US
                     else now_us)
            dp = dv = da = 0.0
            for i in range(g.first, g.last):
                po, vo, ao = self._sched_eval(g, i, t_cmp)
                pn, vn, an = play_eval(f['t_start_us'], f['p0'][i], f['v0'][i],
                                       f['p1'][i], f['v1'][i], f['p2'][i],
                                       f['v2'][i], t_cmp)
                dp = max(dp, abs(pn - po))
                dv = max(dv, abs(vn - vo))
                da = max(da, abs(an - ao))
            over = dp > g.tol[0] or dv > g.tol[1] or da > g.tol[2]
            resume_ok = dp <= g.rtol[0] and dv <= g.rtol[1] and da <= g.rtol[2]
            if g.phase != SCHED_PLAY and not resume_ok and out_en:
                g.latched = True
                self.sched_refused += 1
                self._sched_set_base_pos(g, f)
                return
            if counts:
                g.dp_max = max(g.dp_max, dp)
                g.dv_max = max(g.dv_max, dv)
                g.da_max = max(g.da_max, da)
                if over:
                    g.promo_over += 1
            if g.phase != SCHED_PLAY:
                self.sched_resumes += 1
        elif g.active and g.restart and g.first == 0 and counts:
            self.sched_slew_rearms += 1      # firmware re-arms the leg recovery slew here
        for i in range(g.first, g.last):
            g.p0[i], g.v0[i] = f['p0'][i], f['v0'][i]
            g.p1[i], g.v1[i] = f['p1'][i], f['v1'][i]
            g.p2[i], g.v2[i] = f['p2'][i], f['v2'][i]
        self._sched_set_base_pos(g, f)
        if g.first == 0:
            for i in range(NUM_LEGS):
                self.base_vel[i] = f['v0'][i]
                self.base_accel[i] = f['accel'][i]
                self.base_torque[i] = f['torque'][i]
        else:
            self.hand_base_vel = f['v0'][HAND_AXIS]
            self.hand_base_accel = f['accel'][HAND_AXIS]
            self.hand_base_torque = f['torque'][HAND_AXIS]
            self.hand_active = True
        g.t0_us = g.frame_t_start_us = f['t_start_us']
        g.phase = SCHED_PLAY
        g.active = True
        g.latched = g.restart = False
        g.hand_ff_gain = f['hand_ff_gain']

    def _sched_set_base_pos(self, g, f):
        if g.first == 0:
            for i in range(NUM_LEGS):
                self.base_pos[i] = f['p0'][i]
        else:
            self.hand_base_pos = f['p0'][HAND_AXIS]

    def sched_step(self, now_us):
        """leg_interp.cpp sched_tick_begin — once per tick (idempotent per now_us)."""
        if self._sched_last_step_us == now_us:
            return
        self._sched_last_step_us = now_us
        self._sched_promote(now_us)
        # Promote FIRST, then advance (leg_interp.cpp sched_tick_begin): advancing
        # first would stop on every cover end that falls between two ticks and
        # refuse the on-time frame stamped at it.
        for g in self._sg:
            if g.active:
                self._sched_advance(g, now_us)

    def _sched_promote(self, now_us):
        q = self._sq
        if not q or q[0]['t_start_us'] > now_us:
            return
        cand = None
        k = 0
        while k < len(q) and q[k]['t_start_us'] <= now_us:
            if now_us - q[k]['t_start_us'] >= SCHED_COVER_US:
                self.sched_expired += 1
            else:
                if cand is not None:
                    self.sched_superseded += 1
                cand = q[k]
            k += 1
        del q[:k]
        if cand is None:
            return
        out_en = self.sched_out_en
        counts = out_en and not self.sched_coldstart
        self._sched_apply(self._sg[0], cand, now_us, out_en, counts)
        if cand['has_hand']:
            self._sched_apply(self._sg[1], cand, now_us, out_en, counts)

    def sched_phase(self, group):
        """0 off, 1 play, 2 stop, 3 hold (interp_sched_phase)."""
        g = self._sg[group]
        return 0 if not g.active else g.phase + 1
