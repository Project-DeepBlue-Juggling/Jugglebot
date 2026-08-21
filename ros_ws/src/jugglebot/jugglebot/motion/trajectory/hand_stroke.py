"""Hand stroke model — THE host-side copy of ``Teensy_code_platform/Trajectory.h``.

The Platform Teensy builds every hand profile from a closed form parameterised
by one number, the commanded ``event_vel``.  Anything on the Jetson that needs
to know *when* the hand is moving — how long after ball release the throw stroke
is still decelerating, how much lead a catch stroke needs before its event — has
to reproduce that closed form.  This module is the one place that does.

**Why one place.**  The model already existed twice before this file: the
firmware (``Teensy_code_platform/Trajectory.h``) and the simulation mirror
(``sim/hand/trajectory.py``); the 2026-07-25 Phase-0 timeline probe
(``tools/probes/hand_stroke_timeline.py``) briefly made a third, and
``catch_coordinator_node`` needing it would have made a fourth.  That is not a
hypothetical drift risk — it is a failure this codebase has already realised:
``JB_OP_HAND_CATCH_PRIME_REV`` was a hand-maintained 9.858 against a stroke top
of 9.9594 rev, 3.2 mm off, so a "catch from rest at the top" never actually
started from rest at the top.  (Closed 2026-07-26: the prime is now the derived
stroke top, guarded by ``tests/motion/test_hand_stroke.py``
``::test_catch_prime_equals_the_stroke_top``.  The lesson stands even though the
instance is fixed — it drifted silently for the life of the constant, which is
exactly what a second copy of the *timing* would do.)  A second host copy of the
timing would fail the same way and much more quietly, because the probe that
scores the bench capture and the node that ships the suppression window would
then be scoring different models.  The probe imports this module; there is no
second copy to pin.

Positions are motor revs measured from the firmware's encoder zero (the homed
physical bottom of travel), exactly as ``buildSegment`` emits them
(``pos * LINEAR_GAIN``).  Throw times are relative to BALL RELEASE — the
``makeThrow`` convention, ``shiftTime(-t2)``.  Catch times are relative to the
firmware's kind-1 anchor, which ``shiftTime(-(t5-t4))`` puts at the START of the
catch velocity hold (``calcCatch``'s "centre of vel hold" comment describes
``t2 + airT``, not ``t5``).

Constants come from ``jugglebot.hardware_config`` (generated from
``config/hardware_config.yaml``) — the same YAML that generates the firmware's
``Teensy_code_platform/hardware_config.h`` ``TeensyTraj::`` block, so constant drift
between host and firmware is structurally impossible.  Verified header-vs-codegen
equal for all twelve ``TeensyTraj::`` stroke constants plus ``TeensyOp::``'s
``SAFETY_GAP_US`` (``tests/motion/test_hand_stroke.py``
``::test_shipped_header_matches_generated_constants``).

*This module* imports only ``math`` + ``jugglebot.hardware_config`` — no ROS2, no
numpy, no repo-root or ``controller`` imports.  Note that *importing* it through
the package (``from jugglebot.motion.trajectory import hand_stroke``) still pulls
numpy in, because ``motion/trajectory/__init__.py`` eagerly imports the MVP
trajectory generator.  Measured: numpy only — no scipy, no casadi, no rclpy.  A
consumer that genuinely needs the stdlib-only property must import the module
file directly rather than through the package.
"""

from __future__ import annotations

import math

import jugglebot.hardware_config as hw

# ── firmware geometry (Trajectory.h:32, :63) ─────────────────────────────
LINEAR_GAIN_REV_PER_M = (hw.TEENSY_TRAJ_LINEAR_GAIN_FACTOR
                         / (math.pi * hw.TEENSY_TRAJ_HAND_SPOOL_RADIUS_M * 2.0))
TOTAL_STROKE_M = (hw.TEENSY_TRAJ_HAND_STROKE_M
                  - 2.0 * hw.TEENSY_TRAJ_STROKE_MARGIN_M)
TOTAL_STROKE_REV = TOTAL_STROKE_M * LINEAR_GAIN_REV_PER_M

#: ``x3`` — the throw stroke's end AND the catch trajectory's first sample.
#: ``x3 = accelSt + velHold = totalStroke`` holds ALGEBRAICALLY, independent of
#: the commanded velocity (the decel segment contributes
#: ``0.5·INERTIA_RATIO·v·t_acc``, so ``x3 = 0.5·v·t_acc·(1 + INERTIA_RATIO)
#: + velHold = accelSt + velHold``).  Verified to 1.7e-16 m over
#: ``v in [0.3, 7.0]``.  This identity is why a correctly-timed catch arm costs
#: nothing: the hand is already standing exactly where the catch profile starts.
STROKE_TOP_REV = TOTAL_STROKE_REV

#: ``Teensy_code_platform.ino:642`` compares ``now + smoothDur + SAFETY_GAP`` against the
#: main trajectory's first absolute sample and REFUSES the whole command if it
#: does not fit — printing to serial only (``:643``), invisible to ROS.
SAFETY_GAP_S = hw.TEENSY_SAFETY_GAP_US / 1e6

# ── Phase-1 timing policy ────────────────────────────────────────────────

#: Slack added to the modelled stroke end before a scheduled kind-0/1/2 command
#: may be dispatched.
#:
#: NOT optional padding.  The announcement's ``throw_time`` is the *intended*
#: release; the physical release lands LATER because nothing compensates the
#: ROS-service transit between the caller's clock read
#: (``catch_coordinator_node``/``reload_coordinator_node``) and
#: ``teensy_bridge_node._svc_set_hand_traj``'s re-stamp from its OWN clock
#: (``int(time.time()*1000) + int(event_delay*1000)``), and
#: ``JB_OP_TOSS_RELEASE_LATENCY_MS`` ships 0.0.  Measured across the 2026-07-25
#: evidence set (seven tosses, three sessions, two independent timestamp paths):
#: **+12.8 to +23.4 ms**.  A window of ``throw_time + t_dec`` with zero margin
#: therefore expires while the stroke is still on its deceleration ramp and
#: reproduces the defect exactly.
#:
#: 40 ms = **1.7x the worst measured shift**, and it costs almost nothing: it
#: consumes 40 ms of a window that is 395 ms wide at the nominal 0.80 s flight
#: and 50 ms wide at ``FLIGHT_TIME_MIN_S``, which is the DERIVED band floor
#: **0.4949 s** since 2026-08-18 (contract C-HAND-3,
#: ``ros_ws/docs/hand_throw_envelope.md``).  It read 115 ms against the
#: hand-picked 0.55 s floor this line was written for.  Not a coincidence: the
#: floor is defined as the flight at which this very window reaches
#: ``hand_throw_envelope.arm_window_margin_s`` = 0.050 s.
#:
#: NOTE this is NOT ``hardware_config.yaml``'s ``arm_window_margin_s`` (0.05),
#: which is the envelope's ADMISSION criterion.  Two different quantities that
#: happen to be about the same window; do not collapse them.
ARM_SUPPRESS_MARGIN_S = 0.040

#: Assumed worst-case settle error of the hand against ``x3`` when a catch is
#: armed at the top of a completed throw stroke.  Deliberately the SAME 0.10 rev
#: (3.2 mm) band ``tests/hardware/session_anomaly_fixes.md`` § Section HAND gates
#: the post-fix capture's ``dip_below_x3`` on, so any capture that PASSES the
#: bench gate is inside the allowance this arithmetic assumed — one number, two
#: enforcement surfaces, no free parameter.
#:
#: Its *upward* counterpart is NOT one number and must not be quoted as one from
#: here.  This comment paired ``dip_below_x3 <= 0.10`` with ``peak <= 10.060``
#: (= x3 + 0.10) until 2026-08-21; the ``peak`` row went TIER-DEPENDENT on
#: 2026-07-28 because ballistic coast grows with throw speed and a single
#: ``10.060`` aborted 10 of the 17 tosses of the 2026-07-27 sitting for a reason
#: that was never a regression.  ``10.060`` is now the 0.38/0.60 m band only;
#: read the ladder in § PASS/ABORT row 3.  The settle band below is unaffected —
#: it is the DOWNWARD allowance, and it did not move.
HAND_SETTLE_BAND_REV = 0.10


def rev_to_mm(rev: float) -> float:
    """Motor revs → mm of cable travel."""
    return float(rev) / LINEAR_GAIN_REV_PER_M * 1000.0


def mm_to_rev(mm: float) -> float:
    """mm of cable travel → motor revs."""
    return float(mm) / 1000.0 * LINEAR_GAIN_REV_PER_M


def clamp_event_vel(v_mps: float) -> float:
    """Clamp to the range ``teensy_bridge_node._svc_set_hand_traj`` accepts.

    The bridge REJECTS out-of-range values rather than clamping, so a value
    outside the range never reaches the firmware at all; clamping here keeps the
    model finite for an out-of-range input instead of dividing by ~0.
    """
    return max(hw.TEENSY_TRAJ_MIN_EVENT_VEL_MPS,
               min(hw.TEENSY_TRAJ_MAX_EVENT_VEL_MPS, float(v_mps)))


class HandStrokeModel:
    """Closed-form throw + catch stroke — ``Trajectory.h``'s ``calcThrow``
    (:95-115) and ``calcCatch`` (:117-139) with the shipped ``TeensyTraj::``
    constants, in float64.

    The firmware evaluates the same algebra in ``float``; the divergence is
    ~1e-8 and irrelevant to every consumer here (the tightest is a 40 ms
    margin).
    """

    def __init__(self, v_throw_mps: float):
        v = clamp_event_vel(v_throw_mps)
        self.v = v
        ir = hw.TEENSY_TRAJ_INERTIA_RATIO
        total = TOTAL_STROKE_M

        vel_hold = hw.TEENSY_TRAJ_THROW_VEL_HOLD_PCT * total
        accel_st = total - vel_hold
        self.t_acc = 2.0 / (ir + 1.0) * accel_st / v
        self.t_vel = vel_hold / v
        self.t_dec = self.t_acc * ir
        self.throwA = v / self.t_acc
        self.throwD = -self.throwA / ir
        self.x1_m = 0.5 * self.throwA * self.t_acc ** 2
        self.x2_m = self.x1_m + v * self.t_vel
        self.x3_m = (self.x2_m + v * self.t_dec
                     + 0.5 * self.throwD * self.t_dec ** 2)

        vC = -hw.TEENSY_TRAJ_CATCH_VEL_RATIO * v
        irC = 1.0 / ir
        velH = hw.TEENSY_TRAJ_CATCH_VEL_HOLD_PCT * total
        accS = total - velH
        self.t_acc_catch = -(2.0 / (irC + 1.0)) * accS / vC
        self.t_vel_catch = -velH / vC
        self.t_dec_catch = self.t_acc_catch * irC
        self.catchA = vC / self.t_acc_catch
        self.x5_m = self.x3_m + 0.5 * self.catchA * self.t_acc_catch ** 2
        self.x6_m = self.x5_m + vC * self.t_vel_catch
        self.vC = vC

    # ── revs ─────────────────────────────────────────────────────────────
    @property
    def x1_rev(self) -> float:
        return self.x1_m * LINEAR_GAIN_REV_PER_M

    @property
    def x2_rev(self) -> float:
        """Ball release — end of the velocity hold, start of the decel ramp."""
        return self.x2_m * LINEAR_GAIN_REV_PER_M

    @property
    def x3_rev(self) -> float:
        """Throw-stroke end / catch-stroke start (velocity-independent)."""
        return self.x3_m * LINEAR_GAIN_REV_PER_M

    @property
    def x5_rev(self) -> float:
        """Ball contact on the catch stroke."""
        return self.x5_m * LINEAR_GAIN_REV_PER_M

    @property
    def x6_rev(self) -> float:
        return self.x6_m * LINEAR_GAIN_REV_PER_M

    @property
    def stroke_start_rel(self) -> float:
        """Throw-stroke first sample, relative to release (negative)."""
        return -(self.t_acc + self.t_vel)

    @property
    def stroke_end_rel(self) -> float:
        """Throw-stroke last sample (hand at rest at ``x3``), rel. to release."""
        return self.t_dec

    def pos_rev(self, t_rel_release: float) -> float:
        """Commanded throw-stroke position (rev) at ``t`` relative to release."""
        t = float(t_rel_release) - self.stroke_start_rel   # 0 = accel start
        if t <= 0.0:
            return 0.0
        if t <= self.t_acc:
            return 0.5 * self.throwA * t * t * LINEAR_GAIN_REV_PER_M
        if t <= self.t_acc + self.t_vel:
            tau = t - self.t_acc
            return (self.x1_m + self.v * tau) * LINEAR_GAIN_REV_PER_M
        if t <= self.t_acc + self.t_vel + self.t_dec:
            tau = t - (self.t_acc + self.t_vel)
            return ((self.x2_m + self.v * tau + 0.5 * self.throwD * tau * tau)
                    * LINEAR_GAIN_REV_PER_M)
        return self.x3_m * LINEAR_GAIN_REV_PER_M


# ── scalar helpers (the shapes the callers actually need) ────────────────

def throw_decel_s(v_throw_mps: float) -> float:
    """Seconds from ball release to the throw stroke's last sample.

    ``t_dec = INERTIA_RATIO * t_acc`` — 65.1 ms at the nominal 0.80 s flight
    (v 3.93 m/s), 104.9 ms at the DERIVED ``FLIGHT_TIME_MIN_S`` of 0.4949 s
    (v 2.43 m/s; it read 94.5 ms at the hand-picked 0.55 s floor, retired
    2026-08-18 by contract C-HAND-3).  The
    window this bounds is the one a kind-1 catch arm was landing inside: the
    2026-07-25 captures show the arm at release + 8-18 ms, mid-ramp, clearing
    the queue and re-preluding from a position the hand was travelling through
    at ~120 rev/s.
    """
    return HandStrokeModel(v_throw_mps).t_dec


def catch_lead_s(v_armed_mps: float) -> float:
    """Seconds between a catch trajectory's FIRST sample and its event instant.

    ``t_acc_catch = 0.404073 / v`` per COMMANDED velocity (per the hand's own
    catch speed ``|vC| = 0.6·v`` it is 0.242444).  ``makeCatch``'s
    ``shiftTime(-(t5-t4))`` puts the kind-1 ``t = 0`` at the start of the catch
    velocity hold, so the profile's first sample sits ``t_acc_catch`` BEFORE the
    event and the Teensy's time-budget check measures against that instant, not
    the event.
    """
    return HandStrokeModel(v_armed_mps).t_acc_catch


#: Quintic landmarks, generated from the same YAML as the firmware's
#: ``TeensyTraj::`` block.  ``s`` is the rest-to-rest shape
#: (``10t^3 - 15t^4 + 6t^5``); ``h(tau) = tau(1-tau)^3(3tau+1)`` is what a
#: NON-ZERO start velocity contributes, ``pos = x0 + delta*s + (v0*T)*h``.
QUINTIC_S2_MAX = hw.TEENSY_TRAJ_QUINTIC_S2_MAX
QUINTIC_H_MAX = hw.TEENSY_TRAJ_QUINTIC_H_MAX
QUINTIC_H2_MAX = hw.TEENSY_TRAJ_QUINTIC_H2_MAX
MAX_SMOOTH_MOVE_ACCEL_RPS2 = hw.TEENSY_TRAJ_MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2

#: ``|current_hand_velocity|`` at or below which ``makeSmoothMove`` treats the
#: hand as AT REST (``Trajectory.h``).  See the YAML comment for the measured
#: dither / traverse separation this sits between.
SMOOTH_MOVE_V0_DEADBAND_RPS = hw.TEENSY_TRAJ_SMOOTH_MOVE_V0_DEADBAND_RPS

#: Duration floor — ``fmaxf(T, 0.05f)``, ``Trajectory.h``.
SMOOTH_MOVE_MIN_DURATION_S = 0.05


def smooth_move_duration_s(delta_rev: float, v0_rps: float = 0.0) -> float:
    """The ACCEL-LIMITED smooth-move duration (``Trajectory.h``'s ``smoothMoveDuration``).

    **Not the whole of what the firmware emits at non-zero ``v0``.**
    ``makeSmoothMove`` substitutes the rest-to-rest duration whenever the honoured
    profile's excursion would leave the stroke or its duration would exceed
    ``smoothMoveMaxDuration() = 0.78964`` s, so this function over-reports on the
    fallback branch (at 119.6 rev/s it returns 4.71 s where the firmware emits a
    0.05 s floored hold).  Harmless today — every caller in this repository passes
    ``v0 = 0``, where the two agree exactly — but a caller that starts feeding the
    LIVE velocity in to size an arm window must model the fallback too, or it will
    refuse or defer exactly the fast arms the prelude exists to serve.
    ``sim/hand/trajectory.plan_smooth_move`` is the branch-accurate mirror.

    Two properties that matter more than the formula:

    * the "already there" dead-band is ``|delta| < 1e-6`` rev = **3.16e-5 mm**,
      which no live encoder reading will ever satisfy against a float target —
      so on hardware the prelude is essentially never *exactly* empty.  Since
      Phase 4 that branch ALSO requires the hand to be at rest
      (``|v0| <= SMOOTH_MOVE_V0_DEADBAND_RPS``): at the target but moving now
      yields a braking profile rather than nothing;
    * every non-empty duration is floored at **0.05 s** (``fmaxf(T, 0.05f)``).

    Together those mean a catch armed with the hand "at rest at the top" still
    costs 50-76 ms of prelude, not zero.  That is harmless motion (0.63 mm at
    24 mm/s for a 0.02 rev residual) but it is NOT free time, and a fit check
    that assumed a zero prelude would hand the Teensy a command it refuses at
    ``:642`` — printing to serial only, so the catch silently never fires.

    **The ``v0 != 0`` form** (Phase 4).  ``a(tau)*T^2 = delta*s''(tau) +
    (v0*T)*h''(tau)``, bounded by ``|delta|*S2 + |v0|*T*H2``, so the duration is
    the positive root of ``a_max*T^2 - |v0|*H2*T - |delta|*S2 = 0``.  The
    ``v0 == 0`` path is the historical ``sqrt(|delta|*S2/a_max)`` expression
    verbatim, so every rest-to-rest caller — which is every caller in this
    repository today, including ``PRELUDE_ALLOWANCE_S`` and
    ``tools/probes/hand_stroke_timeline.py``'s model of the observed PRE-fix
    from-rest quintic — is unchanged to the last bit.
    """
    if abs(float(delta_rev)) < 1e-6 and abs(float(v0_rps)) <= SMOOTH_MOVE_V0_DEADBAND_RPS:
        return 0.0
    v0 = 0.0 if abs(float(v0_rps)) <= SMOOTH_MOVE_V0_DEADBAND_RPS else float(v0_rps)
    c = abs(float(delta_rev)) * QUINTIC_S2_MAX
    if v0 == 0.0:
        T = math.sqrt(c / MAX_SMOOTH_MOVE_ACCEL_RPS2)
    else:
        b = abs(v0) * QUINTIC_H2_MAX
        T = ((b + math.sqrt(b * b + 4.0 * MAX_SMOOTH_MOVE_ACCEL_RPS2 * c))
             / (2.0 * MAX_SMOOTH_MOVE_ACCEL_RPS2))
    return max(T, SMOOTH_MOVE_MIN_DURATION_S)


def smooth_move_overshoot_rev(v0_rps: float, duration_s: float) -> float:
    """Extra excursion a non-zero start velocity adds, in the direction of travel.

    ``|v0| * T * QUINTIC_H_MAX`` — an exact upper bound, and exact outright when
    the target coincides with the start (the braking case), because ``h`` and
    ``s`` do not peak at the same ``tau``.  This is the quantity a
    velocity-continuous prelude has to be checked against the stroke end for:
    ``GEOM_HAND_MOTOR_HARD_STOP_REVS - smooth_move_excursion_margin_rev``
    above, the homing reference below.  ``Trajectory.h``'s
    ``smoothMoveExcursion`` computes the exact interval; this is the closed-form
    bound a host-side caller can size a window with.
    """
    return abs(float(v0_rps)) * float(duration_s) * QUINTIC_H_MAX


def smooth_move_max_continuous_v0_rps(headroom_rev: float) -> float:
    """Largest ``|v0|`` whose braking profile still fits in ``headroom_rev``.

    Solving ``QUINTIC_H_MAX * H2 * v0^2 / a_max = headroom`` (the braking case,
    ``delta = 0``, where ``T = |v0|*H2/a_max``).  **This is the width of the
    velocity band over which Phase 4's continuity is actually affordable**, and
    it is narrow: 9.1 rev/s against the 0.6406 rev the stroke top leaves below
    the 10.6 rev ceiling, 19.96 rev/s against the 3.10 rev a mid-stroke freeze
    leaves.  Above it the profile falls back to rest-to-rest, because the
    acceleration needed to arrest sooner is not bounded by anything the firmware
    declares — ``MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2 = 100`` rev/s² is a COMFORT
    limit 19-60x below what the throw profile itself commands (1908 rev/s² at
    3.93 m/s, 6055 at the 7.0 m/s band top), and closing that gap is an
    operator decision about envelope semantics, not an implementation detail.
    """
    return math.sqrt(max(0.0, float(headroom_rev)) * MAX_SMOOTH_MOVE_ACCEL_RPS2
                     / (QUINTIC_H_MAX * QUINTIC_H2_MAX))


def smooth_move_peak_vel_rps(delta_rev: float) -> float:
    """Peak velocity ``makeSmoothMove`` actually commands over ``delta_rev``.

    ``Trajectory.h:283`` emits ``vel = delta * s'(tau) / T`` with
    ``s'(tau) = 30tau^2 - 60tau^3 + 30tau^4``, whose maximum is **1.875** at
    ``tau = 0.5``.  So the peak is ``|delta| * 1.875 / T``.

    **Read this before quoting sqrt(a * delta).**  A bang-bang (triangular)
    profile over the same stroke would peak at ``sqrt(a * |delta|)`` — 31.56
    rev/s at the full 9.9594 rev stroke — and that figure was quoted as "the
    peak ascent speed" in ``reload_coordinator_node``'s
    ``_THROW_STROKE_VEL_RPS`` comment, in ``test_toss_coordinator``, and in the
    HAND-3 bench row.  The shipped profile is a quintic, not a triangle: its
    real peak is ``1.875 / sqrt(QUINTIC_S2_MAX)`` = **0.7803x** that figure,
    i.e. **24.63 rev/s** at the full stroke.

    Both numbers have a use and they are not interchangeable:

    * ``sqrt(a * |delta|)`` is a valid conservative UPPER BOUND, which is the
      right thing for a guard threshold to clear — a guard that clears the
      bound clears the real peak with room to spare;
    * this function is the right thing for a BENCH CRITERION, because a
      capture is compared against what the firmware commands.  Scoring a
      measured ascent against the bound accepts a 28 % overspeed as nominal,
      and an overspeed on a smooth move is the signature of a re-seeded or
      clobbered profile — exactly the failure class this plan exists to catch.

    Rest-to-rest, deliberately: it is the model a BENCH row scores a prime or a
    retract ascent against, and both of those are dispatched to a parked hand.
    Since Phase 4 the profile is velocity-continuous when the live velocity
    exceeds :data:`SMOOTH_MOVE_V0_DEADBAND_RPS`, and its peak velocity is then
    ``(|delta|*1.875 + |v0|*T*max|h'|) / T`` — but ``max|h'| = 1`` occurs at
    ``tau = 0``, i.e. at ``v0`` itself, so a velocity-continuous move never peaks
    below its own start speed and a bench row written against the rest-to-rest
    peak would mis-score it.  That is why H3.7 gates a PRIME (parked hand,
    ``v0`` inside the dead-band, rest-to-rest by construction) and not an
    arbitrary smooth move.
    """
    dur = smooth_move_duration_s(delta_rev)
    if dur <= 0.0:
        return 0.0
    return abs(float(delta_rev)) * 1.875 / dur


def smooth_move_peak_vel_bound_rps(delta_rev: float) -> float:
    """Bang-bang upper bound ``sqrt(a * |delta|)`` on the smooth-move peak.

    NOT what the firmware commands — see :func:`smooth_move_peak_vel_rps` for
    why both exist and which one a bench criterion may use.  Kept as a named
    function so the guard comments that legitimately want a conservative bound
    quote a model rather than a hand-written literal.
    """
    return math.sqrt(hw.TEENSY_TRAJ_MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2
                     * abs(float(delta_rev)))


#: Prelude the arm-fit check budgets for — ``smooth_move_duration_s`` of the
#: post-fix settle band (76.0 ms at 0.10 rev).
PRELUDE_ALLOWANCE_S = smooth_move_duration_s(HAND_SETTLE_BAND_REV)


def stroke_clear_time(t_release_s: float, v_throw_mps: float,
                      margin_s: float = ARM_SUPPRESS_MARGIN_S) -> float:
    """Instant (same clock as ``t_release_s``) after which a scheduled kind-0/1/2
    command can no longer land inside a live throw stroke.

    ``t_release + t_dec(v) + margin``.  ``t_release_s`` is the ANNOUNCED release
    (``ThrowAnnouncement.throw_time``); ``margin_s`` covers the announcement's
    measured earliness against the physical release — see
    :data:`ARM_SUPPRESS_MARGIN_S`.
    """
    return (float(t_release_s) + throw_decel_s(v_throw_mps) + float(margin_s))


def required_arm_lead_s(v_armed_mps: float,
                        prelude_s: float = PRELUDE_ALLOWANCE_S) -> float:
    """Lead a kind-1 catch arm needs before its event, or the Teensy refuses it.

    ``t_acc_catch(v) + prelude + SAFETY_GAP`` — the host-side restatement of
    ``Teensy_code_platform.ino:642``'s budget check, so the refusal can be predicted
    instead of discovered on the serial console.

    **It is a lower bound, deliberately, and two terms are excluded.  Read them
    before relying on this number at a margin.**

    1. *The AT-REST prelude.*  ``prelude_s`` defaults to
       :data:`PRELUDE_ALLOWANCE_S`, which is ``smooth_move_duration_s`` of the
       post-fix settle band — the prelude that applies when the arm lands after
       the stroke, which is the case this gate exists to produce.  The firmware
       computes the prelude from the LIVE encoder (``Trajectory.h:531-532``), so
       on the gate's *forced* branch (window closed, hand still mid-stroke) the
       real prelude is 0.37-0.76 s, an order of magnitude larger, and the Teensy
       may refuse that dispatch outright.  That is not a regression — it is
       exactly the pre-fix arithmetic, and ``:642`` returns BEFORE
       ``packedMsgs.clear()`` (``Teensy_code_platform.ino:642`` vs ``:648``), so a refused
       command leaves the live throw stroke intact: the cost is a lost catch, not
       a clobbered stroke.  See ``catch_coordinator_node._throw_stroke_gate_ok``.
    2. *The DOWNSTREAM service transit.*  Only the UPSTREAM leg cancels.
       ``teensy_bridge_node._svc_set_hand_traj`` re-stamps the absolute event from
       its OWN clock (``int(time.time()*1000) + int(event_delay*1000)``), so the
       caller→bridge transit shifts the event and drops out — but ``:642``'s
       ``now_us`` is read at the PLATFORM TEENSY, after the bridge→can-bridge→CAN3
       hop, and that leg is pure budget loss.  It is bounded above by the ~23 ms
       measured announcement-to-release shift (which contains both legs).

    Both exclusions bias the same way — this function UNDER-states the true
    requirement, so the gate is marginally more willing to defer than it should
    be.  Sizing: at the shipped flight band the caller takes
    ``max(_MIN_EVENT_DELAY_S = 0.3, this)`` and the 0.3 floor binds at every
    nominal armed velocity (budget **0.282 s at the 0.4949 s band floor, 0.176 s
    at the 1.1485 s ceiling**; the budget only overtakes the floor below
    ``v_armed`` 1.98 m/s).  The tightest case leaves **18.0 ms** of floor
    headroom against a ~23 ms transit bound — a 5 ms shortfall, against the slack
    the gate actually dispatches with, because it fires at the FIRST tick after
    the window opens rather than at the last.  (Those three figures read
    0.284 s / 0.190 s / 15.9 ms against the hand-picked 0.55-1.10 s band at a
    0.8 ``catch_vel_scale`` default; the band became DERIVED on 2026-08-18 and
    the default moved to 0.9.)  **That slack is 50 ms at the band
    floor since 2026-08-18** (it read 115 ms at the hand-picked 0.55 s floor);
    the floor is now DERIVED as the flight where the window reaches
    ``hand_throw_envelope.arm_window_margin_s``, contract C-HAND-3.  Runbook row H1.5
    (``slack > 0.050 s`` on every withheld line) is the bench guard on that
    margin; H1.6 reads the serial console for the refusal itself.
    """
    return (catch_lead_s(v_armed_mps) + float(prelude_s) + SAFETY_GAP_S)
