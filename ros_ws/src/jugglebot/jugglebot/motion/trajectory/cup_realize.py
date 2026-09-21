"""Stage 2+3 of the unified 7-DoF cycle: tilt schedule, then cup → platform+slider.

The unified planner (`plans/archived/unified-7dof-planner.md` § 4, Phase 1) works in
three stages. Stage 1 (``cup_cycle``) solves ONE cup-opening Cartesian trajectory
for a whole cycle. This module is stages 2 and 3 — the part that turns that
single 3-DoF cup track into the 7 channels the machine actually has:

  * :func:`tilt_schedule` — **where the cup points**, per knot. The cup axis banks
    into the *apparent* gravity ``g − a_cup`` (the tray-carrying / motorcycle-lean
    solution): a ball sitting in the cup feels the specific force ``g − a_cup``, so
    aligning the cup axis anti-parallel to that field is what keeps the ball seated
    with zero lateral force in the cup frame during the carry. Saturated at
    :data:`tilt_geometry.MAX_TILT_DEG` (12°), rate-limited, and **pinned exactly**
    to the receive tilt at the catch knot and the throw tilt (``detach_axis``) at
    release — those two are boundary conditions of the ball-frame physics, not
    preferences, so they win over both the banking objective and the rate limit.
  * :func:`decompose` — **how the machine makes that happen**: per knot, a 6-DoF
    platform pose ``[centroid_xy, z, rx, ry, 0]`` plus a slider position. The
    generalisation of ``sim/juggle_tilt.py::realize_tilted`` from one pose to a
    whole knot series, with the same height-invariant rotation-centre lever-arm
    model (:data:`tilt_geometry.CUP_TILT_CENTER_Z_MM` = 744.3 mm): a tilt swings the
    cup opening sideways and slightly down about a fixed world point, so the
    centroid is offset by ``−cup_lateral_shift`` and the slider raised by the
    vertical drop, landing the *tilted* cup opening exactly on the planned cup xyz.

**The zero-banking parity contract.** With banking disabled and a constant tilt,
:func:`decompose` must reproduce ``realize_tilted`` **bit for bit** — that is the
Phase-1 acceptance bar, and it is why this module calls
:func:`tilt_geometry.cup_axis` and :func:`tilt_geometry.cup_lever_arm_mm` rather
than ``shaping.cup_lateral_shift_mm``. The two axis forms are *mathematically*
identical but not *numerically* identical: ``shaping._cup_axis_xy`` evaluates
Rodrigues in closed form while ``cup_axis`` builds the rotation matrix through the
production IK helper, and they disagree by ~8e-17 (measured, 20k random tilts).
``realize_tilted`` uses the matrix form, so this module does too; the arithmetic
order below (``cup_z_mm − base + drop``, ``cup_xy_mm − shift``) mirrors it
statement for statement for the same reason. ``tests/motion/test_cup_realize.py``
pins max |Δ| == 0.0.

**The z pin has two chains.** ``JB_OP_DEFAULT_ACTIVE_Z_MM`` (170.0) is the
production constant and the one this module uses; ``sim/juggle_tilt.py`` and
``sim/juggle_online.py`` carry their own ``Z_ACTIVE_MM`` literals. They agree
today, and the parity test is what keeps them agreeing — do NOT "unify" them by
editing the sim literals, and do NOT hard-code 170.0 here.

Units: the cup plan is SI (m, m/s, m/s²) because the ballistics are natural
there; everything this module emits is at the plant boundary — mm, rad, and
motor rev. Pure Python + numpy + the generated ``hardware_config``; no ROS2, no
repo-root / ``controller`` / ``sim`` imports.
"""

from __future__ import annotations

import dataclasses

import numpy as np

import jugglebot.hardware_config as hw
from jugglebot.motion.trajectory import tilt_geometry

HAND_REV_PER_M = float(hw.HAND_REV_PER_M)

# ── Realisation constants ─────────────────────────────────────────────────────
#: World z (mm) of the cup opening at zero slider with the platform centroid at
#: :data:`hw.JB_OP_DEFAULT_ACTIVE_Z_MM`: ``cup_z_world = CUP_Z_BASE_MM + slider_mm``
#: at level.  Mirrors ``sim/juggle_tilt.py``'s constant of the same name (and
#: ``sim/juggle_online.py``'s), which is the parity source — the value is a
#: measured morphology offset, not a derived one, so it is pinned here and
#: cross-checked by the parity test rather than recomputed.  When z FLOATS the
#: base moves with the platform: ``base(z) = CUP_Z_BASE_MM + (z − active_z)``.
CUP_Z_BASE_MM = 659.6

#: Gravity vector (m/s², world) for the banking field.  From the generated config
#: so a change to ``GRAVITY_MPS2`` ripples here instead of drifting.
GRAVITY_VEC_MPS2 = np.array([0.0, 0.0, -float(hw.GRAVITY_MPS2)])

#: Slider position (mm, measured from the bottom of the ``GEOM_HAND_STROKE_MM``
#: travel) that corresponds to **0 motor rev** in the firmware's homed frame.
#:
#: ⚠ This is the 20 mm frame divergence documented at
#: ``sim/plant/mujoco_plant.py:130-152`` and owned by
#: ``plans/archived/hand-trajectory-generator-overhaul.md``: the firmware homes
#: downward and measures its stroke from the physical bottom (x3 = 315 mm), while
#: the sim insets its stroke by ``TEENSY_TRAJ_STROKE_MARGIN_M`` (20 mm) inside the
#: 344.75 mm travel.  The sim's own plant already resolves the two exactly this
#: way — ``_hand_prime_mm = 20 mm + HAND_STROKE_TOP_REV/gain`` — so the same
#: relation is used here, which is what makes slider 335 mm land on
#: ``JB_OP_HAND_CATCH_PRIME_REV`` (9.9594 rev) instead of somewhere arbitrary.
#: The unified planner's whole cup-height model (:data:`CUP_Z_BASE_MM`) is in the
#: sim's mm frame, so the offset cannot be dropped without moving the cup.
SLIDER_REV_ZERO_MM = float(hw.TEENSY_TRAJ_STROKE_MARGIN_M) * 1000.0

#: Backward/forward slew sweeps in :func:`tilt_schedule`.  Each ``|tilt[k] −
#: tilt[k±1]| <= rate·dt`` bound is a convex disc, so alternating the two passes is
#: an alternating projection and converges; the loop exits early the moment a full
#: sweep changes nothing, so this is only a ceiling.  It is a ceiling and not an
#: assertion because when the two pins are mutually unreachable at the rate limit
#: the constraint set is EMPTY — the sweeps then settle between the pins, the pins
#: are re-forced, and the rate limit is what gives rather than the physics.
_SLEW_SWEEPS = 16

#: Default tilt slew ceiling (rad/s).  3.0 rad/s = 172 °/s traverses the full
#: ±12° usable range in ~3 knots (70 ms) at the 25 ms grid — fast enough that the
#: banking objective is not the binding constraint during a carry, slow enough
#: that a knot-to-knot tilt step stays inside what the Rung-0 characterisation
#: showed the platform tracks cleanly.  CONFIRMED at 3.0 by the WP3 config pass
#: (2026-08-30) and deliberately NOT config-keyed: it is a property of the tilt
#: SCHEDULE's convergence (the alternating-projection sweeps below), not an
#: operator-facing envelope, and the excursion it produces is already gated by
#: ``validate_cycle``'s leg checks.  A caller with an opinion passes
#: ``RealizeConfig(tilt_rate_limit_rad_s=...)``.
TILT_RATE_LIMIT_DEFAULT_RAD_S = 3.0

#: Default tilt ACCELERATION ceiling (rad/s²) on the banking schedule's tilt
#: vector.  Rate limiting bounds tilt *velocity* and leaves tilt *acceleration*
#: unbounded — WP3 measured the as-built schedule slewing at exactly the 3.0 rad/s
#: cap while its knot-to-knot second difference reached **240 rad/s²**, which
#: ``decompose`` turns into 104k–115k mm/s² of leg acceleration against a 5000
#: mm/s² session limit.  Bounding the rate is therefore not the same as bounding
#: the machine, and this constant closes the gap.
#:
#: **Derivation from the leg-acceleration budget** (all quantities in-tree):
#:
#:   1. A tilt swings the cup opening about the fixed rotation centre
#:      :data:`tilt_geometry.CUP_TILT_CENTER_Z_MM` (744.3 mm), so the platform
#:      centroid is offset by ``arm · cup_axis_xy`` with ``arm = cup_z −
#:      744.3``.  Over the slider-reachable cup band (cup z ≈ 679.6…994.6 mm at
#:      the pinned centroid — see :data:`CUP_Z_BASE_MM` and
#:      ``GEOM_HAND_STROKE_MM``) the largest lever is ``994.6 − 744.3 =
#:      250.3 mm``.
#:   2. A leg attachment also sees the platform's own angular acceleration
#:      through its radius, ``GEOM_PLAT_RADIUS_MM = 219.075 mm``.
#:   3. The two add, so a tilt acceleration ``α`` costs at most
#:      ``L · α`` of leg acceleration with ``L = 250.3 + 219.075 = 469.4 mm``.
#:   4. The cup's own translation needs the rest of the budget, so the tilt term
#:      is allowed **half** of ``JB_TRAJ_LEG_ACC_LIMIT_MMPS2`` (a 2× reserve; the
#:      measured level-platform baseline on the WP3 demo cycle is 394 mm/s², ~8 %,
#:      leaving the factor comfortable rather than tight).
#:
#: ``α_max = 0.5 · 5000 / 469.4 = 5.33 rad/s²`` — and the WP4 sweep confirms the
#: derivation lands where the machine actually is: a banking schedule held near
#: 5–7 rad/s² validates, one at 10 rad/s² does not.  Config-derived, not a
#: literal, so a change to the leg limit or the platform radius ripples here.
#: ``<= 0`` disables the bound (the pre-WP4 behaviour).
TILT_ACCEL_BUDGET_FRACTION = 0.5
#: Cup world z (mm) at the TOP of the hand's operating band — the largest lever.
_CUP_Z_TOP_MM = (CUP_Z_BASE_MM + SLIDER_REV_ZERO_MM
                 + float(hw.JB_OP_HAND_CATCH_PRIME_REV)
                 / HAND_REV_PER_M * 1000.0)
TILT_ACCEL_LEVER_MM = (_CUP_Z_TOP_MM
                       - float(tilt_geometry.CUP_TILT_CENTER_Z_MM)
                       + float(hw.GEOM_PLAT_RADIUS_MM))
TILT_ACCEL_LIMIT_DEFAULT_RAD_S2 = (
    TILT_ACCEL_BUDGET_FRACTION * float(hw.JB_TRAJ_LEG_ACC_LIMIT_MMPS2)
    / TILT_ACCEL_LEVER_MM)

#: Default tilt JERK ceiling (rad/s³) on the banking schedule's tilt vector,
#: derived from the leg-JERK budget through the SAME lever and the SAME reserve
#: fraction as the acceleration cap above — one map, two limits.
#:
#: **Why the schedule needs a third-difference bound at all** (C-CUP-3,
#: ``plans/active/cup-contact-contract.md`` § 2).  ``_accel_bounded_schedule``
#: measured only the SECOND difference before it returned, but the refusal the
#: machine issues is ``LIMIT_JERK`` — a THIRD difference.  Exiting on one and
#: being judged on the other makes the widen loop's branch a coin flip decided by
#: a quantity nobody is gating on: MEASURED 2026-09-18 at the R3 operating point,
#: a 7 mm lateral offset produced 5.221 rad/s² against the 5.2222 cap and exited
#: on attempt 2 with **137 823 mm/s³** of leg jerk, while 8 mm produced 5.223,
#: widened once more, and came out at **49 762** — a 64 % step in the refusal
#: quantity across 1 mm of aim, and 137 k is already 92 % of the 150 k session
#: cap.  With the third difference in the exit test the loop stops when the
#: quantity that refuses is inside budget, so the achieved leg jerk is continuous
#: in the commanded offset instead of being a function of which branch fired.
#: ``<= 0`` disables the bound (the pre-C-CUP-3 behaviour).
#:
#: **Why the jerk lever is not the accel lever, and why it is not a constant.**
#: :data:`TILT_ACCEL_LEVER_MM` is a STATIC model — a tilt swings the cup opening
#: about a fixed rotation centre and a leg attachment also sees the platform's own
#: angular term, so the two add.  That is right for a second difference and wrong
#: for a third, because ``decompose`` writes ``centroid = cup_xy − arm·axis_xy``
#: with ``arm = cup_z − CUP_TILT_CENTER_Z_MM``: the lever is itself a function of
#: time, and the third difference of a PRODUCT is not the product of the third
#: difference.  The correction is the cup's own vertical motion, so it is a
#: property of the CYCLE, not of the geometry — which is why this constant is
#: only the static reference and the cap that reaches
#: :func:`_accel_bounded_schedule` is re-derived per call by
#: :func:`_tilt_jerk_lever_mm` from the cup plan in hand.  See that function for
#: the derivation, the measured validation and the honest statement of how loose
#: it is.
#:
#: The number here is therefore the cap for a cycle with NO vertical cup motion
#: (``v_z = a_z = j_z = 0``), where the discrete-Leibniz cross terms vanish and
#: the effective lever reduces exactly to the static one: the same map, the same
#: reserve fraction, applied to the leg-JERK limit instead of the leg-ACCEL one.
TILT_JERK_LIMIT_DEFAULT_RAD_S3 = (
    TILT_ACCEL_BUDGET_FRACTION * float(hw.JB_TRAJ_LEG_JERK_LIMIT_MMPS3)
    / TILT_ACCEL_LEVER_MM)

#: Minimum seating force, in units of ``g``, for the banking prescription to be
#: DEFINED at a knot (C-CUP-1).  ONE definition, from the generated config —
#: never re-typed as a literal anywhere in this module or its tests.
#:
#: A ball resting in the cup feels the specific force ``f = g − a_cup``; the
#: component that presses it INTO the cup is ``s = −f_z = g + a_cup,z`` (world z
#: up): ``g`` at rest, 0 in free fall, NEGATIVE when the cup dives faster than
#: gravity.  In the ~125 ms before a catch the cup dives at 1.2–2.8 g, so ``s <
#: 0`` and **no attitude seats a ball** — the prescription has no solution, and
#: :func:`tilt_geometry.tilt_to_receive` answers with its 12° clamp at an azimuth
#: taken from the NORMALISED lateral residual, which is scale-free.  Measured
#: 2026-09-16: ``raw_max = 12.000°`` for every lateral offset from 0.5 mm to
#: 40 mm and ``0.000°`` at exactly zero (logbook
#: ``2026-09-16-banking-saturates-on-small-lateral-offsets.md``).
#:
#: Gating on ``s >= ε·g`` makes the prescribed angle ``atan(|f_lat| / s) <=
#: atan(|f_lat| / (ε·g))``, so a milli-g lateral residual is a sub-degree tilt and
#: the demand is amplitude-aware.  The 12° clamp stays as a hard cap that a
#: DEFINED prescription never reaches.
BANKING_SEATING_MIN_G = float(hw.JB_TRAJ_CUP_BANKING_SEATING_MIN_G)

#: Largest ``|S''(u)|`` of the quintic smoothstep ``S(u) = 6u⁵ − 15u⁴ + 10u³``
#: used by the pin blend, attained at ``u = (1 ± 1/√3)/2``.  It is what turns a
#: requested tilt acceleration into a blend half-width (see
#: :func:`_accel_bounded_schedule`).
_BLEND_CURVATURE_MAX = 10.0 / np.sqrt(3.0)

#: Largest ``|S'''(u)|`` of the same quintic smoothstep, ``360u² − 360u + 60``,
#: attained at the window edges ``u = 0`` and ``u = 1``.  The jerk twin of
#: :data:`_BLEND_CURVATURE_MAX`: it turns a requested tilt JERK into a blend
#: half-width the same way (``gap·|S'''|max / L³ <= jerk_cap·dt³``).
_BLEND_JERK_MAX = 60.0

#: ``Σ|Δ³T|·(h+1)²`` for the triangular kernel ``T = box ⊛ box``.  ``Δb`` is two
#: impulses ``(δ₀ − δ_m)/m`` with ``m = h+1``, so ``Δ²T = (δ₀ − 2δ_m + δ_2m)/m²``
#: (the ``4/m²`` the accel estimate uses) and ``Δ³T = Δ(Δ²T)`` doubles the term
#: count without cancellation → ``8/m²``.  Note it falls off as ``m²``, NOT
#: ``m³``: a step in the raw series keeps a third difference of order
#: ``step/m²`` however hard it is smoothed, which is why the widen loop MEASURES
#: rather than trusting this estimate.
_SMOOTH_JERK_COEFF = 8.0

#: Bounded widen attempts in :func:`_accel_bounded_schedule`.  The analytic widths
#: are an ESTIMATE (they bound each mechanism separately and ignore the cross
#: term), so the routine measures the second difference it actually produced and
#: widens if it missed.  Bounded and not asserted for the same reason
#: :data:`_SLEW_SWEEPS` is: when the pins are too close together to join under the
#: cap the constraint set is empty and something must give.  ``validate_cycle`` is
#: the authority on what the machine will accept, not this loop.
_TILT_WIDEN_ATTEMPTS = 8

#: Bisection steps :func:`_accel_bounded_schedule` spends landing the blend width
#: ON the cap once it has bracketed it (C-CUP-3 follow-up, 2026-09-20).
#:
#: **Why a bisection and not the ladder alone.**  The widths are ``ceil``-free
#: reals now, but the ladder still only visits ``floor``, ``analytic``,
#: ``1.4·analytic``… — a discrete set — and the achieved jerk is a steep function
#: of the width (``gap·|S‴|max/L³``), so which rung fires decides the answer.
#: MEASURED 2026-09-20 at the R3 operating point: with ``ceil``ed widths the blend
#: half-width stepped 8 → 9 → 11 → 12 knots across dx = 4 → 8 → 16 → 31 mm and the
#: achieved tilt jerk sawtoothed 32.2 → 59.2 → 36.6 → 48.7 rad/s³ — a 38 % DROP in
#: the refusal quantity for a 2× LARGER commanded aim, which is the
#: discrete-branch defect C-CUP-3 exists to close.  Bisecting between the
#: narrowest admissible width (the floor) and the first passing one makes the
#: schedule sit AT the cap whenever the floor cannot, so the achieved jerk
#: SATURATES instead of jumping: non-decreasing in the aim, then flat.
#:
#: 6 steps resolves the bracket to ~1.6 % of its width, i.e. ~1.2 % in the
#: achieved jerk (it goes as ``L³``) — inside the 2 % the contract test allows
#: for solver noise, and bounded, so the solve-time cost is a constant.
_TILT_BISECT_STEPS = 6

#: Narrowest pin-blend width (knots) :func:`_accel_bounded_schedule` may use.  The
#: curvature estimate sizes ``L`` from the pin gap alone, so a SMALL gap (a launch
#: from rest aimed a few mm off its site: 1-3 mrad against the smoothed banking)
#: gets a 2-knot blend — a corner on the 25 ms grid that the acceleration check
#: passes but the leg JERK does not: at knot 1, and as a one-sided tilt-slope
#: break at the release seam that ``decompose`` turns into a centroid velocity
#: step.  MEASURED 2026-09-13 (skill-stack R3 aim-jerk investigation; logbook
#: 2026-09-13-skill-stack-r3-learner-single-site), a single-site launch at
#: 300/5000/150000: 10 / 20 / 40 mm of aim cost 123k / 125k / 140k mm/s³ (flat in
#: the aim) without a floor; with 8 knots (0.2 s) they cost 14.6k / 29.2k /
#: 58.5k, zero aim, STEADY catch-with-throw and columns peaks unchanged; 6 knots
#: is still linear, 4 leaves the knot-1 corner (40 mm: 140k).
_TILT_BLEND_MIN_KNOTS = 8.0

#: Default vertical band (mm) the platform may absorb when the slider saturates,
#: used only when ``z_float_enabled``.  From the generated config
#: (``trajectory_op.unified_z_band_mm``, 30.0 as landed 2026-08-30) rather than a
#: literal, for the reason the whole module reads ``hw``: a second spelling of a
#: YAML number is a number that drifts.
Z_BAND_DEFAULT_MM = float(hw.JB_TRAJ_UNIFIED_Z_BAND_MM)

#: Whether z floats by default (``trajectory_op.unified_z_float_enabled``, false).
#: Sourced from the config for the same reason, and false for the reason the YAML
#: states: a floating z moves SIX legs to solve a ONE-axis stroke shortfall.
Z_FLOAT_DEFAULT_ENABLED = bool(hw.JB_TRAJ_UNIFIED_Z_FLOAT_ENABLED)


@dataclasses.dataclass
class RealizeConfig:
    """Knobs for :func:`tilt_schedule` and :func:`decompose`.

    Explicit dataclass on purpose: every default is read from the generated
    ``hardware_config`` ONCE, here, so the pure planner is driven by the YAML
    without any caller having to perform the round trip — and a test (or a node
    running an experiment) still overrides any single field per instance.  The
    WP3 config keys landed 2026-08-30: ``trajectory_op.unified_z_float_enabled``
    and ``unified_z_band_mm`` are read through the two module constants above.
    """

    #: Platform centroid height (mm) the pose is pinned to when z does not float.
    active_z_mm: float = float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM)
    #: Let z absorb slider saturation.  False (default) = today's hard pin.
    z_float_enabled: bool = Z_FLOAT_DEFAULT_ENABLED
    #: Max |z − active_z| (mm) when floating.  The excursion is GATED downstream by
    #: ``validate_cycle``'s leg-workspace checks; this only bounds what is offered.
    z_band_mm: float = Z_BAND_DEFAULT_MM
    #: Bank the cup into apparent gravity.  False = the constant/legacy tilts.
    banking_enabled: bool = True
    #: From-vertical saturation for the banking tilt (deg).
    max_tilt_deg: float = tilt_geometry.MAX_TILT_DEG
    #: Tilt slew ceiling (rad/s), on the tilt VECTOR's 2-norm — never per axis;
    #: see :func:`tilt_schedule`.  <= 0 disables rate limiting.
    tilt_rate_limit_rad_s: float = TILT_RATE_LIMIT_DEFAULT_RAD_S
    #: Tilt ACCELERATION ceiling (rad/s²), also on the tilt vector's 2-norm.
    #: Applies to the BANKING schedule only — the zero-banking path is the legacy
    #: two-constant realisation whose bit-for-bit parity with ``realize_tilted`` is
    #: the Phase-1 acceptance bar, so nothing may reshape it.  ``<= 0`` disables
    #: the bound and restores the pre-WP4 (rate-limited-only) schedule.
    #: See :data:`TILT_ACCEL_LIMIT_DEFAULT_RAD_S2` for where the default comes from.
    tilt_accel_limit_rad_s2: float = TILT_ACCEL_LIMIT_DEFAULT_RAD_S2
    #: Tilt JERK ceiling (rad/s³), also on the tilt vector's 2-norm and also
    #: banking-only.  It is the second half of the widen loop's exit test — the
    #: half that measures what ``LIMIT_JERK`` refuses on.  ``<= 0`` disables it
    #: and restores the accel-only exit.  See
    #: :data:`TILT_JERK_LIMIT_DEFAULT_RAD_S3`.
    tilt_jerk_limit_rad_s3: float = TILT_JERK_LIMIT_DEFAULT_RAD_S3
    #: Minimum seating force (in ``g``) for the banking prescription to be
    #: DEFINED at a knot; below it the schedule carries the last valid attitude.
    #: See :data:`BANKING_SEATING_MIN_G` — that constant is the one definition.
    banking_seating_min_g: float = BANKING_SEATING_MIN_G
    #: Usable slider travel (mm).
    slider_stroke_mm: float = float(hw.GEOM_HAND_STROKE_MM)
    #: Cup world z (mm) at zero slider, platform at ``active_z_mm``.
    cup_z_base_mm: float = CUP_Z_BASE_MM
    #: Slider mm that maps to 0 motor rev (see :data:`SLIDER_REV_ZERO_MM`).
    slider_rev_zero_mm: float = SLIDER_REV_ZERO_MM
    #: Gravity magnitude (m/s²) for the banking field.
    gravity_mps2: float = float(hw.GRAVITY_MPS2)


@dataclasses.dataclass
class RealizedCycle:
    """One cup cycle decomposed onto the 7 channels, on the cup plan's own grid.

    Every array is knot-aligned with the source ``CupCyclePlan`` (``n`` rows for an
    ``n``-knot plan, i.e. ``n_steps + 1``).  Velocities are the knot velocities the
    piecewise-cubic reconstruction needs — see :func:`decompose` for which parts of
    them are analytic and which are finite-differenced.
    """

    pose: np.ndarray             #: (n, 6) platform pose — mm, mm, mm, rad, rad, rad
    pose_vel: np.ndarray         #: (n, 6) pose rate — mm/s, rad/s
    slider_mm: np.ndarray        #: (n,) slider position, clamped to [0, stroke]
    slider_rev: np.ndarray       #: (n,) hand motor position (ODrive absolute rev)
    slider_vel_rev_s: np.ndarray  #: (n,) hand motor rate (rev/s)
    t: np.ndarray                #: (n,) knot times (s) from cycle start
    dt: float                    #: knot spacing (s)
    catch_k: int                 #: knot index of the catch
    tilts: np.ndarray            #: (n, 2) the (rx, ry) schedule that was realised
    slider_saturated: np.ndarray  #: (n,) bool — the stroke clamp bit at this knot
    z_excursion_mm: np.ndarray   #: (n,) signed z − active_z (all zero when pinned)


# ── Tilt schedule ─────────────────────────────────────────────────────────────

def _project_to_disc(points: np.ndarray, centre: np.ndarray,
                     radii) -> np.ndarray:
    """Radially project each ``(rx, ry)`` row onto the disc ``|p − centre| <= r``.

    Rows already inside their disc are returned **bit-identically** (not
    recomputed as ``centre + (p − centre)``, which is not an identity in floating
    point).  That is what lets the zero-banking path hand ``decompose`` the exact
    constant tilt the ``realize_tilted`` parity contract requires.
    """
    d = points - centre
    mag = np.hypot(d[:, 0], d[:, 1])
    r = np.broadcast_to(np.asarray(radii, dtype=float), mag.shape)
    over = mag > r
    if not bool(np.any(over)):
        return points
    scale = np.divide(r, mag, out=np.ones_like(mag), where=(mag > 0.0))
    return np.where(over[:, None], centre + d * scale[:, None], points)


def _slew_toward(tilts: np.ndarray, k: int, ref: int, step: float) -> bool:
    """Pull knot ``k`` to within ``step`` of knot ``ref`` (2-norm).  True if moved.

    The bound is on the tilt VECTOR, not per axis: the usable ceiling is a
    from-vertical angle, and a per-axis slew bound does not bound an angle.
    """
    d = tilts[k] - tilts[ref]
    mag = float(np.hypot(d[0], d[1]))
    if mag <= step:
        return False
    tilts[k] = tilts[ref] + d * (step / mag)
    return True


def _smoothstep(u: np.ndarray) -> np.ndarray:
    """Quintic smoothstep ``6u⁵ − 15u⁴ + 10u³`` on ``[0, 1]``, clamped outside.

    Chosen over a raised cosine because ``S'(0) = S'(1) = S''(0) = S''(1) = 0``:
    the blend window below joins the smoothed banking series at BOTH ends with
    matching value, slope and curvature, so the window's own edges add nothing to
    the second difference this routine is trying to bound.
    """
    u = np.clip(u, 0.0, 1.0)
    return u * u * u * (10.0 + u * (-15.0 + 6.0 * u))


def _triangular_smooth(series: np.ndarray, half: int) -> np.ndarray:
    """Edge-replicated triangular (box ⊛ box) moving average, half-width ``half``.

    A moving average is a **convex combination** of its inputs, so a series that
    lies inside the tilt-cap disc stays inside it — the cap is preserved by
    convexity here exactly as it is across the slew sweeps, rather than by a
    re-clamp that would reintroduce the corner this routine exists to remove.

    Its second difference is what the accel bound needs: ``Δ²(T ⊛ x)`` is a
    four-term combination of ``x`` with coefficients ``±1/(half+1)²``, so
    ``|Δ²| <= 4·max|x|/(half+1)²`` — the inverse of that is the width estimate in
    :func:`_accel_bounded_schedule`.
    """
    if half <= 0:
        return series.astype(float, copy=True)
    box = np.full(half + 1, 1.0 / (half + 1.0))
    ker = np.convolve(box, box)
    pad = (ker.shape[0] - 1) // 2
    padded = np.vstack([np.repeat(series[:1], pad, axis=0), series,
                        np.repeat(series[-1:], pad, axis=0)])
    return np.column_stack([np.convolve(padded[:, a], ker, mode='valid')
                            for a in range(series.shape[1])])


def _max_tilt_accel(tilts: np.ndarray, dt: float) -> float:
    """Peak 2-norm of the tilt vector's second difference, as rad/s²."""
    if tilts.shape[0] < 3:
        return 0.0
    d2 = tilts[:-2] - 2.0 * tilts[1:-1] + tilts[2:]
    return float(np.hypot(d2[:, 0], d2[:, 1]).max()) / (dt * dt)


def _max_tilt_jerk(tilts: np.ndarray, dt: float) -> float:
    """Peak 2-norm of the tilt vector's THIRD difference, as rad/s³.

    The twin of :func:`_max_tilt_accel`, and the quantity ``validate_cycle``
    refuses on (``LIMIT_JERK``) once ``decompose`` has put the schedule through
    the ``CUP_TILT_CENTER_Z_MM`` lever.  2-norm of the VECTOR, never per axis,
    for the reason every other bound in this module is: the ceiling is a
    from-vertical angle and per-axis bounds do not bound an angle.
    """
    if tilts.shape[0] < 4:
        return 0.0
    d3 = -tilts[:-3] + 3.0 * tilts[1:-2] - 3.0 * tilts[2:-1] + tilts[3:]
    return float(np.hypot(d3[:, 0], d3[:, 1]).max()) / (dt * dt * dt)


def _tilt_jerk_lever_mm(cup_plan, dt: float) -> float:
    """Effective lever ``Λ`` (mm of leg jerk per rad/s³ of tilt jerk) for THIS plan.

    **The quantity.**  ``decompose`` writes ``centroid_xy = cup_xy − arm·axis_xy``
    with ``arm = cup_z − CUP_TILT_CENTER_Z_MM`` and ``axis_xy`` the cup up-axis'
    horizontal part, and a leg attachment additionally rides the platform's own
    rotation at radius ``GEOM_PLAT_RADIUS_MM``.  The gate refuses on the THIRD
    difference of the leg extensions, so the quantity to bound is the third
    difference of that product — and the third difference of a product is not the
    product of the third difference.

    **The derivation** (discrete Leibniz, on the same knot grid and the same
    operator :func:`_max_tilt_jerk` and ``validate_cycle`` use)::

        Δ³(arm·axis)_k = arm·Δ³axis + 3·Δarm·Δ²axis + 3·Δ²arm·Δaxis + Δ³arm·axis

    Divide through by ``dt³`` and the three cross terms are exactly the cup's own
    vertical speed, acceleration and jerk::

        |leg jerk from tilt|  <=  (|arm| + R_plat)·|θ‴|          direct
                               +  3·|v_z|·|θ″|                   lever rate
                               +  3·|a_z|·|θ′|                   lever accel
                               +  |j_z|·|θ|                      lever jerk

    (``|axis_xy| <= sin θ <= θ`` and each of its differences is bounded by the
    same difference of the tilt vector to first order in θ — 12° costs < 1 % on
    that, and the whole model is the same first-order one
    :data:`TILT_ACCEL_LEVER_MM` already uses for the second difference.)

    **What this function is, and what it is NOT.**  It is the ESTIMATE that sizes
    the widen loop's analytic widths — the knot-scale closure of the three cross
    factors, ``Δ^(3−m)θ ≈ Δ³θ · (W·dt)^m`` at ``W = 1`` knot::

        Λ = max_k [ (|arm_k| + R_plat) + 3|v_z,k|·dt + 3|a_z,k|·dt² + |j_z,k|·dt³ ]

    It is **not** a bound, and no closed form in ``(v_z, a_z, j_z, dt)`` can be
    one: which knot's term dominates is a property of the cycle.  MEASURED
    2026-09-20 against the exact per-knot composite
    (:func:`_tilt_leg_jerk_mmps3`) on the schedules actually produced:

    * R3 catch-with-throw, dx = 0.5…31 mm, 16 window-fixtures — ``Λ`` is an upper
      bound on all of them, loose by 1.15× at the tightest and 2.4× at the
      loosest; the composite there is carried by the ``arm·Δ³θ`` and platform-
      rotation terms, which the closure gets right.
    * the ``test_unified_cycle`` ring's 2.0 s STEADY window — ``Λ`` is **6.3×
      too SMALL**.  Its composite is carried by ``|j_z|·|θ|``: a 2.1 × 10⁶ mm/s³
      release-stroke vertical jerk against a *standing* 0.63° attitude that has
      nothing to do with the schedule's third difference, so the ``dt³`` closure
      under-reads that term by ~70×.

    Closing the same three factors with the caps already in force (``θ″ <=`` the
    accel cap, ``θ′ <= 3.0 rad/s``, ``θ <= 12°``) is rigorous and useless in the
    other direction: those products come to ~560 k mm/s³ against a 75 k reserve,
    i.e. they drive the cap negative, because the rate cap is never approached
    (these schedules run at 0.03–0.17 rad/s).

    So the guarantee is the MEASUREMENT: :func:`_accel_bounded_schedule` computes
    the composite on the series it just produced and widens until it is inside
    the reserved budget.  This estimate only decides how many attempts that
    costs.  Both replace a fitted 2.1× constant (2026-09-18) that was tuned on
    five fixtures at one cycle speed and sat elsewhere on every other.

    **Degenerate / duck-typed plans.**  ``tilt_schedule`` contractually needs only
    ``acc``, ``dt`` and ``catch_k``; a plan without ``pos``/``vel``/``jerk`` gets
    the shipped static lever :data:`TILT_ACCEL_LEVER_MM`, i.e. the cross terms are
    treated as zero — the same answer a cycle with no vertical cup motion gets.
    """
    r_plat = float(hw.GEOM_PLAT_RADIUS_MM)
    pos = getattr(cup_plan, 'pos', None)
    if pos is None:
        return TILT_ACCEL_LEVER_MM
    pos = np.asarray(pos, dtype=float)
    if pos.ndim != 2 or pos.shape[1] != 3 or pos.shape[0] < 1:
        return TILT_ACCEL_LEVER_MM
    n = int(pos.shape[0])
    lever = np.abs(pos[:, 2] * 1000.0
                   - float(tilt_geometry.CUP_TILT_CENTER_Z_MM)) + r_plat

    def _z_column(name, power):
        """|d^k(cup z)/dt^k| in mm, knot-aligned and edge-padded, times dt^k."""
        arr = getattr(cup_plan, name, None)
        if arr is None:
            return 0.0
        arr = np.asarray(arr, dtype=float)
        if arr.ndim != 2 or arr.shape[1] != 3 or arr.shape[0] < 1:
            return 0.0
        col = np.abs(arr[:, 2]) * 1000.0 * (dt ** power)
        if col.shape[0] >= n:
            return col[:n]
        # ``jerk`` is (n_steps, 3) by the CupCyclePlan contract — one row short.
        return np.concatenate([col, np.repeat(col[-1:], n - col.shape[0])])

    lever = (lever + 3.0 * _z_column('vel', 1) + 3.0 * _z_column('acc', 2)
             + _z_column('jerk', 3))
    return float(max(float(np.max(lever)), r_plat))


def _tilt_jerk_terms(cup_plan, dt: float, n: int):
    """The four knot-aligned coefficients of :func:`_tilt_leg_jerk_mmps3`.

    ``(|arm|, 3|v_z|·dt, 3|a_z|·dt², |j_z|·dt³)`` in mm, each an ``(n,)`` array —
    the cup plan's own vertical state at every knot, pre-multiplied by the ``dt``
    powers that make the composite a pure sum of tilt DIFFERENCES.  Returns
    ``None`` when the plan does not carry ``pos`` (the duck-typed minimum), which
    switches the composite test off and leaves the rad/s³ cap alone.
    """
    pos = getattr(cup_plan, 'pos', None)
    if pos is None:
        return None
    pos = np.asarray(pos, dtype=float)
    if pos.ndim != 2 or pos.shape[1] != 3 or pos.shape[0] < n:
        return None
    arm = np.abs(pos[:n, 2] * 1000.0 - float(tilt_geometry.CUP_TILT_CENTER_Z_MM))

    def _col(name, power, coeff):
        arr = getattr(cup_plan, name, None)
        if arr is None:
            return np.zeros(n)
        arr = np.asarray(arr, dtype=float)
        if arr.ndim != 2 or arr.shape[1] != 3 or arr.shape[0] < 1:
            return np.zeros(n)
        col = coeff * np.abs(arr[:, 2]) * 1000.0 * (dt ** power)
        if col.shape[0] >= n:
            return col[:n]
        # ``jerk`` is (n_steps, 3) by the CupCyclePlan contract — one row short.
        return np.concatenate([col, np.repeat(col[-1:], n - col.shape[0])])

    return (arm, _col('vel', 1, 3.0), _col('acc', 2, 3.0), _col('jerk', 3, 1.0))


def _tilt_leg_jerk_mmps3(tilts: np.ndarray, dt: float, terms) -> float:
    """The tilt channel's own contribution to peak leg jerk (mm/s³), measured.

    This is the derivation in :func:`_tilt_jerk_lever_mm` evaluated on the
    schedule in hand instead of closed with an assumed feature width — the
    discrete Leibniz expansion of ``Δ³(arm·axis_xy)``, plus the platform's own
    angular term at ``GEOM_PLAT_RADIUS_MM``, maximised over knots::

        max_k [ |arm_k|·|Δ³θ_k| + 3|v_z,k|dt·|Δ²θ_(k+1)|
                + 3|a_z,k|dt²·|Δθ_(k+2)| + |j_z,k|dt³·|θ_(k+3)| ] / dt³
        + R_plat · max_k|Δ³θ_k| / dt³

    with the index shifts the discrete product rule prescribes, so each cross
    term is read where it actually coincides with the lever's own difference.

    **Why the tilt vector stands in for the cup axis.**  ``|axis_xy| = sin θ <=
    θ`` and the map ``(rx, ry) → axis_xy`` has unit Jacobian to first order, so
    every difference of the axis series is bounded by the same difference of the
    tilt series up to ``O(θ²)`` — under 1 % at the 12° ceiling, and it keeps this
    to vectorised numpy on the series the routine already holds instead of ``n``
    rotation-matrix builds per widen attempt.

    At ``v_z = a_z = j_z = 0`` the three cross terms vanish and this reduces
    exactly to ``(|arm| + R_plat)·|Δ³θ|/dt³`` — the static lever
    :data:`TILT_ACCEL_LEVER_MM` applies to the second difference, which is why
    the budget the caller compares against is that same lever times the rad/s³
    cap.
    """
    n = int(tilts.shape[0])
    if terms is None or n < 4:
        return 0.0
    arm, vz, az, jz = terms
    d1 = np.abs(np.diff(tilts, n=1, axis=0))
    d2 = np.abs(np.diff(tilts, n=2, axis=0))
    d3 = np.abs(np.diff(tilts, n=3, axis=0))
    m = n - 3
    n1 = np.hypot(d1[:, 0], d1[:, 1])
    n2 = np.hypot(d2[:, 0], d2[:, 1])
    n3 = np.hypot(d3[:, 0], d3[:, 1])
    val = np.hypot(tilts[:, 0], tilts[:, 1])
    per_knot = (arm[:m] * n3[:m] + vz[:m] * n2[1:m + 1]
                + az[:m] * n1[2:m + 2] + jz[:m] * val[3:m + 3])
    return float(per_knot.max() + float(hw.GEOM_PLAT_RADIUS_MM) * n3.max()) \
        / (dt * dt * dt)


def _banking_raw(acc: np.ndarray, cfg, start=None) -> np.ndarray:
    """The C-CUP-1 banking prescription: defined only under seating force.

    Per knot, the seating force is ``s = g + a_cup,z`` — the component of the
    apparent gravity ``f = g − a_cup`` that presses a ball INTO the cup (``g`` at
    rest, 0 in free fall, negative in a faster-than-g dive).  Where ``s >= ε·g``
    the attitude is :func:`tilt_geometry.tilt_to_receive(g − a_cup)`, unchanged
    in its geometry.  Where it is not, the field has no seating component, NO
    attitude seats a ball, and the prescription **has no solution** — so the
    schedule carries the last valid attitude (a hold, smoothed afterwards by the
    existing blend) instead of calling ``tilt_to_receive`` with an unseatable
    field and receiving its 12° clamp at a scale-free azimuth.  See
    :data:`BANKING_SEATING_MIN_G` for the measurement that made this necessary.

    **The no-predecessor case** — an undefined knot with no valid knot before it,
    which is knot 0 of every post-release window (the cup is in free fall there,
    ``s = 0``).  In precedence order:

    1. ``start``, the seam pin, when the caller supplied one.  It IS the last
       valid attitude: the attitude the PRECEDING window ended at
       (``unified_cycle._start_tilt_for``), so using it is this same carry rule
       applied across the seam rather than a second convention.  It also makes
       the seam free — ``raw[0]`` then equals the knot-0 anchor, so the pin blend
       closes a gap of exactly zero there and the flat prefix leaves the first
       difference at knot 0 at zero, i.e. no centroid step through the 744.3 mm
       ``CUP_TILT_CENTER_Z_MM`` lever.  Back-filling instead would seat knot 0 on
       a value disagreeing with the seam by up to the whole throw tilt (12° ≈
       155 mm of centroid x) and hand the blend that gap AT the seam — the
       2026-09-06 ``STALE_STATE`` / ``LIMIT_JERK`` failure ``_start_tilt_for``
       exists to prevent.
    2. Otherwise back-fill from the FIRST valid knot (a constant prefix).  That is
       continuous at the resume knot by construction (Δ¹ = 0 there); level would
       manufacture a step equal to the whole first prescription at exactly the
       knot where the prescription is worst conditioned (``s ≈ ε·g``).
    3. Otherwise level — no knot in the window is ever seated, so there is nothing
       to carry, level is the neutral attitude, the pins still carry the boundary
       conditions, and the ``a_cup = 0`` degenerate case is unchanged.
    """
    n = int(acc.shape[0])
    g = float(cfg.gravity_mps2)
    g_vec = np.array([0.0, 0.0, -g])
    floor = float(cfg.banking_seating_min_g) * g
    raw = np.zeros((n, 2), dtype=float)
    seated = (g + acc[:, 2]) >= floor
    carry = None if start is None else np.asarray(start, dtype=float)
    first_valid = -1
    for k in range(n):
        if seated[k]:
            raw[k] = tilt_geometry.tilt_to_receive(g_vec - acc[k],
                                                   max_tilt_deg=cfg.max_tilt_deg)
            if first_valid < 0:
                first_valid = k
            carry = raw[k]
        elif carry is not None:
            raw[k] = carry
    if start is None and first_valid > 0:
        raw[:first_valid] = raw[first_valid]
    return raw


def _rate_limit_sweeps(tilts: np.ndarray, anchor_set, step: float) -> np.ndarray:
    """Impose ``|tilt[k] − tilt[k±1]| <= step`` by alternating backward/forward sweeps.

    The BACKWARD pass is the *lookahead* — it is what makes a knot approach the pin
    ahead of it instead of the schedule jumping to the pin at the last moment; the
    FORWARD pass bounds each knot against its predecessor; the pair is run to
    convergence.  Neither pass ever touches a pin, so the pins stay exact no matter
    how the sweeps land, and every move is toward a point already inside the tilt
    cap, so the cap survives too.  Each bound is a convex disc, so alternating the
    two passes is an alternating projection and converges; the loop exits early the
    moment a full sweep changes nothing.  When the pins are too far apart to join at
    the rate limit there is nothing to converge to — the schedule settles between
    them and the rate limit is what gives.
    """
    n = int(tilts.shape[0])
    if step <= 0.0 or n < 2:
        return tilts
    for _ in range(_SLEW_SWEEPS):
        moved = False
        for k in range(n - 2, -1, -1):
            if k not in anchor_set:
                moved = _slew_toward(tilts, k, k + 1, step) or moved
        for k in range(1, n):
            if k not in anchor_set:
                moved = _slew_toward(tilts, k, k - 1, step) or moved
        if not moved:
            break
    return tilts


def _accel_bounded_schedule(raw: np.ndarray, anchors, dt: float,
                            accel_cap: float, tilt_cap: float,
                            rate_step: float, jerk_cap: float = 0.0,
                            jerk_terms=None,
                            jerk_budget_mm: float = 0.0) -> np.ndarray:
    """Smooth ``raw`` until its tilt acceleration AND jerk are in budget, pins exact.

    **Why a smoother and not an accel-limited slew.**  The obvious construction —
    keep the alternating-projection architecture of the rate sweeps and add a
    projection onto each second-difference disc — was prototyped and measured
    (WP4, 2026-09-01): it is an alternating projection over ``n`` heavily
    overlapping triples and it converges **linearly and far too slowly to bound**.
    On the WP3 demo cycle, asking for 5.33 rad/s² left the schedule at 38.9 rad/s²
    after 16 sweeps, 8.8 after 200 and still 5.5 after **800** — an unbounded
    iteration count for a bound that would then only hold approximately. A bounded
    sweep count that silently returns a schedule violating its own cap is exactly
    the silent-wrongness shape the sibling ``cup_cycle`` module refuses, so it was
    rejected. This construction is O(n) per attempt, and the second difference it
    produces is *measured* before it is returned.

    **The construction, and why each piece is the piece.**

    1. ``sm = triangular_smooth(raw, half)`` — the banking objective is a
       *direction*, and near release the apparent-gravity field ``g − a_cup``
       passes through ZERO (free fall is weightlessness), so ``raw``'s direction
       is genuinely ill-conditioned there and slews at up to 8.4 rad/s with 633
       rad/s² of second difference. The smoother is what bounds that, and it keeps
       the tilt inside its cap by convexity (see :func:`_triangular_smooth`).
    2. A **blend**, not an additive correction, carries the exact pins::

           out = (1 − W_c − W_t)·sm + W_c·recv + W_t·throw

       With ``W_c``/``W_t`` non-negative, summing to ``<= 1``, and each equal to 1
       at its own pin and 0 at the other's, every knot is a convex combination of
       three points already inside the cap disc — so the pins are exact AND the cap
       survives, both by construction rather than by a re-clamp. ``W`` is the
       quintic smoothstep, so the windows contribute no curvature of their own at
       their edges, and its ``S(u) + S(1 − u) = 1`` symmetry is what lets the two
       windows overlap without breaking the sum bound (see the width comment
       below — insisting on disjoint supports instead costs a factor of 4 in
       curvature and is the difference between holding the cap and missing it).
    3. The widths start from the analytic inverses of the two mechanisms —
       ``4·tilt_cap/(half+1)² <= a·dt²`` for the smoother and
       ``|pin − sm|·S''max/L² <= a·dt²`` for the blend — then a bounded
       measure-and-widen loop closes the cross term the two estimates ignore.
       **Each mechanism carries a jerk twin** (C-CUP-3): ``8·amp/(half+1)² <=
       j·dt³`` and ``|pin − sm|·S'''max/L³ <= j·dt³``, and the width used is the
       larger of the two requests.  The smoother's jerk term is sized off the
       amplitude of the series ACTUALLY passed in rather than off ``tilt_cap``,
       because with C-CUP-1 in force the raw banking series is sub-degree while
       ``tilt_cap`` is 12°, and a ``tilt_cap``-sized jerk estimate would ask for
       ``half = 25`` on a 34-knot window — a kernel wider than the window, which
       is "hold level" wearing a smoother's clothes.  The measure-and-widen loop
       is the guarantee; the estimates only decide how many attempts it costs.
    4. **The exit test measures BOTH differences.**  Exiting on the second while
       the machine refuses on the third is how a 1 mm change of aim moved the
       achieved leg jerk by 64 % (see :data:`TILT_JERK_LIMIT_DEFAULT_RAD_S3`).
       **And the search is floor-first, then a bisection onto the cap**, so the
       width is a continuous, saturating function of the pin gap rather than
       whichever rung of a ×1.4 ladder happened to fire: widths are reals (no
       ``ceil``), the narrowest admissible blend is probed first, and when it
       misses the cap the routine bisects between it and the first passing width.
       Below the cap the achieved jerk therefore RISES with the commanded aim and
       above it it is FLAT at the cap — see :data:`_TILT_BISECT_STEPS` for the
       measurement that made this necessary.
    5. **The rate sweeps run INSIDE the loop**, so the second difference that is
       measured is the one the caller will actually receive. Running them
       afterwards would let the rate clip re-corner a schedule this routine had
       just certified — a guarantee measured on a series nobody returns is not a
       guarantee. In the normal case the smoothed schedule is already well inside
       the rate cap and the sweeps cost one no-op pass.

    Returns the last attempt whichever way the loop exits; when the pins are too
    close together for any width to bound the join, the cap is what gives and
    ``validate_cycle`` is what refuses. That is the same honesty the rate limit
    already carries (see :data:`_SLEW_SWEEPS`).
    """
    n = int(raw.shape[0])
    anchor_set = {j for j, _ in anchors}
    if n < 3 or accel_cap <= 0.0:
        out = raw.astype(float, copy=True)
        for j, v in anchors:
            out[j] = v
        return _rate_limit_sweeps(out, anchor_set, rate_step)

    budget = accel_cap * dt * dt                      # allowed |Δ²tilt| per knot²
    # allowed |Δ³tilt| per knot³ (0.0 when the jerk bound is disabled)
    budget_j = max(jerk_cap, 0.0) * dt * dt * dt
    # Mechanism 1: the smoother's own second difference, |Δ²| <= 4·max|x|/(h+1)².
    half = int(np.ceil(2.0 * np.sqrt(max(tilt_cap, 1e-12) / budget))) - 1
    if budget_j > 0.0:
        # ...and its third, |Δ³| <= 8·max|x|/(h+1)², sized off the series in hand
        # (see the docstring: tilt_cap here would out-smooth the window).
        amp = float(np.hypot(raw[:, 0], raw[:, 1]).max())
        half = max(half, int(np.ceil(
            np.sqrt(_SMOOTH_JERK_COEFF * max(amp, 1e-12) / budget_j))) - 1)

    idx = np.arange(n, dtype=float)
    # The blend weights must sum to <= 1 — that is what makes each knot a convex
    # combination, and therefore what preserves BOTH the cap and the pins.  Window
    # ``j`` reaches ``|k − j| < L``.  For two anchors ``a < b`` the sum at
    # ``k = a + x·L`` is ``S(1 − x) + S(x − (s − 1))`` with ``s = (b − a)/L``, and
    # because ``S`` is increasing with ``S(u) + S(1 − u) = 1``, that is ``<= 1``
    # for every ``s >= 1``.  So the windows may OVERLAP freely as long as
    # ``L <= b − a``, and at exactly that width each is still zero at the other's
    # pin.  (Demanding disjoint supports instead — ``L <= (b − a)/2`` — halves the
    # width and therefore QUADRUPLES the blend's curvature; measured on the WP4
    # fixtures that alone was the difference between holding the cap and missing
    # it by 2.5×.)  Only gaps BETWEEN anchors constrain L: a window running off
    # either end of the array simply truncates, with no other anchor out there.
    order = sorted(anchor_set)
    inner = [float(b - a) for a, b in zip(order, order[1:])]
    blend_max = max(1.0, float(min(inner)) if inner else float(n))
    half_max = max(0, n // 2)

    smoothed = {}

    def _attempt(h: int, scale: float):
        """One shaped schedule at kernel half-width ``h`` and blend scale ``scale``.

        Returns ``(out, ok, L)``.  ``scale`` multiplies the analytic blend width,
        and ``scale = 0`` means "the floor" — the narrowest admissible blend.
        """
        sm = smoothed.get(h)
        if sm is None:
            sm = _triangular_smooth(raw, h)
            smoothed[h] = sm
        # Mechanism 2: the blend window's curvature, ``gap·S''max/L² <= budget``.
        # The gap is measured against the SMOOTHED series, not the raw one, because
        # that is the distance the blend actually has to close — and it GROWS with
        # ``h`` (a heavier smoother pulls the schedule further from a pin it is not
        # allowed to move). Sizing L off ``raw`` instead reads the gap at ``h = 0``
        # and under-widens by up to 2× on exactly the cases that need it most.
        pin_gap = max((float(np.hypot(*(np.asarray(v, dtype=float) - sm[j])))
                       for j, v in anchors), default=0.0)
        # NO ``ceil`` on either request: ``L`` is a divisor inside the smoothstep,
        # not an index, so it may be fractional — and rounding it up was the
        # quantiser that made the achieved jerk jump between branches (see
        # :data:`_TILT_BISECT_STEPS`).  ``scale`` is what the search below turns:
        # the estimate bounds the window's OWN curvature and ignores the cross
        # term ``2·W'·sm'``, which is real wherever the smoothed series is still
        # moving under the window.
        want = np.sqrt(pin_gap * _BLEND_CURVATURE_MAX / budget)
        if budget_j > 0.0:
            want = max(want, (pin_gap * _BLEND_JERK_MAX / budget_j) ** (1.0 / 3.0))
        L = float(min(max(scale * want, _TILT_BLEND_MIN_KNOTS), blend_max))
        weight = np.zeros(n)
        out = sm.astype(float, copy=True)
        for j, v in anchors:
            w = _smoothstep(1.0 - np.abs(idx - j) / L)
            out += w[:, None] * (np.asarray(v, dtype=float)[None, :] - sm)
            weight += w
        # Belt and braces: with L <= b − a the overlapping sum is exactly
        # S(x) + S(1−x) = 1 at worst, so this never fires — but a caller-supplied
        # anchor set is data, and a weight above 1 would leave the convex hull
        # (and the cap with it).
        if float(weight.max()) > 1.0 + 1e-9:       # pragma: no cover - unreachable
            raise AssertionError("tilt blend windows overlap: %.6f" % weight.max())
        for j, v in anchors:
            out[j] = v
        out = _rate_limit_sweeps(out, anchor_set, rate_step)
        # The score is the worst of the three constraints as a RATIO of its
        # bound, so <= 1 is "inside" and, when nothing is inside, the smallest
        # score is the least-bad schedule — see the docstring's closing note.
        # The third constraint is the tilt channel's own contribution to LEG
        # jerk, measured on the series just produced (the guarantee; the rad/s³
        # cap and the analytic widths are only the estimate — see
        # :func:`_tilt_jerk_lever_mm`).
        score = _max_tilt_accel(out, dt) / accel_cap
        if budget_j > 0.0:
            score = max(score, _max_tilt_jerk(out, dt) / jerk_cap)
        if jerk_budget_mm > 0.0 and jerk_terms is not None:
            score = max(score, _tilt_leg_jerk_mmps3(out, dt, jerk_terms)
                        / jerk_budget_mm)
        return out, bool(score <= 1.0), L, float(score)

    # The FLOOR first.  The analytic widths bound each mechanism separately and
    # ignore the cross term, so they are an over-estimate by a factor that varies
    # with the pin gap — accepting them makes the achieved jerk a function of how
    # conservative the estimate happened to be at this aim rather than of the cap.
    # Probing the narrowest admissible blend first costs nothing when it is also
    # the analytic width (every small-aim case: ``want`` is below the floor), and
    # it is what turns the search into a SATURATION: below the cap the floor's own
    # schedule is returned and the achieved jerk rises with the aim; above it, the
    # bisection lands on the cap and it stops rising.
    h = int(min(max(half, 0), half_max))
    out, ok, L, score = _attempt(h, 0.0)
    if ok:
        return out
    best_fail, best_score = out, score
    lo_scale, lo_h = 0.0, h              # widest KNOWN-FAILING …
    boost = 1.0
    for _ in range(_TILT_WIDEN_ATTEMPTS - 1):
        out, ok, L, score = _attempt(h, boost)
        if score < best_score:
            best_fail, best_score = out, score
        if ok:
            if lo_h != h:
                # The kernel widened between the failing and the passing attempt,
                # so a scale bisection would interpolate the wrong axis: ``half``
                # is an integer kernel size and cannot take a fractional value.
                # Take the passing schedule.  This is the one branch where the
                # width — and so the achieved jerk — is still quantised; it is
                # reached only when the SMOOTHER, not the blend, is the binding
                # mechanism (the measured R3 grid never reaches it).
                return out
            # … and bisect down to it, so the schedule sits AT the cap.
            hi_scale, best = boost, out
            for _ in range(_TILT_BISECT_STEPS):
                mid = 0.5 * (lo_scale + hi_scale)
                mid_out, mid_ok, _L, _s = _attempt(h, mid)
                if mid_ok:
                    hi_scale, best = mid, mid_out
                else:
                    lo_scale = mid
            return best
        if h >= half_max and L >= blend_max:
            break
        lo_scale, lo_h = boost, h
        half = max(h + 1, int(np.ceil(h * 1.4)))
        h = int(min(max(half, 0), half_max))
        boost *= 1.4
    # Nothing satisfied the constraints.  Make sure the WIDEST admissible
    # schedule is among the ones tried — the ladder is a bounded geometric walk
    # from the analytic estimate, so where the constraint set is empty the rung
    # it happens to stop on is an artefact of where it started — and then return
    # the attempt that violates least.  ``validate_cycle`` is still the authority
    # on whether that is good enough; this only stops the routine from handing
    # back a needlessly rough schedule when it already knows it is over budget.
    if h < half_max or L < blend_max:
        out, ok, L, score = _attempt(half_max, float(n))
        if ok:
            return out
        if score < best_score:
            best_fail, best_score = out, score
    return best_fail


def _as_tilt_pair(value, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.shape != (2,):
        raise ValueError(f"{name} must be an (rx, ry) pair, got shape "
                         f"{np.shape(value)}")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must be finite, got {arr.tolist()}")
    return arr


def tilt_schedule(cup_plan, receive_tilt, throw_tilt, cfg=None, *,
                  start_tilt=None) -> np.ndarray:
    """Per-knot cup tilt ``(rx, ry)`` for ``cup_plan``.  Returns an ``(n, 2)`` array.

    **Banking (``cfg.banking_enabled``, the default).**  A ball resting in the cup
    feels the specific force ``g − a_cup`` — real gravity plus the pseudo-force of
    the cup's own acceleration.  Point the cup's up-axis anti-parallel to that
    field and the ball is pressed straight down the cup axis with zero lateral
    component, exactly the invariant :func:`tilt_geometry.tilt_to_receive` already
    encodes for the *arrival velocity* at a catch.  So the banking tilt is simply
    ``tilt_to_receive(g − a_cup)``: at ``a_cup = 0`` it returns level, and it
    inherits that function's ``max_tilt_deg`` saturation and its degenerate-input
    guards for free.  (The plan's prose says "the cup axis tracks
    ``normalize(g − a_cup)``" — that vector is apparent *down*; the cup axis is
    apparent *up*, which is what ``tilt_to_receive`` produces.)

    **...but only where that field can seat a ball (C-CUP-1).**  The prescription
    is evaluated ONLY at knots whose seating force ``s = g + a_cup,z`` reaches
    ``cfg.banking_seating_min_g · g``; elsewhere the schedule carries the last
    valid attitude.  Without the gate the pre-catch dive (1.2–2.8 g) and the
    post-release free fall hand ``tilt_to_receive`` a field with no seating
    component at all, and it answers with its 12° clamp at an azimuth read off
    the NORMALISED lateral residual — full-scale tilt for a 0.4 milli-g residual
    and for a 60 milli-g one alike.  :func:`_banking_raw` carries the rule, the
    measurement behind it, and the no-predecessor case.

    **Zero banking.**  No apparent-gravity content at all: hold ``receive_tilt``
    through the catch knot and ``throw_tilt`` after it.  When the two are equal the
    result is exactly that constant at every knot — the legacy single-tilt
    realisation, and the input the ``realize_tilted`` parity test drives.

    **Pins beat the rate limit.**  The catch knot gets ``receive_tilt`` and the
    final (release) knot gets ``throw_tilt``, exactly, always — they are set before
    the slew passes and no pass ever touches a pinned knot.  The rate limit is then
    imposed by alternating backward and forward sweeps (:data:`_SLEW_SWEEPS`): the
    backward sweep is the *lookahead* that makes the schedule bend toward a pin
    over several knots instead of jumping to it at the last one, the forward sweep
    bounds each knot against its predecessor, and the pair is run to convergence.
    When the pins are too far apart to join at the rate limit there is nothing to
    converge to; the schedule settles between them and the rate limit is what
    gives.

    **The rate limit does not bound the machine; the accel limit does.**  A
    rate-limited schedule can slew at exactly its cap and then STOP within one
    knot, and WP3 measured precisely that: 3.0 rad/s of tilt velocity arriving
    with **240 rad/s²** of tilt acceleration, which ``decompose`` turns into
    104k–115k mm/s² of leg acceleration against a 5000 mm/s² session limit.  So
    the banking schedule is additionally bounded in its second difference by
    ``cfg.tilt_accel_limit_rad_s2`` (default derived from the leg-acceleration
    budget — see :data:`TILT_ACCEL_LIMIT_DEFAULT_RAD_S2`), which makes it C1 in
    the tilt vector.  :func:`_accel_bounded_schedule` carries the construction and
    why it is a smoother rather than another projection sweep.

    **Zero banking is untouched by that bound, deliberately.**  Its output is
    pinned bit-for-bit against ``realize_tilted`` (the Phase-1 acceptance bar), so
    the accel bound applies to banking-on paths only.

    **Every bound is on the tilt VECTOR (2-norm), never per axis.**  The 12°
    ceiling is a from-vertical *angle*, so bounding ``rx`` and ``ry`` independently
    does not bound it — a per-axis version of this leaked a saturated bank to
    12.12° (caught by ``test_banking_never_exceeds_the_12_degree_cap``).  Every
    slew move is toward a point already inside the cap disc, so the cap is
    preserved by convexity rather than by a re-clamp.  A pin outside the ceiling is
    a caller bug and raises — the pins are exact by contract, so the schedule
    cannot saturate them (the ``toss_release`` "gate the aim, don't rely on the
    clamp" precedent).

    **``start_tilt`` — the seam pin (``None`` by default, so every pre-existing
    caller is bit-identical).**  A window that abuts another one at knot 0 — the
    second half of a chained pair, or the tail of a mid-cycle re-plan — must
    start at the tilt the preceding window ENDED at, and the banking objective
    cannot supply it.  Two independent reasons:

    * At a release the cup is in free fall, so the apparent-gravity field
      ``g − a_cup`` is exactly ZERO and ``tilt_to_receive`` of it is LEVEL.  That
      is why the release knot is pinned to ``throw_tilt`` at the far end; the
      same degeneracy sits at knot 0 of the window that follows, so the same pin
      is needed there or the two windows disagree by the whole throw tilt
      (up to 12°) at one shared instant.
    * The disagreement is not cosmetic.  ``decompose`` turns a tilt into a
      centroid offset through the 744.3 mm ``CUP_TILT_CENTER_Z_MM`` lever, so a
      tilt step of θ at the seam is a centroid-xy step of ``arm·sin θ`` — tens of
      millimetres of leg position inside one 25 ms knot, which is a step command
      on six legs, the thing this stack refuses to emit anywhere else.

    The pins resolve in knot order with the terminal pin strongest: ``start_tilt``
    is applied first, then the catch pin, then the throw pin, so a collision (a
    catch on knot 0, or a single-knot plan) leaves the physically stronger pin
    standing rather than depending on list order.

    If ``catch_k`` coincides with the final knot the throw pin wins — release is
    the plan's terminal boundary condition and the next cycle chains off it.
    """
    cfg = RealizeConfig() if cfg is None else cfg
    acc = np.asarray(cup_plan.acc, dtype=float)
    if acc.ndim != 2 or acc.shape[1] != 3:
        raise ValueError(f"cup_plan.acc must be (n, 3), got {acc.shape}")
    n = int(acc.shape[0])
    if n < 1:
        raise ValueError("cup_plan must carry at least one knot")
    dt = float(cup_plan.dt)
    if not dt > 0.0:
        raise ValueError(f"cup_plan.dt must be > 0, got {dt}")
    catch_k = int(cup_plan.catch_k)

    recv = _as_tilt_pair(receive_tilt, 'receive_tilt')
    throw = _as_tilt_pair(throw_tilt, 'throw_tilt')
    start = (None if start_tilt is None
             else _as_tilt_pair(start_tilt, 'start_tilt'))

    if cfg.banking_enabled:
        raw = _banking_raw(acc, cfg, start)
    else:
        idx = np.arange(n)
        raw = np.where((idx <= catch_k)[:, None], recv[None, :], throw[None, :])

    # Pins, weakest first so a collision leaves the stronger pin standing (see
    # the docstring): seam, then catch, then the terminal release.  The release
    # pin is unconditional; the catch pin only exists when the catch is a
    # distinct interior knot; the seam pin only when the caller supplied one.
    pinned = {}
    if start is not None:
        pinned[0] = start
    if 0 <= catch_k < n - 1:
        pinned[catch_k] = recv
    pinned[n - 1] = throw
    anchors = [(j, pinned[j]) for j in sorted(pinned)]
    anchor_set = set(pinned)

    cap = np.radians(float(cfg.max_tilt_deg))
    origin = np.zeros(2)
    checks = [('receive_tilt', recv), ('throw_tilt', throw)]
    if start is not None:
        checks.append(('start_tilt', start))
    for name, v in checks:
        mag = float(np.hypot(v[0], v[1]))
        if mag > cap * (1.0 + 1e-9) + 1e-12:
            raise ValueError(
                f"{name} is {np.degrees(mag):.3f}° from vertical, past the "
                f"{cfg.max_tilt_deg}° usable ceiling. The pins are exact, so the "
                "schedule cannot saturate them — gate the aim upstream (the "
                "toss_release precedent) instead of relying on a silent clamp.")

    out = _project_to_disc(raw.astype(float, copy=True), origin, cap)
    out = out.astype(float, copy=True)
    step = float(cfg.tilt_rate_limit_rad_s) * dt
    accel_cap = float(cfg.tilt_accel_limit_rad_s2)
    if cfg.banking_enabled and accel_cap > 0.0:
        # The accel bound reshapes the BANKING solution only.  The zero-banking
        # branch above is the legacy two-constant realisation, and its bit-for-bit
        # agreement with ``realize_tilted`` is the Phase-1 acceptance bar — a
        # smoother there would move the very series the parity test pins at
        # max |Δ| == 0.0.  A caller wanting a smooth handover between two
        # DIFFERENT constant tilts is asking for a banking schedule.
        # The rate sweeps run inside this call, on every widen attempt.
        #
        # The JERK bound is re-derived for THIS cycle.  ``cfg.tilt_jerk_limit_rad_s3``
        # carries the reserved leg-jerk budget divided by the STATIC lever, so
        # multiplying it back by that lever recovers the budget in mm/s³ — and
        # that budget is spent against the tilt channel's OWN measured
        # contribution (:func:`_tilt_leg_jerk_mmps3`), because a fast cup moves
        # the tilt lever under the tilt and the product-rule content that creates
        # is leg jerk no static lever has a term for.  The rad/s³ cap stays as
        # the second, geometry-only ceiling, and the per-cycle lever estimate
        # only sizes the widen loop's first guess.  The ACCEL cap is a second
        # difference and the static lever is right for it, so it is untouched.
        jerk_cap = float(cfg.tilt_jerk_limit_rad_s3)
        budget_mm = max(jerk_cap, 0.0) * TILT_ACCEL_LEVER_MM
        jerk_terms = _tilt_jerk_terms(cup_plan, dt, n)
        if jerk_cap > 0.0 and jerk_terms is not None:
            jerk_cap = min(jerk_cap,
                           budget_mm / _tilt_jerk_lever_mm(cup_plan, dt))
        return _accel_bounded_schedule(out, anchors, dt, accel_cap, cap, step,
                                       jerk_cap, jerk_terms, budget_mm)

    for j, v in anchors:
        out[j] = v
    return _rate_limit_sweeps(out, anchor_set, step)


# ── Decomposition ─────────────────────────────────────────────────────────────

def _knot_derivative(values: np.ndarray, dt: float) -> np.ndarray:
    """Second-order knot derivative of a uniformly-spaced series (central inside,
    second-order one-sided at both ends).  Shape-preserving; zeros for n < 2."""
    a = np.asarray(values, dtype=float)
    n = a.shape[0]
    d = np.zeros_like(a)
    if n < 2:
        return d
    if n == 2:
        d[0] = d[1] = (a[1] - a[0]) / dt
        return d
    d[1:-1] = (a[2:] - a[:-2]) / (2.0 * dt)
    d[0] = (-3.0 * a[0] + 4.0 * a[1] - a[2]) / (2.0 * dt)
    d[-1] = (3.0 * a[-1] - 4.0 * a[-2] + a[-3]) / (2.0 * dt)
    return d


def decompose(cup_plan, tilts, cfg=None) -> RealizedCycle:
    """Cup track + tilt schedule → per-knot platform pose and slider.

    The whole-series generalisation of ``sim/juggle_tilt.py::realize_tilted``.  Per
    knot, with the cup opening's planned world position ``(cup_xy, cup_z)`` and the
    scheduled tilt ``(rx, ry)``:

      * ``arm    = cup_z_mm − CUP_TILT_CENTER_Z_MM``  (signed; the cup rides above
        the rotation centre at a high slider and below it at a low one, so the
        lever is a height-dependent quantity, NOT a fixed mm/deg)
      * ``shift  = arm · cup_axis_xy``   → ``centroid_xy = cup_xy_mm − shift``
      * ``drop   = arm · (1 − cup_axis_z)`` → the slider is raised by it so the
        tilted cup opening still reaches ``cup_z``
      * ``slider = cup_z_mm − base + drop``, clamped to ``[0, stroke]``

    **z behaviour.**  With ``cfg.z_float_enabled`` false (the default) the pose z is
    ``cfg.active_z_mm`` at every knot, bit-exactly — the pin ``realize_tilted``
    hard-codes.  With it true, z moves ONLY where the slider saturates, by the
    smaller of the shortfall and ``cfg.z_band_mm``: a slider demand above the
    stroke raises the platform (less slider needed), a demand below zero lowers it.
    Raising z by ``dz`` raises ``base`` by ``dz`` and therefore takes ``dz`` off the
    slider — the platform is buying back exactly the stroke that ran out.  This
    function only *offers* the excursion; ``validate_cycle`` (WP4) is what gates it
    against the leg workspace, so an unreachable z leaves here and is refused
    there, loudly, rather than being silently trimmed here.

    **Knot velocities.**  Analytic wherever the cup plan knows the answer, which is
    everything that carries the throw: the lever arm's rate is exactly the cup's
    vertical speed (``arm = cup_z − const``), so ``d(shift)/dt`` and ``d(drop)/dt``
    keep the cup's own ``vel`` term exactly and finite-difference only the
    *tilt-rate* term (identically zero for a constant tilt).  The pose z rate and
    the tilt rates are finite differences of series this function itself produced.
    The slider rate is forced to zero at a saturated knot — the stroke clamp is a
    real wall, and pretending the hand still moves there would hand the wire a
    velocity the machine cannot follow.
    """
    cfg = RealizeConfig() if cfg is None else cfg

    pos = np.asarray(cup_plan.pos, dtype=float)
    vel = np.asarray(cup_plan.vel, dtype=float)
    if pos.ndim != 2 or pos.shape[1] != 3:
        raise ValueError(f"cup_plan.pos must be (n, 3), got {pos.shape}")
    if vel.shape != pos.shape:
        raise ValueError(f"cup_plan.vel must match pos {pos.shape}, got {vel.shape}")
    n = int(pos.shape[0])
    tilt_arr = np.asarray(tilts, dtype=float)
    if tilt_arr.shape != (n, 2):
        raise ValueError(f"tilts must be ({n}, 2), got {tilt_arr.shape}")
    dt = float(cup_plan.dt)
    if not dt > 0.0:
        raise ValueError(f"cup_plan.dt must be > 0, got {dt}")

    stroke = float(cfg.slider_stroke_mm)
    band = max(0.0, float(cfg.z_band_mm)) if cfg.z_float_enabled else 0.0

    pose = np.zeros((n, 6), dtype=float)
    axis = np.zeros((n, 3), dtype=float)
    arm = np.zeros(n, dtype=float)
    slider_mm = np.zeros(n, dtype=float)
    dz = np.zeros(n, dtype=float)
    saturated = np.zeros(n, dtype=bool)

    for k in range(n):
        rx = float(tilt_arr[k, 0])
        ry = float(tilt_arr[k, 1])
        # Same statement order as realize_tilted — see the module docstring on
        # why bit-exactness here is a contract and not a coincidence.
        cup_z_mm = float(pos[k, 2]) * 1000.0
        a = tilt_geometry.cup_axis(rx, ry)
        arm_k = tilt_geometry.cup_lever_arm_mm(cup_z_mm)
        shift = arm_k * a[:2]
        drop_k = arm_k * (1.0 - float(a[2]))
        slider_raw = cup_z_mm - cfg.cup_z_base_mm + drop_k

        dz_k = 0.0
        if band > 0.0:
            if slider_raw > stroke:
                dz_k = min(slider_raw - stroke, band)
            elif slider_raw < 0.0:
                dz_k = -min(-slider_raw, band)
        s = slider_raw - dz_k
        s_clamped = min(max(s, 0.0), stroke)

        centroid = pos[k, :2] * 1000.0 - shift
        pose[k, 0] = centroid[0]
        pose[k, 1] = centroid[1]
        pose[k, 2] = cfg.active_z_mm + dz_k
        pose[k, 3] = rx
        pose[k, 4] = ry
        pose[k, 5] = 0.0

        axis[k] = a
        arm[k] = arm_k
        dz[k] = dz_k
        slider_mm[k] = s_clamped
        saturated[k] = (s_clamped != s)

    # ── knot velocities ──
    vxy_mm = vel[:, :2] * 1000.0
    vz_mm = vel[:, 2] * 1000.0
    axis_dot = _knot_derivative(axis, dt)                  # (n, 3)
    dz_dot = _knot_derivative(dz, dt)                      # (n,)
    tilt_dot = _knot_derivative(tilt_arr, dt)              # (n, 2)

    # d(arm)/dt == d(cup_z_mm)/dt == vz_mm exactly (arm is cup_z minus a constant).
    shift_dot = vz_mm[:, None] * axis[:, :2] + arm[:, None] * axis_dot[:, :2]
    drop_dot = vz_mm * (1.0 - axis[:, 2]) - arm * axis_dot[:, 2]

    pose_vel = np.zeros((n, 6), dtype=float)
    pose_vel[:, :2] = vxy_mm - shift_dot
    pose_vel[:, 2] = dz_dot
    pose_vel[:, 3:5] = tilt_dot
    # pose_vel[:, 5] stays 0 — rz is pinned to 0 by the realisation.

    slider_vel_mm_s = vz_mm - dz_dot + drop_dot
    slider_vel_mm_s = np.where(saturated, 0.0, slider_vel_mm_s)

    # the mm->rev conversion, vectorised (it is float-only).  The zero offset is a
    # position offset, so it applies to the position and NOT to the rate.
    slider_rev = ((slider_mm - float(cfg.slider_rev_zero_mm)) / 1000.0
                  * HAND_REV_PER_M)
    slider_vel_rev_s = slider_vel_mm_s / 1000.0 * HAND_REV_PER_M

    t = getattr(cup_plan, 't', None)
    t_arr = (np.asarray(t, dtype=float).reshape(n) if t is not None
             else np.arange(n, dtype=float) * dt)

    return RealizedCycle(
        pose=pose,
        pose_vel=pose_vel,
        slider_mm=slider_mm,
        slider_rev=slider_rev,
        slider_vel_rev_s=slider_vel_rev_s,
        t=t_arr,
        dt=dt,
        catch_k=int(cup_plan.catch_k),
        tilts=tilt_arr.astype(float, copy=True),
        slider_saturated=saturated,
        z_excursion_mm=dz,
    )
