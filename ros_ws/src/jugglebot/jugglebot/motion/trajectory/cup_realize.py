"""Stage 2+3 of the unified 7-DoF cycle: tilt schedule, then cup → platform+slider.

The unified planner (`plans/active/unified-7dof-planner.md` § 4, Phase 1) works in
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
from jugglebot.motion.trajectory.hand_stroke import LINEAR_GAIN_REV_PER_M

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
#: ``plans/parked/hand-trajectory-generator-overhaul.md``: the firmware homes
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
                 / LINEAR_GAIN_REV_PER_M * 1000.0)
TILT_ACCEL_LEVER_MM = (_CUP_Z_TOP_MM
                       - float(tilt_geometry.CUP_TILT_CENTER_Z_MM)
                       + float(hw.GEOM_PLAT_RADIUS_MM))
TILT_ACCEL_LIMIT_DEFAULT_RAD_S2 = (
    TILT_ACCEL_BUDGET_FRACTION * float(hw.JB_TRAJ_LEG_ACC_LIMIT_MMPS2)
    / TILT_ACCEL_LEVER_MM)

#: Largest ``|S''(u)|`` of the quintic smoothstep ``S(u) = 6u⁵ − 15u⁴ + 10u³``
#: used by the pin blend, attained at ``u = (1 ± 1/√3)/2``.  It is what turns a
#: requested tilt acceleration into a blend half-width (see
#: :func:`_accel_bounded_schedule`).
_BLEND_CURVATURE_MAX = 10.0 / np.sqrt(3.0)

#: Bounded widen attempts in :func:`_accel_bounded_schedule`.  The analytic widths
#: are an ESTIMATE (they bound each mechanism separately and ignore the cross
#: term), so the routine measures the second difference it actually produced and
#: widens if it missed.  Bounded and not asserted for the same reason
#: :data:`_SLEW_SWEEPS` is: when the pins are too close together to join under the
#: cap the constraint set is empty and something must give.  ``validate_cycle`` is
#: the authority on what the machine will accept, not this loop.
_TILT_WIDEN_ATTEMPTS = 8

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
                            rate_step: float) -> np.ndarray:
    """Smooth ``raw`` until its tilt acceleration is under ``accel_cap``, pins exact.

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
    4. **The rate sweeps run INSIDE the loop**, so the second difference that is
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
    # Mechanism 1: the smoother's own second difference, |Δ²| <= 4·max|x|/(h+1)².
    half = int(np.ceil(2.0 * np.sqrt(max(tilt_cap, 1e-12) / budget))) - 1

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

    out = raw.astype(float, copy=True)
    boost = 1.0
    for _ in range(_TILT_WIDEN_ATTEMPTS):
        h = int(min(max(half, 0), half_max))
        sm = _triangular_smooth(raw, h)
        # Mechanism 2: the blend window's curvature, ``gap·S''max/L² <= budget``.
        # The gap is measured against the SMOOTHED series, not the raw one, because
        # that is the distance the blend actually has to close — and it GROWS with
        # ``h`` (a heavier smoother pulls the schedule further from a pin it is not
        # allowed to move). Sizing L off ``raw`` instead reads the gap at ``h = 0``
        # and under-widens by up to 2× on exactly the cases that need it most.
        pin_gap = max((float(np.hypot(*(np.asarray(v, dtype=float) - sm[j])))
                       for j, v in anchors), default=0.0)
        # ``boost`` is what the widen loop turns: the estimate bounds the window's
        # OWN curvature and ignores the cross term ``2·W'·sm'``, which is real
        # wherever the smoothed series is still moving under the window.
        L = float(min(max(boost * np.ceil(
            np.sqrt(pin_gap * _BLEND_CURVATURE_MAX / budget)), 1.0), blend_max))
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
        if _max_tilt_accel(out, dt) <= accel_cap:
            break
        if h >= half_max and L >= blend_max:
            break
        half = max(h + 1, int(np.ceil(h * 1.4)))
        boost *= 1.4
    return out


def _as_tilt_pair(value, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.shape != (2,):
        raise ValueError(f"{name} must be an (rx, ry) pair, got shape "
                         f"{np.shape(value)}")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must be finite, got {arr.tolist()}")
    return arr


def tilt_schedule(cup_plan, receive_tilt, throw_tilt, cfg=None) -> np.ndarray:
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

    if cfg.banking_enabled:
        g_vec = np.array([0.0, 0.0, -float(cfg.gravity_mps2)])
        raw = np.empty((n, 2), dtype=float)
        for k in range(n):
            raw[k] = tilt_geometry.tilt_to_receive(g_vec - acc[k],
                                                   max_tilt_deg=cfg.max_tilt_deg)
    else:
        idx = np.arange(n)
        raw = np.where((idx <= catch_k)[:, None], recv[None, :], throw[None, :])

    # Pins, in knot order.  The release pin is unconditional; the catch pin only
    # exists when the catch is a distinct interior knot.
    anchors = []
    if 0 <= catch_k < n - 1:
        anchors.append((catch_k, recv))
    anchors.append((n - 1, throw))
    anchor_set = {j for j, _ in anchors}

    cap = np.radians(float(cfg.max_tilt_deg))
    origin = np.zeros(2)
    for name, v in (('receive_tilt', recv), ('throw_tilt', throw)):
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
        return _accel_bounded_schedule(out, anchors, dt, accel_cap, cap, step)

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

    # hand_stroke.mm_to_rev, vectorised (it is float-only).  The zero offset is a
    # position offset, so it applies to the position and NOT to the rate.
    slider_rev = ((slider_mm - float(cfg.slider_rev_zero_mm)) / 1000.0
                  * LINEAR_GAIN_REV_PER_M)
    slider_vel_rev_s = slider_vel_mm_s / 1000.0 * LINEAR_GAIN_REV_PER_M

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
