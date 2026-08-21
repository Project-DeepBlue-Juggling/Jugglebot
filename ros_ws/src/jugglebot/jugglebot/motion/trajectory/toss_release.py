"""Toss release state — STOW→global conversion + toss ballistics (Tiers 8a/8b).

Pure numpy release-state math for the self-toss (``Toss.action``,
``plans/active/single-ball-toss.md`` Phase 1): where the ball leaves the hand,
the launch velocity that lands it back in the cup after ``flight_time_s``, and
the physics fields of the self-``ThrowAnnouncement`` that closes the existing
correlation → catch loop unchanged. Phase 4 adds the Tier-8b displaced case
(:func:`compute_release_state_tilted`): throw from site A, tilt-aimed at the
displaced catch B, with the loud :class:`ThrowTiltInfeasible` clamp gate.
Consumed by ``toss_sequencer`` / ``reload_coordinator_node``; siblings
``ballistics_bc`` (catch-side boundary conditions) and ``tilt_geometry``
(receive tilt + the Phase-4 throw mirror).

**One conversion point per direction.** :func:`stow_to_global_mm` is THE single
tested STOW-relative → global conversion for the toss (plan § Frame convention);
the global → STOW producer already lives in
``catch_coordinator._compute_catch_command``. Do not add another.
:func:`aim_target_offset_mm` is the same rule one layer up: THE single commanded
aim-angle → virtual-target-displacement conversion for the toss aim map
(contract C-TOSS-CAL-1, ``jugglebot/motion/toss_cal.py``), so the aim rides the
already-tested Tier-8b geometry instead of a second copy of it.

**Full ballistic inverse, not the idealised magnitude.** The launch velocity is
the full release-plane → catch-plane solution ``vz = Δz/T + g·T/2`` with
``Δz = HAND_CATCH_OFFSET_MM − HAND_THROW_OFFSET_MM = 6.736 mm`` (the cup plane
sits above the release plane) — that is what the throw must actually deliver.
The plan's ``v = g·T/2`` sanity magnitude is the Δz→0 limit (Δ = Δz/T ≈ 8.4 mm/s
at T = 0.8 s) and is pinned as a test identity, never returned here.

Pure Python + numpy. No ROS2 / repo-root / ``controller`` imports — ballistics
comes from ``ballistics_bc`` (the motion-side copy of ``controller/ballistics``)
so this module has exactly one gravity source, ``GRAVITY_MMS2 = 9806.0`` (never
the tracker-side 9810.0).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import jugglebot.hardware_config as hw
from jugglebot.motion.trajectory import ballistics_bc
from jugglebot.motion.trajectory import tilt_geometry

# Release-plane height of the ball above the platform centroid at the
# traj_type=0 release event — the THROW sibling of hw.HAND_CATCH_OFFSET_MM
# (which generate_config.py derives as hand_axis_bottom_offset_mm +
# HAND_CATCH_POS_M·1000). No generated constant exists for the throw case, so
# it is derived here, once, from the same two generated inputs:
#   hw.GEOM_HAND_AXIS_BOTTOM_OFFSET_MM = -129.0    (hand axis bottom vs centroid)
#   hw.HAND_THROW_POS_M                =  0.187044 (x2 — "hand pos at ball
#       release", the Trajectory.h stroke algebra in generate_config.py)
# = -129.0 + 187.044 = 58.044 mm.
HAND_THROW_OFFSET_MM = hw.GEOM_HAND_AXIS_BOTTOM_OFFSET_MM \
    + hw.HAND_THROW_POS_M * 1000.0


@dataclass(frozen=True)
class ReleaseState:
    """Tier-8a release state (all world/global mm; see :func:`compute_release_state`)."""
    release_pos_global_mm: np.ndarray   # (3,) ball position at release
    launch_vel_mms: np.ndarray          # (3,) ball velocity at release, mm/s
    event_vel_mps: float                # |launch_vel|/1000 — SetHandTrajCmd.event_vel
    catch_point_global_mm: np.ndarray   # (3,) nominated cup point
    flight_time_s: float


def stow_to_global_mm(pos_stow_mm, *,
                      initial_height_mm: float = hw.GEOM_INITIAL_HEIGHT_MM
                      ) -> np.ndarray:
    """Convert a STOW-relative platform-frame point (the Toss.action goal /
    GoToPose/TimedTarget/DynamicTargetCommand pose convention: z = 0 is the
    stow plane, z ≈ 170 the ACTIVE pose — normative frame declaration in
    ``ros_ws/src/jugglebot_interfaces/msg/DynamicTargetCommand.msg`` lines
    6-11, hardware-verified 2026-07-23) to GLOBAL/world mm (floor z = 0, the
    ``ThrowAnnouncement.landing_position`` frame).

    THE single tested STOW→global conversion point for the toss (plan § Frame
    convention): general (x, y, z) — generalises
    ``reload_sequencer.compute_catch_point_mm``, which only produces the
    on-axis x = y = 0 point. The conversion is exactly one addition of
    ``initial_height_mm`` to z; adding any active-z lift here or downstream
    double-counts it (the 2026-07-23 first-sitting z bug,
    ``logbook/2026-07-23-phase7-reload-first-hardware-session.md``).
    """
    p = np.asarray(pos_stow_mm, dtype=float).reshape(3)
    return p + np.array([0.0, 0.0, float(initial_height_mm)])


def flight_time_from_height(throw_height_m: float) -> float:
    """Convert a THROW HEIGHT (m, apex above the release plane) to the internal
    flight time (s, release→catch). THE operator-height → flight-time conversion
    for ``Toss.action`` (the goal nominates a height — the juggling-relevant
    variable; everything downstream stays flight-time-native).

    Idealised co-located relation ``T = sqrt(8·h/g)`` (apex ``h = g·T²/8``). The
    ``Δz = 6.736 mm`` release↔cup offset is negligible at juggling heights
    (~0.8 % at 0.8 m) and is still carried EXACTLY downstream by
    :func:`compute_release_state`'s ``vz = Δz/T + g·T/2`` from this T. Note
    ``h ∝ T²`` — NOT linear (4× height is 2× flight time)."""
    h_mm = float(throw_height_m) * 1000.0
    return float(np.sqrt(8.0 * h_mm / ballistics_bc.GRAVITY_MMS2))


def apex_height_from_flight_time(flight_time_s: float) -> float:
    """Inverse of :func:`flight_time_from_height` (m): ``h = g·T²/8``. The exact
    inverse identity (pinned by test), used for reporting/round-trip checks."""
    return float(
        ballistics_bc.GRAVITY_MMS2 * float(flight_time_s) ** 2 / 8.0 / 1000.0)


def compute_release_state(catch_position_stow_mm, flight_time_s: float, *,
                          initial_height_mm: float = hw.GEOM_INITIAL_HEIGHT_MM,
                          hand_catch_offset_mm: float = hw.HAND_CATCH_OFFSET_MM,
                          hand_throw_offset_mm: float = HAND_THROW_OFFSET_MM
                          ) -> ReleaseState:
    """Tier-8a release state for a co-located vertical toss.

    ``catch_position_stow_mm`` is the Toss.action goal's nominated catch point:
    the PLATFORM pose at catch, STOW-relative (z 170 = ACTIVE plane, same
    convention as TimedTarget.pose). The platform is pre-positioned to this
    pose (POSITIONING) and held level through the flight, so throw site ==
    catch site; the release and cup planes ride ``hand_throw_offset_mm`` /
    ``hand_catch_offset_mm`` above the platform centroid.

    The launch velocity is the FULL release-plane → catch-plane ballistic
    inverse (``ballistics_bc.launch_velocity`` — one gravity source), not the
    idealised ``g·T/2`` magnitude; see the module docstring.

    Raises ``ValueError`` if ``flight_time_s <= 0`` (propagated from
    ``ballistics_bc.launch_velocity``).

    NOTE: ``event_vel_mps`` is returned UNCLAMPED — the caller gates it via
    :func:`validate_event_vel` and rejects the goal (``REJECTED_INFEASIBLE``)
    rather than silently clamping to a physically-different toss.
    """
    platform_global = stow_to_global_mm(catch_position_stow_mm,
                                        initial_height_mm=initial_height_mm)
    release_pos = platform_global + np.array([0.0, 0.0, float(hand_throw_offset_mm)])
    catch_point = platform_global + np.array([0.0, 0.0, float(hand_catch_offset_mm)])
    launch_vel = ballistics_bc.launch_velocity(release_pos, catch_point,
                                               flight_time_s)
    return ReleaseState(
        release_pos_global_mm=release_pos,
        launch_vel_mms=launch_vel,
        event_vel_mps=float(np.linalg.norm(launch_vel)) / 1000.0,
        catch_point_global_mm=catch_point,
        flight_time_s=float(flight_time_s),
    )


@dataclass(frozen=True)
class TiltedReleaseState(ReleaseState):
    """Tier-8b release state: :class:`ReleaseState` + the throw tilt and the
    swing-compensated pre-tilt POSITIONING pose at the throw site A.

    ``release_pos_global_mm`` / ``launch_vel_mms`` describe the TILTED release:
    the ball leaves at nominal A xy (the centroid swing-compensation puts it
    there) with the lateral launch component that carries it to the displaced
    B; ``catch_point_global_mm`` is B's cup point — the announced landing IS B,
    which is what closes the correlation → catch loop at the displaced site.
    """
    tilt_rx: float                    # throw tilt about world x {rad}, rz = 0
    tilt_ry: float                    # throw tilt about world y {rad}
    pretilt_pose_stow: np.ndarray     # (6,) [x, y, z, rx, ry, 0] STOW-frame
    #                                   go_to_pose POSITIONING target at A
    #                                   (centroid swing-compensated)
    displacement_mm: float            # |B_xy − A_xy| — the displacement-cap
    #                                   gate's measurand (JB_OP_TOSS_MAX_…)


class ThrowTiltInfeasible(ValueError):
    """The nominated displaced toss needs a throw aim past ``max_tilt_deg``.

    Raised by :func:`compute_release_state_tilted` INSTEAD of letting
    ``tilt_to_throw``'s clamp silently saturate: a saturated throw tilt
    launches the ball along the clamped — not the required — direction, so it
    lands with a systematic bias short of B (the bb Rung-2a "landing bias").
    The silent clamp is correct only for the CATCH (a partially-nulled arrival
    still seats); a throw must reject loudly (``REJECTED_TILT_CLAMP``) rather
    than fly mis-aimed. **Still a contract, not a live constraint, at the
    Phase-E 150 mm cap**: measured
    (``tools/probes/displaced_reach_frontier.py``, 2026-07-29) the required aim
    at 150 mm runs 5.75° (T = 0.55 s) down to 1.45° (T = 1.10 s), and the 12°
    ceiling first binds at ~320 mm / T = 0.55 s (required 12.1°) — 2.1× the cap.
    Carries ``required_deg`` / ``max_tilt_deg`` for the reject diagnostics.
    """

    def __init__(self, required_deg: float, max_tilt_deg: float):
        self.required_deg = float(required_deg)
        self.max_tilt_deg = float(max_tilt_deg)
        super().__init__(
            "displaced toss needs a {:.2f} deg from-vertical aim > the "
            "{:.2f} deg tilt ceiling — rejected (a silently clamped aim "
            "lands the ball short of B, the Rung-2a landing bias)".format(
                self.required_deg, self.max_tilt_deg))


def _from_vertical_deg(v_mms) -> float:
    """From-vertical angle (deg) of a launch vector — the aim the throw tilt
    must deliver. ``atan2(|v_xy|, v_z)`` (exact on both axes, no normalisation;
    same angle ``tilt_to_receive`` derives via arccos of the unit z)."""
    v = np.asarray(v_mms, dtype=float).reshape(3)
    return float(np.degrees(np.arctan2(np.hypot(v[0], v[1]), v[2])))


def _tilted_release_pos(a_xy, cup_z_world_mm: float, rx: float, ry: float
                        ) -> np.ndarray:
    """Global release point under throw tilt ``(rx, ry)``, swing-compensated.

    The commanded centroid pulls back by the lever-arm swing (see
    :func:`compute_release_state_tilted`), so the tilted release xy sits
    exactly AT nominal A; what remains is the vertical cross-coupling — the
    release point rides ``arm·(1 − cos θ)`` BELOW the level release plane
    (``arm = cup_z − 744.3``, the fixed world tilt centre — the same
    convention as the catch side's ``_toss_catch_pose`` / CCN swing math).
    """
    axis = tilt_geometry.cup_axis(rx, ry)
    arm = tilt_geometry.cup_lever_arm_mm(cup_z_world_mm)
    drop = arm * (1.0 - float(axis[2]))
    return np.array([float(a_xy[0]), float(a_xy[1]),
                     float(cup_z_world_mm) - drop])


def compute_release_state_tilted(catch_position_stow_mm, flight_time_s: float,
                                 *,
                                 throw_site_xy_mm,
                                 initial_height_mm: float = hw.GEOM_INITIAL_HEIGHT_MM,
                                 hand_catch_offset_mm: float = hw.HAND_CATCH_OFFSET_MM,
                                 hand_throw_offset_mm: float = HAND_THROW_OFFSET_MM,
                                 max_tilt_deg: float = tilt_geometry.MAX_TILT_DEG
                                 ) -> TiltedReleaseState:
    """Tier-8b release state for a tilt-aimed displaced throw → catch.

    The throw launches from site A (``throw_site_xy_mm``, STOW xy) at the
    displaced catch B (``catch_position_stow_mm``, the Toss.action goal —
    same convention as :func:`compute_release_state`). Both sites share the
    goal's nominated platform z. All lateral take-off comes from the slider
    stroke projected through the throw tilt — never platform translation.

    ``throw_site_xy_mm`` is **REQUIRED — there is no default A** (it carried
    one, ``hw.JB_OP_TOSS_THROW_SITE_MM``, until 2026-07-29; that config key is
    retired). The only correct throw site is where the platform actually is at
    release, so any default is a phantom site: the aim is then solved from a
    point the ball does not leave, and — worse — POSITIONING obediently
    translates the platform TO that phantom point before throwing, silently
    undoing an operator who parked it elsewhere. Making the argument mandatory
    is the single enforcement point for "A is never inferred"; the caller
    (``reload_coordinator_node._build_toss_cycle``) sources it from
    ``trajectory/commanded_position`` and refuses the goal outright when that
    is stale or absent (``REJECTED_POSE_UNKNOWN``).

    Chain (single-ball-toss plan Phase 4, all-mm unit discipline —
    ``tilt_geometry``'s lever helpers here are the mm-based production
    ``shaping`` re-exports, NOT the sim copies, which take metres):

    1. Level-plane ballistic inverse A → B's cup point over ``flight_time_s``.
    2. **Loud clamp gate**: required from-vertical aim > ``max_tilt_deg`` ⇒
       :class:`ThrowTiltInfeasible` (never a silently clamped, mis-aimed
       throw; the sequencer maps it to ``REJECTED_TILT_CLAMP``).
    3. ``tilt_to_throw`` aim + ONE fixed-point pass: the tilted release point
       sits ``arm·(1 − cos θ)`` lower (≈0.03 mm at 1.8°), so the inverse is
       recomputed from the true release point and the tilt re-derived once —
       the correction is ~0.1 %, converging immediately (<0.1 mm / <1 mm/s
       after one pass, pinned in ``tests/motion/test_toss_release.py``). The
       returned state is exactly self-consistent: ``launch_vel_mms`` is the
       inverse from the returned ``release_pos_global_mm``.
    4. Swing-compensated pre-tilt pose at A: the tilt swings the release
       point sideways by ``cup_lateral_shift_mm(rx, ry, cup_z)``, so the
       commanded centroid pulls back by exactly that and the release xy sits
       AT nominal A — the throw-side analogue of the catch's
       ``_toss_catch_pose`` centroid offset, same fixed-world-tilt-centre
       lever convention.

    Degenerate identity: ``throw_site == B_xy`` reproduces
    :func:`compute_release_state` BITWISE (tilt exactly level, zero shift and
    drop) — the Tier-8a regression net.

    ``event_vel_mps`` is ``|launch|/1000`` — inside the clamp gate the cup
    axis is parallel to the launch, so the slider speed ALONG the tilted axis
    IS the full launch magnitude; same UNCLAMPED semantics and
    :func:`validate_event_vel` gating as Tier 8a. Raises ``ValueError`` for
    ``flight_time_s <= 0`` (propagated from ``ballistics_bc``).
    """
    b = np.asarray(catch_position_stow_mm, dtype=float).reshape(3)
    if throw_site_xy_mm is None:
        # Explicit refusal, not a fallback: see the "REQUIRED" paragraph above.
        raise ValueError(
            "compute_release_state_tilted: throw_site_xy_mm is required — the "
            "throw site A is the platform's live commanded xy, never a default")
    a_xy = np.asarray(throw_site_xy_mm, dtype=float).reshape(2)
    z_nom = float(b[2])

    # B's cup point (global) — the announced landing IS B.
    catch_platform_global = stow_to_global_mm(
        b, initial_height_mm=initial_height_mm)
    catch_point = catch_platform_global + np.array(
        [0.0, 0.0, float(hand_catch_offset_mm)])

    # Level release plane at A (platform holds the nominated z at both sites).
    throw_platform_global = stow_to_global_mm(
        (a_xy[0], a_xy[1], z_nom), initial_height_mm=initial_height_mm)
    release_level = throw_platform_global + np.array(
        [0.0, 0.0, float(hand_throw_offset_mm)])
    # World z of the release-plane cup at level — the lever-arm height the
    # swing compensation and vertical-drop terms use.
    cup_z_world = float(release_level[2])

    # Pass 0 (level aim): ballistic inverse from the level plane, gated LOUDLY
    # before any tilt is derived (step 2 of the chain).
    v0 = ballistics_bc.launch_velocity(release_level, catch_point,
                                       flight_time_s)
    required_deg = _from_vertical_deg(v0)
    if required_deg > float(max_tilt_deg):
        raise ThrowTiltInfeasible(required_deg, float(max_tilt_deg))
    rx, ry = tilt_geometry.tilt_to_throw(v0, float(max_tilt_deg))

    # Pass 1 (tilted): recompute the inverse from the true (tilted) release
    # point and re-derive the tilt once (step 3 of the chain).
    release_pos = _tilted_release_pos(a_xy, cup_z_world, rx, ry)
    v1 = ballistics_bc.launch_velocity(release_pos, catch_point, flight_time_s)
    rx, ry = tilt_geometry.tilt_to_throw(v1, float(max_tilt_deg))
    release_pos = _tilted_release_pos(a_xy, cup_z_world, rx, ry)
    launch_vel = ballistics_bc.launch_velocity(release_pos, catch_point,
                                               flight_time_s)

    # Swing-compensated pre-tilt POSITIONING pose at A (step 4 of the chain).
    shift = tilt_geometry.cup_lateral_shift_mm(rx, ry, cup_z_mm=cup_z_world)
    pretilt_pose_stow = np.array([a_xy[0] - shift[0], a_xy[1] - shift[1],
                                  z_nom, rx, ry, 0.0])

    return TiltedReleaseState(
        release_pos_global_mm=release_pos,
        launch_vel_mms=launch_vel,
        event_vel_mps=float(np.linalg.norm(launch_vel)) / 1000.0,
        catch_point_global_mm=catch_point,
        flight_time_s=float(flight_time_s),
        tilt_rx=rx,
        tilt_ry=ry,
        pretilt_pose_stow=pretilt_pose_stow,
        displacement_mm=float(np.hypot(b[0] - a_xy[0], b[1] - a_xy[1])),
    )


def aim_target_offset_mm(aim_rx: float, aim_ry: float, flight_time_s: float,
                         catch_z_stow_mm: float, *,
                         initial_height_mm: float = hw.GEOM_INITIAL_HEIGHT_MM,
                         hand_catch_offset_mm: float = hw.HAND_CATCH_OFFSET_MM,
                         hand_throw_offset_mm: float = HAND_THROW_OFFSET_MM,
                         max_tilt_deg: float = tilt_geometry.MAX_TILT_DEG
                         ) -> np.ndarray:
    """The VIRTUAL-TARGET lateral offset (2,) mm that makes
    :func:`compute_release_state_tilted` command exactly the aim tilt
    ``(aim_rx, aim_ry)``.

    THE aim → displacement conversion for contract C-TOSS-CAL-1
    (``plans/active/toss-selftuning.md`` D1): the toss aim map stores a
    *commanded tilt* in radians, and the only sanctioned way to command it is to
    displace the tilted path's target by this offset and let the existing,
    test-pinned Tier-8b math derive the tilt. Riding the shipped path is what
    makes "zero bias ⇒ today's machine" **provable** rather than argued, and it
    means there is no second implementation of the aim geometry for a sign to be
    wrong in (D2). Do not add another.

    ``catch_z_stow_mm`` is the goal's nominated platform z (STOW mm); both sites
    share it, so the offset is a function of the aim, the flight time and the
    height only — never of xy.

    Derivation, exact — no small-angle step. The cup axis is ``a =
    cup_axis(rx, ry)`` and the ball leaves ALONG it, so ``v = |v|·a``. The
    ballistic inverse gives ``v_z = Δz/T + g·T/2`` with ``Δz = catch_z −
    release_z`` and ``release_z`` the tilted release plane (``arm·(1 − a_z)``
    below the level one — the same ``_tilted_release_pos`` term). Eliminating
    ``|v|``::

        d = v_xy·T = (Δz + g·T²/2) · a_xy / a_z

    **Zero aim returns exactly ``[0.0, 0.0]``** (``a = (0, 0, 1)``), so the
    disabled path adds no floating-point operation at all.

    Verified against the production path by ``/tmp/probe_toss_cal_aim.py``
    (2026-08-11) over h ∈ {0.45, 0.60, 0.78, 1.00} m × |aim| ∈ {0, 0.05, 0.10,
    0.15, 0.5, 1.0}° × 5 azimuths × 3 goal poses: worst round-trip tilt error
    **8.35e-13 rad (4.8e-11°)**, i.e. the fixed-point pass in
    :func:`compute_release_state_tilted` closes on the requested aim to machine
    precision. The same probe pins the gain against the design's ``4h``
    approximation: 54.578 mm/deg at h = 0.78 m vs the idealised 54.454, a 0.23 %
    difference carried by the ``Δz`` and drop terms this form keeps exactly.
    ``tests/motion/test_toss_release.py`` re-runs the round trip at a 1e-9 rad
    gate.

    **D3 (2026-08-21) — that 54.578 is a SECANT, not the derivative.** It is the
    landing offset at a FULL 1° aim divided by 1°, which is the right quantity
    for "what does saturating the clamp buy" and is why ``toss_cal`` sizes
    ``TOTAL_MAX_RAD`` against it. The derivative at zero aim, which is what a
    Jacobian reports (``toss_trim.aim_landing_jacobian``,
    ``ilc_fit_lib.sensitivity``), is **54.5718 mm/deg** at exactly h = 0.78 m —
    smaller by a factor 1.0001044, i.e. ``tan(1°)/1°`` (1.0001016) plus the
    tilted-release drop. Both are correct and they are NOT interchangeable: a
    sizing argument about the clamp wants the secant, a linearised update law
    wants the derivative, and the exact rule behind both is ``dL/dθ = 4h + Δz``
    with ``Δz = 6.7360 mm`` (``ilc_fit_lib``'s header carries the canonical
    statement).
    """
    axis = tilt_geometry.cup_axis(float(aim_rx), float(aim_ry))
    a_z = float(axis[2])
    from_vertical_deg = float(np.degrees(np.arccos(np.clip(a_z, -1.0, 1.0))))
    if from_vertical_deg > float(max_tilt_deg):
        # Unreachable from a validated map (the aim authority is 1°, the ceiling
        # 12°), but the offset diverges as the axis approaches horizontal — a
        # 90° "aim" would produce a target ~1e17 mm away and a plausible-looking
        # pose command. Refuse at the SAME ceiling compute_release_state_tilted
        # enforces, rather than inventing a second one: an aim past that ceiling
        # has no sanctioned throw to be the target of.
        raise ValueError(
            "aim_target_offset_mm: aim ({:.6f}, {:.6f}) rad is {:.2f} deg from "
            "vertical, past the {:.2f} deg tilt ceiling — there is no "
            "sanctioned throw for it".format(
                float(aim_rx), float(aim_ry), from_vertical_deg,
                float(max_tilt_deg)))
    # Release plane at the nominated pose, level; the lever arm is taken there,
    # exactly as compute_release_state_tilted takes it.
    cup_z_world = (float(catch_z_stow_mm) + float(initial_height_mm)
                   + float(hand_throw_offset_mm))
    arm = tilt_geometry.cup_lever_arm_mm(cup_z_world)
    delta_z = (float(hand_catch_offset_mm) - float(hand_throw_offset_mm)
               + arm * (1.0 - a_z))
    T = float(flight_time_s)
    if T <= 0.0:
        raise ValueError(f"flight_time_s must be > 0 (got {T})")
    scale = (delta_z + 0.5 * ballistics_bc.GRAVITY_MMS2 * T * T) / a_z
    return np.array([scale * float(axis[0]), scale * float(axis[1])])


def validate_event_vel(event_vel_mps: float, *,
                       vmin: float = hw.TEENSY_TRAJ_MIN_EVENT_VEL_MPS,
                       vmax: float = hw.TEENSY_TRAJ_MAX_EVENT_VEL_MPS) -> bool:
    """True iff the teensy bridge will accept it: ``teensy_bridge_node``'s
    ``_svc_set_hand_traj`` raises outside [0.3, 7.0] m/s (bounds
    inclusive-accept, ``hw.TEENSY_TRAJ_MIN/MAX_EVENT_VEL_MPS``)."""
    return float(vmin) <= float(event_vel_mps) <= float(vmax)


def build_announcement_fields(release: ReleaseState, throw_time_s: float
                              ) -> dict:
    """The physics fields of the self-``ThrowAnnouncement`` (all positions mm
    GLOBAL, per ``ThrowAnnouncement.msg`` — ``landing_position`` is global).

    Tier 8b rides through unchanged: pass a :class:`TiltedReleaseState` and
    ``initial_position``/``initial_velocity`` carry the tilted release (with
    its lateral components) while ``landing_position`` is B's cup point — the
    announced landing IS the displaced B, which is exactly what the
    correlation → catch loop needs.

    ``throw_time_s`` is the ABSOLUTE ROS time (float s) of the release event
    = dispatch_time + event_delay (the bridge schedules the stroke event at
    now + event_delay); ``landing_time_s = throw_time_s + flight_time_s`` is
    the predicted cup-plane arrival on the same clock. The caller packs the
    ``builtin_interfaces/Time`` stamps and sets ``thrower_name='jugglebot'``,
    ``target_id='jugglebot'`` (target_id is MANDATORY: an empty target_id ⇒
    destination "" ⇒ never a catch candidate in ``ball_tracker_node``, and
    ``catch_coordinator``'s pre-tilt gate requires the strict match).

    Command→release latency is NOT modelled here — the BB-side 44 ms is a BB
    hand calibration; the JB hand's own latency is Phase-5 T0's measurand
    (``toss_release_latency_ms`` config slot, reserved at 0.0). Catch timing
    stays tracker-driven, so announced-time error is tolerated (±0.75 s arm
    window). Known bounded approximation: the tracker predicts crossings at
    its FIXED plane z = 809.08 mm; a nominated catch z off ACTIVE shifts the
    true cup plane by up to ±30 mm ⇒ timing skew Δt = Δz/|vz_land| ≈ 8 ms at
    3.9 m/s — negligible vs the window.
    """
    return dict(
        initial_position=release.release_pos_global_mm,
        initial_velocity=release.launch_vel_mms,
        predicted_tof_sec=release.flight_time_s,
        landing_position=release.catch_point_global_mm,
        landing_velocity=ballistics_bc.arrival_velocity(
            release.launch_vel_mms, release.flight_time_s),
        landing_time_s=float(throw_time_s) + release.flight_time_s,
    )
