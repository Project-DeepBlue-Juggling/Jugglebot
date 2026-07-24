"""Toss release state — STOW→global conversion + vertical-toss ballistics (Tier 8a).

Pure numpy release-state math for the self-toss (``Toss.action``,
``plans/active/single-ball-toss.md`` Phase 1): where the ball leaves the hand,
the launch velocity that lands it back in the cup after ``flight_time_s``, and
the physics fields of the self-``ThrowAnnouncement`` that closes the existing
correlation → catch loop unchanged. Consumed by ``toss_sequencer`` /
``reload_coordinator_node``; siblings ``ballistics_bc`` (catch-side boundary
conditions) and ``tilt_geometry`` (receive tilt; Phase 4 adds the throw tilt).

**One conversion point per direction.** :func:`stow_to_global_mm` is THE single
tested STOW-relative → global conversion for the toss (plan § Frame convention);
the global → STOW producer already lives in
``catch_coordinator._compute_catch_command``. Do not add another.

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
