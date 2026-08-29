"""The deferred A→B catch reach, asked BEFORE the ball leaves the hand.

WHAT THIS ANSWERS
-----------------
A Tier-8b displaced self-toss throws from A and catches at B. The platform is
NOT commanded to B before the throw — ``reload_coordinator_node``'s
``_toss_positioning_xyz`` pre-positions to the swing-compensated **pre-tilt pose
at A** for any tilted release, and the A→B translation is the *deferred* reach
that ``catch_coordinator`` commands at ``t_release``, with the ball already
airborne. So under 8b nothing that judges a commanded pose ever sees B: the
``go_to_pose`` feasibility gate judges the pose at A, and the FSM's surviving
pre-throw bounds are the z band (a scalar on B.z) and the closed-form |B−A|
reach bound (a scalar on the DISPLACEMENT). Neither bounds |B| itself.

That gap is real and was measured on this tree (2026-08-29): B = (250, 0) at
T = 0.80 s and B = (500, 0) at T = 1.00 s are both ADMITTED at CHECKING —
the closed-form bound is 256 mm at T = 0.80 and 500 mm at T = 1.00, so the
displacement gate passes them — and are then refused ``WORKSPACE`` by the
planner at ``t_release``, mid-flight. That is the C-REACH-1 mid-flight refusal
class: a miss, with the ball in the air and the catch dead.

This module closes it by asking the PLANNER the question, at cycle build, with
the goal still refusable. It is the production-faithful chain, not a model of
it: the same ``compute_release_state_tilted`` seed, the same receive tilt from
the arrival velocity, the same swing-compensated catch centroid
``catch_coordinator.predicted_catch_command`` produces, and the same
``planner.build_catch`` (which gates through ``feasibility.validate``) with the
flight as the lead.

ONE BODY, TWO CALLERS. ``tools/probes/displaced_reach_frontier.py`` mapped this
frontier first and its ``reach_verdict`` was the original of this code; the
probe now delegates here. A probe that measures a frontier the machine does not
gate on is worse than no probe, so the two share a body rather than a
convention.

COST, MEASURED (2026-08-29, this Jetson, throwaway ``/tmp/probe_reach_cost.py``
against :func:`catch_reach_feasible` itself at the default session limits,
median of 15 after one warm call):

  * ``A=(70,70) B=(-70,-70) T=0.95`` → ``OK``     **18.9 ms**  (worst measured:
    a refusal-free call pays the whole ``build_catch`` + ``validate`` pass)
  * ``A=(0,0)   B=(100,0)   T=0.80`` → ``OK``         17.6 ms
  * ``A=(0,0)   B=(0,0)     T=0.60`` → ``OK``         13.1 ms
  * ``A=(0,0)   B=(250,0)   T=0.80`` → ``WORKSPACE``   1.5 ms  (the IK/stroke
    half short-circuits before the sampled-limits pass)
  * ``A=(0,0)   B=(500,0)   T=1.00`` → ``WORKSPACE``   1.5 ms

**The FULL chain ships** — the audit's fallback (the IK + leg-stroke half only,
which is what actually returns ``WORKSPACE`` in those measurements) was not
needed: 18.9 ms is an eighth of the ~150 ms budget, and dropping the
sampled-limits pass would have cost the ``LIMIT_VEL``/``LIMIT_ACC``/
``LIMIT_JERK`` verdicts for nothing. The cost lands on ONE tick of
``_build_toss_cycle``, which runs once per cycle inside a blocking loop — and it
is small against what already sits there: the measured 0.16–0.54 s body overruns
come from the SYNCHRONOUS ``go_to_pose`` round trip in the same loop. That whole
class — "the FSM tick does blocking work" — is owned by *Unblocking the loop
from the positioning service round trip*, the named sibling change in
``plans/active/toss-multi-catch-pose.md`` § Explicit non-goals, which carries its
own evidence and its own logbook. It is not this gate's to solve, and this gate
does not move its needle.

The default geometry is a module singleton on purpose: ``feasibility``'s
``_WLIMITS_CACHE`` is keyed on ``id(geom)``, so a fresh ``StewartGeometry()``
per call would re-run its SVD and roughly double the numbers above.

Pure Python + numpy + the generated ``jugglebot.hardware_config`` constants —
no ROS2, no repo-root / ``controller`` imports (the ``motion/`` boundary rule).
"""

from __future__ import annotations

import math

import numpy as np

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory import planner
from jugglebot.motion.trajectory import tilt_geometry
from jugglebot.motion.trajectory import toss_release
from jugglebot.motion.trajectory.feasibility import TrajectoryInfeasible
from jugglebot.motion.trajectory.limits import TrajectoryLimits

#: The ballistics-side gravity — NEVER the tracker's 9810. Same value and same
#: reason as ``toss_sequencer.GRAVITY_MMS2`` and ``ballistics_bc``: the arrival
#: velocity this module derives has to be the one the release math produced.
GRAVITY_MMS2 = 9806.0

#: ``compute_release_state_tilted`` hit the tilt ceiling — a distinct verdict
#: from every ``feasibility`` code, because the refusal is about the AIM and not
#: about the reach. Production never sees it (the node's tilt-clamp branch has
#: already refused and passes no release here); the probe surveys it.
TILT_CLAMP = 'TILT_CLAMP'

#: The feasible verdict. Deliberately the same spelling as
#: ``feasibility.OK`` so a caller can compare either against this or against
#: the module it actually came from.
OK = 'OK'

_GEOM = None                    # lazily built singleton — see the docstring's
_LIMITS = None                  # last paragraph (the id(geom) cache)


def _default_geom():
    global _GEOM
    if _GEOM is None:
        _GEOM = StewartGeometry()
    return _GEOM


def _default_limits():
    global _LIMITS
    if _LIMITS is None:
        _LIMITS = TrajectoryLimits.from_config(hw)
    return _LIMITS


def _rest_state(pose6):
    """A plan seed at ``pose6`` with zero velocity and zero acceleration."""
    return (np.asarray(pose6, dtype=float), np.zeros(6), np.zeros(6))


def catch_reach_verdict(throw_site_xy, catch_xy, flight_s, *,
                        catch_z_mm=None, limits=None, geom=None,
                        settle_hold_s=None):
    """``(code, info)`` for the deferred A→B catch reach over ``flight_s``.

    ``code`` is :data:`OK`, :data:`TILT_CLAMP`, or the ``feasibility`` code the
    planner refused with (``WORKSPACE`` / ``UNREACHABLE`` / ``LIMIT_VEL`` /
    ``LIMIT_ACC`` / ``LIMIT_JERK`` / ``STEP_BOUND``). ``info`` carries the
    numbers behind it — the required throw tilt, the release speed, |B−A|, and
    (on ``OK``) the three sampled leg-space peaks.

    ``throw_site_xy`` is A and ``catch_xy`` is B, both STOW-frame xy in mm;
    ``catch_z_mm`` defaults to the ACTIVE plane. ``limits`` is the LIVE session
    ``TrajectoryLimits`` when the caller has one — the same reason the
    closed-form bound prefers the live triple: this gate fronts for the
    feasibility gate that will run at ``t_release``, so it must judge against
    what THAT gate will enforce, not against the YAML default.

    The AIM correction (C-TOSS-CAL-1) is deliberately NOT applied: it displaces
    the virtual target by ≤ 1° of tilt (≈ 1 mm of cup swing), it is a refinement
    and never a gate anywhere else in this pipeline, and judging reachability
    from the uncorrected release keeps this verdict a function of the goal the
    operator gave rather than of a calibration table that can be reloaded
    mid-session.
    """
    geom = _default_geom() if geom is None else geom
    limits = _default_limits() if limits is None else limits
    z = (float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM) if catch_z_mm is None
         else float(catch_z_mm))
    hold_s = (float(hw.JB_TRAJ_CATCH_SETTLE_HOLD_S) if settle_hold_s is None
              else float(settle_hold_s))
    t = float(flight_s)
    bx, by = float(catch_xy[0]), float(catch_xy[1])
    ax, ay = float(throw_site_xy[0]), float(throw_site_xy[1])
    try:
        rs = toss_release.compute_release_state_tilted(
            (bx, by, z), t, throw_site_xy_mm=(ax, ay))
    except toss_release.ThrowTiltInfeasible as exc:
        return TILT_CLAMP, {'required_deg': float(exc.required_deg)}
    # The ARRIVAL velocity as the tracker will see it: the launch velocity minus
    # g·T on z. `tilt_to_receive` then gives the catch policy's receive tilt, and
    # `cup_lateral_shift_mm` the swing that policy compensates — so the pose
    # planned here is the pose `catch_coordinator.predicted_catch_command`
    # publishes, not an idealised level one at B.
    v_arrival = np.asarray(rs.launch_vel_mms, dtype=float).copy()
    v_arrival[2] -= GRAVITY_MMS2 * t
    rx, ry = tilt_geometry.tilt_to_receive(v_arrival)
    shift = tilt_geometry.cup_lateral_shift_mm(
        rx, ry, cup_z_mm=float(rs.catch_point_global_mm[2]))
    catch_pose = np.array([bx - shift[0], by - shift[1], z, rx, ry, 0.0])
    info = {
        'throw_tilt_deg': math.degrees(math.hypot(rs.tilt_rx, rs.tilt_ry)),
        'event_vel_mps': float(rs.event_vel_mps),
        'displacement_mm': float(rs.displacement_mm),
    }
    try:
        _plan, report = planner.build_catch(
            _rest_state(rs.pretilt_pose_stow), catch_pose, t, limits, geom,
            settle_hold_s=hold_s, receive_tilt=(rx, ry))
    except TrajectoryInfeasible as exc:
        return exc.code, info
    info['peak_leg_vel_mmps'] = float(report.peak_leg_vel_mmps)
    info['peak_leg_acc_mmps2'] = float(report.peak_leg_acc_mmps2)
    info['peak_leg_jerk_mmps3'] = float(report.peak_leg_jerk_mmps3)
    return OK, info


def catch_reach_feasible(throw_site_xy, catch_xy, flight_s, **kwargs) -> str:
    """:func:`catch_reach_verdict`'s code alone — the PRODUCTION entry point.

    ``'OK'`` ⇒ the deferred reach plans; anything else is the verdict the
    planner would have produced at ``t_release`` with the ball airborne, minted
    now instead as a pre-throw refusal. Keyword arguments are
    :func:`catch_reach_verdict`'s.
    """
    return catch_reach_verdict(throw_site_xy, catch_xy, flight_s, **kwargs)[0]
