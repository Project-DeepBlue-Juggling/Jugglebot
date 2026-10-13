"""Reference-event layer for the K1-K6 feasibility contract.

Defines ``ReferenceEvent`` (a timed pose/twist/accel waypoint) and the
canonical enforcement point :func:`make_feasible_events`, with
:func:`flat_target_to_events` as a thin compat wrapper that delegates to it.
Every reference this module emits must satisfy the K1-K6 feasibility contract
documented in ``controller/REFERENCE_LAYER_CONTRACT.md``. As of 2026-10-13 the
enforcement point has no live caller outside its tests; ``ReferenceEvent`` is used
by ``sim/input/toss_loop.py``.

The target-SOURCE half (``TargetCommand``, the ``TargetSource`` protocol and
its static/auto-sequence/waypoint implementations, ``sample_ref_fn``,
``compute_stretch_ms``, ``is_warm_start_invalidating``) was deleted
2026-10-13 at the skill-stack R6 close-out.

This module has NO sim-specific dependencies (no MuJoCo, no hand/ball types).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, List, Optional, Tuple

import numpy as np

from .feasibility import segment_is_feasible

logger = logging.getLogger(__name__)


@dataclass
class ReferenceEvent:
    """A timed waypoint for the MPC reference trajectory.

    Used to build a time-varying reference that the MPC tracks across its
    horizon.  The MPC interpolates between events using quintic Hermite
    splines when acceleration is provided (C2-continuous), falling back to
    cubic Hermite (C1) when acceleration is ``None``.

    Parameters
    ----------
    time : float
        Absolute time (seconds, same clock as ``PlantState.time``).
    pose : (6,) ndarray
        Target platform pose [x, y, z, rx, ry, rz] in mm / rad.
    twist : (6,) ndarray or None
        Desired twist [vx, vy, vz, wx, wy, wz] in mm/s / rad/s at this
        event.  ``None`` is treated as zero (hold at pose).
    accel : (6,) ndarray or None
        Acceleration [ax, ay, az, alphax, alphay, alphaz] in mm/s² / rad/s²
        at this event.  When both bracketing events have ``accel``, the MPC
        uses quintic Hermite interpolation (C2-continuous).  ``None`` falls
        back to cubic Hermite (C1).
    """
    time: float
    pose: np.ndarray
    twist: np.ndarray | None = None
    accel: np.ndarray | None = None


_ZERO6 = np.zeros(6)


_DEFAULT_TAU_S = 0.04
_FLAT_NO_VMAX_WARNED = False

# Bumped manually whenever the K1–K6 *semantic* contract changes (event
# structure, anchor logic, stretch policy, etc.).  Hashed into the AOT NLP
# cache key by ``controller/mpc.py::_nlp_source_hash`` so that a semantic
# change forces an AOT rebuild while pure docstring/comment edits do not.
# See ``logbook/2026-04-19-bundle-a-mpc-overshoot-saturation-fix.md`` for
# the W9 rationale this constant replaces.
_REFERENCE_LAYER_VERSION = 1


def flat_target_to_events(
    current_pose: np.ndarray,
    current_twist: np.ndarray,
    target_pose: np.ndarray,
    t_now: float,
    response_time: float = 0.3,
    target_twist: np.ndarray | None = None,
    arrival_time: float | None = None,
    cruise_speed: float = 80.0,
    clamp_start_twist_mmps: float | None = None,
    v_max_mmps: float | None = None,
    tau_s: float | None = None,
    beta: float = 0.85,
    no_stretch: bool = False,
    return_reason: bool = False,
):
    """Convert a flat target into a K1–K6-compliant quintic reference.

    This is a thin compat wrapper around :func:`make_feasible_events`.  The
    canonical API for new callers is ``make_feasible_events`` directly — it
    accepts a full event list rather than just a target, and returns the
    rejection reason alongside the events.

    Behaviour:

    * When ``v_max_mmps`` and ``tau_s`` are both provided, the proposal is
      routed through ``make_feasible_events`` with the full K1–K6 contract
      (subsumes ``clamp_start_twist_mmps``).
    * When either is missing, the pre-K1–K6 behaviour is preserved with a
      one-shot ``DeprecationWarning``.  Bundle A's ``clamp_start_twist_mmps``
      still works on the legacy path.  This fallback is scheduled for removal
      once all call sites pass ``v_max_mmps``/``tau_s``.

    Parameters
    ----------
    current_pose, current_twist, target_pose, t_now, response_time,
    target_twist, arrival_time, cruise_speed
        Unchanged pre-Bundle-A kwargs.
    clamp_start_twist_mmps : float or None
        Bundle A legacy kwarg — element-wise clamp linear start-twist to
        ±value.  No-op on the K1–K6 path (subsumed by K6's β·v_max clamp).
    v_max_mmps : float or None
        Per-leg velocity limit in mm/s.  When provided, enables K1–K6.
        Typically ``MPCParams.max_leg_vel_mmps``.
    tau_s : float or None
        Actuator time constant in s.  Required alongside ``v_max_mmps`` for
        K3.  Typically ``MPCParams.tau``.
    beta : float
        Feasibility safety margin passed through to ``make_feasible_events``.
    no_stretch : bool
        Catch-path policy: reject rather than stretch infeasible segments.
        See ``controller/REFERENCE_LAYER_CONTRACT.md``.
    return_reason : bool
        When True, return ``(events, rejection_reason)``.  When False
        (default for backward compat), return just ``events``.

    Returns
    -------
    events : list[ReferenceEvent]
        Two events (start + arrival) — start anchored at ``(t_now, current)``,
        arrival at ``(t_arrival, target)`` — with any stretching already
        applied.  Degenerate cases (target at current) return a single hold
        event.
    rejection_reason : str or None
        Only returned when ``return_reason=True``.  None on success; non-None
        when ``no_stretch`` + infeasible, or when max_stretch_ratio exhausted.
    """
    global _FLAT_NO_VMAX_WARNED

    cur = np.asarray(current_pose, dtype=np.float64)
    tgt = np.asarray(target_pose, dtype=np.float64)
    tw_start = np.asarray(current_twist, dtype=np.float64).copy()
    if clamp_start_twist_mmps is not None:
        c = float(clamp_start_twist_mmps)
        tw_start[:3] = np.clip(tw_start[:3], -c, c)
    tw_end = np.asarray(target_twist if target_twist is not None else _ZERO6,
                        dtype=np.float64)

    # Compute arrival time (same rules as pre-K1–K6)
    if arrival_time is not None and arrival_time > t_now:
        t_arrival = float(arrival_time)
    else:
        dist = float(np.linalg.norm(tgt[:3] - cur[:3]))
        duration = max(response_time, dist / max(cruise_speed, 1.0))
        t_arrival = t_now + duration

    # Degenerate-arrival guard (preserves pre-K1–K6 behaviour: single hold event)
    if t_arrival - t_now < 0.05:
        events = [ReferenceEvent(
            time=t_now, pose=tgt.copy(), twist=tw_end.copy(),
            accel=_ZERO6.copy(),
        )]
        return (events, None) if return_reason else events

    proposal = [
        ReferenceEvent(time=t_now, pose=cur.copy(), twist=tw_start.copy(),
                       accel=_ZERO6.copy()),
        ReferenceEvent(time=t_arrival, pose=tgt.copy(), twist=tw_end.copy(),
                       accel=_ZERO6.copy()),
    ]

    if v_max_mmps is None or tau_s is None:
        # Legacy path — no K2/K3 peak-velocity enforcement.  Warn once so
        # call sites can migrate.  Bundle A's clamp_start_twist_mmps already
        # applied above.
        if not _FLAT_NO_VMAX_WARNED:
            import warnings
            warnings.warn(
                "flat_target_to_events called without v_max_mmps/tau_s — "
                "K2/K3 peak-velocity/acceleration bounds not enforced.  "
                "Migrate by passing v_max_mmps=params.max_leg_vel_mmps and "
                "tau_s=params.tau.",
                DeprecationWarning, stacklevel=2,
            )
            _FLAT_NO_VMAX_WARNED = True
        return (proposal, None) if return_reason else proposal

    events, reason = make_feasible_events(
        cur, tw_start, proposal, t_now,
        v_max_mmps=float(v_max_mmps), tau_s=float(tau_s),
        beta=beta, no_stretch=no_stretch,
    )
    return (events, reason) if return_reason else events


# ---------------------------------------------------------------------------
# K1–K6 reference-feasibility layer
# ---------------------------------------------------------------------------

# Default binary-search bounds for the stretch operation in
# ``make_feasible_events``.  Kept module-level (not function-default kwargs)
# so tests can patch them without juggling partial closures.
_STRETCH_BS_MAX_ITERS = 24      # ~ log2(16) * 3 — plenty for mm-precision
_STRETCH_BS_SAFETY_MARGIN = 0.98  # accept once peak < 0.98·limit (2% guard)
_K4_MIN_SPAN_S = 0.05           # minimum inter-event span after stretching


def _clamp_twist_in_place(twist: np.ndarray, clamp_mmps: float | None) -> None:
    """Element-wise clamp linear twist components to ±clamp_mmps.

    Angular components (indices 3..5) are untouched — see K6 rationale in
    ``controller/REFERENCE_LAYER_CONTRACT.md``.  ``None`` is a no-op.
    """
    if clamp_mmps is None:
        return
    c = float(clamp_mmps)
    twist[:3] = np.clip(twist[:3], -c, c)


def _stretch_segment_to_feasibility(
    p0: np.ndarray, v0: np.ndarray, a0: np.ndarray,
    p1: np.ndarray, v1: np.ndarray, a1: np.ndarray,
    T_req: float,
    v_max_mmps: float,
    a_max_mmps2: float,
    beta: float,
    max_ratio: float,
) -> tuple[float, str | None]:
    """Binary-search a stretched duration that satisfies K2 and K3.

    Returns ``(T_stretched, failure_reason)`` — ``failure_reason`` is None
    on success, otherwise a short string like ``"peak_vel_still_exceeds"``
    or ``"max_stretch_ratio_exhausted"``.  When feasible at ``T_req`` already,
    returns ``(T_req, None)`` without iterating.
    """
    feasible, vr, ar = segment_is_feasible(
        p0, v0, a0, p1, v1, a1, T_req, v_max_mmps, a_max_mmps2, beta=beta,
    )
    if feasible:
        return T_req, None

    T_lo = T_req
    T_hi = T_req * max_ratio
    # First check that the upper bound is feasible — otherwise no amount of
    # stretching will help (boundary conditions themselves infeasible).
    feasible_hi, _, _ = segment_is_feasible(
        p0, v0, a0, p1, v1, a1, T_hi, v_max_mmps, a_max_mmps2, beta=beta,
    )
    if not feasible_hi:
        # Boundary dominates (v0 or v1 too large).  K6 clamp should prevent
        # this; if we get here, the caller's boundary is already at-limit and
        # we need an even tighter margin.
        return T_hi, f"peak_exceeds_at_max_stretch_ratio (vr={vr:.2f} ar={ar:.2f})"

    # Binary search for the smallest T satisfying both bounds.
    for _ in range(_STRETCH_BS_MAX_ITERS):
        T_mid = 0.5 * (T_lo + T_hi)
        feas_mid, vr_mid, ar_mid = segment_is_feasible(
            p0, v0, a0, p1, v1, a1, T_mid, v_max_mmps, a_max_mmps2,
            beta=beta * _STRETCH_BS_SAFETY_MARGIN,
        )
        if feas_mid:
            T_hi = T_mid
        else:
            T_lo = T_mid
        if (T_hi - T_lo) / max(T_req, 1e-6) < 1e-3:
            break
    return T_hi, None


def make_feasible_events(
    current_pose: np.ndarray,
    current_twist: np.ndarray,
    events_proposal: List[ReferenceEvent],
    t_now: float,
    v_max_mmps: float,
    tau_s: float,
    *,
    no_stretch: bool = False,
    beta: float = 0.85,
    max_stretch_ratio: float = 4.0,
) -> Tuple[List[ReferenceEvent], Optional[str]]:
    """Enforce the K1–K6 reference-feasibility contract on a proposed event list.

    See ``controller/REFERENCE_LAYER_CONTRACT.md`` for the normative K1–K6 spec.

    Parameters
    ----------
    current_pose : (6,) ndarray
        Live platform pose at ``t_now``.
    current_twist : (6,) ndarray
        Live platform twist at ``t_now``.  Linear components are clamped to
        ±(beta · v_max_mmps) before use, per K6.
    events_proposal : list[ReferenceEvent]
        The caller's desired events, sorted ascending by time.  The first
        event may optionally be at exactly ``t_now`` with matching pose/twist;
        if not, a K1 anchor is prepended automatically.
    t_now : float
        Current absolute time (seconds).  Defines K1's anchor time.
    v_max_mmps : float
        Per-leg velocity limit.  Used for K2 (interior peak) and K6 (event
        endpoint clamp).  Typically from ``MPCParams.max_leg_vel_mmps``.
    tau_s : float
        Actuator time constant.  K3's acceleration bound is
        ``beta · v_max_mmps / tau_s`` — this approximates the actuator
        bandwidth derived from the first-order lag model used by the MPC.
    no_stretch : bool, default False
        When True, K2/K3 violations cause the function to return
        ``(events_proposal, <reason>)`` UNSTRETCHED.  The caller is
        responsible for acting on the rejection (e.g. hold at the last
        feasible pose).  This is the catch-path policy from
        ``controller/REFERENCE_LAYER_CONTRACT.md``.
    beta : float, default 0.85
        Safety margin applied to both K2 and K3 limits (fraction of v_max
        and a_max).  Values < 1 give IPOPT tracking headroom.
    max_stretch_ratio : float, default 4.0
        Ceiling on any single segment's stretch factor.  If the binary search
        cannot satisfy K2/K3 within this ratio, the segment is returned at
        the ratio-cap duration and a failure reason is reported.

    Returns
    -------
    events : list[ReferenceEvent]
        K1–K6-compliant events.  May have an extra K1-anchor event prepended,
        may have segment durations stretched, may have linear twists clamped.
    rejection_reason : str or None
        None on clean success.  Non-None when either:
          * ``no_stretch=True`` and a segment violated K2/K3, or
          * stretching hit ``max_stretch_ratio`` without finding feasibility
            (extremely rare — indicates caller boundary conditions are
            themselves infeasible even after K6 clamp).
        The string is short and machine-readable (e.g.
        ``"peak_vel_exceeds_no_stretch_allowed"``), suitable for logging or
        publishing via ``TargetFeedback``.

    Notes
    -----
    * K1 (kinematic continuity): first emitted event always at ``t_now`` with
      live pose/twist.
    * K2 (velocity bound): interior peak |v_i(t)| ≤ β · v_max for all t and
      linear axes i ∈ {0,1,2}.
    * K3 (acceleration bound): interior peak |a_i(t)| ≤ β · (v_max / τ).
    * K4 (monotonic time): inter-event spans ≥ 50 ms.
    * K5 (twist consistency): single ReferenceEvent per time stamp ensures
      no impulse at boundaries.  A caller that supplies two events at the
      same time with different twists triggers a ``ValueError``.
    * K6 (bounded twist values): linear twist clamped to ±(β · v_max) at
      each event; angular left alone.
    """
    if v_max_mmps <= 0 or tau_s <= 0:
        raise ValueError(
            f"make_feasible_events: v_max_mmps and tau_s must be positive "
            f"(got {v_max_mmps=}, {tau_s=})"
        )

    v_clamp = beta * v_max_mmps
    a_max = v_max_mmps / tau_s  # full limit; β applied inside segment check

    cur_pose = np.asarray(current_pose, dtype=np.float64)
    cur_twist = np.asarray(current_twist, dtype=np.float64).copy()
    _clamp_twist_in_place(cur_twist, v_clamp)

    if not events_proposal:
        # Caller provided nothing — emit a single K1 anchor (hold at current).
        return [ReferenceEvent(
            time=t_now, pose=cur_pose.copy(), twist=cur_twist.copy(),
            accel=_ZERO6.copy(),
        )], None

    # Copy and normalize: every event gets its own arrays, accel defaults to
    # zero, linear twist clamped to ±v_clamp.
    norm: List[ReferenceEvent] = []
    for ev in events_proposal:
        twist = (
            np.asarray(ev.twist, dtype=np.float64).copy()
            if ev.twist is not None else np.zeros(6)
        )
        _clamp_twist_in_place(twist, v_clamp)
        accel = (
            np.asarray(ev.accel, dtype=np.float64).copy()
            if ev.accel is not None else np.zeros(6)
        )
        norm.append(ReferenceEvent(
            time=float(ev.time),
            pose=np.asarray(ev.pose, dtype=np.float64).copy(),
            twist=twist, accel=accel,
        ))

    # K5 check: no two events at the same time with mismatched state.
    for i in range(1, len(norm)):
        if abs(norm[i].time - norm[i - 1].time) < 1e-9:
            if (not np.allclose(norm[i].pose, norm[i - 1].pose)
                    or not np.allclose(norm[i].twist, norm[i - 1].twist)):
                raise ValueError(
                    f"make_feasible_events: K5 violation — two events at "
                    f"t={norm[i].time:.6f} with inconsistent state"
                )

    # K1 anchor: prepend a (t_now, cur_pose, cur_twist) event unless the
    # first event is already within tolerance of that state.
    first = norm[0]
    anchor_needed = (
        abs(first.time - t_now) > 1e-6
        or not np.allclose(first.pose, cur_pose, atol=1e-6)
        or not np.allclose(first.twist, cur_twist, atol=1e-6)
    )
    if anchor_needed:
        # If the proposal's first event is earlier than t_now, drop it
        # (stale) and anchor at t_now.
        while norm and norm[0].time < t_now + 1e-6:
            norm.pop(0)
        norm.insert(0, ReferenceEvent(
            time=t_now, pose=cur_pose.copy(), twist=cur_twist.copy(),
            accel=_ZERO6.copy(),
        ))

    # K4: enforce minimum inter-event span.  If a proposed event is within
    # 50 ms of its predecessor, drop it (merging).
    pruned: List[ReferenceEvent] = [norm[0]]
    for ev in norm[1:]:
        if ev.time - pruned[-1].time < _K4_MIN_SPAN_S:
            logger.debug(
                "make_feasible_events: dropping event at t=%.3f (K4 span %.3fs < %.3fs)",
                ev.time, ev.time - pruned[-1].time, _K4_MIN_SPAN_S,
            )
            continue
        pruned.append(ev)
    norm = pruned

    if len(norm) < 2:
        # Single K1-anchor only: trivially K1–K6 compliant (no interior).
        return norm, None

    # K2/K3: scan consecutive pairs, stretching each as needed.  Stretching
    # a segment cascades all *later* event times by the delta so the sequence
    # stays monotone.
    rejection_reason: Optional[str] = None
    i = 0
    while i + 1 < len(norm):
        e0, e1 = norm[i], norm[i + 1]
        T = e1.time - e0.time
        feas, vr, ar = segment_is_feasible(
            e0.pose, e0.twist, e0.accel,
            e1.pose, e1.twist, e1.accel,
            T, v_max_mmps, a_max, beta=beta,
        )
        if feas:
            i += 1
            continue

        if no_stretch:
            # Catch-path contract: refuse to silently alter timing.
            reason = (
                f"peak_exceeds_no_stretch_allowed "
                f"(segment {i}: vr={vr:.2f} ar={ar:.2f})"
            )
            logger.info("make_feasible_events: %s", reason)
            return norm, reason

        T_new, stretch_fail = _stretch_segment_to_feasibility(
            e0.pose, e0.twist, e0.accel,
            e1.pose, e1.twist, e1.accel,
            T, v_max_mmps, a_max, beta=beta, max_ratio=max_stretch_ratio,
        )
        delta = T_new - T
        if delta > 0:
            new_time = e0.time + T_new
            # Cascade: shift this event and all subsequent times by delta
            for j in range(i + 1, len(norm)):
                norm[j] = ReferenceEvent(
                    time=norm[j].time + delta,
                    pose=norm[j].pose, twist=norm[j].twist, accel=norm[j].accel,
                )
            # Check stretch-warning threshold (>25% of requested duration)
            if delta / T > 0.25:
                logger.warning(
                    "make_feasible_events: stretched segment %d by %.1f%% "
                    "(%.3fs → %.3fs) for K2/K3",
                    i, 100.0 * delta / T, T, T_new,
                )
        if stretch_fail is not None and rejection_reason is None:
            rejection_reason = stretch_fail
            logger.warning(
                "make_feasible_events: segment %d could not be made feasible "
                "within max_stretch_ratio — %s",
                i, stretch_fail,
            )
        i += 1

    return norm, rejection_reason


def pose_6dof_from_state(state: Any) -> np.ndarray:
    """Extract [x,y,z,rx,ry,rz] from a PlantState."""
    return np.concatenate([state.platform_pos_mm, state.platform_rot])
