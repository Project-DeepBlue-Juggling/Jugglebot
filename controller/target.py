"""Target-setting interface for the MPC controller.

Defines the ``TargetCommand`` data structure and the ``TargetSource`` protocol
that all input modes (spacemouse, GUI, catch coordinator) must satisfy.

The MPC solver consumes ``TargetCommand`` each control step:
    target_pose   → reference pose for the optimization
    arrival_time  → deadline (absolute time) or None for ASAP
    target_twist  → desired velocity at arrival (or None = hold)
    ref_events    → optional time-varying reference trajectory (list of events)

Every reference this module emits must satisfy the K1–K6 feasibility contract
documented in ``controller/REFERENCE_LAYER_CONTRACT.md``.  :func:`make_feasible_events`
is the canonical enforcement point; :func:`flat_target_to_events` is a thin
compat wrapper that delegates to it.

This module has NO sim-specific dependencies (no MuJoCo, no hand/ball types).
Sim-specific fields (hand_cmd, ball_spawn) are added by a subclass in
``sim/main.py``.  When ``controller/`` is extracted to a top-level package
(Phase 3F), this file moves with it unchanged.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Callable, List, Optional, Protocol, Tuple, runtime_checkable

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


def sample_ref_fn(
    ref_fn: Callable[[float], Tuple[np.ndarray, np.ndarray]],
    t_now: float,
    cumulative_times: np.ndarray,
) -> List[ReferenceEvent]:
    """Sample a reference callback at horizon node times to produce events.

    Convenience adapter: wraps an existing ``ref_fn(t) -> (pose, twist)``
    callable into a list of ``ReferenceEvent`` that the MPC can consume.

    Parameters
    ----------
    ref_fn : callable
        ``ref_fn(t_abs) -> (pose_6, twist_6)`` returning numpy arrays.
    t_now : float
        Current absolute time (same clock as ``PlantState.time``).
    cumulative_times : (N+1,) ndarray
        Relative times at each horizon node (from ``MPCParams.cumulative_times``).

    Returns
    -------
    list[ReferenceEvent]
        One event per horizon node, sorted by time.
    """
    events = []
    for dt in cumulative_times:
        t_abs = t_now + dt
        pose, twist = ref_fn(t_abs)
        events.append(ReferenceEvent(
            time=t_abs,
            pose=np.asarray(pose, dtype=np.float64),
            twist=np.asarray(twist, dtype=np.float64),
        ))
    return events


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


def compute_stretch_ms(
    proposed_events: List[ReferenceEvent],
    realised_events: List[ReferenceEvent],
) -> float:
    """Total time-shift (ms) between proposed and realised event lists.

    Used by ref-sources to compute the ``stretch_ms`` field for
    ``TargetFeedback`` messages.  Measures arrival-time delay at the final
    event; returns 0.0 when no stretch occurred.
    """
    if not proposed_events or not realised_events:
        return 0.0
    # Align by index — after K1 anchor the realised list may be one longer,
    # so compare the *last* event in each (the target's arrival).
    return max(0.0, (realised_events[-1].time - proposed_events[-1].time) * 1000.0)


@dataclass
class TargetCommand:
    """Command returned by a target source each MPC control step.

    Fields
    ------
    target_pose : (6,) ndarray
        Target platform pose [x, y, z, rx, ry, rz] in mm / rad.
    arrival_time : float or None
        Absolute time to arrive at the target.  ``None`` = converge ASAP.
    target_twist : (6,) ndarray or None
        Desired twist [vx, vy, vz, wx, wy, wz] at arrival.  ``None`` = zero
        (hold at target).
    ref_events : list[ReferenceEvent] or None
        Optional time-varying reference trajectory.  When provided, the MPC
        uses quintic Hermite interpolation (C2) between events that have
        acceleration, falling back to cubic Hermite (C1) otherwise.  Takes
        precedence over the flat-reference path (target_pose repeated at all
        nodes).  ``None`` = legacy flat-reference behavior.
    """
    target_pose: np.ndarray
    arrival_time: float | None = None
    target_twist: np.ndarray | None = None
    ref_events: List[ReferenceEvent] | None = None
    boost_vel_weights: bool = False
    warm_start_valid: bool = True
    """W5: source hint that the previous IPOPT warm-start may be structurally
    inapplicable to the new reference.  Sources set this False on target
    transitions that represent a large direction change or a distance jump
    (see ``is_warm_start_invalidating`` in target.py).  When False, the MPC
    invalidates ``_prev_w`` / ``_prev_lam_g`` / ``_prev_lam_x`` /
    ``_timeout_hint`` and cold-starts with W6's per-node-IK initial guess.
    The MPC is free to override this hint in either direction (e.g. on
    sustained consecutive failures).  See ``controller/REFERENCE_LAYER_CONTRACT.md``."""


# ---------------------------------------------------------------------------
# W5: warm-start invalidation heuristic
# ---------------------------------------------------------------------------

_WARM_INVALIDATE_POSE_DELTA_MM = 20.0
_WARM_INVALIDATE_VEL_REVERSAL_THRESHOLD_MMPS = 10.0


def is_warm_start_invalidating(
    prev_ref_horizon_end: np.ndarray | None,
    prev_ref_vel_horizon_end: np.ndarray | None,
    new_ref_horizon_end: np.ndarray,
    new_ref_vel_horizon_end: np.ndarray,
) -> bool:
    """W5 invalidation predicate.

    Returns True when the previous IPOPT warm-start is unlikely to be useful
    for the new reference — i.e. either:

    * ``‖new_ref[N] − prev_ref[N]‖_∞ > 20 mm`` on any axis (a structural
      position-landscape shift), or
    * sign flip in any linear velocity axis where the previous speed exceeded
      10 mm/s (ignore near-zero noise).

    Both criteria come from the Day-2 agreement.  The 10 mm/s threshold is a
    noise guard — without it, FK-noise around zero velocity would trip
    invalidation constantly.  The 20 mm threshold is Jetson-calibrated: any
    larger shift produces a >22 ms warm-start recomposition in IPOPT.
    """
    if prev_ref_horizon_end is None:
        return False  # no previous reference to compare to
    # Position shift on any linear axis
    pos_delta = np.abs(new_ref_horizon_end[:3] - prev_ref_horizon_end[:3])
    if np.max(pos_delta) > _WARM_INVALIDATE_POSE_DELTA_MM:
        return True
    # Velocity reversal on any linear axis
    if prev_ref_vel_horizon_end is not None:
        for i in range(3):
            prev_v = prev_ref_vel_horizon_end[i]
            new_v = new_ref_vel_horizon_end[i]
            if (abs(prev_v) > _WARM_INVALIDATE_VEL_REVERSAL_THRESHOLD_MMPS
                    and prev_v * new_v < 0):
                return True
    return False


@runtime_checkable
class TargetSource(Protocol):
    """Protocol for objects that provide MPC targets each control step.

    Implementations must provide ``update(sim_time, state) -> TargetCommand``.
    Optional lifecycle methods (``reset``, ``close``, ``key_callback``, etc.)
    are duck-typed by the control loop — they are NOT part of this protocol.

    Parameters
    ----------
    sim_time : float
        Current simulation / wall-clock time (seconds).
    state : PlantState (or compatible)
        Current plant state snapshot.  Typed as ``Any`` here to avoid a
        dependency on ``plant.interface`` — concrete implementations should
        type-narrow to ``PlantState``.

    Returns
    -------
    TargetCommand
        The MPC target for this control step.
    """
    def update(self, sim_time: float, state: Any) -> TargetCommand: ...


# ---------------------------------------------------------------------------
# Concrete TargetSource implementations (generic, no sim dependencies)
# ---------------------------------------------------------------------------

def pose_6dof_from_state(state: Any) -> np.ndarray:
    """Extract [x,y,z,rx,ry,rz] from a PlantState."""
    return np.concatenate([state.platform_pos_mm, state.platform_rot])


class StaticTargetSource:
    """Adapts a pose schedule (--pose / --sequence) to the TargetSource protocol.

    Caches the quintic reference events so the MPC sees a stable optimization
    landscape between target changes.  Without caching, rebuilding the quintic
    from current state every tick defeats IPOPT warm-starting and causes solver
    timeouts on constrained hardware (Jetson).
    """

    def __init__(self, schedule: List[Tuple[float, np.ndarray]],
                 clamp_start_twist_mmps: float | None = None,
                 v_max_mmps: float | None = None,
                 tau_s: float | None = None):
        self._schedule = schedule
        self._target = np.zeros(6)
        self._cached_events: List[ReferenceEvent] | None = None
        self._cached_target: np.ndarray | None = None
        self._clamp_twist = clamp_start_twist_mmps
        self._v_max_mmps = v_max_mmps
        self._tau_s = tau_s
        # W5: prev-reference tracking for warm-start invalidation hint.
        self._prev_ref_horizon_end_pose: np.ndarray | None = None
        self._prev_ref_horizon_end_twist: np.ndarray | None = None
        # W4b: pre-allocated TargetCommand.  ``update`` mutates the
        # fields in place and returns the same instance every tick —
        # consumers read synchronously, so aliasing is safe.  See
        # ``controller/HOT_LOOP_CONTRACT.md`` for the
        # "new TargetSource" template.
        self._tc = TargetCommand(
            target_pose=self._target,
            ref_events=None,
            boost_vel_weights=True,
            warm_start_valid=True,
        )

    def update(self, sim_time: float, state: Any) -> TargetCommand:
        # W4b: dead ``prev_target = self._target.copy()`` removed — the
        # variable was never read.  Schedule advance is now the only
        # work in the non-recompute fast path.
        for t, pose in self._schedule:
            if sim_time >= t:
                self._target = pose
            else:
                break

        # Regenerate quintic only when target changes or on first call
        target_changed = (self._cached_target is None
                          or not np.array_equal(self._target, self._cached_target))
        warm_start_valid = True
        if target_changed:
            self._cached_events = flat_target_to_events(
                pose_6dof_from_state(state), state.platform_twist,
                self._target, sim_time,
                clamp_start_twist_mmps=self._clamp_twist,
                v_max_mmps=self._v_max_mmps,
                tau_s=self._tau_s)
            self._cached_target = self._target.copy()
            # W5: structural-shift check against previous ref.
            new_end_pose = self._cached_events[-1].pose
            new_end_twist = self._cached_events[-1].twist
            if is_warm_start_invalidating(
                self._prev_ref_horizon_end_pose,
                self._prev_ref_horizon_end_twist,
                new_end_pose, new_end_twist,
            ):
                warm_start_valid = False
            self._prev_ref_horizon_end_pose = new_end_pose.copy()
            self._prev_ref_horizon_end_twist = new_end_twist.copy()

        # W4b: mutate the pre-allocated TargetCommand in place.
        self._tc.target_pose = self._target
        self._tc.ref_events = self._cached_events
        self._tc.boost_vel_weights = True
        self._tc.warm_start_valid = warm_start_valid
        return self._tc


class AutoSequenceTargetSource:
    """Drives the MPC through a fixed pose sequence, advancing only after each
    move has cleanly converged (tracking error and platform-twist magnitude
    both below threshold for ``settle_ticks`` consecutive ticks, then held for
    ``hold_s`` seconds).  Temporary scaffolding for reproducible leg-1 gain
    A/B tests — see logbook/2026-04-19-leg1-pose-dependent-hold-twitch.md.

    Call ``request_abort()`` to collapse the remaining sequence to STOW only;
    the source plans a fresh quintic from the current pose to ``STOW_POSE``
    and flags ``done=True`` once it has been reached and held.
    """

    STOW_POSE = np.zeros(6)

    def __init__(self, poses: List[np.ndarray],
                 pos_tol_mm: float = 1.0,
                 vel_tol_mmps: float = 5.0,
                 settle_ticks: int = 12,
                 hold_s: float = 1.5,
                 clamp_start_twist_mmps: float | None = None,
                 append_stow: bool = True,
                 v_max_mmps: float | None = None,
                 tau_s: float | None = None):
        self._poses: List[np.ndarray] = [np.asarray(p, dtype=float) for p in poses]
        if append_stow:
            self._poses.append(self.STOW_POSE.copy())
        self._idx = 0
        self._pos_tol = pos_tol_mm
        self._vel_tol = vel_tol_mmps
        self._settle_ticks = settle_ticks
        self._hold_s = hold_s
        self._clamp = clamp_start_twist_mmps
        self._v_max_mmps = v_max_mmps
        self._tau_s = tau_s

        self._settled = 0
        self._hold_until: float | None = None
        self._done = False
        self._cached_idx = -1
        self._cached_events: List[ReferenceEvent] | None = None
        # W5: prev-ref tracking for warm-start invalidation hint.
        self._prev_ref_horizon_end_pose: np.ndarray | None = None
        self._prev_ref_horizon_end_twist: np.ndarray | None = None
        # W4b: pre-allocated TargetCommand — see StaticTargetSource above.
        self._tc = TargetCommand(
            target_pose=self.STOW_POSE,
            ref_events=None,
            boost_vel_weights=True,
            warm_start_valid=True,
        )

    @property
    def done(self) -> bool:
        return self._done

    def request_abort(self) -> None:
        """Replace the rest of the sequence with a single STOW target."""
        print("AutoSequenceTargetSource: abort requested — stowing platform.")
        self._poses = [self.STOW_POSE.copy()]
        self._idx = 0
        self._settled = 0
        self._hold_until = None
        self._cached_idx = -1
        self._done = False

    def update(self, sim_time: float, state: Any) -> TargetCommand:
        if self._idx >= len(self._poses):
            self._done = True
            target = self.STOW_POSE.copy()
        else:
            target = self._poses[self._idx]
            if self._hold_until is None:
                pose = pose_6dof_from_state(state)
                err_mm = float(np.linalg.norm(pose[:3] - target[:3]))
                vel_mmps = float(np.linalg.norm(state.platform_twist[:3]))
                if err_mm < self._pos_tol and vel_mmps < self._vel_tol:
                    self._settled += 1
                    if self._settled >= self._settle_ticks:
                        self._hold_until = sim_time + self._hold_s
                        print(f"AutoSequenceTargetSource: pose "
                              f"{self._idx + 1}/{len(self._poses)} converged "
                              f"at t={sim_time:.2f}s "
                              f"(err={err_mm:.2f}mm vel={vel_mmps:.2f}mm/s) "
                              f"— holding {self._hold_s:.1f}s")
                else:
                    self._settled = 0
            elif sim_time >= self._hold_until:
                self._idx += 1
                self._settled = 0
                self._hold_until = None
                if self._idx < len(self._poses):
                    nxt = self._poses[self._idx]
                    print(f"AutoSequenceTargetSource: advancing to pose "
                          f"{self._idx + 1}/{len(self._poses)}: "
                          f"({nxt[0]:+.1f},{nxt[1]:+.1f},{nxt[2]:+.1f})")
                    target = nxt

        warm_start_valid = True
        if self._idx != self._cached_idx:
            self._cached_events = flat_target_to_events(
                pose_6dof_from_state(state), state.platform_twist,
                target, sim_time,
                clamp_start_twist_mmps=self._clamp,
                v_max_mmps=self._v_max_mmps,
                tau_s=self._tau_s)
            self._cached_idx = self._idx
            new_end_pose = self._cached_events[-1].pose
            new_end_twist = self._cached_events[-1].twist
            if is_warm_start_invalidating(
                self._prev_ref_horizon_end_pose,
                self._prev_ref_horizon_end_twist,
                new_end_pose, new_end_twist,
            ):
                warm_start_valid = False
            self._prev_ref_horizon_end_pose = new_end_pose.copy()
            self._prev_ref_horizon_end_twist = new_end_twist.copy()

        # W4b: mutate pre-allocated TargetCommand in place.
        self._tc.target_pose = target
        self._tc.ref_events = self._cached_events
        self._tc.boost_vel_weights = True
        self._tc.warm_start_valid = warm_start_valid
        return self._tc


class WaypointTargetSource:
    """Adapts a waypoint list (T1-T6) to the TargetSource protocol.

    Advances through waypoints as their arrival times are reached.
    Caches the quintic reference per waypoint for solver warm-start stability.
    """

    def __init__(self, waypoints: List[Tuple[np.ndarray, float]],
                 clamp_start_twist_mmps: float | None = None,
                 v_max_mmps: float | None = None,
                 tau_s: float | None = None):
        """
        Parameters
        ----------
        waypoints : list of (pose_6dof, arrival_time)
            Sorted by arrival_time.
        clamp_start_twist_mmps : float or None
            Legacy Bundle A kwarg — clips the FK-derived start twist that
            seeds each quintic.  Subsumed by K6 when ``v_max_mmps`` is set.
        v_max_mmps, tau_s : float or None
            When both provided, each waypoint is routed through
            ``make_feasible_events`` for K1–K6 enforcement
            (``controller/REFERENCE_LAYER_CONTRACT.md``).  Without them the legacy
            path is used and a deprecation warning is emitted.
        """
        self._waypoints = waypoints
        self._idx = 0
        self._cached_idx: int = -1
        self._cached_events: List[ReferenceEvent] | None = None
        self._clamp_twist = clamp_start_twist_mmps
        self._v_max_mmps = v_max_mmps
        self._tau_s = tau_s
        # W5: prev-ref tracking for warm-start invalidation hint.
        self._prev_ref_horizon_end_pose: np.ndarray | None = None
        self._prev_ref_horizon_end_twist: np.ndarray | None = None
        # W4b: pre-allocated TargetCommand — see StaticTargetSource above.
        _seed_pose = (waypoints[0][0] if waypoints
                      else np.zeros(6))
        self._tc = TargetCommand(
            target_pose=_seed_pose,
            arrival_time=None,
            ref_events=None,
            boost_vel_weights=True,
            warm_start_valid=True,
        )

    def update(self, sim_time: float, state: Any) -> TargetCommand:
        while (self._idx + 1 < len(self._waypoints)
               and sim_time >= self._waypoints[self._idx][1]):
            self._idx += 1
        pose, arrival = self._waypoints[self._idx]

        # Regenerate quintic only when waypoint index advances
        warm_start_valid = True
        if self._idx != self._cached_idx:
            self._cached_events = flat_target_to_events(
                pose_6dof_from_state(state), state.platform_twist,
                pose, sim_time, arrival_time=arrival,
                clamp_start_twist_mmps=self._clamp_twist,
                v_max_mmps=self._v_max_mmps,
                tau_s=self._tau_s)
            self._cached_idx = self._idx
            new_end_pose = self._cached_events[-1].pose
            new_end_twist = self._cached_events[-1].twist
            if is_warm_start_invalidating(
                self._prev_ref_horizon_end_pose,
                self._prev_ref_horizon_end_twist,
                new_end_pose, new_end_twist,
            ):
                warm_start_valid = False
            self._prev_ref_horizon_end_pose = new_end_pose.copy()
            self._prev_ref_horizon_end_twist = new_end_twist.copy()

        # W4b: mutate pre-allocated TargetCommand in place.
        self._tc.target_pose = pose
        self._tc.arrival_time = arrival
        self._tc.ref_events = self._cached_events
        self._tc.boost_vel_weights = True
        self._tc.warm_start_valid = warm_start_valid
        return self._tc
