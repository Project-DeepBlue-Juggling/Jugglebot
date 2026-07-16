"""Kinetics-aware lean shaping — Phase 4 (ships ON: JB_TRAJ_LEAN_GAIN = 0.6 since 2026-07-17).

A translation move accelerates the platform; a ball resting in the cup feels an
inertial pseudo-force opposing that acceleration. Leaning the cup *into* the
acceleration keeps the ball seated: the cup's up-axis is tilted toward the
effective-gravity-plus-inertia direction ``(a_x, a_y, g)`` so the in-cup lateral
force is reduced. This module superposes that lean tilt on a base
:class:`~jugglebot.motion.trajectory.plan.TrajectoryPlan`.

The lean is a **refinement**, not a smoothness mechanism — the always-on
smoothness guarantee is the feasibility gate's duration stretch. The Phase-4
hardware A/B + gain sweep (2026-07-16) resolved **KEEP**: measured leg jerk
dropped −30 %/−23 % at gains 0.3/0.6 and the operator confirmed the motion
calmer, so lean ships ON at ``JB_TRAJ_LEAN_GAIN = 0.6`` since 2026-07-17 (see
``logbook/2026-07-17-s4-closed-working-point-persisted.md``; an explicit
``lean_gain: 0.0`` in a request still forces OFF). It is capped at
``LEAN_TILT_CAP_DEG`` (5°) and always re-gated
(``shaping runs BEFORE validate`` — the gate always sees the shaped plan).

Sign convention (walked through physically, pinned by ``tests/motion/test_trajectory_shaping.py``)
------------------------------------------------------------------------------
For base lateral acceleration ``a = (a_x, a_y)`` (mm/s², the analytic quintic
accel of the base plan) and gain ``k``:

  * ``rx(t) += -k · a_y(t) / g``   (tilt about world +x)
  * ``ry(t) += +k · a_x(t) / g``   (tilt about world +y)

A rotation of ``+ry`` swings the cup up-axis toward ``+x``; a rotation of ``-rx``
swings it toward ``+y`` (right-hand rule on the ``[rx, ry, rz]`` rotvec). So the
signs above point the cup axis toward ``(a_x, a_y, +g)`` — i.e. it leans into the
acceleration, which is exactly what nulls the in-cup lateral pseudo-force.

Lever-arm compensation (cup-height-derived; constants ``juggle_tilt.py:59-76``, functions + ``realize_tilted`` ``:159-209``)
------------------------------------------------------------------------------
The cup opening sits a height ``arm = cup_z − CUP_TILT_CENTER_Z_MM`` **above** the
tilt centre, so a tilt of ``(rx, ry)`` swings the opening sideways by
``arm · cup_axis_xy(rx, ry)`` (the reference's ``cup_lateral_shift_mm``). To keep
the *cup* on the intended translation path, the platform **centroid** is offset by
the negative of that swing (``centroid_xy = target_xy − shift``, exactly the
reference's ``realize_tilted``). At the lean operating point the arm is
``+95.1 mm`` and the single-axis lever is ``LEVER_ARM_MM_PER_DEG = 1.66`` mm of
cup travel per degree — positive, and reproduced from the cup height rather than
hard-coded (pinned by the geometry test). The height-aware **per-pose** arm (arm
scales with the commanded z + hand slider) is the Phase-6 ``tilt_geometry.py``
port's job; lean tilt is ≤5° so the fixed-arm residual here is second order.

Small-angle closed form
-----------------------
For small ``(rx, ry)``, ``cup_axis_xy ≈ (ry, −rx)``, so the compensation shift
collapses to ``shift = (arm · k / g) · a`` — proportional to the acceleration
itself. Both the tilt and its compensation are therefore analytic functions of
the base quintic's accel/jerk/snap, so the shaped plan returns a fully
self-consistent ``(pose, twist, accel)`` at every sample (no pose↔velocity
feedforward mismatch — the emitter's ``vel_mm_s`` matches the position-knot
slope). The lean **vanishes in position** at both segment ends (quintic boundary
accel is zero); a bounded tilt-*rate* transient remains at the ends (the base
quintic's boundary jerk is nonzero), which the gate measures as leg velocity and
bounds — see the Phase-4 logbook Discussion.

Pure Python + numpy. No ROS2 / repo-root / ``controller`` imports.
"""

from __future__ import annotations

import math

import numpy as np

from jugglebot.motion.trajectory.plan import TrajectoryPlan
from jugglebot.motion.trajectory.segment import POSE_DIM, QuinticSegment

# ── Ported lever-arm geometry (Jugglebot-bb/sim/juggle_tilt.py:59-76) ──────────
# World z (mm) of the tilt rotation centre — the point the cup opening swings ABOUT
# under a platform tilt (measured constant across the slider range in the Rung-0
# characterisation). The lever arm is cup_z − this, so it SCALES with cup height.
CUP_TILT_CENTER_Z_MM = 744.3
# Nominal world cup height (mm) at the lean operating point. juggle_tilt's Rung-0:
# at the 180 mm slider the cup rides ~840 mm, giving arm ≈ +95 mm and reproducing
# the reported 1.66 mm/deg. The lean shaper uses this fixed nominal arm; the
# per-pose height-aware arm is the Phase-6 tilt_geometry.py port's job.
LEAN_CUP_Z_MM = 839.4
# Rung-0's reported single-axis lever (mm lateral cup shift per degree of tilt),
# POSITIVE. Kept as the cross-check the geometry test pins the ported arm against;
# the shaper itself uses the height-derived arm, not this constant.
LEVER_ARM_MM_PER_DEG = 1.66

# Default gravity (mm/s²) — the production node passes hw.GRAVITY_MMPS2; this
# module-level default keeps shaping.py importable/testable standalone.
G_MM_S2_DEFAULT = 9806.0

# Hard cap on the ADDED lean tilt from vertical. A safety clamp on the lean itself,
# independent of the leg-limit gate (which separately bounds leg vel/acc/jerk).
LEAN_TILT_CAP_DEG = 5.0

# ── Lean plateau window (kills the boundary vel_ff step) ───────────────────────
# The lean tilt is ∝ the base quintic's accel and its RATE ∝ the base jerk; a
# rest-to-rest quintic has NONZERO boundary jerk (j(0)=j(T)=60·d/T³), so the raw
# lean tilt_d STEPS at every move boundary → an emitted leg vel_ff step of ~70-180
# mm/s (confirmed in the ramp-battery bags: peak iq rose 5.93→8.48 A at gain 0.3;
# this is the operator's "sharp change at the preparatory tilt"). Multiplying the
# ENTIRE lean contribution (tilt + lever-arm shift + all their time derivatives) by
# a plateau window w(s), s = tl/T within the shaped segment, that is C2-zero at the
# boundaries removes the vel_ff step AND its accel-spike cousin while leaving the
# lean UNTOUCHED across the plateau where the accel peak (s≈0.211) lives.
#
#   w(s) = S(s/EDGE)·S((1−s)/EDGE),  S = the C2 quintic smoothstep 6u⁵−15u⁴+10u³
#          clamped to [0,1].
#     • w(0)=w(1)=0, w'(0)=w'(1)=0, w''(0)=w''(1)=0  (C2 → no vel_ff/accel STEP)
#     • w ≡ 1 for s∈[EDGE, 1−EDGE]                    (lean untouched at the peak)
#
# EDGE=0.15 keeps the base-plan accel peak (s≈0.211, fixed for a rest-to-rest
# quintic) comfortably inside the [0.15, 0.85] plateau, while confining the ramp to
# the near-rest first/last 15% of the move where the base lateral accel — and hence
# the lean it drives — is small anyway. The window is applied in ``state_at`` only;
# the shaping-before-validate ordering is untouched, so the gate still sizes the
# windowed leg peaks.
# NB: w is per-SEGMENT (s = tl/T_seg). Today shaped plans are single-segment
# rest-to-rest moves only (build_move), so segment ends == plan ends. A future
# MULTI-segment shaped plan (move chaining/supersede) needs a plan-scoped window,
# or the lean will dip to zero at every interior join — C2-smooth and gate-sized,
# but behaviorally wrong for chaining.
LEAN_WINDOW_EDGE_FRAC = 0.15


def _smoothstep(u: float):
    """C2 quintic smoothstep ``S(u)=6u⁵−15u⁴+10u³`` clamped to [0,1].

    Returns ``(S, dS/du, d²S/du²)`` in the clamped sense (flat, all-zero
    derivatives outside [0,1]). ``S'(u)=30u²(u−1)²`` and ``S''(u)=60u(2u−1)(u−1)``
    both vanish at ``u=0`` and ``u=1`` — the C2 property that kills the boundary
    velocity step and its acceleration-spike cousin.
    """
    if u <= 0.0:
        return 0.0, 0.0, 0.0
    if u >= 1.0:
        return 1.0, 0.0, 0.0
    s = u * u * u * (10.0 + u * (-15.0 + 6.0 * u))
    ds = 30.0 * u * u * (u * (u - 2.0) + 1.0)        # 30u²(u−1)²
    dds = 60.0 * u * (u * (2.0 * u - 3.0) + 1.0)     # 60u(2u−1)(u−1)
    return s, ds, dds


def _lean_window(s: float, edge: float = LEAN_WINDOW_EDGE_FRAC):
    """Plateau window ``w(s)`` and its s-derivatives ``(w, w', w'')`` for s∈[0,1].

    ``w(s) = S(s/edge)·S((1−s)/edge)``. Both smoothstep factors are chained to ``s``
    (``d(s/edge)/ds = +1/edge``, ``d((1−s)/edge)/ds = −1/edge``) and combined with the
    product rule so ``w''`` is the exact second s-derivative. Time derivatives are
    obtained downstream via ``d/dt = (1/T) d/ds`` (see :meth:`_ShapedPlan.state_at`).
    """
    ie = 1.0 / edge
    L, dL, ddL = _smoothstep(s * ie)             # derivatives w.r.t. s/edge
    R, dR, ddR = _smoothstep((1.0 - s) * ie)     # derivatives w.r.t. (1−s)/edge
    Ls, Lss = dL * ie, ddL * ie * ie             # chain to s (+1/edge)
    Rs, Rss = -dR * ie, ddR * ie * ie            # chain to s (−1/edge; sign² = +)
    w = L * R
    ws = Ls * R + L * Rs
    wss = Lss * R + 2.0 * Ls * Rs + L * Rss
    return w, ws, wss


# Float-noise tolerance on the segment→hold boundary in ``_locate``. The gate's dense
# grid samples the final point at ``seg.duration·(n−1)/(n−1)``, which floating-point
# rounding can land ~1 ULP ABOVE ``total_duration`` (e.g. a stretch-loop duration of
# 0.7800000000000002 s). A strict ``t > total_duration`` hold test would then push
# that endpoint into the terminal hold — the exact seam-spike this module fixes —
# only for the durations that happen to round up. Absorbing ≤1 ULP (this ≫ ULP,
# ≪ the 25 ms knot spacing) makes the boundary robust for ALL durations while keeping
# the terminal hold (emitter sampling well past T) unaffected.
_SEG_END_EPS_S = 1e-9


def cup_lever_arm_mm(cup_z_mm: float = LEAN_CUP_Z_MM) -> float:
    """Signed lever arm (mm) of the cup opening above the tilt centre.

    ``cup_z − CUP_TILT_CENTER_Z_MM``: positive when the cup rides above the centre
    (swings WITH +ry toward +x), negative below. Ported from ``juggle_tilt.py``.
    """
    return float(cup_z_mm) - CUP_TILT_CENTER_Z_MM


def _cup_axis_xy(rx: float, ry: float) -> np.ndarray:
    """xy of the world-frame cup up-axis for tilt ``(rx, ry)`` rad (rz = 0).

    Exact (not small-angle): the rotvec ``[rx, ry, 0]`` applied to world +z. Used
    by :func:`cup_lateral_shift_mm` so the geometry test pins the ported lever arm
    against the exact ``juggle_tilt.py`` form. (The shaper's per-sample derivatives
    use the small-angle collapse — see the module docstring.)
    """
    theta = math.hypot(rx, ry)
    if theta < 1e-12:
        return np.zeros(2)
    # Rodrigues on +z: R·ẑ = ẑ·cosθ + (k×ẑ)·sinθ + k(k·ẑ)(1−cosθ), k = axis/θ.
    kx, ky = rx / theta, ry / theta
    s = math.sin(theta)
    # (k×ẑ)_xy = (ky, −kx); k·ẑ = 0 (k has no z component) ⇒ no (1−cosθ) xy term.
    return np.array([ky * s, -kx * s])


def cup_lateral_shift_mm(rx: float, ry: float,
                         cup_z_mm: float = LEAN_CUP_Z_MM) -> np.ndarray:
    """Lateral (xy) shift (mm) of the cup opening under tilt ``(rx, ry)`` rad.

    ``arm · cup_axis_xy(rx, ry)`` — the exact ``juggle_tilt.py`` swing. Rung-0 sign
    check (arm > 0): ``+ry`` swings ``+x``, ``+rx`` swings ``−y``.
    """
    return cup_lever_arm_mm(cup_z_mm) * _cup_axis_xy(rx, ry)


def _seg_xy_derivs(seg: QuinticSegment, tl: float):
    """Analytic (accel, jerk, snap) of the base quintic's x/y axes at local time.

    Returns three ``(2,)`` arrays ``[x, y]``. The lean tilt is ``∝ accel``, its rate
    ``∝ jerk``, its curvature ``∝ snap`` — all closed-form from the quintic Hermite
    coefficients (the boundary conditions the segment already stores), so the shaped
    plan is analytically self-consistent (no finite-difference noise).
    """
    T = seg.duration
    s = min(max(tl / T, 0.0), 1.0)
    idx = np.array([0, 1])
    p0 = seg.p0[idx]
    p1 = seg.p1[idx]
    V0 = seg.v0[idx] * T
    A0 = seg.a0[idx] * T * T
    V1 = seg.v1[idx] * T
    A1 = seg.a1[idx] * T * T
    # Quintic Hermite polynomial coefficients in normalised time s (verified against
    # QuinticSegment.eval in tests/motion/test_trajectory_shaping.py).
    c2 = 0.5 * A0
    c3 = -10 * p0 - 6 * V0 - 1.5 * A0 + 10 * p1 - 4 * V1 + 0.5 * A1
    c4 = 15 * p0 + 8 * V0 + 1.5 * A0 - 15 * p1 + 7 * V1 - 1.0 * A1
    c5 = -6 * p0 - 3 * V0 - 0.5 * A0 + 6 * p1 - 3 * V1 + 0.5 * A1
    accel = (2 * c2 + 6 * c3 * s + 12 * c4 * s ** 2 + 20 * c5 * s ** 3) / T ** 2
    jerk = (6 * c3 + 24 * c4 * s + 60 * c5 * s ** 2) / T ** 3
    snap = (24 * c4 + 120 * c5 * s) / T ** 4
    return accel, jerk, snap


class _ShapedPlan(TrajectoryPlan):
    """A base plan with lean tilt + lever-arm xy compensation superposed.

    Subclasses :class:`TrajectoryPlan` (so it is isinstance-compatible and reuses
    the segment/starts/total_duration bookkeeping) and overrides :meth:`state_at`
    to add the analytic lean contribution. Reached ONLY through the analytic
    :func:`feasibility.validate` gate (``planner.build_move``); the terminal hold
    and any ``HoldPlan`` carry no lean (their accel is zero).
    """

    def __init__(self, base: TrajectoryPlan, gain: float, *,
                 arm_mm: float, g_mm_s2: float, tilt_cap_rad: float):
        super().__init__(segments=base.segments, final_pose=base.final_pose)
        self._gain = float(gain)
        self._arm = float(arm_mm)
        self._g = float(g_mm_s2)
        self._cap = float(tilt_cap_rad)

    def _locate(self, t: float):
        """Active segment + local time for ``t``, or ``(None, 0.0)`` for the hold.

        The hold branch is deferred to just past the segment end (``t > T + ε``, not
        ``t >= T``): at ``t = T`` (± float noise) the shaped SEGMENT-END state must be
        returned, not the terminal hold. The base quintic's boundary accel is zero, so
        tilt(T) ∝ a(T) = 0 and the shaped pose at T equals ``final_pose`` exactly — but
        the boundary JERK is nonzero, so the shaped twist/accel at T carry the genuine
        tilt_d/tilt_dd content. Returning the hold (zero twist/accel) at the gate's
        final grid sample made the jerk finite-difference fabricate a seam spike (the
        plan C0-steps in accel at T−dt→T), which the stretch loop then "fixed" by
        inflating shaped lateral moves ~5× past the honest minimum. The ``ε`` absorbs
        the ≤1-ULP overshoot of the gate's ``seg.duration·(n−1)/(n−1)`` endpoint (see
        :data:`_SEG_END_EPS_S`) so EVERY candidate duration — not just the ones that
        round at/below T — sees the continuous boundary content and the gate bounds
        the real tilt-rate transient honestly.
        """
        if not self.segments or t > self.total_duration + _SEG_END_EPS_S:
            return None, 0.0
        idx = 0
        for i, start in enumerate(self._starts):
            if t >= start:
                idx = i
            else:
                break
        return self.segments[idx], float(t) - self._starts[idx]

    def state_at(self, t: float):
        pose, twist, accel = super().state_at(t)
        seg, tl = self._locate(float(t))
        if seg is None or self._gain == 0.0:
            return pose, twist, accel

        a, j, s = _seg_xy_derivs(seg, tl)           # each [ax, ay]
        kg = self._gain / self._g
        # Added lean tilt and its time derivatives (rx uses −a_y, ry uses +a_x).
        tilt = kg * np.array([-a[1], a[0]])          # [rx, ry]
        tilt_d = kg * np.array([-j[1], j[0]])
        tilt_dd = kg * np.array([-s[1], s[0]])
        # Lever-arm compensation: shift = arm·cup_axis_xy ≈ arm·(ry, −rx) collapses
        # to shift = (arm·k/g)·a. Subtract from the xy translation (centroid pulls
        # back so the CUP stays on the intended path).
        ck = self._arm * kg
        shift = ck * a[:2]
        shift_d = ck * j[:2]
        shift_dd = ck * s[:2]

        # Plateau window: multiply the ENTIRE lean contribution (tilt + shift + all
        # their time derivatives) by w(s), s = tl/T, C2-zero at the segment ends so
        # the emitted vel_ff/accel no longer STEP at a move boundary (the base
        # quintic's boundary jerk is nonzero, so raw tilt_d/shift_d step). w is a
        # pure function of s; its time derivatives are w'/T and w''/T² (d/dt =
        # (1/T)d/ds). Apply the full product rule so the windowed twist/accel remain
        # the EXACT analytic derivatives of the windowed pose (the self-consistency
        # invariant this module guards — a mismatch reintroduces the FF-bug class).
        T = seg.duration
        sn = min(max(tl / T, 0.0), 1.0)
        w, ws, wss = _lean_window(sn)
        w_d = ws / T                                 # dw/dt
        w_dd = wss / (T * T)                          # d²w/dt²
        tilt_dd = w_dd * tilt + 2.0 * w_d * tilt_d + w * tilt_dd
        tilt_d = w_d * tilt + w * tilt_d
        tilt = w * tilt
        shift_dd = w_dd * shift + 2.0 * w_d * shift_d + w * shift_dd
        shift_d = w_d * shift + w * shift_d
        shift = w * shift

        # Hard cap on the added-tilt magnitude, applied to the WINDOWED tilt (window
        # first, then cap). Because w ≤ 1 the windowing can only shrink the tilt, so
        # the cap binds no more often than it would unwindowed. When it binds, scale
        # the whole lean contribution (tilt + compensation + their derivatives) by the
        # same instantaneous factor. NOTE: scaling the derivatives by the instantaneous
        # factor makes the capped twist/accel no longer the exact analytic derivative
        # of the capped pose (a small FF inconsistency), but this is a SAFETY CLAMP
        # that never binds at the A/B operating gain (0.3 needs base lateral accel
        # ≈ cap·g/gain ≈ 2.85 m/s², far above the ≤5° tilts the ramp battery reaches)
        # — so the inconsistency lives only in a regime that is never commanded. See
        # the Phase-4 logbook Discussion; make the derivative tangential-exact only if
        # a future session runs a gain/cap where this binds.
        mag = float(np.hypot(tilt[0], tilt[1]))
        if mag > self._cap:
            scale = self._cap / mag
            tilt *= scale; tilt_d *= scale; tilt_dd *= scale
            shift *= scale; shift_d *= scale; shift_dd *= scale

        pose = np.asarray(pose, dtype=float).copy()
        twist = np.asarray(twist, dtype=float).copy()
        accel = np.asarray(accel, dtype=float).copy()
        pose[0:2] -= shift;    pose[3:5] += tilt
        twist[0:2] -= shift_d; twist[3:5] += tilt_d
        accel[0:2] -= shift_dd; accel[3:5] += tilt_dd
        return pose, twist, accel


class LeanShaper:
    """Wraps a base plan with lean shaping (identity when the gain is ≤ 0).

    Constructed by ``trajectory_node`` from the config gain (or a per-call
    ``GoToPose`` override) and ``hw.GRAVITY_MMPS2``; passed to
    ``planner.build_move`` so the gate validates the SHAPED plan.
    """

    def __init__(self, gain: float, *,
                 arm_mm: float | None = None,
                 g_mm_s2: float = G_MM_S2_DEFAULT,
                 tilt_cap_deg: float = LEAN_TILT_CAP_DEG):
        self.gain = max(0.0, float(gain))
        self.arm_mm = cup_lever_arm_mm() if arm_mm is None else float(arm_mm)
        self.g_mm_s2 = float(g_mm_s2)
        self.tilt_cap_rad = math.radians(float(tilt_cap_deg))

    def shape(self, plan: TrajectoryPlan) -> TrajectoryPlan:
        """Return the shaped plan, or ``plan`` unchanged when the gain is ≤ 0.

        A zero-segment ``HoldPlan`` is returned unchanged too (nothing to lean).

        A shaped plan is gated **only** by the analytic
        :func:`~jugglebot.motion.trajectory.feasibility.validate` (the service /
        ``build_move`` path). It must NEVER be handed to the fast
        :func:`~jugglebot.motion.trajectory.feasibility.validate_follow`: that gate's
        finite differences measure the base quintic's leg peaks, not the lean
        superposition, so it would silently under-gate the added tilt (it raises
        ``TypeError`` on a ``_ShapedPlan`` to enforce this). The follower path never
        shapes.
        """
        if self.gain <= 0.0 or not getattr(plan, 'segments', ()):  # noqa: E501
            return plan
        return _ShapedPlan(plan, self.gain, arm_mm=self.arm_mm,
                           g_mm_s2=self.g_mm_s2, tilt_cap_rad=self.tilt_cap_rad)

    __call__ = shape


assert POSE_DIM == 6  # the sign convention above hard-codes the 6-DoF axis layout
