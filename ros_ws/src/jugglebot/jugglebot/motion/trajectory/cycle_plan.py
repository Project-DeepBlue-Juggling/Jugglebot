"""The 7-channel cycle plan — six legs and the hand on ONE clock.

``CyclePlan`` is what the unified planner
(`plans/active/unified-7dof-planner.md` § 4, Phase 1) hands to
``trajectory_node``.  Today's plans carry 6 DoF and the hand is a separate
actor with its own timeline; a cycle plan carries all seven channels sampled
from a single ``t``, so "the hand strokes while the platform is here" is a
property of the plan rather than a hoped-for coincidence of two schedulers.

**It IS a TrajectoryPlan.**  It subclasses :class:`plan.TrajectoryPlan` and
honours :meth:`state_at`'s contract *exactly* — ``(pose, twist, accel)`` as three
``(6,)`` float arrays, the terminal hold (``final_pose``, zero twist, zero accel)
at and past ``total_duration``, and the first-knot boundary conditions at
``t <= 0``.  That is the whole point: the 40 Hz emitter
(``emitter.py`` samples ``state_at`` at τ, τ+dt, τ+2·dt) and
``trajectory_node``'s ``_install`` continuity machinery (which seeds every
replan from ``old_plan.state_at(t_install)``) must work on a cycle plan with no
changes at all.  :meth:`hand_at` is the *additional* surface Phase 2's emitter
reads for the 7th channel.

**Piecewise-cubic by construction.**  Between knots every channel is a cubic
Hermite built from the two knot values and the two knot velocities.  That is not
an interpolation convenience — it is the same reconstruction the can-bridge's
500 Hz interp lane performs from ``(u0, v0, u1, v1)`` once the wire carries the
``v1`` array (Phase 2, ``HAS_V1``).  Host and firmware therefore agree on the
curve between knots instead of merely at them, which is what Phase 0's decision
2 measured as float-exact (≤ 3.6e-15 rev) on knot-aligned piecewise cubics.

**Duck-typed input.**  :meth:`from_realized` takes anything with
``cup_realize.RealizedCycle``'s field contract, and the arrays can equally be
built by hand (the tests do).  Nothing here imports ``sim``.

⚠ **Zero segments is a real hazard for the WP4 gate.**  A cycle plan is knot-based,
so ``self.segments`` is empty and ``total_duration`` is set directly.  Every
existing ``feasibility`` entry point that measures the platform loops
``plan.segments`` — with none, ``validate`` samples nothing and
``validate_follow`` falls into its degenerate single-held-pose branch (its
``if not plan.segments`` block).  ``validate_cycle`` must therefore sample
:meth:`state_at` on the knot grid itself and must NOT delegate to ``validate``.

⚠ **The terminal hold is a cliff at release.**  The contract's hold is
``final_pose`` with ZERO twist — but a cycle ends at the throw, moving at the
takeoff velocity.  Sampling past ``total_duration`` therefore reads a hard stop,
and a replan seeded from there would seed at rest.  That is inherited from
``TrajectoryPlan`` deliberately (deviating would break the contract this class
exists to satisfy); Phase 4's orchestrator owns installing the next cycle before
the clock runs off the end.

Pure Python + numpy.  No ROS2 / repo-root imports.
"""

from __future__ import annotations

import numpy as np

from jugglebot.motion.trajectory.plan import TrajectoryPlan
from jugglebot.motion.trajectory.segment import POSE_DIM

_ZERO_POSE = np.zeros(POSE_DIM)


def _hermite(p0, v0, p1, v1, h: float, s: float):
    """Cubic Hermite value/1st/2nd derivative at ``s ∈ [0, 1]`` over a span ``h``.

    Works elementwise on numpy arrays and on plain floats alike (the hand channel
    is scalar), so there is one implementation for all seven channels.  Returns
    derivatives w.r.t. *time*, not ``s``.
    """
    s2 = s * s
    s3 = s2 * s
    # Position basis.
    h00 = 2.0 * s3 - 3.0 * s2 + 1.0
    h10 = s3 - 2.0 * s2 + s
    h01 = -2.0 * s3 + 3.0 * s2
    h11 = s3 - s2
    pos = h00 * p0 + (h10 * h) * v0 + h01 * p1 + (h11 * h) * v1
    # d/ds of the basis, then chain by 1/h.
    d00 = 6.0 * s2 - 6.0 * s
    d10 = 3.0 * s2 - 4.0 * s + 1.0
    d01 = -6.0 * s2 + 6.0 * s
    d11 = 3.0 * s2 - 2.0 * s
    vel = (d00 * p0 + d01 * p1) / h + d10 * v0 + d11 * v1
    # d²/ds² of the basis, chained by 1/h².
    e00 = 12.0 * s - 6.0
    e10 = 6.0 * s - 4.0
    e01 = -12.0 * s + 6.0
    e11 = 6.0 * s - 2.0
    acc = (e00 * p0 + e01 * p1) / (h * h) + (e10 * v0 + e11 * v1) / h
    return pos, vel, acc


class CyclePlan(TrajectoryPlan):
    """A knot-sampled 7-channel plan: 6-DoF platform pose + hand, one clock.

    Parameters
    ----------
    pose, pose_vel : (n, 6) array-like
        Platform pose and its time derivative at each knot (mm / rad, per second).
    hand_rev, hand_vel_rps : (n,) array-like
        Hand motor position (ODrive absolute rev) and rate at each knot.
    dt : float
        Uniform knot spacing (s) — 0.025 for the planner's grid.
    catch_k : int, optional
        Knot index of the catch, carried through for the orchestrator; ``-1`` when
        the plan has no catch.
    """

    __slots__ = ('pose', 'pose_vel', 'hand_rev', 'hand_vel_rps', 'dt',
                 'n_knots', 't', 'catch_k')

    def __init__(self, pose, pose_vel, hand_rev, hand_vel_rps, dt,
                 catch_k: int = -1):
        p = np.asarray(pose, dtype=float)
        pv = np.asarray(pose_vel, dtype=float)
        hr = np.asarray(hand_rev, dtype=float).reshape(-1)
        hv = np.asarray(hand_vel_rps, dtype=float).reshape(-1)

        if p.ndim != 2 or p.shape[1] != POSE_DIM:
            raise ValueError(f"pose must be (n, {POSE_DIM}), got {p.shape}")
        n = int(p.shape[0])
        if n < 2:
            raise ValueError(
                f"CyclePlan needs at least 2 knots to span time, got {n} "
                "(a single-knot plan is a HoldPlan, not a cycle)")
        if pv.shape != p.shape:
            raise ValueError(f"pose_vel must match pose {p.shape}, got {pv.shape}")
        if hr.shape != (n,) or hv.shape != (n,):
            raise ValueError(
                f"hand_rev/hand_vel_rps must be ({n},), got {hr.shape}/{hv.shape}")
        dt = float(dt)
        if not dt > 0.0:
            raise ValueError(f"dt must be > 0, got {dt}")
        for name, arr in (('pose', p), ('pose_vel', pv), ('hand_rev', hr),
                          ('hand_vel_rps', hv)):
            if not np.all(np.isfinite(arr)):
                raise ValueError(f"{name} contains non-finite values (NaN/Inf)")

        # Zero segments: the pose track is knot-based, not quintic-segment based.
        # See the module docstring's WP4 warning before reusing a segment-looping
        # feasibility entry point on this class.
        super().__init__(segments=(), final_pose=p[-1])
        self.pose = p
        self.pose_vel = pv
        self.hand_rev = hr
        self.hand_vel_rps = hv
        self.dt = dt
        self.n_knots = n
        self.catch_k = int(catch_k)
        self.t = np.arange(n, dtype=float) * dt
        self.total_duration = (n - 1) * dt

    # ── construction ──

    @classmethod
    def from_realized(cls, realized) -> 'CyclePlan':
        """Build from anything with :class:`cup_realize.RealizedCycle`'s fields."""
        return cls(pose=realized.pose,
                   pose_vel=realized.pose_vel,
                   hand_rev=realized.slider_rev,
                   hand_vel_rps=realized.slider_vel_rev_s,
                   dt=realized.dt,
                   catch_k=getattr(realized, 'catch_k', -1))

    # ── TrajectoryPlan surface ──

    @property
    def kind(self) -> str:
        """Always ``'move'``.

        The base class infers ``'hold'`` from an empty ``segments`` tuple, which is
        exactly backwards here: a cycle plan has no segments *and* is the most
        active plan the machine ever runs.  ``trajectory_node`` gates its
        "is a move still running" checks on this, so the override is load-bearing.
        """
        return 'move'

    def _locate(self, t: float):
        """``(k, s)`` — the knot interval index and the normalised position in it."""
        n = self.n_knots
        if t <= 0.0:
            return 0, 0.0
        k = int(t / self.dt)
        if k > n - 2:
            k = n - 2
        s = (t - k * self.dt) / self.dt
        if s < 0.0:
            s = 0.0
        elif s > 1.0:
            s = 1.0
        return k, s

    def state_at(self, t: float):
        """``(pose, twist, accel)`` at ``t`` seconds since install — the base contract.

        ``t >= total_duration`` returns the terminal hold (``final_pose``, zero
        twist, zero accel); ``t <= 0`` returns the first knot's boundary conditions
        (its pose, its velocity, and the cubic's accel there) exactly as a
        segment-based plan returns its first segment's start BCs.
        """
        t = float(t)
        if t >= self.total_duration:
            return self.final_pose.copy(), _ZERO_POSE.copy(), _ZERO_POSE.copy()
        k, s = self._locate(t)
        pos, vel, acc = _hermite(self.pose[k], self.pose_vel[k],
                                 self.pose[k + 1], self.pose_vel[k + 1],
                                 self.dt, s)
        return pos, vel, acc

    # ── the 7th channel ──

    def hand_at(self, t: float):
        """``(rev, rev_s)`` — hand motor position and rate at ``t``.

        Same clock, same clamping discipline as :meth:`state_at`: past the end the
        hand holds its final position at zero rate; before the start it sits at the
        first knot's position with the first knot's rate.
        """
        t = float(t)
        if t >= self.total_duration:
            return float(self.hand_rev[-1]), 0.0
        k, s = self._locate(t)
        pos, vel, _ = _hermite(float(self.hand_rev[k]), float(self.hand_vel_rps[k]),
                               float(self.hand_rev[k + 1]),
                               float(self.hand_vel_rps[k + 1]),
                               self.dt, s)
        return float(pos), float(vel)
