"""``motion/skills/learner`` -- the Learner: global plant model + residual memory + evidence cap
(plan ``plans/active/learner-generalisation.md`` §§ 2.2-2.4; L1, L2, L3).

Given a target landing ``y_d`` and a :class:`ThrowContext` (the physics of the Throw about to be
commanded), :meth:`Learner.command` returns the command ``u``::

    model command   u_m = f0^-1(y_d)                                  (plant_model.PlantModel)
    residual        r_hat = sum w_i (y_i - f0(u_i)) / sum w_i         (L2, over the k most recent
                    w_i = exp(-d_i^2),                                 support rows with the SAME
                    d^2 = |dp|^2/h_p^2 + |dv|^2/h_v^2 (+ ddwell^2/h_d^2)   ball_id and release_kind)
    trust           alpha = sum w / (sum w + w0)
    command         u = f0^-1(y_d - alpha r_hat), then |u - u_m| <= c_max (1 - exp(-sum w / w0))
                    per lateral axis and in apex                       (L3, the cap about the MODEL)

``dp`` is over the release Site xy and release height (:func:`_kernel_pos`: the release position
net of the catch offset), ``dv`` over the planned release velocity (xyz) of the
MODEL command (the caller computes it from f0^-1(y_d) with ``ballistics_bc.launch_velocity``;
the Learner's own answer would be circular). Support = rows with ``d^2 <= support_d2`` (9, the
old law's gate). Empty memory -> exactly ``y_d`` (the reported model is the identity, plan T-R2);
empty support -> exactly the model command.

The old local-linear law (anchor, ridge fit of (C, D, d), damped step) is DELETED (owner decision
D1): after f0 the u -> y map is the identity plus a residual, so the fixed-Jacobian step is the
whole law. The old law survives frozen for the replay probe only, in
``tools/probes/learner_replay_old_law.py``.

The constants come from the P1 replay probe (``tools/probes/learner_replay.py``, 2026-11-06,
3276-row corpus; ``temp/probes/learner_replay/SUMMARY.md``): the L2 surface is flat (969-1003 mm^2
summed MSE over 243 settings), best k = 8, h_v = 0.15, w0 = 4, h_p anywhere in 0.02-0.10, the
dwell term irrelevant (k and w0 nonetheless take the plan's start values 16 and 2: the predictor
optimum is within noise, and the closed loop needs the trust; see ``LearnerConfig.k``); L2's proposed apex corrections have p99 12.9 mm and max 18.0 mm in command
space, so an apex c_max of 20 mm covers them.

Rows enter the residual memory only if the plant model ACCEPTED them (its inlier rule): the
probe's pools are the accepted rows; a 300 mm outlier must not steer r_hat for k throws.

Queries (:meth:`model_command`, :meth:`command`, :meth:`predict`, :meth:`local_model`,
:meth:`evidence`) are READ-ONLY: the executor's early command calls them on a tick that must not
change state. Only :meth:`observe` changes state.

Errors: any non-finite input raises ``ValueError`` naming it (a ThrowContext's ``dwell_s`` may be
NaN, meaning unknown: it then contributes nothing to ``d^2``); nothing else escapes.

Pure Python + numpy. No ROS2 imports (the ``motion/`` rule). ``memory.py`` is not imported: an
Experience is duck-typed on its v2 field names.
"""

from __future__ import annotations

import dataclasses
import math
from typing import Optional, Tuple

import numpy as np

from jugglebot.motion.skills.plant_model import PlantModel, _ball_id, _finite_vec

__all__ = ['RELEASE_KINDS', 'SUPPORT_D2', 'ThrowContext', 'LearnerConfig', 'Learner']

#: The categorical release kinds; L2 requires an exact match (plan § 2.3).
RELEASE_KINDS = ('free_cup', 'carried')
#: Support gate on the kernel distance d^2 (the old law's ``support_d2``: 3 bandwidths).
SUPPORT_D2 = 9.0


@dataclasses.dataclass(frozen=True)
class ThrowContext:
    """The physics of one Throw, as the Learner keys its memory on. SI units.

    ``site_xy_m`` is the RELEASE Site's cup xy (its x is f0's ``site_x``); ``release_m`` where the
    ball leaves; ``release_vel_mps`` the planned release velocity of the MODEL command;
    ``dwell_s`` (NaN = unknown); ``seat_offset_m`` is the catch offset (kept for the row).

    The kernel measures position on the release SITE's xy and the release height
    (:func:`_kernel_pos`), i.e. the release position net of the catch offset: a same-Site fold
    releases where the ball was caught (Site + offset), a carried fold and a launch Throw at
    the Site exactly. The offset is the Catch aim, which follows the tracker's in-flight
    estimate; a command that moved with it would (a) partition the memory by where each ball
    happened to land, the failure the old law closed on 2026-10-12 with its 1 m ``h_seat``
    (sitting 7b), and (b) change between the early admissibility check and the dispatch
    (measured 2026-11-06 on the 3276-row corpus with the offset IN the kernel: a 10 mm aim
    shift moved the command by more than the 1 mm early-command tolerance on 29 % of
    queries). Construction validates every field (``ValueError`` by name); arrays are tuples."""

    site_xy_m: Tuple[float, float]
    release_m: Tuple[float, float, float]
    release_vel_mps: Tuple[float, float, float]
    dwell_s: float
    ball_id: int
    release_kind: str
    seat_offset_m: Tuple[float, float] = (0.0, 0.0)

    def __post_init__(self) -> None:
        for name, n in (('site_xy_m', 2), ('release_m', 3), ('release_vel_mps', 3),
                        ('seat_offset_m', 2)):
            object.__setattr__(self, name, tuple(float(v) for v in
                                                 _finite_vec(getattr(self, name), n, name)))
        object.__setattr__(self, 'dwell_s', _dwell(self.dwell_s))
        object.__setattr__(self, 'ball_id', _ball_id(self.ball_id))
        object.__setattr__(self, 'release_kind', _kind(self.release_kind))


def _dwell(value) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError('dwell_s must be a number or NaN, got %r (%s)' % (value, exc))
    if math.isinf(v):
        raise ValueError('dwell_s must be finite or NaN (unknown), got %r' % (v,))
    return v


def _kind(value) -> str:
    if not isinstance(value, str) or value not in RELEASE_KINDS:
        raise ValueError('release_kind must be one of %r, got %r' % (RELEASE_KINDS, value))
    return value


@dataclasses.dataclass(frozen=True)
class LearnerConfig:
    """Constants of L2 and L3. Origin: the P1 replay probe (``tools/probes/learner_replay.py``,
    2026-11-06, 3276-row corpus, 243-setting sweep) unless stated. SI units."""

    #: Most recent support rows used. The plan's start value (§ 2.3), NOT the probe's k = 8: the
    #: probe scored each law as a PREDICTOR on rows flown under the old law, where the (k, w0)
    #: surface is flat within noise (969-1003 mm^2 over 243 settings), so its k = 8 / w0 = 4 optimum
    #: could not see the closed loop. In the loop a persistent offset f0 does not carry is
    #: corrected only by alpha <= k / (k + w0), leaving (1 - alpha) of it for ever (k 8 / w0 4: a
    #: third; measured 2026-11-06 in the P1 L1 synthetic loop). k 16 / w0 2: alpha up to 0.889,
    #: still inside the flat part of the surface.
    k: int = 16
    #: Release-position bandwidth [m] (probe: 0.02-0.10 within 0.03 mm of each other).
    h_p: float = 0.05
    #: Release-velocity bandwidth [m/s] (probe sweep best).
    h_v: float = 0.15
    #: Dwell bandwidth [s]; inf = the term is inert (probe: no effect on the corpus, which cannot
    #: size it; the field stays so a later Sitting can).
    h_d: float = math.inf
    #: Trust pseudo-count: alpha = sum w / (sum w + w0); cap factor 1 - exp(-sum w / w0). The
    #: plan's start value 2 (the probe's predictor optimum 4 is within noise of it; see ``k``: the
    #: closed loop needs the trust). At full evidence the cap factor is 1 - e^-8 = 0.9997, so the
    #: cap reaches c_max (at k 8 / w0 4 it topped out at 0.865 c_max).
    w0: float = 2.0
    #: Lateral cap about the model command [m] (the node passes ``learner_lateral_authority_mm``;
    #: 30 mm is the owner's JB-139 value, newer than the plan's 40 mm start).
    c_max_xy_m: float = 0.030
    #: Apex cap about the model command [m] (probe: L2 apex corrections p99 12.9, max 18.0 mm).
    c_max_apex_m: float = 0.020
    #: Support gate on d^2 (the old law's gate value).
    support_d2: float = SUPPORT_D2

    def __post_init__(self) -> None:
        if not isinstance(self.k, int) or isinstance(self.k, bool) or self.k < 1:
            raise ValueError('LearnerConfig.k must be an int >= 1, got %r' % (self.k,))
        for name in ('h_p', 'h_v', 'w0', 'support_d2'):
            v = getattr(self, name)
            if not (isinstance(v, (int, float)) and math.isfinite(v) and v > 0):
                raise ValueError('LearnerConfig.%s must be finite and > 0, got %r' % (name, v))
        if not (isinstance(self.h_d, (int, float)) and not math.isnan(self.h_d) and self.h_d > 0):
            raise ValueError('LearnerConfig.h_d must be > 0 (inf = inert), got %r' % (self.h_d,))
        for name in ('c_max_xy_m', 'c_max_apex_m'):
            v = getattr(self, name)
            if not (isinstance(v, (int, float)) and math.isfinite(v) and v >= 0):
                raise ValueError('LearnerConfig.%s must be finite and >= 0, got %r' % (name, v))


def _kernel_pos(site_xy_m, release_m) -> np.ndarray:
    """The position the residual kernel measures distance on: the release Site's xy and the
    release height. See :class:`ThrowContext`."""
    return np.array([float(site_xy_m[0]), float(site_xy_m[1]), float(release_m[2])])


class _Rows(object):
    """Append-only row arrays with amortised growth (rows held in observe order). ``rel`` is
    the KERNEL position (:func:`_kernel_pos`), not the raw release position."""

    def __init__(self) -> None:
        self.n = 0
        cap = 64
        self.rel = np.zeros((cap, 3))
        self.vel = np.zeros((cap, 3))
        self.dwell = np.zeros(cap)
        self.u = np.zeros((cap, 3))
        self.y = np.zeros((cap, 3))
        self.site_x = np.zeros(cap)
        self.ball = np.zeros(cap, dtype=int)
        self.kind = np.zeros(cap, dtype=int)
        self.accepted = np.zeros(cap, dtype=bool)

    def append(self, rel, vel, dwell, u, y, site_x, ball, kind, accepted) -> None:
        if self.n == len(self.dwell):
            for name in ('rel', 'vel', 'dwell', 'u', 'y', 'site_x', 'ball', 'kind', 'accepted'):
                a = getattr(self, name)
                b = np.zeros((2 * a.shape[0],) + a.shape[1:], dtype=a.dtype)
                b[:a.shape[0]] = a
                setattr(self, name, b)
        i = self.n
        self.rel[i], self.vel[i], self.dwell[i] = rel, vel, dwell
        self.u[i], self.y[i], self.site_x[i] = u, y, site_x
        self.ball[i], self.kind[i], self.accepted[i] = ball, kind, accepted
        self.n += 1


class Learner(object):
    """The Learner (L1 + L2 + L3). ``model`` is the plant model it owns and updates."""

    #: Lets the executor tell this context API (``command(ctx, y_d)``) from the old
    #: ``command(x, y_d)`` duck-typed test fakes.
    takes_context = True

    def __init__(self, model: Optional[PlantModel] = None,
                 config: Optional[LearnerConfig] = None) -> None:
        self.model = PlantModel() if model is None else model
        if not isinstance(self.model, PlantModel):
            raise ValueError('model must be a PlantModel, got %r' % (self.model,))
        self.config = LearnerConfig() if config is None else config
        if not isinstance(self.config, LearnerConfig):
            raise ValueError('config must be a LearnerConfig, got %r' % (self.config,))
        self._rows = _Rows()

    @property
    def n_rows(self) -> int:
        """Rows observed (accepted by the plant model or not)."""
        return self._rows.n

    # ------------------------------------------------------------------ state change
    def observe(self, exp) -> bool:
        """Learn from one v2 ``Experience`` (duck-typed: ``site_xy_m``, ``release_m``,
        ``release_vel_mps``, ``dwell_s``, ``u``, ``y``, ``ball_id``, ``release_kind``). Updates
        the plant model and appends the row. Returns whether the plant model accepted it (only
        accepted rows enter the residual memory). Validates everything before changing state."""
        try:
            site = _finite_vec(exp.site_xy_m, 2, 'site_xy_m')
            rel = _finite_vec(exp.release_m, 3, 'release_m')
            vel = _finite_vec(exp.release_vel_mps, 3, 'release_vel_mps')
            dwell = _dwell(exp.dwell_s)
            u = _finite_vec(exp.u, 3, 'u')
            y = _finite_vec(exp.y, 3, 'y')
            ball = _ball_id(exp.ball_id)
            kind = RELEASE_KINDS.index(_kind(exp.release_kind))
        except AttributeError as exc:
            raise ValueError('the experience lacks a v2 field: %s' % (exc,))
        accepted = self.model.observe(u, y, site[0], ball)
        self._rows.append(_kernel_pos(site, rel), vel, dwell, u, y, site[0], ball, kind, accepted)
        return accepted

    # ------------------------------------------------------------------ read-only queries
    def _ctx(self, ctx) -> ThrowContext:
        if not isinstance(ctx, ThrowContext):
            raise ValueError('ctx must be a ThrowContext, got %r' % (type(ctx).__name__,))
        return ctx

    def _residual(self, ctx: ThrowContext) -> Tuple[np.ndarray, float, int]:
        """(alpha r_hat (3,), sum w, rows used) at ``ctx``; zeros when the support is empty."""
        cfg = self.config
        R = self._rows
        n = R.n
        if n == 0:
            return np.zeros(3), 0.0, 0
        kind = RELEASE_KINDS.index(ctx.release_kind)
        pool = np.flatnonzero(R.accepted[:n] & (R.ball[:n] == ctx.ball_id) & (R.kind[:n] == kind))
        if pool.size == 0:
            return np.zeros(3), 0.0, 0
        dp = R.rel[pool] - _kernel_pos(ctx.site_xy_m, ctx.release_m)
        dv = R.vel[pool] - np.asarray(ctx.release_vel_mps)
        d2 = (dp * dp).sum(1) / cfg.h_p ** 2 + (dv * dv).sum(1) / cfg.h_v ** 2
        if math.isfinite(cfg.h_d) and math.isfinite(ctx.dwell_s):
            dd = R.dwell[pool] - ctx.dwell_s
            d2 = d2 + np.where(np.isfinite(dd), dd * dd, 0.0) / cfg.h_d ** 2
        idx = np.flatnonzero(d2 <= cfg.support_d2)[-cfg.k:]    # the k most RECENT support rows
        if idx.size == 0:
            return np.zeros(3), 0.0, 0
        sup = pool[idx]
        w = np.exp(-d2[idx])
        th = self.model.theta()
        # residual against the CURRENT reported f0 (the probe's th_q), vectorised forward map
        r = R.y[sup] - self.model.forward_rows(R.u[sup], R.site_x[sup], ctx.ball_id)
        sw = float(w.sum())
        r_hat = (w[:, None] * r).sum(0) / sw
        corr = (sw / (sw + cfg.w0)) * r_hat
        if not (np.all(np.isfinite(corr)) and math.isfinite(sw)):
            raise ValueError('the learner residual is not finite (%r) over %d rows; theta %r'
                             % (corr.tolist(), sup.size, th.tolist()))
        return corr, sw, int(sup.size)

    def evidence(self, ctx: ThrowContext) -> Tuple[float, int]:
        """(sum of kernel weights, rows used) at ``ctx``; (0.0, 0) at empty support."""
        _, sw, n = self._residual(self._ctx(ctx))
        return sw, n

    def model_command(self, ctx: ThrowContext, y_d) -> np.ndarray:
        """f0^-1(y_d): the command the plant model alone gives (exactly ``y_d`` when empty)."""
        ctx = self._ctx(ctx)
        return self.model.inverse(y_d, ctx.site_xy_m[0], ctx.ball_id)

    def command(self, ctx: ThrowContext, y_d) -> np.ndarray:
        """The Learner's command (3,) [m]: the model command corrected by the residual memory,
        capped about the model command by the evidence (L3)."""
        ctx = self._ctx(ctx)
        y_d = _finite_vec(y_d, 3, 'y_d')
        u_m = self.model.inverse(y_d, ctx.site_xy_m[0], ctx.ball_id)
        corr, sw, n = self._residual(ctx)
        if n == 0:
            return u_m
        u = self.model.inverse(y_d - corr, ctx.site_xy_m[0], ctx.ball_id)
        f = 1.0 - math.exp(-sw / self.config.w0)
        lim = np.array([self.config.c_max_xy_m, self.config.c_max_xy_m, self.config.c_max_apex_m]) * f
        u = u_m + np.clip(u - u_m, -lim, lim)
        if not np.all(np.isfinite(u)):
            raise ValueError('the learner command is not finite (%r)' % (u.tolist(),))
        return u

    def predict(self, ctx: ThrowContext, u, y_d=None) -> np.ndarray:
        """Expected landing (3,) [m] of command ``u`` at ``ctx``: f0(u) + alpha r_hat (uncapped;
        what the Catch aim prior uses). ``y_d`` is accepted for the executor's call shape and
        unused (the query physics is the context's)."""
        ctx = self._ctx(ctx)
        y = self.model.forward(u, ctx.site_xy_m[0], ctx.ball_id) + self._residual(ctx)[0]
        if not np.all(np.isfinite(y)):
            raise ValueError('the learner prediction is not finite (%r)' % (y.tolist(),))
        return y

    def local_model(self, ctx: ThrowContext, y_d):
        """The affine forward map ``(u_bar, D, d)`` with ``predict(ctx, u) == d + D (u - u_bar)``:
        u_bar = the model command, D = diag(1, 1, K), d = f0(u_bar) + alpha r_hat. Never None."""
        ctx = self._ctx(ctx)
        u_bar = self.model.inverse(y_d, ctx.site_xy_m[0], ctx.ball_id)
        K = float(self.model.theta()[0])
        D = np.diag([1.0, 1.0, K])
        d = self.model.forward(u_bar, ctx.site_xy_m[0], ctx.ball_id) + self._residual(ctx)[0]
        if not np.all(np.isfinite(d)):
            raise ValueError('the learner local model is not finite (%r)' % (d.tolist(),))
        return u_bar, D, d
