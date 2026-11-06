"""``motion/skills/plant_model`` -- f0, the global plant model of the Learner (plan § 2.2, L1).

One global model of what a Throw does with its command, fitted online from every Learner row
regardless of Site or Pattern. For a command ``u = (u_x, u_y, u_apex)`` [m] from a release Site
whose cup x coordinate is ``site_x`` [m], thrown with ball ``ball_id``::

    forward   y_apex = K u_apex          y_xy = u_xy + b + (s site_x, 0) [+ Delta_ball]
    inverse   u_apex = y_d,apex / K      u_xy = y_d,xy - b - (s site_x, 0) [- Delta_ball]

theta = (K, b_x, b_y, s) [+ (Delta_bx, Delta_by) for every ball b >= 1 when ``ball_offsets``;
Delta_0 == 0, ball 0 is the reference]. K is the apex gain, b the lateral offset, s the axis-0
centre pull per metre of release-Site x (axis 0 ONLY until P2(b) says it is radial).

Why this shape (the P1 replay probe, ``tools/probes/learner_replay.py``, 2026-11-06, 3276-row
corpus over 39 Sittings): ONE global apex gain holds (K = 1.081; per Sitting 1.070-1.095, per Site
1.070-1.090, flat over apex 0.8-1.4 m, and a pure gain extrapolates better than an affine fit);
the lateral offsets are NOT global, so the Learner's residual memory (L2, ``learner.py``) carries
what b and s miss. Cold-start apex error at a new Site: 73.5 mm under the identity prior, 17.8 mm
under f0.

The fit (RLS, Kalman form, measurement variance R, identity prior, three independent scalar-
measurement filters: apex [K]; axis 0 [b_x, s, Delta_x...]; axis 1 [b_y, Delta_y...]). The
probe's prototype (``learner_replay.Filt`` / ``run_rls``) is the reference; this module ports it.
Three rules the probe decided:

* **Forget K and b only** (lambda per ACCEPTED row; s and Delta are never forgotten). One lambda
  for every parameter winds ``s`` up during a single-Site stretch (its direction is unexcited, so
  its variance grows by 1/lambda per row) until it falls back to its prior: on the corpus after
  410-1958 rows. Covariance cross-terms are divided by sqrt(lambda_i lambda_j), as the probe does.
* **Report the prior while uncertain.** A parameter whose posterior sd exceeds
  ``fallback_frac`` x its prior sd REPORTS its prior mean, so an empty memory is exactly the
  identity (plan T-R2) and a never-seen ball's offset is zero. On the corpus K leaves its prior
  after 2 rows, b after 3-8, s after 14. K and b_y follow this per-PARAMETER rule (each is
  observed directly). The axis-0 offset b_x + s x follows it per QUERY on the predicted quantity
  (2026-11-06 follow-up): at one Site only that sum is observed, so b_x and s each stay uncertain
  while the offset AT that Site is well determined; the model applies raw b_x + s x where its
  predictive sd is under ``fallback_frac`` x the prior predictive sd, and the identity (0) where
  the b/s split is still unknown (a far Site). Delta stays on the per-parameter rule.
* **The inlier rule is relative to the current model** (never an innovation gate: that admits
  early outliers while the covariance is large). A row updates theta only if its lateral
  innovation (against the RLS's own current estimate, as the probe computes it) is under
  ``inlier_lateral_m`` on each axis AND ``|y_apex / u_apex - K| < max(inlier_apex_frac K,
  inlier_apex_sd_mult sd_K)`` with K the REPORTED gain. The second term widens the apex band
  while K is still uncertain so a plant whose gain is far from 1 (the sim's 1.23) is learned from
  cold rather than rejected forever; once K has converged it is the probe's fixed 9 % band.
  Rejected rows are counted (``n_rejected``) and the last one's reason kept: never silently lost.

Errors: every non-finite input raises ``ValueError`` naming the quantity (plan T-U5), and nothing
else escapes (the executor converts only ``ValueError`` into a refusal): numeric failures of an
update are caught, the update is discarded and a ``ValueError`` raised. State is rebuilt by
replaying a memory's rows in file order; nothing here is persisted.

Pure Python + numpy. No ROS2 imports (the ``motion/`` rule).
"""

from __future__ import annotations

import dataclasses
import math
from typing import List, Optional, Tuple

import numpy as np

__all__ = ['PlantModelConfig', 'PlantModel']


@dataclasses.dataclass(frozen=True)
class PlantModelConfig:
    """Constants of f0. Origin of every value: the P1 replay probe
    (``tools/probes/learner_replay.py``, run 2026-11-06 on the 846- and 3276-row corpora,
    ``temp/probes/learner_replay/SUMMARY.md``) unless stated. SI units."""

    #: Prior sd of the apex gain K about 1 (probe: K 1.070-1.095 per Sitting; 1 +- 0.10 covers it).
    prior_sd_K: float = 0.10
    #: Prior sd of each lateral offset b about 0 [m] (probe: per-Site b_x -2..+18 mm).
    prior_sd_b_m: float = 0.030
    #: Prior sd of the axis-0 centre pull s about 0 [per m] (probe: tested at 0.30; s = -0.09/m).
    prior_sd_s_per_m: float = 0.30
    #: Prior sd of each per-ball offset Delta about 0 [m] (probe: Delta_1 converges after 7 rows).
    prior_sd_delta_m: float = 0.030
    #: Measurement sd of the apex [m] (probe: apex residual ~10 mm about the fitted gain).
    noise_sd_apex_m: float = 0.010
    #: Measurement sd of each lateral axis [m] (probe: lateral shot noise ~20 mm per axis).
    noise_sd_lateral_m: float = 0.020
    #: Forgetting factor per ACCEPTED row, applied to K and b only (probe: lambda 0.99 with s and
    #: Delta unforgotten, 'band+struct', was the best variant; ~100-row memory for K and b).
    forgetting: float = 0.99
    #: A parameter reports its prior mean while its posterior sd > this x its prior sd (probe).
    fallback_frac: float = 0.5
    #: Inlier rule, lateral: |innovation| on each axis must be under this [m] (the 2026-10-18
    #: band's 100 mm, made relative to the current model: the probe's 'mband').
    inlier_lateral_m: float = 0.100
    #: Inlier rule, apex: |y_apex/u_apex - K| < max(this x K, ...) (the probe's 9 % 'mband').
    inlier_apex_frac: float = 0.09
    #: ... max(..., this x sd_K): widens the band while K is uncertain (P1 design, 2026-11-06:
    #: the sim's gain 1.23 must be learned, not rejected).
    inlier_apex_sd_mult: float = 3.0
    #: Per-ball lateral offsets Delta_b (b >= 1). Implemented and tested but OFF until the P2(c)
    #: ball-swap Sitting separates ball from Site (plan § 2.7; probe: Delta_1 is confounded with
    #: the Site, the one same-Site contrast is -4.7 / -4.4 mm, not +16).
    ball_offsets: bool = False
    #: Number of ball ids the offsets cover (ids 0..n_balls-1; used only when ``ball_offsets``).
    n_balls: int = 2

    def __post_init__(self) -> None:
        for name in ('prior_sd_K', 'prior_sd_b_m', 'prior_sd_s_per_m', 'prior_sd_delta_m',
                     'noise_sd_apex_m', 'noise_sd_lateral_m', 'inlier_lateral_m',
                     'inlier_apex_frac'):
            v = getattr(self, name)
            if not (isinstance(v, (int, float)) and math.isfinite(v) and v > 0):
                raise ValueError('PlantModelConfig.%s must be finite and > 0, got %r' % (name, v))
        for name in ('fallback_frac', 'inlier_apex_sd_mult'):
            v = getattr(self, name)
            if not (isinstance(v, (int, float)) and math.isfinite(v) and v >= 0):
                raise ValueError('PlantModelConfig.%s must be finite and >= 0, got %r' % (name, v))
        if not (isinstance(self.forgetting, (int, float)) and math.isfinite(self.forgetting)
                and 0.0 < self.forgetting <= 1.0):
            raise ValueError('PlantModelConfig.forgetting must be in (0, 1], got %r'
                             % (self.forgetting,))
        if not isinstance(self.n_balls, int) or isinstance(self.n_balls, bool) or self.n_balls < 1:
            raise ValueError('PlantModelConfig.n_balls must be an int >= 1, got %r' % (self.n_balls,))


class _Filter(object):
    """One RLS filter (Kalman form) with per-parameter forgetting and fallback to the prior.

    A port of the probe's ``Filt`` (unbounded form): ``P <- P / sqrt(outer(lam, lam))`` before
    each update, then the scalar-measurement Kalman update with variance ``R``."""

    def __init__(self, prior, sd0, R: float, lam) -> None:
        self.prior = np.array(prior, dtype=float)
        self.th = self.prior.copy()
        self.sd0 = np.asarray(sd0, dtype=float)
        self.P = np.diag(self.sd0 ** 2)
        self.R = float(R)
        lam = np.broadcast_to(np.asarray(lam, dtype=float), self.th.shape)
        self.lam = np.sqrt(np.outer(lam, lam))

    def innov(self, phi: np.ndarray, z: float) -> float:
        return z - float(phi @ self.th)

    def proposed(self, phi: np.ndarray, z: float) -> Tuple[np.ndarray, np.ndarray]:
        """The (theta, P) one update would produce; nothing is committed."""
        P = self.P / self.lam
        S = float(phi @ P @ phi) + self.R
        g = P @ phi / S
        th = self.th + g * (z - float(phi @ self.th))
        P = P - np.outer(g, phi @ P)
        return th, 0.5 * (P + P.T)

    def sd(self) -> np.ndarray:
        return np.sqrt(np.maximum(np.diag(self.P), 0.0))

    def reported(self, frac: float) -> np.ndarray:
        return np.where(self.sd() > frac * self.sd0, self.prior, self.th)


def _finite_vec(value, n: int, name: str) -> np.ndarray:
    try:
        arr = np.asarray(value, dtype=float).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise ValueError('%s must be a %d-vector of numbers, got %r (%s)' % (name, n, value, exc))
    if arr.shape != (n,):
        raise ValueError('%s must be a %d-vector, got shape %s' % (name, n, arr.shape))
    if not np.all(np.isfinite(arr)):
        raise ValueError('%s must be finite, got %r' % (name, arr.tolist()))
    return arr


def _finite_scalar(value, name: str) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError('%s must be a number, got %r (%s)' % (name, value, exc))
    if not math.isfinite(v):
        raise ValueError('%s must be finite, got %r' % (name, v))
    return v


def _ball_id(value) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise ValueError('ball_id must be an int, got %r' % (value,))
    b = int(value)
    if b < 0:
        raise ValueError('ball_id must be >= 0, got %r' % (b,))
    return b


class PlantModel(object):
    """f0 with its online RLS fit. See the module docstring for the law and its origins."""

    def __init__(self, config: Optional[PlantModelConfig] = None) -> None:
        cfg = PlantModelConfig() if config is None else config
        if not isinstance(cfg, PlantModelConfig):
            raise ValueError('config must be a PlantModelConfig, got %r' % (cfg,))
        self.config = cfg
        lam = cfg.forgetting
        nd = cfg.n_balls - 1 if cfg.ball_offsets else 0
        self._nd = nd
        self._fa = _Filter([1.0], [cfg.prior_sd_K], cfg.noise_sd_apex_m ** 2, lam)
        self._fx = _Filter([0.0] * (2 + nd), [cfg.prior_sd_b_m, cfg.prior_sd_s_per_m]
                           + [cfg.prior_sd_delta_m] * nd,
                           cfg.noise_sd_lateral_m ** 2, [lam, 1.0] + [1.0] * nd)
        self._fy = _Filter([0.0] * (1 + nd), [cfg.prior_sd_b_m] + [cfg.prior_sd_delta_m] * nd,
                           cfg.noise_sd_lateral_m ** 2, [lam] + [1.0] * nd)
        names = ['K', 'b_x', 'b_y', 's']
        for b in range(1, nd + 1):
            names += ['delta%d_x' % b, 'delta%d_y' % b]
        #: Names of the entries of :meth:`theta`, :meth:`raw` and :meth:`sd`, in order.
        self.param_names = tuple(names)
        self.n_accepted = 0
        self.n_rejected = 0
        #: Why the last rejected row was rejected (None before the first rejection).
        self.last_rejection: Optional[str] = None

    # ------------------------------------------------------------------ helpers
    def _ball(self, ball_id) -> int:
        b = _ball_id(ball_id)
        if self._nd and b >= self.config.n_balls:
            raise ValueError('ball_id %d is outside the %d balls the offsets cover'
                             % (b, self.config.n_balls))
        return b

    def _onehot(self, b: int) -> np.ndarray:
        e = np.zeros(self._nd)
        if b >= 1 and self._nd:
            e[b - 1] = 1.0
        return e

    def _phis(self, site_x: float, b: int) -> Tuple[np.ndarray, np.ndarray]:
        e = self._onehot(b)
        return np.concatenate(([1.0, site_x], e)), np.concatenate(([1.0], e))

    def _pack(self, ra, rx, ry) -> np.ndarray:
        out = [ra[0], rx[0], ry[0], rx[1]]
        for i in range(self._nd):
            out += [rx[2 + i], ry[1 + i]]
        return np.array(out, dtype=float)

    def _reported_K(self) -> float:
        K = float(self._fa.reported(self.config.fallback_frac)[0])
        if not (math.isfinite(K) and K > 0.0):
            raise ValueError('the plant model gain K is not usable (%r)' % (K,))
        return K

    def _offset_x_rows(self, site_x: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """The applied axis-0 offset b_x + s x at each ``site_x`` (vectorised), and whether it is
        at its prior. Decided per QUERY on the predicted quantity (2026-11-06 follow-up): the raw
        b_x + s x when its posterior predictive sd sqrt(a^T P a), a = [1, x] over (b_x, s), is at
        most ``fallback_frac`` x its PRIOR predictive sd sqrt(sd_b^2 + x^2 sd_s^2); else 0."""
        x = np.asarray(site_x, dtype=float)
        P = self._fx.P
        var = P[0, 0] + 2.0 * x * P[0, 1] + x * x * P[1, 1]
        sd0 = self._fx.sd0
        var0 = sd0[0] ** 2 + x * x * sd0[1] ** 2
        at_prior = np.sqrt(np.maximum(var, 0.0)) > self.config.fallback_frac * np.sqrt(var0)
        value = self._fx.th[0] + self._fx.th[1] * x
        return np.where(at_prior, 0.0, value), at_prior

    def offset_x(self, site_x) -> Tuple[float, bool]:
        """(axis-0 offset [m] the model APPLIES at ``site_x`` for ball 0, whether it is the prior)."""
        sx = _finite_scalar(site_x, 'site_x')
        v, p = self._offset_x_rows(np.array([sx]))
        return float(v[0]), bool(p[0])

    def _lateral(self, site_x: float, b: int) -> np.ndarray:
        """Applied lateral offset at a Site for a ball: (b_x + s site_x) by the per-query rule,
        b_y by the per-parameter rule [+ Delta_ball, per-parameter rule]."""
        frac = self.config.fallback_frac
        rx = self._fx.reported(frac)
        ry = self._fy.reported(frac)
        e = self._onehot(b)
        ox = self._offset_x_rows(np.array([site_x]))[0][0]
        if self._nd:
            return np.array([ox + float(e @ rx[2:]), ry[0] + float(e @ ry[1:])])
        return np.array([ox, ry[0]])

    # ------------------------------------------------------------------ the model
    def forward(self, u, site_x, ball_id) -> np.ndarray:
        """Predicted landing y (3,) [m] of command ``u`` (3,) under the APPLIED model (reported K, b_y;
        the per-query axis-0 offset)."""
        u = _finite_vec(u, 3, 'u')
        sx = _finite_scalar(site_x, 'site_x')
        b = self._ball(ball_id)
        off = self._lateral(sx, b)
        y = np.array([u[0] + off[0], u[1] + off[1], self._reported_K() * u[2]])
        if not np.all(np.isfinite(y)):
            raise ValueError('the plant model forward map is not finite (%r)' % (y.tolist(),))
        return y

    def forward_rows(self, U: np.ndarray, site_x: np.ndarray, ball_id) -> np.ndarray:
        """:meth:`forward` of many rows of ONE ball: ``U`` (n, 3), ``site_x`` (n,) -> (n, 3).
        The caller guarantees finite arrays (the Learner's own validated rows)."""
        b = self._ball(ball_id)
        frac = self.config.fallback_frac
        rx = self._fx.reported(frac)
        ry = self._fy.reported(frac)
        e = self._onehot(b)
        dx = float(e @ rx[2:]) if self._nd else 0.0
        dy = ry[0] + float(e @ ry[1:]) if self._nd else ry[0]
        U = np.asarray(U, dtype=float).reshape(-1, 3)
        out = np.empty_like(U)
        ox = self._offset_x_rows(np.asarray(site_x, dtype=float).reshape(-1))[0]
        out[:, 0] = U[:, 0] + (ox + dx) if self._nd else U[:, 0] + ox
        out[:, 1] = U[:, 1] + dy
        out[:, 2] = self._reported_K() * U[:, 2]
        return out

    def inverse(self, y_d, site_x, ball_id) -> np.ndarray:
        """The command u (3,) [m] whose predicted landing is ``y_d`` (3,) under the APPLIED model."""
        y_d = _finite_vec(y_d, 3, 'y_d')
        sx = _finite_scalar(site_x, 'site_x')
        b = self._ball(ball_id)
        off = self._lateral(sx, b)
        u = np.array([y_d[0] - off[0], y_d[1] - off[1], y_d[2] / self._reported_K()])
        if not np.all(np.isfinite(u)):
            raise ValueError('the plant model inverse map is not finite (%r)' % (u.tolist(),))
        return u

    def observe(self, u, y, site_x, ball_id) -> bool:
        """One row: update theta if the row is an inlier. Returns whether it was accepted."""
        u = _finite_vec(u, 3, 'u')
        y = _finite_vec(y, 3, 'y')
        sx = _finite_scalar(site_x, 'site_x')
        b = self._ball(ball_id)
        if not u[2] > 0.0:
            raise ValueError('u apex must be > 0, got %r' % (float(u[2]),))
        cfg = self.config
        px, py = self._phis(sx, b)
        pa = np.array([u[2]])
        zx, zy, za = y[0] - u[0], y[1] - u[1], y[2]
        try:
            with np.errstate(all='ignore'):
                ix = self._fx.innov(px, zx)
                iy = self._fy.innov(py, zy)
                K = float(self._fa.reported(cfg.fallback_frac)[0])
                sdK = float(self._fa.sd()[0])
                ratio = y[2] / u[2]
                band = max(cfg.inlier_apex_frac * K, cfg.inlier_apex_sd_mult * sdK)
                reason = None
                if not abs(ix) < cfg.inlier_lateral_m:
                    reason = 'axis 0 innovation %.1f mm >= %.1f mm' % (1e3 * ix, 1e3 * cfg.inlier_lateral_m)
                elif not abs(iy) < cfg.inlier_lateral_m:
                    reason = 'axis 1 innovation %.1f mm >= %.1f mm' % (1e3 * iy, 1e3 * cfg.inlier_lateral_m)
                elif not abs(ratio - K) < band:
                    reason = 'apex ratio %.4f outside K %.4f +- %.4f' % (ratio, K, band)
                if reason is not None:
                    self.n_rejected += 1
                    self.last_rejection = reason
                    return False
                new = [f.proposed(p, z) for f, p, z in
                       ((self._fa, pa, za), (self._fx, px, zx), (self._fy, py, zy))]
        except (ArithmeticError, np.linalg.LinAlgError) as exc:   # pragma: no cover - defensive
            raise ValueError('the plant model update failed numerically: %s' % (exc,))
        for th, P in new:
            if not (np.all(np.isfinite(th)) and np.all(np.isfinite(P))):
                raise ValueError('the plant model update is not finite (u=%r, y=%r); row discarded'
                                 % (u.tolist(), y.tolist()))
        for f, (th, P) in zip((self._fa, self._fx, self._fy), new):
            f.th, f.P = th, P
        self.n_accepted += 1
        return True

    # ------------------------------------------------------------------ read-outs
    def theta(self) -> np.ndarray:
        """The REPORTED parameters (prior mean where still uncertain), order ``param_names``."""
        frac = self.config.fallback_frac
        return self._pack(self._fa.reported(frac), self._fx.reported(frac), self._fy.reported(frac))

    def raw(self) -> np.ndarray:
        """The posterior mean, order ``param_names``."""
        return self._pack(self._fa.th, self._fx.th, self._fy.th)

    def sd(self) -> np.ndarray:
        """The posterior sd, order ``param_names``."""
        return self._pack(self._fa.sd(), self._fx.sd(), self._fy.sd())

    def at_prior(self) -> List[str]:
        """Names of the parameters still reporting their prior."""
        frac = self.config.fallback_frac
        sd0 = self._pack(self._fa.sd0, self._fx.sd0, self._fy.sd0)
        return [n for n, s, s0 in zip(self.param_names, self.sd(), sd0) if s > frac * s0]

    def summary(self, site_x: Optional[float] = None) -> str:
        """One log line: K, b [mm], s (per-parameter reported values), rows accepted / rejected,
        parameters still at their prior; with ``site_x``, also the axis-0 offset the model APPLIES
        there (the per-query rule)."""
        th = self.theta()
        parts = ['K %.4f' % th[0], 'b %+.1f/%+.1f mm' % (1e3 * th[1], 1e3 * th[2]),
                 's %+.4f/m' % th[3]]
        for i in range(self._nd):
            parts.append('delta%d %+.1f/%+.1f mm' % (i + 1, 1e3 * th[4 + 2 * i], 1e3 * th[5 + 2 * i]))
        if site_x is not None:
            ox, p = self.offset_x(site_x)
            parts.append('applied axis-0 offset at x %+.3f m: %s' % (
                float(site_x), 'prior (0)' if p else '%+.1f mm' % (1e3 * ox)))
        prior = self.at_prior()
        return ('plant model: %s; rows %d accepted / %d rejected; at prior: %s'
                % (', '.join(parts), self.n_accepted, self.n_rejected,
                   ', '.join(prior) if prior else 'none'))
