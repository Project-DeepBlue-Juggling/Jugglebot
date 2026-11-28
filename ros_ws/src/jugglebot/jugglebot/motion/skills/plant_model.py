"""``motion/skills/plant_model`` -- f0, the global plant model of the Learner (plan § 2.2, L1).

One global model of what a Throw does with its command, fitted online from every Learner row
regardless of Site or Pattern. For a command ``u = (u_x, u_y, u_apex)`` [m] from a release Site
whose cup x coordinate is ``site_x`` [m], thrown with ball ``ball_id``::

    forward   y_apex = K u_apex          y_xy = u_xy + b + (s site_x, 0) [+ Delta_ball]
    inverse   u_apex = y_d,apex / K      u_xy = y_d,xy - b - (s site_x, 0) [- Delta_ball]

theta = (K, b_x, b_y, s) [+ (Delta_bx, Delta_by) for every ball b >= 1 when ``ball_offsets``;
Delta_0 == 0, ball 0 is the reference]. K is the apex gain, b the lateral offset, s the axis-0
centre pull per metre of release-Site x (axis 0 ONLY until P2(b) says it is radial).

**K is shared; the lateral law is per release kind** (BUG-138, owner's option 3 in TKT-93,
2026-11-26). One apex filter (K) takes every accepted row; the lateral law (b_x, s, b_y and the
per-ball Deltas) is one filter pair PER release kind (:data:`RELEASE_KINDS`: ``free_cup`` for a
Columns Throw from rest, ``carried`` for the oval's Throw out of a carry) and never pools across
kinds: a row updates K and its own kind's lateral pair only, and a query reads K and its own kind's
lateral pair only. Why: one K has held across Sittings and Sites, while the oval's lateral fit
(b_x +179 mm, s +1.23 per m on its own rows) differs from Columns in offset AND slope, and pooled
it moved the Columns offset at the +62.5 mm Site from -3.3 to +4.8 mm. A kind with no rows is at
its prior: the identity laterally, whatever the other kind has learned. Every map and update
(``forward``, ``forward_rows``, ``inverse``, ``observe``) REQUIRES ``release_kind`` and raises on
an unknown one: a default would let a caller that forgets the kind silently read and write the
Columns law (BUG-138 audit, 2026-11-27). Only the read-out ``offset_x`` keeps the default
``'free_cup'``; it never feeds a command or an update. The read-outs keep the free_cup kind's
names (``K, b_x, b_y, s, deltaN_x/y``) and append the other kinds' lateral parameters suffixed
``@<kind>`` (``b_x@carried`` ...).

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

* **Forget K and b only** (per ACCEPTED row; s and Delta are never forgotten). One lambda
  for every parameter winds ``s`` up during a single-Site stretch (its direction is unexcited, so
  its variance grows by 1/lambda per row) until it falls back to its prior: on the corpus after
  410-1958 rows. K is forgotten by lambda (P <- P / lambda, its filter is 1-D). b is forgotten by
  a random walk, P_bb += q per row, q = (1 - lambda)^2 R / lambda (the same steady gain 1 - lambda
  and steady sd as lambda; ``PlantModelConfig.q_b_m2``), since BUG-139 (2026-11-27). Until then b
  had a per-parameter lambda with the probe's sqrt(lambda_i lambda_j) cross-terms, which is
  I <- diag(sqrt lam) I diag(sqrt lam) in information form and MANUFACTURES information in a
  direction no row excited: at one Site I_ss grew with every row while I_bb saturated, so that
  filter called s known after ~150 single-Site rows (BUG-137) and its VALUE of s went wrong
  with it (BUG-139). A random walk only removes information. Forgetting sets the VALUE (theta)
  only, never a decision.
* **Report the prior while uncertain, judged on the information received.** Every decision below
  reads a companion covariance of the same accepted rows with NO forgetting (lambda = 1, same
  prior and R; ``sd_unforgotten``), never the forgetting filter's (BUG-137, 2026-11-25). A
  parameter whose unforgotten sd exceeds ``fallback_frac`` x its prior sd REPORTS its prior mean,
  so an empty memory is exactly the identity (plan T-R2) and a never-seen ball's offset is zero.
  On the corpus K leaves its prior after 2 rows, b after 3-8, s after 14. K and b_y follow this
  per-PARAMETER rule (each is observed directly). The axis-0 offset b_x + s x follows it per QUERY
  on the predicted quantity (2026-11-06 follow-up): at one Site only that sum is observed, so b_x
  and s each stay uncertain while the offset AT that Site is well determined; the model applies
  the forgetting filter's b_x + s x where its unforgotten predictive sd is under
  ``fallback_frac`` x the prior predictive sd, and the identity (0) where the b/s split is still
  unknown (a far Site) -- for as long as the Memory stays single-Site, at any length. Delta stays
  on the per-parameter rule. ``sd()`` still reports the forgetting filter's sd (the inlier rule's
  sd_K reads it).
* **The inlier rule is relative to the current model** (never an innovation gate: that admits
  early outliers while the covariance is large). A row updates theta only if its lateral
  innovation (against the RLS's own current estimate, as the probe computes it) is under
  ``inlier_lateral_m`` on each axis AND ``|y_apex / u_apex - K| < max(inlier_apex_frac K,
  inlier_apex_sd_mult sd_K)`` with K the REPORTED gain. The second term widens the apex band
  while K is still uncertain so a plant whose gain is far from 1 (the sim's 1.23) is learned from
  cold rather than rejected forever; once K has converged it is the probe's fixed 9 % band.
  Rejected rows are counted (``n_rejected``) and the last one's reason kept: never silently lost.
* **Run-length recovery of the lateral gate** (BUG-153, owner's option 1 in TKT-102,
  2026-11-29). The lateral gate is relative to the kind's CURRENT law, which for a cold kind is
  its prior (offset 0) and widens with nothing, so a kind whose true offset is past ~130 mm
  rejected every row forever (the oval's measured +179 mm: 0 of 300). A kind therefore keeps a
  run of its consecutive lateral-only rejections and accepts the row that completes
  :data:`LATERAL_RECOVERY_RUN` agreeing ones; see that constant for the exact rule.

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

__all__ = ['RELEASE_KINDS', 'LATERAL_RECOVERY_RUN', 'LATERAL_RECOVERY_AGREE_M',
           'PlantModelConfig', 'PlantModel']

#: The categorical release kinds (the same tuple as ``learner.RELEASE_KINDS``, which re-exports
#: this one, and ``memory.RELEASE_KINDS``). The first is the reference kind whose lateral
#: parameters keep the unsuffixed names in :attr:`PlantModel.param_names`.
RELEASE_KINDS = ('free_cup', 'carried')

#: Run-length recovery of the lateral gate (BUG-153, owner's option 1 in TKT-102, 2026-11-29):
#: the number of consecutive agreeing lateral rejections of ONE release kind after which the row
#: that completes the run is accepted.
#:
#: Why: the lateral gate (``inlier_lateral_m``, 100 mm) is measured against the kind's current law,
#: and while a kind has no rows that law is its prior (offset 0). Unlike the apex gate, the
#: lateral gate does not widen while the law is uncertain, so a kind whose consistent offset is
#: past ~130 mm never gets the first row that would move its law within reach: probe 2026-11-27,
#: 5 seeds x 300 rows, 150 mm 0-128 accepted, the oval's measured 179 mm 0 of 300. A plant that
#: lands five rows in a row at one consistent offset outside the gate is telling the model its
#: law is wrong, not throwing outliers; a warm kind (live Memory, 2026-11-27: carried 2 of 1085
#: rows rejected, free_cup 42 of 4976, no run past one row) never sees five agreeing ones in a
#: row, so its commands are unchanged.
#:
#: The rule, per release kind (a row of another kind neither extends nor resets the run):
#:
#: * A row that fails the lateral gate ONLY (its apex ratio is inside the apex band, which this
#:   rule never touches) is a candidate. It JOINS the run when, on each axis, its innovation is
#:   within :data:`LATERAL_RECOVERY_AGREE_M` of the MEAN innovation of the rows already in the run
#:   (an empty run takes any candidate). A candidate that does not agree (a wild row) RESETS the
#:   run, and starts a new run of one: a run is never broken by a wild row and then resumed.
#: * Any other row of the kind resets the run to empty: an accepted row, and a row the apex gate
#:   rejects (whatever its lateral innovation).
#: * The innovations of a run are all against the SAME law: the kind's lateral law changes only
#:   when a row of that kind is accepted, and that resets the run.
#: * When a candidate makes the run :data:`LATERAL_RECOVERY_RUN` long, THAT ROW ALONE is accepted
#:   by the ordinary update (one RLS step, forgetting, unforgotten covariance, counters, exactly as
#:   an inlier) and the run is cleared. The earlier rows of the run stay rejected (counted in
#:   ``n_rejected``; the Learner's residual memory never holds them). Why not replay the run: K is
#:   shared, so rows of the other kind accepted between the run's rows have moved K, and replaying
#:   older apex data after newer would reorder the forgetting filter's rows; one row in file order
#:   keeps the update law exactly. One row is enough: at a cold kind the update's gain on the
#:   offset at the row's Site is ~0.7-0.8 (prior sd 30 mm, shot noise 20 mm), so a 179 mm offset
#:   is pulled to within ~40-50 mm and the next rows pass the ordinary gate.
LATERAL_RECOVERY_RUN = 5

#: The agreement band of a recovery run [m], per axis (see :data:`LATERAL_RECOVERY_RUN`): the
#: lateral gate's own width, ``PlantModelConfig.inlier_lateral_m`` (100 mm), so a candidate agrees
#: with the run when it would pass the lateral gate of a law centred on the run's mean. Why that
#: width: rows at one consistent offset scatter about their mean with the 20 mm shot noise per axis
#: (a candidate's distance to the mean of k earlier rows has sd 20 sqrt(1 + 1/k) <= 28 mm, so a
#: genuine run breaks with probability < 1e-3 per row, a Gaussian estimate, not measured), while a lone outlier (a tracker glitch,
#: the 300 mm lateral row of ``test_a_300_mm_lateral_outlier_is_rejected_and_counted``) has no
#: four neighbours within 100 mm of it in a row. Kept a module constant (not config) beside the
#: gate it qualifies; it does not follow a changed ``inlier_lateral_m`` on its own.
LATERAL_RECOVERY_AGREE_M = 0.100


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
    #: Forgetting factor per ACCEPTED row (probe: lambda 0.99 with s and Delta unforgotten,
    #: 'band+struct', was the best variant; ~100-row memory for K and b). Applied AS lambda to K
    #: only (the 1-parameter apex filter, P <- P / lambda). b is forgotten by a random walk whose
    #: variance per row is :attr:`q_b_m2`, derived from this lambda (BUG-139, 2026-11-27).
    forgetting: float = 0.99
    #: Process-noise variance added to each lateral offset b per ACCEPTED row [m^2] (BUG-139).
    #: None = derived from ``forgetting`` (:attr:`q_b_m2`); an explicit value overrides it.
    process_noise_b_m2: Optional[float] = None
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
        q = self.process_noise_b_m2
        if q is not None and not (isinstance(q, (int, float)) and not isinstance(q, bool)
                                  and math.isfinite(q) and q >= 0):
            raise ValueError('PlantModelConfig.process_noise_b_m2 must be None or finite and >= 0, '
                             'got %r' % (q,))
        if not isinstance(self.n_balls, int) or isinstance(self.n_balls, bool) or self.n_balls < 1:
            raise ValueError('PlantModelConfig.n_balls must be an int >= 1, got %r' % (self.n_balls,))

    @property
    def q_b_m2(self) -> float:
        """The random-walk variance added to P_bb per accepted row [m^2] (BUG-139).

        Derived so the random walk forgets b exactly as lambda did, in steady state, for a scalar
        filter observing b directly with variance R (one Site at x = 0, or b_y):

        * lambda:  P- = P / lam, gain k = P- / (P- + R), P = (1 - k) P-. Fixed point: 1/P = lam/P
          + 1/R, so P = (1 - lam) R and k = 1 - lam.
        * process noise: P- = P + q, same update. Fixed point: k P- = q with P- = k R / (1 - k),
          so q = k^2 R / (1 - k).

        Matching the gain k = 1 - lam gives q = (1 - lam)^2 R / lam, and then the steady posterior
        P = k R = (1 - lam) R matches too: the same steady gain, the same exponential weight lam per
        row on old data (a 1/(1 - lam) = 100-row memory at 0.99), the same steady sd. At lambda 0.99
        and R = (20 mm)^2: q = 4.04e-8 m^2, a 0.20 mm random-walk step per row. ``forgetting`` = 1
        gives q = 0 (no forgetting)."""
        if self.process_noise_b_m2 is not None:
            return float(self.process_noise_b_m2)
        lam = float(self.forgetting)
        return (1.0 - lam) ** 2 * self.noise_sd_lateral_m ** 2 / lam


class _Filter(object):
    """One RLS filter (Kalman form) with forgetting and fallback to the prior.

    Before each update ``P <- P / lam + diag(q)``, then the scalar-measurement Kalman update with
    variance ``R``. ``lam`` is a SCALAR forgetting factor, used only where the filter has one
    parameter (the apex filter's K: in 1-D, P / lam is P + P (1/lam - 1), a process noise, and
    there is no second direction to manufacture information in); ``q`` is a per-parameter
    process-noise variance (a random walk), used for b in the lateral filters (BUG-139). ``th`` and
    ``P`` are the forgetting filter: ``th`` is the value applied, ``P`` its own covariance.

    Why not per-parameter lambda (the probe's ``Filt``, ``P <- P / sqrt(outer(lam, lam))``, live
    until 2026-11-27): in information form that step is I <- diag(sqrt lam) I diag(sqrt lam), which
    is not <= I (PSD order), so with b forgotten and s not, I_ss kept growing at one Site while I_bb
    saturated: the filter called s known although only b + s x0 was observed (BUG-137), and its
    VALUE of s was corrupted with it (BUG-139: one second-Site row then applied a line up to ~54 mm
    off at a far Site). Adding q >= 0 to the predicted covariance can only REMOVE information
    (P + Q >= P), and s is a constant of the random-walk model, so its posterior variance never
    falls below what the rows determine: a single-Site stretch leaves s as uncertain as the data
    leave it, and a second Site moves it at once.

    ``P_unforgotten`` (BUG-137) is the covariance of the SAME accepted rows under the same prior and
    ``R`` with NO forgetting (lambda = 1, q = 0): the information actually received. Every "is this
    determined?" decision reads it, never ``P`` (kept although the random walk manufactures no
    information: the decision stays on the information received, never on a tuning constant)."""

    def __init__(self, prior, sd0, R: float, lam: float = 1.0, q=0.0) -> None:
        self.prior = np.array(prior, dtype=float)
        self.th = self.prior.copy()
        self.sd0 = np.asarray(sd0, dtype=float)
        self.P = np.diag(self.sd0 ** 2)
        self.P_unforgotten = self.P.copy()
        self.R = float(R)
        self.lam = float(lam)
        if self.lam != 1.0 and self.th.size != 1:
            # a lambda on a multi-parameter filter is the per-parameter forgetting BUG-139 removed
            raise ValueError('_Filter: lambda forgetting is for a 1-parameter filter only; use q')
        self.Q = np.diag(np.broadcast_to(np.asarray(q, dtype=float), self.th.shape))

    def innov(self, phi: np.ndarray, z: float) -> float:
        return z - float(phi @ self.th)

    def proposed(self, phi: np.ndarray, z: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """The (theta, P, P_unforgotten) one update would produce; nothing is committed."""
        P = self.P / self.lam + self.Q
        S = float(phi @ P @ phi) + self.R
        g = P @ phi / S
        th = self.th + g * (z - float(phi @ self.th))
        P = P - np.outer(g, phi @ P)
        Pu = self.P_unforgotten
        gu = Pu @ phi / (float(phi @ Pu @ phi) + self.R)
        Pu = Pu - np.outer(gu, phi @ Pu)
        return th, 0.5 * (P + P.T), 0.5 * (Pu + Pu.T)

    def sd(self) -> np.ndarray:
        """Posterior sd of the forgetting filter (the applied value's own uncertainty)."""
        return np.sqrt(np.maximum(np.diag(self.P), 0.0))

    def sd_unforgotten(self) -> np.ndarray:
        """Posterior sd from the information actually received (no forgetting): the decision sd."""
        return np.sqrt(np.maximum(np.diag(self.P_unforgotten), 0.0))

    def reported(self, frac: float) -> np.ndarray:
        return np.where(self.sd_unforgotten() > frac * self.sd0, self.prior, self.th)


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


def _release_kind(value) -> str:
    if not isinstance(value, str) or value not in RELEASE_KINDS:
        raise ValueError('release_kind must be one of %r, got %r' % (RELEASE_KINDS, value))
    return value


class _Lateral(object):
    """The lateral law of ONE release kind: the axis-0 filter ``fx`` over (b_x, s, Delta_x...) and
    the axis-1 filter ``fy`` over (b_y, Delta_y...). The per-ball Deltas split with b and s: a
    carried row never moves a free_cup Delta (BUG-138). Each kind's b is forgotten by its own
    random walk ``q_b_m2`` per row of THAT kind (BUG-139); s and Delta are never forgotten; each
    filter keeps its own unforgotten companion covariance (BUG-137)."""

    def __init__(self, cfg: PlantModelConfig, nd: int) -> None:
        q = cfg.q_b_m2
        self.fx = _Filter([0.0] * (2 + nd), [cfg.prior_sd_b_m, cfg.prior_sd_s_per_m]
                          + [cfg.prior_sd_delta_m] * nd,
                          cfg.noise_sd_lateral_m ** 2, q=[q, 0.0] + [0.0] * nd)
        self.fy = _Filter([0.0] * (1 + nd), [cfg.prior_sd_b_m] + [cfg.prior_sd_delta_m] * nd,
                          cfg.noise_sd_lateral_m ** 2, q=[q] + [0.0] * nd)
        self.n_accepted = 0
        #: The current recovery run (BUG-153): the (axis 0, axis 1) lateral innovations [m] of this
        #: kind's consecutive agreeing lateral-only rejections. See :data:`LATERAL_RECOVERY_RUN`.
        self.run: List[Tuple[float, float]] = []
        #: Rows of this kind accepted by run-length recovery (each also counts in ``n_accepted``).
        self.n_recovered = 0

    def extended_run(self, ix: float, iy: float) -> List[Tuple[float, float]]:
        """The run after a lateral-only rejection with innovations (``ix``, ``iy``): the current
        run plus this row when it agrees with the run's mean on each axis, else this row alone (a
        wild row resets the run). Nothing is committed."""
        if self.run:
            mx = sum(r[0] for r in self.run) / len(self.run)
            my = sum(r[1] for r in self.run) / len(self.run)
            if abs(ix - mx) < LATERAL_RECOVERY_AGREE_M and abs(iy - my) < LATERAL_RECOVERY_AGREE_M:
                return self.run + [(ix, iy)]
        return [(ix, iy)]


class PlantModel(object):
    """f0 with its online RLS fit. See the module docstring for the law and its origins."""

    def __init__(self, config: Optional[PlantModelConfig] = None) -> None:
        cfg = PlantModelConfig() if config is None else config
        if not isinstance(cfg, PlantModelConfig):
            raise ValueError('config must be a PlantModelConfig, got %r' % (cfg,))
        self.config = cfg
        nd = cfg.n_balls - 1 if cfg.ball_offsets else 0
        self._nd = nd
        #: The apex gain K: ONE filter, every accepted row of every release kind. Forgotten by
        #: lambda (1-D, manufactures nothing; BUG-139 left it so).
        self._fa = _Filter([1.0], [cfg.prior_sd_K], cfg.noise_sd_apex_m ** 2, lam=cfg.forgetting)
        #: The lateral law per release kind (BUG-138): never pooled across kinds.
        self._lat = dict((k, _Lateral(cfg, nd)) for k in RELEASE_KINDS)
        names = ['K']
        for i, kind in enumerate(RELEASE_KINDS):
            sfx = '' if i == 0 else '@' + kind
            names += ['b_x' + sfx, 'b_y' + sfx, 's' + sfx]
            for b in range(1, nd + 1):
                names += ['delta%d_x%s' % (b, sfx), 'delta%d_y%s' % (b, sfx)]
        #: Names of the entries of :meth:`theta`, :meth:`raw` and :meth:`sd`, in order: K, then
        #: the free_cup lateral parameters under their historical names, then each other release
        #: kind's lateral parameters suffixed ``@<kind>``.
        self.param_names = tuple(names)
        self.n_accepted = 0
        self.n_rejected = 0
        #: Rows accepted by run-length recovery (BUG-153; each also counts in ``n_accepted``).
        self.n_recovered = 0
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

    def _pack(self, ra, per_kind) -> np.ndarray:
        """``ra`` K's vector; ``per_kind`` a function (fx, fy) -> (vx, vy) over the filters."""
        out = [ra[0]]
        for kind in RELEASE_KINDS:
            lat = self._lat[kind]
            rx, ry = per_kind(lat.fx), per_kind(lat.fy)
            out += [rx[0], ry[0], rx[1]]
            for i in range(self._nd):
                out += [rx[2 + i], ry[1 + i]]
        return np.array(out, dtype=float)

    def _reported_K(self) -> float:
        K = float(self._fa.reported(self.config.fallback_frac)[0])
        if not (math.isfinite(K) and K > 0.0):
            raise ValueError('the plant model gain K is not usable (%r)' % (K,))
        return K

    def _offset_x_rows(self, site_x: np.ndarray, kind: str) -> Tuple[np.ndarray, np.ndarray]:
        """The applied axis-0 offset b_x + s x of release kind ``kind`` at each ``site_x``
        (vectorised), and whether it is at its prior. Decided per QUERY on the predicted quantity
        (2026-11-06 follow-up): the raw b_x + s x when its posterior predictive sd sqrt(a^T P a),
        a = [1, x] over (b_x, s), is at most ``fallback_frac`` x its PRIOR predictive sd
        sqrt(sd_b^2 + x^2 sd_s^2); else 0. P is the UNFORGOTTEN covariance (BUG-137: the
        forgetting filter's P turns confident about a far Site after ~150 single-Site rows); the
        value applied is the forgetting filter's. A kind with no rows is at its prior everywhere."""
        fx = self._lat[kind].fx
        x = np.asarray(site_x, dtype=float)
        P = fx.P_unforgotten
        var = P[0, 0] + 2.0 * x * P[0, 1] + x * x * P[1, 1]
        sd0 = fx.sd0
        var0 = sd0[0] ** 2 + x * x * sd0[1] ** 2
        at_prior = np.sqrt(np.maximum(var, 0.0)) > self.config.fallback_frac * np.sqrt(var0)
        value = fx.th[0] + fx.th[1] * x
        return np.where(at_prior, 0.0, value), at_prior

    def offset_x(self, site_x, release_kind: str = 'free_cup') -> Tuple[float, bool]:
        """(axis-0 offset [m] the model APPLIES at ``site_x`` for ball 0 of ``release_kind``,
        whether it is the prior)."""
        sx = _finite_scalar(site_x, 'site_x')
        v, p = self._offset_x_rows(np.array([sx]), _release_kind(release_kind))
        return float(v[0]), bool(p[0])

    def _lateral(self, site_x: float, b: int, kind: str) -> np.ndarray:
        """Applied lateral offset of release kind ``kind`` at a Site for a ball: (b_x + s site_x)
        by the per-query rule, b_y by the per-parameter rule [+ Delta_ball, per-parameter rule]."""
        frac = self.config.fallback_frac
        lat = self._lat[kind]
        rx = lat.fx.reported(frac)
        ry = lat.fy.reported(frac)
        e = self._onehot(b)
        ox = self._offset_x_rows(np.array([site_x]), kind)[0][0]
        if self._nd:
            return np.array([ox + float(e @ rx[2:]), ry[0] + float(e @ ry[1:])])
        return np.array([ox, ry[0]])

    # ------------------------------------------------------------------ the model
    def forward(self, u, site_x, ball_id, release_kind: str) -> np.ndarray:
        """Predicted landing y (3,) [m] of command ``u`` (3,) under the APPLIED model (reported K;
        ``release_kind``'s reported b_y and per-query axis-0 offset)."""
        u = _finite_vec(u, 3, 'u')
        sx = _finite_scalar(site_x, 'site_x')
        b = self._ball(ball_id)
        off = self._lateral(sx, b, _release_kind(release_kind))
        y = np.array([u[0] + off[0], u[1] + off[1], self._reported_K() * u[2]])
        if not np.all(np.isfinite(y)):
            raise ValueError('the plant model forward map is not finite (%r)' % (y.tolist(),))
        return y

    def forward_rows(self, U: np.ndarray, site_x: np.ndarray, ball_id,
                     release_kind: str) -> np.ndarray:
        """:meth:`forward` of many rows of ONE ball and ONE release kind: ``U`` (n, 3), ``site_x``
        (n,) -> (n, 3). The caller guarantees finite arrays (the Learner's own validated rows)."""
        b = self._ball(ball_id)
        kind = _release_kind(release_kind)
        frac = self.config.fallback_frac
        lat = self._lat[kind]
        rx = lat.fx.reported(frac)
        ry = lat.fy.reported(frac)
        e = self._onehot(b)
        dx = float(e @ rx[2:]) if self._nd else 0.0
        dy = ry[0] + float(e @ ry[1:]) if self._nd else ry[0]
        U = np.asarray(U, dtype=float).reshape(-1, 3)
        out = np.empty_like(U)
        ox = self._offset_x_rows(np.asarray(site_x, dtype=float).reshape(-1), kind)[0]
        out[:, 0] = U[:, 0] + (ox + dx) if self._nd else U[:, 0] + ox
        out[:, 1] = U[:, 1] + dy
        out[:, 2] = self._reported_K() * U[:, 2]
        return out

    def inverse(self, y_d, site_x, ball_id, release_kind: str) -> np.ndarray:
        """The command u (3,) [m] whose predicted landing is ``y_d`` (3,) under the APPLIED model
        of ``release_kind``."""
        y_d = _finite_vec(y_d, 3, 'y_d')
        sx = _finite_scalar(site_x, 'site_x')
        b = self._ball(ball_id)
        off = self._lateral(sx, b, _release_kind(release_kind))
        u = np.array([y_d[0] - off[0], y_d[1] - off[1], y_d[2] / self._reported_K()])
        if not np.all(np.isfinite(u)):
            raise ValueError('the plant model inverse map is not finite (%r)' % (u.tolist(),))
        return u

    def observe(self, u, y, site_x, ball_id, release_kind: str) -> bool:
        """One row: update theta if the row is an inlier. Returns whether it was accepted. An
        accepted row updates the shared K and ``release_kind``'s lateral law only; the lateral
        innovations of the inlier rule are against that kind's own lateral law. A row that fails the
        lateral gate only is accepted when it completes ``release_kind``'s run of
        :data:`LATERAL_RECOVERY_RUN` agreeing such rows (BUG-153); ``n_recovered`` counts those."""
        u = _finite_vec(u, 3, 'u')
        y = _finite_vec(y, 3, 'y')
        sx = _finite_scalar(site_x, 'site_x')
        b = self._ball(ball_id)
        kind = _release_kind(release_kind)
        if not u[2] > 0.0:
            raise ValueError('u apex must be > 0, got %r' % (float(u[2]),))
        cfg = self.config
        lat = self._lat[kind]
        px, py = self._phis(sx, b)
        pa = np.array([u[2]])
        zx, zy, za = y[0] - u[0], y[1] - u[1], y[2]
        try:
            with np.errstate(all='ignore'):
                ix = lat.fx.innov(px, zx)
                iy = lat.fy.innov(py, zy)
                K = float(self._fa.reported(cfg.fallback_frac)[0])
                sdK = float(self._fa.sd()[0])
                ratio = y[2] / u[2]
                band = max(cfg.inlier_apex_frac * K, cfg.inlier_apex_sd_mult * sdK)
                reason = None
                if not abs(ix) < cfg.inlier_lateral_m:
                    reason = 'axis 0 innovation %.1f mm >= %.1f mm' % (1e3 * ix, 1e3 * cfg.inlier_lateral_m)
                elif not abs(iy) < cfg.inlier_lateral_m:
                    reason = 'axis 1 innovation %.1f mm >= %.1f mm' % (1e3 * iy, 1e3 * cfg.inlier_lateral_m)
                lateral_only = reason is not None
                if not abs(ratio - K) < band:
                    lateral_only = False
                    if reason is None:
                        reason = 'apex ratio %.4f outside K %.4f +- %.4f' % (ratio, K, band)
                # BUG-153 run-length recovery (LATERAL_RECOVERY_RUN): a lateral-only rejection
                # extends (or, wild, restarts) this kind's run; any other row of the kind resets it
                run: List[Tuple[float, float]] = []
                recovered = False
                if lateral_only:
                    run = lat.extended_run(ix, iy)
                    if len(run) >= LATERAL_RECOVERY_RUN:
                        recovered, run, reason = True, [], None
                if reason is not None:
                    lat.run = run
                    self.n_rejected += 1
                    self.last_rejection = '%s (%s)' % (reason, kind)
                    return False
                filters = (self._fa, lat.fx, lat.fy)
                new = [f.proposed(p, z) for f, p, z in zip(filters, (pa, px, py), (za, zx, zy))]
        except (ArithmeticError, np.linalg.LinAlgError) as exc:   # pragma: no cover - defensive
            raise ValueError('the plant model update failed numerically: %s' % (exc,))
        for th, P, Pu in new:
            if not (np.all(np.isfinite(th)) and np.all(np.isfinite(P)) and np.all(np.isfinite(Pu))):
                raise ValueError('the plant model update is not finite (u=%r, y=%r); row discarded'
                                 % (u.tolist(), y.tolist()))
        for f, (th, P, Pu) in zip(filters, new):
            f.th, f.P, f.P_unforgotten = th, P, Pu
        lat.run = []
        self.n_accepted += 1
        lat.n_accepted += 1
        if recovered:
            self.n_recovered += 1
            lat.n_recovered += 1
        return True

    # ------------------------------------------------------------------ read-outs
    def theta(self) -> np.ndarray:
        """The REPORTED parameters (prior mean where still uncertain), order ``param_names``."""
        frac = self.config.fallback_frac
        return self._pack(self._fa.reported(frac), lambda f: f.reported(frac))

    def raw(self) -> np.ndarray:
        """The posterior mean, order ``param_names``."""
        return self._pack(self._fa.th, lambda f: f.th)

    def sd(self) -> np.ndarray:
        """The forgetting filter's posterior sd, order ``param_names`` (the applied value's own
        uncertainty; the inlier rule's sd_K). NOT the at-prior decision: see :meth:`sd_unforgotten`."""
        return self._pack(self._fa.sd(), lambda f: f.sd())

    def sd_unforgotten(self) -> np.ndarray:
        """The posterior sd from the information actually received (no forgetting), order
        ``param_names``: what :meth:`at_prior`, :meth:`theta` and the axis-0 guard decide on."""
        return self._pack(self._fa.sd_unforgotten(), lambda f: f.sd_unforgotten())

    def at_prior(self) -> List[str]:
        """Names of the parameters still reporting their prior (decided on :meth:`sd_unforgotten`)."""
        frac = self.config.fallback_frac
        sd0 = self._pack(self._fa.sd0, lambda f: f.sd0)
        return [n for n, s, s0 in zip(self.param_names, self.sd_unforgotten(), sd0) if s > frac * s0]

    def summary(self, site_x: Optional[float] = None) -> str:
        """One log line: K, then per release kind b [mm], s (per-parameter reported values) and its
        accepted rows, rows accepted / rejected, parameters still at their prior; with ``site_x``,
        also the axis-0 offset each kind APPLIES there (the per-query rule)."""
        th = self.theta()
        names = self.param_names
        parts = ['K %.4f' % th[0]]
        for i, kind in enumerate(RELEASE_KINDS):
            sfx = '' if i == 0 else '@' + kind
            pre = '' if i == 0 else '%s: ' % kind
            j = names.index('b_x' + sfx)
            parts += ['%sb %+.1f/%+.1f mm' % (pre, 1e3 * th[j], 1e3 * th[j + 1]), 's %+.4f/m' % th[j + 2]]
            for d in range(self._nd):
                parts.append('delta%d %+.1f/%+.1f mm' % (d + 1, 1e3 * th[j + 3 + 2 * d],
                                                         1e3 * th[j + 4 + 2 * d]))
            if site_x is not None:
                ox, p = self.offset_x(site_x, kind)
                parts.append('applied axis-0 offset at x %+.3f m: %s' % (
                    float(site_x), 'prior (0)' if p else '%+.1f mm' % (1e3 * ox)))
            parts.append('%d %s rows' % (self._lat[kind].n_accepted, kind))
        prior = self.at_prior()
        recovered = (' (%d by run-length recovery)' % self.n_recovered) if self.n_recovered else ''
        return ('plant model: %s; rows %d accepted%s / %d rejected; at prior: %s'
                % (', '.join(parts), self.n_accepted, recovered, self.n_rejected,
                   ', '.join(prior) if prior else 'none'))
