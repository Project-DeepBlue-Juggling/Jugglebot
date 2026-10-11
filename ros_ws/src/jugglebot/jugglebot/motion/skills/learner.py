"""``motion/skills/learner`` -- the memory-based command learner (plan § 2.5).

Given a target landing ``y_d``, the current state ``x`` and a memory of past
(``x``, ``u``, ``y``) rows, :func:`command` returns the next commanded
``u`` -- a local, kernel-weighted affine correction of the identity prior
``u = y_d`` (paper arXiv:2608.26800v2, eq. 17, S18/S19, 21). This module is
the closed form only: it does not clip into an :class:`AdmissibleBox` (that
is the executor's job, via ``admissible.clip``) and it does not touch a
memory's storage (that is ``memory.Memory``).

Centring (paper eq. S18, verbatim, confirmed by the R3 hyperparameter probe
-- the plan text alone does not state it): ``delta_x_i = x_i - x`` is about
the QUERY state, never a weighted mean; ``delta_u_i = u_i - u_bar`` is about
the kernel-weighted mean command of step 2. The paper's ``Theta_0 = [0, I,
u_bar]`` comes from the identity prior ``f0(x, u) = u`` (``df0/dx = 0``,
``df0/du = I``, ``f0(x, u_bar) = u_bar``); this module shifts that prior by
the neighbourhood's mean residual, ``Theta_0 = [0, I, u_bar + r_bar]``
(2026-10-12, :func:`_local_fit`). A cold memory still returns the identity
prior exactly -- the shift exists only where support rows exist.

Units: SI throughout (metres, seconds) -- ``gamma`` is unit-dependent (it
competes with ``sum_i v_i * (u_i - u_bar)^2`` in m^2 in the ridge), so a
caller must never rescale ``x``/``u``/``y`` without re-tuning ``gamma``/``eta``.

**The forward model never sees a sample chosen by its own outcome** (A2,
2026-10-04). Invariant: which rows enter the local fit, and the weight each
carries IN THE FIT, must not depend on how close the row's observed outcome
``y_i`` came to the target ``y_d`` -- except through a gross gate far outside
the plant's scatter. Enforcement point: :func:`_neighbourhood` (selection)
and :func:`_local_fit` (fit weights). Pinned by ``test_learner.py``'s
real-memory and frozen-learner tests.

Why, by mechanism. The fit's intercept ``d`` is the model's belief of where
the plant lands from ``u_bar``; the step is ``y_d - d``. The paper's law
(eq. 17) takes the k NEAREST rows in the joint (state, OUTCOME) metric and
weights the fit by the same kernel. Once more than k rows share a state --
159 self-tosses at P1 by 2026-10-02 -- "the k nearest" are the k rows whose
balls happened to land closest to the target: an order statistic of the
NOISE (landing sd ~23 mm against a command spread of ~1.4 mm). Their mean
landing is ``y_d`` by construction, so ``d ~= y_d``, the step vanishes and
the command freezes at whatever ``u_bar`` the history left it. Replayed on
the real memory: ``d_y = -1.0`` mm against a plant that lands ~+10 mm beyond
its command, step +0.9 mm, command ``u_y = +4.1`` mm -- the wrong sign; R4's
smaller memory (fewer than k rows per state, so no truncation) gave -5.4 mm.
The slope ``D`` was NOT the cause: with u nearly constant the data's
information on it is ~3e-5 against gamma = 1e-2, so ``D`` is the identity
prior to 1 % either way. The soft kernel is the same defect class in a
milder form: weighting the fit by ``exp(-|y_i - y_d|^2/h_y^2)`` drags ``d``
toward ``y_d`` by ~30 % at this scatter, and further on a 16-row sample.

The law as implemented (plan § 2.5, steps 1 and 3 changed; 2 and 4 as the
paper):

1. Support: rows with the paper's joint ``d2_i <= support_d2`` (3 bandwidths).
   Neighbourhood: the ``k`` MOST RECENT support rows (row order = append
   order). Time is independent of a throw's noise, so this sample is not
   selected on the outcome. Fewer than ``k_min`` support rows -> the
   identity prior, exactly as for a cold memory.
2. Anchor ``u_bar``: kernel mean of the neighbourhood's commands with the
   paper's joint weights ``w_i = exp(-d2_i)`` -- the inverse lookup "commands
   whose throws landed near the target". The outcome may choose WHERE the
   step starts from; that is what keeps a near-deterministic plant converging
   in a handful of throws.
3. Forward fit ``(C, D, d)``: eq. S19 with STATE-only weights
   ``v_i = exp(-|x_i - x|^2/h_x^2)`` -- the outcome never decides what the
   plant is believed to do at ``u_bar``.
4. Unchanged (eq. 21).

Failure modes of this law, enumerated:

* **Window lag.** A plant change is absorbed over ~k throws at its state,
  not instantly; the old law did not absorb one at all once frozen.
* **Window noise.** ``d`` is a k-row mean: command jitter ~ s / (sqrt(k) *
  (1 + eta)), ~4-5 mm at s = 23 mm, k = 16 (synthetic loop, 8 seeds) --
  ~0.4 mm added to a 23 mm landing sd, against an unbounded frozen bias.
* **Outliers inside the gate** (closer than 3 h_y = 150 mm lateral / 0.3 m
  apex) carry full fit weight: one row moves ``d`` by up to ~150/k mm for k
  throws. The apex band, the quarantine convention, the lateral authority
  clip and the admissible box bound what reaches the platform.
* **Gate asymmetry.** The gate is centred on ``y_d``, so a plant biased by
  ~3 bandwidths loses its tail rows on one side. That is the cold-start
  regime in which the old kernel's weights (< e^-9 each against gamma)
  already collapsed onto the prior (R3 probe finding 1); unchanged.
* **Order dependence.** The window follows row order: a caller must pass
  rows oldest-first (``Memory`` does: append-only file, read in order).
* **Wasted slots.** Support rows at a different state (low ``v``) still
  occupy the window; a small effective sample leans the fit on the prior,
  the conservative direction.
* **Mixed operating points inside the gate** (e.g. apex targets < 0.3 m
  apart) share one affine fit; the joint-weighted anchor separates them and
  ``D`` is identified exactly when their commands are spread apart (the
  identity prior otherwise). Residual: the plant's non-affinity across them.
* **Thin state support inside the gate** (support rows present, but all at
  a different state, so ``sum v`` falls toward ``gamma``): the ridge pulls
  the fit onto its prior. Until 2026-10-12 that prior was "apex = command",
  ~75 mm wrong for this plant, and the command walked UP toward ``y_d``
  (sitting 7b rows 465/467, sum v 0.03/0.02: u_apex 0.891/0.901 against a
  supported 0.865-0.882; the second ball's catch was refused LIMIT_JERK).
  Closed twice: the seat-offset half of ``x`` no longer partitions the
  memory (``LearnerConfig.h_seat``), and the prior carries the window's
  mean residual (:func:`_local_fit`), so thin support degrades to "what
  this site's rows say the plant does". Replayed on the real memory: the
  two rows return 0.874; well-supported rows move <= 1.9 mm in apex from
  the prior shift alone.
* **No row near the query** now returns the identity prior instead of
  raising on an underflowed weight sum (the gate guarantees every weight is
  >= e^-support_d2). A non-finite result still raises by name.

Pure Python + numpy. No ROS2 imports (the ``motion/`` rule).
"""

from __future__ import annotations

import dataclasses
import math
from typing import Tuple

import numpy as np

__all__ = ['LearnerConfig', 'command']


@dataclasses.dataclass(frozen=True)
class LearnerConfig:
    """Hyperparameters pinned by the R3 probe (owner decision, 2026-09-13;
    ``probe_learner.py``, logbook slug
    ``2026-09-13-skill-stack-r3-learner-single-site``). SI units.
    """

    #: Neighbours considered (probe stage-1/2 sweep over hard-case scatter+bias):
    #: the k MOST RECENT support rows since 2026-10-04 (module docstring).
    k: int = 16
    #: Below this many SUPPORT rows (``d2 <= support_d2``), return the
    #: identity prior untouched.
    k_min: int = 2
    #: State kernel bandwidth (m) on the SITE half of ``x`` (``x[0:2]``).
    #: Sites are >= 62.5 mm apart, so 10 mm keeps every site's rows separate
    #: (the support gate's 3 bandwidths = 30 mm).
    h_x: float = 0.01
    #: State kernel bandwidth (m) on the SEAT-OFFSET half of ``x``
    #: (``x[2:4]``, the carried release's xy off its site). INERT by design
    #: (2026-10-12, U1): a row at the far corner of the lateral authority
    #: from the query (2 x 20 mm per axis, 56.6 mm) keeps
    #: ``exp(-(0.0566/1)^2) = 0.997`` of a matched row's weight. Until then
    #: the offset shared ``h_x`` and partitioned the memory into 10 mm cells;
    #: a throw from a sparsely visited corner found support rows only at
    #: ``v ~ e^-7`` and the fit fell to the identity prior (sitting 7b rows
    #: 465/467: u_apex 0.891/0.901 against 0.865-0.882, the second's catch
    #: refused LIMIT_JERK). The pre-registered test on the real memory found
    #: no offset effect above noise on any channel (landing x/y, apex):
    #: |t| <= 2.5 and <= 0.4 sd for a full 20 mm offset. The offset still
    #: enters the fit's ``delta_x`` regressor, so a real slope remains
    #: learnable -- across the whole site instead of per cell. If a later
    #: measurement finds one, this is the knob to shrink, to the measured
    #: scale.
    h_seat: float = 1.0
    #: Target kernel bandwidth (m, m, m) -- must stay >= the cold-start error
    #: (probe finding 1) or the weights underflow before any correction lands.
    #:
    #: The third entry became an APEX bandwidth on 2026-09-18, when the
    #: learner's third channel stopped being a flight time. 0.10 m is ~3x the
    #: measured apex scatter at a fixed command (+-0.02-0.04 m, 2026-09-17),
    #: so a neighbour is a throw aimed at a comparable height rather than any
    #: throw at all: the retired 0.2 s was 1.5x the ENTIRE explored command
    #: range (0.569-0.6996 s), which made every row a neighbour and the
    #: "local" fit a global one over noise.
    h_y: Tuple[float, float, float] = (0.05, 0.05, 0.10)
    #: Ridge-toward-prior weight (paper eq. S19); unit-dependent, see module docstring.
    gamma: float = 1e-2
    #: Ridge-toward-mean weight on the command solve (paper eq. 21).
    eta: float = 0.2
    #: Support gate on the joint metric ``d2`` (dimensionless, 3 bandwidths).
    #: A row beyond it carries < e^-9 = 1.2e-4 of a matched row's kernel
    #: weight -- where the paper's kernel already stopped carrying the fit
    #: against ``gamma`` (k * e^-9 = 2e-3 < 1e-2) -- and lands 6.5 measured
    #: landing sd (23 mm, 2026-10-02) from the target laterally, so the gate
    #: rejects other flights and other operating points without ranking rows
    #: inside the noise (module docstring).
    support_d2: float = 9.0

    def __post_init__(self):
        if not (isinstance(self.k, int) and isinstance(self.k_min, int)):
            raise ValueError('k and k_min must be ints, got k=%r k_min=%r'
                              % (self.k, self.k_min))
        if not (self.k_min >= 1 and self.k >= self.k_min):
            raise ValueError('must have k >= k_min >= 1, got k=%r k_min=%r'
                              % (self.k, self.k_min))
        if not (math.isfinite(self.h_x) and self.h_x > 0.0):
            raise ValueError('h_x must be finite and > 0, got %r' % (self.h_x,))
        if not (math.isfinite(self.h_seat) and self.h_seat > 0.0):
            raise ValueError('h_seat must be finite and > 0, got %r'
                              % (self.h_seat,))
        h_y = tuple(float(v) for v in self.h_y)
        if len(h_y) != 3 or not all(math.isfinite(v) and v > 0.0 for v in h_y):
            raise ValueError('h_y must be a 3-tuple of finite positive floats, '
                              'got %r' % (self.h_y,))
        object.__setattr__(self, 'h_y', h_y)
        if not (math.isfinite(self.gamma) and self.gamma > 0.0):
            raise ValueError('gamma must be finite and > 0, got %r' % (self.gamma,))
        if not (math.isfinite(self.eta) and self.eta > 0.0):
            raise ValueError('eta must be finite and > 0, got %r' % (self.eta,))
        if not (math.isfinite(self.support_d2) and self.support_d2 > 0.0):
            raise ValueError('support_d2 must be finite and > 0, got %r'
                              % (self.support_d2,))


def _vec(value, n: int, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float)
    if arr.shape != (n,):
        raise ValueError('%s must be a %d-vector, got shape %s'
                          % (name, n, np.shape(value)))
    if not np.all(np.isfinite(arr)):
        raise ValueError('%s must be finite, got %r' % (name, arr.tolist()))
    return arr


def _leading_matrix(value, n_cols: int, name: str) -> np.ndarray:
    """Validate ``value`` as ``(n, n_cols)`` for any ``n >= 0`` and return it."""
    arr = np.asarray(value, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != n_cols:
        raise ValueError('%s must have shape (n, %d), got %s'
                          % (name, n_cols, np.shape(value)))
    return arr


def _matrix(value, n_rows: int, n_cols: int, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float)
    if arr.ndim != 2 or arr.shape[0] != n_rows or arr.shape[1] != n_cols:
        raise ValueError('%s must have shape (%d, %d), got %s'
                          % (name, n_rows, n_cols, np.shape(value)))
    return arr


def _state_d2(X, x, cfg: LearnerConfig) -> np.ndarray:
    """Each row's squared STATE distance from the query, in bandwidths:
    ``h_x`` on the site half, ``h_seat`` on the seat-offset half. The one
    definition both the support gate and the fit's state kernel use."""
    h = np.array([cfg.h_x, cfg.h_x, cfg.h_seat, cfg.h_seat])
    return (((X - x) / h) ** 2).sum(axis=1)


def _neighbourhood(x, y_d, X, Y, cfg: LearnerConfig):
    """``(idx, d2)``: the neighbourhood rows (oldest-first indices into
    ``X``/``Y``, at most ``cfg.k``) and every row's joint ``d2``.

    THE enforcement point of "never select on the outcome": ``y_i`` enters
    only through the ``d2 <= support_d2`` gate, never through a rank. The
    window is the most recent ``k`` support rows (row order = append order).
    """
    h_y = np.asarray(cfg.h_y, dtype=float)
    d2 = _state_d2(X, x, cfg) + (((Y - y_d) / h_y) ** 2).sum(axis=1)
    support = np.flatnonzero(d2 <= cfg.support_d2)
    return support[-cfg.k:], d2


def _local_fit(x, Xs, Us, Ys, d2s, cfg: LearnerConfig):
    """``(u_bar, Theta)`` over the neighbourhood rows (plan § 2.5 steps 2-3).

    ``u_bar`` is the paper's joint-kernel mean of the commands (eq. 17), the
    anchor of the step. ``Theta = [C, D, d]`` (3, 8) is eq. S19 weighted by
    the STATE kernel only, so the intercept ``d`` -- where the plant is
    believed to land from ``u_bar`` -- is not dragged toward ``y_d`` by
    rows chosen for having landed there (module docstring).

    The ridge prior ``Theta0`` is the identity SHIFTED by the neighbourhood's
    mean residual ``r_bar = mean(y_i - u_i)``: ``f0(x, u) = u + r_bar``, so
    ``d0 = u_bar + r_bar`` (2026-10-12, U1). Where the state kernel carries
    the fit (``sum v >> gamma``) the prior is invisible; where it does not --
    support rows that are all at a different state -- the fit falls back to
    what those rows say the plant does, not to "apex = command", which this
    plant misses by ~75 mm. ``r_bar`` is UNWEIGHTED over the window, so the
    outcome never weights it (the A2 invariant: the window itself is chosen
    by recency inside the gate).
    """
    kk = Xs.shape[0]
    w = np.exp(-d2s)
    sw = w.sum()
    u_bar = (w[:, None] * Us).sum(axis=0) / sw if sw > 0.0 else None
    if u_bar is None or not np.all(np.isfinite(u_bar)):  # step 2 (eq. 17)
        raise ValueError(
            'the learner command is not finite (neighbourhood weight sum %r, '
            'u_bar %r over %d rows)'
            % (float(sw), None if u_bar is None else u_bar.tolist(), kk))

    v = np.exp(-_state_d2(Xs, x, cfg))                  # state kernel only
    delta_x = Xs - x                                    # about the QUERY state (eq. S18)
    delta_u = Us - u_bar                                # about the weighted mean
    Z = np.vstack([delta_x.T, delta_u.T, np.ones((1, kk))])   # (8, kk)
    r_bar = (Ys - Us).mean(axis=0)                      # the residual prior (docstring)
    Theta0 = np.hstack([np.zeros((3, 4)), np.eye(3), (u_bar + r_bar)[:, None]])  # (3, 8)
    A = (Z * v) @ Z.T + cfg.gamma * np.eye(8)            # Z V Z^T + gamma I -- SPD, gamma > 0
    B = (Ys.T * v) @ Z.T + cfg.gamma * Theta0            # Y V Z^T + gamma Theta0
    Theta = np.linalg.solve(A, B.T).T                    # step 3 (eq. S19)
    return u_bar, Theta


def command(x, y_d, X, U, Y, cfg: LearnerConfig = None) -> np.ndarray:
    """The next commanded ``u`` (3,) -- plan § 2.5 steps 1-4 as amended in the
    module docstring (step 5, clipping into the
    :class:`~jugglebot.motion.skills.admissible.AdmissibleBox`, is the
    caller's job).

    ``x`` (4,) is the query state (site xy, seat-offset xy); ``y_d`` (3,) the
    target (landing xy, apex); ``X``/``U``/``Y`` are ``(n, 4)``/``(n, 3)``/
    ``(n, 3)`` memory rows, OLDEST-FIRST: the neighbourhood is the most
    recent ``cfg.k`` rows inside the support gate, so row order is part of
    the input. Fewer than ``cfg.k_min`` rows in the memory, or inside the
    gate, returns a COPY of ``y_d`` exactly -- the identity prior; the caller
    may mutate the result freely.

    Determinism: a pure function of its arguments (no RNG, no clock).

    Raises ``ValueError`` when the command is not finite (e.g. a non-finite
    memory row inside the gate). There is no fallback: a command that cannot
    be computed refuses the skill by name rather than reaching a terminal as
    NaN.
    """
    if cfg is None:
        cfg = LearnerConfig()
    x = _vec(x, 4, 'x')
    y_d = _vec(y_d, 3, 'y_d')
    X = _leading_matrix(X, 4, 'X')
    n = X.shape[0]
    U = _matrix(U, n, 3, 'U')
    Y = _matrix(Y, n, 3, 'Y')

    if n < cfg.k_min:
        return y_d.copy()
    idx, d2 = _neighbourhood(x, y_d, X, Y, cfg)          # step 1
    if idx.size < cfg.k_min:
        return y_d.copy()
    u_bar, Theta = _local_fit(x, X[idx], U[idx], Y[idx], d2[idx], cfg)

    D = Theta[:, 4:7]
    d = Theta[:, 7]
    u_star = u_bar + np.linalg.solve(                    # step 4 (eq. 21)
        D.T @ D + cfg.eta * np.eye(3), D.T @ (y_d - d))
    if not np.all(np.isfinite(u_star)):
        raise ValueError('the learner command is not finite (%r) over %d '
                         'neighbourhood rows' % (u_star.tolist(), idx.size))
    return u_star
