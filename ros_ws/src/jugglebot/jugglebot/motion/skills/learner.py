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
the kernel-weighted mean command of step 2. ``Theta_0 = [0, I, u_bar]``
because the identity prior ``f0(x, u) = u`` gives ``df0/dx = 0``,
``df0/du = I``, ``f0(x, u_bar) = u_bar``.

Units: SI throughout (metres, seconds) -- ``gamma`` is unit-dependent (it
competes with ``sum_i w_i * (u_i - u_bar)^2`` in m^2/s^2 in the ridge), so a
caller must never rescale ``x``/``u``/``y`` without re-tuning ``gamma``/``eta``.

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

    #: Neighbours considered (probe stage-1/2 sweep over hard-case scatter+bias).
    k: int = 16
    #: Below this many memory rows, return the identity prior untouched.
    k_min: int = 2
    #: State kernel bandwidth (m) -- site xy + seat-offset xy all share one scale.
    h_x: float = 0.01
    #: Target kernel bandwidth (m, m, s) -- must stay >= the cold-start error
    #: (probe finding 1) or the weights underflow before any correction lands.
    h_y: Tuple[float, float, float] = (0.05, 0.05, 0.2)
    #: Ridge-toward-prior weight (paper eq. S19); unit-dependent, see module docstring.
    gamma: float = 1e-2
    #: Ridge-toward-mean weight on the command solve (paper eq. 21).
    eta: float = 0.2

    def __post_init__(self):
        if not (isinstance(self.k, int) and isinstance(self.k_min, int)):
            raise ValueError('k and k_min must be ints, got k=%r k_min=%r'
                              % (self.k, self.k_min))
        if not (self.k_min >= 1 and self.k >= self.k_min):
            raise ValueError('must have k >= k_min >= 1, got k=%r k_min=%r'
                              % (self.k, self.k_min))
        if not (math.isfinite(self.h_x) and self.h_x > 0.0):
            raise ValueError('h_x must be finite and > 0, got %r' % (self.h_x,))
        h_y = tuple(float(v) for v in self.h_y)
        if len(h_y) != 3 or not all(math.isfinite(v) and v > 0.0 for v in h_y):
            raise ValueError('h_y must be a 3-tuple of finite positive floats, '
                              'got %r' % (self.h_y,))
        object.__setattr__(self, 'h_y', h_y)
        if not (math.isfinite(self.gamma) and self.gamma > 0.0):
            raise ValueError('gamma must be finite and > 0, got %r' % (self.gamma,))
        if not (math.isfinite(self.eta) and self.eta > 0.0):
            raise ValueError('eta must be finite and > 0, got %r' % (self.eta,))


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


def command(x, y_d, X, U, Y, cfg: LearnerConfig = None) -> np.ndarray:
    """The next commanded ``u`` (3,) -- plan § 2.5 steps 1-4 (step 5, clipping
    into the :class:`~jugglebot.motion.skills.admissible.AdmissibleBox`, is the
    caller's job).

    ``x`` (4,) is the query state (site xy, seat-offset xy); ``y_d`` (3,) the
    target (landing xy, flight); ``X``/``U``/``Y`` are ``(n, 4)``/``(n, 3)``/
    ``(n, 3)`` memory rows, oldest-first order irrelevant to the result other
    than as a stable tie-break (see below). Fewer than ``cfg.k_min`` rows
    returns a COPY of ``y_d`` exactly -- the identity prior; the caller may
    mutate the result freely.

    Determinism: ties in distance are broken by ``np.argsort(..., kind=
    'stable')``, i.e. by row order in ``X``/``U``/``Y``.

    Raises ``ValueError`` when the command is not finite -- the neighbourhood
    weights underflowed to 0 (``sum(w)`` rounds to 0.0, a query far outside
    every scaled neighbour) and ``u_bar`` is 0/0. There is no fallback: a
    command that cannot be computed refuses the skill by name rather than
    reaching a terminal as NaN. Weights far below ``gamma`` without
    underflowing collapse the fit onto the prior instead (R3 probe finding 1),
    which is why ``h_y`` must stay at or above the cold-start error.
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

    h_y = np.asarray(cfg.h_y, dtype=float)
    d2 = (((X - x) / cfg.h_x) ** 2).sum(axis=1) + (((Y - y_d) / h_y) ** 2).sum(axis=1)
    kk = min(cfg.k, n)
    idx = np.argsort(d2, kind='stable')[:kk]           # step 1: k nearest, stable tie-break
    w = np.exp(-d2[idx])
    Xs, Us, Ys = X[idx], U[idx], Y[idx]
    sw = w.sum()
    if not sw > 0.0:
        raise ValueError(
            'the learner command is not finite (neighbourhood weight sum %r '
            'over %d rows): no memory row is near this state and target'
            % (float(sw), kk))
    u_bar = (w[:, None] * Us).sum(axis=0) / sw          # step 2 (eq. 17)

    delta_x = Xs - x                                    # about the QUERY state (eq. S18)
    delta_u = Us - u_bar                                # about the weighted mean
    Z = np.vstack([delta_x.T, delta_u.T, np.ones((1, kk))])   # (8, kk)
    Ymat = Ys.T                                          # (3, kk)
    Theta0 = np.hstack([np.zeros((3, 4)), np.eye(3), u_bar[:, None]])  # (3, 8)

    A = (Z * w) @ Z.T + cfg.gamma * np.eye(8)            # Z W Z^T + gamma I -- SPD, gamma > 0
    B = (Ymat * w) @ Z.T + cfg.gamma * Theta0            # Y W Z^T + gamma Theta0
    Theta = np.linalg.solve(A, B.T).T                    # step 3 (eq. S19)

    D = Theta[:, 4:7]
    d = Theta[:, 7]
    u_star = u_bar + np.linalg.solve(                    # step 4 (eq. 21)
        D.T @ D + cfg.eta * np.eye(3), D.T @ (y_d - d))
    return u_star
