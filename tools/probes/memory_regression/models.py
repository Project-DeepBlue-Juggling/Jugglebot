"""Data loading, analysis-set definition, model specs and estimators for ``regress.py``.

Pure numpy/pandas, Python 3.8-compatible.  Nothing here touches the robot or the repo
packages; it only reads the learner memory corpus CSV (see ``regress.py``).

Conventions: e_i = y_i - u_i (landing minus commanded landing, axis i = 0 (x) or 1 (y),
metres); apex ratio r = y2_apex_m / u2_apex_m; a "sitting" is a run of rows with no gap
longer than ``SITTING_GAP_S``.
"""
from __future__ import annotations

import numpy as np
import pandas as pd

SITTING_GAP_S = 1800.0        # > 30 min between consecutive rows opens a new sitting
OUTLIER_LAT_M = 0.100         # |e| beyond this on either axis = miss / bad mocap fit
RATIO_LO, RATIO_HI = 1.0, 1.2  # apex ratios outside are mis-fits, not plant behaviour
CLAMP_M = 0.0199              # learner clamps u at +-20 mm; u1 <= -CLAMP_M counts as clamped


def load(csv_path: str) -> pd.DataFrame:
    """All rows, time-sorted, with ``sit`` (sitting index), ``date``, ``e0``, ``e1``, ``ratio``."""
    d = pd.read_csv(csv_path).sort_values('t_abs_s').reset_index(drop=True)
    d['sit'] = (d.t_abs_s.diff() > SITTING_GAP_S).astype(int).cumsum()
    d['date'] = pd.to_datetime(d.t_abs_s, unit='s')
    d['e0'] = d.y0 - d.u0
    d['e1'] = d.y1 - d.u1
    d['ratio'] = d.y2_apex_m / d.u2_apex_m
    d['site'] = d.x0.round(4)
    return d


def inlier_mask(d: pd.DataFrame) -> pd.Series:
    return ((d.e0.abs() < OUTLIER_LAT_M) & (d.e1.abs() < OUTLIER_LAT_M)
            & (d.ratio > RATIO_LO) & (d.ratio < RATIO_HI))


# --------------------------------------------------------------------------- estimators
def clse(X: np.ndarray, y: np.ndarray, groups) -> tuple:
    """OLS with cluster-robust (CR1) standard errors.  Returns (beta, se, resid)."""
    b = np.linalg.lstsq(X, y, rcond=None)[0]
    r = y - X @ b
    if np.linalg.matrix_rank(X) < X.shape[1]:
        raise ValueError('rank-deficient design (%d columns, rank %d): a constant or collinear column'
                         % (X.shape[1], np.linalg.matrix_rank(X)))
    xtxi = np.linalg.inv(X.T @ X)
    groups = np.asarray(groups)
    meat = np.zeros((X.shape[1],) * 2)
    for g in np.unique(groups):
        m = groups == g
        s = X[m].T @ r[m]
        meat += np.outer(s, s)
    ng = len(np.unique(groups))
    n, k = X.shape
    v = xtxi @ meat @ xtxi * ng / max(ng - 1, 1) * (n - 1) / max(n - k, 1)
    return b, np.sqrt(np.abs(np.diag(v))), r


def naive_se(X: np.ndarray, resid: np.ndarray) -> np.ndarray:
    n, k = X.shape
    return np.sqrt(np.diag(np.linalg.inv(X.T @ X)) * float(resid @ resid) / (n - k))


def dummies(key) -> np.ndarray:
    return pd.get_dummies(pd.Series(np.asarray(key))).values.astype(float)


# --------------------------------------------------------------------------- lateral model families
# name -> (offset_is_u, terms).  If offset_is_u the regression target is y - u (so the
# prediction is u + X b); otherwise the target is y itself.  terms=None is the identity.
#   one = constant, u = own-axis command, uj = other-axis command, x = x0, x2/x3 = carried
#   release offsets, a4 = 4 * commanded apex (aim-angle regressor: landing offset 4 a theta),
#   da = commanded apex minus its corpus mean.
SPEC = {
    'M0 identity': (True, None),
    'M1 pooled const': (True, ['one']),
    'M2 own-axis gain+b': (False, ['u', 'one']),
    'M3 affine A u + A2 u_other + B x0 + b': (False, ['u', 'uj', 'x', 'one']),
    'M3x M3 + x2,x3 (carried offset)': (False, ['u', 'uj', 'x', 'x2', 'x3', 'one']),
    'G1 const + x0-linear': (True, ['one', 'x']),
    'P1 aim only: u+4a*theta': (True, ['a4']),
    'P2 gain+aim: K u+4a*theta': (False, ['u', 'a4']),
    'P3 const+aim-slope: u+c+4*theta*(a-abar)': (True, ['one', 'da']),
    'P4 P3 + x0-linear': (True, ['one', 'da', 'x']),
}


class Corpus(object):
    """Analysis set plus the frame-level constants the model columns need."""

    def __init__(self, d: pd.DataFrame):
        self.all = d
        self.D = d[inlier_mask(d)].reset_index(drop=True)
        self.ubar = float(self.D.u2_apex_m.mean())

    def cols(self, df: pd.DataFrame, i: int) -> dict:
        a = df.u2_apex_m.values
        return dict(u=df['u%d' % i].values, uj=df['u%d' % (1 - i)].values, a=a,
                    one=np.ones(len(df)), x=df.x0.values, x2=df.x2.values, x3=df.x3.values,
                    da=a - self.ubar, a4=4 * a)

    def design(self, df, i, terms):
        c = self.cols(df, i)
        return np.column_stack([c[t] for t in terms])

    def target(self, df, i, offset_is_u):
        y = df['y%d' % i].values
        return y - df['u%d' % i].values if offset_is_u else y

    def fit(self, df, i, name):
        off, terms = SPEC[name]
        if terms is None:
            return None
        X = self.design(df, i, terms)
        return np.linalg.lstsq(X, self.target(df, i, off), rcond=None)[0]

    def predict(self, df, i, name, b):
        off, terms = SPEC[name]
        u = df['u%d' % i].values
        if terms is None:
            return u.copy()
        return self.design(df, i, terms) @ b + (u if off else 0.0)


def rmse_mm(df: pd.DataFrame, i: int, pred: np.ndarray) -> float:
    return 1000.0 * float(np.sqrt(np.mean((df['y%d' % i].values - pred) ** 2)))
