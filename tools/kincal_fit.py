#!/usr/bin/env python3
"""Kinematic calibration fit — the Stewart geometry from a mocap pose sweep.

Plan: ``plans/active/kinematic-calibration.md`` (the design, its decisions and
its pass criteria; §§ referenced below are that plan's).

What it fits
------------
The model is exactly the production IK (``motion/ik_solver.py``)::

    l_i = | c + R p_i - b_i |          leg i joint-centre-to-joint-centre, mm
    l_i - L0_i = rev_i / k_i           extension from the motor, k = mm_to_rev

against a capture in which, at every dwell pose, the mocap ``Platform`` body
gives ``c`` (its origin was placed at the leg-joint centroid with the owner's
jig, so it is trusted) and an attitude ``R_m`` whose axes were set parallel to
global at body definition, so the true joint-pattern attitude is
``R = R_m · Exp(reg)`` with ``reg`` a fitted 3-vector (§ 2). The ENCODER
revolutions are the measurement being reproduced — never the commanded ones,
so servo tracking error cannot leak into the geometry, and no forward
kinematics sits inside the fit.

Parameters (51): base nodes (18), platform nodes (18), leg zero lengths L0 (6),
leg scales k (6), registration (3). All carry a weak prior toward the CAD /
current config value. The platform-node pattern's rigid motion is the gauge
the registration would otherwise be confused with, so six linear constraints
pin it: the node deltas have zero sum (the centroid stays at the trusted
origin) and zero net rotation (``sum p_i x d_i = 0`` — zero attitude is the
leg-joint pattern, § 2, owner Q14). A parameter the data cannot pin
(posterior sd over the § 2 thresholds) is FROZEN at its prior value and the
fit re-run; the report names every frozen parameter and why.

Capture CSV (written by ``tests/hardware/kincal_capture.py``)
-------------------------------------------------------------
One row per dwell (means over the dwell window). Required columns::

    pose_id        str   the commanded pose's id (same id = same command)
    role           fit | holdout     holdout rows never enter the fit
    x_mm y_mm z_mm       mocap Platform origin, Base frame (QTM global), mm
    qw qx qy qz          mocap Platform attitude, body -> Base
    rev_0 .. rev_5       leg motor position, encoder, revolutions

Optional (empty = not applicable)::

    repeat_group   str   rows of one § 3 two-direction repeat pose
    direction      str   approach label inside a repeat group (e.g. A / B)
    phase          pre_home | post_home     the § 4 re-home subset

``post_home`` rows are a different homing and never enter the geometry fit;
they are used only for the re-home verdict. Unknown columns are ignored.

Outputs (``--out-dir``, default ``temp/reports/kincal/<capture stem>/``)
------------------------------------------------------------------------
``report.md`` (the verdicts and § 8 criteria), ``result.json`` and
``proposed_geometry.yaml``. **It never writes ``config/hardware_config.yaml``**
— applying the geometry is a deliberate commit (§ 6 step 5).

Usage
-----
    python tools/kincal_fit.py temp/logs/kincal_capture_<ts>.csv

    # the per-session homing re-fit (§ 4): only the six L0 are free
    python tools/kincal_fit.py temp/logs/kincal_check_<ts>.csv \\
        --offsets-only --geometry temp/reports/kincal/<run>/proposed_geometry.yaml

Offline analysis only: no ROS, no hardware, no motion.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import os
import sys
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional, Sequence, Tuple

# One BLAS thread, before numpy loads (as run_tests.sh does): on the Jetson a
# multi-threaded OpenBLAS under load turns a 400x51 SVD from ~20 ms into ~4 s.
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')

import numpy as np  # noqa: E402
import yaml  # noqa: E402
from scipy.optimize import least_squares  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_SCRIPT_DIR)
_HW_CONFIG_PY = os.path.join(_REPO_ROOT, 'config', 'generated',
                             'hardware_config.py')

N_LEGS = 6
GROUPS = ('base', 'plat', 'L0', 'k', 'reg')
_GROUP_SIZE = {'base': 18, 'plat': 18, 'L0': 6, 'k': 6, 'reg': 3}
N_PARAMS = sum(_GROUP_SIZE.values())

# ── Noise and prior (§ 2) ────────────────────────────────────────────────────
#: Leg-space measurement sd, mm: mocap origin (~0.1-0.2 mm at rest) propagated
#: through the leg directions plus encoder quantisation. Sets the chi-square
#: scale only; the posterior sd is re-scaled by the achieved reduced chi-square.
DEFAULT_SIGMA_MM = 0.3
#: Weak prior toward CAD. Wide on purpose: it only has to hold parameters the
#: data cannot see, not pull the ones it can.
PRIOR_SD = {'base': 5.0, 'plat': 5.0, 'L0': 10.0,
            'k_rel': 0.02, 'reg': 0.1}
#: Gauge rows are constraints, not observations.
GAUGE_WEIGHT = 1.0e3

# ── Identifiability freeze thresholds (§ 2, owner Q16) ───────────────────────
FREEZE_SD = {'base': 1.0, 'plat': 1.0, 'L0': 1.0,
             'k_rel': 0.002,                 # 0.2 % = 0.56 mm over the stroke
             'reg': math.radians(0.1)}

# ── Verdict thresholds (§§ 3, 4, 8 — owner-agreed) ───────────────────────────
REPEAT_SPREAD_MM = 1.0
REHOME_MM = 1.0
PASS_RMS_MM = 1.0
PASS_MAX_MM = 2.0
PASS_ATT_DEG = 0.1
PLAUSIBLE_NODE_MM = 5.0


# ═════════════════════════════════════════════════════════════════════════════
# Geometry
# ═════════════════════════════════════════════════════════════════════════════

@dataclass
class KinGeom:
    """The calibrated quantities. Positions in mm, ``k`` in rev/mm."""

    base: np.ndarray          # (6, 3) base-frame joint centres
    plat: np.ndarray          # (6, 3) platform-frame joint centres
    L0: np.ndarray            # (6,)   joint-to-joint length at motor 0
    k: np.ndarray             # (6,)   mm_to_rev
    reg: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def to_vector(self) -> np.ndarray:
        return np.concatenate([self.base.ravel(), self.plat.ravel(),
                               self.L0, self.k, self.reg]).astype(float)

    @classmethod
    def from_vector(cls, v: np.ndarray) -> 'KinGeom':
        v = np.asarray(v, dtype=float)
        return cls(base=v[0:18].reshape(6, 3).copy(),
                   plat=v[18:36].reshape(6, 3).copy(),
                   L0=v[36:42].copy(), k=v[42:48].copy(),
                   reg=v[48:51].copy())

    def copy(self) -> 'KinGeom':
        return KinGeom.from_vector(self.to_vector())

    def to_dict(self) -> dict:
        return {'base_nodes_mm': np.round(self.base, 4).tolist(),
                'init_plat_nodes_mm': np.round(self.plat, 4).tolist(),
                'init_leg_lengths_mm': np.round(self.L0, 4).tolist(),
                'mm_to_rev': np.round(self.k, 9).tolist(),
                'mocap_registration_rotvec_rad': np.round(self.reg, 7).tolist()}

    @classmethod
    def from_dict(cls, d: dict) -> 'KinGeom':
        return cls(base=np.array(d['base_nodes_mm'], dtype=float),
                   plat=np.array(d['init_plat_nodes_mm'], dtype=float),
                   L0=np.array(d['init_leg_lengths_mm'], dtype=float),
                   k=np.array(d['mm_to_rev'], dtype=float),
                   reg=np.array(d.get('mocap_registration_rotvec_rad',
                                      [0.0, 0.0, 0.0]), dtype=float))


def _load_hw():
    spec = importlib.util.spec_from_file_location('_kincal_hw', _HW_CONFIG_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def nominal_geometry() -> KinGeom:
    """The current config (CAD) geometry, in absolute Base-frame terms.

    ``L0`` is ``init_leg_lengths_mm``: the model's joint-to-joint length at
    motor 0 (STOW), which ``pose_to_leg_lengths`` subtracts.
    """
    hw = _load_hw()
    return KinGeom(base=np.array(hw.GEOM_BASE_NODES_MM, dtype=float),
                   plat=np.array(hw.GEOM_INIT_PLAT_NODES_MM, dtype=float),
                   L0=np.array(hw.GEOM_INIT_LEG_LENGTHS_MM, dtype=float),
                   k=np.array(hw.GEOM_MM_TO_REV, dtype=float),
                   reg=np.zeros(3))


def nominal_init_height_mm() -> float:
    return float(_load_hw().GEOM_INITIAL_HEIGHT_MM)


def leg_lengths(c: np.ndarray, R: np.ndarray, base: np.ndarray,
                plat: np.ndarray) -> np.ndarray:
    """Joint-to-joint leg lengths. ``c`` (N,3), ``R`` (N,3,3) -> (N,6)."""
    P = c[:, None, :] + np.einsum('nij,kj->nki', R, plat)
    return np.linalg.norm(P - base[None, :, :], axis=2)


def rev_to_length(rev: np.ndarray, g: KinGeom) -> np.ndarray:
    return g.L0[None, :] + np.asarray(rev) / g.k[None, :]


def length_to_rev(length: np.ndarray, g: KinGeom) -> np.ndarray:
    return (np.asarray(length) - g.L0[None, :]) * g.k[None, :]


def _exp(rotvec: np.ndarray) -> np.ndarray:
    return Rotation.from_rotvec(np.asarray(rotvec, dtype=float)).as_matrix()


def quat_wxyz_to_matrix(q: np.ndarray) -> np.ndarray:
    q = np.atleast_2d(np.asarray(q, dtype=float))
    return Rotation.from_quat(q[:, [1, 2, 3, 0]]).as_matrix()


def matrix_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    q = Rotation.from_matrix(R).as_quat()
    q = np.atleast_2d(q)
    return q[:, [3, 0, 1, 2]]


# ═════════════════════════════════════════════════════════════════════════════
# Capture
# ═════════════════════════════════════════════════════════════════════════════

@dataclass
class Capture:
    pose_id: np.ndarray       # (N,) str
    role: np.ndarray          # (N,) 'fit' | 'holdout'
    pos: np.ndarray           # (N, 3) mm
    quat: np.ndarray          # (N, 4) wxyz
    rev: np.ndarray           # (N, 6)
    repeat_group: np.ndarray  # (N,) str, '' = none
    direction: np.ndarray     # (N,) str
    phase: np.ndarray         # (N,) str, '' | pre_home | post_home

    def __len__(self) -> int:
        return len(self.pose_id)

    def subset(self, mask: np.ndarray) -> 'Capture':
        return Capture(**{f: getattr(self, f)[mask] for f in (
            'pose_id', 'role', 'pos', 'quat', 'rev', 'repeat_group',
            'direction', 'phase')})

    @property
    def R_mocap(self) -> np.ndarray:
        return quat_wxyz_to_matrix(self.quat)


_REQUIRED = (['pose_id', 'role', 'x_mm', 'y_mm', 'z_mm', 'qw', 'qx', 'qy', 'qz']
             + ['rev_%d' % i for i in range(N_LEGS)])


def read_capture(path: str) -> Capture:
    with open(path, newline='', encoding='utf-8') as fh:
        rows = [r for r in csv.DictReader(
            (line for line in fh if not line.startswith('#')))]
    if not rows:
        raise ValueError('%s: no rows' % path)
    missing = [c for c in _REQUIRED if c not in rows[0]]
    if missing:
        raise ValueError('%s: missing columns %s' % (path, missing))
    roles = [r['role'].strip() for r in rows]
    bad = sorted(set(roles) - {'fit', 'holdout'})
    if bad:
        raise ValueError('%s: unknown role(s) %s' % (path, bad))

    def col(name):
        return np.array([(r.get(name) or '').strip() for r in rows], dtype=object)

    def num(names):
        return np.array([[float(r[n]) for n in names] for r in rows], dtype=float)

    return Capture(pose_id=col('pose_id'), role=np.array(roles, dtype=object),
                   pos=num(['x_mm', 'y_mm', 'z_mm']),
                   quat=num(['qw', 'qx', 'qy', 'qz']),
                   rev=num(['rev_%d' % i for i in range(N_LEGS)]),
                   repeat_group=col('repeat_group'), direction=col('direction'),
                   phase=col('phase'))


def write_capture(path: str, cap: Capture) -> None:
    """Write the CSV format ``read_capture`` reads (the capture tool's writer
    and the tests share it, so the format has one definition)."""
    cols = (['pose_id', 'role', 'repeat_group', 'direction', 'phase',
             'x_mm', 'y_mm', 'z_mm', 'qw', 'qx', 'qy', 'qz']
            + ['rev_%d' % i for i in range(N_LEGS)])
    with open(path, 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for i in range(len(cap)):
            w.writerow([cap.pose_id[i], cap.role[i], cap.repeat_group[i],
                        cap.direction[i], cap.phase[i]]
                       + ['%.4f' % v for v in cap.pos[i]]
                       + ['%.9f' % v for v in cap.quat[i]]
                       + ['%.6f' % v for v in cap.rev[i]])


# ═════════════════════════════════════════════════════════════════════════════
# Fit
# ═════════════════════════════════════════════════════════════════════════════

def _group_slices() -> Dict[str, slice]:
    out, i = {}, 0
    for g in GROUPS:
        out[g] = slice(i, i + _GROUP_SIZE[g])
        i += _GROUP_SIZE[g]
    return out


_SLICES = _group_slices()


def param_names() -> List[str]:
    names = []
    for i in range(N_LEGS):
        names += ['base[%d].%s' % (i, a) for a in 'xyz']
    for i in range(N_LEGS):
        names += ['plat[%d].%s' % (i, a) for a in 'xyz']
    names += ['L0[%d]' % i for i in range(N_LEGS)]
    names += ['k[%d]' % i for i in range(N_LEGS)]
    names += ['reg.%s' % a for a in 'xyz']
    return names


def group_of(index: int) -> str:
    for g, s in _SLICES.items():
        if s.start <= index < s.stop:
            return g
    raise IndexError(index)


def _prior_sd(prior: KinGeom) -> np.ndarray:
    sd = np.empty(N_PARAMS)
    sd[_SLICES['base']] = PRIOR_SD['base']
    sd[_SLICES['plat']] = PRIOR_SD['plat']
    sd[_SLICES['L0']] = PRIOR_SD['L0']
    sd[_SLICES['k']] = PRIOR_SD['k_rel'] * np.abs(prior.k)
    sd[_SLICES['reg']] = PRIOR_SD['reg']
    return sd


def _freeze_sd(prior: KinGeom) -> np.ndarray:
    sd = np.empty(N_PARAMS)
    sd[_SLICES['base']] = FREEZE_SD['base']
    sd[_SLICES['plat']] = FREEZE_SD['plat']
    sd[_SLICES['L0']] = FREEZE_SD['L0']
    sd[_SLICES['k']] = FREEZE_SD['k_rel'] * np.abs(prior.k)
    sd[_SLICES['reg']] = FREEZE_SD['reg']
    return sd


def free_mask(groups: Sequence[str]) -> np.ndarray:
    m = np.zeros(N_PARAMS, dtype=bool)
    for g in groups:
        m[_SLICES[g]] = True
    return m


def _gauge_rows(theta: np.ndarray, prior: KinGeom) -> np.ndarray:
    """Six constraint residuals pinning the platform pattern's rigid motion."""
    d = theta[_SLICES['plat']].reshape(6, 3) - prior.plat
    return np.concatenate([d.sum(axis=0),
                           np.cross(prior.plat, d).sum(axis=0)])


@dataclass
class FitResult:
    geom: KinGeom
    free: np.ndarray                 # (51,) bool — what was actually fitted
    frozen: List[Tuple[str, float]]  # (name, posterior sd that froze it)
    sd: np.ndarray                   # (51,) posterior sd, NaN where not free
    rms_mm: float                    # leg-space residual over the fit rows
    max_mm: float
    reduced_chi2: float
    n_rows: int
    success: bool
    message: str


def _gauge_jacobian(prior: KinGeom) -> np.ndarray:
    """d(gauge rows)/d(theta): linear in the platform nodes, constant."""
    G = np.zeros((6, N_PARAMS))
    s = _SLICES['plat'].start
    for i in range(N_LEGS):
        cols = slice(s + 3 * i, s + 3 * i + 3)
        G[0:3, cols] = np.eye(3)
        p = prior.plat[i]
        # d/dd (p x d) = [p]x
        G[3:6, cols] = np.array([[0.0, -p[2], p[1]],
                                 [p[2], 0.0, -p[0]],
                                 [-p[1], p[0], 0.0]])
    return G


def _meas_residual_and_jacobian(cap: Capture, R_m: np.ndarray, g: KinGeom):
    """Leg-space residual ``pred - meas`` (N,6) and its Jacobian (N*6, 51).

    Analytic: with unit leg vector ``u``, ``dl/db = -u``, ``dl/dp = R^T u``,
    ``d(meas)/dL0 = 1``, ``d(meas)/dk = -rev/k^2``; the registration uses the
    small-rotation derivative ``dl/d(reg) = p x (R^T u)``, exact at ``reg = 0``
    and first-order accurate at the sub-degree values it takes — it steers
    the solve and sizes the posterior, and the residual itself is exact.
    """
    R = R_m @ _exp(g.reg)
    P = cap.pos[:, None, :] + np.einsum('nij,kj->nki', R, g.plat)
    V = P - g.base[None, :, :]
    ell = np.linalg.norm(V, axis=2)
    U = V / ell[:, :, None]                                  # (N,6,3)
    meas = rev_to_length(cap.rev, g)
    e = ell - meas
    n = len(cap)
    J = np.zeros((n, N_LEGS, N_PARAMS))
    RtU = np.einsum('nji,nkj->nki', R, U)                    # R^T u, (N,6,3)
    for i in range(N_LEGS):
        b = _SLICES['base'].start + 3 * i
        p = _SLICES['plat'].start + 3 * i
        J[:, i, b:b + 3] = -U[:, i, :]
        J[:, i, p:p + 3] = RtU[:, i, :]
        J[:, i, _SLICES['L0'].start + i] = -1.0
        J[:, i, _SLICES['k'].start + i] = cap.rev[:, i] / g.k[i] ** 2
        J[:, i, _SLICES['reg']] = np.cross(g.plat[i][None, :], RtU[:, i, :])
    return e, J.reshape(n * N_LEGS, N_PARAMS)


def _solve(cap: Capture, start: KinGeom, prior: KinGeom, free: np.ndarray,
           sigma_mm: float):
    theta0 = start.to_vector()
    ptheta = prior.to_vector()
    psd = _prior_sd(prior)
    R_m = cap.R_mocap
    gauge = bool(free[_SLICES['plat']].any())
    G = _gauge_jacobian(prior)

    def unpack(x):
        th = theta0.copy()
        th[free] = x
        return th

    def resid(x):
        th = unpack(x)
        e, _ = _meas_residual_and_jacobian(cap, R_m, KinGeom.from_vector(th))
        parts = [(e / sigma_mm).ravel(), (th[free] - ptheta[free]) / psd[free]]
        if gauge:
            parts.append(GAUGE_WEIGHT * _gauge_rows(th, prior))
        return np.concatenate(parts)

    def jac(x):
        th = unpack(x)
        _, Jm = _meas_residual_and_jacobian(cap, R_m, KinGeom.from_vector(th))
        parts = [Jm[:, free] / sigma_mm, np.diag(1.0 / psd[free])]
        if gauge:
            parts.append(GAUGE_WEIGHT * G[:, free])
        return np.vstack(parts)

    if free.any():
        sol = least_squares(resid, theta0[free], jac=jac, x_scale='jac',
                            method='trf', xtol=1e-14, ftol=1e-14, gtol=1e-12,
                            max_nfev=500)
        x, success, message = sol.x, bool(sol.success), str(sol.message)
    else:
        x, success, message = theta0[free], True, 'no free parameters'
    th = unpack(x)
    g = KinGeom.from_vector(th)
    e, _ = _meas_residual_and_jacobian(cap, R_m, g)
    dof = max(e.size - int(free.sum()) + (6 if gauge else 0), 1)
    chi2 = float(np.sum((e / sigma_mm) ** 2)) / dof
    sd = np.full(N_PARAMS, np.nan)
    if free.any():
        # Posterior covariance from the SVD of the column-scaled Jacobian:
        # the k columns (~1e4) and the gauge rows (x1e3) make J^T J too
        # ill-conditioned to invert directly.
        Jf = jac(x)
        scale = np.linalg.norm(Jf, axis=0)
        scale[scale == 0.0] = 1.0
        _, s, Vt = np.linalg.svd(Jf / scale, full_matrices=False)
        keep = s > s[0] * 1e-12
        var = np.full(len(s), np.inf)
        var[keep] = 1.0 / s[keep] ** 2
        cov_diag = np.einsum('ki,k->i', Vt ** 2, var) / scale ** 2
        sd[free] = np.sqrt(cov_diag * max(chi2, 1e-12))
    return g, sd, e, chi2, success, message


def fit(cap: Capture, *, start: Optional[KinGeom] = None,
        prior: Optional[KinGeom] = None, groups: Sequence[str] = GROUPS,
        sigma_mm: float = DEFAULT_SIGMA_MM, freeze: bool = True) -> FitResult:
    """Fit the geometry to the ``role == fit`` rows that are not ``post_home``.

    ``prior`` is the CAD centre the weak prior pulls toward and the value a
    frozen parameter takes; ``start`` is where the solve starts and what a
    parameter outside ``groups`` is held at (they differ only in
    ``--offsets-only``, where ``start`` is a previously fitted geometry).
    """
    prior = prior or nominal_geometry()
    start = start or prior
    rows = cap.subset((cap.role == 'fit') & (cap.phase != 'post_home'))
    if len(rows) < 3:
        raise ValueError('need at least 3 fit rows, have %d' % len(rows))
    free = free_mask(groups)
    g, sd, e, chi2, ok, msg = _solve(rows, start, prior, free, sigma_mm)

    frozen: List[Tuple[str, float]] = []
    if freeze:
        over = free & (sd > _freeze_sd(prior))
        if over.any():
            names = param_names()
            frozen = [(names[i], float(sd[i])) for i in np.flatnonzero(over)]
            free = free & ~over
            held = start.to_vector()
            held[over] = prior.to_vector()[over]
            g, sd, e, chi2, ok, msg = _solve(rows, KinGeom.from_vector(held),
                                             prior, free, sigma_mm)
    return FitResult(geom=g, free=free, frozen=frozen, sd=sd,
                     rms_mm=float(np.sqrt(np.mean(e ** 2))),
                     max_mm=float(np.max(np.abs(e))), reduced_chi2=chi2,
                     n_rows=len(rows), success=ok, message=msg)


# ═════════════════════════════════════════════════════════════════════════════
# Forward kinematics and pose-space evaluation
# ═════════════════════════════════════════════════════════════════════════════

def forward(rev: np.ndarray, g: KinGeom, c0: np.ndarray, R0: np.ndarray,
            iters: int = 30) -> Tuple[np.ndarray, np.ndarray]:
    """The joint-pattern pose whose leg lengths match ``rev`` under ``g``.

    Newton on (c, rotation increment) with the Stewart Jacobian
    ``dl/dc = u``, ``dl/dw = (R p) x u``, started at the mocap pose (a
    fraction of a mm away), so it converges in a handful of steps.
    """
    target = rev_to_length(np.atleast_2d(rev), g)[0]
    c = np.array(c0, dtype=float)
    R = np.array(R0, dtype=float)
    for _ in range(iters):
        Rp = (R @ g.plat.T).T
        V = c[None, :] + Rp - g.base
        ell = np.linalg.norm(V, axis=1)
        r = ell - target
        if np.max(np.abs(r)) < 1e-10:
            break
        U = V / ell[:, None]
        J = np.hstack([U, np.cross(Rp, U)])
        step = np.linalg.solve(J, -r)
        c = c + step[:3]
        R = _exp(step[3:]) @ R
    return c, R


def pose_errors(cap: Capture, g: KinGeom) -> Tuple[np.ndarray, np.ndarray]:
    """Per row: |FK(encoders) - mocap| in mm, and the attitude error in deg.

    The mocap attitude is registered with ``g.reg`` — for the nominal
    geometry that is zero, so its attitude error includes the registration.
    """
    R_true = cap.R_mocap @ _exp(g.reg)
    dp = np.empty(len(cap))
    da = np.empty(len(cap))
    for i in range(len(cap)):
        c, R = forward(cap.rev[i], g, cap.pos[i], R_true[i])
        dp[i] = np.linalg.norm(c - cap.pos[i])
        da[i] = math.degrees(np.linalg.norm(
            Rotation.from_matrix(R.T @ R_true[i]).as_rotvec()))
    return dp, da


def stow_pose(g: KinGeom, init_height_mm: float) -> Tuple[np.ndarray, np.ndarray]:
    """FK of all-zero revolutions: where STOW really is under ``g``."""
    c, R = forward(np.zeros(N_LEGS), g, np.array([0.0, 0.0, init_height_mm]),
                   np.eye(3))
    return c, Rotation.from_matrix(R).as_rotvec()


# ═════════════════════════════════════════════════════════════════════════════
# Verdicts (§ 3, § 4)
# ═════════════════════════════════════════════════════════════════════════════

def _max_pairwise(p: np.ndarray) -> float:
    if len(p) < 2:
        return 0.0
    d = p[:, None, :] - p[None, :, :]
    return float(np.max(np.linalg.norm(d, axis=2)))


_RANK = {'STATIC': 0, 'DIRECTIONAL': 1, 'RANDOM': 2}


def repeat_verdicts(cap: Capture, threshold_mm: float = REPEAT_SPREAD_MM
                    ) -> Tuple[Optional[str], List[dict]]:
    """§ 3: spread of the mocap arrival at each two-direction repeat pose."""
    groups = sorted(set(g for g in cap.repeat_group if g))
    out = []
    for grp in groups:
        m = cap.repeat_group == grp
        pos = cap.pos[m]
        dirs = cap.direction[m]
        by_dir = {d: pos[dirs == d] for d in sorted(set(dirs))}
        overall = _max_pairwise(pos)
        within = max((_max_pairwise(p) for p in by_dir.values()), default=0.0)
        means = np.array([p.mean(axis=0) for p in by_dir.values()])
        between = _max_pairwise(means)
        if overall <= threshold_mm:
            v = 'STATIC'
        elif within <= threshold_mm:
            v = 'DIRECTIONAL'
        else:
            v = 'RANDOM'
        out.append({'group': grp, 'n': int(m.sum()),
                    'directions': {d: int(len(p)) for d, p in by_dir.items()},
                    'overall_mm': overall, 'within_mm': within,
                    'between_mm': between, 'verdict': v})
    worst = max((r['verdict'] for r in out), key=_RANK.get, default=None)
    return worst, out


def rehome_verdict(cap: Capture, fitted: FitResult,
                   sigma_mm: float = DEFAULT_SIGMA_MM,
                   threshold_mm: float = REHOME_MM) -> Optional[dict]:
    """§ 4: does a re-home move the platform, and by how much per leg?"""
    pre = cap.phase == 'pre_home'
    post = cap.phase == 'post_home'
    if not post.any():
        return None
    ids = sorted(set(cap.pose_id[pre]) & set(cap.pose_id[post]))
    per_pose = []
    for pid in ids:
        a = cap.pos[pre & (cap.pose_id == pid)].mean(axis=0)
        b = cap.pos[post & (cap.pose_id == pid)].mean(axis=0)
        per_pose.append({'pose_id': str(pid),
                         'shift_mm': float(np.linalg.norm(b - a))})
    post_cap = cap.subset(post)
    post_cap.role = np.array(['fit'] * len(post_cap), dtype=object)
    post_cap.phase = np.array([''] * len(post_cap), dtype=object)
    refit = fit(post_cap, start=fitted.geom, prior=fitted.geom, groups=('L0',),
                sigma_mm=sigma_mm, freeze=False)
    dL0 = refit.geom.L0 - fitted.geom.L0
    max_shift = max((p['shift_mm'] for p in per_pose), default=0.0)
    ok = bool(max_shift <= threshold_mm and np.max(np.abs(dL0)) <= threshold_mm)
    return {'poses': per_pose, 'max_shift_mm': max_shift,
            'dL0_mm': dL0.tolist(), 'max_abs_dL0_mm': float(np.max(np.abs(dL0))),
            'verdict': 'PASS' if ok else 'FAIL'}


# ═════════════════════════════════════════════════════════════════════════════
# Report
# ═════════════════════════════════════════════════════════════════════════════

def _stats(dp: np.ndarray, da: np.ndarray) -> dict:
    if len(dp) == 0:
        return {'n': 0}
    return {'n': int(len(dp)), 'pos_rms_mm': float(np.sqrt(np.mean(dp ** 2))),
            'pos_max_mm': float(np.max(dp)), 'att_max_deg': float(np.max(da)),
            'att_rms_deg': float(np.sqrt(np.mean(da ** 2)))}


def analyse(cap: Capture, *, sigma_mm: float = DEFAULT_SIGMA_MM,
            freeze: bool = True, offsets_only_from: Optional[KinGeom] = None
            ) -> dict:
    """Everything the report says, as one dict (also ``result.json``)."""
    nominal = nominal_geometry()
    h0 = nominal_init_height_mm()
    if offsets_only_from is not None:
        res = fit(cap, start=offsets_only_from, prior=offsets_only_from,
                  groups=('L0',), sigma_mm=sigma_mm, freeze=False)
        reference = offsets_only_from
    else:
        res = fit(cap, prior=nominal, sigma_mm=sigma_mm, freeze=freeze)
        reference = nominal

    main = (cap.phase != 'post_home')
    hold = cap.subset(main & (cap.role == 'holdout'))
    fitr = cap.subset(main & (cap.role == 'fit'))
    fitted_hold = _stats(*pose_errors(hold, res.geom)) if len(hold) else {'n': 0}
    fitted_in = _stats(*pose_errors(fitr, res.geom))
    base_hold = (_stats(*pose_errors(hold, reference)) if len(hold)
                 else {'n': 0})

    node_dev = np.concatenate([
        np.linalg.norm(res.geom.base - nominal.base, axis=1),
        np.linalg.norm(res.geom.plat - nominal.plat, axis=1)])
    stow_c, stow_rv = stow_pose(res.geom, h0)
    repeat_worst, repeats = repeat_verdicts(cap)
    rehome = (rehome_verdict(cap, res, sigma_mm)
              if offsets_only_from is None else None)

    criteria = {}
    if fitted_hold.get('n'):
        criteria['holdout_pos_rms'] = fitted_hold['pos_rms_mm'] <= PASS_RMS_MM
        criteria['holdout_pos_max'] = fitted_hold['pos_max_mm'] <= PASS_MAX_MM
        criteria['holdout_att_max'] = fitted_hold['att_max_deg'] <= PASS_ATT_DEG
    if offsets_only_from is None:
        criteria['nodes_plausible'] = bool(np.max(node_dev) <= PLAUSIBLE_NODE_MM)

    names = param_names()
    return {
        'mode': 'offsets-only' if offsets_only_from is not None else 'full',
        'generated': datetime.now().isoformat(timespec='seconds'),
        'rows': {'total': len(cap), 'fit': res.n_rows, 'holdout': len(hold),
                 'post_home': int(np.sum(cap.phase == 'post_home'))},
        'solver': {'success': res.success, 'message': res.message,
                   'reduced_chi2': res.reduced_chi2, 'sigma_mm': sigma_mm,
                   'leg_rms_mm': res.rms_mm, 'leg_max_mm': res.max_mm},
        'frozen': [{'param': n, 'sd': s} for n, s in res.frozen],
        'params': [{'param': names[i], 'value': float(res.geom.to_vector()[i]),
                    'reference': float(reference.to_vector()[i]),
                    'sd': (None if math.isnan(res.sd[i]) else float(res.sd[i])),
                    'fitted': bool(res.free[i])} for i in range(N_PARAMS)],
        'node_deviation_mm': node_dev.tolist(),
        'pose_fit_rows': fitted_in, 'pose_holdout': fitted_hold,
        'pose_holdout_reference': base_hold,
        'stow': {'centre_mm': stow_c.tolist(), 'rotvec_rad': stow_rv.tolist(),
                 'config_initial_height_mm': h0},
        'repeat': {'verdict': repeat_worst, 'groups': repeats},
        'rehome': rehome,
        'criteria': criteria,
        'pass': bool(criteria) and all(criteria.values()),
        'geometry': res.geom.to_dict(),
    }


def render_report(a: dict, capture_path: str) -> str:
    L = ['# Kinematic calibration fit — %s' % os.path.basename(capture_path), '',
         'Mode **%s**, generated %s by `tools/kincal_fit.py`. Plan: '
         '`plans/active/kinematic-calibration.md`.' % (a['mode'], a['generated']),
         '', '**Overall: %s**' % ('PASS' if a['pass'] else 'NOT PASSED'), '']
    L += ['## Criteria (§ 8)', '', '| criterion | result |', '|---|---|']
    for k, v in a['criteria'].items():
        L.append('| %s | %s |' % (k, 'pass' if v else '**FAIL**'))
    s = a['solver']
    L += ['', '## Solve', '',
          '- rows: %(total)d total, %(fit)d fitted, %(holdout)d hold-out, '
          '%(post_home)d post-home' % a['rows'],
          '- leg-space residual: RMS %.3f mm, max %.3f mm; reduced chi² %.2f '
          'at sigma %.2f mm; %s' % (s['leg_rms_mm'], s['leg_max_mm'],
                                   s['reduced_chi2'], s['sigma_mm'],
                                   s['message'])]
    if a['frozen']:
        L += ['- **frozen at prior** (posterior sd over the § 2 threshold): '
              + ', '.join('`%s` (sd %.3g)' % (f['param'], f['sd'])
                          for f in a['frozen'])]
    else:
        L += ['- nothing frozen']

    def pose_line(label, st):
        if not st.get('n'):
            return '| %s | — | — | — | — |' % label
        return '| %s | %d | %.3f | %.3f | %.4f |' % (
            label, st['n'], st['pos_rms_mm'], st['pos_max_mm'],
            st['att_max_deg'])
    L += ['', '## Pose-space error (FK of the encoders vs mocap)', '',
          '| set | n | pos RMS mm | pos max mm | att max deg |', '|---|---|---|---|---|',
          pose_line('hold-out, fitted geometry', a['pose_holdout']),
          pose_line('hold-out, reference geometry', a['pose_holdout_reference']),
          pose_line('fit rows, fitted geometry', a['pose_fit_rows'])]
    st = a['stow']
    L += ['', '## STOW under the fitted geometry', '',
          'FK of all-zero revolutions: centre (%.2f, %.2f, %.2f) mm, attitude '
          'rotvec (%.5f, %.5f, %.5f) rad. Config `initial_height_mm` = %.1f.'
          % (tuple(st['centre_mm']) + tuple(st['rotvec_rad'])
             + (st['config_initial_height_mm'],))]
    rp = a['repeat']
    L += ['', '## Path dependence (§ 3): %s' % (rp['verdict'] or 'no repeat groups'), '']
    if rp['groups']:
        L += ['| group | n | overall mm | within mm | between mm | verdict |',
              '|---|---|---|---|---|---|']
        for g in rp['groups']:
            L.append('| %s | %d | %.3f | %.3f | %.3f | %s |' % (
                g['group'], g['n'], g['overall_mm'], g['within_mm'],
                g['between_mm'], g['verdict']))
    rh = a['rehome']
    if rh:
        L += ['', '## Re-home (§ 4): %s' % rh['verdict'], '',
              '- max arrival shift across the re-home: %.3f mm' % rh['max_shift_mm'],
              '- per-leg ΔL0 (post − pre), mm: '
              + ', '.join('%+.3f' % v for v in rh['dL0_mm'])]
    L += ['', '## Parameters', '',
          '| param | value | reference | Δ | sd | fitted |', '|---|---|---|---|---|---|']
    for p in a['params']:
        L.append('| %s | %.6g | %.6g | %+.4g | %s | %s |' % (
            p['param'], p['value'], p['reference'], p['value'] - p['reference'],
            '—' if p['sd'] is None else '%.3g' % p['sd'],
            'yes' if p['fitted'] else 'held'))
    L += ['', 'Node deviation from CAD, max: %.3f mm (plausibility bound %.1f).'
          % (max(a['node_deviation_mm']), PLAUSIBLE_NODE_MM), '']
    return '\n'.join(L)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('capture', help='capture CSV (format in the module docstring)')
    ap.add_argument('--out-dir', default=None)
    ap.add_argument('--sigma-mm', type=float, default=DEFAULT_SIGMA_MM)
    ap.add_argument('--no-freeze', action='store_true',
                    help='report posterior sd but never freeze a parameter')
    ap.add_argument('--offsets-only', action='store_true',
                    help='§ 4 per-session re-fit: only the six L0 are free')
    ap.add_argument('--geometry', default=None,
                    help='proposed_geometry.yaml to hold fixed (--offsets-only)')
    args = ap.parse_args(argv)

    if args.offsets_only and not args.geometry:
        ap.error('--offsets-only needs --geometry')
    held = None
    if args.geometry:
        with open(args.geometry, encoding='utf-8') as fh:
            held = KinGeom.from_dict(yaml.safe_load(fh)['geometry'])
    cap = read_capture(args.capture)
    a = analyse(cap, sigma_mm=args.sigma_mm, freeze=not args.no_freeze,
                offsets_only_from=held if args.offsets_only else None)

    stem = os.path.splitext(os.path.basename(args.capture))[0]
    out = args.out_dir or os.path.join(_REPO_ROOT, 'temp', 'reports', 'kincal', stem)
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, 'report.md'), 'w', encoding='utf-8') as fh:
        fh.write(render_report(a, args.capture))
    with open(os.path.join(out, 'result.json'), 'w', encoding='utf-8') as fh:
        json.dump(a, fh, indent=1)
    with open(os.path.join(out, 'proposed_geometry.yaml'), 'w',
              encoding='utf-8') as fh:
        fh.write('# PROPOSED by tools/kincal_fit.py from %s — NOT applied.\n'
                 '# Applying it is a deliberate commit '
                 '(plans/active/kinematic-calibration.md § 6 step 5).\n'
                 % os.path.basename(args.capture))
        yaml.safe_dump({'capture': os.path.basename(args.capture),
                        'mode': a['mode'], 'pass': a['pass'],
                        'stow_centre_mm': a['stow']['centre_mm'],
                        'geometry': a['geometry']}, fh, sort_keys=False)
    print('%s — %s; report: %s' % (a['mode'], 'PASS' if a['pass'] else
                                   'NOT PASSED', os.path.join(out, 'report.md')))
    return 0


if __name__ == '__main__':
    sys.exit(main())
