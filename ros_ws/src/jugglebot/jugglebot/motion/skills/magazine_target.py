"""Magazine-throw target: config loader and base-frame <-> world <-> plan frame maps.

Pure Python (no ROS). The base ("elbow") frame is ``bb_base_frame.canonical_base_frame``:
origin = marker C, columns of ``R_base_to_world`` = base x, y, z axes in world, so
``p_world = origin_world_mm + R @ p_base``. The plan frame is world xy/z minus the
skill_node frame-check offset: ``p_plan = p_world - [off_x, off_y, 0]``.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Sequence

import numpy as np

FLIGHT_S_MIN = 0.5
FLIGHT_S_MAX = 1.3
_CONFIG_NAME = 'bb_magazine.yaml'


@dataclass(frozen=True)
class MagazineConfig:
    target_base_mm: np.ndarray
    flight_s: float
    source_path: str


def default_config_path() -> str:
    from jugglebot.repo_root import resolve_repo_root
    return os.path.join(resolve_repo_root(), 'config', _CONFIG_NAME)


def _bad(msg: str) -> ValueError:
    return ValueError(f'MAGAZINE_CONFIG: {msg}')


def load_magazine_config(path: str) -> MagazineConfig:
    """Strict loader: missing file/key, wrong shape, non-finite values or a
    flight_s outside [0.5, 1.3] s raise ``ValueError('MAGAZINE_CONFIG: ...')``."""
    import yaml
    try:
        with open(path, 'r') as f:
            raw = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as e:
        raise _bad(f'cannot read {path}: {e}')
    if not isinstance(raw, dict):
        raise _bad(f'{path} is not a mapping')
    for key in ('target_base_mm', 'flight_s'):
        if key not in raw:
            raise _bad(f'{path} missing {key}')
    return MagazineConfig(target_base_mm=_valid_target(raw['target_base_mm']),
                          flight_s=_valid_flight(raw['flight_s']),
                          source_path=str(path))


def _valid_target(t) -> np.ndarray:
    if not isinstance(t, (list, tuple, np.ndarray)) or len(t) != 3 or \
            any(isinstance(v, bool) or not isinstance(v, (int, float, np.floating, np.integer))
                for v in t):
        raise _bad('target_base_mm must be a list of 3 numbers')
    target = np.array([float(v) for v in t])
    if not np.all(np.isfinite(target)):
        raise _bad(f'target_base_mm not finite: {t}')
    return target


def _valid_flight(fs) -> float:
    if isinstance(fs, bool) or not isinstance(fs, (int, float, np.floating, np.integer)) \
            or not math.isfinite(float(fs)):
        raise _bad(f'flight_s must be a finite number, got {fs!r}')
    fs = float(fs)
    if not (FLIGHT_S_MIN <= fs <= FLIGHT_S_MAX):
        raise _bad(f'flight_s {fs} outside [{FLIGHT_S_MIN}, {FLIGHT_S_MAX}] s')
    return fs


def with_overrides(cfg: MagazineConfig, target_base_mm=None,
                   flight_s=None) -> MagazineConfig:
    """``cfg`` with the session's on-the-fly overrides applied (owner ask,
    2026-10-13: nudge the target from the console between throws without
    editing the file) — skill_node's ``magazine_target_base_mm`` /
    ``magazine_flight_s`` parameters.  ``None``, an all-zero triple and a
    ``0.0`` flight are the parameters' "unset" sentinels (the same "0 => the
    node's default" spelling every Juggle goal field uses) and keep the
    file's value; anything else REPLACES it, validated exactly like the
    file (``ValueError('MAGAZINE_CONFIG: ...')``).  The file stays the
    persistent record: the final numbers belong back in it."""
    target = cfg.target_base_mm
    fs = cfg.flight_s
    if target_base_mm is not None:
        t = list(target_base_mm)
        unset = len(t) == 0 or (len(t) == 3 and all(
            isinstance(v, (int, float, np.floating, np.integer))
            and not isinstance(v, bool) and float(v) == 0.0 for v in t))
        if not unset:
            target = _valid_target(t)
    if flight_s is not None and not (
            isinstance(flight_s, (int, float, np.floating, np.integer))
            and not isinstance(flight_s, bool) and float(flight_s) == 0.0):
        fs = _valid_flight(flight_s)
    return MagazineConfig(target_base_mm=target, flight_s=fs,
                          source_path=cfg.source_path)


def _v3(p) -> np.ndarray:
    a = np.asarray(p, dtype=float).reshape(3)
    return a


def base_to_world_mm(p_base, R_base_to_world, origin_world_mm) -> np.ndarray:
    return _v3(origin_world_mm) + np.asarray(R_base_to_world, dtype=float) @ _v3(p_base)


def world_to_base_mm(p_world, R_base_to_world, origin_world_mm) -> np.ndarray:
    return np.asarray(R_base_to_world, dtype=float).T @ (_v3(p_world) - _v3(origin_world_mm))


def _off3(frame_offset_xy_mm: Sequence[float]) -> np.ndarray:
    return np.array([float(frame_offset_xy_mm[0]), float(frame_offset_xy_mm[1]), 0.0])


def base_to_plan_mm(p_base, R_base_to_world, origin_world_mm,
                    frame_offset_xy_mm: Sequence[float] = (0.0, 0.0)) -> np.ndarray:
    return base_to_world_mm(p_base, R_base_to_world, origin_world_mm) - _off3(frame_offset_xy_mm)


def plan_to_base_mm(p_plan, R_base_to_world, origin_world_mm,
                    frame_offset_xy_mm: Sequence[float] = (0.0, 0.0)) -> np.ndarray:
    return world_to_base_mm(_v3(p_plan) + _off3(frame_offset_xy_mm),
                            R_base_to_world, origin_world_mm)


# ── the miss report's pure fit (U2c) ────────────────────────────────────

#: Fewest samples a ballistic fit is trusted on (z quadratic needs 3; the
#: margin is for noise).
MIN_FIT_SAMPLES = 6


@dataclass(frozen=True)
class CrossingFit:
    """Where a fitted ballistic crosses a descending height."""

    t_cross_s: float           #: same clock as the input times
    xy_mm: np.ndarray          #: (x, y) at the crossing, input frame
    n_samples: int
    rms_z_mm: float            #: residual of the z quadratic


def fit_descending_crossing(t_s, pos_mm, target_z_mm: float, *,
                            t_from_s: float = -math.inf,
                            min_samples: int = MIN_FIT_SAMPLES):
    """Fit z(t) as a quadratic and x(t), y(t) as lines to the samples at or
    after ``t_from_s``, and return the :class:`CrossingFit` where the fit
    crosses ``target_z_mm`` DESCENDING; raises ``ValueError`` (message names
    why) when there are too few samples or the fit never reaches the height.
    Pure; times are shifted internally so large epoch values lose no precision."""
    t = np.asarray(t_s, dtype=float).reshape(-1)
    p = np.asarray(pos_mm, dtype=float).reshape(-1, 3)
    if t.shape[0] != p.shape[0]:
        raise ValueError('t_s and pos_mm differ in length')
    keep = np.isfinite(t) & np.all(np.isfinite(p), axis=1) & (t >= t_from_s)
    t, p = t[keep], p[keep]
    n = int(t.shape[0])
    if n < min_samples:
        raise ValueError('only %d usable samples (need %d)' % (n, min_samples))
    if float(t[-1] - t[0]) <= 0.0:
        raise ValueError('samples span no time')
    t0 = float(t[0])
    tt = t - t0
    cz = np.polyfit(tt, p[:, 2], 2)
    if not cz[0] < 0.0:
        raise ValueError('z fit is not concave (a=%.3g) -- not a ballistic' % cz[0])
    roots = np.roots([cz[0], cz[1], cz[2] - float(target_z_mm)])
    real = [float(r.real) for r in roots if abs(r.imag) < 1e-9]
    if not real:
        raise ValueError('the fitted flight never reaches z=%.0f mm' % target_z_mm)
    tc = max(real)          # concave: the larger root is the descending one
    cx = np.polyfit(tt, p[:, 0], 1)
    cy = np.polyfit(tt, p[:, 1], 1)
    rms = float(np.sqrt(np.mean((np.polyval(cz, tt) - p[:, 2]) ** 2)))
    return CrossingFit(t_cross_s=tc + t0,
                       xy_mm=np.array([np.polyval(cx, tc), np.polyval(cy, tc)]),
                       n_samples=n, rms_z_mm=rms)


def quat_to_rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    """3x3 rotation matrix of a quaternion (normalised here); the inverse of
    mocap_node's ``_rotation_to_quat`` for ``R_base_to_world``."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if not n > 0.0:
        raise ValueError('zero quaternion')
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
