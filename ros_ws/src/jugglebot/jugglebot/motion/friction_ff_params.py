"""Friction-FF parameter loader.

Reads the ``friction_ff`` block of ``config/hardware_config.yaml`` directly
at runtime — NOT via the codegen'd ``hardware_config`` module — so that
on-platform tuning of the per-leg friction parameters does not require a
``generate_config.py`` rebuild + ROS2 redeploy cycle.

The block is intentionally kept out of ``generate_config.py``'s
``HW_SECTIONS`` so it is not emitted into Python/C++/JS consumers that
don't need it.  See the comment block above ``friction_ff:`` in
``config/hardware_config.yaml`` and §3.1 of
``plans/archived/2026-05-08 friction-ff-motor-guard-integration.md``.

Lazy-loaded by ``MotorGuard.__init__`` at construction time (not at module
import) so test environments without a reachable YAML do not fail when the
``motor_guard`` module is imported in isolation.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np
import yaml


# Walk up from ros_ws/src/jugglebot/jugglebot/motion/friction_ff_params.py
# to the repo root: motion → jugglebot → jugglebot → src → ros_ws → repo.
_THIS_FILE = os.path.abspath(__file__)
_REPO_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(_THIS_FILE), '..', '..', '..', '..', '..'))
_HW_YAML = os.path.join(_REPO_ROOT, 'config', 'hardware_config.yaml')

_N_LEGS = 6


@dataclass(frozen=True)
class FrictionFFParams:
    """Per-leg Stribeck friction-FF parameters loaded from YAML.

    The per-leg arrays are length-6 ``float64`` ndarrays, one entry per
    motor index.  ``ff_sign`` is also stored as a float ndarray (rather
    than int) so the multiplication in
    ``MotorGuard._compute_friction_ff_Nm`` stays in float dtype without
    an upcast.

    Schema notes:
        - ``v_gate_rps`` is a global scalar (the same gate scale is used
          for every leg).  It tunes the smooth low-velocity gate that
          replaces the bench-era boost band — see
          logbook/2026-05-08-friction-ff-platform-limit-cycle.md.
        - The bench's ``stiction_boost_threshold_rps`` was removed in
          PR 2.1 because the boost band's hard 0 → τ_s step at v=1e-4
          bootstrapped a 5 Hz limit cycle on platform.  The smooth gate
          is the structural replacement.
    """

    enabled: bool
    v_gate_rps: float              # smooth low-v gate scale [rev/s]
    ff_sign: np.ndarray            # (6,) ±1.0 per leg
    tau_c_A: np.ndarray            # (6,) kinetic Coulomb floor [A]
    tau_s_A: np.ndarray            # (6,) stiction peak [A]
    omega_s_rps: np.ndarray        # (6,) Stribeck breakaway scale [rev/s]
    b_A_per_rps: np.ndarray        # (6,) viscous slope [A/(rev/s)]
    load_offset_A: np.ndarray      # (6,) constant load [A]
    motor_kt_nm_per_a: float       # iq → torque conversion [Nm/A]


def _arr(name: str, raw, n: int = _N_LEGS) -> np.ndarray:
    a = np.asarray(raw, dtype=float)
    if a.shape != (n,):
        raise ValueError(
            f"friction_ff.{name} must be a length-{n} array, got shape {a.shape}")
    if not np.all(np.isfinite(a)):
        raise ValueError(f"friction_ff.{name} contains non-finite values: {a}")
    return a


def load_params(yaml_path: str | None = None) -> FrictionFFParams:
    """Load friction-FF params from ``config/hardware_config.yaml``.

    Parameters
    ----------
    yaml_path : str, optional
        Override the default path (mainly for tests).  Defaults to the
        repo's ``config/hardware_config.yaml``.
    """
    path = yaml_path or _HW_YAML
    with open(path, 'r') as f:
        cfg = yaml.safe_load(f)

    if 'friction_ff' not in cfg:
        raise KeyError(
            f"{path} is missing the top-level 'friction_ff' block; "
            f"see plans/archived/2026-05-08 friction-ff-motor-guard-integration.md §3.1")
    ff = cfg['friction_ff']

    dyn = cfg.get('dynamics', {})
    kt = dyn.get('motor_kt_nm_per_a')
    if kt is None:
        raise KeyError(
            f"{path} is missing dynamics.motor_kt_nm_per_a; "
            f"required for the iq → torque conversion in the friction FF")

    v_gate = float(ff['v_gate_rps'])
    if v_gate <= 0.0:
        raise ValueError(
            f"friction_ff.v_gate_rps must be > 0 (smooth-gate scale; see "
            f"logbook/2026-05-08-friction-ff-platform-limit-cycle.md), "
            f"got {v_gate}")

    return FrictionFFParams(
        enabled=bool(ff['enabled']),
        v_gate_rps=v_gate,
        ff_sign=_arr('ff_sign', ff['ff_sign']),
        tau_c_A=_arr('tau_c_A', ff['tau_c_A']),
        tau_s_A=_arr('tau_s_A', ff['tau_s_A']),
        omega_s_rps=_arr('omega_s_rps', ff['omega_s_rps']),
        b_A_per_rps=_arr('b_A_per_rps', ff['b_A_per_rps']),
        load_offset_A=_arr('load_offset_A', ff['load_offset_A']),
        motor_kt_nm_per_a=float(kt),
    )
