"""Friction-FF parameter loader.

Reads the ``friction_ff`` block of ``config/hardware_config.yaml`` directly
at runtime — NOT via the codegen'd ``hardware_config`` module.

The block is intentionally kept out of ``generate_config.py``'s
``HW_SECTIONS`` so it is not emitted into Python/C++/JS consumers that
don't need it.  See the comment block above ``friction_ff:`` in
``config/hardware_config.yaml`` and §3.1 of
``plans/archived/2026-05-08 friction-ff-motor-guard-integration.md``.

Path resolution (``_resolve_yaml_path``) covers both the source tree and
the colcon install tree, because ``motor_guard`` runs from the install
tree in production (launched via the ``motor_guard`` console-script entry
point under ``install/jugglebot/lib/...``).  Resolution order, first
existing wins:

  1. ``$JUGGLEBOT_HW_CONFIG`` — explicit override.
  2. The ament *share* dir (``<install>/share/jugglebot/config/
     hardware_config.yaml``, installed via ``setup.py`` data_files).  This
     is the canonical production path.
  3. The source-tree YAML (for pytest / running from a checkout).

CONTRACT CHANGE (2026-06-24): the loader previously did a single
rigid five-level ``__file__`` walk that resolved correctly only from the
source tree; under the install tree it pointed at the never-installed
``install/jugglebot/config/`` and raised ``FileNotFoundError`` (the
motor_guard startup crash).  The fix installs the YAML into the ament
share dir and resolves via that.  Because ``ament_python`` *copies*
data_files at ``colcon build`` time (it does not symlink them), a friction
re-tune of the source YAML now takes effect only after a ``colcon build
--packages-select jugglebot`` — OR immediately via ``$JUGGLEBOT_HW_CONFIG``
pointed at the source file, which is the supported in-place-tune path.

Lazy-loaded by ``MotorGuard.__init__`` at construction time (not at module
import) so test environments without a reachable YAML do not fail when the
``motor_guard`` module is imported in isolation.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

import numpy as np
import yaml


# Source-tree YAML: walk up from
# ros_ws/src/jugglebot/jugglebot/motion/friction_ff_params.py to the repo
# root (motion → jugglebot → jugglebot → src → ros_ws → repo).  Used as
# the resolution fallback; correct only when run from the source checkout.
_THIS_FILE = os.path.abspath(__file__)
_SRC_REPO_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(_THIS_FILE), '..', '..', '..', '..', '..'))
_SRC_HW_YAML = os.path.join(_SRC_REPO_ROOT, 'config', 'hardware_config.yaml')

# Explicit-override env var (escape hatch for in-place tuning without a
# colcon rebuild, and for any detached deployment).
_HW_CONFIG_ENV = 'JUGGLEBOT_HW_CONFIG'

_N_LEGS = 6


def _resolve_yaml_path() -> str:
    """Locate ``hardware_config.yaml`` across the source and install trees.

    See the module docstring for the resolution order.  Raises
    ``FileNotFoundError`` listing every candidate tried if none exist
    (preserves the loud fail-fast property of the original loader).
    """
    tried = []

    env_path = os.environ.get(_HW_CONFIG_ENV)
    if env_path:
        if os.path.exists(env_path):
            return env_path
        tried.append(f'${_HW_CONFIG_ENV}={env_path}')

    # ament share dir — present only under a built+sourced colcon overlay.
    try:
        from ament_index_python.packages import get_package_share_directory
        share_path = os.path.join(
            get_package_share_directory('jugglebot'),
            'config', 'hardware_config.yaml')
        if os.path.exists(share_path):
            return share_path
        tried.append(share_path)
    except Exception:
        # ament_index not importable (e.g. the pytest venv without ROS2
        # sourced) or the package is not on the ament index — fall through
        # to the source tree.
        pass

    if os.path.exists(_SRC_HW_YAML):
        return _SRC_HW_YAML
    tried.append(_SRC_HW_YAML)

    raise FileNotFoundError(
        'Could not locate hardware_config.yaml. Tried (in order): '
        + '; '.join(tried)
        + f'. Set ${_HW_CONFIG_ENV} to override, or run '
          '`colcon build --packages-select jugglebot` to install it into '
          'the ament share dir.')


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
        Override the default path (mainly for tests).  When ``None``, the
        path is resolved via :func:`_resolve_yaml_path` (env override →
        ament share dir → source tree).
    """
    path = yaml_path or _resolve_yaml_path()
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


def friction_ff_torque_nm(params: FrictionFFParams, axis: int, v_rps: float) -> float:
    """Scalar single-axis Stribeck friction-FF torque [Nm].

    Computes the SAME quantity as ``MotorGuard._compute_friction_ff_Nm()[axis]``
    for a commanded velocity ``v_rps`` on ``axis`` (all other axes at rest), but
    as a plain scalar so it can be dependency-injected into the bench replay
    source (``controller.teensy_link.replay_setpoint.RecordedThrowSource``'s
    ``torque_ff_fn``) WITHOUT pulling the full ``motor_guard`` / ROS 2 stack into
    ``controller/`` (which must stay pure). The bench driver builds a
    ``lambda v: friction_ff_torque_nm(params, axis, v)`` and injects it.

    Formula (mirrors ``motor_guard.py:_compute_friction_ff_Nm`` exactly)::

        stribeck = τ_c + (τ_s − τ_c)·exp(−(|v|/ω_s)²) + b·|v|
        gate     = 1 − exp(−(|v|/v_gate)²)
        iq_total = −sign(v)·(stribeck·gate) + load_offset
        torque   = iq_total · ff_sign · Kt

    ``np.sign(0) == 0`` is reproduced via ``math.copysign`` guarded on ``av == 0``
    so the v=0 case yields ``load_offset·ff_sign·Kt`` (only the constant load),
    matching the production path's dead-zone-free behaviour.

    **Sign convention:** returns motor_guard-convention ``torque_ff`` (YAML
    ``ff_sign``, +1 platform default). The can-bridge applies ``leg_sign`` to
    ``torque_ff`` on the way to the ODrive (``odrive_protocol.h``
    ``encode_set_pos_with_ff``, which mirrors ``can_node._leg_sign``), so feeding
    this value straight into ``Setpoint.torque_ff`` makes the bench Teensy-side path
    sign-identical to the production ``motor_guard → can_node → ODrive`` chain.

    Returns ``0.0`` when ``params.enabled`` is False (the plain Teensy-side path). Parity
    with the production vectorised path is pinned by
    ``tests/motion/test_motor_guard_friction_ff.py``.
    """
    if not params.enabled:
        return 0.0
    av = abs(v_rps)
    sgn = 0.0 if av == 0.0 else math.copysign(1.0, v_rps)
    stribeck = (params.tau_c_A[axis]
                + (params.tau_s_A[axis] - params.tau_c_A[axis])
                  * math.exp(-(av / params.omega_s_rps[axis]) ** 2)
                + params.b_A_per_rps[axis] * av)
    gate = 1.0 - math.exp(-(av / params.v_gate_rps) ** 2)
    iq_friction = stribeck * gate
    iq_total = -sgn * iq_friction + params.load_offset_A[axis]
    return float(iq_total * params.ff_sign[axis] * params.motor_kt_nm_per_a)
