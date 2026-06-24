"""Sanity tests for the friction-FF YAML loader.

Covers the contract from
``plans/archived/2026-05-08 friction-ff-motor-guard-integration.md`` §3.1: the loader
returns finite, length-6 per-leg arrays, the global flag, the smooth-gate
scale, and Kt.  Defaults are pinned against the values committed in
``config/hardware_config.yaml`` so a YAML drift breaks this test loudly.

Schema is post-PR-2.1: ``v_gate_rps`` replaced ``stiction_boost_threshold_rps``;
``ff_sign`` defaults flipped from -1 (bench) to +1 (platform).  See
``logbook/2026-05-08-friction-ff-platform-limit-cycle.md``.
"""

from __future__ import annotations

import os

import numpy as np

from jugglebot.motion import friction_ff_params as ffp
from jugglebot.motion.friction_ff_params import (
    FrictionFFParams,
    load_params,
)


def test_load_params_returns_well_formed_arrays():
    p = load_params()
    assert isinstance(p, FrictionFFParams)
    # Global flag — enabled in PR 3b (2026-05-08) after on-platform A/B
    # validation of the smooth-gate fix.  See logbook entry
    # 2026-05-08-friction-ff-platform-limit-cycle.md.
    assert p.enabled is True
    # Smooth-gate scale — sized larger than the platform's hold-noise
    # floor (BASELINE 99%ile vel_ff during hold ≈ 0.02 rev/s).
    assert 0.0 < p.v_gate_rps < 1.0
    # Kt — from the canonical D6374-150Kv multi-weight bench fit.
    assert 0.01 < p.motor_kt_nm_per_a < 0.5
    # Per-leg arrays: shape (6,), all finite.
    for name in ('ff_sign', 'tau_c_A', 'tau_s_A', 'omega_s_rps',
                 'b_A_per_rps', 'load_offset_A'):
        arr = getattr(p, name)
        assert isinstance(arr, np.ndarray), f'{name} should be ndarray'
        assert arr.shape == (6,), f'{name} shape {arr.shape} != (6,)'
        assert np.all(np.isfinite(arr)), f'{name} has non-finite values: {arr}'


def test_load_params_bench_defaults_present():
    """Bench-fit Stribeck values + platform-derived ff_sign / v_gate.

    The Stribeck fit (τ_c, τ_s, ω_s, b) comes from
    ``logbook/2026-04-27-friction-feedforward-bench-validation.md``.
    ``ff_sign`` and ``v_gate_rps`` are platform-derived; see
    ``logbook/2026-05-08-friction-ff-platform-limit-cycle.md``.

    If anyone refits any of these on platform, they should update this
    test alongside the YAML so divergence is caught.
    """
    p = load_params()
    # Stribeck taper: bench-fit values.
    np.testing.assert_allclose(p.tau_c_A, 1.094, atol=1e-9)
    np.testing.assert_allclose(p.tau_s_A, 1.953, atol=1e-9)
    np.testing.assert_allclose(p.omega_s_rps, 0.251, atol=1e-9)
    np.testing.assert_allclose(p.b_A_per_rps, 0.0173, atol=1e-9)
    # Platform default: load_offset = 0 (per integration plan §6.4 — let
    # the per-leg gain integrator handle gravity until kinematics-derived
    # per-leg gravity FF lands as a Phase-2 refinement).
    np.testing.assert_allclose(p.load_offset_A, 0.0, atol=1e-12)
    # Platform sign convention: +1 for all legs (can_node._send_position_target
    # negates torque_ff for leg axes; bench's -1 was correct without that
    # negation but inverts on platform — see motor_guard.py docstring).
    np.testing.assert_array_equal(p.ff_sign, np.full(6, +1.0))
    # Smooth-gate scale: 0.05 rev/s — sized above platform hold-noise.
    assert p.v_gate_rps == 0.05


# ── Path resolution (Phase 11 install-tree fix, 2026-06-24) ────────────
# The loader must resolve hardware_config.yaml from BOTH the source tree
# (pytest, checkout) and the colcon install tree (production motor_guard).
# See friction_ff_params._resolve_yaml_path.

def test_resolver_returns_existing_yaml(monkeypatch):
    """With no override, resolution finds a real hardware_config.yaml.

    Robust to either branch: ament share dir (if a built+sourced overlay
    is present) or the source-tree fallback — both end in
    config/hardware_config.yaml and must exist.
    """
    monkeypatch.delenv(ffp._HW_CONFIG_ENV, raising=False)
    path = ffp._resolve_yaml_path()
    assert os.path.exists(path), f'resolved path does not exist: {path}'
    assert path.endswith(os.path.join('config', 'hardware_config.yaml'))


def test_resolver_honors_env_override(tmp_path, monkeypatch):
    """$JUGGLEBOT_HW_CONFIG wins over the share dir / source tree."""
    fake = tmp_path / 'hw.yaml'
    fake.write_text('friction_ff: {}\n')
    monkeypatch.setenv(ffp._HW_CONFIG_ENV, str(fake))
    assert ffp._resolve_yaml_path() == str(fake)


def test_resolver_skips_missing_env_override(tmp_path, monkeypatch):
    """A non-existent override is skipped, not fatal — falls through."""
    monkeypatch.setenv(ffp._HW_CONFIG_ENV, str(tmp_path / 'does_not_exist.yaml'))
    path = ffp._resolve_yaml_path()
    # Falls through to the real (existing) source-tree / share YAML.
    assert os.path.exists(path)
    assert path.endswith(os.path.join('config', 'hardware_config.yaml'))


def test_resolver_raises_when_nothing_found(tmp_path, monkeypatch):
    """All candidates missing → loud FileNotFoundError, not silent."""
    monkeypatch.setenv(ffp._HW_CONFIG_ENV, str(tmp_path / 'nope.yaml'))
    monkeypatch.setattr(ffp, '_SRC_HW_YAML', str(tmp_path / 'also_nope.yaml'))
    # Force the ament branch to miss too (import may or may not succeed;
    # either way the share path won't exist under tmp_path).
    import builtins
    real_import = builtins.__import__

    def _no_ament(name, *args, **kwargs):
        if name.startswith('ament_index_python'):
            raise ImportError('ament_index_python unavailable (test)')
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', _no_ament)
    try:
        ffp._resolve_yaml_path()
    except FileNotFoundError as exc:
        assert 'hardware_config.yaml' in str(exc)
        assert ffp._HW_CONFIG_ENV in str(exc)
    else:
        raise AssertionError('expected FileNotFoundError when no YAML exists')


def test_load_params_honors_env_override(monkeypatch):
    """Full round-trip: load_params reads the env-pointed YAML."""
    monkeypatch.setenv(ffp._HW_CONFIG_ENV, ffp._SRC_HW_YAML)
    p = load_params()
    assert isinstance(p, FrictionFFParams)
    assert p.enabled is True
