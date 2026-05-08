"""Sanity tests for the friction-FF YAML loader.

Covers the contract from
``plans/active/friction-ff-motor-guard-integration.md`` §3.1: the loader
returns finite, length-6 per-leg arrays, the global flag, the smooth-gate
scale, and Kt.  Defaults are pinned against the values committed in
``config/hardware_config.yaml`` so a YAML drift breaks this test loudly.

Schema is post-PR-2.1: ``v_gate_rps`` replaced ``stiction_boost_threshold_rps``;
``ff_sign`` defaults flipped from -1 (bench) to +1 (platform).  See
``logbook/2026-05-08-friction-ff-platform-limit-cycle.md``.
"""

from __future__ import annotations

import numpy as np

from jugglebot.motion.friction_ff_params import (
    FrictionFFParams,
    load_params,
)


def test_load_params_returns_well_formed_arrays():
    p = load_params()
    assert isinstance(p, FrictionFFParams)
    # Global flag — should ship disabled until on-platform A/B validates
    # the smooth-gate fix.
    assert p.enabled is False
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
