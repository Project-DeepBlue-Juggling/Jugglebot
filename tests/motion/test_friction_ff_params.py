"""Sanity tests for the friction-FF YAML loader.

Covers the contract from
``plans/active/friction-ff-motor-guard-integration.md`` §3.1: the loader
returns finite, length-6 per-leg arrays, plus the global flag, the
boost threshold, and Kt.  Behaviour-neutral defaults are checked against
the bench-fit values committed in ``config/hardware_config.yaml``.
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
    # Global flag — should ship disabled until PR 3 platform validation.
    assert p.enabled is False
    # Boost threshold — bench-tuned at 0.20 rev/s.
    assert 0.0 < p.stiction_boost_threshold_rps < 1.0
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
    """Bench-fit values from logbook 2026-04-27 are the starting estimate.

    These are the values committed by PR 2; if they drift, somebody has
    refit on platform and should update this test alongside the YAML.
    """
    p = load_params()
    np.testing.assert_allclose(p.tau_c_A, 1.094, atol=1e-9)
    np.testing.assert_allclose(p.tau_s_A, 1.953, atol=1e-9)
    np.testing.assert_allclose(p.omega_s_rps, 0.251, atol=1e-9)
    np.testing.assert_allclose(p.b_A_per_rps, 0.0173, atol=1e-9)
    # Platform default: load_offset = 0 (per integration plan §6.4 — let the
    # per-leg gain integrator handle gravity until kinematics-derived per-leg
    # gravity FF lands as a Phase-2 refinement).
    np.testing.assert_allclose(p.load_offset_A, 0.0, atol=1e-12)
    # Bench convention is uniform −1 across all six legs; verify before
    # PR 3 flag-flip.
    np.testing.assert_array_equal(p.ff_sign, np.full(6, -1.0))
