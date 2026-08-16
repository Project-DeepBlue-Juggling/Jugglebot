"""Regression test: ensure the canonical motor torque-constant Kt value is
consistent across the YAML config and the few test-side literals that quote it.

The value 0.0624 Nm/A is the measured Kt from the Phase 2 multi-weight
bench fit (R²=0.994; canonical historical reference is
``tests/hardware/single_leg_test.py:92``).  As of 2026-05-07 it was promoted
to ``config/hardware_config.yaml:dynamics.motor_kt_nm_per_a`` to serve as the
single source of truth that the friction-FF integration (PR 2 of
``plans/archived/friction-ff-motor-guard-integration.md``) reads at runtime.

This test pins down: if anyone re-measures Kt and updates one location, they
must update *all* of them — the yaml value and the test-side literals must
agree, or the test fails and forces explicit reconciliation.

Why a test rather than a centralised constant?
  - The test-side literals exist for didactic purposes (they document the
    measurement source inline in test code that's read in isolation).
  - The runtime FF code reads YAML directly to avoid a generate_config.py
    codegen step (matching the pattern for ``motor_rotor_inertia_kgm2``,
    ``motor_pole_pairs``, ``motor_stator_slots`` — also runtime-loaded).
  - This test bridges the two by making divergence loud.
"""
from __future__ import annotations

import os
import re

import yaml


_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
_CONFIG_YAML = os.path.join(_REPO_ROOT, 'config', 'hardware_config.yaml')


def _load_yaml_kt() -> float:
    with open(_CONFIG_YAML, 'r') as f:
        cfg = yaml.safe_load(f)
    val = cfg.get('dynamics', {}).get('motor_kt_nm_per_a')
    assert val is not None, (
        'config/hardware_config.yaml is missing dynamics.motor_kt_nm_per_a; '
        'see plans/archived/friction-ff-motor-guard-integration.md §3.5')
    return float(val)


def test_yaml_kt_is_present_and_finite():
    kt = _load_yaml_kt()
    assert 0.01 < kt < 0.5, (
        f'motor_kt_nm_per_a = {kt} is outside the plausible range '
        f'for a small BLDC motor (~0.05 Nm/A typical); did the units change?')


def test_yaml_kt_matches_single_leg_test_literal():
    """tests/hardware/single_leg_test.py:92 has ``KT_MEASURED = 0.0624``.
    The YAML value must agree."""
    kt_yaml = _load_yaml_kt()
    leg_test_path = os.path.join(_REPO_ROOT, 'tests', 'hardware', 'single_leg_test.py')
    with open(leg_test_path, 'r') as f:
        text = f.read()
    m = re.search(r'^KT_MEASURED\s*=\s*([\d.]+)', text, re.MULTILINE)
    assert m, 'tests/hardware/single_leg_test.py no longer defines KT_MEASURED — update this test.'
    kt_literal = float(m.group(1))
    assert kt_yaml == kt_literal, (
        f'Kt mismatch: yaml={kt_yaml}, single_leg_test.py:KT_MEASURED={kt_literal}. '
        f'Update both consistently — the YAML value is the canonical source.')


def test_yaml_kt_matches_friction_ff_demo_literal():
    """tests/hardware/friction_ff_demo.py has ``KT_NM_PER_A = 0.0624``."""
    kt_yaml = _load_yaml_kt()
    demo_path = os.path.join(_REPO_ROOT, 'tests', 'hardware', 'friction_ff_demo.py')
    with open(demo_path, 'r') as f:
        text = f.read()
    m = re.search(r'^KT_NM_PER_A\s*=\s*([\d.]+)', text, re.MULTILINE)
    assert m, 'tests/hardware/friction_ff_demo.py no longer defines KT_NM_PER_A — update this test.'
    kt_literal = float(m.group(1))
    assert kt_yaml == kt_literal, (
        f'Kt mismatch: yaml={kt_yaml}, friction_ff_demo.py:KT_NM_PER_A={kt_literal}. '
        f'Update both consistently — the YAML value is the canonical source.')


def test_yaml_kt_matches_test_dynamics_literal():
    """tests/motion/test_dynamics.py has ``Kt = 0.0624`` in the gravity-FF test."""
    kt_yaml = _load_yaml_kt()
    test_path = os.path.join(_REPO_ROOT, 'tests', 'motion', 'test_dynamics.py')
    with open(test_path, 'r') as f:
        text = f.read()
    # The literal form in test_dynamics.py is ``Kt = 0.0624`` — match that.
    m = re.search(r'^\s*Kt\s*=\s*([\d.]+)', text, re.MULTILINE)
    assert m, 'tests/motion/test_dynamics.py no longer defines Kt as a local literal — update this test.'
    kt_literal = float(m.group(1))
    assert kt_yaml == kt_literal, (
        f'Kt mismatch: yaml={kt_yaml}, test_dynamics.py:Kt={kt_literal}. '
        f'Update both consistently — the YAML value is the canonical source.')
