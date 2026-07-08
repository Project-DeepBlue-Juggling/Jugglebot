"""Reload-gate harness smoke tests (Phase 6).

A small-N smoke of the production-in-the-loop reload harness (the full 20-run gate
runs manually via ``python sim/reload_gate.py --trials 20``). Asserts the invariants
the plan makes load-bearing:
  * the harness drives the REAL ``planner.build_catch`` + ``KnotEmitter`` and every
    emitted knot is accepted by a real ``SetpointPump`` (zero pump rejects);
  * the platform physically catches the ball and holds it (contact→hold);
  * the hold-phase platform is quiescent (< 5 mm travel, < 1° tilt);
  * zero feasibility violations in accepted runs;
  * the JSON gate report is well-formed and publishes the required leg limits.

Kept to a few trials so CI stays fast (each MuJoCo trial is ~0.7 s wall). The
hand-contact velocity-match criterion is a documented light-scope deferral (see the
logbook), so it is reported but NOT asserted here.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pytest

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_REPO, os.path.join(_REPO, 'sim'),
           os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(_REPO, 'config', 'generated')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

pytest.importorskip('mujoco')

from sim.reload_gate import ReloadGate, ReloadGateConfig     # noqa: E402


@pytest.fixture(scope='module')
def smoke_report():
    """Run a 4-trial gate once for the whole module (MuJoCo is the slow part)."""
    return ReloadGate(ReloadGateConfig(trials=4, seed=0)).run()


def test_harness_catches_and_holds(smoke_report):
    """The production planner+emitter drive the plant to catch and hold the ball."""
    assert smoke_report['caught'] >= 3          # >= 3/4 caught (smoke)
    for t in smoke_report['results']:
        if t['caught']:
            assert t['held_at_end'], f"trial {t['idx']} caught but not held"


def test_every_emitted_knot_pump_accepted(smoke_report):
    """The load-bearing production-in-the-loop invariant, re-asserted in the gate:
    every knot the emitter produced was accepted by a real SetpointPump."""
    assert smoke_report['total_pump_rejects'] == 0


def test_no_feasibility_violations_in_accepted(smoke_report):
    assert smoke_report['feasibility_violations_in_accepted'] == 0


def test_hold_phase_is_quiescent(smoke_report):
    """Caught trials hold the platform still while the ball seats."""
    assert smoke_report['worst_hold_travel_mm'] < 5.0
    assert smoke_report['worst_hold_tilt_deg'] < 1.0


def test_no_post_contact_separation(smoke_report):
    """Contact→hold capture ⇒ a caught ball never separates."""
    assert smoke_report['worst_separation_ms'] <= 10.0


def test_report_publishes_required_limits(smoke_report):
    """The gate report carries the required leg vel/acc/jerk for Phase 4."""
    for key in ('required_leg_vel_mmps', 'required_leg_acc_mmps2',
                'required_leg_jerk_mmps3'):
        assert smoke_report[key] > 0.0
    # Required limits stay within the YAML hard ceilings.
    assert smoke_report['required_leg_vel_mmps'] < 280.0
    assert smoke_report['required_leg_acc_mmps2'] < 4000.0
    assert smoke_report['required_leg_jerk_mmps3'] < 200000.0


def test_report_well_formed(smoke_report):
    r = smoke_report
    assert r['gate'] == 'reload'
    assert r['trials'] == 4
    assert 'vel_match_frac' in r['deferred_criteria']     # documented deferral
    assert isinstance(r['core_passed'], bool)
    assert len(r['results']) == 4


def test_core_criteria_mostly_pass_on_smoke(smoke_report):
    """Most trials catch cleanly on the smoke run. (The 90 % ``core_passed`` gate is
    calibrated for the full N=20 run; a 4-trial subset can straddle the ≤80 mm reach
    envelope on one edge case — still CAUGHT — so assert the majority here and leave
    the full-gate threshold to the manual 20-run.)"""
    assert smoke_report['core_clean'] >= 3      # >= 3/4 fully core-clean


def test_write_json_report(tmp_path):
    """run_gate writes a well-formed JSON report to the requested path."""
    import json
    from sim.reload_gate import run_gate
    path = str(tmp_path / 'gate.json')
    rep = run_gate(ReloadGateConfig(trials=2, seed=1, report_path=path))
    assert os.path.exists(path)
    with open(path) as f:
        loaded = json.load(f)
    assert loaded['trials'] == 2
    assert rep['report_path'] == path
