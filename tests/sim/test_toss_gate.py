"""Toss-gate harness smoke tests (single-ball-toss Phase 2, Tier 8a).

A small-N smoke of the production-in-the-loop toss harness (the full 290-trial
factored sweep runs manually via ``python sim/toss_gate.py``). Asserts the
invariants the plan makes load-bearing:
  * the harness drives the REAL ``planner.build_move`` + ``planner.build_catch``
    + ``KnotEmitter`` and every emitted knot (BOTH the pre-position and catch
    pumps) is accepted by a real ``SetpointPump`` (zero pump rejects);
  * the production pre-position actually arrives at the nominated pose;
  * the platform physically tosses, catches, and holds the ball (contact→hold);
  * the hold-phase platform is quiescent (< 5 mm travel, < 1° tilt);
  * zero feasibility violations in accepted runs;
  * the JSON gate report is well-formed, stamps the binding-band and
    hardware-marginal annotations, and lists the vel-match deferral;
  * the Phase-1-deferred corner-pose announcement→build_catch wire crossing
    (incl. the pump crossing) is accepted.

Kept to a few trials so CI stays fast (each MuJoCo toss trial is ~2 s wall).
The hand-contact velocity-match criterion is the reload gate's documented
light-scope deferral, so it is reported but NOT asserted here.
"""

from __future__ import annotations

import math
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

from sim.toss_gate import (                                   # noqa: E402
    TossGate, TossGateConfig, default_grid, _toss_catch_pose,
)


@pytest.fixture(scope='module')
def smoke_report():
    """ONE small-N run shared by the assertions (MuJoCo is the slow part):
    centre pose at the T1-hardware flight + the sweep's worst corner."""
    cfg = TossGateConfig(
        points=[(0, 0, 170, 0.80), (60, 60, 200, 0.60)],
        trials_per_point=2, seed=0, contact_diag=False)
    return TossGate(cfg).run()


def test_harness_tosses_and_catches(smoke_report):
    """The production stack tosses the in-plant ball and catches + holds it."""
    assert smoke_report['caught'] >= 3          # >= 3/4 caught (smoke)
    for t in smoke_report['results']:
        if t['caught']:
            assert t['held_at_end'], f"trial {t['idx']} caught but not held"
            # Gate honesty (silent-dead-arm guard): a caught trial must have
            # actually ARMED the arm-and-forget catch from the tracked flight
            # — capture_rel_ms finite ⇔ t_arrival_pred was set at capture.
            assert t['catch_armed'], f"trial {t['idx']} caught without arming"
            assert math.isfinite(t['capture_rel_ms']), \
                f"trial {t['idx']} caught with no arrival prediction"


def test_every_emitted_knot_pump_accepted(smoke_report):
    """The load-bearing production-in-the-loop invariant, re-asserted: every
    knot BOTH pumps saw (pre-position AND catch) was accepted. Non-vacuous
    form: the harness counts the frames a pump actually returned a Setpoint
    for (non-None build), so accepted == emitted > 0 — a reject counter that
    silently never increments cannot fake this."""
    assert smoke_report['total_pump_rejects'] == 0
    assert smoke_report['total_pump_frames_emitted'] > 0
    assert (smoke_report['total_pump_frames_accepted']
            == smoke_report['total_pump_frames_emitted'])


def test_no_feasibility_violations_in_accepted(smoke_report):
    """Zero feasibility violations — made non-tautological by also pinning
    that all 4 smoke trials were ACCEPTED (an accepted row never carries a
    reject_code by construction, so the count alone can pass vacuously on a
    reject-everything run)."""
    assert smoke_report['feasibility_violations_in_accepted'] == 0
    assert smoke_report['accepted'] == 4


def test_preposition_reaches_nominated_pose(smoke_report):
    """The production build_move pre-position actually arrives (incl. z=200)."""
    assert smoke_report['worst_preposition_err_mm'] is not None
    assert smoke_report['worst_preposition_err_mm'] <= 2.0


def test_hold_phase_is_quiescent(smoke_report):
    """Caught trials hold the platform still while the ball seats."""
    assert smoke_report['worst_hold_travel_mm'] < 5.0
    assert smoke_report['worst_hold_tilt_deg'] < 1.0


def test_release_is_kinematic_and_ball_flies(smoke_report):
    """Per caught trial the released ball lands measurably near the nominated
    point and the capture time sits within +-150 ms of the ANNOUNCED landing
    (the Phase-2 quantification of the Phase-1 '~10 ms-class skew' note; the
    recorded value carries the documented sim-vs-hardware frame delta)."""
    caught = [t for t in smoke_report['results'] if t['caught']]
    assert caught, "no caught trials to check"
    for t in caught:
        assert math.isfinite(t['landing_err_mm'])
        assert abs(t['announced_landing_err_ms']) <= 150.0, \
            f"trial {t['idx']}: announced-landing error " \
            f"{t['announced_landing_err_ms']:.1f} ms"


def test_report_well_formed(smoke_report):
    r = smoke_report
    assert r['gate'] == 'toss'
    assert 'vel_match_frac' in r['deferred_criteria']     # documented deferral
    assert r['hardware_marginal_note']
    assert isinstance(r['passed'], bool)
    # BOTH binding bands are reported separately (orchestrator amendment).
    assert isinstance(r['passed_2_3_mps_band'], bool)
    assert isinstance(r['passed_T080_z170_band'], bool)
    assert len(r['results']) == 4
    rows = {p['point_id']: p for p in r['points']}
    assert len(rows) == 2
    for p in r['points']:
        assert 'binding_2_3_mps' in p
        assert 'binding_T080_z170' in p
        assert 'hardware_marginal_flight' in p
        # pass_9_of_10 is the recorded core_clean count against the exact
        # integer threshold (n=2 in the smoke ⇒ ceil(0.9·2) = 2).
        assert p['n'] == 2
        assert p['pass_threshold'] == 2
        assert p['pass_9_of_10'] == (p['core_clean'] >= p['pass_threshold'])
    t060 = rows['x60_y60_z200_T0.60']
    t080 = rows['x0_y0_z170_T0.80']
    assert t060['hardware_marginal_flight'] is True       # T=0.60 < 0.7 s
    assert t080['hardware_marginal_flight'] is False      # T=0.80 >= 0.7 s
    assert t060['binding_2_3_mps'] is True                # 2.95 m/s <= 3.0
    assert t080['binding_2_3_mps'] is False               # 3.93 m/s
    assert t080['binding_T080_z170'] is True              # the amendment band
    assert t060['binding_T080_z170'] is False


def test_pass_threshold_exact_integer_math():
    """ceil(0.9·n) pins — pure. n=10 must be 9 (0.9*10 float-rounds ABOVE 9,
    the reason _pass_threshold uses exact integer math), n=2/6/1 pin the
    small-N smoke and debug slices."""
    from sim.toss_gate import _pass_threshold
    assert _pass_threshold(10) == 9
    assert _pass_threshold(2) == 2
    assert _pass_threshold(6) == 6
    assert _pass_threshold(1) == 1


def test_sweep_points_inside_fsm_bands():
    """Drift guard between the gate sweep and the toss FSM's CHECKING rejects:
    every default-grid point satisfies the sequencer bands using the IMPORTED
    constants. Pure — no plant."""
    from jugglebot.toss_sequencer import (
        TOSS_XY_LIMIT_MM, TOSS_ACTIVE_Z_MM, TOSS_Z_BAND_MM,
        FLIGHT_TIME_MIN_S, FLIGHT_TIME_MAX_S,
        TEENSY_MIN_EVENT_VEL_MPS, TEENSY_MAX_EVENT_VEL_MPS,
    )
    from jugglebot.motion.trajectory import toss_release
    pts = default_grid('factored')
    assert len(pts) == 29
    for (x, y, z, T) in pts:
        assert abs(x) <= TOSS_XY_LIMIT_MM and abs(y) <= TOSS_XY_LIMIT_MM
        assert abs(z - TOSS_ACTIVE_Z_MM) <= TOSS_Z_BAND_MM
        assert FLIGHT_TIME_MIN_S <= T <= FLIGHT_TIME_MAX_S
        ev = toss_release.compute_release_state((x, y, z), T).event_vel_mps
        assert TEENSY_MIN_EVENT_VEL_MPS <= ev <= TEENSY_MAX_EVENT_VEL_MPS


def test_write_json_report(tmp_path):
    """run_gate writes a well-formed JSON report to the requested path."""
    import json
    from sim.toss_gate import run_gate
    path = str(tmp_path / 'gate.json')
    rep = run_gate(TossGateConfig(points=[(0, 0, 170, 0.80)],
                                  trials_per_point=1, seed=1,
                                  report_path=path, contact_diag=False))
    assert os.path.exists(path)
    with open(path) as f:
        loaded = json.load(f)
    assert loaded['trials'] == 1
    assert rep['report_path'] == path


def test_corner_pose_announcement_crosses_into_accepted_build_catch():
    """The Phase-1 DEFERRED wire-crossing test (Phase-1 logbook Known
    limitations, last bullet). No plant stepping: for the sweep's worst corner
    (60, 60, 200) at T = 0.60 the production announcement feeds the gate's
    catch-pose helper into an ACCEPTED ``planner.build_catch``, and every knot
    of the resulting plan is accepted by a real SetpointPump. Pins the
    announced-global → catch-frame xy identity, the nominated-z path, corner
    feasibility, and pump acceptance."""
    import jugglebot.hardware_config as hw
    from jugglebot.motion.geometry import StewartGeometry
    from jugglebot.motion.trajectory import KnotEmitter, TrajectoryLimits
    from jugglebot.motion.trajectory import planner
    from jugglebot.motion.trajectory import toss_release
    from controller.teensy_link.setpoint_pump import SetpointPump
    from sim.gate_common import CUP_Z_BASE_MM, Z_ACTIVE_MM
    from sim.hand.trajectory import HandCatchTrajectory

    rs = toss_release.compute_release_state((60, 60, 200), 0.60)
    ann = toss_release.build_announcement_fields(rs, throw_time_s=100.0)
    armed_v = float(np.linalg.norm(ann['landing_velocity'])) / 1000.0
    cup_world_z = (CUP_Z_BASE_MM + HandCatchTrajectory(armed_v).sample(0.0)
                   + (200.0 - Z_ACTIVE_MM))
    catch_pose, _ = _toss_catch_pose(
        ann['landing_position'][:2], ann['landing_velocity'],
        cup_world_z, 200.0)
    # The nominated-z adaptation (the ONE delta vs reload's helper).
    assert catch_pose[2] == 200.0
    # Announced-global xy == nominated xy, within the lever-arm shift.
    assert abs(catch_pose[0] - 60.0) < 5.0
    assert abs(catch_pose[1] - 60.0) < 5.0

    geom = StewartGeometry()
    limits = TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=250.0, leg_acc_mmps2=3000.0, leg_jerk_mmps3=150000.0)
    pose_nom = np.array([60.0, 60.0, 200.0, 0.0, 0.0, 0.0])
    # Accepted (no TrajectoryInfeasible) at the gate's lead = prep_gap + T.
    plan, report = planner.build_catch(
        (pose_nom, np.zeros(6), np.zeros(6)), catch_pose, 0.8 + 0.60,
        limits, geom, settle_hold_s=float(hw.JB_TRAJ_CATCH_SETTLE_HOLD_S))
    assert report.ok
    # Full wire crossing: every emitted knot through a real pump. Non-vacuous
    # form: assert the ACCEPT (a returned Setpoint) per frame — a build that
    # skipped (None, None) or a reject counter that never increments would
    # slip a rejects-only check.
    emitter = KnotEmitter(geom)
    pump = SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV,
                        max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV)
    total = sum(float(s.duration) for s in plan.segments)
    n_knots = int(math.ceil(total / 0.025)) + 2
    for n in range(n_knots):
        frame = emitter.frame(plan, n * 0.025, n)
        sp, reason = pump.build(frame, t_origin_us=int(n * 25000))
        assert sp is not None, f"knot {n} not accepted: {reason}"
    assert pump.frames_rejected == 0


def test_contact_diag_column_present():
    """The non-gating contact-physics diagnostic column produces its report
    shape (mode + rows). Catch outcome NOT asserted — the column is
    diagnostic-only and never feeds ``passed``."""
    cfg = TossGateConfig(points=[(0, 0, 170, 0.80)], trials_per_point=1,
                         seed=0, contact_diag=True, diag_trials=1)
    gate = TossGate(cfg)
    # The GATING plant must be the kinematic-hold one (contact physics is the
    # documented low-fidelity element); only the diag plant carries contact.
    assert gate.plant.contact_carry is False
    diag = gate._run_contact_diag(flights=[0.80])
    assert gate._diag_plant.contact_carry is True
    assert diag['mode'] == 'detach'
    assert diag['trials'] == 1
    assert len(diag['rows']) == 1
    assert diag['pump_rejects'] == 0
    row = diag['rows'][0]
    for key in ('caught', 'held', 'landing_err_mm', 'separated',
                'seat_offset_mm', 'pump_rejects'):
        assert key in row
