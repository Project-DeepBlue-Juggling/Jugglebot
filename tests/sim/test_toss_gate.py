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

pytest.importorskip('mujoco')

from sim.toss_gate import (                                   # noqa: E402
    TossGate, TossGateConfig, default_grid, default_grid_8b, _toss_catch_pose,
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
    constants. Pure — no plant.

    Since 2026-08-18 the flight band is DERIVED (contract C-HAND-3) and the
    sweep's outer rungs are read from it, so the two now move together by
    construction — but the guard stays, and gains teeth: it also drives the REAL
    gate (``throw_envelope.evaluate``) on each point's production release speed,
    which the band alone cannot see. A sweep that measured refused points would
    be reporting a catch rate for throws the machine will not make."""
    from jugglebot.toss_sequencer import (
        TOSS_XY_LIMIT_MM, TOSS_ACTIVE_Z_MM, TOSS_Z_BAND_MM,
        FLIGHT_TIME_MIN_S, FLIGHT_TIME_MAX_S,
        TEENSY_MIN_EVENT_VEL_MPS, TEENSY_MAX_EVENT_VEL_MPS,
    )
    from jugglebot.motion.trajectory import throw_envelope, toss_release
    pts = default_grid('factored')
    assert len(pts) == 29
    for (x, y, z, T) in pts:
        assert abs(x) <= TOSS_XY_LIMIT_MM and abs(y) <= TOSS_XY_LIMIT_MM
        assert abs(z - TOSS_ACTIVE_Z_MM) <= TOSS_Z_BAND_MM
        assert FLIGHT_TIME_MIN_S <= T <= FLIGHT_TIME_MAX_S
        ev = toss_release.compute_release_state((x, y, z), T).event_vel_mps
        assert TEENSY_MIN_EVENT_VEL_MPS <= ev <= TEENSY_MAX_EVENT_VEL_MPS
        verdict = throw_envelope.evaluate(T, ev)
        assert verdict.ok, f'sweep point {(x, y, z, T)}: {verdict.message}'

    # The Tier-8b grid needs its own pass, and it is NOT covered by the band
    # check above: an 8b goal is AIMED, so its commanded release is faster than
    # the co-located inverse of its flight time. At the 8a ceiling a 50 mm
    # displacement is already 0.6 mm/s over the END_STOP bound — the first
    # attempt at this change put the advisory spots exactly there and they were
    # refused. Only `evaluate` on the AIMED speed can see that.
    from jugglebot.motion.trajectory.toss_release import compute_release_state_tilted
    for (x, y, z, T) in default_grid_8b():
        rel = compute_release_state_tilted((x, y, z), T,
                                           throw_site_xy_mm=(0.0, 0.0))
        verdict = throw_envelope.evaluate(T, rel.event_vel_mps)
        assert verdict.ok, f'8b sweep point {(x, y, z, T)}: {verdict.message}'


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
    from teensy_link.setpoint_pump import SetpointPump
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


# ── Tier 8b (Phase 4): displaced tilt-aimed throw→catch ────────────────────────

@pytest.fixture(scope='module')
def smoke_8b_report():
    """ONE small-N Tier-8b run shared by the 8b assertions: the centre (A,
    displacement 0) + one 50 mm-ring point (binding) at T = 0.80, 2 trials
    each, no contact diag (the gating column)."""
    from sim.toss_gate import TossGateConfig, TossGate
    cfg = TossGateConfig(
        tier='8b', throw_site_xy=(0.0, 0.0),
        points=[(0.0, 0.0, 170.0, 0.80), (50.0, 0.0, 170.0, 0.80)],
        trials_per_point=2, seed=0, contact_diag=False)
    return TossGate(cfg).run()


def test_8b_trial_smoke(smoke_8b_report):
    """An 8b trial drives the tilted-release + hold→reach production choreography
    end to end: the displaced point commands a real pre-tilt (rx/ry != 0), the
    A→B reach installs at t_release (reach_lead_s finite, ~= the flight), the
    hold→reach crossing is pump-accepted (zero pump rejects), and a caught trial
    actually ARMED from the tracked flight."""
    r = smoke_8b_report
    assert r['tier'] == '8b'
    assert r['total_pump_rejects'] == 0             # incl. the hold→reach crossing
    assert r['total_pump_frames_accepted'] == r['total_pump_frames_emitted']
    ring = [t for t in r['results']
            if t['accepted'] and t['displacement_mm'] > 1.0]
    assert ring, "no accepted displaced 8b trial in the smoke"
    for t in ring:
        # A displaced throw is aimed via a real pre-tilt at A (rx or ry != 0) —
        # the swing-compensated pre-position pose the trial reaches.
        assert abs(t['pretilt_err_deg']) < 5.0      # commanded pre-tilt reached
        assert math.isfinite(t['reach_lead_s'])     # the A→B reach installed
        assert 0.6 <= t['reach_lead_s'] <= 0.85     # lead ~= T (minus one tick)
        assert math.isfinite(t['displacement_mm'])
    for t in r['results']:
        if t['caught']:
            assert t['catch_armed'], f"trial {t['idx']} caught without arming"


def test_8b_binding_band_geometric_and_honest(smoke_8b_report):
    """Third-band honesty pins (mirroring the Phase-2 lessons): binding_8b_ring
    is PURELY geometric (centre + 50 mm ring at T=0.80/z=170; a rejected binding
    point can never leave the band vacuously), the 70 mm ring / T=0.95 spots are
    ADVISORY, and passed_8b_ring gates on core_clean >= ceil(0.9 n) — which
    itself requires catch_armed (no ball-into-a-parked-cup pass)."""
    r = smoke_8b_report
    assert isinstance(r['passed_8b_ring_band'], bool)
    rows = {p['point_id']: p for p in r['points']}
    centre = rows['x0_y0_z170_T0.80']
    ring = rows['x50_y0_z170_T0.80']
    assert centre['binding_8b_ring'] is True         # displacement 0 = A, binding
    assert ring['binding_8b_ring'] is True           # 50 mm ring, binding
    assert centre['displacement_mm'] == pytest.approx(0.0)
    assert ring['displacement_mm'] == pytest.approx(50.0)
    # The 8a bands do not apply to an 8b run.
    assert centre['binding_2_3_mps'] is False
    assert centre['binding_T080_z170'] is False
    # pass_9_of_10 is core_clean vs the exact integer threshold (n=2 ⇒ 2).
    from sim.toss_gate import _pass_threshold
    for p in r['points']:
        assert p['pass_threshold'] == _pass_threshold(p['n'])
        assert p['pass_9_of_10'] == (p['core_clean'] >= p['pass_threshold'])
    # passed conjoins ONLY the 8b ring band with the invariants (not the 8a bands).
    assert r['passed'] == bool(r['passed_8b_ring_band']
                               and r['feasibility_violations_in_accepted'] == 0
                               and r['total_pump_rejects'] == 0
                               and r['total_pump_frames_accepted']
                               == r['total_pump_frames_emitted'])


def test_8b_advisory_rings_not_binding():
    """The advisory rings ({70, 100, cap} since Phase E) and the T=0.95 spots
    are ADVISORY — never in the binding band, so a dirty edge point cannot fail
    the gate.

    Re-pointed 2026-07-29 from a single 70 mm advisory ring. The cap ring is
    read from the shipped config key so the gate and the shipped
    ``toss_max_displacement_mm`` can never describe different machines — and it
    stays ADVISORY on purpose: the catch RATE at the cap is dominated by a
    release-noise magnitude that is still the Phase-5 T0 PLACEHOLDER, so gating
    the shipped cap on it would make the cap an artefact of an unmeasured
    number. What the advisory rings DO gate is the invariant half (every emitted
    knot pump-accepted, zero feasibility violations), which they feed because
    they run the full production pipeline."""
    import jugglebot.hardware_config as hw
    from sim.toss_gate import (default_grid_8b, _TOSS_8B_RING_MM,
                               _TOSS_8B_ADVISORY_RINGS_MM)
    cap = float(hw.JB_OP_TOSS_MAX_DISPLACEMENT_MM)
    assert _TOSS_8B_ADVISORY_RINGS_MM[-1] == pytest.approx(cap)
    pts = default_grid_8b((0.0, 0.0))
    # 1 centre + 8 binding ring + 3×8 advisory rings + 2 T=0.95 spots = 35.
    assert len(pts) == 1 + 8 + 8 * len(_TOSS_8B_ADVISORY_RINGS_MM) + 2 == 35
    disps = sorted({round(math.hypot(x, y), 1) for (x, y, z, T) in pts})
    assert 0.0 in disps and 50.0 in disps and 70.0 in disps
    assert 100.0 in disps and round(cap, 1) in disps
    # No binding-band membership is claimed for any advisory-ring point.
    for (x, y, z, T) in pts:
        d = math.hypot(x, y)
        for r in _TOSS_8B_ADVISORY_RINGS_MM:
            if abs(d - r) < 1e-6:
                assert d > _TOSS_8B_RING_MM   # excluded from the binding ring
    # …and an explicit override replaces them (the CLI's --advisory-rings).
    slim = default_grid_8b((0.0, 0.0), advisory_rings_mm=(70.0,))
    assert len(slim) == 19


def test_8b_prepare_commands_nonzero_pretilt():
    """A displaced 8b throw is aimed via a NON-ZERO pre-tilt at A (rx/ry != 0 —
    all lateral take-off comes from the slider through the tilt, never platform
    translation); the co-located centre (displacement 0) is a level throw. Pure
    synthesis (no plant stepping)."""
    from sim.toss_gate import TossGate, TossGateConfig
    gate = TossGate(TossGateConfig(tier='8b', throw_site_xy=(0.0, 0.0),
                                   contact_diag=False))
    s_disp = gate._prepare_toss((50.0, 0.0, 170.0, 0.80))
    assert s_disp.reject_code is None
    assert s_disp.hold_reach_swap is True                # deferred hold→reach path
    assert abs(s_disp.pretilt_rx) + abs(s_disp.pretilt_ry) > 1e-3   # a real aim
    assert s_disp.displacement_mm == pytest.approx(50.0)
    s_ctr = gate._prepare_toss((0.0, 0.0, 170.0, 0.80))
    assert abs(s_ctr.pretilt_rx) < 1e-9 and abs(s_ctr.pretilt_ry) < 1e-9
    assert s_ctr.displacement_mm == pytest.approx(0.0)


def test_8b_asymmetry_map_radii_reach_the_shipped_cap():
    """The MAP's default radii must span the shipped displacement cap, or the
    directional evidence the operator's ladder relies on stops short of the
    range the FSM actually permits."""
    import jugglebot.hardware_config as hw
    from sim.toss_gate import _ASYMMETRY_RADII_MM
    assert 70.0 in _ASYMMETRY_RADII_MM and 100.0 in _ASYMMETRY_RADII_MM
    assert max(_ASYMMETRY_RADII_MM) >= float(hw.JB_OP_TOSS_MAX_DISPLACEMENT_MM)


def test_8b_asymmetry_map_present_and_non_gating():
    """The ±{70,100,150} mm directional-asymmetry MAP lives in the DETACH diag column
    (contact_diagnostic.asymmetry_map), is NON-GATING (gating flag False, never
    feeds passed), and reports the per-direction seated_n (post-catch seat) +
    landing-error-vs-B metric. A 1-cell slice keeps CI fast (the full map is
    8×2×2×diag_trials)."""
    from sim.toss_gate import TossGateConfig, TossGate
    cfg = TossGateConfig(tier='8b', throw_site_xy=(0.0, 0.0),
                         points=[(0.0, 0.0, 170.0, 0.80)], trials_per_point=1,
                         seed=0, contact_diag=True, diag_trials=1)
    gate = TossGate(cfg)
    amap = gate._run_asymmetry_map(dirs=[(1.0, 0.0)], radii=[100.0],
                                   flights=[0.80])
    assert amap['gating'] is False                   # advisory by construction
    assert amap['radii_mm'] == [100.0]
    assert len(amap['cells']) == 1
    cell = amap['cells'][0]
    for key in ('direction', 'radius_mm', 'flight_time_s', 'target_xy_mm',
                'n', 'caught', 'seated_n', 'landing_err_mm_mean',
                'landing_err_mm_worst'):
        assert key in cell
    assert cell['radius_mm'] == 100.0
    assert cell['target_xy_mm'] == [100.0, 0.0]      # +x at 100 mm (Rung-2a good cell)
    assert cell['n'] == 1
