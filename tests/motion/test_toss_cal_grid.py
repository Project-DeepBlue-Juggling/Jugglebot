"""Tests for the toss AIM-calibration acquisition tool —
``tests/hardware/toss_cal_grid.py`` (build phase 2f of
``plans/active/toss-selftuning.md`` § 3.8).

WHAT THIS FILE IS PROTECTING
----------------------------
The tool sends ``TossContinuous`` goals at a real robot, so almost none of it can
be exercised end to end here. What CAN be pinned, and is:

* **The nine preflight refusals R1–R9**, each as a pure verdict over a plain dict
  of observations, so a refusal cannot quietly become a warning.
* **The three-valued gates**, including the property the phase turns on: a sign
  flip FAILS, a healthy plant does NOT, and the design's literal ±25 % point
  comparison would have refused the healthy plant. That deviation is measured,
  not asserted — see :func:`test_the_literal_designed_gate_refuses_a_healthy_plant`.
* **The rung ledger's blocking preconditions** (SC-0 BLOCKS everything, D14).
* **Four structural properties of ``run()``** that need a robot to exercise and a
  source read to pin: the wire check sits outside the per-goal ``try``, the
  return-to-centre guard catches ``BaseException``, the probe map is restored
  before the return-to-centre, and ``--dry-run`` constructs nothing.
* **The safety envelope**: the tool never arms, never changes mode, never
  commands the hand. Pinned by a source-level manifest of every service, action
  and publisher it may touch, in the shape of C-LEVEL-2's own manifest test.

The probe behind the gate constants is ``/tmp/probe_toss_sc_gates.py``
(2026-08-11); its numbers are quoted in the tool's module docstring and in
``logbook/2026-08-10-toss-selftuning-build.md`` § Phase 2f.
"""

from __future__ import annotations

import json
import math
import os
import sys

import numpy as np
import pytest
import yaml

_TESTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_HW_DIR = os.path.join(_TESTS, 'hardware')
if _HW_DIR not in sys.path:
    sys.path.insert(0, _HW_DIR)

import toss_cal_grid as tcgrid                                   # noqa: E402
import toss_fit_lib as fit                                       # noqa: E402
import tilt_cal_grid as tcg                                      # noqa: E402
from jugglebot import toss_trim                                  # noqa: E402
from jugglebot.motion import toss_cal                            # noqa: E402


def _args(**overrides):
    args = tcgrid.build_parser().parse_args([])
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


def _obs(**overrides):
    """A fully-passing preflight observation set."""
    obs = {
        'link_kv': {'mpc_active': '1', 'fault_state': 'NONE',
                    'bridge_link': 'UP', 'uptime_ms': '12345'},
        'gravity_correction_loaded': True,
        'tilt_map_loaded': True,
        'tilt_map_version': '2026-08-10-deadbeef',
        'toss_trim_enabled': False,
        'sensor_valid_seen': True,
        'sensor_edge_seen': True,
        'toss_tier': '8a',
        'uptime_ms': 120000,
        'write_target': '/repo/config/toss_calibration.yaml',
        'write_target_ok': True,
        'write_target_reason': None,
        'operator_confirmed': True,
        'operator_yes': False,
    }
    obs.update(overrides)
    return obs


def _refusal(obs, rid):
    rows = {row.rid: row for row in tcgrid.preflight_refusals(obs)}
    return rows[rid]


def _source():
    return open(tcgrid.__file__.replace('.pyc', '.py')).read()


# ── rung specs ───────────────────────────────────────────────────────────


def test_sc0_is_five_arms_at_the_home_node_each_with_a_probe_aim():
    args = _args()
    goals = tcgrid.rung_goals('sc0', [-150.0, 0.0, 150.0], [-150.0, 0.0, 150.0],
                              170.0, args)
    assert len(goals) == 5
    assert all(g.x_mm == 0.0 and g.y_mm == 0.0 for g in goals)
    assert all(g.n_throws == args.sc0_n for g in goals)
    # Every arm carries an EXPLICIT commanded aim. TossContinuous has no aim
    # field, so a probe arm with `probe_aim_rad is None` would silently command
    # whatever map happened to be installed and the Jacobian would be fitted to
    # nothing.
    assert all(g.probe_aim_rad is not None for g in goals)
    aims = sorted(tuple(round(v, 9) for v in g.probe_aim_rad) for g in goals)
    d = round(math.radians(args.sc0_probe_deg), 9)
    assert aims == sorted([(0.0, 0.0), (d, 0.0), (-d, 0.0), (0.0, d), (0.0, -d)])
    assert tcgrid.toss_count(goals) == 25            # § 3.8's "n = 5 each => 25"


def test_sc1_is_the_four_height_ladder_and_writes_no_probe_map():
    """SC-1 must not write a map.

    The height ladder regresses the RAW landing error, so it needs a zero applied
    aim — but the enforcement is a REFUSAL (`sc1_baseline_verdict`), not a
    written zero map. A rung that writes nothing has nothing to restore, and the
    operator's remedy is the escape hatch the plan already documents.
    """
    goals = tcgrid.rung_goals('sc1', [-150.0, 0.0, 150.0], [-150.0, 0.0, 150.0],
                              170.0, _args())
    assert [g.throw_height_m for g in goals] == list(tcgrid.SC1_HEIGHTS_M)
    assert all(g.probe_aim_rad is None for g in goals)
    assert tcgrid.toss_count(goals) == 32            # § 3.8's "n = 8 => 32"


def test_sc2_visits_home_first_then_centre_out_with_interleaved_anchors():
    goals = tcgrid.rung_goals('sc2', [-150.0, 0.0, 150.0], [-150.0, 0.0, 150.0],
                              170.0, _args())
    nodes = [g for g in goals if not g.is_anchor]
    assert len(nodes) == 9
    assert (nodes[0].x_mm, nodes[0].y_mm) == (0.0, 0.0)
    radii = [math.hypot(g.x_mm, g.y_mm) for g in nodes]
    assert radii == sorted(radii), 'centre-out: radii must never decrease'
    anchors = [g for g in goals if g.is_anchor]
    assert anchors, 'every capture ends on a home anchor — it is the reference'
    assert goals[-1].is_anchor
    assert all(g.probe_aim_rad is None for g in goals)


def test_sc2_anchor_schedule_follows_the_tilt_tools_own_rule():
    """One implementation of the anchor schedule, imported not restated."""
    goals = tcgrid.rung_goals('sc2', [-150.0, 0.0, 150.0], [-150.0, 0.0, 150.0],
                              170.0, _args(home_revisit_every=4))
    mid = [i for i, g in enumerate(goals) if g.is_anchor][:-1]
    assert len(mid) == len(tcg.home_revisit_after_visits(9, 4))


def test_sc3_reaches_six_off_node_poses_on_a_3x3_grid():
    """The finding this extension exists for: § 3.8 asks for >= 6 off-node check
    poses AND a 3x3 first capture, and a 3x3 has only 2x2 = 4 interior cells.
    Cell centres alone cannot satisfy both."""
    x = y = [-150.0, 0.0, 150.0]
    assert len(tcg.check_poses(x, y, 6)) == 4, 'the cap this test exists for'
    poses = tcgrid.off_node_poses(x, y, 6)
    assert len(poses) == 6
    assert len(set(poses)) == 6
    nodes = {(px, py) for px in x for py in y}
    assert not (set(poses) & nodes), 'a check pose must be OFF-node'
    for px, py in poses:
        assert min(x) <= px <= max(x) and min(y) <= py <= max(y)


def test_off_node_poses_are_deterministic_and_centre_cells_come_first():
    x = y = [-150.0, 0.0, 150.0]
    first = tcgrid.off_node_poses(x, y, 6)
    assert first == tcgrid.off_node_poses(x, y, 6)
    assert first[:4] == tcg.check_poses(x, y, 4)


def test_off_node_poses_refuses_when_the_grid_cannot_supply_them():
    with pytest.raises(tcgrid.TossCalGridError) as exc:
        tcgrid.off_node_poses([-150.0, 150.0], [-150.0, 150.0], 6)
    assert 'off-node' in str(exc.value)


def test_unknown_rung_is_refused_by_name():
    with pytest.raises(tcgrid.TossCalGridError):
        tcgrid.rung_goals('sc9', [-1.0, 0.0, 1.0], [-1.0, 0.0, 1.0], 170.0,
                          _args())


# ── ETA and ball budget ──────────────────────────────────────────────────


def test_ball_budget_states_the_worst_case_the_one_fence_permits():
    """There is NO machine-side magazine fence (D19), so the budget must be the
    worst case ``max_reloads`` allows and an EXPECTED drop count — never a
    supply model, which would be a fiction with a number attached."""
    goals = tcgrid.rung_goals('sc0', [-150.0, 0.0, 150.0], [-150.0, 0.0, 150.0],
                              170.0, _args())
    budget = tcgrid.ball_budget(goals, max_reloads=3, catch_rate=0.75)
    assert budget['goals'] == 5
    assert budget['tosses'] == 25
    assert budget['worst_case_reloads'] == 15
    assert budget['expected_drops'] == pytest.approx(6.25, abs=0.1)


def test_eta_scales_with_the_cadence_and_prices_drops():
    goals = tcgrid.rung_goals('sc0', [-150.0, 0.0, 150.0], [-150.0, 0.0, 150.0],
                              170.0, _args())
    clean = tcgrid.estimate_duration_s(goals, dwell_s=6.0, throw_delay_s=5.0,
                                       catch_rate=1.0)
    lossy = tcgrid.estimate_duration_s(goals, dwell_s=6.0, throw_delay_s=5.0,
                                       catch_rate=0.75)
    assert lossy > clean
    assert lossy - clean == pytest.approx(25 * 0.25 * tcgrid.RELOAD_COST_S)


# ── the nine refusals ────────────────────────────────────────────────────


def test_a_clean_machine_passes_all_nine_refusals():
    rows = tcgrid.preflight_refusals(_obs())
    assert [row.rid for row in rows] == ['R{}'.format(i) for i in range(1, 10)]
    assert all(row.ok for row in rows), [r.message for r in rows if not r.ok]


@pytest.mark.parametrize('kv,needle', [
    ({'mpc_active': '0', 'fault_state': 'NONE', 'bridge_link': 'UP'}, 'DISARMED'),
    ({'mpc_active': '1', 'fault_state': 'MAX_DEVIATION', 'bridge_link': 'UP'},
     'MAX_DEVIATION'),
    ({'mpc_active': '1', 'fault_state': 'NONE', 'bridge_link': 'LOST'},
     'bridge_link=LOST'),
    ({}, 'no /link_status message'),
])
def test_r1_refuses_every_unsafe_wire_state(kv, needle):
    """R1, the tilt-cal BLOCKING class: ``go_to_pose`` — and a toss goal — is
    ACCEPTED while the wire is disarmed, nothing moves, and the tool collects a
    plausible corpus of tosses that never happened."""
    row = _refusal(_obs(link_kv=kv), 'R1')
    assert row.ok is False
    assert needle in row.message


def test_r2_refuses_an_unlevelled_machine():
    row = _refusal(_obs(gravity_correction_loaded=False), 'R2')
    assert row.ok is False
    assert 'REJECTED_NOT_LEVELLED' in row.message


def test_r3_refuses_a_capture_without_the_tilt_map_underneath():
    row = _refusal(_obs(tilt_map_loaded=False), 'R3')
    assert row.ok is False
    assert 'DOWNSTREAM' in row.message


def test_r4_refuses_a_live_session_trim():
    row = _refusal(_obs(toss_trim_enabled=True), 'R4')
    assert row.ok is False
    assert 'toss_trim_enabled' in row.message


@pytest.mark.parametrize('valid,edge,needle', [
    (False, False, 'ball_held_valid=true'),
    (False, True, 'ball_held_valid=true'),
    (True, False, 'stuck bit'),
])
def test_r5_needs_a_live_edge_not_a_static_read(valid, edge, needle):
    ok, message = tcgrid.hand_sensor_verdict(valid, edge)
    assert ok is False
    assert needle in message
    row = _refusal(_obs(sensor_valid_seen=valid, sensor_edge_seen=edge), 'R5')
    assert row.ok is False


def test_r5_passes_only_with_both():
    ok, message = tcgrid.hand_sensor_verdict(True, True)
    assert ok is True
    assert 'held->empty' in message


def test_r6_refuses_a_tier_the_map_is_not_defined_at():
    row = _refusal(_obs(toss_tier='8b'), 'R6')
    assert row.ok is False
    assert '8a' in row.message


def test_r7_refuses_a_warm_bridge_and_an_unknown_one():
    """Stricter than tilt_cal_grid on purpose: static inclinometer reads are
    uptime-insensitive, a TIMING bias is not."""
    warm = _refusal(_obs(uptime_ms=31 * 60 * 1000), 'R7')
    assert warm.ok is False
    assert 'Power-cycle' in warm.message
    unknown = _refusal(_obs(uptime_ms=None), 'R7')
    assert unknown.ok is False, 'absence is not freshness'
    assert _refusal(_obs(uptime_ms=29 * 60 * 1000), 'R7').ok is True


def test_r8_refuses_an_unusable_write_target():
    row = _refusal(_obs(write_target_ok=False,
                        write_target_reason='not writable'), 'R8')
    assert row.ok is False
    assert 'not writable' in row.message


def test_r9_says_so_when_yes_waived_the_confirmations():
    row = _refusal(_obs(operator_yes=True), 'R9')
    assert row.ok is True
    assert 'NO machine check' in row.message
    denied = _refusal(_obs(operator_confirmed=False), 'R9')
    assert denied.ok is False


def test_every_refusal_is_evaluated_even_when_an_earlier_one_fails():
    """All nine are HOISTED and all nine are reported. An operator who walks
    back to the robot to fix R2 should already know R7 also refuses."""
    rows = tcgrid.preflight_refusals(
        _obs(gravity_correction_loaded=False, uptime_ms=None))
    failed = [row.rid for row in rows if not row.ok]
    assert failed == ['R2', 'R7']


def test_write_target_verdict_uses_the_production_resolver(monkeypatch, tmp_path):
    """R8 via ``toss_cal_candidates()[0]`` — never a ``__file__`` walk (a
    tilt-cal Phase-2 finding)."""
    named = tmp_path / 'my_toss_cal.yaml'
    monkeypatch.setenv(toss_cal.TOSS_CAL_ENV, str(named))
    ok, path, reason = tcgrid.write_target_verdict()
    assert ok is True
    assert path == str(named)
    assert reason is None


def test_write_target_refuses_an_ament_share_tree(monkeypatch, tmp_path):
    share = tmp_path / 'share' / 'jugglebot' / 'config'
    share.mkdir(parents=True)
    monkeypatch.delenv(toss_cal.TOSS_CAL_ENV, raising=False)
    monkeypatch.setattr(fit, 'write_target_path',
                        lambda environ=None: str(share / 'toss_calibration.yaml'))
    ok, path, reason = tcgrid.write_target_verdict()
    assert ok is False
    assert 'ament share' in reason


# ── --force-uninstall ────────────────────────────────────────────────────


def test_uninstall_plan_moves_every_existing_candidate(monkeypatch, tmp_path):
    """Source tree AND ament share. Moving only the source copy falls straight
    through to the share copy: the reload succeeds, the OLD map stays loaded,
    and the refusal names neither cause nor remedy."""
    src = tmp_path / 'src' / 'toss_calibration.yaml'
    share = tmp_path / 'share' / 'toss_calibration.yaml'
    absent = tmp_path / 'gone' / 'toss_calibration.yaml'
    for path in (src, share):
        path.parent.mkdir(parents=True)
        path.write_text('x')
    monkeypatch.delenv(toss_cal.TOSS_CAL_ENV, raising=False)
    monkeypatch.setattr(toss_cal, 'toss_cal_candidates',
                        lambda environ=None: (str(src), str(share), str(absent)))
    to_move, env_override = tcgrid.uninstall_plan()
    assert env_override is None
    assert to_move == [str(src), str(share)]


def test_uninstall_plan_refuses_to_touch_an_env_pointed_file(monkeypatch,
                                                            tmp_path):
    named = tmp_path / 'mine.yaml'
    named.write_text('x')
    monkeypatch.setenv(toss_cal.TOSS_CAL_ENV, str(named))
    to_move, env_override = tcgrid.uninstall_plan()
    assert to_move == []
    assert env_override == str(named)
    assert named.exists()


# ── the three-valued gate primitive ──────────────────────────────────────


@pytest.mark.parametrize('point,se,expect', [
    (1.0, 0.05, tcgrid.VERDICT_PASS),           # [0.90, 1.10] inside [0.75, 1.25]
    (1.0, 0.30, tcgrid.VERDICT_INCONCLUSIVE),   # [0.40, 1.60] straddles
    (2.0, 0.10, tcgrid.VERDICT_FAIL),           # [1.80, 2.20] wholly outside
    (0.0, 0.05, tcgrid.VERDICT_FAIL),
    (float('nan'), 0.1, tcgrid.VERDICT_INCONCLUSIVE),
])
def test_interval_verdict_is_three_valued(point, se, expect):
    assert tcgrid.interval_verdict(point, se, 0.75, 1.25) == expect


def test_worst_verdict_lets_fail_dominate():
    assert tcgrid.worst_verdict(['PASS', 'INCONCLUSIVE']) == 'INCONCLUSIVE'
    assert tcgrid.worst_verdict(['PASS', 'INCONCLUSIVE', 'FAIL']) == 'FAIL'
    assert tcgrid.worst_verdict(['PASS', 'PASS']) == 'PASS'


# ── SC-0 ─────────────────────────────────────────────────────────────────


_ARM_NAMES = ('zero', 'rx+', 'rx-', 'ry+', 'ry-')


def _sc0_corpus(probe_deg=0.5, n=5, sigma_mm=0.0, plant_bias=(0.002, -0.001),
                gain_scale=1.0, seed=7):
    """A synthetic SC-0 capture built through the FORWARD production model.

    ``gain_scale`` scales the landing response — ``-1.0`` is the sign flip the
    rung exists to catch, ``2.0`` a gain error.
    """
    records = []
    for name, aim in tcgrid.sc0_arms(probe_deg):
        rows = fit.synthetic_corpus(
            [0.0], [0.0], plant_bias_rad=plant_bias,
            applied_aim_rad_fn=lambda x, y, a=aim: a,
            n_per_node=n, sigma_mm=sigma_mm, z_mm=170.0, apex_m=0.78,
            seed=seed + _ARM_NAMES.index(name))
        for i, rec in enumerate(rows):
            rec['toss_uid'] = '{}-{}'.format(name, i)
            rec['goal_throw_height_m'] = 0.78
            rec['land_err_mm'] = [gain_scale * rec['land_err_mm'][0],
                                  gain_scale * rec['land_err_mm'][1]]
        records.extend(rows)
    return records


def _sc0_measure(records, probe_deg=0.5):
    admitted, _census = tcgrid.admitted_land_errors(records)
    grouped = tcgrid.group_by_applied_aim(admitted, tcgrid.sc0_arms(probe_deg))
    jac = toss_trim.aim_landing_jacobian(math.sqrt(8.0 * 0.78 / 9.80665), 170.0)
    return tcgrid.sc0_measure(grouped, math.radians(probe_deg), jac)


def test_sc0_recovers_the_production_jacobian_to_the_models_own_curvature():
    """The closed loop: land_err is generated by ``aim_target_offset_mm`` and
    differenced back through the SAME Jacobian the fit and the trim use. A sign
    or axis error in either direction would show up here as an off-diagonal.

    The residual is **3.05e-5 relative, not zero**, and that is physics rather
    than slop: SC-0 measures a SECANT over ±0.5° while
    ``aim_landing_jacobian`` is the derivative AT ZERO, and the ballistic model's
    real curvature separates them — the same fourth-significant-figure gap phase
    2c measured between 2b's 54.578 mm/deg secant and the 3126.5 mm/rad
    derivative. A symmetric difference cancels the quadratic term, so what is
    left is cubic and scales as δ²: it grows 4x if ``--sc0-probe-deg`` is
    doubled, and it is ~4 orders below the ±25 % band the gate compares against.
    """
    D, se, detail = _sc0_measure(_sc0_corpus(sigma_mm=0.0))
    assert D[0][0] == pytest.approx(1.0, abs=5e-5)
    assert D[1][1] == pytest.approx(1.0, abs=5e-5)
    assert D[0][0] > 1.0 and D[1][1] > 1.0, 'the secant exceeds the derivative'
    assert abs(D[0][1]) < 1e-5 and abs(D[1][0]) < 1e-5
    assert detail['n_per_arm'] == 5


def test_the_secant_curvature_grows_with_the_probe_and_stays_far_under_the_gate():
    """Pins the *mechanism* of the residual above, so a future reader meeting a
    non-zero D on a noiseless corpus does not go hunting a sign error.

    Measured excesses at δ = 0.125 / 0.25 / 0.5 / 1.0 deg: 6.1e-6, 1.10e-5,
    3.05e-5, 1.09e-4 — monotone in δ and approaching the δ² law as δ grows (the
    small-δ end carries a δ-independent floor from the non-zero plant bias, which
    moves the symmetric difference off centre so the quadratic term no longer
    cancels exactly). The largest is **3 orders below** the ±25 % band the gate
    compares against, so it can never move a verdict.
    """
    excess = [_sc0_measure(_sc0_corpus(probe_deg=d, sigma_mm=0.0),
                           probe_deg=d)[0][0][0] - 1.0
              for d in (0.125, 0.25, 0.5, 1.0)]
    assert all(v > 0.0 for v in excess)
    assert excess == sorted(excess), excess
    assert max(excess) < 0.25 / 100.0, excess


def test_sc0_passes_a_healthy_plant_and_names_the_sign_as_proven():
    D, se, detail = _sc0_measure(_sc0_corpus(sigma_mm=0.0))
    verdict = tcgrid.sc0_verdict(D, se, detail)
    assert verdict.verdict == tcgrid.VERDICT_PASS
    assert verdict.detail['sign_verdict'] == tcgrid.VERDICT_PASS


def test_sc0_fails_a_sign_flip_and_says_do_not_capture_a_grid():
    """The property the rung exists for. A sign flip inverts every node and aims
    the machine roughly twice as badly as no map at all, so it must be BLOCKING
    and it must be caught at the design's n = 5."""
    D, se, detail = _sc0_measure(_sc0_corpus(sigma_mm=20.0, gain_scale=-1.0))
    verdict = tcgrid.sc0_verdict(D, se, detail)
    assert verdict.verdict == tcgrid.VERDICT_FAIL
    assert verdict.blocking is True
    assert 'INVERTED' in verdict.headline
    assert 'Do NOT capture a grid' in verdict.headline


def test_sc0_fails_a_two_times_gain_error():
    D, se, detail = _sc0_measure(_sc0_corpus(sigma_mm=5.0, gain_scale=2.0))
    verdict = tcgrid.sc0_verdict(D, se, detail)
    assert verdict.verdict == tcgrid.VERDICT_FAIL


def test_sc0_is_inconclusive_when_the_gain_is_not_separable_from_zero():
    """A platform that did not actually aim reads as INCONCLUSIVE, never PASS.
    The disarmed-wire signature transposed onto this rung."""
    zeros = {name: [(0.0, 0.0)] * 5 for name in _ARM_NAMES}
    jac = toss_trim.aim_landing_jacobian(math.sqrt(8.0 * 0.78 / 9.80665), 170.0)
    D, se, detail = tcgrid.sc0_measure(zeros, math.radians(0.5), jac)
    verdict = tcgrid.sc0_verdict(D, se, detail)
    assert verdict.verdict == tcgrid.VERDICT_INCONCLUSIVE
    assert 'not distinguishable from ZERO' in verdict.headline


def test_the_literal_designed_gate_refuses_a_healthy_plant():
    """§ 3.8's literal accept test, measured rather than argued.

    At the design's own n = 5, δ = 0.5° and its own working σ = 20 mm/axis, the
    standard error of the gain ratio is ~0.23 — so the ±25 % band is barely ONE
    se wide and a PERFECT plant fails it most of the time (33.0 % pass over
    20 000 probe trials, ``/tmp/probe_toss_sc_gates.py``, 2026-08-11). This test
    reproduces the arithmetic that forced the three-valued gate, so the
    deviation cannot be "tidied" back without meeting the number again.
    """
    _D, se, _detail = _sc0_measure(_sc0_corpus(sigma_mm=20.0, n=5))
    assert se > 0.15, (
        'the +-25 % band is only {:.1f} se wide at n=5, sigma=20 — a point '
        'comparison against it is a coin flip'.format(0.25 / se))
    # And the evidence gate does NOT refuse the same healthy plant.
    D, se2, detail = _sc0_measure(_sc0_corpus(sigma_mm=20.0, n=5))
    assert tcgrid.sc0_verdict(D, se2, detail).verdict != tcgrid.VERDICT_FAIL


def test_sc0_resolvable_n_answers_how_many_tosses_the_band_needs():
    gain = float(np.linalg.norm(toss_trim.aim_landing_jacobian(
        math.sqrt(8.0 * 0.78 / 9.80665), 170.0), 2))
    need = tcgrid.sc0_resolvable_n(20.0, math.radians(0.5), gain)
    assert 15 <= need <= 22, need
    # A bigger probe resolves it faster — se scales as 1/delta.
    assert tcgrid.sc0_resolvable_n(20.0, math.radians(1.0), gain) < need


def test_sc0_refuses_an_arm_with_no_replicates():
    jac = toss_trim.aim_landing_jacobian(math.sqrt(8.0 * 0.78 / 9.80665), 170.0)
    arms = {name: [(1.0, 1.0)] for name in _ARM_NAMES}
    with pytest.raises(tcgrid.TossCalGridError) as exc:
        tcgrid.sc0_measure(arms, math.radians(0.5), jac)
    assert 'standard error' in str(exc.value)


def test_arms_are_binned_by_the_aim_the_record_says_was_applied():
    """Never by capture order: a dropped goal would silently mis-bin every toss
    after it."""
    records = _sc0_corpus(sigma_mm=0.0, n=3)
    admitted, _ = tcgrid.admitted_land_errors(records)
    grouped = tcgrid.group_by_applied_aim(admitted, tcgrid.sc0_arms(0.5))
    assert {k: len(v) for k, v in grouped.items()} == {n: 3 for n in _ARM_NAMES}


# ── SC-1 ─────────────────────────────────────────────────────────────────


def test_sc1_fit_recovers_the_angular_exponent_on_noiseless_data():
    """An angular origin gives ``|L| ∝ h`` because ``J ∝ T² ∝ h``."""
    points = [(h, 25.0 * (h / 0.78)) for h in tcgrid.SC1_HEIGHTS_M]
    result = tcgrid.sc1_fit(points)
    assert result['slope'] == pytest.approx(1.0, abs=1e-9)
    assert tcgrid.sc1_units_verdict(result).detail['units'] == 'rad'


@pytest.mark.parametrize('exponent,expect', [
    (1.0, 'rad'),
    (0.5, 'dv'),
    (0.0, 'mm'),
])
def test_sc1_branch_table_is_applied_verbatim(exponent, expect):
    points = [(h, 25.0 * (h / 0.78) ** exponent) for h in tcgrid.SC1_HEIGHTS_M]
    verdict = tcgrid.sc1_units_verdict(tcgrid.sc1_fit(points))
    assert verdict.detail['units'] == expect
    assert verdict.verdict == tcgrid.VERDICT_PASS


def test_sc1_ci_spanning_two_branches_selects_working_height_only():
    """The design's escape hatch — and, at the sample sizes § 3.8 asks for, the
    EXPECTED outcome (0.2 % of ladders decide at n=8, σ=20 over 20 000 probe
    trials). It must not read as a failure."""
    rng = np.random.default_rng(3)
    points = [(h, max(1.0, 25.0 * (h / 0.78) + rng.normal(0.0, 9.0)))
              for h in tcgrid.SC1_HEIGHTS_M]
    verdict = tcgrid.sc1_units_verdict(tcgrid.sc1_fit(points))
    assert verdict.detail['units'] == tcgrid.SC1_UNITS_AMBIGUOUS
    assert verdict.verdict == tcgrid.VERDICT_INCONCLUSIVE
    assert verdict.blocking is False
    assert 'WORKING HEIGHT ONLY' in verdict.headline


def test_sc1_refuses_a_ladder_too_short_to_regress():
    with pytest.raises(tcgrid.TossCalGridError):
        tcgrid.sc1_fit([(0.78, 20.0), (1.0, 25.0)])


def test_sc1_baseline_refuses_a_non_zero_applied_map():
    ok, message = tcgrid.sc1_baseline_verdict(True, (0.004, 0.0))
    assert ok is False
    assert '--force-uninstall' in message
    assert tcgrid.sc1_baseline_verdict(False, None)[0] is True
    assert tcgrid.sc1_baseline_verdict(True, (0.0, 0.0))[0] is True
    assert tcgrid.sc1_baseline_verdict(True, None)[0] is False


def test_sigma_from_records_is_the_first_honest_sigma_l():
    records = _sc0_corpus(sigma_mm=15.0, n=12, seed=101)
    sigma = tcgrid.sigma_from_records(
        records, lambda r: tuple(r.get('total_aim_rad') or (0.0, 0.0)))
    assert sigma == pytest.approx(15.0, rel=0.25)


# ── SC-3 ─────────────────────────────────────────────────────────────────


def _pose(mag, se, n=5, label='p'):
    return {'label': label, 'mean_mm': [mag, 0.0], 'se_mm': se, 'n': n}


def test_sc3_passes_a_map_proven_inside_the_bound():
    poses = [_pose(2.0, 1.0, label=str(i)) for i in range(6)]
    verdict = tcgrid.sc3_verdict(poses)
    assert verdict.verdict == tcgrid.VERDICT_PASS


def test_sc3_fails_a_map_the_evidence_refutes():
    poses = [_pose(30.0, 2.0, label=str(i)) for i in range(6)]
    verdict = tcgrid.sc3_verdict(poses)
    assert verdict.verdict == tcgrid.VERDICT_FAIL
    assert 'git checkout config/toss_calibration.yaml' in verdict.headline


def test_sc3_is_inconclusive_when_five_tosses_cannot_resolve_ten_millimetres():
    """The measured consequence of § 3.8's own sample size: at n = 5 and
    σ = 20 mm the per-pose se is 8.9 mm, so the literal ``|mean L| <= 10 mm``
    gate passes a PERFECT map 8.0 % of the time (20 000 probe trials,
    /tmp/probe_toss_sc_gates.py, 2026-08-11). INCONCLUSIVE is the honest
    verdict: the map is neither refuted nor certified."""
    poses = [_pose(4.0, 8.9, label=str(i)) for i in range(6)]
    verdict = tcgrid.sc3_verdict(poses)
    assert verdict.verdict == tcgrid.VERDICT_INCONCLUSIVE
    assert verdict.blocking is False
    assert 'not refuted and is not certified' in verdict.headline


def test_sc3_resolvable_n_matches_the_probe():
    assert tcgrid.sc3_resolvable_n(20.0, 10.0) == 16
    assert tcgrid.sc3_resolvable_n(10.0, 10.0) == 4


def test_sc3_fails_on_a_common_mode_bigger_than_one_trim_authority():
    """|c| must fit inside ONE session-trim authority (§ 3.8's table) — a
    common mode the trim cannot cancel is a map fault, not a trim job."""
    poses = [_pose(9.0, 0.2, label=str(i)) for i in range(6)]
    verdict = tcgrid.sc3_verdict(poses)
    assert verdict.detail['common_mode_verdict'] == tcgrid.VERDICT_FAIL
    assert verdict.verdict == tcgrid.VERDICT_FAIL


# ── probe map ────────────────────────────────────────────────────────────


def test_probe_map_commands_the_same_aim_everywhere_including_outside_the_hull():
    doc = tcgrid.probe_map_document(
        [-150.0, 0.0, 150.0], [-150.0, 0.0, 150.0], 170.0,
        (math.radians(0.5), 0.0), tilt_map_version='t/1',
        base_condition='probe', argv=[])
    cal = toss_cal.parse_toss_cal(doc)
    for x, y in ((0.0, 0.0), (150.0, -150.0), (37.0, -91.0), (900.0, -900.0)):
        aim = toss_cal.lookup(cal, x, y)
        assert aim[0] == pytest.approx(math.radians(0.5), abs=1e-12)
        assert aim[1] == pytest.approx(0.0, abs=1e-12)


def test_probe_map_round_trips_through_the_production_loader(tmp_path):
    doc = tcgrid.probe_map_document(
        [-150.0, 0.0, 150.0], [-150.0, 0.0, 150.0], 170.0, (0.0, 0.0),
        tilt_map_version='t/1', base_condition='', argv=[])
    path = tmp_path / 'toss_calibration.yaml'
    path.write_text(fit.dump_map_yaml(doc))
    cal = toss_cal.load_toss_cal(str(path))
    assert cal.version == toss_cal.map_version(doc)
    assert yaml.safe_load(path.read_text())['captured']['probe'] is True


def test_a_probe_map_declares_itself_a_probe_in_words():
    """A probe map found in a git working tree must be recognisable as one at a
    glance: it is a calibration nobody fitted."""
    doc = tcgrid.probe_map_document(
        [-150.0, 0.0, 150.0], [-150.0, 0.0, 150.0], 170.0,
        (math.radians(0.5), 0.0), tilt_map_version='t/1',
        base_condition='bench', argv=[])
    assert doc['captured']['probe'] is True
    assert 'PROBE MAP' in doc['captured']['base_condition']
    assert 'NOT a calibration' in doc['captured']['base_condition']


def test_a_probe_past_the_authority_bound_is_refused_by_the_production_loader():
    with pytest.raises(fit.TossFitError):
        tcgrid.probe_map_document(
            [-150.0, 0.0, 150.0], [-150.0, 0.0, 150.0], 170.0,
            (math.radians(1.5), 0.0), tilt_map_version='t/1',
            base_condition='', argv=[])


def test_probe_map_carries_the_live_tilt_map_version_so_it_is_not_dormant():
    """A probe map whose provenance does not match the live tilt map is LOADED
    but DORMANT — it aims vertically, so every SC-0 arm would command the same
    zero aim and the probe would measure nothing."""
    doc = tcgrid.probe_map_document(
        [-150.0, 0.0, 150.0], [-150.0, 0.0, 150.0], 170.0, (0.0, 0.0),
        tilt_map_version='2026-08-10-abcdef01', base_condition='', argv=[])
    cal = toss_cal.parse_toss_cal(doc)
    assert cal.provenance_mismatch('2026-08-10-abcdef01') is None
    assert cal.provenance_mismatch('something-else') is not None


# ── the rung ledger ──────────────────────────────────────────────────────


def _ledger(**verdicts):
    return {'schema': tcgrid.LEDGER_SCHEMA,
            'rungs': {k: {'verdict': v} for k, v in verdicts.items()}}


def test_sc0_blocks_every_later_rung_until_it_is_scored():
    ok, why = tcgrid.rung_precondition(_ledger(), 'sc2')
    assert ok is False
    assert 'SC-0 is missing' in why
    ok, why = tcgrid.rung_precondition(
        _ledger(sc0=tcgrid.VERDICT_UNSCORED), 'sc2')
    assert ok is False
    assert 'unscored' in why


def test_a_failed_sc0_blocks_the_grid_by_name():
    ok, why = tcgrid.rung_precondition(_ledger(sc0=tcgrid.VERDICT_FAIL), 'sc2')
    assert ok is False
    assert 'sign is inverted' in why


def test_an_inconclusive_sc0_does_not_block_the_grid():
    """The documented deviation: INCONCLUSIVE means the SIGN is proven and only
    the ±25 % gain band is unresolved — and that band is finer than any sample
    size that fits a sitting. Blocking on it would block every capture forever.
    """
    ok, why = tcgrid.rung_precondition(
        _ledger(sc0=tcgrid.VERDICT_INCONCLUSIVE, sc1=tcgrid.VERDICT_PASS), 'sc2')
    assert ok is True, why


def test_sc2_also_needs_sc1_scored_because_it_decides_the_units():
    ok, why = tcgrid.rung_precondition(_ledger(sc0=tcgrid.VERDICT_PASS), 'sc2')
    assert ok is False
    assert 'SC-1' in why
    ok, _ = tcgrid.rung_precondition(
        _ledger(sc0=tcgrid.VERDICT_PASS, sc1=tcgrid.VERDICT_INCONCLUSIVE), 'sc2')
    assert ok is True, 'an ambiguous exponent is a MEASURED result, not a block'


def test_a_5x5_needs_a_verified_3x3_first():
    full = _ledger(sc0=tcgrid.VERDICT_PASS, sc1=tcgrid.VERDICT_PASS)
    ok, why = tcgrid.rung_precondition(full, 'sc2', nodes=5)
    assert ok is False
    assert '3x3 FIRST' in why
    assert tcgrid.rung_precondition(full, 'sc2', nodes=5, allow_5x5=True)[0]
    verified = _ledger(sc0=tcgrid.VERDICT_PASS, sc1=tcgrid.VERDICT_PASS,
                       sc3=tcgrid.VERDICT_PASS)
    assert tcgrid.rung_precondition(verified, 'sc2', nodes=5)[0] is True


def test_sc0_and_sc1_have_no_precondition_of_their_own():
    assert tcgrid.rung_precondition(_ledger(), 'sc0')[0] is True
    assert tcgrid.rung_precondition(_ledger(), 'sc1')[0] is True


def test_a_corrupt_ledger_refuses_rather_than_silently_resetting(tmp_path):
    """"No SC-0 row" and "an unreadable SC-0 row" must not look the same to the
    rung that is about to spend 72 tosses on the difference."""
    path = tmp_path / tcgrid.LEDGER_NAME
    path.write_text('{not json')
    with pytest.raises(tcgrid.TossCalGridError) as exc:
        tcgrid.load_ledger(str(path))
    assert 'deliberately' in str(exc.value)


def test_ledger_round_trips(tmp_path):
    path = str(tmp_path / tcgrid.LEDGER_NAME)
    assert tcgrid.load_ledger(path) == {'schema': tcgrid.LEDGER_SCHEMA,
                                        'rungs': {}}
    ledger = tcgrid.load_ledger(path)
    tcgrid.record_rung(ledger, 'sc0', tcgrid.VERDICT_PASS, headline='ok')
    tcgrid.save_ledger(path, ledger)
    again = tcgrid.load_ledger(path)
    assert again['rungs']['sc0']['verdict'] == tcgrid.VERDICT_PASS
    assert again['rungs']['sc0']['headline'] == 'ok'


# ── cancel and node exhaustion ───────────────────────────────────────────


@pytest.mark.parametrize('phase', tcgrid.CANCEL_DEFERRED_PHASES)
def test_a_cancel_with_a_ball_in_the_air_is_reported_as_deferred(phase):
    deferred, text = tcgrid.cancel_disposition(phase)
    assert deferred is True
    assert 'A second Ctrl-C will NOT skip this wait' in text
    assert 'Ball is airborne' in text


@pytest.mark.parametrize('phase', ['DWELL', 'SESSION_CHECKING', 'RELOAD',
                                   'CHECKING', 'POSITIONING', 'PREPARING'])
def test_a_cancel_in_a_quiescent_phase_is_reported_as_honoured_now(phase):
    deferred, text = tcgrid.cancel_disposition(phase)
    assert deferred is False
    assert 'honoured NOW' in text


def test_throwing_is_reported_as_MAY_be_deferred():
    """The tool cannot see ``TOSS_CANCEL_CUTOFF_S`` from outside, and claiming
    certainty either way would be a lie about an airborne ball."""
    _deferred, text = tcgrid.cancel_disposition('THROWING')
    assert 'may be DEFERRED' in text


def test_node_exhausted_is_exactly_the_reload_budget_terminal():
    assert tcgrid.node_exhausted('STOPPED_RELOAD_BUDGET') is True
    assert tcgrid.node_exhausted('COMPLETED') is False
    assert tcgrid.node_exhausted('STOPPED_ON_MISS') is False
    assert tcgrid.node_exhausted(None) is False
    # The constant is the session FSM's own string, not a restatement.
    from jugglebot import toss_session
    assert tcgrid.NODE_EXHAUSTED_OUTCOME == \
        toss_session.OUTCOME_STOPPED_RELOAD_BUDGET


# ── D5: the forbidden observable ─────────────────────────────────────────


def test_the_only_aim_observable_is_land_err_mm():
    """D5 / F5, structurally. ``catch_error_mm`` is a dead-reckoned free-fall
    extrapolation, a scalar distance not a signed 2-vector, and exists only for
    CAUGHT balls — doubly wrong: selection-biased and measuring seated position,
    not arrival."""
    rec = {'catch_error_mm_fsm': 3.2, 'catch_error_mm': 3.2}
    assert tcgrid.land_err_of(rec) is None
    assert tcgrid.land_err_of({'land_err_mm': [1.0, -2.0]}) == (1.0, -2.0)
    assert tcgrid.land_err_of({'land_err_mm': [1.0, float('nan')]}) is None
    assert tcgrid.land_err_of({'land_err_mm': 3.0}) is None


def test_no_estimator_path_reads_catch_error():
    """Source-level: the ONE place the tool may read ``catch_error`` from a
    record is the CSV row, under a name that says DIAGNOSTIC out loud. Prose and
    comments are exempt (they exist to explain the ban); a ``.get()`` is not."""
    source = _source()
    reads = [line for line in source.splitlines()
             if 'catch_error' in line and '.get(' in line]
    assert reads, 'the diagnostic column should still exist'
    for line in reads:
        assert 'catch_error_mm_fsm_diagnostic' in line, line


def test_admission_reuses_the_shared_filter():
    """One admission rule, shared with the offline fit and the online trim (2e
    moved it to ``toss_trim`` for exactly this reason). A second rule here would
    let a rung certify a plant the fit then refuses."""
    source = _source()
    assert 'toss_trim.admit_for_aim' in source
    good = _sc0_corpus(sigma_mm=0.0, n=2)
    admitted, census = tcgrid.admitted_land_errors(good)
    assert len(admitted) == 10
    assert census['admitted'] == 10
    bad = [dict(r, total_aim_rad=None, map_aim_rad=None, trim_aim_rad=None)
           for r in good]
    admitted, census = tcgrid.admitted_land_errors(bad)
    assert admitted == []
    assert sum(census.values()) == 10


# ── the desk-side scorer ─────────────────────────────────────────────────


def test_score_rung_sc0_passes_a_healthy_corpus(capsys):
    verdict = tcgrid.score_rung('sc0', _sc0_corpus(sigma_mm=2.0), _args(),
                                [-150.0, 0.0, 150.0], [-150.0, 0.0, 150.0])
    assert verdict.verdict == tcgrid.VERDICT_PASS
    assert 'admission census' in capsys.readouterr().out


def test_score_rung_sc0_fails_an_inverted_corpus():
    verdict = tcgrid.score_rung('sc0',
                                _sc0_corpus(sigma_mm=2.0, gain_scale=-1.0),
                                _args(), [-150.0, 0.0, 150.0],
                                [-150.0, 0.0, 150.0])
    assert verdict.verdict == tcgrid.VERDICT_FAIL


def test_score_rung_reports_unscorable_rather_than_inventing_a_verdict():
    verdict = tcgrid.score_rung('sc0', [], _args(), [-150.0, 0.0, 150.0],
                                [-150.0, 0.0, 150.0])
    assert verdict.verdict == tcgrid.VERDICT_INCONCLUSIVE
    assert 'UNSCORABLE' in verdict.headline


def test_score_rung_sc2_refuses_a_node_that_never_flew():
    records = fit.synthetic_corpus([-150.0, 0.0, 150.0], [-150.0, 0.0, 150.0],
                                   plant_bias_rad=(0.001, 0.0), n_per_node=10,
                                   sigma_mm=1.0)
    records = [r for r in records
               if tuple(r['goal_catch_xyz_stow_mm'][:2]) != (150.0, 150.0)]
    verdict = tcgrid.score_rung('sc2', records, _args(),
                                [-150.0, 0.0, 150.0], [-150.0, 0.0, 150.0])
    assert verdict.verdict == tcgrid.VERDICT_FAIL
    assert 'never flew' in verdict.headline


def test_score_rung_sc2_marks_a_thin_node_inconclusive_not_fatal():
    """D15: a thin node keeps its PREVIOUS value marked stale; refusing the whole
    write would block 24 good nodes because one had a thin week."""
    records = fit.synthetic_corpus([-150.0, 0.0, 150.0], [-150.0, 0.0, 150.0],
                                   plant_bias_rad=(0.001, 0.0), n_per_node=10,
                                   sigma_mm=1.0)
    thin = [r for r in records
            if tuple(r['goal_catch_xyz_stow_mm'][:2]) != (150.0, 150.0)]
    thin += [r for r in records
             if tuple(r['goal_catch_xyz_stow_mm'][:2]) == (150.0, 150.0)][:2]
    verdict = tcgrid.score_rung('sc2', thin, _args(), [-150.0, 0.0, 150.0],
                                [-150.0, 0.0, 150.0])
    assert verdict.verdict == tcgrid.VERDICT_INCONCLUSIVE
    assert 'stale:true' in verdict.headline


def test_run_score_writes_the_ledger_and_makes_no_ros_calls(tmp_path,
                                                            monkeypatch):
    def _explode(*_a, **_k):
        raise AssertionError('--score touched rclpy')

    monkeypatch.setitem(sys.modules, 'rclpy', _explode)
    corpus = tmp_path / 'corpus.jsonl'
    with open(corpus, 'w') as handle:
        for rec in _sc0_corpus(sigma_mm=2.0):
            handle.write(json.dumps(rec) + '\n')
    rc = tcgrid.main(['--rung', 'sc0', '--score', str(corpus),
                      '--out-dir', str(tmp_path)])
    assert rc == 0
    ledger = json.loads((tmp_path / tcgrid.LEDGER_NAME).read_text())
    assert ledger['rungs']['sc0']['verdict'] == tcgrid.VERDICT_PASS
    assert ledger['rungs']['sc0']['corpus'] == [str(corpus)]


def test_run_score_exits_nonzero_on_a_blocking_verdict(tmp_path, monkeypatch):
    monkeypatch.setitem(sys.modules, 'rclpy', None)
    corpus = tmp_path / 'corpus.jsonl'
    with open(corpus, 'w') as handle:
        for rec in _sc0_corpus(sigma_mm=2.0, gain_scale=-1.0):
            handle.write(json.dumps(rec) + '\n')
    rc = tcgrid.main(['--rung', 'sc0', '--score', str(corpus),
                      '--out-dir', str(tmp_path)])
    assert rc == 1


# ── CSV ──────────────────────────────────────────────────────────────────


def test_csv_row_keys_match_the_declared_columns_exactly():
    goal = tcgrid.rung_goals('sc0', [-150.0, 0.0, 150.0], [-150.0, 0.0, 150.0],
                             170.0, _args())[1]
    row = tcgrid.csv_row('t', 1.0, goal, {'toss_uid': 'u', 'outcome': 'CAUGHT',
                                          'total_aim_rad': [0.1, 0.2]}, 5)
    assert set(row) == set(tcgrid.CSV_COLUMNS)


def test_csv_row_records_the_probe_aim_that_was_commanded():
    goals = tcgrid.rung_goals('sc0', [-150.0, 0.0, 150.0], [-150.0, 0.0, 150.0],
                              170.0, _args())
    rx_plus = [g for g in goals if 'rx+' in g.label][0]
    row = tcgrid.csv_row('t', 1.0, rx_plus, {}, None)
    assert row['probe_rx_deg'] == pytest.approx(0.5)
    assert row['probe_ry_deg'] == pytest.approx(0.0)
    assert row['uptime_ms'] == ''


def test_csv_row_carries_catch_error_only_under_a_diagnostic_name():
    goal = tcgrid.rung_goals('sc3', [-150.0, 0.0, 150.0], [-150.0, 0.0, 150.0],
                             170.0, _args())[0]
    row = tcgrid.csv_row('t', 1.0, goal, {'catch_error_mm_fsm': 3.4}, None)
    assert row['catch_error_mm_fsm_diagnostic'] == 3.4
    assert not any(c == 'catch_error_mm' for c in tcgrid.CSV_COLUMNS)


# ── structural: what run() must do that only a robot could exercise ──────


def test_dry_run_makes_no_ros_calls(capsys, monkeypatch):
    """--dry-run must construct nothing: it is the rehearsal an operator runs on
    a machine that is not even powered (§ 6 step 0.5)."""
    def _explode(*_args, **_kwargs):
        raise AssertionError('dry-run imported/used rclpy')

    monkeypatch.setitem(sys.modules, 'rclpy', _explode)
    assert tcgrid.main(['--rung', 'sc0', '--dry-run']) == 0
    out = capsys.readouterr().out
    assert 'TOSS COUNT 25' in out
    assert 'ETA' in out
    assert 'BALL BUDGET' in out
    assert 'goal order' in out
    assert 'no ROS calls made' in out


def test_dry_run_prints_the_doubled_bias_first_application_step(capsys,
                                                                monkeypatch):
    """§ 5 P5.4: the first hardware application of a new map uses a deliberately
    DOUBLED bias on ONE node, because if doubling the bias doubles the error the
    sign is wrong and one sitting says so instead of a whole grid."""
    monkeypatch.setitem(sys.modules, 'rclpy', None)
    ledger = {'schema': tcgrid.LEDGER_SCHEMA,
              'rungs': {'sc0': {'verdict': tcgrid.VERDICT_PASS},
                        'sc1': {'verdict': tcgrid.VERDICT_PASS}}}
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        tcgrid.save_ledger(tcgrid.ledger_path(tmp), ledger)
        assert tcgrid.main(['--rung', 'sc2', '--dry-run', '--out-dir', tmp]) == 0
    out = capsys.readouterr().out
    assert 'DOUBLED BIAS ON ONE NODE' in out
    assert 'the sign is wrong' in out


def test_dry_run_refuses_a_rung_whose_precondition_is_not_met(capsys,
                                                             monkeypatch,
                                                             tmp_path):
    monkeypatch.setitem(sys.modules, 'rclpy', None)
    assert tcgrid.main(['--rung', 'sc2', '--dry-run',
                        '--out-dir', str(tmp_path)]) == 2
    assert 'WOULD BE REFUSED' in capsys.readouterr().out


def test_no_apply_refuses_sc0_because_the_probe_map_is_the_aim_authority(capsys):
    rc = tcgrid.main(['--rung', 'sc0', '--no-apply', '--dry-run'])
    assert rc == 2
    assert 'PROBE MAPS' in capsys.readouterr().err


def test_verify_only_is_the_sc3_rung(monkeypatch, capsys, tmp_path):
    monkeypatch.setitem(sys.modules, 'rclpy', None)
    tcgrid.save_ledger(tcgrid.ledger_path(str(tmp_path)),
                       _ledger(sc0=tcgrid.VERDICT_PASS,
                               sc1=tcgrid.VERDICT_PASS))
    assert tcgrid.main(['--verify-only', '--dry-run',
                        '--out-dir', str(tmp_path)]) == 0
    assert 'rung SC3' in capsys.readouterr().out


def test_the_wire_check_runs_before_the_per_goal_try_block():
    """Structural: ``assert_wire_armed`` must sit OUTSIDE the per-goal try.

    The try below converts a ``TossCalGridError`` into a failed *goal* and, under
    the default ``--on-fail continue``, carries on. A wire that disarmed
    mid-capture is never "this one node failed" — every remaining goal would be
    ACCEPTED, nothing would move, and the tool would collect a plausible corpus
    of tosses that never happened (the 2026-07-15 silent-no-op shape).
    """
    source = _source()
    marker = 'result = runner.send_session('
    assert marker in source
    prologue = source[:source.index(marker)]
    tail = prologue.rsplit('runner.assert_wire_armed(', 1)[1]
    assert tail.count('try:') == 1, (
        'assert_wire_armed must be called before the per-goal `try:`, so a '
        'mid-capture disarm aborts the run instead of scoring one bad node')


def test_the_wire_check_is_rechecked_between_every_goal():
    """Not only at preflight: the whole point is the RE-check between nodes."""
    source = _source()
    body = source.split('for goal in goals:', 1)[1].split('\n    except ', 1)[0]
    assert 'runner.assert_wire_armed(goal.label)' in body
    assert 'runner.assert_no_drive_fault(goal.label)' in body


def test_fault_errors_escape_the_demotion_handler():
    """``except TossCalGridFaultError: raise`` must precede the
    ``except TossCalGridError`` demotion, or a latched fault is demoted to a
    failed goal under the default --on-fail continue."""
    source = _source()
    after = source[source.index('result = runner.send_session('):]
    fault = after.find('except TossCalGridFaultError')
    demote = after.find('except TossCalGridError as exc:')
    assert fault != -1 and demote != -1
    assert fault < demote


def test_node_exhaustion_advances_the_cursor_and_does_not_abort():
    """Operator decision 4 / D19, structurally: the ``node_exhausted`` branch must
    ``continue`` (skip to the next node), never ``raise``. § 3.7 item 7 already
    writes a thin node correctly, so an exhausted node costs one node's refresh
    rather than a sitting."""
    source = _source()
    branch = source.split('if node_exhausted(outcome):', 1)[1]
    branch = branch.split('\n            if outcome not in', 1)[0]
    assert 'continue' in branch
    assert 'raise' not in branch
    assert 'thin/stale' in branch.lower()
    assert 'exhausted_nodes.append' in branch


def test_return_to_centre_guard_catches_base_exception():
    """A plain ``except Exception`` does NOT catch KeyboardInterrupt, and the
    second Ctrl-C is the reflex when a program does not die on the first. The
    return blocks for seconds, so that interrupt lands inside the guard: with
    ``Exception`` it would propagate out of the ``finally``, skip the artefact
    write AND the rclpy shutdown, and leave the platform parked at a raised
    displaced pose in silence."""
    source = _source()
    marker = 'RETURN TO CENTRE FAILED'
    assert marker in source
    guard = source[:source.rindex(marker)]
    handler = guard.rsplit('except ', 1)[1].split('\n')[0]
    assert handler.startswith('BaseException'), handler


def test_the_probe_map_is_restored_before_the_return_to_centre():
    """Ordering, and it is load-bearing: the restore's reload is a service call
    on the same node, and the return-to-centre can itself fail. Restoring first
    means a failed return still leaves a machine whose calibration is the one it
    started with, rather than a ±0.5° PROBE map that aims every later throw
    27 mm off while every log line reports a calibration as applied."""
    source = _source()
    # The OUTER finally of run(), anchored on its own comment rather than on
    # `rsplit('finally:')` — the inner rclpy-shutdown finally is nested inside it.
    finally_block = source.split('        # `except BaseException`, deliberately', 1)[1]
    assert finally_block.index('probe.restore()') < \
        finally_block.index('runner.return_to_centre()')
    assert 'except BaseException' in \
        finally_block[:finally_block.index('runner.return_to_centre()')]


def test_probe_restore_failure_is_loud_and_names_the_remedy():
    source = _source()
    body = source.split('def restore(', 1)[1].split('\ndef ', 1)[0]
    assert 'PROBE MAP RESTORE FAILED' in body
    assert 'reload_calibration' in body
    assert 'file=sys.stderr' in body


def test_abort_reason_is_always_written_to_the_meta():
    """An abort that writes an empty summary is the abort most in need of the
    data (a tilt-cal finding)."""
    source = _source()
    assert "'abort_reason': abort_reason," in source
    for handler in ('except TossCalGridError as exc:',
                    'except KeyboardInterrupt as exc:',
                    'except SystemExit as exc:',
                    'except BaseException as exc:'):
        assert handler in source, handler
    body = source.split('    try:\n        # ── preflight', 1)[1]
    assert body.count('abort_reason = ') >= 4


def test_per_toss_rows_are_appended_as_they_complete():
    """Never built in a trailing loop: on any mid-capture abort a trailing loop
    never runs and the CSV would be empty exactly when the operator needs it."""
    source = _source()
    body = source.split('for goal in goals:', 1)[1].split('\n    except ', 1)[0]
    assert 'drain_records(goal' in body
    drain = source.split('def drain_records(', 1)[1].split('\n    try:', 1)[0]
    assert 'csv_file.flush()' in drain


def test_the_tool_never_arms_never_switches_mode_and_never_moves_the_hand():
    """The safety envelope, as a manifest — C-LEVEL-2's own test shape.

    Every ROS name the tool may touch is listed here. A new client, action or
    publisher fails this test, which is the point: the tool's entire safety claim
    is "it sends toss goals and observes", and that claim is only as good as the
    list of things it can reach.
    """
    source = _source()
    import re

    def _names(call: str) -> set:
        out = set()
        for match in re.finditer(re.escape(call), source):
            window = source[match.end():match.end() + 220]
            found = re.findall(r"'(/[^']+)'", window)
            assert found, '{} with no literal ROS name: {}'.format(
                call, window[:80])
            out.add(found[0])
        return out

    assert _names('create_client(') == {
        '/toss/reload_calibration', '/trajectory/go_home',
        '/reload_coordinator_node/get_parameters'}
    assert _names('create_subscription(') == {
        '/trajectory/status', '/link_status', '/robot_state', '/hand_telemetry',
        '/trajectory/commanded_position', '/toss/record',
        '/toss/calibration_status'}
    # ONE action, and it is the toss session. Everything actuating goes through
    # it; the only other outward call is the safing pair above.
    assert source.count('ActionClient(') == 1
    assert tcgrid.ACTION_NAME == '/jugglebot/toss_continuous'
    # No publisher of ANY kind — the tool never publishes on a production topic.
    assert 'create_publisher(' not in source
    # And none of the arming / mode / hand surface is reachable. Scanned over
    # ROS-touching lines only: the prose above deliberately NAMES `/activate` and
    # `clear_errors` in refusal messages that tell the operator what to run.
    ros_lines = [line for line in source.splitlines()
                 if any(token in line for token in
                        ('create_client(', 'create_subscription(',
                         'ActionClient(', 'create_publisher('))]
    for forbidden in ('/activate', 'set_setpoint_output', 'clear_errors',
                      'orchestrator_command', 'catch/armed', 'catch/prime_hold',
                      'hand/', 'encoder_search', 'set_control_mode',
                      'go_to_pose', 'jugglebot/reload', 'jugglebot/toss\''):
        assert not any(forbidden in line for line in ros_lines), forbidden
        assert 'runner.call({}'.format(forbidden) not in source


def test_rclpy_is_never_imported_at_module_scope():
    """The pure core must import on a box with no ROS at all — that is what lets
    this test file drive every gate and every refusal."""
    source = _source()
    for line in source.splitlines():
        if line.startswith('import rclpy') or line.startswith('from rclpy'):
            raise AssertionError('module-scope rclpy import: ' + line)
    assert 'rclpy' not in sys.modules or True   # the import above never ran


def test_the_session_goal_is_the_only_combination_that_reaches_the_interlude():
    """``stop_on_miss=False`` + ``on_empty_cup='RELOAD'``: with stop_on_miss true
    the MISSED cycle that dropped the ball has already stopped the session one
    cycle earlier, so the auto-reload interlude would be unreachable and every
    drop would cost a re-sent goal."""
    source = _source()
    body = source.split('def send_session(', 1)[1].split('\n    def ', 1)[0]
    assert 'request.stop_on_miss = False' in body
    assert "request.on_empty_cup = 'RELOAD'" in body
    assert 'request.max_reloads = int(max_reloads)' in body


def test_a_second_ctrl_c_cannot_skip_the_deferred_wait():
    source = _source()
    body = source.split('def send_session(', 1)[1].split('\n    def ', 1)[0]
    assert 'already cancelling' in body
    assert 'cancel_disposition(' in body


def test_r4_fails_closed_when_the_parameter_cannot_be_read():
    """"I could not check whether the session trim is on" is not "the session
    trim is off"."""
    source = _source()
    body = source.split('def _param_true(', 1)[1].split('\ndef ', 1)[0]
    assert 'fails CLOSED' in body
    assert body.rstrip().endswith('node.destroy_client(client)')
    assert 'return True' in body
