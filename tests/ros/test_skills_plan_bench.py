"""Offline tests for ``tests/hardware/skills_plan_bench.py`` — the skill-stack
R2 gate driver (plan § 4 R2: per-skill plan time of
``trajectory/install_segment`` on the loaded Jetson, launch UP, robot never
moving).

Scope, and why it stops where it does. Same split as every other rclpy tool in
``tests/hardware/``: a PURE CORE (schedule/verdict/precondition arithmetic, the
request-field mirror, the synthetic tracker) that imports no ROS, and a thin
live half that imports ``rclpy`` and ``jugglebot_interfaces`` lazily. Only the
pure core is tested here — the live half is what a hardware sitting exercises.

Lives under ``tests/ros/`` because the driver's subject is a ROS service
contract (``InstallSegment.srv``) and the mocked-ROS conftest here is what
makes ``jugglebot_interfaces`` importable for the field/enum pins.
"""

from __future__ import annotations

import ast
import os
import re
import sys
import types

import numpy as np
import pytest

_TESTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO = os.path.dirname(_TESTS)
_HW_DIR = os.path.join(_TESTS, 'hardware')
if _HW_DIR not in sys.path:
    sys.path.insert(0, _HW_DIR)

import skills_plan_bench as spb  # noqa: E402

from jugglebot.motion.geometry import StewartGeometry  # noqa: E402
from jugglebot.motion.skills import executor as ex  # noqa: E402
from jugglebot.motion.skills import schedule as sc  # noqa: E402
from jugglebot.motion.skills import segments as sg  # noqa: E402
from jugglebot.motion.skills import sites as si  # noqa: E402
from jugglebot.motion.trajectory.limits import TrajectoryLimits  # noqa: E402


_SRV = os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot_interfaces', 'srv',
                    'InstallSegment.srv')


def _srv_kind_enums():
    out = {}
    with open(_SRV) as f:
        for line in f:
            m = re.match(r'\s*uint8\s+(KIND_\w+)\s*=\s*(\d+)', line)
            if m:
                out[m.group(1)] = int(m.group(2))
    return out


# ═════════════════════════════════════════════════════════════════════════════
# Drift pins
# ═════════════════════════════════════════════════════════════════════════════

def test_the_pure_core_imports_no_ros_at_module_scope():
    """``rclpy`` and ``jugglebot_interfaces`` are imported INSIDE functions/
    methods only — an AST check, not a source-text one, so ``--dry-run`` and
    ``--rehearse`` are a true rehearsal on a box with a stale colcon build."""
    src = open(os.path.join(_HW_DIR, 'skills_plan_bench.py')).read()
    tree = ast.parse(src)
    banned = ('rclpy', 'jugglebot_interfaces', 'diagnostic_msgs', 'std_msgs',
              'geometry_msgs', 'std_srvs')
    for node in tree.body:                      # module scope ONLY
        names = []
        if isinstance(node, ast.Import):
            names = [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            names = [node.module or '']
        for n in names:
            assert not any(n == b or n.startswith(b + '.') for b in banned), (
                '%s is imported at module scope' % (n,))


def test_the_kind_enum_matches_the_srv():
    """The mirrored ``KIND_*`` ints ARE ``InstallSegment.srv``'s.

    Mirrored (not imported) so ``--dry-run``/``--rehearse`` work without a
    colcon build; a silently renumbered kind would install a THROW where a
    CATCH was meant, on real hardware.
    """
    e = _srv_kind_enums()
    assert e == {'KIND_THROW': 0, 'KIND_CATCH': 1, 'KIND_REST': 2}
    assert (spb.KIND_THROW, spb.KIND_CATCH, spb.KIND_REST) == (
        e['KIND_THROW'], e['KIND_CATCH'], e['KIND_REST'])
    assert spb._WIRE_KIND == {sg.THROW: e['KIND_THROW'], sg.CATCH: e['KIND_CATCH'],
                              sg.REST: e['KIND_REST']}


def test_the_splice_budgets_equal_the_schedule_constant_arithmetic():
    """200.0 ms / 150.0 ms, derived from ``schedule``'s own knot budget — never
    a restated literal.

    Was 125 / 75 ms until 2026-09-18, when the budget was re-sized on the
    LOADED robot (``schedule.SOLVE_BUDGET_KNOTS``: CATCH solves to 134.2 ms
    max, and 16 of 23 catch attempts had refused ``SPLICE_TOO_LATE``).  The
    literals stay pinned here so the bench's printed gate can never drift from
    the schedule silently, and so re-sizing the budget is a deliberate edit."""
    dt = float(spb.hw.JB_TRAJ_KNOT_DT_S)
    assert dt == pytest.approx(0.025)
    assert spb.G2_HANDOFF_BUDGET_MS == pytest.approx(
        (sc.HANDOFF_LEAD_KNOTS - sc.WIRE_READ_KNOTS) * dt * 1e3)
    assert spb.G2_HANDOFF_BUDGET_MS == pytest.approx(200.0)
    assert spb.G2_UNPINNED_BUDGET_MS == pytest.approx(
        (sc.LEAD_KNOTS - sc.WIRE_READ_KNOTS) * dt * 1e3)
    assert spb.G2_UNPINNED_BUDGET_MS == pytest.approx(150.0)


# ═════════════════════════════════════════════════════════════════════════════
# The request builder
# ═════════════════════════════════════════════════════════════════════════════

_SITE = si.Site('P1', np.array([-50.0, 0.0, 830.0]))


def test_the_request_builder_names_only_real_srv_fields():
    from jugglebot_interfaces.srv import InstallSegment
    req = InstallSegment.Request()

    throw_t = sg.ThrowTerminal(site_mm=[1.0, 2.0, 860.0],
                               target_mm=[1.0, 2.0, 860.0],
                               flight_s=0.857, t_release_s=1.0)
    catch_t = sg.CatchTerminal(landing_mm=[1.0, 2.0, 830.0],
                               landing_vel_mm_s=[0.0, 0.0, -4200.0],
                               t_land_s=1.5, rest_site_mm=[1.0, 2.0, 689.6])
    rest_t = sg.RestTerminal(rest_site_mm=[1.0, 2.0, 689.6], t_rest_s=2.0)
    then_throw = sg.ThrowAfterCatch(t_release_s=1.8, site_mm=[1.0, 2.0, 860.0],
                                    target_mm=[1.0, 2.0, 860.0], flight_s=0.857)
    catch_throw_t = sg.CatchTerminal(landing_mm=[1.0, 2.0, 830.0],
                                     landing_vel_mm_s=[0.0, 0.0, -4200.0],
                                     t_land_s=1.5, rest_site_mm=[1.0, 2.0, 689.6],
                                     then_throw=then_throw)

    for kind, terminal in ((sg.THROW, throw_t), (sg.CATCH, catch_t),
                          (sg.REST, rest_t), (sg.CATCH, catch_throw_t)):
        fields = spb.install_request_fields(kind, terminal)
        for key in fields:
            assert hasattr(req, key), (
                '%r is not an InstallSegment.Request field' % (key,))


def test_a_catch_with_throw_fills_the_release_fields():
    then_throw = sg.ThrowAfterCatch(t_release_s=1.8, site_mm=[1.0, 2.0, 860.0],
                                    target_mm=[3.0, 4.0, 860.0], flight_s=0.857)
    catch_throw_t = sg.CatchTerminal(landing_mm=[1.0, 2.0, 830.0],
                                     landing_vel_mm_s=[0.0, 0.0, -4200.0],
                                     t_land_s=1.5, rest_site_mm=[1.0, 2.0, 689.6],
                                     then_throw=then_throw)
    fields = spb.install_request_fields(sg.CATCH, catch_throw_t)
    assert fields['t_event_s'] == pytest.approx(1.5)
    assert fields['t_release_s'] == pytest.approx(1.8)
    assert fields['release_site_mm'] == pytest.approx([1.0, 2.0, 860.0])
    assert fields['target_mm'] == pytest.approx([3.0, 4.0, 860.0])
    assert fields['flight_s'] == pytest.approx(0.857)
    # A standalone catch (no then_throw) carries none of the release fields.
    plain_t = sg.CatchTerminal(landing_mm=[1.0, 2.0, 830.0],
                               landing_vel_mm_s=[0.0, 0.0, -4200.0],
                               t_land_s=1.5, rest_site_mm=[1.0, 2.0, 689.6])
    plain_fields = spb.install_request_fields(sg.CATCH, plain_t)
    assert 't_release_s' not in plain_fields
    assert 'release_site_mm' not in plain_fields


# ═════════════════════════════════════════════════════════════════════════════
# kind_display / budget_class / solves
# ═════════════════════════════════════════════════════════════════════════════

def _skill(kind, then_throw=None, lead_s=sc.LEAD_S):
    return sc.Skill(kind=kind, ball_id=0, site=_SITE, t_abs_s=1.0, window_s=0.3,
                    then_throw=then_throw, lead_s=lead_s)


def test_kind_display_buckets():
    assert spb.kind_display(_skill(sg.THROW)) == 'THROW'
    assert spb.kind_display(_skill(sg.REST)) == 'REST'
    assert spb.kind_display(_skill(sg.CATCH)) == 'CATCH'
    tt = sc.ThenThrow(t_release_abs_s=2.0, y_d=(np.zeros(2), 0.857), target=_SITE)
    assert spb.kind_display(_skill(sg.CATCH, then_throw=tt)) == 'CATCH+throw'
    assert spb.kind_display(_skill(sg.CATCH, then_throw=tt), is_resend=True) == 're-send'
    assert spb.kind_display(_skill(sg.THROW), is_preposition=True) == 'REST-pre'


def test_budget_class_classification():
    assert spb.budget_class(0, False) == 'fresh'
    assert spb.budget_class(0, True) == 'fresh'          # splice_k==0 wins
    assert spb.budget_class(12, True) == 'handoff'
    assert spb.budget_class(12, False) == 'unpinned'
    assert spb.budget_class(-1, False) == 'unpinned'      # a refusal


def _sv_row(code='OK', server_plan_ms=10.0):
    return {'code': code, 'server_plan_ms': server_plan_ms}


def test_solves_excludes_guard_refusals_but_keeps_real_ones():
    rows = [_sv_row('OK', 10.0), _sv_row('WRONG_MODE', 0.0),
           _sv_row('GUARD_LATCHED', 0.0), _sv_row('SERVICE_UNAVAILABLE', 0.0),
           _sv_row('SERVICE_TIMEOUT', 0.0), _sv_row('WINDOW_TOO_SHORT', 0.0),
           _sv_row('STALE_STATE', 0.4),        # guard-speed: excluded
           _sv_row('STALE_STATE', 12.0),       # a real timed solve: kept
           _sv_row('LIMIT_JERK', 22.0)]
    kept = spb.solves(rows)
    assert [r['code'] for r in kept] == ['OK', 'STALE_STATE', 'LIMIT_JERK']
    assert kept[1]['server_plan_ms'] == 12.0


# ═════════════════════════════════════════════════════════════════════════════
# The synthetic tracker
# ═════════════════════════════════════════════════════════════════════════════

def _tiny_schedule():
    return spb.build_schedule(n_throws=3, t0_abs_s=0.0)


def test_arm_a_zero_jitter_returns_the_nominal_landing_exactly():
    sched = _tiny_schedule()
    nominal, arrival = spb.build_nominal_landings(sched)
    tracker = spb.make_tracker(sched, jitter_mm=0.0, seed=0,
                              now_fn=lambda: 0.0)
    for ball_id, entries in nominal.items():
        pos, t_land = entries[0]
        landing = tracker(ball_id)
        assert landing is not None
        assert landing.pos_mm == pytest.approx(pos)
        assert landing.t_land_abs_s == pytest.approx(t_land)
        assert landing.vel_mm_s == pytest.approx(arrival)


def test_arm_b_jitter_stays_within_bound_and_is_deterministic_per_seed():
    sched = _tiny_schedule()
    nominal, _arrival = spb.build_nominal_landings(sched)
    ball_id = next(iter(nominal))
    pos0, _t = nominal[ball_id][0]

    t = {'v': 0.0}
    tracker_a = spb.make_tracker(sched, jitter_mm=3.0, seed=7, now_fn=lambda: t['v'])
    landing_a = tracker_a(ball_id)
    assert landing_a is not None
    delta = np.abs(np.asarray(landing_a.pos_mm) - np.asarray(pos0))
    assert np.all(delta[:2] <= 3.0 + 1e-9)
    assert delta[2] == pytest.approx(0.0)

    tracker_b = spb.make_tracker(sched, jitter_mm=3.0, seed=7, now_fn=lambda: t['v'])
    landing_b = tracker_b(ball_id)
    assert landing_b.pos_mm == pytest.approx(landing_a.pos_mm)   # same seed

    tracker_c = spb.make_tracker(sched, jitter_mm=3.0, seed=8, now_fn=lambda: t['v'])
    landing_c = tracker_c(ball_id)
    assert not np.allclose(landing_c.pos_mm, landing_a.pos_mm)   # different seed


def test_tracker_returns_none_past_the_landing_window():
    sched = _tiny_schedule()
    tracker = spb.make_tracker(sched, jitter_mm=0.0, seed=0,
                              now_fn=lambda: 1e6)      # long after every landing
    nominal, _ = spb.build_nominal_landings(sched)
    ball_id = next(iter(nominal))
    assert tracker(ball_id) is None
    assert tracker(999) is None                        # unknown ball


# ═════════════════════════════════════════════════════════════════════════════
# Preconditions
# ═════════════════════════════════════════════════════════════════════════════

def _good_snapshot(**kw):
    base = dict(status_age_s=0.2, mode='TRAJECTORY', streaming=True,
                leg_vel=spb.SESSION_LEG_VEL_MMPS, leg_acc=spb.SESSION_LEG_ACC_MMPS2,
                leg_jerk=spb.SESSION_LEG_JERK_MMPS3,
                gravity_correction_loaded=False, cycle_active=False,
                robot_state_age_s=0.05, is_homed=True, mpc_active='0',
                teensy_mpc_active='0')
    base.update(kw)
    return spb.Snapshot(**base)


def _by_id(checks):
    return {c['id']: c for c in checks}


def test_a_healthy_snapshot_passes_every_precondition():
    checks = spb.check_preconditions(_good_snapshot(), install_available=True)
    assert [c['id'] for c in checks] == ['P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7']
    assert all(c['ok'] for c in checks), _by_id(checks)


@pytest.mark.parametrize('kw,install_available,failing', [
    (dict(mode='STANDBY'), True, 'P1'),
    (dict(streaming=False), True, 'P1'),
    (dict(status_age_s=99.0), True, 'P1'),
    (dict(is_homed=False), True, 'P2'),
    (dict(robot_state_age_s=99.0), True, 'P2'),
    (dict(mpc_active='1'), True, 'P3'),
    (dict(teensy_mpc_active='1'), True, 'P3'),
    (dict(leg_jerk=30000.0), True, 'P4'),
    (dict(leg_vel=1000.0), True, 'P4'),
    (dict(gravity_correction_loaded=True), True, 'P5'),
    (dict(cycle_active=True), True, 'P6'),
    ({}, False, 'P7'),
])
def test_each_precondition_fails_on_its_own_symptom(kw, install_available, failing):
    """One symptom, one named refusal; every OTHER check stays green."""
    checks = _by_id(spb.check_preconditions(
        _good_snapshot(**kw), install_available=install_available))
    assert checks[failing]['ok'] is False, checks[failing]
    for cid, c in checks.items():
        if cid != failing:
            assert c['ok'] is True, '%s also failed: %s' % (cid, c)
    assert checks[failing]['fix'], 'every failure must carry a fix line'


def test_multiple_failures_are_ALL_reported_never_just_the_first():
    checks = spb.check_preconditions(
        _good_snapshot(mode='STANDBY', is_homed=False, mpc_active='1'),
        install_available=False)
    bad = {c['id'] for c in checks if not c['ok']}
    assert bad == {'P1', 'P2', 'P3', 'P7'}
    for cid in bad:
        assert _by_id(checks)[cid]['fix']


# ═════════════════════════════════════════════════════════════════════════════
# The verdict — each gate fails on its own symptom
# ═════════════════════════════════════════════════════════════════════════════

def _healthy_row(**kw):
    base = dict(attempt=0, t_rel_s=0.1, kind='THROW', ball_id=0, accepted=True,
                code='OK', server_plan_ms=20.0, client_rtt_ms=25.0,
                dispatch_late_ms=1.0, is_resend=False, is_preposition=False,
                splice_k=0, seeded_post_release=False, budget_class='fresh',
                disarmed_marker=True, load1=0.5, mpc_active='0',
                max_emit_gap_ms=24.0, message='fresh origin: THROW [wire DISARMED]')
    base.update(kw)
    return base


def _attempt_meta(ended_early=False, end_code='', n_skills=8, n_dispatched=8):
    return {'ended_early': ended_early, 'end_code': end_code,
           'n_skills': n_skills, 'n_dispatched': n_dispatched}


def test_G1_fails_at_the_bar_and_passes_just_under_it():
    rows = [_healthy_row(server_plan_ms=50.0)]
    assert _by_id(spb.evaluate(rows))['G1']['verdict'] == 'FAIL'
    rows = [_healthy_row(server_plan_ms=49.9)]
    assert _by_id(spb.evaluate(rows))['G1']['verdict'] == 'PASS'


def test_G1_skips_with_no_solves():
    assert _by_id(spb.evaluate([]))['G1']['verdict'] == 'SKIP'
    only_guard = [_healthy_row(code='WRONG_MODE', server_plan_ms=0.0)]
    assert _by_id(spb.evaluate(only_guard))['G1']['verdict'] == 'SKIP'


def test_G2_fails_on_a_single_SPLICE_TOO_LATE():
    rows = [_healthy_row(), _healthy_row(code='SPLICE_TOO_LATE', accepted=False,
                                        splice_k=-1, budget_class='unpinned',
                                        server_plan_ms=0.0)]
    assert _by_id(spb.evaluate(rows))['G2']['verdict'] == 'FAIL'


def test_G2_fails_on_a_handoff_RTT_at_the_bar():
    rows = [_healthy_row(budget_class='handoff', client_rtt_ms=spb.G2_HANDOFF_BUDGET_MS)]
    assert _by_id(spb.evaluate(rows))['G2']['verdict'] == 'FAIL'
    rows = [_healthy_row(budget_class='handoff',
                         client_rtt_ms=spb.G2_HANDOFF_BUDGET_MS - 0.1)]
    assert _by_id(spb.evaluate(rows))['G2']['verdict'] == 'PASS'


def test_G2_fails_on_an_unpinned_RTT_at_the_bar():
    rows = [_healthy_row(budget_class='unpinned', client_rtt_ms=spb.G2_UNPINNED_BUDGET_MS)]
    assert _by_id(spb.evaluate(rows))['G2']['verdict'] == 'FAIL'


def test_G3_fails_on_a_40ms_emit_gap_and_SKIPs_offline():
    rows = [_healthy_row(max_emit_gap_ms=40.0)]
    checks = _by_id(spb.evaluate(rows, status_gaps=[24.0, 40.0], mode='live'))
    assert checks['G3']['verdict'] == 'FAIL'
    ok = _by_id(spb.evaluate(rows, status_gaps=[24.0, 26.0], mode='live'))
    assert ok['G3']['verdict'] == 'PASS'
    rehearsed = _by_id(spb.evaluate(rows, status_gaps=[24.0, 40.0], mode='rehearse'))
    assert rehearsed['G3']['verdict'] == 'SKIP'
    no_samples = _by_id(spb.evaluate(rows, status_gaps=[], mode='live'))
    assert no_samples['G3']['verdict'] == 'SKIP'


def test_G4_fails_when_an_attempt_ends_early():
    good = _by_id(spb.evaluate([], attempts_meta=[_attempt_meta()]))
    assert good['G4']['verdict'] == 'PASS'
    bad = _by_id(spb.evaluate(
        [], attempts_meta=[_attempt_meta(ended_early=True, end_code='LIMIT_JERK',
                                        n_dispatched=3)]))
    assert bad['G4']['verdict'] == 'FAIL'
    assert 'LIMIT_JERK' in bad['G4']['detail']


def test_G4_allows_refused_resends_and_counts_them():
    rows = [_healthy_row(is_resend=True, accepted=False, code='LIMIT_JERK'),
           _healthy_row(is_resend=True, accepted=True)]
    checks = _by_id(spb.evaluate(rows, attempts_meta=[_attempt_meta()]))
    assert checks['G4']['verdict'] == 'PASS'
    assert '1 accepted' in checks['G4']['detail'] and '1 refused' in checks['G4']['detail']


def test_G5_fails_on_an_accepted_row_missing_the_marker():
    rows = [_healthy_row(disarmed_marker=False)]
    checks = _by_id(spb.evaluate(rows, mpc_active_samples=['0'], mode='live'))
    assert checks['G5']['verdict'] == 'FAIL'
    ok = _by_id(spb.evaluate([_healthy_row()], mpc_active_samples=['0'], mode='live'))
    assert ok['G5']['verdict'] == 'PASS'


def test_G5_fails_if_mpc_active_was_ever_seen_armed():
    rows = [_healthy_row()]
    checks = _by_id(spb.evaluate(rows, mpc_active_samples=['0', '1', '0'], mode='live'))
    assert checks['G5']['verdict'] == 'FAIL'
    assert 'armed=True' in checks['G5']['detail']


def test_G5_skips_offline():
    checks = _by_id(spb.evaluate([_healthy_row(disarmed_marker=None)], mode='rehearse'))
    assert checks['G5']['verdict'] == 'SKIP'


def test_verdict_rc_is_zero_iff_no_FAIL():
    assert spb.verdict_rc([{'verdict': 'PASS'}, {'verdict': 'SKIP'}]) == 0
    assert spb.verdict_rc([{'verdict': 'PASS'}, {'verdict': 'FAIL'}]) == 1


# ═════════════════════════════════════════════════════════════════════════════
# The attempt loop — ONE deterministic smoke on a VIRTUAL clock
# ═════════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize('start, n_throws', [(0.0, 2), (1789263419.5, 6)])
def test_the_attempt_loop_dispatches_the_whole_schedule_pre_position_first(
        start, n_throws):
    """A fake clock advanced by the tick period, a pure ``install_segment``
    installer (``t_install_s=None`` so ``SPLICE_TOO_LATE`` cannot fire from a
    virtual clock racing ahead of a real one), 1 attempt: every scheduled skill
    accepted, and the pre-position ran before any of them.

    Run at t = 0 AND at a ROS wall-clock start (the R2 gate sitting's own,
    2026-09-13). Near t = 0 the schedule was always exact; at ~1.79e9 s it held
    two unfolded catch/throw pairs, whose THROW refused LIMIT_JERK one knot after
    a rest-terminal catch. Six throws, so there are pairs to fold."""
    from jugglebot.motion.geometry import StewartGeometry
    from jugglebot.motion.trajectory.cup_realize import RealizeConfig
    from jugglebot.motion.trajectory.limits import TrajectoryLimits
    from jugglebot.motion import unified_cycle as uc

    limits = TrajectoryLimits.from_config(spb.hw).with_session_limits(
        leg_vel_mmps=spb.SESSION_LEG_VEL_MMPS,
        leg_acc_mmps2=spb.SESSION_LEG_ACC_MMPS2,
        leg_jerk_mmps3=spb.SESSION_LEG_JERK_MMPS3,
        hand_acc_rps2=spb.SESSION_HAND_ACC_RPS2)
    geom = StewartGeometry()
    park = uc.CycleState.at_rest(
        np.array([0.0, 0.0, float(spb.hw.JB_OP_DEFAULT_ACTIVE_Z_MM), 0.0, 0.0, 0.0]),
        float(spb.hw.JB_OP_HAND_ACTIVATE_POSITION_REV), RealizeConfig())

    clock = {'t': float(start)}

    def now_fn():
        return clock['t']

    def sleep_fn(period_s):
        clock['t'] += period_s

    state = {'rec': None}
    call_meta = []

    def installer(kind, terminal, t_now_s, ball_id=0):
        rec = state['rec']
        seed_rest = park if rec is None else None
        new_rec, res, _seg = ex.install_segment(
            rec, seed_rest, kind, terminal, t_now_s, limits=limits, geom=geom,
            t_install_s=None)
        if res.accepted:
            state['rec'] = new_rec
        call_meta.append({'client_rtt_ms': 0.0, 'disarmed_marker': None,
                          'load1': 0.0, 'mpc_active': None,
                          'max_emit_gap_ms': None})
        return res

    sites = si.columns_sites(spb.SEPARATION_MM)
    pre_terminal = sg.RestTerminal(rest_site_mm=sites[0].rest_site_mm(),
                                   t_rest_s=now_fn() + 1.0)
    pre_new_rec, pre_res, _seg = ex.install_segment(
        None, park, sg.REST, pre_terminal, now_fn(), limits=limits, geom=geom,
        t_install_s=None)
    assert pre_res.accepted, pre_res.message
    state['rec'] = pre_new_rec
    while now_fn() < state['rec'].end_s + 0.05:
        sleep_fn(0.005)

    t0 = now_fn() + spb._START_LEAD_S
    schedule = spb.build_schedule(n_throws=n_throws, t0_abs_s=t0)
    assert len(schedule.skills) == n_throws + 2
    tracker = spb.make_tracker(schedule, jitter_mm=0.0, seed=0, now_fn=now_fn)
    exe = ex.SkillExecutor(schedule, installer, tracker=tracker)

    rows, ended, end_code, abort_reason = spb.drive_executor(
        exe, call_meta, run_t0=float(start), attempt=0, now_fn=now_fn,
        sleep_fn=sleep_fn)

    assert not ended, end_code
    assert abort_reason == ''
    assert len(exe.dispatched) == len(schedule.skills)

    # Every FIRST occurrence of each skill index (the primary dispatch, never
    # a re-send) was accepted.
    first_seen = {}
    for idx, _sk, r in exe.results:
        if idx not in first_seen:
            first_seen[idx] = r
    assert all(r.accepted for r in first_seen.values()), first_seen
    assert set(first_seen) == set(range(len(schedule.skills)))

    # The pre-position ran, and strictly before the schedule's own rows.
    assert pre_res.accepted
    assert all(row['t_rel_s'] >= 0.0 for row in rows)


def test_an_aborted_install_is_not_a_solve():
    """The live installer refuses 'ABORTED' without calling the node once an abort
    has tripped; that row carries no solve and must not enter G1's statistics."""
    assert not spb.is_solve({'code': 'ABORTED', 'server_plan_ms': 0.0})



# ═════════════════════════════════════════════════════════════════════════════
# Budget classes for refused installs, and what G2 reads
# ═════════════════════════════════════════════════════════════════════════════

def test_expected_class_names_the_budget_an_install_would_spend():
    """Known before the answer: a re-send and any general-lead splice are
    'unpinned', a handoff-lead skill is 'handoff', the launch THROW is 'fresh'."""
    import types
    from jugglebot.motion.skills.schedule import HANDOFF_LEAD_S, LEAD_S
    handoff = types.SimpleNamespace(kind=sg.CATCH, lead_s=HANDOFF_LEAD_S)
    launch = types.SimpleNamespace(kind=sg.THROW, lead_s=LEAD_S)
    rest = types.SimpleNamespace(kind=sg.REST, lead_s=LEAD_S)
    assert spb.expected_class(handoff, is_resend=False) == 'handoff'
    assert spb.expected_class(handoff, is_resend=True) == 'unpinned'
    assert spb.expected_class(launch, is_resend=False) == 'fresh'
    assert spb.expected_class(rest, is_resend=False) == 'unpinned'


def test_a_refused_row_keeps_its_expected_class_and_an_accepted_row_its_real_one():
    """2026-09-13: a refused handoff (no splice knot) was classed 'unpinned' and
    read against the 75 ms budget. An accepted install is classed by where it
    actually spliced, whatever was expected."""
    kw = dict(attempt=0, t_rel_s=0.0, kind='CATCH+throw', ball_id=1,
              is_resend=False, is_preposition=False, dispatch_late_ms=1.0,
              client_rtt_ms=50.0, disarmed_marker=None, load1=1.0,
              mpc_active='0', max_emit_gap_ms=25.0)
    refused = ex.InstallResult(False, 'SPLICE_TOO_LATE', 'late', 0.05)
    assert spb._row(res=refused, expected_class='handoff', **kw)['budget_class'] == 'handoff'
    accepted = ex.InstallResult(True, 'OK', 'ok', 0.05, splice_k=21,
                                seeded_post_release=True)
    assert spb._row(res=accepted, expected_class='unpinned', **kw)['budget_class'] == 'handoff'


def test_G2_ignores_the_round_trip_of_an_install_that_spent_no_budget():
    """A planning refusal installed nothing, so its round trip spent no splice
    budget; only accepted installs and SPLICE_TOO_LATE rows are read."""
    rows = [_healthy_row(),
            _healthy_row(accepted=False, code='LIMIT_JERK', splice_k=-1,
                         budget_class='unpinned', client_rtt_ms=500.0)]
    assert _by_id(spb.evaluate(rows))['G2']['verdict'] == 'PASS'


def test_the_driver_caps_the_blas_pool_before_anything_imports_numpy():
    """The rehearsal solves in THIS process, so it must run with the planner
    nodes' one-thread BLAS pool. MEASURED 2026-09-13 (idle Jetson, warm start
    carried): uncapped worst install 94.9 ms, capped 31.8 ms. The cap is inert
    unless it precedes the first numpy import, so the order is what is pinned."""
    src = open(os.path.join(_HW_DIR, 'skills_plan_bench.py')).read()
    cap = src.index("os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')")
    assert src.index("os.environ.setdefault('OMP_NUM_THREADS', '1')") > cap
    first_numpy = min(i for i in (src.find('\nimport numpy'), src.find('\nfrom jugglebot'),
                                  src.find('\nimport jugglebot')) if i >= 0)
    assert cap < first_numpy


# ═════════════════════════════════════════════════════════════════════════════
# R3-g — the self-toss pattern selector (schedule / tracker / learner / memory)
# ═════════════════════════════════════════════════════════════════════════════

def test_default_self_toss_site_is_the_owner_P1():
    site = spb.default_self_toss_site()
    assert site.name == 'P1'
    assert site.cup_mm == pytest.approx(
        [spb.SELF_TOSS_SITE_X_MM, spb.SELF_TOSS_SITE_Y_MM, si.CATCH_CUP_Z_MM])


@pytest.mark.parametrize('n_throws', [1, 2, 3, 4])
def test_build_self_toss_schedule_has_n_throws_plus_3_skills(n_throws):
    """The opening REST (floor lift) + the folded catch-with-throws + the
    final standalone catch + the closing REST — one ball throughout (schedule.
    compile_one_ball's own count), verified against the real compiler rather
    than restated as a literal."""
    sched = spb.build_self_toss_schedule(n_throws=n_throws)
    assert len(sched.skills) == n_throws + 3
    assert sched.skills[0].kind == sg.REST
    assert sched.skills[-1].kind == sg.REST
    assert all(sk.site.name == 'P1' for sk in sched.skills)
    assert all(sk.ball_id == 0 for sk in sched.skills)


def test_build_self_toss_schedule_honours_an_explicit_site():
    other = si.Site('P9', np.array([12.0, -34.0, si.CATCH_CUP_Z_MM]))
    sched = spb.build_self_toss_schedule(n_throws=1, site=other)
    assert all(sk.site is other for sk in sched.skills)


def test_self_toss_leg_jerk_matches_the_swept_admissible_box():
    """The box `config/generated/admissible_box.yaml` was swept UNDER a
    specific leg-jerk ceiling (`admissible.check_limits`'s job at runtime); if
    this constant silently drifted from that sweep, every self-toss rehearsal
    would clip against the WRONG envelope with no loud complaint until a live
    `check_limits` call — this pins the two together offline."""
    box_path = os.path.join(_REPO, 'config', 'generated', 'admissible_box.yaml')
    if not os.path.exists(box_path):
        pytest.skip('admissible_box.yaml not generated in this checkout')
    from jugglebot.motion.skills import admissible as adm
    boxes = adm.load(box_path)
    p1p1 = next(b for b in boxes if b.site_pair == ('P1', 'P1'))
    assert p1p1.limits['leg_jerk_mmps3'] == pytest.approx(
        spb.SELF_TOSS_LEG_JERK_MMPS3)


# ── biased_landing — the analytic R3 plant-error model ─────────────────────

def test_biased_landing_is_deterministic():
    site_mm = np.array([-50.0, 0.0, 860.0])
    target_mm = np.array([-50.0, 0.0, 830.0])
    a = spb.biased_landing(site_mm, target_mm, 0.8568805868963759)
    b = spb.biased_landing(site_mm, target_mm, 0.8568805868963759)
    assert a[0] == pytest.approx(b[0])
    assert a[1] == pytest.approx(b[1])
    assert a[2] == pytest.approx(b[2])


def test_biased_landing_shifts_xy_and_lengthens_flight_for_a_vertical_self_toss():
    """A self-toss's commanded flight is a pure vertical launch (site == the
    target xy); the plan's +11% speed / +8.5 mrad aim bias must show up as a
    NONZERO horizontal miss and a LONGER observed flight — otherwise the
    tracker would feed the learner an unbiased (useless) row. MEASURED
    2026-09-13 (this function, apex 0.9 m, site P1): x offset -12.7 mm,
    flight +10.8% — both asserted with generous tolerance bands rather than
    pinned exactly, since the bias direction/scale is what matters here, not
    the last digit."""
    site_mm = np.array([-50.0, 0.0, 860.0])
    target_mm = np.array([-50.0, 0.0, 830.0])
    flight_s = 0.8568805868963759
    pos, vel, t_land = spb.biased_landing(site_mm, target_mm, flight_s)
    assert pos[2] == pytest.approx(target_mm[2])          # crosses the same z
    assert 5.0 < abs(pos[0] - target_mm[0]) < 60.0         # tens-of-mm miss
    assert pos[1] == pytest.approx(target_mm[1], abs=1e-6)  # no y for a +x tilt
    assert 0.05 < (t_land - flight_s) / flight_s < 0.20     # ~11% longer


def test_release_physics_of_a_throw_and_a_carried_catch_throw():
    throw_t = sg.ThrowTerminal(site_mm=[1.0, 2.0, 860.0], target_mm=[3.0, 4.0, 830.0],
                               flight_s=0.8, t_release_s=5.0)
    physics = spb._release_physics_of(sg.THROW, throw_t)
    assert physics[0] == pytest.approx([1.0, 2.0, 860.0])
    assert physics[1] == pytest.approx([3.0, 4.0, 830.0])
    assert physics[2] == pytest.approx(0.8)
    assert physics[3] == pytest.approx(5.0)

    then_throw = sg.ThrowAfterCatch(t_release_s=6.5, site_mm=[1.0, 2.0, 860.0],
                                    target_mm=[3.0, 4.0, 830.0], flight_s=0.8)
    catch_t = sg.CatchTerminal(landing_mm=[1.0, 2.0, 830.0],
                               landing_vel_mm_s=[0.0, 0.0, -4200.0],
                               t_land_s=6.0, rest_site_mm=[1.0, 2.0, 689.6],
                               then_throw=then_throw)
    physics2 = spb._release_physics_of(sg.CATCH, catch_t)
    assert physics2[3] == pytest.approx(6.5)

    plain_catch = sg.CatchTerminal(landing_mm=[1.0, 2.0, 830.0],
                                   landing_vel_mm_s=[0.0, 0.0, -4200.0],
                                   t_land_s=6.0, rest_site_mm=[1.0, 2.0, 689.6])
    assert spb._release_physics_of(sg.CATCH, plain_catch) is None
    rest_t = sg.RestTerminal(rest_site_mm=[1.0, 2.0, 689.6], t_rest_s=7.0)
    assert spb._release_physics_of(sg.REST, rest_t) is None


def test_make_self_toss_tracker_reports_nothing_before_a_noted_release():
    tracker, note_release = spb.make_self_toss_tracker(now_fn=lambda: 0.0)
    assert tracker(0) is None
    throw_t = sg.ThrowTerminal(site_mm=[-50.0, 0.0, 860.0], target_mm=[-50.0, 0.0, 830.0],
                               flight_s=0.8568805868963759, t_release_s=2.0)
    note_release(0, sg.THROW, throw_t)
    landing = tracker(0)
    assert landing is not None
    assert landing.pos_mm[2] == pytest.approx(830.0)
    assert landing.t_land_abs_s > 2.0 + 0.8568805868963759 * 0.9  # a later, biased landing
    assert tracker(999) is None                                  # unknown ball


def test_make_self_toss_tracker_layers_jitter_deterministically_by_seed():
    throw_t = sg.ThrowTerminal(site_mm=[-50.0, 0.0, 860.0], target_mm=[-50.0, 0.0, 830.0],
                               flight_s=0.8568805868963759, t_release_s=2.0)
    t1, note1 = spb.make_self_toss_tracker(jitter_mm=3.0, seed=7, now_fn=lambda: 0.0)
    note1(0, sg.THROW, throw_t)
    t2, note2 = spb.make_self_toss_tracker(jitter_mm=3.0, seed=7, now_fn=lambda: 0.0)
    note2(0, sg.THROW, throw_t)
    assert t1(0).pos_mm == pytest.approx(t2(0).pos_mm)      # same seed
    t3, note3 = spb.make_self_toss_tracker(jitter_mm=3.0, seed=8, now_fn=lambda: 0.0)
    note3(0, sg.THROW, throw_t)
    assert not np.allclose(t3(0).pos_mm, t1(0).pos_mm)      # different seed


# ── print_dry_run / CLI defaulting ──────────────────────────────────────────

def test_print_dry_run_self_toss_names_the_site_and_pattern(capsys):
    spb.print_dry_run(2, pattern='self-toss')
    out = capsys.readouterr().out
    assert 'self-toss schedule' in out
    assert 'P1' in out


def test_main_defaults_n_throws_to_1_for_self_toss_dry_run(capsys):
    rc = spb.main(['--dry-run', '--pattern', 'self-toss'])
    assert rc == 0
    out = capsys.readouterr().out
    assert 'n_throws=%d' % (spb.SELF_TOSS_DEFAULT_N_THROWS,) in out


def test_dry_run_self_toss_honours_the_apex_option(capsys):
    """The 2026-09-14 apex ladder rehearses every rung (0.5-0.9 m) with
    ``--apex-m``; before it the self-toss dry run and rehearsal were pinned to
    ``APEX_M`` (0.9 m), so a 0.5 m rung could not be rehearsed offline at
    all."""
    rc = spb.main(['--dry-run', '--pattern', 'self-toss', '--apex-m', '0.5'])
    assert rc == 0
    out = capsys.readouterr().out
    assert 'apex=0.50 m' in out
    assert 'flight=0.6387 s' in out


def test_main_leaves_columns_n_throws_default_unchanged(capsys):
    rc = spb.main(['--dry-run'])
    assert rc == 0
    out = capsys.readouterr().out
    assert 'n_throws=%d' % (spb.DEFAULT_N_THROWS,) in out


# ── the self-toss rehearsal loop, on a virtual clock — the R3-g proof ───────

def test_self_toss_rehearsal_pre_positions_then_runs_the_schedule_clean():
    """One attempt, virtual clock, no learner: the pre-position (R3-g finding
    — schedule.compile_one_ball's own FLOOR_LIFT_S refuses LIMIT_JERK at the
    150 000 mm/s^3 R3 session limit, see SELF_TOSS_PRELIFT_S's docstring) plus
    the whole schedule install cleanly, exactly like the manual rehearsal run
    2026-09-13 (`--rehearse --pattern self-toss --arm A`, all gates PASS)."""
    limits = TrajectoryLimits.from_config(spb.hw).with_session_limits(
        leg_vel_mmps=spb.SESSION_LEG_VEL_MMPS, leg_acc_mmps2=spb.SESSION_LEG_ACC_MMPS2,
        leg_jerk_mmps3=spb.SELF_TOSS_LEG_JERK_MMPS3, hand_acc_rps2=spb.SESSION_HAND_ACC_RPS2)
    geom = StewartGeometry()
    site = spb.default_self_toss_site()

    clock = {'t': 0.0}

    def now_fn():
        return clock['t']

    def sleep_fn(period_s):
        clock['t'] += period_s

    rows, meta = spb.rehearse_attempt(
        0, n_throws=1, jitter_mm=0.0, limits=limits, geom=geom, run_t0=0.0,
        now_fn=now_fn, sleep_fn=sleep_fn, state={'rec': None},
        pattern='self-toss', site=site)

    assert not meta['ended_early'], meta['end_code']
    assert meta['n_dispatched'] == meta['n_skills']
    kinds = [r['kind'] for r in rows]
    assert kinds[0] == 'REST-pre'
    assert all(r['accepted'] for r in rows)


def test_self_toss_rehearsal_grows_memory_and_the_learner_changes_the_command(
        tmp_path):
    """Three cold-start attempts (n_throws=1 each, policy A), ONE memory
    shared across them: by the third attempt the memory holds >= LearnerConfig
    .k_min (2) rows, so the commanded (landing offset, flight) must differ
    from the identity prior (0, 0, flight) — the whole point of wiring the
    learner + memory into this rehearsal (R3-g brief). Matches the manual
    rehearsal 2026-09-13 (`--rehearse --pattern self-toss --arm A --attempts
    3`): rows 1-2 command the prior exactly, row 3 does not."""
    limits = TrajectoryLimits.from_config(spb.hw).with_session_limits(
        leg_vel_mmps=spb.SESSION_LEG_VEL_MMPS, leg_acc_mmps2=spb.SESSION_LEG_ACC_MMPS2,
        leg_jerk_mmps3=spb.SELF_TOSS_LEG_JERK_MMPS3, hand_acc_rps2=spb.SESSION_HAND_ACC_RPS2)
    geom = StewartGeometry()
    site = spb.default_self_toss_site()

    from jugglebot.motion.skills.memory import Memory
    from jugglebot.motion.skills import learner as lr
    memory = Memory(str(tmp_path / 'memory.csv'))
    cfg = lr.LearnerConfig()
    learner = types.SimpleNamespace(command=lambda x, y_d: memory.command(x, y_d, cfg))

    clock = {'t': 0.0}

    def now_fn():
        return clock['t']

    def sleep_fn(period_s):
        clock['t'] += period_s

    shared = {'rec': None}
    commands = []
    for a in range(3):
        rows, meta = spb.rehearse_attempt(
            a, n_throws=1, jitter_mm=0.0, limits=limits, geom=geom, run_t0=0.0,
            now_fn=now_fn, sleep_fn=sleep_fn, state=shared, pattern='self-toss',
            site=site, learner=learner, boxes=None, on_experience=memory.append)
        assert not meta['ended_early'], (a, meta['end_code'])
        commands.append(memory.arrays()[1][-1].copy())    # this attempt's u

    assert len(memory) == 3
    identity_u = commands[0]
    assert commands[1] == pytest.approx(identity_u)        # still cold (1 row)
    assert not np.allclose(commands[2], identity_u)        # warm (>= k_min rows)
