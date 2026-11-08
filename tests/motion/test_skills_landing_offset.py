"""BUG-80: the commanded landing offset of a one-Site one-ball goal, on the
pure-Python side (``schedule.OneBallPattern.landing_offset_m``,
``nominate.landing_offset_refusal`` / ``build_pattern``).

The offset rides ``y_d``, the identity-prior command, so every consumer of
``y_d`` (the Throw terminal, the Catch prior, the Learner query, the
nomination) sees one number.  The arithmetic the Tilt tests pin is BUG-80's own
(``map/bug-the-flyback-over-corrects-...md`` § Findings (2026-11-07), replay
row s = 1.00): at Apex 0.9 a 20 mm lateral displacement from rest flies at
5.60 mrad.  Recipe: ``cup_cycle.takeoff_velocity`` from the Site's throw point
to its catch point + 20 mm over ``schedule.flight_s(0.9)``, then
``tilt_geometry.tilt_to_throw``; ``python`` on the Jetson, 2026-11-09.

The bit-identity tests compare against a Pattern built WITHOUT the field, and
with the real planner; the same statement against the pre-change code was
checked once by digest (logbook ``2026-11-09-landing-offset-parameter``).
"""
from __future__ import annotations

import dataclasses
import math
from types import SimpleNamespace

import numpy as np
import pytest

from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import learner as lrn
from jugglebot.motion.skills import nominate as nm
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import sites as st
from jugglebot.motion.trajectory import cup_cycle as cc
from jugglebot.motion.trajectory import tilt_geometry as tg

P4 = nm.sitting_limits(600.0, 8000.0, 200000.0, 4500.0)
SITE_XY_MM = (-50.0, 0.0)


def _config(offset_mm=(0.0, 0.0), apex=1.0, planner_mode='pose'):
    return nm.NominationConfig(
        'self_toss', apex, 100.0, 0.27, planner_mode=planner_mode,
        self_toss_site_xy_mm=SITE_XY_MM, landing_offset_mm=offset_mm)


def _site():
    return st.Site('P1', np.array([SITE_XY_MM[0], SITE_XY_MM[1],
                                   st.CATCH_CUP_Z_MM]))


def _plain_pattern(apex=1.0, n_throws=3):
    """The Pattern as today's code builds it: no offset field passed."""
    return sc.OneBallPattern(sites=(_site(),), apex_m=apex, dwell_s=0.27,
                             n_throws=n_throws)


def _y_ds(schedule):
    out = []
    for sk in schedule.skills:
        if sk.y_d is not None:
            out.append(sk.y_d)
        if sk.then_throw is not None:
            out.append(sk.then_throw.y_d)
    return out


def _same(a, b):
    """Exact, recursive equality of plan data, down to the float bits."""
    if isinstance(a, np.ndarray) or isinstance(b, np.ndarray):
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        return a.shape == b.shape and a.tobytes() == b.tobytes()
    if dataclasses.is_dataclass(a) and not isinstance(a, type):
        return all(_same(getattr(a, f.name), getattr(b, f.name))
                   for f in dataclasses.fields(a))
    if isinstance(a, (list, tuple)):
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b))
    if isinstance(a, float):
        return repr(a) == repr(b)
    if isinstance(a, (int, str, bool, type(None))):
        return a == b
    return True     # solver handles and other opaque objects: not plan data


def _plans(config, monkeypatch):
    """Every Segment the real planner returns for a nomination of ``config``."""
    real, segments = sg.plan_segment, []

    def record(*a, **k):
        seg = real(*a, **k)
        segments.append(seg)
        return seg
    monkeypatch.setattr(nm.sg, 'plan_segment', record)
    nm.clear_cache()
    rep = nm.nominate(config, P4, search=False)
    monkeypatch.setattr(nm.sg, 'plan_segment', real)
    nm.clear_cache()
    return rep, segments


# ── default 0 is today, bit for bit ──────────────────────────────────────────

@pytest.mark.parametrize('zero', [(0.0, 0.0), (-0.0, 0.0), (0.0, -0.0)])
def test_a_zero_offset_compiles_the_schedule_of_todays_pattern(zero):
    plain = nm.compile_schedule(dataclasses.replace(_config()))
    got = nm.compile_schedule(_config(offset_mm=zero))
    assert len(_y_ds(got)) == len(_y_ds(plain)) == 3
    for (d0, a0), (d1, a1) in zip(_y_ds(plain), _y_ds(got)):
        assert d1.tobytes() == d0.tobytes() == np.zeros(2).tobytes()
        assert a1 == a0
    assert _same(got.skills[1].site.cup_mm, plain.skills[1].site.cup_mm)


def test_zero_offset_pattern_equals_a_pattern_built_without_the_field():
    with_field = sc.OneBallPattern(sites=(_site(),), apex_m=1.0, dwell_s=0.27,
                                   n_throws=3, landing_offset_m=(0.0, -0.0))
    plain = _plain_pattern()
    assert with_field.landing_offset_m == (0.0, 0.0)
    assert plain.landing_offset_m == (0.0, 0.0)
    assert repr(with_field.landing_offset_m[1]) == '0.0'    # -0.0 normalised


@pytest.mark.parametrize('mode', ['pose', 'level'])
def test_every_knot_is_bit_identical_at_zero_offset(mode, monkeypatch):
    """The real planner on the short self-toss (launch + two carried folds, at
    all three rungs), offset field defaulted vs. the same config with an
    explicit zero: Tilt, Knots and takeoff all equal to the float bits."""
    rep0, plain = _plans(_config(planner_mode=mode), monkeypatch)
    rep1, zero = _plans(dataclasses.replace(
        _config(planner_mode=mode), landing_offset_mm=(0.0, -0.0)), monkeypatch)
    assert rep0.ok and rep1.ok
    assert len(plain) == len(zero) > 0
    for a, b in zip(plain, zero):
        assert _same(a.plan, b.plan)
        assert _same(a.takeoff_vel_mm_s, b.takeoff_vel_mm_s)
        assert _same(a.release_state, b.release_state)


# ── +20 mm in x moves the landing by +20 mm at the expected Tilt ────────────

def _terminal(schedule):
    """THROW 0's real terminal, from a real executor with no Learner."""
    def installer(*_a, **_k):
        raise AssertionError('nothing may install in this test')
    execu = ex.SkillExecutor(schedule, installer)
    idx = next(i for i, s in enumerate(schedule.skills) if s.kind == sc.THROW)
    return execu._throw_terminal(idx, schedule.skills[idx])


def test_a_20_mm_offset_moves_the_commanded_landing_by_20_mm():
    schedule = nm.compile_schedule(_config(offset_mm=(20.0, 0.0)))
    term = _terminal(schedule)
    catch_point = _site().catch_site_mm()
    assert term.target_mm[0] == pytest.approx(catch_point[0] + 20.0, abs=1e-9)
    assert term.target_mm[1] == pytest.approx(catch_point[1], abs=1e-9)
    assert term.target_mm[2] == catch_point[2]
    # The release stays AT the Site: this is a Throw from rest.
    assert term.site_mm[0] == pytest.approx(_site().throw_site_mm()[0])


@pytest.mark.parametrize('off', [(20.0, 0.0), (-20.0, 0.0), (0.0, 20.0)])
def test_the_offset_gives_the_expected_tilt_sign_and_size(off):
    """BUG-80's from-rest arithmetic: 5.60 mrad at Apex 0.9 for 20 mm (the
    Item's replay row at s = 1.00), along the offset's own axis, in the
    direction of the offset (the cup axis leans toward the landing)."""
    schedule = nm.compile_schedule(_config(offset_mm=off, apex=0.9))
    term = _terminal(schedule)
    v = cc.takeoff_velocity(np.asarray(term.site_mm) / 1000.0,
                            np.asarray(term.target_mm) / 1000.0,
                            float(term.flight_s))
    tilt = tg.tilt_to_throw(v)
    axis = tg.cup_axis(float(tilt[0]), float(tilt[1]))
    lean = np.array([axis[0], axis[1]]) / float(axis[2])
    want = np.array(off, dtype=float) / 20.0
    assert abs(np.hypot(*lean) * 1000.0 - 5.60) < 0.1
    assert lean == pytest.approx(want * np.hypot(*lean), abs=1e-6)
    assert np.sign(lean[np.argmax(np.abs(off))]) == np.sign(max(off, key=abs))


def test_the_real_planner_flies_the_offset_at_the_commanded_lateral_speed(monkeypatch):
    rep, segs = _plans(_config(offset_mm=(20.0, 0.0)), monkeypatch)
    assert rep.ok, rep.summary()
    t_f = sc.flight_s(1.0)
    # One THROW 0 per commanded-flight rung (nominate.RUNGS, in order).
    launches = [s for s in segs if s.kind == sc.THROW]
    assert len(launches) == len(nm.RUNGS)
    for frac, seg in zip(nm.RUNGS, launches):
        want = 20.0 / (t_f * (1.0 + frac))
        assert seg.takeoff_vel_mm_s[0] == pytest.approx(want, rel=2e-3)
        assert seg.takeoff_vel_mm_s[1] == pytest.approx(0.0, abs=1e-6)


def test_the_catch_prior_and_clamp_centre_move_with_the_offset():
    """The Catch is aimed at Site + offset, and the clamp's authority is
    measured from there, through the ordinary ``_predicted_landing``."""
    schedule = nm.compile_schedule(_config(offset_mm=(20.0, -10.0)))

    def installer(*_a, **_k):
        raise AssertionError('nothing may install in this test')
    execu = ex.SkillExecutor(schedule, installer)
    idx = next(i for i, s in enumerate(schedule.skills) if s.kind == sc.CATCH)
    predicted = execu._predicted_landing(idx, schedule.skills[idx])
    catch_point = _site().catch_site_mm()
    assert predicted.pos_mm[0] == pytest.approx(catch_point[0] + 20.0, abs=1e-9)
    assert predicted.pos_mm[1] == pytest.approx(catch_point[1] - 10.0, abs=1e-9)


# ── the reload tail is the same shared tail ──────────────────────────────────

def test_a_self_toss_reload_carries_the_offset_too():
    pattern = sc.OneBallPattern(sites=(_site(),), apex_m=1.0, dwell_s=0.27,
                                n_throws=1, landing_offset_m=(0.02, 0.0),
                                floor_lift_s=sc.FLOOR_LIFT_S)
    t_land = 100.0
    sch = sc.compile_reload(np.array([-50.0, 0.0, st.CATCH_CUP_Z_MM]),
                            np.array([900.0, 0.0, -4200.0]), t_land, pattern,
                            t_land - 5.0)
    throws = [s for s in sch.skills if s.kind == sc.THROW]
    assert len(throws) == 1
    assert throws[0].y_d[0].tolist() == [0.02, 0.0]


# ── the Learner reads an offset as a command, never as a miss ───────────────

def _row(u_xy, y_xy, apex=1.0):
    return SimpleNamespace(
        site_xy_m=np.array([-0.05, 0.0]), release_m=np.array([-0.05, 0.0, 0.86]),
        release_vel_mps=np.array([u_xy[0] / 0.9, u_xy[1] / 0.9, 4.4]),
        dwell_s=0.27, u=np.array([u_xy[0], u_xy[1], apex]),
        y=np.array([y_xy[0], y_xy[1], apex]), ball_id=0, release_kind='free_cup')


def _ctx():
    return lrn.ThrowContext(site_xy_m=(-0.05, 0.0), release_m=(-0.05, 0.0, 0.86),
                            release_vel_mps=(0.0, 0.0, 4.4), dwell_s=0.27,
                            ball_id=0, release_kind='free_cup')


def test_rows_flown_at_an_offset_do_not_shift_a_zero_offset_command():
    """A row stores ``u`` and ``y`` measured from the Site, so a commanded
    +20 mm that lands +20 mm has a zero residual: at ``y_d = 0`` the command
    stays at 0.  A landing that really missed DOES move it (the control)."""
    y_d = np.array([0.0, 0.0, 1.0])
    offset_learner = lrn.Learner()
    for sign in (1.0, -1.0) * 10:
        offset_learner.observe(_row((0.02 * sign, 0.0), (0.02 * sign, 0.0)))
    assert offset_learner.command(_ctx(), y_d)[:2] == pytest.approx([0.0, 0.0],
                                                                    abs=1e-3)
    missing_learner = lrn.Learner()
    for _ in range(20):
        missing_learner.observe(_row((0.0, 0.0), (0.02, 0.0)))
    assert missing_learner.command(_ctx(), y_d)[0] < -0.005


# ── who may fly one, and how far ────────────────────────────────────────────

@pytest.mark.parametrize('pattern', ['columns', 'columns_highlow', 'columns_turned',
                                     'columns_1ball', 'hop', 'oval_1ball',
                                     'magazine', 'reload'])
def test_a_non_zero_offset_refuses_every_pattern_but_the_self_toss(pattern):
    msg = nm.landing_offset_refusal(20.0, 0.0, 30.0, pattern=pattern)
    assert msg is not None and pattern in msg and 'self_toss' in msg
    assert nm.landing_offset_refusal(0.0, 0.0, 30.0, pattern=pattern) is None


@pytest.mark.parametrize('pattern', ['columns', 'columns_1ball', 'hop', 'oval_1ball'])
def test_build_pattern_refuses_a_non_zero_offset_on_other_patterns(pattern):
    cfg = nm.NominationConfig(pattern, 0.9, 100.0, 0.27,
                              landing_offset_mm=(20.0, 0.0))
    with pytest.raises(ValueError, match='landing offset'):
        nm.build_pattern(cfg)


@pytest.mark.parametrize('sites, roles', [
    (2, 'alternate'), (2, 'fixed')])
def test_the_one_ball_pattern_refuses_an_offset_on_more_than_one_site(sites, roles):
    two = tuple(st.columns_sites(100.0))[:sites]
    with pytest.raises(ValueError, match='landing offset'):
        sc.OneBallPattern(sites=two, apex_m=0.9, dwell_s=0.27, n_throws=2,
                          roles=roles, landing_offset_m=(0.02, 0.0))


def test_the_bound_is_the_lateral_authority_per_axis():
    assert nm.landing_offset_refusal(30.0, -30.0, 30.0, pattern='self_toss') is None
    for x, y in [(30.1, 0.0), (0.0, -30.1)]:
        msg = nm.landing_offset_refusal(x, y, 30.0, pattern='self_toss')
        assert msg is not None and 'learner_lateral_authority_mm' in msg


def test_authority_zero_refuses_every_non_zero_offset():
    assert nm.landing_offset_refusal(1.0, 0.0, 0.0) is not None
    assert nm.landing_offset_refusal(0.0, 0.0, 0.0) is None


@pytest.mark.parametrize('bad', [(math.nan, 0.0), (0.0, math.inf), ('x', 0.0)])
def test_a_non_number_is_refused(bad):
    assert nm.landing_offset_refusal(bad[0], bad[1], 30.0) is not None
