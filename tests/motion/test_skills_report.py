"""The operator's attempt lines (jugglebot.motion.skills.report): the
throw line's fields, the refusal shortener, the start/end lines and their
severities, and the Juggle result counts taken from reports."""

from __future__ import annotations

import pytest

from jugglebot.motion.skills import report as rp
from jugglebot.motion.skills import schedule as sch


def _r(**over):
    fields = dict(throw_no=3, n_throws=5, ball_id=0, caught=True, row=True,
                  no_row_reason='', apex_m=0.9175, landing_err_mm=(-4.5, 9.4),
                  release_err_s=0.015, arrival_err_s=-0.008, seat_s=0.078,
                  reaims=1, reaim_refused='')
    fields.update(over)
    return rp.ThrowReport(**fields)


def test_the_throw_line_carries_every_observed_field():
    assert rp.throw_line(_r()) == (
        'INFO', 'throw 3/5 CAUGHT  apex 0.917 m  landed -4, +9 mm  '
                'release +15 ms  arrival -8 ms  seat +0.08 s  re-aimed 1x')


def test_a_miss_is_a_warning_and_unobserved_fields_are_left_out():
    severity, text = rp.throw_line(_r(
        caught=False, apex_m=None, landing_err_mm=None, release_err_s=None,
        arrival_err_s=None, seat_s=None, reaims=0, row=False,
        no_row_reason='no landing estimate was ever observed'))
    assert severity == 'WARN'
    assert text == ('throw 3/5 MISSED  no learner row: no landing estimate '
                    'was ever observed')


def test_a_refused_re_aim_is_named():
    _sev, text = rp.throw_line(_r(reaims=1, reaim_refused='LIMIT_JERK'))
    assert text.endswith('re-aimed 1x, then refused (LIMIT_JERK)')
    _sev, text = rp.throw_line(_r(reaims=0, reaim_refused='INFEASIBLE'))
    assert text.endswith('re-aimed 0x, then refused (INFEASIBLE)')


def test_no_observer_reads_released_not_a_verdict():
    assert rp.throw_line(_r(caught=None))[1].startswith('throw 3/5 released ')


def test_a_throw_released_on_time_reads_zero_release_error():
    """The flight the planner COMMANDS for a 0.9 m skill: release at the
    860 mm throw height, land on the 830 mm catch plane flight_s later. Its
    launch speed is the 4.166 m/s skill_node announced on the robot
    (2026-09-29 19:11) — so this is the physical flight, and an on-time
    release of it must read 0, a 15 ms late one +15 ms.

    ``z_land`` is FROZEN at the pre-catch-high plane (830). This happens to
    equal the live ``sites.CATCH_CUP_Z_MM`` again as of 2026-10-06 (CATCH
    HIGH moved it to 930 on 2026-10-05; the owner returned it to 830 the next
    day) — coincidence, not a dependency: this characterises a dated robot
    announcement, not the live geometry, and ``release_error_s`` takes
    ``z_land`` as a plain parameter — nothing here reads the constant."""
    g, z_rel, z_land = 9806.0, 860.0, 830.0
    t_flight = sch.flight_s(0.9)
    v0 = (z_land - z_rel + 0.5 * g * t_flight ** 2) / t_flight
    assert v0 == pytest.approx(4166.0, abs=1.0)
    v_land = v0 - g * t_flight
    t_rel = 100.0
    assert rp.release_error_s(t_rel + t_flight, z_land, v_land, z_rel,
                              t_rel) == pytest.approx(0.0, abs=1e-9)
    assert rp.release_error_s(t_rel + t_flight + 0.015, z_land, v_land, z_rel,
                              t_rel) == pytest.approx(0.015, abs=1e-9)
    assert rp.release_error_s(t_rel, z_land, +100.0, z_rel, t_rel) is None


@pytest.mark.parametrize('message, short', [
    ("REJECTED_CYCLE_INFEASIBLE(LIMIT_JERK: times below are on the re-gated "
     "range's own clock, which starts at knot 185 = 4.6250 s of the plan (the "
     "head before it was gated when it was built and is carried bit for "
     "bit); peak leg jerk 209271 mm/s³ > 150000)",
     'LIMIT_JERK: leg jerk 209k > 150k mm/s³'),
    ('REJECTED_CYCLE_INFEASIBLE(HAND_LIMIT_ACC: peak hand acceleration 3509.7 '
     'rev/s^2 > 3500.0 rev/s^2 [trajectory_op.hand_acc_limit_rps2])',
     'HAND_LIMIT_ACC: hand acceleration 3509.7 > 3500 rev/s^2'),
    ('REJECTED_CYCLE_INFEASIBLE(INFEASIBLE: QP infeasible: unbounded dual step '
     'admitting inequality 174 (working set size 14))',
     'INFEASIBLE: QP infeasible: unbounded dual step admitting inequality 174 '
     '(working set size 14)'),
    ('the firmware refused the streamed hand lane and is holding the hand',
     'the firmware refused the streamed hand lane and is holding the hand'),
])
def test_short_refusal(message, short):
    assert rp.short_refusal(message) == short


def test_a_long_unstructured_refusal_is_cut():
    assert len(rp.short_refusal('x' * 400)) == 90


def test_start_line():
    assert rp.start_line('self_toss', 5, 0.9, memory_rows=4,
                         frame_offset_mm=1.54) == (
        'self_toss started: 5 throws at apex 0.90 m · memory 4 rows · '
        'mocap frame offset 1.5 mm')
    assert rp.start_line('hop', 1, 0.9, extra='P1 -> P2') == (
        'hop started: 1 throw at apex 0.90 m P1 -> P2')


def test_end_lines_by_how_the_attempt_ended():
    reports = [_r(throw_no=1, apex_m=0.896, landing_err_mm=(-4.0, 2.3),
                  release_err_s=-0.012),
               _r(throw_no=2, apex_m=0.929, landing_err_mm=(30.9, -3.4),
                  release_err_s=0.030, caught=False)]
    tally = ('1/2 caught · apex 0.90–0.93 m · worst landing 31 mm · release '
             '-12 ms..+30 ms')
    assert rp.end_line('self_toss', '', '', reports) == (
        'INFO', 'self_toss done: ' + tally)
    assert rp.end_line('self_toss', 'STOPPED', '', reports) == (
        'INFO', 'self_toss stopped by the operator: ' + tally)
    assert rp.end_line(
        'self_toss', 'LIMIT_JERK',
        'REJECTED_CYCLE_INFEASIBLE(LIMIT_JERK: peak leg jerk 152393 mm/s³ > '
        '150000)', reports, end_kind='REST') == (
        'ERROR', 'self_toss ENDED (LIMIT_JERK) at the REST: leg jerk 152k > '
                 '150k mm/s³ · ' + tally)


def test_an_attempt_with_no_throws_still_ends_with_a_count():
    assert rp.end_line('reload', 'ABORTED_BB_NOT_READY', 'Ball Butler never '
                       'became ready', []) == (
        'ERROR', 'reload ENDED (ABORTED_BB_NOT_READY): Ball Butler never '
                 'became ready · 0/0 caught')


def test_the_juggle_result_counts_come_from_reports():
    assert rp.summarise([_r(), _r(caught=False), _r(caught=None)]) == (3, 1)


def test_an_end_with_no_reason_beyond_its_code_says_the_code_once():
    assert rp.end_line('hop', 'NO_LANDING', '', []) == (
        'ERROR', 'hop ENDED (NO_LANDING) · 0/0 caught')


def test_a_value_that_rounds_to_zero_prints_plus_zero():
    _sev, text = rp.throw_line(_r(landing_err_mm=(-0.3, 0.2),
                                  release_err_s=-0.0004, arrival_err_s=-0.0002,
                                  seat_s=-0.001))
    assert 'landed +0, +0 mm' in text and 'release +0 ms' in text
    assert 'arrival +0 ms' in text and 'seat +0.00 s' in text
    assert '-0' not in text
