"""P3a: skill_node `_checker` (the check_only client) and the shared
`_segment_request` builder `_installer` now goes through.

Mocked ROS (tests/ros/conftest.py): a canned `InstallSegment.Response` stands
in for trajectory_node; the trajectory side is pinned in test_install_segment.
"""

from __future__ import annotations

import math

import pytest

from jugglebot.motion.skills import admissible as adm
from jugglebot.motion.skills.segments import (CatchTerminal, RestTerminal,
                                              ThrowAfterCatch, ThrowTerminal)
import jugglebot.skill_node as sn
from jugglebot_interfaces.srv import InstallSegment
from tests.ros.test_skill_node import _node_with_client, _response

_K = InstallSegment.Request


def _throw():
    return 'THROW', ThrowTerminal(site_mm=[1.0, 2.0, 860.0],
                                  target_mm=[10.0, -5.0, 830.0], flight_s=0.6,
                                  t_release_s=100.0)


def _catch():
    return 'CATCH', CatchTerminal(
        landing_mm=[3.0, 4.0, 830.0], landing_vel_mm_s=[5.0, 6.0, -2500.0],
        t_land_s=100.0, rest_site_mm=[0.0, 0.0, 750.0])


def _catch_throw():
    kind, t = _catch()
    t = CatchTerminal(
        landing_mm=t.landing_mm, landing_vel_mm_s=t.landing_vel_mm_s,
        t_land_s=t.t_land_s, rest_site_mm=t.rest_site_mm,
        then_throw=ThrowAfterCatch(t_release_s=101.0, site_mm=[0.0, 0.0, 860.0],
                                   target_mm=[10.0, -5.0, 860.0], flight_s=0.5))
    return kind, t


def _rest():
    return 'REST', RestTerminal(rest_site_mm=[0.0, 0.0, 750.0], t_rest_s=102.0)


def _verdict(node, **resp_kw):
    if resp_kw.get('accepted') and 'message' not in resp_kw:
        resp_kw = dict(resp_kw, message='checked CATCH from a fresh origin: '
                                        'margin 0.250')
    node._install_cli._response = _response(**resp_kw)
    kind, term = _catch_throw()
    return node._checker(kind, term, 100.0, ball_id=1)


# ── _checker mapping table ────────────────────────────────────────────────

@pytest.mark.parametrize('resp_kw, ok, code, evaluated', [
    (dict(accepted=True, code='OK', margin=0.25), True, 'OK', True),
    (dict(accepted=True, code='OK', margin=0.0), True, 'OK', True),
    (dict(accepted=True, code='OK', margin=-0.01), False, 'MARGIN', True),
    (dict(accepted=True, code='OK', margin=math.nan), False, 'MARGIN', True),
    (dict(accepted=False, code='LIMIT_JERK', margin=-0.3), False,
     'LIMIT_JERK', True),
    (dict(accepted=False, code='HAND_STROKE', margin=math.nan), False,
     'HAND_STROKE', True),
    (dict(accepted=False, code='SOLVER_UNAVAILABLE'), False,
     'SOLVER_UNAVAILABLE', False),
    (dict(accepted=False, code='SKIPPED_LATE'), False, 'SKIPPED_LATE', False),
    (dict(accepted=False, code='GUARD_LATCHED'), False, 'GUARD_LATCHED', False),
    (dict(accepted=False, code='WRONG_MODE'), False, 'WRONG_MODE', False),
    (dict(accepted=False, code='STALE_STATE'), False, 'STALE_STATE', False),
    (dict(accepted=False, code='PLANNER_MODE'), False, 'PLANNER_MODE', False),
    (dict(accepted=False, code='WINDOW_TOO_SHORT'), False, 'WINDOW_TOO_SHORT',
     False),
    (dict(accepted=False, code='MAGAZINE_NEEDS_REST'), False,
     'MAGAZINE_NEEDS_REST', False),
    (dict(accepted=False, code='MAGAZINE_NOT_CHECKABLE'), False,
     'MAGAZINE_NOT_CHECKABLE', False),
    # version skew: an accepted reply that is not a CHECK reply (an older
    # trajectory_node ignored check_only and may have installed)
    (dict(accepted=True, code='OK', margin=0.25, message='installed'), False,
     'CHECK_NOT_HONOURED', False),
])
def test_checker_maps_the_reply_to_a_verdict(resp_kw, ok, code, evaluated):
    node, client = _node_with_client(response=_response())
    v = _verdict(node, **resp_kw)
    assert isinstance(v, adm.CommandVerdict)
    assert (v.ok, v.code, v.evaluated) == (ok, code, evaluated)
    assert client.calls[-1].check_only is True


def test_checker_service_unavailable_is_not_evaluated():
    node, _ = _node_with_client(response=_response(), ready=False)
    kind, term = _throw()
    v = node._checker(kind, term, 100.0, ball_id=0)
    assert (v.ok, v.code, v.evaluated) == (False, 'SERVICE_UNAVAILABLE', False)
    assert math.isnan(v.margin)


def test_checker_timeout_is_not_evaluated():
    node, _ = _node_with_client(response=_response())
    node._wait_future = lambda future, *a, **k: None
    kind, term = _throw()
    v = node._checker(kind, term, 100.0, ball_id=0)
    assert (v.ok, v.code, v.evaluated) == (False, 'SERVICE_TIMEOUT', False)
    assert math.isnan(v.margin)


def test_the_debug_forced_catch_refusal_does_not_apply_to_a_check():
    node, client = _node_with_client(response=_response(
        margin=0.5,
        message='checked CATCH from a fresh origin: margin 0.500'))
    node._consume_debug_force_catch_refusal = lambda: True
    kind, term = _catch()
    assert node._installer(kind, term, 100.0, ball_id=0).code == 'CATCH_AXIS'
    assert client.calls == []
    v = node._checker(kind, term, 100.0, ball_id=0)
    assert v.evaluated is True and v.ok is True
    assert len(client.calls) == 1


# ── _installer builds the same request it always did ──────────────────────

def _nan_tag(x):
    # NaN != NaN; `_wire_tilt_out` writes NaN beside a false *_set flag.
    return 'nan' if isinstance(x, float) and math.isnan(x) else x


def _fields(req):
    return {k: ([_nan_tag(e) for e in v] if isinstance(v, (list, tuple))
                else _nan_tag(v))
            for k, v in vars(req).items()}


def _expected(**over):
    base = dict(
        kind=0, ball_id=7, t_event_s=0.0, site_mm=[0.0, 0.0, 0.0],
        target_mm=[0.0, 0.0, 0.0], flight_s=0.0,
        landing_vel_mm_s=[0.0, 0.0, 0.0], rest_site_mm=[0.0, 0.0, 0.0],
        t_release_s=0.0, release_site_mm=[0.0, 0.0, 0.0],
        hold_tilt_set=False, hold_tilt_rad=[0.0, 0.0],
        receive_tilt_set=False, receive_tilt_rad=[0.0, 0.0],
        rest_tilt_set=False, rest_tilt_rad=[0.0, 0.0],
        bump_delta_xy_mm=[0.0, 0.0], t_dispatch_s=0.0,
        planner_mode=None, carry_zvd=None, check_only=False)
    base.update(over)
    return base


@pytest.mark.parametrize('make, over', [
    (_throw, dict(kind=_K.KIND_THROW, t_event_s=100.0,
                  site_mm=[1.0, 2.0, 860.0], target_mm=[10.0, -5.0, 830.0],
                  flight_s=0.6)),
    (_catch, dict(kind=_K.KIND_CATCH, t_event_s=100.0,
                  site_mm=[3.0, 4.0, 830.0],
                  landing_vel_mm_s=[5.0, 6.0, -2500.0],
                  rest_site_mm=[0.0, 0.0, 750.0])),
    (_catch_throw, dict(kind=_K.KIND_CATCH, t_event_s=100.0,
                        site_mm=[3.0, 4.0, 830.0],
                        landing_vel_mm_s=[5.0, 6.0, -2500.0],
                        rest_site_mm=[0.0, 0.0, 750.0], t_release_s=101.0,
                        release_site_mm=[0.0, 0.0, 860.0],
                        target_mm=[10.0, -5.0, 860.0], flight_s=0.5)),
    (_rest, dict(kind=_K.KIND_REST, t_event_s=102.0,
                 rest_site_mm=[0.0, 0.0, 750.0])),
])
def test_installer_request_is_field_for_field_what_it_was(make, over):
    node, client = _node_with_client(response=_response())
    kind, term = make()
    node._installer(kind, term, 100.0, ball_id=7)
    exp = _expected(planner_mode=node._planner_mode(),
                    carry_zvd=node._carry_zvd(), **over)
    nan2 = ['nan', 'nan']
    if make in (_catch, _catch_throw):
        exp.update(hold_tilt_rad=nan2, receive_tilt_rad=nan2)
    elif make is _rest:
        exp.update(rest_tilt_rad=nan2)
    got = _fields(client.calls[-1])
    assert got == exp
    # the check request is the same request with only the flag raised
    node._checker(kind, term, 100.0, ball_id=7)
    chk = _fields(client.calls[-1])
    assert chk.pop('check_only') is True
    exp.pop('check_only')
    assert chk == exp
