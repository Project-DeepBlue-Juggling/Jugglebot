"""skill_node — the mocked-ROS shell smoke test (R2, Unit D2).

The pure-Python schedule/segment/executor policy is pinned by
``tests/motion/test_skills_*.py``; ``trajectory/install_segment`` itself is
pinned by ``tests/ros/test_install_segment.py``. What this file exercises is
the SHELL: ``skills/start_columns`` compiles a schedule, the 40 Hz tick
dispatches the first due skill through the (mocked) install_segment client
with the right request fields, a refusal ends the attempt, and a ``/balls``
message feeds the executor's tracker callable.
"""

from __future__ import annotations

import numpy as np
import pytest

from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import BallStateArray, HandTelemetryMessage
from jugglebot_interfaces.srv import InstallSegment

from jugglebot import ball_possession
from jugglebot import skill_node as sn

from tests.ros.conftest import MockFuture


class _RecordingClient:
    """Stand-in for the trajectory/install_segment service client."""

    def __init__(self, response=None, ready=True):
        self.calls = []
        self._response = response
        self._ready = ready

    def wait_for_service(self, timeout_sec=None):
        return self._ready

    def call_async(self, request):
        self.calls.append(request)
        future = MockFuture()
        if self._response is not None:
            future.set_result(self._response)
        return future


def _response(**overrides):
    resp = InstallSegment.Response()
    resp.accepted = True
    resp.code = 'OK'
    resp.message = 'ok'
    resp.plan_wall_ms = 12.0
    resp.splice_k = 0
    resp.t0_mono = 100.0
    resp.t_event_mono = 100.4
    resp.duration_s = 0.7
    for k, v in overrides.items():
        setattr(resp, k, v)
    return resp


def _node_with_client(response=None, ready=True):
    node = sn.SkillNode()
    client = _RecordingClient(response=response, ready=ready)
    node._install_cli = client
    return node, client


# ═════════════════════════════════════════════════════════════════════════════
# start_columns / stop
# ═════════════════════════════════════════════════════════════════════════════

def test_node_starts_idle():
    node, _client = _node_with_client()
    assert node._executor is None
    assert node._possession_evidence == ball_possession.EVIDENCE_UNKNOWN


def test_start_columns_compiles_a_schedule_at_the_owner_operating_point():
    node, _client = _node_with_client()
    resp = node._svc_start_columns(Trigger.Request(), Trigger.Response())
    assert resp.success is True, resp.message
    assert node._executor is not None
    schedule = node._executor.schedule
    assert len(schedule.skills) > 0
    assert schedule.skills[0].kind == 'THROW'


def test_a_second_start_while_running_is_refused():
    node, _client = _node_with_client()
    assert node._svc_start_columns(Trigger.Request(),
                                   Trigger.Response()).success is True
    resp2 = node._svc_start_columns(Trigger.Request(), Trigger.Response())
    assert resp2.success is False
    assert 'already running' in resp2.message


def test_stop_ends_the_attempt():
    node, _client = _node_with_client()
    node._svc_start_columns(Trigger.Request(), Trigger.Response())
    assert node._executor is not None
    resp = node._svc_stop(Trigger.Request(), Trigger.Response())
    assert resp.success is True
    assert node._executor is None


def test_stop_with_no_attempt_is_a_harmless_success():
    node, _client = _node_with_client()
    resp = node._svc_stop(Trigger.Request(), Trigger.Response())
    assert resp.success is True
    assert 'no attempt' in resp.message


# ═════════════════════════════════════════════════════════════════════════════
# the tick
# ═════════════════════════════════════════════════════════════════════════════

def test_the_tick_dispatches_the_first_throw_through_the_mocked_client():
    node, client = _node_with_client(response=_response())
    assert node._svc_start_columns(Trigger.Request(),
                                   Trigger.Response()).success is True
    first_throw = node._executor.schedule.skills[0]
    assert first_throw.kind == 'THROW'
    t_dispatch = first_throw.dispatch_s()

    node._executor.tick(t_dispatch)

    assert len(client.calls) == 1
    req = client.calls[0]
    assert req.kind == InstallSegment.Request.KIND_THROW
    assert req.ball_id == first_throw.ball_id
    assert req.t_event_s == pytest.approx(first_throw.t_abs_s)
    assert req.flight_s == pytest.approx(first_throw.y_d[1])


def test_a_refusal_ends_the_attempt():
    node, _client = _node_with_client(
        response=_response(accepted=False, code='WRONG_MODE',
                          message='not in TRAJECTORY mode'))
    node._svc_start_columns(Trigger.Request(), Trigger.Response())
    first = node._executor.schedule.skills[0]
    node._executor.tick(first.dispatch_s())
    assert node._executor.attempt_ended is True
    assert node._executor.end_code == 'WRONG_MODE'
    # `_on_tick` retires the executor once the attempt has ended.
    node._on_tick()
    assert node._executor is None


# ═════════════════════════════════════════════════════════════════════════════
# tracking
# ═════════════════════════════════════════════════════════════════════════════

def test_a_balls_message_feeds_the_tracker():
    node, _client = _node_with_client()

    class _Time:
        sec = 10
        nanosec = 500_000_000

    class _Vec:
        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z

    class _Ball:
        id = 3
        landing_position = _Vec(1.0, 2.0, 3.0)
        landing_velocity = _Vec(0.0, 0.0, -2000.0)
        time_at_land = _Time()

    node._on_balls(BallStateArray(balls=[_Ball()]))
    landing = node._tracker(3)
    assert landing is not None
    np.testing.assert_array_equal(landing.pos_mm, [1.0, 2.0, 3.0])
    np.testing.assert_array_equal(landing.vel_mm_s, [0.0, 0.0, -2000.0])
    assert landing.t_land_abs_s == pytest.approx(10.5)
    assert node._tracker(999) is None


def test_hand_telemetry_updates_possession_evidence():
    node, _client = _node_with_client()
    node._on_hand_telemetry(HandTelemetryMessage(ball_held_raw=True,
                                                 ball_held_valid=True))
    assert node._possession_evidence == ball_possession.EVIDENCE_SEATED
    node._on_hand_telemetry(HandTelemetryMessage(ball_held_raw=False,
                                                 ball_held_valid=True))
    assert node._possession_evidence == ball_possession.EVIDENCE_EMPTY
    node._on_hand_telemetry(HandTelemetryMessage(ball_held_raw=False,
                                                 ball_held_valid=False))
    assert node._possession_evidence == ball_possession.EVIDENCE_UNKNOWN


# ═════════════════════════════════════════════════════════════════════════════
# installer plumbing
# ═════════════════════════════════════════════════════════════════════════════

def test_installer_reports_service_unavailable():
    node, _client = _node_with_client(ready=False)
    node._svc_start_columns(Trigger.Request(), Trigger.Response())
    first = node._executor.schedule.skills[0]
    node._executor.tick(first.dispatch_s())
    assert node._executor.attempt_ended is True
    assert node._executor.end_code == 'SERVICE_UNAVAILABLE'


def test_a_catch_with_throw_terminal_copies_the_carried_release_onto_the_request():
    """A CATCH whose terminal carries ``then_throw`` (Unit G's fold) must reach
    the wire with ``t_release_s`` / ``release_site_mm`` set from it — the SAME
    generic ``target_mm`` / ``flight_s`` fields a plain THROW uses, reused
    here for the carried release (``InstallSegment.srv``'s one pair of fields,
    not a second copy per kind)."""
    from jugglebot.motion.skills.segments import CatchTerminal, ThrowAfterCatch

    node, client = _node_with_client(response=_response())
    then_throw = ThrowAfterCatch(t_release_s=101.0, site_mm=[0.0, 0.0, 860.0],
                                 target_mm=[10.0, -5.0, 860.0], flight_s=0.6)
    terminal = CatchTerminal(landing_mm=[0.0, 0.0, 830.0],
                             landing_vel_mm_s=[0.0, 0.0, -2500.0],
                             t_land_s=100.0, rest_site_mm=[0.0, 0.0, 750.0],
                             then_throw=then_throw)
    node._installer('CATCH', terminal, 100.0, ball_id=1)
    assert len(client.calls) == 1
    req = client.calls[0]
    assert req.t_event_s == pytest.approx(100.0)
    assert req.t_release_s == pytest.approx(101.0)
    assert list(req.release_site_mm) == pytest.approx([0.0, 0.0, 860.0])
    assert list(req.target_mm) == pytest.approx([10.0, -5.0, 860.0])
    assert req.flight_s == pytest.approx(0.6)


def test_a_plain_catch_terminal_leaves_the_release_fields_at_the_sentinel():
    node, client = _node_with_client(response=_response())
    from jugglebot.motion.skills.segments import CatchTerminal

    terminal = CatchTerminal(landing_mm=[0.0, 0.0, 830.0],
                             landing_vel_mm_s=[0.0, 0.0, -2500.0],
                             t_land_s=100.0, rest_site_mm=[0.0, 0.0, 750.0])
    node._installer('CATCH', terminal, 100.0, ball_id=1)
    req = client.calls[0]
    assert req.t_release_s == 0.0


def test_the_installer_propagates_seeded_post_release_from_the_response():
    node, client = _node_with_client(
        response=_response(seeded_post_release=True))
    from jugglebot.motion.skills.segments import RestTerminal

    terminal = RestTerminal(rest_site_mm=[0.0, 0.0, 750.0], t_rest_s=100.0)
    result = node._installer('REST', terminal, 100.0, ball_id=0)
    assert result.seeded_post_release is True
