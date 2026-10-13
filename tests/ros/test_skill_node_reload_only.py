"""skill_node ``reload`` program (2026-10-13): the GUI's plain "Reload" button.

Ball Butler feeds one ball, Jugglebot catches it and settles back to the
level rest at P1 with it seated — nothing thrown. The feed half of the
magazine -> BB -> JB -> magazine tuning loop (`test_skill_node_magazine.py`
covers the other half). It reuses the self-toss reload's choreography end to
end (bridge REST, bb/reload, bb/throw_at_target, the announced
`compile_reload`) with a zero-throw pattern, so these pin only what differs.
"""
from __future__ import annotations

from unittest.mock import patch

import pytest

from jugglebot_interfaces.srv import InstallSegment
from tests.ros.test_skill_node import (_bb_announcement, _bb_ready, _finish_bridge,
                                       _freshen, _goal, _good_box_path,
                                       _node_with_client, _prelevel_ready,
                                       _reload_ready, _response)

import jugglebot.skill_node as sn


def _start(node, tmp_path):
    node._params['plant_id'] = 'test_reload_only_' + str(id(node))
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    _prelevel_ready(node)
    reload_client, throw_client = _reload_ready(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_goal('reload'))
    return resp, reload_client, throw_client


def test_reload_only_runs_the_bb_round_trip_with_a_zero_throw_pattern(tmp_path):
    node, client = _node_with_client(response=_response())
    resp, reload_client, throw_client = _start(node, tmp_path)
    assert resp.success is True, resp.message
    # It IS a Ball Butler feed whatever the goal's flag said (the GUI sends
    # the bare token) -- recorded on the attempt, not doubled on the label.
    assert resp.attempt['pattern'] == 'reload' and resp.attempt['reload'] is True
    assert node._attempt_label == 'reload'
    ctx = node._reload_ctx
    assert ctx is not None and ctx.kind == 'reload' and ctx.phase == 'await_bb_idle'
    assert ctx.pattern.n_throws == 0 and ctx.site.name == 'P1'
    assert len(reload_client.calls) == 1 and throw_client.calls == []
    bridge = node._executor.schedule.skills
    assert [s.kind for s in bridge] == ['REST'] and bridge[0].holds_ball is False
    node._executor.tick(bridge[0].dispatch_s())
    assert client.calls[0].kind == InstallSegment.Request.KIND_REST
    _bb_ready(node)
    node._maybe_fire_reload_throw()
    assert len(throw_client.calls) == 1
    req = throw_client.calls[0]
    assert req.target_point_global_mm.x == pytest.approx(-50.0)
    assert req.target_point_global_mm.z == pytest.approx(sn.CATCH_CUP_Z_MM)


def test_reload_only_announced_schedule_ends_at_the_decay_rest(tmp_path):
    node, _client = _node_with_client(response=_response())
    resp, _, _ = _start(node, tmp_path)
    assert resp.success is True, resp.message
    _bb_ready(node)
    node._maybe_fire_reload_throw()
    _finish_bridge(node)
    assert node._executor is None
    ann = _bb_announcement(
        target_id=node._robot_name, landing_mm=(-50.0, 0.0, sn.CATCH_CUP_Z_MM),
        landing_vel_mm_s=(1200.0, 0.0, -2500.0),
        throw_time_s=2.0, landing_time_s=2.7)
    node._on_announcement(ann)
    assert node._reload_ctx is None and node._executor is not None
    skills = node._executor.schedule.skills
    assert [s.kind for s in skills] == ['REST', 'CATCH', 'REST']
    assert node._executor.n_throws == 0
    assert skills[1].landing_prior is not None and skills[1].hold_tilt is not None
    decay = skills[-1]
    assert decay.holds_ball is True and decay.rest_tilt == (0.0, 0.0)
    assert decay.site.name == 'P1'


def test_reload_only_skips_the_throw_only_box_coverage_check(tmp_path):
    """Nothing is thrown, so an apex the committed box does not cover (which
    refuses a self-toss reload before any motion) does not refuse this."""
    node, _client = _node_with_client(response=_response())
    node._params['apex_m'] = 0.3
    resp, _, _ = _start(node, tmp_path)
    assert resp.success is True, resp.message
    node2, _client2 = _node_with_client(response=_response())
    node2._params['apex_m'] = 0.3
    node2._params['plant_id'] = 'test_reload_only_contrast_' + str(id(node2))
    _freshen(node2, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    _prelevel_ready(node2)
    _reload_ready(node2)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp2 = node2._start_pattern(_goal('self_toss', reload=True))
    assert resp2.success is False and 'admissible box' in resp2.message
