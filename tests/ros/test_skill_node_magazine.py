"""skill_node `magazine` program (U2c): refusal ladder, the install, the miss report."""

from __future__ import annotations

import numpy as np
import pytest

from types import SimpleNamespace

from jugglebot.motion.skills import magazine_target as mt
from tests.ros.test_skill_node import (_freshen, _goal, _node_with_client,
                                       _prelevel_ready, _response)

import jugglebot.skill_node as sn

NOW = 1000.0


def _yaml(tmp_path, target=(550.0, 300.0, 750.0), flight=0.9):
    p = tmp_path / 'bb_magazine.yaml'
    p.write_text('target_base_mm: %s\nflight_s: %s\n' % (list(target), flight))
    return str(p)


def _ready(tmp_path, **kw):
    node, client = _node_with_client(response=_response(), **kw)
    node._now_s = lambda: NOW
    _prelevel_ready(node)
    _freshen(node)
    c, s = np.cos(0.3), np.sin(0.3)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    node._bb_base_pose = (np.array([2000.0, -500.0, 100.0]), R, NOW - 5.0)
    return node, client, R


def _param(node, name, value):
    from tests.ros.conftest import _MockParameter
    node.set_parameters([_MockParameter(value, name=name)])


def _setup(tmp_path, **kw):
    node, client, R = _ready(tmp_path, **kw)
    _param(node, 'magazine_config_path', _yaml(tmp_path))
    return node, client, R


def test_happy_path_installs_a_magazine_terminal_at_the_transformed_target(tmp_path):
    node, client, R = _setup(tmp_path)
    calls = []
    node._installer = lambda kind, term, t, *, ball_id: (
        calls.append((kind, term)) or sn.InstallResult(True, 'OK', 'ok', 0.0))
    res = node._start_pattern(_goal('magazine'))
    assert res.success, res.message
    (kind, term), = calls
    assert kind == sn.MAGAZINE
    want = mt.base_to_plan_mm([550, 300, 750], R, [2000, -500, 100], (0, 0))
    np.testing.assert_allclose(term.target_mm, want)
    assert term.flight_s == pytest.approx(0.9)
    assert term.t_release_s == pytest.approx(NOW + 1.3)
    assert term.release_site_mm[2] == sn.RELEASE_CUP_Z_MM
    assert node._magazine_ctx is not None
    assert not node._goal_done_event.is_set()


def test_frame_offset_is_subtracted_like_the_tracker_landings(tmp_path):
    node, client, R = _setup(tmp_path)
    node._mocap_to_schedule_mm = (0.0, 0.0)
    calls = []
    node._installer = lambda kind, term, t, *, ball_id: (
        calls.append(term) or sn.InstallResult(True, 'OK', 'ok', 0.0))
    node._frame_check_error = lambda: ''
    node._mocap_to_schedule_mm = (12.0, -7.0)
    assert node._start_pattern(_goal('magazine')).success
    want = mt.base_to_plan_mm([550, 300, 750], R, [2000, -500, 100], (12.0, -7.0))
    np.testing.assert_allclose(calls[0].target_mm, want)


@pytest.mark.parametrize('pose', [None, 'old'])
def test_stale_base_frame_refuses(tmp_path, pose):
    node, client, R = _setup(tmp_path)
    node._bb_base_pose = None if pose is None else (np.zeros(3), R, NOW - 500.0)
    res = node._start_pattern(_goal('magazine'))
    assert not res.success and 'MAGAZINE_BASE_FRAME_STALE' in res.message
    assert client.calls == []


def test_bad_config_refuses_with_the_config_message(tmp_path):
    node, client, R = _setup(tmp_path)
    _param(node, 'magazine_config_path', _yaml(tmp_path, flight=9.0))
    res = node._start_pattern(_goal('magazine'))
    assert not res.success and 'MAGAZINE_CONFIG' in res.message


def test_no_ball_refuses(tmp_path):
    node, client, R = _setup(tmp_path)
    _freshen(node, seated=False)
    res = node._start_pattern(_goal('magazine'))
    assert not res.success and 'REJECTED' in res.message
    assert client.calls == []


def test_run_state_refusal_applies_to_magazine(tmp_path):
    node, client, R = _setup(tmp_path)
    node._executor = object()
    res = node._start_pattern(_goal('magazine'))
    assert not res.success and 'already running' in res.message


def test_installer_refusal_reaches_the_operator_verbatim(tmp_path):
    node, client, R = _setup(tmp_path)
    node._installer = lambda kind, term, t, *, ball_id: sn.InstallResult(
        False, 'WORKSPACE', 'leg 3 exceeds stroke at knot 9', 0.0)
    res = node._start_pattern(_goal('magazine'))
    assert not res.success
    assert 'WORKSPACE' in res.message and 'leg 3 exceeds stroke at knot 9' in res.message
    assert node._magazine_ctx is None


def _ctx_with_samples(node, R, noise=0.0, n=40, miss_base=(30.0, -20.0), dt=0.0):
    origin = np.array([2000.0, -500.0, 100.0])
    tgt_base = np.array([550.0, 300.0, 750.0])
    p_plan = mt.base_to_plan_mm(tgt_base, R, origin, (0, 0))
    flight = 0.9
    t_rel = NOW
    g = 9810.0
    aim = mt.base_to_plan_mm(tgt_base + np.array([*miss_base, 0.0]), R, origin, (0, 0))
    tc = t_rel + flight + dt
    rel = aim - 0  # crossing point
    rng = np.random.RandomState(1)
    samples = []
    ts = np.linspace(t_rel + 0.05, tc - 0.02, n)
    v_down = 2500.0                      # downward speed at crossing
    for t in ts:
        tau = tc - t
        z = p_plan[2] + v_down * tau - 0.5 * g * tau * tau
        x = rel[0] + 100.0 * (t - tc)
        y = rel[1] - 50.0 * (t - tc)
        samples.append((t, x + noise * rng.randn(), y + noise * rng.randn(),
                        z + noise * rng.randn()))
    cfg = mt.MagazineConfig(target_base_mm=tgt_base, flight_s=flight, source_path='')
    node._magazine_ctx = dict(cfg=cfg, R=R, origin=origin, off_xy=(0.0, 0.0),
                              p_plan=p_plan, t_rel=t_rel, t_end=t_rel + 1.3,
                              samples={7: samples}, preexisting=frozenset(),
                              collecting=False, timer=None)
    return node._magazine_ctx


def test_miss_report_recovers_the_injected_miss(tmp_path):
    node, client, R = _setup(tmp_path)
    ctx = _ctx_with_samples(node, R, noise=1.0, dt=0.012)
    ok, text, fields = node._magazine_miss(ctx)
    assert ok, text
    assert float(fields['dx_mm']) == pytest.approx(30.0, abs=6.0)
    assert float(fields['dy_mm']) == pytest.approx(-20.0, abs=6.0)
    assert float(fields['dt_ms']) == pytest.approx(12.0, abs=6.0)
    assert 'MAGAZINE miss: dx=' in text and 'elbow frame' in text


def test_finish_ends_the_goal_thrown(tmp_path):
    node, client, R = _setup(tmp_path)
    _ctx_with_samples(node, R)
    node._magazine_finish(node._magazine_ctx)
    assert node._goal_end_code == 'THROWN'
    assert node._goal_done_event.is_set()
    assert node._magazine_ctx is None
    assert len(node._goal_reports) == 1 and 'MAGAZINE miss' in node._goal_outcome_lines[0]


def test_too_few_samples_is_no_track(tmp_path):
    node, client, R = _setup(tmp_path)
    ctx = _ctx_with_samples(node, R, n=3)
    ok, text, _ = node._magazine_miss(ctx)
    assert not ok and text.startswith('NO_TRACK')
    ctx['samples'] = {}
    ok, text, _ = node._magazine_miss(ctx)
    assert not ok and text.startswith('NO_TRACK')
