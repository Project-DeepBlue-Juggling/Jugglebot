"""PWAY-11 P2 audit findings 1-8 on the node (the policy / tracker halves are
``tests/motion/test_skills_human_catch_bounded.py`` and
``tests/tracking/test_human_throw_event.py``)."""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import patch

from jugglebot import skill_node as sn
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import human_catch as hcp
from tests.ros.test_skill_node_human_catch import (
    _EMPTY, _SEATED, _assert_a_new_goal_is_accepted, _catch_land, _ctx,
    _event, _fill_cup, _hc_node, _lines, _phase, _plan_one, _rest_to_rest,
    _retire, _start, _texts, _tick, _tick_until)


def test_ended_code_for_swap_reads_the_executor_once():
    """Finding 1: the executor nulled by the tick thread between two reads."""
    live = SimpleNamespace(attempt_ended=True, end_code='X')
    reads = []

    class Stub:
        _goal_end_code = ''

        @property
        def _executor(self):
            reads.append(1)
            return live if len(reads) == 1 else None

    assert sn.SkillNode._ended_code_for_swap(Stub()) == 'X'
    assert len(reads) == 1


def test_a_refusal_after_the_pre_lock_check_ends_once_with_one_hold():
    """Finding 2: the counter moves AFTER `_check_reload_lane_refused` and
    before the live CATCH's own observation in the same tick."""
    node, client, clock, _cup = _hc_node()
    _start(node, clock)
    t_plan = clock.t
    _plan_one(node, clock, client)
    L = _catch_land(node)
    _fill_cup(node, t_plan - 0.05, L + 2.0, seated_from=L + 0.02)
    orig = node._check_reload_lane_refused

    def _check_then_move():
        orig()
        node._sched_refused_at_start = 3
        node._sched_refused = 4
        node._sched_refused_mono = sn.time.perf_counter()
    node._check_reload_lane_refused = _check_then_move
    with patch.object(sn.rp, 'end_line', wraps=sn.rp.end_line) as end_line, \
            patch.object(node, '_end_attempt',
                         wraps=node._end_attempt) as end_attempt:
        _tick_until(node, clock, L + 1.2)
    assert end_line.call_count == 1
    assert end_line.call_args.args[1] == ex.LANE_REFUSED
    # the live executor owns the refusal: no second `_end_attempt` (which
    # would force a second hold)
    assert end_attempt.call_count == 0
    assert node._goal_end_code == ex.LANE_REFUSED and node._reload_ctx is None


def test_an_unschedulable_return_ends_once_inside_the_lock(tmp_path):
    """Finding 3: no stale RETURNING phase for another tick to plan from."""
    node, client, clock, cup = _hc_node()
    _start(node, clock)
    _plan_one(node, clock, client)
    n_calls = len(client.calls)
    with patch.object(node, '_opening_rest_period',
                      return_value=(0.0, 'no lift period')), \
            patch.object(sn.rp, 'end_line', wraps=sn.rp.end_line) as end_line:
        _retire(node, clock, ex.MISSED_CATCH)
        assert node._reload_ctx is None            # decided in the lock
        assert node._goal_end_code == sn.HC_RETURN_UNSCHEDULABLE
        for _ in range(10):
            _tick(node, clock, clock.t + 0.025)
        _event(node, clock, lead=0.60, seq=5)
        _tick(node, clock, clock.t + 0.025)
    assert end_line.call_count == 1
    assert end_line.call_args.args[1] == sn.HC_RETURN_UNSCHEDULABLE
    assert len(client.calls) == n_calls            # no further plan
    assert node._goal_done_event.is_set() and node._executor is None
    _assert_a_new_goal_is_accepted(node, cup, tmp_path)


def test_a_plan_that_raises_ends_the_attempt_and_frees_the_node(tmp_path):
    """Finding 4: the claimed context is released, not left in CATCHING."""
    node, client, clock, cup = _hc_node()
    _start(node, clock)
    log = _lines(node)
    with patch.object(node, '_build_hc_executor',
                      side_effect=RuntimeError('boom')), \
            patch.object(sn.rp, 'end_line', wraps=sn.rp.end_line) as end_line:
        _event(node, clock, lead=0.60)
        _tick(node, clock, clock.t + 0.005)
        for _ in range(5):
            _tick(node, clock, clock.t + 0.025)
    assert end_line.call_count == 1
    assert end_line.call_args.args[1] == sn.HC_PLAN_ERROR
    assert any('boom' in t for t in _texts(log))
    assert node._reload_ctx is None and node._executor is None
    assert node._goal_end_code == sn.HC_PLAN_ERROR
    assert node._goal_done_event.is_set()
    _assert_a_new_goal_is_accepted(node, cup, tmp_path)


def test_the_throw_count_is_compiled_plans_not_claims():
    """Finding 5: one claim the compile refused does not count."""
    node, client, clock, _cup = _hc_node()
    _start(node, clock, n=2)
    _event(node, clock, lead=0.56, seq=0)
    _tick(node, clock, clock.t + 0.30)             # too short: refused
    assert _phase(node) == hcp.WAITING
    assert _ctx(node).policy.claimed == 1 and node._goal_human_claimed == 0
    _plan_one(node, clock, client, seq=1)
    _retire(node, clock, ex.MISSED_CATCH)
    _rest_to_rest(node, clock)
    assert node._reload_ctx is not None            # 1 compiled of 2: goes on
    assert not node._goal_done_event.is_set()
    _plan_one(node, clock, client, seq=2)
    _retire(node, clock, ex.MISSED_CATCH)
    _rest_to_rest(node, clock)
    assert node._goal_human_claimed == 2
    assert node._reload_ctx is None and node._goal_done_event.is_set()


def test_holding_logs_one_visibility_line_every_five_seconds():
    """Finding 7: a Ball never removed stays visible; no timeout."""
    node, client, clock, cup = _hc_node()
    _start(node, clock)
    _plan_one(node, clock, client)
    cup[0] = _SEATED
    log = _lines(node)
    _retire(node, clock, '')
    assert _phase(node) == hcp.HOLDING
    t0 = clock.t
    _tick_until(node, clock, t0 + 12.0, dt=0.5)
    held = [t for t in _texts(log) if 'HUMAN-CATCH-HOLDING' in t]
    assert len(held) == 2, held
    assert held[0].startswith('HUMAN-CATCH-HOLDING cup=%s for 5 s' % _SEATED)
    assert _phase(node) == hcp.HOLDING and node._reload_ctx is not None


def test_stop_between_executors_says_the_last_rest_completes():
    """Finding 8."""
    node, client, clock, _cup = _hc_node()
    _start(node, clock)
    log = _lines(node)
    node._stop_attempt()
    text = ' '.join(_texts(log))
    assert 'completes to rest' in text and 'nothing was moving' not in text
