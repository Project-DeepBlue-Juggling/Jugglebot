"""`catch_resend_max` (2026-09-20): the re-send cap is a ROS parameter.

A bound on a COST (solves one committed catch may spend re-aiming from later
fits), not a policy -- so it is clamped to 0..5 with a WARN rather than
refused, and 0 disables re-aiming outright: the A/B knob for a sitting after
2026-09-18 16:16, where 18 of 21 timing-only re-sends were refused LIMIT_JERK.
"""
from __future__ import annotations

import pytest

from tests.ros.conftest import _MockParameter
from tests.ros.test_skill_node import _columns_goal, _node_with_client


def test_the_default_cap_reaches_the_executor():
    node, _client = _node_with_client()
    assert node.get_parameter('catch_resend_max').value == 2
    node._start_pattern(_columns_goal())
    assert node._executor.resend_max_per_catch == 2


@pytest.mark.parametrize('value, expect', [(0, 0), (5, 5), (9, 5), (-1, 0)])
def test_the_cap_is_clamped_to_zero_to_five(value, expect):
    node, _client = _node_with_client()
    node.set_parameters([_MockParameter(value, name='catch_resend_max')])
    node._start_pattern(_columns_goal())
    assert node._executor.resend_max_per_catch == expect
