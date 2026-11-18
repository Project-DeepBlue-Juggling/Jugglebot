"""The automatic Platform IMU upload trigger on teensy_bridge_node (IDEA-20 P3c): skills/attempt events and a
Fault edge turn into attempt_ended calls with the right span, and nothing in it can raise or wait."""
from __future__ import annotations

import types

from tests.ros._bridge_harness import _build_paired_node, _teardown


def _ev(name):
    return types.SimpleNamespace(name=name)


def _node_with_spy():
    teensy, client, node = _build_paired_node()
    calls = []
    node.attempt_ended = lambda t0, t1, reason='attempt_end', attempt_id=None: calls.append((t0, t1, reason)) or True
    node.abort_upload = lambda reason='abort': calls.append(('abort', reason))
    return teensy, client, node, calls


def test_attempt_end_requests_the_span_with_margins():
    teensy, client, node, calls = _node_with_spy()
    try:
        node._imu_on_attempt_event(_ev('juggle_start'), 1000.0)
        node._imu_on_attempt_event(_ev('juggle_end'), 1006.0)
        assert calls == [(998.0, 1009.0, 'attempt_end')]
    finally:
        _teardown(teensy, client, node)


def test_a_long_attempt_asks_for_the_last_32_s():
    teensy, client, node, calls = _node_with_spy()
    try:
        node._imu_on_attempt_event(_ev('juggle_start'), 1000.0)
        node._imu_on_attempt_event(_ev('juggle_end'), 1100.0)
        assert calls == [(1071.0, 1103.0, 'attempt_end')]
    finally:
        _teardown(teensy, client, node)


def test_a_fault_mid_attempt_uploads_once_with_reason_fault():
    teensy, client, node, calls = _node_with_spy()
    try:
        node._imu_on_attempt_event(_ev('juggle_start'), 1000.0)
        node._imu_on_fault(1004.0)
        node._imu_on_attempt_event(_ev('juggle_end'), 1004.5)       # skill_node's own end after the Fault
        assert calls == [(998.0, 1007.0, 'fault')]
    finally:
        _teardown(teensy, client, node)


def test_a_refused_or_unrelated_event_requests_nothing():
    teensy, client, node, calls = _node_with_spy()
    try:
        node._imu_on_attempt_event(_ev('juggle_refused'), 1.0)
        node._imu_on_attempt_event(_ev('MAGAZINE_MISS'), 2.0)
        node._imu_on_fault(3.0)                                      # no Attempt open, no upload running
        assert calls == []
    finally:
        _teardown(teensy, client, node)


def test_an_attempt_start_aborts_a_running_upload():
    teensy, client, node, calls = _node_with_spy()
    try:
        node.upload_in_progress = lambda: True
        node._imu_on_attempt_event(_ev('juggle_start'), 5.0)
        assert calls == [('abort', 'attempt_start')]
    finally:
        _teardown(teensy, client, node)


def test_a_fault_between_attempts_aborts_a_running_upload():
    teensy, client, node, calls = _node_with_spy()
    try:
        node.upload_in_progress = lambda: True
        node._imu_on_fault(5.0)
        assert calls == [('abort', 'fault')]
    finally:
        _teardown(teensy, client, node)


def test_exceptions_are_swallowed_and_logged_never_raised():
    teensy, client, node, calls = _node_with_spy()
    try:
        def boom(*a, **k):
            raise RuntimeError('service dead')
        node.attempt_ended = boom
        node._imu_on_attempt_event(_ev('juggle_start'), 1.0)
        node._imu_on_attempt_event(_ev('juggle_end'), 2.0)           # must not raise
        node._imu_on_attempt_event(None, 3.0)                        # a malformed event must not raise
        node._imu_span.started(1.0)
        node._imu_on_fault(2.0)
    finally:
        _teardown(teensy, client, node)


def test_toggle_off_sends_nothing_to_the_platform_and_no_imu_is_not_an_error():
    teensy, client, node = _build_paired_node()
    try:
        node.set_imu_upload_after_attempt(False)
        node._imu_on_attempt_event(_ev('juggle_start'), 1.0)
        node._imu_on_attempt_event(_ev('juggle_end'), 2.0)
        assert node.upload_in_progress() is False                    # attempt_ended returned False: ring never frozen
        node.set_imu_upload_after_attempt(True)
        node._imu_on_attempt_event(_ev('juggle_start'), 3.0)
        node._imu_on_attempt_event(_ev('juggle_end'), 4.0)           # no IMU_STATUS: skipped, not raised
        assert node.upload_in_progress() is False
    finally:
        _teardown(teensy, client, node)


def test_toggle_off_mid_upload_aborts_it():
    teensy, client, node, calls = _node_with_spy()
    try:
        node.upload_in_progress = lambda: True
        res = node._svc_imu_set_upload_after_attempt(types.SimpleNamespace(data=False), types.SimpleNamespace())
        assert res.success and calls == [('abort', 'toggle_off')]
        calls.clear()
        node._svc_imu_set_upload_after_attempt(types.SimpleNamespace(data=True), types.SimpleNamespace())
        assert calls == []
    finally:
        _teardown(teensy, client, node)
