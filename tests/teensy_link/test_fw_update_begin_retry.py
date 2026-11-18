"""FwUpdate BEGIN retry (IDEA-20 P3b): a Platform in an IMU fault-reboot loop is alive only about 10 s of
every 18.5 s (ros_ws/docs/platform_imu_protocol.md section 4), so the uploader must keep sending BEGIN through
silence for longer than one cycle. Ball Butler keeps its single BEGIN. Time is injected."""

from __future__ import annotations

import struct

from teensy_link import RpcMethod, rpc_args


class _Waiter:
    """Silent until ``answer_at`` waits have happened; each wait advances the fake clock by the 1 s timeout."""

    def __init__(self, clock, answer_at=None):
        self.clock, self.answer_at, self.waits = clock, answer_at, 0

    def clear(self, can_id):
        pass

    def wait(self, can_id, expected_dlc, timeout):
        self.waits += 1
        self.clock[0] += timeout
        if self.answer_at is not None and self.waits >= self.answer_at:
            return struct.pack('<BBHI', 0x01, rpc_args.PLATFORM_FW_STATUS_OK, 0, 0)
        return None


class _Rpc:
    def __init__(self):
        self.calls = []

    def call(self, method, payload=b"", **kw):
        self.calls.append(method)


def _clock():
    t = [1000.0]
    return t, (lambda: t[0])


def test_platform_begin_retries_through_silence_for_longer_than_a_reboot_cycle():
    import tools.teensy_link_bridge as tlb
    t, now = _clock()
    waiter, rpc = _Waiter(t), _Rpc()
    assert tlb._begin_session(rpc, waiter, tlb._FW_TARGETS["platform"], 100000, monotonic=now) is False
    assert t[0] - 1000.0 >= 20.0 and len(rpc.calls) >= 20           # at least 20 one-second attempts
    assert set(rpc.calls) == {int(RpcMethod.PLATFORM_FW_BEGIN)}
    assert tlb._FW_BEGIN_RETRY_S > 18.5 + 1.0                        # outlasts one full fault-reboot cycle


def test_platform_begin_succeeds_the_moment_the_board_answers():
    import tools.teensy_link_bridge as tlb
    t, now = _clock()
    waiter, rpc = _Waiter(t, answer_at=14), _Rpc()
    assert tlb._begin_session(rpc, waiter, tlb._FW_TARGETS["platform"], 100000, monotonic=now) is True
    assert len(rpc.calls) == 14


def test_platform_begin_refusal_is_not_retried_and_bb_keeps_one_begin():
    import tools.teensy_link_bridge as tlb
    t, now = _clock()
    busy = struct.pack('<BBHI', 0x01, rpc_args.PLATFORM_FW_STATUS_BUSY, 0, 0)

    class _Busy(_Waiter):
        def wait(self, *a, **k):
            return busy
    rpc = _Rpc()
    assert tlb._begin_session(rpc, _Busy(t), tlb._FW_TARGETS["platform"], 10, monotonic=now) is False
    assert len(rpc.calls) == 1
    rpc2 = _Rpc()
    assert tlb._begin_session(rpc2, _Waiter(t), tlb._FW_TARGETS["bb"], 10, monotonic=now) is False
    assert len(rpc2.calls) == 1 and tlb._FW_TARGETS["bb"].begin_retry_s == 0.0
