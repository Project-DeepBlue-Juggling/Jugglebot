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

    def take(self, can_id, expected_dlc, timeout):        # BEGIN reads arrival order (BUG-147)
        return self.wait(can_id, expected_dlc, timeout)


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


# BUG-147: BEGIN and COMMIT take only a reply carrying their own opcode.

class _Script:
    """take() hands back scripted frames in order; None (silence) once empty. Each call advances the fake clock."""

    def __init__(self, clock, frames, step=0.0):
        self.clock, self.frames, self.step, self.takes = clock, list(frames), step, 0

    def clear(self, can_id):
        pass

    def take(self, can_id, expected_dlc, timeout):
        self.takes += 1
        self.clock[0] += self.step if self.frames else timeout
        return self.frames.pop(0) if self.frames else None

    wait = take


def _frame(op, status=rpc_args.PLATFORM_FW_STATUS_OK):
    return struct.pack('<BBHI', op, status, 0, 0)


def test_begin_ignores_a_wrong_opcode_frame_before_its_own_ok():
    import tools.teensy_link_bridge as tlb
    t, now = _clock()
    # an INFO-opcode BUSY, then a DATA BUSY: either read as BEGIN's verdict would be a refusal
    waiter = _Script(t, [_frame(0x05, rpc_args.PLATFORM_FW_STATUS_BUSY),
                         _frame(0x02, rpc_args.PLATFORM_FW_STATUS_BUSY), _frame(0x01)])
    rpc = _Rpc()
    assert tlb._begin_session(rpc, waiter, tlb._FW_TARGETS["bb"], 100, monotonic=now) is True
    assert len(rpc.calls) == 1 and waiter.takes == 3


class _QueueWaiter:
    """Arrival-order queue; clear() empties it, take() pops the oldest (real PlatformFrameWaiter semantics)."""

    def __init__(self):
        self.q = []

    def clear(self, can_id):
        self.q.clear()

    def take(self, can_id, expected_dlc, timeout):
        return self.q.pop(0) if self.q else None

    wait = take


class _AnsweringRpc:
    """Answers each fw RPC by queueing the board's reply; COMMIT is preceded by a wrong-opcode frame."""

    def __init__(self, waiter, junk_before_commit):
        self.waiter, self.junk, self.methods = waiter, junk_before_commit, []

    def call(self, method, payload=b"", **kw):
        self.methods.append(int(method))
        q = self.waiter.q
        if int(method) == int(RpcMethod.PLATFORM_FW_BEGIN):
            q.append(_frame(0x01))
        elif int(method) == int(RpcMethod.PLATFORM_FW_DATA):
            q.append(struct.pack('<BBHI', 0x02, rpc_args.PLATFORM_FW_STATUS_OK, 0, 1))
        elif int(method) == int(RpcMethod.PLATFORM_FW_VERIFY):
            q.append(_frame(0x03))
        elif int(method) == int(RpcMethod.PLATFORM_FW_COMMIT):
            if self.junk:
                q.append(_frame(0x05, rpc_args.PLATFORM_FW_STATUS_BUSY))   # unsolicited, wrong opcode
            q.append(_frame(0x04))

    send_nowait = None


def test_commit_ignores_a_wrong_opcode_frame_before_its_own_ok():
    import binascii
    import tools.teensy_link_bridge as tlb
    image = b"\xA5"
    crc = binascii.crc32(image) & 0xFFFFFFFF
    for junk in (False, True):
        waiter = _QueueWaiter()
        rpc = _AnsweringRpc(waiter, junk)
        rc = tlb._run_fw_update_session(rpc, waiter, 0x6F1, image, crc, commit=True,
                                        target=tlb._FW_TARGETS["platform"])
        assert rc == 0, junk                 # the BUSY junk frame was not read as the COMMIT verdict
        assert rpc.methods[-1] == int(RpcMethod.PLATFORM_FW_COMMIT)


def test_only_wrong_opcode_frames_time_out_like_silence_within_the_deadline():
    import tools.teensy_link_bridge as tlb
    t, now = _clock()
    # a wrong-opcode frame every 0.4 s forever: must not extend the 1.0 s overall deadline
    waiter = _Script(t, [_frame(0x05)] * 50, step=0.4)
    assert tlb._wait_for_op_reply(waiter, 0x7D7, 0x01, 1.0, monotonic=now) is None
    assert t[0] - 1000.0 <= 1.0 + 0.4 and waiter.takes <= 4
