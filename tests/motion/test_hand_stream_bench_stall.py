"""Offline cover for ``hand_stream_bench.py --stage stall`` (BUG-123, can-bridge FW 47).

The stage COMMANDS THE HAND MOTOR (it pushes the bench Hand into its bottom stop until the
FW 47 lane stall guard IDLEs it) and lives in tests/hardware; its sequence, abort, restores
and verdict live in ``tests/hardware/hand_stall_stage.py`` and run here against a fake
link with a virtual clock and a toy plant: the Hand follows the command down to the stop,
the current then grows with the command's overshoot up to the drive's limit, and the fake
guard (when enabled) IDLEs the Hand after 150 ms at >= 40 A.
"""
from __future__ import annotations

import os
import re
import sys

import pytest

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_HW = os.path.join(_REPO, "tests", "hardware")
if _HW not in sys.path:
    sys.path.insert(0, _HW)

import hand_stall_stage as ST  # noqa: E402

STOP_HOMED = -0.10


class FakeIO:
    def __init__(self, *, fw=47, guard=True, clamp_gain=200.0, orig_limit=70.0,
                 limit_takes=True, park=0.5, raise_on_send=None, raise_on_sample_after=None,
                 second_rise=False, bit3_before=False, temp=30.0):
        self.t = 0.0
        self.fw, self.guard, self.k = fw, guard, clamp_gain
        self.limit = orig_limit
        self.vel_limit = 1000.0
        self.limit_takes = limit_takes
        self.offset = 0.0            # lane frame = homed + offset
        self.pos_h = park            # homed frame
        self.state = ST.AXIS_IDLE
        self.cmd = None
        self.fault = ST.FAULT_NONE
        self.flags = ST.DIAG_FLAG_LANE_STALL if bit3_before else 0
        self.iq = 0.0
        self.t40 = None
        self.trip_t = None
        self.armed = False
        self.log = []
        self.sends = 0
        self.samples = 0
        self.raise_on_send = raise_on_send
        self.raise_on_sample_after = raise_on_sample_after
        self.second_rise = second_rise
        self.temp = temp

    # plant
    def _step(self):
        if self.state == ST.AXIS_CLOSED_LOOP and self.cmd is not None:
            c = self.cmd - self.offset                     # homed frame
            self.pos_h = max(c, STOP_HOMED)
            over = max(0.0, STOP_HOMED - c)
            self.iq = min(self.limit, self.k * over)
            if self.iq >= ST.GUARD_IQ_A:
                if self.t40 is None:
                    self.t40 = self.t
                if self.guard and self.t - self.t40 >= ST.GUARD_HOLD_S:
                    self.state, self.iq = ST.AXIS_IDLE, 0.0
                    self.fault, self.flags = ST.FAULT_HAND_STALL, ST.DIAG_FLAG_LANE_STALL
                    self.trip_t = self.t
            else:
                self.t40 = None
        elif self.state == ST.AXIS_IDLE:
            self.iq = 0.0
            if self.second_rise and self.trip_t is not None and self.t - self.trip_t > 0.3:
                self.iq = 20.0

    # reads
    def preflight(self):
        return dict(fw=self.fw, fault=self.fault, fault_name="NONE", axis_state=self.state,
                    active_errors=0, flags=self.flags, temp_motor=self.temp,
                    pos=self.pos_h + self.offset)

    def read_limits(self):
        self.log.append(("read_limits", self.t, self.limit))
        return self.vel_limit, self.limit

    def pos(self):
        return self.pos_h + self.offset

    def sample(self):
        self.samples += 1
        if self.raise_on_sample_after and self.samples > self.raise_on_sample_after:
            raise KeyboardInterrupt
        self._step()
        return ST.Sample(pos=self.pos(), vel=0.0, telem_age=0.005, iq_meas=self.iq,
                         iq_set=self.iq, axis_state=self.state, flags=self.flags,
                         fault=self.fault, temp_motor=self.temp)

    def confirm(self, text):
        return True

    def clock(self):
        return self.t

    def sleep(self, s):
        self.t += s

    # writes
    def set_limits(self, vel, curr):
        self.log.append(("set_limits", self.t, curr, self.state))
        if self.limit_takes or curr > 60:
            self.limit, self.vel_limit = curr, vel

    def set_abs_pos(self, p):
        self.log.append(("set_abs_pos", self.t, p))
        self.offset = p - self.pos_h

    def idle_once(self, timeout):
        self.log.append(("idle", self.t))
        self.state, self.cmd = ST.AXIS_IDLE, None
        self.t += 0.002
        return True

    def bring_up(self):
        self.log.append(("bring_up", self.t))
        self.state = ST.AXIS_CLOSED_LOOP
        self.cmd = self.pos()
        return True, None

    def arm(self, hold_pos):
        self.log.append(("arm", self.t))
        self.armed = True
        return True, None

    def send(self, traj, t):
        self.sends += 1
        if self.raise_on_send and self.sends > self.raise_on_send:
            raise RuntimeError("link died")
        assert self.armed
        self.cmd = traj.sample(t)[0]

    def disarm(self):
        self.log.append(("disarm", self.t))
        self.armed = False

    def hand_move_probe(self, target):
        self.log.append(("probe", self.t, target))
        return "refused: ERR_BUS_DOWN" if self.fault != ST.FAULT_NONE else "accepted (answered OK)"


def _names(io):
    return [x[0] for x in io.log]


def _run(io, **kw):
    lines = []
    rc = ST.run_stall(io, ST.StallConfig(**kw), say=lines.append)
    return rc, lines


def _limit_writes(io):
    return [x[2] for x in io.log if x[0] == "set_limits"]


# ── the pass path ───────────────────────────────────────────────────────────

def test_pass_path_times_the_trip_and_restores_everything():
    io = FakeIO()
    rc, lines = _run(io)
    assert rc == 0, lines
    verdict = lines[-1]
    m = re.fullmatch(r"STALL-GUARD PASS: IDLE (\d+) ms after the first 40 A sample "
                     r"\(limit 200\), peak ([\d.]+) A, fault HAND_STALL, bit3 set", verdict)
    assert m, verdict
    # the fake guard trips 150 ms after its first 40 A sample; the stage sees it within a poll
    assert 150 <= int(m.group(1)) <= 150 + 2 * ST.POLL_S * 1e3
    assert float(m.group(2)) == pytest.approx(45.0)
    assert _limit_writes(io) == [45.0, 70.0]
    assert io.limit == 70.0
    # the frame went out by the configured shift and came back to the homed frame
    assert io.offset == pytest.approx(0.0, abs=1e-9)
    assert io.pos() == pytest.approx(STOP_HOMED, abs=1e-6)
    # the refusal probe ran while latched, after the trip
    assert "probe" in _names(io)
    assert any("refused: ERR_BUS_DOWN" in ln for ln in lines)


def test_the_limit_is_restored_only_after_the_hand_is_idle():
    io = FakeIO()
    _run(io)
    restore = [x for x in io.log if x[0] == "set_limits"][-1]
    assert restore[2] == 70.0 and restore[3] == ST.AXIS_IDLE


def test_shift_puts_the_ramp_end_past_the_stop():
    cfg = ST.StallConfig()
    assert cfg.shift_rev == pytest.approx(0.65)
    # the ramp ends at RAMP_END_REV in the lane frame = the homed target, 0.5 rev past the stop
    assert ST.RAMP_END_REV - cfg.shift_rev == pytest.approx(cfg.target_rev)
    assert cfg.target_rev - STOP_HOMED == pytest.approx(-0.5)
    assert ST.RAMP_END_REV > 0.0      # inside the lane clip [0, 10.501]


# ── refusals ────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("fw", [46, 36, None])
def test_firmware_before_47_is_refused_before_anything(fw):
    io = FakeIO(fw=fw)
    rc, lines = _run(io)
    assert rc == 2
    assert not [n for n in _names(io) if n != "read_limits"]
    assert ST.stall_fw_refusal(fw) is not None
    assert ST.stall_fw_refusal(47) is None and ST.stall_fw_refusal(48) is None


def test_preflight_reports_every_refusal_at_once():
    pre = dict(fw=46, fault=4, fault_name="MAX_DEVIATION", axis_state=8, active_errors=1,
               flags=0, temp_motor=55.0, pos=1.2)
    out = ST.preflight_refusals(pre, ST.StallConfig())
    assert len(out) == 6, out


@pytest.mark.parametrize("a,ok", [(40.9, False), (41.0, True), (45.0, True), (50.0, True),
                                  (50.1, False)])
def test_current_limit_range(a, ok):
    assert (not ST.config_refusals(a, -0.6, 1.0)) is ok


def test_limit_lowering_that_does_not_take_stops_before_energising():
    io = FakeIO(limit_takes=False)
    rc, lines = _run(io)
    assert rc == 1
    assert "bring_up" not in _names(io) and "set_abs_pos" not in _names(io)
    assert _limit_writes(io) == [45.0, 70.0]
    assert "STALL-GUARD FAIL" in lines[-1]


# ── the independent abort ───────────────────────────────────────────────────

def test_abort_fires_600_ms_after_the_first_40_A_sample_and_commands_idle():
    io = FakeIO(guard=False)
    rc, lines = _run(io)
    assert rc == 1
    idles = [x[1] for x in io.log if x[0] == "idle"]
    assert idles, io.log
    at_clamp = idles[0] - io.t40
    assert ST.ABORT_AFTER_40_S <= at_clamp <= ST.ABORT_AFTER_40_S + 2 * ST.POLL_S
    assert at_clamp <= ST.CLAMP_BOUND_S
    assert _limit_writes(io) == [45.0, 70.0] and io.limit == 70.0
    assert lines[-1].startswith("STALL-GUARD FAIL: no trip 600 ms after the first 40 A sample")


def test_clamp_bound_is_the_sum_and_under_750_ms():
    s = (ST.OBS_LAG_S + ST.ABORT_AFTER_40_S + ST.POLL_S
         + ST.IDLE_ATTEMPTS * ST.IDLE_RPC_TIMEOUT_S + ST.JITTER_S)
    assert ST.CLAMP_BOUND_S == pytest.approx(s)
    assert ST.CLAMP_BOUND_S <= 0.75


def test_no_clamp_abort_when_the_current_never_reaches_40_A():
    io = FakeIO(guard=False, clamp_gain=50.0)     # 0.5 rev past the stop -> 25 A
    rc, lines = _run(io)
    assert rc == 1
    assert io.t40 is None
    assert "idle" in _names(io)
    assert "current never reached 40 A" in lines[-1]
    assert io.limit == 70.0


# ── restores on every path ──────────────────────────────────────────────────

def test_exception_mid_ramp_idles_then_restores_and_reraises():
    io = FakeIO(raise_on_send=5)
    with pytest.raises(RuntimeError):
        _run(io)
    names = _names(io)
    assert names.index("idle") < max(i for i, n in enumerate(names) if n == "set_limits")
    assert io.limit == 70.0
    assert io.offset == pytest.approx(0.0, abs=1e-9)


def test_keyboard_interrupt_idles_then_restores_and_reraises():
    io = FakeIO(raise_on_sample_after=40)
    lines = []
    with pytest.raises(KeyboardInterrupt):
        ST.run_stall(io, ST.StallConfig(), say=lines.append)
    names = _names(io)
    assert "idle" in names
    assert _limit_writes(io) == [45.0, 70.0]
    assert lines[-1].startswith("STALL-GUARD FAIL")
    assert "interrupted" in lines[-1]
    # the fake keeps raising (a mashed Ctrl-C): the frame restore fails LOUDLY, the
    # verdict still prints, and it tells the operator to re-home
    assert any("re-home" in ln for ln in lines)
    assert names.index("idle") < max(i for i, n in enumerate(names) if n == "set_limits")


def test_second_current_rise_after_the_trip_fails():
    io = FakeIO(second_rise=True)
    rc, lines = _run(io)
    assert rc == 1
    assert "current rose again after the trip" in lines[-1]


def test_sticky_bit3_from_an_earlier_trip_does_not_fake_a_trip():
    io = FakeIO(bit3_before=True)
    rc, lines = _run(io)
    assert rc == 0, lines
    assert lines[-1].endswith("bit3 set (already set before this run)")


# ── the profiled ramp ───────────────────────────────────────────────────────

@pytest.mark.parametrize("p0,p1,v", [(1.15, 0.05, 1.0), (0.95, 0.05, 2.0), (0.06, 0.05, 1.0)])
def test_ramp_is_profiled_with_no_step(p0, p1, v):
    r = ST.StallRamp(p0, p1, v)
    dt = 1e-4
    n = int((r.duration + 0.1) / dt)
    prev = r.sample(0.0)
    assert prev == (pytest.approx(p0), 0.0)
    for i in range(1, n):
        cur = r.sample(i * dt)
        assert abs(cur[0] - prev[0]) <= v * dt + 1e-9
        assert abs(cur[1] - prev[1]) <= ST.RAMP_ACCEL_RPS2 * dt + 1e-9
        assert abs(cur[1]) <= v + 1e-9
        prev = cur
    assert r.sample(r.duration + 1.0) == (pytest.approx(p1), 0.0)


# ── verdict format ──────────────────────────────────────────────────────────

def test_verdict_line_format():
    r = ST.StallResult(t_first40=1.0, t_trip=1.163, peak_a=45.2, fault_end=8,
                       fault_name="HAND_STALL", bit3=True, limit_orig=70.0,
                       limit_readback=70.0, limit_ok=True, probe="refused: ERR_BUS_DOWN")
    ok, line = ST.stall_verdict(r)
    assert ok
    assert line == ("STALL-GUARD PASS: IDLE 163 ms after the first 40 A sample (limit 200), "
                    "peak 45.2 A, fault HAND_STALL, bit3 set")
    r.t_trip = 1.25
    ok, line = ST.stall_verdict(r)
    assert not ok and line.startswith("STALL-GUARD FAIL: IDLE 250 ms after the first 40 A")


# ── mirrored constants against their sources ────────────────────────────────

def _cfg_h():
    p = os.path.join(_REPO, "ros_ws", "src", "jugglebot", "Teensy_code_canbridge",
                     "canbridge_config.h")
    with open(p) as f:
        return f.read()


def test_mirrored_guard_constants_match_the_firmware():
    src = _cfg_h()
    assert re.search(r"HAND_LANE_STALL_IQ_A\s*=\s*40\.0f", src)
    assert re.search(r"HAND_LANE_STALL_VEL_RPS\s*=\s*1\.0f", src)
    assert re.search(r"HAND_LANE_STALL_HOLD_US\s*=\s*150000ull", src)
    assert (ST.GUARD_IQ_A, ST.GUARD_VEL_RPS, ST.GUARD_HOLD_S) == (40.0, 1.0, 0.150)
    fw = int(re.search(r"FW_VERSION\s*=\s*(\d+)", src).group(1))
    assert fw >= ST.STALL_FW_MIN
    # the 0.5 rev overshoot is inside both hand guards, so the lane guard is what acts
    dev = float(re.search(r"MAX_DEVIATION_HAND_REV\s*=\s*([\d.]+)f", src).group(1))
    lead = float(re.search(r"MAX_LEAD_HAND_REV\s*=\s*([\d.]+)f", src).group(1))
    over = STOP_HOMED - ST.TARGET_MIN_REV
    assert over < lead < dev


def test_mirrored_wire_and_config_constants():
    sys.path.insert(0, _REPO)
    sys.path.insert(0, os.path.join(_REPO, "ros_ws", "src", "jugglebot"))
    from teensy_link import FaultState
    from teensy_link.activate import DIAG_FLAG_LANE_STALL
    import jugglebot.hardware_config as hw
    assert int(FaultState.HAND_STALL) == ST.FAULT_HAND_STALL
    assert DIAG_FLAG_LANE_STALL == ST.DIAG_FLAG_LANE_STALL
    assert hw.HOMING_HAND_ABS_POS_REV == pytest.approx(ST.HOME_REF_REV)


def test_bench_dispatches_the_stall_stage():
    with open(os.path.join(_HW, "hand_stream_bench.py")) as f:
        src = f.read()
    assert '"stroke_cut", "stall"]' in src
    assert "return _stall_main(args, rpc, client)" in src
    # the bridge path sets the limit; USB only reads it
    body = src[src.index("class _StallBenchIO"):]
    assert "RpcMethod.SET_VEL_CURR_LIMITS" in body
    assert "current_soft_max =" not in body
