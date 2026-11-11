"""The bench Hand frame tool's decisions (tests/hardware/bench_hand_frame_bench.py).

The tool commands the motor, so what is pinned here is what stops it: the guarded home's
abort rules and stall detection against a scripted drive, that the home restores the
drive's mode and limits on every exit, and the two gates' verdicts. The drive itself is
never touched.
"""
from __future__ import annotations

import os
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "hardware"))
import bench_hand_frame_bench as bhf  # noqa: E402


class FakeAxis:
    def __init__(self):
        self.controller = SimpleNamespace(
            input_vel=0.0, input_torque=0.0,
            config=SimpleNamespace(control_mode=3, input_mode=1, vel_limit=1000.0,
                                   enable_torque_mode_vel_limit=True))
        self.config = SimpleNamespace(motor=SimpleNamespace(current_soft_max=70.0))
        self.active_errors = 0
        self.pos_estimate = 2.0
        self.current_state = bhf.IDLE
        self.abs_pos = None
        self.states = []

    @property
    def requested_state(self):
        return None

    @requested_state.setter
    def requested_state(self, s):
        self.states.append(s)
        self.current_state = s

    def set_abs_pos(self, x):
        self.abs_pos = x


def scripted(axis, rows):
    it = iter(rows)

    def sample():
        iq, vel, pos = next(it)
        axis.pos_estimate = pos
        return iq, vel, pos, axis.active_errors, axis.current_state
    return sample


def assert_restored(a):
    assert a.current_state == bhf.IDLE
    assert (a.controller.config.control_mode, a.controller.config.input_mode) == (3, 1)
    assert a.controller.config.vel_limit == 1000.0 and a.config.motor.current_soft_max == 70.0
    assert a.controller.input_vel == 0.0


def test_home_stalls_on_the_iq_ema_and_sets_the_reference():
    a = FakeAxis()
    rows = [(-1.0, -3.0, 2.0 - 0.03 * i) for i in range(10)] + [(-11.0, 0.0, 1.7)] * 10
    ok, msg = bhf.run_home(SimpleNamespace(axis0=a), -1.0, -0.1, sample=scripted(a, rows))
    assert ok and "STALL" in msg and a.abs_pos == -0.1
    assert_restored(a)


@pytest.mark.parametrize("rows, why", [
    ([(0.0, +0.6, 2.0)], "WRONG WAY"),
    ([(0.0, -6.5, 2.0)], "OVERSPEED"),
    ([(0.0, -3.0, 2.0 - 12.5)], "TRAVEL"),
])
def test_home_aborts_to_idle_without_setting_a_reference(rows, why):
    a = FakeAxis()
    ok, msg = bhf.run_home(SimpleNamespace(axis0=a), -1.0, -0.1, sample=scripted(a, rows))
    assert not ok and why in msg and a.abs_pos is None
    assert_restored(a)


def test_home_aborts_on_a_drive_error():
    a = FakeAxis()

    def sample():
        a.active_errors = 0x200
        return 0.0, -3.0, 2.0, a.active_errors, a.current_state
    ok, msg = bhf.run_home(SimpleNamespace(axis0=a), -1.0, -0.1, sample=sample)
    assert not ok and "error" in msg and a.abs_pos is None
    assert_restored(a)


def test_wrong_way_threshold_is_signed_by_the_down_direction():
    assert bhf.home_abort_reason(+1.0, -0.6, 0.0, 0, bhf.CLOSED_LOOP, 0.1).startswith("WRONG WAY")
    assert bhf.home_abort_reason(+1.0, +3.0, 0.5, 0, bhf.CLOSED_LOOP, 0.1) is None
    assert bhf.home_abort_reason(-1.0, -3.0, -0.5, 0, bhf.CLOSED_LOOP, 0.1) is None
    assert "TIMEOUT" in bhf.home_abort_reason(-1.0, 0.0, 0.0, 0, bhf.CLOSED_LOOP, bhf.HOME_TIMEOUT_S + 0.1)


def test_the_ema_reaches_the_detect_threshold_only_on_a_sustained_stall():
    ema = 0.0
    for _ in range(3):
        ema = bhf.ema_step(ema, 11.0)
    assert abs(ema) < bhf.HOME_DETECT_A           # a 30 ms spike does not stall
    for _ in range(5):
        ema = bhf.ema_step(ema, 11.0)
    assert abs(ema) >= bhf.HOME_DETECT_A


def test_push_verdict():
    assert bhf.push_verdict(-3.0)[0]
    assert not bhf.push_verdict(+3.0)[0]                 # a mirrored frame
    assert not bhf.push_verdict(-0.2)[0]                 # too small to judge
    assert bhf.push_verdict(+3.0, expect_down_sign=+1.0)[0]


def test_torque_pulse_stops_and_verdicts():
    assert bhf.pulse_stop_reason(0.0, -0.3, +1, 0.05) == "wrong"
    assert bhf.pulse_stop_reason(0.31, 0.5, +1, 0.05) == "moved"
    assert bhf.pulse_stop_reason(0.05, 2.1, +1, 0.05) == "moved"
    assert bhf.pulse_stop_reason(0.0, 0.0, +1, bhf.TC_PULSE_S) == "time"
    assert bhf.pulse_stop_reason(0.1, 0.5, +1, 0.1) is None
    assert bhf.pulse_verdict(+1, 0.8, 0.3) and bhf.pulse_verdict(-1, -0.8, -0.3)
    assert not bhf.pulse_verdict(+1, -0.8, -0.3)          # moved against the torque
    assert not bhf.pulse_verdict(+1, 0.1, 0.01)           # did not move
