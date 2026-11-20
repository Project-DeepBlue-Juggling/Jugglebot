"""hand_stream_bench: the triangle stage's automated cut (BUG-105) and the
stroke_cut stage's open-ended --repeat (FEAT-45), 2026-11-20.

The automated cut must put on the wire exactly what a Ctrl-C at the same
frame puts there: the cut frame is the last SETPOINT, the only call after it
is the heartbeat flags going to 0 (DISARMED), then rpc.close() and
client.stop(); no RPC, no further frame. Both paths are driven through the
REAL main() with a recording fake link, and their tails after the last frame
are compared. The driver lives in tests/hardware/ (kept out of collection),
so it goes on sys.path explicitly, the test_hand_stream_bench_gap.py idiom.
"""
from __future__ import annotations

import contextlib
import csv
import io
import os
import re
import sys
import types

import pytest

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_HW_DIR = os.path.join(_REPO, 'tests', 'hardware')
if _HW_DIR not in sys.path:
    sys.path.insert(0, _HW_DIR)

import hand_stream_bench as B  # noqa: E402
from teensy_link import rpc_args  # noqa: E402
from teensy_link.protocol import Setpoint  # noqa: E402

POS_GAIN = float(B.hw.ODRIVE_HAND_POS_GAIN)


# ---------------------------------------------------------------------------
# A recording fake link, driving the real main()
# ---------------------------------------------------------------------------

def _install_fakes(monkeypatch, events, start_rev, kbi_after_armed_sends=None):
    state = {'armed_sends': 0, 'flags': 0}

    class FakeClient:
        def __init__(self, *a, **kw):
            pass

        def start(self):
            pass

        def subscribe(self, *a, **kw):
            return lambda: None

        def start_heartbeat(self, hz=10.0, flags=0):
            state['flags'] = flags

        def set_heartbeat_flags(self, flags):
            events.append(('flags', int(flags)))
            state['flags'] = int(flags)
            with B._lock:
                B._cache['hb'] = _hb(0x9 if flags & 1 else 0x1)

        def send_stream(self, mt, payload):
            events.append(('send', bytes(payload)))
            if state['flags'] & 1:
                state['armed_sends'] += 1
                if (kbi_after_armed_sends is not None
                        and state['armed_sends'] == kbi_after_armed_sends):
                    raise KeyboardInterrupt       # Ctrl-C right after this send

        def stop(self, timeout=1.0):
            events.append(('stop',))

    class FakeRpc:
        def __init__(self, *a, **kw):
            pass

        def call(self, method, *a, **kw):
            events.append(('rpc', int(method)))

        def close(self):
            events.append(('rpc_close',))

    monkeypatch.setattr(B, 'TeensyLinkClient', FakeClient)
    monkeypatch.setattr(B, 'RpcServer', lambda *a, **kw: object())
    monkeypatch.setattr(B, 'TimeOfDayServer', lambda *a, **kw: object())
    monkeypatch.setattr(B, 'RpcClient', FakeRpc)
    monkeypatch.setattr(B, 'TRI_CUT_SETTLE_S', 0.05)
    monkeypatch.setattr(B, 'TRI_CUT_DRIFT_S', 0.05)
    tm = types.SimpleNamespace(pos_rev=[0.0] * 6 + [start_rev], vel_rps=[0.0] * 7)
    # The belt and the settled read see a fresh, static encoder at start_rev.
    monkeypatch.setattr(B, '_telem_with_age', lambda: (tm, 0.0))
    with B._lock:
        B._cache.update({
            'telem': tm, 'telem_mono': 0.0, 'hb': _hb(0x1), 'echo': None,
            'ident': types.SimpleNamespace(
                fw_version=rpc_args.EXPECTED_BRIDGE_FW_VERSION),
            'diag': {B.HAND: types.SimpleNamespace(
                axis_id=B.HAND, axis_state=B.CLOSED_LOOP, active_errors=0,
                temp_motor=0.0, temp_fet=0.0)},
            'cd_prev': None, 'cd_rows': [], 'rec': None})
    return state


def _hb(flags):
    return types.SimpleNamespace(flags=flags, fault_state=0, lead_clamp_mask=0,
                                 max_dev_leg=0xFF, max_dev_value=0.0,
                                 max_dev_u0=0.0, max_dev_enc=0.0)


def _main(argv):
    out = io.StringIO()
    old = sys.argv
    sys.argv = ['hand_stream_bench.py'] + argv
    try:
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
            try:
                return B.main(), out.getvalue()
            except KeyboardInterrupt:
                return 'KeyboardInterrupt', out.getvalue()
    finally:
        sys.argv = old


def _tail_after_last_send(events):
    last = max(i for i, e in enumerate(events) if e[0] == 'send')
    return events[last], events[last + 1:]


START = 3.0
TRI = ['--stage', 'triangle', '--tri-span', '0.4', '--tri-speed', '1.0', '--arm']


@pytest.mark.parametrize('direction', ['up', 'down'])
def test_the_cut_lands_mid_ramp_and_prints_the_last_command(direction, monkeypatch, tmp_path):
    events = []
    _install_fakes(monkeypatch, events, START)
    rc, out = _main(TRI + ['--tri-cut', direction, '--tri-cut-cycle', '1',
                           '--csv', str(tmp_path / 'cut.csv')])
    assert rc == 0, out
    last, tail = _tail_after_last_send(events)
    sp = Setpoint.unpack(last[1])
    # mid-ramp of cycle 1: up at P/4 (0.2 s), down at 3P/4 (0.6 s); the cut is
    # the first tick at or after it, so within a couple of knots of p_mid.
    p_mid = START + 0.2
    sign = 1.0 if direction == 'up' else -1.0
    assert sp.v0[B.HAND] == pytest.approx(sign * 1.0)
    assert 0.0 <= sign * (sp.u0[B.HAND] - p_mid) <= 3 * B.SEG_T * 1.0 + 1e-6
    # the printed last command is the frame that was sent
    m = re.search(r"knot ([+-][0-9.]+) rev at ([+-][0-9.]+) rev/s", out)
    assert m, out
    assert float(m.group(1)) == pytest.approx(sp.u0[B.HAND], abs=1e-4)
    assert float(m.group(2)) == pytest.approx(sp.v0[B.HAND], abs=1e-2)
    assert f"tri-cut: {direction} ramp, cycle 1" in out
    assert "settled position: +3.0000 rev" in out
    # nothing after the cut frame but the disarm and the shutdown
    assert tail == [('flags', 0), ('rpc_close',), ('stop',)]


def test_the_cut_sends_exactly_what_ctrl_c_sends(monkeypatch, tmp_path):
    events_cut = []
    _install_fakes(monkeypatch, events_cut, START)
    rc, out = _main(TRI + ['--tri-cut', 'up', '--tri-cut-cycle', '1',
                           '--csv', str(tmp_path / 'cut.csv')])
    assert rc == 0, out
    i_arm = events_cut.index(('flags', 1))
    n_armed = sum(1 for e in events_cut[i_arm:] if e[0] == 'send')

    events_kbi = []
    _install_fakes(monkeypatch, events_kbi, START, kbi_after_armed_sends=n_armed)
    rc2, out2 = _main(TRI + ['--csv', str(tmp_path / 'kbi.csv')])
    assert rc2 == 'KeyboardInterrupt', out2
    _, tail_cut = _tail_after_last_send(events_cut)
    _, tail_kbi = _tail_after_last_send(events_kbi)
    assert tail_cut == tail_kbi == [('flags', 0), ('rpc_close',), ('stop',)]
    # the same number of frames went out after the arm, and no RPC at all
    i_arm2 = events_kbi.index(('flags', 1))
    assert sum(1 for e in events_kbi[i_arm2:] if e[0] == 'send') == n_armed
    assert not [e for e in events_cut + events_kbi if e[0] == 'rpc']


def test_the_report_uses_the_firmware_echo_when_there_is_one(monkeypatch, tmp_path):
    events = []
    _install_fakes(monkeypatch, events, START)
    with B._lock:
        B._cache['echo'] = (0, START - 0.3, 10.0)   # settled sits vel_ff/35 above it
    rc, out = _main(TRI + ['--tri-cut', 'up', '--tri-cut-cycle', '1',
                           '--csv', str(tmp_path / 'cut.csv')])
    assert rc == 0, out
    assert "input_pos +2.7000 rev, vel_ff +10.00 rev/s" in out
    assert "offset vs the firmware's last command: +0.3000" in out
    assert f"expected vel_ff/{POS_GAIN:g} = {10.0 / POS_GAIN:+.4f}" in out


def test_a_down_cut_near_the_bottom_stop_is_refused_before_anything_is_commanded(
        monkeypatch, tmp_path):
    events = []
    _install_fakes(monkeypatch, events, 0.5)
    rc, out = _main(['--stage', 'triangle', '--tri-span', '4', '--tri-speed', '20',
                     '--tri-cut', 'down', '--close-loop', '--arm',
                     '--csv', str(tmp_path / 'x.csv')])
    assert rc == 2
    assert "--tri-cut refused" in out and "nothing was commanded" in out
    assert "bottom stop" in out
    assert not [e for e in events if e[0] in ('rpc', 'send')]
    assert ('flags', 1) not in events


def test_plan_tri_cut_refusal_matrix():
    # the Sitting's commands: span 4 cuts up at 10 and 20 rev/s from a 0.3-0.7
    # park; the 20 rev/s down cut needs span 6
    for park in (0.3, 0.5, 0.7):
        for v in (10.0, 20.0):
            assert B.plan_tri_cut(park, 4, v, 'up', 2, POS_GAIN, 10.4)[1] is None
        assert B.plan_tri_cut(park, 6, 20.0, 'down', 2, POS_GAIN, 10.4)[1] is None
        assert B.plan_tri_cut(park, 4, 20.0, 'down', 2, POS_GAIN, 10.4)[1] is not None
    # the up side and the triangle top respect --top-stop
    assert B.plan_tri_cut(0.5, 4, 20.0, 'up', 2, POS_GAIN, 5.5)[1] is not None
    assert "triangle itself" in B.plan_tri_cut(0.5, 6, 10.0, 'up', 2, POS_GAIN, 6.8)[1]
    plan, _ = B.plan_tri_cut(0.5, 4, 10.0, 'up', 2, POS_GAIN, 10.4)
    assert plan['t_cut'] == pytest.approx(0.8 + 0.2)            # cycle 2, P/4
    assert plan['expected_offset'] == pytest.approx(10.0 / POS_GAIN)
    # the reach covers one knot, the lane's whole coast and the full offset
    v = 20.0
    assert B.tri_cut_reach(v, POS_GAIN) >= v * (B.SEG_T + B.SEG_T + B.MAX_EXTRAP_DT_S)
    assert B.tri_cut_reach(v, POS_GAIN) <= v * (B.SEG_T + B.HAND_COAST_S + 1.0 / POS_GAIN)


@pytest.mark.parametrize('argv,needle', [
    (['--stage', 'hold', '--tri-cut', 'up'], 'only applies to --stage triangle'),
    (['--stage', 'moving_gap', '--tri-cut', 'down'], 'only applies to --stage triangle'),
    (['--stage', 'triangle', '--tri-cut-cycle', '3'], 'needs --tri-cut'),
    (['--stage', 'triangle', '--tri-cut', 'up', '--tri-cut-cycle', '0'], '1 or more'),
    (['--stage', 'triangle', '--tri-cut', 'up', '--duration', '1'], 'ends before the cut'),
    (['--stage', 'stroke_cut', '--repeat', '0'], 'positive'),
])
def test_argument_refusals(argv, needle, monkeypatch):
    def _boom(*a, **kw):
        raise RuntimeError('validation passed')
    monkeypatch.setattr(B, 'TeensyLinkClient', _boom)
    out = io.StringIO()
    old = sys.argv
    sys.argv = ['hand_stream_bench.py'] + argv
    try:
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
            with pytest.raises(SystemExit) as ei:
                B.main()
    finally:
        sys.argv = old
    assert ei.value.code == 2
    assert needle in out.getvalue()


def test_repeat_150_is_accepted(monkeypatch):
    def _boom(*a, **kw):
        raise RuntimeError('validation passed')
    monkeypatch.setattr(B, 'TeensyLinkClient', _boom)
    old = sys.argv
    sys.argv = ['hand_stream_bench.py', '--stage', 'stroke_cut', '--top-stop', '10.4',
                '--repeat', '150', '--stroke-peak-vel', '108', '--cut-at-vel', '103',
                '--stroke-accel', '2300', '--stroke-top', '8.3']
    try:
        with pytest.raises(RuntimeError, match='validation passed'):
            B.main()
    finally:
        sys.argv = old


# ---------------------------------------------------------------------------
# stroke_cut's loop on a fake clock: the stop rules and the tally
# ---------------------------------------------------------------------------

class _Clock:
    def __init__(self):
        self.t = 1.0e9

    def time(self):
        return self.t

    def monotonic(self):
        return self.t

    def sleep(self, s):
        self.t += max(0.0, s)


def _drive_stroke_cut(monkeypatch, verdicts, repeat, fault_at_run=None):
    clock = _Clock()
    monkeypatch.setattr(B, 'time', clock)
    B._SC_LINES.clear()
    B._SC_RUNS.clear()
    sent = []
    runs_seen = {'n': 0}

    class Client:
        def send_stream(self, mt, payload):
            sent.append(Setpoint.unpack(payload))

    def fake_eval(plan, echo_w, enc_w, t_end, d_c1, d_stops, d_ref):
        i = runs_seen['n']
        runs_seen['n'] += 1
        summ = {'cmd_peak': 9.0, 'enc_peak': 9.2, 'hold': 9.0,
                'overshoot': 0.2, 'settle_s': 0.06}
        return verdicts[i], [], summ

    monkeypatch.setattr(B.SC, 'evaluate_stroke_cut', fake_eval)

    def fault():
        if fault_at_run is not None and runs_seen['n'] >= fault_at_run - 1 \
                and len(sent) > 0 and B._SC_LINES[-1].startswith(f"run {fault_at_run} "):
            return 3
        return None
    monkeypatch.setattr(B, '_fault', fault)
    with B._lock:
        B._cache.update({'telem': None, 'hb': None, 'echo': None, 'diag': {},
                         'rec': None})
    start = 0.5
    plan = B.SC.plan_stroke_cut(start, 8.3, 'upstroke', 108.0, 103.0, 2300.0, 10.4)
    pump = B.SetpointPump(mm_to_rev=B.hw.GEOM_MM_TO_REV, num_legs=B.p.NUM_LEGS,
                          max_step_hand_rev=(float(B.hw.JB_TRAJ_HAND_VEL_LIMIT_RPS)
                                             * float(B.hw.JB_TRAJ_KNOT_DT_S)))
    leg = [0.0] * 6

    def sched_cmd(epoch_us, kn=None):
        return B.SC.sched_setpoint_cmd(leg, leg, epoch_us, kn)

    args = types.SimpleNamespace(repeat=repeat, max_dev=1.5, cut_max_dev=1.9)
    w = csv.writer(io.StringIO())
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        rc = B._run_stroke_cut(args, Client(), pump, sched_cmd, w, lambda: None,
                               plan, start)
    return rc, out.getvalue(), sent


def test_repeat_beyond_ten_runs_to_the_end_with_a_tally(monkeypatch):
    rc, out, sent = _drive_stroke_cut(monkeypatch, ['PASS'] * 12, 12)
    assert rc == 0, out
    assert "TALLY: 12 of 12 brakes flown, all PASS" in out
    assert out.count("run 12 of 12: 11 flown, 11 PASS so far") == 1
    assert "temperatures: no hand DIAGNOSTIC cached" in out


def test_the_first_brake_that_is_not_pass_stops_the_stage_after_its_return(monkeypatch):
    rc, out, sent = _drive_stroke_cut(monkeypatch, ['PASS', 'FAIL', 'PASS', 'PASS'], 4)
    assert rc == 1, out
    assert "STOP (run 2): the brake was FAIL" in out
    assert "TALLY: 2 of 4 brakes flown, 1 PASS; stopped at run 2 of 4" in out
    assert "run 3 of 4" not in out
    # the stage stopped from a hold at the start: the last frame is a hand
    # frame at the start with zero velocity (the return's tail hold)
    assert sent[-1].u0[B.HAND] == pytest.approx(0.5, abs=1e-6)
    assert sent[-1].v0[B.HAND] == pytest.approx(0.0, abs=1e-9)


def test_a_fault_stops_the_stage_and_the_tally_names_the_run(monkeypatch):
    rc, out, sent = _drive_stroke_cut(monkeypatch, ['PASS'] * 5, 5, fault_at_run=3)
    assert rc == 2, out
    assert "ABORT (run 3, stroke frame 0): firmware fault" in out
    assert "TALLY: 2 of 5 brakes flown, 2 PASS; stopped at run 3 of 5: ABORT (run 3" in out


def test_tally_line_shapes():
    assert B.sc_tally_line(150, 150, 150, 150, None, True) == \
        "TALLY: 150 of 150 brakes flown, all PASS"
    assert "interrupted" in B.sc_tally_line(40, 150, 40, 41, None, False)
    assert B.sc_temp_text(types.SimpleNamespace(temp_motor=41.26, temp_fet=35.0)) == \
        "motor 41.3 C, FET 35.0 C"
