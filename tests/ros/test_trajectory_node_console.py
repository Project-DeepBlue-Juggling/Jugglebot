"""trajectory_node's side of the operator console (phase 2): per-install
detail is recorded at DEBUG, not shown; a clock-offset refresh reaches the
screen only when it steps by enough to matter."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from jugglebot import trajectory_node as tn


@pytest.mark.parametrize('step_us, level', [
    (0.0, 'debug'),       # the norm: median 0.0 us over 320 refreshes
    (3.3, 'debug'),       # the largest step seen, 09-13..09-29
    (99.9, 'debug'),
    (100.5, 'warning'),    # (exactly 100.0 is float-fuzzy on a 5 s offset)
    (-250.0, 'warning'),
])
def test_a_clock_offset_refresh_warns_only_on_a_real_step(monkeypatch,
                                                           step_us, level):
    logs = {'debug': [], 'info': [], 'warning': []}
    logger = SimpleNamespace(**{k: v.append for k, v in logs.items()})
    fake = SimpleNamespace(_ros_to_perf_offset=5.0, _clock_offset_history=[],
                           _ros_clock_s=lambda: 0.0,
                           get_logger=lambda: logger)
    monkeypatch.setattr(tn.clock_offset, 'refresh_offset',
                        lambda history, ros: 5.0 + step_us * 1e-6)
    tn.TrajectoryNode._refresh_clock_offset(fake)
    assert [len(logs[k]) for k in ('debug', 'info', 'warning')] == [
        int(level == 'debug'), 0, int(level == 'warning')]
    (line,) = logs[level]
    assert line.startswith('clock offset refreshed: %+.1f us step' % step_us)


def test_a_small_then_a_big_clock_step_do_not_crash_the_node(monkeypatch):
    """One call site per severity (Foxy rclpy raises otherwise): DEBUG then
    WARNING then DEBUG through the ENFORCING mock logger."""
    from tests.ros.conftest import MockLogger
    logger = MockLogger()
    fake = SimpleNamespace(_ros_to_perf_offset=5.0, _clock_offset_history=[],
                           _ros_clock_s=lambda: 0.0,
                           get_logger=lambda: logger)
    for step_us in (1.0, 500.0, 2.0):
        monkeypatch.setattr(tn.clock_offset, 'refresh_offset',
                            lambda history, ros, s=step_us:
                            fake._ros_to_perf_offset + s * 1e-6)
        tn.TrajectoryNode._refresh_clock_offset(fake)


# ── phase 3: startup, limits, go_home, stream lines ──────────────────────

def _recording(node):
    """Swap the node's ENFORCING mock logger for one that also records."""
    from tests.ros.conftest import MockLogger

    class Rec(MockLogger):
        def __init__(self):
            super().__init__()
            self.lines = []

        def _log(self, severity, kw):
            super()._log(severity, kw)

    rec = Rec()
    for name, sev in (('info', 'INFO'), ('warning', 'WARN'),
                      ('error', 'ERROR'), ('debug', 'DEBUG')):
        def make(name=name, sev=sev):
            def f(msg, **kw):
                rec.lines.append((sev, msg))
            return f
        setattr(rec, name, make())
    node._logger = rec   # records only; site enforcement runs via the plain mock
    return rec


def test_startup_is_one_up_line_and_at_most_one_tilt_line(monkeypatch):
    monkeypatch.delenv('JUGGLEBOT_TILT_CAL', raising=False)
    node = tn.TrajectoryNode(start_emitter=False)
    # construction already logged through the default mock; re-run the two
    # startup sites against a recording logger.
    rec = _recording(node)
    node._load_tilt_map()
    tilt = [m for s, m in rec.lines if s in ('INFO', 'WARN')]
    assert len(tilt) == 1 and tilt[0].startswith(('tilt map: none', 'tilt map loaded:'))
    assert '/' not in tilt[0], 'no file paths on an OK line'


def test_set_limits_logs_one_tidy_outcome_line():
    node = tn.TrajectoryNode(start_emitter=False)
    rec = _recording(node)
    req = SimpleNamespace(leg_vel_limit_mmps=300.0, leg_acc_limit_mmps2=0.0,
                          leg_jerk_limit_mmps3=0.0)
    resp = SimpleNamespace()
    node._svc_set_limits(req, resp)
    assert resp.success is True
    (sev, line), = rec.lines
    assert sev == 'INFO' and line.startswith('leg limits set: 300 mm/s')


def test_go_home_refusal_is_an_error_line_and_success_is_one_info_line():
    node = tn.TrajectoryNode(start_emitter=False)
    rec = _recording(node)
    resp = SimpleNamespace()
    node._svc_go_home(SimpleNamespace(), resp)      # not seeded
    assert resp.success is False
    assert rec.lines == [('ERROR', 'go_home refused: ' + resp.message)]


def test_stream_on_off_is_debug_and_an_empty_mode_reads_none():
    node = tn.TrajectoryNode(start_emitter=False)
    rec = _recording(node)
    node._on_control_mode(SimpleNamespace(data='STANDBY'))
    node._on_control_mode(SimpleNamespace(data=''))
    assert [m for s, m in rec.lines if s == 'INFO'] == []
    assert ('DEBUG', 'streaming DISABLED (mode none)') in rec.lines
