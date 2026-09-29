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
