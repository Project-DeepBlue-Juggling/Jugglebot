"""BUG-111 / BUG-89: tools/probes/hand_late_knot_replay.py — log parsing, the hole
arithmetic, and the fit + firmware-twin replay on a synthetic Throw.

The synthetic Throw has its release knot (+1200 -> -2000 rev/s^2) exactly at the
dropped frame's predecessor's cover end, as on 2026-11-18 and 2026-11-20: a
forwarded frame 12.6 ms late must step the command there, and the same arrivals
sent one knot earlier (emit lead 1) must not.
"""
from __future__ import annotations

import importlib.util
import os

import numpy as np
import pytest

_P = os.path.join(os.path.dirname(__file__), '..', '..', 'tools', 'probes',
                  'hand_late_knot_replay.py')
_spec = importlib.util.spec_from_file_location('hand_late_knot_replay', _P)
hl = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(hl)

# launch_2026-11-20_1.log:5297, verbatim including the ANSI colour codes
LINE_1 = ('\x1b[2m13:22:42.695\x1b[0m \x1b[36mteensy      \x1b[0m \x1b[1;33mWARN  \x1b[0m\x1b[33m'
          'setpoint drain dropped 1 older frame(s) (newest-only forward): dropped seq 50752, '
          'stamp 1795141362.646479, lag 27.3 ms at drain; forwarded seq 50753, lag 2.3 ms; '
          'total 27 (0 more since the last such line)\x1b[0m')
# the n > 1 spelling of the same format string (teensy_bridge_node.py)
LINE_2 = ('13:04:00.001 teensy       WARN  setpoint drain dropped 2 older frame(s) '
          '(newest-only forward): dropped seq 10..11, stamp 1795141362.646479, lag 51.3 ms '
          'at drain; forwarded seq 12, lag 1.3 ms; total 9 (3 more since the last such line)')


def test_drop_line_parses_both_spellings():
    m = hl._DROP.match(hl._ANSI.sub('', LINE_1))
    assert m and m.group('n') == '1' and m.group('seqs') == '50752'
    assert float(m.group('dlag')) == 27.3 and float(m.group('flag')) == 2.3
    assert m.group('stamp') == '1795141362.646479' and m.group('more') == '0'
    m = hl._DROP.match(LINE_2)
    assert m and m.group('n') == '2' and m.group('seqs') == '10..11'
    assert m.group('fseq') == '12' and m.group('more') == '3'


def test_hole_arithmetic():
    # lead 0: one drop opens a hole of the forwarded frame's own lag
    assert hl.hole_ms(1, 2.3, 0, 'newest') == 2.3
    assert hl.hole_ms(2, 1.3, 0, 'newest') == pytest.approx(26.3)
    # lead 1, newest-only: a single drop is covered, a double drop is not
    assert hl.hole_ms(1, 12.6, 1, 'newest') == 0.0
    assert hl.hole_ms(1, 27.4, 1, 'newest') == pytest.approx(2.4)
    assert hl.hole_ms(2, 1.3, 1, 'newest') == 25.0
    # lead 1, due-aware: the newest due frame is forwarded too
    assert hl.hole_ms(1, 12.6, 1, 'due') == 0.0
    assert hl.hole_ms(2, 1.3, 1, 'due') == 1.3


def _synthetic_throw():
    T = hl.T_US
    vk = [0.0] * 14 + [30.0, 70.0, 105.0, 126.0, 75.0, 28.0, 4.0] + [0.0] * 5
    q = [0.1]
    for j in range(1, len(vk)):
        q.append(q[-1] + T * 1e-6 * 0.5 * (vk[j - 1] + vk[j]))
    stamp_us = 1_000_000_000_000                   # the dropped frame's stamp (knot 16)
    origin = stamp_us - 16 * T
    rows = []
    for i in range(0, (len(vk) - 1) * T // 10000):
        t = origin + 10000 * i + 544
        p, v, _a = hl.plan_eval(np.array(q), np.array(vk), origin, t)
        rows.append((t, p, v, 0.0))
    d = dict(dropped_seq=1, dropped_stamp=stamp_us * 1e-6, dropped_lag_ms=37.6, fwd_seq=2,
             fwd_lag_ms=12.6, n=1, drain_wall=stamp_us * 1e-6 + 0.0376)
    return np.array(rows), d


def test_late_frame_steps_the_command_at_lead_0_and_not_at_lead_1():
    echo, d = _synthetic_throw()
    r = hl.analyse_drop(echo, d, transit_ms=1.0)
    assert r['fit']['vel_max'] < 1e-6
    assert r['plan_at_cover_end']['acc'] < -1000.0   # the plan brakes from the cover end
    sc = r['scenarios']
    flown = sc['flown_lead0_newest']
    assert flown['max_dev_vel'] > 20.0               # the continued cubic overshoots
    assert flown['peak_vel'] > flown['plan_peak_vel'] + 5.0
    assert flown['worst_tick_step_vel'] < -20.0      # ... and steps down on promotion
    assert sc['lead1_newest']['max_dev_vel'] < 1e-6
    assert sc['lead1_due_aware']['max_dev_vel'] < 1e-6
    assert 0.0 < sc['lead0_cv_grace']['max_dev_vel'] < flown['max_dev_vel']
