"""Sweep kinematics (start/end angle, duration, fitted lag) on every attempt message.

Phase 0 of plans/archived/bb-calibration-one-attempt.md: the hardware pin needs
the sweep end angle and the yaw-source lag recorded per attempt, refusals
included. Format pinned here: ``<text> · sweep <start>→<end>° <dur> s, lag <±x.x> ms``
(``lag n/a`` when no estimate ran). The suffix is reserved INSIDE
``MAX_CALIBRATION_FAILURE_CHARS`` so it survives truncation.
"""
from __future__ import annotations

import math
import re

import numpy as np

from jugglebot.bb_calibration import sweep_kinematics, sweep_suffix
from jugglebot.protocol_config import BallButlerStates
from tests.ros.test_mocap_node_keep_last_good import (
    LONG_SUMMARY, _hb, _node, _result)
from unittest.mock import patch

SUFFIX_RE = re.compile(r' · sweep -?\d+\.\d→-?\d+\.\d° \d+\.\d s, lag (n/a|[+-]\d+\.\d ms)$')


def _triangle(peak=121.3, dur=12.4, n=125):
    """0 -> peak -> 0 over ``dur`` s."""
    t = np.linspace(0.0, dur, n)
    y = peak * (1.0 - np.abs(2.0 * t / dur - 1.0))
    return [(1000.0 + float(a), float(b)) for a, b in zip(t, y)]


def _run_sweep(node, samples, result=None, error=None):
    import jugglebot.mocap_node as mn
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    node._calib_yaw_samples = list(samples)
    node._calib_data = {0: [np.zeros(3)]}
    kw = {'side_effect': error} if error is not None else {'return_value': result}
    with patch.object(mn, 'run_calibration', **kw):
        node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    return node.pub_calibration_attempt.published[-1]


def test_sweep_kinematics_on_a_0_121_0_sweep():
    k = sweep_kinematics(_triangle())
    assert k['start_deg'] == 0.0 and abs(k['end_deg'] - 121.3) < 0.5
    assert abs(k['span_deg'] - 121.3) < 0.5 and abs(k['duration_s'] - 12.4) < 1e-9
    assert k['n_samples'] == 125


def test_sweep_kinematics_empty_and_nonfinite():
    assert sweep_kinematics([]) is None and sweep_kinematics(None) is None
    assert sweep_kinematics([(0.0, float('nan'))]) is None
    assert sweep_suffix([]) == ''


def test_sweep_suffix_format():
    assert sweep_suffix([(0.0, 0.0), (12.4, 121.3)], 0.0031) == ' · sweep 0.0→121.3° 12.4 s, lag +3.1 ms'
    assert sweep_suffix([(0.0, 0.0), (1.0, 5.0)]).endswith(', lag n/a')


def test_a_refused_sweep_carries_the_kinematics(tmp_path):
    node, _ = _node(tmp_path)
    msg = _run_sweep(node, _triangle(), error=ValueError('CONSTELLATION_TOO_FEW_MOVING: 3 samples'))
    assert not msg.success
    assert msg.message.startswith('CONSTELLATION_TOO_FEW_MOVING: 3 samples')
    assert SUFFIX_RE.search(msg.message) and msg.message.endswith('lag n/a')
    assert '→121.' in msg.message


def test_an_accepted_sweep_carries_kinematics_and_the_fitted_lag(tmp_path):
    node, _ = _node(tmp_path)
    msg = _run_sweep(node, _triangle(), result=_result(0.926))
    assert msg.success, msg.message
    assert SUFFIX_RE.search(msg.message) and msg.message.endswith('lag +83.0 ms')


def test_the_suffix_survives_truncation_inside_the_bound(tmp_path):
    import jugglebot.mocap_node as mn
    node, _ = _node(tmp_path)
    msg = _run_sweep(node, _triangle(), error=ValueError('CONSTELLATION_X: ' + 'detail\n' * 200))
    assert len(msg.message) <= mn.MAX_CALIBRATION_FAILURE_CHARS
    assert '…' in msg.message and SUFFIX_RE.search(msg.message)


def test_no_yaw_samples_means_no_suffix(tmp_path):
    node, _ = _node(tmp_path)
    node._publish_calibration_failure('No marker data collected')
    assert node.pub_calibration_attempt.published[-1].message == 'No marker data collected'
