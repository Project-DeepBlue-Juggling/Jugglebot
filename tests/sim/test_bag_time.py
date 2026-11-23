"""Pure pieces of tools/probes/_bag_time.py (BUG-134): bridge tick time for /robot_state
samples and the Mocap envelope alignment. No bag, ports or fixed files: parallel-safe."""
from __future__ import annotations

import importlib.util
import os

import numpy as np

_P = os.path.join(os.path.dirname(__file__), "..", "..", "tools", "probes", "_bag_time.py")
_spec = importlib.util.spec_from_file_location("_bag_time", _P)
bt = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(bt)

# robot_state publish stamps run 13 ms after the tick; commands carry the tick stamp
CMD_STAMP = np.array([10.00, 10.01, 10.02, 10.03])
CMD_LOG = CMD_STAMP + 0.0135


def test_field_preferred_over_pairing():
    t, src = bt.robot_state_bridge_time([10.013], [10.0135], [10.0001], [True], CMD_STAMP, CMD_LOG)
    assert t[0] == 10.0001 and src[0] == bt.SRC_FIELD


def test_invalid_field_falls_to_pair():
    t, src = bt.robot_state_bridge_time([10.0235], [10.0236], [9.9], [False], CMD_STAMP, CMD_LOG)
    assert t[0] == 10.01 and src[0] == bt.SRC_PAIRED


def test_pairing_within_gap_and_old_bag_without_fields():
    t, src = bt.robot_state_bridge_time([0, 0], [10.0140, 10.0338], None, None, CMD_STAMP, CMD_LOG)
    assert list(src) == [bt.SRC_PAIRED, bt.SRC_PAIRED]
    assert np.allclose(t, [10.00, 10.02])  # log 10.0140 pairs with 10.0135; 10.0338 with 10.0335


def test_no_pair_gives_nan_never_publish_stamp():
    t, src = bt.robot_state_bridge_time([77.0], [77.0], None, None, CMD_STAMP, CMD_LOG)
    assert np.isnan(t[0]) and src[0] == bt.SRC_NONE
    t, src = bt.robot_state_bridge_time([5.0], [5.0], [0.0], [True], [], [])
    assert np.isnan(t[0]) and src[0] == bt.SRC_NONE  # zero stamp is not a valid field


def test_mocap_sign_and_worked_number():
    # envelope T on the Jetson clock, bridge behind Jetson by err: bridge-clock time = T - err - 13 ms
    assert np.isclose(bt.mocap_time_on_bridge_clock(100.0), 100.0 - 0.013)
    assert np.isclose(bt.mocap_time_on_bridge_clock(100.0, 0.0034), 99.9836)
    assert bt.MOCAP_ENVELOPE_LATE_VS_BRIDGE_S == 0.013
