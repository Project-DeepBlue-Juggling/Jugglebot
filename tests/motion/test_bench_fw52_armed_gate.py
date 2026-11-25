"""can-bridge FW 52 (BUG-99 D1, D2): the bench tools' side.

* ``bench_leg_sysid._clear_errors`` sends the ALL-AXES clear (FW 52: only that one
  releases the Soft E-stop latch) and treats ERR_TIMEOUT (the absent axes of a
  one-Leg bench) as done, re-raising every other status.
* ``bench_leg_sysid._apply_gains`` waits for the bridge to report disarmed and raises,
  naming the reason, if it never does (FW 52 refuses gain writes while Armed).
* ``lead_clamp_gap_bench`` refuses to run on FW >= 52 (its gaps are mid-stream SDO
  writes, which FW 52 refuses), instead of flying a run with no gap injected.
"""
from __future__ import annotations

import importlib.util
import os
import sys
import threading
import types

import pytest

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_HW = os.path.join(_ROOT, 'tests', 'hardware')
if _HW not in sys.path:
    sys.path.insert(0, _HW)

import bench_leg_sysid as bls  # noqa: E402
from teensy_link import protocol as pr  # noqa: E402
from teensy_link import rpc_args  # noqa: E402
from teensy_link.rpc import RpcError  # noqa: E402


class _Rpc:
    def __init__(self, status=None):
        self.calls, self.status = [], status

    def call(self, method, args, **_kw):
        self.calls.append((int(method), bytes(args)))
        if self.status is not None:
            raise RpcError(int(method), int(self.status))
        return b''


def _bench(rpc, hb_flags=None):
    b = object.__new__(bls.BridgeSysID)
    b._rpc, b._rpc_args, b._RpcMethod = rpc, rpc_args, pr.RpcMethod
    b._lock = threading.Lock()
    b._cache = {'hb': None if hb_flags is None else types.SimpleNamespace(flags=hb_flags)}
    b.axis_id = 0
    return b


def test_clear_errors_is_the_all_axes_clear():
    rpc = _Rpc()
    _bench(rpc)._clear_errors()
    assert rpc.calls == [(int(pr.RpcMethod.CLEAR_ERRORS), rpc_args.encode_clear_errors(rpc_args.AXIS_ALL))]


def test_clear_errors_err_timeout_from_absent_axes_is_done():
    _bench(_Rpc(pr.RpcStatus.ERR_TIMEOUT))._clear_errors()   # no raise


def test_clear_errors_other_refusals_still_raise():
    with pytest.raises(RpcError):
        _bench(_Rpc(pr.RpcStatus.ERR_BUS_DOWN))._clear_errors()


def test_apply_gains_waits_for_disarm_and_raises_if_the_bridge_stays_armed(monkeypatch):
    rpc = _Rpc()
    b = _bench(rpc, hb_flags=int(pr.HeartbeatT2JFlags.MPC_ACTIVE))
    monkeypatch.setattr(b, '_wait_bridge_disarmed',
                        lambda what, timeout_s=1.0: bls.BridgeSysID._wait_bridge_disarmed(b, what, 0.05))
    with pytest.raises(RuntimeError, match='refuses gain writes while Armed'):
        b._apply_gains(types.SimpleNamespace(pos_gain=40.0, vel_gain=0.1, vel_int_gain=0.2))
    assert rpc.calls == []


def test_apply_gains_disarmed_sends_both():
    rpc = _Rpc()
    _bench(rpc, hb_flags=0)._apply_gains(types.SimpleNamespace(pos_gain=40.0, vel_gain=0.1, vel_int_gain=0.2))
    assert [m for m, _ in rpc.calls] == [int(pr.RpcMethod.SET_POS_GAIN), int(pr.RpcMethod.SET_VEL_GAINS)]


def _probe():
    path = os.path.join(_ROOT, 'tools', 'probes', 'lead_clamp_gap_bench.py')
    spec = importlib.util.spec_from_file_location('lead_clamp_gap_bench_fw52', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_lead_clamp_gap_bench_refuses_fw52_and_unknown():
    mod = _probe()
    assert mod.armed_sdo_refusal(51) is None
    for fw in (52, 53, None):
        msg = mod.armed_sdo_refusal(fw)
        assert msg is not None and 'SDO_WRITE while Armed' in msg and 'BUG-99' in msg
