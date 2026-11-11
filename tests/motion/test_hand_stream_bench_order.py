"""BUG-88: ``triangle`` and ``moving_gap`` refuse BEFORE the Hand is energised.

Same idea as ``test_driver_checks_the_plan_before_energising`` (stroke_cut):
the driver opens a UDP link, so it cannot run here and the order is pinned on
its source. The envelope helper is pure and is also checked directly.
"""
from __future__ import annotations

import importlib.util
import os

import pytest

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_BENCH = os.path.join(_REPO, "tests", "hardware", "hand_stream_bench.py")


def _src():
    with open(_BENCH) as f:
        return f.read()


def test_triangle_and_moving_gap_envelope_is_checked_before_bring_up():
    src = _src()
    bring = src.index('print("hand bring-up: POSITION/PASSTHROUGH')
    pre = src.index("triangle_envelope_refusal(tri_pre_start")
    assert 0 < pre < bring
    # the pre-check covers both stages, reads a fresh position first and falls
    # back to the baseline, and says nothing was commanded
    block = src[src.rindex("if args.stage in (", 0, pre):pre + 400]
    assert '"triangle", "moving_gap"' in block
    assert "tri_pre_start = _hand_pos()" in block and "tri_pre_start = start" in block
    assert "nothing was commanded" in src[pre:bring]
    assert "return 2" in src[pre:bring]


def test_post_bring_up_recheck_stays():
    src = _src()
    bring = src.index('print("hand bring-up: POSITION/PASSTHROUGH')
    late = src.index("triangle_envelope_refusal(start, args.tri_span)")
    assert late > bring
    # the recheck is the only place the inline comparison used to live: it must
    # not have been left behind next to the helper
    assert "start + args.tri_span > HAND_MAX_POS" not in src


def test_no_rpc_before_the_envelope_checks():
    """No Hand-energising RPC call sits between the preconditions and the pre-check."""
    src = _src()
    pre = src.index("triangle_envelope_refusal(tri_pre_start")
    head = src[:pre]
    # the first rpc.call in the driver body is the clear-errors verb (reads and
    # clears a latch, never energises) -- every SET_* RPC comes after the check
    for verb in ("SET_CONTROLLER_MODE", "SET_POS_GAIN", "SET_VEL_GAINS", "SET_AXIS_STATE"):
        assert "RpcMethod.%s" % verb not in head, verb
    # and the clear-errors call is the only rpc.call between the precondition
    # block and the pre-check
    tail = head[head.index("if args.clear_errors:"):]
    assert tail.count("rpc.call(") == 1, tail.count("rpc.call(")


def test_triangle_envelope_refusal_helper():
    spec = importlib.util.spec_from_file_location("hand_stream_bench_under_test", _BENCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    limit = mod.HAND_MAX_POS - mod.HAND_MARGIN
    assert mod.triangle_envelope_refusal(0.4855, 2.0) is None
    assert mod.triangle_envelope_refusal(limit - 2.0, 2.0) is None        # exactly at the edge
    msg = mod.triangle_envelope_refusal(limit - 2.0 + 0.01, 2.0)
    assert msg is not None and "Park lower" in msg
    # a position that moves on bring-up flips the verdict: the recheck matters
    assert mod.triangle_envelope_refusal(9.0, 2.0) is not None
