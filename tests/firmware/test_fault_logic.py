"""Firmware-anchored golden-vector conformance for the Jetson fault host mirror.

**What this file used to be, and why it changed.** It previously held a hand-
maintained Python *transcription* of `fault_machine.cpp` (`FaultMirror` /
`StowMirror`) plus the present-axis / fb-stale logic re-derived in Python — one
of the three copies kept in sync by hand. That transcription is now retired: the
REAL firmware is compiled and asserted directly by the native harness
(`tests/firmware/native/test_fault_machine.cpp`, run by
`tests/firmware/test_native_firmware.py`), which is the authoritative
robustness contract for the C++.

**What it is now.** The bridge still runs a Jetson-side Python mirror —
`controller/teensy_link/fault_logic.py` (`FaultEvaluator` / `DeferredStowLatch`,
and the live `LinkLossLatch`) — as the canonical port for the eventual
`can_node.py` deletion. This file pins that mirror to a **firmware-anchored
golden vector** (`native/fault_golden.json`, emitted by the compiled
`fault_step()`), so the host mirror can no longer silently drift from the
firmware either. It needs NO compiler — it replays the committed golden through
the Python mirror — so it runs everywhere (incl. dev boxes without g++), while
`test_native_firmware.py` keeps the golden itself honest to the live firmware on
the Jetson.

  firmware (native harness)  ⇒  fault_golden.json  ⇒  fault_logic.py (here)

`tests/teensy_link/test_fault_logic_mirror.py` (the hand-scenario tests for the
same classes, incl. `LinkLossLatch`) is KEPT and complementary — it asserts named
transitions; this asserts firmware equivalence over the golden set.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from controller.teensy_link.fault_logic import FaultEvaluator, DeferredStowLatch

_GOLDEN = Path(__file__).resolve().parent / "native" / "fault_golden.json"


def _golden() -> dict:
    return json.loads(_GOLDEN.read_text())


def _error_scenarios():
    return _golden()["error_scenarios"]


def _stow_scenarios():
    return _golden()["stow_scenarios"]


def test_golden_present_and_populated():
    """The committed firmware-anchored golden exists and carries both scenario
    families — a missing/empty golden must fail loudly, not silently skip."""
    g = _golden()
    assert g["error_scenarios"], "no error scenarios in the golden"
    assert g["stow_scenarios"], "no stow scenarios in the golden"


@pytest.mark.parametrize("sc", _error_scenarios(), ids=lambda s: s["name"])
def test_error_eval_conformance(sc):
    """fault_logic.py FaultEvaluator reproduces the firmware's error-eval verdict
    (fatal / undervoltage / soft-reset budget / clear-broadcast count) step for
    step, over the firmware-anchored golden."""
    m = FaultEvaluator()
    for step in sc["steps"]:
        if step["notify_clear"]:
            m.notify_clear_errors()
        else:
            if step["preset_fatal"]:
                m.fatal_error = True
            m.active = list(step["active"])
            m.disarm = list(step["disarm"])
            m.state = list(step["state"])
            m.evaluate_errors()
        out = step["out"]
        assert m.fatal_error == out["fatal_error"]
        assert m.undervoltage_error == out["undervoltage_error"]
        assert m.soft_reset_attempts == out["soft_reset_attempts"]
        assert m.clear_calls == out["clear_calls"]


@pytest.mark.parametrize("sc", _stow_scenarios(), ids=lambda s: s["name"])
def test_deferred_stow_conformance(sc):
    """fault_logic.py DeferredStowLatch reproduces the firmware's deferred-stow
    latch state (fatal / stow_pending / stowing) step for step, over the
    firmware-anchored golden — including the terminal completion clear."""
    latch = DeferredStowLatch()
    for step in sc["steps"]:
        if step["complete"]:
            latch.stow_complete = True
        latch.step(step["stale"], step["fresh"])
        out = step["out"]
        assert latch.fatal == out["fatal"]
        assert latch.stow_pending == out["stow_pending"]
        assert latch.stowing == out["stowing"]
