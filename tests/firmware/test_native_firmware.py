"""Pytest entry point for the native (compiled) firmware test harness.

The safety-critical firmware (`fault_machine.cpp`, `leg_interp.cpp`) had **no
compiled test** before this — its correctness rested on three hand-synced Python
transcriptions, so a C++ edit that diverged from the mirrors passed the whole
suite (the standing risk flagged in `logbook/2026-06-27-can-node-teensy-parity-
audit.md` §5). This wrapper compiles the REAL `.cpp` on the build host behind the
~10-line HAL shims + the fake HAL (`tests/firmware/native/`) and runs the binary,
so a C++ divergence fails `pytest tests/ -q`.

It does three things, all hash-cached (zero recompile when the sources are
unchanged) and g++-gated (a host without a compiler SKIPS, not fails — the Jetson
run is authoritative):

  1. build + run `test_fault_machine` (soft-reset limiter, UV gating, the
     deferred-stow 5 invariants, present-axis scoping, fb-stale suppression);
  2. build + run `test_leg_interp` (lead/stroke/present-axis clamps, modes, stow
     descent);
  3. assert a freshly-emitted golden still equals the committed
     `native/fault_golden.json` — so a firmware behaviour change must regenerate
     the golden deliberately (and `tests/firmware/test_fault_logic.py` then pins
     the Jetson host mirror `fault_logic.py` to that same golden).

SCOPE: the harness validates DECISION LOGIC, not FreeRTOS/ISR concurrency or the
500 Hz deadline — those remain on-hardware-replay gaps (see native/README.md).
"""

from __future__ import annotations

import importlib.util
import json
import subprocess
from pathlib import Path

import pytest

_NATIVE = Path(__file__).resolve().parent / "native"
_BUILD_PY = _NATIVE / "build.py"
_GOLDEN = _NATIVE / "fault_golden.json"


def _load_build():
    spec = importlib.util.spec_from_file_location("cb_native_build", _BUILD_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_build = _load_build()

pytestmark = pytest.mark.skipif(
    not _build.have_gpp(),
    reason="g++ not available — the native firmware harness needs a host C++ compiler "
           "(the Jetson run is authoritative)")


@pytest.fixture(scope="module")
def binaries():
    """Hash-cached build of both native test binaries (compiles only on change)."""
    return _build.build_all()


def _run(path, *args):
    return subprocess.run([str(path), *args], capture_output=True, text=True)


def test_native_fault_machine_binary_passes(binaries):
    """The compiled fault_machine.cpp passes every doctest assertion (the
    robustness contract: a C++ divergence from the safety logic fails here)."""
    r = _run(binaries["test_fault_machine"])
    assert r.returncode == 0, (
        "native test_fault_machine FAILED — fault_machine.cpp diverged from the "
        f"safety contract:\n{r.stdout}\n{r.stderr}")


def test_native_leg_interp_binary_passes(binaries):
    """The compiled leg_interp.cpp passes every behaviour assertion (lead/stroke/
    present-axis clamps, mode transitions, stow descent)."""
    r = _run(binaries["test_leg_interp"])
    assert r.returncode == 0, (
        "native test_leg_interp FAILED — leg_interp.cpp diverged from the expected "
        f"interpolator behaviour:\n{r.stdout}\n{r.stderr}")


def test_committed_golden_matches_live_firmware(binaries, tmp_path):
    """A freshly-emitted golden equals the committed native/fault_golden.json.

    The golden is the firmware-anchored conformance source the Jetson host mirror
    (`fault_logic.py`) is pinned to. If the fault machine's behaviour changes, the
    fresh emission diverges from the committed file and this fails — forcing a
    deliberate regeneration (and a matching `fault_logic.py` update), which is
    exactly the three-way coupling the manual transcription used to rely on, now
    enforced by a test instead of by hand.
    """
    fresh = tmp_path / "fault_golden.json"
    r = _run(binaries["test_fault_machine"], "--emit-golden", str(fresh))
    assert r.returncode == 0, f"golden emission failed:\n{r.stdout}\n{r.stderr}"
    committed = json.loads(_GOLDEN.read_text())
    regenerated = json.loads(fresh.read_text())
    assert regenerated == committed, (
        "tests/firmware/native/fault_golden.json is stale vs the live firmware "
        "(fault_machine.cpp behaviour changed). Regenerate it:\n"
        "  python tests/firmware/native/build.py --golden "
        "tests/firmware/native/fault_golden.json\n"
        "and update controller/teensy_link/fault_logic.py to match if the change "
        "was intended.")
