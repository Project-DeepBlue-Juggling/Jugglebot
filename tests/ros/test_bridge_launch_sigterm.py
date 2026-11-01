"""JB-82: both launch files give teensy_bridge_node a SIGINT->SIGTERM window longer
than its own bounded shutdown Stow, so the node can log the Stow's outcome.

launch's default is 5 s; the node's Stow is bounded at
``_SHUTDOWN_DEACTIVATE_TIMEOUT_S`` plus ``_SHUTDOWN_DISARM_SETTLE_S``. On 2026-10-31
two shutdowns were SIGTERM-killed at 5 s (13:06 and 13:13 launch logs). The descent
itself is ODrive/firmware-owned and finishes even if the node dies; the window is
about the record, and about not killing the node while it still has work to do.
"""
from __future__ import annotations

import ast
from pathlib import Path

import pytest

LAUNCH = Path(__file__).resolve().parents[2] / "ros_ws" / "src" / "jugglebot" / "launch"
MARGIN_S = 1.0


def _bridge_kwargs(path: Path) -> dict:
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            kw = {k.arg: k.value for k in node.keywords if k.arg}
            ex = kw.get("executable")
            if isinstance(ex, ast.Constant) and ex.value == "teensy_bridge_node":
                return kw
    raise AssertionError("no teensy_bridge_node Node in %s" % path.name)


@pytest.mark.parametrize("name", ["jugglebot_launch.py", "teensy_bridge_launch.py"])
def test_bridge_sigterm_window_covers_the_shutdown_stow(name):
    from jugglebot import teensy_bridge_node as tbn
    kw = _bridge_kwargs(LAUNCH / name)
    assert "sigterm_timeout" in kw, "%s: teensy_bridge_node has no sigterm_timeout" % name
    window = float(ast.literal_eval(kw["sigterm_timeout"]))
    budget = tbn._SHUTDOWN_DEACTIVATE_TIMEOUT_S + tbn._SHUTDOWN_DISARM_SETTLE_S
    assert window >= budget + MARGIN_S, (window, budget)
    assert window <= 15.0, "a stuck node must still be killed promptly on Ctrl+C"
