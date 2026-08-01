"""Boot-time config-identity logging in teensy_bridge_node.

Production is BUILD-FROZEN: `jugglebot.hardware_config` resolves to the
colcon-installed copy, so editing `config/hardware_config.yaml` and relaunching
changes nothing — silently. `hardware_config_identity()` is the observability
half of the Phase 5 answer (plans/active/refactor-2026-07.md): the bridge states
at boot exactly which file it got, its sha256 and its mtime, so an operator can
reconcile a session's behaviour against a specific artifact afterwards.

What these tests protect:
  * the string names a REAL file and a hash that matches that file's bytes
    (a plausible-looking but wrong identity is worse than none);
  * it reports the module that is actually bound, not a recomputed path;
  * it can never raise, because it runs inside the node constructor.

ROS 2 is mocked by tests/ros/conftest.py. Read-only, no ports, xdist-safe.
"""

from __future__ import annotations

import hashlib
import os
import re

import jugglebot.hardware_config as hw
from jugglebot.teensy_bridge_node import hardware_config_identity


def test_identity_names_the_bound_module_and_hashes_its_bytes():
    line = hardware_config_identity()
    m = re.match(r"hardware_config: (?P<path>.+) sha256=(?P<sha>[0-9a-f]{16}) "
                 r"mtime=(?P<mtime>\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}[-+]\d{4})$",
                 line)
    assert m, f"unparseable identity line: {line!r}"

    path = m.group("path")
    assert path == hw.__file__, (
        "identity must report the module that is actually bound "
        f"({hw.__file__}), not {path}")
    with open(path, "rb") as f:
        expected = hashlib.sha256(f.read()).hexdigest()[:16]
    assert m.group("sha") == expected, (
        "sha256 prefix does not match the file it names — a wrong identity is "
        "worse than no identity")


def test_identity_survives_a_module_without_a_file(monkeypatch):
    """No __file__ (frozen/namespace import) degrades to a named UNKNOWN."""
    monkeypatch.delattr(hw, "__file__", raising=False)
    line = hardware_config_identity()
    assert "UNKNOWN source" in line, line


def test_identity_never_raises_when_the_file_is_gone(monkeypatch):
    """A vanished/unreadable artifact must not stop the bridge constructing."""
    monkeypatch.setattr(hw, "__file__", "/nonexistent/hardware_config.py",
                        raising=False)
    line = hardware_config_identity()
    assert "identity unavailable" in line, line
    assert "FileNotFoundError" in line, line


def test_identity_mtime_matches_the_file():
    line = hardware_config_identity()
    stamp = line.rsplit("mtime=", 1)[1]
    import time as _time
    expected = _time.strftime("%Y-%m-%dT%H:%M:%S%z",
                              _time.localtime(os.path.getmtime(hw.__file__)))
    assert stamp == expected, (stamp, expected)


def test_identity_mtime_carries_a_utc_offset():
    """Offset-less local time cannot be reconciled across a DST change."""
    line = hardware_config_identity()
    stamp = line.rsplit("mtime=", 1)[1]
    assert re.match(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}[-+]\d{4}$", stamp), (
        f"mtime must carry a UTC offset to be reconcilable later: {stamp!r}")
