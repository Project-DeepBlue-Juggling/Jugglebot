"""The Wave camera adapter's native tests, in the gate (2026-11-07).

``ros_ws/src/jugglebot/WaveCamera_ESP_code/test/`` holds 80 Unity cases that were run only
by hand, so no gate ever saw them. This runs them with ``-e native``, hard-coded on
purpose: the project's default environment is the ESP32, and a bare ``pio test`` would
build for the board and try to upload. The Night Shift's deny list refuses ``pio test``
outright, so this file is its only way to run them. Build output goes under tmp_path, never
into the source tree. About 24 s on the Jetson (2026-11-07).

The Unity library comes from the PlatformIO registry, and a fresh build directory resolves it
over the network each time: fine with a warm registry cache, a 60 s hang per case with a cold
one and no network, which is how the Night Shift runs this (10 cases hit the 600 s limit on
2026-11-07). So it lives in one stable directory, fetched once, and after that no run needs
the network. With no copy and no network the test skips and says so.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PROJECT = os.path.join(ROOT, "ros_ws", "src", "jugglebot", "WaveCamera_ESP_code")


def _pio():
    beside = os.path.join(os.path.dirname(sys.executable), "pio")
    return beside if os.path.exists(beside) else shutil.which("pio")


def test_wave_native_suite_passes(tmp_path):
    pio = _pio()
    if not pio:
        pytest.skip("PlatformIO is not installed here")
    libdeps = os.path.join(os.path.expanduser("~"), ".cache", "jugglebot", "wave-native-libdeps")
    env = dict(os.environ, PLATFORMIO_WORKSPACE_DIR=str(tmp_path / "workspace"),
               PLATFORMIO_LIBDEPS_DIR=libdeps,
               PLATFORMIO_SETTING_ENABLE_TELEMETRY="false",
               PLATFORMIO_SETTING_CHECK_PLATFORMIO_INTERVAL="9999")
    if not os.path.isdir(os.path.join(libdeps, "native", "Unity")):
        try:
            got = subprocess.run([pio, "pkg", "install", "-e", "native"], cwd=PROJECT, env=env,
                                 capture_output=True, universal_newlines=True, timeout=120,
                                 stdin=subprocess.DEVNULL)
        except subprocess.TimeoutExpired:
            pytest.skip("no local Unity copy and no network to fetch one")
        if got.returncode != 0:
            pytest.skip("no local Unity copy and no network to fetch one: %s" % got.stdout[-200:])
    r = subprocess.run([pio, "test", "-e", "native"], cwd=PROJECT, env=env, capture_output=True,
                       universal_newlines=True, timeout=300, stdin=subprocess.DEVNULL)
    assert r.returncode == 0, r.stdout[-3000:] + r.stderr[-1000:]
    assert " succeeded " in r.stdout
    assert "esp32dev" not in r.stdout.split("Processing")[-1].splitlines()[0]
