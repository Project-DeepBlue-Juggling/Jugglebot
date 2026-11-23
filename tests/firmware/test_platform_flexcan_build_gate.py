"""Build-gate contract: the Platform Teensy must build against the can-bridge's patched FlexCAN_T4 (BUG-132).

The failure this prevents: the Platform silently building against the *framework's* FlexCAN_T4, whose ``events()``
pop races the RX interrupt and leaks ``_available`` counts. The ring then becomes a delay line (up to ~114 ms on this
bus), the Platform's wall clock (the IMU capture stamps) runs early by tens of ms and every command to it is late by
the same amount. It compiles and runs, so nothing fails until a Sitting; this file makes the resolution a per-commit
check on three links of the chain:

1. ``platformio.ini``'s base env resolves the library from the can-bridge's copy (``lib_extra_dirs``);
2. the sketch ``#error``s unless ``JUGGLEBOT_FLEXCAN_P3`` is defined, so a changed resolution fails the build;
3. the patched header actually defines that macro (and the framework copy, by construction, never does).
"""
from __future__ import annotations

import configparser
import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PLATFORM = REPO / "ros_ws" / "src" / "jugglebot" / "Teensy_code_platform"
BRIDGE = REPO / "ros_ws" / "src" / "jugglebot" / "Teensy_code_canbridge"
MACRO = "JUGGLEBOT_FLEXCAN_P3"


def test_platformio_resolves_flexcan_from_the_bridge_copy():
    cp = configparser.ConfigParser(interpolation=None, strict=False)
    cp.read(PLATFORM / "platformio.ini")
    raw = cp["env:teensy40"]["lib_extra_dirs"]
    dirs = [d.strip() for d in raw.replace(",", "\n").splitlines() if d.strip() and not d.strip().startswith(";")]
    resolved = [(PLATFORM / d).resolve() for d in dirs]
    assert (BRIDGE / "lib").resolve() in resolved, "env:teensy40 no longer takes lib_extra_dirs from the bridge"
    assert (BRIDGE / "lib" / "FlexCAN_T4" / "FlexCAN_T4.h").is_file()
    # Every other env must inherit it, or one of them builds the unpatched framework copy.
    for name in cp.sections():
        if name.startswith("env:") and name != "env:teensy40":
            assert cp[name].get("extends", "").strip() == "env:teensy40" or "lib_extra_dirs" in cp[name], \
                "%s neither extends env:teensy40 nor sets lib_extra_dirs" % name


def test_sketch_errors_when_the_patched_library_is_not_resolved():
    src = (PLATFORM / "Teensy_code_platform.ino").read_text()
    m = re.search(r"#\s*if\s+!\s*defined\(\s*%s\s*\)\s*\n\s*#\s*error" % MACRO, src)
    assert m, "the sketch lost its #error guard on %s" % MACRO


def test_patched_header_defines_the_marker():
    hdr = (BRIDGE / "lib" / "FlexCAN_T4" / "FlexCAN_T4.h").read_text()
    assert re.search(r"^\s*#\s*define\s+%s\b" % MACRO, hdr, re.M), "patched FlexCAN_T4.h does not define %s" % MACRO
