#!/usr/bin/env python3
"""Hash-cached host build of the native firmware test binaries.

Compiles the REAL safety-critical firmware (`fault_machine.cpp`, `leg_interp.cpp`)
on the build host behind the ~10-line HAL shims + the fake HAL, and links the
per-module test drivers (which `#include` the .cpp to reach its file-statics).
The build is:

  * Deterministic — fixed flags, same sources → byte-identical binaries.
  * Hash-cached — a stamp over every source's CONTENTS + the flag string + the
    g++ version. Unchanged sources → zero recompilation (so `pytest tests/ -q`
    pays the ~10 s build only when the firmware or the harness actually changes).
  * g++-gated — `have_gpp()` lets the pytest wrapper degrade to *skipped* (not
    red) on a host without a compiler. The Jetson run is authoritative.

No .cpp is ever both #included AND linked-as-object in the same binary (the rule
that avoids ODR clashes):
  * test_fault_machine #includes fault_machine.cpp AND leg_interp.cpp (the two
    share no symbol names, so one TU is ODR-clean) — so it can drive the static
    interp_isr() to complete a stow and assert the fault machine's terminal-IDLE
    handling (deferred-stow invariant 5) as a COMPILED assertion. Both .cpp are
    #included once; neither is also linked as an object.
  * test_leg_interp    #includes leg_interp.cpp (does NOT link leg_interp.o)

Importable (the pytest wrapper calls build_all()) and runnable
(`python build.py` to build, `python build.py --golden <path>` to emit goldens).
"""

from __future__ import annotations

import hashlib
import shutil
import subprocess
import sys
from pathlib import Path

NATIVE_DIR = Path(__file__).resolve().parent
SHIM_DIR = NATIVE_DIR / "hal_shims"
REPO_ROOT = NATIVE_DIR.parents[2]
FIRMWARE_DIR = REPO_ROOT / "ros_ws" / "src" / "jugglebot" / "Teensy_code_canbridge"
# All runtime build artifacts live under temp/ (gitignored), per the repo rule.
CACHE_DIR = REPO_ROOT / "temp" / "firmware_native"

# Fixed, reproducible flags. Exceptions/RTTI are ENABLED (doctest uses them); the
# compiled firmware TUs neither throw nor use RTTI, so this does not change their
# behaviour. -Wno-unused-function silences firmware helpers unused in a test TU.
FLAGS = ["-std=c++17", "-O2", "-Wall", "-Wno-unused-function"]
INCLUDES = ["-I", str(SHIM_DIR), "-I", str(NATIVE_DIR), "-I", str(FIRMWARE_DIR)]

# Shared firmware/HAL objects compiled once and linked per binary. leg_interp is
# NOT here: it is #included by both drivers (so they reach its file-statics +
# static interp_isr), never linked as an object.
_OBJECTS = {
    "axis_state":        FIRMWARE_DIR / "axis_state.cpp",
    "ball_butler_state": FIRMWARE_DIR / "ball_butler_state.cpp",
    "fake_hal":          NATIVE_DIR / "fake_hal.cpp",
    # Cold-start move drivers (homing/activate/deactivate): a SEPARATE self-contained fake HAL (never linked
    # alongside fake_hal.o) for the leg_homing/activate/deactivate drivers — their
    # bus/fault gate + *_active() ODR needs differ from the fault/interp TUs.
    "coldstart_hal":     NATIVE_DIR / "coldstart_hal.cpp",
}

# Test binaries: (driver source, objects to link). Each driver #includes the .cpp
# it tests (fault_machine.cpp/leg_interp.cpp), so those are NOT in the link list.
_BINARIES = {
    "test_fault_machine": (
        NATIVE_DIR / "test_fault_machine.cpp",
        ["axis_state", "ball_butler_state", "fake_hal"],
    ),
    "test_leg_interp": (
        NATIVE_DIR / "test_leg_interp.cpp",
        ["axis_state", "ball_butler_state", "fake_hal"],
    ),
    # Platform-Teensy relay seam. #includes platform_relay.cpp; the
    # fake HAL supplies can_jugglebot_send + jugglebot_commands_allowed, and
    # is_platform_reply_id / hand_axis6_permitted are header-inline / generated.
    "test_platform_relay": (
        NATIVE_DIR / "test_platform_relay.cpp",
        ["axis_state", "ball_butler_state", "fake_hal"],
    ),
    # Get_Version sweep + raw-version cache. #includes version_check.cpp;
    # the fake HAL supplies can_jugglebot_send + jugglebot_commands_allowed + the
    # inbound-CAN3 injection hook, axes[] comes from axis_state.o.
    "test_version_check": (
        NATIVE_DIR / "test_version_check.cpp",
        ["axis_state", "ball_butler_state", "fake_hal"],
    ),
    # Hand ball-sensor poller. #includes gpio_poll.cpp AND version_check.cpp (the
    # Get_Version cache its gate reads — the two share no symbol names, so one TU
    # is ODR-clean); the fake HAL supplies can_jugglebot_send +
    # jugglebot_commands_allowed + the clock + time_synced, and the Serial stub
    # (hal_shims/Arduino.h) swallows the console/park lines.
    "test_gpio_poll": (
        NATIVE_DIR / "test_gpio_poll.cpp",
        ["axis_state", "ball_butler_state", "fake_hal"],
    ),
    # Hand traj / smooth-move conduit. #includes hand_ops.cpp; the fake
    # HAL supplies can_jugglebot_send + jugglebot_commands_allowed (+ the send-fail
    # hook for the preamble-abort test). The ODrive encoders + 0x6D0 id are
    # header-inline / generated.
    "test_hand_ops": (
        NATIVE_DIR / "test_hand_ops.cpp",
        ["axis_state", "ball_butler_state", "fake_hal"],
    ),
    # Cold-start move ladders. Each driver #includes ONE real
    # module .cpp (defining its own *_active) + supplies the two sibling predicates
    # inline, and links coldstart_hal (NOT fake_hal — that would ODR-clash on the
    # module's own *_active and lacks the can_buses_stats/fault gate these use).
    "test_leg_activate": (
        NATIVE_DIR / "test_leg_activate.cpp",
        ["axis_state", "ball_butler_state", "coldstart_hal"],
    ),
    "test_leg_deactivate": (
        NATIVE_DIR / "test_leg_deactivate.cpp",
        ["axis_state", "ball_butler_state", "coldstart_hal"],
    ),
    "test_leg_homing": (
        NATIVE_DIR / "test_leg_homing.cpp",
        ["axis_state", "ball_butler_state", "coldstart_hal"],
    ),
    # UDP-framing coverage: EXECUTE the hand-written C++ UDP framing (the network
    # trust boundary). Pure header (udp_protocol.h) — no linked objects.
    "test_udp_framing": (
        NATIVE_DIR / "test_udp_framing.cpp",
        [],
    ),
    # NetLock-coverage hardening: EXECUTE udp_link.cpp's NetLock coverage + RX drain budget. The
    # fake QNEthernet + recursive-mutex shims (hal_shims/QNEthernet.h,
    # arduino_freertos.h, net_lock_probe.h) let it assert that Ethernet.loop() and
    # every socket call run under NetLock, and that drain_socket is bounded by
    # UDP_RX_DRAIN_BUDGET. #includes udp_link.cpp (reaches its file-statics); the two
    # non-inline externals (micros64/net_ethernet_service) are stubbed in the driver,
    # so no linked objects.
    "test_udp_link": (
        NATIVE_DIR / "test_udp_link.cpp",
        [],
    ),
    # RPC-dispatch coverage: EXECUTE rpc.cpp dispatch()/send_axis_frame() — the
    # (method,axis) enforcement point + gate-basis routing. #includes rpc.cpp and
    # links the standard triple; the leg/hand/relay/version handlers are stubbed
    # inline in the driver (routing isolation), so no leg_*/relay/version .cpp link.
    "test_rpc_dispatch": (
        NATIVE_DIR / "test_rpc_dispatch.cpp",
        ["axis_state", "ball_butler_state", "fake_hal"],
    ),
    # Clapboard CLAP_LINK beacon. #includes clap_link.cpp; the fake HAL supplies
    # the controllable clock (micros64), and can_cone_send — which fake_hal does
    # NOT define, clap_link being its first caller besides the time-sync fan-out —
    # is a recording stub in the driver.  clap_link_step takes link state as a
    # PARAMETER, which is the whole reason this TU is reachable from a host build
    # at all (link_state() is static in the .ino, which no test compiles).
    "test_clap_link": (
        NATIVE_DIR / "test_clap_link.cpp",
        ["axis_state", "ball_butler_state", "fake_hal"],
    ),
    # ODrive-encoder coverage: EXECUTE the real odrive_protocol.h encoders + emit a
    # cross-language golden (pinned by the Python xref to odrive.py). Pure header.
    "test_odrive_protocol": (
        NATIVE_DIR / "test_odrive_protocol.cpp",
        [],
    ),
    # Ball-butler-encoder coverage: EXECUTE the real ball_butler_protocol.h encoders + emit a
    # cross-language golden (pinned by the Python xref to ball_butler.py). Pure header
    # (the driver defines its own fixed clock for the deterministic throw_time).
    "test_ball_butler_protocol": (
        NATIVE_DIR / "test_ball_butler_protocol.cpp",
        [],
    ),
}


def have_gpp() -> bool:
    return shutil.which("g++") is not None


def _gpp_version() -> str:
    try:
        return subprocess.check_output(["g++", "--version"], text=True).splitlines()[0]
    except Exception:
        return "unknown"


def _source_hash() -> str:
    """Stamp over every input that can change a binary: all native harness files
    (shims, fake_hal, drivers, doctest.h), all firmware .cpp/.h, the flags, and
    the compiler version. Any change → cache miss → rebuild."""
    h = hashlib.sha256()
    h.update(" ".join(FLAGS).encode())
    h.update(_gpp_version().encode())
    h.update(Path(__file__).read_bytes())   # this file defines INCLUDES/_OBJECTS/_BINARIES
    files = []
    for d in (NATIVE_DIR, SHIM_DIR):
        files += sorted(p for p in d.glob("*") if p.is_file() and p.suffix in (".h", ".cpp"))
    files += sorted(p for p in FIRMWARE_DIR.glob("*") if p.suffix in (".h", ".cpp", ".ino"))
    for p in sorted(set(files)):
        h.update(p.name.encode())
        h.update(p.read_bytes())
    return h.hexdigest()


def _run(cmd: list) -> None:
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(
            "native build command failed:\n  "
            + " ".join(str(c) for c in cmd)
            + f"\n--- stdout ---\n{res.stdout}\n--- stderr ---\n{res.stderr}")


def build_all(force: bool = False) -> dict:
    """Build both test binaries (hash-cached). Returns {name: Path}. Raises if g++
    is missing (callers should check have_gpp() first to skip gracefully)."""
    if not have_gpp():
        raise RuntimeError("g++ not found — native firmware harness needs a host C++ compiler")
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    key = _source_hash()
    stamp = CACHE_DIR / "build.stamp"
    bins = {name: CACHE_DIR / name for name in _BINARIES}

    if (not force and stamp.exists() and stamp.read_text() == key
            and all(b.exists() for b in bins.values())):
        return bins

    # Cache miss: recompile all shared objects, relink all binaries.
    objs = {}
    for name, src in _OBJECTS.items():
        obj = CACHE_DIR / f"{name}.o"
        _run(["g++", *FLAGS, *INCLUDES, "-c", str(src), "-o", str(obj)])
        objs[name] = obj
    for name, (driver, link_objs) in _BINARIES.items():
        _run(["g++", *FLAGS, *INCLUDES, str(driver),
              *[str(objs[o]) for o in link_objs], "-o", str(bins[name])])
    stamp.write_text(key)
    return bins


def main(argv) -> int:
    if not have_gpp():
        print("g++ not found — cannot build the native firmware harness", file=sys.stderr)
        return 2
    bins = build_all(force="--force" in argv)
    print("Built native firmware test binaries:")
    for name, path in bins.items():
        print(f"  {name}: {path}")
    if "--golden" in argv:
        out = argv[argv.index("--golden") + 1]
        _run([str(bins["test_fault_machine"]), "--emit-golden", out])
        print(f"Emitted fault goldens → {out}")
    if "--odrive-golden" in argv:
        out = argv[argv.index("--odrive-golden") + 1]
        _run([str(bins["test_odrive_protocol"]), "--emit-golden", out])
        print(f"Emitted odrive goldens → {out}")
    if "--bb-golden" in argv:
        out = argv[argv.index("--bb-golden") + 1]
        _run([str(bins["test_ball_butler_protocol"]), "--emit-golden", out])
        print(f"Emitted BB goldens → {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
