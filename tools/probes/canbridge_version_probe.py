#!/usr/bin/env python3
"""Can-bridge firmware Get_Version bench probe (Phase 3 gate validator).

READ-ONLY: issues only the GET_AXIS_VERSIONS RPC (a bridge-LOCAL cache read — no
CAN3 round-trip) and decodes the cached ODrive versions. It NEVER commands a motor
and NEVER writes RobotState (honours the never-command-motors rule in
tools/probes/README.md). Safe with motor power OFF — the ODrives need only logic
power + CAN3 to heartbeat, which is all the firmware's Get_Version sweep requires
to populate the cache.

Validates the Phase-3 probe gate (plan canbridge-foundation-coldstart-parity
Testing-plan probe table; logbook 2026-06-29-canbridge-phase3-version-validated):
do the live legs+hand ODrives report the EXPECTED_HW_VERSIONS tuple? A mismatch
means the expected-version config is stale (or a wrong-firmware ODrive), and the
Phase-4 powered cold-start would force-FAULT — reconcile before Phase 4.

Mirrors the Jetson production path: the bridge firmware sweeps Get_Version and
caches the raw replies; this probe pulls them via GET_AXIS_VERSIONS and runs the
same hw-match + fw-consistency check as MotorStateTracker.validate_group (inlined
here so the probe has NO ROS2 dependency — it runs in the bare venv).

Run:  python tools/probes/canbridge_version_probe.py
(needs the can-bridge powered + Phase-3 firmware flashed + the Jetson teensy-link
iface up; the ROS bridge node must NOT be running — it binds the same UDP ports.)
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path("/home/jetson/Desktop/Jugglebot")))
sys.path.insert(0, str(Path("/home/jetson/Desktop/Jugglebot/ros_ws/src/jugglebot")))

from controller.teensy_link import protocol as p
from controller.teensy_link.client import TeensyLinkClient
from controller.teensy_link.rpc import RpcClient, RpcError, RpcTimeout
from controller.teensy_link import rpc_args
from jugglebot.can import odrive
import jugglebot.hardware_config as hw_cfg

JUGGLEBOT_AXES = list(odrive.JUGGLEBOT_AXES)              # legs 0..5 + hand 6
EXPECTED_HW = {aid: tuple(getattr(hw_cfg, f"ODRIVE_VER_AXIS_{aid}"))
               for aid in JUGGLEBOT_AXES}


def _validate_group(versions):
    """Inlined mirror of MotorStateTracker.validate_group (no ROS dep): per-axis hw
    check + fw consistency among axes sharing the same expected hw. versions:
    {aid: (fw_tuple, hw_tuple)}. Returns an error string or None."""
    errors = []
    for aid in JUGGLEBOT_AXES:
        hw = versions.get(aid, (None, None))[1]
        exp = EXPECTED_HW.get(aid)
        if exp is not None and hw != exp:
            errors.append(f"Axis {aid} hw mismatch — expected "
                          f"{exp[0]}.{exp[1]}.{exp[2]}, got "
                          f"{hw[0]}.{hw[1]}.{hw[2]}" if hw else
                          f"Axis {aid} hw mismatch — expected "
                          f"{exp[0]}.{exp[1]}.{exp[2]}, got NONE")
    groups = {}
    for aid in JUGGLEBOT_AXES:
        exp = EXPECTED_HW.get(aid)
        fw = versions.get(aid, (None, None))[0]
        groups.setdefault(exp, {}).setdefault(fw, []).append(aid)
    for hw_ver, fw_set in groups.items():
        if len(fw_set) > 1:
            parts = [f"axes {ids}: fw {v[0]}.{v[1]}.{v[2]}" for v, ids in fw_set.items()]
            errors.append(f"Jugglebot firmware mismatch among hw "
                          f"{hw_ver[0]}.{hw_ver[1]}.{hw_ver[2]} axes — " + ", ".join(parts))
    return "; ".join(errors) if errors else None


def main():
    link = TeensyLinkClient()             # default peer 192.168.42.2:5005/5006
    link.start()
    rpc = RpcClient(link, default_timeout=0.5, default_retries=2)
    time.sleep(0.5)

    # Poll GET_AXIS_VERSIONS until the firmware sweep has populated every Jugglebot
    # axis (or a deadline). The sweep sends one Get_Version per cold-start-monitor
    # tick after each axis heartbeats, so the cache fills within a few hundred ms of
    # the ODrives being up.
    print("=== polling GET_AXIS_VERSIONS until all Jugglebot axes report (<=15 s) ===")
    deadline = time.monotonic() + 15.0
    per_axis = {}
    while time.monotonic() < deadline:
        try:
            blob = rpc.call(int(p.RpcMethod.GET_AXIS_VERSIONS), timeout=0.5, retries=2)
        except (RpcError, RpcTimeout) as e:
            print(f"  GET_AXIS_VERSIONS RPC failed: {e}  (old firmware? link down?)")
            time.sleep(0.5)
            continue
        per_axis = rpc_args.decode_axis_versions_result(blob)
        if set(JUGGLEBOT_AXES).issubset(per_axis.keys()):
            break
        time.sleep(0.5)

    print(f"\n=== {len(per_axis)} axis/axes reported ===")
    versions = {}
    for aid in JUGGLEBOT_AXES:
        exp = EXPECTED_HW[aid]
        if aid not in per_axis:
            print(f"  axis {aid}: NO VERSION (not heartbeating / not swept)  "
                  f"expected hw {exp[0]}.{exp[1]}.{exp[2]}")
            continue
        v = odrive.decode_get_version(per_axis[aid])
        fw = (v[4], v[5], v[6])
        hw = (v[1], v[2], v[3])
        versions[aid] = (fw, hw)
        tag = "unreleased" if v[7] else "release"
        mark = "OK" if hw == exp else "MISMATCH"
        name = "hand" if aid == JUGGLEBOT_AXES[-1] else f"leg{aid}"
        print(f"  axis {aid} ({name}): hw {hw[0]}.{hw[1]}.{hw[2]}  "
              f"fw {fw[0]}.{fw[1]}.{fw[2]} ({tag})  "
              f"expected hw {exp[0]}.{exp[1]}.{exp[2]}  {mark}")

    print("\n=== VERDICT ===")
    if not set(JUGGLEBOT_AXES).issubset(per_axis.keys()):
        missing = [a for a in JUGGLEBOT_AXES if a not in per_axis]
        print(f"  INCONCLUSIVE — axes {missing} reported no version. Check ODrive "
              "logic power + CAN3 (the sweep needs heartbeats), and that Phase-3 "
              "firmware is flashed (old firmware answers ERR_NOT_IMPL).")
    else:
        err = _validate_group(versions)
        if err:
            print(f"  FAIL — {err}")
            print("  -> EXPECTED_HW_VERSIONS is stale OR a wrong-firmware ODrive; the")
            print("     Phase-4 powered cold-start would force-FAULT. Reconcile before Phase 4.")
        else:
            print("  PASS — all Jugglebot axes match EXPECTED_HW_VERSIONS; validate_group clean.")
            print("     firmware_validated would latch True on the production bridge.")

    rpc.close()
    link.stop()


if __name__ == "__main__":
    main()
