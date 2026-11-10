---
title: "BUG-7: the levelling offset persists in rounded 0.1 mrad counts, not truncated whole milliradians (can-bridge FW 43, Platform FW 10, PROTOCOL 14)"
type: bugfix
date: 2026-11-10
status: complete
phase: "BUG-7"
subsystem: platform-teensy
files_changed:
  - config/protocol_config.yaml
  - config/generate_config.py
  - config/generate_udp_protocol.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/platform_relay.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - teensy_link/rpc_args.py
  - ros_ws/docs/levelling_frame.md
  - tests/firmware/native/test_platform_relay.cpp
  - tests/firmware/test_platform_fw_version_xref.py
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/hardware/session_tilt_calibration.md
  - tests/hardware/tilt_cal_grid.py
  - tests/ros/test_teensy_bridge_node_coldstart.py
  - tests/ros/test_teensy_bridge_node_relay.py
  - tests/teensy_link/test_hand_move_to.py
  - tests/teensy_link/test_protocol_codec.py
  - tests/teensy_link/test_v5_wire_regression.py
  # Generated from the two config generators above and not listed one by one:
  # config/generated/*, every consumer protocol_config.h and protocol_config.py,
  # udp_protocol.h, tools/probes/teensy_link_profiling/jetson/udp_protocol.py and
  # docs/teensy-udp-protocol.md.
---

# BUG-7: the levelling offset persists in rounded 0.1 mrad counts

## Symptom

A fresh `level` flies the full float offset, but a relaunch restores the copy held on the Platform Teensy. That copy was truncated to whole milliradians: 0.0264 deg lost on 2026-07-27 (`[0.014455, 0.00207]` persisted as `[0.014, 0.002]`), and 0.0573 deg worst case per axis against the +-0.05 deg band. LVL-3 cannot see it, because it scores the commanded park against the same quantised value.

## Diagnosis

The Item and the July sitting named the Platform Teensy's cast. The loss on the write path is earlier: the can-bridge's `state_write` truncated the float to integer mrad before the frame ever reached the Platform Teensy. The Platform Teensy held `x / 1000.f` and re-encoded it with a second truncating cast. A one-off single-precision probe of that round trip (2026-11-10, measured once, script not retained) lost a count at only 4 of the 601 values in +-300 mrad (+-251, +-253), all far above any real offset. So the bridge was the defect and the Platform Teensy's float was a latent one.

## Discussion

**Why finer units and not only `lroundf`.** Rounding whole mrad removes the bias but leaves 0.5 mrad (0.0286 deg, 57 % of the band) between a fresh `level` and the same `level` after a relaunch, and leaves the "score against the persisted value" caveat in force. Either fix needs a can-bridge flash, so the extra cost of finer units is a host decode change and a protocol bump. Rounded 0.1 mrad counts leave 0.0029 deg. The owner chose finer units on 2026-11-10.

**Why PROTOCOL_VERSION moves when no frame layout does.** The failure this prevents is a stale peer reading the offset ten times wrong: 14.5 mrad read as 145 mrad is 8.3 deg of commanded tilt. Platform and bridge FW skew are logged and never refused, so a version number alone would not stop it. A protocol mismatch makes the link dark, which is the only enforced refusal available.

**Why the Platform Teensy stores the raw integer.** It only stores and echoes the offset, so it needs no scale. An `int16_t` removes both the float round trip and its truncating cast, and a future scale change cannot touch this board.

**A gap accepted.** `teensy_bridge_node.py` runs from the colcon install while `teensy_link/` runs from the live tree, so a stale install stays on the link and decodes counts ten times too large. The launch's `_install_drift` check reports that but does not refuse it. It is closed here by rebuilding in the same unit, not by a new refusal; the install-skew checks stay advisory by their existing policy.

**Control implications.** The offset is written once per `level` and pushed to `trajectory_node` once per boot, so nothing in the 40 Hz path changes and no mid-stream step is possible. Against an unchanged `level`, the first relaunch after this fix moves the Platform by less than 1 mrad, toward the true value. The level reference's own scatter is 1.2 to 1.7 mrad per axis (`tests/hardware/tilt_cal_grid.py`).

## Fix

- One scale, `platform_state.TILT_COUNTS_PER_RAD: 10000` in `config/protocol_config.yaml`, generated for C++ and Python.
- `platform_relay.cpp` rounds with `std::lround` and returns `ERR_BAD_ARGS` for a finite value past the int16 span (+-3.2767 rad), so it can never wrap.
- The Platform Teensy holds `int16_t` counts and echoes them exactly (FW 10). The host decodes with the generated constant.
- Can-bridge FW 43, PROTOCOL_VERSION 14, `PLATFORM_FW_VERSION_EXPECTED` 10.

## Verification

- 2026-11-10, `./run_tests.sh -q` on the rebased tree with every code and test change in it: parallel rc=0, serial rc=0, `RESULT: PASS`, 568 s, exit 0. Only this logbook line changed after that run. Nothing in `controller/` or `sim/` changed, so `--full` was not required.
- New tests: `state_write` rounds symmetrically (0.014455 rad to 145 counts, -0.014455 to -145), accepts the exact edge of 3.2767 rad (32767 counts) and refuses 3.2768 rad (32768 counts, the first that would wrap); the shipped Platform codec echoes +145 and -21 count for count.
- Flashed 2026-11-10 from this unit's firmware source, launch down; every edit after the flash was tests or docs. Can-bridge: `pio run -e teensy41 -t upload`, SUCCESS. Platform Teensy over CAN: `pio run -e teensy40 -t upload`, receipt `FW version: 9 -> 10`.
- Link receipt, 2026-11-10, `tools/teensy_link_bridge.py --duration 15` at PROTOCOL 14: 6224 frames received, `crc_err=0`, `decode_err=0`.
- Motion-free read-back on the flashed boards, 2026-11-10, a throwaway RPC script with both flags false: STATE_WRITE of `(0.014455, 0.00207)` read back `(145, 21)` counts where the old path held `(14, 2)` mrad; the negated pair read back `(-145, -21)`; 3.3 rad returned `ERR_BAD_ARGS`; zeros restored and confirmed.

## Robot check (2026-11-10, passed; BUG-7 closed)

From the launch logs under `~/.ros/log/`. The 19:00 launch ran `level`, and its last "Gravity offset published" line read `[-0.0018433, +0.0057132]` rad. The 19:07 and 19:09 relaunches, without re-levelling, each restored `(-0.0018, +0.0057)` rad, shown identically on the bridge's cold-start read, the orchestrator's "Restored saved level" line and `trajectory_node`'s "level correction updated" line. That is -18 and 57 counts, so the four printed decimals are the exact value.

| Axis | Error now | Error with the old truncation |
|---|---|---|
| x | 0.043 mrad | 0.84 mrad |
| y | 0.013 mrad | 0.71 mrad |

Both errors are inside the 0.1 mrad pass criterion and the 0.05 mrad rounding bound. Every launch logged `PLATFORM_FW_CHECK: OK — Platform Teensy v10` and `BRIDGE_FW_CHECK: OK — can-bridge v43 [install_skew=0]`, with no Fault line. The three `level` runs in the 19:00 launch spread 1.1 mrad on x, which is the level sensor's own scatter, not this fix.
