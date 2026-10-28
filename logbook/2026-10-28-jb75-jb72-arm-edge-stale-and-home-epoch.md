---
title: "can-bridge FW 34: SETPOINT_STALE measures from the arm edge and output waits for a post-arm setpoint (JB-75); a per-home epoch in the Diagnostic attributes a fast home's result (JB-72), and the 2026-10-27 bench false-timeout was the bench driver dropping homing_result"
type: investigation
date: 2026-10-28
status: in-progress
phase: "JB-75 / JB-72 - can-bridge arm edge and homing attribution"
related_entries:
  - 2026-10-27-leg-lead-clamp-age-aware-p3-robot.md
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_homing.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_homing.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - teensy_link/homing.py
  - teensy_link/rpc_args.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/hardware/teensy_home_bench.py
  - tests/teensy_link/test_homing.py
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/firmware/native/test_fault_machine.cpp
  - tests/firmware/native/test_leg_homing.cpp
subsystem:
  - can-bridge
  - teensy_link
tags:
  - setpoint-stale
  - homing
  - firmware
---

# can-bridge FW 34: arm-edge staleness (JB-75) and a per-home epoch (JB-72)

## What

Two can-bridge defects from the 2026-10-27 lead-clamp Sittings, fixed in one firmware bump (FW 33 -> 34, PROTOCOL_VERSION stays 10). Built, unit-tested natively and on the host. **Not flashed and not bench-verified**: the dongle reached the ROBOT's bridge (BridgeIdentity FW 33 / proto 10, all seven axes present and homed), not the single-leg bench rig, so the flash was left to the owner's session.

- **JB-75.** `s_last_setpoint_us` survives across arms. On a bridge that streamed before, a 10 Hz fault tick landing between a re-arm and the first new setpoint (16-37 ms) saw the previous stream's age and latched SETPOINT_STALE.
- **JB-72.** `HomingMonitor` trusted a terminal `homing_result` only after seeing RUNNING.

## Findings

- **The JB-72 bench symptom had a simpler cause than the item says.** `tests/hardware/teensy_home_bench.py` built `AxisStatus` without `homing_result`, so the monitor read NONE for ever and *every* home on that driver false-timed-out (`temp/logs/lead-clamp-p2-home-2.log`: `saw_running=False` on a successful 0.8 s home).
- **The 1 Hz premise is also wrong for the Diagnostic.** The firmware forces a Diagnostic on every `homing_result` transition (telemetry.cpp, 100 Hz check), so production `_run_home`, which does pass the result, normally sees RUNNING. Its remaining exposure is narrower: a short home whose single forced RUNNING frame is lost on the UDP stream. A RUNNING->FAILED inside one homing tick is a third case, but that one already fails safe.
- **A latent u64 wrap in the old staleness check.** `micros64() - interp_last_setpoint_us()` left the evaluation order unspecified. If the net task stamped a setpoint between the two reads, the stamp would sit after `now` and the age would wrap into a false trip. The window is tiny, and the new code reads both stamps before `now`.

## Discussion

**Why a separate arm-edge stamp beats writing `s_last_setpoint_us` on the arm (and beats calling `interp_reset()`).** `s_last_setpoint_us` also means "last ACCEPTED setpoint" to `interp_on_setpoint`'s seq re-baseline: a stream older than the staleness bound is treated as dead, so its seq high-water is forgotten. The host restarts its seq counter on every bridge-node restart. If the arm stamped that field, the first post-restart frame would see a fresh age and skip the re-baseline. Every frame would then be dropped as stale-seq, and the arm would trip 250 ms later anyway, which is the phantom latch the re-baseline exists to prevent. `interp_reset()` at the arm edge is worse. It runs in the net task against ISR-owned interp state (latches, knot history, slew state, the hand lane) that only a quiescent caller may zero, and it would drop a frame staged by stream-then-arm. So `fault_set_mpc_active` stamps `s_arm_edge_us` on the false->true edge, before setting `s_mpc_active`, and the check uses `max(last setpoint, arm edge)`.

**Why output also waits for a post-arm setpoint.** The grace alone would turn the old latch into something worse. In that window the guard would ENABLE output with the interp's latched base still set to the PREVIOUS stream's last frame, and the recovery slew would start moving the legs toward a pose nobody commands now (homing, activate or deactivate may have moved the legs since then). `sp_since_arm = last_sp >= arm_edge` gates `allow` only. `guard_mode` still reads ENABLED, so the host, which streams after it arms, is not deadlocked. Output then opens on the first fault tick after a fresh frame, from the encoder through the usual slew, which is the same path a lucky pre-FW 34 arm took. Walking one tick through each case:
- dead stream after an arm: held until arm + 250 ms, then SETPOINT_STALE;
- mid-Attempt stream death: unchanged (age from the last frame);
- clear_errors with a dead stream and mpc_active still set: re-latches at once, unchanged;
- a never-streamed bridge: `ever_cmd` false, so no latch and no hold. It has no previous base, because the interp has not latched and transmits nothing until a frame lands. The first gate draft held this case too, and three native hand-deviation tests caught it: they enable output before their first frame and need that frame to transmit on its own ISR tick.

**Why the epoch beats holding RUNNING longer.** The other way to make RUNNING observable is to hold it in the firmware for at least one Diagnostic period, or until the host acknowledges it. That slows every home, couples a firmware timer to a host poll rate, and still loses to a dropped datagram. Telling OK->RUNNING->OK apart from a stale OK needs information the result field cannot carry. The epoch carries it:
- it is bumped in `homing_request` inside the same IRQ-guarded section that now sets RUNNING at accept;
- it is read as a pair with the result (`homing_status`), so no frame can pair a new epoch with the previous home's OK;
- the monitor records it from the last status before it asks for HOME and trusts a terminal result under a newer epoch (u8, wrap-safe; 128..255 forward reads as older, so after a bridge reboot the home fails false, never succeeds false).

`saw_running` is kept as the second trust path, and the silent-abort guard is unchanged (an abort reports FAILED under the new epoch).

**Why PROTOCOL_VERSION was not bumped (deviates from the brief).** The epoch takes the Diagnostic's zero pad byte, so the payload size is unchanged. An older host ignores the byte, and a newer host on an FW <= 33 board reads epoch 0 for ever, which never reads newer, so it falls back to `saw_running`. Both mismatches degrade to today's behaviour. The repo's rule is that a same-size additive change keeps the version and re-pins the layout hash (the u32-flags precedent in `generate_udp_protocol.py`), and a bump would have darkened the robot link until a lockstep flash. `EXPECTED_BRIDGE_FW_VERSION` 33 -> 34 makes the skew loud in the GUI instead.

## Fix

As above. Also: the bench driver now passes `homing_result` + `homing_epoch`, prints with `flush=True`, and imports its own checkout's `teensy_link` rather than a hard-coded `~/Desktop/Jugglebot`. `_homing_axis_status` passes the epoch.

## Verification

- Firmware build (`pio run -d ros_ws/src/jugglebot/Teensy_code_canbridge -e teensy41`, 2026-10-28): SUCCESS, no new warnings.
- Scoped tests (2026-10-28, `pytest -q tests/firmware/test_native_firmware.py tests/firmware/test_udp_protocol_xlang.py`, after the never-streamed fix): **63 passed in 287.75 s**. `pytest -q tests/teensy_link/test_homing.py`: **20 passed** (8 new epoch cases).
- Gate (2026-10-28, `./run_tests.sh`): **7369 passed, 13 skipped in 367.81 s; serial tail 5 passed; exit 0**.
- Wire check (2026-10-28, read-only sniff `scratchpad/F/sniff_bridge.py`, mpc_active=0): BridgeIdentity **FW 33 / proto 10**, axes 0-6 all `heartbeat_seen` and `homing_result=2`. That is the robot, not the single-leg bench rig, so **nothing was flashed**.
- **Outstanding (hardware):** flash FW 34 on the bench bridge or the robot's bridge (owner's session, launch down), then:
  - re-run `python -u tests/hardware/teensy_home_bench.py` for one normal home and one from within ~1 rev of the stop, both `succeeded=[0]`;
  - for JB-75, arm, stream, disarm, wait > 250 ms, re-arm with no latch, and check that a dead stream after an arm still latches at ~250 ms.

## Flash receipt (main session, 2026-10-28 12:50 AEDT)

Flashed from the merged tree (main `3e7cc38a`) with the launch down: `pio run -e teensy41 -t upload`,
SUCCESS in 51 s. Receipt: BridgeIdentity `(fw 34, protocol 10)`, Diagnostics for axes 0-6 present
and fresh, homed state cleared by the reboot as usual (re-home at the next Sitting). The two
on-robot checks above (fast home, arm-edge re-arm) are still outstanding and gate the close of
JB-75 and JB-72.
