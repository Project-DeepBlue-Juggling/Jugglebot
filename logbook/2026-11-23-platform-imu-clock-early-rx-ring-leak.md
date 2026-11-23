---
title: "Platform clock ran early by a standing receive-queue delay (FlexCAN_T4 _available leak, stock library): IMU stamps 23 to 55 ms early and every command to the Platform late; FW 15 stamps 0x7DD at reception (BUG-132)"
type: investigation
date: 2026-11-23
status: in-progress
phase: "carry-ring-zvd"
related_plan: "carry-ring-zvd.md"
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino
  - ros_ws/src/jugglebot/Teensy_code_platform/sync_rx.h
  - ros_ws/src/jugglebot/Teensy_code_platform/imu_service.h
  - ros_ws/src/jugglebot/Teensy_code_platform/imu_capture.h
  - ros_ws/src/jugglebot/Teensy_code_platform/platformio.ini
  - ros_ws/src/jugglebot/Teensy_code_canbridge/lib/FlexCAN_T4/FlexCAN_T4.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/lib/FlexCAN_T4/PROVENANCE.md
  - ros_ws/docs/platform_imu_protocol.md
  - teensy_link/rpc_args.py
  - teensy_link/platform_imu.py
  - teensy_link/platform_imu_capture.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tools/probes/platform_rtt_probe.py
  - tools/probes/imu_clock_check.py
  - tests/firmware/native/test_platform_sync_rx.cpp
  - tests/firmware/test_platform_flexcan_build_gate.py
  - tests/teensy_link/test_platform_imu_clock_quality.py
  - tests/sim/test_imu_clock_check.py
  - map/bug-platform-imu-capture-timestamps-run-23-to-55-ms-early.md
subsystem: [platform-imu, firmware, ros, tools]
---

## Summary

BUG-132 (IMU capture stamps 23 to 55 ms early against Mocap and the encoders) is a Platform firmware defect, not a
Jetson or bridge clock error.

- The Platform Teensy was built against the stock FlexCAN_T4. Its 256-slot software receive ring has an `_available`
  counter that the receive interrupt and the loop's pop update without a lock. A lost update strands one frame for good,
  so the ring becomes a delay line of L frames. The can-bridge had the same defect and was fixed at its FW 14
  (logbook/2026-08-14-ring-audit-available-leak-delay-line.md); the Platform never got the patch.
- The 0x7DD time sync was stamped at dispatch (when the loop popped the frame), so the Platform wall clock ran early by
  the queue delay D = L / frame rate, and every command to the Platform (Hand fan, tilt read, STATE_UPDATE, IMU commands,
  FREEZE) arrived late by the same D. The FREEZE stamp cancels the error by construction, which is why it hid.
- FW 15 fixes it in firmware: the patched library, the beacon stamped in the interrupt at reception, and an envelope
  estimator. It is built and tested, not flashed.
- The queue explains the 2026-11-22 captures (37 to 49 ms) and the 61 ms slew at the 572D re-arm. It explains almost
  nothing on the short-uptime days (2026-11-20, 2026-11-23); their 14 to 22 ms is reference stamps that run late
  (BUG-134), which no firmware change removes.

## Symptoms

The identification study (logbook/2026-11-23-platform-ring-identification-from-the-imu.md) found the IMU stamps early
by 53 to 57 ms in the 2026-11-22 captures and 22 to 24 ms on 2026-11-23, constant inside a capture and different by day.
Round two of this study re-measured with the gyro against orientation rate (Kabsch fit, 15 Hz low pass, r 0.8 to 0.99).

## Diagnosis

Lag is the IMU stamp minus the reference stamp of the same event, so negative means the IMU is early. "Tick-restamped
encoders" means each `/robot_state` sample re-stamped to the bridge tick stamp of the same telemetry tick (paired with
`/leg_cmd_executed`), which puts the encoders on the bridge clock like the IMU.

| Capture | Day | vs Mocap | vs encoder FK | vs executed command | vs tick-restamped encoders |
| --- | --- | --- | --- | --- | --- |
| AEDF | 11-20 | -16.2 | -19.2 | +5.2 | -2.2 |
| AEE0 | 11-20 | -14.8 | -19.0 | +3.6 | -2.8 |
| AEE1 | 11-20 | -13.8 | -17.6 | +3.8 | -3.4 |
| 572E | 11-22 | -56.0 | -55.8 | -38.4 | -48.8 |
| 5730 | 11-22 | -50.2 | -53.6 | -33.2 | -43.0 |
| 5731 | 11-22 | -50.6 | -52.0 | -33.0 | -40.0 |
| 4AFA | 11-23 | -18.2 | -21.6 | +6.8 | -4.2 |

All values in ms. Against the bridge-stamped command the day difference of about 37 ms is already present, while the
bridge-to-Jetson error is under 10 ms every day and does not move: the Platform-to-bridge link is what differs.

- **The 572D slew.** The Platform's wall-minus-local offset rose by 61 ms in about 0.4 s (+34, +19, +5 ms per 120 ms
  anchor interval, a geometric decay, no SYNC_STEP flag) at the guard clear and re-arm. The model (255 slots times the
  change in inverse frame rate, 2,240 to 5,250 frames/s) predicts +65 ms, and the start time matches the logged ARMED.
  More traffic means a fresher beacon, so the sign (later) fits.
- **Saturated delay.** D_sat = 255 / frame rate: about 114 ms idle, 44 to 49 ms Armed at rest, 36 to 39 ms in an Attempt.
  The 2026-11-22 captures (Platform up about 2.8 h) sit at 37 to 49 ms.
- **FW 14 round-trip probe (2026-11-23, read-only, robot unpowered, launch down).** `STATE_READ` answered by the Platform
  against a bridge-only `NOP`, 100 each, three runs a minute apart: medians 44.87 / 46.01 / 45.57 ms against
  1.27 / 1.24 / 1.28 ms. Bridge receive rate 2,150 frames/s, so about 95 stranded frames. The bridge's own leak
  counters read 0 and its SDO round trip was flat at 0.9 ms, so the delay is on the Platform.
- **Why the FREEZE hid it.** FREEZE passes through the same ring: it arrives D late and is stamped with a clock that is D
  early, so `freeze_wall_us` agreed with the Jetson send to within about 10 ms on every day.

## Discussion

What was withdrawn or ruled out, in the order it happened.

- **Withdrawn: "a per-boot offset in the wall-time model."** The first round of data fit it (constant within a boot, different
  across boots, B-J constant). It is not a fixed offset. It is a queue delay that moves with bus traffic and with the
  stranded-frame count, which the 572D slew (61 ms inside one boot, at a re-arm) shows directly.
- **Withdrawn: "one frame per `events()` call strands the queue."** Refuted by the loop's 32-frame drain per pass, which is
  more than the 7.7k frames/s the bus can carry. Without the leak a queue cannot stand. The race on the `_available`
  read-modify-write is what strands frames (the interrupt can preempt the pop, never the reverse, so the count only
  under-reads).
- **Withdrawn: "the IMU should lag the command by about 20 ms."** That took the encoders' 19 to 26 ms lag behind the
  command as plant lag. It is the encoders' stamp lateness. With full velocity feedforward the Legs track the command
  within 1.3 to 3 ms (5.6 ms at 5 Hz), and the measured 0 to 5 ms plant lag agrees, so an IMU that lags the command by
  4 to 7 ms is correct.
- **Not supported: "the leak grows linearly with uptime."** A rate of 11 microseconds per second would put 24 ms into 4AFA
  at 36 min, but 4AFA is bounded under 4 ms by two independent checks. The rate probably depends on Armed traffic time
  (the race needs interrupt arrivals during pops). Unproven, and the reason the growth with Armed time is listed as
  untested below.
- **Not present: the 1 to 2 ms/s drift hint in 5731.** Window lags move under 0.25 ms/s there.
- **Excluded:** the bridge TX queue (the 0x7DD is stamped before TX; at most a few ms of arbitration, `tx_deferred` 0);
  a constant sample-to-wall bias in the IMU path (the stamp reads the live clock at every drain, and the 572D jump sits
  in the wall offset that only the sync handler writes); sensor group delay (a few ms at most); the fit sign
  (a sign error would not change by day).

### The residual 14 to 22 ms on short-uptime days

On 2026-11-20 and 2026-11-23 the queue share is about 0 (bound at most 4 ms) and the IMU still reads 14 to 22 ms early.
That is reference stamps running late:

- `/robot_state` is stamped at publish on the Jetson, 12 to 14 ms after the bridge tick (measured), plus an encoder
  sample age of 0 to 10 ms (about 5.5 ms, from code, not measured).
- The Mocap stamp is QTM frame time plus the minimum receive latency and omits QTM's own latency, about 14 to 16 ms
  (inferred from the budget, never measured). Mocap and the encoders agree within 1.4 to 4 ms only because two
  different latenesses on the same Jetson clock happen to match.

| Term (ms) | AEDF | AEE0 | AEE1 | 4AFA | Grade |
| --- | --- | --- | --- | --- | --- |
| Observed IMU minus encoders | -19.2 | -19.0 | -17.6 | -21.6 | measured |
| Encoder stamp lateness after the tick | 11.8 | 13.1 | 14.1 | 14.1 | measured |
| Encoder sample age | 5.5 | 5.5 | 5.5 | 5.5 | from code, assumed mean |
| Mocap lateness (receive stamp) | about 15.3 | about 14.5 | about 14.4 | about 16.3 | derived, inferred |
| Platform queue share | 4 or less | about 0 | about 0 | 4 or less | measured bound |
| IMU own delay (filter group delay) | +1.5 | +2.4 | +3.9 | +2.4 | derived |
| Unexplained | 0 +- 3 | 0 +- 3 | 0 +- 3 | 0 +- 3 | |

After the firmware fix a fresh short-uptime capture should still read about -14 to -22 ms against encoder FK and -14 to
-18 ms against Mocap as stamped. The fix removes only the term that grows with uptime (about 0 at short uptime, about
-40 ms after hours: 2026-11-22 would move from -52..-56 to about -14..-18). Restamping to the bridge tick brings the
encoders within about 3 ms; Mocap cannot validate the IMU to better than about 15 ms until its stamp is calibrated
(BUG-134).

### Why firmware and not a Jetson-side correction

- The error changes with traffic inside a capture (D = L / rate, and the rate steps at arm and in an Attempt), so one
  scalar per capture leaves ms-scale errors.
- FREEZE cancels the error by construction, so `freeze_wall_us` cannot measure it.
- Command latency to the Platform and the other devices that slave to 0x7DD are untouched by a correction on stamps.
- A model correction (D_sat from the bridge frame rate and arm state) is fit for annotating old captures, not for
  silently rewriting `t_wall_us`. Stamps are not rewritten.

### Why no hardware acceptance filter

The Platform can only be reflashed over CAN (frame 0x6F0; the USB port is dead). A filter that drops 0x6F0 bricks remote
updates, and the traffic counter (`receivedCount`) counts all frames and would change meaning. With the patched pop the
load is no longer a correctness hazard (32 frames drained per pass against at most 7.7k frames/s on the wire). If a
filter is wanted later, it needs the exact id list and a native test that every id the sketch tests is in the table.

## Fix

Platform FW 14 to 15 (`ros_ws/src/jugglebot/Teensy_code_platform/`):

1. `platformio.ini`: `lib_extra_dirs` points at the can-bridge's patched FlexCAN_T4, so one library copy serves both boards.
   The patched header defines `JUGGLEBOT_FLEXCAN_P3` (a macro only, the bridge ignores it) and the sketch `#error`s
   without it, so a silent fall-back to the framework copy fails the build. `PROVENANCE.md` § P5 records that editing the
   library now changes two firmwares.
2. `sync_rx.h` (new, pure header) and `Teensy_code_platform.ino`: 0x7DD is stamped in the receive interrupt through
   FlexCAN_T4's `ext_output1` hook (called for every frame before the software ring), into a 16-slot stamp ring.
   `TimeSync::service()` in the loop feeds the estimator.
3. Estimator: offset is the largest (wall minus local) sample over a 10 x 100 ms block window, because every residual error
   is an added delay. A delayed run shorter than 0.9 s changes nothing; a forward step is taken at once; a backward step
   is followed after 0.9 to 1.0 s. Moves over 5 ms report as a sync step.
4. Exports in reserved space: capture header bytes 58 to 61 (`sync_rx_delay_10us`, `rx_ring_depth_max`,
   `rx_ring_leak_max`) and diag bytes 26 to 29. Sizes and other offsets unchanged (`ros_ws/docs/platform_imu_protocol.md`).

Jetson side:

- `teensy_link/rpc_args.py`: `PLATFORM_FW_VERSION_EXPECTED` 14 to 15 (ships with the flash).
- `teensy_link/platform_imu.py`, `platform_imu_capture.py`: new header and diag layouts (null for FW below 15), a
  `clock_suspect` flag with reason, derived `sync_rx_delay_ms`. Flagged when `sync_rx_delay_10us` reads the saturated or unmatched
  sentinel or `rx_ring_leak_max` is over 0 (a finite delay is informational: a loop() stall raises it without moving the clock), and for every FW below 15 at upload from now on.
- `ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py`: `platform_imu/status` keys `last_clock_suspect`,
  `last_sync_rx_delay_ms`, `last_rx_ring_depth_max`, `last_rx_ring_leak_max`, and a warning on a leak.
- `tools/probes/platform_rtt_probe.py`: the round-trip probe (PASS at a floor excess of 5 ms or less, see Verification).
  `tools/probes/imu_clock_check.py` with `tests/sim/test_imu_clock_check.py`: capture plus bag to IMU lag against the
  tick-restamped encoders and the executed command (PASS within 5 ms and the IMU lagging the command by 0 to 10 ms).
- The seven measured captures (AEDF, AEE0, AEE1, 572E, 5730, 5731, 4AFA) carry a `bug132` annotation in their json with
  these lags (done 2026-11-23, by the author run of `tools/probes/imu_clock_check.py`: IMU early by 2.2 / 2.8 / 3.4 ms for
  AEDF / AEE0 / AEE1, 48.8 / 43.0 / 40.0 ms for 572E / 5730 / 5731, 4.2 ms for 4AFA, against tick-restamped encoders).
  Their stamps are not rewritten.

### Control-safety walk-through (one loop pass)

Added interrupt work is one id compare per received frame plus, for a beacon, an 8-byte decode, one `micros64()` and a
16-byte slot write (well under 0.5 microseconds). Loop code masks only IRQ_CAN1 for windows under 0.2 microseconds
(at most 32 pops and 16 copies per pass), against a FIFO that tolerates about 690 microseconds of latency. The Platform
drives no motion from wall time. Hand fan, tilt, STATE_UPDATE and IMU commands only get faster, since no standing
queue remains. `wall_offset_us` is now written and read only in loop context, so the 64-bit value cannot tear.

## Verification

Run on 2026-11-23 in the worktree on branch `bug132-platform-rx-ring`, nothing flashed.

- Native sync test: `python tests/firmware/native/build.py` then `temp/firmware_native/test_platform_sync_rx`:
  9/9 cases, 152/152 assertions pass.
- Builds: `cd ros_ws/src/jugglebot/Teensy_code_platform && pio run -e <env>` for teensy40, teensy40_imu2k,
  teensy40_imudiag, teensy40_imudiag_mask, teensy40_imudiag_const, teensy40_imudiag_const_400k and
  teensy40_imudiag_const_100k: all SUCCESS, no new warnings. `pio run -e teensy40 -v` resolves FlexCAN_T4 to the
  can-bridge copy. can-bridge regression build `cd ros_ws/src/jugglebot/Teensy_code_canbridge && pio run -e teensy41`: SUCCESS.
- Firmware scoped pytest (`python -m pytest -q tests/firmware/test_native_firmware.py
  tests/firmware/test_platform_imu_native.py tests/firmware/test_platform_imu_protocol.py
  tests/firmware/test_platform_sketch_source.py`): 54 passed in 1.86 s.
- Scoped pytest over the readers of the changed paths (`flock /tmp/jugglebot-tests.lock python -m pytest -q tests/firmware/
  tests/ros/test_launch_nodes.py tests/ros/test_teensy_bridge_node_bridge_diag.py
  tests/ros/test_teensy_bridge_node_coldstart.py tests/ros/test_teensy_bridge_node_relay.py
  tests/ros/test_teensy_bridge_node_ring_diag.py tests/teensy_link/ -m "not nightly" -n 4`): 3 failed, 1256 passed, 4 skipped
  in 752.83 s. The 3 failures were `tests/firmware/test_platform_fw_version_xref.py` against the old pin, cleared by the
  `PLATFORM_FW_VERSION_EXPECTED` edit.
- Jetson side after that edit (`flock /tmp/jugglebot-tests.lock python -m pytest -q tests/firmware/ tests/teensy_link/
  tests/ros/test_teensy_bridge_node_platform_imu.py tests/ros/test_teensy_bridge_node_coldstart.py
  tests/ros/test_teensy_bridge_node_relay.py -m "not nightly" -n 4`): 1216 passed, 4 skipped; plus 11 ros, gui, imu and
  docs test files: 465 passed.
- Docs and map (`python -m pytest -q tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_docs_links.py tests/sim/test_plans_index.py tests/sim/test_mapkit*.py tests/sim/test_map_items.py tests/sim/test_glossary_avoid_words.py`, 2026-11-23): 340 passed in 30.29 s; `python -m mapkit check` clean.
- Clock checker (2026-11-23, `python -I tools/probes/imu_clock_check.py --capture <stem>` on the seven captures): the four
  short-uptime captures PASS (lag1 -2.2 / -2.8 / -3.4 / -4.2 ms for AEDF / AEE0 / AEE1 / 4AFA), the three 2026-11-22
  captures FAIL (-48.8 / -43.0 / -40.0 ms for 572E / 5730 / 5731); all 14 lags (lag1 and lag2 of the seven) are within
  0.1 ms of the earlier analysis. Its pure pieces (`python -m pytest tests/sim/test_imu_clock_check.py -q`, 2026-11-23):
  4 passed in 0.82 s.
- Default gate (2026-11-23, `flock /tmp/jugglebot-tests.lock ./run_tests.sh` in the scratch worktree): **PASS**, parallel
  11026 passed, 63 skipped, 1 xfailed in 758.50 s; serial 7 passed in 29.32 s. The four `test_bb_fw_update_xref.py` tests
  skip in a worktree (no sibling BallButler checkout).
- Flash (2026-11-23 18:37 local, `pio run -e teensy40 -t upload` from the scratch worktree at this entry's first commit,
  launch down): image 94208 bytes, crc32 0xA5DD8DC4, 0 rewinds, receipt "Platform Teensy FW version: 14 -> 15".
- Round-trip probe on FW 15 just flashed (2026-11-23, `python tools/probes/platform_rtt_probe.py --runs 3 --n 100`,
  launch down, idle bus): Platform-answered round trips 2.1 to 13.1 ms, medians 6.95 / 7.72 / 8.17 ms against the
  bridge-only 1.27 / 1.33 / 1.31 ms; a second pass read floor excess 1.52 ms, median excess 5.48 ms, exit 0. FW 14 read
  medians of 44.87 / 46.01 / 45.57 ms with a floor of 36.6 to 38.2 ms. The healthy spread is the reply waiting 0 to 10 ms
  for the bridge's relay tick, so the probe's verdict was changed from the median excess (which read 6.39 ms and failed
  the first 5 ms limit on a healthy board) to the floor excess (fastest minus fastest, limit 5 ms). This run does not
  yet show that the leak is gone: a FW 14 board would also read near zero minutes after boot. The discriminating check
  is the same probe after Armed time, with the capture header's `rx_ring_leak_max` at 0.
- Robot Sitting step on BUG-132: **pending** (capture header `fw_version` 15, `clock_suspect` false,
  `rx_ring_leak_max` 0, `imu_clock_check.py` PASS).

## Open

- Items filed from this study: BUG-133 (Ball Butler firmware has the same library and dispatch-time stamp), BUG-136
  (Catching cone, same, leak expected almost never), BUG-134 (reference stamps run late), BUG-135 (receive queue overflow
  during the inclinometer re-probe stall).
- Not tested: whether the leak grows with Armed time (the 2026-11-22 board had hours of it, the other days almost none);
  the Platform's uptime (readable only through the local clock and an assumed wrap count); the Wave camera adapter
  (ESP32, syncs to 0x7DD, not audited); what the bridge broadcasts before its own Jetson sync (a forward jump is taken on
  that beacon and a backward one after 1.0 s, but this was read from code only); the inference that Wire (I2C) is polled
  on Teensy 4 and so masking IRQ_CAN1 cannot stall it (from the core library, not re-read).
- The roughly 0.11 to 0.13 ms beacon wire-time bias is not compensated (the bridge stamps before TX).
- A live view of the leak counters is a proposal only: IMU_STATUS has no spare byte, so extending the Platform traffic
  report to 8 bytes (and the bridge relay whitelist) would be another unit's change.
