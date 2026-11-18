---
title: "Platform IMU P2a: FW 11 robustness, tilt served from loop() with an explicit invalid reply, bounded inclinometer probe, multi-frame CAN dispatch (IDEA-20)"
type: feature
date: 2026-11-19
status: in-progress
phase: "platform-imu P2"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino
  - ros_ws/src/jugglebot/Teensy_code_platform/loop_service.h
  - config/protocol_config.yaml
  - config/generate_config.py
  - teensy_link/rpc_args.py
  - tests/firmware/native/test_platform_loop_service.cpp
  - tests/firmware/native/build.py
  - tests/firmware/test_native_firmware.py
  - tests/ros/test_teensy_bridge_node_conduit.py
---

# Platform IMU P2a: FW 11 robustness ahead of the IMU wiring

**What.** Platform Teensy FW 10 to 11, the robustness half of P2 in `plans/active/platform-imu.md` (IMU wiring is P2b):

1. The 0x7DE tilt request no longer runs the SCL3300 inside the CAN callback. `canSniff` sets a flag; `loop()` serves it
   through `LoopService::Tilt::Service` (new pure header `loop_service.h`): the first read in the same pass, at most one
   sensor operation per pass, up to 3 reads 10 ms apart.
2. A tilt the board cannot read gets an explicit invalid reply (owner, 2026-11-18): same id, dlc 8, bytes 0-3 =
   `0x7FC00000`, bytes 4-7 = `0x7FC00000 | reason` (NOT_READY 1, READ_FAILED 2), both quiet float32 NaNs
   (`protocol_config.yaml` `platform_tilt`, generated). FW 10 sent (pi, pi).
3. `setup()` no longer waits for ever for the SCL3300: at most 3 `begin()` calls (about 310 ms); absent, the board runs
   with tilt NOT_READY and `loop()` re-probes at 1 s, doubling to a 60 s ceiling, never while a firmware update session
   is open, and a tilt request brings the next probe forward to a 1 s start-to-start floor.
4. `loop()` dispatches up to 32 queued CAN frames per pass (`events()` until its returned RX count is zero).

**Why.** Firmware over CAN is this board's only way in (dead USB), and it only runs while `loop()` keeps calling
`can1.events()`. FW 10 had a forever-wait in `setup()` and a >= 600 ms stall inside the callback on a failing sensor.

## Decisions

- **Invalid-reply encoding.** Today's decoder (`teensy_bridge_node.py` `relay_read_tilt` :7671-7686, `_svc_get_platform_tilt`
  :9697-9737) waits for dlc 8 on 0x7DE, unpacks `<ff` and accepts only `|tilt| <= JB_OP_MAX_VALID_TILT_RAD` (0.785). NaN
  fails that comparison, so a FW 10 host retries and returns NaN, as it did for (pi, pi); keeping dlc 8 makes it an
  answer, not a 0.5 s timeout. A different dlc would have been ignored by the `(can_id, dlc)` correlator and read as
  silence. Pinned by `test_get_platform_tilt_rejects_the_fw11_invalid_reply`.
- **The probe is rationed, not split.** The SCL3300 library's `begin()` is one ~103 ms call (1 + 1 + 100 ms busy waits
  plus 1 ms in `endTransmission()`, `SCL3300.cpp:48-65,364`; `reset()` is `begin()`), and its word transfer is
  private, so it cannot be made non-blocking without re-implementing the driver. It runs only from `loop()`, never in a
  pass that read, never during an update session, at most once per second.
- **Retries re-read, the re-initialisation is a separate pass.** FW 10 re-initialised between reads (inside the
  callback). FW 11 reads 3 times, replies READ_FAILED, and re-initialises on the next pass, so a glitch is usually cleared
  before the host's own retry (it makes 3 attempts).

## P3 must

Decode 0x7DE dlc 8 as raw `<II` first: if word 0 == `PLATFORM_TILT_INVALID_NAN_BITS` and (word 1 & ~0xFF) == it, the
reply is invalid with reason `word1 & 0xFF` (1 NOT_READY, 2 READ_FAILED). Log it as such (not "out of range"), keep the
NaN result to the orchestrator, and on NOT_READY retry no sooner than about 1 s (the probe floor) rather than three
times back to back. Treat (pi, pi) as the FW <= 10 failure reply.

## Control and timing walk-through

Costs: an ordinary callback 3-5 us; SCL3300 read (`available()`) about 1.3 ms; `begin()` about 103 ms; FwUpdate sector
flush 45 ms typical, 400 ms worst (once per 819 DATA frames); VERIFY CRC about 20 ms; bus 5,500-6,800 frames/s, so the
256-frame RX queue covers 38-46 ms.

**FW 10 pass:** `events()` (one frame and its callback) -> `reportStatus()` -> `FwUpdate::tick()` -> `updateHandFan()`.
**FW 11 pass:** up to 32 `events()` calls -> tilt service (at most one read or one probe) -> the same three.

| Case | FW 10 worst gap between `tick()` calls / `events()` drains | FW 11 |
| --- | --- | --- |
| (a) healthy | 1.3 ms on a tilt pass, else µs | the same: dispatch <= 32 x 5 us = 0.16 ms, plus the read on a request pass |
| (b) SCL3300 absent | `setup()` never returns; `loop()` never runs (callbacks run in the ISR) | `setup()` <= 310 ms, then µs passes, plus one 103 ms probe pass at 1, 3, 7, 15, 31, 63 s, then every 60 s |
| (c) failing mid-run | >= 613 ms in the callback per request (3 x (1.3 + 103 + 100)), reply after the Jetson's 0.5 s timeout | reads at +0/+10/+20 ms, 1.3 ms each; READ_FAILED at about 21 ms; then one 103 ms probe pass; afterwards as (b) |
| (d) firmware update | 400 ms (a flush inside the callback) | 400 ms, unchanged: the flush is still inside `events()`; probes are suppressed while the session is open; no new code runs inside a callback |
| (e) Hand fan command | µs, in the callback | unchanged; the fan's 5 s no-reading timeout is far above any pass |

**(d) in detail.** BEGIN opens the session, so the tilt service stops probing. A tilt request during the transfer still
gets a 1.3 ms SPI read in `loop()` (no interrupts masked). Flash erase and writes run inside `FwUpdate::handleCommand`,
inside `events()`; the dispatch loop only calls `events()` again afterwards. COMMIT's `delay(50)` and copy never return,
so `loop()` never runs again. No interrupt source was added: the COMMIT inter-call windows see the same CAN, SysTick and
USB ISRs as FW 10.

**Tilt reply latency** against the Jetson's `_RELAY_READ_TIMEOUT_S = 0.5` (`teensy_bridge_node.py:7543`), from the
request entering the board's RX queue: healthy about 3 ms (up to 8 passes to drain a full queue plus the 1.3 ms read);
sensor down, about 103 ms plus the reply if a probe is running; failing, about 21 ms plus a probe already running, about
125 ms. The worst is a request queued behind a 400 ms worst-case flush during an update (probes cannot add to it), about
425 ms, the same exposure FW 10 had. The bridge adds up to 10 ms (8-slot ring, 4 frames per 10 ms tick) plus UDP.

**Dispatch bound (32).** Arrivals are 6.8 frames/ms at most; a 1.3 ms read lets about 9 in, so one pass drains more than
a read-bearing pass adds, and a full queue empties in 8 passes. `events()` is never re-entered (the callbacks do not
call it). FlexCAN's duplicate-TX path (`FlexCAN_T4.tpp:1086-1093`) runs only when all 8 TX mailboxes are busy; this
board's TX bursts follow the received requests, not the pass count, and single-frame passes were already µs long, so
the exposure is unchanged. Until the first `events()` callbacks run in the CAN ISR (`isEventsUsed`, `.tpp:1075,1156`);
the only thing moved out of the callback is a `volatile bool` store, so no SPI runs in the ISR while `setup()`'s
`begin()` uses the bus, which FW 10 allowed.

**What could make the board unreachable now that could not before.** Nothing new can hang: the dispatch loop is bounded
by a counter, the tilt service has no loops, and `begin()`'s busy waits are `millis()`-based (SysTick, priority 32,
runs in loop context). No pointers, no new ISR, no new heap. Residual risks: (1) with the SCL3300 absent or broken,
each probe is a 103 ms CAN blackout that drops about 60 ms of traffic (about 400 frames), so a host request in that
window (STATE_READ, fan command, an update's BEGIN) is lost and the host times out and retries; at the 60 s ceiling that
is about 0.1 % of requests, about 1 % in the first 31 s after boot. (2) FW 11 is first run on the robot (no bench, owner
decision); a fault in the new code early in `loop()` would crash-loop every 8 s, shorter than an update, which is the
generic risk of every flash and is why the new code is small and tested natively. (3) Power loss during COMMIT, as before.

## Verification

- Native (2026-11-19, `temp/firmware_native/test_platform_loop_service` built by `tests/firmware/native/build.py`):
  13/13 cases, 104,411 assertions pass; also compiles clean with `-Wall -Wextra`.
- Readers of the changed files (grep: `rpc_args.PLATFORM_FW_VERSION_EXPECTED`, `Teensy_code_platform`,
  `protocol_config.yaml`/`generate_config`; 2026-11-19): `pytest tests/firmware tests/teensy_link/test_rpc_args.py
  tests/ros/test_teensy_bridge_node_coldstart.py tests/ros/test_teensy_bridge_node_relay.py
  tests/ros/test_teensy_bridge_node_conduit.py tests/ros/test_gui_console.py tests/ros/test_teensy_bridge_node_bridge_diag.py
  tests/ros/test_ball_possession.py tests/ros/test_gui_geometry.py tests/motion/test_endpoint_id_contract.py
  tests/sim/test_night_shift_gate.py -q -p no:cacheprovider`: **930 passed, 4 skipped in 214.35 s**.
- Logbook and link tests plus all of `tests/teensy_link` (2026-11-19, `pytest tests/sim/test_logbook_front_matter.py
  tests/sim/test_logbook_index.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_docs_links.py tests/teensy_link -q
  -p no:cacheprovider`): **567 passed in 26.15 s**.
- Build (2026-11-19, `pio run -e teensy40` in a scratch copy of the sketch): SUCCESS, no warnings from the sketch.
  Against the same command on `imu-p1` (FW 10): `.text.itcm` 56,416 -> 56,672 B (2 ITCM banks, about 8.8 KB
  headroom), `.data` 6,848 -> 7,872, `.bss` 16,608 -> 16,640; the only new data symbols are `tiltIo` (8 B),
  `tiltService` (20 B) and `tiltRequested` (1 B), so the `.data` growth is section alignment.
- Not run: `./run_tests.sh --full` (the P2 phase gate, after P2b and the phase audit). Not flashed.
