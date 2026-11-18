---
title: "Platform IMU P2: FW 11 robustness (tilt served from loop() with an explicit invalid reply, bounded inclinometer probe, multi-frame CAN dispatch) and the IMU capture, status and upload responder wired into loop() (IDEA-20)"
type: feature
date: 2026-11-19
status: in-progress
phase: "platform-imu P2"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino
  - ros_ws/src/jugglebot/Teensy_code_platform/loop_service.h
  - ros_ws/src/jugglebot/Teensy_code_platform/imu_service.h
  - ros_ws/src/jugglebot/Teensy_code_platform/icm42688.h
  - ros_ws/src/jugglebot/Teensy_code_platform/icm42688.cpp
  - tests/firmware/native/test_platform_imu_service.cpp
  - plans/active/platform-imu.md
  - config/protocol_config.yaml
  - config/generate_config.py
  - teensy_link/rpc_args.py
  - tests/firmware/native/test_platform_loop_service.cpp
  - tests/firmware/native/build.py
  - tests/firmware/test_native_firmware.py
  - tests/ros/test_teensy_bridge_node_conduit.py
---

# Platform IMU P2: FW 11 robustness (P2a), then the IMU wired into loop() (P2b)

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

### P2b: the IMU in the loop

**What P2b adds.** `imu_service.h` (pure, native-tested) wires P1's `Engine`, ring and driver into `loop()`. The ring
(32000 x 12 B) and the 400-slot anchor table are static `DMAMEM` arrays; `Wire` (pins 18/19) is configured in `setup()`
at 1 MHz with no device traffic; the 0x6E3 callback only copies the frame into a 7-deep pending slot; the 0x7DD handler
feeds a sync-quality accumulator (integer only). Kill switch `PLATFORM_IMU_ENABLED` (default 1; `0` compiles every IMU
path out, ITCM identical to P2a).

**The pass.** `events()` x <= 32 -> tilt service -> `reportStatus()` -> `FwUpdate::tick()` -> **`imuService.pass()`**
-> `updateHandFan()`. `pass()` does, in order: (0) FwUpdate gate: session open -> FW_HALTED (I2C stops, capture
discarded); session closed while FW_HALTED -> PROBING, first probe 1 s later; (1) pending commands (state only, no I2C);
(2) `Engine::tick`; (3) AT MOST ONE step: one 8192 B CRC slice, or one I2C transaction (an init step, a FIFO_COUNT read,
one burst of <= 8 packets, a flush, a WHO_AM_I), or nothing; (4) frames: replies, then the 1 Hz STATUS, then, only if
step (3) did nothing, one IMU_DATA frame. A failed transaction is never retried in the same pass.

**Wire bounds, from the core** (`framework-arduinoteensy` 1.160.0, `libraries/Wire/WireIMXRT.cpp`): `wait_idle()` spins
until the bus is free or 16 ms (`elapsedMillis > 16`, so up to ~17 ms), then `force_clock()` (10 us + 9 SCL pulses of
10 us) and gives up (:75-91, :232-262); each transfer loop ends on NACK, arbitration loss, the pin-low timeout PLTF
(`CLOCK_STRETCH_TIMEOUT 15000` us, :480-512) or `timeout > 50` ms (:139, :196). There is no `setTimeout` for the master
path (`Stream::setTimeout` only bounds `readBytes`), so these bounds cannot be tightened without patching the core; the
design keeps their duty small instead. `yield()` (`cores/teensy4/yield.cpp`) runs only `serialEvent`, HardwareSerial
events and `EventResponder::runFromYield`; this sketch defines none of them and FlexCAN_T4 registers no yield hook, so a
Wire wait never re-enters `can1.events()` or `canSniff`. One Wire call therefore blocks for:

| Fault | One transaction | Why |
| --- | --- | --- |
| IMU absent | ~0.1 ms | address NACK (`MSR_NDF`) |
| SDA stuck low | ~17 ms | the stuck line reads as a START with no STOP, so BBF stays set; `wait_idle` times out, `force_clock` fails |
| SCL stuck low | ~15-17 ms typical, <= ~68 ms | PLTF after 15 ms; the bound is `wait_idle` (17) + the 50 ms transfer timeout |
| Misbehaving chip (stretching) | <= ~68 ms per Wire call; ~120 ms theoretical for a register read whose write half succeeds slowly | two Wire calls per register read |

**Back-off.** Any failure (a probe that ends in an error, an error while RUNNING, a failed RESUME check) adds one to a
streak; the next probe waits 1, 2, 4, 8, 16, 32, then 60 s (`backoff_ms`), counted from the failure; 10 s of clean
RUNNING resets the streak. RUNNING to BACKOFF stays the Engine's 1 s minimum (spec); the extra wait is spent in PROBING
with no I2C. The first probe is 2 s after boot (`BOOT_SETTLE_MS`: CAN only). A probe of an absent or stuck bus is two
failed address probes (0x68, 0x69) in two passes. Duty after the first two minutes: two transactions per 60 s, i.e.
0.0003 % absent, 0.06 % with SDA stuck (2 x 17 ms), <= 0.23 % at the 68 ms bound. The test drives 10 minutes of an
absent chip: <= 34 transactions in all, <= 12 in the last 5 minutes, never two in one pass.

**Drain cadence.** A FIFO_COUNT read every 4 ms (`DRAIN_PERIOD_US`), then one pass per burst of <= 8 packets (at
1 MHz 128 B + framing is about 1.2-1.3 ms; at the 400 kHz fallback about 3 ms). At 1 kHz a drain is normally 4 packets
and one burst, so the chip FIFO (128 packets = 128 ms) holds at most a few ms of data. A pass that also served a tilt
read is 1.3 + 1.3 ms. Overflow (FIFO_FULL or a count >= 127) or a bad packet: the FIFO is flushed and the next anchor is
DISCONTINUITY. Before any discontinuity (and before a FREEZE) the samples pushed since the last anchor get a
**closing anchor** with the stamp of the drain they came from, so the pre-gap segment ends exactly at an ordinary anchor
and the DISCONTINUITY anchor closes the first post-gap drain: the Jetson's segment split is exact, not "somewhere since
the last 100 ms anchor". Timestamps: `micros64()` and `micros64() + wall_offset_us` taken together immediately before the
FIFO_COUNT read. `wall_offset_us` is written only by `handleSyncFrame`, which after `loop()`'s first `events()` runs only
inside `events()` in loop context, as `stamp()` does, so the 64-bit read cannot tear.

**CRC slice.** 8192 B = 1365 words per pass; per word a ring index (one modulo), 6 bytes through the nibble table (12
lookups). Estimate 80-120 cycles a word at 600 MHz, about 0.2-0.3 ms a slice, plus about 256 D-cache line fills from
RAM2 (tens of us). A full capture (64,960 payload words, 389,760 B) is about 48 slices: the FREEZE reply follows about
48 passes. To be timed at the Sitting.

**TX rule.** `ImuTx::place` masks the CAN IRQ, counts TX_INACTIVE mailboxes after the RX FIFO area (the formula of
FlexCAN_T4's private `mailboxOffset()`: MB 8-15 here), reads `getTXQueueCount()`, and calls `write()` only if
`tx_may_place()`: SW queue empty and more than 2 mailboxes free. `write()` then always lands the frame (it scans for the
first TX_INACTIVE mailbox, `FlexCAN_T4.tpp:1024-1050`), so an IMU frame never enters the queued path whose `events()`
writes one frame into every free mailbox (`.tpp:1086-1093`), and the IMU always leaves 2 mailboxes for the update, tilt,
state and fan replies. A frame that cannot be placed waits (replies before status before data); `write() != 1` is
counted in `imuTxQueuedAnyway` (unreachable by construction).

**FwUpdate.** BEGIN is handled inside `events()`; `pass()` runs after `FwUpdate::tick()` in the same `loop()` pass and
checks `FwUpdate::sessionOpen()` before anything else, so no I2C runs after a BEGIN. A failed VERIFY keeps the session
open (still halted); a flash error, a BEGIN refused as TOO_BIG or the 60 s idle timeout close it, and only then does
`pass()` return the IMU to PROBING (spec: "FW_HALTED to PROBING if that update is abandoned"). COMMIT replies, waits 50 ms
and copies inside `events()` and never returns, so `loop()` and with it every IMU path never runs again before the
reset. No interrupt source was added (no IntervalTimer, no INT pin, no Wire in an ISR): the COMMIT windows see the same
CAN, SysTick and USB ISRs as FW 10. STATUS goes on at 1 Hz while FW_HALTED.

**Worst-case interval between consecutive `FwUpdate::tick()` calls and CAN drains** (the RX queue holds 256 frames,
38-46 ms of traffic):

| Case | Worst extra time the IMU adds to one pass | Can the board become unreachable? |
| --- | --- | --- |
| (a) IMU healthy, capturing | one burst, ~1.3 ms (2.6 ms with a tilt read in the same pass) | No: 2.6 ms is 18 frames, the dispatch drains 32 a pass |
| (b) IMU absent | ~0.1 ms on 2 passes per probe; probes at 2, 3, 5, 9, 17, 33, 65, 125 s, then every 60 s | No |
| (c) SDA stuck low | ~17 ms on 2 passes per probe, same schedule | No: 17 ms is inside the RX queue's 38 ms, nothing is dropped |
| (d) SCL stuck low | ~15-17 ms typical, <= 68 ms bound, 2 passes per probe | No: at the bound one pass can overrun the RX queue (about 30 ms of frames lost, at most twice a minute after back-off); a lost request is retried by its host |
| (e) unplugged mid-capture | the failing transaction: ~0.1 ms on a clean unplug, (c)/(d) if a line is left stuck; then BACKOFF and (b)-(d) | No; recovery appends behind a DISCONTINUITY anchor (tested) |
| (f) upload at the 625 us cap | one IMU_DATA frame, a few us; no I2C while FROZEN/UPLOADING | No: <= 1600 frames/s, 2 mailboxes always left free |
| (g) freeze CRC in progress | one slice, ~0.3 ms estimated (bound < 1 ms) for ~48 passes | No |
| (h) firmware update BEGIN..COMMIT | none: FW_HALTED, no I2C, a 1 Hz STATUS frame | No: the update path is as in P2a |
| (i) inclinometer absent and re-probing, IMU capturing | the P2a 103 ms probe pass plus <= 1.3 ms; the FIFO holds ~110 packets, under the 127 overflow mark, so the drain catches up in ~14 passes without a discontinuity; a pass longer than ~127 ms would overflow and be marked DISCONTINUITY (tested at 140 ms), never suppressed | Not by the IMU; the 103 ms blackout is P2a's residual (1) |

**Residual risks (P2b).** (1) A stuck-SCL fault can cost one <= 68 ms blackout per probe, at most twice a minute after
the back-off; a request landing then is lost and retried by its host, including an update's BEGIN. (2) First run on the
robot with no bench rehearsal (owner): host tests, two `pio` builds and this reasoning are the only evidence; a fault in
IMU code early in `loop()` would crash-loop every 8 s. The 2 s CAN-only window after boot is not a safe mode: a
CrashReport-based "boot with the IMU DISABLED after a crash" would be, but the spec boots into PROBING, so it is an owner
decision (below). (3) 1 MHz on the real wiring is unproven; repeated errors show as BACKOFF cycling and `i2c_errors` in
STATUS; the fallback is 400 kHz by a reflash. (4) The CAN IRQ is masked for the few microseconds of each IMU `place()`.
(5) `boot_id` mixes the cycle counter with the timing of CAN frames received during `setup()`; on a silent bus it may
repeat across boots (no hardware RNG call, because the TRNG library busy-waits unbounded). (6) The `GYRO_ACCEL_CONFIG0`
UI filter is still the unverified bench value (P1 TODO). (7) The ITCM grew into a third 32 KB bank.

**Decisions taken in P2b, within the spec.** Probe failures stay in PROBING with the paced re-probe (the spec defines
BACKOFF only from RUNNING); header `temp_*_raw` are the FIFO temperature byte x 64 (TEMP_DATA counts, since 132.48/2.07 =
64, at 8-bit resolution, with no extra register read); the closing anchor above; the TX reserve of 2 mailboxes is
stricter than the spec's "a free mailbox"; up to 7 pending commands, the 8th dropped without a reply (the RPC retry
covers it).

**P3 must (from P2b).** Use the closing anchor: in each segment split, samples with `k < closing.index` belong to the old
segment and those in `[closing.index, disc.index)` to the new one. Expect a final anchor at the FREEZE. Expect replies to
wait behind a busy bus (the TX reserve) and STATUS during FW_HALTED. A board with no IMU reports PROBING, SENSOR_OK clear
and a climbing `i2c_errors`: not an error to alarm on. The FREEZE reply comes about 48 passes after the command.

**Sizes** (2026-11-19, `pio run -e teensy40` in scratch copies of the sketch, `arm-none-eabi-size -A` and `nm`):

| | P2a | P2b | P2b with `-DPLATFORM_IMU_ENABLED=0` |
| --- | --- | --- | --- |
| `.text.itcm` | 56,672 B (2 banks) | 67,504 B (**3 banks**, 30,800 B headroom) | 56,672 B (2 banks) |
| `.data` / `.bss` | 7,872 / 16,640 | 8,896 / 16,672 | 7,872 / 16,640 |
| `.bss.dma` (RAM2) | 12,416 | 406,016 | 12,416 |
| flash image (`_flashimagelen`) | - | 88,064 B | 74,752 B |

The third ITCM bank comes out of DTCM: RAM1 data+stack goes from 448 KB to 416 KB, of which `.data` + `.bss` use 25.6 KB
(about 390 KB left). RAM2: `imuAnchors` at 0x20200000 (9,600 B), `imuRingBuf` at 0x20202580 to 0x20260180 (384,000 B),
`_heap_start` 0x20263200, so 118,272 B of heap room under 0x20280000 and the CrashReport area at 0x2027FF80 untouched (the
sketch allocates no heap). Upload path: the staging base is the first 4 KB boundary above the running image, so a P2b
image staged from a FW 10 or P2a board has about 1.9 MB of room (`EEPROM_RESERVE` 0x601F0000) for 88 KB; neither the
receiver (`len > stage_cap`) nor `tools/teensy_link_bridge.py` (`_prepare_image`: base address and identity checks only)
has another size limit, and the commit copy's overlap invariant holds for an image of any length. The hex is 247,787 B.

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
- P2b native (2026-11-19, `g++ -std=c++17 -O1 -Wall -Wextra` of `tests/firmware/native/test_platform_imu_service.cpp`,
  then the binary): **11/11 cases, 1,228,799 assertions pass**, no warnings. Covers the IMU absent for 10 minutes, a
  healthy capture, a 103 ms and a 140 ms slow pass, unplug and re-plug, a bad packet, FwUpdate BEGIN in all 7 engine
  states, the full FREEZE -> header -> anchors -> samples -> both CRCs -> RESUME cycle, READ pacing with 0, 2 and 3 free
  mailboxes and a non-empty SW queue, the STATUS cadence, and the command slots; every pass is checked for <= 1 I2C
  transaction.
- P2b readers (grep of `tests/` and `tools/` for `Teensy_code_platform`, `icm42688`, `imu_service`; 2026-11-19, all code
  edits done before the run): `flock /tmp/jugglebot-tests.lock pytest tests/firmware
  tests/ros/test_teensy_bridge_node_coldstart.py tests/ros/test_teensy_bridge_node_relay.py
  tests/sim/test_night_shift_gate.py -q -p no:cacheprovider`: **537 passed, 4 skipped in 458.32 s** (includes the native
  build of every binary, `test_platform_imu_service` among them).
- P2b docs (2026-11-19, `pytest tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py
  tests/sim/test_no_legacy_item_ids.py tests/sim/test_docs_links.py tests/sim/test_plans_index.py
  tests/sim/test_mapkit*.py -q -p no:cacheprovider`): **324 passed, 1 failed in 26.95 s**; the failure is
  `test_the_tree_holds_no_old_item_id` on `logbook/2026-11-18-stop-warn-handless-hand-lapse.md`, a file this branch does
  not touch (it arrived on main with BUG-119), so it fails on main too.
- P2b builds (2026-11-19, `pio run -e teensy40` in scratch copies, and again with
  `PLATFORMIO_BUILD_FLAGS=-DPLATFORM_IMU_ENABLED=0`): both SUCCESS, no warnings from the sketch; sizes in the table above.
- Not run: `./run_tests.sh --full` (the P2 phase gate, after the phase audit). Not flashed.
