---
title: Platform IMU — an ICM-42688-P on the Platform Teensy, buffered capture over CAN for offline input-shaping work
created: 2026-11-18
status: active
owner: Harrison
last_updated: 2026-11-20
pathway: two-ball-oval
scope: >-
  Put an ICM-42688-P on the Platform's 2020 extrusion (the one carrying the Hand's rail), read it over I2C from the
  Platform Teensy at 1 kHz into a 32 s RAM ring, and have the Jetson freeze, upload and store the ring at the end
  of every Attempt, so the ring mode (about 4.5 to 5.4 Hz, carry-ring-zvd) can be tuned offline from real
  acceleration instead of twice-differentiated Mocap. No online use and no live stream. Closes when a Level plus
  X/Y Tilt mount calibration is fitted and a real Attempt's capture is on disk, complete, CRC-checked and handed to
  the carry-ring-zvd analysis.
steps:
  - {id: P1, title: Off-board foundations, kind: phase, status: in_progress, effort: m, hardware: none}
  - {id: P2, title: Platform Teensy firmware FW 11, kind: phase, status: in_progress, effort: l, hardware: none}
  - {id: P3, title: can-bridge firmware and the Jetson side, kind: phase, status: not_started, effort: l, hardware: none}
  - {id: P4, title: "Sitting 1, flash and bench the capture path", kind: rung, status: complete, effort: m, hardware: robot}
  - {id: P5, title: "Sitting 2, mount calibration and first Attempt captures", kind: rung, status: in_progress, effort: m, hardware: robot}
related_logbook:
  - 2026-11-18-imu-bench-proof-of-concept.md
  - 2026-11-18-platform-imu-p1-foundations.md
related_code:
  - tools/probes/imu_bench/firmware/src/icm42688.cpp
  - ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - config/protocol_config.yaml
  - ros_ws/docs/platform_imu_protocol.md
  - ros_ws/src/jugglebot/Teensy_code_platform/imu_capture.h
  - ros_ws/src/jugglebot/Teensy_code_platform/imu_proto.h
  - ros_ws/src/jugglebot/Teensy_code_platform/icm42688.cpp
---

# Platform IMU

Map Item: IDEA-20 (`map/idea-platform-accelerometer.md`), under the two-ball-oval Pathway because its one customer is
the ring-mode tuning of `plans/active/carry-ring-zvd.md` (FEAT-32), which sits on that Pathway. Decisions below were
agreed with the owner on 2026-11-18 in the design round. Facts cite the three surveys of that round: [fw] Platform
Teensy firmware, [can] CAN path and bus load, [motion] motion envelope. A line marked **open** is not decided.

## 1. Purpose

The ring mode is a Platform vibration the planner cannot see: its frequency is 5.4 Hz from encoders and 4.4 to 4.7 Hz
from Mocap (the discrepancy is itself open, section 6). Acceleration from Mocap is a second derivative of a
roughly 300 Hz stream with tens of milliseconds of clock-bound latency, so the high-frequency content that input
shaping needs is the part it loses [motion]. An accelerometer on the Platform gives that signal directly. The
data is used offline only.

## 2. Measured numbers that sized the design

| Number | Value | Source |
| --- | --- | --- |
| Jugglebot-bus load, idle | about 25 % | [can] `/profile` frame counts, 11-13 and 11-18 bags |
| Jugglebot-bus load, in an Attempt | worst 64.2 % (5780 fps at 111 bit/frame) | [can] same bags; 75 % at 130-bit worst-case stuffing |
| Added load per frame/s in an Attempt | 0.0111 % to 0.013 % | [can] derived from the above |
| Free RAM2 (DMAMEM) on the Platform Teensy | about 499 KB of 512 KB | [fw] `imxrt1062.ld` and `.bss.dma` 12.4 KB |
| Peak specific force of the Platform origin | 2.1 to 2.6 g | [motion] bags filtered at 8 to 10 Hz |
| Peak angular rate | 73 dps (27 to 38 dps in other bags) | [motion] 11-05 bag |
| Planner tilt-rate cap | 172 dps (3.0 rad/s) | [motion] `cup_realize.py:115` |
| Ring-mode acceleration amplitude | 0.19 to 0.41 g estimated, at about 4.9 Hz | [motion] `platform_mode_fit.py`, estimate |
| Bridge uplink for Platform replies | 8-slot drop-newest ring, 4 frames per 10 ms tick, at most 400 fps | [can] `can_buses.cpp:536-570`, `telemetry.cpp` |
| FwUpdate download throughput (Jetson to Platform) | about 800 to 1000 fps flown | [can] logbook 2026-09-28 |
| Priority of an upload frame | Every ODrive and setpoint frame has a lower id (higher priority) than 0x6E6, so an upload frame delays a control frame by at most one frame time (about 0.13 ms) | [can] id map |
| Between-Attempts bus load | The bridge emits setpoints from a 500 Hz timer interrupt to every Armed axis on every tick, HOLD included (`leg_interp.cpp`), also feeding the ODrive watchdog, so the load is about the Attempt's: about 59 % Hand Idle, 64 % Hand Closed-loop (2280 versus 5277 frames/s) | [can] `/profile`, 11-13 and 11-18 bags |
| Upload at 1250 frames/s | adds about 14 % (111-bit frames). The risk is bridge loop time (47 ms worst emit gap; Leg scheduler grace 30 ms), not arbitration | [can] derived; the dress-rehearsal residue (`plans/archived/skill-stack-dress-rehearsals-and-residue.md` section 2) |
| Bench IMU result | 0 FIFO overflows and 0 I2C errors at 1 and 2 kHz; 999.97 Hz | IDEA-20 bench proof of concept |

The ranges (accel +/-16 g, gyro +/-1000 dps) are wider than the table needs, by the owner's request. The gyro step
is 0.031 dps, under the 0.05 to 0.07 dps rms bench noise, so the wide range costs nothing measurable.

## 3. Decisions and the failure each one prevents

| Decision | Failure it prevents |
| --- | --- |
| I2C on Platform Teensy pins 18 and 19 (Stemma QT), 1 MHz with 400 kHz fallback. No INT1 or CLKIN wire, no new interrupt source | An interrupt running during the FwUpdate COMMIT copy touches flash-resident code and can brick a board whose only recovery is that update path [fw] |
| 2.2 kOhm external pull-ups on SDA and SCL at the Platform Teensy are required (2026-11-20, logbook 2026-11-20-platform-imu-i2c-error-bursts), in parallel with the existing 3.9 kOhm | Corrupted I2C reads (the chip stops driving SDA mid-read) and 1 s holes in captures: the error rate was about one per 15 to 40 s without them and 0 in 300 s with them |
| ICM-42688-P at ODR 1 kHz, UI filter ODR/2, `GYRO_ACCEL_CONFIG0` 0x00 (decided 2026-11-20, owner: the data is used offline and filtered in software, and it is the code the firmware writes and the capture header records; replaces the earlier ODR/4), 16-bit FIFO packets, driver lifted from `tools/probes/imu_bench/firmware/src/icm42688.*` | A second, untested driver; 20-bit packets that break the 6 x int16 layout |
| No live stream. One 1 Hz status frame during an Attempt (state, temperature, FIFO overflow and I2C error counters, clock-sync quality, capture fill). Live-stream IDs reserved, not used | Adding bus load where an Attempt already runs about 64 %, for data the owner will not use online |
| Capture ring in RAM2 (`DMAMEM` static array, about 384 KB, clear of the top 128 B CrashReport area), recording continuously while the IMU is healthy, 6 x int16 per sample, 32 s at 1 kHz. The Jetson freezes it at Attempt end and on a Fault or Soft E-stop, uploads, then resumes. Attempts over 32 s keep the last 32 s | Missing the capture of the Attempt that went wrong (the freeze on Fault); a ring overrunning into CrashReport |
| Each FIFO drain batch is stamped with `TimeSync::get_wall_time_us()`; sample times are rebuilt on the Jetson from sample count and a fitted chip rate (chip clock good to about 1 %, FIFO tick 32/30 us). Metadata carries the anchors. Clock-sync quality goes in the status frame | Trusting the nominal ODR and drifting about 1 % against the Mocap and encoder logs; clock quality that today only prints to the dead USB console [fw] |
| Upload is host-paced pull, the FwUpdate pattern reversed: the Jetson requests blocks, the Platform sends sequence-numbered 8-byte frames, the Jetson re-requests gaps, CRC-32 over the capture. Platform TX is paced | A babbling Platform loading the loop in flight; the FlexCAN_T4 all-mailboxes-busy defect [fw surprise 3]; a corrupt capture that looks good |
| Uploads run while Armed (owner, 2026-11-18/19), holding the Active pose between Attempts. The can-bridge cannot know what an Attempt is, so the Jetson gates: it uploads only when no Attempt is running. With the GUI toggle "Upload IMU data after each Attempt" ON (default) the Jetson REFUSES the Attempt-starting command and any other motion-starting command while an upload runs; pacifying commands (deactivate, Soft E-stop and the like) always pass and abort the upload (capture flagged incomplete). Toggle OFF: ring never frozen, nothing refused; ON applies from the next Attempt. The GUI shows progress and time remaining. The bridge no longer ABORTs on arming | A bridge-side Armed refusal making the upload impossible in the one state the Platform sits in between Attempts; an Attempt starting with the ring frozen; a stop command waiting behind a transfer |
| Target: the full 32 s window (64000 data frames plus anchors and header) uploads in under 60 s, about 1250 frames/s on the bus (51 s). Only the Attempt's span plus a margin is requested, so a short Attempt uploads proportionally faster. The earlier "a wait of a few minutes is accepted" is superseded | A 66 to 70 s upload at 1000 frames/s holding every Attempt back; fetching 32 s when 8 s were recorded |
| Bus protection: while Armed the bridge enforces a hard minimum inter-frame gap (cap about 1600 frames/s, 625 us) by clamping the READ's gap argument; the operating pace is a Jetson config value passed as that argument, so it is tunable without a reflash | A mistyped config value flooding the bus the Legs and Hand share |
| Uplink: IMU data frames get their own bridge ring and a new ADDITIVE batched UDP message (about 16 CAN data frames per datagram, flushed at the 100 Hz telemetry tick; no PROTOCOL bump, per the additive-message precedent). Interface only in the spec; P3 defines the struct | One UDP datagram per CAN frame at 1250 frames/s taking bridge loop time from the loop that emits the Leg and Hand setpoints |
| New CAN IDs from the free block next to the Platform's (0x6E3 to 0x6EF, 0x6F2 to 0x6FF [fw][can]), defined in `config/protocol_config.yaml` and generated; commands as new typed RpcMethod(s), next free 0x64 | A hand-written ID colliding with an ODrive node range or the cone and Wave block; generated headers drifting from the YAML |
| can-bridge whitelists the new Platform IDs and gives IMU frames their own uplink ring (batched, above). FW bump, no PROTOCOL bump | IMU frames crowding the 8-slot ring that tilt, fan and FwUpdate replies share, so a capture dump starves them (drop-newest) [fw][can] |
| Platform Teensy robustness changes ride along: the tilt request becomes a flag serviced in `loop()` with a bounded failure path (today at least 300 ms stalls inside the CAN callback); the SCL3300 wait-forever in `setup()` is bounded; `loop()` dispatches several CAN frames per pass (today one). FW 10 to 11 | A busy callback blocking CAN reception while the IMU is also being served; a missing inclinometer hanging boot [fw] |
| Tilt request with the inclinometer absent or failing (owner, 2026-11-18): the Platform Teensy answers with an explicit invalid reply, never silence and never a stale value | A Jetson caller waiting out a timeout on a missing sensor, or acting on a tilt that is no longer measured |
| The IMU is strictly optional: a non-blocking probe and init state machine in `loop()` (never `setup()`), bounded Wire timeouts, back-off on errors, IMU work stops at FwUpdate BEGIN, a CAN disable command | An unplugged or misbehaving IMU stopping the board from booting, answering CAN or accepting a firmware update |
| Recovery stance (owner, 2026-11-18): the CAN firmware update is the trusted recovery path and power is reliable. No bench self-update rehearsal, no safe-boot machinery, no opt-in-per-boot, no crash breadcrumbs. P2 added a 10 s boot settle and a BEGIN latch (not a safe mode; the CrashReport safe mode was considered and not built, owner 2026-11-18). Off-board checks only: native tests of the pure logic (the `tests/firmware/native` host g++ and doctest pattern), a `pio` compile, then flash | Building recovery machinery for a risk the owner has judged acceptable; it is recorded so a later Session does not reopen it unasked |
| Jetson: `teensy_bridge_node` gets an IMU handler with its own queue (today it keeps only the latest frame per CAN id); status on a topic added to the rosbag list in `jugglebot_launch.py`; captures written to `temp/imu_capture/<stamp>` with metadata (ODR, ranges, clock anchors, temperature, CRC result, complete flag), following `hand_capture.py`. Automatic freeze and upload at Attempt end and on Fault from the start | Latest-frame-wins dropping capture frames; a status topic that is never bagged; a capture needing a hand to start it |
| Mount calibration, the owner's simpler procedure: fasten the sensor; at Level take gravity for the IMU's Z (two mount angles); Tilt the Platform about its local X, then Y (static holds plus the moves, Mocap recording) and fit the third angle. Gyro bias zeroed from a still window before each Attempt (Jetson side). Factory scale and bias otherwise; no six-face solve, no temperature compensation (temperature is logged). Applied on the Jetson, raw counts on the wire. Lever arm from the owner's bracket CAD, needed to about 5 mm | Re-flashing firmware for a calibration change; a lever-arm error that mixes angular acceleration into translation (0.3 to 0.6 g tangential at r of 0.2 to 0.3 m against a ring of 0.2 to 0.4 g [motion]) |

Gravity leakage is the constraint on the ring analysis: a tilt of theta leaks 17.5 mg per degree into a lateral axis,
and Banking moves the tilt by 9 to 12 degrees, so the leak in the 1 to 8 Hz band (20 to 150 mg) is comparable to the
ring [motion]. The calibration above is what makes the attitude correction possible.

## 4. Interfaces

- **Wire protocol (normative, P1a):** `ros_ws/docs/platform_imu_protocol.md`, covering IDs, frames, the frozen-capture
  image, the state machine, the RPCs and the upload walk-through.
- **YAML to generated constants:** the IMU CAN IDs, the status frame layout and the RpcMethod ids live in
  `config/protocol_config.yaml`; `python config/generate_config.py` writes the firmware and Python constants.
- **Platform Teensy:** pure headers for the capture ring, the upload framing (sequence, gap re-request, CRC-32) and
  the timestamp anchors, so each is testable on the host. The sketch only wires them to hardware.
- **can-bridge:** a classifier for the IMU IDs beside `is_platform_reply_id`, a dedicated ring, a UDP route to the
  Jetson with no PROTOCOL bump.
- **Jetson:** the IMU handler in `teensy_bridge_node`, the status topic, and the capture writer. The freeze trigger
  is the end of an Attempt, a Fault, or a Soft E-stop; the resume follows a CRC-checked upload.
- **Jetson trigger, gate and GUI (P3c, as built):** `teensy_bridge_node` subscribes `skills/attempt` (`juggle_start` /
  `juggle_end`, published by `skill_node` for every goal source) and the guard fault edge, and calls `attempt_ended` with
  the span from `platform_imu_gate.AttemptSpanTracker` (2 s before the Attempt starts to 3 s after its end, last 32 s if
  longer), fire-and-forget. The gate (`platform_imu_gate.imu_upload_gate`, cached `platform_imu/status` only, fail-open
  when older than 2 s or absent, hard ceiling 180 s with an abort) has three call sites: the orchestrator command topic,
  the orchestrator Juggle relay and `skill_node`'s Juggle goal callback. The GUI row is in Platform Control.
- **Jetson gate and GUI (P3):** the "Upload IMU data after each Attempt" toggle (default ON); while an upload runs
  with the toggle ON the skill/orchestrator side refuses the Attempt-starting command (the Juggle request, the GUI
  Start button; the owner's word is JUGGLE) and any other command that starts motion (`standby`, `trajectory`,
  `spacemouse`, `gui`, `level`, `home`); `deactivate` and a Soft E-stop always pass and send IMU ABORT. The GUI shows
  progress and time remaining. The owner's "STANDBY pose" is the Active pose held in `ActiveMode.STANDBY`; his
  "IDLE" is the Idle state (`RobotState.IDLE`, Legs Idle).

## 5. Steps

### P1: Off-board foundations

Protocol YAML and codegen for the IDs and RPC; the pure headers (capture ring, upload framing and CRC, timestamp
anchors) with native tests; the driver lifted from the bench probe; the pin test and the `static_assert` updated for
pins 18 and 19 (`tests/firmware/test_platform_hand_fan_xref.py` hard-codes `(10,11,12,13,22,23)` [fw]); a `pio`
compile of the Platform sketch. Hardware: none. **Gate (Suite gate):** `./run_tests.sh --full` (controller and sim
are untouched, but `config/` is tier 1). Status: P1a and P1b landed with the phase-end audit's fixes; the spec is amended for the 2026-11-19 owner decisions (uploads while Armed, the under-a-minute target); the `--full` gate is pending (`logbook/2026-11-18-platform-imu-p1-foundations.md`).

### P2: Platform Teensy firmware FW 11

The robustness changes, the IMU state machine, the ring, the status frame and the upload responder (section 3).
Hardware: none. **Gate (Suite gate):** native tests, `pio` build, `/audit` of the diff, then `./run_tests.sh --full`.
Status: P2 complete (2026-11-19, FW 11 on branch `imu-p2`), pending the phase gate (`./run_tests.sh --full`); the phase-end
audit ran and its findings are closed (0 blocking, 4 behaviour, 4 test, 5 narrative). P2a (robustness: tilt off the CAN callback, bounded inclinometer probe, multi-frame
dispatch) and P2b (the IMU wired into `loop()` through `imu_service.h`: one bounded step per pass, back-off to 60 s,
the TX-mailbox rule, a 10 s boot settle, FW_HALTED at FwUpdate BEGIN with a latch when BEGIN lands in the settle, kill switch `PLATFORM_IMU_ENABLED`) are both on branch `imu-p2`.
Walk-through and sizes: `logbook/2026-11-19-platform-imu-p2-firmware.md`. Not flashed.

### P3: can-bridge firmware and the Jetson side

The whitelist, the IMU ring, the batched IMU_DATA UDP message and the two RPCs on the bridge (READ allowed while
Armed; the gap clamp; no ABORT on arming); decode, capture writer, automatic freeze and upload at the configured pace,
the upload toggle, the refusal of motion-starting commands during an upload, the progress display, status topic and bag
list on the Jetson. Also from P2 (spec section 4, "Recovery if IMU code faults"): (a) the uploader BEGIN retry
(`teensy_link_bridge.py`: retry BEGIN for at least 20 s so a fault-loop window of about 10 s every 18.5 s is always hit;
this replaces the manual "re-run the upload until BEGIN OK" rule once built; today it sends one BEGIN and waits 1.0 s,
`_FW_REPLY_TIMEOUT_S`); (b) Jetson handling of the tilt invalid reply (detect the raw NaN words before the float bound
check, log the reason, retry NOT_READY no sooner than 1 s, treat (pi, pi) as the FW <= 10 failure reply); (c) no IMU_STATUS
for 10 s after a boot is normal; (d) the kill-switch build (ignores IMU_CMD, sends no STATUS, still reports FW 11) reads as
"no IMU", not a fault. Hardware: none. **Gate (Suite gate):** `./run_tests.sh --full`, including the xref
and version tests. Status: P3a (the can-bridge side, FW 45) built on branch `imu-p3`
(`logbook/2026-11-19-platform-imu-p3-bridge-jetson.md`). P3b (the Jetson side: codecs, upload state machine, timestamp fit, capture
files, node wiring and API, tilt invalid reply, uploader BEGIN retry; `teensy_link/platform_imu.py`,
`teensy_link/platform_imu_capture.py`) built on the same branch; items (a), (b), (c), (d) above are done in it. P3c (the Attempt-end trigger, the
refusal of motion-starting commands during an upload, the GUI toggle and progress display) built on the same branch
(2026-11-19): the trigger lives in `teensy_bridge_node` (it listens to `skills/attempt` and the guard fault edge, so
`skill_node` is untouched there); the gate is the pure `jugglebot/platform_imu_gate.py`, called once in
`orchestrator_node._on_command`, once in `_svc_juggle_request` and once in `skill_node._juggle_goal`; the GUI is
`ros_ws/gui/js/imu-upload-panel.js`. Refused: `standby`, `trajectory`, `spacemouse`, `gui`, `level`, `home` and the Juggle
request; always allowed: `deactivate` (aborts the upload), the Soft E-stop (the bridge aborts on the Fault edge), `activate`,
`clear_errors`, `juggle_stop`. Switching the toggle OFF mid-upload aborts it. Hardware proof (the ramp) stays P4.

### P4: Sitting 1, flash and bench the capture path

Hardware: robot. Flash the can-bridge, then the Platform Teensy over CAN. **Rung gate:** identity of both boards
reads as flashed; a tilt request answers; the Hand fan still answers; with the IMU absent and unplugged the board
boots, answers CAN and accepts an update; the status frame arrives; a capture at rest reads |a| about 1 g at 1 kHz
with 0 overflows; an upload passes its CRC and a forced gap is re-requested and healed; the bus-load counters in
`/profile` are unchanged during an Attempt-like move; the status frame returns to RUNNING after the IMU is replugged
without a power cycle; the rate ramp below. Status: passed at the Sitting of 2026-11-20 (owner; `2026-11-20-sitting-debrief-robot-2`): replug, Armed soak and Attempts with 0 I2C errors, automatic uploads complete. Residue carried to P5 and IDEA-25: the forced gap, the rate ramp, the upload after a Fault, and Armed uploads taking 79 to 84 s.

Done at rest (`2026-11-20-platform-imu-status-counters-and-first-capture`, `2026-11-20-platform-imu-i2c-error-bursts`):
both identities read as flashed (FW 12 and bridge FW 46); the tilt request and the Hand fan answer; the status frame
arrives and carries the new counters; the at-rest capture reads |a| 1.0004 g at about 1 kHz with 0 overflows; one
upload passed its CRC with no holes (56.25 s, not Armed). GYRO_ACCEL_CONFIG0 reads 0x00 in the header (ODR/2; the
byte is the firmware's `UI_FILT_CODE` constant, not a chip read), and ODR/2 is now the decided setting (owner, 2026-11-20). The capture header's `temp_raw_start` and
`temp_raw_freeze` of -128 were not a defect: they are TEMP_DATA counts, 24.03 C, matching the status frame's 24 C; the
capture `.json` now carries them decoded (`2026-11-20-platform-imu-capture-header-temperature`).

Remaining after the Sitting of 2026-11-20: the forced-gap upload (no mechanism forces a gap), the rate ramp, the
automatic upload after a Fault, and the Armed upload time (IDEA-25). Clock-sync quality (`sync_peak` saturating at
2.54 ms) stays listed residue.

**Rate ramp.** Holding the between-Attempts pose, step the upload pace through 400, 800, 1250 and 1600 frames/s. At
each step read the bridge's loop and emit-gap counters, RX queue depth, bus error counts, ODrive errors and the Leg
position error. The operating rate is the highest step with no change from the idle baseline. Pre-registered fallback
if 1250 is not clean: the highest clean rate with a longer wait, then tighter sample packing (section 6). The UI-filter
code is settled at 0x00, ODR/2 (owner, 2026-11-20; it was open since 2026-11-19 because the datasheet text could not
be obtained). The tilt request's explicit invalid reply with the inclinometer absent is covered by
P2's native tests (the SCL3300 is not unplugged on the robot); the invalid encoding of the 0x6E0 frame is a P2 item.

### P5: Sitting 2, mount calibration and first Attempt captures

Hardware: robot. The mount calibration (Level, then X and Y Tilts with Mocap recording), then real Attempts with the
automatic freeze and upload. **Rung gate:** a calibration file beside the other calibration artefacts, and one real
Attempt's capture on disk, complete and CRC-checked, handed to the carry-ring-zvd analysis. Status: in progress (2026-11-22): the Sitting passed (`2026-11-22-sitting-debrief`) and the mount rotation is fitted (gravity within 0.013 deg at Level and both Tilts, board tilted 3.0 deg about Platform X). The calibration file exists: `config/platform_imu_mount.json` (2026-11-23, `2026-11-23-platform-imu-mount-calibration-file`), loaded by `teensy_link/platform_imu_mount.py` and served by the GUI as the Platform frame by default. Not yet met: one real Attempt capture, complete and CRC-checked (the Attempt captures so far are span uploads, which the payload CRC does not cover; a span checksum is being added separately). P5 is not complete. The lever arm stays at its CAD value (the Jog moves could not fit it).

## 6. Open questions

- **Lever arm and nominal mount (owner, from CAD, 2026-11-19):** the IMU sits at about (5, -115, 18.5) mm in the Platform frame. Nominal axes: IMU x = Platform +z, IMU y = Platform -x, so IMU z = Platform -y (right-handed). These seed the P5 mount fit, which measures the real rotation; the position is approximate until the bracket is fitted.
- **Bridge FW number:** P3a took 45 (2026-11-19) because the held BUG-93 branch had not landed on main; whichever of
  the two reaches main second renumbers (BUG-93 to 46 if this lands first). No PROTOCOL bump either way.
- **Upload time (owner, 2026-11-19: "keep this area open; I will want to explore minimising it once the MVP works"):**
  three levers, filed as an Idea under this Pathway ("Minimise the Platform IMU upload time after an Attempt"): (1)
  tighter sample packing or compression (the 6 x int16 words are raw chip counts); (2) a slower
  setpoint heartbeat while holding (decided 2026-11-19 to keep the 500 Hz stream as is for now: it feeds the ODrive
  watchdog and avoids a rate switch on the Leg path); (3) a higher measured rate from the P4 ramp.
- **Span-only uploads cannot be CRC-checked (IDEA-25 note, P3 audit 2026-11-19):** the payload CRC covers the whole
  frozen capture, so an upload of only the Attempt span (the default) records `crc_checked = false`. Per-block
  checksums would close it and need a firmware and spec change; until then a span capture's integrity rests on the
  per-frame sequence numbers and the hole-refill, and `complete` means "the requested span arrived", not "verified".
- **Owner confirmations pending (P3 audit):** (1) `activate` and `clear_errors` are not gated while an upload runs
  (the gate refuses only motion-starting commands); (2) toggle OFF aborts a running upload; (3) the Fault upload delay:
  a Fault or Soft E-stop freezes at once but starts the READ stream `imu_fault_upload_delay_s` (default 5 s, clamped
  to 60 s, and held while still Armed up to 30 s) later, so the upload never competes with the deactivate response.
- **Residue, recorded and not solved here:** (a) tighten the wall-clock chain once the status frame shows the real
  sync quality (stamp sync frames nearer the receive interrupt; min-RTT filtering of the bridge's Jetson anchor);
  (b) the encoder-versus-Mocap ring-frequency discrepancy (5.4 versus 4.4 to 4.7 Hz), which the IMU should settle;
  (c) BUG-90's 5 Hz Hand stop-go as a second excitation the IMU will see.

## 7. Out of scope

A live IMU stream or any online use of the data; control-loop feedback from the IMU; a six-face or
temperature-compensated calibration; a bench self-update rehearsal or any extra recovery mechanism (section 3).
