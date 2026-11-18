---
title: Platform IMU — an ICM-42688-P on the Platform Teensy, buffered capture over CAN for offline input-shaping work
created: 2026-11-18
status: active
owner: Harrison
last_updated: 2026-11-18
pathway: two-ball-oval
scope: >-
  Put an ICM-42688-P on the Platform's 2020 extrusion (the one carrying the Hand's rail), read it over I2C from the
  Platform Teensy at 1 kHz into a 32 s RAM ring, and have the Jetson freeze, upload and store the ring at the end
  of every Attempt, so the ring mode (about 4.5 to 5.4 Hz, carry-ring-zvd) can be tuned offline from real
  acceleration instead of twice-differentiated Mocap. No online use and no live stream. Closes when a Level plus
  X/Y Tilt mount calibration is fitted and a real Attempt's capture is on disk, complete, CRC-checked and handed to
  the carry-ring-zvd analysis.
steps:
  - {id: P1, title: Off-board foundations, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: P2, title: Platform Teensy firmware FW 11, kind: phase, status: not_started, effort: l, hardware: none}
  - {id: P3, title: can-bridge firmware and the Jetson side, kind: phase, status: not_started, effort: l, hardware: none}
  - {id: P4, title: "Sitting 1, flash and bench the capture path", kind: rung, status: not_started, effort: m, hardware: robot}
  - {id: P5, title: "Sitting 2, mount calibration and first Attempt captures", kind: rung, status: not_started, effort: m, hardware: robot}
related_logbook:
  - 2026-11-18-imu-bench-proof-of-concept.md
related_code:
  - tools/probes/imu_bench/firmware/src/icm42688.cpp
  - ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - config/protocol_config.yaml
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
| Bench IMU result | 0 FIFO overflows and 0 I2C errors at 1 and 2 kHz; 999.97 Hz | IDEA-20 bench proof of concept |

The ranges (accel +/-16 g, gyro +/-1000 dps) are wider than the table needs, by the owner's request. The gyro step
is 0.031 dps, under the 0.05 to 0.07 dps rms bench noise, so the wide range costs nothing measurable.

## 3. Decisions and the failure each one prevents

| Decision | Failure it prevents |
| --- | --- |
| I2C on Platform Teensy pins 18 and 19 (Stemma QT), 1 MHz with 400 kHz fallback. No INT1 or CLKIN wire, no new interrupt source | An interrupt running during the FwUpdate COMMIT copy touches flash-resident code and can brick a board whose only recovery is that update path [fw] |
| ICM-42688-P at ODR 1 kHz, UI filter ODR/4 (register code checked against the datasheet as the bench README did), 16-bit FIFO packets, driver lifted from `tools/probes/imu_bench/firmware/src/icm42688.*` | A second, untested driver; 20-bit packets that break the 6 x int16 layout |
| No live stream. One 1 Hz status frame during an Attempt (state, temperature, FIFO overflow and I2C error counters, clock-sync quality, capture fill). Live-stream IDs reserved, not used | Adding bus load where an Attempt already runs about 64 %, for data the owner will not use online |
| Capture ring in RAM2 (`DMAMEM` static array, about 384 KB, clear of the top 128 B CrashReport area), recording continuously while the IMU is healthy, 6 x int16 per sample, 32 s at 1 kHz. The Jetson freezes it at Attempt end and on a Fault or Soft E-stop, uploads, then resumes. Attempts over 32 s keep the last 32 s | Missing the capture of the Attempt that went wrong (the freeze on Fault); a ring overrunning into CrashReport |
| Each FIFO drain batch is stamped with `TimeSync::get_wall_time_us()`; sample times are rebuilt on the Jetson from sample count and a fitted chip rate (chip clock good to about 1 %, FIFO tick 32/30 us). Metadata carries the anchors. Clock-sync quality goes in the status frame | Trusting the nominal ODR and drifting about 1 % against the Mocap and encoder logs; clock quality that today only prints to the dead USB console [fw] |
| Upload is host-paced pull, the FwUpdate pattern reversed: the Jetson requests blocks, the Platform sends sequence-numbered 8-byte frames, the Jetson re-requests gaps, CRC-32 over the capture, refused or aborted while armed (an interrupted capture is flagged incomplete). Platform TX is paced. A wait of a few minutes is accepted | A babbling Platform loading the loop in flight; the FlexCAN_T4 all-mailboxes-busy defect [fw surprise 3]; a corrupt capture that looks good |
| New CAN IDs from the free block next to the Platform's (0x6E3 to 0x6EF, 0x6F2 to 0x6FF [fw][can]), defined in `config/protocol_config.yaml` and generated; commands as new typed RpcMethod(s), next free 0x64 | A hand-written ID colliding with an ODrive node range or the cone and Wave block; generated headers drifting from the YAML |
| can-bridge whitelists the new Platform IDs and gives IMU frames their own uplink ring; drain rate raised only when not armed. FW bump, no PROTOCOL bump | IMU frames crowding the 8-slot ring that tilt, fan and FwUpdate replies share, so a capture dump starves them (drop-newest) [fw][can] |
| Platform Teensy robustness changes ride along: the tilt request becomes a flag serviced in `loop()` with a bounded failure path (today at least 300 ms stalls inside the CAN callback); the SCL3300 wait-forever in `setup()` is bounded; `loop()` dispatches several CAN frames per pass (today one). FW 10 to 11 | A busy callback blocking CAN reception while the IMU is also being served; a missing inclinometer hanging boot [fw] |
| The IMU is strictly optional: a non-blocking probe and init state machine in `loop()` (never `setup()`), bounded Wire timeouts, back-off on errors, IMU work stops at FwUpdate BEGIN, a CAN disable command | An unplugged or misbehaving IMU stopping the board from booting, answering CAN or accepting a firmware update |
| Recovery stance (owner, 2026-11-18): the CAN firmware update is the trusted recovery path and power is reliable. No bench self-update rehearsal, no safe-boot window, no opt-in-per-boot, no crash breadcrumbs. Off-board checks only: native tests of the pure logic (the `tests/firmware/native` host g++ and doctest pattern), a `pio` compile, then flash | Building recovery machinery for a risk the owner has judged acceptable; it is recorded so a later Session does not reopen it unasked |
| Jetson: `teensy_bridge_node` gets an IMU handler with its own queue (today it keeps only the latest frame per CAN id); status on a topic added to the rosbag list in `jugglebot_launch.py`; captures written to `temp/imu_capture/<stamp>` with metadata (ODR, ranges, clock anchors, temperature, CRC result, complete flag), following `hand_capture.py`. Automatic freeze and upload at Attempt end and on Fault from the start | Latest-frame-wins dropping capture frames; a status topic that is never bagged; a capture needing a hand to start it |
| Mount calibration, the owner's simpler procedure: fasten the sensor; at Level take gravity for the IMU's Z (two mount angles); Tilt the Platform about its local X, then Y (static holds plus the moves, Mocap recording) and fit the third angle. Gyro bias zeroed from a still window before each Attempt (Jetson side). Factory scale and bias otherwise; no six-face solve, no temperature compensation (temperature is logged). Applied on the Jetson, raw counts on the wire. Lever arm from the owner's bracket CAD, needed to about 5 mm | Re-flashing firmware for a calibration change; a lever-arm error that mixes angular acceleration into translation (0.3 to 0.6 g tangential at r of 0.2 to 0.3 m against a ring of 0.2 to 0.4 g [motion]) |

Gravity leakage is the constraint on the ring analysis: a tilt of theta leaks 17.5 mg per degree into a lateral axis,
and Banking moves the tilt by 9 to 12 degrees, so the leak in the 1 to 8 Hz band (20 to 150 mg) is comparable to the
ring [motion]. The calibration above is what makes the attitude correction possible.

## 4. Interfaces

- **YAML to generated constants:** the IMU CAN IDs, the status frame layout and the RpcMethod ids live in
  `config/protocol_config.yaml`; `python config/generate_config.py` writes the firmware and Python constants.
- **Platform Teensy:** pure headers for the capture ring, the upload framing (sequence, gap re-request, CRC-32) and
  the timestamp anchors, so each is testable on the host. The sketch only wires them to hardware.
- **can-bridge:** a classifier for the IMU IDs beside `is_platform_reply_id`, a dedicated ring, a UDP route to the
  Jetson with no PROTOCOL bump.
- **Jetson:** the IMU handler in `teensy_bridge_node`, the status topic, and the capture writer. The freeze trigger
  is the end of an Attempt, a Fault, or a Soft E-stop; the resume follows a CRC-checked upload.

## 5. Steps

### P1: Off-board foundations

Protocol YAML and codegen for the IDs and RPC; the pure headers (capture ring, upload framing and CRC, timestamp
anchors) with native tests; the driver lifted from the bench probe; the pin test and the `static_assert` updated for
pins 18 and 19 (`tests/firmware/test_platform_hand_fan_xref.py` hard-codes `(10,11,12,13,22,23)` [fw]); a `pio`
compile of the Platform sketch. Hardware: none. **Gate (Suite gate):** `./run_tests.sh --full` (controller and sim
are untouched, but `config/` is tier 1). Status: not started.

### P2: Platform Teensy firmware FW 11

The robustness changes, the IMU state machine, the ring, the status frame and the upload responder (section 3).
Hardware: none. **Gate (Suite gate):** native tests, `pio` build, `/audit` of the diff, then `./run_tests.sh --full`.
Status: not started.

### P3: can-bridge firmware and the Jetson side

The whitelist, the IMU ring and the RPC on the bridge; decode, capture writer, automatic freeze and upload, status
topic and bag list on the Jetson. Hardware: none. **Gate (Suite gate):** `./run_tests.sh --full`, including the xref
and version tests. Status: not started. The bridge FW number is open (section 6).

### P4: Sitting 1, flash and bench the capture path

Hardware: robot. Flash the can-bridge, then the Platform Teensy over CAN. **Rung gate:** identity of both boards
reads as flashed; a tilt request answers; the Hand fan still answers; with the IMU absent and unplugged the board
boots, answers CAN and accepts an update; the status frame arrives; a capture at rest reads |a| about 1 g at 1 kHz
with 0 overflows; an upload passes its CRC and a forced gap is re-requested and healed; the bus-load counters in
`/profile` are unchanged during an Attempt-like move. Status: not started.

### P5: Sitting 2, mount calibration and first Attempt captures

Hardware: robot. The mount calibration (Level, then X and Y Tilts with Mocap recording), then real Attempts with the
automatic freeze and upload. **Rung gate:** a calibration file beside the other calibration artefacts, and one real
Attempt's capture on disk, complete and CRC-checked, handed to the carry-ring-zvd analysis. Status: not started.

## 6. Open questions

- **Lever arm:** from the owner's bracket CAD, needed to about 5 mm. Pending; P5 cannot close without it.
- **Tilt request with no inclinometer:** the owner did not say what the Platform Teensy should reply. Proposal,
  to confirm: an explicit error or invalid reply, never silence and never a stale value.
- **Bridge FW number:** the BUG-93 held branch takes 45, so this plan's bridge build is 45 or 46. Coordinate when
  P3 starts; no PROTOCOL bump either way.
- **Residue, recorded and not solved here:** (a) tighten the wall-clock chain once the status frame shows the real
  sync quality (stamp sync frames nearer the receive interrupt; min-RTT filtering of the bridge's Jetson anchor);
  (b) the encoder-versus-Mocap ring-frequency discrepancy (5.4 versus 4.4 to 4.7 Hz), which the IMU should settle;
  (c) BUG-90's 5 Hz Hand stop-go as a second excitation the IMU will see.

## 7. Out of scope

A live IMU stream or any online use of the data; control-loop feedback from the IMU; a six-face or
temperature-compensated calibration; a bench self-update rehearsal or any extra recovery mechanism (section 3).
