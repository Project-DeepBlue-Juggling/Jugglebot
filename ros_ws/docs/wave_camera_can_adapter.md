# Wave Camera: CAN adapter design

> **Wave Camera sub-project.** Jugglebot starts and stops recording on a
> Freefly Wave high-speed camera from ROS 2 through an ESP32 on the cone-role
> CAN bus (CAN3), and stamps each clip start on the robot wall clock. The ESP32
> firmware lives in [`ros_ws/src/jugglebot/WaveCamera_ESP_code/`](../src/jugglebot/WaveCamera_ESP_code/README.md)
> (a standalone PlatformIO project, built and flashed from a PC, never from the
> Jetson). Decision: [ADR-0014](../../docs/adr/0014-wave-camera-on-can3.md); plan:
> [wave-camera-can3-integration.md](../../plans/active/wave-camera-can3-integration.md). Sibling docs:
> [GPIO explorer design](wave_camera_gpio_explorer.md),
> [CAN adapter design](wave_camera_can_adapter.md),
> [measured findings](wave_camera_findings.md).
> This document is the **binding wire and behaviour contract** that ADR-0014 and the
> plan point at. It was moved here from the former standalone adapter repo on
> 2026-10-25; the "Status" and "Nothing here is implemented yet" lines below
> are its 2026-10-19 approval record, and the plan holds the live status.


Status: APPROVED by owner 2026-10-19 for implementation.

Nothing here is implemented yet.
Paths starting with `JB/` are in the Jugglebot repo (read on `skill-stack`), `EC/` in
Electronic-Clapboard, and bare paths are relative to
`ros_ws/src/jugglebot/WaveCamera_ESP_code/` (formerly the standalone adapter repo). Line numbers are from
the 2026-10-19 checkouts. Lines that start with "Choice:" mark decisions this
document makes where the owner had not ruled.

## 1. Goal and scope

Let the robot start and stop recording on the Freefly Wave from ROS2, and
report each start and stop as a robot wall-clock instant good to 1 ms. The
ESP32 drives GPI2 low to record (camera in LVL mode) and stamps the first
30 Hz camera tick after the falling edge, which [wave_camera_findings.md](wave_camera_findings.md) shows is the tick
that starts the clip. Its clock is slaved to the 0x7DD beacon. It talks to the
can-bridge Teensy on the cone-role bus (the bridge's CAN3 peripheral). On the
Jetson side this adds a decoder, two Trigger services, two topics and a GUI
panel.

Non-goals: working out which recorded frame matches the start tick, i.e. the
optical calibration with a clapboard flash (findings doc, open item 1); edge-mode
GPI; the camera's UART input (it is output-only, findings doc "QX10001");
recording started automatically by the skill stack; running the cone or
clapboard on the same segment at the same time.

Owner-approved behaviours (2026-10-19): the ESP releases GPI2 if the camera
30 Hz tick disappears mid-recording (TICK_LOST, s5.2), and it auto-stops with
GPI2 released if no WAVE_CMD intent arrives for 3 s (INTENT_TIMEOUT, s5.3).

## 2. System diagram

```
 Browser GUI (camera-panel.js)
   | roslib WebSocket :9090
 rosbridge_server
   | ROS2 topics/services: camera/heartbeat 10 Hz, camera/record_event,
   |                       camera/record_start, camera/record_stop,
   |                       camera/clear_faults (Trigger)
 teensy_bridge_node (Jetson, the single UDP owner)
   | 5 Hz RPC WAVE_CMD (0x0062)        ^ CONE_FRAME 0x85 (verbatim relay)
   v UDP RPC port                      | UDP stream port
 can-bridge Teensy 4.1 (FW 33)
   | can_cone_tx(TxCls::RPC) 0x7E2     ^ on_cone_rx -> 16-slot ring -> 4/tick
   | 0x7DD TIME_SYNC 100 Hz            |
   v CAN3 periph, pins 31 TX/30 RX, 1 Mbit/s classic, 11-bit, DLC 8
 ===== cone-role segment: Teensy end ...... 4-way drop (+12 V, GND, CANH, CANL) =====
   |
 SN65HVD230 (3V3) + 120 R at the ESP end
   | GPIO13 TX / GPIO34 RX
 ESP32-WROOM-32D DevKitC (WaveCamera_ESP_code: explorer console + CAN modules)
   | GPIO26 open-drain -> GPI2 (pin 6)    GPIO27 <- 30 Hz tick (pin 4)
   | GPIO32 <- UART "Created new clip." (pin 3)  3V3 -> VIN (pin 2)
 Freefly Wave (GPI = LVL)
```

## 3. Hardware

Pins changed 2026-10-23 (owner-approved): GPI2 GPIO25 -> GPIO26, Pin 3
GPIO26 -> GPIO32, Pin 5 not wired (found inert), GPIO25 unused. CAN pins
unchanged.

| Function | ESP32 pin | Part / wiring | Mode |
|---|---|---|---|
| Wave pin 1 GND | GND | direct | |
| Wave pin 2 VIN | 3V3 | direct, from the buck's 3.3 V rail (camera draws < 20 mA, [GPIO explorer design](wave_camera_gpio_explorer.md) s1) | |
| Wave pin 3 UART TX out | GPIO32 | 1 k series, 10 k pull-up to 3V3 | UART2 RX + edge ISR |
| Wave pin 4 30 Hz tick | GPIO27 | 1 k series, 10 k pull-up | input, edge ISR |
| Wave pin 5 GPI1 | (none) | not connected | |
| Wave pin 6 GPI2 | GPIO26 | direct | open-drain, single writer (s5.4) |
| CAN TX | GPIO13 | SN65HVD230 D | TWAI TX |
| CAN RX | GPIO34 | SN65HVD230 R | TWAI RX (input-only is fine) |
| Transceiver supply | 3V3 (buck rail) | SN65HVD230 Vcc, 100 nF at the pin; Rs to GND (high-speed slope) | |
| Termination | | 120 R across CANH-CANL at the ESP end; the bridge end of the CAN3 segment is already terminated (owner confirmed) | |
| Power | 3V3 pin | 12 V drop -> buck set to 3.30 V (>= 1 A part) -> DevKitC 3V3 pin directly (NOT the 5V pin); no Schottky. Same rail feeds the SN65HVD230 and camera VIN | |
| Drop | | 4-way keyed: +12 V, GND, CANH, CANL | |

Why GPIO34 and GPIO13: neither is a strapping pin (0, 2, 5, 12, 15), a flash
pin (6-11) or a UART0 pin, and both are free of the camera pins
(include/config.h `is_safe_esp32_gpio` and the camera-pin static_asserts). TWAI RX only needs an input, so an input-only pad
is a good use of it. GPIO13 is an ordinary output. `is_safe_esp32_gpio`
refuses anything >= 34 because it is written for camera pins, so add a
separate `static_assert` for the CAN pair rather than relaxing it.

Power budget at 3V3: ESP32 with WiFi/BT off draws about 100-150 mA, with
peaks to about 300 mA at boot. SN65HVD230 up to about 70 mA in the dominant
state (about 10 mA recessive). Camera VIN under 20 mA. Worst case is roughly
400 mA, typically about 200 mA. From 12 V through an 85 % buck that is under
about 0.4 W input. The buck is set to 3.30 V and rated for at least 1 A.

Power caution (owner decision 2026-10-19): the buck feeds the 3V3 pin
directly, with no Schottky diode. The owner uses a USB isolator whenever USB
and CAN are connected at the same time. With USB attached, the isolator's 5 V
still reaches the onboard LDO, so the LDO and the buck sit in parallel on the
3.3 V rail. Set the buck to 3.30 V (not higher) and rate it for at least 1 A.
The ground reference is shared through the drop and through camera pin 1.
The bridge end of the CAN3 segment is terminated (owner confirmed), so the
ESP end gets the 120 R. Before the first power-on, with everything off,
measure CANH-CANL: 60 R means both ends are terminated.

## 4. CAN frame contract

Common to all frames: classic CAN, 11-bit IDs, DLC is always 8, unused bytes
are 0, and payloads are little-endian. The block is 0x7E2-0x7E7, inclusive
bounds `BLOCK_FIRST`/`BLOCK_LAST` as in the clapboard YAML precedent. 0x7E6
and 0x7E7 are reserved. All four IDs are numerically above 0x7DD, so the
time beacon always wins arbitration against this traffic.

### 4.1 WAVE_CMD 0x7E2, Jetson to ESP, 5 Hz

| Byte | Field | Type | Meaning |
|---|---|---|---|
| 0 | desired | u8 | 1 = record, 0 = do not record; anything else is ignored and counted |
| 1 | cmd_seq | u8 | bumped by `record_start` and `record_stop`; repeats while streaming (`camera/clear_faults` re-sends it unchanged) |
| 2 | flags | u8 | bit0 CLEAR_FAULTS (clears the latched fault byte); others 0. One-shot: set on `record_start`'s new-seq frame and on the `camera/clear_faults` frame, never on the 5 Hz repeats |
| 3 | reserved | u8 | 0 |
| 4-7 | session_id | u32 | node start time, Unix s; a change resets `recordings_this_session` |

Choice: the node streams WAVE_CMD at 5 Hz whenever the ESP heartbeat is
fresh, with desired=0 as well as desired=1. A lost stop frame is then
repaired within 200 ms. This also gives the ESP a host-alive bit. Silence
only matters while recording.

### 4.2 WAVE_HEARTBEAT 0x7E3, ESP to Jetson, 10 Hz unconditionally from boot (opens the bridge TX gate)

| Byte | Field | Type | Units / meaning |
|---|---|---|---|
| 0 | state | u8 | 0 BOOT, 1 IDLE, 2 STARTING, 3 RECORDING, 4 STOPPING, 5 FAULT |
| 1 | status | u8 | bit0 CAMERA_ON (tick within 200 ms), bit1 UART_SEEN (a clip line since boot), bit2 TIME_SYNCED, bit3 INTENT_FRESH (WAVE_CMD within 3 s), bit4 DESIRED (last intent), bit5 CONSOLE_OWNS, bit6 GPI2_LOW (pad level from ISR), bit7 0 |
| 2 | faults | u8 | latched: bit0 GPI2_REFUSED, bit1 TICK_LOST_MID_REC, bit2 CLIP_MISSING, bit3 INTENT_TIMEOUT_AUTOSTOP, bit4 BUS_OFF_SEEN, bit5 SYNC_STEP, bit6 EDGE_RING_OVERFLOW, bit7 0 |
| 3-4 | elapsed | u16 | 100 ms units since the start tick, 0 when not recording; saturates at 6553.4 s (109 min) |
| 5 | sync_age | u8 | 100 ms units since the last accepted 0x7DD; 255 = never or >= 25.5 s |
| 6 | recordings | u8 | starts since session_id last changed (wraps) |
| 7 | cmd_seq_echo | u8 | last WAVE_CMD seq the ESP acted on |

Unit choices: GUI elapsed only needs 0.1 s, because the panel interpolates
between frames, and u16 at 100 ms covers any realistic take. sync_age only
needs to tell 0.1 s from 25 s; the precise jitter is in WAVE_INFO.

### 4.3 WAVE_REC_EVENT 0x7E4, ESP to Jetson, on event

| Byte | Field | Type | Meaning |
|---|---|---|---|
| 0-4 | t40 | u40 | if SYNCED: low 40 bits of the event's wall time in µs; else the age of the event at TX in µs |
| 5 | kind/flags | u8 | bits0-2 kind: 1 START, 2 STOP, 3 AUTO_STOP, 4 REFUSED; bit3 SYNCED; bit4 TICK_STAMPED; bits5-7 reason |
| 6 | cmd_seq_echo | u8 | seq of the WAVE_CMD that caused it (AUTO_STOP: last seen) |
| 7 | recording_seq | u8 | recording counter (wraps); the node unwraps it to u32 |

Reason codes (3 bits): 0 NONE, 1 NO_CHANGE (already in the requested state),
2 INTENT_TIMEOUT, 3 TICK_LOST, 4 CAMERA_OFF, 5 CLIP_MISSING, 6 CONSOLE_OWNS,
7 GPI2_FAULT.

40 bits rather than the clapboard's 48: 2^40 µs is 12.7 days, so the node
rebuilds the full time against its own wall clock with a ±6.4-day window.
That is the same algorithm as the clapboard branch's
`reconstruct_fire_time_us` (`git show origin/2026-08_clapboard-can3:ros_ws/src/jugglebot/jugglebot/can/clapboard.py`
lines 247-279) with a 40-bit mask. The byte saved makes room for both
sequence numbers.

Unsynced events carry an age, and the node computes
`event = ConeFrame.t_bridge_us - age`. The bridge stamps `t_bridge_us` as
wall time in `on_cone_rx` (JB/ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp:493-510).
So an unsynced ESP still yields a sub-millisecond-class stamp, as long as the
bridge is synced, and the event is flagged `time_synced=false`.

### 4.4 WAVE_INFO 0x7E5, ESP to Jetson, one page every 250 ms in rotation (each page at 1 Hz)

| Page (byte 0) | Bytes 1-7 |
|---|---|
| 0 identity | 1-2 fw_version u16, 3 wire_version u8 (=1), 4-7 uptime_s u32 |
| 1 camera tick | 1-4 tick_period_ns u32 (mean over the last 30 ticks), 5-6 tick_jitter_us u16 (p-p), 7 ticks_missed u8 (last second, saturating) |
| 2 time sync | 1-2 offset_spread_us u16 (latency spread in the last window), 3-4 track_err_us u16 (last window vs prediction), 5 accepted_per_s u8 (expect 100), 6 rejected_per_s u8, 7 steps u8 (re-anchors since boot) |
| 3 link | 1-2 last_tick_to_clip_us u16 (expect about 15500), 3-4 can_tx_errors u16, 5 bus_off_events u8, 6 intent_rx_per_s u8, 7 0 |

The pages are spread through the second rather than sent as a burst, so they
never meet the 16-slot relay ring and the 4-frames-per-tick budget all at
once (JB/.../Teensy_code_canbridge/telemetry.cpp:198-216,
can_buses.cpp:485). Steady ESP load is about 14 frames/s.

### 4.5 Ack semantics

Each new `cmd_seq` produces exactly one WAVE_REC_EVENT carrying that seq:

- START when the clip is confirmed.
- START with reason CLIP_MISSING (see s5.2).
- STOP once GPI2 has been released.
- REFUSED with a reason. NO_CHANGE means the ESP was already in the
  requested state.

Frames that repeat a seq are keepalives only. `record_start` succeeds on
START with reason NONE, or on REFUSED/NO_CHANGE while recording.
`record_stop` succeeds on STOP, or on REFUSED/NO_CHANGE.

Clearing faults (added 2026-10-25 at the Jetson bring-up): the fault byte
latches, so the node clears it in two places. `record_start` sets
CLEAR_FAULTS on its new-seq frame; the adapter clears before it acts, so the
faults shown during a take are that take's. `camera/clear_faults` (Trigger;
the GUI's warning glyph calls it on click while faults are latched) sends
the CURRENT desired and cmd_seq with CLEAR_FAULTS. An unchanged seq is a
keepalive, so it never starts or stops a recording; a bumped seq would start
one from IDLE if the node's desired were still 1 (after a TICK_LOST
auto-stop). It succeeds when the second heartbeat after the send reads
faults 0, and fails naming any fault that re-latched.

## 5. ESP32 firmware design

### 5.1 Modules added to src/

| Module | Job | Pure, native-testable core |
|---|---|---|
| `can_link.{h,cpp}` | TWAI install at 1 Mbit/s NORMAL mode, accept-all filter, RX task pinned to core 0 at priority 2 that stamps `esp_timer_get_time()` straight after `twai_receive` returns, bus-off recovery (copy EC/src/can.cpp:301-345 and 352-387) | `wave_frames.h` encode/decode |
| `time_slave.{h,cpp}` | 0x7DD validation and offset estimator, `wall_from_local()` | `time_slave_math.h` |
| `record_ctrl.{h,cpp}` | recording state machine, the only caller of `gpi2_drive` in CAN mode | `record_fsm.h` |
| `heartbeat.{h,cpp}` | 10 Hz HEARTBEAT and the 4 Hz (250 ms) rotation of INFO pages, both from `loop()` | (uses `wave_frames.h`) |

Unchanged in role: `edge_capture`, `uart_link`, `gpi2_drive`, `pulse_stats`,
`latency_test`, `console`. The dispatch hooks in src/main.cpp:21-32 gain
calls to `record_ctrl::on_edge`, `record_ctrl::on_clip_line` and
`time_slave` tick bookkeeping. `loop()` (src/main.cpp:49-58) adds
`record_ctrl::poll` and `heartbeat::poll`.

Boot behaviour changes from the explorer's (src/main.cpp:46, "rec mode not
set; uart not bridged"): CAN mode calls `gpi2_drive::set_mode(Level)` and
`uart_link::set_rx(PIN_CAM_P3)` at boot. `include/config.h` gains
`FW_VERSION`, `CAN_TX_PIN 13`, `CAN_RX_PIN 34`,
`INTENT_TIMEOUT_MS 3000`, `TICK_LOST_MS 200` and
`CLIP_WAIT_AFTER_TICK_MS 150`. TWAI code follows the clapboard conventions:
ISRs only stamp and push (EC/CLAUDE.md:77), and the RX task is pinned to
core 0 (EC/src/can.cpp:386) while `loop()` runs on core 1.

### 5.2 Recording state machine

| State | Entry / action | Exit |
|---|---|---|
| BOOT | TWAI up, heartbeat starts immediately | first `loop()` pass -> IDLE |
| IDLE | GPI2 released | intent desired=1 (new seq or rising edge of desired) with CAMERA_ON and CAN owner -> `request_start()` -> STARTING. Otherwise REFUSED (CAMERA_OFF / CONSOLE_OWNS) |
| STARTING | wait for the GPIO26 (GPI2) falling-edge ISR stamp, then the first GPIO27 tick after it, then a `Created new clip.` line | clip line within 150 ms of that tick -> START, RECORDING. No line -> START with reason CLIP_MISSING, fault bit, RECORDING. `request_start()` false or no falling edge within 1 s (500 ms stretch plus margin) -> REFUSED GPI2_FAULT, IDLE |
| RECORDING | elapsed counts from the start tick | desired=0 -> STOPPING. Intent silent 3 s -> STOPPING (AUTO_STOP INTENT_TIMEOUT). No tick for 200 ms -> STOPPING (AUTO_STOP TICK_LOST) |
| STOPPING | `request_stop()`; gpi2_drive stretches to 500 ms minimum low | rising-edge ISR stamp -> emit STOP / AUTO_STOP stamped at the first tick after the rising edge (or the edge itself, TICK_STAMPED clear, if the camera is off) -> IDLE |
| FAULT | only for a refused drive whose pad level disagrees with the command | CLEAR_FAULTS or console `rec assume idle` -> IDLE |

Choice: CLIP_MISSING keeps GPI2 low and reports START with a reason, rather
than aborting. Aborting could kill a good take when the problem is only a lost
UART line. The service reports failure and the operator decides.

Choice: TICK_LOST releases GPI2. Otherwise a camera power-cycle with the pad
still held low would start recording on its own at power-up, because LVL
mode samples the level.

Start stamp: `t_start_local` is the first GPIO27 rising edge with
`t > t_fall(GPIO26)`, both stamped by ISR (src/edge_capture.h). The clip
line cross-checks it: `t_first_byte - t_start_local` must lie in
[10, 25] ms (measured 15.5-16.3 ms, findings doc "latency"). If it does not, the
tick one period earlier or later that satisfies the window is used, and
`last_tick_to_clip_us` records it.

The event is converted with `wall_from_local(t_start_local)` at emit time,
when the event is about 50 ms old; the 30 ppm drift over that time is
negligible. Stop quantisation to the tick is assumed, not measured. The
optical phase will confirm it.

### 5.3 Intent timeout

`last_intent_local_ms` is updated on every valid WAVE_CMD. If it is 3 s old
while in RECORDING or STARTING, the ESP auto-stops with reason
INTENT_TIMEOUT and latches fault bit3. Nothing else times out: an idle ESP
with no host is a normal state.

### 5.4 Single writer for GPIO26 (GPI2), and the console

Choice: CAN owns GPI2 from boot. New console commands `ctl console` and
`ctl can` hand over ownership:

- `ctl console` is refused unless the state is IDLE.
- While the console owns GPI2, `rec start|stop|toggle|assume` and `lat` work
  as today (src/console.cpp:246-275).
- CAN intents get REFUSED CONSOLE_OWNS, and heartbeat bit5 is set.
- While CAN owns it, those console commands print `ERR rec refused: can owns
  gpi2`.

Reason: the robot operator cannot see the serial console, but the console
user can see the heartbeat. A silent "CAN wins" would let a bench
`lat` run and the robot fight over the pad. `record_ctrl` and `latency_test`
are the only callers of `gpi2_drive::request_*`. A lint-style native test
greps that nothing else calls them.

Everything else on the console stays diagnostic: `status` gains a `can` line
(state, sync offset/age, rx/tx/err counters), and new `can` and `sync`
commands print the INFO pages. `arm` still gates `>`, `qx` and `tx hold`
(src/console.cpp:22, 114-117) and has no effect on GPI2.

### 5.5 Time slave

The clapboard slave already rejects DLC != 8 and usec >= 1e6
(EC/src/can_frames.h:191-199), but hard re-anchors on every frame
(EC/src/can.cpp:166-185). Here:

1. Validate: `!extd`, `!rtr`, DLC 8, usec < 1e6, and sec after 2026-01-01.
   Compute `offset = wall - t_rx_local` with `t_rx_local` stamped right after
   `twai_receive`.
2. Use a max-offset (minimum-latency) window rather than a median. The
   bridge stamps the beacon before writing it
   (JB/.../Teensy_code_canbridge/time_sync_master.cpp:50-72), so a deferred
   frame is always late, never early. The windowed maximum of `offset` over
   1 s (about 100 frames) is therefore the least-delayed sample.
3. Track the 30 ppm crystal skew with a least-squares line through the last
   8 window maxima. Prediction: `offset(t) = O_k + slope*(t - t_k)`.
4. Step detection: if a window maximum deviates from the prediction by more
   than 2 ms in 2 consecutive windows, re-anchor (clear history,
   `steps++`, fault bit5). A single frame that is more than 1 ms early is
   rejected as implausible.
5. The first anchor is the maximum of the first 10 valid frames.
   `time_synced` = anchored and sync_age < 5 s. With skew compensated, 5 s of
   coasting costs well under 0.1 ms.
6. Report `offset_spread_us`, `track_err_us` and the accepted/rejected rates
   in INFO page 2. Shared state sits behind a `portMUX` as in the clapboard.

The beacon goes silent when the bridge anchor is more than 90 s stale
(JB/.../time_sync_master.cpp:38, time_base.h:122). The ESP then coasts,
drops TIME_SYNCED after 5 s, and keeps recording. Choice: recording is
allowed unsynced, and events carry an age (s4.3), flagged.

## 6. Bridge firmware changes (JB/ros_ws/src/jugglebot/Teensy_code_canbridge/)

- `config/generate_udp_protocol.py`:
  - add `("WAVE_CMD", 0x0062, "Wave camera: send one WAVE_CMD intent frame on the cone bus")` after HAND_MOVE_TO (line 330), with an ADDITIVE comment in the BB_CANCEL style (line 241-247).
  - add `RpcArg("ArgWaveCmd", "WAVE_CMD", [desired u8, cmd_seq u8, flags u8, pad u8, session_id u32])` to `RPC_ARGS` (line 1615); 8 B.
  - Regenerate to `udp_protocol.h`, config/generated/udp_protocol.py and the Jetson tools copy (lines 2352-2357).
  - No new MsgType: the downlink uses RPC_REQUEST/RESPONSE and the uplink already rides CONE_FRAME. 0x94 stays free.
- `rpc.cpp`: a `case RpcMethod::WAVE_CMD` in `dispatch()` (line 260; the BB_CANCEL case at 592 is the model):
  - `take()` the arg; ERR_BAD_ARGS if `desired > 1` or pad != 0 (validate before gating, the BB_THROW ordering).
  - Build the 0x7E2 frame (`WaveCanId::CMD`, DLC 8, the arg bytes verbatim).
  - Add a new `send_cone_frame()` beside `send_bb_frame` (line 223): `can_cone_tx(f, TxCls::RPC)`. A FAILED result (the presence gate, `BUS_PARTNER_STALENESS_US` = 5 s, canbridge_config.h:356) maps to ERR_BUS_DOWN, any other non-wire result to ERR_TIMEOUT.
- `canbridge_config.h:55`: FW_VERSION 32 -> 33, with a history note. `teensy_link/rpc_args.py:847` EXPECTED_BRIDGE_FW_VERSION -> 33 (pinned by tests/firmware/test_bridge_fw_version_xref.py:83).
- `protocol_config.h` is regenerated (the WaveCanId namespace). `README.md` and docs/teensy-udp-protocol.md gain the method.
- PROTOCOL_VERSION stays 10. Precedent: BB_CANCEL in ed384cb4 ("ADDITIVE ... can-bridge FW 30 — no PROTOCOL_VERSION bump: no existing message changes, and an FW <= 29 board answers the unknown id ... with ERR_UNKNOWN_METHOD"), and likewise HAND_MOVE_TO in 8df4e11a and the BB FW relay in 658c21a3. Every bump in the history at udp_protocol.h:12 grew or removed a struct.
- Idempotency: WAVE_CMD is idempotent. A re-dispatch of the same req_id resends an identical frame with the same cmd_seq, which the ESP treats as a keepalive. Do not add it to `NON_IDEMPOTENT_METHODS` (teensy_link/rpc.py:54). The 5 Hz stream still calls with `retries=0` and a 0.2 s timeout, because the next tick supersedes.
- Tests: a WAVE_CMD case in tests/firmware/native/test_rpc_dispatch.cpp (bad args, gate closed gives ERR_BUS_DOWN, happy path frame bytes), xlang parity via tests/firmware/test_udp_protocol_xlang.py, and tests/teensy_link/test_rpc_args.py for `encode_wave_cmd`.

Stale comments worth fixing in passing: telemetry.cpp:198 and
time_sync_master.cpp:31 still say the cone is "CAN2". The cone role is the
CAN3 peripheral (can_buses.cpp:41).

## 7. Jetson changes (JB/)

- `config/protocol_config.yaml` (can_ids block, lines 84-136): add
  `wave_camera: {CMD: 0x7E2, HEARTBEAT: 0x7E3, REC_EVENT: 0x7E4, INFO: 0x7E5, BLOCK_FIRST: 0x7E2, BLOCK_LAST: 0x7E7}`
  plus a top-level `wave_camera:` section (`heartbeat_timeout_ms: 500`,
  `intent_period_ms: 200`, `intent_timeout_ms: 3000`, `states:`,
  `event_kinds:`, `reasons:`).
- `config/generate_config.py`:
  - `generate_cpp` gets a `WaveCanId` namespace after CatchingConeCanId (lines 183-189), plus the `Wave`/`WaveState` namespaces.
  - `generate_python` gets `CAN_ID_WAVE_*` after the CC IDs (lines 384-385), plus `WAVE_*` scalars and IntEnums `WaveState`, `WaveEventKind` and `WaveReason`.
  - Copy the clapboard branch's diff shape exactly. Run `python config/generate_config.py`. It copies the headers to every firmware directory, including the bridge (lines 1117-1124).
- `ros_ws/src/jugglebot/jugglebot/can/wave_camera.py` (new; pure Python, imports only `jugglebot.protocol_config`):
  - dataclasses `WaveHeartbeat`, `WaveRecEvent` and `WaveInfo`, each with `from_can_frame(data)` that raises on DLC != 8;
  - `encode_cmd(desired, cmd_seq, flags, session_id) -> bytes`;
  - `reconstruct_wall_us_40(low40, host_now_us)`, the 40-bit twin of `reconstruct_fire_time_us`;
  - `event_wall_us(evt, t_bridge_us, host_now_us)`, which picks the synced or age path;
  - `SeqUnwrapper` for recording_seq.

  Add it to `can/__init__.py`'s module list.
- `teensy_bridge_node.py`:
  - `_on_cone_frame` (line 2559) gets `elif` branches for the three uplink IDs. They latch the heartbeat and info under `self._lock` with `time.monotonic()`, and queue events with `cf.t_bridge_us`.
  - A 10 Hz `_publish_wave_heartbeat` timer next to line 2227 copies `_publish_cone_heartbeat` (line 8763): connected comes from host-monotonic staleness against `WAVE_HEARTBEAT_TIMEOUT_MS`, and it publishes connected=False before the first frame.
  - A 5 Hz `_wave_intent_tick` that calls `_call_rpc(RpcMethod.WAVE_CMD, …, retries=0, timeout=0.2)` (line 6389) only while connected.
  - Two Trigger services, `camera/record_start` and `camera/record_stop`, on their own ReentrantCallbackGroup like `bb/cancel` (line 2186-2191). Each bumps cmd_seq, sends immediately, then waits on a condition variable for the matching event (s4.5), bounded at 2.0 s (500 ms min-state stretch + 33 ms tick + 150 ms clip window + margin).
  - On a start timeout or a REFUSED, it sets desired=0, so the robot is never recording without a confirmation.
  - A third Trigger service, `camera/clear_faults`, on the same group (s4.5 "Clearing faults").
  - `camera/record_event` is published from the RX queue by a 100 Hz drain like `_drain_cone_catch_events`.
- Messages in `ros_ws/src/jugglebot_interfaces/msg/`, listed in CMakeLists.txt next to line 42:
  - `WaveCameraHeartbeat.msg`: `bool connected, uint8 state, bool camera_on, bool uart_seen, bool recording, uint32 elapsed_ms, builtin_interfaces/Time start_stamp, uint32 recordings_this_session, bool time_synced, uint32 sync_age_ms, uint16 fw_version, uint32 uptime_s, uint8 fault_bits, bool console_owns, bool intent_fresh, uint16 offset_spread_us`.
  - `WaveRecordEvent.msg`: `std_msgs/Header header, uint8 kind (constants START=1 STOP=2 AUTO_STOP=3 REFUSED=4), uint8 reason, builtin_interfaces/Time event_time, uint64 event_time_us, bool time_synced, bool tick_stamped, uint32 recording_seq, uint8 cmd_seq`.
- tests/ros/conftest.py: add `WaveCameraHeartbeatMsg` and `WaveRecordEventMsg` dataclasses next to line 114, and register them in the `jugglebot_interfaces.msg` mock (line 1020-1030).
- Launch: add `/camera/heartbeat` and `/camera/record_event` to the rosbag list (JB/ros_ws/src/jugglebot/launch/jugglebot_launch.py, near line 649) and to `RECORDED_TOPICS` (tests/ros/test_launch_nodes.py:84).
- New tests:
  - tests/ros/test_wave_camera.py: decoder and encoder byte vectors, 40-bit wrap, unsynced age path, seq unwrap.
  - tests/ros/test_teensy_bridge_node_wave.py: heartbeat staleness, service success/refuse/timeout paths with a fake RPC and injected CONE_FRAMEs, the intent stream stopping when disconnected, desired=0 after a start timeout.
  - A shared JSON wire-vector fixture in tests/fixtures/ consumed by both repos (the clapboard precedent d7989b9e).

## 8. GUI changes (JB/ros_ws/gui/)

- `index.html`: `<div id="camera-panel"></div>` after `#state-minimap` (line 121), at the same z-order rung. Contents:
  - a `.cmd-btn` REC button that toggles between `REC` and `STOP`;
  - a dot (red pulsing while recording);
  - `mm:ss`;
  - a state line: `Camera: off | ESP: disconnected | unsynced | console owns`.
- `css/camera-panel.css` (new, linked after state-minimap.css):
  - `position:absolute; z-index:11; bottom: calc(var(--command-strip-h, 56px) + var(--gap) * 2); left: calc(var(--gap) + 210px + var(--gap));`, the same bottom as the collapsed minimap (state-minimap.css:36-44);
  - `background: var(--bg-overlay); backdrop-filter: blur(8px)`;
  - tokens only from css/theme.css (`--accent-red`, `--accent-green`, `--text-secondary`);
  - shift rule `#viewer-pane.minimap-expanded #camera-panel { left: calc(var(--minimap-width) + var(--gap)); }`, the `#connection-status` precedent at state-minimap.css:76;
  - no transitions on `left` (the snap rule, state-minimap.css:30-33).

  The class toggle itself already exists (state-minimap.js:1460).
- `js/camera-panel.js` (new) has two halves:
  - A pure core `cameraPanelView(hb, nowMs, lastHbMs, {connected, replay})`, which returns `{enabled, label, recording, elapsedText, stateText}`:
    - disabled unless connected && camera_on && !console_owns && !replay;
    - elapsed = `hb.elapsed_ms + (nowMs - lastHbMs)` while recording, clamped to +1 s past the last heartbeat;
    - text formatted as mm:ss (h:mm:ss past an hour);
    - all fields blanked to `--:--` when disconnected or when the heartbeat is older than 1 s.
  - A DOM `init(ros)` that subscribes to `camera/heartbeat` (`jugglebot_interfaces/msg/WaveCameraHeartbeat`) and `camera/record_event`, re-renders on a 250 ms `setInterval`, and on click calls `ros.callService('camera/record_start'|'camera/record_stop', 'std_srvs/srv/Trigger')`. The button stays disabled while the call is pending, and failures are shown in the state line.
  - `isReplayFenced()` is checked first in the click handler (replay/fence.js:24), as in commands.js:54. `callService` refuses in replay anyway (ros-bridge.js:380-383).
- `js/main.js`: import and init, and add `'camera/heartbeat'` and `'camera/record_event'` to `GUI_SUBSCRIBED_TOPICS` (line 1163), which tests/ros/test_rosbridge_websocket_lean.py:1005 pins.
- Tests: tests/ros/js/camera_panel_harness.js and tests/ros/test_gui_camera_panel.py run the core unmodified under node (pattern: tests/ros/test_gui_bb_calibration_status.py). The cases: enable matrix, interpolation, clamp, blanking, replay fence, and string tripwires for the minimap-expanded CSS rule and the subscribed-topics entries.

## 9. Docs and process in Jugglebot

- Branch `2026-10_wave-camera-can3` cut from **origin/skill-stack**. The local `skill-stack` checkout is 2 commits behind origin (8dbfb554 vs 666b006b), so pull first. The owner commits.
- `docs/adr/0014-wave-camera-on-cone-bus.md` in the Nygard format of 0013 (Status/Date/Deciders/Related, then Context, Decision, Consequences, Alternatives considered). Decision: the Wave ESP is a cone-role device with its own ID block, and liveness of recording intent is a 5 Hz stream with a 3 s ESP-side auto-stop. Alternatives: a stateful start/stop command, or a generic raw-frame RPC. Add it to docs/adr/index.md and the mkdocs.yml nav (lines 71-80). Landed as `docs/adr/0014-wave-camera-on-can3.md`.
- `plans/active/wave-camera-can3.md` with frontmatter `title/created/status: active`, plus a row in plans/active/INDEX.md in the same commit (tests/sim/test_plans_index.py enforces this). Landed as `plans/active/wave-camera-can3-integration.md`.
- Logbook entry `logbook/2026-10-DD-wave-camera-can3-integration.md`, `type: feature`, `subsystem: can`, `related_plan: wave-camera-can3.md`. Commits carry `Logbook-Entry: <slug>`. Landed as `logbook/2026-10-19-wave-camera-can3-bridge-and-node.md` (plus `2026-10-19-gui-camera-record-panel.md` and `2026-10-25-wave-camera-gpio-findings.md`).
- Order: edit the YAML, run generate_config.py and generate_udp_protocol.py, stage the regenerated files, then `./run_tests.sh` (and `--full` before the hardware sitting).
- On the Jetson: `colcon build --packages-select jugglebot_interfaces jugglebot`, bridge reflash `pio run -t upload` (Teensy_code_canbridge/BRINGUP.md Step 5), then a full relaunch of `ros2 launch jugglebot jugglebot_launch.py`, so that rosbridge loads the new message types.

## 10. Bring-up and verification, in order

| # | Test | Pass criterion |
|---|---|---|
| 1 | ESP alone, USB-CAN sniffer with 120 R at the sniffer | 0x7E3 at 10.0 ± 0.2 Hz, DLC 8, state IDLE, CAMERA_ON follows camera power; INFO pages 0-3 rotating every 250 ms, each page at 1 Hz; tick_period about 33334 µs |
| 2 | ESP on the robot drop, bridge powered, ROS up | `ros2 topic hz /camera/heartbeat` 10 Hz, connected=true; the bridge `[canhealth]` cone line shows rx and no drops; 0x7DD arrives (accepted_per_s about 100) |
| 3 | Sync stability, 10 min | TIME_SYNCED stays 1; sync_age <= 1 (100 ms units); offset_spread_us p95 < 200; track_err_us < 100; steps = 0 |
| 4 | `ros2 service call /camera/record_start std_srvs/srv/Trigger` | success within 2 s; START event with tick_stamped, synced, reason NONE; the camera clip counter +1; last_tick_to_clip_us 15-17 ms |
| 5 | record_stop | STOP event, the camera stops, elapsed matches the camera clip length within 0.1 s |
| 6 | Kill the node (or block the intent) while recording | AUTO_STOP INTENT_TIMEOUT 3.0-3.4 s after the last intent; fault bit3 set |
| 7 | Camera power-off while recording | AUTO_STOP TICK_LOST within 0.3 s; GPI2 released; no recording at power-up |
| 8 | GUI panel end to end | Button disabled with the ESP unplugged or the camera off; REC/STOP work; mm:ss ticks smoothly; shifts with the minimap expanded; inert in replay |
| 9 | 20-run latency, intent to start tick (node log time of the RPC send vs the START event_time_us) | all 20 confirmed; start minus RPC send in [0, 34 ms + 500 ms stretch] with spacing > 500 ms, spread about one tick period; no CLIP_MISSING |
| 10 | Console coexistence | `rec start` refused while CAN owns; `ctl console` then `lat 3` works and CAN intents are REFUSED CONSOLE_OWNS |

## 11. Open questions for the owner

All resolved by the owner on 2026-10-19; see the decisions log below.

### Decisions log (2026-10-19)

- (a) "Recordings this session" counts from the ROS2 launch, via session_id in WAVE_CMD, as designed.
- (b) If "Created new clip." does not arrive within the window, keep GPI2 low and report START with reason CLIP_MISSING, as designed.
- (c) No Schottky diode. A USB isolator is used instead when USB and CAN are connected together, and the DevKitC is fed 3.3 V at the 3V3 pin (s3).
- (d) The cone and the camera adapter are mutually exclusive on the CAN3 drop.
- (e) Bridge reflash accepted, FW_VERSION 32 -> 33.
- (f) Jugglebot branch `2026-10_wave-camera-can3` is cut from origin/skill-stack; the owner commits.
- Also confirmed: the bridge end of the CAN3 segment is terminated, and the ESP end gets 120 R (s3).

## 12. Phased implementation

| Phase | Repo | Work | Testable on Windows | Needs Jetson/robot |
|---|---|---|---|---|
| 0 | wave | `wave_frames.h`, `time_slave_math.h`, `record_fsm.h` + Unity tests in test/ | `pio test -e native` | no |
| 1 | wave | can_link, time_slave, record_ctrl, heartbeat, console `ctl`/`can`/`sync` | build; bench rows 1 and 10 with a USB-CAN sniffer | no |
| 2 | JB | YAML + both generators, bridge FW 33 (rpc.cpp case, send_cone_frame), rpc_args encoder, version xref | pytest firmware/teensy_link tests; native C++ tests where g++ is installed | flash on the Jetson |
| 3 | JB | wave_camera.py, msgs, bridge node services and timers, launch/rosbag, conftest | pytest with mocks (test_wave_camera, test_teensy_bridge_node_wave) | colcon, rows 2-7 |
| 4 | JB | GUI panel, CSS, main.js wiring | node harness tests | row 8 |
| 5 | both | 20-run latency, docs (ADR-0014, plan, logbook), findings doc update | `./run_tests.sh` | row 9 |
