# Wave Camera: GPIO explorer firmware design

> **Wave Camera sub-project.** Jugglebot starts and stops recording on a
> Freefly Wave high-speed camera from ROS 2 through an ESP32 on the cone-role
> CAN bus (CAN3), and stamps each clip start on the robot wall clock. The ESP32
> firmware lives in [`ros_ws/src/jugglebot/WaveCamera_ESP_code/`](../src/jugglebot/WaveCamera_ESP_code/README.md)
> (a standalone PlatformIO project, built and flashed from a PC, never from the
> Jetson). Decision: [ADR-0014](../../docs/adr/0014-wave-camera-on-can3.md); plan:
> [wave-camera-can3-integration.md](../../plans/archived/wave-camera-can3-integration.md). Sibling docs:
> [GPIO explorer design](wave_camera_gpio_explorer.md),
> [CAN adapter design](wave_camera_can_adapter.md),
> [measured findings](wave_camera_findings.md).
> This document is the bench-phase (no CAN) explorer design, moved here from the
> former standalone adapter repo on 2026-10-25; bare paths such as `src/` and
> `include/` are relative to the firmware folder.


Status: APPROVED by owner 2026-10-19 (wiring as in section 1; USB console 115200; latency default 10 runs, 2 s rec, 3 s gap). Bench phase completed 2026-10-19; results in [wave_camera_findings.md](wave_camera_findings.md).

**Pin change 2026-10-23 (owner-approved):** Wave Pin 5 (GPI1) was found
inert and dropped, GPI2 moved from GPIO25 to GPIO26 and Pin 3 from GPIO26
to GPIO32; GPIO25 is unused. Section 1 shows the current wiring. With no
Pin 5 there is no transmit path to the camera, so `uart tx`, `>`, `>>`,
`qx` and `tx hold` are refused (`ERR no tx pin wired (Pin 5 not
connected)`); the rest of this document describes the 2026-10-19 explorer
as designed, with pin numbers updated. [wave_camera_findings.md](wave_camera_findings.md) measurements were taken
with the old mapping.

Goal of this phase: characterise the Freefly Wave (fw 1.4.1) 6-pin GPIO
connector from an ESP32-WROOM-32D DevKitC, with no CAN. Everything measured
goes into `ros_ws/docs/wave_camera_findings.md`.

## 1. Wiring (host side)

| Wave pin | Name            | ESP32 pin | Host-side parts                     | ESP mode                         |
|----------|-----------------|-----------|-------------------------------------|----------------------------------|
| 1        | GND             | GND       | direct                              |                                  |
| 2        | VIN             | 3V3       | direct (camera side draws < 20 mA)  |                                  |
| 3        | GPO1 / UART out | GPIO32    | 1 kΩ series, 10 kΩ pull-up to 3V3   | input, ISR on CHANGE, UART2 RX   |
| 4        | GPO2 (30 Hz)    | GPIO27    | 1 kΩ series, 10 kΩ pull-up to 3V3   | input, ISR on CHANGE             |
| 5        | GPI1            | (none)    | not connected (dropped 2026-10-23)  |                                  |
| 6        | GPI2 / RS       | GPIO26    | direct                              | open-drain output, ISR on CHANGE |

Why these choices hold up:

- VIN must come from 3V3, not 5 V. The camera pulls GPI2 (and presumably
  GPI1) up to VIN, and the ESP32 is not 5 V tolerant.
- GPIO26/27/32 are not strapping pins, all have internal pull-ups and
  interrupts, and none is input-only. GPIO26 is a DAC pin and GPIO32 is
  an XTAL32K pad; both are ordinary GPIO when unused for those functions.
- The 1 kΩ series resistors cap any fault current at about 3.3 mA.
- The only pin the ESP drives towards the camera (GPIO26, GPI2; originally
  also the Pin 5 Rx candidate) is driven **open-drain**, so the ESP never sources
  3.3 V into the camera. The high level comes from the pull-ups. This is a
  software choice, not a wiring change.
- During ESP reset and flashing every GPIO is a floating input, so the
  camera cannot be triggered by a reboot.
- Risk to note (2026-10-19): if GPI1 is an opto-LED input with its own
  series resistor, the extra 1 kΩ may not pull it low enough. Pin 5 showed
  no response ([wave_camera_findings.md](wave_camera_findings.md)) and was dropped on 2026-10-23.

## 2. Firmware layout (PlatformIO, Arduino, board `esp32dev`)

```
platformio.ini          monitor 115200, upload 921600, CORE_DEBUG_LEVEL=1, no WiFi/BT
include/config.h        pins, timing constants (units in names), guards
src/main.cpp            setup/loop: console + event drain
src/edge_capture.*      ISRs on 26/32/27 -> lock-free ring of {pin, level, t_us}
src/pin_probe.*         per-pin stats + classifier (uart / periodic / static)
src/pulse_stats.*       30 Hz line: period, jitter, width, polarity, phase of events
src/uart_link.*         UART2 on GPIO-matrix-selected pins, line timestamping, hex mode
src/gpi2_drive.*        edge/level drive with min low/high enforcement
src/latency_test.*      N-run GPI2 edge -> "Created new clip." timing
src/qx_protocol.*       24-byte QX10001 packet builder + sender
src/console.*           line parser, help, output formatting
tools/wave_console.py   pyserial helper: send a command, capture output to file
bench_logs/             tools/wave_console.py captures cited by the findings doc
```

Timestamps are `esp_timer_get_time()` (64-bit µs since boot), taken inside
the ISR. ISRs are `IRAM_ATTR`, do no printing, and only push to the ring.
Printing happens in `loop()` so a burst of UART edges (about 200 edges in
1.5 ms for one "Created new clip." line) cannot be dropped by a slow console.

Output lines are machine-parsable: `<t_us> <TAG> <fields...>`.

## 3. Console commands

```
help                      list commands
status                    pin levels, pin roles, UART assignment, rec state guess
probe [seconds]           edge stats on all pins for a window, then classify:
                          periodic ~33.3 ms -> PULSE, bursty with ~8.7 us min
                          pulse -> UART, no edges -> STATIC_HIGH / STATIC_LOW
edges on|off              stream every edge event live
uart rx <pin>             route UART2 RX to GPIO32|27 (default: the pin probe
                          classified as UART, else 32); 26 (GPI2) refused
uart tx <pin>             refused since 2026-10-23: no tx pin wired (Pin 5).
                          Originally: UART2 TX (open-drain) on the Rx
                          candidate, refused on a camera output.
uart hex on|off           also print raw bytes
> <text>                  send text + CRLF to the camera (needs `arm`, see s5)
>>                        send an empty line
pulse                     30 Hz stats over the last N periods
rec mode edge|level       tell the firmware which camera GPI mode is set
rec toggle                edge mode: 20 ms low pulse; enforces >= 500 ms high
rec start|stop            level mode: hold low / release; enforces >= 500 ms
rec assume idle|recording resync the state guess
lat <n> [rec_s] [gap_s]   latency test, n runs (default 10, 2 s rec, 3 s gap)
qx idle|rec|sel|jog <n>   send QX10001 packets (needs `arm`)
qx stream <hz>|off        continuous idle packets
qx raw <24 hex bytes>     send an arbitrary packet (checksum checked, not fixed)
arm | disarm              enable/disable anything that transmits to the camera
```

Latency test sequence per run: `rec toggle` (start) -> stamp the rising edge
of GPIO26 (GPI2; GPIO25 before 2026-10-23) as seen by its own ISR -> wait
for a UART line matching `Created new clip.` -> record t_first_byte - t_edge and t_eol - t_edge, and
the phase relative to the last 30 Hz pulse -> wait rec_s -> `rec toggle`
(stop) -> wait gap_s. Timeout 5 s per run; a timeout flags the state as
unknown and aborts. Report min / mean / max / stdev. Each run creates one
clip on the camera.

## 4. Pulse identification logic

- PULSE: >= 20 edges in the window, period CV < 5 %, period within 30-36 ms.
- UART: idle high, min pulse width in 6-12 us, edges arrive in bursts.
  Only visible when the camera starts a recording, so `probe` reports
  "idle high, no traffic yet" until a start happens.
- STATIC: no edges; report the level.
Roles are remembered until reboot or `probe` is rerun.

## 5. Safety guards (compiled in, not configurable at runtime)

- Only GPIO26 (GPI2) is ever an output, and only open-drain. GPIO32/27
  are always plain inputs (before 2026-10-23: GPIO25 plus the `uart tx`
  pin).
- `uart tx` is refused on a pin whose probe role is PULSE or UART.
- `>` and `qx` do nothing until `arm` is issued; `disarm` on boot.
- `>` rejects (case-insensitive) any line containing: format, erase, delete,
  update, reset, factory, wipe, rm, mkfs, flash, upgrade. There is no
  override.
- Edge-mode pulses: low >= 20 ms, high >= 500 ms since the previous pulse.
  Level mode: both states >= 500 ms. Shorter requests are stretched, not
  dropped, and the stretch is logged.

## 6. Out of scope for this phase

CAN (TWAI), wall-clock sync to 0x7DD, Jugglebot message IDs. Notes for later:
the clapboard contract in `Electronic-Clapboard/docs/protocol.md` section 8
and Jugglebot `config/protocol_config.yaml` define 0x7DD TIME_SYNC (100 Hz,
`<II` sec/usec); 0x7E2-0x7E7 are unallocated.
