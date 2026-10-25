# Wave Camera ESP32 firmware (GPIO explorer + CAN adapter)

Part of the Wave Camera sub-project of Jugglebot (ADR
[0014](../../../../docs/adr/0014-wave-camera-on-can3.md), plan
[wave-camera-can3-integration](../../../../plans/active/wave-camera-can3-integration.md)).
This folder is a standalone PlatformIO project; it is built and flashed from
a PC over USB, never from the Jetson. Bench captures live in `bench_logs/`.

ESP32 firmware for characterising the Freefly Wave's (fw 1.4.1) 6-pin GPIO
connector from an ESP32-WROOM-32D DevKitC. It timestamps every edge on the
camera pins, bridges a UART to whichever pin turns out to be the camera's
serial line, drives GPI2 to start and stop recording, and measures the time
from GPI2 to `Created new clip.`. The CAN phase adds start/stop over CAN
from the robot (see [CAN](#can) below). [wave_camera_gpio_explorer.md](../../../docs/wave_camera_gpio_explorer.md) has the
explorer design, [wave_camera_can_adapter.md](../../../docs/wave_camera_can_adapter.md) the CAN design, and
[wave_camera_findings.md](../../../docs/wave_camera_findings.md) the results.

## Wiring

| Wave pin | Name            | ESP32 pin | Host-side parts                     | ESP mode                         |
|----------|-----------------|-----------|-------------------------------------|----------------------------------|
| 1        | GND             | GND       | direct                              |                                  |
| 2        | VIN             | 3V3       | direct (camera side draws < 20 mA)  |                                  |
| 3        | GPO1 / UART out | GPIO32    | 1 kΩ series, 10 kΩ pull-up to 3V3   | input, ISR on CHANGE, UART2 RX   |
| 4        | GPO2 (30 Hz)    | GPIO27    | 1 kΩ series, 10 kΩ pull-up to 3V3   | input, ISR on CHANGE             |
| 5        | GPI1            | (none)    | not connected                       |                                  |
| 6        | GPI2 / RS       | GPIO26    | direct                              | open-drain output, ISR on CHANGE |

Pins changed on 2026-10-23: GPI2 moved from GPIO25 to GPIO26, Pin 3 from
GPIO26 to GPIO32, and Pin 5 (found inert, see [wave_camera_findings.md](../../../docs/wave_camera_findings.md)) was dropped.
GPIO25 is unused. VIN comes from 3V3, never 5 V. The internal pull-ups on
26/27/32 are also enabled, so a bare board reads clean highs.

With Pin 5 gone there is no transmit path to the camera: `uart tx`, `>`,
`>>`, every `qx` command (except `qx stream off`) and `tx hold` print
`ERR no tx pin wired (Pin 5 not connected)`. The code is kept, behind
`UART_TX_WIRED` in `include/config.h`.

## Build, flash, monitor

```bash
pio run -e esp32dev                 # build
pio run -e esp32dev -t upload       # flash (COM5; edit platformio.ini if the port changes)
pio device monitor -e esp32dev      # 115200 baud
pio test -e native                  # host unit tests
```

On this machine the Git Bash PATH puts Git's `libwinpthread` ahead of
MinGW's, which makes `gcc` fail without printing anything. Run the native
tests with `PATH="/c/ProgramData/mingw64/mingw64/bin:$PATH" pio test -e native`.

## tools/wave_console.py

The script sends commands and captures the output. Each line gets a prefix
with the host time in ISO format, to the millisecond. Run it with the
PlatformIO python, which already has pyserial:

```bash
PY=C:/Users/harri/.platformio/penv/Scripts/python.exe
$PY tools/wave_console.py --cmd help --cmd status
$PY tools/wave_console.py --no-reset --gap 1 --cmd "rec mode edge" --cmd "rec toggle" --out bench_logs/rec.log
$PY tools/wave_console.py --no-reset --listen 20
```

| Option        | Meaning                                                                        |
|---------------|--------------------------------------------------------------------------------|
| `--port`      | default COM5; if it is missing, uses the first port with hwid 10C4:EA60         |
| `--baud`      | default 115200                                                                 |
| `--cmd TEXT`  | can be repeated; commands are sent in order, `--gap` seconds apart (default 0.5) |
| `--wait S`    | how long to keep capturing after the last command (default 3)                  |
| `--listen N`  | capture for N seconds (after any `--cmd`)                                      |
| `--out PATH`  | also append the capture to a file                                              |
| `--no-reset`  | open with DTR and RTS deasserted so the ESP32 is not reset (state survives)     |

Without `--no-reset` the script pulses EN, so every run starts from a fresh boot.

## Console

Every line has the form `<t_us> <TAG> <fields>`, where t_us is
`esp_timer_get_time()`. On EDGE lines t_us is the stamp taken in the ISR. On
all other lines it is the time the line was printed. The tags are EDGE,
UART, UARTHEX, PROBE, PULSE, REC, LAT, QX, INFO, WARN and ERR.

A camera UART line looks like
`<t> UART t_first_byte_us=… src=edge|read t_eol_us=… end=eol bytes=N | <text>`.
The text is printed verbatim with CR/LF stripped, and non-printable bytes
appear as `\xNN`. With `src=edge`, t_first_byte comes from the start bit's
falling edge as seen by the ISR. With `src=read`, it is the time the byte
was read. A line is flushed as partial (`end=partial_overflow` or
`end=partial_timeout`) when it passes 256 bytes or goes 200 ms without a
terminator.

| Command                        | Description                                                              |
|--------------------------------|--------------------------------------------------------------------------|
| `help`                         | list commands                                                            |
| `status`                       | pin levels, roles, UART assignment, rec state, ring/overflow counters    |
| `probe [seconds]`              | edge stats on all pins (default 5 s), then classify PULSE/UART/STATIC    |
| `edges on\|off`                 | stream every edge (EDGE pin, level, dt since the previous edge on that pin) |
| `uart rx [32\|27]`              | route UART2 RX to Pin 3 / Pin 4 (default: probe's UART pin, else 32); 26 refused (GPI2) |
| `uart tx`                      | refused: no tx pin wired (Pin 5 not connected)                           |
| `uart hex on\|off`              | also print raw bytes                                                     |
| `tx hold low\|release`          | refused: no tx pin wired                                                 |
| `uart off`                     | detach UART2                                                             |
| `> <text>` / `>>`              | refused: no tx pin wired                                                 |
| `pulse`                        | 30 Hz stats: period, jitter, width, polarity                             |
| `rec mode edge\|level`          | must be set before any `rec` command or `lat`                            |
| `rec toggle`                   | edge mode: 20 ms low pulse                                               |
| `rec start\|stop`               | level mode: hold low / release                                           |
| `rec assume idle\|recording`    | resync the record-state guess (`lat` needs `idle`)                       |
| `lat [n] [rec_s] [gap_s]`      | latency test (default 10 runs, 2 s, 3 s); `lat abort` stops it           |
| `qx idle\|rec\|sel [v]\|jog <n>`  | refused: no tx pin wired                                                 |
| `qx stream <hz>\|off`           | refused (`off` always allowed)                                           |
| `qx raw <24 hex bytes>`        | refused: no tx pin wired                                                 |
| `arm` / `disarm`               | arm is needed for `sim intent`; transmit commands stay refused           |

## CAN

The board joins the Jugglebot cone-role bus (1 Mbit/s classic, 11-bit IDs,
DLC 8) through an SN65HVD230 and drives GPI2 for the robot. The frame
layouts, state machine and time sync are in [wave_camera_can_adapter.md](../../../docs/wave_camera_can_adapter.md)
sections 4-5; constants with units are in `include/config.h`.

| Signal | ESP32 pin | Part |
|---|---|---|
| CAN TX | GPIO13 | SN65HVD230 D |
| CAN RX | GPIO34 (input-only; the transceiver drives it) | SN65HVD230 R |
| Transceiver Vcc | 3V3 | 100 nF at the pin, Rs to GND |
| Termination | | 120 Ω CANH-CANL at the ESP end |

| ID | Frame | Direction, rate |
|---|---|---|
| 0x7DD | TIME_SYNC `<II` sec/usec | bridge to all, 100 Hz |
| 0x7E2 | WAVE_CMD desired/seq/flags/session | Jetson to ESP, 5 Hz |
| 0x7E3 | WAVE_HEARTBEAT state/status/faults/elapsed/sync age/recordings/seq | ESP, 10 Hz from boot |
| 0x7E4 | WAVE_REC_EVENT 40-bit time, kind/reason, seqs | ESP, per event, up to 3 attempts 5 ms apart |
| 0x7E5 | WAVE_INFO pages 0-3 | ESP, one page per 250 ms (each page 1 Hz) |

At boot the firmware sets rec mode level, bridges UART RX on GPIO32
(Pin 3) and gives GPI2 to CAN. `rec` and `lat` then print `ERR can owns gpi2; use ctl
console`; `ctl console` (only from IDLE) hands GPI2 to the console and
`ctl can` hands it back. Frames are sent single-shot: with no other node
on the bus every frame fails and is counted (`can`), and a frame that
cannot start at all (bus never idle) is aborted after 20 ms, so nothing
stale goes out later.

| Command | Description |
|---|---|
| `can` | rec state, owner, intent, tick stats, bus counters, 0x7DD sync stats, the heartbeat bytes now |
| `ctl console\|can` | hand GPI2 to the console (IDLE only) or back to CAN |
| `events on\|off` | print every CAN frame sent or received as hex (`CAN` lines) |
| `sim intent on\|off` | bench only, needs `arm`: inject a local 5 Hz desired=1 intent; `off` stops it, so a recording auto-stops 3 s later |

Bring-up (wave_camera_can_adapter.md s10):

1. ESP alone on a USB-CAN sniffer with 120 Ω: 0x7E3 at 10 Hz, state IDLE,
   CAMERA_ON follows camera power, INFO pages 0-3 at 1 Hz each.
2. Bench state machine without a bridge: `arm`, `events on`,
   `sim intent on`; expect STARTING then START (or START reason
   CLIP_MISSING), RECORDING; `sim intent off`; AUTO_STOP INTENT_TIMEOUT
   3.0 s later and GPI2 released.
3. On the robot drop with the bridge: `can` shows rx rising,
   `accepted_per_s` about 100, `synced=1`; then the ROS services.
4. Console coexistence: `rec start` refused; `ctl console`, `lat 3`
   works; `ctl can`.

## Safety guards (compiled in, no runtime override)

- GPIO26 (GPI2) is the only camera-side output, and it is open-drain.
  GPIO32 and GPIO27 are inputs everywhere; no pad accepts UART TX. GPIO13
  (CAN TX) is driven only by the TWAI peripheral into the transceiver.
  `src/pin_io.cpp` is the only code that sets pin direction. `PinDrive`
  has no push-pull mode. Every other file has `pinMode`,
  `gpio_set_direction` and `gpio_config` poisoned, so code that tries to
  set a direction anywhere else will not compile.
- `uart tx` is refused (no tx pin wired). If a TX pad is ever wired again,
  it is also refused on a pin that `probe` classified as PULSE or UART.
- The board boots disarmed. `>`, `>>` and `qx` print
  `ERR no tx pin wired (Pin 5 not connected)` before the arm check.
- Even when armed, `>` refuses any line that contains (case-insensitive,
  substring) format, erase, delete, update, reset, factory, wipe, rm, mkfs,
  flash or upgrade, and prints `ERR blocked word`.
- GPI2 timing: in edge mode the pulse is 20 ms low and needs ≥ 500 ms high
  since the previous rising edge. In level mode each state lasts ≥ 500 ms.
  A request that comes too early is stretched and logged as WARN, never
  dropped.
- `lat` times out after 5 s per run, aborts, and sets the rec state to
  unknown.
- WiFi and Bluetooth are never started.
