---
title: Can-bridge (CAN Hub) hardware bringup
type: feature
date: 2026-06-06
status: in-progress
phase: teensy-can-offload
related_plan: plans/archived/HANDOFF-firmware-three-bus-WIP.md
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/platformio.ini
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - logbook/2026-06-06-can-hub-bringup.md
  - logbook/INDEX.md
commits:
  - bdb5710
  - 06cf646
  - f772ec2
subsystem:
  - can
  - firmware
tags:
  - feature
  - hardware
  - firmware
  - bringup
  - can
---

# Can-bridge (CAN Hub) hardware bringup

## Summary

First on-hardware bringup of the can-bridge Teensy 4.1 (the "CAN Hub") — the board
that offloads all CAN responsibility from the Jetson and fans **three
subsystem-isolated FlexCAN buses** (Ball Butler, catching cone, Jugglebot core).
All three buses were brought up live: all 7 Jugglebot ODrives (6 legs + hand)
decode heartbeats cleanly, the cone-absent gating holds without bus-off, Ball
Butler cold-joins cleanly, and the bounded RX drain (finding #10) was confirmed
under real ODrive traffic at IDLE telemetry rates (full CLOSED_LOOP-load
validation still pending). This entry is the durable bringup reference — wiring,
how to flash and read the serial diagnostics, the firmware gotchas that cost time,
and what a healthy bus looks like.

## Bus wiring (ADR-0013)

One FlexCAN_T4 peripheral per subsystem. The wired pins **are** the FlexCAN_T4
library defaults, so `begin()` muxes them with no `setRX()`/`setTX()` call:

| Bus  | Subsystem                                            | TX pin | RX pin | Peripheral base |
|------|------------------------------------------------------|--------|--------|-----------------|
| CAN1 | Ball Butler                                          | 22     | 23     | `0x401D0000`    |
| CAN2 | Catching cone (often absent)                         | 1      | 0      | `0x401D4000`    |
| CAN3 | Jugglebot core (6 legs + hand + platform Teensy 4.0) | 31     | 30     | `0x401D8000`    |

- **1 Mbps**, all nodes (`CanBus::BAUD_RATE` / `CAN_BITRATE`). 120 Ω termination at
  each bus end.
- Transceivers: TJA1051T/3 per bus. A powered, idle bus reads ~3.3 V (recessive)
  on the Teensy RX pin; confirm both VCC and the `/3` logic rail (VIO) are supplied.
- CAN3 also carries the platform Teensy 4.0's traffic-report broadcast (`0x7DF`)
  alongside the ODrive frames.

## Flashing (button-free)

The Jetson can't run PlatformIO's bundled `teensy_reboot` (built against glibc 2.34
vs the Jetson's 2.31), so `platformio.ini` calls `teensy_loader_cli` directly. The
`upload_command` uses `-w -s`: **`-s` soft-reboots the running firmware into the
bootloader over USB — no physical button press** — and `-w` is the manual fallback
if the firmware ever fails to enumerate.

```
pio run -t upload     # build + button-free flash
pio device monitor    # 115200; CLOSE it before flashing (it holds /dev/ttyACM0)
```

Needs the Teensy enumerated as a Teensyduino Serial device (`lsusb` shows
`16c0:0483`). If a flash ever produces firmware that hangs before USB init, only the
physical button recovers it.

## Reading the serial diagnostics

`task_diag` prints once per second:

- `[diag] link fault rx tx crc_err seq_gaps synced heap` — overall status. **`rx`/`tx`
  are the Jetson UDP link, NOT CAN.** `fault`: 5 = ODRIVE_FATAL, 6 = CAN_BUS_DOWN.
  `synced` = time-sync locked (needs the Jetson answering `TIME_OF_DAY`).
- `[canhealth] <bus> sync hwm capHit err flags rec tec flt` — per-bus health.
  **`sync`** is the live `ESR1.SYNCH` bit (1 = controller locked onto the bus *right
  now*). `hwm/capHit/err/rec/tec` are sticky high-water marks. The *real* error signal
  is `flags`/`rec`/`tec`/`flt`; `err` just counts ESR1 state-changes (climbs benignly
  on a busy bus). `capHit` must stay 0 (the rxBuffer-overflow precursor).
- `[canhealth] decode_drops jugglebot: short bad_axis` — `bad_axis` counts CAN3 frames
  whose node-id ≥ 7 (not a leg/hand ODrive), e.g. the platform Teensy's `0x7DF` —
  benign noise.
- `[axes] fresh=N/7  i:s<state>/<age><mark>` — per-ODrive heartbeat. `state` 1 = IDLE,
  8 = CLOSED_LOOP; marks: `?` = never-seen, `!` = stale (>2 s), `*` = active
  error/disarm, ` ` = ok.

**A healthy, alive bus reads:** `sync=1 ... capHit=0 flags=0x00 rec=0 tec=0 flt=active`,
with `[axes]` filling to `fresh=7/7` at small ages.

## Firmware gotchas (read these before debugging RX)

1. **FreeRTOS masks the FlexCAN ISR during `setup()`.** The tsandmann freertos-teensy
   port holds `BASEPRI = configMAX_SYSCALL_INTERRUPT_PRIORITY (32)` until the scheduler
   starts. The Teensy core leaves peripheral IRQs at priority 128 (≥ 32), so **any
   interrupt-driven RX is dead in `setup()`** — a frame lands in the FIFO
   (`IFLAG1.BUF5I` set) but the ISR never runs. Do NOT run interrupt-dependent
   self-tests in `setup()`; they false-fail. `BASEPRI` drops to 0 once the first task
   runs.
2. **The core's `FLEXCAN3_*` register macros don't compile.** `imxrt.h` defines
   `IMXRT_FLEXCAN3` via `IMXRT_FLEXCAN3_ADDRESS0` (a stray trailing `0` — the symbol
   doesn't exist). `FLEXCAN1_*`/`FLEXCAN2_*` are fine. Read CAN3 registers from the base
   address directly (`0x401D8000` + offset; e.g. ESR1 at +0x20).
3. **`ESR1.SYNCH` (bit 18) is the fastest "is the bus electrically alive" check** —
   surfaced as `sync=` in `[canhealth]`. It flips 0→1 the instant a powered
   bus/transceiver appears, *before any frame decodes*. `sync=0` with zero error
   counters ⇒ the controller sees no valid recessive idle (no/unpowered transceiver, or
   nothing on the wire). A *bitrate/signal* fault instead shows form/stuff/CRC errors
   with `rec` climbing — a different signature entirely.
4. **Never transmit un-ACKed frames to "probe" a bus.** A TX with no ACK retransmits
   aggressively → TEC climbs to bus-off → the controller emits error frames that corrupt
   *other* nodes' traffic. RX-only diagnostics (`sync`/`hwm`) are safe; an un-ACKed TX
   probe manufactures the very garbled-bus symptom it's meant to detect.
5. **`events()` only drains the ISR-filled software `rxBuffer`**, and `readFIFO()`/
   `read()` refuse to poll while the FIFO interrupt is enabled. RX therefore depends
   entirely on the ISR firing — see #1.

## Validated on hardware

- All three buses `sync=1`; all 7 ODrives (legs 0–5 + hand) decode heartbeats fresh.
- **Finding #10 (RX drain):** under real ODrive traffic, `hwm ≈ 9` with `capHit=0` — the
  bounded drain (`CAN_RX_DRAIN_BUDGET = 32`) holds with wide margin at IDLE telemetry
  rates. Full CLOSED_LOOP-load validation is still pending (needs the ODrives armed).
- **Resilience:** CAN3 auto-restores after an unplug/replug (FlexCAN auto bus-off
  recovery; `fault=6` clears when leg heartbeats return). CAN1 cold-joins cleanly when
  Ball Butler powers on (`hwm` climbs, no bus-off). CAN2 cone gating holds `sync=1`,
  `tec=0` while the cone is absent — no bus-off.

## Discussion

The observability layer (`[canhealth]` + `[axes]`, now with live `sync`) was the
load-bearing tool during bringup: it turns "the bus is dead" into a specific failure
*class* at a glance. The single most useful discipline for the next bringup is to
**diagnose from the controller outward**:

1. `sync` first — electrical layer (is the controller even seeing a bus?);
2. then `flags`/`rec`/`tec`/`flt` — protocol/signal layer (bitrate, noise, termination);
3. then `decode_drops`/`[axes]` — application layer (which node, which frame).

Two diagnostic traps cost the most time and both produce *false* signals that send you
chasing the wrong layer: a setup()-time interrupt self-test (gotcha #1, always
false-fails) and an active un-ACKed TX probe (gotcha #4, manufactures a garbled bus).
Keep bringup diagnostics RX-only and trust `sync` as the first discriminator.

## Follow-ups

- Promote per-axis state + per-bus health onto the UDP uplink (currently USB-Serial
  only; the wire format is owned by a parallel session).
- Teach `decode_into_cache` to skip the shared broadcast IDs (`0x7DD`/`0x7DF`) so
  `bad_axis` becomes a true garbled-frame alarm rather than counting the platform
  Teensy's reports.
- Full-load RX-drain validation once the ODrives are armed into CLOSED_LOOP, behind a
  no-erroneous-command safety plan (audit of all CAN3 TX paths + the output gate).
