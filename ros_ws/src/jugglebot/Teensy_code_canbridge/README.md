# Teensy 4.1 can-bridge firmware

The new dedicated CAN microcontroller from
[`plans/active/teensy-can-offload.md`](../../../../plans/active/teensy-can-offload.md).
It owns all leg-ODrive CAN traffic (offloaded from the Jetson), is the system
time-sync **master**, and talks to the Jetson over a point-to-point UDP link.

> **Status: hardware-free WIP.** Nothing here has run on a Teensy yet. See
> [`HANDOFF-teensy-can-offload-firmware-wip.md`](../../../../plans/active/HANDOFF-teensy-can-offload-firmware-wip.md)
> for phase status, autonomous decisions, and the hardware-validation checklist.

## Board & toolchain

| Item | Value |
|------|-------|
| Board | **Teensy 4.1** (IMXRT1062, 600 MHz Cortex-M7) |
| Core | Teensyduino **1.59+** on Arduino IDE 2.x (or `teensy` platform in PlatformIO) |
| CPU speed | 600 MHz |
| Optimize | "Faster" (`-O2`) or "Fastest" — the 500 Hz interp wants the cycles |
| USB type | Serial (debug console only; data path is Ethernet) |

## Libraries

Install via the Arduino Library Manager / PlatformIO `lib_deps`, then **pin the
versions you used** here once the build is green on the bench:

| Library | Purpose | Notes |
|---------|---------|-------|
| **FreeRTOS_TEENSY4** | RTOS | greiman fork — umbrella header `FreeRTOS_TEENSY4.h` (see `freertos_shim.h`). The tsandmann/freertos-teensy fork also works; adjust the shim. |
| **QNEthernet** | UDP/lwIP over the built-in MAC | ssilverman/QNEthernet; supersedes NativeEthernet. Needs the PJRC Ethernet kit (DP83825I PHY). |
| **FlexCAN_T4** | all three CAN buses | bundled with Teensyduino |

## Generated / shared headers (do not hand-edit)

These are copied/generated from `config/` by the codegen — regenerate, don't edit:

- `protocol_config.h`, `hardware_config.h` — `python config/generate_config.py`
- `udp_protocol.h` — `python config/generate_udp_protocol.py`

`canbridge_config.h` and `axis_state.h` are hand-authored firmware config.

## Pin map

Three subsystem-isolated CAN buses (ADR-0013). Pin directions are the FlexCAN_T4
silicon-fixed DEF mux — the library default the firmware actually runs (the
`CAN*_*_PIN` constants in `canbridge_config.h` are documentation only):

| Signal | Teensy 4.1 pin (TX / RX) | Notes |
|--------|----------------|-------|
| CAN1 TX / RX | 22 / 23 | Ball Butler bus (BB Teensy only) → TJA1051T/3 transceiver |
| CAN2 TX / RX | 1 / 0 | Catching cone bus (cone Teensy, often disconnected) → second transceiver; firmware tolerates cone-absent TX (no bus-off) |
| CAN3 TX / RX | 31 / 30 | Jugglebot core bus (6 leg ODrives + Hand ODrive + platform Teensy 4.0) → third transceiver |
| Ethernet | dedicated MAC pads | PJRC Ethernet kit 6-pin ribbon |
| LED | 13 | On-board; 1 Hz blink = scheduler alive |

120 Ω termination on each of the three CAN bus ends. CAN3 is the FD-capable
peripheral, run classical 1 Mbps today (the ODrive firmware is classical-CAN
only); a future CAN-FD upgrade is a config change, not a rewire.

> **Pin-direction note.** ADR-0013 and the parent plan list CAN2 and CAN3 TX/RX
> *reversed*; the FlexCAN_T4 silicon mux above (CAN2 TX 1 / RX 0, CAN3 TX 31 /
> RX 30) is authoritative — see the firmware-three-bus HANDOFF decision D1.

## Network

Static, point-to-point, **no DHCP / no mDNS**:

- Teensy `192.168.42.2` / `255.255.255.252` (`/30`), no gateway.
- Jetson `192.168.42.1` (see the parent plan's "Jetson network setup").
- UDP ports: **5005** stream (setpoint dn; telemetry/diag/profile/heartbeat up),
  **5006** RPC (request/response). Protocol: [`docs/teensy-udp-protocol.md`](../../../../docs/teensy-udp-protocol.md).

The PHY is held in reset until `Ethernet.begin()` runs (in `net_ethernet_begin()`),
so the Jetson adapter's link lights stay dark until this firmware boots — expected.

## Build & flash

**Arduino IDE:** open `Teensy_code_canbridge.ino`, select Teensy 4.1, set CPU
600 MHz / Optimize Faster / USB Serial, Upload.

**PlatformIO** (a `platformio.ini` like the cone firmware's can be added):

```ini
[env:teensy41]
platform = teensy
board = teensy41
framework = arduino
build_flags = -O2 -DUSB_SERIAL
lib_deps =
    ssilverman/QNEthernet
    https://github.com/greiman/FreeRTOS_TEENSY4
```

## Module layout

```
Teensy_code_canbridge.ino   FreeRTOS scaffold: Ethernet + tasks + scheduler
canbridge_config.h          pins, task table, control constants (ported 1:1)
axis_state.h / .cpp         per-axis state cache (6 legs + hand)
time_base.h / .cpp          micros64 + wall-clock (time-sync master rate owner)
net_ethernet.h / .cpp       QNEthernet static bring-up
udp_link.h / .cpp           framing (CRC-16), RX dispatch, TX, stats           [Phase 3]
udp_protocol.h              generated wire protocol (structs + framing)        [Phase 4]
odrive_protocol.h / .cpp    ODrive CAN encode/decode (port of odrive.py)       [Phase 5]
can_buses.h / .cpp          FlexCAN_T4 setup + RX dispatch + TX                [Phase 5]
time_sync_master.h / .cpp   0x7DD 100 Hz broadcast + time-of-day RPC           [Phase 5]
telemetry.h / .cpp          100 Hz motor-state + on-change diagnostics uplink  [Phase 6]
leg_interp.h / .cpp         500 Hz Hermite/Taylor ladder (port of motor_guard) [Phase 7]
fault_machine.h / .cpp      fault state machine + watchdog/deferred-stow       [Phase 8]
profiling.h / .cpp          per-task CPU, bus util, RTT, deadline-miss → PROFILE
```

## lwIP threading (hardware-validation item)

lwIP is not reentrant (QNEthernet builds it `NO_SYS=1` — no internal locking).
`udp_link.cpp` serialises **every** QNEthernet call with a recursive mutex
(`NetLock`): `Ethernet.loop()` (the lwIP pump, invoked from `udp_link_service()`),
socket bind, and all socket RX (`parsePacket`/`read`) and TX (`beginPacket`/
`write`/`endPacket`). RX runs in the `net` task; TX is callable from any task.
The pump is taken in its own short `NetLock` scope, separate from the drain, so
the lock releases between pump and drain and a pending prio-3 TX can interleave;
decode + dispatch always run **unlocked** (the `NetLock` scopes wrap QNEthernet
calls only, never the handler chain / `interp_on_setpoint`'s IRQ-off publish).

> **History:** before Tier-2 item 15, `Ethernet.loop()` (and `linkState()`) ran
> *outside* the lock — a real concurrency hazard, since the prio-4 `net` task
> could pump lwIP while a prio-3 task was mid-`send_to()` (holding `NetLock`),
> both walking the same pbuf pool / UDP PCB. That pump call is now under the lock.

RX is bounded per wake by `UDP_RX_DRAIN_BUDGET` (8 frames per socket, per
`udp_link_service()` call — mirrors `can_buses.cpp`'s `CAN_RX_DRAIN_BUDGET`) so a
UDP flood cannot spin the `net` task unboundedly and starve the fault machine /
homing / diag tasks. A budget-bound drain with a frame still queued increments
`UdpStats::drain_cap_hits` (surfaced on the `[diag]` serial line as `drain_cap=`);
the backlog drains on the next tick, it is not dropped.

The robust alternative — a single net task owning all socket I/O with other
tasks enqueuing TX frames — is noted in the handoff under "Needs hardware
validation". Confirm the mutex + drain-budget approach holds under load on the
bench before relying on it (the concurrency behaviour itself is an
on-hardware-replay gap — the native harness validates lock *coverage* and the
drain *budget*, not true preemptive reentrancy).

## What to verify on the bench (per phase)

See the handoff doc's "Needs hardware validation" section. Phase 2/3 bring-up
checklist: LED blinks at 1 Hz, `ping 192.168.42.2` succeeds, the UDP echo /
heartbeat round-trips, and the serial console shows clean FreeRTOS scheduling.
