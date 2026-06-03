# ADR-0003: Teensy 4.1 (not 4.0) for the can-bridge role

- **Status**: Accepted
- **Date**: 2026-06-02 (captured); decision made 2026-05-28
- **Deciders**: Harrison + Claude
- **Related**: [ADR-0002](0002-dedicated-second-teensy.md), [ADR-0005](0005-ethernet-over-usb-serial.md), [parent plan](../../plans/active/teensy-can-offload.md)

## Context

Given the [ADR-0002](0002-dedicated-second-teensy.md) decision to add a
dedicated Teensy for the can-bridge role, the next question was *which*
Teensy variant. Teensy 4.0 and 4.1 share the same IMXRT1062 chip (Cortex-M7
at 600 MHz, 1 MB RAM, identical CAN peripherals), so on raw compute they're
interchangeable.

The differentiating factor in this plan is the **Jetson↔Teensy link** —
which, per [ADR-0005](0005-ethernet-over-usb-serial.md), is Ethernet. Teensy
4.0 and 4.1 differ sharply in how they expose Ethernet:

- **Teensy 4.0**: The IMXRT1062 Ethernet MAC is on-chip, but the pins are not
  broken out. Adding Ethernet requires an external SPI Ethernet controller
  (e.g. W5500) — slower, more code, more board area, and a different lwIP
  configuration.
- **Teensy 4.1**: The MAC pins are broken out to solderable pads on the
  underside of the board. PJRC sells a small magjack-with-PHY kit
  (DP83825I) that solders directly to those pads; lwIP via QNEthernet drives
  it natively.

## Decision

Use **Teensy 4.1** for the can-bridge role.

Cost difference is ~$12 (4.1 is ~$32 vs ~$20 for 4.0). The Ethernet path
difference dominates any cost consideration for a device the project is
building on for years.

## Consequences

**Positive:**

- Native Ethernet via PJRC's Ethernet kit — soldered magjack, no external
  PHY chip wiring, no SPI overhead, no extra firmware library.
- 7.75 MB flash vs 1.94 MB — comfortable headroom for FreeRTOS + lwIP +
  ODrive protocol + fault state machine, with room for future growth
  (firmware tree currently ~205 KB code).
- microSD slot — optional local high-rate logging of CAN traffic and/or
  internal diagnostics, valuable for failure post-mortems.
- VBAT pin for battery-backed RTC — optional CR2032 keeps wall-clock across
  power-off, useful since the can-bridge owns time-sync
  ([ADR-0008](0008-time-sync-master-on-can-bridge.md)).
- Larger physical footprint with side-rail through-holes — easier for
  prototype wiring with the Ethernet kit + three CAN transceivers (per
  the three-bus topology in [ADR-0013](0013-three-can-buses.md)).

**Negative:**

- ~$12 higher BOM cost.
- Slightly larger physical footprint — not a constraint here.

**Neutral:**

- The Teensy 4.0 and 4.1 share two identical classical CAN2.0B peripherals;
  the 4.1 additionally exposes a **third CAN peripheral (CAN3) that is
  FD-capable**. Under the three-bus topology in
  [ADR-0013](0013-three-can-buses.md), CAN3 carries the Jugglebot core
  subsystem (six leg ODrives + Hand ODrive + platform Teensy 4.0), running
  classical 1 Mbps to match the ODrive firmware. The FD-capable peripheral
  choice for CAN3 is a costless future upgrade path if classical bandwidth
  ever binds — a small Positive that pushes this bullet from purely
  Neutral toward additional 4.1 advantage beyond the Ethernet pinout.

## Alternatives considered

- **Teensy 4.0 + external W5500 SPI Ethernet controller.** Rejected: more
  hardware, slower data path, more code, no real cost win after adding the
  W5500 module. The Ethernet kit + Teensy 4.1 is the documented PJRC path.
- **Teensy 4.0 + USB serial to the Jetson (no Ethernet).** Considered as
  part of the broader link-choice question; rejected via
  [ADR-0005](0005-ethernet-over-usb-serial.md) — Ethernet won on
  debuggability, galvanic isolation, cable robustness, and enumeration
  determinism.
