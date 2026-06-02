# ADR-0002: Use a dedicated *second* Teensy for the leg-bridge role, not extend the existing platform Teensy

- **Status**: Accepted
- **Date**: 2026-06-02 (captured); decision made 2026-05-27
- **Deciders**: Harrison + Claude
- **Related**: [ADR-0001](0001-offload-can-and-interpolator-from-jetson.md), [parent plan](../../plans/active/teensy-can-offload.md)

## Context

Given the [ADR-0001](0001-offload-can-and-interpolator-from-jetson.md)
decision to offload CAN ownership and the 500 Hz interpolator to an MCU, the
next question is *which* MCU.

The project already has a platform Teensy 4.0 running an Arduino super-loop
that handles the hand trajectory, time-sync slave-side IIR, inclinometer
readout, and robot-state persistence
([`ros_ws/src/jugglebot/Teensy_code/`](../../ros_ws/src/jugglebot/Teensy_code/)).
A first-pass design considered extending it with the new responsibilities,
provided the firmware was rewritten on top of FreeRTOS to make room for hard
real-time scheduling.

Two pressures pushed back on the extend-the-existing-Teensy approach:

1. **Physical/electrical placement.** The platform Teensy is the *last* node
   on the existing CAN bus, physically far from the Jetson. The new device
   needs to sit near the Jetson (short Ethernet run) with its own private CAN
   bus to the six leg ODrives. Moving the existing Teensy is invasive; adding
   a second device near the Jetson is straightforward.
2. **Scope isolation.** The platform Teensy's existing throw/catch logic is
   load-bearing for current behaviour. Restructuring it on top of FreeRTOS
   risks regressing well-validated code paths for a benefit that is
   conceptually clean but operationally noisy.

## Decision

Add a **second dedicated Teensy** as the leg-bridge MCU. The new device:

- Sits physically near the Jetson (short Ethernet cable).
- Owns CAN1 (shared bus, time-sync master role per
  [ADR-0008](0008-time-sync-master-on-leg-bridge.md)) and a private CAN2 (six
  leg ODrives only — see [ADR-0004](0004-dual-can-buses.md)).
- Runs the 500 Hz interpolator and the fault state machine.
- Talks to the Jetson over a dedicated Ethernet link.

The existing platform Teensy 4.0 is **unchanged**. Its hand trajectory,
inclinometer, state persistence, and slave-side time-sync IIR continue to work
exactly as today. Its master is now the leg-bridge Teensy instead of the
Jetson, but the frame format and rate on ID 0x7DD are identical, so its IIR
filter doesn't notice the change (per ADR-0008).

## Consequences

**Positive:**

- Zero regression risk to existing throw/catch behaviour on the platform
  Teensy.
- Cleaner physical wiring — short Ethernet run, dedicated leg-bus harness.
- Two MCUs each doing one thing, easier to reason about than one MCU doing
  two complex things.
- The catching cone Teensy already sets a precedent for distributing
  responsibility across multiple peer MCUs on the CAN bus.

**Negative:**

- One additional MCU to procure, mount, power, and flash.
- Two firmware codebases to maintain (platform Teensy + leg-bridge Teensy).
  Mitigated by both being on Teensyduino with a shared protocol_config.h.
- The system has more devices, more potential failure points.

**Neutral:**

- Total BOM cost ~$75 extra (Teensy 4.1 + Ethernet kit + two CAN transceivers
  + USB-Ethernet adapter on Jetson) — see parent plan's BOM section.

## Alternatives considered

- **Extend the platform Teensy with FreeRTOS + new responsibilities.**
  Rejected for the physical-placement and scope-isolation reasons above.
  This was the initial proposal; the user's physical-architecture insight
  ("the platform Teensy is at the far end of the bus, far from the Jetson")
  flipped the decision.
- **Single MCU consolidating platform Teensy + leg-bridge + cone.** Rejected:
  the cone Teensy is physically separate (mounted on the catching cone, which
  moves), and consolidating doesn't reflect mechanical reality.
- **Use a more powerful single-board computer instead of another Teensy
  (e.g. RP2040, ESP32, STM32H7 board).** The Teensy 4.1 already has the
  required compute, mature Arduino ecosystem, and the project's existing
  expertise; switching adds friction with no clear benefit.
