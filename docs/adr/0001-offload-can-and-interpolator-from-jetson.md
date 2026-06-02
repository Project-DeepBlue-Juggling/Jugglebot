# ADR-0001: Offload CAN ownership and the 500 Hz interpolator from the Jetson to a dedicated MCU

- **Status**: Accepted
- **Date**: 2026-06-02 (captured); decision made 2026-05-27
- **Deciders**: Harrison (user) + Claude (collaborative design session)
- **Related**: [parent plan](../../plans/active/teensy-can-offload.md); supersedes none

## Context

The Jetson Orin Nano currently owns the entire CAN bus: it transmits 500 Hz leg
setpoints to 6 ODrives, receives ~3000 msg/s of telemetry from the same
ODrives, runs the 500 Hz Hermite interpolator in
[`motor_guard.py`](../../ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py),
and exposes everything to ROS 2 via
[`can_node.py`](../../ros_ws/src/jugglebot/jugglebot/can_node.py).

This architecture has worked, but several pressures have accumulated:

1. The 500 Hz interpolator is hostage to Linux scheduler jitter — there is no
   way to make a Python timer on PREEMPT-non-RT Linux truly deterministic at
   2 ms.
2. CAN handling and ROS 2 publishing share a Python process, conflating two
   reliability requirements (hard real-time CAN ↔ soft real-time ROS 2).
3. Every change to the CAN protocol or ODrive integration has to be coordinated
   across `can_node.py`, `bus.py`, `odrive.py`, and the protocol-config
   codegen — three+ files in two languages.
4. The user wants a **stable substrate for years of future development**, not
   a tactical fix.

## Decision

Move all leg-CAN responsibility and the 500 Hz interpolator off the Jetson
onto a dedicated MCU. The MCU becomes the canonical CAN owner; the Jetson
talks to it over a small explicit network protocol (see ADR-0005, ADR-0006).
The MCU's choice and its placement are addressed in ADR-0002 / ADR-0003.

## Consequences

**Positive:**

- 500 Hz interpolator timing is deterministic by construction (hardware timer
  ISR), not contingent on Linux scheduling.
- One canonical CAN owner — every protocol change goes through one codebase,
  one version, one test harness.
- The Jetson loses CAN entirely; hardware diagnostics, ROS 2 tooling, and the
  Linux network stack no longer compete with CAN handling for CPU.
- Interface to the Jetson becomes small and explicit (a documented binary
  protocol over Ethernet), much easier to reason about long-term than
  *"whatever python-can does this week, plus whatever the ODrive firmware
  emits, plus whatever the Linux scheduler did."*

**Negative:**

- Substantial up-front engineering cost (~2000-3000 lines of new C++ for the
  firmware, plus a Jetson-side bridge rewrite in `can_node.py`).
- Python's interactive SDO scripting becomes an RPC over the new link —
  slower per call, though functionally equivalent.
- A second device to flash and debug in the bench loop.

**Neutral:**

- The fault state machine, deferred-stow latch, and watchdog logic — earned
  over multiple production incidents — must be ported verbatim. Risk of
  regression if porting is sloppy; managed by a Python-mirror test
  (`tests/firmware/test_fault_logic.py`) executable spec.

## Alternatives considered

- **Status quo + better RT tuning on Linux.** Rejected: PREEMPT-RT helps but
  doesn't reach hardware-timer determinism, and the architectural problems
  (mixed concerns, multi-language protocol coordination) remain.
- **Upgrade to an x86 mini-PC (e.g. Helix 401 / i5-1250PE).** Solves the CPU
  pressure but doesn't solve the architectural concerns. Considered as a
  parallel option — orthogonal to this decision, can still be done
  independently if Jetson compute becomes a separate bottleneck.
- **Roll our own bare-metal firmware (no RTOS).** Considered for simplicity;
  FreeRTOS chosen (ADR-0009) to keep the task model explicit and isolated.
