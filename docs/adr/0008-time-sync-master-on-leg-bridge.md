# ADR-0008: Time-sync master moves from the Jetson to the leg-bridge Teensy

- **Status**: Accepted
- **Date**: 2026-06-02 (captured); decision made 2026-05-28
- **Deciders**: Harrison + Claude
- **Related**: [ADR-0001](0001-offload-can-and-interpolator-from-jetson.md), [ADR-0009](0009-freertos-tsandmann-port.md), [ADR-0010](0010-onboard-clock-sufficient.md), [parent plan](../../plans/active/teensy-can-offload.md)

## Context

The Jugglebot system has a "robot wall-clock" used to align events across the
distributed MCUs (platform Teensy, ball-butler Teensy, catching cone Teensy)
and the Jetson. Today the **Jetson** is the master: it broadcasts a 100 Hz
sync frame on CAN1 ID 0x7DD with the current `(sec, usec)` as `pack('<II',
sec, usec)` (see
[`bus.broadcast_time`](../../ros_ws/src/jugglebot/jugglebot/can/bus.py)).
All three slave Teensys IIR-filter the offset between their local monotonic
time and the broadcast.

Once [ADR-0001](0001-offload-can-and-interpolator-from-jetson.md) moves CAN
ownership to the leg-bridge Teensy, the master role can stay where it is
(Jetson on CAN1 alongside the leg-bridge) or move to the leg-bridge. Moving
it is a small change in scope; the question is whether to do it.

## Decision

**Move the time-sync master role from the Jetson to the leg-bridge Teensy.**

- The leg-bridge Teensy broadcasts on CAN1 ID 0x7DD at 100 Hz using the
  *same* `(sec, usec)` payload format the Jetson uses today. Slave IIR
  filters consume it unchanged.
- The leg-bridge's wall-clock anchor comes from the Jetson once at boot
  (UDP `RpcMethod::TIME_OF_DAY_QUERY`) and is re-queried every 30 s for
  drift correction.
- The Jetson stops broadcasting on CAN (it has no CAN bus to broadcast on
  after the cutover) and starts responding to UDP wall-clock queries
  instead.

The change is architecture-driven, not load-driven. The total CPU/load
impact is negligible (~0.5% on the leg-bridge, ~0% elsewhere).

## Consequences

**Positive:**

- **Canonical clock on the device with the hardest real-time properties.**
  The leg-bridge already runs a hardware-timer ISR at 500 Hz for the
  interpolator — sub-microsecond resolution, zero scheduler jitter, no
  preemption from Linux housekeeping. It is the *cleanest* time base on
  the whole system. Logically, the master role belongs there.
- **CAN bus and UDP bus share a single time origin.** Log correlation
  across the two transports becomes straightforward.
- **Robot timing decouples from Jetson load.** The Jetson can GC, do ROS 2
  bookkeeping, reboot, or get bogged down by a heavy DDS subscriber —
  none of it disturbs platform-level timing.
- **All existing slave Teensys are unaffected.** Platform Teensy 4.0,
  ball-butler Teensy, catching cone Teensy all see the same frame ID and
  same payload format; the master identity is invisible to their IIR
  filter. Zero firmware changes on those devices.
- **Composition with related architectural moves is clean.** Once
  [ADR-0001](0001-offload-can-and-interpolator-from-jetson.md) is in
  place, the Jetson is no longer a CAN endpoint at all — making it the
  time master would force a special-case CAN-out path on the Jetson just
  for time-sync. Moving the master removes that special case.

**Negative:**

- The leg-bridge needs a wall-clock anchor at boot. UDP
  `TIME_OF_DAY_QUERY` is small (one round-trip), but it does mean the
  Teensy's broadcast wall-clock is only correct after the Jetson is up
  and reachable. Until then, monotonic-only time is broadcast (slaves
  IIR-lock to it; absolute time corrects on first Jetson query).
- An optional CR2032 on the Teensy 4.1's VBAT pin can preserve
  wall-clock across power-off — recommended but not required.

**Neutral:**

- The Jetson gains a tiny UDP responder for `TIME_OF_DAY_QUERY` (replies
  with `CLOCK_REALTIME` µs). Trivial code.

## Alternatives considered

- **Keep the Jetson as time-sync master.** Means the Jetson must keep a
  foot in the CAN bus just to send 0x7DD frames — undoing part of
  [ADR-0001](0001-offload-can-and-interpolator-from-jetson.md). Rejected.
- **PTP (IEEE 1588) over the dedicated Ethernet link.** Sub-microsecond
  cross-device sync. Massive overkill for this application; the IIR-on-
  CAN approach has worked for years and is sufficient.
- **GPS PPS as the master clock source.** Overkill; no compensating
  benefit for an indoor catching robot.
