# ADR-0010: The Teensy 4.1's on-board crystal is sufficient for the time-sync master role — no external clock module

- **Status**: Accepted
- **Date**: 2026-06-02 (captured); decision made 2026-05-28
- **Deciders**: Harrison + Claude
- **Related**: [ADR-0008](0008-time-sync-master-on-can-bridge.md), [parent plan](../../plans/archived/2026-08-15%20teensy-can-offload.md)

## Context

[ADR-0008](0008-time-sync-master-on-can-bridge.md) makes the can-bridge
Teensy the system's time-sync master. A natural follow-up question: does
the master need a special clock source (TCXO, OCXO, GPS PPS, external RTC
module) to be accurate enough, or is the Teensy's on-board crystal good
enough?

The system's timing requirements:

- **Short-term jitter** for the 500 Hz interpolator: target <100 µs
  jitter per tick.
- **Long-term drift** for cross-device event correlation (catch events,
  throw timing): target <10 ms over a typical session.
- **Absolute wall-clock alignment**: needed only for human-readable
  log timestamps; can be loose (seconds-level OK).

## Decision

Use the **Teensy 4.1's on-board oscillator** for all timing. No external
TCXO, OCXO, GPS, or RTC module is required.

- Hardware timer (IntervalTimer) drives the 500 Hz interpolator ISR
  directly off the chip clock.
- 64-bit monotonic time (`micros64()` in
  [`time_base.cpp`](../../ros_ws/src/jugglebot/Teensy_code_canbridge/time_base.cpp))
  extends 32-bit `micros()` for the time-sync broadcast.
- Wall-clock alignment from the Jetson over UDP (`TIME_OF_DAY_QUERY`)
  once at boot + every 30 s for drift correction.
- **Optional:** a CR2032 + holder soldered to the VBAT pin preserves
  wall-clock across power-off, so the broadcast wall-clock is reasonable
  at boot before the first UDP query lands. Not required for correctness.

## Consequences

**Positive:**

- **Zero extra hardware.** No external module to mount, wire, or
  configure.
- **Adequate accuracy by construction.** The Teensy 4.1's crystal is
  rated at ~30 ppm typical, ~50 ppm worst-case across temperature. At
  30 ppm, drift is ~108 ms/hour without correction. With UDP wall-clock
  re-sync every 30 s, the worst-case drift between corrections is
  ~0.9 ms — well under the 10 ms target.
- **Short-term jitter is microseconds, not milliseconds.** The
  hardware-timer ISR fires off the chip clock directly; no software
  scheduler is between the timer and the ISR handler.
- **Resilient to Jetson outages.** If the Jetson is unreachable, the
  Teensy keeps running on its monotonic clock — only wall-clock
  alignment degrades. The 500 Hz tick is unaffected.

**Negative:**

- **No absolute-time correctness without the Jetson.** First boot has
  arbitrary wall-clock until the first UDP query succeeds. CR2032 on
  VBAT mitigates this for warm boots.
- **Temperature sensitivity.** A 50 ppm crystal in a hot bench
  environment may drift faster than 30 ppm typical. Still well within
  budget for the application's accuracy needs.

**Neutral:**

- **No PTP, no GPS PPS, no NTP daemon on the Teensy.** These are
  available but the Jetson-anchor pattern is simpler and sufficient.

## Alternatives considered

- **External TCXO/OCXO module.** ~1 ppm accuracy, but ~$20-100 extra
  hardware and significant integration work. Rejected — no compensating
  accuracy requirement.
- **GPS PPS input to a Teensy pin.** Sub-microsecond absolute time, but
  requires GPS antenna placement and only works with line of sight to
  satellites — pointless indoors.
- **NTP daemon on the Teensy.** Requires reliable network access to NTP
  servers; the dedicated point-to-point link
  ([ADR-0007](0007-point-to-point-static-link.md)) doesn't route to
  the internet. The Jetson is effectively our local NTP-equivalent via
  the UDP TIME_OF_DAY_QUERY.
- **External RTC chip (DS3231 etc.) over I²C.** Adds wiring and a chip;
  the Teensy 4.1's on-chip RTC with VBAT backup does the same job for
  the same accuracy with no extra parts.
