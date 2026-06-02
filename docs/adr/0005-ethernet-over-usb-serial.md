# ADR-0005: Ethernet (not USB serial) for the Jetson↔Teensy link

- **Status**: Accepted
- **Date**: 2026-06-02 (captured); decision made 2026-05-28
- **Deciders**: Harrison + Claude
- **Related**: [ADR-0003](0003-teensy-4_1-over-4_0.md), [ADR-0006](0006-udp-not-tcp.md), [ADR-0007](0007-point-to-point-static-link.md), [parent plan](../../plans/active/teensy-can-offload.md)

## Context

The Jetson↔Teensy link carries the MPC setpoint stream (40 Hz down), motor
telemetry (100 Hz up), event-driven diagnostics, RPC calls (encoder search,
gain writes, etc.), and heartbeats. The total bandwidth budget is small
(~5-25 kB/s), so the link choice is driven by qualitative properties rather
than throughput.

Two options were on the table: USB serial (CDC-ACM via the Teensy's USB
connector) and Ethernet (via the PJRC Ethernet kit, requiring Teensy 4.1
per [ADR-0003](0003-teensy-4_1-over-4_0.md)).

## Decision

Use **Ethernet** for the Jetson↔Teensy link.

## Consequences

**Positive — properties that matter durably:**

- **Debuggability.** Wireshark captures the entire link end-to-end. Every
  command and telemetry frame is inspectable with standard tools; replaying
  captures into the Teensy or the Jetson side is straightforward. With USB
  serial, debugging means a logic analyser or print-based tracing.
- **Galvanic isolation.** Ethernet magnetics provide ~1.5 kV isolation
  between Jetson and Teensy electrical domains. The leg-bridge Teensy will
  be near six motor drivers switching motor current; ground-loop and noise
  injection problems are real on this kind of system. USB shares grounds.
- **Cable robustness.** RJ45 latches and tolerates vibration. USB-B/USB-C
  connectors don't, and a USB cable working loose on a moving robot has
  bitten plenty of similar projects.
- **Enumeration determinism.** USB serial devices appear under different
  `/dev/ttyACM*` paths depending on boot order, hub topology, and which
  device woke up first. udev rules help but it's fragile. Ethernet: static
  IP, link comes up, done — boot ordering is irrelevant.
- **Extensibility.** A future microcontroller (vision processor, extra
  sensor) joins the network rather than requiring a USB hub.

**Negative:**

- More hardware on the Teensy side (PJRC Ethernet kit, ~$5) and on the
  Jetson side (USB-to-Ethernet adapter for a dedicated link, ~$15).
- A small linker-script gotcha discovered at first build (libgcc's
  `pr-support.o` from libstdc++ → freertos-teensy's std::thread support
  → R_ARM_PREL31 relocation overflow; resolved by an extra_script.py that
  patches the Teensy linker script's section pattern). Captured in firmware
  README; not a recurring concern.

**Neutral — properties that don't actually matter here:**

- **Latency.** Both USB high-speed and point-to-point UDP give sub-ms
  one-way latencies, well under the 25 ms (40 Hz) and 2 ms (500 Hz)
  periods. Not a decision factor.
- **Bandwidth.** ~5-25 kB/s fits comfortably on either link. Not a
  decision factor.

## Alternatives considered

- **USB serial (CDC-ACM).** Cheaper and zero extra wiring, but loses on
  every property above. Considered as the obvious default; rejected when
  the user pushed back specifically on the USB choice.
- **CAN as the Jetson↔Teensy link.** Strange but technically possible:
  the Jetson could join CAN1 alongside the Teensys and exchange data over
  CAN. Rejected because this re-introduces the Jetson into the CAN bus —
  the exact thing [ADR-0001](0001-offload-can-and-interpolator-from-jetson.md)
  removes. Defeats the architecture.
- **Wireless (WiFi).** Rejected — latency, reliability, and security all
  worse than a dedicated wired link, with no compensating advantage.
