# ADR-0006: UDP (not TCP) for every Jetson↔Teensy protocol channel

- **Status**: Accepted
- **Date**: 2026-06-02 (captured); decision made 2026-05-28
- **Deciders**: Harrison + Claude
- **Related**: [ADR-0005](0005-ethernet-over-usb-serial.md), [parent plan](../../plans/archived/2026-08-15%20teensy-can-offload.md)

## Context

Given Ethernet as the transport ([ADR-0005](0005-ethernet-over-usb-serial.md)),
the next question is what runs on top of it. The two natural choices are
UDP and TCP.

The Jetson↔Teensy traffic has three distinct shapes:

1. **High-rate streams** — setpoint downlink at 40 Hz (~166 B/frame), telemetry
   uplink at 100 Hz (~74 B/frame). Latency-sensitive; loss-tolerant (the
   interpolator's late-update ladder gracefully handles missed setpoints).
2. **RPC** — gain writes, state changes, encoder-search start, time-of-day
   queries. Low rate; need reliable delivery; small payloads.
3. **Heartbeats** — both directions, ~10 Hz. Tiny payloads; loss-tolerant
   (the watchdog tolerates a few missed beats before declaring link-fault).

## Decision

**Use UDP for everything** — high-rate streams, RPC, and heartbeats.

Reliable RPC delivery is implemented as a sequence-numbered request/response
pattern with timeout-and-retry on top of UDP. The protocol carries CRC-16
per frame and a sequence number for loss detection.

## Consequences

**Positive:**

- **No head-of-line blocking.** TCP's reliability means a dropped or
  reordered packet stalls the stream until it's recovered. For a 40 Hz
  setpoint stream where a stale setpoint is *worse than no setpoint*, that
  is the wrong trade. UDP delivers what gets through, immediately.
- **No bufferbloat.** TCP's Nagle algorithm + retransmit queues can balloon
  latency under congestion. UDP has no such mechanism.
- **No congestion-control surprises.** TCP probes the link with slow-start
  and back-off algorithms that interact poorly with periodic real-time
  traffic.
- **Simple stack.** lwIP UDP on the Teensy is much smaller and simpler than
  lwIP TCP. Less to go wrong; less to audit.
- **Symmetric handling.** All three traffic shapes use the same framing
  layer (fixed-length typed frames with CRC-16); the protocol design and
  test surface is uniform.

**Negative:**

- **RPC reliability is the application's job.** Sequence numbers,
  timeout-and-retry, response correlation — all explicit code. Mitigated by
  centralising it in one `rpc.h` / `rpc.cpp` module and by the rarity of
  RPC calls (gain writes happen during setup, not in the hot loop).
- **Manual retry logic must respect idempotence.** State-mutating RPCs
  (set_input_pos, set_state) must tolerate accidental duplicate delivery.
  Designed in: each RPC carries a sequence number; the responder
  deduplicates by `(client_id, seq)`.

**Neutral:**

- **No streaming semantics.** Each UDP datagram is exactly one message
  boundary, which means no byte-stream framing layer (COBS, length-prefix
  scan, etc.) is needed. See the fixed-length typed-frame decision
  documented in the handoff doc (D1).

## Alternatives considered

- **TCP for RPC, UDP for streams.** Considered briefly. Rejected because the
  benefit (free RPC reliability) is small (we have to handle session reset,
  reconnect, idempotence anyway) and the cost (two protocol stacks to
  configure on both sides, two sets of failure modes) is real. Single
  protocol throughout is cleaner.
- **Custom protocol over raw Ethernet (no UDP/IP).** Rejected. We lose
  routing flexibility (e.g. future addition of a third device on the
  network) and standard tools (Wireshark dissectors, `nc`-style debugging).
  No benefit to compensate.
