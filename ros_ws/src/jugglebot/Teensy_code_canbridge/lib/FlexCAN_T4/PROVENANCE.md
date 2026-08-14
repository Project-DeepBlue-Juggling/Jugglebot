# FlexCAN_T4 — vendored copy provenance

**Vendored 2026-08-14** from the PlatformIO Teensy framework package:

- Source: `~/.platformio/packages/framework-arduinoteensy@1.159.0/libraries/FlexCAN_T4/`
  (framework-arduinoteensy 1.159.0, pinned transitively by `platform = teensy@5.1.0`
  in `../../platformio.ini`; upstream library dated 2024-02-09, tonton81/FlexCAN_T4)
- Copied **byte-identical** (all 21 files, md5-verified against the package at copy
  time). PlatformIO's Library Dependency Finder resolves a project-local `lib/`
  library ahead of the framework copy, so this directory is what compiles from now
  on — confirmed in the build log at vendor time.

## Why vendored

Two confirmed defects found in this library by this project, both in the paths we
exercise (record: `logbook/2026-08-02-err-timeout-attribution-instrumentation.md`
§ Addendum; the 2026-08-14 RX-ring concurrency audit in the
bridge-temporal-trustworthiness arc):

1. `events()` TX deferral `mb == -1` refill loop lacks a `break`
   (`FlexCAN_T4.tpp` ~1089–1096) — a deferred frame is written into every free TX
   mailbox. Dormant in our config (`defer jb = 0`) but doubled in blast radius by
   the FW 10 mailbox raise.
2. The RX ring pop in `events()` runs before the `NVIC_DISABLE_IRQ` guard, so the
   consumer's non-atomic `_available--`/`head` RMWs race the CAN ISR's
   `_available++` **one-directionally**: increments can be swallowed, `_available`
   monotonically under-counts, frames become stranded, and the ring degrades into
   an uptime-ratcheting delay line capped at one ring depth (256 slots ≈ 114–135 ms
   at Jugglebot-bus rates). Assembly-verified with this project's toolchain.
   This is the prime suspect for the uptime tracking-lag arc.

A library with demonstrated queue-handling defects, pinned only by platform
version and living outside the tree, cannot host the instrumentation or the fix.

## Local modification policy

- This file plus any files listed under **Local patches** below are the ONLY
  divergence from upstream. Keep every patch minimal, commented at the site with
  `// JUGGLEBOT PATCH:`, and listed here with its rationale and commit.
- To diff against pristine upstream: the md5 manifest of the byte-identical copy
  is preserved in the vendor commit itself (this directory at that commit IS the
  pristine manifest).

## Local patches

### P1 — true RX-ring occupancy accessors (2026-08-14, can-bridge FW 13)

Files: `circular_buffer.h` (`Circular_Buffer::probe`), `FlexCAN_T4.h`
(`FlexCAN_T4::rxRingProbe`).

**Rationale.** Defect 2 above makes `_available` monotonically under-count, and
`getRXQueueCount()` returns exactly `_available` — so the bridge's existing
`depth_hwm` and `cap_hits` counters (`can_buses.cpp service_bus`) are blind to a
stranded backlog *by construction*: they are computed from the very number the
race corrupts. Any measurement of the leak has to come from somewhere else.
`head` and `tail` are the somewhere else — each has a single writer in steady
state (the ISR advances `tail`, the task-side pop advances `head`; the ISR
touches `head` only when the ring is FULL), so their difference under the ring's
modulus is the TRUE occupancy. `true_depth − _available` **is** the leak, and it
is the one number that convicts.

**Minimality.** Read-only. No index is written, no pop or push logic is touched,
and the defect itself is deliberately NOT fixed here — the fix is sequenced after
the occupancy measurement so the fix can be judged against a number rather than a
theory.

**Read ordering is load-bearing** and is documented at the accessor: `head`/`tail`
first, `_available` last, so a push concurrent with the probe can only shrink the
reported leak, never invent one. Probing from task context races the ISR by ±1
regardless; that is acceptable for a 1 Hz diagnostic, which is why nothing is
IRQ-masked (masking would put the instrument inside the timing budget of the very
path it is measuring).

### P2 — FIFO overflow / warning counters (2026-08-14, can-bridge FW 13)

Files: `FlexCAN_T4.h` (two `volatile uint32_t` members + two accessors),
`FlexCAN_T4.tpp` (`flexcan_interrupt`, the IFLAG bit 6/7 clear site).

**Rationale.** Upstream *clears* IFLAG bits 6 (FIFO warning) and 7 (FIFO
overflow) and records neither. A bit-7 event is a frame lost inside the
peripheral, upstream of the software ring and therefore upstream of every counter
the bridge keeps — the one RX loss path that is invisible to all existing
telemetry. Counted at the clear site, in ISR context, which is the sole writer:
these two are exact and race-free, in deliberate contrast to the `_available`
counters they sit beside.

**Known scope limit** (documented, not fixed): the clear site lives inside the
`iflag & FLEXCAN_IFLAG1_BUF5I` branch, so an overflow is counted only when the
ISR also has a FIFO frame available. Since an overflow means the FIFO is full,
BUF5I is co-asserted in every realistic case; the counters are nonetheless a
lower bound by construction.

**Minimality.** Two increments and two member initialisers. No control flow, no
clear order, and no register access changes.
