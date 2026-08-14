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

- (none at vendor time — the verbatim copy compiles unmodified)
