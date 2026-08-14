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
   the FW 10 mailbox raise. **FIXED — see P4 (FW 14).**
2. The RX ring pop in `events()` runs before the `NVIC_DISABLE_IRQ` guard, so the
   consumer's non-atomic `_available--`/`head` RMWs race the CAN ISR's
   `_available++` **one-directionally**: increments can be swallowed, `_available`
   monotonically under-counts, frames become stranded, and the ring degrades into
   an uptime-ratcheting delay line capped at one ring depth (256 slots ≈ 114–135 ms
   at Jugglebot-bus rates). Assembly-verified with this project's toolchain.
   This was the prime suspect for the uptime tracking-lag arc; **CONVICTED on
   hardware 2026-08-14** by P1's own instrument (`/ring_diag` on a 4.03 h board:
   `leak_jb` = 247–248, hwm 249 ≈ 97 % of one lap, against 1/0 on the two light
   control buses; e2e leg lag 19.9 ms fresh vs 252.2 ms at 3.80 h).
   **FIXED — see P3 (FW 14).**

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

### P3 — the `_available` leak fix: the RX pop runs IRQ-masked (2026-08-14, can-bridge FW 14)

File: `FlexCAN_T4.tpp` (`FlexCAN_T4::events()`, the RX block).

**Rationale.** Defect 2 above, now convicted on hardware rather than argued:
P1's `RING_DIAG` read `leak_jb` = 247–248 on a 4.03 h board (`true_depth`
247–248 vs `avail_reported` 0, hwm 249 ≈ 97 % of one 256-slot lap) on the
500 Hz-loaded jugglebot bus, against 1 (bb) and 0 (cone) on the two light control
buses — the arrival × pop rate ordering the mechanism predicts, and the ~90
slots/h ratchet it predicts. The ring is a delay line: e2e leg lag 19.9 ms fresh
vs 252.2 ms at 3.80 h. The pop's `memmove` out of `_cabuf[head]`, its `head`
advance and its `_available--` now all execute inside a
`NVIC_DISABLE_IRQ(nvicIrq)` / `NVIC_ENABLE_IRQ(nvicIrq)` window, with `dsb; isb`
after the mask write.

**Why this shape, and what was rejected.** The alternative of masking only the
bookkeeping RMWs would have to be written into `Circular_Buffer::read()` /
`readBytes()` — shared with `busESR1`, `busECR` and every other
`Circular_Buffer` in the tree, and with no knowledge of which NVIC line to mask.
The *narrower-looking* guard is the *wider* blast radius. It also leaves the
payload `memmove` exposed, and that `memmove` racing the ISR's overwrite-oldest
push is the ring-full frame **tear** (id from one message, payload from another,
past both decode guards) recorded as secondary find 1 of the same audit.
Masking the whole pop closes the leak and the tear with one guard, in one file,
at one site. `__disable_irq()` was rejected as well: global masking would put the
500 Hz leg-interp `IntervalTimer` ISR behind a critical section entered up to 32×
per drain tick, for no gain over a per-bus mask.

**IRQ-off budget.** The masked region is a 24-byte `memmove` plus two 16-bit
RMWs — tens of cycles, well under 0.1 µs at 600 MHz. `can_buses.cpp` drains at
most `CAN_RX_DRAIN_BUDGET = 32` `events()` calls per 1 kHz tick, so the worst case
is ~32 disjoint sub-0.1 µs windows ≈ a few µs per ms of *that bus's own* CAN IRQ
masked (< 1 % duty). The FlexCAN RX FIFO is 6 deep and a 1 Mbit/s classical frame
occupies ≥ 115 µs of wire time, so the peripheral tolerates ~690 µs of ISR
latency before it can overflow — four orders of magnitude above one window here.
The existing TX window immediately below already masks the same line for longer
(a `peek_front` plus a `writeTxMailbox` register sequence), so this extends an
idiom the library already depends on rather than introducing a new one.

**`dsb; isb`.** ARMv7-M does not guarantee a write to `NVIC_ICER` takes effect
before subsequent instructions retire; without the barriers an already-pending
CAN IRQ could still be taken a cycle or two *inside* the window, leaving a small
but nonzero hole in a fix whose acceptance criterion is `leak == 0`. The
`"memory"` clobber also stops the compiler sinking the non-volatile `_cabuf`
`memmove` across the (volatile) NVIC write.

**Minimality.** Three added statements and no reordering: `mbCallbacks()` still
runs outside any mask and still runs *before* the TX section, so dispatch order
is byte-for-byte the previous behaviour. `Circular_Buffer` is untouched.

**Concurrency status after the patch.** `head` and `_available` still have two
writers each (ISR push, task pop), but they are now **mutually excluded** — the
ISR cannot preempt the masked window, and task context can never preempt the ISR.
Single-writer-per-field is *not* the property that holds; mutual exclusion is,
and it is sufficient. The ISR-side `write()` therefore needs no patch of its own
(one CAN ISR per bus, non-reentrant). The full-ring overwrite branch — the ISR
advancing `head` — is covered by the same window, and the fix makes ring-full
unreachable in steady state anyway. P1's `probe()` remains deliberately unmasked
and still races the ISR by ±1; that is unchanged and documented at its site.

**Acceptance.** P1/P2 `RING_DIAG` is retained **unchanged** so the fix is judged
by the same instrument that convicted the defect: post-fix, `leak` must read
identically 0 on all three buses at any uptime.

### P4 — the TX deferral refill loop takes one mailbox, not all of them (2026-08-14, can-bridge FW 14)

File: `FlexCAN_T4.tpp` (`FlexCAN_T4::events()`, the `frame.mb == -1` refill loop).

**Rationale.** Defect 1 above — the library's first confirmed defect. Without the
`break`, one peeked deferred frame is written into *every* free TX mailbox
(duplicate transmission) while `txBuffer.pop_front()` runs the same number of
times (silently discarding the next queued frames unsent). One missing statement
producing both duplication and loss.

**This patch cannot change live behaviour.** Every bus on this bridge runs
`tx_deferred == 0` — the jugglebot bus since the FW 10 `setMaxMB` 16→24 raise,
bb/cone never — so `txBuffer` is never entered and the branch never executes. It
is fixed now because (a) the FW 10 raise **doubled** its blast radius (8 → 16 TX
mailboxes on the jugglebot bus), (b) the path re-opens the instant any future TX
producer re-introduces deferral, and (c) we are already in this file with a fix
arc that justifies the edit. The standing warning in `can_buses.cpp`'s `setMaxMB`
comment — "anything that re-opens the deferral path must FIX THE VENDORED LOOP" —
is discharged by this patch.

**Minimality.** One `break;`. The two ISR-side single-mailbox refill sites (in
`flexcan_interrupt()`'s TX-complete handling) are
already correct — they target a single mailbox with no loop — so no other site
needed touching.
