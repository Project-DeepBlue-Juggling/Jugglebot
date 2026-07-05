---
title: canhealth err field now counts wire errors — raw ESR1-change counter renamed chg, error-only ESR1 ring
type: bugfix
date: 2026-07-05
status: resolved
phase: "canhub-hardening (marginal-CAN3 follow-up)"
related_plan: canhub-hardening.md
related_entries:
  - 2026-07-05-canhub-marginal-can3-diagnosis   # the investigation that discovered the mislabel; this entry completes its observability follow-through
sessions: []
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/README.md
commits: []                # backfilled after commit
subsystem:
  - canbridge
tags:
  - can-bus
  - firmware
  - diagnostics
  - observability
---

## Summary

The 1 Hz `[canhealth]` line's headline `err=` field printed the raw FlexCAN_T4
ESR1-change snapshot counter — a value that climbs continuously on a perfectly
healthy bus — and that mislabel was the root of the multi-session marginal-CAN3
ghost hunt closed in [2026-07-05-canhub-marginal-can3-diagnosis](2026-07-05-canhub-marginal-can3-diagnosis.md).
This entry lands the observability follow-through: `err=` now counts only
snapshots carrying an actual wire-error type bit (0 on a healthy bus), the raw
counter is kept but renamed `chg=` (capture-machinery liveness only), and the
8-deep ESR1 diagnostic ring — which was self-erasing genuine error words within
~40 ms under benign traffic — now records only wire errors and one-shot
warning/bus-off interrupt crossings.

## Problem

The 1 Hz `[canhealth] err=` field printed the library's raw ESR1-change
snapshot counter, which climbs in NORMAL operation — ~200/s with the 100 Hz
0x7DD sync broadcast active, ~1-2/s RX-only idle — because FlexCAN_T4 captures
a snapshot on ANY masked ESR1 change, benign IDLE/RX/TX activity-bit flips
included. The 2026-07-05 marginal-CAN3 investigation established this mislabel
as the root of a multi-session ghost hunt ("the bus is erroring 10-15 times a
second"). After the gate validation the operator asked three questions:

1. Does the climbing counter overflow? — uint32, wraps harmlessly after
   ~248 days at 200/s; nothing consumes it but the printf. No hazard.
2. Is there a clean way to make it log actual errors? — yes; this change.
3. Is the raw counter useful at all? — marginally, as liveness of the
   capture machinery itself.

A second REAL defect surfaced while designing the fix: the 8-deep raw-ESR1
diagnostic ring recorded EVERY snapshot, so at ~200 benign flips/s a genuine
error word was flushed out of the ring within ~40 ms — long before the 1 Hz
`[canesr1]` print could show it. The evidence-capture mechanism was
self-erasing under exactly the traffic conditions where you'd want it.

## Root Cause

The `[canhealth]` line inherited the library's `err_events` counter under the
name `err` when the health line was first added. The investigation
instrumented per-type counters alongside it but left the headline field's
semantics unchanged — the label promised "errors" while the counter delivered
"any masked ESR1 change".

## Fix

All four files, ~30 lines:

- `ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h` — new
  `BusRxHealth.wire_errs` counter: snapshots with ≥1 ESR1 error-type bit set.
  Documented as a **lower bound** during sustained identical-error storms (the
  library's change-detect captures a repeated identical ESR1 once until
  another masked bit changes); `tecInc`/`recInc` (1 kHz ECR delta sums) remain
  the rate-accurate signals in that regime. `txctx`/`rxctx` re-documented as
  wire-error-snapshot context.
- `ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp`
  (`poll_bus_errors`) — classify each snapshot (`wire` = any of
  ACK/CRC/FORM/STUFF/BIT0/BIT1); increment `wire_errs` and the TX/RX context
  counters only for wire snapshots; ring-record only snapshots that are wire
  errors OR one-shot warning/bus-off interrupt crossings (TWRN_INT bit 17,
  RWRN_INT bit 16, BOFFINT bit 2 — deliberately NOT the level bits 8/9, which
  stay latched while a counter sits elevated and would re-flood the ring with
  benign flips). Raw `err_events` still counts everything.
- `ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino` —
  `err=` now prints `wire_errs` (0 on a healthy bus); the raw counter is
  appended as `chg=`, kept solely as capture-machinery liveness (`chg` frozen
  at 0 would mean the snapshot path is broken and `err=0` unmeaningful).
  `[canerrs]` print condition changed from `err_events != 0` to
  `(wire_errs | tec_inc_sum | rec_inc_sum) != 0` — healthy buses now print NO
  `[canerrs]` and NO `[canesr1]` lines at all.
- `ros_ws/src/jugglebot/Teensy_code_canbridge/README.md` — serial-console
  field reference updated to match (`err`/`chg` rows, `[canerrs]` gating,
  `[canesr1]` filtered-ring semantics), plus a note that pre-2026-07-05
  captures printed the raw counter under the `err=` name — important for
  anyone comparing captures across builds.

## Discussion

- **Why rename, not remove.** `chg=` preserves the capture-machinery liveness
  signal (a frozen `chg=0` is the tell that the snapshot path itself is
  broken, without which `err=0` proves nothing) and keeps continuity with all
  prior captures. The semantic boundary is explicitly documented in the
  README so cross-build capture comparison doesn't silently mislead.
- **Why the ring filters on interrupt-crossing bits, not warning-level
  bits.** The TWRN/RWRN *level* bits (8/9) stay latched the whole time a
  counter sits elevated, so admitting them would re-flood the ring with
  benign activity flips — reintroducing the exact self-erasure this fix
  removes. The one-shot *interrupt* crossings (bits 17/16/2) fire once per
  threshold event, which is the evidence worth keeping.
- **Class-level lesson.** The investigation entry already records it:
  ambiguous observability labels cost multi-session ghost hunts. This entry
  is the enforcement follow-through — the misleading label is now
  structurally impossible to misread, because a healthy bus prints `err=0`.
- **Overflow non-issue.** uint32 counters with a print-only consumer; a wrap
  takes ~248 days at the worst-case ~200/s rate and is harmless.

## Verification

- Native firmware suite (`pytest tests/firmware/test_native_firmware.py -q`,
  run 2026-07-05): **16/16 pass in 157.35 s** — compile-validates the
  modified headers via the include chain.
- Firmware compiled (`pio run`, teensy41) and flashed 2026-07-05; robot
  idle-verified before flash.
- 45 s bench capture post-flash (2026-07-05): all three buses print `err=0
  flags=0x00 tec=0 rec=0 gated=0`; `chg=89` and climbing only on the active
  CAN3 (RX-only idle, unsynced); zero `[canerrs]` and zero `[canesr1]` lines
  — the console is silent on healthy buses. All 7 ODrives heartbeating
  normally.

## Outcome

A healthy bus now reads `err=0` — the headline field means what it says, and
the raw change counter survives as an explicitly-labelled liveness signal
(`chg=`). The ESR1 evidence ring no longer erases genuine error words under
benign traffic, so the next real wire event will still be in the ring when
the 1 Hz print (or an operator) gets to it. The observability follow-through
from the marginal-CAN3 investigation is complete.
