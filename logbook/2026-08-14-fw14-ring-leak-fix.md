---
title: "FW 14 — the FlexCAN_T4 `_available` leak fixed: the RX pop runs under an IRQ guard"
type: bugfix
date: 2026-08-14
status: resolved
phase: "bridge-temporal-trustworthiness P3"
related_plan: bridge-temporal-trustworthiness.md
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/lib/FlexCAN_T4/FlexCAN_T4.tpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/lib/FlexCAN_T4/PROVENANCE.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - teensy_link/rpc_args.py
subsystem:
  - can
tags:
  - safety
  - performance
---

# FW 14 — the ring-pop IRQ guard

## Summary

THE FIX for the convicted one-way `_available` leak in `FlexCAN_T4::events()` — `/ring_diag` reported `leak_jb` **247–248** at 4.03 h of uptime (conviction data in `2026-08-14-s3-conviction-ring-leak-measured.md`). Two more patches on the vendored library, both listed in `PROVENANCE.md` § Local patches. `FW_VERSION` 13→14, `EXPECTED_BRIDGE_FW_VERSION` 14, **no wire change** — every diag frame is byte-identical and `PROTOCOL_VERSION` stays **5**. **NOT FLASHED**: the operator's flash starts the validation soak.

## Fix

**P3 — the RX pop moves inside the IRQ guard.** The payload `memmove`, the head advance and the `_available` decrement now all run between `NVIC_DISABLE_IRQ(nvicIrq)` and `NVIC_ENABLE_IRQ(nvicIrq)`, with `dsb; isb` after the mask write (on ARMv7-M an ICER write may not take effect before following instructions retire) and a compiler barrier before the re-enable. The barrier is audit-added: head/tail/`_available` are `volatile` but the `_cabuf` payload copy is not, so without it a future toolchain could legally sink those loads past the re-enable and silently re-open the ring-full frame tear while `leak ≡ 0` still passed. It makes the tear-closure a **source guarantee** rather than a toolchain-empirical fact. Masking the *whole* pop is deliberate — one guard closes both the leak and the ring-full tear. The narrower Circular_Buffer-level alternative was rejected: that buffer is shared with every other ring instance and has no knowledge of the NVIC line, so the narrower-looking guard is the wider blast radius. Verified by exhaustive enumeration of the compiled ARM assembly — every head/`_available` store falls inside the window, only benign volatile loads outside, ICER/ISER literals and `dsb; isb` confirmed in the object — and independently re-verified by the audit at both source and object level.

**P4 — the missing `break`** in the TX-deferral `mb == -1` refill loop: the library's first *confirmed* defect, dormant at `tx_deferred = 0` on every bus, blast radius doubled by FW 10's mailbox raise. One line, cannot change live behaviour, and it discharges `can_buses.cpp`'s standing "must fix the vendored loop" warning.

**IRQ-off cost is bounded**: <0.1 µs per window, ≤32 disjoint windows per 1 kHz drain tick, against the 6-deep hardware FIFO's ~690 µs tolerance — four orders of margin. P2's FIFO-overflow counters report if that reasoning is ever wrong.

Comment amendments in `can_buses.cpp`: the REFUTED drain comment now carries CONVICTED-then-FIXED, and the `setMaxMB` deferral warning is marked DISCHARGED.

## Verification

`pio run -e teensy41` (run 2026-08-14, post-audit-fixes incl. the compiler barrier): SUCCESS. Scoped (`python -m pytest tests/firmware/ tests/teensy_link/ tests/ros/test_teensy_bridge_node_bridge_diag.py -q`, run 2026-08-14): **648 passed in 209.08 s**. Gate (`./run_tests.sh`, run 2026-08-15): **5147 passed in 231 s, RESULT: PASS.**

## Outcome

`RING_DIAG` is retained unchanged, as the fix's own proof: the leak criterion for acceptance is **`leak ≡ 0` at any uptime**. Full FW 14 acceptance (plan § S3 RESULTS) additionally requires the aged-soak e2e lag ≤ 20 ms and the two S3 residual reconciliations.
