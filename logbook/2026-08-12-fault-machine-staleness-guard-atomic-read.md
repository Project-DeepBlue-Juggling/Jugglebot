---
title: "The MOTOR_FB_STALE guard read a naked u64 timestamp — tearable against the higher-priority CAN-RX writer, and unordered against it too"
type: bugfix
date: 2026-08-12
status: resolved
related_plan: bridge-temporal-trustworthiness.md
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
subsystem:
  - can
tags:
  - safety
---

# `MOTOR_FB_STALE` — atomic read plus an ordering guard

## Summary

The `MOTOR_FB_STALE` guard in `fault_machine.cpp` read `axes[i].pos_timestamp_us`
with a naked `u64` load. Every sibling guard read in the same file already went
through `atomic_read_u64`; this one did not.

## Problem

A `u64` load is two 32-bit reads on Cortex-M7, and `task_can_rx` — the writer,
at priority 5 — can preempt `task_fault` at priority 3 between them. A torn
timestamp flaps the guard in **both** directions: a false `MOTOR_FB_STALE` is
~100 ms of output suppression mid-motion, and a false-fresh read hides a real
feedback freeze for one 10 Hz tick.

## Root cause

Pre-existing since the guard landed — an inconsistency with the file's own
convention, not a regression. Found by the FW 12 implementation agent while
porting the same read pattern into the new `CACHE_DIAG` sampler.

## Fix

The read now goes through `atomic_read_u64`, **and** the comparison gains a
`now > ts` ordering guard. The audit caught that the atomic read alone only
closes half the hazard: the priority-5 writer can stamp *after* the loop's `now`
capture, and the unsigned subtraction then wraps past the threshold — a false
one-tick trip. Same class as, and found by the same audit as, the `CACHE_DIAG`
age-underflow fix in the companion entry.

The heartbeat guards share the ordering half of this hazard. They are flagged
and **deliberately deferred** to a follow-up, not fixed here.

## Verification

`pio run -e teensy41` (run 2026-08-12, post-fix): **SUCCESS**.

Gate (`./run_tests.sh`, run 2026-08-12): **5035 passed in 226 s, RESULT: PASS.**

## Outcome

Ships in the FW 12 image — same compile, same flash as the `CACHE_DIAG` uplink,
so it goes live when the operator flashes to start the S2 soak.
