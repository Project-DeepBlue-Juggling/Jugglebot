---
title: "hand_stream_bench: --repeat takes any count with explicit stop rules and a tally, and the triangle stage can cut its own stream mid-ramp with Ctrl-C's wire behaviour (FEAT-45, BUG-105)"
type: feature
date: 2026-11-20
status: complete
phase: "Bench Hand Sitting tooling"
related_plan: ""
related_issues: []
files_changed:
  - tests/hardware/hand_stream_bench.py
  - tests/teensy_link/test_hand_stream_bench_tri_cut.py
  - logbook/2026-11-20-bench-stage-repeat-and-auto-cut.md
  - logbook/INDEX.md
subsystem: [tooling, hand]
tags: [bench, hand, stroke_cut, feedforward]
---

## What and why

Owner asks (2026-11-20): one command instead of fifteen for the FEAT-45 volume run, and no hand-timed Ctrl-C for the BUG-105 cuts.

**`--repeat`.** The `[1, 10]` bound came in with the stage (0516f9d4, 2026-11-03) with no recorded reason: not thermal, and the only arithmetic on it is the ARM prompt's duration estimate. It now takes any positive count. What a cap could guard is explicit instead: the stage stops at the first brake that is not PASS (after that run's return, so the disarm is from a hold at the start; before this a FAIL ran on), the first fault or belt abort, and the first `sched_refused` above 0 (both already stopped it). Each run prints `run N of M` with the drive's motor and FET temperatures from its Diagnostic, and a `TALLY:` line ends every outcome, Ctrl-C included. No temperature limit was invented.

**`--tri-cut up|down`** (triangle only, `--tri-cut-cycle`, default 2). Ctrl-C on this stage leaves the loop after a sent frame; `main()`'s `finally` sets the heartbeat flags to 0 (DISARMED on the next 10 Hz heartbeat, no flush frame), sleeps 0.2 s and stops the client. The automated cut breaks out straight after the frame at mid-ramp and makes the same one call. No SETPOINT or RPC follows. It then only listens for 2 s, so the DISARMED heartbeat runs 2 s instead of 0.2 s; with output already gated off that commands nothing (`fault_machine.cpp`). It prints the last knot, the firmware's last command (HAND_CMD_ECHO input_pos and now vel_ff), the settled position and drift from telemetry, the offsets, and any latch. A cut whose worst-case end comes within 0.6 rev of the bottom stop (-0.10) or `--top-stop` is refused before the bring-up.

**Finding for BUG-105.** On this legacy lane the firmware coasts past the last knot until the disarm lands (0 to about 0.1 s), with Ctrl-C too. At 20 rev/s that is up to about 2 rev, so the offset law must be read against the echoed last command, not the last knot. A 20 rev/s down cut with span 4 is refused; the sheet uses span 6.

## Verification

- 2026-11-20, `python -m pytest -q -p no:cacheprovider tests/motion/test_hand_stream_bench_order.py tests/motion/test_hand_stream_bench_stroke_cut.py tests/teensy_link/test_hand_stream_bench_gap.py tests/teensy_link/test_hand_stream_bench_tri_cut.py`: 107 passed (17 new). Importer grep `grep -rln "hand_stream_bench\|hand_stroke_cut" tests tools --include='*.py'`: those four test files; `tools/probes/hand_foc_scope.py` names it in prose only.
- The new tests drive the real `main()` against a recording fake link: the cut frame is mid-ramp, the tail after it equals a Ctrl-C at the same frame, and a refused down cut sends nothing. They also run `_run_stroke_cut` on a fake clock for 12 runs, a stop at the first FAIL, and a fault.
- Not run against hardware.
