---
title: "Bench hand_stream_bench: triangle and moving_gap refuse before the Hand bring-up (BUG-88)"
type: bugfix
date: 2026-11-12
status: complete
phase: night-shift
subsystem: bench/hand
files_changed:
  - tests/hardware/hand_stream_bench.py
  - tests/motion/test_hand_stream_bench_order.py
  - logbook/INDEX.md
  - map/bug-the-triangle-and-moving-gap-bench-stages-still-energise-the.md
tags: [night-shift, BUG-88]
related_items: [BUG-88]
---

# triangle / moving_gap refuse before the Hand is energised

## What changed

The `triangle` and `moving_gap` stages of `hand_stream_bench.py` ran their envelope check (`start + tri_span > HAND_MAX_POS - HAND_MARGIN`) only after the `--close-loop` bring-up, so a refusal left the Hand CLOSED_LOOP and holding. `stroke_cut` was already fixed (2026-11-11, owner: refusals happen before the Hand is energised).

- New pure helper `triangle_envelope_refusal(start_rev, span_rev)` returns the refusal text or None.
- A pre-bring-up block (both stages) plans from a fresh `_hand_pos()` read, the startup baseline as fallback, and aborts with "nothing was commanded" before any `SET_*` RPC.
- The post-bring-up check stays, now through the same helper, and says the Hand is CLOSED_LOOP and holding when it refuses there (true with or without `--close-loop`, whose absence requires a CLOSED_LOOP Hand already; the position can move on bring-up: a drive IDLE at the bottom stop read 0.4855 rev afterwards on 2026-11-11).
- `hold`, `step` and `gap` have no refusal of this kind and are untouched.

Behaviour change is only "refuse earlier, keep the late recheck": no trajectory, belt, timing or frame content changed, and no RPC was added or reordered.

## Verification

Run 2026-11-12: `tools/night_shift_nonet.sh python -m pytest -q tests/motion/test_hand_stream_bench_stroke_cut.py tests/motion/test_hand_stream_bench_order.py` -> **34 passed in 1.15 s**. The same command with the pre-bring-up block deleted: `test_triangle_and_moving_gap_envelope_is_checked_before_bring_up` and `test_no_rpc_before_the_envelope_checks` fail (2 failed, 2 passed); restored, all pass. The bench script itself was not run (hardware, none needed).
