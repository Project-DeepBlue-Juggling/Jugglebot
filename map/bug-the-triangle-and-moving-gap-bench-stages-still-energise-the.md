---
id: BUG-88
kind: bug
title: The triangle and moving_gap bench stages still energise the Hand before a refusal can abort
status: inbox
created: '2026-11-11'
updated: '2026-11-11T10:37:56Z'
owners: []
priority: low
effort: xs
hardware: none
radius: local
diagnosis: confirmed
area: tests-tooling
blocked_by: []
links:
- tests/hardware/hand_stream_bench.py
- logbook/2026-11-11-sitting-debrief.md
filed_by: claude-debrief
---

The `triangle` and `moving_gap` stages of `tests/hardware/hand_stream_bench.py` run their envelope check (`start + tri_span > HAND_MAX_POS - HAND_MARGIN`) after the `--close-loop` bring-up. A refusal there aborts with the Hand already CLOSED_LOOP and holding.

`stroke_cut` was fixed on 2026-11-11 to plan and refuse before any RPC to the Hand (owner: refusals happen before the Hand is energised). `hold`, `step` and `gap` have no post-bring-up refusal.

## Wanted

- Move the `triangle` and `moving_gap` envelope checks ahead of the bring-up, planned from a fresh position read with the baseline as fallback, as `stroke_cut` does.
- A post-bring-up recheck stays, because the position can move on bring-up (a drive IDLE at the bottom stop read 0.4855 after bring-up on 2026-11-11).
- Pin the order with a source-order test like the one for `stroke_cut`.
