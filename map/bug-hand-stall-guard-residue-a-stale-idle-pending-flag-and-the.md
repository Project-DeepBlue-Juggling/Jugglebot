---
id: BUG-131
kind: bug
title: 'Hand stall guard residue: a stale idle-pending flag, and the latch clear and the stage PASS line never flown'
status: inbox
created: '2026-11-22'
updated: '2026-11-22T12:12:59Z'
owners: []
priority: low
effort: s
hardware: bench-hand
owner: hands
diagnosis: confirmed
area: can-bridge
blocked_by: []
links:
- map/bug-a-guard-latch-leaves-the-hand-pushing-at-full-current-no.md
- logbook/2026-11-22-sitting-debrief-bench-hand-2.md
filed_by: claude
night_skip: sitting
night_skip_on: '2026-11-23'
night_skip_note: Items 2 and 3 fly on main's FW 48 at the next bench-hand Sitting; item 1 (stale idle-pending flag) is a firmware edit for the next flash.
---

Three things left over from BUG-123 (firmware 48, merged 2026-11-22).

1. Audit finding, firmware. `s_lane_idle_pending` in `leg_activate.cpp` is cleared only when its IDLE frame is queued. If the trip's IDLE stayed refused until the operator cleared the latch, the flag survives the clear, and the next ladder that targets the Hand sends one IDLE on its first tick. Cure: clear it in `stall_reset_for_targets()` when the Hand is targeted. It needs a persistently refused send to occur at all.

2. The latch clear is unflown. `CLEAR_ERRORS` clears the HAND_STALL latch in the code; at the Sitting of 2026-11-22 the owner reflashed the bridge after each trip instead, so the clear path has not run on hardware.

3. The stage never printed a PASS line. Its "second rise" was the stale current on the trip frame; the stage now waits for the collapse (branch commit 24750721). Whether the current stays down unaided was not observed.

## Sitting

- setup: bridge-cycled
- why: one stall run on main's firmware 48 answers items 2 and 3 together.
- do:
  - Launch down. Bench Hand on the bridge, Hand homed and parked between 0.3 and 0.7 rev, Cup empty, E-stop in hand, motor cool to the touch.
  - From the repo root in the venv: `python tests/hardware/hand_stream_bench.py --stage stall`. Type yes at the prompt. Expect a "current collapsed" line and a STALL-GUARD PASS line.
  - Then `python tests/hardware/hand_stream_bench.py --clear-errors`. Expect "CLEAR_ERRORS OK". Do not reflash.
  - Home the Hand again before anything else.
- pass: the stage prints STALL-GUARD PASS with the collapse inside 250 ms, and `--clear-errors` alone returns the fault state to NONE.
- record: the stage output from "stall plan" to the verdict line, and the clear-errors line.
- minutes: 10
