---
title: "BUG-90 closed by owner decision: carry_zvd stays off, the Hand-smoothness branch is dropped (TKT-85 option 3)"
type: bugfix
date: 2026-11-20
status: complete
phase: night-shift
subsystem: skill-stack
related_items: [BUG-90, TKT-85]
files_changed:
  - map/bug-carry-zvd-rows-make-the-hand-stop-and-go-at-about-5-hz-in.md
  - logbook/2026-11-20-bug-90-closed-carry-zvd-stays-off.md
---

# BUG-90 closed: carry_zvd stays off

## What and why

With `carry_zvd` on, the pose planner made the Hand stop and go at about 5 Hz in flight. The held branch `night-shift/2026-11-15-BUG-90` added a Hand acceleration cost read only with the flag on (TKT-69 option 1). It failed its Sitting on 2026-11-18 (`logbook/2026-11-18-sitting-debrief.md`). At Dwell 0.40 the Hand averaged 0.77 turning points per flight, against 2.05 without the cost and 0.03 with the flag off. Dwell 0.35 was refused LIMIT_JERK three times at the first learned Catch.

The owner answered TKT-85 with option 3: leave `carry_zvd` off and drop the branch. The flag-off Hand is already smooth.

Nothing in the code changes. `config/hardware_config.yaml` keeps `carry_zvd: false` (the default `CARRY_ZVD_DEFAULT` reads it). The strict xfail `tests/motion/test_cup_zvd.py::test_the_zvd_rows_do_not_make_the_hand_stop_and_go_in_flight` stays: it still records how the flag-on plan behaves, and it would flag a ZVD reformulation that fixes it. The Night Shift cannot delete branches, so `night-shift/2026-11-15-BUG-90` stays on the remote for the owner to delete. FEAT-32's ZVD arm is parked in practice until a Session takes it up again.

## Verification

Map and docs only. (2026-11-20, `python -m mapkit check`): map-write check clean. `tests/sim/test_logbook_front_matter.py` and `test_logbook_index.py` read this entry; the Night Shift gate's own suite runs on the next tip that carries it.
