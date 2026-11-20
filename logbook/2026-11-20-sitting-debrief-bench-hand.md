---
title: "Bench Hand Sitting 2026-11-20 debrief: 150 of 150 rung C brakes passed with no fault; the feedforward-offset and stall-guard rows did not fly because of two bench tool defects, both fixed"
type: investigation
date: 2026-11-20
status: complete
phase: "Sitting debrief"
related_plan: ""
related_issues: []
files_changed:
  - logbook/2026-11-20-sitting-debrief-bench-hand.md
  - logbook/INDEX.md
  - map/sheets/2026-11-20-bench-hand.results.yaml
subsystem: [hand, tooling]
---

## Summary

Sheet `2026-11-20-bench-hand`, flown 23:24 to 00:07. One launch-less bench Sitting on the bench Hand, bridge firmware 46 (firmware 47 flashed for steps 15 to 17 and firmware 46 flashed back at step 18).

| Item | step | verdict | measured |
|---|---|---|---|
| FEAT-45 | 11 | pass | 150 of 150 rung C brakes PASS in one run, no fault, no scheduler refusal, top stop 10.4 rev against 10.09 needed, FET 31.7 to 35.9 C, worst send lateness 2.8 ms |
| BUG-105 | 14 | inconclusive | not flown: all 17 triangle runs aborted on the tool's 0.5 rev belt within 50 ms |
| BUG-123 | 16 | inconclusive | not flown: `--stage stall` crashed on a KeyError before the link opened |

Merged: nothing from this sheet (FEAT-45 had no branch and closes as a plain check). Held: `night-shift/2026-11-20-BUG-123` (stall guard, now with the tool fix) and `night-shift/2026-11-21-BUG-105` (triangle fix, waiting on the owner's belt ruling).

## Big picture

What went well: the bench Hand flew 150 hard brakes in a row at a 122 rev/s entry with every one passing, no fault, and the drive's FET temperature rising only 4 C. The new any-count repeat did its job in one eight-minute run.

What did not: the other two rows never flew, and both failures were in tooling this Session built the same evening. The automated cut started its ramp at the moment of arming, when the firmware is still easing the Hand back at no more than 1 rev/s; the tool's belt then correctly stopped a ramp the Hand could not follow. The stall stage crashed on a missing table entry because no test ran it through its real entry point. Both are fixed, and review of the stall stage found a second, worse defect: a lost reply could have left the Hand's zero shifted with no restore.

What it means: the bench Hand does not reproduce the robot's torque loss in 150 brakes, so the fault is either rare below one in a few hundred or specific to the robot's Hand, and the capture has to sit armed on the robot. The stall guard in firmware 47 is still unproven on hardware. Both tool failures were of one kind: a stage tested piece by piece but never end to end, so the fixes add end-to-end tests of both stages.

Next steps: FEAT-45 is closed. BUG-123 and BUG-105 fly at the next bench Sitting with the fixed tool; BUG-105's fix needs the owner's ruling on a wider belt first. BUG-57's capture continues on the robot under FEAT-49.

## Discussion

**The triangle abort.** Two readings were proposed before the code was read: the belt comparing a knot sent ahead by the lane's lead, or the Hand's lag at a velocity step. Both were withdrawn. The cause is the firmware's recovery slew on the output-enable edge (`leg_interp.cpp`): on every arm the Hand's command walks from the encoder toward the target at 5 rev/s^2, capped at 1 rev/s, and hands back only within 0.002 rev. A triangle that is already moving at 10 rev/s when the arm lands is never caught. The bench CSV shows it: the encoder is flat and the echoed command climbs 0.006, 0.012, 0.025 rev, the slew's curve. Earlier triangle runs were at 0.5 rev/s, which the slew can follow, so the defect predates the automated cut and a Ctrl-C run at 10 rev/s would have aborted the same way.

The fix holds still across the arm and starts the triangle clock only when an echo received after the arm is back on the start. It also changes the belt default from a flat 0.5 rev to 0.5 + speed/35 (0.79 rev at 10 rev/s, 1.07 at 20), because the simulated tracking residual at 20 rev/s is 0.53 to 0.56 rev. A belt is a guard, so the change is held for the owner. The alternative is to keep 0.5 rev, which a 10 rev/s run should pass (residual 0.28 rev) and a 20 rev/s run should trip.

**The scope saved nothing, and that is expected.** `hand_stream_bench` never commands IDLE on exit; it disarms and stops the link, and the drive stays in closed loop. With no fault and no exit from closed loop there was no window to save. So this Sitting did not test what FEAT-45's row hoped to test in passing (whether the scope saves a window after a clean exit); that question stays with FEAT-49 on the robot.

**Owner's open questions from the sheet.** The temperature stop for the long run is not needed on this evidence (the bench drive reports no motor temperature and the FET rose 4 C). The 6 rev span for the downward cut fits the measured 10.4 rev top stop. BUG-105's pass line now judges against the echoed feedforward over 35, the value the drive was left holding.

## Fixes

- `night-shift/2026-11-21-BUG-105`, `0bcea663`: the triangle waits for the arm-edge slew; new end-to-end test at up and down, 10 and 20 rev/s, which failed before the fix with the bench's abort line. Held.
- `night-shift/2026-11-20-BUG-123`, `d2976ae9`: the stall stage runs through `main()`; the frame shift is marked before the call and the cleanup restores it from the measured position or says to re-home. Five end-to-end tests, three injecting a failure after the shift.
- Not checkable without hardware: that the real echo settles on the start after the arm; the real residual at 10 and 20 rev/s; the stall stage's USB read alongside the scope.

## Verification

- 2026-11-21, building agent's scoped runs under the test lock: triangle worktree 201 passed in 22.79 s; stall worktree 122 passed in 4.59 s (the agent's reported commands are in its commits' logbook entries).
- 2026-11-21, `pytest tests/sim/test_map_items.py tests/sim/test_mapkit_sheet.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_docs_links.py tests/sim/test_no_legacy_item_ids.py -q` on this commit: 131 passed in 7.59 s (with `tests/sim/test_plans_index.py` added); and `pytest tests/sim/test_mapkit*.py tests/sim/test_sitting_debrief_protocol.py -q -n 2`: 331 passed in 20.87 s. No full gate: this commit is documentation and map files only.
