---
title: "A Catch re-aim shortened by the lateral clamp is judged on the Fit's demand, so an outward authority at or below the re-send tolerance is usable (BUG-161)"
type: bugfix
date: 2026-11-28
status: complete
phase: "Sitting 2026-11-28 follow-up"
related_plan: learner-efficacy.md
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - tests/motion/test_catch_reaim_clamp_tolerance.py
subsystem: [skill-stack]
tags: [catch, re-aim, clamp, columns]
---

# A clamped Catch re-aim is judged on the Fit's demand (BUG-161)

## What and why

In two-ball Columns a Catch dispatched on the schedule Site was asked by the Fit to move 16 to 36 mm outward. The lateral clamp (`catch_outward_authority_mm`, default 10) shortened the move to exactly 10.0 mm, and both re-aim paths then judged that move against `resend_pos_tol_mm` (default 10.0) with `moved_mm <= tolerance`: RESEND-SKIPPED and BUMP-DECLINED WITHIN-TOLERANCE. The Cup never moved outward and the Feed was lost (launch logs of 2026-11-27 19:13 Attempts 3, 7, 10 and 2026-11-28 12:21 Attempts 5, 6, 9). Any authority at or below the tolerance was dead for a Site-committed Catch.

Both paths (`_resend_live_catch`, `_catch_bump_decide`) now use one helper, `_reaim_moved_enough`. An axis counts when its clamped move exceeds the tolerance (unchanged), or when the clamp bit that axis, the unclamped demand there exceeds the tolerance, and the clamped move exceeds min(tolerance, half the authority on that side). C-CATCH-2 in `INVARIANTS.md` carries the rule.

## Discussion

Three alternatives were rejected. Judging the unclamped demand alone would re-solve every Catch already sitting at the clamp edge for a 0 mm move, spending solves (25 to 130 ms each, and solve load is what precedes the bridge stalls of BUG-89). Deriving the tolerance from the authority lowers the noise floor for every re-aim and adds solves on Fit jitter. A strict `<` fixes only authority equal to tolerance and depends on float rounding. The half-authority floor is a choice, not a measurement: from the Site the clamped move is the full authority and at the edge it is 0, so both are far from the threshold.

Behaviour: unchanged when the clamp does not bite, and for any authority of 20 mm or more (Ball 1's 30 mm included). At 10 mm the Cup now moves to the edge where it used to stay on the Site, at the cost of at most one extra solve per affected Catch. Not yet flown; BUG-160's Sitting at the default authority is its first flight, and the refusal count of re-aims is worth reading there.

## Verification

- 2026-11-28, `flock /tmp/jugglebot-tests.lock ./run_tests.sh --full` on the fix rebased onto main: 11,780 passed, 51 skipped, 3 xfailed; serial 16 passed; RESULT PASS.
- 2026-11-28, `pytest tests/motion/test_catch_reaim_clamp_tolerance.py -q`: 36 passed; on the unmodified executor 10 of the 36 fail. The cases cover authority 5, 10, 15 at demand 25 and 8 mm on both paths and both Sites, plus the across and inward clamps (unchanged there).
