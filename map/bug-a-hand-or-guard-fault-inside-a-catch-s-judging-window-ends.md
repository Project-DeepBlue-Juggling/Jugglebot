---
id: JB-175
kind: bug
title: A Hand or guard fault inside a Catch's judging window ends the Attempt MISSED_CATCH instead of the fault
status: complete
created: '2026-11-05'
updated: '2026-11-05T18:02:53Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: subsystem
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- map/bug-full-length-columns-goal-ends-missed-catch-on-its-final.md
- map/bug-hand-overshoot-at-apex-1-35-saturated-the-stepped-brake-at.md
- logbook/2026-11-06-jb175-fault-preempts-catch-verdict.md
filed_by: claude
---

On 2026-11-05 (launch log L2589-2615) a Hand SPINOUT disarm at 503.68 fell 0.03 s before skill 5's judging window closed (~503.71); the verdict fired at 503.767 after the lane latched and the stop REST was refused GUARD_LATCHED.

## Wrong
- END read that the Catch of Ball 1 (skill 5) produced no ball and skill 6's rows were dropped; the true cause (fault) is only in earlier lines.

## Wanted
- If the guard latched or the Hand was disarmed during the window, the end code is the fault and the Catch is booked unverifiable, not a miss.
- Distinct from JB-169 (final Catch of a Columns goal ending in a Stop); JB-172 option 4 would not cover this. Related: JB-147 (the SPINOUT).
