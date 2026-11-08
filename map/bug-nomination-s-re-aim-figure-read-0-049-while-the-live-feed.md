---
id: BUG-87
kind: bug
title: Nomination's re-aim figure read -0.049 while the live Feed-Catch Segment was refused at -0.208 (Dwell 0.32)
status: inbox
created: '2026-11-08'
updated: '2026-11-08T03:03:16Z'
owners: []
priority: medium
hardware: none
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-07-sitting-debrief.md
- map/bug-two-ball-throws-land-about-9-mm-toward-the-partner-site-an.md
filed_by: claude
---

At the Sitting of 2026-11-07 (Columns Apex 0.93 Sep. 125) the goal was accepted at nominal margin 0.189 and the printed re-aim figure was -0.049.

## What flew
- Live, the Segment after Ball 2's first Throw (the Catch of Ball 1, skill 2) was refused at -0.208, -0.155, -0.074 (Dwell 0.32) and -0.034, -0.029 (Dwell 0.30).
- Each refusal followed a Feed Catch whose lateral aim was clamped to -30 mm. The two Attempts with an unclamped Feed planned +0.116 and +0.054.
- So the corner walk in `nominate.py` (`_AimShifts`) reads above the live Feed case.

## Not settled
- Whether the fed schedule's first Segments are walked at all, or whether the splice timing of a late Feed re-aim adds the rest.
- Figure only: acceptance never uses it, so this is about what the operator is told, not a guard.

## Progress

- 2026-11-08: 2026-11-08 owner decision: fix this before anything else on the Feed clamp. The clamped Feed Catch aim (25 of 32 Attempts on 2026-11-07) costs about 0.11 of margin; above Dwell 0.27 it refused every later Attempt. Then check whether Feed misses lean one way (columns_feed_bb_bias_mm). Acceptance stays nominal.
