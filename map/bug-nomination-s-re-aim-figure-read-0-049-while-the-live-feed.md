---
id: BUG-87
kind: bug
title: Nomination's re-aim figure read -0.049 while the live Feed-Catch Segment was refused at -0.208 (Dwell 0.32)
status: backlog
created: '2026-11-08'
updated: '2026-11-08T17:57:50Z'
owners: []
priority: medium
effort: m
hardware: none
owner: none
radius: local
diagnosis: hypothesis
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

## Findings (2026-11-09)

Night Shift, one unit, offline replay with `nominate.py` (throwaway probes, deleted). Columns Apex 0.93, Sep. 125, Dwell 0.32, hop entry, launch-default limits. **Not settled; no code changed.**

### What reproduces
- The printed -0.049 reproduces exactly: across sign -1, Segment idx 2 (the Catch of Ball 1, then its Throw). The other across sign reads +0.061, and the nominal is 0.189, as in the log.
- The fed Segments are walked: idx 0 (hop-entry Throw), idx 1 (the Feed Catch and its folded Throw) and idx 2. The "not walked at all" reading in this Item is refuted.

### What does not reproduce
| Input | Offline margin | Live margin |
|---|---|---|
| Feed clamped at (-10, -30), `u` (+15.3, +10.0), Apex 0.862 | +0.181 | -0.208 |
| Feed unclamped (+11.4, +5.3), `u` (+11.3, +10.0) | +0.198 | +0.116 |

- The clamp costs about 0.32 live and about 0.02 offline.
- A grid over Feed and second-Catch corners at both Dwells never reads below -0.049.
- Feed landing time (+-40 ms) and arrival velocity (+-200 mm/s) move idx 2 by nothing at 3 decimals: it depends only on the Feed's release state.

### Lead (not proven)
- The walk seems the wrong place to fix this. The gap looks to be in how the live executor builds the Feed Catch's release state, which the next Segment is seeded from. The log shows the Feed spliced at Knot 17 after a rebased Fresh-origin.

### Next
- A bag-reading unit: take skill 1's live release state from the 2026-11-07 16:15 bag, plan skill 2 from it, compare with the nominal idx 1 release state. Effort rated up to m for that reason.

## Progress

- 2026-11-08: 2026-11-08 owner decision: fix this before anything else on the Feed clamp. The clamped Feed Catch aim (25 of 32 Attempts on 2026-11-07) costs about 0.11 of margin; above Dwell 0.27 it refused every later Attempt. Then check whether Feed misses lean one way (columns_feed_bb_bias_mm). Acceptance stays nominal.
- 2026-11-09: Night Shift 2026-11-09: -0.049 reproduces and the fed Segments are walked; the live -0.208 does not, so no code changed; next is a bag read of the Feed's release state
