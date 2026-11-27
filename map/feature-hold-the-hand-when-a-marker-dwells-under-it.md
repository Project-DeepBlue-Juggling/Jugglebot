---
id: FEAT-71
kind: feature
title: Hold the Hand when a marker dwells under it
status: inbox
created: '2026-11-27'
updated: '2026-11-27T04:13:12Z'
owners: []
priority: medium
effort: s
hardware: robot
owner: decision
radius: subsystem
area: hand
blocked_by: []
links:
- bug-hand-stalls-at-the-70-a-clamp-mid-dive-probably-on-a-ball
- feature-live-warning-when-a-marker-dwells-under-the-hand
filed_by: claude
---

At the Sitting of 2026-11-27 (14:04:14) a missed Columns Ball rolled under the Hand and the Hand stalled on it at 72.6 A peak. The live warning (FEAT-50) had fired 0.16 s earlier, at a dwell of 413 ms, and nothing acts on it. The Hand motor was overheated on 2026-11-20, so each stall matters.

## Wanted

- Owner decision first: should a marker dwelling under the Hand end the Attempt and hold the Hand where it is (or refuse its next downward move)? It changes what the Hand does mid-Attempt, so it is not made without that decision.
- Numbers for the decision: this event gave 0.16 s between the warning and the stall; the warning's dwell threshold decides how much earlier it could come; the false-warning rate over past bags is not yet counted (trapped_ball_finder lists every UNDER_HAND dwell per bag).

## Sitting

- setup: level, armed, bb-fed
- why: only a real Ball under the Hand shows the hold acts in time.
- do:
  - Not flyable until the decision is made and the hold is built; this section is then rewritten with its steps.
- pass: to be written with the build.
- record: to be written with the build.
- minutes: 10
