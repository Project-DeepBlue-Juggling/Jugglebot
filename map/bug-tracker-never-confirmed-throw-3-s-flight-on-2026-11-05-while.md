---
id: JB-181
kind: bug
title: Tracker never confirmed Throw 3's flight on 2026-11-05 while the previous track kept running past its Catch, so that Throw has no learner row
status: inbox
created: '2026-11-06'
updated: '2026-11-05T19:55:23Z'
owners: []
priority: medium
blocked_by: []
links: []
filed_by: claude
---

Found by the Night Shift on 2026-11-06 while reading JB-176's bag (2026-11-05 22:46, the first fed Attempt). Throw 3 got no learner row because the Tracker never confirmed its flight, not because of the identity gate.

## Findings

- Tracker id 4 (Throw 3) has no `tracker confirm ball 4` line in the launch log. Its status stays 0 or 1, tracking 0, never `from_fit`; at 354.96 its status goes to 4.
- Meanwhile id 3 (Throw 2's track) stayed tracking past its Catch with `from_fit` false, so the only estimate for that flight was unfitted.
- `_consider_landing` rejected every estimate for lack of a converged fit.
- Throws 1, 2, 7 and 9 had their own id confirmed and a fit before the crossing.

## Open

- Whether the previous track kept the marker, or the new flight's detection failed some other way. Replay the `/balls` and mocap topics of that bag through `ball_tracker_node`'s pure logic.
