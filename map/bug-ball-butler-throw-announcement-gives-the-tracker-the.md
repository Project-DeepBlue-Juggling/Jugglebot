---
id: BUG-94
kind: bug
title: Ball Butler Throw announcement gives the Tracker the calibrated origin, not the release point
status: inbox
created: '2026-11-14'
updated: '2026-11-13T23:08:49Z'
owners: []
priority: high
diagnosis: hypothesis
area: tracking
blocked_by: []
links: []
parent: ball-butler
filed_by: claude
---

Ball Butler's Throw announcement tells the Tracker the ball starts at Ball Butler's calibrated origin, while the velocity beside it is the release velocity. Found by the 2026-11-13 maintainer review; the code was read and confirmed, the size of the effect was not computed.

## Findings

- `ball_butler_node.py:990` (`_publish_throw_announcement`) sets `ann.initial_position` from `self._bb_position_mm`, which `_on_bb_calibration` (`:400`) fills with the calibration position.
- `can/throw_ballistics.py:43` `bb_release_state` is documented as the single source of truth for where the ball leaves Ball Butler. It has no caller outside its own module.
- `tracking/matcher.py:176-189` records that the real ball sat 230-380 mm from the analytic position for the whole visible flight on 2026-10-14. That was answered by widening the announced gate and adding a motion test.

## Hypothesis

The offset between the origin and the true release point (the reviewer's estimate is 100-250 mm) explains part of that 230-380 mm. The rest would be the release timing, which has its own measured corrections (`BB_RELEASE_PUSH_LAG_S`, `BB_FLIGHT_BIAS_S`).

## Wanted

1. Offline first: replay recent Feed bags and compare the tracked ball against the analytic path from the origin and from `bb_release_state`. Report both residuals.
2. If the release point wins, announce it, then re-check whether the widened gate and the two timing corrections were partly absorbing the position error.
3. This changes what a live Feed Catch reads, so it needs a Sitting before it is trusted.
