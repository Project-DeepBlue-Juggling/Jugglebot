---
id: BUG-94
kind: bug
title: Ball Butler Throw announcement gives the Tracker the calibrated origin, not the release point
status: inbox
created: '2026-11-14'
updated: '2026-11-14T02:52:54Z'
owners: []
priority: high
diagnosis: confirmed
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

## Offline replay (2026-11-14)

**The release point is right and the origin is wrong, on every flight measured.** 117 Ball Butler Feed flights from 7 bags across 6 Sittings (2026-10-12 to 2026-11-13), raw unlabelled mocap markers, flights selected inside a tube centred midway between the two candidate start points so neither is favoured.

| Measure (median) | From the origin | From the release point |
| --- | --- | --- |
| Residual at the announced time | 288 mm | 136 mm |
| Cross-path part of it | 147 mm | 58 mm |
| Closest approach of the fitted flight to the start point (no timing in it) | 105 mm | 5.7 mm |
| Extra gate radius to admit the first 0.2 s, 95th percentile | 212 mm | 61 mm |

- The release point is closer on 117 of 117 flights; the margin (155 mm) is about ten times its scatter.
- At the Feed aim (pitch 74 deg, yaw 7.8 deg) the release point is 179 mm from the origin: about 139 mm along the flight and 113 mm across it.
- The 5.7 mm closest approach is an independent check of `bb_release_state`: the calibration comes from the body-marker yaw sweep, not from ball flights.
- The 230-380 mm in the matcher comment splits into about 152 mm of start position, about 70 mm of speed and timing, and about 65 mm of pitch, drag and fit error. The split depends on the order applied.

## What it does not explain

The real ball passes the release point about 66 ms before `throw_time`, about 12.5 % faster than announced and 2.7 deg steeper. That is a separate effect and stays after this cure. `BB_RELEASE_PUSH_LAG_S` and `BB_FLIGHT_BIAS_S` were fitted at the Catch plane against `landing_time`, and the solver's Landing already uses the release point, so they were not absorbing the position error and should not be retuned for it.

## Consequences

- The announcement contradicts itself today: a path from the origin misses its own `landing_position` by 179 mm at `landing_time`.
- Today's 200 mm announced gate is marginal under the origin (212 mm needed at the 95th percentile). With the release point about 100 mm would do, though only one aim was flown.
- BUG-92 (static Ball Butler markers taken for a throw) and BUG-70 are worth re-reading with a tighter gate in mind.

## Limits

One aim only; other aims rest on the geometry. Drag ignored. 14 flights were first seen near the Apex. Probe and tables: `temp/probes/bug94_announce_origin/` on the Jetson (untracked).
