---
title: "Ball Butler Throw announcement gives the Tracker the release point, not the calibrated origin (BUG-94, unflown)"
type: bugfix
date: 2026-11-16
status: complete
phase: night-shift
subsystem: ball_butler
related_items: [BUG-94]
files_changed:
  - ros_ws/src/jugglebot/jugglebot/ball_butler_node.py
  - tests/ros/test_ball_butler_node.py
  - logbook/INDEX.md
---

# Ball Butler Throw announcement starts at the release point (BUG-94)

**What.** `_publish_throw_announcement` now sets `initial_position` from
`bb_release_state(sol.yaw_rad, sol.pitch_rad, sol.speed_mps, bb_position, yaw_offset)`
instead of the calibrated BB origin. If that call raises, it falls back to the origin and
warns once. (A missing calibration already refuses the Throw earlier, so the fallback is
only for an exception.)

**Why.** The announced velocity is the release velocity, so the start belongs at the release
point; at the Feed aim it is about 179 mm from the origin. Offline replay of 117 Ball Butler
Feed flights (7 bags, 6 Sittings, 2026-11-14): median residual at the announced time 288 mm
from the origin against 136 mm from the release point; the release point is closer on 117 of 117 (not re-run here: the figures are the Item's § Offline replay (2026-11-14), whose own Limits line names its untracked outputs).

**Unchanged on purpose.** Announced velocity (same `sol` yaw/pitch/speed, same formula as
`bb_release_state`'s velocity), `throw_time`, `landing_*` fields, the Tracker's 200 mm gate,
`BB_RELEASE_PUSH_LAG_S`, `BB_FLIGHT_BIAS_S`. The solver's Landing already uses the release
point, so the two timing constants were not absorbing the position error. Tightening the gate
is a separate call after a Sitting.

**Status: unflown.** This changes what a live Feed Catch reads; it needs a Sitting.

**Verification.** 2026-11-16, `pytest -q tests/ros/test_ball_butler_node.py tests/ros/test_ball_butler.py`
(through the Night Shift no-net wrapper): 107 passed. The new test
`test_announced_initial_position_is_the_release_point_not_the_origin` fails with the source
change reverted and passes with it.

**Follow-up 2026-11-17 (TKT-75, option 1): nonzero yaw offset.** The first test used a zero yaw
offset, so a doubled or missing offset would still pass. Added
`test_announced_release_point_applies_the_yaw_offset_exactly_once` (offset 0.3 rad, BB at
(100, -50, 20) mm, target rotated by the offset). Its expected release point is written out
from the geometry (`hw.BB_GEOM_*` constants and plain trig, no call into `bb_release_state`),
taking the world azimuth and pitch from the announced world velocity, plus a check that the
azimuth is the local bearing plus the offset once. Mutation proof, source restored after each:
doubling the offset passed to `bb_release_state` fails it (x 48.89 vs 59.04 expected); dropping
it (0.0) fails it (x 59.45 vs 59.04). No source change.
Verification, 2026-11-17: `pytest -q tests/ros/test_ball_butler_node.py tests/ros/test_ball_butler.py`
(no-net wrapper, nice 19): 108 passed.
