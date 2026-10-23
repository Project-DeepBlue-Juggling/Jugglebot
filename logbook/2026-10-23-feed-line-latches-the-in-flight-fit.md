---
title: "The feed line latches the in-flight fit: all 21 Ball-Butler feed lines printed -- at the 2026-10-22 sitting"
type: bugfix
date: 2026-10-23
status: in-progress
phase: "two-ball-skill-stack - post-R6 (row 49 follow-up)"
related_plan: two-ball-oval.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - tests/motion/test_skills_executor.py
subsystem:
  - skills
---

## What changed

Row 49 (the catch-time A/B, `2026-10-22-catch-observation-and-catch-time-offset.md`)
was designed around the new `feed ball N CAUGHT ... contact / settle` line, and every
one of the 21 feed lines at the 2026-10-22 19:50 sitting (21 feed lines against 28 BB-fed balls in the bag; the 7 without a line registered no seated feed catch -- not reconciled per ball) printed
`arrival -- seat -- contact -- settle --`. The settle numbers for that row had to be
re-derived from the bag.

Cause (bag + code read, 2026-10-23): `_report_feed` ran at the scheduled landing
+ 0.53 s (the catch's `t_close`) and asked the tracker provider for the fed ball's
landing *then*. `skill_node._tracker` answers only while the ball's status is
IN_FLIGHT and tracking CONFIRMED; in the bag the fed balls (57, 62, 88) left IN_FLIGHT
0.29-0.50 s after the scheduled landing, before the report, so the call returned None.
The in-flight fit had been good the whole time (`from_fit`, landing within 7-12 ms of
the schedule, "tracker landing 2.1 mm / -0.002 s from the committed aim" in the log).
The throw line never had the race because `_consider_landing` polls every tick from
release and keeps the best landing on the pending catch while the ball flies.

Fix: `_PendingCatch` gains `last_fit_landing`. `_tracked_landing`, the one tracker poll
the dispatch aim, the resend and the bump route already share, stores any validated
`from_fit` landing on the pending catch (a fit taken at dispatch, before
`_register_catch`, is parked by catch index and copied onto the pend at registration).
`_report_feed` reads the latch first and calls the provider only when there is none;
the 0.3 s sanity window against the scheduled landing is unchanged. Report-only
plumbing: no timing, dispatch, aim or learner behaviour changes. One tick: the resend
or bump poll at about landing - 0.4 s sees the ball IN_FLIGHT with a converged fit and
stores it; polling stops at `catch_freeze_s`; at landing + 0.53 s the provider is
quiet but the report reads the stored fit, so arrival, seat, contact and settle all
resolve.

Known limits: the latch is the last fit taken at an aim or resend poll, roughly
0.25-0.4 s before landing, not the final fit; a feed with no usable fit at dispatch
and no later poll still prints `--`.

## Verification

- 2026-10-23, `pytest tests/motion/test_skills_executor.py tests/motion/test_skills_report.py tests/ros/test_skill_node.py -q` -> 647 passed in 56.59 s. New tests: `test_feed_line_uses_the_in_flight_fit_after_the_provider_goes_quiet` (fails with the latch disabled) and `test_feed_line_still_rejects_a_stale_latched_fit`.
- Full gate: 2026-10-23, `./run_tests.sh --full` (BLAS single-thread) -> 7025 passed, 9 skipped in 314.81 s + serial tail 7 passed, exit 0 (`temp/logs/gate_merge_20261023.log`); run over this fix and the sitting records together.
- Hardware: NOT FLOWN. The next Ball-Butler-fed goal's `feed ball` lines are the check; they should carry numbers for every seated feed.
