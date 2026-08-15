---
title: Toss gates follow the live session limits — operator authority over toss width
type: feature
date: 2026-08-14
status: resolved
phase: "single-ball-toss — widening (post-Phase F)"
related_plan: single-ball-toss.md
subsystem:
  - ros
  - motion
tags:
  - control
  - config
---

# Toss gates follow the live session limits — operator authority over toss width

**What.** Three changes that make toss width an operator decision instead of a
constant hunt, per the owner's 2026-08-14 direction ("limits set by ROS2 calls
should be the primary values that dictate acceptance of new movements"):

1. **The closed-form reach bound now judges against the LIVE session limits.**
   `trajectory/status` gains `leg_vel_limit_mmps` / `leg_acc_limit_mmps2` /
   `leg_jerk_limit_mmps3` (the values `feasibility.validate` is enforcing right
   now, i.e. the YAML working point as ramped by `trajectory/set_limits`);
   `reload_coordinator_node` caches them (same freshness window as
   `platform_levelled`, getattr-degrading on an old publisher) and threads them
   into `TossObservations`; `toss_sequencer`'s `REJECTED_DISPLACEMENT` reach
   bound prefers them, falling back to the YAML-default module copies when
   absent (0.0) — bit-for-bit the old behaviour. So a `set_limits` ramp-UP
   stops refusing reaches the planner would fly, and a ramp-DOWN refuses
   pre-throw instead of `TOO_FAST`-ing at `t_release` with the ball airborne
   (the C-REACH-1 mid-flight-verdict class; the drift hazard the old
   "runtime set_limits drift is possible" caveat documented is now closed for
   a running system).
2. **The ±150 mm planning box is config-keyed**: `toss_workspace_xy_mm`
   (default **160** = cap + 10), replacing the hardcoded `TOSS_XY_LIMIT_MM`
   that made raising `toss_max_displacement_mm` past 150 inert
   (`REJECTED_WORKSPACE` caught the same goal one line later). Box > cap ×
   1.03 also **dissolves the chaining known-limitation**: the caught-toss park
   places the centroid 2.07 % of displacement outside B, so a box equal to the
   cap refused chains at the cap edge (frontier 146.5/147.0 mm); the parked
   centroid now sits inside the box and DISP-5/DISP-6 run as written. The
   centroid-vs-cup frame question stays open but no longer gates.
3. **The displacement cap is genuinely operator-adjustable**: the drift-guard
   test now pins the MECHANISM (the node passes the YAML value into the ctor;
   the ctor value is what gates) plus the box ≥ cap × 1.03 relational
   invariant, instead of YAML == module-literal equality — the pin that made
   any YAML edit turn the suite red.

**Deliberate tradeoff.** The module literals stay as standalone/bag fallbacks
and are no longer forced equal to YAML for the two policy keys; the fallback
`REACH_*` copies remain pinned to the YAML working point (the fallback path can
still over-permit exactly the way the old copies did — that pin closes it).
Safety posture unchanged: the box and cap are POLICY; the planner's feasibility
gate, the 0.3 rev step gate, and the firmware stroke clamp + `MAX_DEVIATION`
remain the authorities. Also rides in this commit (owner-ratified 2026-08-14):
the `toss_tier` working-tree flip `8a → 8b`.

**Deployment note.** The message change means the next deployment needs the
two-package build: `colcon build --packages-select jugglebot_interfaces
jugglebot` + relaunch (the DISP-0 lesson).

**Verification.**
- 2026-08-14, `pytest tests/ros/test_toss_sequencer.py
  tests/ros/test_toss_coordinator.py tests/ros/test_toss_continuous_node.py
  tests/ros/test_trajectory_node.py tests/sim/test_toss_gate.py -q`:
  **472/472 pass in 64.54 s** (includes the new live-limits widen/narrow/absent
  battery, the ctor-box test, the status-publisher ramp test, the
  coordinator cache/expiry test, and the re-pointed chaining test).
- 2026-08-14, `./run_tests.sh`: **RESULT: PASS** (parallel 237 s rc=0; serial
  phase empty rc=0). One transient single-test failure in the first gate run
  (`test_a_stale_interfaces_build_names_itself_at_construction`) did not
  reproduce scoped, whole-file, or under the gate's xdist config — consistent
  with the parallel bridge session writing `teensy_bridge_node.py` (a file
  that test inspects at source level) mid-run.
- Commit-time gate (after the 2026-08-15 audit-fix edits): covered by the
  2026-08-15 `./run_tests.sh --full` PASS recorded in
  `logbook/2026-08-15-plans-board-cleanup.md` (both commits were cut from that
  verified tree with no intervening code edits).
