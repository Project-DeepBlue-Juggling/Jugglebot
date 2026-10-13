---
title: R5 sitting 7 - two-ball columns PB 15 catches; misses are a +x launch lean (trim lost on relaunch) the learner cannot correct inside the columns box, and the "hand stopped" was a latched LEG lane
type: investigation
date: 2026-10-13
status: resolved
phase: "two-ball-skill-stack — R5 (two-ball columns)"
related_plan: two-ball-skill-stack.md
related:
  - 2026-10-12-skill-stack-r5-bb-feed-recentre-after-absolute-release.md
  - 2026-10-04-skill-stack-r5-sitting-4.md
  - 2026-09-18-opening-rest-homes-the-hand.md
  - 2026-09-23-kinematic-calibration-capture-tool.md
files_changed:
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - tests/ros/test_skill_node.py
  - tests/motion/test_skills_report.py
  - logbook/2026-10-13-skill-stack-r5-sitting-7-columns-pb.md
  - logbook/INDEX.md
subsystem:
  - skill-stack
  - can-bridge
tags:
  - hardware-sitting
  - learner
  - levelling
  - safety
---

# R5 sitting 7: columns PB 15, and why the attempts end

## What happened

Runsheet `tests/hardware/session_skills_r5_sitting7.md`, two launches:
- **L1** 23:24 (`~/Desktop/rosbags/2026-10-12_23-24-02`): fed-feed misses, fixed mid-sitting by `2933ffac` (separate entry).
- **L2** 00:18 (`~/Desktop/rosbags/2026-10-13_00-18-46`, launch.log `~/.ros/log/2026-10-13-00-18-46-323618-jetson-3638285`): two-ball `columns`, hop entry, apex 0.95, sep 125, BB-fed.

**Learning curve** (L2, catches per attempt, in order; every attempt ends at its first miss, so each count is consecutive):
3/4 (4-throw goal, completed), 6, 5, 3, 2 (LIMIT_JERK), 9/10 (10-throw goal, completed), 5, 11, 2 (HAND_LANE_REFUSED), 0 (refused again, same latch), 2, 7, **15 (PB)**, 3.

Terminal causes (12 ends): MISSED_CATCH 9, LIMIT_JERK 1, lane latch 2.
- 5 of the 9 misses: the ball reached the cup 32-48 mm off-centre, on the side toward P2. All five were Ball 2 at P1. The cup never read SEATED, and the next stroke flung the ball.
- 1: a hop catch at P2 that was 38 mm off and aimed open-loop.
- 2: wild throws, -210..-235 mm in y.
- 1: mocap lost the ball, so the cause is unknown.

Health was clean:
- Leg current peaked at 12.9 A against the 15 A clamp. FETs stayed under 35 °C.
- There were no `hand_jam` events.
- Bridge link: 100 % OK.
- BB: 16/16 feeds caught.

Analysis scripts and per-throw CSVs (`l2_throws_joined.csv`, `l2_flights.csv`, `hlr/dump.py`) were scratch only, not committed.

## Discussion

**"Throws go inward" is reframed as one +x lean shared by both columns.**
- Bag parabola fits: P1 throws drift +31 mm in x after release; P2 throws drift +13 mm.
- So P1 lands inward (dx +27, sd 15, n=26) and P2 lands slightly *outward* (dx +9, sd 23, n=31).
- The launch point does not predict the landing (r = -0.07), and neither does the catch re-aim's release offset (slope 0.03).
- That rules out ball-in-cup offset, re-aim drag and transit velocity as the cause.

**Root cause, part 1: the level trim was not applied in L2.**
- `trajectory_node` logged `level_trim_deg = [+0.0000, +0.0000]` and it was never set afterwards. L1 had applied runsheet row 11's `[0.0945, 0.3678]`, but the relaunch dropped it.
- The y component, 0.368°, predicts about 22 mm of x drift over a 0.95 m flight. Measured with it applied, P1's columns bias was **+14 mm** (sitting 7b, n=22); without it, **+30 mm** (L2, n=28). P2 went from +7 to +11 mm.
- The sign is right, and it removes about half of P1's bias. **Fix: the trim is now a launch default.**

**Root cause, part 2: the learner sees the bias but the columns box clips its correction.**
- At apex 0.95, `config/generated/admissible_box.yaml` (gate `5072c89d4ddc`) admits columns landing offsets of only x ∈ [-4, +2] mm (P1) and [-2, +4] mm (P2).
- The learner's unclipped command was P1 dx -23.9 mm and P2 dx -10.0 mm, so every columns throw flew with u_dx pinned at the edge.
- In self_toss, whose box spans ±30-40 mm, the same learner centres the landings.
- Widening the box needs a re-sweep and runs into the LIMIT_* margin class, so it is an owner decision, not done here.

**The lane latch: the "hand stopped near the top" was the LEG lane.**
1. During a 135 ms CATCH solve (59-81 ms is typical) the 40 Hz emitter stalled for 41-46 ms, and the bridge's own 100 Hz timer had a 57 ms gap. That points to CPU contention across the box; the source is not established.
2. With `JB_TRAJ_EMIT_LEAD_KNOTS = 0` there is about 45 ms of slack, so the cover (2T + grace = 70 ms) ran out for both groups.
3. The near-still hand resumed within tolerance and the throw flew. The legs, moving at 0.4 rev/s against the 0.3 rps `SCHED_RESUME_TOL` vel, were refused and latched (`leg_interp.cpp` `sched_apply`). That is correct firmware behaviour.
4. The Jetson then:
   - mislabelled it HAND_LANE_REFUSED, because `sched_refused` is one counter for both groups;
   - forced a hold, which has no hand frames, and so stopped the hand at 8.68 rev on its way down;
   - accepted the next goal, which ended 0.5 s later on the same latch, after BB had already been dispatched. BB threw at the frozen robot 6.8 s later.
5. The operator's `go_home` was also refused and walked the guard residual to 0.37 rev of the 1.0 rev MAX_DEVIATION limit before the disarm cleared it.

This is the same class as the 2026-09-23 kincal latch ("carried, not fixed"). It is not related to FW 29.

**Considered and not done here (owner decisions):**
- widen the columns box;
- aim no-fit catches at the learner's expected landing (28 NO-CONVERGED-FIT lines; every hop catch was open-loop);
- split the learner's lateral authority from the catch re-aim limit;
- refuse `go_home`/`hold` while latched;
- auto-recover via disarm/re-seed/re-arm;
- EMIT_LEAD 0 → 1 knot;
- a BB throw-cancel command (firmware + protocol);
- profile the 45-57 ms stall.

## Fix

1. `jugglebot_launch.py`: `level_trim_deg: [0.0945, 0.3678]` is now a `trajectory_node` launch parameter. It is the sitting-4 measurement and is still overridable live. This needs a `colcon build` of `jugglebot` to reach the installed launch file.
2. Lane-latch refusal at goal start:
   - `teensy_bridge_node` publishes heartbeat bit 15 as `/link_status` `sched_hold_latched`.
   - `skill_node._refuse_if_lane_latched` refuses a Juggle goal, before any segment or `bb/throw_at_target`, while that bit is set or while `sched_refused` has risen within 1 s.
   - A stale `/link_status` does not refuse.
3. Text only: the HAND_LANE_REFUSED messages now say "a streamed lane (legs or hand)". The code name is unchanged.

## Verification

- Scoped, in an isolated worktree of HEAD plus only these hunks (2026-10-13), `pytest tests/ros/test_skill_node.py -k "lane or link_status or sched or refused"`: **41 passed**, including the four new latch tests.
- Full gate: see the commit message's triple.

## Follow-up

The owner decisions listed under "Considered and not done here" were answered the same day by wave 2: `2026-10-13-skill-stack-r5-wave-2-after-sitting-7.md` (status open; the box-widening choice, the stall fix and the park-at-DONE question remain with the owner). This entry stays open for the same reason.

**Follow-up (2026-10-13 night).** Sitting 8 met the R5 gate (`2026-10-13-skill-stack-r5-sitting-8-gate-met.md`); this entry waited on the box choice, the stall fix and park-at-DONE: all three were decided (option 4 and the solver worker landed in wave 3, no park at DONE) and sitting 8 flew them; the remaining items are rows in `plans/active/skill-stack-dress-rehearsals-and-residue.md` § 2.
