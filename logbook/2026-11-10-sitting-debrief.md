---
title: "Sitting 2026-11-10 debrief: the sheet's three rehearsal steps flew as real Attempts on the default plant; turned Columns 45 deg COMPLETED 20/20, 90 deg and high-low failed, and the first Catch after the Feed (BUG-63) ended 7 of 11 two-ball fed Attempts"
type: investigation
date: 2026-11-10
status: complete
phase: "Sitting debrief"
related_plan: pattern-progression.md
related_issues: []
files_changed:
  - logbook/2026-11-10-sitting-debrief.md
  - logbook/2026-11-10-sitting-sheet.md
  - logbook/INDEX.md
  - map/idea-jp-9-high-low-columns-ball-1-at-apex-1-05-m-ball-2-at-0-95-m.md
  - map/idea-jp-10-turned-columns-the-columns-pair-at-fixed-angles-about.md
  - map/feature-p4-ramp-sitting-columns-canary-at-600-8000-and-20-a-then-the.md
  - map/bug-ball-2-s-first-throw-after-the-feed-catch-lands-about-180-mm.md
  - map/bug-two-ball-throws-land-about-9-mm-toward-the-partner-site-an.md
  - map/idea-improve-next-sitting-runsheet.md
  - map/idea-turned-columns-at-90-deg-tilt-the-platform-back-x-at-the.md
  - map/sheets/2026-11-10-robot-2.yaml
  - .claude/commands/sitting-sheet.md
  - docs/agents/map-writes.md
  - docs/project-map.md
  - docs/runbook.md
subsystem: [motion, tracking, map-improvements]
tags: [sitting, learner, testing]
---

# Sitting 2026-11-10 debrief

## Context

The first Sitting flown from a Sitting sheet (`map/sheets/2026-11-10-robot.yaml`), steps 1 to 8 only. Results: `map/sheets/2026-11-10-robot.results.yaml`, no Item verdicts saved; step 3 (bridge power-cycle) `skipped`, step 7 `problem` ("Throws seemed to be quite messy. No Attempts succeeded."), step 8 `done` ("45 deg passed perfectly, but 90 deg once again didn't succeed ... tilt the Platform slightly back (+ve x) when catching"). Steps 6 to 8 were written as dress rehearsals with Juggle goals; once Active those goals are accepted and fly, so the owner flew them as real Attempts (the class fix is 0f028a55, `2026-11-10-sitting-sheet.md`). Verdicts below are derived from the logs by this debrief, which ran as a single agent without the owner in the loop, so they are proposals the owner confirms.

Evidence: launch `~/.ros/log/2026-11-10-20-11-22-071098-jetson-2586385/launch.log`, bag `~/Desktop/rosbags/2026-11-10_20-11-22`. An earlier off-sheet launch (`2026-11-10-19-00-32-870246-jetson-2518801`, bag `2026-11-10_19-00-33`) flew five hop goals: four COMPLETED (4/4, 20/20, 20/20, 20/20) and one `LIMIT_JERK` at the THROW (219k > 200k, leg 5) at Apex 0.5, Sep. 150, a hop the nomination later rated at margin 0.065. The 23:34 launch was an unpowered nominate probe (no bag), used here only for its replies.

## Results

| Step / Item | Flown | Verdict | Killer |
|---|---|---|---|
| 6 / FEAT-39 (oval_1ball_fed 1.00/150/0.40) | 2 Attempts: MISSED_CATCH 2/2, done 4/4 (num_cycles 4) | no verdict: not the row's canary or 30-catch gate | first Catch of Ball 1, Cup empty |
| 7 / IDEA-10 (high-low 1.00/0.90, Sep. 100) | 3 Attempts: 0/0, 0/0, 2/3 | fail | Ball 2's first throw after the Feed outward (157, 23, 37 mm); Ball 1's hop-entry Catch missed twice with the fit 26 mm off; the third ended ABORTED_FAULT on HAND_JAM (iq -70.1 A, a ball under the ring, recovered) |
| (control) straight Columns 0.95/125 | 3 Attempts: 13/13, 0/0, COMPLETED 20/20 | n/a | Ball 2's first throw 196 mm outward; a 403 mm identity refusal at Throw 14 |
| 8 / IDEA-11 (turned 45 and 90 deg) | 45: COMPLETED 20/20. 90: 8/8, 3/3, 3/3, 3/3 | fail (45 met, 90 not) | 90: three of four ended at Ball 2's first Catch after the Feed, fit 47, 26, 14 mm outward along -y and 9-23 mm in -x; owner saw the ball hit the Platform top |

No `HAND_LIMIT` line all night. Nothing merged: no Item passed and no `night-shift/*` branch is held for these Items. No `ros_ws` change, so no rebuild. `docs/records.md` was clean.

## Which plant the Attempts flew on

All of tonight's goals flew on the default plant `jugglebot`. The fed patterns do not print `plant_id=` (only the 19:00 hop compiles did, `plant_id='jugglebot'`), but the one learner's row count rose through every goal of the 20:11 launch (4607 before the oval, 4646 after turned 45, 4676 after the last 90 deg Attempt), and `temp/learn/columns45`, `columns90` and `columns90-2` were last written on 2026-11-07. Of the 135 rows committed in a05787ae: 64 hop, 6 oval_1ball_fed, 2 high-low, 31 straight Columns, 19 turned 45 deg, 13 turned 90 deg.

Two Items meant to keep their rows apart: FEAT-39 (oval on `p4-oval-<date>`) and IDEA-11 (one plant per angle). **Nothing needs undoing.** Every row carries its Site, and the turned rows are the only ones off the x axis, so they separate by site angle and `t_abs_s` whenever a per-angle Memory is wanted. The default Memory already held 197 caught rows at 45 and 90 deg Sites from 2026-11-01, -03, -06 and -07, so tonight's 32 change nothing in kind. The learner's neighbourhood (k 16, release-position bandwidth 50 mm) puts the 45 deg Sites within one bandwidth of the straight Sites, but the straight Sites have thousands of exact-site rows, so their 16 nearest neighbours are unaffected; the borrowing runs the other way, into the turned flights. The per-angle `plant_id` step stays on IDEA-11 for the next Sitting.

## Discussion

**The rehearsal steps were flights, so they are read as Attempts, not as checks.** That reframes step 7's "problem" from "the rehearsal misbehaved" to "the high-low rung failed", and step 8 into a real turned-Columns result.

**The nominate probe's high-low margin (-0.245 with the Catch re-aim at its limit) is not what killed step 7.** It predicts Attempts ending on an admissibility refusal mid-Pattern. None did: one Catch re-aim was refused (`CUP_CONTACT_ACC`, Attempt 1, skill 3) and the Catch kept its schedule; every end was a missed Catch or the HAND_JAM. The margin is still lost headroom at Sep. 100 and stays on IDEA-10.

**What killed it is the class BUG-63 names, not a high-low property.** The first Catch after the Feed ended 7 of the 11 two-ball fed Attempts across three Patterns (high-low 3, straight 1, turned 90 deg 3). The owner's "messy throws" fits that and the two Ball 1 hop-entry misses on an empty Cup with the fit only 26 mm off, which look like contact (with Ball 2 or the rim) rather than aim; the logs cannot separate those, so it is recorded as cannot-tell.

**PWAY-2 P2(b) is answered offline, without per-angle plants.** Caught free-cup rows, landed minus commanded (y - u) in the pair frame: inward toward the partner Site 8.7 mm at 0 deg (n 2268), 12.7 at 45 (n 102), 14.8 at 90 (n 127); tangential -3.2, -2.2, +2.7 mm. A cause fixed in the robot frame would show tangentially at 90 deg and does not, so the pull turns with the pair, which is what BUG-79's arrival-transient mechanism predicts. y - u is a property of the plant, not of the learner's command, which is why mixing the angles in one Memory does not spoil it.

**The 90 deg Catch misses sit with the owner's tilt idea.** The 90 deg fits land outward and slightly in -x, and the owner saw the ball strike the Platform top; filed as IDEA-22 rather than folded into BUG-63, because it is a Catch-geometry change that a nominate probe can price offline.

## Write-backs

- IDEA-10, IDEA-11: Progress lines, status in_progress; IDEA-11's rehearsal line now says `columns_turned` needs mocap on the robot base and Ball Butler's Base markers (probe: `Ball Butler's position is unknown`), and the same rule went into `.claude/commands/sitting-sheet.md`, `docs/agents/map-writes.md` and step 7 of `map/sheets/2026-11-10-robot-2.yaml` (a "position is unknown" reply means bring mocap up and call again). Nothing has been saved against that sheet yet. `oval_1ball_fed` and `columns_highlow` nominated OK unpowered, so FEAT-39 and IDEA-10's "launch up and homed is enough" stands.
- FEAT-39, BUG-63, BUG-79: Progress notes with tonight's numbers.
- IDEA-19 complete (the sheet flew); `2026-11-10-sitting-sheet.md` complete.
- Filed IDEA-22: tilt the Platform back (+x) at the Catch for turned Columns at 90 deg (owner).
- For the owner: the unpowered `/skills/check` reported the Cup SEATED, so a ball is sitting in the Cup now.

## Verification

- `python -m mapkit sheet check map/sheets/2026-11-10-robot-2.yaml` (2026-11-10): `sheet check: OK`.
- Scoped tests: see the commit message (command, count).
