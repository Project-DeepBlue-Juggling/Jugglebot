---
id: BUG-92
kind: bug
title: static Ball Butler frame markers are tracked as human_throw
status: complete
created: '2026-11-13'
updated: '2026-11-18T05:11:29Z'
owners:
- night-shift
priority: medium
effort: s
hardware: robot
owner: decision
radius: subsystem
diagnosis: confirmed
area: tracking
blocked_by: []
links:
- map/idea-jp-17-three-ball-cascade-with-the-owner-as-the-second-hand.md
- logbook/2026-11-18-sitting-debrief.md
- logbook/2026-11-15-human-throw-ball-butler-zone.md
filed_by: claude
---

Static Ball Butler frame markers are tracked as `human_throw` tracks. The operator saw it at the Sitting of 2026-11-13 (step 29, "a few predicted throws were the Ball Butler frame markers").

## Numbers (launch 3 bag, 13:10:40 to 13:12:50)

- 12 of 124 live `/balls` human_throw tracks start within 40 mm of a static marker (about 6.4 per minute).
- 26 of the 30 offline bench phantoms start on the same static cells (x -730 to -988 mm, z 1665 to 1861).
- Ball Butler markers `Ball Butler - 1..7` sit at fixed positions (sd 0.1 to 0.4 mm, present in 99 % of frames), plus four unlabelled static markers. `excluded_label_prefixes` cannot name `Ball Butler`, because QTM labels the flying ball `Ball Butler - 1`.
- A tolerance on the parabola would not help: a static marker never rises.

## Proposed rule (owner decision)

Exclude a human_throw candidate whose first sample lies within about 40 mm of a static cell: a 10 mm cell that an eligible marker occupied in more than 90 % of the last 30 to 120 s of frames. It needs no label and no zone list.

- On the launch 3 slice it removes 26 of 30 bench phantoms (13.8 to 1.8 per minute), 12 of 124 live tracks, and 0 of 114 true human Throws.
- Not checked: Ball Butler own announced Throws (they use `/throw_announcements`), a ball held still in a hand for over a second, the 4 remaining non-static phantoms, other bags.

## Options (2026-11-14)

1. **Build the proposed rule, on by default:** a human_throw candidate whose first sample is within 40 mm of a static 10 mm cell (occupied over 90 % of the last 30 to 120 s) is dropped; a parameter turns it off. Tests on synthetic markers; a Sitting checks the phantom rate and that no real Throw is lost.
2. **The same rule, off by default:** a Sitting turns it on for an A/B with `detect_human_throws`.
3. **A zone in config:** drop starts inside the Ball Butler frame box (x -730 to -988, z 1665 to 1861 mm). Simpler; breaks when Ball Butler moves.
4. **Leave it:** `detect_human_throws` is off by default, so phantoms only appear in human-Throw windows.

Recommendation: 1. On the launch 3 slice it removed 26 of 30 bench phantoms and 0 of 114 true Throws, and it needs no label or zone. FEAT-50 builds a rolling static-cell filter tonight that this could share.

## Sitting

- setup: level
- why: The held branch drops human_throw starts inside a configured Ball Butler frame box (TKT-71 option 3); only a live mocap window shows the phantom rate falls and no real human Throw is lost.
- do:
  - Launch from the branch with mocap up and Ball Butler in its usual place.
  - `ros2 param set /ball_tracker_node detect_human_throws true`
  - Leave the cell idle for 2 minutes, nothing thrown.
  - Make 10 human Throws from your usual spot, away from Ball Butler.
- pass: No human_throw track starts inside the box during the idle 2 minutes, and all 10 human Throws are tracked.
- record: human_throw tracks started during the idle window (and where), human Throws tracked out of 10, the launch log time.
- minutes: 10
- branch: night-shift/2026-11-15-BUG-92

## Sitting result (2026-11-18 13:05)

- verdict: pass
- notes: Made 11 human throws in a three ball cascade (sheet 2026-11-18-robot, step 30) [derived: Bag 2026-11-18_11-31-37 and temp/logs/launch_2026-11-18_3.log: 0 human-Throw tracks started inside the Ball Butler box between detect on (11:34:18.9) and the first human Throw (about 11:36:45); 12 tracks for the 11 Throws (one likely a split track). 8 phantom tracks still started outside the box (4 at a floor cell, 4 near x -900, z 2150-2305).]

## Progress

- 2026-11-15: Night Shift 2026-11-15: held on night-shift/2026-11-15-BUG-92: config is gate-frozen, suite not finished under load, 2 test-coverage findings open; unblocked by a day Session merge and a Sitting window
- 2026-11-18: Passed at the Sitting of 2026-11-18: 0 tracks started inside the box when idle, all 11 human Throws tracked; 8 phantoms still start outside it (a floor cell, and near x -900, z 2150-2305). Not merged: the branch touches config/hardware_config.yaml (frozen path), so tools/sitting_merge.sh refuses it (code 4); needs a hand merge at the owner say-so.
- 2026-11-18: Passed at the Sitting of 2026-11-18 and merged by hand at the owner say-so (frozen config paths, so not through tools/sitting_merge.sh): full suite green on the merged tree, audit 0 behaviour-affecting; the four box constants reach the firmware headers but no firmware reads them, no flash. Open follow-up: phantoms outside the box (PWAY-11 P1).
