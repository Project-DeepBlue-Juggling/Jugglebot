---
id: IDEA-16
kind: idea
title: JP-17 Three-ball cascade with the owner as the second Hand, Platform held tilted toward them
status: in_progress
created: '2026-10-30'
updated: '2026-11-18T09:39:22Z'
owners: []
priority: medium
effort: xl
hardware: robot
owner: decision
radius: cross-cutting
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-02-jb113-human-throw-bench.md
- logbook/2026-11-13-sitting-debrief.md
- pathway-human-partner-juggling
filed_by: claude (jugglebot-f5, pattern exploration 2026-10-30)
night_skip: owner-session
night_skip_on: '2026-11-19'
night_skip_note: Promoted to PWAY-11 (plan human-partner-juggling.md); P1 and P2 on main, the rest is day-Session work and Sittings.
---

Owner 2026-10-30: a plain three-ball cascade, every throw at an equal Apex and timing, the Platform held tilted toward the owner the whole Attempt, catch and throw sites free.

## Timing

- Each ball's period is 3 Beats = flight + Dwell for both hands, so the robot's Dwell equals the owner's.
- The robot's empty gap 2 Beat - Dwell must clear about 0.30 s (transit-free if it catches and throws at one Site, which drops the floor toward 0.15 s).
- At a human-comfortable Dwell of 0.5 s that gives Beat 0.40-0.45 s, flight 0.70-0.85 s, apex 0.60-0.89 m.

## Pass distance at a held tilt

- Pass distance is g t_f^2 tan(theta) / 2: 0.51 m at 12 deg and flight 0.70, 0.75 m at flight 0.85; 9 deg gives 0.38 / 0.56 m.
- A cup tilted toward the owner faces the incoming ball squarely (its velocity reversed is the cup axis), so one held tilt serves both the throw and the catch.

## Blockers, in order

1. Perception of a human throw: `detect_human_throws` exists but is off and spawned 158 phantom tracks on a static marker 2026-09-15; R5 Block C registered 0 lobs 2026-09-30, undiagnosed; a human track has no throw_time so no identity latch resolves it.
2. A reactive CATCH anchored to the observed flight, as `compile_columns(feed=...)` anchors to the Ball Butler's release, with the robot's THROW time derived from the owner's tempo and the catch re-aimed after the fit.
3. A held-tilt planner mode (the aimed-throw reach inside a catch-and-throw window measured only 250-300 mm with the tilt acquired inside the window; untraced).

## First rung

A tracker diagnosis replaying the 2026-09-30 bag through the parabolic detector.

## Human-throw bench (2026-11-02)

Two human-only Sittings (robot moved aside) scored by `tools/probes/human_throw_bench.py`; detail in logbook `2026-11-02-jb113-human-throw-bench.md`.

| | passes, 91 s | cascade, 61 s |
|---|---|---|
| Throws (crossing the plane) | 44 (42) | 82 (71) |
| Good landing estimate (50 mm, 30 ms) 0.30 s before the plane | 24 of 42 | 41 of 71 |
| Good at any lead | 30 of 42, lead median 0.65 s | 55 of 71, lead median 0.56 s |
| Phantoms (upper bound: lost flights count) | 12.5 per minute | 27.3 per minute |
| Landing sd at the robot-spot hand | 75 x 125 mm | 129 x 90 mm |
| Landing vs mean of the previous five | median 140, p90 278 mm | median 108, p90 223 mm |

- **Tracks carry across throws**, and many failing rows hold a landing time 0.5-1.5 s early (hypothesis: the previous flight's frozen fit).
- Phantoms start on Ball Butler markers, on balls handled low, and near the Cup position at z 0.8 m.
- A hand-region gate passes 0 of 19 and 3 of 28 phantoms, mostly through its rising-start test (17 and 21 never rise); its retention of real Throws waits on the carried-track fix.

## Decisions (owner, 2026-11-02)

- The Pattern prior gates association (which marker, when) and confirms earlier through the announced-ball path; it never pulls the landing estimate.
- The Cup moves to the running Pattern-mean landing before the release and re-aims as the fit arrives; the final aim is the fit's.
- The owner expects tighter throws when juggling with the robot (better lighting, one hand); the bench re-measures at the next data Sitting.

## Carried-track fix (2026-11-03)

Confirmed: a ball carried through a hand kept its previous flight in its fit. A human ball's fit now starts fresh at the re-throw. Usable landing 0.3 s ahead: 24 to 37 of 42 passes, 41 to 51 of 71 cascade Throws; phantoms unchanged. Entry `logbook/2026-11-03-jb113-human-ball-fit-segments-flights.md`.

## Next

1. Low Throws: a human ball's fit admits only samples above the Cup's freeze floor (1080 mm), so Throws with apex under about 1160 mm never get a fit (15 of the cascade's 20 remaining failures). Decide a floor for human balls (their push is in a hand near z 450).
2. Phantom rejection and the hand-region gate, scored on the bench.
3. The predicted-Throw announcement (the Pattern prior).
4. Offline reach check: the Cup re-aiming over 150-300 mm in the last 0.5-0.8 s.

## Perception and reach (2026-11-18)

Read-only unit on the 2026-11-18 launch 3 bag (11 cascade Throws beside the robot, apex 1.85 to 2.07 m) and a re-run of the two 2026-11-02 bags. The replay used main's Tracker, so BUG-92's box was not applied. Now worked under PWAY-11, plan `plans/active/human-partner-juggling.md`.

| | 2026-11-18 cascade | 2026-11-02 cascade | 2026-11-02 passes |
|---|---|---|---|
| Good landing estimate (50 mm, 30 ms) 0.30 s before the plane | 10 of 11 | 51 of 71 | 37 of 42 |
| Lead of the first usable estimate, median | 0.71 s | 0.60 s | 0.72 s |
| Landing vs the mean of the previous five, median (p90) | 101 to 163 mm (122 to 387 mm) across the three bags | | |

- Phantoms outside BUG-92's box are static unlabelled markers: two high cells near (-898,-265,2150) and (-913,-254,2305), occupied in every frame, and a floor cell near (809,-398,74), occupied in 28 % of frames.
- A first-state speed of at least 500 mm/s rejects all 9 idle phantoms outside the box and loses 0 of 11, 1 of 44 and 4 of 82 real Throws. "Must rise" loses 16 of 82; a 600 mm height floor loses 15 of 44 and 26 of 82.
- The low-Throw floor is `flight_fit.py:141` (samples at or below 1080 mm are dropped from the landing fit). On 2026-11-02, 23 of 113 crossing Throws had an apex under 1160 mm and 19 of the 25 misses were among them; a floor of 1000, 950, 900 or 850 mm for human balls recovers at most 2, 5, 8 or 11.
- The Catch path takes only a Ball Butler announcement (`skill_node.py:6225`). A human track has landing position, velocity and time but no `throw_time`, so the announced-track latch (`ball_possession.py:2050`) cannot resolve it.
- Re-aim limits today: 35 mm inward, 10 mm outward, 30 mm across, two re-sends, none within 0.25 s of touch-down, at least 0.35 s of lead.
- Rough Cup travel, rest to rest under the Sitting limits less the 0.225 s splice lead: 96 mm at 0.50 s of lead, 156 mm at 0.60 s, 216 mm at 0.70 s (ignores the Jacobian, the contact window and the skid bound).

Not checked: a replay on the candidate Tracker, a replay with a lowered floor, what the static markers physically are.

## Sitting

- setup: level
- why: the three recorded bags are cascades and passes with several Balls in the air, so they cannot score a one-Ball Catch; this records single lobs at the Cup for the offline replay (plan step P3, data row).
- do:
  - Launch, home and Level the Platform; do NOT arm. Leave the Cup empty.
  - `ros2 param set /ball_tracker_node detect_human_throws true`.
  - Lob one Ball at the Cup about 30 times, one Ball in the air at a time, from where you would stand to juggle with the robot, at a height of about 1.8 m or more; let each Ball land in or beside the Cup and pick it up by hand before the next.
  - Afterwards `ros2 param set /ball_tracker_node detect_human_throws false`.
- pass: the bag replays through `tools/probes/human_catch_replay.py` and reports, for the lobs that crossed inside the Cup's reach, how many had an admitted plan and how far the planned Cup centre was from the real crossing.
- record: the bag path, where you stood, the lob count, and the replay report path.
- minutes: 10

## Sitting result (2026-11-13 13:12)

- verdict: pass
- notes: Done with no drops. (sheet 2026-11-13-robot, step 29)

## Sitting result (2026-11-13 13:13)

- verdict: pass
- notes: Done with no drops. I noticed in the GUI that there were a few 'predicted throws' that were actually just the Ball Butler frame markers. Perhaps the tolerance for a human throw ought to be tightened? (sheet 2026-11-13-robot, step 29)

## Progress

- 2026-11-03: Human-throw bench landed: two human-only Sittings scored, owner decisions recorded, data Sitting section written (detail below)
- 2026-11-13: Complete at the Sitting of 2026-11-13: bench report run offline (human_throw_bench.py, 130 s slice of the launch 3 bag): landing sd 73 x 101 mm at the Cup side, phantoms 13.8 per minute against 27.3 on 2026-11-02. The lighting change was not recorded. 26 of 30 phantoms are static Ball Butler frame markers (BUG-92).
- 2026-11-18: Reopened 2026-11-18 (owner): the 2026-11-13 debrief closed the whole Item on its bench re-measure row; the cascade itself has not been attempted. Open: the three Blockers (perception, a reactive Catch on the observed flight, the held-tilt mode) and the owner safety rules for a person inside the Platform reach.
- 2026-11-18: Promoted to Pathway PWAY-11 (plan human-partner-juggling.md): perception and reach numbers from the 2026-11-18 bag are in the body; the owner decided no safety rules are needed and set the ladder (one Ball caught, caught and thrown back, held tilt, three Balls).
- 2026-11-18: P1 and P2 of plan human-partner-juggling.md are built, audited and on main (Pattern human_catch, unflown). The Sitting section is now the single-lob data row. Owner 2026-11-18: the person is assumed not to drop; a Ball is dropped only when neither tracked nor felt by the Hand.
