---
id: IDEA-16
kind: idea
title: JP-17 Three-ball cascade with the owner as the second Hand, Platform held tilted toward them
status: in_progress
created: '2026-10-30'
updated: '2026-11-27T08:59:11Z'
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
- logbook/2026-11-20-sitting-debrief.md
filed_by: claude (jugglebot-f5, pattern exploration 2026-10-30)
night_skip: owner-session
night_skip_on: '2026-11-27'
night_skip_note: PWAY-11 day-Session work; aimed reach from a level rest is capped by the Hand C2 refusal (BUG-146).
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

- setup: level, armed
- why: plan step P3, the first robot Catch of a human Throw: one Ball at a time lobbed at the Cup, the Platform Level and Active, the robot Catches and holds. Flown as a first data Attempt (owner, 2026-11-27): the lob replay predicts about 5 to 9 Catches of 14 lobs in reach, so the gate is scored but not expected to pass.
- do:
  - Launch, home and Level the Platform. Cup empty; check the GUI shows the Hand ball sensor answering before any lob.
  - Rehearse, nothing moves: `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'human_catch'}"`; the reply must read success: True.
  - `ros2 param set /ball_tracker_node detect_human_throws true`
  - Activate.
  - `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: human_catch, num_cycles: 15}" --feedback`
  - From where you stood on 2026-11-27, lob one Ball at the Cup, Apex about 1.8 m or more, one Ball in the air at a time. After a Catch take the Ball out of the Cup by hand; after a miss pick it up. Wait until the Platform is back at rest before the next lob. Count every lob.
  - The goal ends by itself after 15 planned Throws; Ctrl-C on the goal stops it early. If the Platform moves in a way you do not expect, press the E-stop button.
  - `ros2 param set /ball_tracker_node detect_human_throws false`
- pass: the goal's result line reads "human_catch result: C of 15 human Throws caught" with C at least 10, and the debrief's replay of the bag finds no Catch planned on a phantom.
- record: the result line word for word, the number of lobs you threw, where you stood, the start and end clock times, and the bag directory.
- minutes: 20

## Sitting result (2026-11-13 13:12)

- verdict: pass
- notes: Done with no drops. (sheet 2026-11-13-robot, step 29)

## Sitting result (2026-11-13 13:13)

- verdict: pass
- notes: Done with no drops. I noticed in the GUI that there were a few 'predicted throws' that were actually just the Ball Butler frame markers. Perhaps the tolerance for a human throw ought to be tightened? (sheet 2026-11-13-robot, step 29)

## Sitting result (2026-11-20 15:11)

- verdict: data
- notes (split into bullets):
  - Started at 12:52 and finished at 12:56. I wasn't keeping track of how many throws I made, but I think it was around 40-50. Only the very first throw landed in the cup. After that one, I realised it would be difficult for me to reliably throw into the cup, so I caught the rest by hand, near where the cup usually sits (though I dropped a few) (sheet 2026-11-20-robot, step 7) [derived:
  - temp/probes/human_catch_2026-11-20/2026-11-20_12-49-25_replay.md (bag 2026-11-20_12-49-25, 12:52-12:57): the replay ran and reported 55 lobs, 49 crossing the Catch plane, 23 inside the Cup's reach, 16 of those with an admitted plan; planned Cup centre to real crossing median 30.5 mm, worst 52.0 mm. Derived as data, not pass: this is the plan's data row, and most Balls were caught by hand above the Cup, which the replay does not model.]

## Sitting result (2026-11-27 15:08)

- verdict: pass
- notes: Threw approximately 35 lobs (sheet 2026-11-27-robot, step 11) [derived: Bag ~/Desktop/rosbags/2026-11-27_13-49-34 replayed through human_catch_replay.py (13:50:30 to 13:54:30): 33 lobs, 26 crossed the Catch plane, 12 inside reach, 9 of those with an admitted plan; planned Cup centre to real crossing median 28 mm, p90 48 mm over the 14 planned lobs.]

## Progress

- 2026-11-03: Human-throw bench landed: two human-only Sittings scored, owner decisions recorded, data Sitting section written (detail below)
- 2026-11-13: Complete at the Sitting of 2026-11-13: bench report run offline (human_throw_bench.py, 130 s slice of the launch 3 bag): landing sd 73 x 101 mm at the Cup side, phantoms 13.8 per minute against 27.3 on 2026-11-02. The lighting change was not recorded. 26 of 30 phantoms are static Ball Butler frame markers (BUG-92).
- 2026-11-18: Reopened 2026-11-18 (owner): the 2026-11-13 debrief closed the whole Item on its bench re-measure row; the cascade itself has not been attempted. Open: the three Blockers (perception, a reactive Catch on the observed flight, the held-tilt mode) and the owner safety rules for a person inside the Platform reach.
- 2026-11-18: Promoted to Pathway PWAY-11 (plan human-partner-juggling.md): perception and reach numbers from the 2026-11-18 bag are in the body; the owner decided no safety rules are needed and set the ladder (one Ball caught, caught and thrown back, held tilt, three Balls).
- 2026-11-18: P1 and P2 of plan human-partner-juggling.md are built, audited and on main (Pattern human_catch, unflown). The Sitting section is now the single-lob data row. Owner 2026-11-18: the person is assumed not to drop; a Ball is dropped only when neither tracked nor felt by the Hand.
- 2026-11-20: Sitting of 2026-11-20 (data): single lobs recorded 12:52-12:57. Replay (temp/probes/human_catch_2026-11-20/2026-11-20_12-49-25_replay.md): 55 lobs, 49 crossing the Catch plane, 23 inside the Cup's reach, 16 of those with an admitted plan (5 refused BUSY, 2 NO_EVENT); planned Cup centre to real crossing median 30.5 mm, worst 52.0 mm.
- 2026-11-20: Most Balls were caught by hand near the Cup, which the replay does not model.
- 2026-11-20: Rerun on the owner's window 12:53:20-12:56:45 (temp/probes/human_catch_2026-11-20/2026-11-20_12-49-25_125320_125645_replay.md): the same 55 lobs, first at 12:53:20.6, last at 12:56:39.6. Hand frame unchanged (23 in reach, 16 planned, median 30.5 mm, worst now 57.2 mm); robot frame 19 in reach, 13 planned, median 25.8 mm. Declines overall: SETTLE_SITE 19, BUSY 8.
- 2026-11-21: 2026-11-21 replay sweep of the 2026-11-20 lobs: 26 of 49 crossing lobs land outside the 150 mm Cup box (the SETTLE_SITE declines), so aim scatter limits first. Of 23 in reach: 16 planned, 5 BUSY, 2 no event. The wait-Site rule (D7) changes nothing. Clear 0.3 s with return 1.0 s plans 21 and scores 11 good.
- 2026-11-21: Same sweep: the 30 mm median error is the Fit's convergence at 0.62 s of lead (39/30/22/15 mm at 0.70/0.62/0.55/0.45 s); the gate admits plans only at 0.55-0.62 s. D10 good of 23: 8 at 30 mm, 10 at 35, 13 at 40. The 80 % target (19) is out of reach while 7 in-reach lobs get no plan.
- 2026-11-21: 2026-11-21: owner ruled the defaults: clear 0.3 s, return 1.0 s, plan window 0.62 s, first lead 0.55 s, fixed wait Site, D10 bar 35 mm. On the 2026-11-20 lobs these plan 21 and score 13 good of 23 in reach. Other D2-D11 parameters stay proposed.
- 2026-11-26: the untraced 250 to 300 mm aimed-Throw reach is reproduced at Apex 1.5 m (250 mm, refused at 300); 400 mm at 1.8 m, 550 mm at 2.1 m, the same in all directions. The Hand C2 check binds past about 6.3 deg of release tilt, not tilt authority. P4 design note: temp/probes/pway11-p4/design.md.
- 2026-11-27: Sitting 2026-11-27 lob row flown: 33 single lobs replayed (human_catch_replay.py). 26 crossed the Catch plane, 12 inside the Cup's reach, 9 of those had an admitted plan; planned Cup centre to real crossing median 28 mm, p90 48 mm over 14 planned lobs. The 19 rejected tracks were under 500 mm/s (hand motion), no real lob lost.
- 2026-11-27: 2026-11-27 replay (offline): at the 0.62 s plan window the Fit overshoots along the flight by 11 mm (sd 15) and predicts touch-down 5.6 ms early over 20 lobs (likely drag, unchecked). /human_throws reaches subscribers a median 44 ms after its frame stamp (Mocap itself 8.5 ms). human_catch_replay.py now takes --transport-ms.
- 2026-11-27: 2026-11-27 replay, Robot frame: 14 lobs in reach, 10 planned, 7 inside 35 mm at zero delay; 5 to 6 of 14 with the measured 44 ms delay. 3 in-reach lobs were refused SETTLE_SITE with the Fit 1 to 11 mm outside the 150 mm box though the Ball crossed inside it. Every re-aim was refused (0 of 12). Owner decisions listed for step P3 of the partner plan.
