---
id: IDEA-16
kind: idea
title: JP-17 Three-ball cascade with the owner as the second Hand, Platform held tilted toward them
status: in_progress
created: '2026-10-30'
updated: '2026-11-02T22:34:33Z'
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
filed_by: claude (jugglebot-f5, pattern exploration 2026-10-30)
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

## Sitting

- setup: level
- why: re-measure the human-throw bench with the robot in place (moving Platform markers, a ball in the Cup, the robot blocking camera views) and the owner's throws at better lighting.
- do:
  - Improve the lighting; launch, home and Level the Platform; do NOT arm. Rest one ball in the Cup.
  - `ros2 param set /ball_tracker_node detect_human_throws true`.
  - Juggle a three-ball cascade for about 60 s with one hand beside the Cup at its usual catch height, at a steady height; then 60 s of single passes into the space just above the Cup.
  - Afterwards `ros2 param set /ball_tracker_node detect_human_throws false`.
- pass: the bag replays through `tools/probes/human_throw_bench.py` with the robot-spot landing sd and the phantom rate reported.
- record: the bag path, the lighting change, and the bench report path.
- minutes: 15

## Progress

- 2026-11-03: Human-throw bench landed: two human-only Sittings scored, owner decisions recorded, data Sitting section written (detail below)
