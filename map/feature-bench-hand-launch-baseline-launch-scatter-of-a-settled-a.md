---
id: FEAT-68
kind: feature
title: 'Bench Hand launch baseline: launch scatter of a settled, a dropped-in and an off-axis ball (learner-efficacy E1)'
status: inbox
created: '2026-11-25'
updated: '2026-11-25T09:28:57Z'
owners: []
priority: high
effort: s
hardware: bench-hand
owner: hands
radius: local
area: hand
blocked_by:
- feature-bench-hand-launch-scatter-harness-a-throw-with-the-robot-s
links:
- plans/active/learner-efficacy.md
- logbook/2026-11-25-learner-efficacy-e0-unsettled-ball-test.md
- feature-bench-hand-launch-scatter-harness-a-throw-with-the-robot-s
parent: learner-generalisation
filed_by: claude-e0
---

Step E1 of `plans/active/learner-efficacy.md`. It gives the launch scatter of the current Cup as one number, and it tells the owner's account (the scatter comes from the ball not seating properly during the Dwell) from E0's reading (the scatter arises at separation, settled or not). Each Hand or Cup change in E4 is then kept or dropped on the same number.

**No markers on the bench Hand are needed.** The bench Hand does not move sideways, so the ball's lateral launch velocity from Mocap is the launch term directly. The bench Hand only has to stand where the cameras see the ball from the Cup to about 0.3 m above it.

## The three arms

About 30 Throws each at one Apex, the ball's lateral launch velocity sd per axis:

- (a) hand-placed and settled, thrown after at least 2 s;
- (b) dropped in from about Catch height, thrown about 0.27 s after it lands;
- (c) dropped in or placed a few mm off-axis, thrown about 0.27 s after it lands.

## Criterion (pre-registered 2026-11-25, before any bench Throw)

On the robot the launch term is about 17 mm/s per axis.

- **Seating (owner's account):** arm (a) at 6 mm/s or less AND arm (b) or (c) at twice arm (a)'s variance or more.
- **Separation (E0's reading):** arm (a) at 12 mm/s or more, with (b) and (c) within 1.5 times its variance.
- **Neither:** arm (a) at 6 mm/s or less and (b), (c) no worse: the term needs the robot's motion, and a bench Cup change alone is the wrong bet.
- Anything between the thresholds is reported as mixed, with both shares. A variance ratio near 2 at 30 Throws is marginal: fly 60 per arm if it lands there.

## Not ready to fly

FEAT-67 builds the bench Throw with the robot's release, the Mocap recording and the probe's bench mode. Until it lands the steps below name what to do and not the commands; FEAT-67 rewrites them with the exact commands and a rehearsal.

## Sitting

- setup: bridge-cycled
- why: one number for the current Cup's launch scatter, and a test of seating against separation, before any Hand change is made
- do:
  - Stand the bench Hand where the Mocap cameras see the ball from the Cup to about 0.3 m above it. No markers go on the Hand. Check in QTM that the ball in the Cup shows as one unlabelled marker.
  - Launch down. Bench Hand on the bridge (CAN2, drive node 6), 45 V supply on, E-stop in hand. From the repo root in the venv: `python tests/hardware/bench_hand_frame_bench.py status`. It must read node 6, IDLE, no errors.
  - `python tests/hardware/bench_hand_frame_bench.py home`: it drives DOWN at 3 rev/s and ends IDLE on the bottom stop at -0.10 rev. E-stop if it moves up.
  - Top stop: 10.4 rev unless the Hand has changed since 2026-11-22; if it has, lift the IDLE Hand onto the top stop, run `status` and use that reading.
  - Start the Mocap recording and fly one Throw with the Cup empty, using FEAT-67's harness. Stop if the stage's plan needs more stroke than the top stop.
  - Arm (a): place the ball in the Cup by hand, wait at least 2 s, Throw. 30 Throws.
  - Arm (b): drop the ball in from about Catch height; the harness throws about 0.27 s after it lands. 30 Throws.
  - Arm (c): the same with the ball coming in a few mm off the Cup axis. 30 Throws.
  - Run the probe's bench mode on the recording before leaving the rig; fly 30 more of any arm whose variance ratio lands between 1.5 and 2.
- pass: the probe's bench mode gives a launch velocity sd for all three arms from at least 25 Throws each
- record: per arm the number of Throws and anything odd about how the ball sat or left; whether the ball was caught or fell; the recording's name
- minutes: 75
