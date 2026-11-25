---
id: FEAT-68
kind: feature
title: 'Launch baseline on the robot''s Hand: launch scatter of a settled ball and of running Throws at Dwell 0.27, 0.50 and 1.00 (learner-efficacy E1)'
status: inbox
created: '2026-11-25'
updated: '2026-11-25T09:28:57Z'
owners: []
priority: high
effort: s
hardware: robot
owner: hands
radius: local
area: hand
blocked_by: []
links:
- plans/active/learner-efficacy.md
- logbook/2026-11-25-learner-efficacy-e0-unsettled-ball-test.md
- feature-bench-hand-launch-scatter-harness-a-throw-with-the-robot-s
parent: learner-generalisation
filed_by: claude-e0
night_skip: sitting
night_skip_on: '2026-11-26'
night_skip_note: A robot Hand launch-scatter baseline (learner-efficacy E1); its Sitting section is written.
---

Step E1 of `plans/active/learner-efficacy.md`. It gives the launch scatter of the current Cup as one number per block, and it tells the owner's account (the scatter comes from the ball not seating properly during the Dwell) from E0's reading (the scatter arises at separation, settled or not). Each Hand or Cup change in E4 is then kept or dropped by re-flying this same row.

## Why the robot and not the bench (owner, 2026-11-25)

The bench Hand has no ball sensor, so it cannot throw a fixed time after the ball lands. On the robot the Throw is the real one, the launch records Mocap, the Hand ball sensor, the announcements and the Memory rows, and `tools/probes/release_settle_bag_probe.py` runs on the bag unchanged. No markers go on the Hand: the Platform's own markers give its motion, and the probe removes it.

## The four blocks

Self-throw at one Site and one Apex (1.00 m), Platform at the Active pose and Level.

- **S, settled:** the ball placed by hand, at least 3 s of rest, then a goal of one Throw. 30 Throws.
- **D27, D50, D100:** goals of 12 Throws at Dwell 0.27, 0.50 and 1.00 s. Every Throw after the first follows a real Catch. 6 Attempts each, about 66 running Throws per Dwell.

The blocks are flown in two halves in mirrored order, so a warming Hand motor does not line up with one block. The off-axis arm is not flown: it comes from D27, binned by where the ball came in at the Catch (the probe's entry offset, not validated in E0).

## Criterion (pre-registered 2026-11-25, before any Throw)

The figure is the sd per axis of the lateral launch velocity about its block mean, raw and with the measured Platform motion removed (one regression over the whole Sitting). On the E0 bags it was about 17 mm/s.

- **Seating (owner's account):** S at 6 mm/s or less on both axes AND D27 at twice S's variance or more, falling as the Dwell grows.
- **Separation (E0's reading):** S at 12 mm/s or more on both axes, with D27, D50 and D100 all within 1.5 times S's variance.
- **Mixed:** anything between, reported with both shares. A variance ratio known from 30 against 66 Throws carries about 30 % uncertainty; fly block S again if it lands between 1.5 and 2.

## Reading the result

After the Sitting: `python tools/probes/release_settle_bag_probe.py run <bag>` for each bag, then `python tools/probes/release_settle_bag_probe.py blocks temp/probes/release_settle/<bag>.csv --skip-attempts 1` (the warm-up Attempt is the first one). The verdict line reads UNDECIDED when a deciding ratio's interval straddles its threshold; more Throws in block S tighten it.

## Rehearsed offline

2026-11-25, `nominate.nominate(NominationConfig(pattern='self_toss', apex_m=1.0, separation_mm=0.0, dwell_s=D, n_throws=N))` at the default limits for D in 0.27, 0.50, 0.75, 1.00 and N in 1 and the short Pattern: all eight read `nomination OK`, worst margin 1.000. The live limits are checked by the row's own nominate calls.

## Caveats

- The robot Hand motor has slid roughly since the 2026-11-20 overheat. This row measures that Hand. A Cup change judged after a motor repair is not like for like: re-fly this row first.
- The rows go to a scratch plant so the live Memory is untouched. A fresh plant starts with no correction and its first Apex runs about 8 % high; the first Attempt is a warm-up and is not counted.
- With Dwell 1.00 the probe's own "from rest" flag (seated 1 s or more) fires on running Throws. Blocks are told apart by the goal (Throw index in the Attempt and Memory's `dwell_s`), never by that flag. FEAT-67's `blocks` report does this.

## Sitting

- setup: level, armed
- why: one launch-scatter number per block for the current Cup, and a test of seating against separation, before any Hand change is made
- do:
  - Before power, with the launch up and the bag recording: `ros2 param set /skill_node plant_id e1-<date>` (a name no Memory file has) and read it back with `ros2 param get /skill_node plant_id`.
  - Rehearse the three Dwells; nothing moves and each reply must read viable: `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'self_toss,apex_m=1.00,dwell_s=0.27'}"`, then the same with `dwell_s=0.50` and with `dwell_s=1.00`. Drop a Dwell that is refused and note it.
  - Boot, Home, Level, Activate. Check Level with the inclinometer reading on the GUI and write it down.
  - Warm-up, not counted: ball in the Cup, `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: self_toss, apex_m: 1.00, dwell_s: 0.27, num_cycles: 12}" --feedback`.
  - Block S, first half, 15 Attempts of one Throw: place the ball in the Cup by hand, take the hand away, count 3 s, then `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: self_toss, apex_m: 1.00, dwell_s: 0.27, num_cycles: 1}" --feedback`. After a drop, place the ball back and carry on; a dropped Throw still counts.
  - Block D27, 3 Attempts: `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: self_toss, apex_m: 1.00, dwell_s: 0.27, num_cycles: 12}" --feedback`.
  - Block D50, 3 Attempts: `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: self_toss, apex_m: 1.00, dwell_s: 0.50, num_cycles: 12}" --feedback`.
  - Block D100, 3 Attempts: `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: self_toss, apex_m: 1.00, dwell_s: 1.00, num_cycles: 12}" --feedback`.
  - Second half in mirrored order: 3 more Attempts of D100, 3 of D50, 3 of D27, then the last 15 single Throws of block S, each with the same command as in the first half.
  - Afterwards `ros2 param set /skill_node plant_id jugglebot` and read it back.
- pass: at least 25 Throws with a launch fit in block S and at least 50 in each Dwell block that was not refused, so the per-block scatter can be read against the criterion
- record: the plant_id used, the Level reading, the local time each block started, Throws and Catches per Attempt, and anything odd about how the ball sat or left the Cup
- minutes: 60
