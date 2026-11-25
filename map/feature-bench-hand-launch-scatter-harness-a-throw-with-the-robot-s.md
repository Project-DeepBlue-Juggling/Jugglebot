---
id: FEAT-67
kind: feature
title: 'Bench Hand launch-scatter harness: a Throw with the robot''s release, Mocap recorded beside it, a bench mode for the release probe'
status: inbox
created: '2026-11-25'
updated: '2026-11-25T09:28:32Z'
owners: []
priority: high
effort: m
hardware: none
radius: local
area: hand
blocked_by: []
links:
- plans/active/learner-efficacy.md
- logbook/2026-11-25-learner-efficacy-e0-unsettled-ball-test.md
- tools/probes/release_settle_bag_probe.py
parent: learner-generalisation
filed_by: claude-e0
---

E1 of `plans/active/learner-efficacy.md` needs three things that do not exist yet. None needs hardware to build, and none needs markers on the bench Hand: the bench Hand does not move sideways, so the ball's lateral launch velocity from Mocap is the launch term directly. Only the ball has to be in the cameras' view.

## What to build

- **A bench Throw with the robot's release.** `hand_stream_bench.py --stage stroke_cut` ends in the C1 brake at 4500 rev/s^2. E0 placed the launch term at separation, so the bench release must be the robot's Throw deceleration or the result does not transfer. Add a stage that streams the Hand profile of a real Throw Segment at one Apex; check first which Apex the bench stroke (top stop 10.4 rev) allows.
- **Mocap recorded beside the bench driver.** The bench driver owns the UDP link with the launch down, so nothing records `/mocap_data` today. Decide and rehearse one way: the Mocap node on its own with a bag of `/mocap_data`, or the bench Hand run under the launch. The record must carry the envelope stamp (`tools/probes/_bag_time.py`).
- **A bench mode for `tools/probes/release_settle_bag_probe.py`.** No Memory rows and no announcements exist on the bench. Take the release from the ball's own peak upward velocity (the probe's `t_vpk`), fit the launch velocity over 0.035 to 0.2 s after it, and print per arm: n, lateral launch velocity sd per axis with a bootstrap interval, and the variance ratio between arms, against the thresholds in the plan's E1 Criterion.
- **Arms (b) and (c) need the Throw to follow the ball's arrival by about 0.27 s.** Open question for the owner: does the bench Hand carry the ball sensor? If so, the stage fires a fixed delay after the sensor reads the ball. If not, the options are a trigger from live Mocap or a timed drop, and the delay is then read from the recording, not assumed.

## Done when

- The stage is dry-run against the bench stroke limits and flown once on the bench with an empty Cup.
- The probe's bench mode is checked on existing robot bags: from-rest Throws of 2026-11-13 treated as bench Throws must give the launch velocity the bag analysis gave.
- FEAT-68 (E1) has its `## Sitting` steps rewritten with the exact commands.
