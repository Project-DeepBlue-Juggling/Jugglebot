---
id: FEAT-67
kind: feature
title: 'E1 block report: per-block launch scatter from the release probe''s records, checked on the E0 bags'
status: inbox
created: '2026-11-25'
updated: '2026-11-25T09:28:32Z'
owners: []
priority: high
effort: s
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

E1 of `plans/active/learner-efficacy.md` moved from the bench Hand to the robot's Hand on 2026-11-25 (owner: the bench Hand has no ball sensor). That removed most of this Item: the Throw is the robot's own, the launch already records Mocap, and `tools/probes/release_settle_bag_probe.py` runs on the bag unchanged. What is left is the report that turns the probe's per-Throw records into FEAT-68's verdict.

## What to build

- **A `blocks` report on the probe's CSV.** Label every Throw by its goal: block S (the only Throw of a one-Throw Attempt, seated 1 s or more), and D27 / D50 / D100 (Throw index 2 or later in its Attempt, by Memory's `dwell_s`). The first Throw of a long Attempt is from rest and is reported apart. Never use the probe's `from_rest` flag alone: at Dwell 1.00 it fires on running Throws.
- **Per block:** n, the sd per axis of the lateral launch velocity about the block mean, raw and with the measured Platform motion removed by one regression over the whole Sitting (30 Throws cannot carry the ten-regressor fit E0 used), a bootstrap interval by Attempt, and the variance ratios against block S.
- **The verdict line** against FEAT-68's criterion, plus D27 binned by the Catch entry offset.

## Done when

- The report is checked on an E0 bag: 2026-11-06 15:53 must give the from-rest and running figures of `logbook/2026-11-25-learner-efficacy-e0-unsettled-ball-test.md` finding 3 (ratio 1.20, 12 against 254 Throws) to within the difference the simpler regression makes, and that difference is stated.
- FEAT-68's `pass:` can be read straight off its output.
