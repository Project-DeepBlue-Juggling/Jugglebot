---
title: "learner-efficacy re-scoped after E0 (owner): measure the launch scatter on the bench Hand, fix it in the Hand, keep the Learner's two correctness bugs; the transit term, yaw, the shrinkage estimator and the IMU study retired from the plan"
type: refactor
date: 2026-11-25
status: resolved
phase: "learner-efficacy re-scope (PWAY-2), plan edit only"
related_plan: learner-efficacy.md
related_entries:
  - 2026-11-25-learner-efficacy-e0-unsettled-ball-test.md
files_changed:
  - plans/active/learner-efficacy.md
  - map/feature-bench-hand-launch-baseline-launch-scatter-of-a-settled-a.md
  - map/feature-bench-hand-launch-scatter-harness-a-throw-with-the-robot-s.md
  - docs/runbook.md
  - plans/active/INDEX.md
  - docs/project-map.md
  - logbook/2026-11-25-learner-efficacy-rescoped-to-the-hand.md
  - logbook/INDEX.md
subsystem:
  - motion
tags:
  - docs
---

# learner-efficacy re-scoped to the Hand

**What.** `plans/active/learner-efficacy.md` is cut to the steps that can move Throw repeatability. E1 becomes a short bench baseline, E4 becomes the owner's Hand change judged against it, E5 (the Catch) stays, E2 keeps only BUG-137 and BUG-138 with one Memory for the robot. E3 (transit term, yaw) and E6 (the IMU's part) are retired; FEAT-65 leaves E2. BUG-79, IDEA-12, IDEA-18, FEAT-65 and IDEA-27 stay open as Items.

**Why.** Owner, 2026-11-25, after E0: "it might not be worth going too deep down this rabbit hole ... the true solution here would be a fix to the hand itself." Asked whether completing the plan as written would improve repeatability, the answer was mostly no: against about 20 mm sd of white scatter, the quiet Learner has a ceiling of 2-4 % of rms, the transit term removes a 9-15 mm bias that barely moves the rms, and only acting on the launch term (35-70 % of lateral landing variance) is a large lever.

**The disagreement E1 is built to settle.** The owner expects little scatter from a hand-placed, settled ball: in his account the scatter comes from the ball not seating properly during the Dwell. E0's reading predicts g-sized scatter even then (the term appears at separation; Throws from rest on the robot carry about the same g, though that comparison was thin). E1 therefore has three arms (hand-placed and settled; dropped in and thrown after about 0.27 s; off-axis) and thresholds fixed before any bench Throw: 6 mm/s or less settled with twice the variance unsettled supports seating, 12 mm/s or more settled supports separation, against 17 mm/s on the robot. No Cup markers are needed, because the bench Hand does not move sideways.

**Later the same day: E1 moved to the robot's Hand.** The bench Hand has no ball sensor (owner), so it cannot throw a fixed time after the ball lands. E1 is now a Self-throw Sitting on the robot (FEAT-68): 30 one-Throw Attempts with a hand-placed, rested ball, and about 66 running Throws each at Dwell 0.27, 0.50 and 1.00 s, at one Site and Apex on a scratch plant. The thresholds are unchanged (6 and 12 mm/s for the settled block), and the Dwell ladder adds a dose-response the E0 bags could not give, where Dwell was tied to the Pattern. The real Throw, the recording and the E0 probe are all already there, so the bench harness is not built; FEAT-67 shrinks to the per-block report. All eight goals nominate OK offline at the default limits (2026-11-25, `nominate.nominate` with `self_toss`, Apex 1.0, Dwell 0.27 / 0.50 / 0.75 / 1.00, one Throw and the short Pattern). The paragraph above describes the bench form as first written.

**Left open.** The Pathway's Destination still names bias for carried Throws, new configurations and yaw; with E3 retired this plan no longer completes it (plan § 7, owner decision). E1's Item is FEAT-68, with its `## Sitting` section.

**Verification.** Plan and logbook only. 2026-11-25, `flock /tmp/jugglebot-tests.lock python -m pytest tests/sim/test_plans_index.py tests/sim/test_mapkit*.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_docs_links.py tests/sim/test_no_legacy_item_ids.py -q`: result in the commit message.
