---
title: "Release probe gains a per-block report (FEAT-67): launch scatter per block of a Sitting with a verdict against FEAT-68's criterion, checked on the E0 bags"
type: feature
date: 2026-11-25
status: resolved
phase: "learner-efficacy E1 tooling (PWAY-2)"
related_plan: learner-efficacy.md
related_entries:
  - 2026-11-25-learner-efficacy-e0-unsettled-ball-test.md
  - 2026-11-25-learner-efficacy-rescoped-to-the-hand.md
files_changed:
  - tools/probes/release_settle_bag_probe.py
  - tools/probes/README.md
  - tests/sim/test_release_settle_blocks.py
  - map/feature-bench-hand-launch-scatter-harness-a-throw-with-the-robot-s.md
  - map/feature-bench-hand-launch-baseline-launch-scatter-of-a-settled-a.md
subsystem:
  - tracking
tags:
  - testing
---

# Release probe: the `blocks` report (FEAT-67)

**What.** `tools/probes/release_settle_bag_probe.py blocks CSV...` reads the probe's per-Throw records for one Sitting and prints, per block, the sd of the lateral launch velocity about the block mean, raw and with the command and the measured Platform motion removed, a bootstrap interval, the variance ratio against the settled block, and a verdict line against FEAT-68's pre-registered criterion. Blocks come from the goal: `S` (the only Throw of a one-Throw Attempt, seated 1 s), `first` (Throw 1 of a longer Attempt), `D<nn>` (later Throws by Memory's Dwell). Built by an Opus agent; reviewed and re-run by the orchestrating session.

**Why.** FEAT-68 (E1) flies about 30 settled and 66 running Throws per Dwell. E0's two-stage, ten-regressor g cannot be fitted on 30 Throws, and the probe's `from_rest` flag fires on running Throws at Dwell 1.00, so E1 needs its own reduction.

**How it differs from E0's g.** One pooled least-squares fit over the Sitting on 8 regressors (commanded Cup velocity, the Platform's Mocap centroid velocity, normal and normal rate), both sides centred per (block, Site, release kind). Each Throw's degrees of freedom are 1 - 1/n_cell - h_ii, so a small block is not read low: the 12 seated first Throws of 2026-11-06 15:53 read 14.2 mm/s as a plain rms and 19.0 corrected. The bootstrap resamples Attempts over the fitted residuals; refitting per resample treats an Attempt's copies as independent and biased small blocks low. A deciding ratio whose interval straddles its threshold is reported UNDECIDED.

**Checked on the E0 bags (2026-11-25).**
- Per Throw against E0's g on 2026-11-06 15:53 (330 Throws): r 0.955 / 0.939, sd of difference 5.4 / 5.9 mm/s against about 17.
- E0's own classes on that bag (12 seated from rest against 254 running): ratio 1.16 to 1.19, against E0's 1.20; running sd within 0.3 to 0.7 mm/s. The report's own `first` block there reads 1.11 because 7 of its 19 first Throws were not from rest.
- Attempt gap 3.0 s against E0's 6.0 s: identical block counts on all 21 bags.
- 2026-11-13 13:06 has 43 one-Throw settled Attempts at one Site: S 12.3 / 12.4 mm/s, D27 17.2, variance ratio 1.93 (95 % 1.47 to 2.59), verdict UNDECIDED. E0-era data with S and D27 at different Sites and Patterns, so it is a lead and not E1's answer; it does say a settled ball still launches with about 12 mm/s of scatter and a running one with about twice the variance.

**Clarified on FEAT-68 (before any Throw).** The 6 and 12 mm/s thresholds apply to both axes. The warm-up Attempt is dropped with `--skip-attempts 1`.

**Not done.** "Falling as the Dwell grows" is judged on point estimates with no tolerance. With 30 settled Throws a true ratio of 1.0 has an upper bound near 1.5, so Separation is callable only when the observed ratios are about 1.0 or below; more settled Throws tighten it. No robust (median-based) sd beside the rms: `max|e|` is printed per block instead.

**Verification.** 2026-11-25, `flock /tmp/jugglebot-tests.lock python -m pytest tests/sim/test_release_settle_blocks.py tests/sim/test_bag_time.py tests/sim/test_docs_links.py -q`: 20 passed in 0.50 s (9 new, on synthetic rows). `blocks` on 2026-11-06_15-53-05.csv run twice: identical output. `features` on 2026-11-10_19-00-33.npz: byte-identical to the CSV E0 analysed, so the existing modes are unchanged.
