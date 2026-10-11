---
title: Ball Butler does not drift over a sitting - the "late and wide" feeds were cup contact; the real problem is +/-20 ms release jitter
type: investigation
date: 2026-10-12
status: open
phase: "two-ball-skill-stack — R5 (Ball Butler calibration) / two-ball columns"
related_plan: two-ball-skill-stack.md
related:
  - 2026-10-11-skill-stack-r5-skill-node-response-starvation.md
files_changed:
  - logbook/2026-10-12-skill-stack-r5-bb-feed-no-drift-release-jitter.md
  - logbook/INDEX.md
subsystem:
  - ball-butler
tags:
  - ball-butler
  - timing
---

# BB feeds: no drift, but release jitter

## What happened

The sitting-7b review (bag `2026-10-11_22-15-28`) read the feeds as drifting. Feeds 1-7 landed +16..+30 ms late;
feeds 10-15 landed +37..+63 ms late and 30-100 mm wide. The review named BB feed degradation as the cause of the
later attempts' decline. The owner asked whether BB's recent firmware was to blame. This is a read-only
investigation; no code changed.

## Method

BB's ball (apex about 2.5 m) is out of mocap view for most of its flight. Two measurements were used per throw:

- **Flight:** a fixed-gravity, outlier-robust parabola fitted to the last ~0.25 s of the raw mocap track (65-86
  samples, 1-2 mm residual), extrapolated to the 830 mm catch plane.
- **Release:** BB's own hand stroke from `/bb/axis_estimates` (100 Hz hand encoder, bridge clock).

Coverage: all 19 sitting-7b feeds and 39 across the three 2026-10-11 sittings. Hand timing was also checked in
seven sittings back to 2026-09-30. Scripts and outputs (`bb_*.py`, `bbx_*`/`bbf_*` json, `bb_merge_all.txt`,
`BB_drift_report.md`) are in that session's scratchpad.

## Findings

1. **No drift.** Across sitting 7b:
   - apex 2502 +/- 9 mm;
   - horizontal velocity (899 +/- 20, 401 +/- 12) mm/s;
   - landing within about +/-25 mm of the site;
   - hand peak speed and stroke length constant;
   - hand timing slope +0.6 ms/min.

   BB's ODrive warming (25.6 -> 29.9 C) left no trace. None of the FW 6/7/8 failure signatures appear.
2. **The "late and wide" feeds were cup contact.** `feed_catch_bag_probe.py` times the ball's crossing of
   z = 830 mm. In the late feeds, the ball reached the site on time or early (-38..-3 ms). It met the cup 60-100 mm
   above the plane while the cup was still high in its dive, slid sideways, and crossed 830 mm 40-90 ms later. That
   crossing was read as BB being late and wide.
3. **The real issue is release jitter.** The spread is in when BB's hand fires:
   - hand timing sd 14-22 ms on FW 8 (range 53-67 ms), against 12-13 ms on FW 5, so it predates the recent firmware;
   - pooled over 39 throws, arrival error = 0.91 x hand-timing error + 3.5 ms (r 0.964, residual 4.8 ms);
   - the flight itself repeats to about 5 ms;
   - the mean offset is only about -9.5 ms.
4. **Likely source (medium-high confidence): the dispatch path.** `ball_butler_node` stamps the announcement with
   its own `now` but sends a RELATIVE delay. The can-bridge converts it to an absolute time only on receipt
   (`Teensy_code_canbridge/ball_butler_protocol.h:83`), so every millisecond of Jetson-side latency moves the
   release by a millisecond. Two observations support this:
   - BB's THROWING heartbeat onset tracks hand timing (r 0.52, about what a 10 Hz heartbeat allows);
   - the jitter is smallest in the BB-only sitting (8 ms) and largest in the busiest one (19 ms).
5. **The real decline is on the catch side.** Similar early arrivals seated worse later in the sitting:
   - throw 2, 29 ms early, seated at +97 ms;
   - throw 14, 29 ms early, seated at +291 ms;
   - throw 16, 38 ms early, missed.

   The feed catch is pinned to the schedule, so an early ball meets a still-high cup.

## Discussion

- **Hypotheses withdrawn.** Three earlier readings do not survive the flight fit:
  - the sitting-7b review's "BB feed degradation";
  - this arc's "BB-side drift, recalibrate or add a time bias";
  - the owner's firmware suspicion.

  All three rested on the 830 mm crossing as a landing measure, and that crossing includes the cup contact.
- **Neither recalibration nor a constant time bias helps.** The problem is per-throw scatter, not a mean offset.
  The optional ~10 ms re-centring of the announcement bias (52 -> ~42 ms) is minor.

## Remedy options (owner decision; none implemented)

- **(a) Absolute release time end to end.** Send the absolute release time from the same `now` the announcement
  uses. BB already decodes an absolute time, so BB firmware is unchanged. This needs a bridge firmware + RPC
  argument change and a lockstep bridge flash.
- **(b) Retime the feed catch from BB's measured hand stroke.** The stroke is known about 0.85 s before landing and
  predicts arrival to about 5 ms without mocap. Whether the planner can take a +/-40 ms retime that early is a
  design question.
- **(c) Fix the probe.** Make `feed_catch_bag_probe.py` report the free-fall landing beside its 830 mm crossing,
  so future reviews don't misread cup contact.

## Checks pre-registered for the next sitting

- **C1:** log the BB_THROW RPC send time in `teensy_bridge_node`. Expect hand timing minus dispatch latency to
  have sd <= 6 ms. If the dispatch latency is flat instead, look inside BB / the bridge->BB link: BB's `dbg_time`
  sync stats and the per-throw `lead=` log line.
- **C2:** BB-encoder arrival-prediction residual <= 6 ms. Above 10 ms means the flight itself varies.
- **C3:** hand-timing |slope| < 1.5 ms/min (no drift).
- **C4:** the probe reports the free-fall landing.
