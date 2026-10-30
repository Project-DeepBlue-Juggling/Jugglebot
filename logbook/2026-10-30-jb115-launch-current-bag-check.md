---
title: "JB-115 bag check: the loaded launch does not clip the 50 A hand limit in the 2026-10-28 Columns bags (peak 39.9 A), but the bags never flew the Apex the claim is about; the filing's error was an Apex-frame mix-up"
type: investigation
date: 2026-10-30
status: complete
phase: "pattern progression P1"
related_plan:
  - pattern-progression.md
related_entries:
  - 2026-10-30-pattern-progression-plan.md
  - 2026-10-30-two-ball-oval-p3-regate-go-and-dwell-headroom.md
files_changed:
  - plans/active/pattern-progression.md
subsystem:
  - hand
tags:
  - hand
  - current
  - bag-analysis
  - patterns
---

# JB-115 bag check: hand current at the loaded launch

## What

Offline, read-only check of bug JB-115 (the R0 study's side finding: by the throw envelope's B3 constants the launch THROW with the ball aboard asks for 53.6 A at Apex 1.05 and crosses 50 A at about 0.99, which the ball-free 3900 rev/s2 gate cannot see). Two bags: `~/Desktop/rosbags/2026-10-28_14-29-37` (the 49/49 Columns record) and `2026-10-28_21-02-49`. One-off probes in the session scratchpad (decode, analyse, fit, split, summ; `mcap_ros2` decode of `/hand_telemetry`, the setpoint echo and the throw announcements).

## Result

- Peak measured hand iq over every stroke: 39.90 A (14:29) and 39.83 A (21:02). Zero samples at or above 49 A. The record Attempt's launch drew 26.3 A; all 11 launches 24.8 to 39.9 A.
- No saturation signature: measured peak acceleration tracks the plan (median 2595 vs 2542 rev/s2), the hand reaches the planned release speed (measured over planned 1.005), and the tracking error is a constant lag (0.3 rev / 11 rev/s; 0.6 / 25 in the second bag), not a growing shortfall.
- So "the drive already clips today" is REFUTED. The claim itself is INCONCLUSIVE: planned release speeds were 115.7 to 131.6 rev/s, which is release-relative Apex 0.86 to 0.935, and the planned peak acceleration never passed 2797 rev/s2 against the 3509 the claim needs.

## Discussion

The hypothesis was reframed, not rescued. The filing compared R0's Apex, which is measured from the release (`schedule.flight_s`), with the goal `apex_m`, which is above the Catch plane 30 mm lower, and assumed the Learner had corrected commands up toward 1.047. In the bags the per-Throw commands sat about 0.05 m BELOW the 0.95 goal (release-relative median 0.865, about 0.90 above the Catch plane). R0's 50 A crossing at release-relative 0.99 is therefore a planned Catch-plane Apex of about 1.02, which Columns has never flown. Two frames for one word is a trap worth remembering whenever R0 or JB-114 numbers meet a goal Apex.

Extrapolating the flown strokes to the 3509 rev/s2 launch peak brackets 50 A rather than deciding it: about 35 A by regression of iq on acceleration (a lower bound, the slope is diluted by 10 ms timing jitter), 37 to 46 A scaling the median sampled peak, 51 to 56 A scaling the worst sampled stroke. B3 predicts 39.4 A at the flown 2542 rev/s2 and the largest samples read 38.4 to 39.9 A, so the model is not refuted and may run 10 to 20 % high. The instrument cannot say more: `/hand_telemetry` carries iq at about 100 Hz forwarded on change (median 9.9 ms, p95 18 ms inside a stroke), so a one-sample peak can read up to 20 % low; `/robot_state` has `iq_setpoint`, which would show a clamp at exactly 50.0, but at 10 Hz.

Side finding, filed as its own bug: the brake draws far less than the envelope's B2 model (measured −14 to −20 A at about −3000 rev/s2 where B2 says 35 to 40 A, and the 0.281 kg hand alone needs about 22 A at the configured Kt), and the ball-free gravity hold reads 1.5 A against 2.6 A computed. Friction, a real Kt 1.4 to 1.7 times the configured 0.0055133, or a lighter moving mass; unresolved, and it does not touch the 50 A question because the ODrive limits measured current. Also: the hand torque feedforward is live in these bags (`tor_ff_cmd` up to +0.126 / −0.146 N m), so the "zero torque feedforward" note in the `hand_overspeed_bag_probe.py` docstring is stale.

## What decides it

A Sitting with a launch at planned Catch-plane Apex 1.00 to 1.05 and a faster current capture: hand `iq_setpoint` on `/hand_telemetry`, or a 250 Hz Diagnostic on axis 6 (firmware). Stop rule: any `iq_setpoint` sample at 50.0 A. This is decision D-c of `plans/active/pattern-progression.md`; JB-115 goes back to backlog with this result.

## Verification

- (2026-10-30, `python -I decode.py ~/Desktop/rosbags/2026-10-28_14-29-37 out/b1433.pkl` and the same for `2026-10-28_21-02-49`, result: 29584 / 39165 hand samples, 103 / 203 announcements decoded).
- (2026-10-30, `python -I analyse.py out/b{1433,2104}.pkl`, result: 102 / 197 strokes; bag max |iq| 39.90 / 39.83 A, 0 saturated samples).
- (2026-10-30, `python -I split.py out/b{1433,2104}.pkl`, result: ascent slope 8.56e-3 / 8.55e-3 A per rev/s2, brake slope 4.6e-3 / 4.1e-3, release-to-landing drop 31 mm).
- Repo tests for this entry's files: see the plan entry `2026-10-30-pattern-progression-plan.md` (the same scoped set, re-run below at commit time).
