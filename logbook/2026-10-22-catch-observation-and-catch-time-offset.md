---
title: "Catch observation on the throw line (contact, settle, seat re-zeroed to the crossing) and a debug catch-time offset at dispatch: the future catch learner's observation and command channels"
type: feature
date: 2026-10-22
status: in-progress
phase: "two-ball-skill-stack - post-R6 (first sitting on the ASA cup, instrument + two checks)"
related_plan: two-ball-oval.md
related:
  - 2026-10-22-hop-reaims-refused-mid-transit-tilt-rate-seam-and-the-new-cup
  - 2026-10-21-hand-cup-pla-broke-throw-catch-census
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/report.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_skills_report.py
  - tests/ros/test_skill_node.py
  - tests/hardware/session_skill_stack_rehearsals.md
subsystem:
  - motion
  - ros
tags:
  - hand
  - learning
  - observability
---

# Catch observation on the throw line + a debug catch-time offset at dispatch

## What changed

Owner decision 2026-10-22 ("instrument + two checks") after the catch-quality
measurement in `2026-10-22-hop-reaims-refused-mid-transit-tilt-rate-seam-and-the-new-cup`
§ Diagnosis: on the ASA cup a Ball-Butler-fed ball rings ~250 ms in the cup
before it seats (PLA: ~120 ms) at the SAME closing speed and within ±10 ms of
the same timing, and nothing on the throw line could say whether timing or the
cup is at fault. Built as the future catch learner's observation and command
channels; the learner itself is decided by rehearsal row 49.

1. **Throw line** (`report.py`, `executor.py`): `seat` is re-zeroed to the
   tracker's final-fit crossing time (it was release + the COMMANDED apex's
   flight time, which carries the learner's ~40 ms low-apex bias on every
   self-throw) and labelled `(vs crossing)`; new `contact +NN ms` (the first
   hand sample in [crossing−0.08, crossing+0.20] s with `vel_meas − vel_ff_cmd
   < −8 rev/s` and `vel_ff_cmd < 1`, the detector the offline probe used) and
   `settle NNN ms` (contact → cup sensor seated). Ball Butler feed catches,
   which had no line, get `feed ball N CAUGHT  arrival  seat  contact  settle`
   (INFO, one per seated feed, not counted in the Juggle result). Any missing
   piece prints `--`. Example:
   `throw 3/5 CAUGHT  apex 0.917 m  landed -4, +9 mm  release +15 ms  arrival -8 ms  seat +0.30 s (vs crossing)  contact +15 ms  settle 285 ms  re-aimed 1x`.
2. **`catch_time_offset_ms`** node parameter (default 0; outside ±40 ignored
   with one ERROR log per bad value; the goal start line shows
   `DEBUG catch_time_offset +20 ms` when nonzero). Applied to every catch
   plan's touch-down time at DISPATCH inside the one clamp funnel
   (`executor._clamp_lateral_to_schedule`, reached by `_catch_terminal`), so
   live re-sends inherit the same shifted value and it is never a late
   re-aim (the 2026-10-21 +3..+31 ms re-aims were refused `HAND_LIMIT_C2`).
   A nonzero offset also SUPPRESSES the learner row for every self-throw
   (`_finalise_outcome`: reason `no learner row: catch_time_offset in force`
   on the OUTCOME/throw line) so an offset-induced miss cannot train the
   THROW memory; offset 0 writes the row as before.
   The shifted plan goes through the normal `install_segment` /
   `validate_cycle` gate with no bypass; the admissible box was swept at 0,
   so a refusal at ±20 ms is the right answer. The reported `arrival`, the
   CATCH-AIM `t_land` and the dwell move by the same amount.
3. The hand-telemetry ring in `skill_node.py` is 6 wide (adds `vel_ff_cmd`,
   `vel_meas`) for the contact detector; nothing reads it for a decision.
4. Rehearsal row 49: ten BB feeds each at −20 / 0 / +20 ms; median `settle`
   moving > 60 ms between any two runs ⇒ timing is a lever and the catch
   learner goes ahead as a second memory on the existing machinery; otherwise
   the cause is mechanical (the cup's seat/rim geometry, owner to measure; geometry measurement RETIRED 2026-10-23, see Flown 2026-10-22).

Restart the FULL launch before flying (new parameter, new ring layout).

## Verification

| Date | Command | Result |
|---|---|---|
| 2026-10-22 | `pytest tests/motion/test_skills_report.py -q` | 17 passed |
| 2026-10-22 | `pytest tests/motion/test_skills_executor.py -q -k "contact or feed_line or feed_catch_with_no_fit or catch_time_offset or crossing"` | 9 passed (contact ±, `--` cases, window, feed line, offset shifts `t_land_s` by exactly ±20 ms, 0 bit-identical) |
| 2026-10-22 | `pytest tests/ros/test_skill_node.py -q -k "catch_time_offset or contact_probe"` | 6 passed |
| 2026-10-22 | `./run_tests.sh --full` (shared gate with the I-PLAN-11 commit, log `temp/logs/gate_iplan11_20261022.log`) | 7017 passed, 9 skipped in 313.53 s + serial 7 passed; exit 0 |

NOT FLOWN as of the instrument's landing; see Flown 2026-10-22 below.

## Flown 2026-10-22

Rehearsal row 49 flew in the 2026-10-22 19:50 sitting (launch log `/home/jetson/.ros/log/2026-10-22-19-50-38-975040-jetson-50159/launch.log`; 964bd409 build). Criterion NOT met. The feed lines printed `arrival -- seat -- contact -- settle --` for all 21 feeds (21 feed lines against 28 BB-fed balls in the bag; the 7 without a line registered no seated feed catch -- not reconciled per ball): the fed ball's /balls status leaves IN_FLIGHT 0.29-0.50 s after landing, before the +0.53 s report (`_report_feed`, executor.py), so the tracker provider (`ball_possession.flight_in_progress`) returns None and every derived field is `--`. Fix (latch the last converged landing) goes in the next commit; NOT FLOWN. Numbers below are therefore bag-derived: settle = first SEATED minus first cup contact, per BB-fed ball.

| arm (offset) | fed/caught | settle median [range] ms (n) |
|---|---|---|
| before any offset | 6/6 | 118 (3) |
| -20 | 9/9 | 209 [118-224] (9) |
| 0 | 7/7 | 213 [178-220] (4) |
| +20 | 6/6 | 259 [140-260] (3) |

Bimodal ~100-140 vs ~230-270 ms in every arm; no shift between the -20/0/+20 arms exceeds 60 ms (max 50 ms); the pre-offset control sits 91-141 ms below them (n=3, slower-closing balls), so drift between zero-offset groups exceeds the criterion and the A/B is under-powered rather than clean negative evidence; the decision (no catch-timing learner) rests on the within-arm bimodality (100-140 vs 230-270 ms in every arm) and the owner's 48-catch run at 0 ms. Self-throw settle at 0 ms: 125 ms [81-210] (n=45, bag) / 151 [94-235] (n=39, log). Protocol caveat: ten feeds per arm was not met (the -20 runs ended after 1-3 throws: 4 LIMIT_ACC 5030-5398 vs 5000, one LIMIT_VEL 351.8 > 350, two ABORTED_NO_RELEASE after wild throws landed -209/-278 mm; owner: "the balls collided in the air, suggesting messy throws"). The +20 arm was ended 4 of 6 times by a repeatable LIMIT_JERK 240-256k on CATCH skill 2 (leg 5, t=0.100, k_s 40) of the hop-entry columns goal; both the learned aim and the y_d fallback were refused. Owner impressions: "-20 felt like good catching", "0 was great. We got Jugglebot's PB of 48 catches", "+20 was pretty clunky". The 48-catch run (L3402-4405): 47/48 by the log: 47 caught; the goal ended on a genuine MISSED_CATCH at skill 48 (L4394/L4403) that is not in the count; the one MISSED inside the count is the final cross-site throw, the by-design D2 ending (2026-09-30; executor.py ~5001 sets caught=False for the Stop throw regardless; owner's '48 catches' counts that run). The columns_1ball_fed 1/2 results (L1129, 1301, 1673): the real ball lands at the far site on an empty, static cup during the closing REST (no catch planned); the log's 'landed on the held ball' label is wrong for the one-ball patterns.

Decision (orchestrator, 2026-10-23): timing is not a lever on settle at +/-20 ms; no second catch learner. Owner 2026-10-23: "The ASA hand is geometrically identical to the PLA hand", so the seat/rim-height measurement is RETIRED; the remaining candidate for the longer ASA settle is the print's surface/sliding friction (owner: the ASA hand "slides slightly less smoothly"). Status stays in-progress because the feed-line fix is unflown.
