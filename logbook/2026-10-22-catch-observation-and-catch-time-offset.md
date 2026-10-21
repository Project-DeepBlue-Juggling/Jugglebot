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
   the cause is mechanical (the cup's seat/rim geometry, owner to measure).

Restart the FULL launch before flying (new parameter, new ring layout).

## Verification

| Date | Command | Result |
|---|---|---|
| 2026-10-22 | `pytest tests/motion/test_skills_report.py -q` | 17 passed |
| 2026-10-22 | `pytest tests/motion/test_skills_executor.py -q -k "contact or feed_line or feed_catch_with_no_fit or catch_time_offset or crossing"` | 9 passed (contact ±, `--` cases, window, feed line, offset shifts `t_land_s` by exactly ±20 ms, 0 bit-identical) |
| 2026-10-22 | `pytest tests/ros/test_skill_node.py -q -k "catch_time_offset or contact_probe"` | 6 passed |
| 2026-10-22 | `./run_tests.sh --full` (shared gate with the I-PLAN-11 commit, log `temp/logs/gate_iplan11_20261022.log`) | 7017 passed, 9 skipped in 313.53 s + serial 7 passed; exit 0 |

NOT FLOWN.
