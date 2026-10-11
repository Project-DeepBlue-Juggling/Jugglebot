---
title: the two-ball hop entry bypassed the learner, flew an uncorrected apex and arrived late — the P2 catch of Ball 1 died LIMIT_JERK 4/4
type: investigation
date: 2026-10-11
status: resolved
phase: "two-ball-skill-stack — R5 (Ball Butler calibration) / two-ball columns"
related_plan: two-ball-skill-stack.md
related:
  - 2026-10-11-skill-stack-r5-fed-catch-lateral-ring.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/report.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_skills_report.py
  - tests/motion/test_validate_cycle.py
  - tests/motion/test_validate_cycle_vectorised.py
  - config/generated/admissible_box.yaml
  - tests/hardware/session_skills_r5_sitting7.md
  - logbook/2026-10-11-skill-stack-r5-hop-late-limit-jerk.md
  - logbook/INDEX.md
subsystem:
  - motion
tags:
  - catching
  - dynamics
  - learning
---

# The hop entry flew an uncorrected apex; Ball 1 arrived late and the P2 catch refused LIMIT_JERK

## Summary

Sitting 7's first two-ball `columns` attempts all ended `LIMIT_JERK` at the P2 catch of Ball 1 (238 / 246 / 258 /
254k mm/s³ against the 200k cap). Cause: the hop entry's cross-site throw of Ball 1 bypassed the learner
(`u = y_d`), so the plant flew its UNCORRECTED apex (1.010-1.021 m against 0.95, a 7-9 % overshoot); Ball 1 landed
+38..+43 ms late, the tracker-aimed P2 catch squeezed the dwell and the P1->P2 transit exceeded the jerk cap. Fixed
by letting the hop BORROW the learner's P1 column correction read-only (`SkillExecutor._hop_command`). Separately,
`validate_cycle` refusals now name the leg and the time they peaked at, because the ERROR line named no leg.

## Symptoms

Sitting 7 first attempt, 2026-10-11 afternoon (launch.log `~/.ros/log/2026-10-11-15-14-11-595711-jetson-3017477/`,
bag `~/Desktop/rosbags/2026-10-11_15-14-11`). Owner: Ball Butler feeds all caught smoothly; `columns_1ball` attempts
all finished (last catch or two a bit shaky); `columns_1ball_fed` passed at `num_cycles` 4 (messy, not top priority).
The feed-lateral probe was skipped live; run afterwards: n=8, median (+4.2, -0.8) mm, r 7.7 mm, 6/8 seated before
0.20 s (FAIL only on n<10; the bias is optional). The fed ball's first throw now lands -2..+33 mm x / -30..+6 mm y
(morning: +56..+74 / +39..+56), so the previous entry's fix (`94b4b3b3`) worked. Two-ball `columns`: 4/4 attempts
ended `LIMIT_JERK` at the P2 catch of Ball 1, plus one `ABORTED_BB_THROW_TIMEOUT`. The ERROR line said
`peak leg jerk ... > 200000` and named no leg.

## Diagnosis

Opus replay through the real `executor -> install_segment -> validate_cycle` path reproduced the robot's four
numbers to within ~2 % (scratchpad `j_*.txt`).

- The hop entry's throw of Ball 1 (`_command_u`, `entry_hop` branch) set `u = y_d`, bypassing the learner by
  design (a cross-site throw must not write a P1 memory row). Without the learner's apex correction the plant
  overshoots: 1.010-1.021 m against the 0.95 commanded, a 7-9 % overshoot, so Ball 1 lands +38..+43 ms LATE.
- The P2 catch is aimed at the TRACKER (`catch_aim_source` default), NOT at the schedule's plan. (The first brief's
  premise that it was aimed at the schedule plan was wrong and is withdrawn — see Withdrawn claims.) The carried
  Ball-2 release stays on the schedule instant, so a late catch squeezes the dwell between catch and release
  0.270 -> 0.227 s, and the P1->P2 transit peaks at leg 4/5 at Ball-2 release +0.10..0.20 s.
- Counterfactuals through the replay: `94b4b3b3` off = identical refusals (the fed-catch fix is not involved);
  feed angle / speed changes = identical; solve latency = none; an ON-TIME Ball 1 plans at 148-157k, OK. Landing-time
  window at dwell 0.27 is about [-15, +10..20] ms, so +38..+43 ms is well outside it on the late side.
- A learner-corrected P1 column throw flies 0.94-0.97 m (`u_apex` ~0.871 for `y_d` 0.95), which is the on-time case.

## Discussion

Options weighed, with the failure each one prevents or leaves:

1. **Hop borrows the learner's P1 column correction (CHOSEN).** Removes the root cause (the uncorrected apex) with
   the machinery that already makes column throws land on time. The hop's `(x, y_d)` equals a P1 column throw's
   (`dy` is relative to the target site's catch point and `y_d = (0, apex)` as for a column throw), so the query
   reads those rows. Read-only: the hop cannot WRITE rows, because pooling the cross-site throw into the column
   memory would corrupt it (the hop lands +25..+55 mm in y against the ~10 mm column correction — a different lateral
   behaviour). Answers the owner's question "can the hop learn from `columns_1ball` runs": yes, read-only.
2. **Dwell 0.27 -> 0.30 s (owner approved, then REJECTED by a pre-registered measurement).** Criterion fixed before
   the run: the largest dwell that passes the 90 % bench with a >= +35 ms late-landing window. Result: the 90 %
   bench passes only at 0.27 (0.28 / 0.29 / 0.30 all LIMIT_ACC; 0.30 fails even at 100 %, 102.8 %), reproducing the
   2026-10-04 finding recorded in the `skill_node.py` `_DEFAULT_DWELL_S` comment (scratchpad `m_bench_table.txt`,
   `m_window.txt`). A longer dwell just trades LIMIT_JERK for LIMIT_ACC.
3. **Let the carried release absorb ~20 ms of tracker lateness (DEFERRED).** Breaks "the release instant is the
   schedule's" and needs an owner decision. It is the reserve lever if the steady pattern still needs landings within
   about +-15 ms (the sitting-4 10 % margin class, now on the timing axis).
4. **Release the hop 40 ms earlier (REJECTED).** The release time is a calibrated constant and the ball is still at
   1.02 m apex; moving it papers over the overshoot.
   Rejected outright: QP jerk weights / bounds (worse: 234-249k) and separation 100 alone (239k).

Why the leg-naming change leaves `validate`, `_validate_shaped_batched` and `validate_follow` alone: the refusals
the sitting hit all come from `validate_cycle`, the gate `install_segment` runs on a unified cycle; the batched and
follow gates have different clocks and no caller that renders the leg, so adding the suffix there would be untested
surface for no diagnostic gain.

**Residual risks (not fixed here):** the hop throws from REST while most P1 memory rows are chained catch->throw, so
the borrowed correction is an approximation; the lateral residual (~20-35 mm) is probably catchable (Ball 2 seated
at 53-63 mm off today); a dedicated hop memory is the follow-up if Ball 1 still misses sideways.

## Fix

`executor.py`: `_command_u`'s `entry_hop` branch now calls the new `_hop_command(idx, site, dy, apex, x)` (the
`shadow_landing` branch still returns `y_d`). `_hop_command` queries `learner.command(x, y_d)` read-only at the
release site's column context, clips the lateral part to `lateral_authority_m` about `dy`, clips the apex to the
`(site, site)` admissible box when one is selectable, writes NO memory row, and on a learner exception or non-finite
result falls back to `u = y_d` with a note (never a refusal). It logs
`HOP-COMMAND skill N: P1 column correction: apex 0.950 -> 0.871 m, dy (+0, -10) mm (no memory row)`.

`feasibility.py` (`validate_cycle` only): LIMIT_VEL / LIMIT_ACC / LIMIT_JERK refusals now append
`(leg 4 at t=0.200 s on the gated plan's clock)` after the unchanged `peak ... > limit` prefix (0-based leg index,
as the bridge's heartbeat lines print; computed on refusal only, never on the OK hot path). New
`FeasibilityReport.peak_leg_index` / `peak_leg_t_s`, message-only (no gate reads them); an internal `span_acc` local
was renamed `leg_span_acc`. `report.py`: `short_refusal` renders `(leg 4 @ +0.200 s)` via a new `_WHERE` regex.

Tests: `test_skills_executor.py` (5 hop tests + the shadow-landing pin; 4 fail on HEAD's executor),
`test_validate_cycle.py`, `test_skills_report.py`, `test_validate_cycle_vectorised.py` (the parity twin strips the
suffix before comparing). `executor.py` and `feasibility.py` are gated files, so the admissible box needs a re-sweep.

## Verification

- Executor scoped run (2026-10-11, `python -m pytest tests/motion/test_skills_executor.py tests/motion/test_skills_schedule.py tests/ros/test_skill_node.py -q -p no:cacheprovider`):
  552 passed, 19 failed — all 19 in `tests/ros/test_skill_node.py` on the stale gate hash, before the re-sweep;
  the full gate below supersedes it.
- Leg/time tests strengthened after the audit (an independent per-span reference picks the binding leg and time;
  a knot-side jerk case, left and right; wrong-leg and wrong-time perturbations fail 3/3)
  (`python -m pytest tests/motion/test_validate_cycle.py tests/motion/test_validate_cycle_vectorised.py -q -p no:cacheprovider`,
  run 2026-10-11): 65 passed.
- Leg-naming scoped (`pytest tests/motion/ -q -p no:cacheprovider -n 4 -k "feasib or validate or report or refus"`,
  run 2026-10-11): 338 passed, 1 skipped.
- Box re-sweep S10b (2026-10-11 16:52-17:14, `scratchpad sweep_run_s7.sh S10b` = `tools/admissible_sweep.py`
  columns / hop / single, dwell 0.27, 350/5000/200000, hand 3900, BLAS 1 thread; merged single+hop+columns with
  `admissible.dump`): rc 0/0/0, gate `5072c89d4ddc`, all 19 boxes content-identical to `f974f9463a75` (only the
  hash line moved). The P1 columns box's commanded-apex range is 0.608-1.047 m, so the hop's 0.871 passes its
  apex clip untouched.
- Sims (2026-10-11, `python sim/skills_gate.py --learn --no-viewer --pattern columns --apex-m 0.95 --separation-mm 125
  --target-throws 30 --feed-angle-deg 11.9 --feed-speed-mmps 5600 --columns-entry hop [--one-ball-fed]`, BLAS 1 thread):
  two-ball PASS 5/5 seeds (33 makes, 0 drops each); one-ball-fed PASS 5/5 (37 makes, 0 drops). The sim starts from
  empty memory, so its hop gets the identity command — a no-regression check, not a test of the borrow.
- Corrected-hop install check (offline replay, scratchpad `m_drive.py` = brief J's executor -> install_segment ->
  validate_cycle driver, cached hop command U0, 2026-10-11): apex 0.871 with dy (0,0) / (0,-20) / (0,+20) /
  (+20,+20) mm installs OK at 68.9k / 72.8k / 72.6k / 83.5k peak leg jerk (cap 200k); Ball 2's catch 59-68k and
  the P2 catch of Ball 1 148.5k, all OK.
- Full gate (2026-10-11, `./run_tests.sh --full`, log `temp/logs/gate_full_hop_command_20261011.log`): parallel 6697 passed, 9 skipped, 1 xfailed in 379 s; serial 6 passed; exit 0.
- Hardware: owed — `tests/hardware/session_skills_r5_sitting7.md` § 4 (re-fly).

## Outcome

Expected on hardware: Ball 1's hop apex 0.93-0.97 m, landing inside the +-15 ms window, the P2 catch planned. Not
confirmed until the re-fly. If a refusal still occurs it now names the leg and the time.

## Withdrawn claims

- [2026-10-11] The brief for the replay claimed the P2 catch is aimed at the schedule's plan, so a late ball would
  merely be caught late.
  WITHDRAWN: `catch_aim_source` defaults to the tracker; the catch follows the late ball while the carried release
  stays on the schedule instant, which is what squeezes the dwell.
  Superseded by: the Diagnosis section above.

## Open Questions

- Does the borrowed correction hold when the hop throws from REST (most P1 rows are chained)? Watch Ball 1's landing
  time in the re-fly: late beyond ~+20 ms is the jerk wall, early beyond ~-15 ms is the acceleration wall.
- Option 3 (absorb tracker lateness in the carried release) and a dedicated hop memory are the reserve levers.
