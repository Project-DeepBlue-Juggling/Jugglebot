---
title: R5 sitting 8 - two-ball columns GATE MET, 32 consecutive catches (BB feed + 31 throws) in one attempt; every Ball Butler feed flew (14/14, no self-cancel); five solver-late SPLICE_TOO_LATE ends and the residue findings carried
type: investigation
date: 2026-10-13
status: tuned
phase: "two-ball-skill-stack — R5 (two-ball columns)"
related_plan: two-ball-skill-stack.md
related:
  - 2026-10-13-skill-stack-r5-sitting-7-columns-pb.md
  - 2026-10-13-skill-stack-r5-wave-2-after-sitting-7.md
  - 2026-10-13-skill-stack-r5-wave-3-expected-landing-box-and-solver-worker.md
  - 2026-10-13-skill-stack-r5-bb-throw-self-cancel-and-phantom-catch.md
files_changed:
  - logbook/2026-10-13-skill-stack-r5-sitting-8-gate-met.md
  - logbook/INDEX.md
  - plans/active/two-ball-skill-stack.md
  - tests/hardware/session_skills_r5_sitting8.md
subsystem:
  - skill-stack
  - ball-butler
tags:
  - hardware-sitting
  - learner
  - gate
---

# R5 sitting 8: the 30-consecutive gate is met

## Summary

- **Gate (plan § 3 R5, owner ruling 2026-10-13): MET.** Ball Butler initiates (the hop entry counts, owner ruling); five consecutive cycles were met at sitting 7; **32 consecutive catches in one attempt** = the BB feed catch + throws 1-31 (the node's own verdict `31/32 caught`; throw 32 "missed" by construction, see Discussion 3f) against the 30 required.
- 12 fed attempts, catches before the first miss (feed included): **5, 5, 5, 0, 10, 4, 4, 11, 0, 10, 32, 9**.
- Wave 2's cancel fix, first flight: 14 BB throws dispatched, 0 `ENDED_DURING_BB_DISPATCH`, 0 `THROW_ABORTED_BY_HOST`, 0 `REJECTED_BB`. Owner: the feeds looked smooth.
- Not clean: 5 `SPLICE_TOO_LATE` ends (none an owner stop), 4 empty-cup `MISSED_CATCH`, one hand jam at the very end, `max_emit_gap` 47 ms. All carried, nothing fixed here.
- Pre-sitting gate: 2026-10-13 23:04, `./run_tests.sh --full`, parallel `6981 passed, 9 skipped, 1 xfailed`, serial `7 passed`, `RESULT: PASS` (`temp/logs/gate_bbfix_20261013.log`).

## What happened

Sources: LOG = `~/.ros/log/2026-10-13-23-12-50-197471-jetson-3905591/launch.log` (3113 lines; `L` = line); BAG = `~/Desktop/rosbags/2026-10-13_23-12-50_2-Ball-32-Catches`. The flown part is 23:14:26 to 23:22:02 (SIGINT L3100). Firmware per the log: can-bridge v30 (L117), Platform v7 (L116), ODrive 0.6.11-0 on 0-6 and BB 7, 8. `level_trim_deg = [+0.0945, +0.3678]` came from the launch default (L35). Box gate `5072c89d4ddc` per the runbook (not in the launch log). Limits quoted by the refusals: leg acc 5000 mm/s^2, leg jerk 200000 mm/s^3, hand acc 3900 rev/s^2.

**Counting rule (once).** Skill 1 is the fed catch of BB's Ball 2; skill 2 is the catch of Ball 1's hop throw (= "throw 1"); skill k+1 = "throw k". Streak = skill 1 plus the throws caught before the first MISSED / MISSED_CATCH / END. The fed catch has no verdict line of its own (only `CATCH-EVIDENCE-ABSTAIN ... cup read SEATED`, e.g. L1057), so it counts as caught when the cup read SEATED and the pattern went on to throw Ball 2, and as missed when `END MISSED_CATCH (skill 1)` fires.

**Warm-ups** (before the first fed accept, L1034): `reload` (BB throw, JB catch only, 23:14:26); self_toss 4/4; hop 4/4; columns_1ball 2/2; columns_1ball 10 ended `SPLICE_TOO_LATE` twice (2/3, 1/2); columns_1ball `reload` 5/5.

**Fed attempts** (feed seat = first cup-SEATED relative to the scheduled landing):

| # | local | pattern, n | node done | end | streak | feed seat s |
|---|---|---|---|---|---|---|
| 1 | 23:17:10 | columns_1ball_fed, 10 (ran 5) | 4/5 | done; throw 5 MISSED (final throw, apex 1.042) | 5 | +0.118 |
| 2 | 23:17:47 | columns, 10 | 5/6 | SPLICE_TOO_LATE skill 6; throw 5 MISSED | 5 | +0.110 |
| 3 | 23:18:02 | columns, 10 | 4/6 | LIMIT_ACC (stop REST refused CUP_CONTACT_ACC); throws 5, 6 MISSED | 5 | +0.119 |
| 4 | 23:18:17 | columns, 10 | 0/0 | REJECTED_NO_BALL at THROW (finding 3b) | 0 | n/a |
| 5 | 23:18:34 | columns, 10 | 9/10 | done; throw 10 MISSED (final throw) | 10 | +0.095 |
| 6 | 23:19:17 | columns, 32 | 5/5 | MISSED_CATCH Ball 2 skill 5, cup EMPTY 59/59 | 4 | +0.092 |
| 7 | 23:19:29 | columns, 32 | 4/5 | SPLICE_TOO_LATE skill 5; throw 4 MISSED | 4 | +0.080 |
| 8 | 23:19:38 | columns, 32 | 12/12 | MISSED_CATCH Ball 1 skill 12, cup EMPTY 60/60 | 11 | +0.099 |
| 9 | 23:19:57 | columns, 32 | 2/2 | MISSED_CATCH Ball 2 skill 1 = the FEED, cup EMPTY 57/57 | 0 | none |
| 10 | 23:20:07 | columns, 32 | 10/11 | SPLICE_TOO_LATE skill 11; throw 10 MISSED (apex 0.969, +45,+42) | 10 | +0.095 |
| **11** | 23:20:30 | columns, 32 | **31/32** | done (L2817); throws 1-31 CAUGHT, 32 MISSED | **32** | +0.036 |
| 12 | 23:21:20 | columns, 32 | 10/10 | MISSED_CATCH Ball 1 skill 10, cup EMPTY 55/55, then HAND JAM; ABORTED_MODE_CHANGED | 9 | +0.077 |

Throws-only (feed excluded): 4, 4, 4, 0, 9, 3, 3, 10, 0, 9, 31, 8. Three attempts kept catching after their first miss by design ("Ball N continues to its own catch"); the streak stops at the first miss regardless.

**Gate attempt (#11, accept L2415 `Ball 2 lands in 3.25 s`), compact.** Throws 1-31 are numbered without gap (L2455..L2813). Apex 0.936-0.958 m. Landing |x| max 31 mm (throw 30), |y| max 51 mm (throw 6), node `worst landing 51 mm`. Six throws have no learner row (8, 10, 17, 20, 25, 29). Resend refusals ("re-aimed 0x") sat on throws 2, 3, 4, 11, 12, 15, 24, 28, 31 (LIMIT_ACC 2, INFEASIBLE 3, LIMIT_JERK 3, UNVERIFIED 1). Release offset -1 ms at throw 1, drifting to -5 ms by throw 22, **+20 ms from throw 24** to the end. Arrival offset -5..+3 ms except throw 31 (+21). Throw 32: apex 1.036 m, landed +47,+27, seat -0.08, "landed on the held ball".

**Feed quality** (BB, 14 dispatches: 2 warm-up reloads, 1 columns_1ball_fed, 11 columns):

| item | value |
|---|---|
| BB `/bb/throw_outcome` | 14 messages, all `OK`, for 14 dispatches; 13 offered to a catch, 1 thrown at an idle platform (attempt 4) |
| BB command | `in 2.5 s, flight 0.88 s, 3.29 m/s, aim correction (-31, -5) mm` for all 11 columns/fed dispatches (attempt 4: `in 5.9 s`); the aim correction never adapted |
| JB-side feed catch | `CATCH-AIM skill 1 source=schedule` 12/12; `TRACKER-IDENTITY-REFUSED skill 1` 8/12 at about -0.34 s and -0.170 s in the gate attempt (band +-0.150 s) |
| Feed seat | +0.036..+0.119 s after schedule in 11 of 12; attempt 9 cup EMPTY |
| Probe `feed_lateral_miss.py` (`temp/probes/s8_feed_lateral.csv`, `.log`) | n=11, dx med -6.8, dy med -8.6, r med 12.9 mm, 10/11 seated before 0.20 s, PASS; recommended `columns_feed_bb_bias_mm = [-6.8, -8.6]` (frame: +x toward ball A's site) |
| Runbook row 23 premise | "+27 mm toward P2" NOT reproduced |
| Node OUTCOME lines, JB-thrown balls caught at the columns (12 fed attempts) | Ball 2 (P1) n=44 mean (+3.9,+6.7) sd (25.4,23.5); Ball 1 n=43 mean (-1.2,-4.8) sd (17.6,19.3); all n=81 mean (-0.5,+0.9) sd (20.1,22.0), rms radius 29.6 mm |

## Discussion

**1. Why the gate is met under the plan's own wording.** The plan § 3 R5 row reads "five consecutive cycles, then 30 CONSECUTIVE catches in one attempt (owner ruling 2026-10-13)". The owner's ruling on the BB-initiated clause: the hop entry counts as "BB initiates" (wave 2 open item 4, confirmed by wave 3's follow-up). Under the counting rule above attempt 11 is 32 consecutive catches, or 31 if one counts only throw verdicts and excludes the feed catch; both exceed 30, so the gate does not hinge on the feed-counting convention. Honest caveat: the feed catch is counted from the cup-SEATED evidence, not a node verdict. The ground-truth probe cannot independently confirm the 31 (Discussion 3d).

**2. The learning curve against sitting 7's.** Sitting 7 L2 (catches per attempt): 3/4, 6, 5, 3, 2, 9/10, 5, 11, 2, 0, 2, 7, 15 (PB), 3. Sitting 8 fed attempts: 5, 5, 5, 0, 10, 4, 4, 11, 0, 10, 32, 9. Between the sittings three things landed that act on the cause list of sitting 7: wave 2's learner-predicted catch prior with the 35 mm inward re-aim, and the level trim as a launch default (L35); wave 3's expected-landing box (values unchanged) and the segment-solve worker process; the BB self-cancel fix. What the evidence supports: no lane latch occurred (0 `sched_hold_latched`, 0 `lane_recovery`), no `ENDED_DURING_BB_DISPATCH`, and the sitting-7 "ball 32-48 mm off-centre on the P2 side" misses are not the dominant end (the ends are SPLICE_TOO_LATE 5, empty-cup MISSED_CATCH 4, LIMIT_ACC 1, REJECTED_NO_BALL 1, hand jam 1). What it does not support: which of the three changes produced the 32; one attempt of 12 reaching the gate with a 32 against an otherwise 0-11 spread is a high-variance result, and the median fed attempt is 5. No causal attribution is claimed.

**3. Findings** (ranked; none fixed here; carried to `plans/active/skill-stack-dress-rehearsals-and-residue.md` § 2).

*a. SPLICE_TOO_LATE is a live class: 5 ends in 19 goals* (2 columns_1ball warm-ups, fed attempts 2, 7, 10), none an owner Stop (no Stop line exists) and none coincident with an emitter stall (max_emit_gap 25-30 ms within +-0.6 s of each; one window 0.0 at 23:20:16.6). The node ends its own attempt because a CATCH install cannot be wired in time: four of five "reply transit outran the wire deadline", one "solver did not answer by the wire deadline", request-to-refusal 0.23-0.26 s each time, on the catch right after a normal outcome, against CATCH solves of p50 123 / max 208 ms. The refusal carries no solve time and no worker-lateness line exists after the worker's READY (L83), so the interval between worker and wire is invisible in the log. This is the seam wave 3's open item 3 (the solve/plan split) was written for: the split is now measured (plan p50 110.6 / p99 203.6 / max 362.6 ms; solve p50 103.7 / p99 192.2 / max 357.1 ms; 40 solves over 150 ms, all CATCH), and it shows a CATCH solve that regularly spends most of a 0.25 s budget. Attempts 2, 7 and 10 were ending while they were catching (4-10 consecutive), so the streak list is not a ceiling on the machine.

*b. Attempt 4: REJECTED_NO_BALL fired after BB was committed.* Attempt 3 ended LIMIT_ACC with the hand 7.1 rev from home; the opening REST homed it (+7.4214 -> +0.3071 rev over 5.34 s, L1397), so BB was asked `in 5.9 s`; the REST solve (357 ms, the largest of the sitting) outran the origin by 0.436 s. The hop-entry premise "Ball 1 held at P1" was false (attempt 3 left two balls out of the cup) and nothing at accept (L1407) checks the cup; it is tested only when skill 0 fires, 5.4 s later (L1434). The cancel was sent with the release 0.09 s away and BB answered `CANCEL_TOO_LATE` (wind-up under way, release in 23 ms, L1440-1441): the ball left (`bb/throw: OK` L1442) and flew at an idle platform. It is the one of 14 BB throws that went to nothing. The cancel path itself worked as designed; the premise check ran after dispatch.

*c. Four empty-cup MISSED_CATCH events* (attempts 6, 8, 9, 12), attempt 9 on the FEED catch (`TRACKER-IDENTITY-REFUSED -0.350 s`, schedule aim). Common precursor: a re-aim above 10 mm was refused or skipped on the catch before (`AIM-LATERAL-CLAMPED` +54.4 mm in y, `RESEND-SKIPPED NO-CONVERGED-FIT` 22.0 mm off, `LIMIT_JERK` + `BOX-EXPECTED-LANDING-UNMET`, `RESEND-REFUSED LIMIT_ACC`). A correlation across four events, not a mechanism.

*d. Six of 31 gate throws had no learner row* (no converged ballistic fit; 26-35 estimates refused; throws 8, 10, 17, 20, 25, 29; 19 %); 17 across the 12 fed attempts. The log does not say why the fit refuses. Seat was +0.00 or +0.2-0.3 on them, so the catches were fine. Ground-truth probe (`throw_outcome_bag_probe.py`, `temp/probes/throw_outcome_summary.md`): 131 flights, ground truth on 129; cup-SEATED caught 125, not caught 6; every probe not-caught flight lines up with a node miss, and no node CAUGHT is a ground-truth miss. It cannot confirm the 31 independently: the cup sensor is TRUE when the other ball is already held, so five node MISSED throws read "SEATED" in the probe (including throw 32).

*e. Release-timing step at throw 24.* Release offset -2..-5 ms through throws 12-23, then +20 ms at throw 24 and held at +18..+20 through the end: a step of about 24 ms, one knot at 40 Hz (25 ms). Nothing logged at the moment (no corrected-by line, no frame-offset or re-aim line); the only coincident DEBUG is `RESEND-REFUSED LIMIT_JERK` at skill 25 (leg 0 jerk 397547 > 200000), a kind that also sits on throws 12 and 31 without a step. The +20 ms state is a recurring per-attempt mode in this sitting (attempts 3, 8, and 11's second half; others sit at -3..+4 ms), here entered mid-attempt. Catches were unaffected (arrival within -5..+3 ms).

*f. The 32nd throw.* Apex 1.036 m (+8..+10 % over the others), landed +47,+27, seat -0.08, "landed on the held ball". It is the FINAL throw of the pattern, which by design is not caught in the cup: the cup holds the other ball, and the last ball rests on the platform. So `31/32` is 31 of 31 catch opportunities, and throw 32 is not a real miss. Same signature on attempt 1's final throw (apex 1.042 m, +66,-27) and attempt 5's (seat -0.09). It is not a learner command change (the last rows have u2 0.868-0.873). Why the final throw flies high is not in the log.

*g. Stall numbers against the wave-3 targets* (`/trajectory/status` `max_emit_gap_ms`, 2741 five-Hz windows):

| scope | windows | p50 | p99 | max |
|---|---|---|---|---|
| all | 2741 | 25.88 | 29.94 | 47.04 |
| streaming only | 2133 | 25.92 | 29.94 | 47.04 |
| fed attempts | 1303 | 25.94 | 30.19 | 47.04 |
| gate attempt | 113 | 25.96 | 32.54 | 35.13 |

Target p99 < 30 ms: **met** overall (29.94) but not inside the gate attempt (32.5) or the fed-attempts scope (30.19). Target max < 35 ms: **NOT met** (47.04 at 23:18:36, 2.6 s after attempt 5's accept and the same instant as BB throw #7; plus 36.0, 35.4, 35.1). L2's baseline was 34.3 / 45.7, so the worst window is the old one. The worker's RSS was never logged (open).

*h. Frames lost and incidentals.* 4 WARN `QTM frames lost` lines (L204-299, 39-49 %) at 23:13:17-23:14:16, all during the BB calibration sweeps and before the first goal (consistent with the "never calibrate during a goal" rule). 14 leg/hand encoder heartbeat gaps (up to 1397 ms, leg 3 at 23:18:40 inside attempt 5). Mocap frame offset rose 1.7 -> 6.8 mm across the sitting. One `CATCH-AIM-FALLBACK` (learned prior refused LIMIT_ACC, L1375). 79 `AIM-LATERAL-CLAMPED`; 137 `BOX-EXPECTED-LANDING` (18 UNMET); 0 `BOX-FORWARD-CLIP`. LIMIT_* ENDs: LIMIT_ACC x1, LIMIT_JERK 0, HAND_LIMIT_C2 0 (the 10 % margin class did not end a gate-relevant attempt).

*i. Hand jam at the end (attempt 12).* MISSED_CATCH skill 10 (cup EMPTY 55/55, L3016), 0.3 s later `HAND JAM detected: hand stalled at +2.546 rev under a descending command (cmd +0.546 rev), iq -48.4 A` (L3017); guard LATCHED, ACTIVE -> FAULT (L3029), ABORTED_MODE_CHANGED. The node's final `10/10 caught` text then misreports a dead pattern (the streak was 9). Cause of the stuck hand is not in the log (a ball under the cup is one reading).

**4. BB dispatch ordering, for the record.** 14 dispatches: 12 reply-first (`awaiting ThrowAnnouncement`), 2 swap-first (`answered after the announcement swapped`, L341 and L810, both `reload`-family goals whose announcement install has no columns compile). For columns the announcement must compile and SOLVE the fed schedule first (52-357 ms), so the reply wins the race (`_still_current` true). Both branches are guarded (`_swapped_in_and_live`); neither cancelled a throw.

**5. Withdrawn claims.** None. One brief-level correction: `missed_catch_fixture.py` has no CLI (hard-coded to the 2026-10-04 bags), so runbook row 29's flags do not exist; it was not run, and running it with `--help` rewrote `tests/motion/fixtures/missed_catch_20261004.csv` with identical content (`git status` clean).

## Fix

No code change. The R5 hardware gate is met; findings are carried as residue in `plans/active/skill-stack-dress-rehearsals-and-residue.md` § 2 (created tonight), which also holds the § 2 dress rehearsals (rows 17a-20) the owner pushed out of this sitting.

## Verification

| item | (date, command, result) / path |
|---|---|
| Pre-sitting full gate | 2026-10-13 23:04, `./run_tests.sh --full` (`Summary [full]`), parallel `6981 passed, 9 skipped, 1 xfailed in 353.57s`, serial `7 passed, 6991 deselected in 27.72s`, `RESULT: PASS`, `GATE_EXIT=0`; `temp/logs/gate_bbfix_20261013.log` |
| Bag / launch log | `~/Desktop/rosbags/2026-10-13_23-12-50_2-Ball-32-Catches` (rosbag recorder killed by SIGINT at shutdown, L3103); `~/.ros/log/2026-10-13-23-12-50-197471-jetson-3905591/launch.log` |
| Firmware | `BRIDGE_FW_CHECK: OK - can-bridge v30 [install_skew=0]` (L117); Platform Teensy v7 (L116); ODrive checks PASSED (L120, L122) |
| Row 29 probe | 2026-10-13, `python tools/probes/throw_outcome_bag_probe.py --bag <bag>`, 131 flights; `temp/probes/s8_throw_outcome.log`, `temp/probes/throw_outcome_2026-10-13_23-12-50_2-Ball-32-Catches.csv`, `temp/probes/throw_outcome_summary.md` |
| Feed probe | 2026-10-13, `python tools/probes/feed_lateral_miss.py --run s8 <log> <bag> --out temp/probes/s8_feed_lateral.csv`, `n=11 ... PASS` |
| Row 30 memory | `temp/learn/jugglebot/memory_backup_20261013_sitting8.csv` (112239 bytes, 633 lines); rows 535 -> 632, +97 over the sitting |
| Row 32a stall | `/trajectory/status` from the bag, reader `scratchpad/stall8.py` (session scratch); numbers in 3g |
| Logbook/plans tests | 2026-10-14, `pytest -q -p no:cacheprovider tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py tests/sim/test_plans_index.py`, result: 112 passed in 0.85 s |

Not extracted: per-feed landing offsets (the probe prints medians), worker/node RSS, the BB-side release jitter, the cup-sensor state before attempt 4's refusal, `gate_hash` in the launch log.

## Open

- Rehearsals not run (runbook rows 17a-20; owner, time): moved to `plans/active/skill-stack-dress-rehearsals-and-residue.md` and `tests/hardware/session_skill_stack_rehearsals.md`.
- Residue rows (SPLICE_TOO_LATE self-ends, hop-entry accept not checking the cup, the 47 ms emitter gap, the throw-24 release step, the attempt-12 hand jam): rows in `plans/active/skill-stack-dress-rehearsals-and-residue.md` § 2.
- R6 close-out proceeds tonight.
