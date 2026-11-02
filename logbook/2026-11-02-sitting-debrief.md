---
title: "Sitting 2026-11-02 debrief: Apex ladder to 1.35 and Columns 58/58, 64 of 65; JB-120 sharpened, JB-137 sub-hypothesis withdrawn, JB-23 folded into JB-149"
type: investigation
date: 2026-11-02
status: in-progress
phase: "pattern progression"
related_plan: pattern-progression.md
related_issues: []
files_changed:
  - docs/records.md
  - logbook/2026-11-02-sitting-debrief.md
  - logbook/INDEX.md
subsystem: [motion, ros, can, tools]
tags: [safety, performance, dynamics, docs]
---

# Sitting 2026-11-02 debrief

Second run of `/sitting-debrief`. Analysis first, then paperwork. The code this analysis produced is logged separately in `2026-11-02-bump-brake-seed-and-debrief-observability.md`. Related items: JB-120, JB-121, JB-136, JB-139, JB-23, JB-149, JB-137, JB-138, JB-24, JB-98, JB-147.

## Summary

Two Sittings, both on the live limits: Hand 4500 rev/s^2, `hand_curr_limit_a` 60 A, leg vel 350 mm/s, box gate_hash 1749cca8bc23 (100 cells, C-HAND-5 release step).

- **17:00 "Higher-throws-with-JB-144-update"** (launch log `~/.ros/log/2026-11-02-17-00-38-831989-jetson-3156841/launch.log`, 6821 lines): the peer's JB-144 Apex ladder, 0.95 to 1.35. JB-144 was already closed by the peer.
- **18:13** (launch log `~/.ros/log/2026-11-02-18-13-42-782567-jetson-3226387/launch.log`, 11315 lines; bag `~/Desktop/rosbags/2026-11-02_18-13-42/`): the Runbook rows for JB-120, JB-139, JB-121, JB-23, plus an exploratory Hop goal that is not a Runbook row.
- The 16:52 launch is excluded by the owner.

Ends over both Sittings (30 goal ends): MISSED_CATCH 16, LIMIT_VEL 5, HAND_LIMIT_ACC 3, LANE_REFUSED 3, WRONG_MODE 1 (the SPINOUT). **Records:** Columns 58/58 clean (18:20, ended LANE_REFUSED) and 65 Throws / 64 Caught (18:21, LIMIT_VEL), the previous best being 59 / 57; Hop 61/61 at Apex 0.9, sep 250 (exploratory goal, ledger records it anyway). `docs/records.md` committed at close-out.

## Verdicts

Judged literally against each Runbook row's own `pass:` line; the owner's typed box is shown beside it. The owner decides where they differ.

| Item | owner box | literal | evidence | write-back |
|---|---|---|---|---|
| JB-120 | fail | fail | goal L417 (51 Throws, 49/51) ended HAND_LIMIT_ACC 4509.26 > 4500 at L1489 | stays open, branch held |
| JB-139 | pass | **fail (literal)** | mean y of first 5 Ball 2 landings on the fresh plant_id = +16.4 mm, outside +20..+32 | owner then chose "bias confirmed, keep open" |
| JB-121 | pass | pass | 16 of 16 Columns starts reached 2+ Catches; no inward AIM-LATERAL-CLAMPED in the start window | closed |
| JB-23 | pass | passes on the qualifying goal, **differs in substance** | goal L417 max 33.0 ms, but launch max 81.7 ms and 3 of the goals over 35 ms | owner folded it into JB-149; closed as superseded |
| JB-136 | (no row) | verified | `grep -c HAND_STROKE` = 0 in both launch logs, Hop start included | closed |
| JB-138 | (no row) | progress | peak \|iq\| 60.4 A once (the SPINOUT), every other goal at or below 44.7 A | stays open |

## Symptoms and Diagnosis

**HAND_LIMIT_ACC (3 ends: S1 L1779 4655.19, S2 L1521 4509.26, S2 L8868 4629.05).** All three are the first install of a Catch seeded from a bumped predecessor: 3 refusals in 270 bumped successors, 0 in 329 unbumped. They are exactly the three largest inward lateral Bumps of both Sittings (22.2, 22.7, 30.5 mm of 277 accepted); every inward Bump of 21.4 mm or less was planned. The margin is not collapsing in general: no unbumped seed ever set a peak above 3810 (84.7 %) at Apex 0.95.

**Offline mechanism.** The peak sits in span 1-2 after the bumped release, the post-release brake. `bump._bumped_meta` updated the release mark's site and velocity but not `brake_accel_mm_s2`, so the C-HAND-5 seed kept its pre-bump direction. With the seed refreshed to the bumped take-off direction the refused L1489 case goes from 4696 to 4062 offline in the diagnosis variant, and to 3407 with the shipped fix (which also carries the bumped cup track's release-knot acceleration). C-HAND-5 made the bumped case worse above a crossover near 13-15 mm; below it the step lowers the successor by about 600 rev/s^2.

**LIMIT_VEL (5 ends, S1 L936, L1044; S2 L2387, L6328, L6805).** All five are the Ball 2 Catch refused on leg 1 at plan t = 0.175 s, excess +0.2 % to +3.1 % over 350 mm/s. The install lines read 97-99.9 % leg velocity, but those leg figures are running maxima over the plan buffer (the same artefact as the Hand peak, corrected below for the Hand only), so how close an unbumped CATCH runs to 350 mm/s is not known from the log; the gate's compare has zero tolerance. Every refused Ball 2 CATCH followed a re-aimed or bumped Ball 1 catch (2 of 5 a RESEND only), and the brake-seed fix moves the successor's leg velocity by only about 0.6 %. The y_d fallback changes the aim by only 1-5 mm: 10 fallbacks, 5 absorbed, 5 ended. JB-24's class, with the refinement that the margin is absent, not thin. 1 such end on 2026-11-01, 5 today.

**LANE_REFUSED (3 ends, S1 L4334, S2 L1699, S2 L4941).** One mechanism: a streamed lane went more than 70 ms without a forwarded setpoint, the firmware sent STOP, then it refused the resume. Every forward gap over 70 ms produced a LANE_REFUSED (3 of 3) and every LANE_REFUSED had one. Two of the three then tripped the Hand guard (MAX_DEVIATION) before the hold landed: the hold reached the firmware 131-239 ms after END, the guard tripped 73-101 ms after END.
- teensy_bridge_node starved (S1 L4334 forward gap 101.3 ms; S2 L1699 79.6 ms), emitter clean: confirms the 2026-11-01 high-low mechanism. Whole bridge process stalled (link_status publish gap 172 ms).
- trajectory_node emitter starved (S2 L4941, emitter 81.7 ms, bridge loop 12.8 ms): JB-98's original branch.
- Common cause probable, not proven: CFS preemption during CATCH-solve bursts (solves with wall time well above CPU time, ivcsw 47-124; the solver worker is not SCHED_FIFO). The logs cannot tell preemption from a GIL or GC pause.

**MISSED_CATCH (16 ends, not 19: 8 per ball).** Each end is one lost ball. Five wild Throws with an identity-refused fit 240-450 mm off the site; 2 early/short flights; 6 of 8 Ball 2 ends fly with the Ball 2 learner command pinned at x -20 (JB-139's signature, 5 explicit BOX-EXPECTED-LANDING-UNMET); 4 left no evidence. Collisions: no log marker; one candidate (S1 L6198/L6200). The owner reads these as mostly bad Throws, few collisions.

**SPINOUT at 1.35 (S1 L5132, JB-147, the peer's).** Brake -60.4 A saturated at 168 to 78 rev/s, Hand overran to 10.83 rev, WRONG_MODE end. About 190 launches at Apex 1.05-1.35 gave one overshoot. After the disarm `iq_meas` froze at -60.42 A for about 39 s: a stale value, not a 39 s overcurrent.

## Discussion

**Hypotheses reframed or withdrawn:**

1. **JB-137's "node 6 loses CAN arbitration" sub-hypothesis is withdrawn.** All 7 Hand dropout episodes today (502-900 ms) had a fresh encoder (age max 2-24 ms), while the legs, which have higher CAN priority, dropped more often (14 in S1, 31 in S2) and reached encoder age 93-99 ms. The staleness is bus- or bridge-wide, not Hand-specific, and `hb_stale_mask` and `lead_clamp_mask` were 0 in every pre-event window. The 11-01 turned-90 mechanism is cannot-tell; no turned goal was flown.
2. **JB-23 is reframed into JB-149.** The row's literal pass holds on one goal, but the tail is real (S2: 81.7 ms, 63.2 ms and a 45 ms WARN; S1 max 39.0 ms) and it is not an emitter-only fault: it is the 70 ms cliff above, fed by two different starved threads. A single "emitter max" Item cannot own that. JB-149 owns the cliff; JB-23 is closed as superseded.
3. **JB-120 is sharpened.** The 11-01 reading "bump-seeded successor, 7 of 7" becomes: inward Bumps of 22 mm or more, plus a stale brake seed that C-HAND-5 (landed after the Bump code) made worse. The structural cause is a ripple miss of c3cc39bc, not a collapsing margin.
4. **The install-line peak was an artefact.** A first read of the S2 install lines said 61 % of Catches were above 90 % of the Hand limit, "the solve rides 4500". The line printed a running maximum over the plan buffer (`_merged_peaks`, monotone within every goal), so raw unbumped values were earlier bumped Catches' peaks carried forward. Corrected statement: no unbumped install ever set a maximum above 3633 (S2) or 3810 (S1). Production has no in-QP Hand acceleration row, so a peak percentage is true margin to the gate.
5. **Why the stream is now starved (owner Q2).** The 10-28 work protected the solve from the streams; now the solve is the aggressor and takes the CPU from them.

**Owner Q&A.**
- (1) A Bump is the late form of the continuous Catch update; the gap is the uncertified successor seed. Fix (a) closes the seed; the Bump decision also planning Catch N+1 remains the backstop.
- (4) The SPINOUT is the peer's 70 A plus top-margin work. Apex 1.25 is the working ceiling until the capture sees an overshoot.

**ODrive note.** The drive's JSON says soft 70 / hard 80 A (the peer's 3d875cce backup), but the code pushes `hand_curr_limit_a` (60 A at these Sittings) to soft max at configure, so the live drive ran at 60 soft. The owner's "80 hard / 60 soft" matches what ran.

**Choices not taken.** Raising the Hand limit to admit 4655 would sit 1.2 % under the 60 A authority bound and only moves the cliff (a 30 mm Bump extrapolates to about 5.0k). A fitted cap of 21.5 mm on inward Bumps would have touched exactly the 3 refused Bumps of 277, but leaves landing error uncorrected; it is a stopgap, not taken.

## Fix

Code in `2026-11-02-bump-brake-seed-and-debrief-observability.md`: Bump brake seed re-derivation, segment-own Hand peak on the install line, gap-tracker reset on re-arm, MISSED-BALL line. Gate: `./run_tests.sh` (2026-11-02, in the debrief worktree on origin/main 4761e68e): 8004 passed, 54 skipped, 1 failed + 5 serial passed; the one failure was `tests/sim/test_glossary_avoid_words.py::test_no_avoid_words_in_live_docs` on an Avoid-word in the new JB-149 body, fixed (doc-only) and re-run with the named doc tests below. Named doc tests after the fix (2026-11-02, `python -m pytest -q tests/sim/test_glossary_avoid_words.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_plans_index.py tests/sim/test_mapkit*.py`): 186 passed in 49.37 s

## Open decisions for the owner (numbers from the lane analysis)

- **Wire frame (JB-149).** Cover is 2 knots x 25 ms + 20 ms = 70 ms. Gaps seen: 79.6, 82.7, 101.3 ms today, 109 ms on 11-01. A 3-knot frame (95 ms) covers 2 of today's 3 and none of 11-01's; a 4-knot frame (120 ms) covers all 4. Costs a PROTOCOL bump and a firmware flash, and a dead stream holds 50 ms later.
- **Bridge-side reaction.** Start the lane-recovery converge from the forwarder on its first frame after a gap over the cover (it already knows the firmware STOPped), removing about 200 ms of heartbeat -> skill_node -> trajectory_node latency. Needs an offline replay to see whether it beats the guard mid-stroke.
- **Partition.** Pin the solver worker to cores 4-5 and trajectory_node plus teensy_bridge_node to 0-3, or move the 40 Hz forwarder out of Python. Probe first: `gc.callbacks` in the bridge plus `nr_involuntary_switches` sampling around CATCH solves.
- **JB-139: re-seed or widen the clamp.** Re-seeding the main plant's Ball 2 memory is data-only and zero risk; widening the learner clamp from 20 to 30 mm needs the box re-swept (about 2 h) and no gated-file edits until commit.
- **LIMIT_VEL (JB-24):** first, a non-threshold fix: when the y_d fallback is also refused, retry on the next ticks instead of ending; the 0.96 x 350 = 336 mm/s planning row or a 365 mm/s session limit are the threshold options.

## Open Questions

- Which of CPU preemption and a GIL/GC pause stalls the bridge (needs the probe above).
- Whether the lost ball's release is hand-side or tracker-side for the 5 wild Throws and 4 evidence-less ends (only the bag has it; the new MISSED-BALL line covers the future).
- Whether the SPINOUT's cause is resolvable at 100 Hz iq (no; needs the JB-147 capture at Apex 1.25-1.35).

## Close-out

Per the `/sitting-debrief` protocol: no branch merged; JB-120's branch `night-shift/2026-11-01-JB-120` stays held because it failed. `docs/records.md` and the memory files committed; Runbook regenerated. Follow-ups filed: JB-149 (the 70 ms cliff).

## Verification

Analysis only; no Sitting-time test counts are claimed here. Gate for the commit that carries this entry and the code: `./run_tests.sh` (2026-11-02, in the debrief worktree on origin/main 4761e68e): 8004 passed, 54 skipped, 1 failed + 5 serial passed; the one failure was `tests/sim/test_glossary_avoid_words.py::test_no_avoid_words_in_live_docs` on an Avoid-word in the new JB-149 body, fixed (doc-only) and re-run with the named doc tests below. Named doc tests after the fix (2026-11-02, `python -m pytest -q tests/sim/test_glossary_avoid_words.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_plans_index.py tests/sim/test_mapkit*.py`): 186 passed in 49.37 s
