---
title: "LIMIT_JERK at the seam knot after a late-catch BUMP: the bump edits the pose but not the cup track the seam velocity is re-derived from"
type: investigation
date: 2026-10-19
status: resolved
phase: "two-ball-skill-stack - post-R6 (columns, 17:47 sitting) / late catch re-aim BUMP (I-CATCH-5)"
related_plan: late-catch-reaim-bump.md
related:
  - 2026-10-19-cup-seating-cone-refuses-columns-stroke-braking.md
  - 2026-10-16-late-catch-reaim-bump-design.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/bump.py
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - tests/motion/test_skills_bump.py
  - tests/motion/test_unified_cycle_splice.py
  - tests/hardware/session_skill_stack_rehearsals.md
  - config/generated/admissible_box.yaml
subsystem:
  - motion
tags:
  - safety
  - dynamics
  - testing
---

# LIMIT_JERK at the seam knot after a late-catch BUMP

## Summary

The 2026-10-19 17:47 columns sitting flew 6 attempts; 5 of them ended `LIMIT_JERK` at the CATCH. All 16 distinct refusal events peak at t = 0.025 s on the re-gated range's clock, which is the SEAM knot itself, on legs 0, 2, 4 and 5 at 288k-609k mm/s^3 against the 200k limit. The owner saw no physical jerk: every one was a software refusal. Every ended attempt had an installed I-CATCH-5 BUMP (late catch re-aim) on the previous catch 21-24 knots earlier, and the refused catch splices at that bumped catch's carried RELEASE knot. Root cause: a bumped plan has two representations of one motion, `plan.pose` / `plan.pose_vel` (edited by the bump) and `meta.cup_plan` (not edited). `splice_at` -> `extend(head_truncated=True)` re-derives the seam knot's velocity from the stale cup track, so that one knot loses the bump's velocity `v_b` while both neighbours keep it. That is an acceleration jump of `8*v_b/h` at the seam, which the gate reads as jerk. The gate was right; a deterministic offline reproduction confirms it and shows the peak scales with `v_b`. Fix chosen: make the plan coherent (the bump overlays the cup track) and enforce coherence at the splice with a refusal rather than a silent repair. Status in-progress: diagnosis and reproduction done, Code landed 2026-10-19 evening, NOT FLOWN.

## Symptoms

17:47 sitting (`~/.ros/log/2026-10-19-17-47-52-793166-jetson-3314007/launch.log`, 1405 lines, bag `~/Desktop/rosbags/2026-10-19_17-47-52`). 6 columns attempts (hop entry, BB-fed). 47 `LIMIT_JERK` lines are 16 distinct events (the rest are skill_node echoes); one more refusal (A3's stop REST) is `CUP_CONTACT_ACC`, not jerk. Zero `CUP_SEATING_CONE`: the cone amendment did what it was for.

| # | outcome | refused skill (what) | range start knot | leg | peak (mm/s^3) | t on range clock |
|---|---|---|---|---|---|---|
| A1 | COMPLETED 3/4 | skill 4 RESEND of a CATCH (12.8 mm re-aim) | none (old wording) | 0 | 288850 | 0.400 s |
| A2 | `LIMIT_JERK` 7/8 | skill 8 CATCH (+ y_d fallback, + stop REST) | 177 = 4.425 s | 4 | 308857 | 0.025 s |
| A3 | `LIMIT_JERK` 5/6 | skill 6 CATCH (+ fallback; stop REST was `CUP_CONTACT_ACC`) | 131 = 3.275 s | 2 | 609339 | 0.025 s |
| A4 | `LIMIT_JERK` 2/3 | skill 3 CATCH | 319 = 7.975 s | 0 | 287782 | 0.025 s |
| A5 | `LIMIT_JERK` 1/2 | skill 2 CATCH | 267 = 6.675 s | 5 | 339163 | 0.025 s |
| A6 | `LIMIT_JERK` 1/3 | skill 3 CATCH | 329 = 8.225 s | 4 | 320193 | 0.025 s |

Peak values are the learned-aim-fallback refusal, the one the ENDED line reports. Each A2-A6 attempt has the same three refusals 0.2-0.4 s apart at the SAME range start (first dispatch with the learned prior, fallback at y_d, stop REST after the next release); A2-A6 refusals are never SPLICE_TOO_LATE (cliff margins +230..+380 ms). The first catch refusal ends the skill (`END LIMIT_JERK at skill N (CATCH)`); the stop REST refusal and the "hold installed" are the wind-down. The in-flight ball then lands on a held cup: 6 MISSED throws after the holds, 0 before the refusals in A2-A5. In every ended attempt the range start is `k_s - 1`, where `k_s` is the splice knot.

Owner report (2026-10-19 evening): no physical jerk at the robot, motion looked normal; nothing changed at the console this sitting (working point, limits, lean, robot position); the owner does not recall when LIMIT_JERK was last this consistent and asked for the history to be swept. A planner-side (seam) fix is acceptable if that is where the class lives, with the full gate and a rehearsal rung before it flies.

Frequency history (all launch.log directories 2026-10-02..2026-10-19, grep counts, read 2026-10-19): by line count 47 was last matched by 2026-10-14 21:27 (48) and 2026-10-16 10:50 (104), but those were RESEND refusals with peaks at t = 0.125-0.35 s and 0-1 ends. Five `ENDED (LIMIT_JERK)` in one sitting is matched only by 2026-10-04 19:52 (7) and 2026-10-11 15:14 (4). No log before 15:23 today has a t = 0.025 s refusal as an END, and the BUMP install count per sitting is: all sittings before 15:23: 0; 15:23: 2; 17:47: 6. 15:23 contained the same signature once (range knot 62, peaks 229k-237k, ended 1/3 caught), preceded by a BUMP 24 knots earlier. The 14:24 sitting had no t = 0.025 refusal (smallest t 0.050) and 0 bumps.

## Diagnosis

**The bump correlation.** 6 BUMPs installed at 17:47; 5 were followed 0.32-0.41 s later by a refused CATCH whose range starts 21-24 knots after the bump's `k_b`. The 6th (v_b 11.4 mm/s) was followed by an accepted catch. No refused CATCH lacked a preceding bump; the ~20 non-bumped catches installed fine. Peak jerk scales with the launch bias `v_b` (bump size alone does not separate the cases: 9.4-34.7 mm among the refusals).

| sitting / skill | BUMP v_b (mm/s) | next splice: peak leg jerk | per mm/s |
|---|---|---|---|
| 17:47 skill 7 -> 8 | 19.1 | 308857 (leg 4) | 16.2k |
| 17:47 skill 5 -> 6 | 34.8 | 609339 (leg 2) | 17.5k |
| 17:47 skill 2 -> 3 (A4) | 13.8 | 287782 (leg 0) | 20.9k |
| 17:47 skill 1 -> 2 (A5) | 20.1 | 339163 (leg 5) | 16.9k |
| 17:47 skill 2 -> 3 (A6) | 16.5 | 320193 (leg 4) | 19.4k |
| 15:23 skill 2 -> 3 | 11.2 | 229372-236601 | 20.5k |
| 17:47 skill 3 | 11.4 | accepted (about 200k predicted) | - |

**The mechanism.** One plan is held in two representations: `plan.pose` / `plan.pose_vel` (what the emitter streams and the gate reads) and `meta.cup_plan` (the cup translation track the pose was decomposed from, plus the tilts). The bump (`skills/bump.py`) adds `b(t)` to `plan.pose` xy and `b_dot` to `plan.pose_vel` xy; `_bumped_meta` overlays the pose and the release mark (`site_mm`, `vel_mm_s`) and the catch site, but not `meta.cup_plan`. The next catch splices at the bumped carried release (`_snap_to_release` pins `k_s` there; with 17 knots of handoff lead this is the case whenever the release is inside the dispatch window), seeded from `release_state_at_knot`, which takes pose/pose_vel from the bumped plan and cup pos/vel from the bumped MARK. The seed is consistent, so the positions-only `_seam_check` passes. Then `splice_at` calls `extend(head_truncated=True)`, and `extend` REPLACES the seam knot's velocity with `_seam_velocity`, which differences the concatenated `cup_plan` of head and tail. Index K of the head's `cup_plan` is the UNBUMPED value, so the re-derived `V[K]` has lost `v_b` exactly, while knot K-1 (bumped head) and K+1 (tail seeded from the bumped mark) still carry it. A velocity error `dV` at one knot with positions and neighbours unchanged moves the Hermite end accelerations by `+4*dV/h` (below the knot) and `-4*dV/h` (above it): an acceleration jump of `8*v_b/h`, about 6100 mm/s^2 of platform xy at `v_b` = 19 mm/s. The gate's per-knot straddling difference divides a jump by the 6.25 ms sub-step.

**Why exactly t = 0.025 s.** The re-gated range starts at `k_s - 1` (the stencil reads one head knot). View knot 0 has no straddling difference; view knot 1 is plan knot `k_s`, the seam knot, so a defect at the seam knot peaks at 0.025 s. A pure seam acceleration step also peaks there, whereas a seam velocity kink (the knot's own V consistent, head and tail differing) peaks at K+1, t = 0.050; the 0.050 s refusals seen at 11:04 and 14:24 are that other mechanism and were non-terminal RESENDs. A probe of the scalar stencil gives 43.7k mm/s^3 of platform-axis jerk per mm/s of `dV`, peaking at view t = 0.025; the observed 16-21k per mm/s of LEG jerk implies a leg projection of about 0.4 that depends on the bump direction, which is why legs 0/2/4/5 vary.

**Offline reproduction** (2026-10-19, scratchpad probe `repro_bump_splice.py`, real `compile` fixture of `tests/motion/test_skills_bump.py`, production `bump.apply_catch_bump` at 0.45 s before landing, production `executor.install_segment` splice, limits 350/5000/200000; deterministic, two runs identical; a copy is promoted to `tools/probes/` only if the orchestrator does so, until then the recipe lives in the session scratchpad). Splice snapped to K = 62; range starts at knot 61.

| v_b (mm/s) | pose_vel[K-1] | pose_vel[K] | cup_plan.vel[K] (stale) | seam V[K] | gate |
|---|---|---|---|---|---|
| 0 (control) | 0.17 | 1.49 | 0 | -0.43 | OK |
| 11.39 | -11.22 | -9.91 | 0 | -0.37 | LIMIT_JERK 208601 (leg 1, t = 0.025) |
| 18.98 | -18.81 | -17.50 | 0 | -0.32 | LIMIT_JERK 356446 (leg 1, t = 0.025) |
| 34.17 | -34.00 | -32.69 | 0 | -0.23 | LIMIT_JERK 664529 (leg 1, t = 0.025) |

Slope 18.3k, 18.8k, 19.4k per mm/s, inside the hardware 16-21k band, and the same first-knot signature in the message. The control passes. Two probe-only monkeypatches clear all three: (A) adding `b`, `b_dot`, `b_ddot` to `cup_plan` pos/vel/acc xy over all knots (seam V[K] then within 1.8 mm/s of `pose_vel[K]`, the same baseline re-derivation offset as the unbumped control's -0.43 vs 1.49); (A') setting the seam xy velocity to `plan.pose_vel[K]` (equal exactly). One discrepancy: the 11.4 mm/s case REFUSES offline (208.6k, just over 200k) but the hardware 11.4 case was accepted. The fixture's leg geometry differs from the hardware attempt (leg 1 offline against legs 0/2/4/5), the slope band overlaps, and this case sits within a few percent of the limit, so it is marginal, not a contradiction of the mechanism. Not checked offline: peak-jerk margin under the fixes, non-release splices, the tilt-borne channels with an analytic `b_ddot` against a finite difference.

## Discussion

**What was ruled out, and how.** The brief opened with three seam hypotheses and two alternatives.

- *The stencil at view knot 1 straddles the carried head and reads data built under another clock.* No. Reading `validate_cycle`, view knot 1 reads knot K-1 (position and velocity) and the replaced `V[K]`; it never reaches K-2, and the view is bit-identical to the joined plan on the retained span, so gating the whole plan reads the same floats. The head's carried values are correct; the REPLACED seam velocity is wrong.
- *Live-state drift or a head/tail dt mismatch.* Closed by construction. The head is carried bit for bit below K and the seed comes from plan floats, not the live state; `extend` raises on a differing dt. Drift would present as `STALE_STATE` or `SPLICE_TOO_LATE`, and no refusal here has either (cliff margins positive).
- *The trajectory shape, the working point, or this morning's merges.* No. No working point or limit changed at the console, and the refused catches are shaped like the ~20 non-bumped catches that installed fine. The robot-frame, capture-move and cone-amendment commits of the day do not touch `bump.py`, `_seam_velocity`, `splice_at`, `extend` or the jerk stencil. The bump commit of 2026-10-16 is the only commit on the path, and it first flew at 15:23 today. The cone amendment is an ENABLER, not a cause: it removed the `CUP_SEATING_CONE` ends that used to finish an attempt before a bump could fire, so attempts now live long enough to reach one.

**Why the gate was right, and why widening 200k is rejected.** The 200k value was never measured against a hardware tolerance. It first appears on 2026-07-07 as an administrative ceiling, was relabelled administrative on 2026-07-16, and became the R5 working point on 2026-10-05; the only physical content found is a guard-margin A/B at a much lower jerk. The firmware self-stop is about 9x higher (`stream_stop_leg_jerk_rps3` 25000 rev/s^3 is roughly 1.76e6 mm/s^3), and the real leg-path authority is the Teensy-side MAX_DEVIATION guard. So 200k is a smoothness and trackability assumption, not a damage bound, and raising it would be the cheapest edit. It is rejected on root cause: in this case the refusal caught a REAL command defect. The refused plan puts a one-knot velocity-feedforward dip of about `0.4*v_b` and a leg-acceleration doublet on the wire (the Teensy interpolates the Hermite at 500 Hz, so a C1 knot jump reaches the ODrive within a 2 ms tick). And the refusal scales with an unbounded overlay parameter: at the 50 mm/s lateral cap the same defect reads about 900k, so no widening that is safe for the bump is also meaningful as a gate. Widening would hide a coherence bug behind a larger number.

**Why Contract A over the narrower A', and over disabling the bump.** A' (set the seam xy velocity to `plan.pose_vel[K]`) clears the seam in the reproduction at all three `v_b`. It is rejected because it patches the consumer for one kind of splice. `meta.cup_plan` stays stale, and `state_at_knot` reads cup pos/vel/acc from it for every non-release splice after a bump (a resend, for instance), where the QP's start rows treat the cup acceleration as exact. The next consumer to read the stale track manufactures the next defect. Disabling the bump for columns (`BUMP_ENABLED_PATTERNS`) removes the defect today but gives up the late re-aim's catch gain, and the bump is the generic answer to the raw re-send refusals (87 of 87 raw re-aims past the snap band refused in the 10-16 sweep); it stays available only as an interim operational mitigation if the fix cannot fly. Contract A removes the cause: the pose and the track it was decomposed from agree after the overlay.

**The tradeoff accepted: a splice-time refusal, not a silent repair.** `splice_at` gets a coherence check: before `_seam_velocity`, compare the head's cup track over `[k_s-1, k_s+1]` with the pose over the same knots, and refuse `CHAIN_DISCONTINUITY` naming the stale representation when they disagree. The alternative is to re-decompose the cup track from the pose inside `splice_at` and carry on. That would have made today's sitting pass without anyone fixing `_bumped_meta`, which is exactly the problem: a silent repair hides the next overlay that forgets the tracks (a levelling trim or tilt bias, say), and the bug returns as a smaller, intermittent velocity-feedforward error nobody sees. The accepted cost is that a future incoherent overlay refuses a splice (a skill-ending refusal on the live stack) instead of flying slightly wrong. A loud refusal in the sim gate and the replay test is the cheaper failure.

**The class.** Not "a bump bug": a plan whose derived tracks disagree with its pose after an overlay. Any post-hoc edit of `plan.pose` that leaves `meta.cup_plan` or `meta.tilts` untouched feeds a wrong value to every reader of the derived track (`_seam_velocity`, `state_at_knot`, `release_state_at_knot`), and the symptom depends on which reader runs. The contract has the usual three parts: (1) invariant I-PLAN-10 in `INVARIANTS.md` ("`meta.cup_plan` and `meta.tilts` decompose to `plan.pose` within tolerance on every knot; any overlay of `plan.pose` updates them in the same function"); (2) one enforcement point, the coherence check in `splice_at`; (3) one test, a bump -> splice at its release replay that asserts the seam velocity equals `plan.pose_vel[K]` within the baseline re-derivation offset and that the gate passes, plus a negative test where an overlay that forgets the track is refused. A second discriminator for the other first-knot mechanisms is logged separately (below); only this one is closed here.

**The K-1 straddle hole, found alongside.** `extend_gate_range` starts the range at `k_s - 1`, but straddling differences are computed for interior knots only, so view knot 0 (= K-1) gets none. Re-deriving `V[K]` also changes `a_above(K-1)` (by `2*dV/h`), a jump nobody measures: the head's own gate saw the old `V[K]`, and the stencil derivation assumed span K-1 -> K unchanged, which `head_truncated` falsifies. The hole can only let a defect through, never refuse one. It is closed by starting the range one knot earlier when `head_truncated` (cost: one knot of gate), and the gate message's range-start text will change accordingly.

Open class members, not closed here (they differ in signature): a tilt-acceleration mismatch at K (tilt pinned in position only, peak also at 0.025 s but independent of any bump; a `seam_acc_mismatch` DEBUG field beside `seam_vel_mismatch` would measure it), and the velocity-kink class (peak at 0.050 s, the 11:04 and 14:24 RESENDs, non-terminal).

## Fix

Landed 2026-10-19 evening, in contract order (document, enforcement, tests). Four files.

**1. `INVARIANTS.md` row I-PLAN-10 (written first).** A plan's derived tracks agree with its pose: `meta.cup_plan` (pos/vel/acc) and `meta.tilts` decompose to `plan.pose` on every knot, and any overlay of the pose updates them in the same function with the analytic derivatives. The splice is the enforcement point. The I-CATCH-5 row's Fate now records that the bump flew at 15:23 / 17:47 and left `meta.cup_plan` stale, fixed the same day.

**2. `skills/bump.py`: the overlay updates the cup track.** `retarget_profile(..., with_acc=True)` returns the analytic `b_ddot` of the same three profile pieces (the default two-value return is bit-identical to before; the analytic acceleration agrees with a fine finite difference of `b_dot` to 0.035 mm/s² on a 3330 mm/s² peak). `_bumped_meta` now adds `b`, `b_dot`, `b_ddot` to `cup_plan.pos/vel/acc` xy in metres on every knot (zero through `k_b`, as the pose overlay is), and the per-span `jerk` as the difference of `b_ddot`, so the track's own rollout stays exact at the knots. `cup_plan.takeoff_vel` follows the existing `takeoff_vel_mps` rule. Not touched, with the reason: `meta.tilts` (the bump never moves `pose[:, 3:5]`), the shift terms (functions of cup z and tilt only, so adding the offset to cup xy is the exact preimage: the bumped track re-decomposes to the bumped pose within 1.4e-14 mm), the catch mark's velocity (`b_dot` is zero at the catch).

**3. `unified_cycle.py`: the splice refuses a stale track.** New `_refuse_stale_cup_track(plan, meta, k_s, limits)` decomposes the head's cup track over `[k_s-1, k_s+1]` with the same config `_seam_velocity` uses and compares xy against `plan.pose` within `_CUP_TRACK_POS_TOL_MM` (1e-3 mm) and tilt against `pose[:, 3:5]` within `_CUP_TRACK_TILT_TOL_RAD` (1e-6 rad). On a mismatch it raises `CycleInfeasible(CHAIN_DISCONTINUITY, ...)` naming `meta.cup_plan` (or `meta.tilts`), the knot, the axis, both values and the difference. `splice_at` calls it exactly when `k_s < n - 1`, which is when `head_truncated` makes `_seam_velocity` run, after the detach-cone refusal and before the head view is built. Positions are compared, not velocities: a coherent plan re-decomposes to identical floats, while the seam velocity legitimately differs from the stored `pose_vel` by a ~1.9 mm/s baseline on healthy and fixed plans alike, so a velocity comparison cannot separate stale from healthy. z is not compared because the decomposition's z uses a config rebuilt from the live limits. A plan with no cup track (magazine throws) is not checked.

**4. `unified_cycle.py`: the K-1 straddle hole.** `extend_gate_range(..., head_truncated=True)` starts the gated range at `k_s - 2` instead of `k_s - 1`; a pure `extend` is unchanged. Re-deriving `V[K]` changes the acceleration jump at `K-1`, and the old range omitted that straddle. This restores the width the retired `replan_tail` gated, costs one knot of gate on a splice, and re-judges the already-gated span `K-2 -> K-1` at the live limits. The `_VALIDATE_STENCIL_KNOTS` derivation comment records why its "span K-1 -> K unchanged" assumption is false for a truncated head. Chosen over evaluating the straddle inside `validate_cycle`, which would touch every caller of every gate.

Skipped: a `seam_acc_mismatch` report field (optional; kept the diff minimal). Not added: an A'-style silent repair of the seam velocity from `pose_vel[K]`; the contract wants the stale representation named, and with piece 2 in place the check never fires on the bump path.

Feedforward at the seam, one cycle (the 17:47 cycle at line 805, K = 178 = the bumped release, emitter near knot 165): before, `V[K]` lost `v_b` while `K-1` and `K+1` carried it, an acceleration jump of `8 v_b / h` at K that the gate refused. After, `V[K] = v + v_b` plus the ordinary ~1.9 mm/s seam baseline, so the tangent is continuous; every knot below K is bit-identical and the timing is unchanged.

## Verification

The (date, command, result) triple is required for each row; the full gate and the box re-sweep are filled when they finish.

| date | command | result |
|---|---|---|
| 2026-10-19 | `pytest tests/motion/test_skills_bump.py tests/motion/test_unified_cycle*.py tests/motion/test_validate_cycle.py tests/ros/test_install_segment.py tests/motion/test_skills_executor.py -q -n 4 --dist loadfile` (before the four new tests) | 523 passed, 2 skipped, 3 failed in 662.75 s; the 3 are wall-clock tests run under xdist at load average 15 (one is `serial`-marked); re-run serially the same day: 3 passed in 2.36 s |
| 2026-10-19 | `pytest tests/motion/test_skills_bump.py tests/motion/test_unified_cycle_splice.py -q -p no:cacheprovider` (the four I-PLAN-10 tests included) | 34 passed in 6.77 s |
| 2026-10-19 | repro re-run with the production code and the monkeypatches removed (scratchpad recipe above, `v_b` 0 / 11.39 / 18.98 / 34.17) | control OK; all three bumped splices ACCEPTED; with the old stale `meta.cup_plan` injected the same splice is refused `CHAIN_DISCONTINUITY` ("meta.cup_plan is STALE at knot 61 ... diff 16.7") |
| 2026-10-19 | `./run_tests.sh --full` (after the audit fixes and the box merge) | parallel: 6979 passed, 9 skipped, 1 failed in 316.09 s; serial: 7 passed in 27.85 s. The one failure is the known load flake `tests/ros/test_replay_overview.py::test_a_pass_still_exiting_for_an_older_version_neither_reports_computing_nor_blocks_the_requeue` (it also flaked in this morning's merged-tree gate); re-run scoped 3 x the same day: 3/3 passed (3.35 / 3.38 / 3.37 s). PASS |
| 2026-10-19 | `bash temp/sweeps/sweep_P1.sh` (four families x 2 runs, BLAS one thread, gated files frozen during the run; `unified_cycle.py` is one of the eight gate-hashed files) then the scratchpad merge against the committed box | single / hop / columns / oval each IDENTICAL across the two runs; merged box content-identical to the committed one (20 boxes, same limits); only `gate_hash` moved 2e23d662ead8 -> 1be1b456acc7 |

Already verified: 2026-10-19, offline reproduction with probe-only monkeypatches (A and A'), all three `v_b` accepted; control passes unmodified. NOT run: any hardware sitting.

## Outcome

Was NOT FLOWN at landing (2026-10-19 evening): the mechanism was traced and reproduced, and the contract, enforcement and tests landed that evening (full gate and box re-sweep recorded in Verification). The next columns sitting was the flight check, with rehearsal row 47 (`tests/hardware/session_skill_stack_rehearsals.md`) naming what to count: every BUMP install followed by an accepted catch, zero first-knot (`t = 0.025 s`) `LIMIT_JERK`, no `CHAIN_DISCONTINUITY` from the new check. The interim mitigation held in reserve (disable the bump for columns) was not needed.

**Flown 2026-10-21.** The 22:53 sitting (first on the ASA cup; `2026-10-22-hop-reaims-refused-mid-transit-tilt-rate-seam-and-the-new-cup.md`) was the flight check, row 47 PASSED: the launch ran the 95d148cd build (installed `unified_cycle.py` and `bump.py` md5-identical to source, `INSTALL_SKEW: OK`), the late-catch BUMP installed 20 times, 0 of them was followed by `LIMIT_JERK` at t = 0.025 s (10-19: 5 of 6 columns attempts), and 0 `CHAIN_DISCONTINUITY` fired. The sitting's other refusals were different classes, see that entry.

Residue, not scheduled:

- The 11.4 mm/s offline-vs-hardware marginal case (leg geometry), unexplained.
- Why 15:23's peaks (229k-237k) are lower than 17:47's (288k-609k) at similar `v_b`: the leg projection by bump direction is the likely reading, not tested.
- The `SPLICE_TOO_LATE` suffix on the throw line after a refusal names the ball's previous catch's last RESEND refusal (sticky per ball), not the current catch; a log-text clarity issue.
- The end-reason attribution gap (missed catch reported under the wrong end code) is unchanged.
- The sim `--reload` gate LIMIT_JERK regression (bisected to the 2026-09-30 change) is a separate, owner-decision item and is not this class.
