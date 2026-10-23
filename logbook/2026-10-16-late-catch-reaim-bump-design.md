---
title: "The late catch re-aim is refused 87/87 past the snap band because it re-solves the catch-with-throw from a mid-swing seed; the same corrections installed as a platform-xy bump on the installed plan (hand untouched, held through the carried release with a retargeting launch velocity) pass validate_cycle on 87 % of the columns cases offline, worst-case direction"
type: investigation
date: 2026-10-16
status: resolved
phase: "late-catch-reaim-bump — design + offline prototype (not implemented)"
related_plan: late-catch-reaim-bump.md
related:
  - 2026-10-16-handoff-lead-flown-zero-splice-too-late-tracker-fit-late.md
  - 2026-10-16-tracker-flight-fit-late-since-matcher-fix.md
  - 2026-10-16-handoff-lead-8-knots-and-snap-aware-resend-fences.md
files_changed:
  - tools/probes/late_reaim/parse_resends.py
  - tools/probes/late_reaim/capture_records.py
  - tools/probes/late_reaim/bump_replay.py
  - plans/archived/late-catch-reaim-bump.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/bump.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - tests/motion/test_skills_bump.py
  - tools/probes/README.md
  - ros_ws/src/jugglebot_interfaces/srv/InstallSegment.srv
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - sim/skills_gate.py
  - tests/ros/conftest.py
  - tests/ros/test_install_segment.py
  - tests/ros/test_skill_node.py
  - tests/hardware/session_skill_stack_rehearsals.md
---

# The late catch re-aim as a bump: characterisation and offline prototype

**Data:** launch logs `~/.ros/log/2026-10-16-10-50-42-798495-jetson-1033523` (am, columns + reload), `…-16-10-37-689586-jetson-1220502` (pm1, hop / magazine / columns_1ball_fed), `…-16-23-01-479044-jetson-1230939` (pm2, columns_1ball_fed). Bags were not needed: the RESEND lines carry the delta, and the CATCH-AIM lines carry the committed landing instant.

## Symptom
The tracker fit now converges about 0.6 s before landing. Only ~15 % of catches take the correction. Re-aims that installed: 25 caught / 0 missed. Refused: 45 caught / 10 missed. (These outcome numbers are from the P2 entry, not from this entry's scripts.)

## Characterisation (`parse_resends.py`)
- **146 re-sends: 53 installed, 93 refused.** The refused set is 87 once the 6 `CATCH_AXIS` refusals on the fresh-origin reload/hop catch from a tilted rest are excluded. That is a geometry refusal of the seed and out of scope here.
- The 87 by pattern: columns 72, hop 8, columns_1ball_fed 7. By code: LIMIT_JERK 48, LIMIT_ACC 13, INFEASIBLE 12, HAND_LIMIT_C2 5 (all hop), SPLICE_TOO_LATE 5, LIMIT_VEL 2, SINGULAR 1, WINDOW_TOO_SHORT 1.
- **Time to the committed landing at the re-send:** refused p50 0.463 s (min 0.360, p90 0.572). Installed p50 0.588 s (min 0.445): the installed ones are mostly the pinned path inside the snap band.
- **Delta** (the `moved_mm` max component): p50 20 mm, p90 35, max 40, the authority bounds. Timing |Δt| p50 6 ms, max 41 ms.
- The raw splice is at `t + LEAD_S` (0.225 s), so 0.135-0.43 s is left to landing (p50 ~0.24 s). Ending the ramp at the contact window (0.125 s) leaves p50 ~0.11 s. A quintic 35 mm over 0.11 s needs 16.7 m/s² of Cartesian acceleration and ~1600 m/s³ of jerk before any of the plan's own demand (a Cartesian-demand proxy; the session limits are LEG-space, 5000 mm/s² and 200 000 mm/s³). Over the bump's 5-knot lead with a 50 ms pre-hold (T p50 ≈ 0.29 s) it needs v 228 mm/s, a 2.4 m/s², j 88 m/s³ (Cartesian).
- Why the raw path fails, as far as the logs show:
  - `CycleState` carries no hand acceleration, so the re-solved hand joins the head C1. HAND_LIMIT_C2 measures exactly that knot-acceleration step (4.44 vs 3.9 rev/s²).
  - The leg LIMIT_JERK/ACC peaks sit at t = 0.275-0.35 s on the gated plan's clock. That is past a ~0.24 s splice-to-landing, i.e. in the carry/throw that the catch-with-throw re-solve also re-plans. This assumes the gated clock starts at the splice, which was not verified.
  - A bump changes neither the hand nor the throw's shape, so neither mechanism applies.

## Discussion
**Representation.** Two choices were on the table:
- (a) An offset on the cup xy before IK, then a re-realise. Rejected: it re-runs the realiser (tilts, slider) and needs a solve, which is the cost being avoided.
- (b) An offset on the knot setpoints of the installed record, `pose[:,0:2] += b`, `pose_vel[:,0:2] += ḃ`. **Chosen.**

For (b): a platform-xy translation moves the cup by exactly b at any tilt or slider position. The knots are Hermite (pos, vel) on the 25 ms grid, so the bumped plan is the installed Hermite plus the Hermite of b, and `validate_cycle` measures precisely what the wire will emit. The bump is identically zero, with zero derivatives, through `k_b = k_wire + 2`. The record keeps its `t0`, and every knot the wire may have read is bit-identical. The splice contract (3-knot read, `k_wire`) reduces to `k_b > k_wire` at install, with no seam to gate. The hand arrays are untouched.

**The tail after the catch** was the decisive question. There were three candidates:
- **Return before the release.** At 5 knots, pre-hold 0: 917/1152 (79.6 %), with 84 LIMIT_JERK on the return. It is FORBIDDEN in any case: the owner rule of 2026-09-28 (`executor._catch_terminal`) is that the platform does not translate while it holds a ball, and a 20 mm return in the dwell made throws miss by 32-114 mm.
- **Hold Δ through the release, unchanged.** 1005/1152 (87.2 %). But the throw then lands Δ off its target and the next catch must bump by Δ. In columns that makes the landing error a random walk, error_{n+1} = error_n + scatter. Rejected.
- **Hold through the release AND retarget.** **Chosen.** It is the same rate as plain hold, because neither the retarget nor the settle term adds a refusal.
  - **First version (withdrawn after audit, 2026-10-16).** `v_b = -Δ/T_f` with the velocity ramp starting at the catch. That ramp drifts the cup by `v_b·(T_a/2 + pre_s)` before release, so the release sat at ~0.8 Δ and the throw landed ~0.2 Δ short. The offset was also never returned, so the rest-terminal sat ~0.65 Δ off the site.
  - **Corrected.** `v_b = -Δ/(T_f + T_a/2 + pre_s)` makes the landing exact, with `b(t_rel) = Δ + v_b(T_a/2 + pre_s)`. After the post-release hold, a rest-to-rest quintic returns offset and velocity to 0 by the record's last knot.
  - **Measured** with `--check` and by direct call, Δ = (20, 0) on the record with t_c 1.705, t_rel 1.975, end 2.275: `b(t_rel)` = 16.52 mm, `ḃ(t_rel)` = -18.77 mm/s, landing offset 0.000, `b(t_end)` = `ḃ(t_end)` = 0.000.
  - **The carry rule.** This is a DELIBERATE, BOUNDED exception to the 2026-09-28 rule that the platform does not translate between catch and release. That rule came from an unplanned 85-110 mm/s launch. The planned launch here is ≤ 38 mm/s at columns timing (≤ 46 mm/s bound, |Δ| ≤ 40.3 mm from the clamped landing), against today's ≤ 25 mm/s. This is an open owner decision.

**Fences.** The bump costs about one `validate_cycle` on the bumped span (~6 ms here: 1152 cases × 2 gate calls ran in 14.2 s), so its lead is 5 knots (3 wire + 2), not `LEAD_S`. The lead matters: the same bump at 9 knots with a 0.125 s pre-hold installs 5 %. A gate refusal is `BUMP-WINDOW`: non-terminal, no cap spent, retried only when the fitted Δ has shrunk by `resend_pos_tol_mm`. That is the converse of the 2026-09-20 terminal rule, which was justified by a 40-130 ms solve per refusal. `BUMP-FREEZE` (no ramp time left) is terminal. The full list of which existing fences still apply (NO-CONVERGED-FIT, WITHIN-TOLERANCE, CAP-SPENT, the lateral clamp, PINNED-BUDGET), which are replaced (CONTACT-WINDOW, MIN_WINDOW, `catch_freeze_s` → BUMP-FREEZE at `t_c - 0.225`), the non-terminal node-side `SPLICE_TOO_LATE`, the 25-50 ms latency allowance of `k_b = k_wire + 2`, and the 0.050 s pre-hold against the 0.125 s contact window, is in the plan, § 2.

**Learner.** The row keeps the cached `u` and records `x` with the BUMPED release offset. The bumped-and-retargeted throw is exactly "release offset b(t_rel), command u" (b(t_rel) is the REALISED release offset, ~0.83 Δ at columns timing, not Δ), the same row an installed re-solve writes. Recording the committed offset would teach the prior a release that never happened.

**Ruled out:** a scaled-down partial bump. It is a silent clip; the brief and the I-PLAN contracts say refuse.

## Offline prototype (`capture_records.py` → `bump_replay.py`)
**Method:**
- Sim columns at apex 0.95 / sep 125 and the live session limits (350 / 5000 / 200 000, hand 3900).
- 4 interior catches × every recorded (Δ, time-to-landing) × 4 directions. The diagonal is the worst case, since `moved_mm` is the max component.
- Each bump is applied to the record active at that instant, and `validate_cycle` runs on knots `[k_b-4, end]`. An unbumped slice is gated first; zero baseline failures.

**Caveat:** the hop and the reload geometry are not matched (sep 250 plans no catch in this gate), so hop is a columns proxy.

| Variant (`--retarget`, corrected) | columns 72 rows: diag / axis | hop 8 (proxy) | 1ball_fed 7 | 53 live-installed rows |
|---|---|---|---|---|
| live raw re-solve | 0 % | 0 % | 0 % | 100 % (selection) |
| 5 kn, pre-hold 0 | 1005 (87.2 %) / 1084 (94.1 %) | - | - | - |
| **5 kn, pre-hold 0.050 s (design)** | **1001 (86.9 %) / 1091 (94.7 %)** | 128/128 | 112/112 | 848/848 |
| 5 kn, pre-hold 0.125 s | 558 (48.4 %) / 761 (66.1 %, pre-W1 run) | 121/128 (pre-W1 run) | 92/112 (pre-W1 run) | - |
| 9 kn, pre-hold 0.125 s | 60 (5.2 %) | - | - | - |

"Pre-W1 run" marks numbers measured with the withdrawn retarget. Every refusal in both versions is on the ramp-in, so those numbers are unchanged in kind. The corrected re-runs that overlap reproduce the old counts exactly: 1005, 1084, 1001, 558, 60.

Every remaining refusal is on the ramp-in (LIMIT_VEL first, then ACC/JERK), none in the carry or settle.

## Fix
Not implemented in this unit. The design, interface, fences, invariant (I-CATCH-5) and rehearsal row are in `plans/archived/late-catch-reaim-bump.md`. Implementing it needs a node-side request on the install service (the record lives in trajectory_node), an interface rebuild, and the executor path. It is safety-bearing leg-path work and needs its own unit with a 40 Hz walk.

## Verification
All runs are from the repo root with the project venv and `OPENBLAS_NUM_THREADS=1`. `B` = `python tools/probes/late_reaim/bump_replay.py --records temp/probes/late_reaim/records_columns_0.pkl --rows temp/probes/late_reaim/resends_2026-10-16.json --lead-knots 5`. The corrected-version runs are saved in `temp/probes/late_reaim/runs_w1.txt`.
- (2026-10-16, `python tools/probes/late_reaim/parse_resends.py temp/probes/late_reaim/resends_2026-10-16.json`) → 146 rows, the counts above.
- (2026-10-16, `python tools/probes/late_reaim/capture_records.py --throws 10`) → 12 installs, 10 CATCH accepted, `records_columns_0.pkl`. `--apex 0.9 --sep 250 --tag hopproxy` (and `--sep 200`, `--sep 175`) → 2 installs, 0 CATCH accepted.
- (2026-10-16, `B --retarget --patterns columns --hold-s 0`) → 1005/1152; `… --dirs axis` → 1084/1152.
- (2026-10-16, `B --retarget --patterns columns --hold-s 0.05`) → 1001/1152; `… --dirs axis` → 1091/1152.
- (2026-10-16, `B --retarget --hold-s 0.05 --patterns hop`) → 128/128; `--patterns columns_1ball_fed` → 112/112; `--installed-only` → 848/848.
- (2026-10-16, `B --retarget --patterns columns --hold-s 0.125`) → 558/1152; `--lead-knots 9 --retarget --patterns columns --hold-s 0.125` → 60/1152.
- (2026-10-16, pre-W1 retarget) `B --retarget --patterns columns --hold-s 0.125 --dirs axis` → 761/1152; `--patterns hop --hold-s 0.125` → 121/128; `--patterns columns_1ball_fed --hold-s 0.125` → 92/112.
- (2026-10-16, `B --hold-through --patterns columns --hold-s 0`) → 1005/1152; `B --patterns columns --hold-s 0` (return-before-release) → 917/1152. The ~14 s wall time for 1152 cases is the source of "~6 ms per gate call".
- (2026-10-16, `B --retarget --patterns columns_1ball_fed --check`, default hold 0.05) → `b(t_rel)` 16.67 mm per axis for Δ (20.2, 20.2), landing offset 0, `b(t_end)` 0.

## Implementation G1 (2026-10-16): the pure layer
**What landed:**
- `motion/skills/bump.py::apply_catch_bump` is the ONE enforcement point of the new I-CATCH-5 (INVARIANTS § 7). It is the probe's `retarget_profile` math ported unchanged, with the 40 Hz walk in the module docstring.
- `SkillExecutor.catch_bump` (pure, applies the fences) and `commit_catch_bump` (counts, `_carried_release_mm` = old + `b(t_rel)`, live terminal).
- `tests/motion/test_skills_bump.py` (11 tests).
- The probe's `--retarget` branch now calls the production function; its own copy of the math is deleted.
- Nothing is wired into `_resend_live_catch` or the node. That is G2.

**Owner decisions 2026-10-16, recorded in the plan § 2:**
- The launch bias is accepted with a bound of 50 mm/s (`BUMP-LAUNCH`, non-terminal).
- Columns first: `BUMP_ENABLED_PATTERNS` = columns / columns_1ball_fed, and the executor defaults to off. Hop and reload stay off until a matching-record capture exists.

**Two differences from the prototype, both deliberate:**
- (a) `k_b` is knot-aligned: the ramp starts at `(floor(t/dt) + 5)·dt`, up to one knot EARLIER than the probe's continuous `t + 0.125`. Zero-through-`k_b` holds either way. The ramp is longer, so the rate rises. `t_ramp_start_s` may only move the start later (an earlier value raises), and that is what `--legacy-start` uses for the exact reproduction.
- (b) The 50 mm/s bound applies to the Euclidean `|v_b|`. The probe's "diagonal" cases (Δ up to (40, 40), |Δ| 56.6 mm) exceed the clamped |Δ| ≤ 40.3 mm, so the bound declines 80 of them. Those cases sit beyond what the clamp can deliver, so the axis row is the physically representative one.

**Verification:**
- All runs use the project venv, `OPENBLAS_NUM_THREADS=1`, `B` as above with `--retarget --hold-s 0.05`.
- The inputs were regenerated: the earlier worktree's `temp/` was gone. `parse_resends.py` gave 146 rows again, and `capture_records.py --throws 10` gave 12 installs, 10 CATCH accepted. Outputs are in `temp/probes/late_reaim/runs_g1*.txt`.
- (2026-10-16, `B --legacy-start --no-launch-bound` × {columns, columns axis, hop, columns_1ball_fed, installed-only, columns `--hold-s 0`}) → **1001 / 1091 / 128 / 112 / 848 / 1005**. That is the design row exactly, through the production function.
- (2026-10-16, `B` production defaults: knot-aligned, 50 mm/s) → columns diagonal **1000/1152** (BUMP-LAUNCH 80, LIMIT_VEL 50, JERK 14, ACC 8), `--no-launch-bound` 1049/1152; axis **1116/1152** (the bound declines none); hop 128/128; columns_1ball_fed 112/112; installed-only 848/848.
- `--check`: landing offset 0.0, `b(t_end)` 1e-14, hand bit-identical.
- (2026-10-16, `pytest tests/motion/test_skills_bump.py tests/motion/test_skills_executor.py tests/motion/test_cup_cycle.py tests/motion/test_cup_contact_contract.py -q -p no:cacheprovider`) → **353 passed in 10.37 s**.
- `./run_tests.sh` was not run in this unit; it is owed before the commit.

## Implementation G2 (2026-10-16): the wiring

- **Service shape.** `InstallSegment.srv` gained `KIND_BUMP=4` plus `bump_delta_xy_mm`, `t_dispatch_s` (request) and `terminal`, `release_offset_xy_mm`, `launch_bias_xy_mm_s` (response; `splice_k` carries `k_b`, `t_event_s` the committed landing). It is one service rather than a second because the bump has to share the install path's guard ladder (guard latch, lane latch, mode, seeded, fresh telemetry), its hold epoch and its mutually-exclusive callback group. A second service on its own group could run in the middle of a segment solve on the same record. Interfaces changed, so the rebuild is `colcon build --packages-select jugglebot_interfaces jugglebot` followed by a full launch restart (rehearsal row 31).
- **Node.** `trajectory_node._svc_bump_catch` runs after the guard ladder, where any guard refusal is terminal for a bump. It refuses `BUMP-NO-RECORD` (terminal) when there is no active segment record or the record holds no catch within dt/2 of `t_event_s`, and refuses `BUMP_HANDOFF` (non-terminal) while a StreamHandoff is pending. Otherwise it calls `bump.bump_record(..., t_apply_s=now)` outside `_plan_lock`, asserts the hand arrays are bit-identical, and swaps through `_install(..., t0=<same t0>, require_epoch, cycle=(plan, meta, t0), cycle_owner=segment, reset_replans=True, stream_handoff=False, precondition=...)`. The precondition re-checks record identity, the handoff and `k_b > k_wire(now)` (`SPLICE_TOO_LATE`, non-terminal) in the same lock acquisition that does the swap. The new `precondition` parameter on `_install` is the only change to that function. `_refusal_timing` is not used for a bump, which has no solve, no worker reply and no cliff. Its refusal line ends `[check=bump:<code>, plan X ms, k_b N, k_wire@dispatch N, k_wire@node N, dispatch->entry +Y ms]`. The 40 Hz walk is in the method docstring. `_install_continuity_ok` is not called: knots `<= k_b` are bit-identical and `k_b > k_wire(now)`, so the stream state at now cannot differ.
- **Executor.** `SkillExecutor(bumper=...)`. `_resend_live_catch` decides the route BEFORE the `catch_freeze_s` return. The bump route is taken when the bump is enabled, a bumper is wired, the catch is outside the snap band (`maybe_snapped` False) and the catch carries a throw. On that route only `resend_min_interval_s` and the `catch_bump` fences apply, with BUMP-FREEZE at t_c − 0.225 s. Inside the band the pinned re-solve keeps PINNED-BUDGET and its fences. `catch_bump` / `_bump_live_catch` share `_catch_bump_decide(t, apply, landing)`. Log lines are `BUMP skill N: ... installed b(t_rel) X mm, v_b Y mm/s` and `BUMP-DECLINED <code> skill N: ...`, the latter once per catch and code, all at DEBUG through skill_node's single tick-line call site. Bumps are not appended to `results`, whose consumers read InstallResult fields.
- **BUMP-NO-CARRY decision: fall back to the raw re-solve.** The last catch carries no release, so the bump has no throw to retarget. A plain-catch re-solve is the easier problem: all 87 refusals were catch-with-throw. A refused re-solve on the attempt's final catch also costs nothing further, because no later catch depends on it. The executor therefore keeps the raw path when `terminal.then_throw is None`. A node-side NO-CARRY on a catch that does carry a throw (carry or settle shorter than 2 knots) stays terminal, since a raw re-solve's window there is tighter still.
- **Two small rules added.** BUMP-LAUNCH joins BUMP-WINDOW's shrink-retry rule: |v_b| is linear in Δ at fixed timing, so the same Δ can only be refused again. `BUMP_HANDOFF` / `SPLICE_TOO_LATE` retry as-is (`bump.BUMP_RETRY_CODES`).
- **Switch.** `skill_node._start_pattern` sets `_bump_enabled = bump_enabled_for(goal pattern)` after the refusal ladder, and all seven `SkillExecutor` constructions take `**self._bump_kwargs()`. The sim gate keys on `Schedule.pattern`, which is `'columns'` for both columns compilers, so the sim's `columns_1ball` is on where the robot's goal name `columns_1ball` is off. Its bumper is `bump_record` over the local record, and it also moves the carried release's pending entry (site + b(t_rel), takeoff + v_b).
- **ThrowAnnouncement of a bumped release (measured verdict, no consumer affected).** The announcement still carries the pre-bump release site and launch velocity: off by at most `b(t_rel)` ~ 40 mm in release site and `|v_b|` <= 50 mm/s in launch velocity. Its only consumers are the tracker's announced-flight gate (`announced_gate` 200 mm plus the speed-ratio / direction-cosine tests on a ~3.8 m/s flight: 50 mm/s is 1.3 % of the speed, ~0.8 deg) and the executor's `TRACKED_LANDING_LATERAL_IDENTITY_MM` 200 mm. Landing position and time are unchanged because the retarget is exact. The raw re-send path already left the announcement stale by up to 20 mm.
- **Launch bound = TOTAL lateral launch velocity (owner reading, applied after the audit).** `apply_catch_bump` now tests `|rm.vel_mm_s[:2] + v_b|` against 50 mm/s (the plan's own release lateral velocity is ~0 for a columns self-toss, <= ~25 mm/s after a carried offset); `BumpResult.launch_bias_xy_mm_s` still reports the bias alone. The bound is unchanged. `bump_replay.py --retarget --hold-s 0.05` under the total bound (2026-10-16): columns diagonal **993/1152** (was 1000; BUMP-LAUNCH 76, LIMIT_VEL 62, JERK 13, ACC 8), columns axis **1116/1152** (unchanged), columns_1ball_fed **112/112** (unchanged), installed-only **839/848** (was 848/848; 9 BUMP-LAUNCH at |delta| ~35 mm, to_land ~0.55-0.60), hop 128/128 (off anyway). The stricter bound lowers diagonal and installed-only by a few cases; `--check` still prints landing offset 0.0. The `test_skills_bump` executor fixture's then-throw targeted `p1` from `p2` (a cross-column throw with ~117 mm/s lateral velocity, which the total bound correctly declines); it now self-tosses to `p2`, like the real columns compiler.
- **Peaks.** After a successful bump `_install` the node writes `_last_peak_vel/acc/jerk` from the bumped plan's `validate_cycle` report (as `_accept_segment` does), so `/trajectory/status` and `/diagnose` do not read 0.0 for the bumped record.
- **Swap-time re-checks tested.** Node tests cover: the clock passing k_b between the entry check and the swap (non-terminal SPLICE_TOO_LATE, `_active_plan` / `_cycle` / `_plan_t0` unchanged), the record superseded inside the compute (terminal BUMP-NO-RECORD, record unchanged), and `_install(precondition=lambda: False)` leaving the record untouched.
- **Sim coverage of the bump: none; hardware row 31 is the first exercise.** The sim bumper keeps a tally (`sim.skills_gate.BUMP_STATS`, printed in the summary lines as `bump: N accepted, M declined (...)`). Measured 2026-10-16: the default columns gate (`gate.run()`, 4 throws) and the small columns learner run (seed 0, target_throws 5) both reach the bump route **0 times** (0 accepted, 0 declined): no sim columns trial produces a post-band `from_fit` re-aim. No test asserts otherwise; none was faked. Two sim constructions (the learner runs, `skills_gate.py` lines ~2172 and ~2681) pass `resend_max_per_catch=0`, where CAP-SPENT blocks the bump regardless. The sim's `columns_1ball` has the bump ON (it keys on `Schedule.pattern == 'columns'`) while the robot's goal of that name has it OFF.
- **Verification (G2).**
  - (2026-10-16, `pytest tests/motion/test_skills_bump.py tests/ros/test_install_segment.py tests/ros/test_skill_node.py tests/motion/test_skills_executor.py tests/sim/test_skills_gate.py -q -p no:cacheprovider -k "not real_worker_process"`) -> 649 passed, 1 deselected in 150.27 s (auditor's run).
  - 2026-10-16, `./run_tests.sh --full` on the branch rebased onto skill-stack 7e74f42f (load average < 1): parallel 6701 passed, 11 skipped, 1 failed — `tests/ros/test_skills_plan_bench.py::test_the_kind_enum_matches_the_srv`, which pins the srv kind enum and had not been told about `KIND_BUMP=4`; the expected dict was extended and that file re-run: 69 passed; serial tail 7 passed.

## Outcome

Implemented 2026-10-16, the stale cup-track seam defect closed by I-PLAN-10 on 2026-10-19, flown with the criterion MET on 2026-10-21 and 2026-10-22 (74.1 % and 79.5 % installed, zero bump-caused LIMIT/guard events), and the plan archived 2026-10-23.
