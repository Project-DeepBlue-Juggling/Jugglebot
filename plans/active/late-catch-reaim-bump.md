---
title: The late catch re-aim as a platform-xy bump on the installed plan (the raw-path re-solve is refused on physics every time)
created: 2026-10-16
status: active
owner: harrison
last_updated: 2026-10-16
related_logbook:
  - 2026-10-16-late-catch-reaim-bump-design.md
  - 2026-10-16-handoff-lead-flown-zero-splice-too-late-tracker-fit-late.md
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py::SkillExecutor._resend_live_catch
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py::SkillExecutor._catch_terminal
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py::validate_cycle
  - tools/probes/late_reaim/bump_replay.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/bump.py::apply_catch_bump
---

# Late catch re-aim as a bump

## 1. Problem
Past the snap band, the re-send re-solves the whole catch-with-throw from a mid-swing splice at `t + LEAD_S`. On 2026-10-16 that was refused 87/87 (columns 72, hop 8, columns_1ball_fed 7; LIMIT_JERK 48, LIMIT_ACC 13, INFEASIBLE 12, HAND_LIMIT_C2 5, other 9; `parse_resends.py`), and each refusal ends re-aiming. The fit arrives with a median 0.46 s to go (0.36-0.65). The move is 20 / 35 / 40 mm (p50 / p90 / max component). The outcome split is from the P2 entry, not scripted: installed re-aims 25 caught / 0 missed, refused 45 / 10.

## 2. Decision (owner decisions recorded 2026-10-16, see below)
Replace the raw path after the snap band with a **bump** `b(t)` on the ACTIVE record's `pose[:,0:2]` / `pose_vel[:,0:2]`. Inside the band, keep the pinned path.
- **Zero through `k_b`, no seam.** `b`, `ḃ`, `b̈` are zero through `k_b` = dispatch + 5 knots (`BUMP_LEAD_KNOTS = WIRE_READ_KNOTS + 2`). Every knot the wire may have read is bit-identical, and the hand arrays are bit-identical.
- **Ramp-in:** a quintic ramp 0 -> Δ ending `BUMP_PRE_HOLD_S = 0.050 s` before the catch. 0.050 s covers the largest timing slip measured (|Δt| ≤ 41 ms). The ramp does run 75 ms into the 0.125 s contact window. That window is the cup's `CUP_CONTACT_ACC` margin, and `validate_cycle` still gates it on the bumped plan. A ball up to 50 ms early meets the full Δ; one 125 ms early finds ~11 % of Δ still to go (≤ 4 mm at 35 mm, T ≈ 0.29 s).
- **Carry and release (retarget):** Δ is held into the carry. A quintic lateral **velocity** ramp to `v_b = -Δ/(T_f + T_a/2 + pre_s)` runs over the carry, `T_a = [t_c, t_rel - pre_s]`, and `v_b` is held constant through the pre/post-release hold, so it adds no acceleration across the detach cone. The ramp's own drift is accounted for, so the release offset is `b(t_rel) = Δ + v_b(T_a/2 + pre_s)` and the landing offset is exactly 0. Measured on a captured record, Δ = (20, 0): `b(t_rel)` = 16.52 mm, `ḃ` = -18.77 mm/s, landing offset 0.000, `b(t_end)` = 0.000.
- **Settle:** after the post-release hold, a rest-to-rest quintic returns offset AND velocity to 0 by the record's last knot. The rest-terminal is therefore the plan's own `rest_site_mm`, which is the 2026-09-28 note's "re-centring belongs in the SETTLE". A later install seeds from the bumped record (`state_at_knot`), whatever it splices into.
- **The 2026-09-28 carry rule is BROKEN DELIBERATELY, within a bound.** The rule (`_catch_terminal`) is that the platform does not translate between catch and release; it came from an UNPLANNED 85-110 mm/s launch caused by a 20 mm return drag. The retarget is a PLANNED translation in that window. Its size: |v_b| ≤ |Δ|max / (T_f + T_a/2 + pre_s), with Δ from the CLAMPED landing (lateral 20, inward 35, so |Δ| ≤ 40.3 mm), giving ≤ 38 mm/s at columns timing (T_f 0.88, T_a 0.17) and ≤ 46 mm/s for any T_a. The drift before release is ≤ 0.2 Δ (≤ 8 mm).
- **Refusals:** the bumped record must pass `validate_cycle` on `[k_b-4, end]` at the live limits. A refusal is a decline, never a clip or a scale-down.
- **Fences that still apply:** NO-CONVERGED-FIT, WITHIN-TOLERANCE (10 mm / 10 ms), CAP-SPENT (2 *accepted* bumps), `_clamp_lateral_to_schedule` (Δ comes from the clamped landing), and PINNED-BUDGET (snapped path only, unchanged).
- **Fences replaced on the bump path:**
  - CONTACT-WINDOW (`t + LEAD_S >= t_land - 0.125`, i.e. `t_c - 0.35`), MIN_WINDOW and `catch_freeze_s` (`t_c - 0.25`) all protect a QP re-solve that has no runway. The bump solves nothing, and its own runway is checked directly, so they become **`BUMP-FREEZE`** (terminal): `t + 0.125 > t_c - 0.050 - 2·dt`, i.e. after `t_c - 0.225`.
  - A gate refusal becomes **`BUMP-WINDOW`** (non-terminal): no cap spent, `_live_catch` kept, retried only once the fitted Δ has shrunk by ≥ `resend_pos_tol_mm`.
  - A node-side `SPLICE_TOO_LATE` on a bump (`k_b <= k_wire` at apply) is non-terminal, by I-CATCH-4's reasoning: a late reply is a latency fact, not a limit fact, the record is unchanged, and BUMP-FREEZE bounds the retries.
- **Latency allowance:** `k_b = k_wire + 2` gives the install 25-50 ms from dispatch (knot quantisation). Expected cost: a service round trip, plus one gate at ~6 ms offline (1152 cases × 2 calls in 14.2 s), plus the lock.

**Owner decisions (both DECIDED 2026-10-16):**
1. DECIDED 2026-10-16: the retargeted lateral launch velocity is ACCEPTED, with the bound raised to **50 mm/s** (`bump.BUMP_MAX_LAUNCH_LATERAL_MM_S`, enforced in `apply_catch_bump` as `BUMP-LAUNCH`, non-terminal) on the **total lateral launch velocity <= 50 mm/s, i.e. the plan's own <= ~25 mm/s plus the bias** (coordinator's reading, 2026-10-16). The design's ≤ 38 mm/s typical / 46 mm/s bound sits inside it. The executor's ≤ 25 mm/s figures (`_catch_terminal`, `_release_pos_mm`) still describe the UNBUMPED carried throw and stay as they are; `v_b` is a separate planned bias that exists only on a bumped catch.
2. DECIDED 2026-10-16: **columns first.** The bump is enabled for `bump.BUMP_ENABLED_PATTERNS` = `{'columns', 'columns_1ball_fed'}` only; `SkillExecutor(catch_bump_enabled=False)` is the default. Hop and reload (sep 250) stay off. **Unpark condition:** a matching-record capture for that pattern, replayed through `apply_catch_bump` with `bump_replay.py`.

## 3. Interface (landed in G1, 2026-10-16: pure layer)
- `motion/skills/bump.py::apply_catch_bump(plan, meta, k_wire, delta_xy_mm, t_catch_s, limits, geom, *, pre_hold_s, max_launch_lateral_mm_s, post_hold_knots, t_ramp_start_s) -> BumpResult`. This is the ONE enforcement point. `BumpResult` carries a full-length bumped `CyclePlan` + meta (catch site +Δ, carried release site +`b(t_rel)`, cup velocity +`v_b`) or a decline: `BUMP-FREEZE` / `BUMP-NO-CARRY` (terminal), `BUMP-LAUNCH`, the gate's own code (non-terminal). `k_b` is knot-aligned: `k_b = k_wire + 2`, ramp starting at `k_b·dt`.
- `bump.bump_record(plan, meta, t0_s, t_dispatch_s, delta, t_catch_abs_s, limits, geom, t_apply_s=)`: the wall-clock wrapper. `k_wire` comes from the dispatch instant; `t_apply_s` is the node's own check (`k_b <= k_wire_at(t_apply)` -> non-terminal `SPLICE_TOO_LATE`).
- Executor: `SkillExecutor.catch_bump(plan, meta, t0_s, t_abs_s, limits, geom, landing=None) -> CatchBumpDecision` is pure and applies the § 2 fences. `commit_catch_bump(decision, t_abs_s)` records the outcome. On accept that is: one re-aim spent; `_carried_release_mm[idx]` = old + `b(t_rel)`, so the learner's `x[2:4]` carries it; the live terminal's landing and release moved. A `BUMP-WINDOW` remembers |Δ|. A terminal decline ends re-aiming.
- **G2 (not done):** wire it into `_resend_live_catch` after the snap band; add the `trajectory_node` `bump_catch` request on the install service under the stream lock (`BUMP_HANDOFF` while an I-INSTALL-1 `StreamHandoff` is pending); interface rebuild; `skill_node` passes `catch_bump_enabled=bump.bump_enabled_for(pattern)`.
- **I-CATCH-5** is in INVARIANTS § 7, pinned by `tests/motion/test_skills_bump.py`.

## 4. Offline numbers (2026-10-16, `bump_replay.py --retarget`, sim columns apex 0.95 / sep 125, 4 catches × 4 directions; commands in the entry)
| Variant (lead / pre-hold) | columns 72 refused: diag / axis | hop 8 (proxy) | 1ball_fed 7 | 53 installed (sanity) |
|---|---|---|---|---|
| live raw re-solve | 0 % | 0 % | 0 % | 100 % (selection) |
| bump 5 kn / 0 | 1005 (87.2 %) / 1084 (94.1 %) of 1152 | - | 112/112 | - |
| **bump 5 kn / 0.050 s (design)** | **1001 (86.9 %) / 1091 (94.7 %)** | **128/128** | **112/112** | **848/848** |
| bump 5 kn / 0.125 s | 558 (48.4 %) / 761 (66.1 %) | 121/128 | 92/112 | - |
| bump 9 kn / 0.125 s | 60 (5.2 %) | - | - | - |
Every refusal is on the ramp-in; the carry, retarget and settle add none. The rejected shapes at 5 kn / 0: hold without retarget 1005 (87.2 %); return-before-release 917 (79.6 %, 84 LIMIT_JERK on the return).

## 5. Rehearsal row (for § 1b, when implemented)
`grep -E 'BUMP |BUMP-WINDOW|BUMP-FREEZE|RESEND-REFUSED' <launch.log>`:
- every post-band re-aim is a `BUMP` line, with no `RESEND-REFUSED LIMIT_*`;
- the bag shows a 0.0000 rev leg step at `k_b`;
- the throw after a bumped catch lands inside the un-bumped throws' scatter.

## 6. Done means
- **G1 (DONE 2026-10-16):** I-CATCH-5, `apply_catch_bump`, the executor's pure layer and the § 3 tests. The probe calls the production function and reproduces the design row exactly with `--legacy-start --no-launch-bound`. Scoped gate green: `tests/motion/test_skills_bump.py tests/motion/test_skills_executor.py tests/motion/test_cup_cycle.py tests/motion/test_cup_contact_contract.py`.
- **G2 (CODE DONE 2026-10-16, not committed, not flown):** the node request and the executor dispatch path have landed. The node request is `InstallSegment.srv` `KIND_BUMP` (one service, sharing the install path's guards, hold epoch and callback group) → `trajectory_node._svc_bump_catch`. On the executor side, `_resend_live_catch` routes post-band carried-catch re-aims to `SkillExecutor.bumper`, ahead of the `catch_freeze_s` return. A catch with no carried throw keeps the raw re-solve. `skill_node` turns the bump on per goal pattern and the sim gate wires in a bumper. **The interfaces changed: `colcon build --packages-select jugglebot_interfaces jugglebot` + a FULL launch restart before flying** (rehearsal row 31). Scoped gate: . `./run_tests.sh` and `--full` are still owed (coordinator). Details: design entry § "Implementation G2".
- **Flown:** one sitting shows ≥ 70 % of post-band re-aims installed, zero bump-caused LIMIT/guard events, and no throw-landing regression.
