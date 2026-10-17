---
title: "Launch/reload THROW fresh-origin lead 9 -> 13 knots (150 -> 250 ms of solve budget, 62 ms over the 188 ms refused solve), with one per-kind lead (schedule.install_lead_s) read by the schedule, install_segment and trajectory_node so the origin, window and release do not move"
type: feature
date: 2026-10-16
status: resolved
phase: "throw-fresh-origin-lead - implemented and first flown 2026-10-16 16:10"
related_plan: throw-fresh-origin-lead.md
related:
  - 2026-10-16-handoff-lead-flown-zero-splice-too-late-tracker-fit-late.md
  - 2026-10-16-handoff-lead-8-knots-and-snap-aware-resend-fences.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/stream_handoff.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/motion/test_skills_schedule.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_skills_install_origin.py
  - tests/motion/test_solver_worker.py
  - tools/probes/throw_fresh_lead_replay.py
  - tools/probes/README.md
  - tests/ros/test_install_segment.py
  - tests/ros/test_skill_node.py
  - tests/hardware/session_skill_stack_rehearsals.md
  - plans/active/throw-fresh-origin-lead.md
  - plans/active/INDEX.md
  - plans/active/handoff-solve-chaining.md
  - logbook/2026-10-16-throw-fresh-origin-lead.md
  - logbook/INDEX.md
subsystem:
  - skill-stack
  - motion
---

# THROW fresh-origin lead: 9 -> 13 knots

## What happened

On 2026-10-16 at 10:50 (`~/.ros/log/2026-10-16-10-50-42-798495-jetson-1033523/launch.log:425`), the first fed-columns attempt ended `ORIGIN_TOO_LATE` at THROW 0. The solve took 188 ms against the 150 ms that the 9-knot `LEAD_S` leaves after the 3-knot wire read. A parallel session was loading the box at the time.

## What changed

**New constants.** `schedule.FRESH_THROW_LEAD_EXTRA_KNOTS = 4`, giving `FRESH_THROW_LEAD_KNOTS = 13`. That is 0.325 s of lead and 250 ms of solve budget.

**One function at both ends.** `schedule.install_lead_s(kind)` is the single place the lead is chosen. It returns the THROW lead for a THROW and `LEAD_S` for every other kind. It is read in four places:
- `_assign_leads`, for a skill that does not follow a release (a pinned THROW keeps `HANDOFF_LEAD_S`).
- The `Skill.lead_s` default.
- The `install_segment` default (`lead_s=None`), so the sim gate gets the new lead.
- `trajectory_node._svc_install_segment`, which uses it for the fresh predicate, the deadline, `splice_knot_for` and the worker request.

**Columns-feed gate.** The gate in `skill_node` now uses `FRESH_THROW_LEAD_S`, because THROW 0 is the schedule's first dispatch.

**No interfaces change.** The service request already carries `kind`.

**Why one number.** The schedule's dispatch lead and the executor's reserved origin must agree. Any difference becomes extra window: the release does not move, but the origin does. That is the 2026-10-02 class that `reserve_fresh_lead` closed.

## Discussion: why the emitter fix, not 12 knots (I-INSTALL-1)

The first cut of this change left `_install` as it was: it swapped the wire onto the new plan at install, and the emitter played the THROW's knot 0 (the running REST's TERMINAL rest) from install until `t0`. When THROW 0 follows a REST that is still streaming (`compile_one_ball`, `compile_rest_columns`, `compile_reload*`), that cut the REST short and STEPPED the wire onto its end state. Measured on the opening lift (below), the step was 67 / 85 / 92 % of the 0.06 rev install-continuity guard at 9 / 12 / 13 knots.

The step was not new and was not caused by the lead. It existed at 9 knots, and the lead only set how much tail it cut off. Two ways out:
- **Fall back to 12 or 9 knots.** That keeps a step the guard was never meant to admit as normal. It also ties the solve budget to an emitter artefact, and leaves DECAY RESTs unmeasured.
- **Play the running record on to `t0`.** This removes the step at every lead, so the lead becomes a free parameter at the wire.

Owner, 2026-10-16: fix it, keep 13 knots. The design is one invariant (I-INSTALL-1, `motion/skills/INVARIANTS.md`), one enforcement point and the tests named in its row.

**Two views of "the live record".**
- **The planning head.** `_active_plan` / `_cycle` hold the new record FROM INSTALL. These readers need it: the next skill's splice (`splice_knot_for`, `_snap_to_release`, the `fresh` test), RESEND re-aims, `_refusal_timing`, `/trajectory/status`. The first columns CATCH dispatches 0.300 s after THROW 0, which is before THROW 0's `t0`, and it must splice into the THROW record.
- **The stream view.** `_stream_plan_locked()` is what the wire plays. It returns a `StreamHandoff`: the running record until `t0`, then the head, chosen PER SAMPLE so a frame's u1/u2 lookahead straddles the switch correctly. Its readers are `_emit_once`, `_current_state` and `_commanded_hand_state`. Those are the readers that mean "what is the machine commanded to": stops, holds, guard descents, live seeds and the continuity guard.
- **What arms and retires it.** Only `_svc_install_segment` arms a handoff (`_install(stream_handoff=True)`), and only while `t0` is still ahead. That covers fresh THROW/CATCH, and splices into a head whose origin is still pending, which re-wrap the pending handoff. A fresh REST is exempt: its origin is now, and it is seeded from the stream view.
- **Stops win.** Every other install retires the handoff. The handoff is bound to its head by identity, so the emitter's direct backstop-freeze assignment retires it too. A graceful stop between install and `t0` is seeded from the moving REST tail and wins from there.

**The continuity guard.** It now asks its question at `t0`: does the running record ARRIVE on knot 0? A rest-terminal record that ends by `t0` does, with zero step by construction. A stop or hold landing during the solve does not, and still refuses `STALE_STATE` (tested). `t0 <` the running record's end is unreachable from the compilers, because `fresh` implies `t_now + lead >= end` and the grid snap only moves the origin later. It is refused rather than truncated.

**Ruled out: a composite plan installed as `_active_plan`.** Every identity check (`cycle[0] is active`, the hold epoch, peak tracking) would have had to learn it.

## Control walk at 40 Hz (13 knots, zero solve, emit lead `JB_TRAJ_EMIT_LEAD_KNOTS` taken as 0; the logic is unchanged by L)

The REST ends at `E`. THROW 0 dispatches at `t2 = E - 0.325 + 1 ms` and is snapped up to the grid, so `t0 = snap(t2) + 0.325 >= E`. Each tick `t_k` ships `u0, u1, u2` sampled at `T = t_k`, `T + dt` and `T + 2dt`.

1. **Install (`t_k ≈ t2`).**
   - Before: from this tick, `u0` = THROW knot 0 = `REST(E)`. That is a 0.055 rev leg step, and hand 0.037 rev, from `REST(t2)`.
   - After: `u0 = REST(T - rest_t0)`, the tail the REST was already playing. The emitter's per-knot leg change stays at most 0.0075 rev, which is the REST's own motion.
   - `_reply_outran_wire`'s `k_wire = floor((now - t0)/dt) + 3` is negative, so there is no refusal. `_refusal_timing` and `/trajectory/status` read the THROW.
2. **`E <= T < t0`.** The REST's terminal hold (`CyclePlan.state_at` clamps past the end). That equals knot 0, so there is no step.
3. **`T = t0 - dt`.**
   - `u0 = REST(t0 - dt)`.
   - `u1 = REST(t0)`, the terminal rest, which is knot 0.
   - `u2 = THROW(dt)`, which is knot 1. The lookahead names what the next frame actually plays.
4. **`T = t0`.** `u0` = THROW knot 0. Measured switch step: **0.0 rev** on the legs and on the hand, at 9, 12 and 13 knots.
5. **`T > t0`.** The THROW's knots 1, 2, and so on. The handoff view is dropped once `tau > 2dt`.

A first CATCH dispatched at `t0 - 0.025` splices into the THROW (same `t0`, `splice_k > 0`), and the wire still plays the REST tail (tested).

**Before/after table.** `compile_reload_wait` opening lift, dispatch at `E - lead + 1 ms`, zero solve. The handler, then the REAL emitter, are ticked on the grid by `test_a_launch_throw_at_the_fresh_throw_lead_takes_over_the_opening_lift_tail`:

| Dispatch lead | Leg step before (at install) | Hand step before | Leg step after (at the switch) |
|---|---|---|---|
| 9 knots | ~0.040 rev (67 %) | 0.018 rev | 0 |
| 12 knots | ~0.051 rev (85 %) | 0.031 rev | 0 |
| 13 knots | ~0.055 rev (92 %) | 0.037 rev | 0 |

"Before" varies by about 0.001 rev with the clock's phase against the knot grid. It was 0.0393 to 0.0403 at 9 knots across runs. In the 9- and 12-knot rows the node still reserves 13 knots, so `t0` lands 0.1 s and 0.025 s after `E`, and the wire holds the terminal rest in between (step 2).

## Numbers

- **Replay.** `tools/probes/throw_fresh_lead_replay.py` (recipe in its docstring) read the executor's `solve` (`t_inst - t_now`) from five launch logs dated 2026-10-13 to 2026-10-16, 62 fresh THROWs in total. The four earlier sessions ran 21-86 ms. On 2026-10-16, accepted solves ran 22-140 ms, plus the one refusal at 188 ms.

  | Lead | Solve budget | Refused | Worst margin |
  |---|---|---|---|
  | 9 knots | 150 ms | 1/62 | -38 ms |
  | 12 knots | 225 ms | 0/62 | +37 ms |
  | 13 knots | 250 ms | 0/62 | +62 ms |

- **Why 13 and not 12.** The pre-registered rule (solve budget >= 188 + 25 ms) gives 12 knots. 13 knots buys solve margin at the price of a slightly larger REST-tail step:
  - Solve margin over the worst recorded solve: +62 ms at 13 knots against +37 ms at 12.
  - REST-tail step on the opening lift: 92 % of the guard at 13 knots against 85 % at 12.
  - The THROW has no re-aim window and its seam does not move, so the tail step is the only cost.

  The tail-step cost is gone since I-INSTALL-1 (Discussion above), so 13 costs nothing at the wire.
- **Ball Butler `delay_s` stays on `LEAD_S`.** `skill_node.py` ~2796 deliberately still adds `LEAD_S`. That margin is between the bridge REST's end and Ball Butler's *release*. The 2.5 s floor dominates it, and the 2026-10-16 fed attempt had about 2.2 s of slack before THROW 0's dispatch.
- **Dispatch-gap law.** The binding gap is THROW 0 to the first handoff CATCH. It moves from `0.400 - 0.025·EXTRA` to `0.500 - 0.025·EXTRA`, which is 0.300 s at EXTRA 8. The test was renamed to `..._500_ms_...`.

## Verification

- **Scoped tests, 2026-10-16.**
  - `pytest tests/motion/test_skills_schedule.py tests/motion/test_skills_executor.py tests/ros/test_install_segment.py tests/ros/test_skills_plan_bench.py tests/motion/test_skills_install_origin.py tests/motion/test_solver_worker.py tests/ros/test_skill_node.py -q -p no:cacheprovider -n 4`: 842 passed, 2 failed, in 472 s.
  - The two failures were load-sensitive: the worker bitwise round-trip, and `test_a_slow_test_run_cannot_age_the_fixture_sample_past_the_accept_check`. Both passed when re-run with `-k`.
  - The worker test's reference now passes the request's own `lead_s`.
  - `pytest tests/sim/test_skills_gate.py tests/motion/test_solver_worker.py -q -p no:cacheprovider -n 4`: 45 passed in 530 s.
- **I-INSTALL-1 scoped tests, 2026-10-16.** `pytest tests/ros/test_trajectory_node.py tests/ros/test_trajectory_tilt_map.py tests/ros/test_levelling_frame.py tests/ros/test_unified_cycle_levelling.py tests/motion/test_skills_executor.py tests/motion/test_skills_install_origin.py tests/ros/test_skills_plan_bench.py -q -p no:cacheprovider -n 4`: 561 passed in 391 s. `pytest tests/ros/test_install_segment.py tests/sim/test_logbook_front_matter.py tests/sim/test_plans_index.py -q -p no:cacheprovider -n 3`: 147 passed, 2 skipped, 1 failed. The failure is `test_a_fresh_throw_installs_through_the_real_worker_process` ("solver did not answer by the wire deadline", four tries). It is load-flaky both serially and under `-n` (the auditor saw it fail 2 of 3 serial runs at load 7-12, same "solver did not answer by the wire deadline") and fails identically on the unmodified HEAD (bde285ae) under load, so it is not a regression of this change. Reachability note (audit): `_live_cup_accel_mm_s2` reads the head view (tau < 0 while a handoff is pending), but no compiler dispatches a moving-seeded fresh REST between THROW 0's install and its t0 (a CATCH splices; the closing REST waits two rest tails past touch-down), so it is documented, not guarded.
- **Full gate, 2026-10-16.** `./run_tests.sh --full` on the branch rebased onto skill-stack 763d5b0c: 6666 passed, 11 skipped in 319.42 s; serial tail 7 passed. Landed as 205c527c.
- **Hardware, first flown 2026-10-16 16:10** (launch `~/.ros/log/2026-10-16-16-10-37-689586-jetson-1220502`, bag `~/Desktop/rosbags/2026-10-16_16-10-37`; install built 15:55 carries `class StreamHandoff`). One fresh THROW at the new lead (the `columns_1ball_fed` reload, skill 0): `fresh origin: THROW over 0.700 s from rest, origin +0.325 s after dispatch (solve 0.028 s of a 0.250 s budget)`; zero `ORIGIN_TOO_LATE`, zero `STALE_STATE` in the log. Bag `/leg_setpoint_echo` (25 ms samples) around that THROW: max per-sample leg step 0.0060 rev dispatch -> install (the REST tail), **0.0000 rev install -> t0 and 0.0000 rev across t0 +-30 ms**, 0.0293 rev once the THROW plays (its own motion). Row 30 PASS on a sample of one; the hop pattern's two THROWs were splices, not fresh origins.
- **Second sitting, 2026-10-16 16:23** (launch `~/.ros/log/2026-10-16-16-23-01-479044-jetson-1230939`, bag `~/Desktop/rosbags/2026-10-16_16-23-01`, ten `columns_1ball_fed` attempts): nine fresh THROWs at `origin +0.325 s`, solves 0.062 / 0.062 / 0.068 / 0.081 / 0.083 / 0.092 / 0.093 / 0.093 / 0.167 s of 0.250 s, 9 of 9 accepted (the 0.167 s one would have been refused at the old 150 ms budget); `grep -c "ORIGIN_TOO_LATE\|STALE_STATE"` = 0. Bag `/leg_setpoint_echo` around each of the nine: max per-sample leg step 0.0000 rev in dispatch -> install, install -> t0 and t0 +-30 ms, 0.015-0.017 rev once the THROW plays. Row 30 is now n = 10 across two sittings and the plan is archived. Not this plan's: the sitting's ends were ABORTED_NO_RELEASE x4 (throw 2/4 never left the hand), MISSED_CATCH, LIMIT_ACC, ABORTED_MODE_CHANGED (counted from the skill_node ENDED lines, one per attempt; END lines repeat within an attempt); one non-terminal RESEND-REFUSED SPLICE_TOO_LATE on a 0.226 s re-aim solve.
