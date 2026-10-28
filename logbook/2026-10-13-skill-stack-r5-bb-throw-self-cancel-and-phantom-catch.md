---
title: R5 first flight after wave 3 - every Ball Butler throw was cancelled by skill_node at dispatch (the announcement's swap was read as an end) and JB then caught a phantom; BB calibration refused BB_IN_BASE_MOVED
type: investigation
date: 2026-10-13
status: resolved
phase: "two-ball-skill-stack — R5 (two-ball columns)"
related_plan: two-ball-skill-stack.md
related:
  - 2026-10-13-skill-stack-r5-wave-2-after-sitting-7.md
  - 2026-10-13-skill-stack-r5-wave-3-expected-landing-box-and-solver-worker.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/ros/test_skill_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - tests/hardware/session_skills_r5_sitting8.md
  - logbook/2026-10-13-skill-stack-r5-wave-2-after-sitting-7.md
  - logbook/2026-10-13-skill-stack-r5-bb-throw-self-cancel-and-phantom-catch.md
  - logbook/INDEX.md
subsystem:
  - skill-stack
  - ball-butler
tags:
  - ball-butler-cancel
  - hardware-sitting
  - testing
---

# R5: the wave-2 BB cancel cancelled every throw it was written to protect

## Summary

Wave 2's Ball Butler throw cancel (commit `ed384cb4`) was never flown before tonight. On its first flight it cancelled every BB throw about 20 ms after dispatch (14 ms after the announcement), because `_fire_reload_throw` read "the ctx is no longer current" as "the attempt ended" when the announcement's own executor swap is the normal success ordering. A second defect made it worse: the resulting `THROW_ABORTED_BY_HOST` verdict was ignored unconditionally, so the platform ran REST -> CATCH -> REST for a ball that never left BB. Both are fixed; sitting 8 verifies live (status stays `open`).

## What happened

Launch 22:29 (`~/.ros/log/2026-10-13-22-29-08-811681-jetson-3875811/launch.log`, bag `~/Desktop/rosbags/2026-10-13_22-29-08`). Four Juggle goals with BB holding a ball (three `reload`, one `self_toss reload=True`). Every one:

```
1791891165.954 ball_butler_node  Throw at jugglebot: in 3.0 s · flight 0.88 s · 3.30 m/s · aim correction (-32, -5) mm
1791891165.960 skill_node        reload: ThrowAnnouncement received -- compiled the reload schedule (3 skills) and swapped the executor ...
1791891165.963 skill_node        reload: Ball Butler's ball is in the air (hold tilt 10.6 deg, cap 12.0 deg) — catching it, then 0 throws
1791891165.968 ball_butler_node  Throw cancel sent to the bridge (requested)
1791891165.975 skill_node        END ENDED_DURING_BB_DISPATCH: Ball Butler throw cancel (release was 3.00 s away): cancel sent (bb/cancel); ...
1791891165.991 teensy_bridge     bb/cancel: CANCELLED: BB dropped the throw 2940 ms before its release (ball retained, BB to IDLE)
1791891165.998 ball_butler_node  ERROR Throw at jugglebot NOT thrown: THROW_ABORTED_BY_HOST (axis=n/a, detail1=2940)
1791891167.404 trajectory_node   install_segment REST Ball 1 ...          <- the announced schedule runs on
1791891169.177 trajectory_node   install_segment CATCH Ball 1 ...         <- JB "catches" a ball that never left BB
1791891169.992 skill_node        Goal ... finished with state 4 (SUCCEEDED)
```

| Attempt | Pattern | Outcome |
|---|---|---|
| 1, 2 | `reload` | cancelled at dispatch, phantom REST -> CATCH -> REST, goal SUCCEEDED (state 4) |
| 3 | `reload` | cancel DEFERRED ("throw goal not yet accepted by the bridge") -> `CANCELLED_BEFORE_DISPATCH`, same phantom catch |
| 4 | `self_toss reload=True` | phantom catch, then `END REJECTED_NO_BALL at skill 3 (THROW)` |

Same session, BB calibration: `BB_IN_BASE_MOVED` six times in the 22:00 launch (delta-kappa +0.212..+0.266 deg) and six in 22:29 (+0.216..+0.338 deg); limit +-0.093 deg, pooled n 34. The owner set `bb_moved:=true`; two sweeps were accepted 22:31:51 / 22:32:07 local (kappa 179.682 / 179.569 deg, delta-yaw -0.131 deg, within +-0.2 deg), pool reseeded n=2. Each sweep's finalisation stalls mocap_node 10-13 s (22:29 log: "1870 of 3437 expected QTM frames lost (54.4 %) ... frame queue full: starved for over 0.5 s"); in the 22:00 launch a `self_toss` was refused `REJECTED_FRAME_OFFSET (only 10 mocap Platform samples in the 1.0 s window)` because it landed 0.8 s into one of those stalls.

## Discussion

1. **A swap is not an end.** ball_butler_node publishes the ThrowAnnouncement synchronously inside `bb/throw_at_target`; `_on_announcement` -> `_install_announced_reload` claims the ctx, compiles, installs the executor and clears `_reload_ctx` (sets `_reload_throw_inflight`) BEFORE the service reply returns to `_fire_reload_throw`. `_still_current(ctx)` is therefore False on EVERY successful dispatch. Wave 2 put `_maybe_cancel_bb_throw('ENDED_DURING_BB_DISPATCH')` on that branch, reading "ctx retired" as "attempt ended"; before wave 2 the branch was a silent return, which is why sitting 7's feeds (2026-10-11) worked. Columns' ball-B feed is the same call (`_fire_reload_throw` is the only `bb/throw_at_target` caller, one throw per attempt), so two-ball columns would have died identically. The discriminator is `_swapped_in_and_live(ctx)`: the installed executor is still the node's executor, the throw is inflight and no END decided. `_ended_code_for_swap()` alone is not enough: it answers "did an END decide this goal" but is also False in the true-overtaken case where a Stop retired the bridge before any announcement, which must still cancel. It reads under `_reload_lock` only because `_fire_reload_throw` runs from `_on_tick` before the tick lock is taken; taking `_tick_lock` there would invert the lock order.
2. **Why the gate did not catch it.** The wave-2 test `test_bb_cancel_our_own_throw_aborted_by_host_does_not_end_an_attempt` pinned the bug: it delivered BY_HOST to a LIVE attempt and asserted it was ignored. `test_c2_announcement_delivered_while_the_throw_call_is_in_flight` modelled the exact race but never looked at the cancel client. Mocked-ROS tests are blind to choreography unless the assertion names the side effect (feedback_testing_discipline). The new tests were written FIRST and 4 of 13 failed on HEAD.
3. **The phantom-catch contract.** A NOT-thrown verdict for the throw an attempt waits on ends the attempt; `THROW_ABORTED_BY_HOST` is ignored only as the echo of an end that already happened (`_bb_throw_awaited_live`). Without it, the sitting-4 `MISSED_CATCH` logic ends the pattern only after the feed catch has been dispatched and its evidence window elapsed.
4. **The calibration refusals were a real yaw change of about +0.28 deg** (22:29 deltas +0.216..+0.338) of BB relative to its own shelf markers (kappa = BB yaw minus shelf heading; a Motive global re-transform cancels out of it; the shelf frame itself moved 6.0 mm / +0.047 deg, which the base-frame gate accepted). This is the first calibration since the FW 9 flash + power cycle; the FW 9 diff (BallButler commit 5e6eee8) touches no yaw/homing/encoder code, so homing repeatability after the power cycle or a physical nudge remain the candidates, unresolved. `bb_moved:=true` was the right call. The aim correction (`resources/throw_affine_correction.json`, refit 2026-10-09) predates the shift: at the `reload` throw's ~0.91 m horizontal range (3.30 m/s, pitch 71.7 deg, ToF 0.882 s) 0.28 deg is ~4.5 mm lateral, so watch the first fed landing.
5. **The mocap stall is the defect class wave 3 removed from trajectory_node** (a long solve inside a single-threaded spin). A worker-process move for `mocap_node._finalize_calibration` is the recommended follow-up, not built tonight. The orchestrator's first framing of the 22:00 refusal ("the mocap stream is thin after the frame change") was WITHDRAWN: outside the stalls the stream ran at its nominal rate (mocap_node's `qtm clock sync` DEBUG lines report 296-301 frames/s between sweeps in the 22:29 log; the 22:00 bag analysis measured 305 Hz).

## Fix

In `skill_node.py` (function names only): `ctx.installed_executor` is set at the three swaps (the `columns_1ball` branch, the plain `reload` branch, `_install_columns_schedule`) inside their existing locked install section; new `_swapped_in_and_live(ctx)`; the overtaken branch of `_fire_reload_throw` logs one DEBUG line ("reload: bb/throw_at_target answered after the announcement swapped the schedule in -- the attempt is live, the throw stays pending") and returns when the swap is live, else cancels with `ENDED_DURING_BB_DISPATCH` as before (the pending release is still recorded BEFORE any end path, so exactly one cancel results in each of: normal, Stop during the round trip, END winning the compile, END between swap and reply). `_on_bb_throw_outcome` now ends a live attempt on `THROW_ABORTED_BY_HOST` as `REJECTED_BB(THROW_ABORTED_BY_HOST)` via `_bb_throw_awaited_live`, and ignores it only as the echo of an end already taken; every acted-on verdict drops the pending release record. The audit's one behaviour finding was applied: `_bb_throw_awaited_live`'s post-swap arm is `_reload_throw_inflight and _bb_throw_dispatched`, a node-level flag set in `_fire_reload_throw` under `_reload_lock` just before the service call and cleared in `_end_attempt`, at both ctx-creation sites and at the announced executor's retirement (only when `_reload_ctx` is already None, so the bridge REST's retirement mid-wait cannot wipe it) — without it a stray BY_HOST verdict would have ended a TRACKER-fed columns attempt that never asked BB to throw (`_install_columns_schedule` sets `_reload_throw_inflight` on that path too). Class sweep: `_check_reload_timeout`, `_check_reload_lane_refused` and the `reloading` gate do not conflate swap with end; the NOT_SETTLED retry gate does not either but is effectively unreachable (F1 below).

## Verification

| Date | Command | Result |
|---|---|---|
| 2026-10-13 | `pytest -q -p no:cacheprovider tests/ros/test_skill_node.py -k "bb_cancel or aborted_by_host"` | before fix 4 failed / 9 passed; after fix **13 passed** |
| 2026-10-13 | `pytest -q -p no:cacheprovider tests/ros/test_skill_node.py tests/ros/test_skill_node_magazine.py tests/ros/test_skill_node_reload_only.py tests/ros/test_skill_node_resend_param.py tests/ros/test_skills_plan_bench.py` | **345 passed in 13.93 s** |
| 2026-10-13 | `pytest -q -p no:cacheprovider tests/motion/test_skills_report.py tests/motion/test_lane_latch_rehearsal.py tests/motion/test_feed_lateral_miss_probe.py tests/ros/test_orchestrator_conduit_contract.py tests/ros/test_launch_console.py tests/ros/test_ball_butler_node.py tests/ros/test_two_ball_association.py tests/ros/test_gui_juggle_defaults.py tests/ros/test_install_segment.py` | **206 passed** |
| 2026-10-13 (after the audit fixes: the `_bb_throw_dispatched` flag + three negative-case BY_HOST tests, the tracker-fed one failing before the flag: `-k aborted_by_host` 1 failed / 5 passed) | the same five-file command | **348 passed in 14.80 s** |

The first three rows are the unit's first handoff (before the audit fixes); the counts in Discussion 2 are from that state.
Full gate: see the commit message's triple. Live verification is runbook row 16b (`tests/hardware/session_skills_r5_sitting8.md`).

## Open

- F1: the NOT_SETTLED retry (`_on_bb_throw_outcome`'s `retry_ctx` gate) is effectively dead in the field, since the announcement swap always retires the ctx before the firmware's verdict; it fails closed as `REJECTED_BB(THROW_ABORTED_NOT_SETTLED)`. A real retry is design work.
- F2: `bb/throw_outcome` carries no throw identity; a late verdict from attempt N could be attributed to N+1 (timing makes it implausible; the fix is in the ball_butler_node / bridge relay).
- F3: ball_butler_node's `_cancel_on_accept` is a node-wide flag that fires on the next goal acceptance; correct for attempt 3, worth a look in the ball_butler unit.
- F4: `_end_attempt` with an already-ended executor and empty `_goal_end_code` writes the code and the executor's retirement overwrites it; benign, but "first end wins" is not literally true in that window.
- Mocap finalisation worker (see Discussion 5).
- BB yaw-homing repeatability: owner check is re-home / power-cycle BB, one sweep; delta-kappa ~0 means homing, still +0.28 deg means physical.
- Aim-correction refit if the first fed landing is more than 10 mm off (`tools/probes/feed_lateral_miss.py`).
- Sitting 8 verifies the BB dispatch path live (runbook row 16b).

**Follow-up (2026-10-13 night).** Sitting 8 met the R5 gate (`2026-10-13-skill-stack-r5-sitting-8-gate-met.md`) and verified the BB dispatch path live: 14/14 throws dispatched, 0 `ENDED_DURING_BB_DISPATCH`, 0 `THROW_ABORTED_BY_HOST`. F1-F4, the mocap finalisation worker, the BB yaw-homing check and the aim-correction refit trigger stay carried; the remaining items are rows in `plans/active/skill-stack-dress-rehearsals-and-residue.md` § 2.

> Closed 2026-10-28: F1 JB-25, F2 JB-21, finalise worker JB-30; F3/F4 in residue plan section 2 (JB-34 survey).
