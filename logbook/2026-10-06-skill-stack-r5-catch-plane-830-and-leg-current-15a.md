---
title: "Catch plane back to 830 mm, leg current limit 10 -> 15 A (owner decisions, 2026-10-06 evening)"
type: investigation
date: 2026-10-06
status: in-progress
phase: "two-ball-skill-stack — R5"
related_plan: two-ball-skill-stack.md
related_entries:
  - 2026-10-06-skill-stack-r5-sitting-6-catch-high-seat-verdict.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/sites.py
  - config/hardware_config.yaml
  - config/generated/hardware_config.h
  - config/generated/hardware_config.py
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/report.py
  - plans/active/two-ball-skill-stack.md
  - tests/motion/test_skills_sites.py
  - tests/motion/test_skills_report.py
  - tests/motion/test_skills_admissible.py
  - tests/motion/test_skills_executor.py
  - tests/ros/test_skills_plan_bench.py
  - tests/ros/test_install_segment.py
  - tests/hardware/session_skills_r5_sitting6.md
  - tools/probes/feed_lateral_miss.py
  - tests/hardware/free_platform_test.py
  - tests/hardware/supported_platform_test.py
  - tests/hardware/single_leg_test.py
---

# Catch plane back to 830 mm, leg current limit 10 -> 15 A

## What changed and why

Two independent owner decisions, 2026-10-06 evening, both final:

1. **`sites.CATCH_CUP_Z_MM` 930 -> 830 mm** (release stays 860). CATCH HIGH (2026-10-05) raised the
   plane to 930 to buy more with-ball stroke, but the hand brakes to the top of its stroke
   (~998 mm) after every throw regardless of plane — the plane only sets the empty drop before
   contact and, through the C-CUP-2 contact floor, the contact speed: 830 = 169 mm drop, cup
   -1.61 m/s, ~2.7 m/s relative impact, hold 0.08-0.11 s; 930 = 68 mm drop, -0.96 m/s, ~3.2 m/s
   relative impact, hold 0.16-0.19 s. 930 produced a slow with-ball carry-down read as a lazy
   hand, a cup-sensor seat landing around the re-release, and dropped the columns apex ceiling to
   0.950 m (was 1.047 at 830). Returning to 830 restores the original contact speed; the hold at
   the top is kept by the stroke itself, not by the plane. No learner memory migration needed —
   rows are release-relative since `schedule.apex_from_crossing`.
2. **`jugglebot_odrive_defaults.leg_curr_limit_a` 10.0 -> 15.0 A**. Legs 1/2 touch the 10 A clamp
   ~1 % of transit time (peak 10.3 A, p95 6-7 A); when it binds the position loop has no authority.
   Thermal headroom is large (FET <= 35 C over two 7-17 min sessions, bus minimum 44.9 V, time at
   >= 9.5 A under 0.3 % of a session). To watch at the next sitting: peak current, FET temp,
   tracking error.

**Firmware answer (file:line):** the can-bridge's compiled-in `ODriveDefaults::LEG_CURR_LIMIT_A`
(`Teensy_code_canbridge/hardware_config.h:140`) is used **only** as the cold-start-restore
fallback inside `axis_shipped_curr_limit()` (`axis_state.h:112-116`), returned when
`axes[a].curr_limit_A` is still its 0.0f initial value (i.e. no `SET_VEL_CURR_LIMITS` RPC has
landed yet this boot). The **live, operative** limit follows the Python constant
`hw.ODRIVE_LEG_CURR_LIMIT_A` (`jugglebot/hardware_config.py:125`, regenerated from
`leg_curr_limit_a`): `teensy_bridge_node._run_configure` (`teensy_bridge_node.py:6684-6685`) calls
`teensy_set_vel_curr_limits(axis, hw.ODRIVE_LEG_VEL_LIMIT_RPS, hw.ODRIVE_LEG_CURR_LIMIT_A)`
(`teensy_bridge_node.py:5922-5924`), which sends `RpcMethod.SET_VEL_CURR_LIMITS`; the firmware's
handler (`rpc.cpp:283-294`) sends a **live CAN frame** (`ODrive::encode_set_vel_curr_limits`)
that writes the ODrive's own `current_soft_max` register directly, and on success caches the
value into `axes[a].curr_limit_A` for the next cold-start restore. So **the live 15 A limit takes
effect the next time `_run_configure` runs (post-homing/post-activate) — no can-bridge reflash,
no ODrive reflash.** `can/odrive.py:58`'s `DEFAULT_VEL_CURR['leg_curr']` is another live consumer,
already correct. The one consumer NOT wired through this path is the static ODrive config backup
`config/ODrive config Files/odrive_pro_leg_config.json:154` (`current_soft_max: 10.0`), applied
only by the operator-run `tools/odrive_fleet_reflash.py`; no test cross-checks it against
`leg_curr_limit_a`, so it now silently disagrees with the live value. Left alone — out of this
ripple's scope and not something to touch without the operator's own reflash action.

## Ripple

- `sites.py`'s CATCH HIGH comment rewritten into a three-stage history (830 until 2026-10-05, 930
  from 2026-10-05, back to 830 on 2026-10-06); both probe tables kept.
- `hardware_config.yaml`: `leg_curr_limit_a` dated comment added; the four present-tense "10 A"
  mentions in the `torque_ff_max_nm` / `torque_ff_firmware_clamp_wire_nm` comment block reworded
  to name both the old and new value (the downstream Nm/A arithmetic in that block was computed
  against the old 10 A figure and was **not** re-derived — flagged in the comments, not fixed).
- Config regenerated (`python config/generate_config.py`): `hardware_config.{h,py}` (generated +
  3 copied-to consumers) changed; `LEG_CURR_LIMIT_A` is `15.0f`/`15.0` everywhere.
- `report.py`'s module docstring (live flight-timing example) and `two-ball-skill-stack.md` § 2.7
  (the `/balls` prediction-plane description) both stated the plane as a present-tense 930 mm
  fact — corrected to 830, with the 930 window kept as dated history. § 0 item 5's catch-high
  amendment kept, with a note that the FLIGHT-EQUIVALENT apex definition it introduced stays
  necessary (release 860 / catch 830 still don't coincide) and needed no further migration.
- Tests pinning the live constant's **value** (not a deliberately-frozen scenario) were fixed:
  `test_skills_sites.py`'s provenance-pin test, and two `test_skills_executor.py` tests whose
  measured numbers shift with the plane (`test_an_observed_flight_outside_the_band_leaves_no_row`:
  6.140 -> 6.090 m — note this is *not* the old pre-rise-aware 830 mm figure either, since the
  rise-aware apex computation and 830 were never both exercised before now;
  `test_the_next_catch_s_priors_fly_from_the_offset_release`: back to -3.5582, byte-identical to
  the original pre-catch-high measurement).
- Tests that deliberately **freeze** a specific catch-plane value for a characterisation
  (`test_skills_admissible.py`'s `_TINY_SWEEP_CATCH_Z_MM`, `test_skills_plan_bench.py`'s
  `r3_catch_plane`) were left at their pinned value, with a short note that the freeze now
  coincides with the live default again.
- `tests/ros/test_install_segment.py`'s `_sitting6_rest_after_a_high_catch` harness: tried
  following the live constant per plan, recomputing `vel_z` for 830 mm from the given apex formula
  (-4380 mm/s) — this made the STALE_STATE refusal it exists to characterise **never reproduce**,
  even at the slowest solve ever measured on the robot (73 ms), because the shorter 830 mm runway
  no longer moves the hand far enough to cross the 1.0 rev deviation bound. That is itself a
  useful finding (reverting the catch plane also removes the REST-RETRY risk catch-high
  introduced) but left nothing for the three tests to exercise, so the harness was instead
  **pinned** at the historical 930 mm / -5520 mm/s sitting-6 numbers (same pattern as the two
  frozen fixtures above) — all three tests pass unchanged from before the ripple.
- `session_skills_r5_sitting6.md` and `tools/probes/feed_lateral_miss.py`: one-line notes added
  pointing at the 2026-10-06 reversion; their historical 830/930 narrative is otherwise untouched.
- `tests/hardware/{free_platform,supported_platform,single_leg}_test.py`'s
  `SAFE_CURRENT_LIMIT_A = hw.ODRIVE_LEG_CURR_LIMIT_A * 0.5` already follows the live constant; only
  the stale `# 10A` trailing comment was corrected to `# 7.5A`.
- Left alone (bench-rig / unrelated constants, not consumers of `leg_curr_limit_a` or
  `sites.CATCH_CUP_Z_MM`): `tests/hardware/{torque_step_test,cogging_bench_test,kt_lib,
  friction_ff_demo,bench_leg_sysid}.py`'s standalone `HARD_CURRENT_LIMIT_A`/`BENCH_CURRENT_LIMIT_A`
  (brake-resistor-sized bench safety caps, numerically 10 A for their own reasons);
  `hand_jam.py`'s `relief_curr_a` default and its test assertions (hand-axis jam relief current,
  unrelated to the leg clamp); `admissible.py:252` and `executor.py`'s sitting-6 bug-provenance
  comments (historical narrative justifying a permanent fix, not live-value claims).
- Did **not** touch `config/generated/admissible_box.yaml` (orchestrator re-sweeps it).

## Verification

- 2026-10-07 (continuing the 2026-10-06 evening session past midnight),
  `python -m pytest tests/motion/test_skills_executor.py tests/motion/test_skills_schedule.py
  tests/motion/test_skills_admissible.py tests/ros/test_install_segment.py
  tests/ros/test_skill_node.py tests/ros/test_ball_tracker_flight_fit.py
  tests/ros/test_ball_tracker_frame_stamp.py tests/ros/test_ball_tracker_gate.py
  tests/ros/test_ball_tracker_identity_wire.py -q -p no:cacheprovider -n 4 --dist loadfile`
  (`tests/motion/test_hardware_config*.py` from the original command matches no file in this
  worktree and was omitted): **695 passed, 20 failed in 374.79 s**. All 20 failures are the one
  expected, pre-announced cascade — `sites.py` is a gated file (`admissible.gate_hash`), so
  editing it changed the live gate hash to `f9e081deeaea` against the committed
  `config/generated/admissible_box.yaml`'s `gate_hash='3c9533225417'` for site pair `('P1','P1')`
  — the orchestrator re-sweeps this next:
  `tests/motion/test_skills_admissible.py::test_the_committed_box_is_swept_at_the_launch_defaults_and_the_live_gate`
  and, in `tests/ros/test_skill_node.py`: `test_self_toss_compiles_a_schedule_at_the_owner_operating_point`,
  `test_self_toss_while_running_is_refused`, `test_a_bad_plant_id_is_refused_before_the_platform_moves`,
  `test_an_unreadable_memory_file_is_refused_not_raised`, `test_self_toss_is_refused_when_go_to_pose_is_unavailable`,
  `test_self_toss_is_refused_when_commanded_position_is_stale`, `test_self_toss_is_refused_when_the_prelevel_move_is_refused`,
  `test_the_tick_dispatches_the_opening_rest_bridge_through_the_mocked_client`,
  `test_check_reports_ok_when_everything_is_fresh_and_the_box_is_valid`, `test_installer_reports_service_unavailable`,
  `test_the_live_catch_aim_default_is_the_trackers_converged_fit` (manifests as an `AttributeError`
  on `node._executor` — `_start_pattern` never created an executor because the same box refusal
  fired first), `test_the_aim_source_parameter_reaches_the_executor[schedule]`,
  `test_the_aim_source_parameter_reaches_the_executor[schedule_hand]`,
  `test_an_unknown_aim_source_falls_back_to_the_live_default`,
  `test_the_opening_rest_is_sized_to_home_a_displaced_hand`, `test_a_hand_already_at_home_gets_the_default_opening_rest`,
  `test_the_activate_park_is_not_a_special_case`, `test_the_homing_rest_is_refused_when_the_hand_position_is_unread`,
  `test_the_sized_rest_is_logged_with_its_own_peaks`. No other failures.
- **Admissible re-sweep S7** (`scratchpad/sweep_run_s7.sh`, 2026-10-07 00:24–00:46, the same
  grids and arguments as S4d/S5/S6b: `tools/admissible_sweep.py --dwell-s 0.27 --leg-vel 350
  --leg-acc 5000 --leg-jerk 200000 --hand-acc 3900` with `--site-pairs columns --single-apex 0.85
  0.90 0.95 --single-apex-halfwidth 0.025 --separation-mm 125` / `--site-pairs hop --single-apex
  0.80 0.85 0.90 0.95 --single-apex-halfwidth 0.025 --hop-separation-mm 250` / `--site-pairs single
  --single-apex 0.5 0.6 0.7 0.8 0.9`, three in parallel at one BLAS thread each; logs
  `temp/logs/admissible_sweep_S7_{columns,hop,single}_20261007.log`, driver
  `temp/logs/sweep_S7_driver.log`): columns 12.6 min, hop 20.3 min, single 22.3 min, all rc 0;
  merged single+hop+columns into `config/generated/admissible_box.yaml` (19 boxes, gate
  `f9e081deeaea`, md5 `879a054760de7390c203dc9068343003`). **Every box is content-identical to the
  pre-catch-high box of 2026-10-05 morning** (`7a0e4384^`, gate `6f535fc4f888`): the only
  differences are `swept_at` and `gate_hash`, so the reversal reproduces the 830 geometry
  exactly and the columns apex ceiling is back at 1.047 m. (The current limit does not enter the
  sweep; the gate hash moves because `hardware_config.py` is gated.)
- After the merge (2026-10-07, `python -m pytest tests/motion/test_skills_admissible.py
  tests/ros/test_skill_node.py tests/motion/test_skills_sites.py tests/motion/test_skills_report.py
  tests/ros/test_skills_plan_bench.py -q -n 3 --dist loadfile`): **386 passed in 170.11 s** — the 20
  gate-hash failures above cleared.
- Fed sims at the 830 plane, hop entry (2026-10-07, `python sim/skills_gate.py --learn --no-viewer
  --pattern columns --apex-m 0.95 --separation-mm 125 --target-throws 30 --feed-angle-deg 11.9
  --feed-speed-mmps 5600 [--one-ball-fed] --columns-entry hop`, logs
  `temp/logs/skills_gate_columns{,_1ball_fed}_hop_830_20261007.log`): two-ball **PASS 5/5** (1
  attempt each, 33 makes, 0 drops, longest 30, 270.7 s); one-ball fed **PASS 5/5** (6 attempts,
  37 makes, 0 drops, longest 30, 356.9 s) — the same counts as the 930 runs of 2026-10-05.
- Audit (`/audit --unstaged`, 2026-10-07, one pass over both this unit and the orchestrator stow
  unit): no behaviour-affecting finding on this unit; two narrative findings applied
  (`feed_lateral_miss.py`'s plane-window dates now name 2026-10-05 ~21:30 to 2026-10-06 evening in
  both the docstring and `--plane-mm`'s help; the INDEX row's "box re-swept" is true as of this
  commit).
- Full gate (`./run_tests.sh --full`, run 2026-10-07, log `temp/logs/gate_full_r5_plane830_curr15_stow_20261007.log`, on the tree holding this unit, the orchestrator stow unit and the merged box): **parallel 6125 passed, 9 skipped, 1 xfailed in 305.26 s; serial 6 passed in 20.51 s; RESULT PASS, exit 0.** The only edits after that run are these Verification lines; the logbook tests were re-run after them.
