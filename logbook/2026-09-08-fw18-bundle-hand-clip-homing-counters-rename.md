---
title: FW 18 bundle — host-side ripple (hand clip, homing restore, counter gate, rename) + Platform FW 4
type: feature
date: 2026-09-08
status: resolved
phase: "unified-7dof-planner — FW 18 bundle"
related_plan: unified-7dof-planner.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/can/odrive.py
  - sim/hand/trajectory.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/hand_stroke.py
  - ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino
  - ros_ws/src/jugglebot/Teensy_code_platform/Trajectory.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - teensy_link/rpc_args.py
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - experimenting/platform_calibration/measuring_leg_mapping/can_interface.py
  - tools/probes/hand_stroke_timeline.py
  - tests/ros/test_odrive.py
  - tests/ros/test_unified_cycle_bench.py
  - tests/ros/test_gui_geometry.py
  - tests/sim/test_hand_trajectory.py
  - tests/motion/test_throw_envelope.py
  - tests/motion/test_hand_stroke.py
  - tests/firmware/test_hand_smooth_move_xref.py
  - tests/hardware/unified_cycle_bench.py
  - tests/hardware/hand_stream_bench.py
  - tests/hardware/bench_leg_sysid.py
  - tests/hardware/teensy_guard_validation.py
  - tests/hardware/session_fw18_flash.md
  - plans/active/unified-7dof-planner.md
subsystem:
  - can
  - motion
  - sim
  - config
tags:
  - safety
  - testing
---

# FW 18 bundle — host-side ripple (hand clip, homing restore, counter gate, rename) + Platform FW 4

## Summary

Unit F2 of the FW 18 bundle: the host-side ripple of the firmware unit (F1)
that moved the hand's compiled clip off the metal (`stop − hand_clip_margin_rev`
= 10.501 rev), restored axis-6 mode/limits after homing, gated the hand-lane
counters on `s_output_enabled`, added `hand7 reset`, and renamed
`MPC_CMD_STALENESS_US` → `SETPOINT_STALENESS_US`. This unit propagates the
stop's move (10.8 → 10.701 rev) through every host-side consumer and test that
had it hardcoded or derived a now-stale number from it, fixes the two items the
firmware handoff called out as urgent (the host's own hand clip in
`can/odrive.py`, and `sim/hand/trajectory.py`'s hardcoded stop feeding the
Platform-Teensy xref), and — because that xref exposed a real behavioural
change to a *different* firmware image — bumps the Platform Teensy to FW 4 and
follows through its own version-bump rule and reserved-number consequences.

## Fix

**Urgent items (named by the firmware handoff):**

- `ros_ws/src/jugglebot/jugglebot/can/odrive.py:54` — `HAND_MOTOR_MAX_POSITION`
  now reads `hw.GEOM_HAND_MOTOR_HARD_STOP_REVS - hw.GEOM_HAND_CLIP_MARGIN_REV`
  (10.501), not the bare metal stop. It no longer disagrees with the firmware's
  own clip by 0.2 rev. `tests/ros/test_odrive.py::test_hand_above_max_clips`
  updated to match.
- `sim/hand/trajectory.py:26,109` — imports `jugglebot.hardware_config as hw`
  (the same convention `sim/plant/mujoco_plant.py` already uses) and
  `HAND_MOTOR_HARD_STOP_REVS` now reads `hw.GEOM_HAND_MOTOR_HARD_STOP_REVS`
  instead of a hardcoded `10.8`. This is what makes
  `tests/firmware/test_hand_smooth_move_xref.py` compare the Platform Teensy's
  `Trajectory.h` against a mirror that agrees with the current YAML, not a
  frozen literal — and it moves the smooth-move ceiling **10.6 → 10.501 rev**,
  because `SMOOTH_MOVE_POS_CEIL_REV = HAND_MOTOR_HARD_STOP_REVS −
  SMOOTH_MOVE_EXCURSION_MARGIN_REV` and only the base moved this time (see
  Discussion).

**Platform Teensy version bump (this unit's own decision):**

- `ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino:107` —
  `FW_VERSION = 3 → 4`, with a bump-history clause following the file's own
  rule ("bump on any behavioural change worth telling a bench operator about").
  `teensy_link/rpc_args.py:342` — `PLATFORM_FW_VERSION_EXPECTED = 3 → 4`, with
  the matching history paragraph.
  `ros_ws/src/jugglebot/Teensy_code_platform/Trajectory.h:379` — the
  `smoothMoveMaxDuration()` doc comment updated from `0 -> 10.8 rev = 0.78964 s`
  to `0 -> 10.701 rev = 0.78602 s` (a live description of current behaviour,
  not historical narrative — it was wrong the moment the constant moved).
  `ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h:246` — the
  `MAX_DEVIATION_HAND_REV` sizing-rationale comment's stroke reference updated
  10.8 → 10.701 (conclusion unchanged, arithmetic now correct).
- `plans/active/unified-7dof-planner.md` — Phase 6's reserved "Platform Teensy
  FW 4" (stroke-engine retirement) renumbered to **FW 5** at all four sites
  (the exclusivity diagram, the phase table, the Phase 6 prose, the retire-list
  table), since this unit consumed FW 4. A one-paragraph status update was
  added to § "FW 18 bundle": BUILT 2026-09-08, NOT FLASHED, hex md5, and a
  pointer to this entry.

**Every remaining host consumer of the stop, swept and fixed** (grep sweep
below shows the accounting): `tests/sim/test_hand_trajectory.py` (four tests
recompute against the new ceiling/cap: `test_the_overshoot_is_bounded_by_v0_T_
times_h_max`, `test_the_continuity_band_is_narrow_and_that_is_reported`,
`test_an_honoured_prelude_never_outlasts_a_rest_to_rest_move`,
`test_the_deepest_honoured_brake_is_bounded_and_the_probe_knows_it`, plus the
`10.8`/`10.701` literal at `test_the_ceiling_is_converted_with_no_stroke_
margin_term`); `tests/motion/test_throw_envelope.py` (four tests recompute:
`test_the_end_stop_claim_survives_a_strict_upper_envelope`,
`test_the_torque_bounds_bind_and_end_stop_does_not`,
`test_the_clamp_was_the_coast_mechanism`,
`test_the_shipped_working_point_is_now_admitted`); `tests/motion/test_hand_
stroke.py::test_the_overshoot_and_the_affordable_velocity_band`;
`ros_ws/.../motion/trajectory/hand_stroke.py`'s docstring numbers;
`tests/firmware/test_hand_smooth_move_xref.py` (`test_the_end_stop_ceiling_
agrees_with_the_firmware_expression`, `test_velocity_continuity_is_actually_
achieved_at_the_seam` — `v0=9.0` no longer fits the shrunk headroom, replaced
with a value under the new limit); `tests/ros/test_unified_cycle_bench.py`
(the hard-stop-vs-generated-config test now expects `stop − margin`; the
"clip sits BEYOND the metal" test renamed and inverted — the clip now sits
*short* of the metal); `tests/hardware/unified_cycle_bench.py` (the constant,
the module docstring's stroke-clip warning, and three narrative blocks
rewritten to describe the fixed state while flagging that the board is still
FW 17 until flashed); `tests/hardware/hand_stream_bench.py`,
`experimenting/platform_calibration/measuring_leg_mapping/can_interface.py`
(the standalone actuating script's own literal — a real safety-relevant clip,
also moved 10.8 → 10.701), `tools/probes/hand_stroke_timeline.py`,
`tests/ros/test_gui_geometry.py` (docstring accuracy).

**The historical `MPC_` rename, host side:** `config/generate_udp_protocol.py`
line 159's `SETPOINT` MsgType comment ("40 Hz MPC setpoint waypoints" → "40 Hz
setpoint waypoints"), regenerated into `config/generated/udp_protocol.{h,py}`
and the canbridge consumer header (comment-only diff, confirmed with `git
diff`). `tests/hardware/bench_leg_sysid.py` and
`tests/hardware/teensy_guard_validation.py`'s stale `MPC_CMD_STALENESS_US`
comments updated to `SETPOINT_STALENESS_US`.

## Discussion

**Why the smooth-move ceiling moved this time, when the 2026-08-18 correction
held it fixed.** Both corrections move the same base
(`HAND_MOTOR_HARD_STOP_REVS`), and the ceiling is `base − margin`. In
2026-08-18 the margin was widened in the same commit (0.5 → 0.2) specifically
to hold the ceiling at 10.6 exactly — a deliberate choice to avoid touching a
second firmware image's behaviour. This unit's base move (10.8 → 10.701) has
no compensating margin change, because `SMOOTH_MOVE_EXCURSION_MARGIN_REV` is
the *Platform* Teensy's own margin and is untouched by the can-bridge's new
`hand_clip_margin_rev` key (they are different guards on different firmware —
see the firmware handoff's item 1 for why a shared key would be wrong). So the
ceiling really did move, 10.6 → 10.501, and that is a behavioural change to a
board this unit does not flash. Per `ros_ws/docs/platform_fw_version.md`'s
rule ("bump on any behavioural change worth telling a bench operator about" —
the exact wording used for the 2→3 bump over the same kind of change), the
Platform Teensy takes a version bump rather than a silent drift. Reserving "the
next number" for Phase 6 in the plan would have made this unit's bump land on
a number already promised elsewhere, so Phase 6's `FW 4` moves to `FW 5`
instead of leaving two different behavioural changes claiming the same version.

**Why the host's own hand clip and the firmware's must be read from the same
generated key, not just numerically equal today.** Before this fix,
`can/odrive.py`'s `HAND_MOTOR_MAX_POSITION` was `hw.GEOM_HAND_MOTOR_HARD_STOP_
REVS` — the bare metal stop, with no margin. That was already wrong under FW
17 (a 0.099 rev disagreement with the firmware's own zero-margin alias was
masked only because both were "the stop" under different names), and it would
have been *silently* wrong again the next time either margin changed
independently, because nothing pinned the two clips to the same expression.
Reading `stop − GEOM_HAND_CLIP_MARGIN_REV` ties the host clip to the exact
expression the firmware now computes, so a future margin change ripples to
both sides through one generated key instead of needing two hand-edits kept in
sync by discipline alone.

## Verification

```
2026-09-08  pytest tests/ros/test_teensy_bridge_node_recover.py tests/ros/test_teensy_bridge_node_shutdown_stow.py tests/ros/test_state_machine.py tests/ros/test_orchestrator_node.py tests/ros/test_trajectory_node.py tests/ros/test_launch_nodes.py tests/ros/test_gui_geometry.py tests/motion/test_blas_threads.py tests/motion/test_bench_sysid_bridge.py tests/teensy_link -q
            → 1016 passed in 33.49 s   (pre-existing MPC_STALE->SETPOINT_STALE rename verification, inherited from before this unit)

2026-09-08  pytest tests/firmware/test_native_firmware.py -q
            → 18 passed in ~193 s   (firmware unit F1's own verification, inherited)

2026-09-08  pytest tests/firmware -q
            → 414 passed in 21.22 s   (all three F1-reported xref failures fixed by this unit)

2026-09-08  pytest tests/motion/test_hand_stroke.py tests/motion/test_throw_envelope.py tests/sim/test_hand.py tests/sim/test_hand_throw_decel_ff.py tests/ros/test_teensy_bridge_node_recover.py tests/ros/test_launch_nodes.py -q
            → 130 passed in 6.82 s

2026-09-08  pytest tests/ros/test_odrive.py tests/ros/test_unified_cycle_bench.py tests/sim/test_hand_trajectory.py -q
            → 364 passed in 2.77 s

2026-09-08  pytest tests/motion/test_hand_stroke_timeline_probe.py -q
            → 22 passed in 3.13 s

2026-09-08  pytest tests/sim/test_plans_index.py tests/sim/test_logbook_search.py tests/sim/test_logbook_front_matter.py tests/ros/test_choreography_map.py -q
            → 137 passed in 9.14 s
```

**First full gate on the bundle** (2026-09-08 10:48–10:56, `./run_tests.sh --full`): parallel
**3 failed / 6962 passed / 4 skipped / 2 xfailed in 477.63 s**, serial 4 passed, exit 1. All
three were tests pinning a number derived from the old 10.8 rev stop that the host sweep
missed — the catch prime's clearance to the metal (0.8406 → **0.7416 rev**, 23.5 mm, and a new
assertion that the prime sits below the FW 18 wire clip, 0.5416 rev of headroom), the
smooth-move duration cap (0.78964 → **0.78602 s**, it scales with √stop), and the validate-cycle
end-stop message ('10.80 rev' → the YAML value, '10.70 rev'). Fixed at the sites with the
derivation in the comment; the fixed files alone (2026-09-08,
`pytest tests/ros/test_reload_coordinator_node.py tests/ros/test_catch_coordinator_node.py tests/motion/test_validate_cycle.py -q`):
**199 passed in 11.16 s** (10:58).

**Full gate after those fixes** (2026-09-08 10:58–11:06, `./run_tests.sh --full`): parallel
**6965 passed / 4 skipped / 2 xfailed in 456.21 s**, serial **4 passed in 26.33 s**, total 489 s,
**exit 0**.

## Open items

- `ros_ws/gui/js/state-minimap.js`'s three `MPC_STALE` comments are
  deliberately untouched — a parallel session owns `ros_ws/gui/` this session.
- The geometry correction queued next (`plans/active/hand-geometry-correction.md`
  — `hand_stroke_mm`, `linear_gain_factor`, `hand_spool_radius_m`,
  `teensy_trajectory.hand_stroke_m`) was deliberately not touched here; several
  comments this unit edited (e.g. `config/hardware_config.yaml`'s
  `hand_stroke_mm` derivation, still saying "10.8 − 0.2 = 10.6") are left for
  that unit on purpose.
- `tests/hardware/session_anomaly_fixes.md` (a 5432-line historical
  bench-validation runbook for the 2026-07-25 anomaly-fix run, already carrying
  several dated 10.6/10.8 corrections inline and superseded for live toss
  testing by `session_phase8_toss_hardware.md`) was NOT swept for the FW 18
  hard-abort numbers — a full rewrite of its ~30 occurrences was judged out of
  proportion to this unit's scope and risk of introducing errors in a large
  archival document under time budget. Flagged here rather than silently
  skipped; a future session should either sweep it or explicitly mark it
  superseded.
- The flash itself (both Teensies) is the operator's — see
  `tests/hardware/session_fw18_flash.md`. Neither Teensy has been flashed by
  this unit; the can-bridge board still runs FW 17 and the Platform Teensy
  still runs FW 3.
