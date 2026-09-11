---
title: "Skill-stack R1 — one hand master: the can-bridge streamed lane is the sole axis-6 writer; hand_source latch, hand_ops conduit, Platform stroke engine, SetHandTrajCmd and the hand_stroke.py timing twin deleted; PROTOCOL_VERSION 7, bridge FW 21, Platform FW 7 — committed UNFLASHED"
type: refactor
date: 2026-09-11
status: in-progress
phase: "two-ball-skill-stack — R1 (software landed; flash sitting pending)"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/ (hand_source.*, hand_ops.* deleted; leg_interp, fault_machine, telemetry, rpc, platform_relay, can_buses, leg_activate, leg_deactivate, canbridge_config FW 21)
  - ros_ws/src/jugglebot/Teensy_code_platform/ (Trajectory.h deleted; Teensy_code_platform.ino FW 7)
  - config/generate_udp_protocol.py (PROTOCOL_VERSION 7) + config/generated/*
  - config/hardware_config.yaml (hand_mm_per_rev, hand_settle_band_rev, hand_park_band_rev, hand_activate_position_rev; linear_gain_factor, hand_spool_radius_m, arm_window_margin_s and the stroke-engine profile keys deleted)
  - config/protocol_config.yaml (TRAJ_CMD 0x6D0 retired)
  - ros_ws/src/jugglebot_interfaces/srv/SetHandTrajCmd.srv (deleted)
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/hand_stroke.py (deleted); cup_cycle, cup_realize, feasibility, throw_envelope, unified_cycle re-pointed
  - ros_ws/src/jugglebot/jugglebot/{teensy_bridge_node,catch_coordinator_node,reload_coordinator_node,toss_sequencer,toss_session}.py
  - teensy_link/{rpc,rpc_args,protocol}.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - tests/hardware/session_skill_stack_r1_flash.md (new); tests/hardware/{hand_stream_bench,unified_cycle_bench}.py
  - tests/firmware/test_hand_single_master.py (new); tests/firmware/native/* ; ~40 test files re-pointed, 9 deleted
  - sim/unified_gate.py (MIRROR_TOL_LEG_REV re-banded), sim/plant/mujoco_plant.py
  - ros_ws/docs/{platform_fw_version,hand_throw_envelope,hand_command_continuity}.md; ros_ws/docs/hand_decel_feedforward.md (deleted)
  - tools/probes/{hand_stroke_timeline,cadence_rung_check,ilc_speed_band}.py, tools/probes/archived/hand_dispatch_ladder.py (deleted)
  - plans/active/two-ball-skill-stack.md
subsystem:
  - firmware
  - motion
  - config
  - teensy_link
tags:
  - safety
  - protocol
---

# Skill-stack R1 — one hand master (software; UNFLASHED)

Plan `two-ball-skill-stack.md` § 4 R1, owner decision 1.3 (2): retire the Platform Teensy
stroke engine first. After this commit exactly one thing writes CAN node 6 at any instant:
the can-bridge's 500 Hz interp tick, whenever a `HAS_HAND` Setpoint frame is latched. The
mastery latch (`hand_source`), the host-driven stroke conduit (`hand_ops`, `HAND_TRAJ_CMD`,
`HAND_SOURCE_SET`, `ERR_HAND_SOURCE`, HeartbeatT2J bit 6, the BridgeTxDiag per-stage
counters), the Platform's 0x6D0 decode / 0x0C9 hand-encoder cache / `Trajectory.h`, the
`SetHandTrajCmd` service and every client, and `hand_stroke.py` (the host twin of the
stroke engine's timing) are deleted, not parked. Removing message types and shrinking a
struct is a wire change, so **PROTOCOL_VERSION 6 → 7**: a FW ≤ 20 board against this host
is DARK (`link=NO_HEARTBEAT`, `decode_errors == rx_frames`) by design. Bridge FW 20 → **21**,
Platform FW 6 → **7**, both built (`pio run`, no upload) — **nothing is flashed**; the
lockstep flash and the bench ladder are `tests/hardware/session_skill_stack_r1_flash.md`.

## Owner decisions (2026-09-11, asked before the host units ran)

1. **Hand deviation guard boots ARMED.** With one master and no stroke-engine prelude an
   observing guard is no guard: a jammed hand would be driven at the ODrive current limit
   with nothing watching. `hand7 observe` stays as a bench verb and lasts ONE armed session: the
   disarm edge that ends the session (`interp_set_output_enabled` true→false) returns
   the switch to ARMED, so an operator who forgets `hand7 arm` cannot leave the guard
   off for the next session (the audit caught the first cut claiming an "arm edge"
   did this through `interp_reset()`, which has no runtime caller; the disarm-edge hook
   and its native test are the fix). The trip has never fired on hardware (row 18 was closed on thermal
   grounds), so the runbook observes it COLD: a gap re-entry with a 3 rev delta passes the
   5 rev pump gate and exceeds the 2.5 rev band on the first tick, hand free, no rotor
   holding. Nuisance margin measured 2026-09-05: worst 0.25 rev = 10 % of band, zero trips.
2. **ACTIVATE parks the hand at 0 rev.** Homing is unchanged (IDLE on the stop at −0.10 rev;
   the FW 18 mode/limits restore at SET_REF was already the fix the prompt asked for).
   `leg_activate` widens to axis 6: CLOSED_LOOP → TRAP_TRAJ move to
   `JBOp::HAND_ACTIVATE_POSITION_REV` (0.0, the clip floor) at `GENTLE_MOVE_VEL_LIMIT_RPS`
   → POSITION/PASSTHROUGH hand-off, a failed hand-off failing the activate; `leg_deactivate`
   idles axis 6 first. The FW 17 row-13 residual (a park 0.1 rev below the clip floor read as
   a permanent 0.1 rev deviation, 1:1) is gone by construction, and the operator
   `--close-loop` energise step leaves the launch-up path.
3. **Reload: keep R1 lean.** The Ball Butler reload's reactive catch stroke dies with
   `set_hand_traj_cmd`; `catch_coordinator_node` never arms a hand stroke (the plan owns the
   hand); R3's reset is operator ball placement. No throwaway LANDING port for R4 to delete.
4. **The legacy kind-0 toss branch is made unreachable and loud now.** Its device is gone
   (`hand_stroke.py`, `set_hand_traj_cmd`, the kind-0 dispatch budget and dwell floors in
   `toss_sequencer`/`toss_session`), and — after the audit caught the flattened
   `_dispatch_toss` returning a lying THROW_DISPATCH_OK for a non-unified goal — a
   `TossContinuous` goal with `unified_cycle` false and any plain `Toss` goal are REFUSED
   at accept with one code naming the fact — `REJECTED_STROKE_ENGINE_RETIRED`
   (`reload_coordinator_node._stroke_engine_retired_detail`) — and the dispatch seam
   refuses with the same code. The FSM's other `unified` branches stay in place behind that gate
   until R4 deletes the stack under `fsm-final` (plan § 1.3 (3)); their tests go now as
   tests of an unreachable branch.

## What changed, by layer

- **Bridge firmware.** Lane gate is `HAS_HAND` alone (I-HAND-4 arm-edge knot latch and
  I-HAND-5 unseen-skip unchanged; the falling-edge decay unchanged). `TxCls::HAND` deleted
  (console-only, renumbered); wire ids 0x54/0x55/ERR 0x07/flag bit 6 are never-reuse holes.
  `s_hand_dev_guard_armed` power-on `true`, re-armed by the disarm edge in
  `interp_set_output_enabled` (native test "hand7 observe lasts ONE armed session").
  `leg_activate`/`leg_deactivate` cover
  `NUM_AXES`; the `coldstart` interlock in `leg_interp.cpp` keeps the streamed producer
  silent through the whole park, so there is one writer at every instant. BridgeTxDiag
  42 → 18 B. Native harness: `test_hand_ops.cpp` deleted, activate/deactivate cases added,
  the INVARIANTS Gaps 5 SUBCASE landed ("setpoint staleness E-STOPs and LATCHES until an
  explicit clear"). New `tests/firmware/test_hand_single_master.py` (Gaps 1) allow-lists the
  four TUs that may touch axis 6 by role and asserts exactly one streamed producer.
- **Platform firmware.** `Teensy_code_platform.ino` 1233 → 991 lines; SCL3300, TimeSync slave,
  0x6E0 RobotState + FW_VERSION reply and the cold-start state stay; the board never
  transmits to node 6 (I-FW-15 block at the top). `ST_BUSY` kept as a reserved status so a
  pre-7 board's replies keep their meaning.
- **Config.** One measured key `jugglebot_geometry.hand_mm_per_rev: 32.567` (owner readings
  2026-09-06 and 2026-09-11 over 352.0 mm stop-to-stop, agreeing to 0.01 %) generates
  `HAND_REV_PER_M` = 30.706 rev/m, replacing the 1.035 "just 'cuz" factor ÷ 2π·0.00521
  (31.617 rev/m, +3 %). `hand_settle_band_rev` 0.10 and `hand_park_band_rev` 0.5 are the
  single sources of two bands that had three and two homes (INVARIANTS Gaps 9);
  `hand_activate_position_rev` 0.0 is the park. Stroke-engine profile keys with no live
  consumer deleted; `teensy_trajectory` keeps only physical limits.
- **Host.** `teensy_link` loses the hand RPC packers; `EXPECTED_BRIDGE_FW_VERSION` 21,
  `PLATFORM_FW_VERSION_EXPECTED` 7. `teensy_bridge_node`'s arm gate runs the CLOSED_LOOP /
  POSITION-PASSTHROUGH hand preamble unconditionally on a `HAS_HAND` frame (the idle-axis
  trap is real; the latch never was). `catch_coordinator_node` 1511 → 993 lines,
  `reload_coordinator_node` 12 955 → 12 731, the kind-0 dispatch budget and dwell floors
  gone from `toss_sequencer`/`toss_session`. `throw_envelope.evaluate()` loses the
  ARM_WINDOW carve-out and its `arm_window` kwarg; it keeps end stop, decel/carry torque
  authority (C-HAND-2 restated there), current headroom, regen, wire band and the measured
  coast ladder. `hand_stream_bench.py` loses `--source-only`, `--no-source-switch` and the
  stroke stage, and gained a real fix: its `sys.path` was hardcoded to the main checkout, so
  a worktree run imported the wrong tree.
- **Two physics findings from the units, both kept.** (a) `throw_envelope` keeps the
  generated release-top position (`HAND_STROKE_TOP_REV`) rather than the clip ceiling: both
  margins are 0.2 rev, so the clip-ceiling version refused every throw (END_STOP bound at
  every speed). (b) With the ARM_WINDOW floor gone, `MIN_FLIGHT_TIME_S` sits at the search
  bracket floor for a self-toss (release and catch heights ≈ 6.7 mm apart); no hardware
  bound is affected; R2's admissible sweep owns the flight-time floor.
- **Sim gate re-band.** The measured gain moved the 60 mm ring's plan; the masked-`HAS_V1`
  fault on its leg lane now lands at 4.067e-04 rev, inside the old 5e-4 band, which the
  non-vacuity test correctly refused. Honest reconstruction unchanged (5.2e-7). Band re-cut
  to 1e-4 rev (fault 4.1× outside, honest 200× inside, 2026-09-05 grid worst 9.4× inside).
  The decay test's 1e-9 clamp slack became 1e-6 = one float32 ulp at 10 rev (measured 9.2e-8
  over; the twin clamps the float32 wire value).
- **Tests that pinned sitting numbers through the old gain** (`test_unified_launch_floor`,
  `test_unified_cycle_bench`) now derive them from `hw.HAND_REV_PER_M`; the 2026-09-08
  sitting's bag inversion keeps its own frozen floor `FLOOR_REV_SITTING_2026_09_08`.

## Deletion ledger (grep-to-zero, 2026-09-11, over `ros_ws/src teensy_link tools tests sim config controller`)

`hand_source` 125 → 0 code (dated retirement notes remain), `hand_ops` 69 → 0,
`HAND_TRAJ_CMD` 26 → 0, `HAND_SOURCE_SET` 25 → 0, `ERR_HAND_SOURCE` 15 → 0, `0x6D0` 46 → 0
(one retired-id literal in `test_platform_relay.cpp`), `hand_stroke` module 24 importers → 0,
`LINEAR_GAIN_REV_PER_M` / `TEENSY_LINEAR_GAIN` / `linear_gain_factor` / `hand_spool_radius_m`
(jugglebot side) → 0, `Trajectory.h` → firmware-history comments only. Deleted files: 4
firmware, 1 header, 1 srv, 1 motion module, 4 probes, 1 doc, 9 test files. `sim/hand/trajectory.py`
KEPT — live sim importers (`sim/hand/planner.py`, `coordinator.py`, `sim/toss_gate.py`,
`sim/input/*`); it is the sole implementation now, not a mirror.

## Process note (for the token-budget rule)

Eight agent units plus three follow-ons (H3b, and H6/H7 for the audit's dispatch-seam
finding and its test fallout). F1a 87 calls, P 62, F1b 72, RB 49 were in budget; H1 236,
H2 252, H3 180 (+ H3b 270), H4 200, H6 188, H7 115 were not — the host units were scoped by
subsystem, not by call count, and each of the four found a blast radius the enumeration
grep understated (tests that hardcode numbers through the gain; FSM tests pinning the
deleted device). Next rung: split host units by TEST FILE COUNT, ~10 files each, and give the
gain-dependent tests their own unit.

## Verification

- 2026-09-11 — `python tests/firmware/native/build.py --force` + every binary under
  `temp/firmware_native/` → 14/14 built, all SUCCESS; `pytest tests/firmware -q` → 215
  passed, 1 skipped.
- 2026-09-11 — `pio run -e teensy41` (bridge, build only) → SUCCESS; `pio run -e teensy40`
  (Platform, build only) → SUCCESS. **No upload.**
- 2026-09-11 — `pytest tests/ --collect-only -q` → 6449 tests collected, 0 errors.
- 2026-09-11 — `./run_tests.sh --full` (log `temp/logs/r1_gate_full_20260911_175551.log`):
  **parallel 6361 passed, 9 skipped, 1 xfailed, 0 failed in 317.83 s; serial 4 passed in
  25.50 s; total 349 s, rc 0.** The first full run of the day (16:13) was RED at 41
  failures — 29 continuous-toss FSM tests broken by the flattened dispatch seam (the
  audit's BLOCKING finding), 8 decel-feedforward sim-twin tests and 2 ILC-band tests
  pinned through the old gain, the generated choreography map stale, and
  `test_hand_trajectory` literals — all fixed in this entry's diff, none waived.
- 2026-09-11 — audit (`/audit --unstaged`, two lenses): 2 BLOCKING fixed here —
  (1) the "arm edge re-arms the guard" claim named `interp_reset()`, which has no
  runtime caller; replaced by the disarm-edge hook in `interp_set_output_enabled` +
  native test; (2) `_dispatch_toss(unified=False)` returned a lying OK; replaced by
  `REJECTED_STROKE_ENGINE_RETIRED` at accept and at the seam. 2 HIGH: the plan/logbook
  "legacy branch deleted" wording corrected to "refused at accept until R4"; the
  launch-floor tests re-pinned as dated absolutes, not the formula re-run. LOW: stale
  observe-first labels, the sim-gate provenance sentence. The scoped logbook/plans tests
  were re-run after this section was filled in (the only edit after the gate).

## Handoff (the sitting, then R2)

The operator's sitting is `tests/hardware/session_skill_stack_r1_flash.md`: host build →
bridge flash (banner `v21`) → Platform flash over CAN with the launch down (STATE_READ 6 → 7)
→ relaunch (`bridge_fw_version 21 (proto 7)`, no `hand_source` row) → the ladder rows
12–21 re-cut for a latch-less lane (12b ACTIVATE park, 12c hard-stop span 10.81 ± 0.01 rev
replacing the archived geometry plan's G3, 18 the cold trip) → one streamed self-toss caught
with no latch step. ⚠ The runbook is honest that no dry-run path exists for the ladder or for
`plan_cycle` — the dress-rehearsal rule needs either a validate-only verb (R2's runtime
assert is the natural home) or an explicit owner waiver for this sitting. Open after R1:
a dedicated second-master sniff counter in `telemetry.cpp` (row 19 currently proves the
Platform banner instead); `ros_ws/docs/hand_throw_envelope.md`'s numeric tables are flagged
stale pending R2's re-derivation; the `hand-geometry-correction` worktree is removed once
the sitting closes R1.
