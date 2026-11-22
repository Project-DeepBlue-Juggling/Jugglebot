---
title: "BUG-105: the Hand's vel_ff is zeroed on every suppress edge (Soft E-stop, disarm, link loss, not only MOTOR_FB_STALE): one hold frame at the last command, the torque drain carries vel_ff 0; the Legs' flush unchanged; can-bridge FW 49, NOT flashed"
type: bugfix
date: 2026-11-23
status: complete
phase: "BUG-105"
subsystem: can-bridge
related_issues:
  - BUG-105
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - teensy_link/rpc_args.py
  - tests/firmware/native/test_leg_interp.cpp
  - tests/firmware/test_bridge_fw_version_xref.py
  - docs/can_bridge/safety.md
---

# BUG-105: the Hand's vel_ff is zeroed on every suppress edge (FW 49)

**What.** On a Soft E-stop, a disarm or a link loss the Hand stays in Closed-loop and nothing commands it Idle. The ODrive held the last frame including its vel_ff and settled at `input_pos + vel_ff/35`. The bench Hand measured 0.284 rev at 10 rev/s against 0.286 predicted (2026-11-22). At the 300 rev/s cap that is about 8.6 rev on a ~10.5 rev stroke. The FW 22 tau drain also re-sent that stale vel_ff. The FW 35 hold flush (BUG-19) that zeroes it fired on MOTOR_FB_STALE only. Now the flush's **Hand** branch in `interp_isr` keys off `!out_en && !coldstart`, so every suppress reason counts; it used to key off `hold_flush`. On the first suppressed tick after the Hand was on the wire, the record's vel_ff is set to 0. A tau != 0 session's drain then carries vel_ff 0 for its 3 frames. A tau == 0 session gets one hold frame: the last transmitted pos, vel_ff 0, tau 0, sent through the single Hand encode site. FW 48 → 49, `EXPECTED_BRIDGE_FW_VERSION` 49. There is no wire change, so PROTOCOL_VERSION stays 14 and `protocol_config.yaml` does not carry the version. The comments in `leg_interp.cpp` / `.h` and `fault_machine.cpp` and `docs/can_bridge/safety.md` now describe the new scope.

## Discussion

- **Hand only.** The Legs' flush stays MOTOR_FB_STALE-only (`hold_flush`, unchanged). The Legs' vel_ff cap of 3.5 rev/s bounds their offset to about 0.09 rev, and a new frame on a Leg E-stop changes what the Platform does under gravity. That decision was not asked for. The Hand's offset is two orders larger and the Hand already transmitted on these edges through the drain. So the owner's 2026-11-14 objection ("transmit nothing a stop forbids") does not block this change.
- **Edge only, keyed off `!out_en`, not off the lane.** Keying off `!hand_wire` would also fire on a one-tick lane gap with output still enabled. That would put a vel_ff 0 frame on the wire mid-stroke, which is a jerk. A probe implementation keyed off the lane was built and the lane-gap test caught it (see Verification).
- **The Hand now brakes to the last commanded position.** It no longer runs on to `input_pos + vel_ff/35`. This is the intended outcome and BUG-105's Sitting pass criterion: settle within 0.05 rev of the firmware's last command. The vel_ff step from v to 0 is a hard stop. Its deceleration is bounded by the ODrive's own current limit, and the position loop still has the authority it had before.
- **Checked unchanged.** The constraints were checked as follows:
  - **Leg branch:** not edited.
  - **Cold-start:** the gate is `!coldstart`, and the record dies with `s_hand_tx_prev = hand_wire`.
  - **No Hand record:** boot, a disarmed bridge and the FW 34 shut all leave `s_hand_tx_prev` false, so nothing is sent.
  - **Dead bus:** `can_jugglebot_tx`'s presence gate refuses the frame, as before.
  - **Deferred Stow:** it forces output on and returns before the Hand path.
  - **FW 48 stall guard (`leg_activate.cpp` `hand_lane_stall_guard`, `s_lane_idle_pending`, `stall_reset_for_targets`, the HAND_STALL latch):** it reads only feedback and its own state, and `leg_interp.cpp` touches none of that. On a stall latch a tau == 0 Hand now gets one Set_Input_Pos (vel_ff 0) beside the IDLE Set_Axis_State, in either order. A Set_Input_Pos does not change an IDLE axis's state, and a tau != 0 Hand already got 3 drain frames there before FW 49.
- **Census.** `interp_hold_flush_edges()` now counts Hand-only edges. Its documented meaning was already "suppress edges that flushed (any lane)", and it is not uplinked.
- **Residual, not addressed.** The lane can go idle with output enabled (hand_wire false, the drain started with the last vel_ff), and then output can drop. In that order `s_hand_tx_prev` is already false at the suppress edge, so no zeroing happens. The lane decays its own vel_ff to 0 before it idles, so this window should be empty, but no test proves that.

## Verification

- 2026-11-23, `tools/night_shift_nonet.sh python -m pytest tests/firmware/test_native_firmware.py tests/firmware/test_bridge_fw_version_xref.py tests/firmware/test_hand_torque_ff_twin.py tests/firmware/test_hand_stall_guard_xref.py tests/firmware/test_config_drift.py tests/ros/test_teensy_bridge_node_bridge_diag.py tests/ros/test_teensy_bridge_node_install_skew.py tests/teensy_link/test_hand_stream_bench_tri_cut.py tests/sim/test_mapkit_sheet.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_docs_links.py tests/sim/test_no_legacy_item_ids.py -q` → **250 passed in 392.34 s**. The extra files are the readers of `EXPECTED_BRIDGE_FW_VERSION` (`grep -rln`) and the logbook and docs readers. The default gate (`./run_tests.sh`) was not run by this unit.
- 2026-11-23, the new and changed doctest cases were run by name (`/tmp/probe_bug105.py`, uncommitted) against three builds:
  - **The fix:** all pass.
  - **The Hand branch reverted to `hold_flush`:** three cases fail. They are "BUG-105: a disarm-style suppress puts ONE Hand hold frame out", the updated FW 22 LEGACY no-drain case and the updated FW 22 DRAIN case.
  - **A lane-keyed `!coldstart` variant:** "BUG-105: a lane gap with output ENABLED" fails.
  - **Guards that pass on all three builds:** the cold-start and no-record cases.
- Not flashed, not flown. The proof is the Item's bench-Hand Sitting.
