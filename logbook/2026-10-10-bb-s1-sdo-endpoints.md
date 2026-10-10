---
title: Ball Butler's hand ODrive S1 gets its own SDO endpoint ids from the S1 0.6.11-1 table (can.input_torque_scale 273, input_vel_scale 272, node_id 262, fw/hw version 10-12/6-7) for BB FW 8's at-arm input-scale guard; the S1 commutation_mapper_pos_abs was 488 (= task_times.can_heartbeat.length), now 451; BB_FW_VERSION_EXPECTED 7 -> 8
type: bugfix
date: 2026-10-10
status: resolved
phase: "two-ball-skill-stack — R5 (Ball Butler calibration)"
related_plan: two-ball-skill-stack.md
related_entries:
  - 2026-10-10-bb-own-input-scales
  - 2026-07-29-hand-sensor-endpoint-id-contract
files_changed:
  - config/protocol_config.yaml
  - config/generated/protocol_config.h
  - config/generated/protocol_config.py
  - ros_ws/src/jugglebot/jugglebot/protocol_config.py
  - ros_ws/src/jugglebot/Teensy_code_platform/protocol_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/protocol_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/protocol_config.h
  - "config/ODrive config Files/odrive-s1-0.6.11-1_flat_endpoints.json"
  - teensy_link/rpc_args.py
  - logbook/2026-10-10-bb-s1-sdo-endpoints.md
  - logbook/INDEX.md
external_changes:
  - "BallButler branch bb-fw8-sdo-scale-guard: ball_butler_main/protocol_config.h (copied from config/generated/), ball_butler_main/CanInterface.h, ball_butler_main/CanInterface.cpp (the hand input-scale guard), ball_butler_main/FwUpdate.h (FW_VERSION 7 -> 8), logbook/2026-10-10-bb-hand-torque-ff-scale.md, logbook/INDEX.md"
subsystem:
  - config
  - can
tags:
  - safety
---

# Ball Butler's hand S1 gets its own SDO endpoint ids

## Problem

BB FW 7 (`2026-10-10-bb-own-input-scales`) gave Ball Butler its own CAN input
scales but nothing compares them with the drive: the at-arm SDO readback of
node 8's `axis0.config.can.input_torque_scale` was deferred because no
authoritative ODrive S1 0.6.11 endpoint table was in either repo. The only
`input_torque_scale` id here was the Pro's (283; in the S1 table 283 is
`axis0.config.P_bus_soft_max`).

The owner has now supplied the S1 table:
`odrive-s1-0.6.11-1_flat_endpoints.json` (fw_version 0.6.11-1, hw_version
5.2.0, tree crc 52326, 631 endpoints).

## Change

- The table is committed as `config/ODrive config Files/odrive-s1-0.6.11-1_flat_endpoints.json`
  (sha256 `f9907410…2944ef6`, byte-identical to the owner's copy in
  `~/bb_calibration_sessions/hand_jolt_20261010/`). It is the source of every
  id in `endpoints.odrive_s1_0_6_11`, as the Pro's table is for the Pro's block.
- `config/protocol_config.yaml` `endpoints.odrive_s1_0_6_11` (the existing S1
  block; the Pro block is untouched) gains `can_input_torque_scale: 273`,
  `can_input_vel_scale: 272` (uint32 rw), `can_node_id: 262`,
  `fw_version_major/minor/revision: 10/11/12`, `hw_version_major/minor: 6/7`,
  with a comment naming the file and its fw/hw version. Every id in the block
  was checked against the table by a script (all 10 equal).
- **`commutation_mapper_pos_abs` 488 -> 451.** The block's comment said its
  values were "proven in production on BB"; that holds for `get_gpio_states`
  700 (BB's ball-in-hand poll invokes it every session) but not for 488. In the
  S1 0.6.11-1 table 488 is `axis0.task_times.can_heartbeat.length`;
  `axis0.commutation_mapper.pos_abs` is 451 (`axis0.pos_vel_mapper.pos_abs` is
  429). Nothing sends it: BB's only user, `CanInterface::isEncoderSearchComplete`,
  is never called, and no Jugglebot firmware uses the S1 block. Only
  `tests/firmware/test_gpio_poll_xref.py` reads the Python constant, as a
  bare-literal screen. 488 most likely came from an older S1 firmware build
  (BallButler `zTesting/sensored_hand_testing/CanInterface.h`, copied from this
  repo 2026-03-19, hardcodes 488/700); the `pDOE…` ELF in
  `~/.cache/odrivetool/firmware/` has pos_abs at 488 but gpio at 675, so no
  known build has both 488 and 700.
- Regenerated with `config/generate_config.py --no-external`; the header was
  copied into the BallButler FW 8 worktree by hand. The header diff is the S1
  block only (8 added lines, 488 -> 451).
- `teensy_link/rpc_args.py` `BB_FW_VERSION_EXPECTED` 7 -> 8, with a history
  line. Nothing else on the Python/ROS side pins BB's FW version
  (`tests/teensy_link/test_rpc.py`'s `== 7` is a scripted INFO reply value);
  `EXPECTED_BRIDGE_FW_VERSION` stays 28.
- BB FW 8 (BallButler `logbook/2026-10-10-bb-hand-torque-ff-scale.md`): at every
  CLOSED_LOOP request to the hand it SDO-READs (opcode 0, the frame
  `odrive_protocol.h encode_sdo_read` builds) 273, 272, 262, 10, 11, 12, one at
  a time, and zeroes node 8's `set_input_pos` vel_ff/tor_ff while a scale reads
  back different from `bb_hand_tor` / `bb_hand_vel`. USB serial only; no frame
  layout changes.

## Verification

- `generate_config.py --check`: CONFIG FRESH, 14 artifacts; one EXTERNAL DRIFT
  line for `../BallButler/ball_butler_main/protocol_config.h` (the owner's main
  tree, until the BallButler branch merges): expected.
- Tests: see the commit's `Tests:` lines.
- Not verified here (no hardware): that BB's hand drive actually runs S1
  0.6.11-1. FW 8 prints the drive's fw version and node_id on its first check.
