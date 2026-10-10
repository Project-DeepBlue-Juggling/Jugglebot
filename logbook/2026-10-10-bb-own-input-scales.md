---
title: Ball Butler gets its own CAN input scales (bb_hand_* 100/100, bb_pitch_* 1000/1000) — BB FW 6 had borrowed Jugglebot's hand_tor (1000) for BB's hand S1 (input_torque_scale 100), a 10x torque feedforward; BB_FW_VERSION_EXPECTED 6 -> 7
type: bugfix
date: 2026-10-10
status: in-progress
phase: "two-ball-skill-stack — R5 (Ball Butler calibration)"
related_plan: two-ball-skill-stack.md
files_changed:
  - config/protocol_config.yaml
  - config/generated/protocol_config.h
  - config/generated/protocol_config.py
  - ros_ws/src/jugglebot/jugglebot/protocol_config.py
  - ros_ws/src/jugglebot/Teensy_code_platform/protocol_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/protocol_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/protocol_config.h
  - teensy_link/rpc_args.py
  - logbook/2026-10-10-bb-own-input-scales.md
  - logbook/INDEX.md
subsystem:
  - config
  - can
tags:
  - safety
---

# Ball Butler gets its own CAN input scales

## Problem

BB FW 6 (BallButler `f296441`) regenerated `ball_butler_main/protocol_config.h`
from this repo. Its `InputScale::hand_tor` is 1000, Jugglebot's hand ODrive Pro
value since FW 22 (`75030f73`, with `odrive_pro_hand_config.json`); FW 5's copy
had 100. BB's `CanInterface` scaled set_input_pos `vel_ff` / `tor_ff` for EVERY
node by `InputScale::hand_vel` / `hand_tor`, and BB's hand drive (ODrive S1,
node 8) is configured `input_torque_scale = 100` (`odrive_s1_bb_hand_config.json`;
the owner read 100 off node 8 with odrivetool). Every BB hand torque
feedforward has been sent at 10x since FW 6: pre-throw kicks, spinouts,
215 mm strokes, throws 7 % slow. Analysis:
`~/bb_calibration_sessions/hand_jolt_20261010/REPORT.md`; BallButler logbook
`2026-10-10-bb-hand-torque-ff-scale.md`.

## Root Cause

One set of keys served two different drives. `hand_*` documents Jugglebot's
hand Pro; BB's firmware reused it for its own S1, so a change made correctly
for one drive silently changed the other. Nothing on either side compared the
borrowed scale with BB's drive.

## Change

Owner's principle: each axis with unique limits has its own set of keys.

- `config/protocol_config.yaml` `encoding.input_scales`: new
  `bb_hand_vel: 100.0`, `bb_hand_tor: 100.0` (BB hand, node 8, ODrive S1;
  matches `odrive_s1_bb_hand_config.json`) and `bb_pitch_vel: 1000.0`,
  `bb_pitch_tor: 1000.0` (BB pitch, node 7, ODrive Micro; matches BallButler's
  `bb_pitch_odrive_micro_config.json`). Jugglebot's `hand_*` / `leg_*` are
  unchanged. Regenerated with `config/generate_config.py --no-external` (the
  default write path would have written into the owner's `../BallButler`
  main tree from this worktree); the header was copied into the BallButler
  FW 7 worktree by hand.
- BB axis audit (`ball_butler_main/`): the only scaled feedforward is
  `CanInterface::sendInputPos`. Hand (node 8): streamed vel/tor FF from
  `HandTrajectoryStreamer`, gets `bb_hand_*`. Pitch (node 7): `PitchAxis`
  sends `vel_ff = tor_ff = 0` (TRAP_TRAJ), so its scale is wire-inert today;
  it gets its own keys anyway so it can never inherit the hand's. Caveat: the
  pitch config file is a 2026-03 snapshot whose `node_id` reads 0 (live drive
  is 7), so 1000/1000 is not confirmed on the live drive. Yaw is a brushed DC
  motor on an H-bridge (PWM), not an ODrive: no keys. `sendInputVel` (homing)
  sends floats, not scaled ints: no keys needed.
- `teensy_link/rpc_args.py`: `BB_FW_VERSION_EXPECTED` 6 -> 7.
  `EXPECTED_BRIDGE_FW_VERSION` stays 28 (the bridge does not change).
- Host side audited: no Python or can-bridge code decodes or encodes BB's
  set_input_pos with a scale (`INPUT_SCALE_HAND_*` consumers are all
  Jugglebot's hand, axis 6). Nothing else to fix.

Not done: an at-arm SDO readback of the S1's `axis0.config.can.input_torque_scale`.
No authoritative local source gives the S1 0.6.11 endpoint id (odrivetool's
cached ELFs embed endpoint trees, but none matches the S1 0.6.11 ids the repo
has proven, `get_gpio_states` 700 / `commutation_mapper.pos_abs` 488).
Follow-up in the BallButler entry.

## Verification

See the commit's `Tests:` lines. Hardware verification is owed: flash BB FW 7,
confirm the fw-update tool's `FW version: 6 -> 7` receipt, and a throwing
sitting with normal strokes and launch speed.

external_changes: BallButler branch `bb-fw7-own-hand-scales` —
`ball_butler_main/protocol_config.h` (copied, + `InputScale::bb_*`),
`ball_butler_main/CanInterface.{h,cpp}` (per-node scales), `ball_butler_main/FwUpdate.h`
(FW_VERSION 7), `logbook/2026-10-10-bb-hand-torque-ff-scale.md`, `logbook/INDEX.md`.
