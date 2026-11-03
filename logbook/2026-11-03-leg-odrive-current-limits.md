---
title: "Leg ODrive current limits to hard 30 A / soft 20 A: the fleet tool gains a no-erase --set mode, the leg snapshot follows"
type: feature
date: 2026-11-03
status: complete
phase: ""
files_changed:
  - tools/odrive_fleet_reflash.py
  - tests/motion/test_odrive_fleet_reflash.py
  - config/ODrive config Files/odrive_pro_leg_config.json
  - logbook/2026-11-03-leg-odrive-current-limits.md
  - logbook/INDEX.md
subsystem: tools
---

# Leg ODrive current limits: hard 30 A, soft 20 A

**Owner decision (2026-11-03):** leg ODrive `current_hard_max` 30 A and `current_soft_max` 20 A. Each leg's NVM held soft 10 / hard 20 (the backup JSON). The operating value the software commands lives in `config/hardware_config.yaml` `leg_curr_limit_a` and is changed by session jugglebot-b1 together with its next box sweep (not here: that file is gated and re-hashes the box).

**Why a new mode.** The fleet tool could only erase and restore a whole config, which also resets each drive's commutation offset and forces a calibration. `--set KEY=VALUE` (repeatable) writes only the named properties to the LEG drives (the hand at node 6 is never written; `--only` still narrows). Order: discover and record the map, back each leg up, then validate every drive before the first write (axis IDLE; key is an rw endpoint in `config/ODrive config Files/odrive-pro-4.4.58-0.6.11-1_flat_endpoints.json`; value parses to the endpoint type; soft <= hard on the would-be result, the drive's current value standing in for a key not set; `axis0.config.can.node_id` refused). Then write, `save_configuration()`, reboot, reconnect, read every key back from NVM; a mismatch fails the run (exit 1). No erase, restore, calibration or motion. `--dry-run` prints before -> after per drive and writes nothing. Mutually exclusive with `--config` and the erase/calibration flags. The reboot drops each leg's encoder zero: power-cycle or reboot the can-bridge before the next launch (JB-146 hazard); the tool prints this reminder.

**Snapshot.** `odrive_pro_leg_config.json`: `axis0.config.motor.current_soft_max` 10.0 -> 20.0, `current_hard_max` 20.0 -> 30.0. The inverter ceiling (`config.inverter0.current_hard_max` 150 A) stays above. Readers (`git grep -n odrive_pro_leg_config`): `test_leg_torque_ff.py` and `test_kt_lib.py` pin only `torque_constant`; nothing pins the old limits (a comment in `test_leg_torque_ff.py` and `tests/hardware/kt_bench_test.py` still says 10 A as history).

**Verification.** 2026-11-03, `python -m pytest -q -p no:cacheprovider tests/motion/test_odrive_fleet_reflash.py tests/motion/test_leg_torque_ff.py tests/motion/test_kt_lib.py` (the tool's own tests plus the files from `git grep -n "odrive_fleet_reflash\|odrive_pro_leg_config" tests`): **262 passed in 2.13 s**. The new tests drive `main()` with fake drives: legs-only write/save/reboot/verify with the hand untouched, non-IDLE, unknown key, soft > hard, dry-run, verify mismatch, `--config` exclusion. Not run against hardware.
