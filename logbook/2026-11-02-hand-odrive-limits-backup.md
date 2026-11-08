---
title: "Hand ODrive config backup matches the drive: current soft 70 / hard 80 A, motor thermistor 60 / 70 °C; the code limit stays 60 A"
type: refactor
date: 2026-11-02
status: complete
phase: "Sitting 2026-11-02 follow-up"
subsystem: hand
related_entries:
  - 2026-11-02-hand-release-step.md
files_changed:
  - config/ODrive config Files/odrive_pro_hand_config.json
  - config/hardware_config.yaml
  - ros_ws/docs/hand_throw_envelope.md
  - logbook/INDEX.md
---

# Hand ODrive limits: the backup JSON follows the drive

**What changed.** The owner set the Hand ODrive (in the ODrive GUI, 2026-11-02)
to `current_soft_max` 70 A / `current_hard_max` 80 A and motor thermistor
`temp_limit_lower` 60 °C / `temp_limit_upper` 70 °C.
- `odrive_pro_hand_config.json` was soft 50 / hard 70 A and thermistor 70 / 80
  °C; it now matches the drive.
- The `hand_curr_limit_a` comment in `hardware_config.yaml` and the envelope
  doc's drive-limit sentence are updated to match.

**What did not change.** `hand_curr_limit_a` stays **60 A**.
- The owner, earlier the same day: "Let's stick with a current limit of 60 A in
  our code for now until we've tested this thoroughly."
- Configure pushes it to the drive's soft max at every launch, so a launch still
  runs at 60 A; that is what capped the BUG-57 brake at -60.4 A.
- The YAML change is comment-only: `hardware_config.py` is byte-identical, so
  the box gate_hash stays 1749cca8bc23 and no re-sweep is needed.

**The GUI-save trap.** Saving these settings in the ODrive GUI rebooted the
Hand ODrive and erased its homed zero (BUG-56). Re-home after any GUI save.

**Verification.**
- Readers of the JSON (2026-11-02): `pytest -q
  tests/motion/test_odrive_fleet_reflash.py
  tests/motion/test_hand_ball_detect_config.py
  tests/firmware/test_hand_torque_ff_twin.py tests/motion/test_throw_envelope.py
  tests/sim/test_docs_links.py` gave 111 passed.
- Gate (2026-11-02, `./run_tests.sh`): 7991 passed, 56 skipped (parallel 424 s)
  + 5 passed (serial), RESULT PASS.
