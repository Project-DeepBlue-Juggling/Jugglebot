---
title: "A forced Hand fan returns to AUTO when the launch shuts down, so a fan switched ON by hand stops once the motor is below the off threshold"
type: feature
date: 2026-11-08
status: complete
phase: "hand cooling fan"
subsystem: ros
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/ros/test_teensy_bridge_node_shutdown_hand_fan.py
  - tests/ros/test_bridge_launch_sigterm.py
---

# A forced Hand fan returns to AUTO on shutdown

## What changed

The owner asked that a fan switched ON by hand not run past the end of a Sitting for longer than needed. Forced modes live in the Platform Teensy's RAM and outlast the launch, so before this a fan left ON ran until the board next rebooted.

`teensy_bridge_node.on_shutdown()` now calls `_shutdown_hand_fan_auto()` right after the disarm and before the stow. If the newest status frame says the fan is forced, it sends `HAND_FAN_SET` auto with the board's current thresholds, then logs the result. In AUTO, or with no status ever seen, it sends nothing.

- **Order and budget:** about one RPC on a healthy link. The worst case with no reply is `_SHUTDOWN_HAND_FAN_BUDGET_S` (0.3 s x 2 = 0.6 s), which `test_bridge_launch_sigterm.py` now adds to the stow budget. The total of 7.8 s still fits the launch's 8 s SIGTERM window with its 1 s margin. Placing it before the stow means a stow that runs long cannot starve it, and the disarm settle after it is unchanged.
- **Not covered:** a node killed hard, a crash, or a bridge powered off first. The fan then stays forced until the Platform Teensy reboots or `hand_fan/set` is called, and the warning says so.

The MOSFET is an IRLZ44N (owner, 2026-11-08): a logic-level part, fine at 3.3 V gate drive for this load.

## Verification

Gate (`./run_tests.sh`, run 2026-11-08, on this tree before this section was filled in): **9168 passed, 53 skipped** in the parallel phase, **7 passed** in the serial phase, `RESULT: PASS`, `EXIT=0`. After filling this in: `pytest -q tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py` (run 2026-11-08), the two tests that read this file.

`pytest -q tests/ros/test_teensy_bridge_node_shutdown_hand_fan.py tests/ros/test_bridge_launch_sigterm.py tests/ros/test_teensy_bridge_node_shutdown_stow.py tests/ros/test_teensy_bridge_node_relay.py` (2026-11-08): 36 passed. Not yet seen on the robot: the next Ctrl-C with the fan forced should log `shutdown: Hand fan on -> auto`.
