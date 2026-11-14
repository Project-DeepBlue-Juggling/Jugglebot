---
title: "GUI Hardware panel shows the firmware version of the Ball Butler Teensy"
type: feature
date: 2026-11-14
status: complete
phase: "maintainer review"
subsystem: skill-stack/gui
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/gui/js/hardware-versions.js
  - ros_ws/gui/js/geometry-config.js
  - config/hardware_config.yaml
  - config/generated/geometry-config.js
  - tests/ros/test_gui_geometry.py
  - tests/ros/test_teensy_bridge_node_bb.py
  - tools/probes/gui_synthetic_stack.py
  - logbook/INDEX.md
---

## What

The Hardware panel listed the can-bridge Teensy, the Platform Teensy and the ODrives,
but not the Teensy inside Ball Butler. `teensy_bridge_node` now asks it for its
`FW_VERSION` and publishes it as the `bb_fw_version` KeyValue on `link_status`; the
panel has a row for it, in a group of its own.

No firmware changed. Ball Butler already answers the firmware INFO query, which its
source describes as a read with no session and no state change
(`ball_butler_main/FwUpdate.cpp:316-317` in the BallButler repo), and the can-bridge has
relayed `BB_FW_INFO` since FW 21. The flasher tool already read the version this way.

## Why this shape

- The ask rides the existing 1 Hz Ball Butler version tick, so there is no new timer,
  and it is sent only while Ball Butler is heartbeating. It never waits for the CAN
  reply, which is recorded on the receive thread; the only wait is the RPC
  acknowledgement at 0.3 s with one retry, the same bound as its sibling.
- The row renders the number, `N (SKEW — expected v9)` against
  `BB_FW_VERSION_EXPECTED`, or a distinct reason: no Ball Butler heartbeat, asked once,
  not answered, or a bridge older than FW 21. A skew logs a warning and refuses nothing.
- The Teensy in Ball Butler is the same model as the Platform Teensy. Grouped by model,
  two unrelated version numbers would show an ODD badge on a healthy robot, so the row
  carries a `family` field; rows without one group exactly as before.

## Accepted limits

- The version is read once per launch. After Ball Butler is flashed, the panel shows the
  old number until `teensy_bridge_node` restarts.
- If Ball Butler heartbeats but never answers, the node asks once a second for as long
  as it runs.
- On a link where the bridge stops acknowledging, the shared tick can block about 1.2 s
  where it blocked 0.6 s; that is the callback-group weakness filed as BUG-98.

## To see it

`colcon build --packages-select jugglebot`, relaunch, then a hard refresh of the GUI page.

## Verification

- 2026-11-14, `./run_tests.sh --full` on this change together with the `schedule_hand`
  deletion: 9787 passed, 77 skipped, 1 xfailed (parallel, 635 s), 10 passed (serial);
  RESULT PASS.
- 2026-11-14, one audit pass: no blocking finding; the regenerated config artifacts are
  complete (only the two `geometry-config.js` copies read `hardware_models`).
- Not verified on the robot: the row has not been seen against the live Ball Butler.
