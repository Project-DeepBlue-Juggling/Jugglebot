---
title: "BB calibration: seven markers. All seven fit the yaw axis, only co-planar QTM 3–7 set the x/y-plane Z, and QTM 4 anchors the yaw offset"
type: feature
date: 2026-09-27
status: resolved
phase: "Ball Butler — calibration"
files_changed:
  - ros_ws/src/jugglebot/jugglebot/bb_calibration.py
  - ros_ws/src/jugglebot/jugglebot/mocap_interface.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - ros_ws/src/jugglebot/jugglebot/mocap_status.py
  - ros_ws/src/jugglebot/jugglebot/tests/test_bb_calibration.py
  - tests/ros/test_bb_calibration_coplanar.py
  - tests/ros/test_bb_calibration_arc_span.py
  - tests/ros/test_mocap_node.py
  - tests/ros/test_mocap_status.py
  - tests/ros/test_orchestrator_node.py
  - tests/ros/test_teensy_bridge_node_bb.py
subsystem:
  - tracking
tags:
  - calibration
  - mocap
---

# BB calibration with the seven-marker constellation

## What

The owner added two markers to Ball Butler so it stays tracked through the
calibration sweep. QTM now labels the markers `Ball Butler - 1..7`. The new pair
are QTM 1–2. The original five are now QTM 3–7, and they are the ones that lie
on one plane. The new pair sit at a different height.

- `bb_calibration.py` owns three constants: `BB_MARKER_COUNT = 7`,
  `BB_PLANE_MARKER_INDICES = (2..6)` (QTM 3–7) and `BB_YAW_ANCHOR_INDEX = 3`
  (QTM 4, as the owner specified). Indices are zero-based throughout.
- **Axis fit:** `run_calibration` fits the rotation axis from all seven
  markers.
- **Plane Z:** the x/y-plane Z comes from the co-planar markers only. If none of
  them recorded data, calibration refuses and names the cause. It does not fall
  back to the off-plane pair's Z.
- **Yaw offset:** anchored on QTM 4. If that marker is missing, the refusal
  names it.
- **Mocap ingest and readiness:** `mocap_interface` reads labels 1–7.
  `mocap_status` reports "N/7 … incl. yaw-anchor Marker 4". The wire key
  `marker3_visible` is kept so bags and consumers stay readable; a comment
  records that it now reports the anchor.

## Why

Including the off-plane pair in the Z average raises the plane: +15 mm on the
test geometry. BB's reported position is the point where the axis crosses that
plane, so its Z is off by the full shift. Through any axis tilt, the XY moves
as well (>1 mm at 5°). Anchoring the yaw offset on a different physical marker
shifts the offset by the angle between the two markers, and every throw is then
misaimed with no error raised.

## Verification

- `tests/ros/test_bb_calibration_coplanar.py` has 8 new tests. Each is built so
  the old rule (all-marker Z, anchor index 2) would fail it. The fixtures moved
  to a 7-marker layout.
- Test 8 of the standalone `test_bb_calibration.py` failed until its oracle
  averaged Z over the co-planar set only. That failure was expected: the oracle
  encoded the old rule.
- Gate (`./run_tests.sh`, run 2026-09-27): **5430 passed, 8 skipped, plus 3
  serial passed, RESULT: PASS (223 s)**.
- Standalone (`python -m jugglebot.tests.test_bb_calibration`, run 2026-09-27):
  **47/47 passed**.
- Hardware (owner, 2026-09-27): calibration was flawless after the colcon
  build, and aiming was spot-on.
