---
title: "GUI: Ball Butler Yaw / Pitch readouts become manual-aim inputs while the orchestrator is IDLE"
type: feature
date: 2026-09-23
status: resolved
phase: "GUI — Ball Butler panel"
files_changed:
  - ros_ws/src/jugglebot_interfaces/srv/BallButlerAim.srv
  - ros_ws/src/jugglebot_interfaces/CMakeLists.txt
  - ros_ws/src/jugglebot/jugglebot/ball_butler_node.py
  - ros_ws/gui/js/bb-aim.js
  - ros_ws/gui/js/panels.js
  - ros_ws/gui/js/main.js
  - ros_ws/gui/css/panels.css
  - config/generate_config.py
  - config/generated/geometry-config.js
  - ros_ws/gui/js/geometry-config.js
  - tests/ros/conftest.py
  - tests/ros/test_ball_butler_node.py
  - ros_ws/docs/choreography.md
---

# GUI: manual Ball Butler aim

## What

When the orchestrator is IDLE and BB is connected and IDLE or TRACKING, clicking
the BB panel's Yaw or Pitch readout opens an input (Enter sends, Esc or clicking
away cancels). Yaw is in BB's own frame (as the heartbeat reports it), within
the yaw soft limits 0–120°. Pitch is degrees from horizontal, 12–90°.
Out-of-range input is **refused, never clamped**, and nothing is sent. The axis
you didn't edit gets the held target if there is one, otherwise its measured
position.

The path is a new `bb/aim` service (`BallButlerAim.srv`) on `ball_butler_node`.
It forwards a **speed-0 `bb/throw` goal**, which the BB firmware already routes
to `requestTracking` (aim only, no ball leaves). There's **no firmware and no
can-bridge change**, and PROTOCOL_VERSION is untouched. The node re-checks the
whole gate on every call (orchestrator IDLE, BB connected and IDLE/TRACKING,
both angles in range, all finite), so the gate doesn't live only in the
browser. Aim results use their own done-callback, so they never publish on
`bb/throw_outcome`: its consumers read that topic as a Throw's terminal outcome.

## Why the hold is a GUI-renewed lease

BB drops out of TRACKING 5 s after its last aim (`idle_no_cmd_timeout_ms`), and
entering IDLE returns pitch to its rest angle. A one-shot aim would therefore
fall back after 5 s. The owner chose (2026-09-23) a hold that lasts until
released. The GUI renews it by calling `bb/aim` again every 1 s, and **that
renewal is the lease**. If the tab closes or rosbridge drops, the calls stop
and BB returns pitch on its own, so no pose outlives the operator and the node
holds no timer or hold state. The hold ends on Release, on ACTIVATE or any
other exit from IDLE, on BB leaving IDLE/TRACKING or disconnecting, and on any
renewal the node refuses. A firmware change to remove the timeout was ruled
out because it needs a BB flash and fails the wrong way (the pose stays put
with nobody watching).

The yaw and pitch ranges are generated into `geometry-config.js` from
`hardware_config.yaml` (`ball_butler_yaw.lim_*_deg`,
`ball_butler_pitch.deg_*`). The frame was traced in firmware: the heartbeat's
`yaw_deg` and the command's yaw both land on the same yaw-axis degrees, and
0–120° is inside both wire ranges.

## Deploy

Needs the two-package `colcon build --packages-select jugglebot_interfaces
jugglebot` (new `.srv`) and a relaunch. The build was done on 2026-09-23.

## Verification

- (2026-09-23, `pytest tests/ros/test_ball_butler_node.py -q -k TestManualAim`) **23 passed**. Covers the orchestrator and BB state gates, out-of-range, NaN and inf refusal, inclusive edges, and no `bb/throw_outcome` publish.
- (2026-09-23, `colcon build --packages-select jugglebot_interfaces jugglebot`) **2 packages finished**. `ros2 interface show jugglebot_interfaces/srv/BallButlerAim` shows the fields.
- (2026-09-23, live `ros2 run jugglebot ball_butler_node` + `ros2 service call /bb/aim …`, no bridge) **refused with "orchestrator IDLE (it is unknown)"**. After publishing `IDLE` on `/orchestrator_state`, it was **refused with "BB IDLE or TRACKING (it is disconnected)"**. This shows the real-rclpy subscription and gate wiring.
- (2026-09-23, uncommitted Node probe with a stub DOM and stub `callService` over the real `bb-aim.js`) **17/17 lifecycle checks pass**. The checks cover:
  - the gate
  - refusal in place for out-of-range input
  - Esc restoring the readout
  - the untouched axis getting the held target, otherwise its measured position
  - 1 s renewal
  - Release
  - ACTIVATE, BB THROWING, disconnect, and a node refusal each ending the hold
  - an open input closing when the gate drops
- The full gate triple is in the commit message.
- (2026-09-24, owner, on the robot after a relaunch of the `skill-stack` build plus a GUI hard-refresh) **Works as specified.** The fields are editable in IDLE and the owner reported "it's working quite well". The feature first landed on `mvp-trajectory-bringup`, which is not the live branch, so it was cherry-picked onto `skill-stack` before this test.
