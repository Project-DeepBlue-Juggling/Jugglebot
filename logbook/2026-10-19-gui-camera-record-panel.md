---
title: "GUI record panel for the Wave camera adapter: REC/STOP, a pulsing dot, interpolated mm:ss and a one-line status, beside the collapsed minimap"
type: feature
date: 2026-10-19
status: in-progress
phase: "wave-camera-can3 - GUI (CAN adapter design s8)"
related_plan: ""
files_changed:
  - ros_ws/gui/js/camera-panel.js
  - ros_ws/gui/css/camera-panel.css
  - ros_ws/gui/index.html
  - ros_ws/gui/js/main.js
  - ros_ws/gui/js/replay/policy.js
  - ros_ws/gui/js/replay/allowlist.js
  - ros_ws/gui/replay/schema.py
  - tests/ros/test_gui_record_panel.py
  - tests/ros/js/record_panel_harness.js
subsystem:
  - gui
tags:
  - ux
  - testing
---

# GUI record panel for the Wave camera adapter

## Summary

The web GUI gains a small overlay that starts and stops the Wave camera through
the CAN adapter (`camera/record_start` / `camera/record_stop`, `std_srvs/Trigger`)
and shows what the adapter reports on `camera/heartbeat`
(`jugglebot_interfaces/msg/WaveCameraHeartbeat`, 10 Hz). Status is
`in-progress`: the ROS side (message, bridge node services) lands in the same
branch from a parallel unit, and nothing has been browser-tested against a live
heartbeat.

## Motivation

The owner wants to start a camera take from the GUI and see that it is running,
for how long, and why it cannot start (camera off, adapter disconnected, console
owns the camera, a latched fault). Design: s8 of
[`ros_ws/docs/wave_camera_can_adapter.md`](../ros_ws/docs/wave_camera_can_adapter.md)
(the Wave Camera CAN adapter design), plus the owner's panel decisions of 2026-10-19.

## Design

- **Position.** `#camera-panel` sits on the collapsed minimap's bottom line
  (`bottom: calc(var(--command-strip-h, 56px) + var(--gap) * 2)`), to its right
  (`left: calc(var(--gap) + 210px + var(--gap))`), at the minimap's z-index 11,
  with the same translucent overlay styling. When `#viewer-pane.minimap-expanded`
  is set it moves to `left: calc(var(--minimap-width) + var(--gap))`, the
  `#connection-status` rule. `left`/`bottom` never transition (the minimap snap
  rule). A `max-width` keeps it clear of the replay trail-length overlay at the
  bottom right; the status text ellipsises first.
- **Enable rule.** The button is live only when ROS is connected, the heartbeat
  is fresh (at most 1 s old), the adapter is connected, the camera is on, the
  console does not own it, the state is IDLE or RECORDING, replay is not fenced,
  and no call is in flight. Warnings (faults, unsynced clock, console owns)
  show an amber glyph with a tooltip; faults and the unsynced clock do not
  disable the button.
- **Elapsed.** The last heartbeat's `elapsed_ms` plus monotonic time since it
  arrived, clamped to +1 s, so a stale heartbeat freezes the readout; repainted
  at 4 Hz, snapped on every heartbeat. Monotonic (`performance.now()`, marked
  `// wall-clock:` for the clock contract) rather than `clock.now()`, so an NTP
  step cannot jump it. Consequence: a paused replay greys the panel after 1 s,
  harmless because replay fences the button anyway.
- **Status line.** Precedence: no heartbeat / ROS disconnected, Adapter
  disconnected, Camera off, `Fault: <names>` (state FAULT), Recording,
  Starting/Stopping/Booting, `Idle · N clips`. A failed call replaces it for 2 s.
- **Deviations from the CAN adapter design s8, by owner decision.** The panel does not
  subscribe to `camera/record_event`, so only `camera/heartbeat` joins
  `GUI_SUBSCRIBED_TOPICS`. The enable rule adds the IDLE/RECORDING state gate.
  The status wording follows the owner's list. The reducer reads the CAN adapter design s7's
  `fault_bits` and falls back to `faults`.
- **Replay contract.** A new live subscription must also be converted for
  replay: `/camera/heartbeat` was added to `replay/schema.py` SUBSCRIBED, the
  `js/replay/allowlist.js` mirror and `js/replay/policy.js` (state-render,
  throttle 0). No "not in this recording" region was added (`absent.js`
  unchanged).

## Implementation

`js/camera-panel.js` has a DOM-free core (`reduceHeartbeat`, `canPress`,
`interpolatedElapsedMs`, `statusText`, `warnings`, `faultNames`,
`cameraPanelView`) and the glue (`initCameraPanel`, `cameraPanelOnHeartbeat`,
`setCameraPanelDisconnected`). `main.js` imports it, calls `initCameraPanel()`
after `initStateMinimap()`, subscribes `camera/heartbeat` in `subscribeAll()`,
and blanks the panel in `blankDisconnectedState()`. `mm:ss` reuses
`fmtDuration` from `js/replay/ui/format.js`.

## Verification

- `python -m pytest tests/ros/test_gui_record_panel.py -q` (run 2026-10-19,
  Windows, Python 3.11, node v22.14.0): **13 passed**.
- `python -m pytest tests/ros -q -k "gui or replay or rosbridge"` (run
  2026-10-19, Windows, Python 3.11 with hypothesis/mcap/rosbags from a scratch
  path appended to `sys.path`, `PYTHONUTF8=1`): **7 failed, 624 passed, 2
  skipped, 18 errors**. Every failure and error is environmental on this desktop
  and outside the GUI change: `sensor_msgs` absent (bridge, tracker and mocap
  collection errors), `mcap` absent in the converter's subprocess
  (`test_replay_convert` CLI, `test_replay_overview`), the CRLF checkout of the
  vendored MCAP bundle (`test_gui_vendor_pin`), and two `teensy_bridge_node`
  tests. All GUI and replay contract tests touched by this change pass,
  including `test_rosbridge_websocket_lean`, `test_replay_allowlist`,
  `test_gui_replay_policy`, `test_gui_clock_contract` and
  `test_replay_mcap_oracle`.
- The contract files on their own, `PYTHONUTF8=1 python -m pytest
  tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py
  tests/ros/test_gui_record_panel.py tests/ros/test_gui_clock_contract.py
  tests/ros/test_rosbridge_websocket_lean.py tests/ros/test_replay_allowlist.py
  tests/ros/test_gui_replay_policy.py tests/ros/test_replay_mcap_oracle.py
  tests/ros/test_gui_geometry.py -q` (run 2026-10-19): **236 passed**. Without
  `PYTHONUTF8=1` on Windows, the lean and allowlist tests fail on a cp1252
  decode of the pre-existing box-drawing and arrow characters in `panels.js` and
  `udp-traffic.js`. That failure is not from this change.
- `python ros_ws/gui/gui_server.py --port 8089`, then curl: `/` serves the
  `css/camera-panel.css` link and the `#camera-panel` markup, and both new files
  return 200.
- Not yet done: a browser check against a live heartbeat (CAN adapter design s10 row 8).
