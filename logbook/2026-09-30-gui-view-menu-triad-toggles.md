---
title: "GUI View menu: Global Axes, Local Triads (every rigid-body triad, one toggle) and Jugglebot"
type: feature
date: 2026-09-30
status: resolved
phase: "GUI — 3D scene"
subsystem:
  - gui
files_changed:
  - ros_ws/gui/js/viewer.js
  - ros_ws/gui/js/stewart-model.js
  - ros_ws/gui/js/mocap-markers.js
  - ros_ws/gui/js/main.js
---

# GUI View menu: Global Axes, Local Triads, Jugglebot

The 3D scene's View menu now reads, in order: Grid, Global Axes, Local Triads,
Jugglebot, Ball Butler, Mocap Markers.

- **"Axes" → "Global Axes"**: the world-frame axes at the origin, as distinct
  from the rigid-body triads.
- **"Platform" → "Jugglebot"**: the group it hides is the whole robot (base,
  six legs, moving platform, hand). "Jugglebot" is the whole robot in the
  project glossary (`CONTEXT.md` on `mvp-trajectory-bringup`). The mocap
  info box's "Platform" marker group is the Platform rigid body's markers and
  keeps its name.
- **Local Triads**: every `rigid_body_poses` triad (Base, Platform, Cone,
  Ball Butler, any other body) sits in one scene group,
  `initRigidBodyTriads()` in `mocap-markers.js`. **Behaviour change:** the
  triads used to be children of the Mocap Markers group, so they were hidden
  with the markers. They now toggle apart from the markers.

The menu lists scene groups in registration order, so `main.js` calls
`initRigidBodyTriads()` straight after `initViewer()`.

The first cut (`0cb47745`) gave each of the four named bodies its own toggle.
The owner reviewed it the same day and asked for one group instead, so the
triad pool is back to the original index-keyed form with a new parent.

No ROS, message or server change: `gui_server.py` serves `ros_ws/gui/`
directly, so a browser reload picks this up and no `colcon build` is needed.

## Verification

- 2026-09-30, headless Chromium driven over DevTools against the live
  `gui_server.py` on :8081 (scratchpad script, not committed). The menu read
  Grid, Global Axes, Local Triads, Jugglebot, Ball Butler, Mocap Markers. Fed
  four rigid bodies through `updateRigidBodyAxes`, it showed 4 triads, 0 with
  Local Triads off, 4 with it back on, and 4 with Mocap Markers off.
- 2026-09-30, `./run_tests.sh`: see the commit message for the counts. No
  test reads the menu: `tests/ros/test_gui_geometry.py` only checks that
  `stewart-model.js` and `viewer.js` exist, so the check above is this
  change's verification.
