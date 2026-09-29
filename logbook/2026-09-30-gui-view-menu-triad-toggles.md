---
title: "GUI View menu: the robot's toggle reads Jugglebot, and the Base, Platform, Cone and Ball Butler triads toggle one by one"
type: feature
date: 2026-09-30
status: resolved
phase: "GUI — 3D scene"
subsystem:
  - gui
files_changed:
  - ros_ws/gui/js/stewart-model.js
  - ros_ws/gui/js/mocap-markers.js
  - ros_ws/gui/js/main.js
---

# GUI View menu: Jugglebot label and per-body triad toggles

The owner asked for two changes to the 3D scene's View menu.

- **"Platform" → "Jugglebot".** The group it hides is the whole robot (base,
  six legs, moving platform, hand), so the old label named only a part of it.
  "Jugglebot" is the whole robot in the project glossary (`CONTEXT.md` on
  `mvp-trajectory-bringup`). The mocap info box's "Platform" marker group is
  the Platform rigid body's markers and keeps its name.
- **Triads section.** It holds one toggle each for the Base, Platform, Cone and
  Ball Butler rigid-body triads. `mocap-markers.js` now keys triads by body
  name, not message index. Each listed body (`Base`, `Platform`,
  `Catching_Cone`, `Ball_Butler`, the names after `mocap_interface`'s
  space/hyphen cleanup) gets its own scene-level group in `triadGroups`. Any
  other body's triad stays under the Mocap Markers group, as before.
  **Behaviour change:** the Mocap Markers toggle no longer hides the four
  listed triads; each has its own toggle now. A body missing from a
  `rigid_body_poses` message still has its triad hidden.

No ROS, message or server change: `gui_server.py` serves `ros_ws/gui/`
directly, so a browser reload picks this up and no `colcon build` is needed.

## Verification

- 2026-09-30, headless Chromium `--dump-dom` against the live
  `gui_server.py` on :8081 shows the menu as Grid, Axes, Jugglebot,
  Ball Butler, Mocap Markers, then a Triads section: Base, Platform, Cone,
  Ball Butler.
- 2026-09-30, the same page driven over DevTools (scratchpad script, not
  committed). It fed `updateRigidBodyAxes` five bodies (the four above plus
  an unlisted `Wand`) and clicked the menu checkboxes. Each listed body's
  triad landed in its own group. The Cone checkbox hid and restored only the
  Cone triad. Mocap Markers off left the four triads shown. A message with
  only `Base` hid the other three.
- 2026-09-30, `./run_tests.sh`: parallel phase **5622 passed, 9 skipped
  in 275.54 s**, serial phase **3 passed**, exit 0. No test reads the menu:
  `tests/ros/test_gui_geometry.py` only checks that `stewart-model.js`
  exists, so the checks above are this change's verification.
