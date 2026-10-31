---
title: "GUI: default 3D camera view set to the owner-framed view; Scene menu gains Default and Copy view (JB-88)"
type: feature
date: 2026-10-31
status: resolved
phase: ""
files_changed:
  - ros_ws/gui/js/viewer.js
  - ros_ws/gui/js/camera-presets.js
  - map/bug-change-default-camera-position-in-3d-scene.md
  - docs/project-map.md
subsystem:
  - gui
tags:
  - ux
---

# GUI: default 3D camera view set to the owner-framed view; Scene menu gains Default and Copy view (JB-88)

## Summary

JB-88 asked for a different startup camera in the GUI's 3D scene, and noted that a
readout tool would be needed: saved views live in the browser's localStorage, which
the Jetson cannot read.

- `viewer.js`: the startup view, previously literals inside `initViewer()`, is one
  exported frozen `DEFAULT_CAMERA` constant (Three.js scene coords, metres). Its value is
  the view the owner framed and read out: position (-0.341, 1.584, -2.589), target
  (-0.231, 0.763, 0.37). The now-unused `INITIAL_HEIGHT_MM` import went with it.
- `camera-presets.js`: Scene menu → Camera gains **Default** (snap back to
  `DEFAULT_CAMERA`) and **Copy view**, which prints the current camera as a paste-ready
  `position: {...}, target: {...}` literal. The GUI is served over plain http on the LAN
  (not a secure context, so `navigator.clipboard` is absent); it falls back to a
  `window.prompt` with the text selected, and always logs to the console. To change the
  default again: frame the view, Copy view, paste into `DEFAULT_CAMERA`.

## Verification

- 2026-10-31, `node --check` on both modules (copied to `.mjs`): OK.
- 2026-10-31, headless Chromium `--dump-dom http://localhost:8081/` against the live GUI:
  the Default / Top / Front / Side / Iso row, Copy view and the three slots render.
- 2026-10-31, owner used Copy view on the live GUI to produce the value above.
- Suite: see the commit message.
