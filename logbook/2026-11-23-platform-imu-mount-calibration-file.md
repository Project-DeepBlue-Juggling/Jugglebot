---
title: "Platform IMU mount calibration file (config/platform_imu_mount.json) and its use: loader, Platform-frame GUI charts by default, frame named on every label"
type: feature
date: 2026-11-23
status: resolved
phase: "IDEA-20 P5"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - config/platform_imu_mount.json
  - teensy_link/platform_imu_mount.py
  - teensy_link/platform_imu_capture.py
  - ros_ws/gui/imu_api.py
  - ros_ws/gui/gui_server.py
  - ros_ws/gui/index.html
  - ros_ws/gui/js/imu-charts-core.js
  - ros_ws/gui/js/imu-charts.js
  - ros_ws/docs/platform_imu_protocol.md
  - plans/active/platform-imu.md
  - tests/teensy_link/test_platform_imu_mount.py
  - tests/ros/test_gui_imu_api.py
  - tests/ros/test_gui_imu_charts.py
  - tests/ros/js/imu_charts_harness.js
subsystem: [platform-imu, gui, teensy-link]
tags: [imu, calibration]
---

# Platform IMU mount calibration file

**What.** The mount rotation fitted at the Sitting of 2026-11-22 (`2026-11-22-sitting-debrief`, fit write-up
`temp/debrief/2026-11-22/mountfit.md`) is now a committed file, `config/platform_imu_mount.json`: schema version,
R_PI (IMU frame to Platform frame, `v_platform = R_PI @ v_imu`), the CAD nominal and the deviation from it (3.10
deg, (-3.03, -0.13, +0.65) deg about Platform x, y, z), the gyro bias (single-Sitting, 24.0 C, not applied), the CAD
lever arm marked "CAD, not measured", provenance (captures 572D and 572E, bag, method), accuracy (tilt about 0.4 deg,
yaw +/-0.3 deg) and what Level meant (the Jog Home pose; the held Active pose is 0.6 deg off, BUG-130).

**Where and why JSON.** The closest precedent for a measured, committed, human-readable artefact is
`config/tilt_calibration.yaml` (schema-versioned, all-or-nothing validation, absent is soft, invalid is loud). The
file is JSON, not YAML, because `ros_ws/gui/imu_api.py` is stdlib only and the GUI must read it. It is not routed
through `config/generate_config.py` (nothing generated is derived from it).

**Use.** `teensy_link/platform_imu_mount.py` (stdlib only): `load_mount` returns None for an absent file and raises
`MountCalError` for a malformed one (orthonormal within 1e-6, det +1, finite, schema 1); `rotate_xyz` rotates flat
arrays. `platform_imu_capture.to_platform_frame` rotates (n, 3) numpy arrays. Rotation only: no gravity subtraction,
no gyro bias, no lever arm. Raw `.npz` files are untouched. The GUI data endpoint serves the Platform frame by
default when the file is present (`?frame=imu` raw, `?frame=platform` 409 without a file, a malformed file is a 500
on `/data` and `cal_error` in the list); list rows carry `frame` and `cal_date`; the browser requests exactly the
frame it labels; series read `x (Platform)` or `x (IMU)`, the chart-head note names the frame and calibration date,
and the CSV columns are `plat_*` or `imu_*`. imu_api loads the mount module by file path, so it still imports no
numpy and no `teensy_link` package code.

**Endpoint timing** (capture 572E, 32000 samples, this Jetson, 2026-11-23, in-process `imu_api.handle`): before 16.6
ms (mean of 3); after, raw `?frame=imu` first request 22 ms; Platform frame first request 70 ms (two pure-Python
3x3 rotations, about 26 ms each); repeats 0.6 ms from a 4-entry blob cache keyed on the capture and calibration file
signatures. The browser also caches per capture, so the 70 ms is paid once per capture view.

**Real-data check** (capture 572E, Platform-frame mean accel in g, by the debrief's windows): Level (Home hold, 5963
samples) (-0.00004, +0.00011, +0.99997); Rx +5 (+0.00013, +0.0872, 0.9958); Ry +5 (-0.0873, -0.00014, 0.9958);
pre-Tilt Active (-0.0104, -0.0008, 0.9998), the 0.6 deg off-Level of BUG-130 showing as -0.010 g on x. Signs: Level
reads +1 g up; Rx +5 gives +y, Ry +5 gives -x, as the debrief found.

**Not done / unverified.** The lever arm is the CAD value; the accelerometer zero-g offset is unmeasured; the gyro
bias is not corrected anywhere; the chart code was exercised under node with fakes, not in a browser; P5 still needs
the CRC-checked Attempt capture.

## Verification

- 2026-11-23, `flock /tmp/jugglebot-tests.lock pytest tests/teensy_link tests/ros -k "imu or gui" -q`: **982 passed, 1 skipped, 3930 deselected in 491.18 s** (includes the new `tests/teensy_link/test_platform_imu_mount.py`, the frame cases in `tests/ros/test_gui_imu_api.py` and `test_gui_imu_charts.py`). Doc tests (`pytest tests/sim/test_mapkit*.py tests/sim/test_plans_index.py tests/sim/test_logbook_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_no_legacy_item_ids.py -q`, 2026-11-23): **325 passed in 26.96 s**.
