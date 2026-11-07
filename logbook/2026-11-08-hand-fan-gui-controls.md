---
title: "Hand fan GUI controls: read-only pill in System, On/Off toggle in Platform Control, Auto offered while forced"
type: feature
date: 2026-11-08
status: complete
phase: "hand cooling fan"
subsystem: gui
files_changed:
  - ros_ws/gui/js/hand-fan-panel.js
  - ros_ws/gui/js/main.js
  - ros_ws/gui/js/panels.js
  - ros_ws/gui/js/replay/allowlist.js
  - ros_ws/gui/js/replay/policy.js
  - ros_ws/gui/replay/schema.py
  - ros_ws/gui/index.html
  - ros_ws/gui/css/panels.css
  - tests/ros/test_gui_hand_fan_panel.py
  - tests/ros/js/hand_fan_panel_harness.js
---

# Hand fan GUI controls

## What changed

Follows `2026-11-08-hand-fan-runtime-control`; the owner confirmed the service path on an unpowered Sitting and asked for GUI controls.

- **System panel:** a read-only `Hand Motor Fan` pill, ON, OFF or UNKNOWN. UNKNOWN covers no status yet and a live status older than 3.5 s, so it never freezes on a state that may no longer be true.
- **Platform Control:** an OFF/ON toggle with a mode label. The toggle lights the side the fan is actually on, from `/hand_fan/status`, not the side last clicked. A refused or unanswered call shows REFUSED or NO REPLY for 4 s.
- **Thresholds stay out of the GUI (owner):** the calls send 0, which keeps whatever the config or a service call set.
- **Replay:** `/hand_fan/status` is in the replay allowlist and policy, so a recording shows the recorded state with the controls disabled, and a paused replay does not fall to UNKNOWN. Bags from before FW 9 show UNKNOWN.

## Discussion

**Why an Auto button the owner did not ask for.** A plain On/Off toggle forces the fan, so one click ends the temperature rule until a service call or a Platform Teensy reboot, and left Off the fan will not start however hot the motor gets. The owner chose On/Off plus a small Auto button that appears only while the fan is forced, over a strict toggle and over Off meaning auto. The second would have made a forced-off A/B arm need a service call, and Off might not stop the fan.

## Verification

Gate (`./run_tests.sh`, run 2026-11-08, on this tree before this section was filled in): **9096 passed, 62 skipped** in the parallel phase, **7 passed** in the serial phase, `RESULT: PASS`, `EXIT=0`. The first run failed only `tests/ros/test_gui_clock_contract.py`, because the panel's staleness clock lacked its `// wall-clock:` marker; the run cited here is after that.

After filling this in: `pytest -q tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py` (run 2026-11-08), the two tests that read this file.

Rendered: the real `index.html` in headless Chromium with software WebGL (2026-11-08) shows the pill in System and the toggle row in Platform Control, both in their no-status state, with no uncaught errors. The forced and auto looks were checked on a static copy of the markup with the real stylesheet. Not yet clicked in the live GUI against the robot.
