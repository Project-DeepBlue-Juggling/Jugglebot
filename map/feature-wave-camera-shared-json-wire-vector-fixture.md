---
id: JB-54
kind: feature
title: 'Wave camera: a shared JSON wire-vector fixture for the adapter firmware and the node'
status: inbox
created: '2026-10-25'
updated: '2026-10-25T09:00:00Z'
owners: []
priority: low
blocked_by: []
links:
- plans/archived/wave-camera-can3-integration.md
- ros_ws/docs/wave_camera_can_adapter.md
- tests/ros/test_wave_camera.py
parent: wave-camera-can3
filed_by: claude
---

The CAN adapter design s7 calls for one JSON fixture of frame vectors that the ESP32 firmware tests and the node tests both consume (the clapboard precedent). Not written; the vectors live in `tests/ros/test_wave_camera.py` until it is.
