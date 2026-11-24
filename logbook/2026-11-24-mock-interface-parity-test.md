---
title: "BUG-116 slice 1: a test that compares the mock ROS interface classes with the real .msg/.srv/.action definitions (no drift found)"
type: feature
date: 2026-11-24
status: complete
phase: "BUG-116 slice 1"
related_plan: ""
related_issues: []
files_changed:
  - tests/ros/test_mock_interface_parity.py
subsystem: [tests-tooling]
---

## Summary

`tests/ros/test_mock_interface_parity.py` parses the real definitions in `ros_ws/src/jugglebot_interfaces/{msg,srv,action}` as text (no built ROS needed) and compares them, per message and per Request/Response/Goal/Result/Feedback section, with the mock classes `conftest.py` registers as `jugglebot_interfaces.{msg,srv,action}`: field names both ways, and the type class (bool/int/float/str/list) wherever the mock declares one (dataclass annotation, or the default's Python type for the plain-class mocks; `object`/`None` defaults compare by name only; an int default on a float field is accepted). Constants (`KIND_*`) are not compared.

Why: the mocks accept any attribute, so a node test can pass on a field the real message lacks. This is slice 1 of BUG-116; the strict `__slots__`/`__setattr__` change (slice 2) is not done and `conftest.py` is untouched.

Result: no drift today across the 40 mocked interfaces (24 msg, 13 srv, 3 action), so the allow-list `KNOWN_DRIFT` is empty and the test pins that. A second allow-list pins the three real interfaces with no mock (`LevelPlatform`, `Target`, `GetRobotGeometry`) so a new interface must get a mock or be listed. geometry_msgs / std_msgs / std_srvs are out of scope (no source text in the repo).

## Verification

- 2026-11-24, `python -m pytest tests/ros/test_mock_interface_parity.py tests/ros/test_ros_mock_hygiene.py -q`: 50 passed.
- Fails without the property it checks (mutate once, restore): adding `float32 new_field` to `BallButlerHeartbeat.msg` fails `[msg-BallButlerHeartbeat]`; changing the mock annotation `hand_pos_mm: float` to `bool` fails it; renaming `self.carry_zvd` in the `InstallSegment.Request` mock fails `[srv-InstallSegment]`. All three restored. (A first draft read `str(f.type)` and silently skipped every type check; the mutation caught it.)
