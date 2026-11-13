---
id: FEAT-55
kind: feature
title: A real-rclpy launch smoke test in the full tier
status: inbox
created: '2026-11-14'
updated: '2026-11-13T23:09:59Z'
owners: []
priority: medium
area: tests-tooling
blocked_by: []
links: []
filed_by: claude
---

Add one test tier that runs the nodes against real rclpy on the Jetson. From the 2026-11-13 maintainer review: no test under `tests/` calls `rclpy.init`, uses `launch_testing` or imports the real `jugglebot_interfaces`; every node test runs on the mock layer in `tests/ros/conftest.py`.

## What the mock layer cannot see

- Callback groups, executor threading, reentrancy and deadlock: the mock calls callbacks synchronously.
- QoS mismatches between a publisher and a subscriber.
- Message field types, and drift between the hand-copied mock messages and the real interface files.
- Parameter typing and undeclared parameters.

## Wanted

A small launch test, in the full tier only, that starts the nodes with a fake bridge peer and asserts that every node comes up, declared parameters load, and one message crosses each topic the skill stack depends on. It must refuse to start when `live_session` says a Sitting is live.

## Open questions

- Whether Foxy's `launch_testing` is workable here, given the `-p no:launch*` options in `pyproject.toml`, or a plain subprocess harness is simpler.
- Which fake stands in for the can-bridge: `tools/teensy_link_bridge.py` or the sim's stream chain.
