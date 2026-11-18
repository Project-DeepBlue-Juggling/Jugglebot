---
id: BUG-116
kind: bug
title: ROS mock messages accept misspelt fields and are never compared with the real interface definitions
status: inbox
created: '2026-11-18'
updated: '2026-11-18T15:51:17Z'
owners: []
priority: medium
effort: l
hardware: none
owner: none
radius: subsystem
diagnosis: confirmed
area: tests-tooling
blocked_by: []
links:
- bug-tests-that-never-run-or-cannot-fail-two-permanent-skips-node
filed_by: claude
night_skip: frozen-path
night_skip_on: '2026-11-19'
night_skip_note: 'Needs tests/ros/conftest.py, a never-list path: a day Session with the full gate.'
---

The ROS mock layer accepts a misspelt message field, so a node test can pass on a field the real message does not have. Split from BUG-103 on 2026-11-18.

## Findings

- `tests/ros/conftest.py` (1,517 lines) mocks messages as 37 plain dataclasses. Assigning a field that does not exist passes silently.
- Nothing compares the mock definitions with the 49 real `.msg`, `.srv` and `.action` files in `jugglebot_interfaces`.

## Wanted

- `__slots__` or a strict `__setattr__` on the mock messages.
- A test that the mock fields equal the real interface definitions.

## Risks

- This changes the base under about 3,800 ROS tests. Expect existing misspellings to surface; each is a fix in a node or in a test.
- `tests/ros/conftest.py` is on the Night Shift never-list, so this needs a day Session and the full gate.
