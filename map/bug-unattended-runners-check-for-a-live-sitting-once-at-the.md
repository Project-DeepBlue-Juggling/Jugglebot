---
id: BUG-101
kind: bug
title: Unattended runners check for a live Sitting once at the start, and cannot see a bench rig
status: inbox
created: '2026-11-14'
updated: '2026-11-13T23:09:04Z'
owners: []
priority: medium
diagnosis: confirmed
area: tests-tooling
blocked_by: []
links: []
parent: night-shift
filed_by: claude
---

The unattended runners ask "is a robot Sitting live" once, at the start, and then work for hours; and the question itself cannot see a bench rig. Found by the 2026-11-13 maintainer review. BUG-85 covers the false-positive half (the pattern matches any process whose arguments name a node file; the review itself saw a peer pytest make the box read live).

## Findings

- `tools/night_shift.sh:534` checks, then may wait up to an hour for the 04:00 suite and work for up to four hours. The next check is before the app deploy (`:906`).
- `tools/nightly_suite.sh:88` checks once, then `run_tests.sh` may wait up to 30 minutes on the suite lock.
- An operator who powers up at 05:30 gets a four-worker full suite underneath the launch on a 7.3 GB box.
- Bench Sittings use standalone scripts under `tests/hardware/` and `tools/probes/`, which the pattern in `tools/lib/live_session.sh:22` never matches.
- The auto-pull is not guarded by `live_session` at all: it can fast-forward `ros_ws/` or `teensy_link/` under a running launch, whose `PYTHONPATH` points at the live tree. Whether any lazy import makes that harmful was not checked.

## Wanted

A positive signal in place of the argv guess: a lock or heartbeat file that the launch and the bench scripts hold, read by `live_session`, and re-checked by the runners before each suite and each merge. That closes BUG-85 in the same change.
