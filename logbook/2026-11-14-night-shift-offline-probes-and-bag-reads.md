---
title: "Night Shift runs probes and bag reads offline: nonet binds the evidence read-only and masks serial"
type: feature
date: 2026-11-14
status: complete
phase: "night-shift close-out"
subsystem: tools
files_changed:
  - tools/night_shift_nonet.sh
  - tools/night_shift_settings.json
  - .claude/commands/night-shift.md
  - tests/sim/test_night_shift_settings.py
  - logbook/INDEX.md
---

## What

Close-out C7(a). The owner wants the night to run offline probes over the rosbags and
launch logs (TKT-68's nominate/tilt-back sweep, BUG-75's capture/replay), which the deny
list refused outright (`*tools/probes/*`, `*ros2 bag*`). `tools/night_shift_nonet.sh` now
also binds `~/Desktop/rosbags` and `~/.ros/log` read-only, binds `/dev/null` over every
`/dev/ttyACM*` and `/dev/ttyUSB*`, mounts an empty read-only tmpfs on `/dev/serial`, and
points an unset `ROS_LOG_DIR` at a temp dir (rcl would otherwise fail on the read-only
log dir). Any failed mount exits 125 without running the command. The two blanket denies
are gone; `ros2 bag play|record|reindex|convert` stay refused even wrapped, and new
anchored rules (no leading `*`) refuse the unwrapped spellings of a probe or `ros2 bag`
(line start, or after `&&`, `;`, `|`), which the wrapped form never matches. Invariant 2
of the protocol says how to run one: `tools/night_shift_nonet.sh python tools/probes/<p>.py`.

## Why a nested namespace

The mounts are made in the outer user namespace and the command runs in a nested one.
In a single namespace the command is root there and simply `umount`s the read-only bind
and writes (control probe below); a nested user namespace makes the kernel lock the
inherited mounts, so `umount` and `remount,rw` are refused.

## Verification

- Probe 2026-11-14, `/tmp/probe_c7_nonet.sh 1..3` (uncommitted; stand-in dirs for every
  write, a non-mutating `access(W_OK)` + mountinfo check on the real dirs, `stat` only on
  `/dev/ttyACM0`, TEST-NET 192.0.2.1 for the network): 3/3 trials, 14/14 checks each
  (EROFS on write, append and mkdir; umount/remount refused; ttyACM0 is 1:3; `/dev/serial`
  empty; only `lo`; host stand-ins untouched; no mount leaked). Control, single namespace:
  `umount` succeeded and the write landed.
- 2026-11-14, `python -m pytest tests/sim/test_night_shift_settings.py -q`: 92 passed; against
  the old wrapper and settings 13 of the new cases fail.
- 2026-11-14, every test file that reads a changed file, run through the new wrapper with
  Foxy sourced (so `~/.ros/log` was read-only): `tools/night_shift_nonet.sh python -m pytest
  tests/sim/test_night_shift_{settings,gate,runner,candidate}.py
  tests/sim/test_sitting_debrief_protocol.py tests/ros/test_launch_{console,nodes}.py -q -n 4
  --dist loadfile`: 399 passed in 206.29 s. The default gate is run on the stacked branch.
