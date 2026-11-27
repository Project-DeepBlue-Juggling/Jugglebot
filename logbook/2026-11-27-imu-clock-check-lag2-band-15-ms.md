---
title: "imu_clock_check.py accepts lag2 up to 15 ms: lag2 also holds the Legs' lag behind the command, and lag1 is the clock measure (BUG-132)"
type: bugfix
date: 2026-11-27
status: complete
phase: "BUG-132 Platform IMU clock"
related_plan: ""
related_issues: []
files_changed:
  - tools/probes/imu_clock_check.py
  - tools/probes/README.md
  - tests/sim/test_imu_clock_check.py
subsystem: [platform-imu, tooling]
---

## What changed

`LAG2_HI_MS` in `tools/probes/imu_clock_check.py` goes from 10 to 15 ms (docstring, README row and the verdict test
with it). The lag1 rule (abs at most `--max-abs-ms`, default 5) is unchanged.

## Why

At the Sitting of 2026-11-27 the three FW 15 captures read lag2 12.0, 9.0 and 10.6 ms and two of them failed the
0 to 10 ms band, which failed BUG-132's Sitting. The same captures read lag1 -3.2, -3.8 and -2.0 ms, the same as the
healthy FW 12-14 captures (-2.2 to -4.2 ms) and nowhere near the defect (-40 to -49 ms). lag1 compares the IMU with
the encoders and is the clock measure. lag2 compares the IMU with the command, so it also contains how far the Legs
trail the command: lag2 minus lag1 was 12.8 to 15.2 ms on 2026-11-27 against 6.4 to 11.0 ms on the earlier rows.
The band was failing a healthy clock for a Leg-following lag it was never meant to judge. Owner decision
2026-11-27: widen the band.

The larger Leg-following lag is a separate question (new firmware since FW 49, or the motion flown) and is filed as
its own Item.

## Verification

- 2026-11-27, `python -m pytest -q tests/sim/test_imu_clock_check.py`: passed (in the 115 of the scoped run with the
  tracking tests). The new row `v(-3.2, 12.0, 5.0, 0.1) == PASS` fails on the old bound. Readers found with
  `grep -rln imu_clock_check tests tools docs map plans`: the test file and the README row.
