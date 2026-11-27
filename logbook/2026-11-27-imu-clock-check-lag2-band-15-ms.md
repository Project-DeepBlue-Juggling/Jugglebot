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

## Correction (same day, BUG-151)

The comparison above ("12.8 to 15.2 ms on 2026-11-27 against 6.4 to 11.0 ms on the earlier rows") is not a change in
the Legs. Measured directly from the bags (commanded against encoder position, no IMU; 10 oval Attempts on
2026-11-27, 22 on 11-18, 11-20 and 11-23), the Legs trail the command by 13.2 ms (sd 0.5) and, method for method, by
1.4 ms less than before (CI95 -2.1 to -0.7). Bags recorded before 2026-11-23 21:52 have no `bridge_stamp` on
`/robot_state`; the pairing fallback stamps about a quarter of the encoder samples one tick early and reads the lag
5.6 ms short. The earlier rows carry that bias. The 15 ms bound stands for a different reason than the one given: a
healthy lag2 is about 9 to 12 ms, so the old 10 ms bound sat in the middle of the healthy range.

## Verification

- 2026-11-27, `python -m pytest -q tests/sim/test_imu_clock_check.py`: passed (in the 115 of the scoped run with the
  tracking tests). The new row `v(-3.2, 12.0, 5.0, 0.1) == PASS` fails on the old bound. Readers found with
  `grep -rln imu_clock_check tests tools docs map plans`: the test file and the README row.
