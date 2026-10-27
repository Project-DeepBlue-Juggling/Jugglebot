---
title: "Leg lead clamp P3: one level 0.50 oval Sitting on FW 33 - zero staircase episodes, zero clamp ticks through real encoder-frame drops; plan complete"
type: investigation
date: 2026-10-27
status: resolved
phase: "leg-lead-clamp-age-aware — P3 (robot validation)"
related_plan:
  - leg-lead-clamp-age-aware.md
  - two-ball-oval.md
related_entries:
  - 2026-10-25-leg-lead-clamp-age-aware-p2-bench.md
  - 2026-10-24-leg-lead-clamp-age-aware-p1-firmware.md
sessions: []
files_changed:
  - tools/probes/lead_clamp_staircase_probe.py
  - tests/hardware/session_oval_rung1.md
---

# Leg lead clamp P3: robot validation

## What

One powered Sitting on 2026-10-27, 22:59 launch. The bridge reported FW 33 (`BRIDGE_FW_CHECK: OK — can-bridge v33`). Planner was
`level` throughout. Ten goals were accepted. Four were `oval_1ball` at dwell 0.50; a fifth 0.50 goal, `oval_1ball_fed`, was refused
`REJECTED_BALL_IN_HAND`. The other six were three oval goals at 0.65 (including the one `oval_1ball_fed` that flew) and three hop
goals. Paths:
- Bag: `/home/jetson/Desktop/rosbags/2026-10-27_22-59-40/2026-10-27_22-59-40_0.mcap`.
- Launch log: `/home/jetson/.ros/log/2026-10-27-22-59-39-920154-jetson-2913798/launch.log`.

The Runbook is `tests/hardware/session_oval_rung1.md` § 10. The operator reported no leg stutter.

**Gate met: zero staircase episodes.**

`tools/probes/lead_clamp_staircase_probe.py` gained a `--scan <mcap>` mode for this read-back. It does two things:
- runs the 2026-10-24 staircase detector (at least 3 frozen `/leg_cmd_executed` runs while the target moves faster than 0.5 rev/s);
- reports the bridge's `leg_clamp_diag` census, differenced across the bag.

Oracle check: on the 2026-10-24 `11-34-29` bag (FW 30) the scan finds the known leg-1 episode at 1792802191.80, a 12.4 mm max step.

## Verification

- **Scan.** 2026-10-27, `python tools/probes/lead_clamp_staircase_probe.py --scan /home/jetson/Desktop/rosbags/2026-10-27_22-59-40/2026-10-27_22-59-40_0.mcap`
  (output `temp/logs/lead-clamp-p3-scan.log`):
  - **0 staircase episodes** over 505.6 s of `/leg_cmd_executed`.
  - Census delta: clamp `0 0 0 0 0 0` | stale `0 920 605 590 649 700` ticks.
- **The frame drops were real and were absorbed.** Five of the eight stale bursts match a per-leg heartbeat-dropout episode in the bridge log. Leg 3 (584) and leg 4 (706, 853) have none: that log's threshold is 500 ms, and a stale burst can occur below it.

  | Leg | Time | Stale ticks |
  |---|---|---|
  | 1 | 635 | +530 |
  | 3 | 584 | +590 |
  | 5 | 705 | +375 |
  | 4 | 706 | +339 |
  | 5 | 847 | +325 |
  | 4 | 853 | +310 |
  | 2 | 866 | +605 |
  | 1 | 874 | +390 |

  Two of these happened in motion:
  - **Leg 2 at 865–868:** up to 3.68 rev/s, 112 ms max frame age. The recorded command's max step was 2.33 mm per 10 ms sample, which is smooth at about 260 mm/s.
  - **Leg 3 at 583–586:** 1.59 rev/s.
- **Counterfactual on those two windows.** The `hermite_xref` twin was replayed through `replay_window` with the fixture recipe, using the bag's echo knots and fresh encoder samples:

  | Window | FW 30 twin (`firmware_fw30`) | FW 33 twin (`firmware`) |
  |---|---|---|
  | Leg 2 | 251 engaged ticks, max step 20.1 mm, 7 large steps (the 2026-10-24 staircase) | 0 engaged ticks, max 2.8 mm |
  | Leg 3 | 82 engaged ticks, max 4.8 mm | 0 engaged ticks, max 1.1 mm |

- **Current.** The flight `|iq_setpoint|` peak was 10.0 A (leg 2); per leg: 6.2 / 7.6 / 10.0 / 9.3 / 5.1 / 6.2 A. No sample reached 14.9 A. On 2026-10-24, two vibration episodes sat on the 15 A clamp.
- **`MOTOR_FB_STALE`.** No trip during the Sitting.
- **Pre-power full gate.** `./run_tests.sh --full`, 2026-10-27: 7367 passed / 4 skipped + serial 8 passed, PASS (`temp/logs/lead-clamp-p2-full-gate-2.log`). No `controller/`, `sim/` or `ros_ws/` edit landed between that run and power-on.
- **This commit's gate:** the `./run_tests.sh` triple is in the commit message.

## Findings outside the gate (filed, not fixed)

- **JB-75: `SETPOINT_STALE` latch 117 ms after the first arm.** Sequence: armed at .376, latched at .493, first `/leg_setpoint_echo` sample at .412.
  - An earlier latch was already reported at node start (1793102384.89, output disarmed) and was cleared at 385.95. It was most likely left over from the P2 bench session on this un-power-cycled bridge; the source is not established.
  - The bridge had not been power-cycled since the P2 reflash, so Runbook row LC-1 was skipped. Uptime was 5523 s at launch, and leg 0's census still held the P2 bench leg's counts (clamp 361, stale 4071).
  - Mechanism: `s_last_setpoint_us` survives across arms, so on a bridge that has streamed before, the 10 Hz staleness check can land between the arm and the first new setpoint.
  - This predates FW 31 and has nothing to do with the clamp. It fails safe; clear_errors recovered it, and the re-arm at 563.2 held for the rest of the Sitting.
- **JB-76: `SPLICE_TOO_LATE` ended 5 of 10 goals.** These were Jetson-side CATCH solves (median 168 ms, p90 276 ms, max 380 ms) against the 250 ms deadline, a tail that has grown since 2026-10-23. The bridge is not involved.
- **Leg 0's stale census did not move during the Sitting**, including its 1.7 s heartbeat-dropout episode at 827–829.
  - This is not evidence of a defect. Stale ticks and dropped frames are loosely coupled: leg 2 counted 605 stale ticks with "0 co-dropped" frames, so delivery is bursty.
  - Leg 0's clamp counter also stayed at 0, and the staircase scan reads `/leg_cmd_executed` independently of the census.
  - Power-cycle the bridge before the next Sitting, and read leg 0's census there.

## Outcome

The plan is complete and archived. Its gate opens `two-ball-oval.md` runbook § 9, the P3 re-gate, and P4's `leg_curr_limit_a` 15 → 30, as far as this prerequisite goes. JB-76 still stands in front of any 30-catch run.
