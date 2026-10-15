# BB calibration - kappa-step pin, part 2: three power cycles (Phase 0 of `plans/active/bb-calibration-one-attempt.md`)

Plan: `plans/active/bb-calibration-one-attempt.md` § Phase 0, "Pin of the kappa step (P7)". Part 1 (the offline
clip-yaw replay) is done before this sitting and its verdict is in the Phase 0 logbook entry; read it first. This
sitting answers one question: **does a BB power cycle, on its own, move kappa (BB's yaw offset in the shelf frame) or
the sweep's overshoot?** Nothing is tuned here and no gate changes; Phase 1 has not landed, so a sweep may still be
REFUSED by `BB_IN_BASE_MOVED` or `TEMPLATE_RESIDUAL`. That is fine for this sitting: the attempt line and the bag
carry the measurement either way, and the replay harness re-solves every sweep from the bag.

**If your physical intuition disagrees with the framing - that a power cycle could change the yaw drive's dynamics
(supply state, friction) and the offset together, or that it cannot - that's load-bearing signal, say so before we
start.**

**Operator runs every motion command and every power cycle; Claude never does.** No balls. Jugglebot legs stay
unpowered or parked; only BB moves.

## 0. Pre-flight

| # | Step | Expect |
|---|---|---|
| 1 | `cd ~/Desktop/Jugglebot-skills && git fetch && git status -sb && git log --oneline -1` | On `skill-stack`, in sync, and the Phase 0 commit from branch `bb-calibration` is merged in (the attempt line carries a `sweep` suffix only after that merge). |
| 2 | Rebuild: `cd ros_ws && colcon build --packages-select jugglebot && source install/setup.bash` (mocap_node changed). | Built after the HEAD commit. |
| 3 | `tools/nightly_ticker.sh check --who "bb kappa pin"` | `CLAIMED …`, or a `LOWERED …` line only, or `NEVER` (the nightly runs in the main checkout; expected in this worktree). |
| 4 | **Do NOT re-align the QTM global frame and do NOT touch BB, its shelf or any marker** for the whole sitting. QTM streaming, all 7 BB markers and the 4 shelf markers visible at rest. | GUI mocap panel shows BB and the base. |
| 5 | Launch (tee to `temp/logs/`), GUI up, **bag recording ON** (the harness replays the bag; without it the sitting is lost). BB connected, state IDLE. | `/bb/heartbeat` connected, IDLE. |
| 6 | Note the launch's calibration parameters from launch.log: `grep -n "bb_pose_source\|bb_yaw_source\|pooled\|base_ref" <launch.log> \| head`. | Same values as the 2026-10-14 sittings (`auto` / `auto`). |

## 1. Baseline (no power cycle)

| # | Step | Expect |
|---|---|---|
| 7 | BB at rest, untouched, 15 s (the park reading: the harness takes BB's rest heading vs the encoder from this window). | Nothing moves. |
| 8 | One sweep: `ros2 service call /bb/calibrate std_srvs/srv/Trigger`. Wait for the attempt line in launch.log (`grep -n "calibration" <launch.log> \| tail -3`). | An attempt line ending `· sweep <start>→<end>° <dur> s, lag <x> ms`. Record end angle and verdict. ACCEPTED or REFUSED both fine. |
| 9 | Repeat rows 7-8 once more. | Two baseline sweeps, 15 s rest before each. |

## 2. Three power cycles

| # | Step | Expect |
|---|---|---|
| 10 | **Power-cycle BB** (mains off, 10 s, on). Wait for `/bb/heartbeat` connected and IDLE. Do not touch BB. | Heartbeat back; yaw reads near the pre-cycle parked value (absolute encoder, constant 5 deg boot offset). Note the yaw. |
| 11 | 15 s rest (park reading), then TWO sweeps (row 8 twice, 15 s rest between). Record end angle, duration, verdict, and any `BB_IN_BASE_MOVED` delta for each. Two per cycle because one sweep against one sweep only resolves 0.19 deg; pair mean vs pair mean resolves 0.13 deg (bands.md, P7 prediction). | Two attempt lines with the sweep suffix. |
| 12 | Repeat rows 10-11 twice more. | Six post-cycle sweeps, three park readings. |
| 13 | Stop the bag. Note the bag directory name here: `________`. | |

## 3. Offline, same day (Claude)

| # | Step | Expect |
|---|---|---|
| 14 | `source ~/Desktop/PDJ_venv/venv/bin/activate && python tools/probes/bb_calibration_replay.py --bags ~/Desktop/rosbags/<bag> -v` | One row per sweep: kappa, sweep_end_deg, sweep_start_deg, lag, verdicts. |
| 15 | Compare the pair-mean kappa and the overshoot (end angle minus 119) across the four pairs (baseline + three cycles) against the Phase 0 band. | **Criterion (plan Phase 0, part 2):** a pair-mean kappa change across a cycle of more than 0.20 deg (the within-epoch advisory band; within-sitting SD is 0.04 deg) WITH the overshoot also changing = drive dynamics and kappa move together, next step is a BB-side physical check (encoder magnet seat, yaw mount). No change beyond 0.20 deg = the power cycle is innocent. Note the 10-14 20:54 sitting drifted 0.22 deg on its own over 110 s with the base steady, so also look at the trend WITHIN each pair. |
| 16 | Logbook: short-form entry with the five-row table and the verdict; plan § 5 row "0 Characterise" to done. | |
