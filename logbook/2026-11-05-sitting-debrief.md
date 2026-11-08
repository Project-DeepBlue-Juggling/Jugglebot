---
title: "Sitting 2026-11-05 (22:46) debrief: FEAT-32 A/B not flown (every Attempt cut by a firmware STOP); the three Hand-only STOPs are the first live FW 37 C1 stall stops, each mid Throw upstroke; BUG-62 refuted; FW 39 widens the C1 grace 6 -> 14 ms"
type: investigation
date: 2026-11-05
status: in-progress
phase: "carry-ring-zvd P3"
related_plan: carry-ring-zvd.md
related_issues: []
files_changed:
  - logbook/2026-11-05-sitting-debrief.md
  - logbook/INDEX.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - teensy_link/rpc_args.py
  - teensy_link/gap_monitor.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tools/probes/teensy_link_profiling/hermite_xref/teensy_interp.py
  - tests/firmware/test_bridge_fw_version_xref.py
  - tests/firmware/native/test_leg_interp.cpp
  - tests/firmware/test_sched_c2_twin.py
  - tests/hardware/hand_stroke_cut.py
  - tests/teensy_link/test_gap_monitor.py
  - tests/ros/test_teensy_bridge_node_bridge_diag.py
  - plans/active/carry-ring-zvd.md
  - docs/runbook.md
  - docs/project-map.md
subsystem: [can, ros, tracking, motion]
tags: [safety, performance, dynamics, IPC, testing]
---

# Sitting 2026-11-05 (22:46) debrief

Fifth run of `/sitting-debrief`, after `2026-11-04-sitting-debrief.md`. One Sitting, launch log `~/.ros/log/2026-11-05-22-46-00-334370-jetson-688679/launch.log`, bag `~/Desktop/rosbags/2026-11-05_22-46-00`. Code on the robot: 17a33b01 (carry-ring-zvd P0-P2, box gate_hash e8d303ae93bc, `carry_zvd` default false), bridge FW 38 / PROTOCOL 11. Runbook steps 1-3 were reached; steps 4-6 (B arm, Dwell 0.35, Columns canary) were not. Related Items: FEAT-32 BUG-58 BUG-66 BUG-55 BUG-42 BUG-34 BUG-57 BUG-62 FEAT-40 BUG-67 FEAT-39 (filed tonight: FEAT-41 BUG-68 BUG-69 FEAT-42).

## Results

| Item | Verdict | Evidence |
|---|---|---|
| FEAT-32 A/B (hold 0 / carry-ring-zvd) | FAIL (owner box FAIL; confirmed by the data) | B arm never set (no `carry_zvd` string in the launch log, no RING refusal). A arm (flag off, fed 1.00/0.40): 9/9, 3/3, 0/0 Catches, each Attempt cut by a firmware STOP (LANE_REFUSED). An unfed 0.95/0.65 Attempt reached 25/26. Pass clauses (30 consecutive, bias +/-10 mm, sd < 17.5 mm) unmeasured. |
| FEAT-32 identification | not identified | The +/-10 mm GUI clicks were two clicks 0.1-0.2 s apart (a 20 mm step in ~0.1 s), 4 x then 4 y, all positive. A damped-cosine fit of the 8 steps railed overdamped: the steps excite no ring above the 0.2-0.7 mm noise. Whole-bag forced fit (`platform_mode_fit.py`): x 5.47 Hz (encoder) / 4.68 Hz (mocap), zeta 0.26 / 0.36, delay 21 / 15 ms; y 4.06 / 4.24 Hz, zeta 0.17 / 0.14, delay 21 / 16 ms; provisional 4.9 Hz / 0.20 / 20 ms. The x encoder-vs-mocap disagreement P0 flagged reproduces. |
| Flag-off carry numbers | repeat 11-03 | Fed 0.40 Attempt: from rest +25.4 mm, carried raw x +58.7 mm (sd 4.3, n=7), excess velocity at separation +50 mm/s (encoder) / +82 mm/s (mocap) at Dwell 0.40 (11-03: +40 / +75). The unfed Dwell 0.65 Attempt carries almost none (+2 / +13 mm/s; carried raw x +22 mm, sd 19): a Dwell <= 0.50 problem. |
| BUG-58 probe row | PASS (owner typed pending; resolved to pass on the literal clause with the owner) | All four LANE_REFUSED ends have GC_PROBE lines from both stream nodes within 1 s of the STOP WARN, and the WARN carries verdict, fw_sched counters and frame ages. The Item stays open as a ride-along. |
| BUG-62 | refuted, closed | Hand telemetry source gaps <= 10 ms, Hand at rest on its held setpoint when MAX_DEVIATION tripped. |
| BUG-57 | confirmed once more | SPINOUT on a C1 brake, see Discussion (b). |
| BUG-67 | different class | Tonight's MISSED_CATCH is a fault inside the Catch window, filed as BUG-68. |

Rejections that were operator sequencing, no Item: REJECTED_BALL_IN_HAND twice (Cup not emptied after an ended Attempt), WRONG_MODE once (STANDBY after `clear_errors`). One install refused CATCH_AXIS 13.2 mm. Install-line leg peaks stayed under 50 % of every limit (max 47.5 % vel, 30.8 % acc, Hand 2617 of 4500 rev/s^2).

## Symptoms

Five Attempts, five firmware scheduler STOPs on a host stream that the bridge reported clean. Owner: the robot "just froze in position in varying ways (sometimes the Platform froze while the Hand kept moving; sometimes both)". Ends: LANE_REFUSED x4 (two followed 130-230 ms later by a Hand MAX_DEVIATION, +3.913 and +4.956 rev), and one ODRIVE_FATAL SPINOUT_DETECTED on the Hand ending MISSED_CATCH. STOP WARNs reported forward gaps of 38.2 / 39.6 / 99.5 / 44.1 / 40.7 ms against the ~70 ms cover.

## Diagnosis

Per-STOP table (bridge STOP WARN; frame age = max uplinked firmware rx age):

| STOP | forward / loop gap (ms) | fw_sched delta | frame age max (ms) | what it was |
|---|---|---|---|---|
| L1186 | 38.2 / 17.9 | stops +2, refused +4, c1 +0 | 23.6 | C2 stop, Hand at rest in the Cup before the Throw |
| L1399 | 39.6 / 14.9 | stops +1 = c1 +1 | 16.6 | FW 37 C1 stop mid Throw upstroke, then MAX_DEVIATION |
| L1602 | 99.5 / 77.5 | expired +1 | 55.6 | real bridge stall: 60.036 ms gen-2 GC pause in `teensy_bridge_node` (L1615), 0.14 s before the WARN |
| L2361 | 44.1 / 17.8 | stops +1 = c1 +1 | 15.6 | C1 stop mid upstroke (~98 rev/s), then MAX_DEVIATION |
| L2589 | 40.7 / 19.1 | stops +1 = c1 +1 | 27.6 | C1 stop mid upstroke (~100 rev/s), then SPINOUT |

The three C1 stops hold the Hand at 1.526 / 4.398 / 4.396 rev, within 0.03 rev of `P + V^2/2A + 0.03` from `hand_c1_stop_eval`. The stop itself works as coded; the problem is that it fires.

## Discussion

**(a) BUG-58 host-starvation hypothesis.** The hypothesis was that the firmware STOPs are the host starving the 40 Hz stream. Confirmed for ONE of five STOPs: L1602, where the 60.0 ms gen-2 GC pause in `teensy_bridge_node` (the only GC pause above 5 ms all Sitting) lines up with a 77.5 ms loop gap, 99.5 ms forward gap and 55.6 ms frame age. Withdrawn for the other four: loop gaps <= 19 ms, GC_PROBE maxima <= 0.44 ms, frame ages <= 27.6 ms, and the `expired`/`superseded`/`stale` counters did not move. The firmware-side-expiry reading from 11-03 stands for those four.

**(b) The Hand-only STOPs are FW 37 C1 stall stops (new).** `sched_stops +1 = hand_c1_stops +1` for L1399, L2361 and L2589: the first firings of the C1 stop on the robot. Each fired mid Throw upstroke at 52-100 rev/s, 40-70 ms after motion began, nowhere near a release (the stop did fit the 10.501 rev clip, holding at 1.5-5.0 rev). A stop mid-launch is unrecoverable by design: the resume rule demands velocity within 8 rev/s of the plan, the refused frames walk on, so every one ends the Attempt. The two MAX_DEVIATION trips are the refused plan base (5.42 / 9.37 rev) compared with a Hand at rest on its held setpoint (FW 22 latched-hold rule), not coast and not stale telemetry. This refutes the BUG-62 reading and closes it; it confirms the BUG-34 mechanism ("Hand held where it stands, refused plan walks on"). BUG-57 is confirmed once more: the L2589 SPINOUT is a C1 brake losing torque (decel 4826 -> ~500 rev/s^2 25 ms into the brake, at 41 A of the 70 A limit). The STOP WARN text said "~70 ms cover"; the C1 decision point is 50 + 6 = 56 ms under FW 37/38 (6 ms grace) and 50 + 14 = 64 ms under FW 39, which the host fix below derives instead of printing 70.

**(c) Why the Hand cover ran out.** All host drop counters are 0, frame ages <= 27.6 ms, every numbered frame was sent, and the host stamps show no hole (`hand_frame_hole` analysis). A frame lost in the downlink is the only candidate left standing, and it is invisible to every counter we have (firmware never uplinks accepted/received frame counts; FEAT-41 files them). Each stop follows the first post-Throw CATCH install by 316 / 316 / 334 ms, the heaviest solves of the chain (94-107 ms); splice coincidence holds for only one of three, so that correlation is unexplained, not a cause. I withdrew the first reading that 6 ms of grace converted survivable jitter (gap + age excursion 56-68 ms) into stops: the arithmetic fits the three C1 stops but not the C2 stops at gaps of 34.8-38.2 ms, so it is only a bound.

**(d) Tradeoff accepted: FW 39 raises the C1 grace 6 -> 14 ms** (owner, 2026-11-05 23:20). A one-knot hole now rides through when the next frame is under 14 ms old. The price is that a release-knot C1 stop fits the 10.501 rev clip only up to Apex 1.10 (`tools/probes/hand_stall_stop.py --option 2`): margin to clip +0.47 rev at Apex 1.00, +0.08 at 1.10, -0.93 at 1.35. So Apex > 1.10 is NOT flown under FW 39 until FEAT-41's counters land. The owner first asked for 20 ms, which the firmware's static_assert forbids and which overruns the clip even at Apex 1.00 (18 ms: -0.25 rev). The hole unit's dissent (counters before thresholds: change nothing until a Sitting separates lost datagram from flagless frame) is recorded; the owner's call was to ship the grace and the counters.

**(e) FEAT-40 probe.** STOP -> hold took about 130-340, 300, 320 and 275 ms (L1186/L1399/L1602/L2361); the guard latched before the hold in 2 of 4 (L1399 by 170 ms, L2361 by 51 ms). The END -> hold-install path is the larger term.

**(f) BUG-67 / BUG-68.** L2615's MISSED_CATCH is not BUG-67's class: the Hand SPINOUT disarm landed 0.03 s before the judging window closed, so the verdict blames the Catch for a fault (BUG-68).

**(g) TRACKER-IDENTITY-REFUSED (hypothesis, BUG-69).** Five CAUGHT Throws in the first Attempt (3, 4, 5, 6, 8) lost their learner row: fitted landing +1.30 s from scheduled, which is exactly one beat (flight 0.903 + Dwell 0.40), so the fit is the same ball's next flight and the Catch evidence arrived ~0.4 s late. Unconfirmed in the bag.

## Fix

- **FW 39** (no wire change, PROTOCOL stays 11): `HAND_C1_STOP_GRACE_US` 6000 -> 14000 and `FW_VERSION` 38 -> 39 in `canbridge_config.h` (both static_asserts kept, 14000 % INTERP_PERIOD_US 2000 == 0); `EXPECTED_BRIDGE_FW_VERSION` 38 -> 39 in `rpc_args.py`; the Python twin `teensy_interp.py`; native and twin tests re-planned (the old v0=90/a=1200 plan overruns at 14 ms, which is the Apex 1.35 case). Flashed with `pio run -e teensy41 -t upload`, SUCCESS 2026-11-05 ~23:30, launch down.
  Receipt (2026-11-05 ~23:40 AEDT, launch down, read-only `TeensyLinkClient` listener on PORT_STREAM for 8 s, scratch script `identity_receipt.py`): 8 BridgeIdentity frames, all `(fw_version=39, protocol_version=11)`.
- **Host STOP WARN**: `gap_monitor.py` gains `HAND_C1_STOP_GRACE_MS` (mirror, now 14.0) and `HAND_C1_COVER_MS`; `teensy_bridge_node._warn_on_sched_stop` judges against the C1 cover and names "Hand C1 stop (fast upstroke)" when the c1 delta is > 0 (C2 text unchanged). A new test pins the mirror to the header, so a future grace change fails until updated.
- **Write-backs**: notes on FEAT-32, BUG-66, FEAT-40, BUG-34, FEAT-39, BUG-67, and BUG-58/BUG-55/BUG-42/BUG-57 (forced, held by peer sessions); BUG-62 closed; FEAT-41 (hole counters), BUG-68, BUG-69, FEAT-42 (data-only Sitting verdict) filed; FEAT-32 and BUG-57 Sitting sections updated (FW 39, identification, the Apex > 1.10 caveat); the FEAT-39 and BUG-55 rows now say FW 39; plan `carry-ring-zvd` P3 `in_progress`.

## Verification

- 2026-11-05, `pio run -e teensy41` (build only): SUCCESS in 10.02 s.
- 2026-11-05, `python -m pytest tests/firmware tests/teensy_link tests/ros/test_teensy_bridge_node_bridge_diag.py -q -p no:cacheprovider`: 801 passed in 261.00 s.
- 2026-11-05, `python -m pytest tests/ros/test_teensy_bridge_node_bridge_diag.py tests/firmware/test_sched_c2_twin.py tests/teensy_link/test_gap_monitor.py -q -p no:cacheprovider` (host WARN fix): 64 passed in 11.57 s.
- 2026-11-05, `pytest tests/sim/test_map_items*.py test_plans_index.py test_sitting_sections.py test_mapkit_docs.py test_glossary_avoid_words.py` (write-backs): 191 passed, 44 skipped.
- 2026-11-05, `python tools/probes/hand_stall_stop.py --option 2 --grace-ms 14 1.00 1.10`: clip margin +0.47 rev (Apex 1.00), +0.08 rev (Apex 1.10).
`./run_tests.sh` (2026-11-05 ~23:58 AEDT, after the stroke-cut test fix): 8230 passed, 48 skipped in 427.25 s; serial tail 6 passed; exit 0 (a first run before that fix: 5 failed in tests/motion/test_hand_stream_bench_stroke_cut.py, the 6 ms pins).

## Outcome

FEAT-32 is not passed and the B arm has no data; the Sitting's reusable output is the whole-bag mode fit and the diagnosis of the five STOPs. FW 39 is flashed but unflown.

## Open Questions

- gc_freeze A/B for `teensy_bridge_node` (the one confirmed 60 ms pause): bridge-node section of BUG-58.
- FEAT-41 counters (firmware accepted/received frame counts, datagram `seq_gaps`), then revisit the 14 ms grace against Apex > 1.10.
- Re-fly FEAT-32 under FW 39 at Apex 1.00 (identification with a fast-onset hit, A arm to 30 Catches, then B arm).
- BUG-69 hypothesis (learner row lost one beat late) needs confirming in the bag.
