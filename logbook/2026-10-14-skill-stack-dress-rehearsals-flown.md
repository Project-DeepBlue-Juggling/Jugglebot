---
title: Dress rehearsals flown - lane-latch auto-recovery PASSED on a real latch, BB cancel and possession refusals PASSED, hand-stroke and late-cancel UNVERIFIED BY DECISION
type: feature
date: 2026-10-14
status: resolved
phase: "two-ball-skill-stack — residue (post-R6)"
related_plan: skill-stack-dress-rehearsals-and-residue.md
related:
  - 2026-10-13-skill-stack-r5-wave-2-after-sitting-7.md
  - 2026-10-13-skill-stack-r5-sitting-7-columns-pb.md
files_changed:
  - tests/hardware/session_skill_stack_rehearsals.md
  - plans/active/skill-stack-dress-rehearsals-and-residue.md
  - tools/probes/lane_latch_rehearsal.py
  - tests/motion/test_lane_latch_rehearsal.py
subsystem:
  - skill-stack
  - hardware-sitting
tags:
  - dress-rehearsal
  - lane-latch
---

# Dress rehearsals flown (2026-10-14)

## What
- **Rung 1, row 17a (no signal): PASSED.** 21:58:03, +50 mm over 1.2 s; pre-flight peak rps 0.25 0.40 0.40 0.25 0.15 0.15; no spurious latch over 56 samples; live_deviation max 0.0166 rev; fault NONE; go_home accepted; exit 0.
- **Rung 2, row 17b (real latch): PASSED.** Freeze STOP->CONT 101 ms; bit 15 rose at t=2.39 s; `lane_recovery` in_progress +0.30 s, done +1.00 s (WARN `lane recovery DONE in 0.7 s`), bit 15 cleared, fault NONE throughout, `sched_refused` 0 -> 24, go_home accepted. 17d bag check: /leg_setpoint_echo exactly flat across disarm and arm, measured position moved <= 0.0015 rev (0.1 mm), one executed-command re-seed of <= 0.0067 rev (0.48 mm) 9 ms after re-arm.
- **Rung 4, rows 19 and 21: PASSED** (owner report, matched in the launch.log below). Row 19: one `bb/cancel` line leading `CANCELLED`, 1807 ms before release, no ball thrown. Row 21: one accepted `reload`, then `REJECTED_BALL_IN_HAND` and `REJECTED_NO_BALL`.
- **Rung 3 (row 18) and row 20: UNVERIFIED BY DECISION** (below).
- **Probe fixed the same night.** First live runs (21:48:21, 21:48:51) at the old 40 mm / 0.5 s were refused by the planner `TOO_FAST: requested duration 0.500s < minimal feasible 0.822-0.823 s`; nothing moved, and the second run's "freeze did not latch" line was downstream of the refusal. Defaults are now 50 mm / 1.2 s / freeze at 450 ms. The runbook's shell prefix `P` held a literal `<flags>` that bash read as a redirect (a flag after the closing quote became `$0`); it now carries two full commands with the flags inside the quotes.

## The 17b live_deviation FAIL (max 0.1577 rev vs 0.10)
The deviation grew 0.034 -> 0.158 rev over t=0.7..1.2 s while the firmware was refusing the stream (`sched_refused` rising over the same samples): the Jetson kept emitting the rest of the quintic to a held lane, so the deviation is the remaining travel on the refused stream, 62 % of the legs-2/3 travel of 0.256 rev = 0.159 rev. It collapsed to 0.0006 rev at converge and stayed <= 0.002 rev through disarm, arm and DONE: no sag across the disarm dwell, no step at the arm edge. Margins: firmware `MAX_DEVIATION_REV = 1.0` (`ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h:240`), bridge early-disarm escape `_LANE_ESCAPE_DEV_REV = 0.5`; the probe's 0.10 expectation measured the wrong window and is split into an INFO peak-while-latched row (FAIL at >= 0.5) and a PASS/FAIL recovery-window row.

## Owner ruling: rows 18 and 20 not flown
Owner decision 2026-10-14: both need a manual action inside a sub-second window (a `kill -STOP/-CONT` hand stroke; the ~0.6 s BB wind-up), which is not reliably doable by hand, so a failed attempt would be uninformative. Marked UNVERIFIED BY DECISION. Fallbacks that stand: `/recover` (DEACTIVATE then ACTIVATE), E-stop. To verify later: automate the SIGSTOP timing in the probe (row 18), a scripted late `juggle_stop` (row 20). The "hand stays where it stopped" ruling stands on the owner's word and on 17b (hand held at -0.01 rev).

## Verification
- Probe, 2026-10-14 21:48:21 and 21:48:51, old defaults (`lane_latch_rehearsal.py`): REFUSED `TOO_FAST`, nothing moved.
- Probe, 2026-10-14 21:58:03, `... lane_latch_rehearsal.py --no-freeze --dx-mm 50 --move-s 1.2 --at-ms 450`: exit 0, all rows PASS (`temp/probes/lane_latch_rehearsal_20261014_215803.json`).
- Probe, 2026-10-14 21:58:43, same flags without `--no-freeze`: all rows PASS but the old `live_deviation` row (0.1577 rev, wrong window; `_215843.json`).
- launch.log `~/.ros/log/2026-10-14-21-56-45-798357-jetson-4154869/launch.log`, bag `~/Desktop/rosbags/2026-10-14_21-56-45`: l.452 `Throw at jugglebot: in 2.5 s` (fed attempt); l.474 `teensy_bridge_node ... bb/cancel: CANCELLED: BB dropped the throw 1807 ms before its release (ball retained ...`; l.478 `END STOPPED: Ball Butler throw cancel (release was 1.89 s away)`; l.479 `Throw at jugglebot NOT thrown: THROW_ABORTED_BY_HOST (axis=n/a, detail1=1807)`; l.538 `Throw at jugglebot: in 3.0 s` (accepted `reload`); l.574 and l.588 `jugglebot/juggle REJECTED: REJECTED_BALL_IN_HAND -- refused: ball already in hand`; l.620 `REJECTED_NO_BALL -- refused: no ball in hand -- columns (reload) opens by throw...`.
- Not found in the launch logs: `REJECTED_BALL_UNKNOWN` (not exercised); launch 21-47-31 holds none of the grepped lines; launch 21-27-21 holds earlier `REJECTED_NO_BALL` lines (l.552, l.1537) from the previous sitting.
- 17d verdict: bag `2026-10-14_21-56-45` shows no leg step across disarm/arm (echo flat 0.0000 rev, measured <= 0.0015 rev, one <= 0.0067 rev (0.48 mm) executed-command re-seed 9 ms after re-arm; the 0.052 rev echo jump is the pre-latch go_to_pose motion)
- Doc tests, 2026-10-14: `pytest -q -p no:cacheprovider tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py tests/sim/test_plans_index.py` -> 112 passed in 0.95 s

## Open
- Rows 18 and 20 stay unverified until the probe automates the SIGSTOP timing / a scripted late `juggle_stop` exists.
