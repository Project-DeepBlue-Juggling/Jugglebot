---
title: "Robot Sitting 2026-11-20 (evening) debrief: Platform IMU Rung P4 passed Armed, 0 I2C errors through 34 Throws; Armed uploads are slow because 0.4 % of frames go missing and the hole-fill re-reads what it holds"
type: investigation
date: 2026-11-20
status: complete
phase: "Sitting debrief"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - logbook/2026-11-20-sitting-debrief-robot-2.md
  - logbook/INDEX.md
  - map/sheets/2026-11-20-robot-2.results.yaml
  - map/idea-platform-accelerometer.md
  - map/idea-minimise-the-platform-imu-upload-time-after-an-attempt.md
  - map/bug-missed-catch-end-line-names-the-empty-carried-throw-not-the.md
  - plans/active/platform-imu.md
  - .claude/commands/sitting-sheet.md
  - docs/project-map.md
  - docs/runbook.md
subsystem: [platform-imu, can-bridge, skill-stack, tooling]
tags: [sitting, hardware, diagnostics, debrief]
---

# Robot Sitting 2026-11-20 (evening) debrief

Sheet `2026-11-20-robot-2`, 18 steps, IDEA-20 only, two launches (the first ended at the owner's power cycle after the
replug). Platform Teensy FW 12 with the 2.2 kOhm I2C pull-ups, can-bridge FW 46. Bags `~/Desktop/rosbags/2026-11-20_22-49-44`
and `2026-11-20_22-53-34`; captures `temp/imu_capture/20261120T115806Z_AEDE` (whole ring, Armed), `120305Z_AEDF`,
`120406Z_AEE0`, `120634Z_AEE1` (the three oval Attempts).

## Summary

- IDEA-20, Rung P4: **pass**, confirmed by the owner 2026-11-20 23:45. The analysis unit derived `fail` on the literal reading (the
  forced gap was not flown and uploads raise the bus load); the debrief offered `inconclusive`; the owner set pass and
  treats both as residue.
- Replug: the status returned to RUNNING at 22:52:41.89 on the same boot (`platform_boot` 175), 7 s before the owner's
  Ctrl-C. The power cycle was not needed; the firmware re-probes every 60 s once its short retries are spent, and
  failed probes are visible at 22:50:41 and 22:51:41.
- Armed: `i2c_errors` and `fifo_overflows` were 0 in all 1593 status messages of launch 2 (22:53:39 to 23:06:50),
  covering the ring upload, a 4 min 12 s soak and three oval Attempts (34 Throws caught).
- Captures: all four complete, 0 discontinuities, 0 unfilled holes; the ring capture's CRC is OK (span captures carry
  no CRC check). Chip rate 993.87 Hz, quiet |a| 1.0004 g, peak |a| 11.0 g and peak rate 351 deg/s in the Attempts
  (66 % and 35 % of full scale, no saturation). `temp_c_start` and `temp_c_freeze` are within 0.48 C of the status
  temperature, which closes the check left open in `2026-11-20-platform-imu-capture-header-temperature`.
- Uploads while Armed were slow: 79 s (ring), 37 s, 45 s and 84 s (32 s span) against 56 s at rest, with 14651 to
  19974 duplicate frames (158 at rest) and up to 6 block timeouts. CAN1 ran about 42 % idle Armed, 64 % in an Attempt
  and 74 to 77 % during an upload. `setpoints_coalesced` went 0 to 12 over the launch (the no-IMU bag
  `2026-11-20_13-48-32` went 0 to 22 in 7.5 minutes of Attempts); `interp_deadline_misses`, `crc_errors` and `seq_gaps`
  stayed 0.
- Oval Attempts on the fresh plant `imu-oval-2026-11-20`: two ended MISSED_CATCH on their first Throws, the third ran
  32 of 32 (landing median 16 mm). A juggle goal was refused 7 s into a 37 s upload (launch log line 2106).
- Trapped-Ball scan: no Hand stalls. One under-Hand dwell, 23:03:20.2 to 23:03:21.8, the lost Ball of the second
  Attempt, which then lay on the Platform for 2.7 s.

## Big picture

What went well. The Platform IMU now works on the robot under power. With the pull-ups fitted there was not one I2C
error or FIFO overflow in about twelve Armed minutes, including 34 Throws, and every capture arrived whole. The sensor
has ample range: the hardest Throw reached 11 g of 16. The replug recovery works without a power cycle; it only needs
up to a minute of patience. The first real Attempt recordings exist, including a full 32 s of a clean 32-Throw run.

What did not. Uploads take about half as long again when the robot is Armed (about 80 s against 56 s), and a goal sent
during an upload is refused, so the wait is felt between Attempts. Two Attempts were lost at their first Throws
because a fresh plant starts with an empty Memory.

What it means. The belief that the bridge relays more slowly when Armed is withdrawn. About 0.4 % of data frames go
missing on the way when Armed (0.05 % at rest), and the Jetson's hole-fill then re-reads long stretches it already
holds; that re-reading, not the bus, is most of the lost time. The bus stays safe: no deadline miss, and uploads never
overlap an Attempt. The fresh-plant loss is the cost IDEA-23 already describes, seen again.

Next steps. IDEA-20: Rung P5, the mount calibration (Level, then X and Y Tilts with Mocap), so the recordings can be
read in the Platform frame; the GUI gains IMU charts so the owner can look at captures. IDEA-25: a Jetson-only change
to the hole-fill (owner approval pending) is modelled to bring an Armed upload from 79 s to about 64 s; counters for
where the frames are lost need a firmware flash. Pathway two-ball-oval: the 32 s capture can settle the ring-mode
frequency question once the mount is calibrated.

## Discussion

**Why Armed uploads are slow (read-only diagnosis, Opus unit, 2026-11-20).** Hypotheses tested against the code and the
capture `stats` blocks:

- (a) The bridge drains its IMU ring more slowly when Armed, the Jetson times out and re-requests: **withdrawn**. The
  drain rate does not depend on Armed, the Armed gap clamp did not bind (gap 80 in every capture), and the first pass
  has no duplicates at all.
- (b) The bridge ring overflows: **refuted**. `ring_drops`, `send_fails`, `dropped_batches` and `fifo_overflows_jb`
  are 0, and the bridge's received-frame count equals the Jetson's `frames_rx` for three uploads.
- (c) Bus arbitration slows the Platform's paced transmit: minor, about 4 % (0.83 ms per frame against 0.80).
- (d) The Jetson timeout is too short for the Armed delivery rate: **refuted**; a timeout also needs 150 ms of silence.
- Found instead: about 0.4 % of IMU data frames never reach the bridge when Armed. `_Region.next_block`
  (`teensy_link/platform_imu.py`) asks for everything from the first hole to the last hole within 256 words, so each
  hole-fill READ re-sends the words between holes that are already held. One omission rate (p = 0.004) run through the
  Jetson's own region logic reproduces capture AEDE: 397 blocks against 398, 14470 duplicates against 14651, 78.9 s
  against 79.2 s. Of the 23 s lost, about 13 s is re-reading, 5 s the 30 ms settle after each holed block, 2 s pace
  drift and 1 s timeouts.
- Not determinable from the code: where the 0.4 % is lost (the Platform's transmit side, or uncounted at the bridge).
  That needs transmit and receive counters for the data frame, which is a firmware change on both boards.

**Proposed, not applied (owner decision).** Jetson only, no flash: end each hole-fill READ at a run of more than 16
present words and cut the settle from 30 ms to about 5 ms. Modelled: 79 s to about 64 s Armed, duplicates from about
14500 to about 200, about 1.3 times as many READs. Checks: at rest fewer than 100 duplicates and 56 s or less; Armed
fewer than 1000 duplicates and 62 to 66 s with the CRC OK. The detailed working is not in the repo (Session scratchpad).

**The missed Catches.** Neither was a Ball Butler Feed miss: both Feeds were caught. The Hand's own first Throws on the
empty Memory landed about 45 mm off with the Apex at 1.08 m for 1.00 m commanded (K about 1.08), beyond the 10 mm
re-aim authority; in the first Attempt a re-aim was also refused SPLICE_TOO_LATE (solve 163 ms against a 153 ms
budget). With 2 Memory rows the third Attempt's landing errors fell below 15 mm by Throw 5. This matches IDEA-23
(fresh-plant K seed; that Item is held by the Night Shift, so its note lives here) and repeats BUG-114 (both end lines
name the empty carried Throw). The line numbers in the unit's notes were approximate.

**Safety read of the bus load.** One 77 ms host forward gap during the ring upload (22:57:44) coincided with one
`sched_stops` count; a 63 ms gap also occurred in the soak with no upload running, and the median never moved. Uploads
are gated out of Attempts, so the exposure is the Active hold.

## Write-backs

- IDEA-20: Sitting result written (pass), progress line, and its `## Sitting` section replaced by the Rung P5 row
  (mount calibration, plus the rate ramp left from P4). The 1 degree gravity criterion in that row is a proposal for
  the owner. Plan `platform-imu.md`: P4 complete, P5 in progress.
- IDEA-25: progress line with the upload numbers. BUG-114: seen-again line.
- `/sitting-sheet` rule 4: no plant restore step is needed when the next step ends the launch (the launch starts
  `plant_id` at `jugglebot`; owner question at step 16).
- Not done: `docs/records.md` (the runtime ledger is ahead; `python tools/runtime_records.py merge` refused because a
  peer Session's idle shell matched its live-Sitting check, and the override flag was not permitted to this Session).
  No held branches, so nothing merged and no rebuild.

## Verification

Read-only analysis of the two bags and four captures (2026-11-20, scripts in the Session scratchpad; counters above).
Paperwork tests: see the commit message.
