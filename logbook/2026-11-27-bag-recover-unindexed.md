---
title: "Bags: bag_compress.py --recover rebuilds the index of recordings whose recorder was killed, so the 35 'no index' bags open in the GUI replay"
type: feature
date: 2026-11-27
status: in-progress
phase: ""
related_plan: ""
related_issues: []
files_changed:
  - tools/bag_compress.py
  - tests/sim/test_bag_compress.py
subsystem: [gui]
tags: [testing]
---

## Summary

35 bags (3.4 GB) showed as "no index" in the GUI replay and could not be opened. An MCAP gets its DataEnd, summary and footer only when the recorder closes cleanly; these recorders were SIGKILLed. 30 are from 2026-11-25 07:21-10:49, written by the BUG-129 launch-series probe (`tools/probes/bug129_launch_series.sh`), whose stop escalates to killing the launch group; the other five are 2026-06-08 (three), 2026-10-25 and 2026-11-23. None was touched by the 2026-11-25 compression run, which skips unindexed files.

`python tools/bag_compress.py <root> --recover` rebuilds them. All 35 have the same shape: every chunk is complete and the file ends inside the MessageIndex records after the last chunk (the recorder flushes in 4 KiB blocks). Recovery copies every complete chunk (zstd-compressed, with a CRC), regenerates the message indexes, and writes DataEnd, the summary section, the footer and the magic. `--write-metadata` adds the `metadata.yaml` these directories never got, derived only from the recovered summary (Foxy stores `offered_qos_profiles` in the MCAP channel metadata).

## Discussion

**What cannot be recovered.** The chunk still in the recorder's memory at the kill (up to about 1 MiB, a few seconds) never reached disk.

**Why the original is kept.** Compression replaces a file because the output is proven equal to a readable input. Here the input is damaged, and although the recovered file's content hash must equal the mcap library's stream read of the original up to the cut, the original stays beside it as `<name>.mcap.unindexed-orig` (a name no `*.mcap` glob matches) unless `--discard-original` is given. The cost is 3.6 GB until the owner deletes them.

**Partial-chunk salvage** is implemented (complete inner records of a cut chunk, each checked against the chunk header's time range and compared with the original's raw bytes) but no real file ends mid-chunk, so it is covered by synthetic tests only.

**Why write metadata.yaml.** The GUI opens a recording from the `.mcap` alone, but takes duration, message count and topics from `metadata.yaml`; Foxy `ros2 bag info <dir>` fails without one; `tools/probes/hand_cup_census.py` and `imu_clock_check.py` find bags by it.

**Independent check on the parser.** Every MessageIndex record present in the 35 inputs (about 14,000) equals the one regenerated from its chunk, byte for byte.

## Verification

- 2026-11-27, `python -m pytest tests/sim/test_bag_compress.py -q -p no:cacheprovider`: **29 passed** (8 existing, 21 new: seven cut points with CRCs on and off, nothing-to-recover cases, verify failures leaving the directory unchanged, metadata.yaml shape).
- 2026-11-27, on copies of six real files (942 MB 2026-06-08_22-43-14, 2026-10-25_16-06-52, 2026-11-23_18-42-55 and three from 2026-11-25): all messages recovered (3,871,851 in the largest), the GUI's `openRecording` plus a CRC-validated `readMessages` pass in node returned the Statistics count, and `ros2 bag info -s mcap` agreed.
- The corpus run's result is appended below when it has run.
