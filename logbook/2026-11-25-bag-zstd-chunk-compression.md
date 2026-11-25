---
title: "Bags: lossless in-place zstd chunk compression (tools/bag_compress.py) and a zstd decoder in the GUI replay, so the bag corpus shrinks about 4x with every path and reader unchanged"
type: feature
date: 2026-11-25
status: in-progress
phase: ""
related_plan: ""
related_issues: []
files_changed:
  - tools/bag_compress.py
  - tests/sim/test_bag_compress.py
  - ros_ws/gui/js/replay/mcap-decode.js
  - ros_ws/gui/js/replay/ui/format.js
  - ros_ws/gui/lib/mcap-bundle.min.js
  - ros_ws/gui/lib/VENDORED.md
  - tools/gui_vendor/entry.mjs
  - tools/gui_vendor/package.json
  - tools/gui_vendor/package-lock.json
  - tests/ros/test_replay_mcap_oracle.py
  - tests/ros/test_gui_vendor_pin.py
  - tests/ros/_replay_fixture.py
  - .claude/commands/sitting-debrief.md
subsystem: [gui]
tags: [performance, testing]
---

## Summary

The SSD was at 89 % (25 GB free) and the bags were the only consumer that grows with Sittings: 69 GB, 6.5 GB a week over the last four weeks, every chunk uncompressed (Foxy `ros2 bag record -s mcap` cannot reach the plugin's chunk-compression options). Two changes:

- `tools/bag_compress.py` rewrites a bag in place with each MCAP Chunk zstd-3 compressed. Records are copied byte for byte; only chunk payloads, the offsets that move (ChunkIndex, attachment and metadata indexes, SummaryOffset, Footer) and the CRCs change. File name, `metadata.yaml` and mtime are kept, so the 247 bag paths cited in the logbook, map and tests still resolve. Dry run by default; `--replace` swaps a file only after it verifies.
- The GUI replay opens zstd chunks: `fzstd` 0.1.1 is bundled into `mcap-bundle.min.js` (+7 kB) and passed as `decompressHandlers`. Any other chunk compression is still refused as `compressed`.

`/sitting-debrief` gains a closing step that runs the tool.

## Discussion

**Why chunk-level, not whole-file `.mcap.zst` or record-time compression.** Whole-file zstd compresses slightly better (4.74x against 3.98x at level 10 on one November bag) but renames the file: every `*.mcap` glob in `tools/` and the GUI's recordings list would miss it, and no reader could seek. Foxy's own `--compression-mode file` has the same defect. Chunk-level keeps the path and the index, and every python reader (`mcap`, `mcap_ros2`) and Foxy's C++ reader (`ros2 bag info`, `ros2 bag play -s mcap`) read it unchanged.

**Why a record-level transcoder.** The python `mcap` Writer rewrites at about 1 MB/s (17 h for the corpus). Copying records and recompressing only chunk payloads runs at 70-90 MB/s on this box.

**Why level 3.** Level 10 is about 2 % smaller on recent bags for 5x the CPU; level 19 gains 5-8 % for 30-70x.

**What makes an in-place rewrite of sole-copy data acceptable.** No backup of the bags existed on the day. A file is replaced only after: the output's summary CRC recomputes; a SHA-256 over every schema, channel and message in file order (read by the `mcap` library's StreamReader with CRC checks on, not by the transcoder's parser) equals the original's; statistics, schemas, channels and chunk-index lists are equal; and every ChunkIndex seeks to a chunk that decompresses with a valid CRC and whose message indexes land on the right messages. Any mismatch deletes the partial and leaves the original. Residual: both sides of the content hash are read by the same library, mitigated by the chunk CRCs being computed from the original raw bytes and by Foxy's C++ reader playing four compressed real bags end to end.

**Accepted cost.** Replay scrubbing is about 1.7x slower per 10 s slot (about 1.1 s against 0.7 s in node on a 543 MB bag, loaded box). Lossy options (gating or decimating mocap while idle) were rejected: 8-22 % saving against 4x, and idle mocap is what the robot-frame and trapped-ball tools read.

**Not handled.** 35 bags (3.4 GB) have no footer, 30 of them recorded 2026-11-25 07:21-10:49; the tool skips them untouched. Recovering or diagnosing them is separate work.

## Verification

- 2026-11-25, `python -m pytest tests/sim/test_bag_compress.py -q -p no:cacheprovider`: **8 passed in 0.91 s**.
- 2026-11-25, `python -m pytest tests/ros/test_replay_mcap_oracle.py tests/ros/test_gui_vendor_pin.py tests/ros/test_gui_replay_ui_lobby.py tests/ros/test_gui_replay_mcap_source.py tests/ros -q -k "replay or vendor or gui or mcap"`: **922 passed, 1 skipped**.
- Real data, on copies: 33 MB (2025), 567 MB (2026-06) and 1.57 GB (2026-11-13) bags compressed 2.69x, 3.84x and 3.77x; `ros2 bag info` reported identical message counts and topics; peak RSS 32 MB.
- The gate result is in the commit message; the corpus run's per-file report (content hashes included) is `temp/reports/bag_compress_2026-11-25.json`.
