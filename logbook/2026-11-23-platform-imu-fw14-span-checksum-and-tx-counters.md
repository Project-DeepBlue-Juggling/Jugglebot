---
title: "Platform FW 14: a chunk-CRC table makes every IMU upload verifiable (span uploads included) and a diag region counts IMU_DATA placed, sent and pending per TX mailbox plus CAN error-counter rises; no can-bridge change; built, not yet flashed"
type: feature
date: 2026-11-23
status: in-progress
phase: "IDEA-25 firmware 14"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - config/protocol_config.yaml
  - ros_ws/src/jugglebot/Teensy_code_platform/imu_capture.h
  - ros_ws/src/jugglebot/Teensy_code_platform/imu_service.h
  - ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino
  - teensy_link/platform_imu.py
  - teensy_link/rpc_args.py
  - ros_ws/docs/platform_imu_protocol.md
  - tests/firmware/native/test_platform_imu_fw14.cpp
  - tests/firmware/native/build.py
  - tests/firmware/test_native_firmware.py
  - tests/teensy_link/fake_platform_imu.py
  - tests/teensy_link/test_platform_imu.py
---

# Platform FW 14: span checksum and TX counters

## What and why

Before FW 14 the only payload check was `payload_crc32` over the whole frozen capture, so the span upload that
follows every Attempt was logged `crc not checked`: nothing proved the assembled span (words from pass 1, hole
refills and chained READs) equal to what the board held. And the IDEA-25 investigation located the Armed frame loss
only as "before the bridge software sees it", with no counter separating the Platform's transmit mailboxes from the
wire and the bridge's receiver.

FW 14 adds two READ-able regions and no op, no frame and no can-bridge change (contract:
`ros_ws/docs/platform_imu_protocol.md` sections 3 and 6):

- **Chunk-CRC table** at 0xFF40: one word `[crc32][n_words | READY]` per 512-word chunk, 125 sample chunks and 3
  anchor chunks (128 words, one READ). Computed lazily after the FREEZE reply, 128 words per otherwise idle loop()
  pass (about 0.1 ms; whole table under 0.5 s). The host widens a span to whole chunks (at most 255 samples per
  edge), checks every chunk it holds, re-reads the words of a chunk that disagrees (2 times at most), then writes the
  capture `crc_ok = false` with an ERROR. `flags.crc_method` says what verified it (`chunks`, `chunks+payload`,
  `payload`); the launch line reads `crc ok (chunks)`.
- **Diag** at 0xFA40, the findings' section 4 layout (30 bytes): `data_placed`, `data_tx_done` from a TX-mailbox
  tracker, mailbox losses, pending over 2 ms, the peak placing-to-sent time, ECR TEC/REC rises and live values,
  `tx_deferred`, `tx_queued_anyway`. Read after the header and before RESUME; deltas in `stats.platform_tx` beside
  the bridge `rx_frames`/`batches_sent` deltas. Any error or timeout on either region only leaves that check undone.

Both are gated on the frozen header's `fw_version >= 14`, so an FW 13 board is read exactly as before and an older
host never touches the new regions.

## Discussion

**Why a chunk table over the alternatives.** (i) A "span CRC" region answering CRC(first, count) of the last
requested span needs the span passed to the board, but the only argument the bridge relays per READ is
`first_seq`/`count` (count at most 256), so a 64000-word span cannot be named without a new op, which the bridge
(FW 47, case-by-case relay) would refuse; and it would need an O(span) computation on demand, inside one pass or as
a new deferred reply. (ii) A CRC trailer per READ block adds a frame to every block, verifies what one block sent
rather than the assembled span (refills mix blocks), and a lost trailer is a new kind of hole; chained blocks would
need the trailer at the merged end. (iii) Per-chunk CRCs are computed once on an immutable image, travel by ordinary
READs, localise a mismatch to 512 words (two READs to heal) and cost 512 B. Their one weakness, unaligned span
edges, is closed exactly by reading the rest of each edge chunk (at most 2 x 255 samples, about 0.4 s of upload in
the worst case), rather than by an edge-CRC mechanism that would need a span argument again.

**Why 512-word chunks and not 256.** 256 would halve the edge cost and the re-read, but 250 + 5 entries do not fit
any free, non-contiguous gap of the 16-bit seq space (the gaps are 0xFA00-0xFA7F, 128 words, and 0xFF30-0xFFEF, 192),
and two regions would complicate the host. CRC16 per 256 words would fit (3 per word) but weakens detection to
2^-16 per bad chunk for a check whose point is proving assembly.

**Laziness, not at FREEZE.** Computing the table in the FREEZE CRC walk would double that walk and delay the FREEZE
reply (the host's 1.5 s timeout has margin, but the delay buys nothing): the table is needed only at the end of an
upload, seconds later. The slice runs only in a pass that sent no IMU_DATA frame, so the fixed-schedule pacing
absorbs at most 0.1 ms of lateness.

**Spec details changed against FlexCAN_T4 1.159.0** (`FlexCAN_T4.tpp`): `write()` scans from `mailboxOffset()` only
when `msg.seq == 0` (default; the sketch never sets it), so the prediction replicates exactly that path. Data word 0
holds `buf[0] << 24 | buf[1] << 16 | ...`, so the seq is unpacked from bits 31-16. A mailbox found TX_ONCE but holding
a different ID or seq counts as sent (another writer reused it after our frame went out), not as an anomaly. The
tracker tracks MB 0-15 (setMaxMB(16)); a prediction outside clears flag bit 0. `data_tx_lost_mb`,
`data_pend_over_2ms` and the TEC/REC sums wrap (deltas) rather than saturate; the spec named saturation only for
`data_pend_max_us` and `tx_queued_anyway`, which keep it. The 4 reserved bytes at offset 26 stay 0: the span
checksum did not need them.

**Cost.** Tracker: at most 8 records x 3 register reads plus one ECR read per pass, under 1 us, nothing waits.
Image size against FW 13 (`pio run -e teensy40`, 2026-11-23): text 75072 -> 76096 B, data 14016 -> 15040 B (the
structures are about 780 B: 512 B CRCs, 30 B diag, 192 B mailbox records), bss unchanged.

## Verification

- Native: `tests/firmware/native/test_platform_imu_fw14.cpp` (10 cases).
- Results: see the commit message (date, command, counts).
- Builds: `pio run -e teensy40` and `pio run -e teensy40_imu2k` (2026-11-23) both SUCCESS, no warnings.
- Not flashed: the board still runs FW 13. At-rest check after flashing (launch up, robot at rest, more than
  10 s after the Platform boot): the launch log shows `PLATFORM_FW_CHECK: OK — Platform Teensy v14`; then
  `ros2 service call /platform_imu/upload jugglebot_interfaces/srv/SetString "{data: '{\"t_start\": T0, \"t_end\": T1}'}"`
  with T0 = now - 12 s and T1 = now - 8 s (wall seconds, `date +%s.%N`) must log `Platform IMU upload finished
  (..., complete yes, crc ok (chunks), file ...)`, and that capture's `.json` must carry `flags.crc_method` =
  `chunks`, `flags.partial_span` true, `stats.chunks_checked` >= 2, `stats.chunk_mismatches` absent and
  `stats.platform_tx` with `tracking_valid` true, `data_placed` = `data_tx_done`, `bridge_rx_frames` within
  about 30 of `data_placed` (the batch window and the snapshot window differ by the first batch and the last diag
  READ), `data_tx_lost_mb` 0, `tx_queued_anyway` 0. Unverified until then: the mailbox prediction and the register reads
  on the real FlexCAN, and the table's run time on the board.
