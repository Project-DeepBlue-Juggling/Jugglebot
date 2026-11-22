---
title: "Pipelined IMU refill READs: in a hole-refill pass the next READ goes out after the block's own length on the board plus 4 ms (15 ms floor), not after its frames land; refill_merge_words 16 to 12; refill_s and bridge-minus-Jetson frame count in stats; Jetson only, unflown"
type: optimization
date: 2026-11-23
status: in-progress
phase: "IDEA-25 pipelined refill"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - teensy_link/platform_imu.py
  - ros_ws/docs/platform_imu_protocol.md
  - tests/teensy_link/fake_platform_imu.py
  - tests/teensy_link/test_platform_imu.py
---

# Pipelined IMU refill READs

**What.** IDEA-25 findings section 6, Jetson side only (no firmware, no bridge change). In a refill pass (pass 2 on,
anchors and samples) `UploadMachine.tick` sends the next refill READ `max(refill_spacing_s, count x gap +
refill_margin_s)` (15 ms, 4 ms) after the previous READ's RPC returned, without waiting for its frames. A pass still ends
serially, header, FW 14 diag and table READs and every first sweep are unchanged, and chunk re-reads after a CRC mismatch
run as before (their first sweep serial or chained, their refills pipelined like any refill). The stall guard counts
whole dry refill passes while pipelining. A late reply is matched to its own already-ended READ. New stats:
`reads_pipelined`, `refill_s`, the full `pass1_missing_seqs` (was cut at 500), `bridge_rx_frames`,
`bridge_rx_minus_jetson_rx`; the launch line gains `refill N s`. `refill_merge_words` 16 to 12. The worker also wakes
for the next pipelined READ, and `pump` now records the bridge counters for every batch, as `_run` did. `refill_pipeline`
stays code-level: the node's IMU parameters take a declare, a validator and a set-parameters branch each, not two lines.

**Why.** Refill READs cost about 60 ms each, one at a time: 6.9 / 10.0 / 17.0 s of captures 572D / 572E / 572F.

## Discussion: why the spacing cannot cut a block on the board

The spacing runs from the RPC's return, and its OK means the bridge has queued the READ on its in-order CAN TX path
(`platform_relay.cpp` `send_gated`). So READ n+1 reaches the Platform at least the spacing after READ n was queued, and the
Jetson's UDP jitter is outside the argument. A fresh block's first frame waits at most one gap. Each later frame is due one
gap after the previous one's due time, with a lag clamped to 2 gaps (`imu_proto.h` `commit_data`, `due_after_last`).
So block n ends at most `count x gap` after it arrives, plus any delay the catch-up did not recover. The 4 ms margin
covers that delay, the bridge TX queue wait and one Platform loop pass. If it does not, the board replaces the block
(`imu_proto.h` READ case: only a READ at the active block's end extends it). Frames already in TX mailboxes still go out
and the unsent rest becomes holes for the next pass. Each frame carries its `seq`, and the board returns to FROZEN when
the new block ends, so nothing is corrupted and nothing gets stuck. The fake board now models arrival jitter (FIFO)
and a per-block unrecovered pacing delay. With 3 ms of the two together no block is replaced. With 6.4 ms (over margin
plus one gap) blocks are replaced and the upload still completes bit-exact. Bus: blocks never overlap on the board, so
IMU_DATA stays at one frame per effective gap (Armed: about 1590/s at 63). Pipelining adds at most 67 IMU_CMD and 67
IMU_REPLY frames/s. The bridge's relay ring (8 slots, 7 usable, drained 4 per 10 ms tick) then holds at most about
3 entries.

## Verification

- (2026-11-23, `flock /tmp/jugglebot-tests.lock pytest tests/teensy_link tests/ros -k "platform_imu" -q`): **250 passed,
  4657 deselected in 35.46 s**.
- (2026-11-23, `flock /tmp/jugglebot-tests.lock pytest tests/firmware/test_platform_imu_protocol.py
  tests/ros/test_gui_imu_charts.py tests/ros/test_gui_imu_upload_panel.py tests/sim/test_docs_links.py -q`): **92 passed
  in 2.75 s**.
- SIMULATION (the real machine, fake board 13, batched, 25 ms each way, the real pass-1 hole sets;
  `scratchpad/idea25/sim_real.py`, 2026-11-23): wall 58.6 / 62.3 / 45.4 s before, 54.0 / 55.7 / 32.6 s after
  (572D / 572E / 572F); refill 6.4 / 10.1 / 17.2 s to 1.8 / 3.6 / 4.4 s; 0 blocks replaced. Merge 12 vs 16: same time,
  572E duplicates 1730 to 1406. The fake has no RPC cost, so treat these as best cases.

Unflown. At the next Sitting: an Armed whole-ring upload at gap 80 and 63, reading `refill_s`, READs, duplicates, CRC,
`hole_passes` (a block cut on the board shows as an extra pass) and `bridge_rx_minus_jetson_rx == 0`.
