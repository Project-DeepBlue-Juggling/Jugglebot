---
title: "Platform firmware-over-CAN: the uploader derailed into two BAD_SEQ rewinds per window and read a DATA straggler as VERIFY's verdict; the host now reads every window reply in arrival order, acts on the highest lower bound after a settle time, and matches VERIFY by opcode"
type: investigation
date: 2026-11-19
status: in-progress
phase: "platform-imu P3"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - tools/teensy_link_bridge.py
  - teensy_link/rpc.py
  - tests/teensy_link/test_fw_update_window_convergence.py
  - tests/teensy_link/test_rpc.py
subsystem:
  - tools
  - can
tags:
  - safety
  - testing
---

# Platform firmware-over-CAN: rewind derail and a straggler read as VERIFY

## Summary

Three `--verify-only` runs of the 88064-byte FW 11 image against the Platform Teensy (FW 10 receiver), with can-bridge
FW 45 and again with FW 44, all failed the same way: isolated BAD_SEQ rewinds, then from one of them on two rewinds per
16-frame window to the end of the image (923 and 1657 rewinds), then `VERIFY — BAD_SEQ`. The derail and the VERIFY
failure are host defects in `tools/teensy_link_bridge.py`, reproduced frame for frame by a virtual-time model of the FW 10
receiver and the bridge's reply uplink, and fixed. The isolated events that start each derail are a separate,
still-open question: the logs carry the signature of reordered and duplicated DATA frames, not only lost ones. Nothing
was run on hardware in this entry.

## Symptoms

From `verify3.log` (bridge FW 45) and `verify_fw44.log` (bridge FW 44), both 2026-11-19, kept in that session's
scratchpad only:

- BEGIN OK; isolated `BAD_SEQ — board expects seq=N` at arbitrary N (445, 1534; 205, 1283), each recovered.
- Then the same N twice about 0.5 s apart (2455, close to the sector-3 filler frame 2457, so the 0.5 s pause sits inside
  that window), and from there two rewinds per window: `2464, 2464, 2480, 2480, ...`.
- The doubled runs end only at windows whose LAST frame fills a sector (4080..4095, 12272..12287, 16368..16383).
- The last report is `expects seq=17613` (the frame count), DATA "complete, 0 window retries, 0 missing acks", then
  `VERIFY — BAD_SEQ`.
- `verify_fw44.log` also shows `N, N+1, window end` triples: 3901/3902/3904, 5076/5077, 6977/6978, 9461/9462/9472,
  10994/10995, 14410/14411; `verify3.log` has 17561/17562/17568.

## Diagnosis

**The reply path is late and batched.** The board answers on 0x6F1; the bridge pushes each reply into an 8-slot ring
(7 usable, newest dropped when full, `can_buses.cpp` `platform_ring_push`) that `platform_uplink_step` drains 4 frames
per 10 ms tick. The host sends one DATA frame per 1 ms bridge tick, so a 16-frame NAK burst comes back over 20-30 ms,
and its tail lands after the host has finished reading and started the next window.

**The derail.** `_wait_for_window_reply` accepted any BAD_SEQ naming a frame at or after the window's start. A straggler
naming exactly the next window's first frame passes that filter. Frame by frame, from the first double at 2455:

1. Window [2448..2464): 2455 does not arrive as sent. The board NAKs 2456..2463 with "expects 2455"; the host reads one
   and rewinds.
2. Window [2455..2464): the host sends 2455..2457, pauses 0.5 s after the sector filler 2457, sends 2458..2463. The
   board accepts all of them and ACKs 2463. The first burst's tail ("expects 2455") arrived during the pause, after the
   window's `clear()`. The host reads it before the ACK and rewinds to 2455 again: the second "2455".
3. Resend [2455..2464): the board now expects 2464 and NAKs all nine frames. The host reads "expects 2464", which names
   the window end, so it moves on: the first "2464".
4. Window [2464..2480): the board accepts it all, but the tail of step 3's burst ("expects 2464") lands first. The
   window's start is 2464, so the filter takes the straggler as the verdict and the host rewinds an accepted window: the
   second "2464". The resend provokes 16 NAKs "expects 2480", whose tail derails window [2480..2496), and so on.

The loop breaks only where the window's last frame is a sector filler: the host's 0.5 s pause comes after the last frame,
so every straggler has landed before the board's post-flush ACK, which is then the latest reply. This matches the three
exits in the logs.

**VERIFY.** The last window ended on "expects 17613" from a resend burst. That is true (the board held every frame), so
DATA ended correctly. VERIFY then cleared the slot, sent, and read the next straggler of that burst: op 0x02, status
BAD_SEQ. The FW 10 VERIFY handler (`git show 1814578d:...Teensy_code_platform.ino`, `case OP_VERIFY`) answers only
BAD_STATE (no session, or `staged() != image_len`), FLASH_ERR, BAD_CRC, BAD_IDENTITY or OK, always with opcode 0x03. It
has no BAD_SEQ path, so the `VERIFY — BAD_SEQ` line is a stale DATA reply. The board's own VERIFY answer, most likely
OK, came after it and was never read. The `namespace FwUpdate` block is byte-identical in FW 10 and in FW 11 on `main`
(318 lines, empty `diff`). FW 11 changes only the loop around it: one frame per pass while a session is open, plus
the IMU service.

**The model.** `tests/teensy_link/test_fw_update_window_convergence.py` models the FW 10 receiver (DATA, ACK on
`seq % 16 == 15`, BAD_SEQ, 52 ms sector flush that drops frames and delays its frame's reply, VERIFY, COMMIT from
VERIFIED only), the 1 kHz RPC tick, and the 7-slot ring at 4 replies per 10 ms. It runs the real
`_run_fw_update_session` against that model on a virtual clock. On the unfixed tool, ONE injected loss at almost any
window phase reproduces the whole signature: `1603, 1603, 1616, 1616, 1632, 1632, ...`, about 150 rewinds on a
2798-frame image, then `VERIFY — BAD_SEQ`. 44 of the 48 new cases fail before the fix.

## Discussion

**Not acting on a NAK at once, rather than tightening the stale-reply filter.** No filter on a single reply can work:
"expects S" for the window start S is a real NAK when frame S was lost and a straggler when it was not, and the two are
byte-identical (the detail field, `staged()`, is identical too). The disambiguating fact is the window's own last reply,
which is always later. The receiver's expected seq is monotone within a session, so every reply is a lower bound on the
board's state that may be stale but is never too high. The host therefore reads every reply in arrival order. It stops
at once on a decisive one (the ACK of the window's last frame, or a NAK naming the window end). Otherwise, after a
settle time, it acts on the highest bound. Resuming at a lower bound is always safe, because the worst case is a resend
that the board NAKs. A stale bound can no longer be acted on while a fresher reply is still in flight.

**A queue, not the latest-reply slot.** `PlatformFrameWaiter.wait()` keeps only the latest reply, and the old reader's
`clear()` after a stale read could drop a fresh reply that landed in between. With the bridge delivering up to 4 replies
back to back per tick, that race is likely, not theoretical. `take()` adds an arrival-order queue. `wait()` and `clear()`
keep their meaning, so BEGIN, the version reads and COMMIT are unchanged.

**Settle time 0.1 s.** The reply to a window's last frame can sit behind up to 6 ring entries, which is about 20 ms plus
one 10 ms tick. 0.1 s is about 3x that. It is paid only when no decisive reply comes, which means a frame really is
missing on the board. That is about 0.1 s per fault, or roughly 1 s per flash at the observed rate.

**The isolated events are not plain losses.** In the model, each fault type leaves its own rewind signature under the old
tool: a loss gives `N, N, end, end`, a duplicate gives `N+1, end`, and an overtaken frame gives `N, N+1, ...`. The
fw44 log's seven `N, N+1, end` triples and its `N, end` pairs (3430/3440, 16071/16080 in verify3) cannot come from a pure
loss, because the board already held N..end-1 when the host resent them. So the brief's candidate (a), the board's loop
dropping RX frames, cannot be the whole story. Frames are also arriving reordered or twice. Two mechanisms fit:
- a same-id mailbox overtake on the relay's TX, which needs a frame to wait more than 1 ms on a 24 % loaded bus;
- FlexCAN_T4's duplicate-TX path (`.tpp:1086-1093`), on either board.

A host RPC retry does not fit. Each retry costs a 0.5 s ack timeout (`default_timeout=0.5, retries=1`), but
verify3's 42.8 s is fully accounted for without any: at most 17613 + 923 × 16 frames at 1 ms, plus 21 pauses × 0.5 s, is
about 42.9 s. The fix converges for all three fault types, at no more than one window per fault (a duplicate costs
nothing).

## Fix

- `tools/teensy_link_bridge.py` `_wait_for_window_reply`: reads replies with `waiter.take()`.
  - Returns at once on the last frame's ACK, on a NAK naming the window end, or on a refusal.
  - Skips replies about frames before the window, older ACKs, and non-DATA opcodes.
  - Otherwise returns the highest in-window NAK `_FW_REPLY_SETTLE_S` (0.1 s) after the window's last frame. It logs the
    window's reply trail at INFO, so the next hardware run records which fault type each rewind was.
- `_wait_for_op_reply`: VERIFY's verdict is the first reply with opcode 0x03. DATA stragglers are skipped.
- `teensy_link/rpc.py` `PlatformFrameWaiter.take()`: arrival-order queue (64 per can_id). `clear()` empties it too.
- Unchanged: BEGIN, the window send loop and its retry, the sector pause, every profile constant, and COMMIT and
  everything after VERIFY OK.

**Safety.** COMMIT is still sent only after a reply with opcode 0x03 and status OK. The FW 10 receiver sends that only
after flushing the tail, checking `staged() == image_len`, a CRC-32 over the staged flash that equals the host's, and the
identity marker, and it enters VERIFIED only then. Its COMMIT handler refuses (BAD_STATE) in any other phase. The new
code only decides which DATA frames to resend and which reply to read. The worst a bug there can do is resend too much,
stall into the window retry, or abort before COMMIT, which leaves the running image untouched and a staged session that
expires after 60 s. It cannot make the board accept a COMMIT for an image it has not CRC-verified.

## Outcome

`flock /tmp/jugglebot-tests.lock python -m pytest tests/teensy_link/ tests/firmware/test_bb_fw_update_xref.py
tests/ros/test_teensy_bridge_node_hand_jam.py tests/sim/test_night_shift_settings.py
tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_no_legacy_item_ids.py
tests/sim/test_docs_links.py -q`, run 2026-11-19: **847 passed, 4 skipped in 57.88 s**. Readers found with
`grep -rln "teensy_link_bridge\|PlatformFrameWaiter\|teensy_link.rpc" tests tools ros_ws`. The new file alone: 48
passed after the fix, 44 failed and 4 passed before it (the passes are the controls: a clean transfer, a corrupted image,
and faults at window phases the old tool happened to survive).

Not yet flown. Next step: one `--verify-only` run with the fixed tool. Expect a few dozen rewinds at most and VERIFY OK.
Each rewind's INFO trail shows which fault type it was.

## Withdrawn claims

- [2026-11-19] The brief framed the isolated events as "the isolated loss every 1 to 2 s".
  WITHDRAWN: the `N, N+1, end` triples and `N, end` pairs cannot come from a lost frame (see Discussion).
  Superseded by: the events are reordered or duplicated frames, and some may still be losses; the type is open.
- [2026-11-19] "VERIFY returns BAD_SEQ."
  WITHDRAWN: the FW 10 VERIFY handler has no BAD_SEQ path; the host read a DATA straggler (Diagnosis).

## Open Questions

- What reorders or duplicates the DATA frames? Candidates: the relay's same-id mailbox overtake on CAN2, FlexCAN_T4's
  duplicate-TX path, or the Platform's RX. The fixed tool's per-rewind trail separates them: a trail of `N` then `N+1`
  NAKs means a reorder, all `N` means a loss, and a duplicate shows no rewind at all. A passive capture of 0x6F0 on the
  Jugglebot bus would settle it outright.
- Does the latched ODRIVE_FATAL change what the bridge transmits on CAN2? That would matter only through the mailbox
  overtake.
- COMMIT still reads its reply from the latest-reply slot. A late DATA straggler there would misreport a COMMIT, but only
  host-side: the board decides by itself. It was deliberately left unchanged.

## On hardware, 2026-11-19 (launch down, E-stop button pressed, 48 V off)

Run by the orchestrating Session with this branch's tool, before the branch was pushed (the board checks the CRC-32
itself, so an unpushed host tool cannot commit a bad image):

- 18:16, `python tools/teensy_link_bridge.py --fw-update <FW 11 firmware.hex> --verify-only` against Platform FW 10,
  can-bridge FW 45: BEGIN OK, **88064 B in 32.6 s, 0 rewinds, 0 window retries, 0 missing acks, VERIFY OK**
  crc32 0x235796D4. The old tool failed this rehearsal three times the same hour (923 and 1657 rewinds, VERIFY BAD_SEQ).
- 18:18, the same command without `--verify-only`: 0 rewinds, VERIFY OK, COMMIT OK, receipt
  **`Platform Teensy FW version: 10 -> 11`**.
- 18:19, 20 s read-only listen: 20 of 20 BridgeIdentity (45, 14); 20 HAND_FAN_STATUS 0x6E2; 18 IMU_STATUS 0x6E5 with
  state PROBING, SENSOR_OK clear and i2c_errors rising 0 to 5 on the back-off schedule (no IMU is wired yet).
- 18:19, `--verify-only` again, now against FW 11: 0 rewinds, VERIFY OK, and IMU_STATUS read FW_HALTED with i2c_errors
  held at 5 for the transfer. So FW 11 accepts a firmware update and stops its I2C while one runs.

Not observed in any of these three runs: the periodic bad frame that started each derail. Its cause is still open.
Not verified: tilt request, Hand fan control, anything with the launch up or the robot Armed (Rung P4, IDEA-20).
