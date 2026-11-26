---
title: "BUG-147: the firmware flasher's BEGIN and COMMIT waits now take only a reply carrying their own opcode (0x01 / 0x04); a frame of any other opcode is skipped against the same overall deadline, so it can neither be read as the verdict nor extend the timeout"
type: bugfix
date: 2026-11-27
status: complete
phase: "BUG-147"
related_issues: [BUG-147]
files_changed:
  - tools/teensy_link_bridge.py
  - tests/teensy_link/test_fw_update_begin_retry.py
  - tests/teensy_link/test_rpc.py
---

# Flasher BEGIN / COMMIT wait on their own opcode (BUG-147)

**What and why.** `_begin_session` and the COMMIT step called `waiter.wait(reply_id)` and decoded whatever frame it returned. VERIFY and the DATA window already filtered on the reply opcode (2026-11-19 fix); BEGIN and COMMIT did not, so any unsolicited frame on 0x6F1 / 0x7D7 inside those two steps would have been read as the verdict (a BUSY from a stray frame aborts BEGIN; a stray OK reads as "board is copying flash" before COMMIT answered). Nothing sends such a frame today (the Ball Butler FW 10 status frame sits on the command-result channel for this reason), so this closes the class rather than a live failure. Both steps now go through `_wait_for_op_reply` (arrival-order `take`, one overall deadline, one info line for the first skipped frame) with new `_FW_OP_BEGIN` / `_FW_OP_COMMIT`. The helper gained an injectable `monotonic` so BEGIN's fake-clock tests keep working.

**Control walk-through.** Host-side flasher only, no robot motion, not on the leg path. Per attempt: clear, send BEGIN, read frames until opcode 0x01 or the 1 s `_FW_REPLY_TIMEOUT_S` (unchanged); PARKING re-send, silence retry and refusal handling are untouched. Wire bytes unchanged.

**Tests.** `tests/teensy_link/test_fw_update_begin_retry.py`: wrong-opcode BUSY frames before the real BEGIN OK are ignored (BEGIN succeeds); the same for COMMIT through a full tiny-image session; a stream of wrong-opcode frames every 0.4 s times out within the 1 s deadline like silence. The two skip tests fail on the old code (BEGIN and COMMIT return the stray BUSY as a refusal); the timeout test uses the new `monotonic` argument, so it guards the deadline against a stream of stray frames from here on rather than pinning old behaviour. Both receivers echo the op on every BEGIN and COMMIT reply, PARKING and PARK_FAILED included, so no legitimate answer is skipped: Platform `Teensy_code_platform.ino:768` and its `OP_COMMIT` case; Ball Butler (BallButler repo) `ball_butler_main/FwUpdate.cpp:189,351,360,445`. The scripted waiter in `test_rpc.py` and the one in the begin-retry file gained `take` (the helper reads arrival order).

**Verification.** 2026-11-27, `night_shift_nonet.sh timeout 900 python -m pytest tests/teensy_link/test_fw_update_begin_retry.py tests/teensy_link/test_rpc.py tests/teensy_link/test_fw_update_window_convergence.py -q -p no:cacheprovider` (run against the BUG-147 worktree by absolute path with `--rootdir`): **97 passed in 24.13 s**.
