# Bench session — FW 18 flash (hand clip, homing restore, counter gate, hand7 reset)

Flash-and-verify checklist for the FW 18 bundle
(`plans/archived/unified-7dof-planner.md` § "FW 18 bundle",
`logbook/2026-09-08-fw18-bundle-hand-clip-homing-counters-rename.md`). PROTOCOL_VERSION
stays 6 — wire-identical to FW 17. Can-bridge FW 18 is **required** (the
hand-lane fixes below); Platform FW 4 is **optional** (its only change is the
smooth-move ceiling 10.6 → 10.501 — see the Platform section below). No
planner work depends on either landing; the can-bridge flash closes four bench
findings from the FW 17 sitting.

## What changes for the operator

1. **The hand clip moves off the metal.** Commanded hand setpoints are now
   clipped to **10.501 rev**, not 10.8. The metal itself is unchanged (measured
   10.701 rev, 2026-09-06). Nothing in the normal repertoire goes near either
   number (catch prime 9.959, retract 0.0) — this only matters if something is
   already wrong.
2. **`HOME(6)` now leaves axis 6 in POSITION/PASSTHROUGH on the shipped hand
   limits (1000 rev/s / 50 A), and fails the home if it can't.** Before FW 18,
   the hand could be left in VELOCITY/VEL_RAMP after a home — a mode an ODrive
   silently swallows `set_input_pos` in, so the streamed lane would look fine
   on the wire and do nothing. Expect telemetry's per-axis `ctrl_mode` /
   `input_mode` fields to read `3` / `1` (POSITION / PASSTHROUGH) for axis 6
   after any home, and a `HOMING_FAILED` (rather than silence) if the restore
   itself fails.
3. **`[hand7]` counters (`lead=`, `dev_over=`) now zero across an aborted
   stage**, instead of staying non-zero forever once a stage transmits nothing.
   The runbook's "non-zero `lead` during a throw ⇒ hard-abort" rule can now be
   read as an absolute rather than a difference across the stage.
4. **New console verb: `hand7 reset`.** Zeroes every `[hand7]` counter and
   residual (`sent`, `discard_legacy`, `unseen`, `stale`, `lead`, `dev_over`,
   `dev_last`, `dev_max`, `dev_cmd`, `dev_fb`) without a Teensy reboot — so a
   stage can be read as absolute without losing the boot-cumulative uptime
   state a sitting often exists to interrogate. It does **not** touch the
   observe/arm switch, the `hand_source` latch, or the lane's knot state.
5. **The fault console/GUI name is `SETPOINT_STALE`, not `MPC_STALE`.** Same
   mechanism (the 40 Hz setpoint stream going stale), renamed because the MPC
   chain was removed 2026-09-01. If you see `MPC_STALE` anywhere after this
   flash, the board did not take the flash.

## Pre-flight

- [ ] `git log --oneline -1` on the tree you are about to build from shows the
      FW 18 bundle commit (this session's commit, `Logbook-Entry:
      2026-09-08-fw18-bundle-hand-clip-homing-counters-rename`).
- [ ] No other session holds the bench (`ps` for a live `teensy_bridge_node` /
      `hand_stream_bench.py`; check with whoever else might be on this branch).
- [ ] **Whole sitting is BENCH. NO BALL anywhere near the hand.** E-stop within
      reach and tested before arming.

## Flash — can-bridge Teensy (do this first)

1. Plug the can-bridge Teensy 4.1 into the Jetson (or the dev box) by USB.
2. `cd ros_ws/src/jugglebot/Teensy_code_canbridge && pio run -e teensy41 -t upload`
   — the `-t upload` is what flashes. A bare `pio run -e teensy41` **only
   builds**; a matching hex md5 is **not** a flash receipt.
3. **The boot banner is the only receipt.** Open the USB serial console and
   read the `[boot] jugglebot-canbridge vNN` line — it must say **18**. From a
   running system, `/link_status`'s `bridge_fw_version` (the 1 Hz
   BRIDGE_IDENTITY frame) says the same thing.
4. **A healthy link is not a receipt.** PROTOCOL_VERSION stays 6, so an
   unflashed (FW 17) board and this host tree still talk perfectly in both
   directions — there is no link darkness to tell you the flash is
   outstanding, unlike the 16→17 lockstep flash. The only symptom of a stale
   flash is one advisory log line per launch: `BRIDGE_FW_CHECK: FAIL —
   can-bridge Teensy reports v17, OLDER than the v18 this host tree expects.`
   The host logs it and carries on (commands are never refused).
5. Rollback, if ever needed, is an ordinary reflash of the FW 17 image — no
   host checkout change, because the wire did not move.

## Flash — Platform Teensy (optional, only for the smooth-move ceiling fix)

The hand-clip re-measurement also moved the Platform Teensy's compiled
`SMOOTH_MOVE_POS_CEIL_REV` 10.6 → 10.501 rev (behavioural — see
`ros_ws/docs/platform_fw_version.md`'s bump rule). This is **FW 4** on the
Platform Teensy, independent of the can-bridge's FW 18. Nothing on the
streamed hand path depends on it, and the Platform Teensy's own stroke engine
is legacy-path only — skip this if you only want the can-bridge fixes above.

1. **Arduino IDE only** — the Platform Teensy's `pio` image is CAN-MUTE; do
   not flash it via `pio run -t upload`.
2. Flash `ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino`.
3. Receipt: serial boot banner `[boot] jugglebot-platform v4`, or
   `platform_fw_version` on `/link_status` reading `4` after the next
   authoritative read (boot, a UDP reconnect, or a CAN3 recovery). A
   `PLATFORM_FW_CHECK: FAIL` in the launch log names a skew; it is advisory
   only (warn, never refuse).

## Dry-run rehearsal (before power)

Per CLAUDE.md, rehearse before running for real:

- [ ] `python3 tests/hardware/unified_cycle_bench.py --rung throw --dry-run`
      (no ROS calls) to confirm the CLI and the generated constants it reads
      (`HAND_MOTOR_MAX_POSITION_REV` should now print 10.501) resolve.
      (`hand_stream_bench.py` has no `--dry-run` and no
      `HAND_MOTOR_MAX_POSITION_REV` — its constant is `HAND_MAX_POS` = 10.701,
      the metal, at `tests/hardware/hand_stream_bench.py:181`.)
- [ ] `python3 tests/hardware/hand_stream_bench.py --help` to confirm the CLI
      resolves.
- [ ] Re-read "What changes for the operator" above so the expected telemetry
      and console strings are fresh before they appear live.

## Bring-up check (after flash, launch up)

1. `hand_stream_bench.py --source-only streamed` — switches the firmware
   `hand_source` latch to STREAMED (a latch verb, exempt from the energisation
   half of the pre-check).
2. Home the hand (`HOME(6)`, or the normal launch home sequence). Confirm:
   - [ ] The home reports success (not `HOMING_FAILED`).
   - [ ] Telemetry's axis-6 `ctrl_mode` == 3 (POSITION), `input_mode` == 1
         (PASSTHROUGH).
3. On the Teensy console, `hand7` (no argument) prints the status line:
   `[hand7] src=STREAMED guard=observe lane=... sent=... discard_legacy=...
   unseen=... stale=... lead=0 dev_over=0 dev_last=... dev_max=... dev_cmd=...
   dev_fb=...` — confirm `lead=0` and `dev_over=0` at rest.
4. `hand7 reset` — confirm it echoes the same status line format with every
   counter and residual zeroed, and that `src=` / `guard=` are unchanged (a
   reset never touches the source latch or the arm state).
5. Grep the launch log for `SETPOINT_STALE` (should appear where `MPC_STALE`
   used to) and confirm `MPC_STALE` does not appear anywhere in fresh output.
6. If you also flashed the Platform Teensy: confirm `platform_fw_version` on
   `/link_status` reads `4` after the next authoritative read (see above).

## Behavioural deltas to expect once flown

- Commanded hand setpoints between 10.501 and 10.701 rev are now clipped
  (FW 17 passed them). Nothing in the normal repertoire goes near it.
- After `CLEAR_ERRORS` on an armed hand guard, a re-latch takes one extra
  10 Hz poll (the tick right after the clear runs output-suppressed and
  counts nothing — the gate working, not a regression).
