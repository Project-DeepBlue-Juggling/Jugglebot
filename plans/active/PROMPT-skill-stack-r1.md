# R1 — one hand master (two-ball skill stack)

You are a fresh Fable 5.1 session implementing rung R1 of `plans/active/two-ball-skill-stack.md` on the
Jugglebot repo. Work ONLY in the worktree `~/Desktop/Jugglebot-skills` (branch `skill-stack`); never
touch `~/Desktop/Jugglebot` (a parallel firmware-validation session owns it). Start with
`cd ~/Desktop/Jugglebot-skills && git fetch && git status -sb` and expect a clean tree at or after the
R0-close commit; if it is not clean, stop and ask. Use the venv: `source ~/Desktop/PDJ_venv/venv/bin/activate`.

## Read first, in this order (excerpts, not whole files)
1. `plans/active/two-ball-skill-stack.md` § 0 (values — normative), § 1.3, § 2.1, § 2.6, § 4 "R1", § 7.
2. `ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md` — every row naming a path you delete
   must be re-pointed or marked RETIRE@R1 with a reason, in the same commit.
3. `tests/hardware/session_unified7_hand_bringup.md` rows 12–21 and their Results (what the FW 17 hand
   lane proved, the idle-axis trap, the `--close-loop` energise route, the rest bands, the arming row 18
   closed on thermal grounds).
4. Firmware: `Teensy_code_canbridge/{hand_source.h,hand_source.cpp,hand_ops.h,hand_ops.cpp,leg_interp.h,
   canbridge_config.h,udp_protocol.h,rpc.cpp,leg_homing.cpp}`; `Teensy_code_platform/Teensy_code_platform.ino`
   header comment + `Trajectory.h`; `tests/firmware/native/` (the native harness you must keep green).
5. Host: `teensy_link/{rpc.py,rpc_args.py,protocol.py}`, `config/generate_udp_protocol.py`,
   `ros_ws/src/jugglebot_interfaces/srv/SetHandTrajCmd.srv`, `motion/trajectory/hand_stroke.py`,
   `motion/trajectory/throw_envelope.py`, `ros_ws/docs/{hand_throw_envelope,hand_decel_feedforward,
   platform_fw_version}.md`, `tests/hardware/hand_stream_bench.py`.
6. Logbook: `2026-09-02-unified-7dof-planner-phase3-fw17-hand-lane.md`, `2026-09-04-fw17-hand-sitting-
   unflashed-idle-axis.md`, `2026-09-05-fw17-hand-ladder-sitting-three.md`, `2026-09-08-fw18-bundle-hand-clip-
   homing-counters-rename.md`.

## Hardware facts you cannot derive from the tree (verify the live ones before acting)
- The can-bridge board ran FW 19 on 2026-09-09 with FW 20 built but possibly unflashed; the Platform Teensy
  runs FW 6. PROTOCOL_VERSION is 6 at both ends. Read the live versions off the GUI Hardware panel /
  `link_status` boot banner at the sitting and number R1's firmware from there.
- A host and a bridge on different PROTOCOL_VERSIONs are DARK (`link=NO_HEARTBEAT`, `decode_errors ==
  rx_frames`) — that is the intended failure on skew, not a cable fault. R1 bumps 6 → 7.
- Flashing: bridge `pio run -e teensy41 -t upload`; Platform `pio run -e teensy40 -t upload` OVER CAN with the
  launch DOWN (its USB port is dead). A bare `pio run` only builds; the boot banner is the only receipt.
- `hand_source` resets to LEGACY on every bridge reboot; `leg_homing.cpp:195` leaves axis 6 in VELOCITY /
  VEL_RAMP where CLOSED_LOOP silently swallows the stream; the hand at rest is in the retract band
  [−0.20, +0.10] rev or catch-prime 9.9594 ± 0.10; the hand E-STOP trip has never been observed on
  hardware. Guard constants: trust `canbridge_config.h`, not any number in this prompt.
- The operator runs every actuating command (flash, energise, ladder). You write runbooks; you never run them.

## Deliverables (plan § 4 R1), as separate agent units each under ~80 tool calls
1. **Can-bridge firmware** (Opus, one unit): delete the `hand_source` latch and both modes — the hand lane is
   active whenever a `HAS_HAND` frame is latched, the falling-edge decay stays normative; delete `hand_ops`, the
   `HAND_TRAJ_CMD` and `HAND_SOURCE_SET` RPCs and the 0x6D0 forward; fix homing so axis 6 leaves in the
   streamed lane's controller/input mode; PROTOCOL_VERSION 7 in `config/generate_udp_protocol.py` then
   `python config/generate_config.py --no-external`; native tests updated and green, PLUS the SUBCASE
   INVARIANTS.md Gaps 5 asks for (age the mono clock past `SETPOINT_STALENESS_US`, assert `SETPOINT_STALE`
   and the ESTOP latch); FW_VERSION bumped with a dated header note.
2. **Platform firmware** (same or second Opus unit): delete `Trajectory.h`, the 0x6D0 decode, the 0x0C9
   hand-encoder cache and the stroke-engine constants; keep the SCL3300 inclinometer, the time-sync slave and
   the 0x6E0 cold-start state; FW version bumped; `ros_ws/docs/platform_fw_version.md` updated.
3. **Host** (Sonnet, split in two units if it passes ~80 calls): delete `SetHandTrajCmd.srv` and every client
   of it (enumerate with `grep -rlE 'SetHandTrajCmd|set_hand_traj_cmd|HAND_TRAJ_CMD'` over `ros_ws/src`,
   `teensy_link`, `ros_ws/gui`, `tools`, `tests` — do not trust a count from this prompt; the live nodes are
   `teensy_bridge_node.py`, `reload_coordinator_node.py`, `catch_coordinator_node.py`, `toss_sequencer.py`),
   the `hand_ops` client path in `teensy_link/{rpc,rpc_args,protocol}.py`, the `--source-only` machinery in
   `hand_stream_bench.py`, and every `hand_source` / `[hand7] src=` / HeartbeatT2J bit-6 surface (enumerate
   with `grep -rlE 'hand_source|HAND_SOURCE|hand7'` over the same roots; it includes `unified_cycle.py`'s
   latch precondition text and `feasibility.py:941`); replace `hand_stroke.py` with ONE
   generated constant pair (`LINEAR_GAIN_REV_PER_M`, `HAND_HOMED_REST_FLOOR_REV`) in `hardware_config` and
   re-point its five surviving importers (`cup_cycle`, `cup_realize`, `feasibility`, `unified_cycle`,
   `throw_envelope`); strip `throw_envelope.py` of the stroke-engine timing model, keeping the physical limits
   (end stop, regen, torque, the measured coast ladder). Grep before, count to zero after.
4. **Flash runbook** `tests/hardware/session_skill_stack_r1_flash.md`: lockstep order (host build → bridge
   flash → Platform flash → relaunch), the darkness check, then the bench ladder rows 12–21 re-cut for a
   latch-less lane (row 12 becomes "no latch exists: HAS_HAND drives the lane"), every refusal reported at once,
   then one streamed self-toss through the existing unified path, caught, with no operator latch step. Boot
   banners and bag ids recorded.
5. **Owner decisions to surface BEFORE unit 3**, as one AskUserQuestion with a recommendation each:
   (a) the hand E-STOP arming policy (observe-first vs armed); (b) whether homing parks the hand at 0 rev;
   (c) **the reload consequence** — the Ball Butler reload's reactive catch stroke is dispatched through
   `set_hand_traj_cmd` (the dispatch is `catch_coordinator_node.py:1331-1341`; the `catch/unified_mode`
   gating context is `reload_coordinator_node.py:2427-2465`), so deleting
   the stroke engine leaves the reload's hand catch with NO master until R4 re-cuts it as a CATCH skill.
   Options: keep R1 lean and use operator ball placement as R3's reset (recommended: one path, no throwaway
   port), or bring the reload's catch forward to R1 as a LANDING window through the existing
   `trajectory/plan_cycle` path (UH-6 caught 6/7 self-tosses on it; a BB throw is another announced ball).
   Do not pick silently.

## Process rules (from CLAUDE.md; non-negotiable)
Sonnet by default, Opus only for the two firmware units; brief agents with excerpts of the plan and the files
the unit touches, never the whole plan; one logbook entry per change (short form); `./run_tests.sh --full`
before every commit (firmware natives included); commit software UNFLASHED with the `Logbook-Entry:` trailer,
push in the same response after `git fetch && git status -sb`; `/audit --unstaged` once, at the rung's end;
the plan's § 3 R1 row and § 6 ledger updated in the closing commit; a rung that has not passed its dress
rehearsal on the loaded Jetson is not on the runsheet. If `hand-geometry-correction`'s G3 bench has passed,
merge that branch before unit 3 (its gain becomes the new constant's value); if not, proceed and note it.

## Definition of done
Software committed and pushed unflashed with green gates; the runbook written; the three owner decisions
answered; then the operator's sitting: bench rows re-pass on the new firmware pair (banners recorded) and one
streamed self-toss is caught with no latch step. Close with the logbook entry carrying the (date, command,
result) triple, the plan rows updated, and a one-paragraph handoff for R2 at the end of that entry.
