---
title: "BUG-130 can-bridge FW 51: additive ACTIVATE_TO (0x66) raises the Platform to six caller Leg targets, each within 0.10 rev of the constant Active pose, so ACTIVATE can land on the Levelled Active pose; firmware band, teensy_link codec and the bridge's _run_activate path built and tested; the Jetson caller (trajectory_node targets, orchestrator refusal) designed, NOT wired; FW 51 NOT flashed"
type: feature
date: 2026-11-25
status: in-progress
phase: "BUG-130"
subsystem: can-bridge
related_issues:
  - BUG-130
  - TKT-92
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_activate.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_activate.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - teensy_link/protocol.py
  - teensy_link/rpc_args.py
  - teensy_link/rpc.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/firmware/native/test_leg_activate.cpp
  - tests/firmware/native/test_rpc_dispatch.cpp
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/firmware/test_bridge_fw_version_xref.py
  - tests/teensy_link/test_activate_to_codec.py
  - tests/ros/test_teensy_bridge_node_activate.py
---

# BUG-130: ACTIVATE_TO, the firmware half of "ACTIVATE lands on the Levelled Active pose"

**What.** The owner answered TKT-92 with option 4 (ACTIVATE lands on the levelled neutral) and ruled on 2026-11-24 that ACTIVATE is REFUSED when no levelling correction is available. This entry builds the firmware enforcement point and the host primitives under it, and designs (but does not wire) the Jetson caller. FW 51 sits on FW 50 (observability only) as its own version so the owner can verify the motion change separately. **This branch must not merge to main before FW 51 is flashed**: the bridge node would then hold an ACTIVATE_TO path an FW 50 board answers ERR_UNKNOWN_METHOD (harmless, but the merge rule is flash first).

## Discussion (the design, written before the code)

**(a) The Levelled Active pose.** The correction is built only in `trajectory_node` (C-LEVEL-1, `ros_ws/docs/levelling_frame.md`): `_corrected_neutral_pose()` (ingest site E5/E6) composes the stored offset (raw `/gravity_offset` + `level_trim_deg`) with the Tilt map residual keyed at the neutral's own (0, 0), where the home-referenced residual is about 0 by construction, and rotates the neutral `[0, 0, 170, 0, 0, 0]` about its own centre (rotation only; position untouched). The Legs follow by IK and `motor_rev = ext_mm x GEOM_MM_TO_REV`, the emitter's convention (`IK(neutral)` equals `JB_OP_ACTIVATE_POSITION_REVS` to 0.0). Measured by an IK probe (`$S/out/bug130/probe_levelled_neutral.py`, 2026-11-25) at the BUG-130 effective offset (-0.044, +0.574) deg: per-Leg deltas `[-0.0248, -0.0042, -0.0001, +0.0265, +0.0247, -0.0225] rev` = `[-1.76, -0.30, -0.01, +1.88, +1.75, -1.59] mm`; worst Leg 0.0265 rev. Sensitivity is linear: the worst Leg moves 0.050 rev (3.55 mm) per degree of correction in the worst direction (0.6 deg: 0.030 rev, 2.0 deg: 0.100 rev, 3.0 deg: 0.150 rev).

**(b) Is a correction available at ACTIVATE, and does "refuse" block the bring-up?** No conflict, from the code: `level` is its OWN state, `IdleHandler` → `LEVELLING` → `IDLE` (`state_machine.py` `LevellingHandler`: `level_activate` → settle → `level_get_tilt` → `level_send_correction` → persist → mocap check → `level_deactivate`). It rises with its own ACTIVATE, measures, publishes `/gravity_offset`, and stows. The bring-up is Boot > Home > Level > Activate (`docs/agents/map-writes.md`; the runbook rows carry `setup: level`), so at the operator's ACTIVATE the correction is loaded; on a relaunch the orchestrator's persisted auto-push on the first IDLE usually restores it (7/7 on 2026-07-27), and when it does not, `level` is available from IDLE. Refusing ACTIVATE while `trajectory/status.gravity_correction_loaded` is false therefore asks the operator to Level first, which the bring-up already does. Two consequences that bind the design: **`level_activate` must keep the constant ACTIVATE** (the offset is defined as the tilt measured while commanded to the uncorrected neutral; measuring at a levelled pose would read about 0 and overwrite a good offset with a wrong one, a feedback loop), and the "available" test must be C-LEVEL-1.O's, `gravity_correction_loaded` fresh from the applier, never `RobotState.levelling_complete`. A Tilt map is NOT required: none is committed in `config/`, so requiring one would refuse every ACTIVATE (question 1 for the owner).

**(c) One firmware enforcement point.** `activate_to_request` (leg_activate.cpp) accepts six targets only if each is finite and within `ACTIVATE_TO_BAND_REV = 0.10 rev` (7.1 mm, a 2.0 deg correction, 3.5x the measured 0.574 deg) of `JBOp::ACTIVATE_POSITION_REVS[i]`, and all six Legs are present; otherwise ERR_BAD_ARGS, nothing latched, never clipped. Then exactly ACTIVATE's gates through one shared `activate_gate_and_latch`. The largest legal Jetson request (raw offset up to about 0.8 deg plus the 1.0 deg per-axis trim ceiling) can reach about 0.11 rev; there the refusal is the right answer (a correction over 2 deg at neutral means the Level is wrong). **Homed precondition: parity with ACTIVATE, no new firmware check.** The brief asked for "all six Legs homed"; today's ACTIVATE has no firmware homed gate, and the can-bridge's homing record is per bridge boot, while `is_homed` lives in the Platform Teensy and survives a bridge power cycle (the `bridge-cycled` setup). A firmware homed gate would refuse ACTIVATE_TO at every bridge-cycled Sitting where ACTIVATE succeeds. The Jetson's homed gate (BootHandler forces HOMING when `is_homed` is false) covers both paths alike.

**(d) What else assumes Active pose == the constant.** Firmware: only `activate_target_rev` (COMMAND and MONITOR) reads `ACTIVATE_POSITION_REVS`; it now reads the latched set, so MONITOR judges arrival against the targets actually sent. DEACTIVATE (`leg_deactivate.cpp`) does not read the constant; it TRAP_TRAJs from the measured position to the stow pose. The Hand park is untouched. Host: `_run_activate`'s ActivateMonitor targets (now the latched set); `trajectory_node`'s seed is the measured encoders (C-LEVEL-1 forbids correcting it), so after a levelled rise the hold is Level with no code change, and the arm is bumpless because the hold is seeded from the same measurement; the pre-seed `_neutral_pose` fallback is unreachable after the first seed. Sitting limits and the Admissible box bound poses, and a 0.6 deg tilt at neutral is far inside them. The three artefacts that pin "commanded level ⇒ activate revs" (levelling_frame.md § Consequences) stay true: they are IK/emitter statements. `levelling_frame.md` § Consequences "The Active pose held straight after ACTIVATE is NOT Level" and "The first go_home after level is a real move" become false once the caller is wired; that edit belongs with the caller commit.

**(e) Control-cycle walk.** The rise: the firmware's SETUP seeds `input_pos` at each Leg's measured hardstop position, COMMAND sends one `encode_leg_setpoint(target)` per axis, and the ODrive's TRAP_TRAJ profiles from about -0.10 rev to the target at `GENTLE_MOVE_VEL_LIMIT_RPS` (2.5 rev/s). A levelled target differs from the constant by at most 0.027 rev on a 2.3 rev move: the six Legs finish within about 11 ms of one another instead of together (0.027 rev at 2.5 rev/s), so the Platform passes through tilts no larger than the 0.6 deg it ends at; no step anywhere, the profile is the same. No 40 Hz stream runs during it (`leg_interp`'s coldstart interlock). The first seed after it: `trajectory_node` seeds from the encoders (now the levelled pose), its STANDBY hold emits that pose, the arm compares the stream against the same measurement, so u0 equals the measured Leg positions to the encoder dead-band: no knot moves. The first `go_home` then plans from the levelled pose to the corrected neutral: a near-zero move instead of today's 2.7 mm one.

**What was ruled out.** Teaching `teensy_bridge_node` to compute the correction itself (a second applier, forbidden by C-LEVEL-1 and C-LEVEL-1.O); an automatic `go_home` after ACTIVATE (option 3, not chosen); clipping out-of-band targets into the band (hides a host bug).

## Fix (this commit)

- Firmware FW 51: `ACTIVATE_TO` 0x66 (`ArgActivateTo`, six f32, 24 B, generated); `activate_to_request` + `activate_to_target_in_band` (leg_activate.h/.cpp); one shared gate-and-latch for ACTIVATE and ACTIVATE_TO; the latched Leg target set read by COMMAND and MONITOR; `ACTIVATE_TO_BAND_REV = 0.10f` (canbridge_config.h); FW_VERSION 51. PROTOCOL_VERSION stays 14.
- teensy_link: `encode_activate_to`, `ACTIVATE_TO_BAND_REV` twin, `activate_to_out_of_band`; ACTIVATE_TO is non-idempotent (never blind-retried).
- teensy_bridge_node: `teensy_activate_to`; `_run_activate(..., leg_targets_rev=)` refuses by name with no RPC unless all six Legs and every target in band, then sends ACTIVATE_TO and monitors arrival on the sent targets. `None` is today's ACTIVATE, unchanged. Nothing calls it yet.

## Not done (the caller; needs the owner's answers below, then one unit)

`trajectory_node` service returning the six levelled Leg revs from `_corrected_neutral_pose()` (refusing when `gravity_correction_loaded` is false); the `ActivateOrDeactivate` request carrying them (an additive `float64[] leg_targets_rev`; empty = constant); `ActiveHandler`'s 'activate' (never `level_activate`) using them; the refusal staying in IDLE with a named WARN instead of FAULT; `levelling_frame.md` new E-site and § Consequences edits; tests.

## Verification

All 2026-11-25, in the worktree on branch offline/bug130:

- Native (`flock /tmp/jugglebot-tests.lock python tests/firmware/native/build.py`, then each binary): `test_leg_activate` **53/53 cases, 599/599 assertions** (six new ACTIVATE_TO cases: band edges both signs on every Leg, one Leg out of band / NaN / inf / a stow pose / mm-as-rev, a Leg absent, ACTIVATE's gates and busy latch, COMMAND sends the six targets plus the unchanged Hand park and MONITOR finishes only on them, ACTIVATE after ACTIVATE_TO re-latches the constants byte for byte); `test_rpc_dispatch` **38/38, 309/309** (args reach `activate_to_request` intact, its status is the reply, a short arg never calls it); `test_leg_deactivate` 14/14.
- Fails without the change: `activate_target_rev` reverted to the constants (built alone against the cached objects): **2 of 53 fail** (the COMMAND/MONITOR case and the re-latch case). In the bridge test, `_run_activate` monitoring the constants instead of the sent targets: `test_activate_to_sends_the_six_targets_and_monitors_arrival_on_them` **fails**; restored, 15/15 pass.
- Scoped Python (`flock /tmp/jugglebot-tests.lock python -m pytest -q -n 4 tests/teensy_link tests/firmware tests/ros/test_teensy_bridge_node_activate.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_docs_links.py`): **1221 passed, 4 skipped, 3 failed**: `test_rpc_args.py::test_method_arg_association_covers_all_commandable_methods` (fixed after: `METHOD[ACTIVATE_TO]`; `pytest -q tests/teensy_link/test_rpc_args.py tests/teensy_link/test_activate_to_codec.py` **43 passed**) and the two `test_logbook_index.py` cases for this entry's INDEX row, which the orchestrator's `mapkit gen` writes at integration.
- Wire layout digest re-pinned (`test_udp_protocol_xlang.py::test_wire_layout_frozen`, additive arg struct only).
- `pio run -e teensy41` (in `Teensy_code_canbridge/`): **SUCCESS** in 45.9 s. **Not uploaded.**

IK probe for (a): `$S/out/bug130/probe_levelled_neutral.py` (scratchpad, uncommitted), output in `probe_out.txt` beside it.
