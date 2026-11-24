---
title: "BUG-130 can-bridge FW 51: additive ACTIVATE_TO (0x66) raises the Platform to six Leg targets within 0.10 rev of the constant Active pose; the Jetson caller (trajectory_node targets service, IDLE refusal without a fresh loaded correction, orchestrator and bridge wiring) built and tested on the held branch; level_activate keeps the constant pose; FW 51 NOT flashed"
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
  - ros_ws/src/jugglebot_interfaces/srv/ActivateOrDeactivate.srv
  - ros_ws/src/jugglebot_interfaces/srv/LevelledActivateTargets.srv
  - ros_ws/src/jugglebot_interfaces/CMakeLists.txt
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/state_machine.py
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md
  - ros_ws/docs/levelling_frame.md
  - tests/ros/conftest.py
  - tests/ros/test_state_machine.py
  - tests/ros/test_orchestrator_node.py
  - tests/ros/test_orchestrator_conduit_contract.py
  - tests/ros/test_orchestrator_conduit_integration.py
  - tests/ros/test_trajectory_node.py
  - tests/ros/test_state_machine_deactivate_confirm.py
  - ros_ws/docs/choreography.md
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

## 2026-11-25 (later): the Jetson caller

**What.** The caller the Discussion designed, built on the same held branch with the orchestrator's four rulings: (1) refuse when `trajectory/status.gravity_correction_loaded` is false or stale, no Tilt map required, never `levelling_complete`; (2) no firmware homed gate; (3) a refusal stays IDLE with a WARN naming `level`; (4) the additive `leg_targets_rev` request field plus a trajectory_node service. Normative text first: `levelling_frame.md` gains ingest site **E9** and rule **C-LEVEL-1.A** (enforcement points and tests in its own table), and its two § Consequences bullets now say the held Active pose IS Level and the first `go_home` is about a no-op; `ARMING_CONTRACT.md` choreography gains step 0 (the IDLE decision) and step 1 says the rise carries the targets.

- `LevelledActivateTargets.srv` (new) and `trajectory_node._svc_levelled_activate_targets` (`trajectory/levelled_activate_targets`): `_pose_to_motor_rev(_corrected_neutral_pose())`, the emitter's chain, no new correction call (the C-LEVEL-1 manifests are unchanged). Refuses when `_gravity_correction_loaded` is false, and when any Leg leaves `ACTIVATE_TO_BAND_REV` (a module copy, pinned equal to `rpc_args` and `canbridge_config.h`), naming each Leg with its size in rev and mm. Never clips.
- `ActivateOrDeactivate.srv`: additive `float64[] leg_targets_rev` (empty = the constant pose). The bridge's service passes it to `_run_activate(leg_targets_rev=)`, and refuses `deactivate` that carries targets.
- `state_machine`: `activate_levelling_refusal(ctx)` is the one named policy (fresh = status younger than `TRAJ_STATUS_STALE_S = 1.0 s`, the skill stack's own value for this 5 Hz topic; never received = refused). `IdleHandler` decides `activate` in IDLE: the refusal, then a `fetch_activate_targets` request; only six fetched targets go to ACTIVE.
- `orchestrator_node`: caches `trajectory/status` and its age each tick; dispatches the fetch to trajectory_node; the `activate` dispatch carries and consumes the targets and FAILS (never sends the constant) if none were fetched; `level_activate` builds its request without targets, by design.

**Fresh, from the code.** C-LEVEL-1.O requires a fresh status and names no number; the only consumers that age it are `skill_node` (`_TRAJ_STATUS_STALE_S = 1.0`, five missed 5 Hz samples) and the executor's `levelled` row on the same status. The orchestrator uses the same 1.0 s. A trajectory_node that died stops publishing, so its last `True` expires within a second and the refusal names the stale status rather than sending the operator to `level` a dead node.

**Control-cycle walk, the refusal.** Orchestrator tick k (10 Hz): `_check_pending_operations` (nothing), `traj_status_age_s` from the cache, `IdleHandler.execute` consumes `activate`, `activate_levelling_refusal` returns a reason, one operator notice, `return None`. No request is queued, so `_process_requests` sends nothing: no service call, no CAN, no RPC. The state is still IDLE, `control_mode` stays blank, the 40 Hz emitter is not streaming (IDLE), the Legs stay at the stow under whatever the bridge last left them. Tick k+1 is identical without the command. A fetch refusal costs one more tick: tick k queues `fetch_activate_targets` (still IDLE, a service call to trajectory_node only), tick k+1 reads `success=False` and logs the one WARN with trajectory_node's reason (for the band: each Leg and its size). Nothing reached the bridge in either case.

**Control-cycle walk, the levelled rise.** Tick k: IdleHandler queues the fetch; trajectory_node answers from its own state (the applier, so the answer cannot be stale), six revs. Tick k+1: targets in `ctx`, IdleHandler returns ACTIVE. Tick k+2: `ActiveHandler.on_enter` requests `activate`; the dispatch sends `ActivateOrDeactivate(command='activate', leg_targets_rev=six)`; the bridge's `_run_activate` re-checks the six Legs and the band (no RPC if not), sends ACTIVATE_TO, the firmware checks the band a third time and runs the same TRAP_TRAJ ladder from the stow to the targets (Discussion (e): each Leg ends within 0.027 rev of the constant pose, the six finish within about 11 ms of each other at 2.5 rev/s, no 40 Hz stream). The bridge monitors arrival on the sent targets, then configures PASSTHROUGH. Next tick: ActiveHandler publishes STANDBY; trajectory_node seeds from the encoders, which now read the levelled pose; its hold's u0 = IK(FK(measured)) = the measured revs (pinned to 1e-6 rev); the arm's 0.25 rev u0-vs-encoder check passes by the dead-band, `mpc_active` rises, the first applied setpoint is the pose the ODrives already hold. The first `go_home` plans from that seed to the same corrected neutral: max per-Leg motion over the 41 sampled knots below 1e-5 rev (pinned), against about 0.027 rev from the constant pose before this change (the same test's False case). Torque and velocity feedforward are zero throughout (a hold, then a zero-length move).

**What the first-applied-setpoint walk does NOT cover.** The arrival tolerance: the firmware and host monitors accept arrival within 0.05 rev, so the measured pose (and therefore the hold and `go_home`'s start) can sit up to that far from the target, exactly as it can from the constant pose today; `go_home` then closes the arrival error, not the correction.

### Verification (caller)

All 2026-11-25, in the worktree on branch offline/bug130, no colcon (the interface mocks in `tests/ros/conftest.py` match the two .srv texts field for field).

- Focused (`flock /tmp/jugglebot-tests.lock nice -n 10 python -m pytest -q -n 4 -p no:cacheprovider tests/ros/test_state_machine.py tests/ros/test_orchestrator_node.py tests/ros/test_orchestrator_conduit_contract.py tests/ros/test_orchestrator_conduit_integration.py tests/ros/test_platform_imu_gate.py tests/ros/test_teensy_bridge_node_activate.py tests/ros/test_teensy_bridge_node_conduit.py tests/ros/test_trajectory_node.py tests/ros/test_levelling_frame.py tests/ros/test_ros_mock_hygiene.py`): **630 passed, 2 failed**. Both were test-side: `TestOperatorConsoleLines::test_mode_switch_in_active_is_one_line` forces ACTIVE without a fetched target set, which the `activate` dispatch now fails by design (the test now sets the targets first), and my band-edge helper test added 0.10 to a float and landed 1e-16 outside (now 1e-9 inside). Re-run of those two (`python -m pytest -q -p no:cacheprovider "tests/ros/test_orchestrator_node.py::TestOperatorConsoleLines" "tests/ros/test_trajectory_node.py::test_band_refusal_helper_names_non_finite_and_wrong_count"`): **19 passed**.
- Fails without the change (`flock /tmp/jugglebot-tests.lock nice -n 10 python $S/out/bug130-caller/mutate.py`, each source mutation applied alone, the named tests run, the source restored): M1 refusal returns None: 6 failures (all four refusal cases, `levelling_complete` is not the answer, the one-WARN node test); M2 staleness ignored: 3; M3 `level_activate` carries cached targets: `test_level_activate_keeps_the_constant_pose`; M4 `activate` sent without its targets: 4, including both end-to-end tests; M5 the service returns the constants: 5, including the go_home no-op `[True]` case and both end-to-end tests; M6 no band check: 2; M7 the bridge drops the targets: 3.
- Wide scoped run, every reader of the touched modules and of `tests/ros/conftest.py` (`flock /tmp/jugglebot-tests.lock nice -n 10 python -m pytest -q -n 4 --dist loadfile -m "not nightly" -p no:cacheprovider tests/ros tests/teensy_link tests/firmware tests/motion/test_levelling.py tests/motion/test_tilt_map.py tests/motion/test_skills_executor.py tests/sim/test_live_session_guard.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_docs_links.py`, load average about 23): **5843 passed, 6 skipped, 22 failed in 3022 s**. Sixteen were `test_state_machine_deactivate_confirm.py`, whose helpers sent `activate` from IDLE with no correction (now through a `_idle_to_active` helper that loads one and resolves the fetch); two were `test_choreography_map.py` (the generated `ros_ws/docs/choreography.md` now lists the new service and the orchestrator's `trajectory/status` subscription: regenerated with `python tools/gen_choreography_map.py`); two are this entry's INDEX row (the orchestrator's `mapkit gen`); `test_skill_node_robot_frame.py::test_moved_frame_announcement_landing_is_the_robot_frame_prior` and `test_stall_watchdog.py::test_gil_held_stall_is_dumped_mid_stall_and_named` touch none of this change and were re-run below.
- Re-runs after those fixes: (`flock /tmp/jugglebot-tests.lock nice -n 10 python -m pytest -q -n 4 -p no:cacheprovider tests/ros/test_state_machine_deactivate_confirm.py tests/ros/test_choreography_map.py tests/ros/test_skill_node_robot_frame.py tests/ros/test_stall_watchdog.py tests/ros/test_state_machine.py tests/ros/test_orchestrator_node.py`): **378 passed, 3 failed**: one more `_idle_to_active` case (a forced ACTIVE to IDLE leaves its exit request queued ahead of the fetch; the helper now checks membership) and the two `test_skill_node_robot_frame.py` cases refusing on a mocap sample "33.20 s old" under load. Then, on a quiet box (load 3.7): `python -m pytest -q -p no:cacheprovider tests/ros/test_state_machine_deactivate_confirm.py` **27 passed**; `python -m pytest -q -p no:cacheprovider tests/ros/test_skill_node_robot_frame.py tests/ros/test_stall_watchdog.py tests/ros/test_choreography_map.py` **52 passed** (the robot-frame and stall-watchdog failures were load, not this change).
- Not run: colcon (the orchestrator rebuilds `jugglebot_interfaces` and `jugglebot` at integration); the firmware native binaries (no firmware change in this commit; the Python `tests/firmware` ran in the wide run).
