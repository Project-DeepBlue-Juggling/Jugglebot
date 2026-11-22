---
title: "BUG-123: a Hand stall guard on the streamed lane and latched holds (40 A / 1 rev/s / 150 ms, then Hand IDLE and a named HAND_STALL latch), can-bridge FW 48, NOT flashed"
type: investigation
date: 2026-11-20
status: in-progress
phase: "BUG-123"
subsystem: can-bridge
related_issues:
  - BUG-123
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/axis_state.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_activate.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_activate.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - teensy_link/rpc_args.py
  - teensy_link/activate.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - tests/firmware/native/coldstart_hal.h
  - tests/firmware/native/coldstart_hal.cpp
  - tests/firmware/native/test_leg_activate.cpp
  - tests/firmware/native/test_fault_machine.cpp
  - tests/firmware/test_hand_stall_guard_xref.py
  - tests/firmware/test_bridge_fw_version_xref.py
  - tests/ros/test_hand_lane_stall_host.py
  - tests/hardware/hand_stream_bench.py
  - tests/hardware/hand_stall_stage.py
  - tests/motion/test_hand_stream_bench_stall.py
---

# BUG-123: a Hand stall guard on the streamed lane and latched holds

## Problem

Sitting 2026-11-20, 13:34:22 (bag `2026-11-20_12-49-25`). The Hand lost its home and stalled against its top stop on the streamed lane. MAX_DEVIATION latched 80 ms in, as designed, but a guard latch only freezes the output. The drive kept its last lead-clamped `input_pos` (measured 5.86 + `MAX_LEAD_HAND_REV` 2.0 = 7.85 rev) and pushed 68-70 A for 4.4 s until the operator's E-stop button. The thermistor read 33.5 C at the start and peaked at 88.3 C 3.4 s after the cut. The motor is damaged. The FW 36 stall guard (BUG-56, 25 A / 0.5 rev/s / 300 ms) covers only the ACTIVATE / HAND_MOVE_TO ladder. The host `hand_jam` detector excludes the top stop by design. The drive's thermistor limits lag by at least 3 s. Diagnosis: `logbook/2026-11-20-sitting-debrief.md` and the Item.

Owner decision (2026-11-20, verbatim): "Let's go with 40 A / 1 rev/s / 150 ms, after which the hand motor should enter IDLE."

## Discussion

**What iq and velocity the firmware has, and how fresh.** The Hand ODrive Pro broadcasts Get_Iq and Get_Encoder_Estimates every 9 ms (`iq_msg_rate_ms 9`, `encoder_msg_rate_ms 9`, `config/ODrive config Files/odrive_pro_hand_config.json`). `can_buses.cpp` decodes them in `task_can_rx` into `axes[6].iq_measured` and `axes[6].vel_rps` (ODrive convention, no sign flip). The encoder decode already stamps `pos_timestamp_us`. iq had no stamp, so this change adds `iq_timestamp_us`, written after the value with `atomic_write_u64`. The 10 Hz figure is the fault poll and the Diagnostic uplink, not the cache: the cached iq is about 9 ms fresh. The guard runs in `activate_step()` on `task_homing` at 100 Hz. It evaluates each NEW sample once, when the stamp has moved. The 150 ms window is measured between sample stamps, never on the tick clock. Consequences:
- A tick with no new sample neither advances nor restarts the window, so stale data cannot trip it.
- A velocity sample older than `HAND_LANE_STALL_GAP_US` (50 ms) at the iq sample counts as unknown, not still, and restarts the window.
- An iq gap over 50 ms restarts the window at the next sample, because "continuously" is not known across a hole.
- At 100 Hz against a 9 ms stream, about one sample in ten is overwritten before the tick sees it. One unseen sample can hide a single 9 ms break in a run. A break of two samples or more is always seen. The error is on the tripping side and is 9 ms against 150.

**When Hand telemetry stops arriving**, the guard cannot see a stall and does not trip. When samples resume, the window restarts. Hand silence is covered by the existing guards:
- `MOTOR_FB_STALE` (`fault_machine.cpp` evaluate_guard, FW 27): Hand encoder older than 150 ms while the lane is armed. It suppresses output and is recoverable. It does not IDLE the Hand.
- The CAN_BUS_DOWN silence watchdog (`CAN_AXIS_SILENCE_TIMEOUT_US` 2 s) is legs-only by design (`fault_machine.cpp` header): a whole-bus loss silences the Legs too and trips it, but a Hand-only silence does not.

So a Hand ODrive that keeps driving while its own CAN TX dies is not covered by this guard. That is an accepted residual: a drive whose transmitter has died is a different fault from a lost reference, and the bridge could not confirm an IDLE to it either.

**False trips, against the code and the 2026-11-20 bags (100 Hz).** Apart from the real stall, no run of |iq| >= 40 A at |vel| < 1 rev/s lasted longer than 0.15 s.
- Throw bottom reversal: velocity crosses zero for a few ms between two fast strokes. Peaks are 28-48 A while moving, so at most one or two samples meet the predicate. Native test "one moving sample restarts it".
- Catch impact and Feed Catch touches: at most 38.7 A in bursts under 0.1 s (0.1-0.2 s by a second read, but below 40 A).
- The trapped-Ball pinch at 2.08 rev: 72.3 A for 0.09 s does not trip (native test). A pinch past 150 ms would trip, which the owner intends. If `hand_jam` is mid-recovery when that happens, see below.
- Homing: about 8-9 A. Not excluded, because a Homing stall at 40 A would be a stall too.
- Parked Hand on the bottom stop, or held at a latched hold under a Ball's weight: gravity hold is `HandEnv::GRAVITY_HOLD_CURRENT_A` 1.5 A. The four multi-rev MAX_DEVIATION holds of 2026-11-20 peaked at 20-36 A (debrief § 3), all under 40 A. The 36.2 A hold (13:40:58) is the closest, at 90 % of the threshold. Note it for the next Sitting's data.
- Ladder moves: excluded. While the ladder targets the Hand, its own guard owns it, and that guard is unchanged (native test "the ladder keeps its own guard").

**What IDLE does physically.** The Hand falls under gravity, with any Ball in it, onto the bottom stop. That is the owner's choice. The guard sends only IDLE and then:
- calls `fault_notify_hand_stall()`. That latch raises guard_mode ESTOP and shuts the interp output gate, so no axis is streamed.
- leaves ACTIVATE and HAND_MOVE_TO refused (`activate_allowed`, ESTOP), so the firmware cannot re-energise the Hand until CLEAR_ERRORS.
- leaves the 500 Hz Hand position stream unable to re-energise it: that stream only sends `Set_Input_Pos`, which never changes an ODrive axis state. The only axis-state senders are the ladders (`leg_activate`, `leg_deactivate`, `leg_homing`) and the raw `SET_AXIS_STATE` RPC.
- On the host, the lane arm path that sends `SET_AXIS_STATE(CLOSED_LOOP)` for the Hand refuses while `fault_state != NONE` (`teensy_bridge_node.py` arm pre-check).

**The Legs** get exactly what a Hand ODrive fault gives them: a Hand active error or disarm-while-CLOSED_LOOP sets `s_fatal_error` and then guard ESTOP with the output gated, and the Leg ODrives hold their last setpoint (`fault_machine.cpp` evaluate_errors, evaluate_guard). The stall latch takes the same path, through guard_mode and the output gate. No new Leg action was invented. The Hand is not IDLEd by the Leg logic and the Legs are not IDLEd by the Hand logic, mirroring the FW 36 rule that a Hand fault must not drop a loaded Platform.

**Its own latch, not `s_estop_state`.** The Soft E-stop freezes its reason at the first trip. On 2026-11-20 that was MAX_DEVIATION, 70 ms before this guard would have fired. A shared latch would then have reported MAX_DEVIATION, and the operator would not learn that the Hand was IDLEd. So `s_hand_stall_latched` is separate:
- reported as `FaultState::HAND_STALL`, above the frozen guard reason and below CAN_BUS_DOWN / ODRIVE_FATAL;
- the MAX_DEVIATION snapshot fields are kept;
- released only by `fault_notify_clear_errors()`.

**Wire.** HAND_STALL = 8 is a new value in the existing u8 `fault_state`. Diagnostic flags bit3 `DIAG_FLAG_LANE_STALL` was a previously-zero bit (the FW 36 bit2 precedent). No layout changes, so PROTOCOL_VERSION stays 14. A FW 46 host would show `8` but still read it as a latched fault (anything not NONE refuses to arm), which is fail-safe. FW_VERSION and EXPECTED_BRIDGE_FW_VERSION move to 47 together.

**Interaction with MAX_DEVIATION holds, /recover_lane and /clear_errors.**
- The guard is armed through a latched hold (native test), because that is where 2026-11-20 burned the motor.
- `/recover_lane` refuses on any `fault_state != NONE`, so it refuses on HAND_STALL.
- The operator's `/clear_errors` releases both latches. Where its path parks the Hand (`_park_hand_disarmed`, the 2026-09-23 hand-park obligation; which paths park was not re-audited here), the park re-ACTIVATEs the Hand: the ladder seeds `input_pos` at the measured position and runs under the FW 36 25 A / 300 ms guard. The lane guard re-arms when the ladder lets go, and its first sample starts a fresh window.
- Recovery therefore re-energises the Hand only on the operator's fresh command and only under the tighter guard.

**Gap closed on the host: the hand-jam recovery's automatic clear.** `hand_jam` fires on a bottom pinch (>= 63 A, 0.10 s). Its recovery runs a converge-first clear (`CONVERGE_CLEAR`) whenever `fault_state != NONE` and then raises the Hand. If the stall guard trips during that recovery, the clear would release HAND_STALL with no operator, and the raise would re-energise the Hand. `_hand_jam_execute` now refuses `CONVERGE_CLEAR` while `fault_state == HAND_STALL`, unless the recovery is the operator's own `/recover` (resume). The recovery then ends unrecovered and the Hand stays IDLE.

**What was ruled out.**
- Running the guard at the 10 Hz fault poll: up to 100 ms of extra push, and samples are 9 ms apart.
- Running it in the 500 Hz interp ISR: an ISR must not own a retrying CAN send, and `leg_activate.cpp` is already one of the four allow-listed axis-6 TUs (`test_hand_single_master.py`).
- Evaluating in `task_can_rx` on each frame: exact, but it would put a TX in the RX context.
- Requiring `axis_state == CLOSED_LOOP` to arm: that state comes from the heartbeat and can lag. An IDLE Hand draws no current, so arming outside the ladder is the same set, without a lag that could disarm the guard.

## Fix

Contract first: `INVARIANTS.md` § 1 gains **I-FW-17**.
- `canbridge_config.h`: `HAND_LANE_STALL_IQ_A` 40, `HAND_LANE_STALL_VEL_RPS` 1, `HAND_LANE_STALL_HOLD_US` 150000 (the owner's numbers, cited), `HAND_LANE_STALL_GAP_US` 50000, static_asserts, and the FW 46→47 history line.
- `axis_state.h` / `can_buses.cpp`: `iq_timestamp_us`.
- `leg_activate.cpp`: `hand_lane_stall_guard()`, run first in every `activate_step()`. It sends IDLE, re-sent each tick until queued, latches, and sets the sticky bit3 record, which the next ladder targeting the Hand clears.
- `fault_machine.cpp`: `fault_notify_hand_stall()` / `fault_hand_stall_latched()`, the HAND_STALL report, guard ESTOP, the output gate, and release on clear only.
- `telemetry.cpp`: Diagnostic bit3, sent on its edge.
- The generator gains FaultState HAND_STALL and the bit3 doc.
- Host:
  - `teensy_link/activate.py` holds `DIAG_FLAG_LANE_STALL`, the mirrored numbers and `hand_lane_stall_line()`, the one operator line.
  - `teensy_bridge_node.py` logs that line on the HAND_STALL fault edge and on the Diagnostic bit3 edge, and the hand-jam clear refusal is added.
  - `trajectory_node.py` carries `[fault_state=HAND_STALL]` in every GUARD_LATCHED refusal, so the Attempt's end reason names the guard.

## Verification

- 2026-11-20, `python tests/firmware/native/build.py` then `./temp/firmware_native/test_leg_activate`: **41/41 test cases, 313/313 assertions** (8 new "lane stall guard" cases, 56 assertions). `./temp/firmware_native/test_fault_machine` (after the harness fix below): **all pass, 667/667 assertions** (new case "FW 48: ...", 35 assertions).
- 2026-11-20, `flock /tmp/jugglebot-tests.lock python -m pytest -q tests/firmware/test_native_firmware.py`: **23 passed**.
- 2026-11-20, `flock /tmp/jugglebot-tests.lock python -m pytest -q tests/firmware/ tests/teensy_link/ tests/ros/test_hand_lane_stall_host.py tests/ros/test_teensy_bridge_node_hand_jam.py tests/motion/test_bench_sysid_bridge.py`: **1145 passed, 4 skipped in 90.95 s**.
- 2026-11-20, `flock /tmp/jugglebot-tests.lock python -m pytest -q -n 4 --dist loadfile -m "not nightly" tests/ros/ tests/sim/test_logbook_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_no_legacy_item_ids.py`: **4098 passed, 2 skipped, 2 failed in 3900 s** on a loaded box. The two failures:
  - `test_no_legacy_item_ids`: the new fault-machine case reused a namespace whose name is a frozen legacy token. Fixed by giving the case its own arm helper.
  - `test_skill_node_robot_frame::test_moved_frame_announcement_landing_is_the_robot_frame_prior`: a load flake. Its file passed alone afterwards (`pytest -q tests/ros/test_skill_node_robot_frame.py`: **15 passed**).
- 2026-11-20, after the fix, `python -m pytest -q tests/firmware/test_native_firmware.py tests/sim/test_no_legacy_item_ids.py`: **45 passed**.
- 2026-11-20, `pio run -e teensy41` in `Teensy_code_canbridge` (compile only, NOT uploaded): **SUCCESS, 132.9 s**.
- The default gate (`./run_tests.sh`) was not run here. It runs at land time.

Not flashed, not proven on hardware. `status: in-progress` until a Sitting proves the trip (the Item's `## Sitting` section).

## Bench proof stage (2026-11-20)

Owner, 2026-11-20: prove the guard on the bench Hand, not the damaged robot motor. `hand_stream_bench.py --stage stall` (pure half `tests/hardware/hand_stall_stage.py`, reasoning in its docstring):
- Refuses before commanding anything unless the bridge reports firmware 48 or later, fault NONE, Hand IDLE with no errors at 0.3-0.7 rev, motor below 50 C, and the drive's limits read over USB.
- Lowers the Hand current limit to 45 A (`--stall-current-limit`, 41..50) with SET_VEL_CURR_LIMITS over the bridge, the RPC homing uses. The bridge has no read path for the limit, so the drive's USB link (already used by the bench home) READS `current_soft_max` before, after lowering (must read 45 or the stage stops before energising) and after the restore.
- The lane clips Hand commands to [0, 10.501] and the homed bottom stop reads -0.10, so the lane cannot command into the bottom stop. The stage re-references the IDLE Hand by 0.65 rev with SET_ABSOLUTE_POSITION (the lost-home failure, on purpose), closes the loop with the raw `--close-loop` sequence (no ladder, so the 25 A ladder guard is never armed), and ramps (trapezoid, 1 rev/s, 4 rev/s^2) to homed -0.60, 0.5 rev past the stop, inside MAX_LEAD_HAND_REV 2.0 and MAX_DEVIATION_HAND_REV 2.5.
- Stops streaming and disarms on the first sign of the trip (iq collapse, bit3 edge, HAND_STALL or axis IDLE), watches 1 s for a second current rise, sends one HAND_MOVE_TO to where the Hand rests (must be refused while latched), and prints `STALL-GUARD PASS: IDLE <t> ms after the first 40 A sample (limit 200), peak <A>, fault HAND_STALL, bit3 set` or FAIL with every reason. It never clears the latch.
- Independent abort: no trip 600 ms after the first sample at 40 A (or no 40 A within 3 s of the stop, or telemetry older than 200 ms), it commands IDLE itself. Bound at the clamp on that path: 20 + 600 + 5 + 2 x 40 + 10 = 715 ms, asserted under 750. The `finally` IDLEs an energised Hand, then restores the limit, then the frame (measured position minus the measured shift), each step on its own, Ctrl-C included.
- Found while building it: a raw SET_AXIS_STATE(CLOSED_LOOP) RPC is not gated by the HAND_STALL latch in firmware (`send_axis_frame` has no guard check); only the host node's arm path refuses it. The stage does not probe it. Diagnostic bit3 is cleared only by a reboot or a Hand ladder, so on a repeat the stage times the trip on the current, the fault and the axis state.

Verification:
- 2026-11-20, `python -m pytest -q -p no:cacheprovider tests/motion/test_hand_stream_bench_stall.py tests/motion/test_hand_stream_bench_order.py tests/motion/test_hand_stream_bench_stroke_cut.py tests/teensy_link/test_hand_stream_bench_gap.py`: **117 passed** (27 new).
- 2026-11-20, `python tests/hardware/hand_stream_bench.py --help`: parses. Not run against hardware.
