---
title: "FW 53 unit 1b: the can-bridge tells the one-Leg bench from the robot (rig_is_bench: exactly one ODrive and no Platform Teensy frame for 5 s on a transmittable bus, sticky toward robot), and the Leg Stow gate does not apply on the bench"
type: feature
date: 2026-11-25
status: in-progress
phase: "FW 53 U1b"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_stow_gate.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_stow_gate.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_deactivate.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - tests/firmware/native/test_leg_stow_gate.cpp
  - tests/firmware/native/test_leg_deactivate.cpp
  - logbook/2026-11-25-fw53-u1b-bench-exemption.md
subsystem: [can-bridge, safety]
---

## Summary

Owner decision, 2026-11-25: "The bench leg doesn't have the same requirements placed on it as robot legs. Notably, the bench leg can be put into IDLE at any position, without any negative consequence. The firmware should be able to discern between a bench from a robot, because the bench setup will only ever have one ODrive on CAN2, while the robot has 7 (and the Platform Teensy)."

This unit adds that discrimination to the Stow gate module (I-FW-19, FW 53 U1) as one predicate, `rig_is_bench(now)`, and one composition every de-energise asks, `leg_idle_permitted(now, except_leg) = rig_is_bench(now) || platform_at_stow(now, except_leg).ok`. On the bench, a Leg IDLE, Reboot or SDO write passes at any position; on the robot nothing changes. The predicate fails toward "robot": the gate stays on whenever there is any doubt. `FW_VERSION` stays 52 (the FW 53 arc bumps it once, at integration).

## What changed

- **`leg_stow_gate.{h,cpp}`.** `rig_is_bench(now)` is true iff all of:
  1. the robot latch is clear;
  2. exactly one ODrive axis (any of node ids 0..6, Leg or Hand) has been heard on the Jugglebot bus since boot. "Heard" is the union of a well-formed frame of any kind (noted by the RX decode, `rig_note_axis_frame`) and the existing `heartbeat_seen` latch, so the census is never smaller than the presence latch the rest of the firmware uses;
  3. no Platform Teensy frame since boot (`rig_note_platform_frame`);
  4. the bus was transmittable (ESR1.SYNCH) at the last 1 kHz sample (`rig_note_bus`), and `now - max(first axis frame, bus last became transmittable) >= RIG_BENCH_SETTLE_US` (5 s). A `now` older than the anchor is not settled.
  
  Two axes or one Platform Teensy frame set the robot latch, which only `leg_stow_gate_init()` (boot) clears. `leg_deenergise_allowed` (behind all three wrappers) now asks `leg_idle_permitted`. Console accessors `rig_axes_heard_mask`, `rig_platform_seen`, `rig_robot_latched`.
- **`can_buses.cpp`.** Three witness hooks: `rig_note_axis_frame(axis, t)` beside the FW 23 liveness stamp in `decode_into_cache` (same `micros64()` sample), `rig_note_platform_frame()` at the top of `on_jugglebot_rx` (above the IMU and relay-ring early returns), and `rig_note_bus(synced, micros64())` after the Jugglebot bus's `service_bus` in `can_buses_service` (1 kHz, task_can_rx).
- **`can_buses.h`.** `is_platform_teensy_tx_id(id)`: the reply ids, IMU_DATA, IMU_LIVE_ACCEL/GYRO and the 2 Hz TRAFFIC_REPORT 0x7DF. Inline, so the native test pins it.
- **`canbridge_config.h`.** `RIG_BENCH_SETTLE_US = 5 s`, with its sizing.
- **`leg_deactivate.cpp`.** The three predicate uses (single-Leg request :216, arrival :365, timeout :383) call `leg_idle_permitted`. On the bench a MONITOR timeout far above Stow now IDLEs the Leg (FW 52 behaviour) instead of freezing it; arrival no longer waits for a fresh heartbeat; the single-Leg request was already vacuously allowed on the bench.
- **`Teensy_code_canbridge.ino`.** A `[rig]` serial line beside `[diag]`: `bench (Stow gate off)`, `undecided (gate on)` or `robot (latched)`, with the heard-axis mask and the Platform witness, printed on every change and once a minute. Evaluating it at 1 Hz also sets the robot latch promptly. No UDP struct or MsgType changed.
- **Tests.** `test_leg_stow_gate.cpp` cases 16 to 22; `test_leg_deactivate.cpp` one case (bench timeout IDLEs). Both drivers already link `leg_stow_gate.o`; the native HAL needed nothing new because the witnesses live in the gate module and the test drives them directly.

## Discussion

### Where the code departs from the brief's definition, and why

- **The settle is anchored on the first axis frame, not on bridge up-time.** The brief proposed "the bridge has been up, with the bus transmittable, for longer than the settle". An idle bus with no powered node is recessive, so the FlexCAN synchronises and SYNCH reads 1 (inferred from the FlexCAN idle-detection rule, unchecked on this board). If the robot's supply comes on after the bridge has been up for a minute, that definition is already satisfied when the first drive speaks, and the robot would read as a bench for the ~ms to ~100 ms until the second drive is heard: exactly the window the brief says must not read as a bench. Anchoring on `max(first axis frame, bus transmittable since)` closes it; case 17 pins both orderings.
- **5 s, not 3 s.** A live ODrive is heard within ~100 ms (10 Hz heartbeat; ~272 frames/s per axis, `canbridge_config.h` CAN_AXIS_SILENCE_TIMEOUT_US comment). The robot's drives share one supply and boot together, so their spread should be well under a second (unchecked: not measured). The longest ODrive gap quoted in the firmware is the ~3.5 s worst-case reboot gap (`BUS_PARTNER_STALENESS_US` comment); 3 s would sit inside it, 5 s clears it with margin and also spans ten Platform TRAFFIC_REPORTs. The only cost is a bench Leg that cannot be idled off-Stow in the first 5 s after its drive appears.
- **"Heard" counts any well-formed frame, not only heartbeats.** More inclusive means more often "robot", the safe direction; and a drive whose heartbeats are dropped under load (the FW 23 finding) still counts.
- **The Platform witness is TRAFFIC_REPORT plus every Platform reply id.** The bridge latched nothing for the Platform Teensy before this unit: its replies went to the relay ring, IMU_DATA to its own ring, and the unsolicited 2 Hz TRAFFIC_REPORT (0x7DF, `reportStatus()` in the Platform's `loop()`, every firmware version since the rename) fell into `decode_into_cache` and was discarded as a bad axis. HAND_FAN_STATUS (1 Hz) needs Platform FW >= 9 and IMU_STATUS needs the IMU build, so TRAFFIC_REPORT is the witness that matters. The 0x6E0 and 0x7DE request ids are also sent by the bridge, but FlexCAN does not receive its own frames (the relay ring would already be full of echoed requests if it did; inferred, unchecked).
- **The DEACTIVATE timeout IDLEs on the bench.** Freezing a bench Leg is also harmless, but the owner's statement is that the bench Leg may be idled anywhere, and FW 52 idled it; one composition for every "may I idle" question keeps the bench rule in one place.

### How a robot could be misread as a bench, and what each would take

All four conditions must hold together, for 5 s, from the moment the first drive is heard after a bridge boot. Two drives or one Platform frame at any time since the boot ends the question until the next boot, so every path below needs a bridge boot (power-up or reboot) during which the anomaly already exists.

1. **Five Leg drives and the Hand unpowered (logic off), one Leg drive powered and posed.** A single Leg cannot hold the Platform up: with the other five IDLE or unpowered, they are back-drivable and sit on their bottom stops, so the Platform already rests on them and the one energised Leg can at most prop one side of it, tilted about the other Legs' lower joints. IDLEing that Leg lets that side settle onto its stop, a drop of at most that Leg's extension. That is the consequence the operator accepted by running the robot with six drives off, and no ladder of this bridge boot could have posed the Platform in that state: the other Legs were never present, so nothing streamed to them. The Platform Teensy must also be silent (it is on the Platform; whether it shares the drives' supply is unchecked).
2. **A loom fault that isolates six drives and the Platform Teensy while one Leg stays reachable, present at a bridge boot, while the robot is posed and the isolated drives still hold Closed-loop.** This is the dangerous one: five Legs holding and one IDLEd leaves the Platform one degree of freedom under gravity. It needs a bridge boot mid-Sitting (brownout or watchdog), a break that cuts exactly six of seven nodes off a 1 Mbit/s bus yet leaves the remaining segment working (unterminated at one end), the isolated drives still in Closed-loop with no stream for 5 s (whether their CAN input watchdog disarms them is unchecked), and then a de-energise request for the reachable Leg. A loom fault after the boot does nothing: the latch already reads robot. Mitigation if the owner wants it: the "expected Legs" mask the design declined (fw53_design.md open decision 5), or a persisted "this board was a robot" flag.
3. **A bridge reboot mid-Sitting with the bus down.** While SYNCH is 0 the settle clock is held at zero, and with the bus down nothing is heard (no axis: never a bench). When the bus returns every drive on it is heard within ~100 ms and the latch reads robot. With the robot supply off, the drives cannot hold the Platform anyway.
4. **The Platform Teensy silent** (2026-11-19: FW <= 10 hung in `setup()` waiting for the SCL3300, a day of silence). This removes one of two independent witnesses; the robot is still a robot by its seven drives, so the misread then needs case 1 or 2 as well. Platform FW >= 11 no longer blocks in `setup()`.

Not a misread: the robot's first ~100 ms after power-up, when one drive has spoken (case 17, the anchor rule), and a robot with the Hand plus any one Leg (case 20).

### The reverse, and the bench Hand rig

A bench read as a robot (the first 5 s; a second ODrive or a Platform Teensy on the bench bus; a flapping bus) keeps the gate on: the bench Leg must be at Stow to idle, which DEACTIVATE(0) provides. Harmless, only slower. The bench Hand rig (one ODrive, the Hand, no Leg) reads as a bench, which changes nothing: the Hand is exempt and no Leg is present.

## Verification

All 2026-11-25, in `S/wt-fw53` on `offline/fw53`.

- Native, by hand (`python S/out/fw53-u1b/quick.py S/out/fw53-u1b <gate.cpp> test_leg_stow_gate test_leg_deactivate`, then each binary): test_leg_stow_gate **22/22 cases, 669/669 assertions**; test_leg_deactivate **26/26 cases, 678/678 assertions**.
- Mutations (`python S/out/fw53-u1b/mutate.py S`, gate mutations on a copy, the two in-place ones restored from a backup): **13/13 caught**.

  | # | Mutation | Caught by |
  |---|---|---|
  | M1 | the wrappers ask `platform_at_stow` only (bench ignored) | 16 |
  | M2 | settle dropped (`now - anchor >= 0`) | 17, 21 |
  | M3 | anchor = bus transmittable only | 17 |
  | M4 | anchor = first axis frame only | 17, 21 |
  | M5 | the Platform witness ignored | 18 |
  | M6 | TRAFFIC_REPORT dropped from `is_platform_teensy_tx_id` (`can_buses.h`, in place) | 18 |
  | M7 | no sticky robot latch | 19 |
  | M8 | only Leg axes noted (the Hand ignored) | 20 |
  | M9 | a non-transmittable sample does not restart the clock | 21 |
  | M10 | `rig_is_bench` ignores the bus (tx == 0) | 21 |
  | M11 | "exactly one axis" weakened to "any" | 19, 20, 22 |
  | M12 | no `now >= anchor` guard (u64 underflow reads settled) | 22 |
  | M13 | DEACTIVATE timeout reverted to `platform_at_stow` (`leg_deactivate.cpp`, in place) | deactivate "MONITOR timeout on the one-Leg bench" |
- Scoped suite (`flock /tmp/jugglebot-tests.lock nice -n 10 pytest -q -p no:cacheprovider tests/firmware tests/motion/test_kt_lib.py tests/ros/test_teensy_bridge_node_deactivate.py tests/ros/test_teensy_bridge_node_can_errors.py tests/teensy_link/test_protocol_codec.py tests/teensy_link/test_fw_update_window_convergence.py tests/teensy_link/test_deactivate.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_no_legacy_item_ids.py`; the non-firmware files are every Python reader of a touched firmware file, by `grep -rl`): **703 passed, 4 skipped in 476.77 s**, the full native build and every native binary included. `tests/sim/test_logbook_index.py` stays red until `mapkit gen` at integration.
- Image (`pio run -e teensy41`, build only, never `-t upload`): **SUCCESS**; `leg_stow_gate.cpp`, `can_buses.cpp`, `leg_deactivate.cpp` and the `.ino` compile with no warnings (the one `fault_machine.cpp` unused-function warning and the FlexCAN_T4 `strncpy` warnings predate this unit). `FW_VERSION` still 52.
