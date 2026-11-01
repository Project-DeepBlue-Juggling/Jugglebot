---
title: "JB-73: can-bridge FW 35 re-sends each lane's last frame once with vel_ff = 0 on the first tick of a MOTOR_FB_STALE suppression, so the ODrive holds AT the last command (built, native-tested and flashed; bench-leg Sitting pending)"
type: bugfix
date: 2026-11-01
status: in-progress
phase: "night-shift follow-up"
subsystem: can-bridge
related_entries:
  - 2026-11-01-night-shift.md
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - teensy_link/rpc_args.py
  - tests/firmware/native/test_leg_interp.cpp
  - tests/firmware/native/test_fault_machine.cpp
  - tests/firmware/test_bridge_fw_version_xref.py
  - logbook/INDEX.md
---

# JB-73: the suppress-edge hold flush (FW 35)

**Summary.** During a MOTOR_FB_STALE suppression the can-bridge stops transmitting setpoints, and
the ODrive keeps its last `Set_Input_Pos` including vel_ff, so in POSITION/PASSTHROUGH a leg
settles at input_pos + vel_ff/pos_gain: 0.05-0.056 rev (3.5-4 mm) past the last command on the
bench leg on 2026-10-27, about 6 mm at the 3.5 rev/s vel_ff cap. The owner chose JB-124 option 1:
one final frame per lane at the suppress edge, vel_ff = 0. FW 35 does that; it is built,
native-tested and flashed (2026-11-01).

**Problem.** The suppressed state is meant to be a hold and was not one. It was bounded (the
velocity error goes to 0, no wind-up; the recovery slew re-baselines from the encoder), but each
leg overshot by its own vel_ff/pos_gain, so with six legs the Platform pose distorted.

## Discussion

**Which lanes, and when a frame is wrong.** The suppression gate bites on the six legs (the ladder
burst) and the Hand (the 7th frame), both in POSITION/PASSTHROUGH: legs after the host's
configure, the Hand after ACTIVATE hands it over. The danger was a `Set_Input_Pos` landing on an
axis in TRAP_TRAJ, which starts a new trajectory: the Stow descent (DEACTIVATE), ACTIVATE, HOME
and HAND_MOVE_TO are such moves. None can meet the flush: they are refused while `mpc_active`,
MOTOR_FB_STALE requires `mpc_active`, the deferred Stow forces `allow = true` in
`fault_machine.cpp` (so it is never a suppression, and its branch now clears the leg record), and
the flush is gated on `!coldstart`.

**What the frame carries.** Each lane's LAST TRANSMITTED input_pos, recorded at the TX site,
never a new position: feedback is stale, so the bridge does not know where the motor is. Legs
keep their last torque_ff (gravity feedforward ships enabled; zeroing it would sag the Platform).
Only vel_ff goes to 0. The frame goes through the same `encode_leg_setpoint`, so its bytes differ
from the last frame only in the vel int16. The Hand keeps the FW 22 torque drain (torque 0), now
also with vel_ff 0.

**When it fires.** Exactly once per lane per suppress edge: on the first suppressed tick, keyed on
"this lane was on the wire last tick". A re-suppress after recovery flushes again; a flap gives one
burst per edge; nothing at boot or while disarmed. `fault_machine.cpp` sets `hold_flush` only
when MOTOR_FB_STALE is the sole reason output is off: never on an E-STOP latch, a fatal error, a
dead bus, guard DISABLED (disarm, link loss) or the FW 34 arm shut. A lane that became present
at the edge has no record and is never flushed from 0. TX budget: the flush replaces the
suppressed tick's normal burst, at most 6 leg frames plus 1 Hand frame, the qualified 7-frame
burst.

**The trade-off accepted (owner, 2026-11-01).** Zeroing vel_ff in one frame steps the ODrive's
velocity setpoint: with leg vel_gain 0.20 N·m/(rev/s) (3.63 A per rev/s) that is a current step of
about 7.3 A at the bench's 2 rev/s and 12.7 A at the 3.5 rev/s cap, both under the 15 A clamp.
The old behaviour coasted to a stop over about 1/pos_gain = 25 ms and overshot; the new one
stops every leg at its last command (a consistent pose) with a sharper deceleration. The
alternative, ramping vel_ff to 0 over about 12 ticks, breaks "one frame" and lets the leg drift
about vel_ff x T/2 = 0.024 rev at 2 rev/s, which misses the 0.01 rev pass criterion. Decision:
fly the single frame on the bench leg and measure the iq step. **Pre-registered fallback:** if the
iq step at the edge exceeds about 10 A at 2 rev/s, or the stop is a visible jolt, switch to the
ramp.

**Not done, deliberately.** The LEG_CMD reporting quirk (target cache written every tick even
while suppressed) is not a behaviour-free change: the cache feeds telemetry during cold-start
moves, so it stays; analysers keep masking. `teensy_interp.py` (the Python twin) has no output
gate, the same documented divergence as the recovery slew. Disables other than MOTOR_FB_STALE
(link loss, disarm) still leave the last vel_ff held. The Item's three other instances
(input-mode check, the Guard against the post-slew command, RETARGET's 0x011 / 0x00C order) are
untouched.

## Fix

`fault_machine.cpp` computes `hold_flush` and passes it to
`interp_set_output_enabled(bool en, bool hold_flush = false)`; `leg_interp.cpp` records each
lane's last transmitted frame (a per-leg bitmask plus the Hand's), sends the one vel_ff = 0 frame
per lane on the first suppressed tick and counts edges (`interp_hold_flush_edges()`).
`FW_VERSION` 34 -> 35 with its history line; `PROTOCOL_VERSION` stays 10 (no wire change);
`EXPECTED_BRIDGE_FW_VERSION = 35`. Flashed the same day, so launches log
`BRIDGE_FW_CHECK: OK`.

## Verification

- Build, 2026-11-01: `pio run -e teensy41` -> success, `251200 35520 111488 398208 61380 firmware.elf`.
- Flashed 2026-11-01 (owner asked), launch down: `pio run -e teensy41 -t upload` from the shared checkout at 36e1085b -> SUCCESS; receipt: a read-only BridgeIdentity listen reported `fw_version=35 protocol_version=10`. The robot now runs FW 35; the bench-leg Sitting is still the behaviour check.
- Firmware tests, 2026-11-01: `pytest tests/firmware/ -q` -> 283 passed, 5 skipped; natives `test_leg_interp` 57/57, `test_fault_machine` 29/29. With `hold_flush` forced false all 6 new cases fail (5 + 1).
- Gate, 2026-11-01: `./run_tests.sh` -> 7916 passed, 49 skipped in 406.57 s; serial 5 passed; exit 0 (the native firmware binaries are pre-built and run inside it).

## Open Questions

- The bench-leg Sitting (JB-73's `## Sitting`): leg within 0.01 rev of the last transmitted command after every rate-0 gap (FW 33 baseline 0.05-0.056 rev), plus the iq step at the edge against the fallback threshold above. The Hand path is covered by native tests only: the bench rig has no axis 6.
