---
title: "JB-145: can-bridge FW 37 stops a fast upward Hand inside the stroke when the scheduled stream runs dry (no grace, C1 entry at 4500 rev/s^2, jerk-limited ramp-out; built and native-tested, NOT flashed; bench-hand Sitting owed)"
type: bugfix
date: 2026-11-03
status: in-progress
phase: "JB-145 / JB-154 option 2"
subsystem: can-bridge
related_entries:
  - 2026-11-02-hand-release-step.md
  - 2026-11-01-hand-limit-4500-and-stroke-floor.md
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - teensy_link/rpc_args.py
  - tools/probes/teensy_link_profiling/hermite_xref/teensy_interp.py
  - tools/probes/hand_stall_stop.py
  - tests/firmware/native/test_leg_interp.cpp
  - tests/firmware/test_sched_c2_twin.py
  - tests/firmware/test_hand_torque_ff_twin.py
  - tests/firmware/test_bridge_fw_version_xref.py
  - logbook/INDEX.md
---

# JB-145: the Hand's stream stop fits the stroke (FW 37)

**Summary.** If the scheduled setpoint stream runs dry near a Throw's release, FW 36 continued the
last span's cubic for SCHED_GRACE_S (20 ms) and then ran the jerk-limited C2 stop, which cannot start
below its entry acceleration. With the Hand at 150-190 rev/s upward that stop ended at 13-16 rev,
against a 10.501 rev stroke clip and a 10.701 rev hard stop. FW 37 (owner decision JB-154, option 2):
for the HAND lane only, when the cover ends with no newer frame and the Hand moves up faster than
HAND_C1_STOP_MIN_UP_VEL_RPS (50 rev/s), the grace is skipped and the stop starts at the cover end,
position and velocity continuous with the covered knot, acceleration stepping to -4500 rev/s^2 (the
existing hand stop authority SCHED_STOP_ACCEL_HAND_RPS2), held to v = A^2/(2J) = 28.9 rev/s, then
ramped out at the stop jerk (350k rev/s^3) to rest and HOLD. Slow or downward Hands and every leg
lane keep the grace and the C2 stop unchanged.

**Problem.** JB-149 shows the trigger is real: three forward emit gaps over 70 ms on 2026-11-02, one
ending in a Hand MAX_DEVIATION of +9.9 rev. A stop that carries the Hand through the clip into the
hard stop is a hazard whatever starves the stream.

## Discussion

**Why option 2 over option 1 and over 2-jerk.** `tools/probes/hand_stall_stop.py --option all 1.05
1.42` (2026-11-03, worst stopping knot, stop end in rev; clip 10.501, hard stop 10.701):

| Apex | release | baseline (FW 36) | opt 1 (no grace, C2 stop) | opt 2 (no grace, C1 at 4500) | opt 2-jerk (jerk-limited ramp-in) |
|---|---|---|---|---|---|
| 1.05 | step | 13.20 FAIL | 9.85 ok | 7.58 ok | 9.16 ok |
| 1.05 | continuous | 15.31 FAIL | 8.71 ok | 7.58 ok | 8.35 ok |
| 1.42 | step | 16.09 FAIL | 11.44 FAIL | 8.34 ok | 10.58 FAIL by 0.07 |

Option 1 removes the main term (the grace, ~3 rev at 150 rev/s) but the C2 stop's jerk-limited entry
from a = +2009 is still too slow at Apex 1.42. Keeping the jerk limit on the ramp-in fails Apex 1.42
by 0.07 rev. Only the C1 step fits everywhere, with 2.9 rev (1.05) and 2.16 rev (1.42) of stop margin
to the clip. Option 3 (fix the starvation, JB-149) stays worth doing but leaves the hazard in place
for whatever starves the stream next.

**The ramp-out is jerk-limited (a choice made here, not in the probe).** The probe's option 2 holds
-A until v = 0 and then steps to 0. FW 37 starts a jerk-J ramp at v_r = A^2/(2J) so a and v reach 0
together: no second 4500 rev/s^2 step, and it lands at the stroke top, the point nearest the clip,
where a position-loop overshoot of the hold would be upward. Cost: final height P + V^2/(2A) +
A^3/(24 J^2), i.e. +0.031 rev and +6.4 ms, 1.1 % (1.05) and 1.4 % (1.42) of the stop margin.
The static_assert on HAND_C1_STOP_MIN_UP_VEL_RPS > v_r keeps the constant-deceleration phase real.

**Tradeoffs accepted.**
1. An acceleration step of about 6500 rev/s^2 (+2009 to -4500 at the Apex 1.42 release) into the
   Hand ODrive's position loop at the cover end, and with it a torque-FF step (J*2pi*4500 = 0.30 N m,
   clamped at 0.234). This happens only when the stream has died at a fast upward Hand.
2. Found while writing the analysis, NOT in the probe: the grace exists because at emit_lead_knots 0
   one dropped knot (an emitter overrun) ends the previous cover and the next frame lands 1-15 ms
   after its stamp, inside the grace. Under FW 37, while the Hand moves up faster than 50 rev/s, that
   frame meets a C1 stop a few ms old, differs by ~5700-6500 rev/s^2 in acceleration (resume tolerance
   1000), and is REFUSED while armed: the latched hold, then the hand deviation guard as the plan walks
   away. One dropped knot during the fast upward stroke therefore aborts the Throw where FW 36 rode it.
   A frame that lands before the first tick past the cover end is still promoted first and no stop
   starts. Pinned by the twin test
   `test_a_single_dropped_frame_at_emit_lead_0_on_a_fast_upward_hand_meets_the_c1_stop_and_latches`
   and the native `... the accepted price ...` case. Two existing tests that dropped a knot at a fast
   upward Hand were moved (native and the torque-FF twin: the dropped frame 17 -> 23, where the Hand
   moves down through a = 0) or
   given a C1 branch (twin: cover-exhaustion K = 14, 17; the 0-40 ms lateness stress now asserts every
   stop is a hand C1 stop). If JB-149's overrun rate makes this common, the middle ground is a shorter
   (not zero) grace for a fast upward Hand: the 1.42 margin admits about 8 ms.

## Fix

- `canbridge_config.h`: `HAND_C1_STOP_MIN_UP_VEL_RPS = 50` + static_assert; FW_VERSION 36 -> 37 with
  the `// 36→37:` history line. No PROTOCOL_VERSION change (no wire change).
- `leg_interp.cpp`: `hand_c1_stop_eval` / `hand_c1_stop_duration`; `SchedGroup.c1_stop`; in
  `sched_advance` the hand group (`first == HAND_AXIS`) checks at `now - t0 >= SCHED_COVER_US` whether
  the cover-end knot velocity v2 > threshold and if so enters STOP at the cover end with entry (u2, v2);
  `sched_eval_axis` and STOP -> HOLD dispatch on `c1_stop`. Census `interp_sched_hand_c1_stops()`
  (not uplinked). The legs call and the below-threshold hand path run the FW 36 code unchanged.
- Twin `teensy_interp.py` mirrors it line for line; `rpc_args.EXPECTED_BRIDGE_FW_VERSION` 36 -> 37.

**One 500 Hz tick at the cover end** (cover end t_c on the tick grid, no newer frame due).
FW 36 / unchanged path: promote nothing; `now - t0 = 50 ms < 70 ms`, stay PLAY; emit span 2 at s = 1
(u2, v2, a(1)); the next ten ticks emit s = 1.08..1.72 (the grace); at t_c + 20 ms the C2 stop starts
from the s = 1.8 state. FW 37, Hand v2 > 50: promote nothing; `now - t0 >= 50 ms` and v2 > 50, so ep/ev
= (u2, v2), t0 = t_c, STOP; emit hand_c1_stop_eval(0) = (u2, v2, -4500). Against the previous sample
(s = 0.92) position and velocity are continuous and a_cmd steps to -4500; the next tick is v2 - 9
rev/s. An on-time frame stamped at t_c is promoted before the advance and the stop never starts.

## Verification

- Native firmware (2026-11-03, `python -m pytest -q -p no:cacheprovider tests/firmware/test_native_firmware.py`):
  **17 passed in 221.06 s**; the compiled `temp/firmware_native/test_leg_interp` reports **62/62 test
  cases, 9589/9589 assertions**. Five new cases: +160 rev/s Hand stops at the cover end with no grace,
  first stop sample = (u2, v2, -4500) exactly, rest at u2 + V^2/(2A) + A^3/(24 J^2) within 2e-3 rev
  (9.876 rev, inside the clip); +30 rev/s keeps the 10-tick grace and the jerk-bound C2 stop (hold at
  3.1 + 15 sqrt(6V/J)); -160 rev/s keeps the grace; a leg lane at +60 rev/s keeps the grace; and the
  accepted price (a frame landing 4 ms after the C1 stop started is refused while armed, latched).
  There is no PlatformIO `native` environment in `platformio.ini`; the repo's native harness is
  `tests/firmware/native/` driven by that pytest wrapper.
- Build (2026-11-03, `pio run -e teensy41` in `ros_ws/src/jugglebot/Teensy_code_canbridge/`, no upload):
  **SUCCESS in 101.64 s**, `.pio/build/teensy41/firmware.hex`; the boot banner and BridgeIdentity
  will report FW_VERSION **37**, PROTOCOL_VERSION 10.
- Python (2026-11-03, `python -m pytest -q -p no:cacheprovider tests/teensy_link/
  tests/firmware/test_bridge_fw_version_xref.py tests/firmware/test_sched_c2_twin.py
  tests/firmware/test_hand_torque_ff_twin.py tests/firmware/test_leg_clamp_age_aware_twin.py
  tests/firmware/test_hermite_xref.py tests/ros/test_teensy_bridge_node_install_skew.py
  tests/ros/test_teensy_bridge_node_bridge_diag.py tests/sim/test_logbook_front_matter.py
  tests/sim/test_docs_links.py`): **611 passed, 1 skipped in 38.32 s**.
- Skill-stack sim gate, which imports the twin (2026-11-03, `python -m pytest -q -p no:cacheprovider
  tests/sim/test_skills_gate.py`): **42 passed in 343.97 s**.
- The default gate (`./run_tests.sh`) was NOT run in this worktree; it is owed before merge to main.

**Owed:** the flash (`pio run -e teensy41 -t upload`, launch down; receipt = BridgeIdentity FW 37) and
a bench-hand Sitting that kills the stream mid-upstroke and checks the stop height and the hold (see
the JB-145 item's Sitting section once filed). Until then the live board is FW 36 and the launch logs
the BRIDGE_FW_CHECK skew advisory (37 expected).
