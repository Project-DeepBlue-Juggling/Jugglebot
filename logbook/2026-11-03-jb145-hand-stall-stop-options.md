---
title: "JB-145: can-bridge FW 37 stops a fast upward Hand inside the stroke when the scheduled stream runs dry (6 ms short grace, then a C1 entry at 4500 rev/s^2, jerk-limited ramp-out; built and native-tested, NOT flashed; bench-hand Sitting owed)"
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
  - tests/hardware/hand_stream_bench.py
  - tests/hardware/hand_stroke_cut.py
  - tests/motion/test_hand_stream_bench_stroke_cut.py
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
for the HAND lane only, when the cover ends with no newer frame the grace is shortened to
HAND_C1_STOP_GRACE_US (6 ms, the same continuation of the span-2 cubic, so a frame landing inside it
is promoted as before); if the continued Hand velocity is then upward above
HAND_C1_STOP_MIN_UP_VEL_RPS (50 rev/s) the stop starts there, position and velocity continuous with
the continued cubic, acceleration stepping to -4500 rev/s^2 (the
existing hand stop authority SCHED_STOP_ACCEL_HAND_RPS2), held to v = A^2/(2J) = 28.9 rev/s, then
ramped out at the stop jerk (350k rev/s^3) to rest and HOLD. Slow or downward Hands keep the rest of
the 20 ms grace and the C2 stop; every leg lane is unchanged. The first FW 37 build (3ad356e5, never
flashed) had no grace at all; the 6 ms grace is the owner's answer to its late-frame cost (below).

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
   Hand ODrive's position loop at the stop entry (with the 6 ms grace: +1738 to -4500 at cover + 6 ms,
   Apex 1.42 release knot), and with it a torque-FF step (J*2pi*4500 = 0.30 N m,
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
   (renamed `..._rides_the_short_grace_or_meets_the_c1_stop` with the grace)
   and the native `... the accepted price ...` case. Two existing tests that dropped a knot at a fast
   upward Hand were moved (native and the torque-FF twin: the dropped frame 17 -> 23, where the Hand
   moves down through a = 0) or
   given a C1 branch (twin: cover-exhaustion K = 14, 17; the 0-40 ms lateness stress now asserts every
   stop is a hand C1 stop). Superseded by the short grace below: the cost now applies only to a frame
   more than 6 ms late.

**The late-frame cost and the owner's grace decision (2026-11-03).** Tradeoff 2 is not rare: the
2026-11-02 Sitting shows 16 forward emit gaps over 50 ms (JB-149), each one a late knot that FW 36
rode inside its 20 ms grace and the no-grace FW 37 would have refused (latched hold, aborted Throw)
whenever it fell in the fast upward stroke. The owner kept the C1 stop at 4500 rev/s^2 and asked for
the LONGEST grace the Apex 1.42 margin permits, on the 2 ms tick grid, with at least 0.30 rev left
under the clip. Sized with the probe, which now models the firmware exactly (`--option 2 --grace-ms
G`: the cubic continues G ms, the fast-up decision reads the CONTINUED velocity, the C1 stop starts
from the continued state, the 0.031 rev ramp-out is added; 2026-11-03, `python
tools/probes/hand_stall_stop.py --option 2 --grace-ms G 1.05 1.42`):

| grace ms | 1.05 cont worst | 1.05 step worst | 1.42 step worst | 1.42 step margin to clip |
|---|---|---|---|---|
| 0 | 10.20 (knot +2) | 9.28 (+2) | 8.78 (+2) | +1.72 |
| 2 | 10.20 (+2) | 9.28 (+2) | 8.84 (release) | +1.66 |
| 4 | 10.20 (+2) | 9.28 (+2) | 9.31 (release) | +1.19 |
| 6 | 10.20 (+2) | 9.28 (+2) | 9.79 (release) | **+0.71** |
| 8 | 10.20 (+2) | 9.28 (+2) | 10.28 (release) | +0.23, misses 0.30 |
| 10 | 10.20 (+2) | 9.48 (release) | 10.76 (release) | -0.26, past the clip |
| 12 | 10.20 (+2) | 9.85 (release) | 11.24 (release) | -0.74, past the clip |

Apex 1.42 with the continuous release is refused by the planner, as before. **Chosen: 6 ms**
(3 ticks). The earlier estimate of about 8 ms came from the release-knot height without the
continued acceleration and the ramp-out; with both, 8 ms leaves +0.23 rev. The 1.05 continuous worst
(10.20 rev, +0.30) is knot +2, a Hand near the stroke top below the threshold that runs the FW 36
grace + C2 path at every G: unchanged by this decision, and identical in the FW 36 lane. A late
frame up to 2T + 6 ms = 56 ms after the previous frame's stamp now rides through the fast upstroke;
frames 6-15 ms late there are still refused (the accepted residue; JB-149 is the cure for the rate).

**Why the decision reads the continued velocity, not the knot v2.** Two reasons. (a) The C1 profile
needs an entry speed above v_r = 28.9 rev/s; the static_assert ties the threshold (50) to v_r, so
deciding on the state the stop actually starts from keeps every entry well formed, whereas deciding
on v2 could start a C1 stop from a continued speed below v_r (the Apex 1.42 knot +1 falls from 64 to
34 rev/s in 8 ms). (b) A Hand that crosses 50 rev/s upward during the short grace now C1-stops there
rather than riding the full 20 ms grace: Apex 1.05 step knot -2 drops from 4.63 to 1.47 rev at 6 ms.
The cost is that a Hand with v2 > 50 that has slowed below 50 by the decision instant keeps the full
grace and the C2 stop (the FW 36 path); at 6 ms in the probe's window that is only Apex 1.42 step
knot +1 (64 rev/s at the cover end, about 41 at the decision), which stops at 8.77 rev.

## Fix

- `canbridge_config.h`: `HAND_C1_STOP_MIN_UP_VEL_RPS = 50` + static_assert; FW_VERSION 36 -> 37 with
  the `// 36→37:` history line. No PROTOCOL_VERSION change (no wire change).
- `canbridge_config.h`: `HAND_C1_STOP_GRACE_US = 6000` with static_asserts that it is shorter than
  SCHED_GRACE_S and a whole number of INTERP_PERIOD_US ticks; the `// 36→37:` line names the grace.
- `leg_interp.cpp`: `hand_c1_stop_eval` / `hand_c1_stop_duration`; `SchedGroup.c1_stop`; in
  `sched_advance` the hand group (`first == HAND_AXIS`) checks at `now - t0 >= HAND_C1_DECIDE_US`
  (cover + 6 ms) whether the span-2 cubic continued to s = HAND_C1_S = 1.24 moves up faster than the
  threshold and if so enters STOP there (t0 += HAND_C1_DECIDE_US, the exact instant) with entry =
  that continued (p, v); otherwise it falls through to the unchanged exhaustion at cover + 20 ms. The
  check repeats each tick until then but reads the same fixed instant, so it cannot flip. Until the
  decision instant the group is PLAY and `play_eval` continues the cubic, so a frame due in that
  window is promoted by `sched_apply` exactly as in the FW 36 grace;
  `sched_eval_axis` and STOP -> HOLD dispatch on `c1_stop`. Census `interp_sched_hand_c1_stops()`
  (not uplinked). The legs call and the below-threshold hand path run the FW 36 code unchanged.
- Twin `teensy_interp.py` mirrors it line for line; `rpc_args.EXPECTED_BRIDGE_FW_VERSION` 36 -> 37.

**The 500 Hz ticks after the cover end** (cover end t_c on the tick grid, no newer frame due).
FW 36 / unchanged path: promote nothing; `now - t0 = 50 ms < 70 ms`, stay PLAY; emit span 2 at s = 1
(u2, v2, a(1)); the next ten ticks emit s = 1.08..1.72 (the grace); at t_c + 20 ms the C2 stop starts
from the s = 1.8 state. FW 37: ticks t_c, t_c + 2, t_c + 4 ms are identical to FW 36 (PLAY, s = 1,
1.08, 1.16; a frame due on any of them is promoted first, as a PLAY frame). Tick t_c + 6 ms: promote
nothing; `now - t0 >= 56 ms`; the cubic at s = 1.24 moves up faster than 50 rev/s, so ep/ev = that
state, t0 = t_c + 6 ms, STOP; emit hand_c1_stop_eval(0) = (p, v at s = 1.24, -4500). Against the
previous sample (s = 1.16) position and velocity are continuous (the same cubic) and a_cmd steps to
-4500; the next tick is v - 9 rev/s. A frame landing on or after that tick meets the stop.

## Bench-hand rung

`tests/hardware/hand_stream_bench.py --stage stroke_cut` (pure half `tests/hardware/hand_stroke_cut.py`,
tests `tests/motion/test_hand_stream_bench_stroke_cut.py`). It needs FW 38 and NO BALL, with the launch down.
The existing stages stream pre-v8 frames, which play on the legacy FW 17 lane and never reach the
FW 37 code, so this stage streams HAS_SCHED frames (knot_epoch_us one knot ahead). It runs a scaled
C2 stroke, because the production 150-190 rev/s at 4500 rev/s^2 is out of reach for a bench drive
capped at about 10 A (an assumption; the cap is not recorded for the bench Hand rig). After the
first frame whose continued velocity at cover end + 6 ms passes `--cut-at-vel`, it withholds the
hand channel and keeps the legs streaming. Flags: `--stroke-peak-vel` (70 rev/s), `--stroke-top`
(8.5 rev), `--stroke-accel` (800 rev/s^2), `--cut-at-vel` (55), `--cut-mode upstroke|downstroke|slow`,
`--repeat N`, `--cut-max-dev` (1.9 rev, the belt against the twin-modelled stop inside the cut window).
Parked at 0.2 rev, the default upstroke cuts after frame 21 with a C1 entry at 56.8 rev/s and a modelled hold
at 2.619 rev; the FW 36 path would hold at 4.531.

Recommended pass criteria, scored by the stage itself:
- upstroke: commanded peak (HAND_CMD_ECHO) at most the modelled stop end + 0.1 rev, under
  10.0 rev, and below the midpoint to the FW 36 hold; `sched_hand_c1_stops` +1 per run;
  `sched_stops` +1; no fault and no `sched_refused`; the encoder settled within 0.05 rev of
  the hold within 150 ms of the modelled stop end. Record the encoder overshoot; it is the
  bench drive's lag, not a firmware property.
- downstroke control (cut at -67 rev/s): no C1 count; the C2 hold within 0.1 rev of the model.
- slow control (peak 40, decision velocity 36.8): no C1 count; the hold within 0.1 rev of the
  model with the full 20 ms grace (2.536 rev), and above the midpoint to a stop at cover + 6 ms
  (1.674 rev). That separation is what makes the grace visible.

After each run the stage resumes CONTINUOUSLY at the echoed hold. A refusal is latched and the
stage disarms at the hold. It then returns to the start, so every disarm happens from a hold:
the bridge does not flush vel_ff on a disarm, and a disarm while moving would leave the Hand
at input_pos + vel_ff/35. Offline, the stage's own loop
(`_run_stroke_cut`) ran against the firmware twin through the real pump and wire struct. It ran
two repeats in each mode, and every criterion passed (2026-11-03, scratchpad probe, not committed).

## Verification

Short-grace build (this revision of FW 37):

- Native firmware (2026-11-03, `python -m pytest -q -p no:cacheprovider tests/firmware/test_native_firmware.py`):
  **17 passed in 484.88 s** (the box was also building the Teensy image); the compiled
  `temp/firmware_native/test_leg_interp` reports **63/63 test cases, 9627/9627 assertions** (the six
  FW 37 cases: 1262 assertions). Changed or new: a +150 rev/s Hand rides exactly 3 grace ticks on the
  continued parabola, then the first stop sample at cover + 6 ms is the continued (p, v) with a = -4500
  and the hold lands at P + V^2/(2A) + A^3/(24 J^2) within 2e-3 rev (inside the clip); NEW, a frame
  landing 4 ms after its stamp, armed, is promoted inside the short grace (PLAY throughout, target on
  the plan, no stop, no refusal); the accepted price now stages its frame 8 ms late, after the C1 stop
  started, and is refused and latched. The +30, -160 and leg cases are unchanged. After a
  comment-only edit to leg_interp.cpp, `python -m pytest -q -p no:cacheprovider
  tests/firmware/test_native_firmware.py::test_native_leg_interp_binary_passes` (2026-11-03):
  **1 passed in 303.55 s**, binary again 63/63 cases, 9627/9627 assertions.
- Python (2026-11-03, `python -m pytest -q -p no:cacheprovider tests/teensy_link/
  tests/firmware/test_bridge_fw_version_xref.py tests/firmware/test_sched_c2_twin.py
  tests/firmware/test_hand_torque_ff_twin.py tests/firmware/test_leg_clamp_age_aware_twin.py
  tests/firmware/test_hermite_xref.py tests/ros/test_teensy_bridge_node_install_skew.py
  tests/ros/test_teensy_bridge_node_bridge_diag.py tests/sim/test_logbook_front_matter.py
  tests/sim/test_docs_links.py`): **613 passed, 1 skipped in 64.50 s**. The twin's dropped-knot case is
  now `..._rides_the_short_grace_or_meets_the_c1_stop` at 1, 3, 5, 7, 8 and 15 ms: up to 7 ms (the
  first tick past cover + 6 ms is at 7.7 ms in that driver) the Hand rejoins the plan exactly with no
  stop and no refusal; at 8 and 15 ms the C1 stop is anchored at cover + 6 ms exactly and the late
  frame is refused (latched). The constants test pins HAND_C1_STOP_GRACE_US against the header.
- Build (2026-11-03, `pio run -e teensy41` in `ros_ws/src/jugglebot/Teensy_code_canbridge/`, no
  upload): **`teensy41 SUCCESS 00:01:46.501`**, and again after the comment-only edit
  **`teensy41 SUCCESS 00:00:16.386`** (incremental, leg_interp.cpp recompiled); no warning from leg_interp.cpp or canbridge_config.h
  (the static_asserts on the grace hold). FW_VERSION stays **37** (the no-grace build was never
  flashed), PROTOCOL_VERSION 10.
- Skill-stack sim gate, which imports the twin (2026-11-03, `python -m pytest -q -p no:cacheprovider
  tests/sim/test_skills_gate.py`): **42 passed in 1572.21 s** (load average about 12 from other
  sessions; 343.97 s on the first build).
- Bench-hand stage (2026-11-03, `python -m pytest -q -p no:cacheprovider
  tests/motion/test_hand_stream_bench_stroke_cut.py tests/teensy_link/test_hand_stream_bench_gap.py
  tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py`): **96 passed in 8.29 s**.
  The new file checks the profile bounds and C2 continuity, the cut frame, the C1 hold against
  `hand_stall_stop.py --option 2 --grace-ms 6` (to 1e-9 rev) and the C2 hold against its
  baseline, the refusals, and the real pump through the wire struct into the twin (hold within
  2e-3 rev of the model, the resume accepted, an off-hold resume refused).
- The default gate (`./run_tests.sh`) was NOT run in this worktree; it is owed before merge to main.

First FW 37 build (3ad356e5, no grace), for the record: native 17 passed in 221.06 s (62/62 cases,
9589 assertions); the python set above 611 passed, 1 skipped in 38.32 s; `tests/sim/test_skills_gate.py`
42 passed in 343.97 s; `pio run -e teensy41` SUCCESS in 101.64 s (all 2026-11-03).

**Owed:** the flash (`pio run -e teensy41 -t upload`, launch down; receipt = BridgeIdentity FW 37) and
a bench-hand Sitting that kills the stream mid-upstroke and checks the stop height and the hold,
and that one late knot (under 6 ms past the cover end) during the upstroke rides through (see
the JB-145 item's Sitting section once filed). Until then the live board is FW 36 and the launch logs
the BRIDGE_FW_CHECK skew advisory (37 expected).
