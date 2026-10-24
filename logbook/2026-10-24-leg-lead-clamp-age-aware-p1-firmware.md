---
title: "Leg lead clamp P1: the can-bridge leg lead clamp becomes frame-age aware (FW 31) — dead-reckoned anchor through an encoder-frame gap, two per-leg counters on a new 1 Hz uplink, twin mirror and the 2026-10-24 staircase replay oracle"
type: investigation
date: 2026-10-24
status: in-progress
phase: "leg-lead-clamp-age-aware — P1 (firmware)"
related_plan:
  - leg-lead-clamp-age-aware.md
  - leg-bus-frame-drops.md
  - two-ball-oval.md
related_entries:
  - 2026-10-24-two-ball-oval-p3-ab-sittings-skid-bound.md
  - 2026-08-15-fw14-validated-arc-closed.md
sessions: []
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - teensy_link/protocol.py
  - teensy_link/__init__.py
  - teensy_link/rpc_args.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tools/probes/teensy_link_profiling/hermite_xref/teensy_interp.py
  - tools/probes/lead_clamp_staircase_probe.py
  - tools/probes/data/lead_clamp_staircase_2026-10-24.json
  - tools/probes/README.md
  - tests/firmware/native/test_leg_interp.cpp
  - tests/firmware/test_leg_clamp_age_aware_twin.py
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/teensy_link/test_leg_clamp_diag_codec.py
  - tests/ros/test_teensy_bridge_node_leg_clamp_diag.py
  - plans/active/leg-lead-clamp-age-aware.md
subsystem:
  - can
  - motion
  - tools
tags:
  - safety
  - testing
---

# Leg lead clamp P1 — frame-age aware

## Summary

**P1 complete; firmware BUILT (FW 31), NOT FLASHED.** The can-bridge leg lead clamp now anchors to a
dead-reckoned encoder while the encoder frame is aged: `anchor = pos_rev + (raw command now − raw command
at the tick the frame landed)`, the shift frozen past `MOTOR_FB_STALENESS_US` (150 ms), reset to exactly
0.0f by every fresh frame — so at age 0 the clamp is FW 30 bit for bit, and through the 76-108 ms
encoder-frame gaps that produced the 2026-10-24 staircases its verdict is frozen instead of pinning the
command at the stale reading + 7.07 mm. Owner decision 2026-10-24 (one AskUserQuestion round): form Z
over the archived plan's velocity-extrapolated X and the widen-and-hold Y, because the replay of the five
bag episodes showed X and Y leave ~75 % of the step (the legs accelerate through the gap) while Z leaves
none while the leg tracks; cap 150 ms; new additive 1 Hz uplink `LegClampDiag` 0x94 with two per-leg
cumulative counters (no `PROTOCOL_VERSION` bump); the twin gains a firmware leg-clamp MODE with the xref's
motor_guard default untouched. Verification: 52/52 native doctest cases on the compiled ISR (five new),
the twin replay oracle on a committed cut of the three main episodes (FW 30 replays 11.6 / 12.4 /
14.9 mm steps against the bag-recorded 11.6 / 12.4 / 12.4; FW 31 cuts them 77 / 31 / 74 % with zero
engagement on any gap tick whose frame-time error was inside the band), xref 0.0 rev before and after, `pio run -e teensy41` SUCCESS. Numbers and triples
in § Verification. P2 (bench) is cleared; the replay half of it is prototyped here.

## Motivation

Three single-leg ~10 Hz vibration events on 2026-10-24, read off the raw bag samples in
`2026-10-24-two-ball-oval-p3-ab-sittings-skid-bound.md` § "The vibration events":

| event | bag | epoch | leg | planner | iq setpoint peak |
|---|---|---|---|---|---|
| S1 A | `~/Desktop/rosbags/2026-10-24_10-44-21/` | 1792799742.4 | 1 | pose 0.65 | 9.8 A |
| S1 B | same | 1792799921.7 | 2 | pose 0.50 | **14.99 A** (the 15 A clamp) |
| S2 | `~/Desktop/rosbags/2026-10-24_11-34-29/` | 1792802191.8 | 1 | level 0.50 | **14.99 A** |

The commanded setpoint echo was smooth in every event; the leg's encoder feedback arrived
every ~90-100 ms instead of every ~10 ms; and the executed target sat at exactly the stale
encoder reading + 7.07 mm (`MAX_LEAD_REV` = 0.10 rev) until the next frame landed, then
jumped 11-12 mm. The can-bridge leg lead clamp (`leg_interp.cpp` `interp_isr`) anchors to
`axes[i].pos_rev` with no regard for how old that reading is, so a leg-bus encoder-frame
drop turns a smooth stream into a staircase the ODrive position loop chases at full
current. The hand lane has had an age-aware anchor since FW 17 (`fb_ex = fb + vel * age`).
The owner made this plan a prerequisite of the next powered two-ball-oval Sitting and of
P4's 15 -> 30 A clamp raise.

## Design

### Ground truth re-read before writing anything (2026-10-24)

- Leg clamp: `leg_interp.cpp:1066-1072` (bare `axes[i].pos_rev` read, `dev` clamped to
  `+-LEAD`, `clamp_mask` bit i). Second clamp site: the re-enable recovery slew re-clamps
  the slewed command at `:1387-1391` against the same bare read. Hand lane anchor
  `:1194-1217` (`atomic_read_u64(&hand_axis().pos_timestamp_us)`, raw age kept, age capped
  at `MOTOR_FB_STALENESS_US` with the cap counted in `s_hand_stale_holds`).
- `MAX_DEVIATION` (`fault_machine.cpp:456-459`): `u0 = interp_base_pos(i)` against the raw
  `axes[i].pos_rev`, at 10 Hz. It never reads the clamp's anchor.
- Leg `MOTOR_FB_STALE` (`fault_machine.cpp:537-538`): age of `pos_timestamp_us` above
  `MOTOR_FB_STALENESS_US` (150 ms) suppresses output; recoverable.
- `atomic_read_u64` (`time_base.h:33`) is PRIMASK save/disable/restore: ISR-safe, and already
  called from the ISR by the hand lane. `snapshot_pos_vel`'s seqlock retry loop is NOT
  (archived § 3.3 item 3).
- Constants: `MAX_LEAD_REV` 0.10, `LEAD_CLAMP_VELFF_LIMIT_RPS` 3.5, `MOTOR_FB_STALENESS_US`
  150000, `HAND_DEV_FRESH_US` 30000, `FW_VERSION` 30 (`canbridge_config.h:209/219/295/319/55` as of
  `2bafbae7`; the FW 30→31 history line shifts the first four to 210/220/296/320 in the committed tree).
- Wire: `Teensy_code_canbridge/udp_protocol.h`, `config/generated/udp_protocol.{h,py}` and
  `docs/teensy-udp-protocol.md` are ALL generated by `config/generate_udp_protocol.py`
  (`MESSAGES` list, `MsgType` table at ~line 180; next free uplink id 0x94 in block 2).
  The plan's "row in `config/protocol_config.yaml`" was a drift: that file holds CAN
  constants, not UDP messages. `teensy_link/protocol.py` re-exports the generated classes;
  `teensy_link/__init__.py` re-exports them again. `teensy_link/rpc_args.py`
  `EXPECTED_BRIDGE_FW_VERSION` is pinned to `FW_VERSION` by
  `tests/firmware/test_bridge_fw_version_xref.py`.
- Twin: `teensy_interp.py` `MAX_LEAD_REV = 0.15` is NOT a stale copy of the firmware — it is
  motor_guard's value, on purpose: the leg block mirrors motor_guard and the xref
  (`tests/firmware/test_hermite_xref.py`) pins it to 0.0 rev, with a `lead_clamp_big_jump`
  case that engages the clamp. Changing the constant alone would fail the xref. The plan's
  § 7 note is refreshed below.
- Bridge node republish: a 1 Hz cumulative-counter frame rides `/link_status` as a KeyValue
  row re-rendered at 10 Hz (`_bridge_tx_diag_str`, `teensy_bridge_node.py:4390`), the
  precedent `can3_errors` / `bridge_tx_diag` share; a bag records it through that topic.

### The measured basis on THIS robot (probe, 2026-10-24)

`/tmp/probe_lead_clamp_staircase.py` (one-off; promoted to
`tools/probes/lead_clamp_staircase_probe.py` for P2/P3) on the two bags, reusing the A/B
review's decoded pickles for `/robot_state`, `/leg_cmd_executed`, `/leg_setpoint_echo` and
decoding `/cache_diag` fresh. Log: `temp/probes/lead_clamp_age_aware_probe.log`.

Staircase signature (executed target frozen for consecutive ~10 ms samples while
`target_vel` > 0.5 rev/s, runs merged within 0.3 s, >= 3 runs): five episodes, all on
legs 1 and 2, none on legs 0/3/4/5.

| bag | leg | epoch | dur | steps | step med / max | encoder interval in episode | `/cache_diag` age_max that second | iq setpoint |
|---|---|---|---|---|---|---|---|---|
| 10-44-21 | 1 | 1792799742.51 | 0.39 s | 6 | 5.9 / 11.6 mm | median 96 ms, max 108 | 95.1 ms (leg 1), 3-9 ms others | 9.8 A |
| 10-44-21 | 1 | 1792799908.88 | 0.18 s | 3 | 8.5 / 11.8 mm | median 21 ms, max 87 | 14.1 ms | 12.7 A |
| 10-44-21 | 2 | 1792799921.77 | 0.84 s | 7 | 10.6 / 12.4 mm | median 100 ms, max 108 | 97.1 ms (leg 2) | **14.99 A** |
| 11-34-29 | 1 | 1792802191.80 | 0.25 s | 4 | 8.1 / 12.4 mm | median 90 ms, max 91 | 99.1 ms (leg 1) | **14.99 A** |
| 11-34-29 | 1 | 1792802192.54 | 0.31 s | 4 | 4.6 / 5.2 mm | median 94 ms, max 105 | 99.1 ms | 7.2 A |

Anchor-age separation (the archived plan's premise, now measured here): `/cache_diag`
`age_max_us` per leg over the whole 10-44-21 bag — median 4.1-6.1 ms, p95 9.2-10.1 ms,
max 95-99 ms; `age_min_us` median 3-5 ms. Healthy peak age stays at or below one 10 ms
refresh; a dropout is 76-108 ms. An order of magnitude, with nothing in between.

First-cut replay at the ~100 Hz telemetry grid (raw command = `/leg_setpoint_echo`
shifted by the fitted pipeline lag, 0-22 ms; anchor = the last fresh `/robot_state`
sample; today's clamp re-derived from that reproduces the recorded target to 0.3-1.6 mm
RMS — the oracle check), cap 150 ms:

| option | S1 A leg 1 pin / max step | S1 B leg 2 | S2 leg 1 | anchor shift from frozen fb, max (bind exposure) |
|---|---|---|---|---|
| today | 0.29 / 11.6 mm | 0.47 / 12.4 mm | 0.31 / 12.7 mm | 0 |
| X extrapolated anchor | 0.18 / 8.8 mm | 0.34 / 9.5 mm | 0.19 / 8.4 mm | 10-27 mm |
| Y widen-and-hold | 0.18 / 8.8 mm | 0.34 / 9.5 mm | 0.19 / 8.4 mm | 10-27 mm |
| Z dead-reckoned anchor | 0.14 / 4.0 mm | 0.28 / 6.1 mm | 0.19 / 5.6 mm | 10-19 mm |

(This is the first-cut 100 Hz per-sample replay used to CHOOSE the form; the 500 Hz twin oracle below,
run after the choice, gives 2.6 / 8.6 / 3.8 mm for the same three windows under the chosen form.)

X and Y are identical on this data (no reversals: the command and the encoder velocity
agree in sign). Both leave ~75 % of the step because the leg ACCELERATES through the gap
(fresh-sample acceleration 42-200 rev/s^2 at the episodes) and a frozen velocity cannot
follow it; the archived D3 gate said the same thing in other words (anchor error p95
0.129 -> 0.064 rev, halved, not removed). Z's residual pins are the replay's own
reconstruction error plus the lag today's clamp had ALREADY built into the recorded
encoder trace (the replay cannot un-happen the staircase); by construction Z's clamp
verdict does not change between frames. Caveat: a 100 Hz replay of a 500 Hz clamp with a
10 ms-quantised age; P2 runs the twin at 500 Hz.

### The three candidate forms

Common to all three: nothing changes at age 0 (bit-identical to today), the band width is
`2 x MAX_LEAD_REV`, the age is read ONCE per leg per tick through `atomic_read_u64` and
capped at `MOTOR_FB_STALENESS_US` (the fault task's `MOTOR_FB_STALE` suppression is the
terminating arm, as for the hand), `vel_ff` is untouched (interpolated, capped at
`VELFF_CAP` 3.5 rev/s), the hand lane is untouched, and the recovery-slew re-clamp uses
the SAME per-leg anchor computed once per tick (archived § 3.3 item 1: one derivation).

- **X — velocity-extrapolated anchor** (archived frame-drops plan § 3.2): anchor =
  `pos + vel * min(age, cap)`. Prevents the frozen-anchor stop at constant velocity.
  Leaves the acceleration residual (measured above), and on a reversal mid-gap it MOVES
  the command to `anchor - LEAD`, a position ahead of the real leg that nobody commanded.
- **Y — widen-and-hold**: anchor stays `pos`, band = `LEAD + |vel| * min(age, cap)`.
  Same coverage as X on the measured data, no reversal manufacture (it only relaxes,
  never relocates), same acceleration residual.
- **Z — dead-reckoned anchor** (archived content-freshness draft § 4.2 Option B): anchor =
  `pos + (cmd_now - cmd_at_frame)`, the shift frozen once age > cap. The clamp's verdict
  is frozen at its last fresh value for the length of the gap: zero manufactured step
  while the leg tracks, no dependence on the ODrive velocity estimate's sign or accuracy.
  What it gives up: a runaway COMMAND that coincides with a gap on that leg passes
  unclamped for up to the cap (X/Y would hold it to `vel * age + LEAD`); `MAX_DEVIATION`
  (1.0 rev of u0 against the raw encoder, 10 Hz) and the ODrive's own velocity and
  current limits remain the bounds in that window.

Bind-plus-gap (the leg physically stops exactly as its frames stop): every option emits a
command that runs ahead of the stuck leg by the dead-reckoned or extrapolated travel
(10-27 mm measured on these windows, bounded by `v * cap`), then snaps back by that amount
at the next frame. Today's clamp holds that case at 7 mm. This is the safety-envelope
trade (archived A1): coast coverage of the frequent gap against a shove into the rare
bind, and the cap is the knob.

### One 500 Hz tick, three ages (leg i, v = 3.0 rev/s = 212 mm/s, healthy tracking lag 0.045 rev)

Per tick the raw ladder gives `cmd` (advancing 0.006 rev/tick) and `cmd_vel` = 3.0.

| | age 0 (frame decoded this tick) | age 80 ms (40 ticks dark; leg really at fb + 0.24) | age >= 150 ms (cap; 75+ ticks dark) |
|---|---|---|---|
| today | anchor fb; dev 0.045; emit cmd | dev 0.285 -> **pinned at fb + 0.10**, 10 mm behind the leg: the staircase | pinned at fb + 0.10 until the fault task suppresses output (10 Hz: 150-250 ms) |
| X | `fb + v*0.0f` == fb exactly; identical | anchor fb + 0.24; dev 0.045; emit cmd. With 70 rev/s^2 of acceleration the leg is at fb + 0.46, anchor still fb + 0.24 -> clamped at fb + 0.34, 9 mm behind: the residual step | anchor frozen at fb + 0.45; band stops following; re-pins as cmd advances; suppression as today |
| Y | `LEAD + |v|*0.0f` == LEAD exactly; identical | band 0.34; dev 0.285 inside; emit cmd. With acceleration dev 0.51 > 0.34 -> clamped at fb + 0.34, same residual | band frozen at 0.55; re-pins; suppression as today |
| Z | ts changed -> `cmd_ref = cmd`, shift 0; anchor fb; identical | shift = cmd - cmd_ref = 0.24 (0.46 with acceleration); anchor follows the command exactly; dev 0.045; emit cmd — no step | shift frozen at its 150 ms value; re-pins as cmd advances; suppression as today |
| `vel_ff` path | interpolated 3.0, cap 3.5 | unchanged in all options | unchanged |
| `MAX_DEVIATION` | u0 - fb = 0.045 | u0 - fb = 0.285 (the frozen fb; < 1.0) — never reads the anchor, exposure unchanged by any option | u0 - fb = 0.5 at 150 ms; trips only past 1.0 rev (333 ms at 3 rev/s), after suppression |
| bind at the frame (leg stuck at fb) | — | today holds the shove at 0.10 (P-term 4 rev/s); X/Y/Z let cmd run to fb + 0.285 (P-term 11.4 rev/s, near `vel_limit` 12, current-limit bounded) and snap back ~0.24 rev at the next frame | the cap bounds it: 0.45 rev at 150 ms, 0.36 at 120, 0.30 at 100 |

Recovery after a gap under Z: the first fresh frame resets the shift to 0 and the anchor
to the live encoder; if the leg tracked, `dev` is the honest tracking error and nothing
jumps; if it did not, the clamp re-engages at `fb_new +- LEAD` exactly as today would.

### Counters and wire

Two per-leg u32 counters, cumulative since boot (the census idiom; the consumer
differences two reads, nothing is read-then-cleared): `lead_clamp_ticks[6]` (ticks the
clamp engaged, counted only when the command reached the wire — `out_en && !coldstart`,
the FW 18 hand-lane gate) and `stale_anchor_ticks[6]` (ticks whose anchor age exceeded
`LEG_ANCHOR_STALE_US`). New additive 1 Hz uplink `LegClampDiag` (`MsgType` 0x94, 48 B)
from `task_telem` in the `BridgeTxDiag` pattern; no `PROTOCOL_VERSION` bump (an old
Jetson ignores the unknown msg_type); `FW_VERSION` 30 -> 31. `/link_status` KeyValue row
`leg_clamp_diag`, rendered like `bridge_tx_diag`, never-seen explicit.

## Implementation

- **Firmware (FW 31, `leg_interp.cpp`):** `leg_clamp_anchor(i, raw_cmd, now_mono)` — the single anchor
  derivation (archived content-freshness § 6.1): one `atomic_read_u64` of `pos_timestamp_us` (PRIMASK
  save/restore, the hand lane's ISR-safe accessor), one bare `pos_rev` read, the per-leg
  `s_leg_anchor_ts / s_leg_cmd_ref / s_leg_dr` state (ISR-only, plain), the stale census
  (`age > LEG_ANCHOR_STALE_US`, ungated, never for `ts == 0`). The lead clamp loop computes `anchor[i]`
  once per tick; the recovery-slew re-clamp consumes the same `anchor[i]` (a second bare read there would
  re-pin through a gap what the primary clamp let coast — archived § 5.2 site 3). Engaged ticks counted
  under `out_en && !coldstart` (the FW 18 hand-lane gate); `interp_reset` zeroes all five arrays;
  `interp_leg_lead_clamp_ticks(i)` / `interp_leg_stale_anchor_ticks(i)` accessors. No new FreeRTOS call,
  no new IRQ-off region beyond the existing accessor, no allocation: six atomic u64 reads and ~10 float
  ops per tick. Hand lane untouched. `FW_VERSION` 30 → 31 with its history line; `LEG_ANCHOR_STALE_US`
  30000 with a `static_assert` inside the cap.
- **Wire:** `config/generate_udp_protocol.py` gains `MsgType.LEG_CLAMP_DIAG` 0x94 (the next free id in
  block 2) and `Message LegClampDiag` (`u32 lead_clamp_ticks[6]`, `u32 stale_anchor_ticks[6]`, 48 B);
  regenerated `udp_protocol.h` (firmware + `config/generated/`), `udp_protocol.py`,
  `docs/teensy-udp-protocol.md`, the jetson profiling copy. `telemetry.cpp::leg_clamp_diag_uplink_step` at
  1 Hz from `task_telem` (the `BridgeTxDiag` emitter pattern, `static_assert` on the array widths), called
  in the `.ino` telemetry loop. No `PROTOCOL_VERSION` bump: additive, an FW ≤ 30 board never sends it.
- **Jetson:** `teensy_link` re-exports `LegClampDiag` / `LEG_CLAMP_DIAG_SIZE`; `rpc_args.EXPECTED_BRIDGE_FW_VERSION`
  31; `teensy_bridge_node` subscribes, caches under the lock in a broad-except RX callback (a short payload
  raises `struct.error`, not `ValueError`), renders `/link_status` row `leg_clamp_diag` =
  `clamp a b c d e f | stale a b c d e f` or `unknown (never seen)`, published unconditionally after
  `bridge_tx_diag` — so every bag records the census at 10 Hz.
- **Twin (`teensy_interp.py`):** `TeensyLegInterp(leg_clamp="motor_guard" | "firmware_fw30" | "firmware")`,
  `tick(t_now, fb_pos, fb_ts=None)`; the firmware modes carry `MAX_LEAD_REV_FW` 0.10, capped (not zeroed)
  `vel_ff`, and `firmware` mirrors `leg_clamp_anchor` line for line with the two census lists. The default
  is byte-identical to before (xref 0.0 rev before and after); `MAX_LEAD_REV = 0.15` keeps its honest role
  (motor_guard's value) and its comment no longer calls it a firmware copy.
- **Probe + fixture:** `tools/probes/lead_clamp_staircase_probe.py` (the staircase scan, the encoder-interval
  and `/cache_diag` age statistics, the four-way per-sample replay, `--emit-fixture`, `--fixture-only`, the
  `replay_window` 500 Hz twin driver the test imports); `tools/probes/data/lead_clamp_staircase_2026-10-24.json`
  (41 KB: the three main windows' echo knots, fresh encoder samples and recorded targets).
- **Tests:** five native doctest cases on the compiled ISR (`tests/firmware/native/test_leg_interp.cpp`):
  fresh-frame bit-identity to FW 30 (exact float equality) + never-seen axis; coast through a 100 ms gap
  (no pin, stale census 34-36, slew re-clamp uses the anchor, frame return with a tracking leg steps nothing,
  frame return with a BOUND leg re-clamps at `fb + LEAD`); cap freeze at 150 ms and reset on the next frame;
  census gating (engaged ticks gated on the wire, stale ticks not, `interp_reset` zeroes both); and the
  **pinned exposure**: a 1 rev command step landing while the frame is one tick aged passes unclamped and
  is re-clamped by the next frame. Twin: `tests/firmware/test_leg_clamp_age_aware_twin.py` (8: constants
  pinned to the header, motor_guard mode untouched, the age-0 hypothesis property, coast, cap freeze, three
  replay-oracle windows). Protocol: `tests/teensy_link/test_leg_clamp_diag_codec.py` (3, incl. the 47-byte
  refusal), `tests/ros/test_teensy_bridge_node_leg_clamp_diag.py` (4), `test_udp_protocol_xlang.py`
  parametrisations + the re-pinned wire-layout digest `b8f8027f…`.

## Verification

Every count below is (date, command, result); all runs 2026-10-24 in the project venv.

- **Inaugural baseline, before any edit:** `./run_tests.sh -q` on `2bafbae7` (`main`), log
  `temp/logs/lead-clamp-p1-baseline-gate.log`: parallel rc=0 in 353 s + serial rc=0 in 24 s,
  **RESULT PASS** (the wrapper's `-q` log carries no tally line; the last tallied full gate on
  this tree is the 13:12 `./run_tests.sh --full` at `80ecf4cd`: 7149 passed / 4 skipped +
  serial 8 passed, and the eight commits between are map items and docs).
- **Native firmware harness** (`python tests/firmware/native/build.py && ./temp/firmware_native/test_leg_interp`):
  before the new cases, the FW 31 clamp code under the 47 existing cases: **47 passed, 7698 assertions**;
  with the five new cases: **52 passed, 8052 assertions** (two fixture mistakes on the first run were
  mine — a slew state set one tick behind the command, and a 1 rev step staged during a gap, which the
  dead-reckoning absorbs by design; the second became the pinned-exposure case). `test_fault_machine`
  27 passed, 540 assertions.
- **Firmware build:** `pio run -e teensy41` (build only, launch untouched, NOT flashed), first build
  `temp/logs/lead-clamp-p1-pio-build-1.log` **SUCCESS 54.4 s**, exit 0; final tree
  `temp/logs/lead-clamp-p1-pio-build-final.log` — see the closing line below. Build warnings are the
  pre-existing three (`actuators_intact_and_holding` unused, FlexCAN `strncpy`, `-fno-rtti` on C).
- **Scoped battery after both units landed:** `pytest tests/firmware tests/teensy_link
  tests/ros/test_teensy_bridge_node_leg_clamp_diag.py tests/ros/test_teensy_bridge_node_bridge_diag.py -q
  --hypothesis-seed=0` (`temp/logs/lead-clamp-p1-scoped-tests.log`): **744 passed in 49.21 s**.
- **Twin regression:** `python tools/probes/teensy_link_profiling/hermite_xref/xref.py` before and after the
  twin edit: **PASS, worst |Δpos| 0.0e+00 rev** both times. `pytest tests/firmware/test_hermite_xref.py
  tests/firmware/test_sched_c2_twin.py tests/firmware/test_hand_torque_ff_twin.py
  tests/firmware/test_leg_clamp_age_aware_twin.py -q --hypothesis-seed=0`: **62 passed in 23.2 s**;
  `pytest tests/firmware/test_leg_clamp_age_aware_twin.py -q --hypothesis-seed=0 --hypothesis-profile=ci-deep`:
  **8 passed in 12.4 s**; `pytest tests/sim/test_skills_gate.py -q` (the twin's sim consumer, default
  mode): **34 passed in 228 s**.
- **Jetson consumer:** `pytest tests/teensy_link/test_leg_clamp_diag_codec.py
  tests/ros/test_teensy_bridge_node_leg_clamp_diag.py tests/ros/test_teensy_bridge_node_bridge_diag.py
  tests/firmware/test_udp_protocol_xlang.py tests/firmware/test_bridge_fw_version_xref.py tests/teensy_link -q`:
  **525 passed in 23.02 s**; `pytest tests/ros -q -k "link_status or bridge_node"`: **534 passed, 2783
  deselected in 110.71 s**. No test enumerates the `/link_status` row set exhaustively.
- **Currency:** `pytest tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py
  tests/sim/test_docs_links.py -q` after the plan edit + `python -m mapkit gen`: **29 passed**.

### Fails-before / passes-after (each test shown to drive its failure path)

| test | mutation (temporary, restored) | command | result |
|---|---|---|---|
| twin age-0 property (`test_age_zero_firmware_equals_fw30_and_never_wider`) | `anchor = fb + dr + (1e-3 if aged else 0.0)` | `pytest tests/firmware/test_leg_clamp_age_aware_twin.py -q --hypothesis-seed=0 -k age_zero` | 1 failed at `pa == pb` (Hypothesis shrank ~77 s); clean twin 8 passed |
| twin coast + the three replay windows | `dr = 0.0` (no dead-reckoning) | same file, `-q --hypothesis-seed=0` | 4 failed; S2 then reproduced FW 30 exactly (14.89 mm, 66.7 % engaged) |
| twin cap freeze | no freeze past the cap | same | 1 failed |
| twin frame-reset line | `dr` not zeroed on a frame tick | same | **0 failed — equivalent mutant** (the same tick recomputes `dr = 0`); recorded in Open Questions |
| bridge row renders every leg | renderer `d.lead_clamp_ticks[:5]` | `pytest tests/ros/test_teensy_bridge_node_leg_clamp_diag.py -q` | 2 failed, 2 passed |
| codec short-payload refusal | slice 47 → 48 bytes | `pytest tests/teensy_link/test_leg_clamp_diag_codec.py -q` | 1 failed, 2 passed |
| native FW 31 cases | the firmware's own first run against two wrong fixtures | `./temp/firmware_native/test_leg_interp` | 2 failed of 51 — both the fixture, documented above |

### Replay oracle (500 Hz twin on `tools/probes/data/lead_clamp_staircase_2026-10-24.json`)

A gap tick is one with frame age > 30 ms; "sub-band" = the frame-time tracking error was < 0.10 rev.

| window | FW 30 max step | FW 30 gap engagement | FW 31 max step | FW 31 gap engagement | FW 31 engaged on sub-band gap ticks |
|---|---|---|---|---|---|
| S1 A leg 1 (1792799742.5) | 11.55 mm | 49.4 % | 2.61 mm (−77 %) | 16.9 % | **0 of 261** |
| S1 B leg 2 (1792799921.8) | 12.39 mm | 76.2 % | 8.57 mm (−31 %) | 40.2 % | **0 of 226** |
| S2 leg 1 (1792802191.8) | 14.89 mm | 66.7 % | 3.81 mm (−74 %) | 37.7 % | **0 of 129** |

FW 30 replays 11.6 / 12.4 / 14.9 mm steps against the bag-recorded 11.6 / 12.4 / 12.4 (the S2 replay
over-reads the step by 2.5 mm: the twin latches the echo knots at their jittery arrival times, so its raw
command is not the one the bridge played to the millisecond). The remaining FW 31 engagement is every tick
whose frame-time error already exceeded the band (lag today's clamp built into the recorded trace) or
past the cap; see Discussion tradeoff 3.

- **Full gate on the final tree:** `./run_tests.sh --full` (2026-10-24 20:32-20:38, after the audit fixes, log
  `temp/logs/lead-clamp-p1-full-gate-final.log`): **parallel 7166 passed / 4 skipped in 322.68 s + serial 8 passed
  in 29.81 s, RESULT PASS** — +17 on the 7149 / 4 + 8 baseline (3 codec + 4 bridge-node + 8 twin tests + 2 xlang
  parametrisations). An earlier `./run_tests.sh --full -q` on the pre-audit tree (20:1x) also passed but its doubled
  `-q` suppressed the tally. The only edit after this run is this line. Final firmware build on the same tree:
  `pio run -e teensy41` SUCCESS, exit 0, text 250176 B (`temp/logs/lead-clamp-p1-pio-build-final.log`). NOT FLASHED.

## Discussion

**Why Z (dead-reckoned anchor) over the archived X.** The archived frame-drops plan recommended the hand
lane's `fb + vel × age`, with D3-gate support of anchor error p95 0.129 → 0.064 rev. Halved is not
removed: the per-sample replay of this robot's five episodes (§ Design) put X at 8.4-9.5 mm steps against
today's 11.6-12.7, because the leg is accelerating out of the catch through every one of these gaps
(fresh-sample acceleration 42-200 rev/s²) and `½·a·age²` over 100 ms is 0.2-0.35 rev — larger than the
band. No extrapolation from a frozen velocity can follow that. Z does not try: through a gap it holds
the clamp's VERDICT (the tracking error measured at the last fresh frame) constant, so a leg that keeps
tracking its command is never stepped, whatever its acceleration, and the ODrive velocity estimate's
sign and accuracy drop out of the clamp entirely. The twin replay at 500 Hz confirmed it: zero engagement
on every gap tick whose frame-time error was inside the band, on all three windows (0 of 261 / 226 / 129).

**What was ruled out.** Y (widen-and-hold) — identical to X on this data (no reversals), the same
acceleration residual. A plausibility envelope on the dead-reckoned shift (`|dr| ≤ |v|·age + ½·A_MAX·age²`)
— at the planner's 5000-8000 mm/s² the envelope over 100 ms is 0.4-0.6 rev, four to six bands wide, so it
bounds nothing but a literal step. A distance cap on the shift below the legit gap travel (35 mm at
350 mm/s over 100 ms; 60 mm at P4's 600 mm/s) — re-introduces pinning on exactly the fast moves the fix is
for. Growing the heartbeat for the counters — a `PROTOCOL_VERSION` bump and darkness against every FW ≤ 30
board for counters the consumer differences anyway; the additive message costs nothing.

**Tradeoffs accepted (owner, 2026-10-24).** (1) *Bind-plus-gap:* a leg that physically stops exactly as
its encoder frames stop is commanded ahead of itself by the gap's command travel (10-27 mm measured on
these windows, bounded by `v × 150 ms` ≈ 0.45 rev at 3 rev/s), with the P-term climbing toward
`vel_limit` for the gap's length, then snapped back at the next frame; today holds that case at 7 mm.
X and Y carry the same exposure; the cap is the knob (120 ms would have cut it 20 % at the price of a
re-pin window before the fault task's 150 ms suppression; no measured gap exceeded 108 ms, so the
difference was academic and hand parity won). (2) *Command discontinuity through a gap:* the shift
follows the RAW command, so a step in the command that lands while no fresh frame has arrived is
absorbed into the anchor and emitted unclamped until the next frame — one frame interval (1-10 ms
healthy, the cap at worst). FW 30 held it at `fb + LEAD` from the first tick. Pinned as a native test so it
stays visible; the bounds that remain are the stream's own continuity (every plan is feasibility-gated;
the recovery slew owns the enable edge; `interp_on_setpoint` drops non-finite frames), `MAX_DEVIATION`
(1.0 rev of u0 against the raw encoder, 10 Hz — its exposure is unchanged by this change because it
never read the clamp's anchor) and the drive's own velocity and current limits. A shift slew bound at the
drive's `vel_limit` rate (0.024 rev/tick) would close (2) to within one tick without touching legit
moves (≤ 8.5 rev/s at P4) and is the first P2 refinement to consider; it was not built here so the
flashed algorithm equals the owner-approved form and its twin line for line. (3) *The replay residue:*
FW 31 still engages on 17-40 % of gap ticks in the recorded windows — every one a tick whose frame-time
error already exceeded the band, i.e. lag today's clamp had built into the recorded trace before the
window. A replay cannot un-happen the staircase it is reading; P3's Sitting is where the number becomes
honest (gate: zero staircase episodes). S1B's step cut (31 %) sits below the 40 % the other two windows
clear for the same reason, and the oracle pins it at 25 % rather than loosening the others.

**The twin surprise.** The plan's § 7 called `teensy_interp.py`'s `MAX_LEAD_REV = 0.15` a stale firmware
copy to fix as a first commit. It is motor_guard's value, by design: the twin's leg block mirrors
motor_guard so the xref can prove the firmware's ladder against the validated Jetson interpolator, and
the `lead_clamp_big_jump` case engages the clamp. A bare 0.15 → 0.10 edit fails the xref. The mode switch
(default untouched, firmware modes pinned to the header) is what lets the twin be the oracle for the leg
clamp without breaking the proof it exists for — and `sim/skills_gate.py`, which runs the twin, keeps
its byte-identical default.

**Generated, not hand-edited.** The plan cited `config/protocol_config.yaml` for the new message; the UDP
wire is `config/generate_udp_protocol.py`, which writes the Teensy header copy, the Python module and the
docs page together. Refreshed in the plan; `test_committed_cpp_matches_generator` is what would have
caught a hand edit.

## Open Questions

- Workstream B (why the ODrives drop encoder frames on legs 1 and 2 under load) stays archived in
  `plans/archived/leg-bus-frame-drops.md` § 4; this phase makes a drop cost tracking accuracy, it does
  not remove the drops. The new `stale_anchor_ticks` census is the cheapest per-leg witness of the
  drops in every future bag, should B ever reopen.
- **Shift slew bound** (Discussion tradeoff 2): bounding the dead-reckoned shift's per-tick growth at the
  drive's `vel_limit` rate would hold a command discontinuity through a gap to within one tick. Not
  built; the first P2 refinement to weigh, with the twin and the native test updated in the same commit.
- **Bind-plus-gap** (tradeoff 1) has no bench measurement yet; P2's injected-gap run should include one
  bound leg (stroke end) with frames withheld, read against the 15 A clamp.
- **Replay residue** (tradeoff 3): the honest FW 31 engagement number needs an FW 31 bag — P3.
- The `/tmp/probe_lead_clamp_staircase.py` one-off used the ~100 Hz telemetry grid and a 10 ms-quantised
  age; the promoted probe's `replay_window` runs the twin at 500 Hz from the echo knots. Both agree on the
  ranking; only the latter is the oracle.
- `test_leg_clamp_age_aware_twin.py`'s mutation sweep found one equivalent mutant: removing the explicit
  `dr = 0` on a frame tick changes nothing (the same tick recomputes `dr = cmd − cmd_ref = 0`). The
  firmware keeps the explicit reset for readability; the tests pin the behaviour, not the line.

## Related

- Plan: `plans/active/leg-lead-clamp-age-aware.md`; design record
  `plans/archived/leg-bus-frame-drops.md` § 3; salvage `plans/archived/lead-clamp-content-freshness.md` §§ 2, 5, 6.
- Evidence: `logbook/2026-10-24-two-ball-oval-p3-ab-sittings-skid-bound.md` § "The vibration events".
