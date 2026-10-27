---
title: "Leg lead clamp P2: shift slew bound (FW 32, carried in FW 33), 2026-08-15 replay windows, and the bench A/B of FW 30 vs FW 33 under injected encoder-frame gaps (ODrive SDO encoder_msg_rate_ms)"
type: investigation
date: 2026-10-25
status: resolved
phase: "leg-lead-clamp-age-aware — P2 (bench validation)"
related_plan:
  - leg-lead-clamp-age-aware.md
  - two-ball-oval.md
related_entries:
  - 2026-10-24-leg-lead-clamp-age-aware-p1-firmware.md
  - 2026-08-15-fw14-validated-arc-closed.md
sessions: []
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - teensy_link/rpc_args.py
  - tools/probes/teensy_link_profiling/hermite_xref/teensy_interp.py
  - tools/probes/lead_clamp_staircase_probe.py
  - tools/probes/data/lead_clamp_staircase_2026-08-15.json
  - tools/probes/lead_clamp_gap_bench.py
  - tools/probes/README.md
  - config/ODrive config Files/odrive-pro-4.4.58-0.6.11-1_flat_endpoints.json
  - tests/firmware/native/test_leg_interp.cpp
  - tests/firmware/test_leg_clamp_age_aware_twin.py
  - plans/active/leg-lead-clamp-age-aware.md
subsystem:
  - can
  - motion
  - tools
tags:
  - safety
  - testing
  - hardware
---

# Leg lead clamp P2: bench validation

## Summary

P2 closes with the age-aware clamp validated three ways:

- **Offline replay.** Both 2026-10-24 Sittings and the archived 2026-08-15 trace replay through the 500 Hz twin.
  - FW 30 reproduces every recorded staircase. On 2026-08-15 ax1 it gives 10.64 mm against 10.64 mm recorded.
  - FW 31/32 cut the max step by 31–77 %.
  - FW 31 engages on zero sub-band gap ticks.
  - Clean ticks are bit-identical to FW 30 in all five windows.
- **Firmware refinement.** P1's open question became **FW 32**: the dead-reckoned shift may grow by at most `vel_limit × tick` (0.024 rev) per tick, so a command discontinuity inside a gap ramps behind the clamp. The Wave camera merge renumbered it to **FW 33** (an additive `WAVE_CMD` RPC; `leg_interp.cpp` is byte-identical to FW 32).
- **Bench A/B (2026-10-27).** The 7th bench leg ran on CAN2 as node 0. The ODrive's own `axis0.config.can.encoder_msg_rate_ms` (endpoint 275) was set over SDO to inject gaps: 100 ms × 1 s, 0 × 0.3 s and 0 × 1 s, mid-stream on a 2 rev/s triangle. Two runs per build.
  - In the 100 ms-rate gap, FW 30 staircases (8 transmitted steps of ~10.7 mm per window, clamp engaged) and FW 33 is smooth (clamp engaged on 0 of 350 stale-anchor ticks).
  - In the dead gaps, `MOTOR_FB_STALE` trips and clears within 0.03–0.06 s on both builds.
- **Reframed mid-run.** The 7–11 mm dead-gap "steps" were LEG_CMD samples the bridge never transmitted (see Discussion).

The board is left on FW 33. P3 reduces to a receipt check plus the oval Sitting.

## Motivation

P1 (`2026-10-24-leg-lead-clamp-age-aware-p1-firmware.md`) left four obligations:
1. Replay the two 2026-10-24 episodes and any surviving 2026-08-15 ones.
2. Show bit-identity on clean windows.
3. Weigh the shift slew bound.
4. Run injected gaps on hardware, where the commanded step must be bounded by the stream's velocity, not by `MAX_LEAD_REV`, and `MOTOR_FB_STALE` must still terminate a dead axis.

The owner's answers before work began:
- Rig: the 7th leg replaces Jugglebot on CAN2, node 0, alone on the bus.
- Gap injection: ODrive SDO `encoder_msg_rate_ms`.
- Slew bound: adopt in P2 as FW 32.
- Bind-plus-gap: twin and native only.
- Envelope: ±0.4 rev at 2 rev/s.
- End state: the board finishes on the new build.

## Design

### The shift slew bound (FW 32)

In `leg_clamp_anchor()`, `leg_interp.cpp` (~232), the coast branch no longer sets `dr = raw_cmd − cmd_ref`. It moves `dr` toward that target by at most `LEG_ANCHOR_SHIFT_SLEW_REV` per tick:

```
LEG_ANCHOR_SHIFT_SLEW_REV = ODriveDefaults::LEG_VEL_LIMIT_RPS (12.0) × INTERP_PERIOD_US (2 ms) = 0.024 rev
```

It has a static_assert of `0 < slew < MAX_LEAD_REV`. A stream at or below `vel_limit` advances at most 0.024 rev per tick, so it never meets the bound and FW 32 equals FW 31 bit-for-bit. Only a command DISCONTINUITY landing in a gap differs. FW 31 dead-reckoned such a jump straight into the anchor, so it passed unclamped until the next frame. FW 32 ramps it at `vel_limit` behind the clamp. There is no wire change.

### One 500 Hz tick, three ages, FW 32 (leg i, stream v ≤ 12 rev/s)

| | age 0 (frame this tick) | age 80 ms, stream ≤ vel_limit | age 80 ms, a 0.3 rev command jump | age ≥ 150 ms |
|---|---|---|---|---|
| anchor | fresh frame: `cmd_ref = cmd`, `dr = 0` — identical to FW 30/31 | `dr` tracks `cmd − cmd_ref` exactly (per-tick growth ≤ 0.024) — FW 31 | `dr` grows 0.024/tick; the clamp holds the emit at anchor + 0.10, so the jump reaches the leg at vel_limit | `dr` frozen (FW 31); re-pins as cmd advances; fault task suppresses at 150–250 ms |
| `vel_ff` path | unchanged (cap 3.5 rev/s) | unchanged | unchanged (the clamp mask zeroes it, as in FW 30/31) | unchanged |
| `MAX_DEVIATION` | reads u0 − fb, never the anchor | unchanged | unchanged | unchanged. On the bench the in-stream peak |`live_deviation`| was 0.57 rev on FW 30 and 0.44 rev on FW 33, inside 1.0 |

### The bench driver

`tools/probes/lead_clamp_gap_bench.py` subclasses the `bench_leg_sysid` bridge bench, using its guard, latch backoff and caps.
- It streams a 12 s triangle at ≤ 2 rev/s around the stroke centre.
- It writes `encoder_msg_rate_ms` over SDO on a schedule thread. The uint32 goes as a float whose bit pattern is N, because `encode_sdo_write` forwards the bytes.
- It records LEG_CMD, TELEMETRY, HEARTBEAT_T2J, LegClampDiag and BridgeIdentity as JSONL.
- In a `finally` it restores 10 ms three times, then disarms.
- `GuardLatchBackoff.recoverable_grace` is widened to 2 s for the run, so the 1 s dead window does not abort itself.

Endpoint 275 was verified against ODrive's release index for the Pro v4.4-58V at 0.6.11-1 (also 0.6.11), saved as `config/ODrive config Files/odrive-pro-4.4.58-0.6.11-1_flat_endpoints.json`. In the same table, `get_gpio_states` = 726 matches `protocol_config.h`. On the S1 table, id 275 is a different endpoint.

## Implementation

- `leg_interp.cpp`, `canbridge_config.h` (FW 32, constant and history line), `teensy_link/rpc_args.py` (expected FW). These are now FW 33 after the Wave camera merge, `459725eb`.
- Twin: `teensy_interp.py` mode `firmware` = FW 32, `firmware_fw31` keeps P1's form, and `LEG_ANCHOR_SHIFT_SLEW_REV` is pinned against the header.
- Native: the P1 "pinned exposure" case is replaced by "a command DISCONTINUITY while the frame is aged is RAMPED at vel_limit behind the clamp".
  - The first tick emits `1.0 + SLEW + MAX_LEAD`.
  - The next 10 ticks each step by SLEW.
  - The frame returns at `1.0 + MAX_LEAD`.
- Replay: `lead_clamp_staircase_probe.py --emit-fixture-0815` writes `tools/probes/data/lead_clamp_staircase_2026-08-15.json`. The windows are ax1 and ax4 from `rosbags/2026-08-15_10-08-06`; ax5 was motionless and ax2's cache age was 18 ms, so neither is a gap.
- Twin tests (23 in `test_leg_clamp_age_aware_twin.py`):
  - the slew constant;
  - age-0 identity across fw30/fw31/FW 32;
  - a Hypothesis property that the bound is invisible to any stream ≤ 11.76 rev/s across 0.03–0.30 s gaps;
  - the in-gap step ramp;
  - the replay oracle over S1A/S1B/S2/ax1;
  - FW 32 departing from FW 31 only on super-vel_limit ticks;
  - ax4 identical under every clamp;
  - clean-tick bit-identity to FW 30 in all five windows.
- Bench driver plus its 20-check `--self-test`. The analyser was revised on hardware; see Discussion.

## Verification

- Native (`python tests/firmware/native/build.py` then `./temp/firmware_native/test_leg_interp`, run 2026-10-25): **52/52 test cases, 8072/8072 assertions** (`temp/logs/lead-clamp-p2-native.log`).
- Scoped battery (`python tools/probes/teensy_link_profiling/hermite_xref/xref.py`, then `pytest tests/firmware/ tests/teensy_link/ tests/ros/test_teensy_bridge_node_leg_clamp_diag.py tests/ros/test_teensy_bridge_node_bridge_diag.py -q --hypothesis-seed=0 -p no:cacheprovider`, run 2026-10-25): **759 passed in 51.17 s**; xref PASS at 0.0 rev (`temp/logs/lead-clamp-p2-scoped-tests.log`).
- `./run_tests.sh --full` (run 2026-10-25): parallel **7181 passed / 4 skipped**, serial **8 passed**, PASS (`temp/logs/lead-clamp-p2-full-gate-1.log`). The closing gate is in the commit message.
- Twin at depth (`pytest tests/firmware/test_leg_clamp_age_aware_twin.py --hypothesis-profile=ci-deep --hypothesis-seed=0`, run 2026-10-27): **23 passed in 52.83 s** (`temp/logs/lead-clamp-p2-twin-cideep.log`).
- Builds: `pio run -e teensy41` for FW 32 and for FW 30 (a detached worktree at `2bafbae7`), both SUCCESS on 2026-10-25. Flashed 2026-10-27 (`pio run -e teensy41 -t upload`, launch down; upload SUCCESS in `temp/logs/lead-clamp-p2-flash-fw{30,33}.log`). BridgeIdentity receipts: **fw 30 / proto 10**, then **fw 33 / proto 10**, recorded in each run's `identity.jsonl` and `meta.json` (`temp/probes/lead_clamp_gap_bench/20261027_212652_fw30/`, `.../20261027_212838_fw33/`).
- Driver (`python tools/probes/lead_clamp_gap_bench.py --self-test`, run 2026-10-27, after the analyser revision): **20/20 checks PASS**.

### Fails-before / passes-after

| Test | Mutation | Result |
|---|---|---|
| native FW 32 discontinuity case | slew reverted to FW 31's coast line | 51/52 cases, 22 assertions fail (`lead-clamp-p2-native-failsbefore.log`) |
| twin step ramp | `slewed = False` | that test fails, 22 pass (`lead-clamp-p2-twin-mut1.log`) |
| twin slew constant + invisibility property | slew 0.024 → 0.005 | both fail (`lead-clamp-p2-twin-mut2.log`). The property first passed under this mutant, because Hypothesis only drew large speeds with gap 0. It now draws speed and gap independently. |

### Replay oracle (500 Hz twin, `fixture_report`, run 2026-10-25)

| window | FW 30 max step | FW 31 | FW 32 | recorded | FW 31 sub-band engaged |
|---|---|---|---|---|---|
| S1A leg 1 (10-24) | 11.55 mm | 2.61 | 2.61 | 11.6 | 0 / 261 |
| S1B leg 2 (10-24) | 12.39 | 8.57 | 8.57 | 12.4 | 0 / 226 |
| S2 leg 1 (10-24) | 14.89 | 3.81 | 3.56 | 12.4 | 0 / 129 |
| ax1 leg 1 (08-15, FW 14) | 10.64 | 3.35 | 3.35 | 10.64 | 0 / 226 |
| ax4 leg 4 (08-15) — no-harm | 0.32 | 0.32 | 0.32 | — | 0 / 190 |

FW 32 engages on one sub-band tick each in S1B and S2. Its emitted command departs from FW 31 on 2 ticks in S1B and 1 in S2 (a separate count, from `test_replay_oracle_fw32_departs_only_on_super_vel_limit_ticks`); see Discussion, tradeoff 2.

### Bench runs (2026-10-27, bench leg on CAN2 node 0, homed to −0.1005 rev)

Run directories are under `temp/probes/lead_clamp_gap_bench/`. Console logs are `temp/logs/lead-clamp-p2-bench-fw{30,33}-run{1,2}.log`. Numbers come from `--analyse` after the revision. "nbig" is transmitted steps above max(2.5 × expected, 0.4 × MAX_LEAD); the expected step at 2 rev/s per 10 ms LEG_CMD sample is 1.41 mm.

| run (uptime at link-up) | 100 ms × 1 s: max / nbig / clamp ticks | 0 × 0.3 s: max (untx max) / clear | 0 × 1 s: max (untx max) / clear |
|---|---|---|---|
| FW 30 `212652` (412 s) | **10.74 mm / 8 / hb 7 of 12** — STAIRCASE | 1.41 (9.78) / 0.04 s | 1.56 (7.05) / 0.04 s |
| FW 30 `212722` (442 s) | **10.71 / 8 / hb 8 of 12** — STAIRCASE | 1.41 (11.03) / 0.06 s | 1.41 (10.25) / 0.06 s |
| FW 33 `212838` (39 s) | 4.51 / 2 / diag **0 of 350** stale ticks — SMOOTH | 1.42 (11.44) / 0.03 s | 1.41 (3.52) / 0.03 s |
| FW 33 `213355` (355 s) | 2.21 / 0 / diag **0 of 350** — SMOOTH | 1.41 (11.87) / 0.04 s | 1.42 (2.99) / 0.04 s |

Every window's rate change was confirmed from the telemetry frame interval: 100 ms median at rate 100, and a 250 / 950 ms no-change span at rate 0. In each dead gap `MOTOR_FB_STALE` tripped 150–250 ms in and cleared after the restore. Nothing latched. In-stream peak |`live_deviation0`| was 0.57 / 0.45 rev (FW 30) and 0.43 / 0.44 rev (FW 33), against the 1.0 rev `MAX_DEVIATION`. One 1.45 rev reading on the fresh FW 33 boot was taken before the stream: `u0` was still 0 against the held leg, the bridge was disarmed, and it cleared at the harness re-baseline.

Per-tick traces from `/tmp/probe_restore_trace.py`, a throwaway probe:
- **0.3 s dead gap.** FW 30 halts the transmitted command 0.1 rev past the stale encoder within 50 ms of frame loss. FW 33 keeps following the stream at −2.00 rev/s for 150 ms, 0.40 rev blind, then holds at its frozen anchor ± lead.
- **1 s gap on FW 33 run 2.** The blind 150 ms crossed a triangle apex. The leg came back 1.7775 rev from home, consistent with the last transmitted command + vel_ff/pos_gain, and the recovery slew rejoined the stream within 60 ms.

## Discussion

### Reframed mid-run: the dead-gap "staircase" was untransmitted

The first FW 33 run read STAIRCASE in the 0.3 s dead gap, with an 11.44 mm step, which is no better than FW 30. The traces showed the step fell exactly at frame return, as one 10 ms dip of about −0.11 rev followed by a snap to the encoder, on both builds.

The firmware explains it:
- `send_leg_cmd()` reports `axes[i].target_pos_rev`.
- `interp_isr` writes that on every tick (`leg_interp.cpp:1571`).
- The CAN TX is gated on `s_output_enabled` (`:1594`), which the 10 Hz fault task holds shut through `MOTOR_FB_STALE`.
- When the first fresh frame lands, the ISR clamps the stale stream to `enc − MAX_LEAD`. That value is computed and reported but never sent.
- The re-enable edge then starts the recovery slew from the encoder.

The encoder confirms it: it does not move through the dip. The hypothesis "FW 33 fails dead gaps" is therefore withdrawn. The dead gaps were never a lead-clamp question past 150 ms, because suppression owns them and the builds differ only in the first 150 ms. FW 33 dead-reckons along the stream there, where FW 30 halts.

### The analyser was revised on hardware data, and how

Two changes were written down before FW 33 run 2 was taken and before any run was re-analysed. They were made after FW 33 run 1 had been read, so they are not blind.
1. **Untransmitted samples are excluded.** Steps are computed only between transmitted LEG_CMD samples. In a window where `MOTOR_FB_STALE` was seen, the span from `t_on + 150 ms` (the earliest possible trip) to the recovery slew's re-baseline sample is dropped. That sample is the first one after restore with |cmd − enc| < 3 mm and |vel_ff| ≤ 0.15 rev/s. The dropped span's largest step is still reported, in the "untx max" column.
2. **STAIRCASE needs ≥ 3 big steps.** One big step is not enough. Outside the gaps the stream alone produces about one 4–5 mm step per 1,200 samples on both builds (max 4.77 mm on FW 30, 3.81 mm on FW 33). The bench sends `flags = 0x3` without HAS_SCHED, so each knot latches on arrival and jitter becomes a catch-up step. FW 33 run 1's two 4.5 mm steps in the 100 ms gap carried no clamp engagement and did not recur in run 2.

A real staircase repeats once per frame: FW 30 shows 8 in each 100 ms window. The P2 brief had already asked for the STAIRCASE threshold to be re-checked against real FW 30 data. These two changes are that re-check.

### Tradeoffs accepted

1. **Bind-plus-gap** is measured offline only (owner: twin and native). A leg bound at a stroke end with frames withheld lets the command run up to the frozen anchor. The shift is bounded by stream speed × 150 ms, which P1's walk puts at 0.45 rev at 3 rev/s. The 15 A clamp bounds the shove. P3's Sitting is the next look.
2. **FW 32 departs from FW 31 in the replays** (emitted command differs) on 2 ticks in S1B and 1 in S2, by up to 1.06 mm. The replay's raw command moves faster than `vel_limit` at a few knot latches, because the twin latches echo knots on arrival. That is not a real-stream event. Rather than weaken P1's zero-engagement claim, the claim is asserted on `firmware_fw31`. A separate test pins exactly where FW 32 may depart (owed ticks after a fast tick, until the next frame) and that its max step is never worse.
3. **The suppression hold is not a pure hold.** During `MOTOR_FB_STALE` the ODrive keeps its last vel_ff and settles at `input_pos + vel_ff / pos_gain`: 0.05 rev at 2 rev/s on the bench, and at most 0.0875 rev at the 3.5 rev/s cap. It is bounded, there is no integrator wind, the recovery slew re-baselines from the encoder, and it is the same on FW 30. It is filed rather than fixed: JB-73.

### What was ruled out

- **CAN-side frame filtering in the bridge.** Rejected because it would test the bridge's own filter, not the ODrive's drop. The ODrive SDO rate is the production failure's own mechanism.
- **The robot itself.** The bench rig gives a same-stream, same-gap A/B with no oval choreography in the way.
- **Pinning the analyser threshold before any hardware.** The first real FW 30 run is what showed the background step rate.

## Fix

FW 32's slew bound, carried unchanged in FW 33, is on the board. No further firmware change came out of the bench.

## Open Questions

- **JB-72.** `HomingMonitor` false-times-out a home shorter than one Diagnostic period, because it never sees RUNNING. It hit the bench homing today (the leg homed fine; the observer waited 35 s). The production `/home` uses the same monitor.
- **JB-73.** The vel_ff is held through suppression, with LEG_CMD reporting untransmitted targets alongside it.
- P3 owes the honest on-robot engagement number from an FW 33 bag (P1 tradeoff 3) and the bind-plus-gap look.
- One early observation is unexplained. After the owner's first homing attempt (piped through `tee`, so its output was buffered), the leg read 1.15 rev with `homing_result = OK`. A second home settled at −0.1005 rev and stayed there. The first attempt's log is empty, so it cannot be reconstructed.

## Related

- Plan: `plans/active/leg-lead-clamp-age-aware.md` § P2.
- P1: `logbook/2026-10-24-leg-lead-clamp-age-aware-p1-firmware.md` (form Z, the three-age walk, the replay oracle).
- Evidence for the original episodes: `logbook/2026-10-24-two-ball-oval-p3-ab-sittings-skid-bound.md`; archived drops plan `plans/archived/leg-bus-frame-drops.md`.
