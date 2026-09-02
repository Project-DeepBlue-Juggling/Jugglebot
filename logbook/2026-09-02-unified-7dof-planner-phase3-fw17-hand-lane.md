---
title: "Unified 7-DoF planner Phase 3 — FW 17 grows the hand lane: a 7th ISR interp block on its own knot clock, owner-signed guards observe-first, and hand mastery behind a firmware latch; SOFTWARE-COMPLETE, NEVER FLASHED"
type: investigation
date: 2026-09-02
status: resolved
phase: "unified-7dof-planner — Phase 3 (software)"
related_plan: unified-7dof-planner.md
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - ros_ws/docs/choreography.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/.gitignore
  - ros_ws/src/jugglebot/Teensy_code_canbridge/README.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hand_ops.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hand_ops.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hand_source.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hand_source.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/platformio.ini
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - teensy_link/protocol.py
  - teensy_link/rpc_args.py
  - tests/firmware/native/build.py
  - tests/firmware/native/test_fault_machine.cpp
  - tests/firmware/native/test_hand_ops.cpp
  - tests/firmware/native/test_leg_interp.cpp
  - tests/firmware/native/test_rpc_dispatch.cpp
  - tests/firmware/test_bridge_fw_version_xref.py
  - tests/firmware/test_hermite_xref.py
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/hardware/hand_stream_bench.py
  - tests/hardware/session_unified7_hand_bringup.md
  - tests/ros/test_teensy_bridge_node_hand.py
  - tests/ros/test_teensy_bridge_node_read.py
  - tests/ros/test_teensy_bridge_node_setpoint.py
  - tests/teensy_link/conftest.py
  - tests/teensy_link/test_rpc_args.py
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - plans/active/unified-7dof-planner.md
  - plans/active/INDEX.md
  - logbook/2026-09-02-unified-7dof-planner-phase3-fw17-hand-lane.md
  - logbook/INDEX.md
subsystem:
  - can
  - ros
  - config
  - tools
tags:
  - safety
  - IPC
  - testing
  - docs
---

# Unified 7-DoF planner Phase 3 — can-bridge FW 17 hand lane

## Summary

Phase 3 of [`plans/active/unified-7dof-planner.md`](../plans/active/unified-7dof-planner.md):
can-bridge **FW 17** — the 7th (hand, axis 6) 500 Hz interp lane, the
owner-signed hand guards, and the `hand_source` mastery latch —
**SOFTWARE-COMPLETE, NEVER FLASHED.** The flash event is **lockstep**: FW 17 +
host ≥ `fb972c3` in ONE operator sitting (runbook
`tests/hardware/session_unified7_hand_bringup.md`); rollback = reflash FW 16 +
the pre-v6 host checkout — the v6 version darkness makes a half-rollback loud,
not silent. The sitting also discharges FW 16's pending first flash (its
poller-cadence + tri-state-TX content is carried forward in full).

**The lane:** a SEPARATE ISR block with its own per-lane knot clock —
structural guard isolation, the leg path byte-stable (parity transferred by an
identical-knots native test; `motor_guard.py` untouched). Mode 1 endpoint
velocity comes from the transmitted `v1` under `HAS_V1` (legs too), with the
`(u2 − u1)/SEG_T` forward difference when clear. Phase 2's carried finding 7 is
discharged: the NORMATIVE falling-edge rule is implemented **via the knot
clock** — segment completes → endpoint Taylor ≤ `MAX_EXTRAP` → Mode-3 decay,
never a held endpoint with `vel_ff` up to 200 rev/s — and because it keys on
knot age, plan expiry and stream stop are covered for free.

**The guards, exactly as owner-signed (Phase 0 decision 4):**
`MAX_DEVIATION_HAND_REV` 2.5, velocity-compensated both sides — residual
computed per 500 Hz tick, the 10 Hz fault task latching off a race-free
cumulative exceed-tick counter — shipped **observe-first** with a runtime
`hand7 arm|observe` console switch (boots observe every boot; arming is the
named second-sitting step, no reflash); `MAX_LEAD_HAND_REV` 2.0 against
`fb + vel·age` with the REQUIRED lead-duty counter (`[hand7] lead=` +
`lead_clamp_mask` bit 6 — non-zero during a throw = hard-abort the sitting);
`HAND_VELFF_LIMIT_RPS` 300; a `fault_machine` hand overspeed guard at 345;
hand clip [0, 10.8]. The leg constants provably never touch axis 6 — separate
loops, natively pinned. Hand `torque_ff` is hard 0 on the wire: no owner-signed
hand torque clamp exists, so none is invented.

**The interlock**, in its own TU (`hand_source.cpp`): `LEGACY_STROKE` (boot) |
`STREAMED`, switched only by the additive `HAND_SOURCE_SET` RPC 0x0055 (**no
version bump**; the xlang layout hash re-pinned via the intentional-change
mechanism), gated on `!mpc_active` + a settle band + fresh telemetry + a
0.5 rev/s velocity term; `ERR_HAND_SOURCE` 0x0007 refuses **before any CAN
side-effect**; LEGACY discards Setpoint index 6 on a counted, `[hand7]`-visible
counter; the 0x6D0 sniff stays live as a second-master detector; the latch
rides HeartbeatT2J bit 6 → `/link_status`. `HAND_CMD_ECHO` re-sources from
`axes[6].target_*` under STREAMED — implemented at `telemetry.cpp`'s uplink
step, not the plan-drafted `can_buses.cpp`: the substance lives where the cache
is consumed, zero ISR cost, and echo == wire bytes.

**Host:** a `std_srvs/SetBool` `set_hand_source` service (no interfaces
rebuild — the Phase 2 `SetTrajectoryLimits` precedent); the arming fold (hand
CLOSED_LOOP + POSITION/PASSTHROUGH preamble plus a u0-vs-encoder **0.625 rev =
`MAX_DEVIATION_HAND`/4** pre-check when STREAMED; LEGACY + hand-knots is a loud
warning); `EXPECTED_BRIDGE_FW_VERSION` → 17. `HAND_SOURCE_SET` is deliberately
NOT in `NON_IDEMPOTENT_METHODS` — the firmware setter is idempotent by design.
The `UNIFIED7_BENCH_BUILD` probe variant is retired outright (env + blocks
deleted; its stale-cache accounting ideas carried into the real lane; a
rebuild-from-git-history note left for the headroom runbook's un-flown arms
C/D). New actuating surface: `tests/hardware/hand_stream_bench.py` — the first
streamed-hand commander (production `SetpointPump` in the loop, stream-then-arm,
disarm-on-fault) — plus the runbook (lockstep flash + rollback, T-H1..T-H4,
gap-re-entry BEFORE arming, observe-then-arm, the lead-duty abort rule).

## Discussion — the review

Per the owner's 2026-09-02 directive this phase ran the lean 2-lens review
shape: **14 distinct findings → 10 adjudicated fixes + 1 deliberate skip**,
each fix natively/ROS-pinned. The headliners:

- **Stale hand-lane latch across armed sessions.** `s_hand_active` was cleared
  only by `interp_reset`, which nothing calls at runtime — a later hand-less
  armed session would replay an ancient decayed hold onto the powered hand via
  the recovery slew, or spuriously latch `MAX_DEVIATION`. Fixed: an arm-edge
  clear in the ISR before staging consume — a fresh `HAS_HAND` latch per armed
  session, and a live stream re-latches the same tick.
- **Heartbeat-freshness arming race.** Switch-then-arm within ~100 ms read a
  pre-switch heartbeat and silently skipped the hand preamble + the 0.625
  pre-check — the exact sequence Phase 4 automates. Fixed: bit-6-CLEAR is
  trusted only ≥ 2 heartbeat generations past the `HAND_SOURCE_SET` ack, with
  the acked STREAMED state OR-ed in; bit-6-SET is trusted as-is (the
  over-inclusive direction is the safe one). The previously-untested arming
  choreography now has 4 node tests.
- **The bench driver would have wedged the sitting three ways.** (a) An
  `MPC_STALE` latch at the interactive arm — the stream stopped during
  `input()`; fixed with a background 40 Hz hold stream through the prompt +
  arm-verify, and a `--clear-errors` verb. (b) A velocity-blind deviation belt
  spuriously aborting the 3 m/s stroke — ~0.95–4.3 rev of apparent deviation
  from encoder age alone, the same arithmetic Decision 4 used to reject a
  static firmware bound; fixed as `|cmd − (enc + vel·age)|` with per-stage
  defaults. (c) T-H4(b) unexecutable because the driver force-switched the
  source; fixed with `--no-source-switch` and runbook rows 18–19 rewritten
  launch-DOWN with `ros2 service call` as the launch-up verb — single-owner
  UDP preserved. Alongside: T-H1's drop-episode criterion got its instrument
  (the driver logs CacheDiag 1 Hz windowed `enc_frames` deficits), and
  T-H2b's echo-vs-analytic comparison now time-aligns at the echo's
  `t_bridge_us`, TIME_SYNCED-gated — a NOW-comparison smears 30–100 mm at
  stroke speed against the 3.25 mm bar.
- **Trip-dedicated deviation snapshot** — an armed trip freezes the trip's own
  residual trio rather than the boot-cumulative max, so observe-block
  excursions no longer misattribute a later trip.
- **Per-axis recovery slew** — legs resume vel/torque FF one tick after the
  enable edge instead of waiting up to ~2 s for the hand's convergence.
- **Deliberately skipped** (`fault_machine.cpp:409-419`): the
  disarm-races-the-10 Hz-poll deviation-attribution window — bounded ≤ 100 ms,
  output already suppressed in every such state, the loss is latch attribution
  only; reviewer and orchestrator agreed it is log-only value.

## Build evidence

Clean `rm -rf .pio/build` builds, 2026-09-03. **Never flashed.**

| Image | md5 | Size | Note |
|---|---|---|---|
| `teensy41` (FW 17 stock) | `2e90c43d5b6889e1059982d2e7aaf961` | 769,144 B `firmware.hex` | reproducible across two clean builds 2026-09-03; the md5 inline in runbook precondition 4 |
| `teensy41_bench_sysid` | `3a011af1e712ac5b557f722117131153` | 774,904 B `firmware.hex` | reproducible across two clean builds 2026-09-03 |

(This build supersedes the 2026-09-02 post-review-fix stock build
`7abe226b…` / 769,144 B after the end-of-phase audit's HAND_CMD_ECHO
echo-stamp age-correction — the streamed echo's `t_bridge_us` now dates the
interp tick that wrote the echoed bytes, not the telemetry emit. The
pre-review-fix stock build `6e44b437…` / 766,264 B and the 2026-09-02
bench_sysid `8e6342c0…` are likewise superseded.)

## Verification

- Implementation (2026-09-02): (`python -m pytest tests/firmware/ -q`,
  **407 passed in 245.07 s** — natives rebuilt inside);
  (`python -m pytest tests/teensy_link/ -q`, **297 passed in 7.45 s**);
  (`python -m pytest tests/ros/ -q -k "teensy_bridge_node"`, **417 passed /
  2216 deselected in 72.70 s**); native binaries direct: `test_leg_interp`
  296/296, `test_hand_ops` 168/168, `test_fault_machine` 245/245,
  `test_rpc_dispatch` 117/117 assertions;
  (`python -m pytest tests/motion/test_leg_torque_ff.py
  tests/motion/test_kt_lib.py tests/ros/test_gui_geometry.py -q`, **315 passed
  in 2.95 s** — the `canbridge_config.h` regex-parsers);
  (`python config/generate_udp_protocol.py`, **5 artifacts, Setpoint 208 B
  unchanged**).
- Post-review-fix (2026-09-02):
  (`python -m pytest tests/firmware/test_native_firmware.py -q`, **18 passed in
  192.39 s**); (`python -m pytest tests/firmware/ -q`, **407 passed in
  20.81 s**); (`python -m pytest tests/teensy_link/ -q`, **297 passed in
  7.45 s**); (`python -m pytest tests/ros/ -q -k "teensy_bridge_node"`,
  **421 passed in 73.17 s**).
- Gate, first run: (2026-09-02, `./run_tests.sh`, **2 failed / 6390 passed /
  4 skipped in 256.60 s — FAIL**, solely `tests/ros/test_choreography_map.py`'s
  drift pin: the committed generated `ros_ws/docs/choreography.md` lagged the
  new `set_hand_source` service — the pin doing its job). Regen: (2026-09-02,
  `python tools/gen_choreography_map.py`, +7 lines;
  `python -m pytest tests/ros/test_choreography_map.py -q`, **28 passed in
  7.13 s**).
- Post-docs gate: (2026-09-02, `./run_tests.sh`, **6392 passed / 4 skipped in
  261.13 s parallel + empty serial phase — PASS**).
- Pre-commit gate, after the end-of-phase audit fixes (2 WARNINGS + 4 NOTES,
  all applied — echo-stamp age-correction included, hence the superseding
  build in the table above): (2026-09-03, `./run_tests.sh`, **6392 passed /
  4 skipped in 256.18 s parallel + empty serial phase — PASS**). `--full`
  deliberately not run (owner directive scopes it to `controller/`/`sim/`
  diffs — neither touched; runbook row 1 mandates `--full` before the
  hardware sitting regardless).

## Outcome

**Phase 3 is SOFTWARE-COMPLETE — built, tested, NEVER FLASHED.** T-U9 landed
(`tests/firmware/native/test_leg_interp.cpp` + `test_hand_ops.cpp` +
`test_fault_machine.cpp` + `test_rpc_dispatch.cpp`, plus the
`test_hermite_xref.py` extension); T-R2 is now FULLY landed (Phase 2's skew
loud-reject + `BRIDGE_FW_CHECK` naming 17). What remains of Phase 3 is
operator-owned: the lockstep flash sitting and the T-H1..T-H4 bench ladder
(runbook + driver ready). The sitting ends this software arc; Phase 4
(unified-cycle mode) is next on the software side.

## Open Questions

1. **Settle-gate specifics need OWNER RATIFICATION**: band 0.10 rev (mirrored
   from `hand_stroke.py`), rests {retract 0.0 with the homing-ref −0.1 lower
   edge, catch-prime 9.9594}, new `HAND_SOURCE_SETTLE_VEL_RPS` 0.5 — the plan
   named only the constant. All conservative-direction choices, the interlock
   boots LEGACY, and the operator can amend pre-flash.
2. `hand_stream_bench.py` wants operator review before the sitting — new
   actuating code, and the runbook rows depend on it.
3. `rej_source` is console/native-only (the BridgeTxDiag payload is frozen) —
   host-derived OK arithmetic overcounts during refusals, documented both
   ends; a future additive diag frame could carry it plus the cumulative
   lead-duty.
4. The pump's >5 rev across-gap re-baseline acceptance is pinned at unit level
   (T-U7); the bench gap stage bounds re-entry to ±1.5 rev.
