---
title: "BUG-99 FW 50, observability and pins only: an overflow-safe stamp bound (2^31 us, both ways) closes a signed-overflow band at 2^63 + wall, a compiled lead-clamp headroom assert with a YAML twin, a console-only scheduled/legacy transition count, native pins of three FW 49 behaviours, FEAT-28's gpio_poll comment; can-bridge FW 50, NOT flashed"
type: bugfix
date: 2026-11-25
status: complete
phase: "BUG-99"
subsystem: can-bridge
related_issues:
  - BUG-99
  - FEAT-28
  - FEAT-64
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/gpio_poll.cpp
  - teensy_link/rpc_args.py
  - teensy_link/protocol.py
  - teensy_link/__init__.py
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/axis_state.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - tests/teensy_link/test_encoder_age_codec.py
  - tests/firmware/test_udp_protocol_xlang.py
  - tools/probes/teensy_link_profiling/hermite_xref/teensy_interp.py
  - tests/firmware/native/test_leg_interp.cpp
  - tests/firmware/test_sched_c2_twin.py
  - tests/firmware/test_config_drift.py
  - tests/firmware/test_bridge_fw_version_xref.py
---

# BUG-99 FW 50: observability and pins, nothing a live stamp can see

**What.** The in-scope half of the 2026-11-24 BUG-99 review of FW 49, built as can-bridge FW 50 (FW_VERSION 49 → 50, `EXPECTED_BRIDGE_FW_VERSION` 50, no wire change, PROTOCOL_VERSION stays 14). Not flashed.

- **A1, overflow-safe stamp bound.** `interp_on_setpoint` now computes `d = (int64)(t_origin_us − wall)` in unsigned arithmetic and drops the frame when `|d| > SCHED_STAMP_RANGE_US = 2^31 us` (35.8 min), counted in `sched_future_drops` ("stamp out of range"), before the existing 250 ms future check. `t_start = mono + d` can no longer overflow. Mirrored in the Python twin (`teensy_interp.sched_on_setpoint`).
- **A2.** `static_assert` that every Leg has `pos_gain × MAX_LEAD_REV + LEAD_CLAMP_VELFF_LIMIT_RPS <= LEG_VEL_LIMIT_RPS` (40 × 0.10 + 3.5 = 7.5 <= 12), the prose invariant at `canbridge_config.h` § MAX_LEAD_REV made compiled. Python twin over `hardware_config.yaml` in `test_config_drift.py`.
- **Row 1 observability.** `s_sched_mode_xfers`: scheduled ↔ legacy lane transitions while output is enabled, console only (`[hand7] ... xfer=`), zeroed by `hand7 reset`, not uplinked.
- **Three PINS of FW 49 behaviour** (native, labelled `PIN (BUG-99 row N)`; they document, they do not fix): row 3 legacy Leg Mode 1 starvation holds p1 with velocity v1; row 2 a 1 ms offset slew is promoted as a counted step; row 1 a mid-stream demotion steps the Hand command by v × frame lead.
- **FEAT-28 rider.** The `gpio_poll.cpp` PROVISIONAL comment on `REPLY_STALE_US` now cites the measured cadence (comment only).

## Discussion

- **The band, reframed by the probe.** The review described the A1 hazard as a stamp "with bit 63 set and within `wall` of 2^63". The throwaway probe (`$S/out/fw50/probe_a1_band.cpp`, the FW 49 expressions verbatim, g++ -O2 and -fwrapv agree; live-like wall 1.7959e15 us, 1 h uptime, recv 1 us before mono, promotion 2 ms after ingest) shows the hazardous set is much narrower and centred elsewhere: stamps `2^63 + wall + [−(mono − recv), ingest-to-promotion latency]`, here offsets [−1, +2000] us, 2002 stamps. Below it, `mono + diff` overflows positive and the future check drops it; above it, `now − t_start` no longer overflows and the frame is expired. Inside it the stamp maps to t_start ≈ INT64_MIN, passes the future check, and is promoted because `now − t_start` overflows negative. The native case confirms it in the real TU: on FW 49 code a stamp at `2^63 + wall` is queued, feeds the staleness clock and activates both the Leg and the Hand group. The band tracks the wall clock, so only a deliberately crafted or bit-flipped stamp can hit it; the fix does not depend on that.
- **Why a symmetric 2^31 bound and not just an overflow-safe future check.** An overflow-safe future check alone would leave far-past stamps queued (and then expired); the symmetric bound makes every later signed sum provably in range (|d| <= 2^31, mono < 2^63) with one comparison, and 2^31 is what the review proposed. The accepted tradeoff: a HAS_SCHED stamp more than 35.8 min in the PAST is now a counted ingest drop instead of a queued frame that expires at promotion. The motion is the same (neither is ever played), but the ingest drop does not feed the staleness clock, so a stream of such stamps now ends in SETPOINT_STALE after 250 ms instead of a stop-and-hold with no fault. That is the stricter outcome, and it is reachable only off the live path (next bullet).
- **No stamp the live host produces is within 35 min of the bound.** Walked per producer:
  - *Who stamps.* HAS_SCHED is set only by `SetpointPump._build` (`teensy_link/setpoint_pump.py:906-910`) when `knot_epoch_us` is present; that is `trajectory_node._perf_to_epoch_us` = `(t_perf − offset) × 1e6`, CLOCK_REALTIME through a perf↔ROS offset refreshed every 30 s (`clock_offset.REFRESH_PERIOD_S`, steps measured median 0.0, max 3.3 us). The bridge wall is the same Jetson CLOCK_REALTIME (`set_wall_anchor` from the ToD reply). In flight a live stamp sits at the emit lead (about +24 ms) plus lateness, and anything beyond +250 ms was already a future drop on FW 49, so the live window is about [−50 ms, +250 ms]: 2^31 us is ~7000× outside it.
  - *Legacy stamps.* `teensy_bridge_node` stamps non-scheduled frames with `time.time()`; without HAS_SCHED they never enter the bounded branch (the bound sits inside `if (sched)`), so no legacy frame is affected whatever its stamp.
  - *Boot before time sync.* `time_synced()` is false until the first ToD anchor, so every HAS_SCHED frame is demoted to legacy before the bound is evaluated; the uninitialised wall (= micros64) is never compared. After 90 s without an anchor it goes false again, same path.
  - *Across a wall-clock anchor step.* The bridge steps its offset when an anchor disagrees by > 20 ms (`TIME_STEP_THRESHOLD_US`), i.e. at re-acquisition after a gap of at most 90 s of free-run (tens of ppm, under 10 ms); a step that size is 5 orders inside 2^31. The only way to put a live stamp 35.8 min off the bridge wall is a Jetson CLOCK_REALTIME step of > 35.8 min while streaming (stamps follow at once, the bridge at its next 30 s anchor). On this box `systemd-timesyncd` is the NTP client with an RTC, synchronised (2026-11-25, `timedatectl`). A forward step was already a future drop on FW 49 (same counter); a backward step > 35.8 min turns FW 49's expire-and-hold into FW 50's SETPOINT_STALE, as above. Any step > 50 ms already makes every frame miss its cover on both versions.
- **Control cycle walk (A1, one 500 Hz tick at 40 Hz frames).** A live frame arrives in the net task: `recv`, `mono`, `wall` read as before; `d` = stamp − wall ∈ [−50 ms, +250 ms] so the new branch is false and `t_start = mono + d` equals FW 49's `mono + ((int64)t_origin − wall)` exactly (no overflow on either version for these values; the probe found 0 disagreements over stamps in [wall − 2^31, wall + 4e9]). The queue insert, `rx_age_note`, the STOP record, `s_last_setpoint_us` and the seq high-water mark are untouched. The ISR's promotion, cover, grace and stop logic is not edited. The transition counter adds one increment in `latch_from_staging` and one in `sched_apply` after every state assignment; neither reads into a control value. So feedforward, timing and the command are bit-identical for every stamp the host can send.
- **Pins, measured.** Row 3: 0.6 rev held with vel_ff 3.0 rev/s after 200 ms of starvation, a static drive overshoot of v1/Kp = 0.075 rev (0.0875 at the 3.5 cap). Row 2: the first frame across a 1 ms slew is promoted at the Hand's acceleration peak, a velocity step dv 1.496 rev/s (= 1500 rev/s² × 1 ms), dp 0.0024 rev, promo_over 1, no stop, no refusal. Row 1: the demoted frame steps the Hand command by −1.712 rev at plan velocity −71.6 rev/s (v × 24 ms lead = −1.717), counted once in `xfer`. Those steps are the out-of-scope owner decisions (row 1 keep playing on the last offset, row 2 offset rate limit, row 3 Leg behaviour); FW 50 only makes them visible and pinned.

## Verification

All on 2026-11-24/25 in the `offline/fw50` worktree; nothing flashed.

- **Baseline, FW 49 sources (df9405a8).** `temp/firmware_native/test_leg_interp` built by `python tests/firmware/native/build.py` before any edit (2026-11-24 23:54): **70/70 cases, 10611/10611 assertions pass**.
- **A1 band probe.** `g++ -std=c++17 -O2 probe_a1_band.cpp && ./probe_O2` (and `-fwrapv`, same result), 2026-11-25: FW 49 band = offsets [−1, +2000] us from 2^63 + wall, 2002 stamps promoted; the fix drops all of them; 0 disagreements between FW 49 and the fix for stamps in [wall − 2^31, wall + 4e9].
- **FW 50 native leg_interp.** `g++ ... test_leg_interp.cpp` (the build.py recipe, by hand) then run, 2026-11-25: **75/75 cases, 10654/10654 assertions pass** (5 new: two A1, three pins).
- **New tests fail without the change.** The A1 cases built against a copy of `leg_interp.cpp` with only the FW 49 ingest restored: **2 cases, 11 assertions fail** (the band frame is queued, feeds the staleness clock, and activates both groups). The twin test with the twin's bound disabled: `pytest tests/firmware/test_sched_c2_twin.py -k stamp` **1 failed**. A2: `g++ -fsyntax-only` on a header copy with `LEG_POS_GAINS[0] = 90` fails the new `static_assert` (12.5 > 12); the YAML twin's own test mutates a temp copy the same way and sees leg 0 at 12.5.
- **Image.** `pio run -e teensy41` in `Teensy_code_canbridge/` (build only, never uploaded), 2026-11-25: SUCCESS, `firmware.elf` text 256320, data 35520, bss 113312 (dec 405152); `firmware.hex` 820984 B; no warning from an edited file.
- **Scoped suite.** `flock /tmp/jugglebot-tests.lock nice -n 10 python -m pytest -q -n 4 --dist loadfile tests/firmware/ tests/teensy_link/` plus the readers of the edited files and of `rpc_args` (`tests/ros/test_teensy_bridge_node_{install_skew,bridge_diag,version,read,setpoint,recover,recover_lane,hand_sensor}.py`, `tests/ros/test_gui_geometry.py`, `tests/motion/test_{hand_stream_bench_stall,leg_torque_ff,skills_segments}.py`, `tests/sim/test_{skills_gate,mapkit_sheet,logbook_front_matter}.py`), run 2026-11-25 00:16: **1790 passed, 4 skipped in 573.88 s**. The default gate was not run (the orchestrator's integration step).

## FEAT-64: per-axis encoder sample age (same FW 50, no second bump)

**What.** New additive uplink `ENCODER_AGE` (MsgType 0x96, 22 B payload, 32 B frame): `u64 t_teensy_us` (IDENTICAL to the paired TELEMETRY) + `u16 age_us[7]` (axes 0..5 Legs, 6 Hand). `send_telemetry` now emits it immediately BEFORE TELEMETRY at the 100 Hz tick (the BB_YAW_ESTIMATE pairing: in-order on one socket, host pairs by exact `t_teensy_us`). PROTOCOL_VERSION stays 14: an old host ignores the unknown type, a new host against an FW <= 49 board never sees one and must tolerate its absence.

- **No new capture point.** `snapshot_pos_vel` already returned each axis's `pos_timestamp_us` (stamped `micros64()` in the CAN3 encoder decode, can_buses.cpp:227) and `send_telemetry` discarded it. FW 50 keeps it. Age = `micros64()` read AFTER all seven snapshots minus that stamp, so a real sample is never newer than now. The stamp is taken at decode in task_can_rx, so FlexCAN FIFO residence before decode is not in it (the CAN-ring lag census covers that separately).
- **Units and saturation.** u16 microseconds. `ENCODER_AGE_MAX = 0xFFFE` (65.534 ms) means "that age or older", `ENCODER_AGE_NONE = 0xFFFF` means no encoder frame yet this boot (stamp 0). A stamp ahead of now reads 0, never a wrapped ~65 ms. Expected live values 4 to 5.5 ms (far from either edge; a frame-drop episode of 76 to 108 ms saturates, which is the honest reading). Helper `encoder_age_us16` in axis_state.h, pure arithmetic.
- **Config/codegen.** Message and the two constants are in `config/generate_udp_protocol.py`; regenerated `config/generated/udp_protocol.{h,py}`, `Teensy_code_canbridge/udp_protocol.h`, the jetson probe copy and `docs/teensy-udp-protocol.md`. `protocol_config.yaml` untouched, so `generate_config.py` was not needed and ~/Desktop/BallButler was never written (0 changed files before and after). Decoder: the generated `teensy_link.EncoderAge` (re-exported from `teensy_link/__init__.py` and `protocol.py` with `ENCODER_AGE_SIZE/NONE/MAX`).
- **Not done: the node.** `teensy_bridge_node.py` does not subscribe to 0x96, so on the live Jetson the message is received and dropped. Publishing it needs a new ROS interface or topic (or new fields on `RobotState`/a `/encoder_age` message in jugglebot_interfaces), deliberately left to the in-flight node change.

**Control cycle walk (one 100 Hz telemetry tick).** task_telem (prio 3) runs send_telemetry: `t_teensy_us` and the seven seqlock snapshots as before (the loop only additionally stores `ts`), then one `micros64()` read, seven subtractions with branches, one extra `udp_send_stream` of 32 B. Nothing is written to any AxisState, target, queue or gate; the 500 Hz interp ISR (NVIC-16, above FreeRTOS), the setpoint staging in task_net, the CAN RX decode and every CAN TX path are not edited. The only shared resource is the UDP stream lock, held for one more ~25 to 45 us send per tick (estimate, unmeasured, the PLATFORM_IMU_BATCH figure), against the 30 ms Leg grace: ~0.15 %.

**Wire cost.** 32 B/frame (22 payload + 8 header + 2 CRC) x 100 Hz = **3200 B/s added to the uplink** (+100 datagrams/s). The TELEMETRY frame it rides beside is 74 B x 100 = 7.4 kB/s, so the telemetry pair grows 43 %, but 25.6 kbit/s is far under 1 % of the 100 Mbit link. The budget that matters is datagrams per second and per-send CPU on the bridge (the stream already carries TELEMETRY, LEG_CMD, BB_AXIS_ESTIMATES and the IMU batches at the same tick), so the cost is one more send per tick, not bandwidth.

## FEAT-64 verification

All 2026-11-25, `offline/fw50` worktree, nothing flashed.

- **Native.** `python tests/firmware/native/build.py` then `temp/firmware_native/test_leg_interp`: **76/76 cases, 10667/10667 assertions pass** (1 new case, 13 assertions: plain age, zero, both sides of the 0xFFFE ceiling, 65536 us (a u16 truncation would read 0), the no-sample stamp 0, a stamp ahead of now, and a real `write_pos_vel` then `snapshot_pos_vel` round).
- **New tests fail without the change.** The native case compiled alone (doctest main + the case body, axis_state.h from a scratch dir) against a header copy with the saturation removed: **3 of 13 assertions fail**; with the `ts == 0` no-sample line removed: **3 of 13 fail**; unmutated 13/13 pass. The wire digest test failed with the new message present until re-pinned (observed: new digest `e5e28031...f343`). The decoder tests import `EncoderAge`/`ENCODER_AGE_*`, which do not exist on the FW 49/50 tree (`git show HEAD:teensy_link/protocol.py` has no `EncoderAge`), so they cannot pass without the change; not separately executed against the old tree.
- **Python.** `flock /tmp/jugglebot-tests.lock nice -n 10 python -m pytest -q -n 4 --dist loadfile tests/firmware tests/teensy_link tests/sim/test_docs_links.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py`: **1200 passed, 4 skipped, 2 failed in 46 s**. The 2 failures are `test_logbook_index.py` (`test_index_is_current`, `test_every_entry_has_exactly_one_row`): this entry has no INDEX row yet (`python -m mapkit gen` is the orchestrator's integration step), not caused by FEAT-64.
- **Image.** `pio run -e teensy41` (build only): SUCCESS in 205 s, text 256320, data 35520, bss 113312 (dec 405152), `firmware.hex` 820984 B. Byte-identical sizes to the BUG-99-only build: the new code (about a hundred bytes) is absorbed by the section's alignment padding; telemetry.cpp.o was rebuilt and carries the 0xFFFE/0xFFFF constants. No warning from an edited file.
- **Not run.** Sibling `tests/ros` bridge-node tests (node untouched; an unsubscribed type is dropped by the client as before), `./run_tests.sh`.
