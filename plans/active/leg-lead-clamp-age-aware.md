---
title: Age-aware leg lead clamp — the frame-drop amplifier fix (prerequisite of the oval P4 clamp raise)
created: 2026-10-24
status: active
owner: harrison
last_updated: 2026-10-24
pathway: two-ball-oval
scope: >-
  Re-opens ONLY Workstream A of the archived leg-bus-frame-drops plan: the can-bridge leg lead clamp anchors
  to a stale encoder frame with no age extrapolation, so a leg-bus encoder-frame drop turns a smooth setpoint
  stream into a 12 mm staircase (single-leg ~10 Hz vibration). Three phases: P1 firmware (age-aware anchor plus
  a per-leg engagement counter on the wire), P2 bench validation against the 2026-10-24 bag episodes, P3 one
  oval level 0.50 Sitting with zero staircase episodes. Prerequisite of the next powered oval Sitting and of
  P4's leg_curr_limit_a 15 -> 30.
steps:
  - {id: P1, title: Firmware - age-aware lead clamp and engagement counter, kind: phase, status: not_started}
  - {id: P2, title: Bench validation, kind: phase, status: not_started}
  - {id: P3, title: Robot validation, kind: phase, status: not_started}
related_plan:
  - two-ball-oval.md
  - leg-bus-frame-drops.md
related_logbook:
  - 2026-08-15-fw14-validated-arc-closed.md
related_config:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h → MAX_LEAD_REV (0.10), MOTOR_FB_STALENESS_US (150000)
related_code:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp::interp_isr (leg lead clamp ~1066-1072; hand lane fb_ex ~1205-1217)
  - ros_ws/src/jugglebot/Teensy_code_canbridge/axis_state.h (pos_timestamp_us, snapshot_pos_vel)
  - tools/probes/teensy_link_profiling/hermite_xref/ (teensy_interp.py, xref.py)
---

# Age-aware leg lead clamp

## 1. Context

Two hardware Sittings on 2026-10-24 showed single-leg ~10 Hz vibrations lasting 0.5-0.9 s: legs 1 and 2 (three events: leg 1 twice, leg 2 once), both
planner modes (`level` and `pose`), so not a planner artefact. One event had the leg's current setpoint on the 15 A
clamp with +-19 mm position error. The bag (`~/Desktop/rosbags/2026-10-24_10-44-21/`) shows the cause: that leg's
encoder feedback on the leg CAN bus dropped to ~10 Hz for ~1 s. The can-bridge leg lead clamp sets target = encoder
feedback +- `MAX_LEAD_REV` (0.10 rev = 7.07 mm) with the feedback frozen at its last frame, so the commanded stream
became a ~12 mm staircase at the encoder-frame rate; the ODrive position loop chased each step. The 60 Hz telemetry
under-sampled the event (the clamp is evaluated at 500 Hz), so engagement was inferred, not read.
Replay oracles: epoch 1792799742.4 (leg 1) and 1792799921.7 (leg 2) in that bag.
Owner decision 2026-10-24: this is a PREREQUISITE of the next powered oval Sitting and of the P4 raise
`leg_curr_limit_a` 15 -> 30 in `two-ball-oval.md` (a staircase against a 30 A clamp is a harder hit, not a softer one).

## 2. What this plan does NOT reopen

Workstream B of `leg-bus-frame-drops.md` (why the ODrives stop sending encoder frames: the TX-suppression A/B, the
SDO read, candidate remedies) stays archived. This plan makes a dropout cost tracking accuracy instead of a
commanded stop-and-jump; it does not remove the dropouts. The archived plan's A3 ("ships at all before B reports")
is decided: it ships, as defence-in-depth, because the drops are now seen on two legs under load.

## 3. Design

Today (`leg_interp.cpp` lines 1066-1072): `fb = axes[i].pos_rev` (a bare single-word read), `dev = cmd_pos - fb`,
clamped to +-`LEAD` (= `MAX_LEAD_REV`), `clamp_mask` bit i set on engagement. No age is consulted.
The hand lane already does the right thing (lines ~1205-1217): `fb_ex = fb + vel_rps * age`, with age =
`now_mono - pos_timestamp_us` capped at `MOTOR_FB_STALENESS_US` and the cap counted (`s_hand_stale_holds`);
the raw age feeds the deviation guard's freshness gate; engagement is counted in `s_hand_lead_clamp_ticks`.

Workstream A's argument (archived § 3.1-3.2): under a dropout no frame is decoded, so `pos_timestamp_us` genuinely
ages (76-96 ms measured, against a 1-10 ms healthy floor, 10 ms nominal refresh), an order-of-magnitude separation.
Delay line means fresh stamp with stale content (age blind); dropout means no write, so age is informative.
Measured support (archived D3 gate, n = 25): anchor error p95 0.129 -> 0.064 rev; freezes exceeding the clamp budget
0.160 -> 0.000.

**Design decision for the implementer (A1/A2, state the choice and the numbers in the logbook):**
- Option X, velocity-extrapolated anchor (archived recommendation): clamp against `pos + vel * age` while aged, the
  band keeping its full 2 x `MAX_LEAD_REV` width; age capped, terminating in the existing recoverable
  `MOTOR_FB_STALE` suppression. Risk: extrapolating a frozen `vel_rps` through a real deceleration.
- Option Y, widen-and-hold: keep the frozen anchor but widen the band by `|vel| * age` up to a cap. Smaller
  behavioural change at age 0 and no reliance on the velocity estimate's sign, but a wider band under a stale
  anchor leans on the ODrive's own limits more.
Whichever is chosen: the clamp must be bit-identical to today at zero age (property test), and the
`MAX_DEVIATION` guard's exposure must be shown unchanged (it compares raw pre-clamp `u0` against the same
anchor; archived § 3.3 item 2). A1 (age threshold and extrapolation cap) is the safety-envelope trade: coast
coverage versus a worst-case P-term sprint under a bind-plus-gap coincidence. Start from cap = `MOTOR_FB_STALENESS_US`
as the hand lane has it, and justify any other value from the bag episodes.
**ISR discipline (archived § 3.3 item 3, load-bearing):** bare single-word loads only. `snapshot_pos_vel`'s seqlock
retry loop would hang the bridge if called from `interp_isr`. The hand lane reads `pos_timestamp_us` through
`atomic_read_u64`; copy that, one ISR-local accessor, not three call sites.
**Telemetry (new, required):** a per-leg lead-clamp duty counter (ticks engaged) and a per-leg stale-anchor counter
(ticks with age above threshold), cumulative since boot, mirroring `s_hand_lead_clamp_ticks` / `s_hand_stale_holds`.
Today `lead_clamp_mask` rides the heartbeat at 10 Hz, which is what under-sampled the event.

## 4. Phases

### P1 — Firmware - age-aware lead clamp and engagement counter
- Implement the chosen option in `leg_interp.cpp` for legs 0-5 only; the hand lane is untouched.
- Add the two per-leg counters. Wire format: prefer a new additive 1 Hz uplink in the `BridgeTxDiag` pattern (a new
  `MsgType` + payload struct in `udp_protocol.h`, decoder in `teensy_link/protocol.py`, row in
  `config/protocol_config.yaml`, then `python config/generate_config.py`). Precedent: BridgeTxDiag and
  CacheDiag-style additions carried NO `PROTOCOL_VERSION` bump ("an old Jetson ignores the unknown msg_type"),
  whereas GROWING an existing struct (HeartbeatT2J, CacheDiag at 8->9) is incompatible and bumps it. So: new
  message means no bump; extending a heartbeat means a bump, lockstep flash, and darkness against old boards.
  Pick the new-message route unless there is a reason not to. Bump `FW_VERSION` (currently 30) either way.
- Mirror the clamp change in `teensy_interp.py` (see § 7: its `MAX_LEAD_REV` is already stale).
- Exit: firmware builds (`pio run -e teensy41`); xref still 0.0 rev on the unchanged-clamp cases; counters decode
  in `teensy_link` with a unit test; ROS bag/telemetry plumbing records them (check the bridge node's republish).

### P2 — Bench validation
- Offline replay of the 2026-10-24 episodes (leg 1 at 1792799742.4, leg 2 at 1792799921.7) through the `hermite_xref`
  twin: today's clamp reproduces the staircase (replay oracle must show it first), the new clamp must not pin at
  `MAX_LEAD_REV` through either, and must match today's output bit-for-bit on the clean windows either side.
  Add the archived plan's four 2026-08-15 episodes (ax1, ax4, ax5, ax2) if their traces still exist.
- Injected frame gaps on the bench rig (hold-last-frame for 76-1000 ms mid-stream): commanded step size stays
  bounded by the setpoint stream's own velocity, not by `MAX_LEAD_REV`; `MOTOR_FB_STALE` still terminates a dead axis.
- Exit: both replays and the injected-gap run pass; the property test (never wider than today at age 0) passes.

### P3 — Robot validation
- Flash per the project rule: launch DOWN, `pio run -e teensy41 -t upload` from the firmware directory; receipt =
  the BridgeIdentity sniff showing the new `fw_version`. Full launch restart afterwards if interfaces changed.
- One oval `level` dwell 0.50 Sitting. Gate: zero staircase episodes (no leg with a >= 5 Hz setpoint-step signature in
  the bag; the new counters say how many ticks any leg's anchor was stale and whether the clamp engaged during
  each). A frame-drop episode that occurs but costs only tracking accuracy is a PASS and is the point.
- Exit: gate met, logbook entry with the (date, command, result) triple; `two-ball-oval.md` P4 note updated.

## 5. Gate

Prerequisite of: the next powered two-ball-oval Sitting, and P4's `leg_curr_limit_a` 15 -> 30
(`config/hardware_config.yaml`, with the ODrive backup JSON). Neither proceeds until P3 passes or the owner waives it.

## 6. Testing

Python: protocol decode test for the new message; `xref.py` regression (0.0 rev on existing cases); replay test
over the bag episodes (extract the two windows into a small committed fixture or a probe under `tools/probes/`).
Firmware: age-0 identity property test; accessor threshold test. Run `./run_tests.sh` before commit, and
`./run_tests.sh --full` before the hardware Sitting (CLAUDE.md case (a)). Control implications to walk through per
the project rule: one 500 Hz tick with age 0, age 80 ms, age >= 150 ms (suppression) for each option, including the
feedforward path (`vel_ff` is bounded by `VELFF_CAP`, unaffected) and the `MAX_DEVIATION` comparison.

## 7. Notes for collaborators

- Claude flashes the bridge Teensy (no ask needed when the new build is the only relevant one): launch down, `cd`
  into the firmware directory, bare `pio run -e teensy41 -t upload` (no compound lines, the classifier denies them).
- The leg path's safety authority is the Teensy `MAX_DEVIATION` guard; nothing on the Jetson sits below the clamp.
  Treat this as live-control-path firmware: a bench rig first, owner present for the first powered flash.
- `hand_source` latch resets on every bridge reboot (latch before arming).
- Stale-against-code notes found while writing: `teensy_interp.py` still has `MAX_LEAD_REV = 0.15` (firmware is 0.10,
  documented as a deliberate divergence in its header, so the twin cannot be the oracle for the leg clamp until P1
  updates it); the archived plan cites `interp_isr` generally, the leg clamp now sits at lines 1066-1072 and the
  hand-lane extrapolation at ~1205-1217; FW_VERSION is 30, not the 14 the archived evidence was gathered on.
