---
title: Age-aware leg lead clamp — the frame-drop amplifier fix (prerequisite of the oval P4 clamp raise)
created: 2026-10-24
status: completed
archived: 2026-10-27
completed: 2026-10-27
owner: harrison
last_updated: 2026-10-27
pathway: two-ball-oval
scope: >-
  Re-opens ONLY Workstream A of the archived leg-bus-frame-drops plan: the can-bridge leg lead clamp anchors
  to a stale encoder frame with no age extrapolation, so a leg-bus encoder-frame drop turns a smooth setpoint
  stream into a 12 mm staircase (single-leg ~10 Hz vibration). Three phases: P1 firmware (age-aware anchor plus
  a per-leg engagement counter on the wire), P2 bench validation against the 2026-10-24 bag episodes, P3 one
  oval level 0.50 Sitting with zero staircase episodes. Prerequisite of the next powered oval Sitting and of
  P4's leg_curr_limit_a 15 -> 30.
steps:
  - {id: P1, title: Firmware - age-aware lead clamp and engagement counter, kind: phase, status: complete, date: 2026-10-24, effort: m, hardware: bench-leg}
  - {id: P2, title: Bench validation, kind: phase, status: complete, date: 2026-10-27, effort: s, hardware: bench-leg}
  - {id: P3, title: Robot validation, kind: phase, status: complete, date: 2026-10-27, effort: s, hardware: robot}
related_plan:
  - two-ball-oval.md
  - leg-bus-frame-drops.md
related_logbook:
  - 2026-08-15-fw14-validated-arc-closed.md
  - 2026-10-24-leg-lead-clamp-age-aware-p1-firmware.md
  - 2026-10-25-leg-lead-clamp-age-aware-p2-bench.md
  - 2026-10-27-leg-lead-clamp-age-aware-p3-robot.md
related_config:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h → MAX_LEAD_REV (0.10), MOTOR_FB_STALENESS_US (150000, the anchor's age cap), LEG_ANCHOR_STALE_US (30000, the stale census threshold, FW 31), LEG_ANCHOR_SHIFT_SLEW_REV (vel_limit x tick = 0.024, FW 32), FW_VERSION (33: FW 32 + the additive Wave camera RPC)
related_code:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp::leg_clamp_anchor (the single anchor derivation, ~232) and ::interp_isr (leg lead clamp ~1136-1146; recovery-slew re-clamp ~1463; hand lane fb_ex ~1265-1288)
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp::leg_clamp_diag_uplink_step (LegClampDiag 0x94 at 1 Hz)
  - config/generate_udp_protocol.py (LegClampDiag; generates udp_protocol.h/.py + docs/teensy-udp-protocol.md)
  - ros_ws/src/jugglebot/Teensy_code_canbridge/axis_state.h (pos_timestamp_us, snapshot_pos_vel)
  - tools/probes/teensy_link_profiling/hermite_xref/ (teensy_interp.py leg_clamp modes, xref.py)
  - tools/probes/lead_clamp_staircase_probe.py (+ data/lead_clamp_staircase_2026-10-24.json, the replay oracle fixture)
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

Through FW 30 (`leg_interp.cpp` lines 1066-1072 at `2bafbae7`): `fb = axes[i].pos_rev` (a bare single-word read), `dev = cmd_pos - fb`,
clamped to +-`LEAD` (= `MAX_LEAD_REV`), `clamp_mask` bit i set on engagement. No age is consulted.
The hand lane already does the right thing (now lines ~1265-1288): `fb_ex = fb + vel_rps * age`, with age =
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
**Decided 2026-10-24 (owner, P1): neither X nor Y — Option Z, the dead-reckoned anchor** of the archived
content-freshness draft § 4.2: `anchor = pos_rev + (raw cmd now - raw cmd at the tick the frame landed)`, the shift
frozen once the age passes `MOTOR_FB_STALENESS_US` (150 ms, the cap). The measured basis is the P1 logbook's replay of
the five 2026-10-24 staircase episodes: the legs ACCELERATE through the 76-108 ms gaps (42-200 rev/s^2), so X and Y
(identical on this data, no reversals) left ~75 % of the step (8.4-9.5 mm against today's 11.6-12.7); Z froze the
clamp's verdict for the gap's length (4.0-6.1 mm in that first-cut 100 Hz replay; 2.6 / 8.6 / 3.8 mm in the 500 Hz
twin oracle run afterwards, the residue being the lag today's clamp had already built into the recorded trace) and
needs no velocity estimate. Trade accepted: through a gap a command discontinuity passes unclamped
until the next frame, and a leg that binds exactly as its frames stop is shoved by the gap's command travel (10-27 mm
measured; bounded by `v x cap`) — the same bind exposure X and Y carry. Age cap 150 ms (hand parity; the fault task's
`MOTOR_FB_STALE` is the terminating arm). Stale census threshold `LEG_ANCHOR_STALE_US` 30 ms (healthy p95 <= 10.1 ms,
drops >= 76 ms). Logbook: `2026-10-24-leg-lead-clamp-age-aware-p1-firmware.md` § Design and § Discussion.

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
  `MsgType` + `Message` in `config/generate_udp_protocol.py`, which GENERATES `udp_protocol.h` for the firmware and
  `config/generated/`, `udp_protocol.py` and `docs/teensy-udp-protocol.md` — the Teensy header is a delivered copy,
  never hand-edited; `teensy_link/protocol.py` re-exports the generated class. The plan originally said
  "`config/protocol_config.yaml` + `generate_config.py`": that file holds CAN constants, not UDP messages — refreshed
  at P1). Precedent: BridgeTxDiag and
  CacheDiag-style additions carried NO `PROTOCOL_VERSION` bump ("an old Jetson ignores the unknown msg_type"),
  whereas GROWING an existing struct (HeartbeatT2J, CacheDiag at 8->9) is incompatible and bumps it. So: new
  message means no bump; extending a heartbeat means a bump, lockstep flash, and darkness against old boards.
  Pick the new-message route unless there is a reason not to. Bump `FW_VERSION` (currently 30) either way.
- Mirror the clamp change in `teensy_interp.py` (see § 7: its `MAX_LEAD_REV` is already stale).
- Exit: firmware builds (`pio run -e teensy41`); xref still 0.0 rev on the unchanged-clamp cases; counters decode
  in `teensy_link` with a unit test; ROS bag/telemetry plumbing records them (check the bridge node's republish).
- **Outcome (2026-10-24): COMPLETE, NOT FLASHED.** Commits by `git log --grep "Logbook-Entry: 2026-10-24-leg-lead-clamp-age-aware-p1-firmware"`;
  logbook `logbook/2026-10-24-leg-lead-clamp-age-aware-p1-firmware.md`. Landed: `leg_clamp_anchor()` (one derivation per
  leg per tick, consumed by the lead clamp AND the recovery-slew re-clamp), the two per-leg cumulative counters,
  `LegClampDiag` 0x94 at 1 Hz from `task_telem`, `FW_VERSION` 31 / `EXPECTED_BRIDGE_FW_VERSION` 31, `/link_status`
  row `leg_clamp_diag`, the twin's `leg_clamp` modes (`motor_guard` default untouched, xref 0.0 rev; `firmware_fw30`;
  `firmware`), the replay oracle on the committed fixture `tools/probes/data/lead_clamp_staircase_2026-10-24.json`
  (FW 30 replays 11.6 / 12.4 / 14.9 mm staircases against the bag-recorded 11.6 / 12.4 / 12.4; FW 31 cuts the max step
  77 / 31 / 74 % and engages on ZERO
  gap ticks whose frame-time error was inside the band), five native doctest cases on the compiled ISR (age-0
  bit-identity, coast, cap freeze, census gating, the pinned command-discontinuity exposure). Build receipt:
  `pio run -e teensy41` 2026-10-24, SUCCESS (`temp/logs/lead-clamp-p1-pio-build-*.log`). Test-count delta and the
  full-gate triple: logbook § Verification. **P2 cleared to start**: the replay half of P2 is prototyped here (the
  fixture + `replay_window`); P2 owes the bench-rig injected-gap run and the archived plan's four 2026-08-15 episodes
  if their traces still exist. Open: a dead-reckoned-shift slew bound (would hold a command discontinuity during a
  gap to the drive's `vel_limit` rate) was considered and not built — see the logbook § Open Questions.

### P2 — Bench validation
- Offline replay of the 2026-10-24 episodes (leg 1 at 1792799742.4, leg 2 at 1792799921.7) through the `hermite_xref`
  twin: today's clamp reproduces the staircase (replay oracle must show it first), the new clamp must not pin at
  `MAX_LEAD_REV` through either, and must match today's output bit-for-bit on the clean windows either side.
  Add the archived plan's four 2026-08-15 episodes (ax1, ax4, ax5, ax2) if their traces still exist.
- Injected frame gaps on the bench rig (hold-last-frame for 76-1000 ms mid-stream): commanded step size stays
  bounded by the setpoint stream's own velocity, not by `MAX_LEAD_REV`; `MOTOR_FB_STALE` still terminates a dead axis.
- Exit: both replays and the injected-gap run pass; the property test (never wider than today at age 0) passes.
- **Outcome (2026-10-27): COMPLETE; the board is on FW 33.** Commits: `git log --grep "Logbook-Entry: 2026-10-25-leg-lead-clamp-age-aware-p2-bench"`.
  Logbook: `logbook/2026-10-25-leg-lead-clamp-age-aware-p2-bench.md`.
  - **Firmware.** P1's open question became FW 32: the dead-reckoned shift grows by at most `LEG_ANCHOR_SHIFT_SLEW_REV` (vel_limit × tick = 0.024 rev) per tick. It is invisible to any stream at or below vel_limit and ramps an in-gap command discontinuity behind the clamp. The Wave camera merge carries it into FW 33 with `leg_interp.cpp` byte-identical.
  - **Replay.** It covers S1A/S1B/S2 plus the surviving 2026-08-15 windows: ax1 (FW 30 10.64 mm = recorded; FW 31/32 3.35 mm) and ax4 (no-harm, identical). ax5 and ax2 had no gap. Clean ticks are bit-identical to FW 30 in all five windows.
  - **Bench A/B (2026-10-27).** The 7th leg ran on CAN2 node 0 with ODrive SDO `encoder_msg_rate_ms` (endpoint 275), two runs per build, on the FW 30 and FW 33 images. In the 100 ms-rate gap, FW 30 staircases (8 transmitted ~10.7 mm steps) and FW 33 is smooth (0 clamp ticks of 350 stale). `MOTOR_FB_STALE` trips and clears in every dead gap, with nothing latched.
  - **Filed, not fixed.** JB-72 (`HomingMonitor` false timeout on a short home) and JB-73 (the last vel_ff is held through suppression).
  - **P3 cleared to start.** Its flash step is now a receipt check (expect `fw_version` 33) plus the oval Sitting.

### P3 — Robot validation
- Flash per the project rule: launch DOWN, `pio run -e teensy41 -t upload` from the firmware directory; receipt =
  the BridgeIdentity sniff showing the new `fw_version`. Full launch restart afterwards if interfaces changed.
- One oval `level` dwell 0.50 Sitting. Gate: zero staircase episodes (no leg with a >= 5 Hz setpoint-step signature in
  the bag; the new counters say how many ticks any leg's anchor was stale and whether the clamp engaged during
  each). A frame-drop episode that occurs but costs only tracking accuracy is a PASS and is the point.
- Exit: gate met, logbook entry with the (date, command, result) triple; `two-ball-oval.md` P4 note updated.
- **Outcome (2026-10-27): COMPLETE; gate met.** Logbook: `logbook/2026-10-27-leg-lead-clamp-age-aware-p3-robot.md`;
  commits: `git log --grep "Logbook-Entry: 2026-10-27-leg-lead-clamp-age-aware-p3-robot"`. No flash (FW 33 receipt).
  - **The Sitting.** One `level` launch, 22:59: four `oval_1ball` goals at dwell 0.50 among ten accepted. `lead_clamp_staircase_probe.py --scan` (new) found **0 staircase
    episodes** in 505.6 s. The `leg_clamp_diag` delta was clamp `0 0 0 0 0 0`, stale `0 920 605 590 649 700`: real frame drops,
    absorbed with no clamp engagement.
  - **The fix in action.** Leg 2 crossed a 112 ms frame-age window at 3.7 rev/s. The FW 30 twin replays it as 251 clamp ticks
    and 20 mm steps; FW 33 recorded none (max step 2.33 mm per 10 ms sample). Peak `iq_setpoint` was 10 A.
  - **Filed, not fixed.** JB-75 (`SETPOINT_STALE` latch on an arm edge: the last-setpoint stamp survives arms; the bridge was not
    power-cycled after P2) and JB-76 (`SPLICE_TOO_LATE` ended 5 of 10 goals; CATCH solve p90 276 ms against the 250 ms deadline).
    Leg 0's stale census stayed flat through its own dropout episode. That is ambiguous, not a defect: read it after a power-cycle.

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
- Stale-against-code notes found while writing (refreshed at P1): `teensy_interp.py`'s `MAX_LEAD_REV = 0.15` is NOT a
  stale firmware copy — it is motor_guard's value and the xref pins the twin to motor_guard at 0.0 rev (a bare 0.15 → 0.10
  edit fails `tests/firmware/test_hermite_xref.py`); since P1 the twin carries the firmware clamp as a MODE
  (`TeensyLegInterp(leg_clamp="firmware")`, constants `MAX_LEAD_REV_FW` 0.10 / `LEAD_CLAMP_VELFF_LIMIT_RPS` 3.5 /
  `LEG_ANCHOR_STALE_S` 0.030 pinned against the header by `tests/firmware/test_leg_clamp_age_aware_twin.py`) and the
  default stays motor_guard's. The archived plan cites `interp_isr` generally; at FW 31 the anchor derivation is
  `leg_clamp_anchor()` (~232), the leg clamp loop ~1136-1146, the slew re-clamp ~1463, the hand-lane extrapolation
  ~1265-1288. FW_VERSION is 33 (30 when the plan was filed; 31 at P1; 32 at P2's slew bound; 33 after the Wave camera merge; the archived evidence was gathered on 14).

## Archival note (2026-10-27)

**What shipped.**
- **Firmware.**
  - The can-bridge leg lead clamp anchors to a dead-reckoned encoder estimate through an encoder-frame gap (FW 31, form Z, capped at `MOTOR_FB_STALENESS_US`).
  - The shift slews at `vel_limit` per tick (FW 32).
  - A per-leg clamp and stale-anchor census goes out on the wire as `LegClampDiag` and is shown on `/link_status` `leg_clamp_diag`.
  - All of it is carried byte-identical in FW 33, alongside the Wave camera RPC.
- **Tooling.**
  - The twin's `leg_clamp` modes.
  - The replay fixtures.
  - The bench gap driver `tools/probes/lead_clamp_gap_bench.py`.
  - The bag read-back `tools/probes/lead_clamp_staircase_probe.py --scan`.

**Why it closed.** All three phases met their exits:
- the replays;
- the bench A/B (FW 30 staircases, FW 33 smooth);
- one `level` 0.50 oval Sitting with zero staircase episodes and zero clamp ticks through real frame drops.

That removes this prerequisite from `plans/active/two-ball-oval.md` (runbook § 9 re-gate, P4's 30 A raise).

**Residue, re-homed.**
- Filed as map Bugs:
  - JB-72: the `HomingMonitor` false timeout on a short home. P2's unexplained first-home 1.15 rev / `HOMING_OK` reading is most likely the same observer gap, and its log is empty.
  - JB-73: the last `vel_ff` is held through `MOTOR_FB_STALE` suppression.
  - JB-75: a `SETPOINT_STALE` latch on an arm edge.
  - JB-76: `SPLICE_TOO_LATE` ends half the goals.
- **Leg 0's stale census** stayed flat through its own dropout episode at P3. It is ambiguous rather than defective; read it after a bridge power-cycle (P3 entry).
- **The bind-plus-gap exposure** (P1 tradeoff 3: a command discontinuity inside a gap) is an accepted trade, bounded since FW 32 by the slew. P3 shows zero engagement on the robot.
- **Workstream B**, why the ODrives drop frames, stays in `plans/archived/leg-bus-frame-drops.md`.
