---
title: Phase 11 U5 — powered six-leg β cold-start (home → configure → activate) + sitting fixes
type: feature
date: 2026-06-26
status: in-progress
phase: "11"
related_plan: teensy-can-offload.md
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_activate.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_activate.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_homing.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/odrive_protocol.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - config/generate_config.py
  - config/generate_udp_protocol.py
  - controller/teensy_link/activate.py
  - controller/teensy_link/homing.py
  - controller/teensy_link/rpc_args.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/teensy_link/test_activate.py
  - tests/firmware/test_activate_xref.py
  - tests/teensy_link/test_homing.py
  - tests/ros/test_teensy_bridge_node_activate.py
  - tests/ros/test_teensy_bridge_node_home.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_deactivate.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_deactivate.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
  - controller/teensy_link/deactivate.py
  - tests/teensy_link/test_deactivate.py
  - tests/firmware/test_deactivate_xref.py
  - tests/ros/test_teensy_bridge_node_deactivate.py
commits:
  - ad7b3b3
  - 2564a77
  - 4c3dbfd
  - dbf32c9
subsystem:
  - can
  - controller
  - motion
tags:
  - safety
  - testing
---

# Phase 11 U5 — powered six-leg β cold-start (home → configure → activate)

**Summary.** U5 brings the β leg path (40 Hz MPC knots → Teensy 500 Hz Hermite,
the can-bridge owning CAN3) onto **all six legs of the real robot** for the first
time. **U5a** (desk-side, commit `ad7b3b3`) added the β-path *cold-start
orchestration* that the cutover never had — there was no equivalent of can_node's
`_setup_odrives_steps` (gains/mode/limits) or `_gentle_move_steps` (move to the
active pose), because the bridge's only leg-motion path is the gated 40 Hz
setpoint stream. **U5b** (the powered operator sitting, this entry's focus)
validated the full cold-start ladder under power: **power + 6 heartbeats →
encoder index search → homing → configure → activate to the active pose → arm**,
all firsts on the can-bridge path. The headline result: a **clean six-leg
TRAP_TRAJ even-rise to Active** — the first coordinated six-leg motion off the
can-bridge. The sitting then **stopped before the closed-loop `run_mpc` hold
(rung 1)** on discovering that `run_mpc`'s `:5556` feedback is published by
`motor_guard` (absent in the bridge-only launch) and has **never been exercised
on the β path** — a fresh sub-investigation deferred to the next session. Three
hardware-forced bugs were found and fixed during the sitting (homing foam-stop
observer, activate CAN-TX burst, homing next-axis race). This entry is the
powered companion to the U5a code record and stays `in-progress` (run_mpc +
decommission remain). Cold-start choreography + design rationale in the U5a
commit; the bench-validation template is [[2026-06-24-phase11-bench-cutover]].

## The sitting (2026-06-26, full six-leg robot, ~45 V bus, e-stop in hand)

Each rung operator-gated. The bridge was brought up **bridge-only**
(`teensy_bridge_launch.py`, sole J→T authority, `enable_setpoint_output=false`)
for the cold-start services.

| Rung | Action | Result |
|---|---|---|
| 0a | Power CAN3; verify heartbeats | 6 legs + hand IDLE, `active_errors=0`, `fault=NONE`, `bus_voltage ≈ 45.3 V`, no motion ✓ |
| 0b | `/encoder_search` (all 6) | every leg `pos: NaN → finite`, no errors ✓ |
| 0c | `/home` (all 6, sequential) + auto-`/configure` | all 6 home; gains/limits/PASSTHROUGH applied ✓ (after the homing fixes below) |
| — | `/activate` (AXIS_ALL, parallel) | **TRAP_TRAJ even-rise to Active; all 6 reach ≈ 2.19 rev, CLOSED_LOOP, holding cleanly (±1.3 A gravity-comp)** ✓ |
| — | `/configure` (TRAP_TRAJ → PASSTHROUGH at Active) | legs hold Active in PASSTHROUGH, no motion ✓ |
| 1 (arm) | bridge relaunch `enable_setpoint_output:=true` | `mpc_active=1`, `setpoints_sent=0`, legs hold Active — **no motion at arm** ✓ |
| 1 (hold) | `run_mpc.py --pose 0,0,170` | **deferred** — feedback-topology gap (below) |

## The bugs the hardware forced out

*Three real bugs surfaced under power; each was diagnosed and fixed in-session
(diagnosis clear, fixes small), with the operator's hardware intuition load-bearing
at every pivot.*

**1. Homing observer asserted a position the foam stop makes unreliable.**
The first `/home` failed: `axis N returned to IDLE at pos −0.152 (expected
|pos|≈0.100) — reference not set`. The Phase-9b `HomingMonitor` inferred success
from `|pos| ≈ |HOMING_LEG_ABS_POS_REV|` after the `CLOSED_LOOP → IDLE` cycle. The
operator's intuition ("can_node homing was rock-solid for months") was the
tell: the legs **bottom into a foam stop that pushes them back out by a variable
amount the instant they switch to IDLE**, so the post-IDLE telemetry position is
not a reliable success signal. **Fix:** trust the firmware's internal current-spike
trip (exactly as can_node did) — success = observed `CLOSED_LOOP → IDLE` with no
active errors; drop the position assertion; a leg that never trips is caught by
the per-axis timeout (`controller/teensy_link/homing.py`).

**2. Activate fired but the legs never armed — a CAN-TX burst overflow.**
`/activate` returned `timed out — did not reach active pose` with every leg's
`pos` unchanged and `vel=0`. Instrumented capture (621 `robot_state` samples)
was decisive: legs 0–4 **stayed in IDLE the whole time; only leg 5 (the last
configured) reached CLOSED_LOOP**. Root cause: the firmware activate SETUP issued
**5 frames × 6 legs = 30 CAN frames in a single 100 Hz tick**, overflowing the
**`TX_SIZE_16`** CAN3 software TX buffer — most `set_state(CLOSED_LOOP)` frames
were silently dropped. Homing never hit this (sequential, ~3 frames/tick). The
operator surfaced the architectural framing: a reliable TX path that never drops
on bursts (the plan's specced-but-unbuilt `can3_tx_task`), echoing the `delay(5)`
calls can_node used after CAN bursts. **Fix (two parts):** stagger the activate
SETUP to **one leg per tick** (mirrors the proven homing pattern), and **deepen
the CAN TX buffers `TX_SIZE_16 → TX_SIZE_64`** on all three buses (the structural
class fix; the ISR drain already exists, it just needed depth). Re-ran: clean
six-leg even-rise to Active.

**3. Homing next-axis race introduced by fix #1.** After fix #1, a later `/home`
failed `axis 4: HOME rejected: ERR_REJECTED (succeeded: [0,1,2,3])`. The
position-free observer declares an axis "done" the instant `axis_state` reads
IDLE — which is **during the firmware's `STOP_SETTLE` window, before
`set_absolute_position` completes and `s_phase` returns to IDLE** (~10–20 ms). So
`_run_home` fired the *next* axis's HOME while the firmware was still finishing
the previous one → the firmware correctly rejected it (busy). (The old `|pos|`
check had accidentally masked this — it waited for `set_absolute_position`.)
**Fix:** retry `HOME` on `ERR_REJECTED` across ticks until the firmware accepts
it (bounded) — the principled form of can_node's post-TX delays
(`teensy_bridge_node._run_home`).

## Where we stopped — and why

`run_mpc.py` (`HardwarePlant`) SUBs **`:5556` for leg feedback, published by
`motor_guard`** (`_publish_telemetry`), fed via `motion_bridge_node` from the
bridge's telemetry. The bridge-only launch runs **none of that middle**, so
`run_mpc` would be feedback-blind and staleness-E-STOP. The operator confirmed
the production MPC path is the **full `jugglebot_launch.py`** (which brings up
motor_guard + motion_bridge) and that **`run_mpc` has never been run end-to-end
on the β path** — rung 1 is genuinely new. Rather than push an untested
closed-loop topology at the tail of a long sitting, U5b lands here as a complete,
validated cold-start unit; the closed-loop hold is the next session's first task.

The robot was safed by the operator: e-stop, lowered by hand, motor PSU off (the
can-bridge stays on 12 V, preserving the ODrive homing reference).

## Verification

- **Off-hardware suite:** `pytest tests/ -q` (run 2026-06-26): **1853 passed,
  1 xfailed in 453.87 s** (baseline 1819 + U5a's 32 new + U5b's 2 homing-retry
  tests; no regressions).
- **Firmware:** `pio run` green, dec **360512** (U5a 356640 + the deeper TX
  buffers' bss). Flashed to the can-bridge 2026-06-26 (HalfKay → programmed →
  booting); bridge healthy on the link post-flash, boots `mpc_active=0` gated.
- **Hardware (live, 2026-06-26):** `/encoder_search` → `success` on `[0..5]`;
  `/home` → `homing complete on axes [0,1,2,3,4,5]; configure complete on
  [0,1,2,3,4,5]`; `/activate` → `activate complete on axes [0,1,2,3,4,5]`;
  `/robot_state` at Active showed all six legs `CLOSED_LOOP`, `pos ≈ 2.19`,
  `active_errors=0`, `fault=NONE`, holding currents ±1.3 A.
- **Audit:** the U5a code (`ad7b3b3`) passed two independent adversarial audits;
  the U5b fixes were tested (homing 18/18; the activate stagger is dec-confirmed
  recompiled).

## Discussion

**The foam stop is why position can't gate homing success.** can_node decided
success by its *own* current-trip (it ran the generator in-process). The β
observer is blind to that trip and tried to infer it from the resting position —
which the foam relaxation corrupts. Trusting the trip (CLOSED_LOOP → IDLE) is the
correct abstraction; the position was never the success signal, only a proxy.

**The TX burst is a class, deepened the buffer to close it.** Fix #2 could have
been a pure activate patch (stagger). But the operator's framing — "ensure no
frames are ever dropped on dense bursts" — is the right altitude: deepening the
TX buffer absorbs *any* realistic burst via the existing ISR drain, so no caller
has to hand-pace its frames. The full `can3_tx_task` (large queue + pacing/
priority) is the plan's design for arbitrary bursts; deferred as over-engineering
for our traffic (worst case ~30 frames), but recorded as the principled endpoint.

**The activate seed is gentler than legacy.** can_node's `_send_position_target`
clips, so a homed leg at ≈ −0.10 rev snaps ~7 mm off the hardstop at CLOSED_LOOP
entry. U5's activate seeds the *actual* (non-clipped) position before the
TRAP_TRAJ move, so the whole rise is profiled with no snap.

**Benign artifacts clarified:** the **−6 A at IDLE** on homed legs is
`iq_measured` latching the last CLOSED_LOOP value (the homing push), not live
current (temps flat, positions static; it correlates exactly with "was this leg
pushed"). And **12 V logic power preserves the ODrive encoder/homing reference**
across motor-bus cycles — a re-home is not needed after a motor-power-off.

## 2026-06-27 continuation — /deactivate service + run_mpc-on-β attempt

The next session resumed the U5 tail. Two outcomes: **`/deactivate` built and
hardware-validated** (closing the U5b shutdown gap), and the **run_mpc closed-loop
hold attempted on β — the feedback chain *connects*, but the hold is blocked by
the known MPC compute-marginality, not by anything in the cutover.** That, plus a
contemplated control-architecture change, re-scoped the remaining work to a
foundation-first `can_node`→Teensy parity audit (see Discussion).

**`/deactivate` — DONE + validated (commit `dbf32c9`, firmware dec 362592).** The
controlled inverse of `/activate`: a firmware `DEACTIVATE` op (`leg_deactivate.cpp`,
`RpcMethod 0x0023`) mirroring `ACTIVATE` — seed live pos, TRAP_TRAJ, CLOSED_LOOP,
descend to the STOW pose (`STOW_OFF_POSE_REV = 0.0`), then **IDLE each leg on
arrival** (the one behavioural difference from activate, which holds Active). The
β-path analogue of `can_node` deactivate. The Jetson `DeactivateMonitor`
(`controller/teensy_link/deactivate.py`) judges completion by **latching arrival
evidence (`|pos−stow| ≤ tol`) while CLOSED_LOOP, then confirming the IDLE** — never
from a post-IDLE position (foam-relaxation lesson). `HOME/ACTIVATE/DEACTIVATE` are
now fully mutually exclusive both directions; the deferred-stow reconnect excludes
all three cold-start moves (this also closed a pre-existing `ACTIVATE` asymmetry in
`fault_machine`). Two adversarial-audit findings were fixed pre-flash (the observer
abort-vs-clean discrimination; the activate deferred-stow exclusion). Operator
verdict on hardware: **"works perfectly."**

**run_mpc on β — the feedback chain CLOSES; the hold does not (compute-bound,
pre-existing).** With the full `jugglebot_launch.py` (gate off) up and the legs
held at Active, `run_mpc.py --pose 0,0,170` was run for the first time on the β
path. It **connected to motor_guard via `:5556`, received leg feedback, and the
first solve `Solve_Succeeded`** — so the `Teensy → teensy_bridge → /robot_state →
motion_bridge → motor_guard → :5556 → MPC` feedback chain is confirmed end-to-end
on β (the U5b headline open item). But the run then cascaded: mean solve 11.3 ms,
yet a ~250 ms loop stall at step ≈70 (`drain=25` telemetry backlog; inflated
`solve_setup`/`hooks` = a transient CPU steal) corrupted the warm-start →
`Maximum_CpuTime_Exceeded` (24-29 ms) on every subsequent solve → the
`motor_pos`-staleness watchdog E-STOP'd at 0.5 s. This is the **2026-04-18 /
2026-05-22 overhead-spike / compute-marginality failure class, not a β defect.**
Diagnosed read-only: clocks are already pinned (`jetson_clocks` active, all cores
`min=max=1510 MHz`; **15 W is the max power mode on this unit — no MAXN**), CPU not
contended (load 0.71) → **not DVFS**. The MPC is simply compute-marginal at 40 Hz
on the Orin Nano (~17-20 ms solve vs 25 ms budget, per
[[2026-05-22-mpc-compute-bound-jetson-profiling]]).

**`ERR_BUS_DOWN` on `/activate` — stale undervoltage, a clear_errors gap.** Before
the run_mpc attempt, `/activate` was rejected `ERR_BUS_DOWN` despite restored motor
power. Root cause (operator): the ODrives still reported their last error as
`DC_BUS_UNDERVOLTAGE` from the earlier unpowered window, tripping the firmware
`activate_allowed()` gate. The existing `/clear_errors` did not robustly clear it.
**Gap → an operator next-step:** a robust can_node-style `clear_errors`.

**Bench-method finding (recorded to memory).** `ros2 topic echo`/`hz` from a fresh
CLI repeatedly returned nothing for the 100 Hz RELIABLE `/robot_state` while the
data was flowing fine (the GUI saw it) — a Foxy discovery/QoS-match race on the
ephemeral CLI node. It briefly read as "feedback chain down." Use a dedicated
rclpy subscriber (explicit RELIABLE QoS + real spin) for read-only verification;
`ros2 service call`/`param get` are reliable. ([[reference_ros2_topic_echo_flaky_foxy]])

### Discussion — the run_mpc reframe + the architecture pivot

**The β cutover is not what failed.** It is worth stating plainly because the
symptom (run_mpc E-STOP) looks like a leg-path problem: the leg command path
(40 Hz knots → Teensy Hermite) and the feedback path (`:5556`) both work — the
solver simply cannot hold a hard 40 Hz deadline on this Jetson when a transient
steal hits. No amount of β-path work fixes that; it is the compute substrate.

**Why the hold was deprioritized rather than chased.** The interim mitigations for
the compute-marginality (reduce launch load / `record:=false`, core-pin run_mpc
with `chrt`+`taskset`) were on the table, but the operator surfaced a larger
reframe: **the MPC may be re-architected from a 40 Hz trajectory generator into a
lower-rate / event-triggered high-level re-planner**, with the Teensy + ODrive
loops as the fast tracker. That change *dissolves* this failure class (a slow
solve becomes a soft latency, not a missed control cycle) and the offload already
sets it up. Catch reactivity — the one risk — is expected to be low (balls follow
mocap-updated projectile motion, highly predictable). Given that, validating the
*current* high-freq MPC hold on β is low-value churn.

**Decision: foundation-first.** Solidify the `can_node`→Teensy port (full feature
parity + hardware-validated robustness) BEFORE any control-arch change — the
replanner rides the same Teensy leg-command substrate either way, so the port is a
prerequisite regardless, and the compute-marginality is orthogonal to it. The next
unit is a `can_node`→can-hub-Teensy **feature-parity audit** (the foundation's
definition-of-done). ([[project_mpc_replanner_direction]])

### Verification (2026-06-27)

- **Off-hardware suite:** `pytest tests/ -q` (run 2026-06-27): **1883 passed,
  1 xfailed in 467.64 s** (1880 after the /deactivate add, +3 net from the audit-fix
  abort-discrimination tests; no regressions).
- **Firmware:** `pio run` green, dec **362592** (U5b 360512 + the additive DEACTIVATE
  op; `leg_interp.cpp` untouched — the 500 Hz Hermite/setpoint/output-gate path is
  byte-identical to the validated dec-360512). Flashed to the can-bridge 2026-06-27.
- **Audit:** independent audit-reporter pass on the /deactivate diff — no BLOCKING;
  2 WARNINGs fixed pre-flash (observer abort-discrimination; activate deferred-stow
  exclusion).
- **Hardware (live, 2026-06-27):** `/deactivate` → clean six-leg controlled lower to
  STOW + IDLE, operator-confirmed "works perfectly". `run_mpc.py --pose 0,0,170` →
  connected to motor_guard, feedback received, first solve `Solve_Succeeded`, then
  the compute-marginality cascade above (CSV `temp/logs/mpc_20260627_133909.csv`,
  158 records; mean solve 11.3 ms, max 29.5 ms, p95 25.6 ms).

## Open Questions / Next

- **`can_node`→Teensy feature-parity audit (the new foundation unit).** Enumerate
  every `can_node` service/handler/behaviour vs its bridge equivalent; tag
  ported+validated / ported+unvalidated / gap. This is the definition-of-done for
  the foundation, and the prerequisite for any MPC re-architecture. Known gaps to
  fold in: the `clear_errors` robustness gap (below), the `home`/`home_motors`
  action-name mismatch (carried U3/U4), and likely-unverified hand-command /
  levelling / state-persistence services.
- **Robust `clear_errors`** — a can_node-style error clear (the stale
  `DC_BUS_UNDERVOLTAGE` → `ERR_BUS_DOWN` confusion this session is the motivating gap).
- **MPC-as-replanner study** — quantify event-triggered replan latency vs the catch
  timing budget, then decide. Post-foundation. ([[project_mpc_replanner_direction]])
- **run_mpc high-freq hold on β** — DEPRIORITIZED. The feedback chain is confirmed;
  the hold is blocked only by the compute-marginality, which the replanner dissolves.
  If ever needed before the re-arch: `record:=false` + core-pin (`chrt`+`taskset`).
- **Measure the disarm-to-output-gate latency directly** — still carried (not reached
  this session; the run_mpc hold didn't sustain long enough to measure cleanly).

### (superseded) original U5 next list

- **rung 1 — run_mpc closed-loop hold on β (FIRST TIME).** Use the full
  `jugglebot_launch.py enable_setpoint_output:=true` so motor_guard + motion_bridge
  supply the MPC's `:5556` feedback. Confirm the chain `Teensy → teensy_bridge →
  motion_bridge → motor_guard → :5556 → MPC` actually closes on β before commanding
  a hold. Then the bringup ladder: tiny → moderate moves (**the coupled-workspace-
  backstop-loss test, U4's deferred confirmation**) → trajectory → catch.
- **Measure the disarm-to-output-gate latency directly** (still the carried
  ~100–200 ms estimate; never characterised).
- **Operator next-step proposals (2026-06-26):**
  1. **A `/deactivate` service** — ✅ **DONE + hardware-validated 2026-06-27**
     (commit `dbf32c9`; see the continuation section above). Firmware TRAP_TRAJ
     lower to STOW + IDLE, mirroring activate.
  1b. **Robust `clear_errors`** (NEW gap, 2026-06-27) — a can_node-style error
     clear; the existing `/clear_errors` did not clear a stale `DC_BUS_UNDERVOLTAGE`,
     which tripped `ERR_BUS_DOWN` on `/activate`.
  2. **Group all Jugglebot services under a `/jb/` prefix** for clarity (rename
     `/home`, `/configure`, `/activate`, `/encoder_search`, etc.). A wide ripple —
     grep all consumers (GUI, orchestrator) first.
  3. **Generalise `/configure`** to set arbitrary control/input mode (TRAP_TRAJ/
     PASSTHROUGH, POSITION/VELOCITY) rather than only the fixed cold-start set.
- **`can3_tx_task`** — the plan's full reliable-TX design (large queue + pacing),
  if bursts ever exceed the deepened buffer.
- **Decommission (Phase 13)** — still deferred (gate on β operating time).
- **Carried from U3/U4:** Finding A (rare feedback freeze) root cause; Finding C
  (deterministic pre-arm sole-authority guard); the `home`/`home_motors` action
  mismatch for orchestrator-driven homing.
