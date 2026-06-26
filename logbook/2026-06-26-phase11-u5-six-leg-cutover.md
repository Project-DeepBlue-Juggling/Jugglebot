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
commits:
  - ad7b3b3
  - 2564a77
  - 4c3dbfd
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

## Open Questions / Next

- **rung 1 — run_mpc closed-loop hold on β (FIRST TIME).** Use the full
  `jugglebot_launch.py enable_setpoint_output:=true` so motor_guard + motion_bridge
  supply the MPC's `:5556` feedback. Confirm the chain `Teensy → teensy_bridge →
  motion_bridge → motor_guard → :5556 → MPC` actually closes on β before commanding
  a hold. Then the bringup ladder: tiny → moderate moves (**the coupled-workspace-
  backstop-loss test, U4's deferred confirmation**) → trajectory → catch.
- **Measure the disarm-to-output-gate latency directly** (still the carried
  ~100–200 ms estimate; never characterised).
- **Operator next-step proposals (2026-06-26):**
  1. **A `/deactivate` service** — a controlled lower/stow from Active. The gap
     was felt at shutdown (no β-path lower path; the operator e-stopped + lowered
     by hand). Likely a firmware TRAP_TRAJ move to a stow pose, mirroring activate.
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
