# Hardware Session — MVP Phase 4: Limit Ramp-up + Lean A/B (multiple short sessions)

> **⚡ S4 CLOSED 2026-07-17** — the working point (1000, 5000, 30000) +
> `lean_gain 0.6` is persisted to YAML (see the runbook S4 Result block and
> `logbook/2026-07-17-s4-closed-working-point-persisted.md`). This protocol is
> retained for future re-ramps — which now START from the hot working point on
> a bare relaunch; `set_limits` down first for a faithful gentle re-run.
> Stale numbers below, superseded: battery geometry is z 170→220→170, x/y ±150,
> tilt rx ±10° since 2026-07-16; the shipped YAML `lean_gain` is **0.6** (not
> 0.0) and the battery's `--lean-gain` defaults to **−1.0 = defer-to-config**
> since 2026-07-17 — pass an explicit `0.0` for the unshaped arm, and note the
> unshaped ±150 traverse latches the guard at the hot working point
> (`logbook/2026-07-17-wobble-latch-unshaped-traverse.md`).

> **⚡ Superseded in part, 2026-07-15 (ARMING CONTRACT)**: arming is now
> **automatic on ACTIVE entry** (`ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md`;
> see the banner in `mvp_bench_runbook.md`). Manual `set_setpoint_output true`
> steps below run only under `auto_arm:=false`; disarm-before-deactivate is now
> enforced in-process by the bridge (A3).
>
> **⚡ MAX_DEVIATION guard + ODrive vel_limit raised, 2026-07-16**: the firmware
> guard is now 1.0 rev (was 0.5 — flashed + validated the same day: bag
> `2026-07-16_17-38-15` ramped vel 100→200→280 mm/s with zero latches) and the
> ODrive leg `vel_limit` is now **12.0 rev/s** (4.0 → 6.0 midday → 12.0 in the
> evening update — config, pushed to the drives at the first homing of a
> session with a **rebuilt** ROS2 install; a bare relaunch of a stale install
> keeps the old value). The `set_limits` ceilings are opened to
> **5000/5000/200000** (administrative — see the ⚡ banner + evening update in
> `mvp_bench_runbook.md` for the real trackable envelope, ~529 mm/s
> clamp-engaged). Step 2's ABORT criterion below is recalibrated accordingly.


**Plan**: `plans/active/mvp-trajectory-bringup.md` § Phase 4
**Logbook**: `logbook/2026-07-08-mvp-phase4-shaping-ramp-tooling.md`
**Goal**: raise the session leg vel/acc/jerk limits toward the levels Phase 6
publishes as required for catching, one small step per session; run one lean
A/B (gain 0.0 vs 0.3). The always-on smoothness mechanism is the feasibility
gate's duration stretch — the ramp raises the ceilings that stretch works within.

This session validates the **software already merged** on branch
`mvp-trajectory-bringup` (the `GoToPose.lean_gain` override, the realized-peak
diagnostics, the `/diagnose` per-move summary, and `traj_ramp_battery.py`). It is
a *per-step* protocol — repeat it once per limit bump across several short
sessions; the code does not change between steps, only the YAML session limits do.

---

## Roles & safety framing

- **The operator (Harrison) runs every robot-actuating command below.** The
  implementing session prepared the exact commands + PASS/ABORT criteria and the
  operator-run battery script; Claude verifies read-only (`/diagnose`, the probe,
  the bridge log).
- **If your physical intuition disagrees with any framing here, that is
  load-bearing signal — say so before proceeding.** Raising a limit is the one
  moment the platform can move *faster/harder* than it has this whole bringup — a
  surprise is a stop-and-discuss, not a push-through.
- E-STOP is always available. Any ABORT criterion ⇒ cut power / trigger the guard,
  then **revert the YAML session limits to the last-good values** before retrying.
- **Disarm before any control-mode change away from a streaming mode** (leaving a
  streaming mode while armed drops the stream → `MPC_STALE` E-STOP within 250 ms).

## Preconditions

- Platform armed and holding in **TRAJECTORY** mode via the new path (the Phase-1
  arm sequence: home → activate → mode STANDBY → `set_setpoint_output true`, then
  set control mode `trajectory`). Verify a bumpless hold first with the probe.
- `run_mpc.py` is **NOT** running (the trajectory emitter is the sole :5557 binder).
- Rosbag recording on (`ros2 launch … record:=true`) — `/trajectory/diagnostics`
  and `/trajectory/status` are in the record list; `/diagnose` reads them back.
- **Know the last-good YAML session limits** (`config/hardware_config.yaml`
  `trajectory_op:` `leg_*_limit_*`) so an ABORT reverts cleanly.

---

## Per-step ramp protocol (repeat once per limit bump)

### Step 1 — Raise ONE limit ~1.5× (in-session, not YAML yet)

Ramp a single limit at runtime so a bad value is one service call to undo — do
NOT edit the YAML until the step passes. **The step order and values come from the
runbook's ladder** (`tests/hardware/mvp_bench_runbook.md` § S4: jerk 10 500 → acc 520
→ acc 660 → vel 130 → vel 156). Example, step 1:

```bash
# raise only jerk; 0 keeps the other two unchanged (each clamped to its YAML ceiling)
ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits \
  "{leg_vel_limit_mmps: 0.0, leg_acc_limit_mmps2: 0.0, leg_jerk_limit_mmps3: 10500.0}"
```

The service echoes the applied (ceiling-clamped) limits. A request above the YAML
hard ceiling is clamped, never applied raw — the ceiling is the physical envelope
the ramp moves *within*.

### Step 2 — Run the move battery (operator-run script)

```bash
python3 tests/hardware/traj_ramp_battery.py --lean-gain 0.0
```

The battery fires, at the current session limits: z 170→190→170, x ±20, y ±20,
tilt rx ±3°, then one deliberately-infeasible `duration_s: 0.05` request. It
prints each move's `accepted / code / planned_duration_s / min_duration_s`.

**PASS** (per move):
- Subjectively smooth — no audible snap, no visible oscillation.
- The infeasible request comes back `accepted=false code=TOO_FAST` with a
  `min_duration_s` > 0 and **moves nothing**.
- No pump rejects in the `teensy_bridge_node` log, no firmware fault, no E-STOP.

**ABORT** (immediately, then revert Step 1 via `set_limits` to last-good):
- Any oscillation, audible snap, gate violation, or E-STOP.
- Tracking error > 0.1 rev **at a hold** — that criterion's original meaning.
  **During the move itself** (operator-confirmed 2026-07-16, see the ABORT
  recalibration note in `mvp_bench_runbook.md` § S4): peak `|live_deviation|`
  must stay under ~0.6 rev (60 % of the 1.0 rev guard) and must collapse back
  under 0.1 rev once the platform settles at arrival — a brief 0.09–0.11 rev
  onset blip that rides the MAX_LEAD 0.10 rev clamp ceiling is expected, not
  an ABORT. Any **MAX_DEVIATION latch** is itself an ABORT regardless of the
  peak value — stop the battery, keep the bag, and review with the firmware's
  per-leg fault log line (`fault_state=MAX_DEVIATION (leg N, dev=... rev at
  trip) live_dev=[...]`).
- A lead-clamp mask that has not released by arrival (still engaged at the
  following hold) is also an ABORT — mask duration alone (e.g. ">0.5 s") is
  not a signal, since mask stretches lengthen legitimately with velocity.

### Step 3 — `/diagnose` review

```
/diagnose --latest
```

Read the **Trajectory Moves** block (the Phase-4 per-move summary):
- Realized leg peaks per move + **% of the session limit used** (headroom =
  100 − used). The raised limit should show the realized peak climbing toward it
  as the moves get more aggressive; keep comfortable headroom on the *other* two.
- Cross-check tracking error (legs/tracking plots) and the hold-phase quiescence.

**Operator PASS** ⇒ persist the bump: edit `config/hardware_config.yaml`
`trajectory_op:` `leg_jerk_limit_mmps3` (etc.) to the validated value →
`python config/generate_config.py` → stage the regenerated artifacts →
`pytest tests/ -q` → commit **between sessions** (one commit per validated bump,
with the `/diagnose` numbers in the message).

**ABORT** ⇒ revert the in-session `set_limits` to the last-good value; leave the
YAML unchanged.

### Step 4 — Lean A/B (once, not every step)

Run the **identical** battery twice — same limits, `--lean-gain 0.0` then
`--lean-gain 0.3` — focusing on the xy moves (that is where lateral acceleration,
hence lean, is largest):

```bash
python3 tests/hardware/traj_ramp_battery.py --lean-gain 0.0
python3 tests/hardware/traj_ramp_battery.py --lean-gain 0.3
```

`/diagnose --compare` the two sessions (the per-move summary tags each move with
its `lean_gain`). **Keep lean only if** measured leg jerk drops AND the motion
looks/sounds calmer at gain 0.3. If kept, set `trajectory_op: lean_gain: 0.3` in
YAML (regenerate + commit); if not, leave it `0.0` (the default) and record the
null result in the logbook.

> **Expect the gain-0.3 arm's moves to run visibly LONGER than the gain-0.0 arm's.**
> The gate sizes the added tilt, so a shaped lateral move costs ~**1.45×** the
> unshaped minimum (x±20 @ gain 0.3: ~0.59 s vs ~0.41 s; y±20 ~1.6×; pure-z moves
> ~1×). This is correct, not a fault — the battery now sleeps
> `max(--settle-s, planned_duration_s + 0.5)` per move and prints each
> `planned_duration_s`, so you will see the unequal durations directly. Do not read
> the longer shaped durations as a regression.

> Note on lean at the boundaries: the lean tilt vanishes in *position* at each
> move's ends (quintic boundary accel is zero) but carries a small tilt-*rate*
> transient at **both** ends (the base quintic's boundary jerk is nonzero) — the
> install seam (`t=0`, hold→shaped) and the segment→hold seam (`t=T`). It is
> position-continuous but velocity/accel-**stepped**; the feasibility gate measures
> the step as leg velocity/acceleration and bounds it, and every shaped frame is
> pump-accepted (the pump gates position steps, not velocity). But if the A/B shows
> the transient as a visible/audible tick at move start/end, that is the signal to
> stop and reconsider a windowed lean (deferred). Report it rather than pushing
> through.

---

## Exit criteria

Session limits at or above the Phase-6-published catch requirements (expected
order: vel ~150–250 mm/s, acc ~1500–3000 mm/s²; Phase 6 provides the real
numbers), each bump validated by a clean battery + `/diagnose` review and
persisted to YAML with the numbers in the commit message. The lean A/B resolved
(kept with `lean_gain: 0.3`, or rejected with the null result logged). Record each
step's `/diagnose` per-move peaks + the rosbag path in the logbook Verification
section.
