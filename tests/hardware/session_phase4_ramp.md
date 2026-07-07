# Hardware Session — MVP Phase 4: Limit Ramp-up + Lean A/B (multiple short sessions)

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
NOT edit the YAML until the step passes. Example (jerk is the binding constraint;
raise it first):

```bash
# raise only jerk; 0 keeps the other two unchanged (each clamped to its YAML ceiling)
ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits \
  "{leg_vel_limit_mmps: 0.0, leg_acc_limit_mmps2: 0.0, leg_jerk_limit_mmps3: 12000.0}"
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
- Any oscillation, audible snap, tracking error > 0.1 rev, gate violation, or
  E-STOP.

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
