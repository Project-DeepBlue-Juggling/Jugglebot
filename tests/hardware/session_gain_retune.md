# Hardware Session — Leg-Gain Retune (fast-motion tier), **on-robot Phase B** — root fix for the 2026-07-10 ~6 Hz stutter

> **⚡ Superseded in part, 2026-07-15 (ARMING CONTRACT)**: arming is now
> **automatic on ACTIVE entry** (`ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md`;
> see the banner in `mvp_bench_runbook.md`). Manual `set_setpoint_output true`
> steps below run only under `auto_arm:=false`; disarm-before-deactivate is now
> enforced in-process by the bridge (A3).


> # ⚠️ SUPERSEDED 2026-07-13 — DO NOT RUN THE GAIN SWEEP IN THIS DOCUMENT
>
> **There is no gain to retune, and no winner to transfer.** The bench "winner" this session
> existed to land was selected on a **~14× inflated error metric** (`bench_leg_sysid.py:2548`
> printed milli-**revolutions** under an `mm` label). Corrected, the leg already tracks to
> **0.054 mm median / 0.192 mm worst at the catch/throw instant** — against an operator spec of
> **±1 mm**, i.e. ~5× inside. The measured cascade at production gains is healthy
> (`ω_v` ≈ 20 Hz / `ω_p` = 6.4 Hz, ratio ≈ 3.2).
>
> **Production `40 / 0.20 / 0.32` STANDS. `MAX_LEAD` stays 0.10** (already shipped — no firmware
> change). **Do not** run the `pos_gain` {25, 40, 55} × `vel_gain` {0.20, 0.35} sweep below, and
> **do not** persist any gains to YAML.
>
> **What to run instead — the ONE test still worth doing, ONCE:** the **loaded S4 replay as a v3
> firmware regression test, at production gains**. The 2026-07-10 ~6 Hz ring was diagnosed
> **structural, not a tuning deficit** — `MAX_LEAD = 0.15` gave `pos_gain × lead = 6.0 rev/s`
> (*above* the 4.0 rev/s `vel_limit`) and `vel_ff` was discontinuously **zeroed at clamp engage**.
> **Both were fixed in v3 firmware**, and the bench under v3 cannot reproduce the ring in any
> regime. The only open question is whether v3 fixed it on the **loaded, coupled, 6-leg** robot.
>
> - **PASS** (no 5.9–6.1 Hz / ~12.3 Hz peak, no guard latch) ⇒ the S4 chapter closes. No gain work.
> - **FAIL** ⇒ that is a **structural/regulation** finding. Investigate the interp/clamp path and
>   inter-leg coupling. A gain change is a *last* resort, and the honest knob would be **`vel_gain`
>   up** (raises the inner loop and the cascade ratio) — **not `pos_gain` up**, which *lowers* the
>   ratio and marches the outer loop into the 15–19 Hz resonance.
>
> Use the safety mechanics below (arming, abort criteria, `/recover`) — they are still correct.
> Ignore every instruction to select, derate, or persist a gain.
> See `logbook/2026-07-13-leg-plant-id-and-the-units-bug.md`.

**Plan**: `plans/active/leg-gain-tuning-methodology.md` § "Fast-motion tier (Level-2f)"
(read it before starting — the tier is now a **two-stage** methodology)
**Runbook**: `tests/hardware/mvp_bench_runbook.md` § S4b (this session gates further S4 ramping)

> ## Two-stage structure — READ FIRST (2026-07-11 rewrite)
>
> The fast-motion tier is now **bench-first**. This document is the **on-robot Phase B**,
> not the whole retune:
>
> - **Phase A — bench leg (aggressive)** — `plans/active/leg-gain-tuning-methodology.md`
>   § "STAGE 1". System-ID + an escalate-until-unstable gain ladder + the loop-vs-structure
>   discriminant, run on the acceptable-loss 7th (bench) leg. **Do Phase A first.** It
>   produces the *winning bench triple* and a definitive loop-vs-structure verdict.
> - **Phase B — Jugglebot transfer (this document)** — take the bench winner, apply the
>   **one-notch derate** (methodology § "STAGE 2, Derating rule"), and run the **reduced
>   verify pass** below (§ "Phase B0"). The full on-robot 3-point sweep (§ "Phase B1/B2")
>   is now the **FALLBACK** — run it only if Phase A was skipped or came back inconclusive.
>
> If the bench Phase A returned **"structural, not the servo loop"**, gains are not the
> lever: do **not** run the on-robot sweep — hand off to the interpolant/mechanical path
> (methodology § "Stage 1c").
**Forensics**: the 2026-07-10 four-agent stutter/latch synthesis (Agent 1 Rank-1 =
retune the leg velocity loop for motion damping; Agent 3 Rank-1 = position lag < the new
MAX_LEAD 0.10 rev with 2× margin, watch braking-current oscillation). *A committed logbook
entry for that forensics does not exist yet — the orchestrator will create it; link it here
once it lands.*
**Goal**: separate the ~6 Hz limit cycle's cause (control-loop pole vs. structural
resonance) with a `pos_gain` sweep, then, if it is the control loop, retune the leg
velocity loop so a fast vertical stroke tracks cleanly — no braking-current oscillation,
position lag < 0.05 rev, no lead-clamp engagement, no MAX_DEVIATION latch. Persist the
winning gains to YAML as the new **fast-motion tier**, without regressing the Level-1
HOLD tier.

---

## Why this session (root cause, not appeal-to-plan)

The 2026-07-10 battery latched `MAX_DEVIATION` on a fast vertical up-stroke (z 170→250 at
156/660/10500). Forensics established:

- The stutter is a **coherent ~6 Hz limit cycle** (Lomb-Scargle 5.9–6.1 Hz across all six
  legs), **speed-independent** in frequency — which kills cogging / velocity-clamp cycling
  and points at either a control-loop pole or a structural resonance.
- The frequency **coincides with the ODrive position-loop bandwidth** `pos_gain/(2π)` =
  40/6.283 = **6.37 Hz**. The current gains **40 / 0.20 / 0.32** were tuned only for quiet
  HOLD at a single low-limit pose (the Level-1 tier in the methodology doc), never for
  fast-motion tracking. The velocity loop's damping term (`vel_gain 0.20`) is too low to
  damp the position loop under aggressive drive.
- The latch is a **symptom** of the stutter: during the 6 Hz stalls/reversals the smooth
  Teensy-interpolated command keeps advancing while the encoder stalls, and the accumulated
  command-vs-encoder lead runs past the 0.5-rev guard. Fixing the servo removes the trigger.
- **The one thing this bag could not decide**: control-loop pole (retune fixes it) vs.
  structural/drivetrain resonance at ~6 Hz (retune will not fix it — a different path is
  needed). Both live at 6.4 Hz and are indistinguishable at a single gain setting. **This
  session's `pos_gain` sweep is the discriminator.**

## What changed since the latch bag (preconditions in code)

- **can-bridge firmware v3 flashed 2026-07-10.** It carries the guard-deviation telemetry
  this session's observability is built on, and the lowered lead clamp:
  - `MAX_LEAD_REV = 0.10` (`canbridge_config.h:139`) — lowered from 0.15 on 2026-07-10 so
    `pos_gain × MAX_LEAD` stays under the leg vel_limit (was the "0.15 → 6 rev/s sprint" trap).
  - `MAX_DEVIATION_REV = 0.5` (`canbridge_config.h:147`), `FAULT_TASK_HZ = 10` — unchanged.
  - New `/link_status` v3 KeyValue fields (10 Hz, `teensy_bridge_node.py:1900-1909` from
    `udp_protocol.h:195-196`): `live_deviation` (per-leg u0−encoder, comma-separated rev, the
    live MAX_DEVIATION quantity), `lead_clamp_mask` (bit *i* = leg *i*'s lead clamp engaged on
    the last 500 Hz tick), and the frozen latch snapshot `max_dev_leg` / `max_dev_value` /
    `max_dev_u0` / `max_dev_enc` (which leg crossed + the dev/u0/encoder at the trip).
- **`/recover` (FIX 3) available** — one-call recovery from a latched guard: it reseeds
  `trajectory_node`'s hold at the *measured* encoder, verifies the streamed u0 has converged
  within tolerance of every leg, then fires `CLEAR_ERRORS`, so the clear cannot instantly
  re-latch. It **refuses** (leaving the guard latched, loudly) if the reseed is unavailable
  or u0 cannot converge. This is why a latch this session is recoverable without a
  deactivate/retract dance (the 2026-07-10 catch-22).
- **Guard fault-state edges are now logged** to `teensy_bridge_node` stdout (a latch and its
  clear land in the launch window, not only inferable from the 10 Hz bag).

## How leg gains are actually applied (READ — this shapes the whole loop)

**There is no runtime ROS service that sets *leg* gains live.** (`set_hand_gains` exists for
axis 6 only.) Leg gains are applied by `teensy_bridge_node._run_configure()` — it calls the
firmware `SET_POS_GAIN` (0x1A) / `SET_VEL_GAINS` (0x1B) RPCs from the **generated** config
(`hw.ODRIVE_LEG_POS_GAINS` / `..._VEL_GAINS` / `..._VEL_INT_GAINS`), which come from
`config/hardware_config.yaml`. `_run_configure` runs:
- automatically at **`/home` completion**,
- on the standalone **`/configure`** service, and
- folded **after activate** in the orchestrator's `activate_or_deactivate` path
  (`teensy_bridge_node.py:3222`). **The bare `/activate` Trigger does NOT fold configure** —
  which is one more reason to arm via `/orchestrator_command` (Sharp Edge #4), because
  `/orchestrator_command activate` is the path that both moves to pose AND re-applies the gains.

**The firmware RPCs do not persist** — `rpc.cpp:167-174` just forwards a CAN frame to the
ODrive; there is no `save_configuration`. So gains are **RAM-only on the ODrive (session-only)**
and are **re-applied from generated config at every `_run_configure`**. A power-cycle or reflash
loses any hand-poked value; the YAML→generated config is the single source of truth.

**Consequence — the per-sweep-point gain-change loop is:**
edit YAML (all six legs uniform) → `python config/generate_config.py` →
`colcon build --packages-select jugglebot` → `source install/setup.bash` → **relaunch** →
the new gains apply at the next `_run_configure` (the `/orchestrator_command activate` below).
- **No mechanical re-home is needed between points**: `is_homed` persists across a bridge
  relaunch (cold-start read from the Platform Teensy), so the orchestrator comes up IDLE and
  `activate` goes straight to the ACTIVE pose.
- **UNVERIFIED**: there is no numeric read-back of the *applied* leg-gain values — the bridge
  logs only `configure: applying gains/limits/PASSTHROUGH on axes {axes}` (names the axes, not
  the numbers). Treat a successful configure + that log line as "gains applied"; the real
  confirmation is the behavioural change in the stroke. If a per-leg applied-gain echo is added
  later, verify against it.
- **UNVERIFIED**: whether a `colcon build` is strictly required vs. relaunch-only depends on
  whether the workspace was built `--symlink-install`. The install copy of `hardware_config.py`
  is a *separate* file from the source copy, so **assume the build is required** (the
  methodology doc's Level-1 procedure also prescribes it). A relaunch alone will silently run
  the *old* gains if the build is skipped.

---

## Roles & safety framing

- **The operator (Harrison) runs every robot-actuating command below.** Claude prepares the
  exact commands + PASS/ABORT and verifies read-only (`ros2 topic echo /link_status`,
  `/diagnose`, the bridge log, the offline ring-frequency analysis). Claude does not run
  motion, arming, or the gain-apply build/relaunch.
- **Physical-intuition pushback is load-bearing.** *If your physical intuition disagrees with
  a framing here — especially "vertical is worst because all six legs move in phase" or "the
  6 Hz is the control loop" — say so before proceeding.* At session start and at every framing
  pivot (each decision gate below). A surprise is a stop-and-discuss, not a push-through.
- **This is the fastest/hardest the platform will move this bringup** at the raised
  156/660/10500 limits, and a wrong gain can make it *ring harder*. Any ABORT criterion ⇒ cut
  power / trigger the guard, recover, then **revert to the last-good gains** before retrying.
- **Disarm before any control-mode change away from a streaming mode** (Sharp Edge #1) and
  **disarm before deactivate** (firmware-enforced).

## Preconditions

- can-bridge **firmware v3 flashed** (2026-07-10) — confirm the `/link_status` v3 keys
  (`live_deviation`, `lead_clamp_mask`, `max_dev_leg`) are present before starting; if they
  are absent the wrong firmware is running and the whole observability plan is void.
- `run_mpc.py` is **NOT** running (`trajectory_node`'s 40 Hz emitter is the sole :5557 binder).
- **Rosbag recording ON** — `/link_status`, `/robot_state`, `/trajectory/status`,
  `/trajectory/diagnostics` in the record list (`/diagnose` and the offline ring-frequency
  analysis read them back). One bag per sweep point, named for the point (see below).
- **Know the last-good YAML gains** (`config/hardware_config.yaml` `jugglebot_odrive_defaults:`
  `leg_pos_gains` / `leg_vel_gains` / `leg_vel_int_gains`, currently uniform
  **40.0 / 0.20 / 0.32**) so an ABORT reverts cleanly.
- Session leg limits will be held **fixed at 156 / 660 / 10 500** for every point (this is a
  gain sweep, not a limit ramp) — a relaunch reverts session limits to the YAML defaults, so
  they are re-set each point via `set_limits`.

---

## Phase B0 — reduced verify (DEFAULT, after the bench Phase A)

If the bench Phase A converged (methodology § "STAGE 1"), you already have a winning bench
triple and a confirmed loop-vs-structure verdict. On the real robot you **verify the derated
winner**, you do not re-sweep. Apply the **one-notch derate** (methodology § "STAGE 2,
Derating rule": reduce `pos_gain` ~10–15 % **or** raise `vel_gain` ~15–20 %, keeping the
125:1 `vel_int` ratio), then run:

1. **Ring-suppression stroke (×1)** — the fixed stroke below (z 170 → 250 → 170 at
   156/660/10500) at the derated triple. PASS = the four PASS criteria (no braking-current
   oscillation, max `live_deviation` < 0.05 rev, `lead_clamp_mask` 0, no ~6 Hz buzz, no
   `MAX_DEVIATION` latch).
2. **Extreme-pose HOLD battery** — Level-1 moves 6 & 7 at (0,−100,200) and (100,100,200);
   per-leg hold stdev within 1.5× of the quietest leg (PASS criterion 4).
3. **If either fails** — apply the *other* derate knob one notch and re-run (a single A/B),
   **not** the full sweep. If neither can be jointly cleared, that is the Level-3 /
   gain-scheduling trigger (methodology Open questions) — do **not** push the ring regime
   harder on the real legs.

Persist the winner (§ "Persist the winning gains to YAML") as the **robot (S2)** row.

## Phase B1/B2 — the full on-robot sweep (FALLBACK — only if bench Phase A was skipped/inconclusive)

Run the sweep matrix below **only** when the bench Phase A did not run or came back
ambiguous. It is the original single-stage on-robot discriminate-then-damp protocol, kept
as the fallback so the on-robot session is self-sufficient without the bench.

Fixed test stroke every point: **z 170 → 250 → 170** (the up-stroke is the one that latched),
at fixed session limits **vel 156 mm/s, acc 660 mm/s², jerk 10 500 mm/s³**, `duration_s: 0.0`
(minimal feasible). Because the planner's minimal duration is **gain-independent** at fixed
limits, the *commanded* stroke is byte-identical across all points — the clean basis for
comparing the ring across gains.

`vel_int_gain` follows the methodology's `pos_gain : vel_int_gain ≈ 125 : 1` ratio rule, so it
is pinned per `pos_gain` and is **not** an independent axis:

| `pos_gain` | pos-loop BW `pos_gain/(2π)` | `vel_int_gain` (=pos/125) | predicted ring freq **if control-loop** |
|---|---|---|---|
| 25 | 3.98 Hz | 0.20 | ~4.0 Hz |
| 40 (baseline) | 6.37 Hz | 0.32 | ~6.4 Hz |
| 55 | 8.75 Hz | 0.44 | ~8.8 Hz |

`vel_gain` (the independent proportional-damping knob) is A/B'd **{0.20, 0.35}**.

### Phase B1 — discriminate control-loop vs. structural (fallback; 3 points, `vel_gain` fixed at 0.20)

Run the fixed stroke at `(pos_gain, vel_gain, vel_int_gain)` =
**(25, 0.20, 0.20)**, **(40, 0.20, 0.32)** [current baseline — the 2026-07-10 bag already
covers this; re-run only if you want a same-day control], **(55, 0.20, 0.44)**.

Record the ring frequency (offline, per point — see observability). **Decision gate:**

- **Ring frequency TRACKS `pos_gain`** (≈ 4.0 / 6.4 / 8.8 Hz as the table predicts) ⇒ it is
  the **control loop** (an under-damped position pole ringing at its own bandwidth). Proceed to
  **Phase B2** to add velocity-loop damping.
- **Ring frequency stays FIXED at ~6 Hz** regardless of `pos_gain` ⇒ it is a **structural /
  drivetrain resonance**. **Gains are not the lever — STOP the sweep.** Revert to the
  last-good gains, log the finding, and hand off to the *different* path: the forensics'
  interpolant / lead-clamp findings (the vel_ff-on-clamp fix — landed in the v3 firmware —
  and the C2/quintic interpolant polish) and a mechanical resonance investigation. Do
  **not** persist any gain change from this session.

### Phase B2 — damp the loop (fallback; only if Phase B1 said "control loop")

At the `pos_gain` that best trades **ring suppression vs. tracking lag** (higher `pos_gain` =
less lag but higher-frequency ring; start from **40** and only move to 55 if 40's lag margin is
tight), A/B `vel_gain` **0.20 → 0.35** (`vel_int` unchanged from Phase B1's per-`pos_gain` value).
If neither `vel_gain` clears the criteria, interpolate (e.g. 0.28) or step `pos_gain` and repeat.

**Winning cell** = lowest ring amplitude AND position lag < 0.05 rev with margin AND
`lead_clamp_mask` stays 0 AND the stroke sounds/looks clean — **and** it must not regress the
Level-1 HOLD tier (see PASS criteria).

---

## Per-point protocol (exact commands)

### 1 — Apply the point's gains (operator; requires build + relaunch)

Edit `config/hardware_config.yaml` `jugglebot_odrive_defaults:` — set **all six** entries
uniform to the point's triple, e.g. for (55, 0.20, 0.44):

```yaml
  leg_pos_gains:     [55.0, 55.0, 55.0, 55.0, 55.0, 55.0]
  leg_vel_gains:     [0.20, 0.20, 0.20, 0.20, 0.20, 0.20]
  leg_vel_int_gains: [0.44, 0.44, 0.44, 0.44, 0.44, 0.44]
```

```bash
python config/generate_config.py
cd ros_ws && colcon build --packages-select jugglebot && source install/setup.bash
# relaunch jugglebot (record:=true); is_homed persists → orchestrator comes up IDLE
ros2 launch jugglebot jugglebot_launch.py record:=true
```

### 2 — Arm via the orchestrator (Sharp Edge #4/#5 — NOT the bare /activate)

```bash
# activate: moves to ACTIVE pose AND folds _run_configure → applies THIS point's gains + PASSTHROUGH
ros2 topic pub -t 3 -r 2 /orchestrator_command std_msgs/msg/String "data: 'activate'"
# confirm in the launch log: "configure: applying gains/limits/PASSTHROUGH on axes [...]"
# arm the 40 Hz downlink
ros2 service call /set_setpoint_output std_srvs/srv/SetBool "{data: true}"
# enter TRAJECTORY (repeat-publish; verify before any move)
ros2 topic pub -t 3 -r 2 /orchestrator_command std_msgs/msg/String "data: 'trajectory'"
```

Confirm `/control_mode_topic` reads `TRAJECTORY` and verify a **bumpless hold** (probe, as in
S1) before commanding motion. `live_deviation` on `/link_status` should sit near 0 at hold.

### 3 — Set the fixed session limits

```bash
ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits \
  "{leg_vel_limit_mmps: 156.0, leg_acc_limit_mmps2: 660.0, leg_jerk_limit_mmps3: 10500.0}"
```

Read back the echoed `applied_*` values (each is ceiling-clamped) before any motion.

### 4 — Watch `/link_status` live (Claude read-only, in a second terminal)

```bash
ros2 topic echo /link_status
```

Watch these KeyValue keys through the stroke:
- **`fault_state`** — must stay `NONE`. Any `MAX_DEVIATION` / `MPC_STALE` ⇒ ABORT.
- **`lead_clamp_mask`** — must stay `0`. A nonzero bit means that leg's output hit the 0.10-rev
  lead clamp — the bang-bang mechanism that manufactures the ring. Persistent/rising mask ⇒ ABORT.
- **`live_deviation`** — per-leg u0−encoder (rev). Watch the max magnitude: PASS wants it
  **< 0.05 rev** through the whole stroke (2× margin below the 0.10 clamp, 10× below the 0.5 guard).
  Approaching 0.10 is the pre-latch signature.
- **Audible** — listen for the ~6 Hz surge/collapse buzz. A clean stroke is a single smooth
  whoosh; the stutter is an audible motor "grinding/ringing" as it surges and brakes.

### 5 — Run the fixed stroke (operator)

```bash
# up-stroke (the one that latched)
ros2 service call /trajectory/go_to_pose jugglebot_interfaces/srv/GoToPose \
  "{pose: {position: {x: 0.0, y: 0.0, z: 250.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, duration_s: 0.0, lean_gain: 0.0}"
# wait for it to finish (go_to_pose returns BUSY if a move is in flight), then return
ros2 service call /trajectory/go_to_pose jugglebot_interfaces/srv/GoToPose \
  "{pose: {position: {x: 0.0, y: 0.0, z: 170.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, duration_s: 0.0, lean_gain: 0.0}"
```

`lean_gain: 0.0` forces lean OFF explicitly (isolate gains from lean). Both should return
`accepted=true code=OK`.

### 6 — Review (Claude read-only)

- `/diagnose --latest` — the Trajectory-Moves block: realized leg peaks, tracking error, and
  the braking-current trace on the up-stroke (the forensics signature is negative iq at each
  velocity collapse). PASS wants **no braking-current oscillation** and tracking error < 0.05 rev.
- **Ring frequency** — offline analysis on the rosbag `robot_state` leg pos/vel over the
  up-stroke (the forensics Lomb-Scargle / FFT recipe). This is the Phase-A discriminant.
  **UNVERIFIED**: whether `/diagnose` surfaces a per-move ring frequency directly — assume it
  does not and compute it offline per point.
- Record for the point: `(pos_gain, vel_gain, vel_int)`, ring freq + amplitude, max
  `live_deviation`, `lead_clamp_mask` peak, braking-current verdict, rosbag path.

---

## PASS criteria (a point is a candidate winner iff ALL hold)

1. **No braking-current oscillation** on the up-stroke (`/diagnose` iq trace: no negative-iq
   surge/collapse cycles; forensics saw −3.8 A/+3.6 A cycling at baseline).
2. **Position lag < 0.05 rev** through the stroke (max `live_deviation` < 0.05 rev — 2× margin
   below MAX_LEAD 0.10, so the lead clamp never engages) and **`lead_clamp_mask` stays 0**.
3. **Stroke completes smoothly** — no audible ~6 Hz buzz, no visible oscillation, `fault_state`
   stays `NONE`, `setpoints_rejected` delta 0, both moves `accepted=true`.
4. **Does not regress the Level-1 HOLD tier** — before persisting a winner, run the
   methodology's extreme-pose HOLD battery (moves 6 & 7 at (0,−100,200) and (100,100,200)) and
   confirm per-leg hold stdev stays within 1.5× of the quietest leg. *This is the
   non-negotiable cross-tier check — a gain "converged" for fast motion that reintroduces the
   pose-dependent hold twitch is not a winner. The methodology doc encodes exactly this failure.*

## ABORT criteria (immediately) + revert-to-last-good

**ABORT on:** any `MAX_DEVIATION` / `MPC_STALE` latch, any oscillation / audible snap /
tracking error > 0.1 rev, `lead_clamp_mask` persistently nonzero with rising `live_deviation`,
or any unexplained bus fault.

**Recovery + revert:**
1. If a guard latched: `ros2 service call /recover std_srvs/srv/Trigger` (one-call: reseeds
   hold at measured, verifies u0 convergence, then clears — will not instantly re-latch). If
   `/recover` refuses, fall back to `set_setpoint_output false` → `/clear_errors` → re-arm.
2. Disarm (`set_setpoint_output false`) → `/orchestrator_command deactivate` (profiled stow).
3. **Revert the gains** to the last-good triple (the baseline 40/0.20/0.32, or the last point
   that PASSed): edit YAML → `generate_config` → `colcon build` → `source` → relaunch. A
   relaunch also reverts session limits to YAML — a clean full reset.

---

## Persist the winning gains to YAML

Once a cell PASSes all four criteria (including the extreme-pose HOLD re-check):

1. Edit `config/hardware_config.yaml` `jugglebot_odrive_defaults:` — set all six
   `leg_pos_gains` / `leg_vel_gains` / `leg_vel_int_gains` uniform to the winning triple, and
   update the comment block above them (record the fast-motion tier, the ring-amplitude
   before/after, and the lag margin).
2. `python config/generate_config.py`
3. Stage the regenerated artifacts (`config/generated/*`, `ros_ws/src/jugglebot/jugglebot/hardware_config.py`).
4. `pytest tests/ -q` (full suite is the pre-commit gate).
5. Commit — cite the `/diagnose` ring-amplitude + lag numbers and the test triple
   `(date, exact pytest command, result)` in the message; add a `Logbook-Entry:` trailer.
6. Register the winner in `plans/active/leg-gain-tuning-methodology.md`
   § "Fast-motion tier" → "Registered fast-motion gains" as the **robot (S2)** row (the
   derated on-robot-verified triple + the measured damping ratio); the bench winner is the
   **bench (S1)** row.
7. If the **bench Phase A** discriminant said **structural**, persist NOTHING — commit only
   the logbook finding and the hand-off to the interpolant/mechanical path.

---

## Link to the methodology

This session is the **on-robot Phase B** (Jugglebot transfer) of the two-stage
**fast-motion tier (Level-2f)** in `plans/active/leg-gain-tuning-methodology.md`; the
aggressive bench work (system-ID + escalate-until-unstable ladder + loop-vs-structure
discriminant) is **Phase A** there (§ "STAGE 1"). That tier's damping-ratio target
(closed-loop ζ ≥ 0.7, ≤ ~5 % overshoot at the pos-loop bandwidth) is the acceptance target
behind PASS criteria 1–3 here; the Level-1 HOLD battery behind PASS criterion 4 is the
cross-tier guard that keeps the two tiers mutually consistent. Read that section before
starting and update the **robot (S2)** tier row when a winner lands.
