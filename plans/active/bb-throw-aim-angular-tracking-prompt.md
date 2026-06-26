# Next-session prompt — (A) Throw-aim platform angular-tracking

Self-contained handoff for a fresh Claude session. Goal: **close the BB-led
two-ball juggle demo's throw aim** under real contact mechanics, so the pattern
catches ≥30 / 0 drops again and the two `xfail(strict=True)` headline tests flip
back to passing.

## Where you are working

- Worktree: **`/home/jetson/Desktop/Jugglebot-bb`**, branch
  **`demo/bb-led-two-ball-juggle`**. A PARALLEL Claude session works
  can-offload in the MAIN worktree (`/home/jetson/Desktop/Jugglebot`,
  `phase-a-plus-accuracy-cal`) — do NOT touch that worktree/branch or its
  processes (it runs its own `pytest tests/` in the main worktree).
- Always `source ~/Desktop/PDJ_venv/venv/bin/activate` before any python/pytest.
- `git log -1` should show **`d3a1534`** (or later). Baseline suite
  (`pytest tests/ -q`, run 2026-06-26): **1500 passed, 4 skipped, 3 xfailed**.
  The 3 xfails are 1 pre-existing + the 2 demo headline cases this work fixes.

## Read first, in order

1. **`logbook/2026-06-26-contact-mechanics-integration.md`** — THE resume
   artifact. The full contact-integration arc, the validated wins, the root
   cause, and the **Open Questions / follow-up A** section that IS this task.
   Read all of it, especially the Discussion (several hypotheses were raised
   and withdrawn — don't re-walk them).
2. `logbook/2026-06-26-velocity-matched-catch-and-contact-mechanics-feasibility.md`
   — the prior entry (concern 2 velocity-matched catch + concern 1 feasibility).
3. `plans/active/bb-led-two-ball-juggle-demo.md` — "Sim2Real fidelity upgrades"
   section (concern 1 marked "infrastructure landed; throw-aim WIP").
4. The code you depend on / will touch:
   - `controller/demo/juggle_optimizer.py` — the offline optimiser. The throw
     and catch velocity constraints now include the ω×r cup-offset term via
     `_casadi_left_jacobian` (rotvec-rate → true world ω). This is where a
     "bound the throw-instant yaw rate" fix would live.
   - `sim/juggle_demo.py` — the runner. Platform + hand commands are
     sub-stepped (`_plat_cmd_at`, `MuJoCoPlant.step(plat_cmd_fn=...)`). The
     linear sub-step is what fixed the carry; the angular analogue (if any) is
     a candidate fix.
   - `sim/plant/mujoco_plant.py` — `step(hand_cmd_fn, plat_cmd_fn)` sub-stepping.
   - `tests/sim/test_demo_juggle_sim.py` — the two `xfail(strict=True)` headline
     cases to flip back when the aim closes.
5. CLAUDE.md "Workflow Rules" + "Engineering Philosophy" — especially
   **"Invite physical-intuition pushback"** (the user's physical intuition was
   load-bearing all through the contact integration — e.g. the "ball leaves
   with exactly the hand velocity" pushback is what cracked the ω×r root cause),
   "Analyze control-system implications before changes", "Checkpoint before
   sinking effort", "Empirical probe before tests", "Cite test-count claims
   with the (date, command, result) triple".

## The problem (precise)

The contact mechanics are **faithful and validated**: the thrown ball leaves
with *exactly* the cup velocity (angle and speed) — proven on a
stationary-platform tilt sweep (cup moves at θ, ball leaves at θ). So the throw
is not the problem; the **cup's velocity at the throw** is.

The cup velocity is `v_centroid + ω×r_hand + R·[0,0,slider_speed]`. The
optimiser now models all three terms correctly (ω via the left-Jacobian). But
the demo throw still lands ~55–180 mm short of the catch, and the measured
reason is: **the platform's *achieved* angular velocity at the throw ≠ the
*planned* one.** A representative measurement (default pattern, seed 0):

- planned rotvec-rate at the throw: `[-0.49, -0.82, -4.13]` rad/s
- `J_l(rotvec)·rate` (planned true ω): `[0.15, -0.92, -4.12]`
- sim **achieved** true ω (MuJoCo angvel): `[1.29, -0.93, -4.50]`

The y and z roughly match; the **x-component is badly off (0.15 vs 1.29)**.
Since the cup velocity depends on ω×r (r_hand ≈ 0.2 m), an x-error of ~1.1 rad/s
is ~0.2 m/s of lateral cup velocity — comparable to the entire planned lateral
(~0.17 m/s). The faithful throw inherits the *achieved* cup velocity, which no
offline plan predicts → it lands short.

Caveat to resolve FIRST: part of the 0.15-vs-1.29 gap may be **measurement
timing** — ω swings fast mid-maneuver (the throw is a ~34° yaw at ~5 rad/s), and
the planned value and the sim sample were taken ~12 ms apart. Re-measure at the
EXACT same instant (substep resolution) before concluding it's a tracking error.

## Investigation plan

1. **Re-measure cleanly.** Instrument the runner at the throw release. Compare,
   at the SAME substep instant: planned cup velocity (from
   `player.command_at(t_rel)` + the optimiser's ω×r model) vs achieved cup
   velocity (`mj_objectVelocity` of the `hand_opening` site) vs ball velocity.
   Decompose into centroid-linear, ω×r, and slider terms. This tells you whether
   the residual is (a) the J_l/ω model still wrong, (b) genuine angular-velocity
   tracking error, or (c) just timing in the earlier measurement. **Probes live
   in `/tmp/probe_*.py` (NOT committed) per CLAUDE.md; promote to
   `tools/probes/` only if reused.**
2. **If it's angular tracking:** characterise it. Does the platform track the
   planned ω at the throw (sub-step-resolution achieved-vs-planned ω over the
   tick)? Is the planned throw-instant yaw (~5 rad/s) beyond what the
   connect-constraint platform + leg position-actuators can track at 40 Hz even
   with the platform-command sub-step? (The linear sub-step fixed the carry; the
   angular response may have its own limit.)
3. **Pick a fix (surface forks to the user, lead with root-cause failure
   modes):**
   - (a) Improve angular tracking (e.g., verify the platform sub-step actually
     helps ω; check actuator gains / whether the connect constraints lag in
     yaw).
   - (b) **Bound the throw-instant yaw rate** in the optimiser so the planned ω
     is trackable (likely the most robust — the pattern doesn't *need* a 5 rad/s
     yaw at the throw; it fell out of the unconstrained solve). Add a soft or
     hard cap on `‖ω_throw‖` (or its yaw component) at the throw knot.
   - (c) A small empirical per-throw aim correction (the user previously
     preferred fully-emergent and declined this; only revisit if (a)/(b) fail).
4. **Re-tune** `THROW_SPEED_CALIB` and the seat thresholds against the closed
   pattern once the aim lands.

## Process gates

1. Baseline: `git fetch && git status -sb` (origin not ahead, no foreign
   working-tree changes); `git log -1` shows `d3a1534`+; `pytest tests/ -q`
   shows 1500 passed / 4 skipped / 3 xfailed.
2. Pre-implementation: re-measure (gate 1 of the investigation), walk the
   control-system implications, surface the fix fork to the user. Tell the user
   explicitly that their physical intuition is load-bearing.
3. Implement the chosen fix. After each step: `python sim/juggle_demo.py
   --duration 30 --no-log --seed 0` (watch captures) + scoped `pytest tests/sim/`.
4. Verify: demo holds ≥30 catches / 0 drops under the real seat metric;
   determinism (same seed → same captures); **flip the two `xfail(strict=True)`
   headline tests back to plain passing** in `tests/sim/test_demo_juggle_sim.py`
   (and update their docstrings/header). Full `pytest tests/ -q` with the
   (date, command, result) triple.
5. Logbook: a NEW entry (or close out the open questions of
   `2026-06-26-contact-mechanics-integration.md`) documenting the angular-
   tracking diagnosis + fix; update `logbook/INDEX.md` (topmost row).
6. Plan: mark concern 1 RESOLVED in
   `plans/active/bb-led-two-ball-juggle-demo.md`'s Sim2Real section; note the
   strict seat-based headline restored.
7. `/audit --unstaged` on the combined code + test + logbook + plan diff. Apply
   LOW/MEDIUM, pause for HIGH.
8. Commit (`feat(demo): …` / `fix(...)`, `Logbook-Entry:` trailer, heredoc/-F —
   NO backticks in `-m`). Backfill the SHA into the logbook front-matter in the
   same response. `git fetch && git status -sb`, then push.

## Success criterion

The two `tests/sim/test_demo_juggle_sim.py` headline cases pass un-xfailed:
`test_full_sim_juggle_reaches_target_catches` (≥30 catches / 0 drops, seed 0)
and `test_short_run_catches_the_bb_primed_ball_and_a_throw` (both balls caught),
under the real contact + seat-based metric — with the throw fully emergent (no
analytic velocity override).
