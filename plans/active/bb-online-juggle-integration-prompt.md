# Next-session prompt — Online juggle integration: throw separation + catch tuning → sustained 2-ball catches

Self-contained handoff for a fresh Claude session. Goal: take the **online
re-planning juggle runner** (`sim/juggle_online.py`) from "runs but catches 0"
to **sustained 2-ball catches**, then port the full demo features and restore
the headline tests. The architecture, planner, characterisation, and realisation
are built and validated; what remains is the **throw-separation modelling** (the
crux) and **catch-seat tuning**.

## Where you are working

- Worktree **`/home/jetson/Desktop/Jugglebot-bb`**, branch
  **`demo/bb-led-two-ball-juggle`**. A PARALLEL Claude session may work the MAIN
  worktree (`/home/jetson/Desktop/Jugglebot`) on can-offload — do NOT touch it.
- Always `source ~/Desktop/PDJ_venv/venv/bin/activate` before python/pytest.
- `git log -1` should show the WIP-scaffold commit chain (…`d642d2e` + the
  pre-roll commit after it). Baseline suite (`pytest tests/ -q`): **1508 passed,
  4 skipped, 2 xfailed** — the 2 xfails are the old demo's headline cases (still
  the offline architecture).
- **Kai Ploeger's reference repo is cloned** at
  `/home/jetson/Desktop/kinematic_planning_for_nball_toss_juggling/` — read
  `task_space_juggling.py` (his `plan_throw` + the online re-plan loop) and
  `mujoco_wrappers.py`. Note: his hand is **kinematically velocity-controlled**
  (unbounded) and his balls are real contact free-bodies.

## Read first, in order

1. **`logbook/2026-06-27-online-replanning-architecture-and-cup-bandlimit.md`** —
   THE resume artifact: the architecture pivot, the band-limit characterisation,
   the level-platform decoupling design, the planner, and the **Integration
   status (WIP)** section.
2. **`logbook/2026-06-27-throw-aim-band-limit-and-closed-loop-catch.md`** — the
   prior entry: why the OFFLINE architecture was abandoned (the platform band
   limit), with the same-instant decomposition that refuted "angular tracking".
3. **`sim/juggle_online.py`** — the WIP runner. Its top **STATUS docstring** has
   the precise current diagnosis (read it — it's the heart of this task).
4. **`controller/demo/juggle_planner.py`** — the per-throw cup NLP (Kai-adapted).
   `tests/sim/test_demo_juggle_planner.py` (7 tests) pins its constraints.
5. **`tools/probes/juggle_cup_bandlimit.py`** — re-run to refresh the band-limit
   numbers (slider near-perfect; platform −3 dB ~5 Hz lateral).
6. CLAUDE.md "Workflow Rules" + "Engineering Philosophy" — especially *invite
   physical-intuition pushback*, *checkpoint before sinking effort*, *empirical
   probe before tests*, *cite test-count claims with the (date, command, result)
   triple*, *no backticks in `git commit -m`*, *backfill SHA after commit*,
   *`/audit --unstaged` before narrative commits*, *auto-push after commits*.

## The architecture (what's built, in one paragraph)

Each throw, the runner observes the in-flight ball, ballistically predicts its
touch-down (`ballistic_touchdown`), and re-solves the cup's Cartesian trajectory
for the next cycle (`plan_cup_cycle`) from the ACHIEVED cup state. It realises
the cup via **level-platform decoupling**: `centroid_xy = cup_xy` (platform —
band-limited but tracks smooth lateral motion), `slider = cup_z − 659.6 mm`
(slider — fast/perfect; cup_z_world ≈ 659.6 mm + slider at centroid z=170 mm).
Banking is dropped entirely. The catch is velocity-matched (soft, 0.7×ball). A
**carry pre-roll** at startup seats ball 0 at the catch point and carries it to
the throw moving up (fixing the cycle-0 static-start). Pattern: separation
**100 mm** (balls are 70 mm — clears; the swipe scales with separation:
200→1.18 m/s, 100→0.29, 60→0.16 m/s), throw_z 0.85 m, catch_z 0.70 m, apex 1.3 m.

## THE KEY PROBLEM — throw separation (do this first)

The ball does NOT launch: it **rides the cup** up to the slider top (~1.0 m) and
back down, never separating (confirmed via a debug probe — recreate it; see
below). Root cause (also in the module STATUS docstring): the planner makes a
**smooth, bounded** cup trajectory (cup z ≤ slider reach, decelerating to vz≈0
at the top), so the cup never *clamps* at the slider top while still moving up —
and without that clamp the ball can't outrun the cup. **Kai's hand is unbounded**
(follows the ball up to apex then returns); our **bounded slider** (0.34 m
stroke) cannot. So the throw is fundamentally a **stroke-to-the-top-and-release
(clamp) event**, not a smooth trajectory the ball inherits velocity from.

**Design choice (surface to the user if unsure — lead with root-cause failure
modes):**
- **(a) Model the release in the planner** — split the cycle at the throw: let
  the cup reach z_max with v_take (the ball separates there because the cup
  can't follow it up), and plan the post-release come-down separately. Cleaner
  conceptually; bigger planner change; the cup-velocity-at-release IS the ball
  launch velocity, so it's physically exact.
- **(b) HYBRID (recommended, smaller)** — keep the planner for the LATERAL (xy)
  positioning + the catch + the come-down, but drive the THROW vertical with an
  explicit **slider stroke to the top** (clamp → release), exactly how the OLD
  demo launched balls (`sim/hand/trajectory.py` `HandThrowTrajectory` /
  `HandThrowSequence` — a known-good throw stroke). At the throw instant,
  override the slider command with the stroke-to-top for ~the release window,
  call `begin_physics_throw`, then hand the slider back to the planner for the
  come-down. The planner's z near the throw becomes a don't-care (the stroke
  owns it); its xy + catch still drive everything else.

Recommendation: **(b)** — it reuses a validated throw and isolates the change to
the realisation/slider, not the NLP. Verify the launched ball reaches ~apex
1.3 m and lands near the planned catch.

## Secondary — catch seating (after the throw works)

With a ball riding the cup (contact carry) the catch contact + velocity-matched
seat need tuning against this loop (the realisation de-risk tracked the cup
WITHOUT a ball). Check `BallManager` `set_contact_stiffness` switch windows
(stiff throw / soft catch), the `SEAT_*` thresholds (radius 40 mm, rel-vel
0.6 m/s, persist 15 substeps), and the catch lateral swipe (0.29 m/s at 100 mm —
should seat; widen separation only if mid-air collisions appear, narrow it for a
gentler swipe if seating is marginal).

## Debug probe to recreate (not committed — `/tmp/probe_online_dbg.py`)

Wrap `runner.plant.step` to log, per substep, `(t, cup_xz, ball0_xz+held,
ball1_xz+held)` over the first ~3 cycles of `OnlineJuggleRunner(
OnlineJuggleConfig(duration_s=2.2)).run()`. This is how the ride-the-cup
(no-separation) behaviour was found. Decimate the print (every 4th sample).

## Investigation plan

1. Baseline: `git fetch && git status -sb` (origin not ahead, no foreign
   working-tree changes); `pytest tests/ -q` shows 1508/4/2; re-run
   `python tools/probes/juggle_cup_bandlimit.py` to refresh the limits.
2. Reproduce the throw-separation failure with the debug probe (ball rides the
   cup, no launch). Confirm the diagnosis.
3. Implement the throw fix (recommended (b)); verify a launched ball reaches
   ~apex and lands near the planned catch (probe the first throw in isolation).
4. Get the FIRST catch (ball 1) seating; then iterate to **sustained** catches
   (the per-cycle re-plan should self-correct — that's the whole point).
5. Once it sustains: port the full demo features the old `juggle_demo.py` has
   (telemetry CSV/dashboard, abort path, BallButlerSim priming if wanted), and
   **retarget the two `tests/sim/test_demo_juggle_sim.py` headline cases** to the
   online runner (un-xfail when they pass), OR replace the old runner outright
   (the user authorised "replace the optimizer+Player outright").

## Process gates

- After code changes: `python sim/juggle_online.py` (watch captures/drops) +
  scoped `pytest tests/sim/`. Full `pytest tests/ -q` before any commit (cite
  the date/command/result triple).
- `/audit --unstaged` before any commit that touches logbook/plan/normative md.
- Commit (`feat(demo): …`, `Logbook-Entry:` trailer, heredoc/-F — NO backticks),
  backfill the SHA into the logbook in the same response, `git fetch && status`,
  push.
- Keep the logbook (`2026-06-27-online-replanning-architecture-and-cup-bandlimit`)
  updated as the resume artifact; new entry if the throw-fix arc is large.

## Success criterion

`sim/juggle_online.py` sustains the 2-ball pattern (≥30 catches / 0 drops, seed
0) under real contact with the fully online per-throw re-planning (no offline
trajectory, no kinematic velocity override) — the cup tracing a continuous carry
oval. Then the headline tests pass un-xfailed against the online runner.
