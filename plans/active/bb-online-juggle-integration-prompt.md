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
- `git log -1` should show the online-replanning commit chain — HEAD at
  **`245cc18`** (pre-handoff tidy-up) or later; the arc is `025d812` (planner +
  band-limit) → `b2d8b09` (runner scaffold) → `2694673` (carry pre-roll +
  throw diagnosis) → `763d0e6` (throw-separation diagnosis CORRECTION) →
  `245cc18` (probe promotion + plan/handoff sync). Baseline suite (`pytest
  tests/ -q`): **1508 passed, 4 skipped, 2 xfailed** — the 2 xfails are the old
  demo's headline cases (still the offline architecture).
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

The ball does NOT separate with the switched contact (it's cohesively dragged
back down by the cup), but a **stiff-contact test catches it (1/0)** — so the
architecture is sound and the throw mechanism works (separation is `a_cup < −g`:
the ball lifts off when the cup decelerates faster than free-fall — NO clamp
needed). NOTE: an earlier version of this prompt mis-diagnosed a "bounded-slider
can't clamp" limitation — that was WRONG; ignore it. The two real issues are
TUNING, measured (see the module STATUS docstring):

1. **Contact cohesion.** With the switched contact the cup *pulls* the ball back
   (the ball reaches ~4.5 m/s up then is decelerated to 0 WITH the cup at
   ~169 m/s² ≫ g — the soft-contact drag the old demo solved with stiff
   contact). Forcing genuinely STIFF contact through the separation makes the
   ball fly free. The `t_rel < 0.10` switch isn't applying stiff effectively at
   the separation instant. **FIX:** stiffen the contact across the whole throw
   separation (and/or through the carry top); verify the ball flies free.
2. **Throw velocity (slam).** Stiff-always launches the ball at ~16.8 m/s (≫ the
   planned 5) because the cup **slams** it: the carry pre-roll leaves the cup
   lagging the plan (ends ~67 mm low), so the main loop's throw command makes the
   cup JUMP up to catch the plan and hit the ball. **FIX:** get the cup to reach
   the throw moving at `v_take` (≈5 m/s) WITHOUT a jump — tighten the pre-roll so
   its end-state matches the plan, and/or add a brief **COAST** (constant
   velocity) at the throw before the cup retracts (the user's model: a short
   coast, then the hand decelerates → the ball separates at v_take before the
   cup pulls back).

Verify the launched ball reaches ~apex 1.3 m and lands near the planned catch
before moving on. **Diagnostic harness (committed):**
`python tools/probes/juggle_online_debug.py [--stiff-always]` — logs the cup/ball
trajectory + the separation analysis (cup z-vel/accel vs ball z-vel through the
first throw, flagging `a_cup < -g`). That's how the cohesion (switched: ball
dragged) vs separation (`--stiff-always`: ball FLIES, catches 1) was found, and
it shows the slam (~16.8 m/s) too. Re-run it after each throw-fix attempt.

## Secondary — catch seating (after the throw works)

With a ball riding the cup (contact carry) the catch contact + velocity-matched
seat need tuning against this loop (the realisation de-risk tracked the cup
WITHOUT a ball). Check `BallManager` `set_contact_stiffness` switch windows
(stiff throw / soft catch), the `SEAT_*` thresholds (radius 40 mm, rel-vel
0.6 m/s, persist 15 substeps), and the catch lateral swipe (0.29 m/s at 100 mm —
should seat; widen separation only if mid-air collisions appear, narrow it for a
gentler swipe if seating is marginal).

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
