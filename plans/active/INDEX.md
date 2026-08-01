# Active plans

One row per plan in `plans/active/`. **Adding or archiving a plan updates this
table in the same commit** — `tests/sim/test_plans_index.py` fails otherwise.
Only *active* plan filenames may appear in this file; name an archived plan here
and the test fails (that is the point — it catches stale rows).

`Last touched` is the date of the last commit that changed the file.

| Plan | Status | Last touched | Scope |
|------|--------|--------------|-------|
| [accel-ff-inertia.md](accel-ff-inertia.md) | active | 2026-07-25 | Inertia-aware acceleration feedforward for lateral platform moves (wired behind a config flag; firmware prereq open) |
| [bb-led-two-ball-juggle-demo.md](bb-led-two-ball-juggle-demo.md) | active | 2026-07-31 | Ball-Butler-initiated two-ball oval juggling demo |
| [bb-online-juggle-tilt-rearchitecture.md](bb-online-juggle-tilt-rearchitecture.md) | active | 2026-07-04 | Online-juggle bring-up: clean catch → single-ball toss → two-ball, tilt-aimed |
| [bridge-clock-frequency-discipline.md](bridge-clock-frequency-discipline.md) | active | 2026-07-25 | Bridge clock rate discipline for µs-stable time across a multi-hour session |
| [catch-reach-degenerate-overshoot.md](catch-reach-degenerate-overshoot.md) | active | 2026-07-28 | Catch-reach near-degenerate overshoot (0.78° target → 2.32° excursion) |
| [fk-convergence-tolerance.md](fk-convergence-tolerance.md) | active | 2026-07-27 | FK convergence tolerance — spurious "did not converge" at round-off residuals |
| [hand-ball-sensor.md](hand-ball-sensor.md) | active | 2026-07-30 | Hand ball-present sensor: Phases 0–6 landed, Phase 7 gated on the hand-ODrive identity check |
| [hand-command-continuity.md](hand-command-continuity.md) | active | 2026-07-31 | Hand command continuity across the Jetson→Platform-Teensy conduit |
| [hand-trajectory-generator-overhaul.md](hand-trajectory-generator-overhaul.md) | active | 2026-07-31 | Jerk-limited, time-budget-parameterised hand trajectory generator |
| [learned-ff-residuals.md](learned-ff-residuals.md) | active | 2026-07-25 | Batch ILC on the leg torque channel (learned feedforward residuals) |
| [leg-gain-tuning-methodology.md](leg-gain-tuning-methodology.md) | reference | 2026-07-16 | Leg PID tuning methodology. The **gain hunt closed 2026-07-13** (40/0.20/0.32 ships, gains FROZEN); the document stays active as the normative procedure for any future tuning round |
| [levelling-frame-contract.md](levelling-frame-contract.md) | active | 2026-07-31 | One gravity/levelling correction applied to every pose request |
| [mvp-trajectory-bringup.md](mvp-trajectory-bringup.md) | active | 2026-07-31 | The live bringup plan: streaming trajectory path, S1–S8 hardware sittings |
| [refactor-2026-07.md](refactor-2026-07.md) | active | 2026-08-01 | 2026-07 refactor programme (Phase 0 landed; Phase 1 process changes 2026-08-01) |
| [single-ball-toss.md](single-ball-toss.md) | active | 2026-07-29 | Single-ball self-toss capability and its follow-on programme |
| [teensy-can-offload.md](teensy-can-offload.md) | active | 2026-07-31 | can_node → Teensy CAN offload; re-scoped to a can_node↔Teensy parity audit |

## Orchestration prompts

`PROMPT-*.md` files are self-contained session prompts for phase-runner
workflows rather than plans, so they are exempt from the table above:
`PROMPT-anomaly-fixes-orchestration.md`, `PROMPT-single-ball-toss-software-run.md`.

## Recently archived

Five plans moved to `plans/archived/` on 2026-08-01 (follower-cadence,
reload-action-catch-latch, hardware-bringup, dashboard-3d-mesh, telemetry
daemon) — each carries an "Archival note" section explaining what shipped and
why it closed. Filenames are deliberately not listed here; see
`plans/archived/` and `plans/active/refactor-2026-07.md` § Phase 1.
