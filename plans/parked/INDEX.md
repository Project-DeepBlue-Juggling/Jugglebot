# Parked plans

Plans that are **deliberately not now**. Everything here is real, unfinished work
that is not schedulable — either a named prerequisite has not cleared, or the
owner has chosen not to spend on it yet. Nothing here is abandoned; nothing here
is done. Done or superseded work goes to `plans/archived/`, and anything actually
schedulable belongs back in `plans/active/`.

One row per plan in `plans/parked/`. **Parking, unparking or archiving a plan
updates this table in the same commit** — `tests/sim/test_plans_index.py` fails
otherwise. Only *parked* plan filenames may appear here as bare markdown links;
name a plan that lives elsewhere and the test fails (that is the point — it
catches stale rows). Cross-directory references are therefore written
path-qualified (`plans/active/<name>.md`), which the guard deliberately ignores.

The load-bearing column is **What would unpark it**. A parked plan without a
stated gate is a plan nobody can pick up, so each row names the concrete
prerequisite, decision or event — derived from the plan's own text, never
invented. Where a plan states no gate, the row says so plainly.

| Plan | Parked | Why parked | What would unpark it |
|------|--------|------------|----------------------|
| [accel-ff-inertia.md](accel-ff-inertia.md) | 2026-08-15 | Nothing schedulable. THE gate — firmware stale-hold torque decay (flash bundle T1.1, `leg_interp.cpp:403`) — is still unwritten, and the motivating velocity-loop deficit was measured on the uptime-degraded plant, so the premise itself is suspect | **Two conditions, one now met.** (1) The firmware stale-hold torque decay is written and flashed — still open. (2) Phase 0 re-measures the motivating deficit on a healthy plant — **now obtainable**: FW 14 validated 2026-08-15 at 5.8 h and 15.2 h with lag flat at 10–20 ms and lead-clamp duty 0 (`logbook/2026-08-15-fw14-validated-arc-closed.md`). The re-measurement is itself the decision: the deficit may shrink substantially or vanish, in which case the plan archives instead of unparking. Resume from Phase 0 |
| [catch-reach-degenerate-overshoot.md](catch-reach-degenerate-overshoot.md) | 2026-08-15 | All five phases DONE and the fix shipped (C-CATCH-1 + seat rate 0.0). What is left is one pre-registered experiment, the seat-rate A/B (SEAT-EXP) | **It does not unpark as its own plan.** SEAT-EXP runs as a `plans/active/catch-robustness.md` Phase 3 row, so the gates are that row's: catch-robustness Phase 0 lands (its § Constraints forbid any catch-parameter tuning until then) **and** the unsolved +25–35 mm BB x-bias prerequisite is resolved. When both clear, fold SEAT-EXP into catch-robustness and archive this plan |
| [hand-trajectory-generator-overhaul.md](hand-trajectory-generator-overhaul.md) | 2026-08-15 | Untouched since its 2026-05-22 creation, and self-declared "a strict improvement, not a prerequisite" — parked on priority, not on a blocker | **No gate stated — the plan names no external prerequisite**, so unparking is an owner scheduling decision, not an event to wait for. It does state a resumption *condition*: the hand firmware changed twice underneath it (hand-command-continuity Phase 4 velocity-continuous `makeSmoothMove` seeding, Phase 7 `throwDecelToTorque`), so Phase 1's baseline characterisation must be re-taken against **Platform FW 2, not FW 0** |
| [learned-ff-residuals.md](learned-ff-residuals.md) | 2026-08-15 | The plan's own rule forbids any implementation until three named gates clear. G-A cleared 2026-08-15; G-B and G-C are open | **G-B and G-C, both open.** G-B: the accel-FF arc is concluded — shipped enabled *or* dormant, either is fine, what matters is that the model baseline is FROZEN before learning starts. G-C: firmware stale-hold torque decay is flashed (same flash bundle as accel-FF T1.1 — no new firmware work). Note the ordering this implies: G-B depends on `plans/parked/accel-ff-inertia.md`, which is itself parked, so this plan cannot unpark first. On resume, re-key the learner's healthy-bag filter off `latency_monitor` / `leak` rather than `uptime_ms` — G-A's closure made uptime a non-predictor |
| [levelling-frame-contract.md](levelling-frame-contract.md) | 2026-08-15 | Contract C-LEVEL-1/1.O shipped. Only Phase 4 residue remains, and it is separable items rather than one blocked deliverable | **No single gate — two independent triggers remain, either of which unparks its own item.** (1) LVL-2 needs only ~30 s of robot time, so it unparks at the next catch sitting it can be bundled into. (2) One OPEN DEFECT with no other owner: the Platform Teensy truncates the persisted offset to int16 milliradians (`Teensy_code_platform.ino:443-444`; 0.0264° measured / 0.0573° worst-case per axis against the ±0.05° band, and LVL-3 is structurally blind to it) — the plan's instruction is to address it at the next levelling touch, so any levelling work at all unparks that item. A **third** trigger, the LG-3 reachability decision, was **DISCHARGED 2026-08-16** (`logbook/2026-08-16-levelling-complete-ownership-resolved.md`): the state is reachable — `level`, then restart `trajectory_node` alone — so LG-3 is now robot time bundled with (1), not a gate anyone is waiting on |
| [refactor-2026-07.md](refactor-2026-07.md) | 2026-08-15 | The headline arc (ERR_TIMEOUT) closed 2026-08-09 on FW 10, and the bridge-uptime bullet was absorbed by `plans/archived/bridge-temporal-trustworthiness.md`. What remains has no technical blocker, only unmade decisions | **Owner decisions, not events.** Three are named — loader unification, tracked-media policy, and the deferred re-plug probe (that one additionally gated on the CAN3 hardware repair) — plus two unscheduled structural refactors (Phase 5 items 1/3, Phase 6 later slices). It unparks the moment the owner schedules any of them. ⚠ **§ Standing coordination rules stays LIVE process text regardless of the parked status** — read it while the plan is parked, do not treat the whole file as dormant |

## Where the other plans are

- `plans/active/INDEX.md` — the active board: what is actually schedulable now.
- `plans/archived/INDEX.md` — completed or superseded, date-sorted.
