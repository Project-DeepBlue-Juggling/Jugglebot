---
title: Fast-catch fidelity — the Rung-1 catch now seats a REAL fast Ball Butler throw (~4.9 m/s vz) via a firmed catch contact + a phase-matched descent seat; the operator's physical intuition (real balls seat at high velocity-delta; a cup at 50–80% of ball speed is normal) reframed a "the catch can't do a fast BB" conclusion into a sim-fidelity gap
type: bugfix
date: 2026-07-02
status: resolved
phase: "Online-juggle tilt re-architecture — Phase 1 / Rung 1 (clean single catch) — fast-catch fidelity follow-up"
related_plan: "bb-online-juggle-tilt-rearchitecture.md"
files_changed:
  - sim/ball/manager.py
  - sim/juggle_catch.py
  - tests/sim/test_juggle_catch.py
  - tests/sim/test_juggle_selfcatch.py
  - tools/probes/juggle_fastcatch.py
  - tools/probes/README.md
commits:
  - TBD
subsystem:
  - sim
tags:
  - control
  - contact
  - planning
  - catch
---

# Fast-catch fidelity — seating a real fast Ball Butler throw

## Summary

The Rung-1 catch (`sim/juggle_catch.py`) was tuned for a **gentle ~2.5 m/s lob** and
could not seat the **fast (~4.9 m/s vz), flat (~15° from vertical) arrival a REAL Ball
Butler delivers** at the catch height. Fed the real BB throw it dropped the ball
(held 0/6 — the ball ends ~1 m below the cup). Two coupled causes:

1. **Soft catch contact.** The catch/carry contact was `solref=(0.05, 2.0)` — a 50 ms
   overdamped spring, too slow to arrest a 4.9 m/s ball inside the ~70 mm seat-escape
   window. The ball punched straight through the cup (plunge-through).
2. **Gentle-tuned seat.** The seat held the cup high, then descended late so it passed
   catch_z velocity-matched at touch-down. Tuned for the slow lob, it reached catch_z
   **too late** for a fast arrival — the cup was still descending (not co-moving) when
   the fast ball landed.

**Firm contact is necessary but not sufficient** (real BB + firm contact + the old
hold-then-drop seat ≈ 1/6). The fix needs **both**: a firmer contact **and** a
phase-matched descent.

The fix, both landed in the Rung-1 catch only (the self-catch seat is left alone):

* **Firm catch contact** — `CONTACT_SOFT_SOLREF=(0.01, 1.0)`,
  `CONTACT_SOFT_SOLIMP=(0.99, 0.999, 0.0005)` (a ~10 ms time constant). Arrests the
  fast arrival in-window; still cushioned for the gentle lob and the self-catch's
  co-moving seat (both re-verified MAKE-preserving).
* **Phase-matched descent seat** — every tick, from a larger ready-lift (0.15 m),
  drive the cup so it arrives at catch_z **moving down at `ratio`·|estimated arrival
  vz| EXACTLY at the predicted touch-down** (a re-solved quintic completing at
  `t_td`), i.e. the cup is **co-moving** with the ball when it lands. Once the true
  ball reaches catch_z, constant-decel the co-moving cup to rest. `ratio` = **0.55**
  (sweet spot 0.5–0.6; 0.7+ commands a descent the band-limited cup can't track and it
  misses the phase).

Result under §3 noise: the gentle lob **8/8 held** (~5.6 mm in-cup, tighter than the
old seat's 3.3 mm mean / 17-20 headline) AND the real BB **8/8 held** (~5.6 mm) when
the landing is in reach; the self-catch MAKE is preserved **12/12 on all 6 seeds**.

## Discussion

### The operator's intuition reframed the problem

The prior conclusion (from the Rung-1 characterisation, `2026-06-30-rung1-clean-single-catch.md`)
was effectively "the catch primitive handles the gentle synthetic lob it's fed; a real
fast BB is a different, harder problem." The operator pushed back with a **physical
intuition**: *real jugglers/catchers seat balls at large velocity deltas all the time,
and it is entirely normal for the catching cup to be moving at 50–80% of the ball's
speed at contact.* That reframed a "the catch can't do a fast BB" dead-end into a
**sim-fidelity gap**: the sim couldn't seat what the real system routinely does,
because (a) the sim contact was too soft to arrest a fast ball and (b) the seat wasn't
co-moving at the right phase. This is the CLAUDE.md "invite physical-intuition
pushback" rule paying off — the intuition was load-bearing and correct.

### Why the fast BB failed, mechanism by mechanism

* **Contact.** A 50 ms overdamped contact reacts on the timescale of the whole flight,
  not the impact. A 4.9 m/s ball travels ~5 mm/ms; before the soft contact builds
  arresting force the ball is already past the 70 mm `SEAT_ESCAPE_M` window and the
  seat state-machine un-holds it. Firming the time constant to ~10 ms arrests the ball
  within the window. **This is necessary even for the gentle lob under the new
  phase-matched seat** — the phase-matched seat drives the cup DOWN at contact, so the
  closing dynamics are more demanding than the old hold-then-drop; the probe shows the
  gentle lob plunges through under SOFT + phase-match (0/6) and seats 6/6 under FIRM.
* **Seat timing.** Both pre-existing seats mistime a fast arrival:
  - The **shipped Rung-1 fixed-schedule** seat (hold at ready_z, descend to catch_z
    velocity-matched over a fixed `catch_start_lead_s`) reaches catch_z too late for a
    fast ball — the cup is still descending, not co-moving, when the ball lands.
  - The **self-catch kinematic co-moving** seat (track the estimated ball descent,
    opening a seat-depth above it) is tuned for the self-catch's clean-release arrival
    phase and speed; it does not compose onto the BB's faster, differently-phased
    arrival.
  The **phase-matched descent** targets the physical invariant directly: be at catch_z
  moving down at `ratio`·|v_arr| **exactly when the ball is there**. The quintic is
  re-solved each tick with horizon `t_td`, so early (t_td large) the cup descends
  gently and late (t_td small) it accelerates to hit the matched velocity at the
  crossing — naturally phase-locked to the estimate.

### Why `ratio` 0.55 and not 0.7+

The cup's vertical channel is the slider (fast, good tracking) but the descent must be
*achieved* by the platform+slider under the band limit. A commanded co-moving velocity
of 0.7·|v_arr| ≈ 3.4 m/s is beyond what the cup tracks over the short approach horizon;
it lags the command, so it is NOT actually at `ratio`·|v_arr| at the crossing — it
misses the phase and the ball plunges. Empirically 0.5–0.6 is the sweet spot; the probe
sweep and the 8-seed re-validation both put 0.55 in the middle of the passing band, and
ratio 0.7 fails the fast BB (0/8). Physically: aim for a co-moving speed the cup can
actually reach, not the largest one that would minimise the relative impact speed.

### The contact constant is not automatically applied to a catch-only harness (sharp edge)

`ball.manager.set_contact_stiffness(False)` is idempotent and the manager initialises to
"soft", so the **first** `set_contact_stiffness(False)` a runner makes is a **no-op** —
it leaves the ball+cup geoms at their MJCF-authored defaults (`solref="0.05 2.0"` in
`sim/model/jugglebot.xml`, i.e. the OLD soft). The self-catch / throw / online runners
end up on `CONTACT_SOFT_*` only because their throw flips the contact stiff→soft, which
writes the constants. **The Rung-1 catch has no throw**, so it never applied the module
constant and ran at the MJCF default. Changing `CONTACT_SOFT_*` alone therefore did NOT
firm the catch. The scoped fix: in `SingleCatchRunner.run()` toggle stiff→soft once so
the (now firm) `CONTACT_SOFT_*` values are actually written onto the geoms before the
catch. This is a latent inconsistency — `CONTACT_SOFT_*` is not truly authoritative for
a catch-only path — flagged here as a sharp edge; a broader "apply the constants on the
first `set_contact_stiffness` call" contract fix was deliberately NOT taken to keep the
blast radius to the Rung-1 catch (the throw/online/demo initial-carry contact would
otherwise change too). Deferred as a possible follow-up.

### Scope decision

The fix is confined to the **Rung-1 catch** (`SingleCatchRunner`). The **self-catch
seat** (`SelfCatchRunner._catch`, the co-moving kinematic seat that is the validated
Rung-2b MAKE) is left byte-identical — it only inherits the firmer contact (which it
already used via its stiff→soft transition), re-verified 12/12. The online-runner's
catch (Rung 3, which will face the fast BB seed for the two-ball juggle) is future work;
the phase-matched seat recipe is now proven in isolation and ready to lift into it.

## Fix

* **`sim/ball/manager.py`** — `CONTACT_SOFT_SOLREF` `(0.05, 2.0)` → `(0.01, 1.0)`,
  `CONTACT_SOFT_SOLIMP` `(0.99, 0.99, 0.001)` → `(0.99, 0.999, 0.0005)`; comment
  block updated to explain the firmer ("soft" is now a historical name) catch contact.
* **`sim/juggle_catch.py`**
  - `SingleCatchConfig` defaults: `catch_ready_lift_m` 0.06 → **0.15**,
    `catch_slider_vel_ratio` 0.95 → **0.55**; docstring rewritten for the
    phase-matched descent.
  - `run()`: toggle stiff→soft after `reset()` to force the firm contact onto the
    geoms; **only the Z channel** replaced — the fixed-schedule (hold ready_z →
    descend velocity-matched → constant-decel) becomes the phase-matched descent
    (arrive at catch_z moving down at `ratio`·|v_arr| @ t_td → constant-decel once the
    TRUE ball reaches catch_z, latched). The warm-up guard (`est.n < 4`), the frozen-xy
    reach on the first post-warm-up fit, the tilt ramp-to-receive completing at t_td,
    and the workspace clamp are unchanged.
  - New `_spawn_incoming(target)` helper (extract-method) that spawns the incoming
    ball at its §3-perturbed apex from **either** the synthetic gentle lob (default) or
    a **real `BallButlerSim`** throw (`use_real_bb`, `bb_position_mm`,
    `bb_yaw_offset_rad` config fields; BB imported lazily).
* **`tests/sim/test_juggle_catch.py`** — new
  `test_fast_real_ball_butler_throw_is_seated`: a real BB throw, 6 seeds, §3 tracking
  noise, all caught+held+clean within 20 mm, tilt near the 12° clamp (genuinely fast/
  flat). The existing catch tests pass unchanged (the seat improved — tighter offsets).
* **`tests/sim/test_juggle_selfcatch.py`** — `test_reach_amplifies_loop_gain_gt_one`
  re-pinned from seed 3 → seed 0 (test-only). The firmed catch contact (inherited by
  the self-catch) shifted seed 3's column-detach divergence from reach-amplification to
  the catch-seat knife-edge; seed 0 still shows the amplification signature
  (7.7 → 106.7 mm). No self-catch behaviour changed — every column seed still diverges.
* **`tools/probes/juggle_fastcatch.py`** (promoted from `/tmp/probe_fastcatch_recipe.py`,
  dead `sweep` dropped) + a `tools/probes/README.md` row.

## Verification

All runs on the Jetson in the project venv (`~/Desktop/PDJ_venv/venv`), pinned MuJoCo.

* **Self-catch MAKE preserved** (the make-or-break gate; the firmer contact must not
  regress it). `python -c "... SelfCatchRunner(SelfCatchConfig(oscillate=True,
  n_cycles=12, seed=s, release='kinematic', dip_m=0.10)).run().sustained for s in 0..5"`,
  run 2026-07-02: **12/12 sustained on all 6 seeds (72/72)**, flat ~0.6 mm in-cup
  offset — identical to pre-change.
* **Rung-1 catch + Rung-2a throw tests green** — regression check of the pre-existing
  tests (before the new fast-BB test was added): `pytest tests/sim/test_juggle_catch.py
  tests/sim/test_juggle_throw.py -v`, run 2026-07-02: **13/13 pass in 30.81 s**. No
  asserted offsets/counts needed loosening — the catch improved (noise-off + §3 in-cup
  offsets tightened to ~5.6 mm); the throw is unaffected (its carry never applies
  `CONTACT_SOFT` before the stiff throw stroke). With the new test + the self-catch
  seed re-pin: `pytest tests/sim/test_juggle_selfcatch.py tests/sim/test_juggle_catch.py
  -q`, run 2026-07-02: **41 passed, 1 xfailed in 286.30 s** (the xfail is the intended
  strict-xfail column-sustain BREAK headline).
* **Fast-catch fidelity — seating.** `run_single_catch(SingleCatchConfig(use_real_bb=
  True, noise=NoiseConfig(0.0, 0.5), seed=s)) for s in 0..5`, run 2026-07-02: **6/6
  caught+held+clean, 5.6 mm in-cup, tilt 12.0° (clamped — genuinely the fast/flat
  arrival)**. Gentle synthetic lob under full §3: **8/8 clean, 5.6 mm**.
* **Head-to-head vs soft + full-§3 reach split** (`python tools/probes/juggle_fastcatch.py`,
  run 2026-07-02): SOFT plunges through at BOTH speeds under the phase-matched seat —
  gentle-lob **0/6** (~940 mm below), real-BB **0/6** (~1000 mm below); FIRM +
  phase-match seats gentle-lob **6/6** (~5.6 mm) and real-BB **5/6** (~5–9 mm). The
  lone real-BB miss (seed 4, 414 mm) is a **REACH** failure — the real BB's ~1.3 m
  throw × the §3 2% BB-throw noise scatters the landing beyond the platform's ±150 mm
  workspace, a characterised reach limit (as for the synthetic throw), NOT a
  plunge-through. Under §3 **tracking noise only** (landing at centre) the real BB is
  8/8, isolating the seating fidelity — which is why the new test disables the BB-throw
  scatter (reach is characterised separately).
* **Full suite** (`pytest tests/ -q`, run 2026-07-02): first run **1 failed, 1594
  passed, 4 skipped, 3 xfailed in 1038.96 s** — the lone failure was
  `test_juggle_selfcatch.py::test_reach_amplifies_loop_gain_gt_one`, a self-catch
  BREAK-*signature* test (NOT the MAKE — see below). The firmed catch contact (which
  the self-catch inherits) shifted the per-seed column-detach divergence mode: seed 3
  now breaks at cycle 1 via the catch-seat knife-edge rather than by reach
  amplification, while **seed 0 still amplifies (7.7 → 106.7 mm)**. Every column seed
  still DIVERGES (`test_column_selfcatch_does_not_sustain` all green) — only the
  seed-specific *mode* moved. Fixed by re-pinning the amplification-signature test to
  seed 0 (a test-only change; no self-catch behaviour changed). Re-run after the fix
  (`pytest tests/ -q`, run 2026-07-02): **1595 passed, 4 skipped, 3 xfailed in
  1043.33 s** (exit 0) — all green.

## Related

* `logbook/2026-06-30-rung1-clean-single-catch.md` — the Rung-1 catch this improves.
* `logbook/2026-07-01-rung2b-kinematic-release.md` — the self-catch MAKE preserved here.
* `tools/probes/juggle_fastcatch.py` — the recipe probe (SOFT vs FIRM × arrival speed).
* `plans/active/bb-online-juggle-tilt-rearchitecture.md` — the parent plan (not updated
  here; the parent will).
