---
title: Online-juggle capability bring-up — clean catch → single-ball toss → two-ball (tilt-aimed, hardware-bringup order)
created: 2026-06-29
status: active
related_plan: bb-led-two-ball-juggle-demo.md
related_logbook:
  - 2026-06-29-platform-tilt-tracking-characterisation
  - 2026-06-27-online-replanning-architecture-and-cup-bandlimit
  - 2026-06-27-online-juggle-throw-fix-catch-axis-split-band-limit-cascade
  - 2026-06-27-throw-aim-band-limit-and-closed-loop-catch
---

# Online-juggle capability bring-up

## 1. Context

### Where we are

`sim/juggle_online.py` (+ `controller/demo/juggle_planner.py`) is an online,
per-throw, task-space planner (Kai Ploeger's `plan_throw`, IEEE 9981678; repo at
`/home/jetson/Desktop/kinematic_planning_for_nball_toss_juggling/`). It catches
**5 / 0** then collapses to 1-ball: the catch seats the ball ~15 mm off-centre
(the platform's band-limited lateral tracking error) and the **contact-carry
throw amplifies that beyond reach** → divergence (loop gain > 1; logbook
`2026-06-27-online-juggle-throw-fix-catch-axis-split-band-limit-cascade`).

### Why a capability ladder (the structural decision)

Our struggle came from debugging **coupled failure modes inside the 2-ball
loop** — when a run collapsed we couldn't tell whether the catch, the throw, the
timing, or the startup broke, so fixes to one broke another. We therefore bring
the system up as an **incremental capability ladder** that **mirrors the hardware
bring-up order**, isolating each primitive:

- **catch** is validated standalone with a known-clean input (Rung 1),
- **throw** is validated standalone for accuracy (Rung 2a) then closed into the
  minimal toss-and-self-catch loop with **no 2-ball confound** (Rung 2b — the
  cleanest possible test of "did tilt kill the divergence"),
- **two-ball** then only has to solve *composition* (Rung 3).

Sim mirrors how we'd safely bring up the real robot (catch one throw → controlled
single tosses → juggle), so each rung **de-risks a hardware milestone** and yields
a **reusable primitive** (a general catch, a general parameterised throw) rather
than one bespoke demo.

### Why tilt is the engine

The **level-platform decoupling** forced the lateral throw component onto the
**band-limited platform velocity** (−3 dB ~5 Hz; the throw is a fast transient).
Kai instead **tilts** and detaches **along the tilted axis**, so the *speed* comes
from the fast actuator (slider) and the *aim* from a roughly-constant tilt:
`lateral take-off vel = slider_speed × sin(tilt)` (5 m/s × sin 12° ≈ 1.04 m/s).
Rung 0 confirmed the platform tracks tilt ~6× tighter than lateral translation
(3.1° vs 17° phase lag at the cycle freq; lever arm ~1.66 mm/deg; ≤12° usable;
logbook `2026-06-29-platform-tilt-tracking-characterisation`). Tilt is the
primary lateral-aiming mechanism for **both** the throw (aim) and the catch
(tilt-to-receive).

### Decisions already taken (do not relitigate without the operator)

- **Keep contact-physics fidelity** throughout — no kinematic / controlled-
  velocity throw shortcut. Make tilt-aimed throws + catches work under *real*
  contact.
- **Tilt is the engine** (Rung 0 done); modest, ~constant through a throw, re-aimed
  cycle-to-cycle.
- **Capability-ladder bring-up order** (catch → toss → self-catch → two-ball),
  mirroring hardware.
- **Hardware-faithful noise (cross-cutting, §3):** BB throws carry configurable
  initial-condition noise (start ~2%); all ball tracking carries configurable
  observation noise (start ±0.5 mm). Every rung's acceptance must hold *under the
  noise*.
- **BB model + reload pose:** BB is modelled simply as a ball appearing with an
  initial state `(pos, vel)`, **origin placeable anywhere**. The reload's
  **nominal/home pose is COM (0, 0, 170 mm)**, but the platform **translates to
  reach the *observed* landing** when a throw lands off-nominal (the §3 BB noise
  scatters the landing well beyond the tilt lever-arm, so the catch must reach).
  That catch translation is **slow** (over the ball's flight), so it stays inside
  the platform's good tracking band — the band limit bit only *fast* transients,
  and the old demo's closed-loop catch already reached this way. The **tilt** does
  the clean receive: **orientation driven by the throw geometry** (tilt-to-receive:
  cup axis aligned with the incoming ball's arrival velocity, Kai-collinear-style),
  ≤12°, with the Rung 0 lever-arm compensation on top. So **catch = slow
  translate-to-reach (position) + tilt-to-receive (orientation)**; the *throw*
  stays tilt-aimed with no fast translation.

### Goal (ladder top, for now)

Sustained 2-ball juggling — **≥30 catches / 0 drops** under the noise, BB-seeded,
columns → oval; the headline `test_full_sim_juggle_reaches_target_catches`
(currently `xfail`) passes un-xfailed against the online runner. Beyond that:
3-ball and richer patterns (Rung 4+, future).

## 2. The capability ladder

Each rung is one **workflow phase** (the saved workflow runs phases by integer;
the rung↔phase map is in §4). Each rung is run by a fresh Claude following
**implement → audit → commit (fetch-guarded push) → write the next prompt**.

---

### Phase 0 / Rung 0 — Platform tilt-tracking characterisation  ✅ DONE

Committed `tools/probes/juggle_tilt_bandlimit.py` + logbook
`2026-06-29-platform-tilt-tracking-characterisation`. Key numbers Rungs 1-2 use:
lever arm **−1.66 mm/deg** lateral cup shift (compensate `centroid_xy`), vertical
cross-coupling negligible (~2 mm at 12°), **≤12° usable tilt**, tilt held ~constant
through a throw (tracks 0.994 / 3.1° at the 1.6 Hz cycle freq, >45 mm leg margin
at 24°). Caveat carried forward: Rung 0 proves the platform can *present* an
accurate tilt — not that the ball detaches/seats cleanly *along* it under contact
(that is Rungs 1 + 2b).

---

### Phase 1 / Rung 1 — Clean single catch (BB-reload)  *(GATE after)*

**Goal.** Catch one BB-thrown ball cleanly — seated with minimal, characterised
in-cup offset — from BB **placeable anywhere** (throws landing anywhere in the
workspace, incl. the §3-noise scatter), by **translating to reach the observed
landing** (slow → band-safe; nominal home COM (0,0,170)) and **tilt-to-receive**
for clean orientation, **robust to the §3 noise**. The catch primitive in
isolation, with a known-clean input.

**Approach.**
- **BB as a ball.** Spawn a ball with initial `(pos, vel)` from anywhere (the
  throw origin); apply the §3 **BB throw noise** to the initial conditions — which
  scatters the landing well beyond the tilt lever-arm, so the catch must *reach*.
- **Translate-to-reach (position).** Observe the in-flight ball, predict its
  ballistic touch-down xy, and translate the platform **centroid** to it (full
  ±150 mm workspace). This reach is **slow** (over the flight), so it's inside the
  platform's good band — the band limit bit only *fast* transients, and the old
  demo's closed-loop catch already reached this way. Nominal/home is COM (0,0,170).
- **Tilt-to-receive (orientation).** Compute the cup **tilt** (rx/ry) that aligns
  the cup axis with the observed arrival velocity (Kai-collinear catch), ≤12°. The
  **slider** does the vertical descend-with-the-ball (the existing axis-split
  catch). Apply the Rung 0 lever-arm compensation so the tilted cup opening still
  meets the ball at the reached xy.
- **Observation.** Observe the in-flight ball with the §3 **tracking noise**;
  predict touchdown ballistically; set the catch tilt + slider; confirm the seat.
- Characterise + minimise the **in-cup seat offset** across the placement range
  and under the noise.

**Files.** `sim/juggle_online.py` (or a focused single-catch harness), the BB-as-
ball spawn + noise hook, the tilt-to-receive realisation, the noise model (§3 —
likely a small `sim/` noise helper or plant option), `tools/probes/` measurement
harness if reused, `tests/sim/…`, a logbook entry.

**Acceptance.** Reliable clean catch of BB throws landing across the workspace
(incl. the noise scatter) with a characterised (small) in-cup offset, **holding
under 2% BB noise + 0.5 mm tracking noise** (seed-reproducible). The
translate-to-reach + tilt-to-receive geometry + lever-arm compensation are
verified. **GATE:** is the catch clean + robust enough to feed a throw?

**Status (2026-06-30): landed** (gentle synthetic BB lob; ~3.3 mm in-cup offset
under §3 noise). **Fidelity follow-up (2026-07-02): the Rung-1 catch now seats a
REAL fast `BallButlerSim` throw (~4.9 m/s arrival), not just a gentle synthetic
lob.** The operator's physical intuition (real balls seat well at high velocity
delta; a catching cup at 50–80% of ball speed is normal) reframed a "the catch
can't do a fast BB" dead-end into a sim-fidelity gap: the old **soft** catch
contact (50 ms) plunged fast balls through the 70 mm seat window, and the seat was
tuned for ~2.7 m/s lobs. Fix: a **firmer catch contact** (`solref 0.05→0.01`) + a
**phase-matched descent** (cup co-moving down at ~0.55·|arrival vz| at the predicted
touch-down, from a 0.15 m ready-lift). Result: gentle lob **8/8** AND real BB **8/8**
(in reach), self-catch MAKE preserved **12/12**. Scoped to the Rung-1
`SingleCatchRunner`; the **online-runner catch** (for Rung 3's two-ball BB-seed)
needs the same port — future work. See `logbook/2026-07-02-fast-catch-fidelity.md`.

---

### Phase 2 / Rung 2a — Single-ball throw to arbitrary targets  *(GATE after)*

**Goal.** Throw a single ball (tilt-aimed) to **arbitrary, scoped workspace
targets**, accurately and repeatably, at **settable cadence** — measured
open-loop (no catch yet). This is where tilt enters the *throw*.

**Approach.**
- **Planner** (`controller/demo/juggle_planner.py`): tilted-axis detach (Kai's
  `cross(cacc − g, hand_axis) == 0` along the *tilted* cup axis); aim the take-off
  velocity along it; tilt is a planner input bounded ≤12°; level (`tilt = 0`) must
  remain the unchanged special case (existing tests pass).
- **Realisation** (`sim/juggle_online.py::realize`): re-introduce orientation
  (rx/ry); compensate the lever-arm.
- **Parameterise** the throw **target** (a scoped reachable-workspace box) and the
  **cadence** (beat timing over a useful range). Throw one ball, measure landing
  vs target (open-loop). Tracking noise applies to any observation.

**Files.** `controller/demo/juggle_planner.py`, `sim/juggle_online.py`,
`tests/sim/test_demo_juggle_planner.py` (tilt-constraint tests: lateral take-off
= `slider_speed × sin(tilt)`, detach collinear with the tilted axis),
`tools/probes/` landing-accuracy harness, logbook.

**Acceptance.** Open-loop landing error within a stated bound across the target
range and the cadence range; tilt-aim verified numerically; `tilt = 0` regression-
safe. **GATE:** are the throws accurate enough to close a loop on?

**Status (2026-06-30): implemented — gate pending.** Landed: a tilted-axis detach
in `plan_cup_cycle` (`cross(cup_acc − g, axis) == 0`, byte-identical at `tilt=0`),
`tilt_to_throw` + the unified `realize` (orientation re-introduced, delegates to
`realize_tilted`), and a single-throw harness `sim/juggle_throw.py`. **Result:**
the tilt-aim is *exact* — planned take-off ∥ the tilted axis, `lateral = |v|·sin θ`
to machine precision, and the cup reaches `v_takeoff` to ~1 % under contact in
**every** direction. Open-loop landing on the **reliable box** (column + the full
50 mm-radius ring) is **8.3 mm (column) to ≤33 mm**, *all inside the catch's
~60-80 mm reliable reach*; §3 noise adds ~2-3 mm. A real **directional
separation/aim asymmetry** (cup contact + non-y-symmetric leg layout) caps the
*clean* box at ~±70 mm — beyond ~100 mm pure ±y glue and −x/+x−y overshoot.
**Gate read:** *yes for the Rung-2b self-catch* (a column toss → 8 mm reach ≪ the
reliable reach); the full ±100 mm oval (Rung 3) needs the asymmetry resolved
first. See logbook `2026-06-30-rung2a-single-ball-tilt-throw.md`.

---

### Phase 3 / Rung 2b — Throw-and-self-catch loop  *(GATE after — MAKE-OR-BREAK)*

**Goal.** The cup throws a single ball up (tilt-aimed) and **catches its own
throw, sustained** — the minimal closed loop, with **no 2-ball confound**. This is
the cleanest test of whether tilt makes the throw→catch→throw loop **stable**
(loop gain < 1) under real contact + the §3 noise.

**Approach.** Compose Rung 2a's throw with Rung 1's catch into a single-ball
toss-catch cycle; re-plan each cycle from the achieved cup state + the noisy
observed ball. Measure: does it sustain ≥ N cycles? Does the throw still amplify
the in-cup offset, or does tilt break the amplification (launch velocity
~independent of in-cup offset)? Compare head-to-head with the pre-tilt divergence.

**Files.** `sim/juggle_selfcatch.py` (single-ball self-catch loop — the focused
harness landed in its own file, mirroring `juggle_throw.py`/`juggle_catch.py`,
rather than `sim/juggle_online.py`), measurement probe(s) under `tools/probes/`, a
logbook entry with the make-or-break verdict.

**Acceptance.** Single-ball self-catch sustains **≥ 10 cycles** (or run until it
visibly sustains vs. diverges, reporting the achieved cycle count AND the
per-cycle in-cup-offset trend — flat/decaying = converging, growing = diverging),
loop gain < 1, under the §3 noise. **GATE (MAKE-OR-BREAK):** did tilt kill the
divergence? If **yes** → Rung 3. If **no** → STOP and re-plan with the operator
(this is the riskiest assumption in the arc).

**Status (2026-07-01): implemented — gate = BREAK; STOP and re-plan.** Landed the
single-ball self-catch loop (`sim/juggle_selfcatch.py::run_self_catch`, composing
the Rung-2a throw + Rung-1 catch, re-planned each cycle), the loop-gain probe
(`tools/probes/juggle_selfcatch_loopgain.py`), and tests. **Result:** the pure
**column** self-catch does **not** sustain — it diverges within **0–3 cycles**
across all seeds and all three recover variants (the catch reach amplifies ~8 →
~50 → ~110 → ~210 mm past the catch's ~60–80 mm reliable reach → drop; loop gain
> 1, head-to-head with the pre-tilt divergence). **Two column-specific root causes
(NOT the band-limit cascade):** (1) a caught, centred, **spinless** ball does not
cleanly detach on a column — the free-fall detach (`cup_acc == g`) shares the
ball's acceleration on the same vertical line, so they ride together; Rung-2a's
column separated only via the freshly-spawned ball's incidental spin, which a
Rung-1 catch does not supply; (2) the column throw's separation is chaotically
sensitive to the ball's residual lateral state, so the catch-reach / reposition
lateral motion is amplified → the reach grows → loop gain > 1. **The reframing
(load-bearing):** for a column the commanded tilt is ~0, so the tilt mechanism
never *engages* — the column is a **degenerate** case where tilt is inactive, so
it does not test the tilt hypothesis. A stationary single-ball self-catch is
*necessarily* a column (a lateral throw walks). **Recommended re-plan:** test the
tilt hypothesis with a **two-point single-ball oscillation** (throw A→B, catch at
B, throw B→A) where tilt engages and the cup's lateral recover breaks the column
separation singularity; or fold the tilt make-or-break into a minimal-lateral
two-ball columns test (Rung 3). See logbook
`2026-07-01-rung2b-selfcatch-column-divergence.md`.

**Status (2026-07-01, OPTION-1 re-plan): implemented — gate = BREAK (still);
STOP + operator go/no-go.** Per the operator's OPTION-1 choice, added the
**two-point A↔B oscillation** (`sim/juggle_selfcatch.py::run_self_catch(oscillate=
True)`): shuttle a single ball between two lateral points so every throw is lateral
and the commanded tilt is **non-zero** (default x-40: **1.42°**, vs the column's
**0.0°**) — the honest make-or-break test on a **non-degenerate** geometry.
**Result: STILL a BREAK.** With tilt engaged, the loop diverges within **1–4
cycles** at every separation (20–70 mm) and axis (x / y / diagonal); no config
sustains near 10 (max sustained **4** of ≥ 10, at x-20, swept in the committed
probe). The **in-cup seat offset stays small** (~0.5–2 mm — tilt *does*
centre the ball), but the **landing amplifies** (default x-40 seed 1: landing error
**3.7 → 89 → 728 mm**). **Root cause:** the tilt-aimed throw's landing is
**chaotically/deterministically sensitive to the throw-ORIGIN pose** — a 10 mm
origin shift swings the landing ~40 mm (`dLanding/dOrigin` ≈ 4, up to ~11 with sign
changes) — the **contact-detach knife-edge** Rung 2a flagged and characterised only
*from the origin*; the oscillation throws from *off-origin* A/B where it is
loop-fatal. **The reframing (load-bearing):** tilt fixes the **band-limit** (the
pre-tilt divergence's mechanism) but **not** this pose-chaos, which is the binding
amplification off-origin — so **the tilt re-architecture is necessary but not
sufficient** to close the throw→catch→throw loop. OPTION-1's premise (that the
oscillation inherits Rung-2a's reliable box) is **partly wrong**: that box is a
*from-origin* property and does not transfer to the off-origin throws an
oscillation requires. **Next lever:** the contact-detach knife-edge (the
throw-origin pose-sensitivity / non-y-symmetric leg asymmetry Rung 2a deferred to
Rung 3), explicitly **not** the aim (exact) and **not** the tempo (slower is
worse). See logbook `2026-07-01-rung2b-oscillation-tilt-engaged-diverges.md`.

**Status (2026-07-01, kinematic-release re-architecture): implemented — gate = MAKE.**
Per the operator's Path-A choice, attacked the contact-detach knife-edge at its source.
First **ruled out contact-parameter tuning**: a release-contact frontier scan (`solref`
time-const 4–10 ms × `solimp` `dmin` 0.90–0.99, noise off) found **every** separating
config has `dLanding/dOrigin` **2.4–16** (all ≫ 1); softer settings glue the ball
(a colleague's `solref 0.01 1.5`/`solimp 0.9 0.95 0.001` made it ride the cup out,
~110 mm off). The pose-sensitivity is intrinsic to the contact-carry detach, at **zero
noise** — so the §3 noise is **not** the driver. Then landed the fix: a **kinematic
velocity release** (`Ball.ballistic_release` — cut the ball free at the planned
`v_takeoff`, breaking hand contact for a clean separation, then hand back to the
contact-carry seat), which drops the open-loop knife-edge gain **2.7 → ~0.05** (accuracy
21.5 → 5.1 mm). Wired into `sim/juggle_selfcatch.py` as `release="kinematic"` (default
`"detach"` stays byte-identical, so tilt=0 / Rung-1 / Rung-2a / the two documented
BREAK configs are unchanged). **Result: MAKE.** The default A↔B oscillation (x-40, tilt
~1.4°) sustains **12/12 cycles on all 6 seeds** with a **flat ~0.6 mm in-cup offset**
(the real contact-carry seat offset, REJECTED → loop gain ≪ 1), head-to-head vs the
detach path's mean **1.2/12** (same geometry/seed). De-risked first by an all-kinematic
loop (12/12, ~2.4 mm flat). The tilt + kinematic-release architecture **closes the
single-ball self-catch loop** → proceed to Rung 3. Fidelity caveat: the kinematic
release idealises a clean separation (stops resolving the final detach-contact
millisecond) — a sim-2-real question flagged for hardware bring-up; the catch stays
fully contact-physical. See logbook `2026-07-01-rung2b-kinematic-release.md`.

---

### Phase 4 / Rung 3 — Two-ball (columns → oval, BB-seeded)

**Goal.** Sustained 2-ball — compose the proven catch + throw primitives. BB seeds
the pattern (tilt-to-receive); bring up **columns first** (lowest lateral demand,
≥30 catches / 0 drops under noise), then **ramp tilt + separation toward the
oval**.

**Approach.** Carry-inward catch geometry (Kai's catch-outer → carry-inward →
throw-inner, firmer collinear catch) now that catch + throw are solid; BB priming;
columns→oval as one parameter ramp. Retarget the headline tests.

**Files.** `sim/juggle_online.py` (BB seeding + columns/oval parameterisation),
`tests/sim/test_demo_juggle_sim.py` (retarget `test_full_sim_juggle_reaches_
target_catches`, currently `xfail(strict=True)`, and re-point the passing smoke
test `test_short_run_catches_the_bb_primed_ball_and_a_throw`, both at the online
runner), logbook.

**Acceptance.** ≥30 catches / 0 drops sustained under the noise (columns first),
BB-seeded; the headline test passes un-xfailed; a demonstrated parameter ramp
toward the oval.

---

### Rung 4+ (future, not yet scoped)

3-ball cascade, richer / arbitrary-target patterns, longer runs. Out of scope
until Rung 3 lands; noted so the ladder framing is explicit.

## 3. Cross-cutting constraints (every rung)

### Hardware-faithful noise model (NEW — configurable, on by default)

Two independent, seed-reproducible noise sources, exposed as config knobs so any
rung can sweep/disable them:

- **BB throw noise** — `bb_throw_noise_frac` (default **0.02** = 2%). The BB
  ball's initial conditions get Gaussian noise converted to physical units:
  velocity σ = `frac × |vel|` per component (≈2% of the throw speed, in mm/s),
  position σ = `frac × |throw displacement vector|` per component (2% of how far
  the throw travels, origin→touch-down, in mm) — both anchored to the one
  `frac` knob, no free reference. Models the real Ball Butler's shot-to-shot
  variability.
- **Tracking (observation) noise** — `tracking_noise_mm` (default **0.5**). Every
  *observed* ball position used for re-planning gets ±0.5 mm noise (σ or half-
  range — implementer's call, documented). Models mocap/sensor noise. NB the
  velocity estimate (position differencing) inherits amplified noise — handle
  explicitly (e.g. observe position, derive velocity, or a small filter).

Both default ON; every rung's acceptance must hold *with* them. Seed everything
for determinism (the runner already takes `seed`).

### Engineering constraints

- **Venv:** `source ~/Desktop/PDJ_venv/venv/bin/activate` for all python/pytest.
- **Control rigor:** analyse control/physics implications before changing the
  planner/realisation (tilt touches throw + catch kinematics — reason through one
  cycle first).
- **Tests:** `pytest tests/ -q` after `*.py`/`*.yaml` and as the final pre-commit
  gate; cite the (date, command, result) triple. The flaky
  `test_hot_loop_allocation_contract` passes isolated — re-run isolated to confirm,
  never block on it; any *other* failure blocks the commit.
- **Reusable probes** → `tools/probes/` (committed) + README entry; one-offs →
  `/tmp`.
- **Logbook** per substantive rung, real Discussion section; backfill the SHA.
- **No backticks in `git commit -m`** (heredoc/-F); `/audit --unstaged` before any
  commit touching a logbook/plan/normative .md.

## 4. The workflow + gates

Driven by `.claude/workflows/bb-tilt-phases.js`, invoked **by `scriptPath`** (local
workflows don't resolve by `name`):
`Workflow({scriptPath: ".claude/workflows/bb-tilt-phases.js", args: {from, to}})`.
Each phase = fresh **implement → independent audit-reporter → finalize** (apply
fixes → full `pytest tests/ -q` gate → commit → **fetch-guarded push** (abort + warn
on divergence with `origin/demo/bb-led-two-ball-juggle`) → write the next handoff).

**Phase ↔ rung map and gates.** The operator wants **explicit per-rung gates**
(incl. the Rung 2a/2b sub-gates), so run **one phase per segment**:

| Invoke | Phase / Rung | Gate question |
|---|---|---|
| (done) | 0 / Rung 0 | tilt-tracking good? → YES |
| `{from:1,to:1}` | 1 / Rung 1 | catch clean + noise-robust? |
| `{from:2,to:2}` | 2 / Rung 2a | throws accurate to target + cadence? → tilt-aim exact; reliable box (column + 50 mm ring) ≤33 mm (within catch reach); ±100 mm directional asymmetry — **gate pending** |
| `{from:3,to:3}` | 3 / Rung 2b | **make-or-break:** self-catch sustains (loop gain < 1)? → **MAKE (kinematic-release re-arch).** First two attempts BREAK: column diverges with tilt≈0 (degenerate); OPTION-1 A↔B oscillation (tilt **engaged** ~1.4°) STILL diverges (loop gain > 1) — the throw's contact-detach knife-edge (`dLanding/dOrigin` ≈ 2.7) dominates off-origin, so tilt is **necessary-but-not-sufficient**. **Fix:** contact-parameter tuning ruled out (frontier scan: gain 2.4–16 everywhere separation holds), then a **kinematic velocity release** (`Ball.ballistic_release`, open-loop gain 2.7 → ~0.05) + a clean contact-carry↔kinematic hand-off. **Result: 12/12 cycles on all 6 seeds, flat ~0.6 mm in-cup offset** (real seat offset rejected, loop gain ≪ 1), vs detach's mean 1.2/12 → **the loop closes; proceed to Rung 3** |
| `{from:4,to:4}` | 4 / Rung 3 | 2-ball sustained ≥30? |

(Phases may be batched into a segment, e.g. `{2,3}`, if a gate is waived — but the
default is one-per-segment given the sub-gate decision.)

## 5. References

- Kai Ploeger, *Controlling the Cascade* (IEEE 9981678); repo
  `task_space_juggling.py` — `plan_throw`, `HAND_TILT`, tilted detach, collinear
  catch, carry-inward geometry, the online re-plan loop.
- Rung 0: `tools/probes/juggle_tilt_bandlimit.py`, logbook
  `2026-06-29-platform-tilt-tracking-characterisation`.
- Band limit: `tools/probes/juggle_cup_bandlimit.py`, logbook
  `2026-06-27-online-replanning-architecture-and-cup-bandlimit`.
- The wall: logbook `2026-06-27-online-juggle-throw-fix-catch-axis-split-band-limit-cascade`.
- BB priming + offline-demo tilt reference: `sim/juggle_demo.py`,
  `controller/demo/juggle_optimizer.py`, plan `bb-led-two-ball-juggle-demo.md`.
