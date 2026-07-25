---
title: "Single-ball toss Phase 4: Tier 8b — tilt-aimed displaced throw on the production stack"
type: feature
date: 2026-07-25
status: resolved
phase: "MVP trajectory bringup — Phase 8 / single-ball toss Phase 4 (Tier 8b displaced throw)"
related_plan: single-ball-toss.md
subsystem: ros
tags: [feature, control, sim, testing]
commits:
  - PENDING
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/tilt_geometry.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/__init__.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/toss_release.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
  - sim/juggle_tilt.py
  - sim/toss_gate.py
  - config/hardware_config.yaml (+ regenerated artifacts)
  - tests/motion/test_trajectory_tilt_geometry.py
  - tests/motion/test_toss_release.py
  - tests/sim/test_juggle_tilt.py
  - tests/sim/test_toss_gate.py
  - tests/ros/test_toss_sequencer.py
  - tests/ros/test_toss_coordinator.py
  - tests/ros/test_toss_integration.py
  - tests/ros/test_catch_coordinator_node.py
---

# Single-ball toss Phase 4: Tier 8b — tilt-aimed displaced throw

> **ros_ws changes** — running any of this on the robot requires
> `colcon build --packages-select jugglebot_interfaces jugglebot` and a
> relaunch. Tier 8b ships **behind config** (`JB_OP_TOSS_TIER` defaults
> `'8a'`); the 8b path is exercised only under a config override until the
> Phase-5 T4 bring-up.

## Summary

Tier 8b — throw from a config site A, tilt-aimed at a nominated catch B up
to 70 mm away, catch at B through the unchanged catch path — lands
software-complete behind `JB_OP_TOSS_TIER='8b'`. `tilt_to_throw` is ported
to the production `tilt_geometry` module (a one-line mirror of
`tilt_to_receive`, sim re-exports it); `toss_release.compute_release_state_tilted`
computes the swing-compensated pre-tilt pose at A + the tilt-aimed launch
with a **loud clamp gate** (`ThrowTiltInfeasible`, never a silent
saturation); the coordinator pre-positions *tilted at A*, raises the new
`catch/pretilt_hold` gate, and publishes the ONE deferred A→B reach at
release; the sim gate grows a displaced binding ring + a directional
asymmetry map. Config ships `'8a'` — every prior tier and reload path is
byte-identical.

This phase was **salvaged**: the Fable-5 build workflow hit a spend limit
and died after landing only the port + math (Unit A) and a *partial,
non-functional* coordinator; the choreography wiring, the safety-critical
`catch/pretilt_hold` subscriber gate, and every Tier-8b test were absent,
and the tree carried a latent `None`-deref. The salvage finished the code
and tests; the adversarial review (which had also died) then caught a
**BLOCKING** production gap the mocked tests and the sim gate structurally
could not see. Both are documented below. Built during the 2026-07-25
unattended workflow-orchestrated run (the Tier-8a phases 0–3 landed on
Fable 5; Phase 4's salvage + review + fixes ran on Opus 4.8 after the
Fable-5 spend limit).

## Changes

- **`motion/trajectory/tilt_geometry.py`** — `tilt_to_throw(v_takeoff,
  max_tilt_deg=MAX_TILT_DEG)` = the exact mirror `tilt_to_receive(-v)`
  (bodies character-identical to the sim copy); exported from the package
  `__init__`. `sim/juggle_tilt.py`'s definition becomes a re-export of the
  production symbol (single source of truth; the sim mirror test cross-checks
  production-throw vs sim-receive as the drift guard). The catch-side
  `tilt_to_receive` was already ported (Phase 6); this closes the throw side.
- **`toss_release.py`** — `TiltedReleaseState` (frozen, subclasses
  `ReleaseState` so `build_announcement_fields` rides through unchanged;
  landing = B's cup point, global); `ThrowTiltInfeasible` (the loud clamp
  gate — raises with `required_deg`/`max_tilt_deg` when the from-vertical aim
  exceeds 12°, which needs ~315 mm of displacement and so never binds inside
  the workspace); `compute_release_state_tilted` (level ballistic inverse
  A→B cup point → clamp gate → `tilt_to_throw` → one fixed-point pass over
  the arm·(1−cosθ) vertical drop, converging <0.1 mm — → the
  swing-compensated `pretilt_pose_stow` + `tilt_rx`/`tilt_ry`). Bitwise
  degenerate to `compute_release_state` when the throw site equals B.
- **`toss_sequencer.py`** — the Tier-8b FSM path: CHECKING gates
  `REJECTED_DISPLACEMENT` (|B−A| past the 70 mm cap OR the closed-form
  quintic reach bound over the flight) and `REJECTED_TILT_CLAMP` (via the
  node-fed `tilt_clamp_exceeded` flag); `ACTION_REACH_CATCH` emitted exactly
  once, **time-triggered at `t_release`** (never evidence-triggered — release
  evidence can lag the 0.5 s grace and eat the flight), never for 8a
  (byte-identical decision stream); the announce-lead re-scope (see
  Discussion). New `TossSequencer` kwargs `throw_site_xy_mm`,
  `tilt_clamp_exceeded`.
- **`reload_coordinator_node.py`** — the 8b choreography: pre-position
  *tilted at A* (`_toss_positioning_xyz` sources the go_to_pose position from
  `pretilt_pose_stow` and the orientation from `(tilt_rx, tilt_ry, 0)` via
  `rotvec_to_rot_matrix → rot_matrix_to_quat`, the exact inverse of
  `trajectory_node`'s go_to_pose decode); raise `catch/pretilt_hold` True on
  the PREPARE tick **for 8b only**; `_publish_toss_reach` publishes the ONE
  deferred `catch/dynamic_target` for B at `t_release` (built from the same
  `CatchCoordinator.predicted_catch_command` math CCN uses, single-sourced;
  arrival = the announced landing ⇒ lead = flight); release `pretilt_hold`
  LAST at both terminals iff raised. `_toss_positioning_xyz` is the **single
  source** for both the commanded pose and the mocap cross-check target, so
  they can never diverge. Tier 8a never touches the new topic.
- **`catch_coordinator_node.py`** — the `catch/pretilt_hold` subscriber gate
  (the D2 pattern, additive, default-off): while True, `_on_throw_announcement`
  latches the announcement (`_announcement_seen`/`_announced_landing_time`
  for the hand-arm window) but publishes NO platform target and caches
  `_pretilt_cmd=None`; `_on_balls` treats `pretilt_hold` as forcing the
  open-loop (no reactive platform publish) branch **independent of the
  reload open-loop flag** (the review fix). The catch ARM and every other
  behaviour are untouched; absent-topic behaviour is bit-identical.
- **`sim/toss_gate.py`** — the Tier-8b trial flow (pre-position tilted at A,
  tilted kinematic release, the hold→reach plan swap inside one pump at
  `t_release` keeping the handoff-continuity assertion, catch at B); the
  displaced binding ring `passed_8b_ring` (centre + 50 mm ring ×8 at T=0.80,
  `core_clean ≥ ⌈0.9 n⌉`, geometric membership, `catch_armed` required, no
  vacuous pass); the directional **asymmetry map** in the non-gating detach
  diagnostic column (8 dirs × {70, 100} mm × {0.60, 0.80} s, `seated_n` + landing-error-vs-B
  per cell — honestly labelled: it reports post-catch seat + landing error,
  not the Rung-2a throw-glue, which is not derivable from the existing trial
  data). The two Tier-8a bands stay binding and untouched.
- **Config** — `toss_throw_site_mm` `[0.0, 0.0]` → `JB_OP_TOSS_THROW_SITE_MM`;
  `JB_OP_TOSS_TIER` stays `'8a'`. Artifacts regenerated.

## Sweep result (the Phase-4 gate)

`python sim/toss_gate.py --tier 8b` (seed 0, run 2026-07-25; 656.6 s wall;
report `temp/reports/toss_gate_seed0_n190.json`). Headline: **PASS** — all
three binding bands pass (the two carried-forward Tier-8a bands + the new
8b displaced ring); only the advisory full band fails on the long-flight
tail. `core_clean` 181/190 overall, `caught` 181, `accepted` 190/190,
`feas_viol` 0, `pump_rejects` 0; worst hold travel 0.02 mm / tilt 0.01°;
required leg envelope vel 80 mm/s, acc 330 mm/s², jerk 4528 mm/s³.

**Displaced binding ring (kinematic column, T=0.80, 50 mm from centre — the
certification):**

| Ring point (x, y) | displacement | core_clean |
|---|---|---|
| (0, 0) centre | 0 mm | 9/10 |
| (+50, 0) | 50 mm | 9/10 |
| (±35.4, ±35.4) ×4 diagonals + (0, ±50), (−50, 0) | 50 mm | 10/10 |

All 9 ring points pass ≥9/10 ⇒ `passed_8b_ring_band` PASS. Advisory fails
(non-binding, T=0.95 tail): (+50, 0) 7/10 and (0, +50) 8/10 — the same
long-flight placeholder-noise signature as the Tier-8a sweep.

**Directional asymmetry map (detach diagnostic column, non-gating,
`seated_n`/4 = post-catch seat; this is the contact-detach knife-edge, a
hardware-risk map, NOT a sim pass/fail). Run at two flights — the table below
is the T = 0.80 s sub-map (100 mm physically wants the longer flight; T4-at-100
guidance is based on it):**

| Direction (T = 0.80 s) | 70 mm | 100 mm |
|---|---|---|
| +x (E) | 4/4 | **0/4** |
| +x+y (NE) | **0/4** | 4/4 |
| +y (N) | **0/4** | **0/4** |
| −x+y (NW) | **0/4** | **0/4** |
| −x (W) | 4/4 | 4/4 |
| −x−y (SW) | 4/4 | 4/4 |
| −y (S) | 4/4 | 4/4 |
| +x−y (SE) | 4/4 | 4/4 |

Landing error for the seated cells is ≤2.1 mm everywhere (aim is excellent
where the ball seats; the failures are seat/detach, not aim). The pattern is
a strong **+y-hemisphere weakness**: the whole −y hemisphere and −x seat
cleanly out to 100 mm, while +y (N) and −x+y (NW) fail even at 70 mm and the
+x/NE cells flip between radii. This is the Rung-2a directional asymmetry
reproduced on the production stack, and it is the data the plan's T4-at-100 mm
fork needs: a first 100 mm hardware attempt should aim into the −y hemisphere
or −x (robust to 100 mm in sim), never +y/NW (fails at 70 mm). The shipped
70 mm cap keeps every gated toss inside the clean-in-most-directions box; the
map is why 100 mm is not shipped un-gated. The map was also run at **T = 0.60 s**
(below the 0.7 s hardware flight floor): it seats far worse everywhere — only
−x and +y seat at 70 mm, and the whole −y hemisphere flips to 0/4 — so the
T4-at-100 aim guidance is drawn from the T = 0.80 s sub-map only, not the short
flight (which is not a T4 candidate regardless).

## Discussion

**The salvage.** The Fable-5 build workflow died on a monthly spend limit
after Unit A (port + math, fully tested) and a *partial* coordinator: the
imports, the `CatchCoordinator` policy object, the `catch/pretilt_hold` +
`catch/dynamic_target` publishers, and the tilted-release branch had landed,
but the actual choreography — the `ACTION_REACH_CATCH` dispatch branch, the
`_publish_toss_reach` method, and any `catch/pretilt_hold` publish (True or
False) — was never written, and the `catch_coordinator` subscriber gate was
absent entirely. An 8b goal in that tree would have silently no-op'd the
reach AND left the central hazard unmitigated; the tree also carried a latent
`None`-deref in the tilt-clamp path (the salvage's tests surfaced it, fixed
same-session). The `toss_sequencer` 8b FSM, by contrast, had landed complete
and coherent. The salvage verified this state independently before finishing
by hand, per the workflow-death-salvage discipline.

**The BLOCKING gap the adversarial review caught — why it was invisible
otherwise.** After the salvage finished the code (302 ros + 27 sim tests
green), the re-run adversarial review found that `_position_platform_for_toss`
commanded the *level* pose at B for **all** tiers: the computed tilted
pre-tilt pose at A (`pretilt_pose_stow`/`tilt_rx`/`tilt_ry`) was **orphaned**,
with zero consumers in `ros_ws` outside the module that computes it. An 8b
hardware toss would have pre-positioned level at B and thrown straight up —
the entire displaced-throw purpose silently absent. Two layers of testing
missed it: the mocked node tests monkeypatched `_position_platform_for_toss`
(so none exercised the pose it *sends*), and the **sim gate models the
correct pre-tilt-at-A choreography directly**, so its green certification
actively concealed the production-node gap. This is the canonical value of an
independent adversarial pass over a real diff: a gap that is invisible to
both the unit tests and the integration sim, because both encode the intended
behaviour rather than reading what the production node actually commands. The
fix wires the tilted A pose (position + orientation) into the go_to_pose
request for 8b, byte-identical for 8a, pinned by a test that drives the real
method (no monkeypatch) and round-trips the orientation back to
`(tilt_rx, tilt_ry, 0)`.

**The `catch/pretilt_hold` gate and why the stock pre-tilt cannot serve 8b.**
The hardware-proven catch path pre-tilts the platform on the announcement,
with the pre-tilt's arrival clamped to ~now + 1 s. A toss announces ≥1 s
*before* release. So an un-suppressed stock pre-tilt would translate the
platform A→B and un-tilt it to the *receive* tilt **before the ball is
released** — aim destroyed, a moving platform under a seated ball mid-windup.
The gate (mirroring `catch/prime_hold`) latches the announcement for the
hand-arm window but suppresses every platform publish for the goal's
duration; the coordinator instead publishes the ONE deferred A→B reach at
`t_release`, announcement-derived (so tracking still moves only the hand —
the open-loop pivot holds). The review hardened the suppression to be
self-contained (it forces the open-loop branch regardless of the reload
`JB_OP_RELOAD_PLATFORM_OPEN_LOOP` flag), closing a config-coupling where an
8b toss run with open-loop off would have let the reactive per-ball path
compete with the deferred reach.

**Announce-lead re-scope (a superseded promise, documented not silently
dropped).** The Phase-1 FSM docstring promised "Tier 8b MUST harden the
announce-lead WARN to an abort." Under the deferred-reach design that
constant no longer sizes any 8b platform motion (the reach lead = the flight
time by construction, not the announce lead), so the promise is superseded —
hardening it would only abort every floor-delay 8b toss while protecting
nothing. The load-bearing 8b gates are `REJECTED_DISPLACEMENT` and
`REJECTED_TILT_CLAMP` instead. The docstring and the sequencer comment were
updated to state the supersession rather than leave a stale promise.

**Why `Ball.release` / the port re-export / the 70 mm cap** — carried from
the Phase-2 entry and the Phase-4 spec: the gate's kinematic column uses the
mode-correct ejector; the sim re-exports the production `tilt_to_throw`
rather than keeping an independent copy (the cross-branch split that forced
independent copies on the catch side is now merged, and sim→production import
is already load-bearing); the 70 mm displacement cap is the clean Rung-2a box
∩ the 80 mm reach envelope (captured at A), needing no `trajectory_node`
change and no envelope dilution. The plan's T4-at-100 mm remains an explicit
operator decision after the asymmetry map (options: envelope config raise —
dilutes the global gate; the safety-critical envelope-center-at-B node change;
or T4 re-staged to ≤70 mm).

**Known limitations / follow-ups (documented, deliberate):**
- **The `pretilt_hold`-before-announcement ordering** is guaranteed only by
  the ~2-tick temporal margin (separate topics, no cross-topic transport
  ordering). No test can prove the real-graph ordering; the Phase-3 Tier-8b
  real-ordering dry-trace addendum must confirm `pretilt_hold` reaches
  `catch_coordinator` before the announcement **before T4**.
- The asymmetry map reports post-catch seat + landing-error-vs-B, not the
  Rung-2a throw-glue (no per-trial glue signal exists in the detach column);
  the field is honestly named `seated_n`.
- Tier 8b stays behind `JB_OP_TOSS_TIER='8a'`; the locked `Toss.action` goal
  has no throw-site or tier field, so A comes from config
  (`JB_OP_TOSS_THROW_SITE_MM`) and the tier from config — the plan's
  "per-goal override" remains unimplementable without a goal-schema change
  (a standing operator decision, unchanged from Phase 1).

## Verification

- Scoped (2026-07-25, Jetson venv, final tree):
  `pytest tests/ros/test_toss_coordinator.py tests/ros/test_reload_coordinator_node.py -q`
  → 128 passed; the wider ros toss/reload/catch scope → 302 passed at the
  post-salvage tree; `pytest tests/sim/test_toss_gate.py
  tests/sim/test_reload_gate.py -q` → 27 passed; `pytest tests/motion/ -q`
  → 762 passed (Unit A). py_compile clean under system Python 3.8.
- Full Tier-8b sweep — `python sim/toss_gate.py --tier 8b`, seed 0, run
  2026-07-25: **PASS** (all binding bands; `core_clean` 181/190; 0
  feas_viol; 0 pump_rejects; 656.6 s wall; report
  `temp/reports/toss_gate_seed0_n190.json`). Table + asymmetry map above.
- ci-deep (Phase 4 exit gate) —
  `pytest tests/ -q --hypothesis-profile=ci-deep`, run 2026-07-25 against
  the final post-review-fix tree: **3397 passed, 3 xfailed, 198 warnings in
  3999.28 s (1:06:39)** (+67 toss-8b tests over the 3330 Phase-3 baseline;
  same 3 documented xfails). Collection: 3400 tests, zero import errors.
- `/audit` (pre-commit, 2026-07-25): **CLEAN** — no BLOCKING/WARNING. The
  auditor independently re-verified the safety gate (both CCN platform-publish
  paths suppressed under hold; released on every exit path via
  `_safe_toss_on_early_exit`; `seq.prepared` guarantees the release-on-exit),
  the tilted-positioning fix (real-method-driven test, A-pose + tilt round-trip),
  8a/reload byte-identity, the no-z-double-add invariant, every sweep number
  against the JSON, and re-ran the three scoped triples (128/27/762 exact). One
  NOTE fixed-and-landed: the asymmetry-map table showed the T=0.80 s sub-map
  unlabelled and omitted the T=0.60 s sub-map (which seats far worse — the −y
  hemisphere flips to 0/4 at 70 mm); the table is now flight-labelled and the
  T=0.60 s sub-map is characterised, so the operator T4 guidance can't be
  mis-paired with a short flight.

## Closing run summary (the 2026-07-25 unattended workflow-orchestrated run)

This is the final entry of the run that executed
`plans/active/PROMPT-single-ball-toss-software-run.md` (operator-authorized
2026-07-24). Phases 0, 1, 2, 4 built + reviewed + gated + committed + pushed;
Phase 3 built prep-only; Phase 5 untouched.

**What landed (each committed + pushed, suite-green, own logbook entry):**
- **Phase 0** (`c1d0e66`) — post-merge doc reconciliation: 4 INDEX backfills,
  `JUGGLE_DEMO.md` banner, bb-led plan note.
- **Phase 1** (`5447f03`) — `Toss.action`, the `toss_sequencer` FSM,
  `motion/trajectory/toss_release.py` (the one STOW→global conversion point +
  ballistic inverse), the ball-ops coordinator (extended
  `reload_coordinator_node`, measured 76.5% shared surface), the
  `catch/prime_hold` gate. Tier 8a.
- **Phase 2** (`4987b6a`) — `sim/toss_gate.py` production-in-the-loop gate +
  `gate_common` extraction; the Tier-8a sweep PASSED both binding bands.
- **Phase 3 PREP** (`2338b0b`) — the real-ordering trace recorder + invariant
  checker + operator runbook + the committed 21-case synthetic probe;
  amended the plan's unreachable "unpowered bench" to a powered no-ball bench.
- **Phase 4** (this entry) — Tier 8b, ships behind config.

**Audit / review findings fixed-and-landed across the run** (the adversarial
panels + `/audit` earned their cost repeatedly — several were BLOCKING gaps
invisible to the unit tests):
- Phase 1: a BLOCKING mocap-frame bug the unit tests had *enshrined* (the z
  double-add re-introduced in the verification path), phantom-track
  poisoning, NaN-goal detonation after arming, prime_hold/armed reorder,
  stroke-signature 15→40 rev/s.
- Phase 2: a BLOCKING silent-dead-arm `core_clean` (a ball into a
  statically-parked cup gate-passed), vacuous empty-band PASS, the pump's
  silent `(None,None)` making the knot-acceptance invariant vacuous, plus
  three wrong narrative numbers.
- Phase 3: an empirically-probed checker hole (DT-5 passed an inverted
  trace); the powered-bench plan amendment.
- Phase 4: a BLOCKING orphaned tilted-positioning gap (invisible to both the
  mocked tests and the sim gate), the open-loop-flag-coupled suppression, the
  paired mocap-target divergence, an honest asymmetry-map field rename.

**Deliberately NOT done (out of the run's charter or a standing operator
decision):**
- **Phase 3 live captures** — operator-run; the harness + runbook are ready.
- **Phase 5 hardware (T0–T4)** — entirely operator-run; T0 measures the real
  release scatter that re-runs the gates (the sim noise is a 1 % placeholder).
- **T4-at-100 mm** — the shipped 8b cap is 70 mm; 100 mm is direction-gated by
  the asymmetry map and needs an operator decision (envelope change vs
  re-stage).
- No goal-schema change (the locked `Toss.action` has no tier/throw-site
  field, so both come from config) and no `trajectory_node` change.

**What a fresh reader needs before the next hardware sitting:**
1. `colcon build --packages-select jugglebot_interfaces jugglebot` + relaunch
   (Phase 1 added an interface + Phase 4 touched `ros_ws`; the launch runs the
   installed copy).
2. Reboot the can-bridge Teensy before the sitting; log `uptime_ms` with every
   timing measurement (the standing tracking-lag-vs-uptime hazard).
3. Run `tests/hardware/session_phase8_toss_trace.md` (the dry-trace + the
   un-waived `REJECTED_NO_BALL`) and resolve the `toss_mocap_body` QTM
   platform-body name — BEFORE any ball. For 8b, that trace needs the
   **`pretilt_hold`-before-announcement ordering addendum** (the one thing no
   test can prove).
4. T0 first (release characterisation) — every gate re-runs with the measured
   noise; the sim's placeholder is explicitly not hardware truth.
5. The open Phase-7 hazards carry forward: the ERR_TIMEOUT dispatch epidemic
   (the tri-state ladder is the mitigation, never blind-re-dispatch), tracker
   verdict corruption (judge CAUGHT by eye + tracker id), and a tracker
   *liveness* gap the toss CHECKING does not cover (a dead tracker passes
   every precondition — verify tracker liveness pre-session).
