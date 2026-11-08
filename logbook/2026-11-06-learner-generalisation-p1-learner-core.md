---
title: "Learner generalisation P1: the Learner core is a global plant model (one apex gain, a lateral offset and a centre-pull, by recursive least squares) plus a residual memory keyed on release position and velocity, with an evidence-scaled cap about the model command; a replay of the 3276-row corpus first showed which parts of the plan's law survive"
type: feature
date: 2026-11-06
status: in-progress
phase: "learner-generalisation P1 (Learner core) - on branch p3-online-admissibility, held for the owner's merge"
related_plan: learner-generalisation.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/plant_model.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/learner.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/memory.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - sim/skills_gate.py
  - tools/migrate_memory_schema_v2.py
  - tools/migrate_memory_catch_plane.py
  - tools/probes/learner_replay.py
  - tools/probes/learner_replay_old_law.py
  - tools/probes/learner_replay_production.py
  - tools/probes/README.md
  - tests/motion/test_plant_model.py
  - tests/motion/test_learner.py
  - tests/motion/test_memory.py
  - tests/motion/test_migrate_memory_schema_v2.py
  - tests/motion/test_migrate_memory_catch_plane.py
  - tests/motion/test_learner_replay.py
  - tests/motion/test_skills_executor_learner.py
  - tests/motion/test_skills_executor.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_skills_plan_bench.py
  - tests/sim/test_skills_gate.py
  - tests/hardware/skills_plan_bench.py
  - plans/active/learner-generalisation.md
subsystem:
  - motion
  - ros
  - sim
  - tools
tags:
  - learning
  - testing
  - performance
---

# Learner generalisation P1: the Learner core

## Summary

The Learner no longer starts every new Site from the identity prior. A global plant model (one apex
gain, a lateral offset and a centre-pull, fitted by recursive least squares over every row) gives the first
Throw at a new Site the correction the whole memory supports; a residual memory keyed on release position
and velocity, with exact match on ball and release kind, carries what the model does not; and the
Learner's authority is a cap about the MODEL command that grows with the evidence at the query. A replay of
the 3276-row corpus came first and changed six things in the plan's law. Through the production classes
the cold-start apex error falls from 74.5 mm to 12.9 mm (mean over first Throws at Sites with any earlier
row; the plan's gate is under 15 mm), steady state is slightly better than the old law on all three axes,
and the share of rows pinned at the authority falls from 28.8 % to 0.5 %. The memory is schema v2 in a new
file; a v1 memory with no v2 file refuses the goal with the migration command. Built on branch
`p3-online-admissibility`, held for the owner's merge; no robot motion, and the live memory was never
opened.

## Context

Plan `plans/active/learner-generalisation.md` §§ 2.2-2.4 and § 4 Phase 1; owner decision D1 (2026-10-18):
retire the paper's local-linear fit for a global plant model plus a fixed-Jacobian residual. Design record:
`logbook/2026-10-18-learner-generalisation-investigation.md`, whose regression was on the 846-row corpus of
that day. The corpus the Learner has now is 3276 rows over 39 Sittings (2026-09-28 to 2026-11-03), nearly a
third of them oval throws that did not exist on 2026-10-18.

The phase's first rule is "probe FIRST": a replay probe decides the constants, and the plan names them as
the probe's outputs (the apex c_max, lambda, h_p, h_v, h_d, w0). The owner was unreachable for this phase;
the decisions below were taken from the probe's numbers and are listed for review under Open Questions.
Nothing here is on `main`.

## Probe first: `tools/probes/learner_replay.py`

The probe replays a corpus in time order and scores each law as a PREDICTOR: for every row, the law is
built only from earlier rows (rows up to i - 2, as the live loop sees them, plus row i - 1 when it belongs
to an earlier Sitting) and predicts where that row's actual command landed. The corpus was flown under the
old law's commands, so a new law's own commands cannot be observed; the predictor form is the 2026-10-18
investigation's method. The old law is today's production functions, called, not re-implemented. Inputs
are copies of the committed corpus (`temp/learn/jugglebot/memory.csv` in the worktree, 3276 rows) and of
the 846-row file at commit 8f9496ff; the live memory was never opened.

Commands (repo root, venv python, 2026-11-06; 19 s on 846 rows, 69 s on 3276):
`OPENBLAS_NUM_THREADS=1 python tools/probes/learner_replay.py --csv temp/probes/learner_replay/corpus_{846,3276}.csv --tag {846,3276} --rule band+struct --lam 0.99`
and the residual-memory grid, one process per k (about 70 s each):
`... --sweep-only --rule band+struct --lam {0.99,0.95} --sweep-k {8,16,32}`.

**What it found**

| Question | Answer |
|---|---|
| Do the plan's constants reproduce on the 846 rows? | Yes: K = 1.0852 (plan 1.085), pooled offset +9.9 / +15.0 mm, s = -0.0915 per m. The ball-1 offset is +2.3 / +16.7 mm once s is in the model; the plan's +16 / +16 holds only without s. |
| Does ONE gain describe all 3276 rows? | Yes: K = 1.0809; per Sitting 1.070-1.095 (sd 0.006), per Site 1.070-1.090, 1.078 on oval carried rows against 1.082 on vertical ones, flat over apex 0.8-1.4 m. The +1 % drift seen in October did not continue (-0.0002 per day). A pure gain extrapolates above 1.0 m better than an affine fit (31.5 against 44.2 mm RMS). |
| Is the lateral offset global? | No: b_x runs from -2 to +18 mm by Site. A global b with a centre-pull is a prior; the residual memory carries the rest. |
| Replay, RMS mm (axis 0 / axis 1 / apex), 3233 inliers | identity 24.5 / 23.3 / 74.2; old law 20.6 / 20.1 / 15.1; f0 alone 21.1 / 19.9 / 14.3; f0 + residual 20.0 / 19.7 / 13.4. Steady state (sixth throw on): old 19.9 / 19.7 / 14.4, new 19.7 / 19.5 / 13.7. |
| Cold start: the first Throw at a new Site | apex error 73.5 mm under the identity prior, 59.3 mm under the old law, 17.8 mm under f0 (7 of 10 under 15 mm); 12.5 mm without the two rows that had no earlier data at all. Lateral 27.0 mm against 31.7 mm for the identity: not worse. |
| Rows pinned at the lateral authority | 28.8 % under the old law's 20 mm clamp; 0.19 % at the probe prototype's cap (0.49 % through the production classes, below). |
| Which constants? | The residual surface is flat (summed MSE 969-1003 mm^2 over 243 settings); its shallow best is k = 8, h_p anywhere in 0.02-0.10 m, h_v = 0.15 m/s, w0 = 4, and the dwell bandwidth has no effect. The residual's apex corrections reach p99 12.9 mm, max 18.0 mm. (k and w0 were then set from the closed loop, not from this table: see Tradeoffs.) |
| What breaks? | One forgetting factor on every parameter winds up `s` during single-Site stretches (it fell back to its prior on 410-1958 rows); not forgetting `s` cures it. An innovation gate is the worst inlier rule: it admits early outliers while the covariance is large (a 35 mm jump in b_x). |
| What can the corpus not tell? | The ball-1 offset is still confounded with the Site (ball 1 is always on the negative Site; the one same-Site contrast gives -4.7 / -4.4 mm, not +16). Aim angle against a constant offset. Dwell: unknown for 22 of 39 Sittings. The 30 mm authority and the P4 working point: no row has either. |
| Which rows are oval? | Exactly the 977 rows at Site x = -0.075 m, all from 2026-10-23 on (launch logs: pattern recovered for 37 of 39 Sittings). |

Raw outputs are session artefacts under `temp/probes/learner_replay/` (not committed); the headline numbers
are pinned by a `nightly` characterisation test that runs the probe on the committed corpus through the
production classes.

## Discussion

### What of the plan's law did not survive, and why it changed

The plan's § 2.2 wrote the law before the oval existed and before 2400 of today's rows. Six things moved
at the probe, and a seventh at the audit:

1. **"Initialise from the corpus fit" is gone.** The same paragraph asks that an unseen parameter fall back
   to the identity. Both cannot hold: a model born at K = 1.085 does not return the identity command on an
   empty memory, which the plan's own T-R2 requires and which a new `plant_id` (a new ball, a changed Hand)
   relies on. theta is therefore a function of the rows only: the recursion starts at the identity with a
   stated prior and is replayed over the memory at load. On the corpus K leaves its prior after two rows.
2. **Only K and b are forgotten.** A single forgetting factor inflates the covariance of a parameter the
   current rows do not excite. `s` is excited only when the Site changes, so a long stretch at one Site let
   it wind up and hand its estimate back to the prior. K and b are excited by every row and do drift
   (per-Sitting K differs by up to 0.025), so they keep lambda = 0.99.
3. **The inlier rule is relative to the model, and widens while K is uncertain.** The October rule (apex
   ratio inside 1.0-1.2) assumes the answer. The band is 9 % about the current K, or three posterior
   standard deviations if that is wider: a plant whose gain is far from 1 (the sim's is 1.23) is learned from
   cold instead of rejected for ever, and once K has converged the band is the probe's.
4. **The per-ball offset is built and switched off.** With `s` in the model the corpus gives ball 1
   +2.3 / +16.7 mm, without it +16 / +16, and the one same-Site contrast -4.7 / -4.4: three answers to one
   question. A parameter that soaks up whichever Site ball 1 happens to sit on would send the wrong sign to
   a new Site. P2(c), the ball-swap Sitting, is what separates it.
5. **The dwell term is inert.** Dwell is unknown for 22 of 39 Sittings and its bandwidth has no effect on
   the replay. The field stays in the row and the kernel so a later Sitting can size it.
6. **The gate is a mean, not a single throw.** Single-throw apex noise is about 14 mm RMS in steady state,
   so "cold-start apex error under 15 mm" cannot hold throw by throw. As a mean over first Throws at new
   Sites that had any earlier row it is met: 12.5 mm in the probe, 12.9 mm through the production classes,
   against 74 mm.

What survived unchanged: one global apex gain (the cleanest fact in the corpus, 73 -> 14 mm), the inverse
through the model, the residual keyed on release position and velocity with exact match on ball and
release kind, and the cap about the MODEL command, which is why the share of rows at the authority falls
from 28.8 % to 0.5 % (0.49 % through the production classes; 0.19 % in the probe's prototype at k = 8).

A seventh change came from the phase's audit, not from the probe:

7. **The kernel measures position on the release Site and the release height, not on the raw release
   position.** A same-Site fold releases where the ball was caught, so the raw release position carries
   the catch offset, and the catch offset is the Catch aim, which follows the tracker's in-flight
   estimate. With it in the kernel the memory is partitioned by where each ball happened to land (the
   failure the old law closed on 2026-10-12 by giving the offset a 1 m bandwidth, after sitting 7b), and
   the command moves between the early admissibility check (P3a) and the dispatch: measured on the corpus,
   a 10 mm aim shift moved the command past the 1 mm early-command tolerance on 29 % of queries, each of
   which would have re-run the check inside the dispatch slack P3a found too thin. Keyed on the Site the
   aim reaches the command only through the fly-back's release velocity: at most 0.37 mm for a 10 mm
   shift, 0.88 mm at p95 for 30 mm. The replay's numbers do not move (the surface is flat in h_p). A
   carried Throw releases at its Site exactly, so the same rule covers both release kinds, and the
   migration now records a carried row's release at the Site.

### Why the residual memory is kept although it buys about a millimetre

On this corpus f0 plus the residual beats f0 alone by 1.1 / 0.2 / 0.9 mm, 2.4 / 0.5 / 1.8 mm on the oval.
That is small against 20 mm of shot noise. It is kept because the lateral offsets are measurably not
global (-2 to +18 mm by Site) and the residual is the only part of the law that can hold a per-Site
offset; because the oval's carried release is where the plan expects f0 to be wrong; and because the cap
scales with the evidence at the query: it cannot move a command further than it has rows to justify.

### What was ruled out

- **An affine apex model** (gain plus offset): fits the corpus as well and extrapolates worse above 1.0 m.
- **Per-Site constants**: the October analysis already showed they lose to a pooled model out of sample.
- **Persisting the recursion's state**: replaying 3276 rows costs milliseconds and leaves the CSV the only
  thing that can go stale.
- **Migrating in place**: the live memory is the robot's only record of 39 Sittings.

## What was built

**The plant model** (`motion/skills/plant_model.py`, new, pure numpy). theta = (K, b_x, b_y, s): a Throw
commanded u from a release Site at x lands at apex K u_apex and laterally at u_xy + b + (s x, 0); the
inverse is the model command. theta is fitted by recursive least squares, one update per accepted row,
starting at the identity (K 1 +- 0.10, b 0 +- 30 mm, s 0 +- 0.30 per m). K and b forget at lambda = 0.99;
s does not. A parameter reports its prior until its posterior standard deviation is under half its prior
one, so an empty memory gives exactly the identity command. The axis-0 offset is decided per query on the
quantity that is actually applied, b_x + s x: at one off-centre Site that sum is well determined although
b_x and s separately are not (see Tradeoffs). A row updates theta only if it lands within 100 mm laterally
of the model's prediction and its apex ratio is within 9 % of the current K (or three posterior standard
deviations while K is still uncertain); rejected rows are counted with the reason of the last one.

**The Learner** (`motion/skills/learner.py`, rewritten; the local-linear law is deleted, a frozen copy lives
in `tools/probes/learner_replay_old_law.py` for the probe's comparison). A `ThrowContext` carries what a
Throw is keyed on: release Site, release position, the planned release velocity of the MODEL command, dwell,
ball and release kind (`free_cup`, or `carried` for the oval's fold). The residual is the weighted mean
miss of the k = 16 most recent rows of the same ball and release kind inside support (bandwidths 0.05 m on
the release Site's xy and the release height, 0.15 m/s on release velocity), trusted by
alpha = sum w / (sum w + w0) with w0 = 2. The
command is the model's inverse at y_d - alpha r, then capped about the MODEL command at
c_max (1 - exp(-sum w / w0)): 30 mm laterally (the node's `learner_lateral_authority_mm`), 20 mm in apex.
Reads (`command`, `predict`, `local_model`, `model_command`) never change state; `observe` does.

**Memory schema v2** (`motion/skills/memory.py`, rewritten). A row is the release Site, release position
and velocity, dwell, seat offset, command, outcome, time, ball, release kind, caught and a `migrated` flag,
in `temp/learn/<plant>/memory_v2.csv`. A v1 `memory.csv` with no v2 file beside it REFUSES the goal with the
migration command in the message: never a silent cold start beside an unmigrated corpus.
`tools/migrate_memory_schema_v2.py` writes a NEW file, is idempotent and never opens its input for writing;
the release velocity of a migrated row is recomputed in closed form from its Site signature (the oval's T
Site at x = -0.075 m is `carried`, the hop's Sites at |x| = 0.125 m cross, everything else is vertical) and
its dwell is recorded as unknown.

**Executor, node and sim** (`executor.py`, `skill_node.py`, `sim/skills_gate.py`). The executor builds one
`ThrowContext` per Throw (`_throw_context`): once with the release velocity of y_d, then once more with the
velocity of the model command it gets back (not iterated). A Learner that declares `takes_context` is asked
through it; the test fakes with the old `command(x, y_d)` signature are untouched and still work. For the
new Learner the candidate is the Learner's own capped command (the executor's fixed lateral clamp about y_d
is not applied on top), and the MODEL command, not y_d, is what the command-time check falls back toward
and what an envelope-refused launch Throw installs. The hop entry commands the model command (the global
gain is the correction it used to borrow from the column) and still writes no row. The row written when a
Throw is caught is a v2 row built from the context, with the latest release position and seat offset. The
node builds the Learner in one place (`_make_learner`): the memory's rows are replayed through `observe` at
goal start (about 1 s for 3276 rows, never inside a feed or announcement callback), one INFO line reports
the model (`learner: K ..., b ..., s ...; rows n (accepted a, rejected r)`), and each new row is appended to
the memory and then observed. The sim gate uses the same construction.

**Control walk** (plan Phase 1). The new command enters the slot the old one did: it is computed where
the old `u` was, checked by the same command-time check (P3a: the planner on the real Segment from the live
seed, and the throw envelope on every commanded Throw), and installed behind the same rest-terminal
Segment. No feedforward path or timing is touched. What changes is the size of the first correction: at a
new Site the apex command moves by about 8 % (y_d / K) on the first Throw instead of creeping there, which
moves the release speed; that command is the model command, and a model command the envelope or the
planner refuses ends the Attempt by name (`NO_ADMISSIBLE_COMMAND`), as before. The lateral command can now
sit further from y_d than the old clamp allowed: the model offset plus up to 30 mm of residual (about
11 + 30 mm at the oval's T Site on today's corpus). The lateral authority is a Learner bound, never an
actuator bound (owner decision D3); the actuator bounds are the planner check, install-time
`validate_cycle` and the Teensy guard, all unchanged.


## Tradeoffs accepted

- **k = 16 and w0 = 2, not the probe's 8 and 4.** The replay scores predictors on rows flown under the old
  law, where the surface is flat, and cannot see the closed loop: an offset the model does not carry is
  corrected only through alpha, which leaves (1 - alpha) of it for ever. At k = 8, w0 = 4 alpha tops out at
  two thirds and a synthetic single-Site run settled 8.7 mm short of a 24.75 mm offset; at the plan's start
  values it settles within noise (1.2 mm on a 5 mm-noise plant) and the cap reaches c_max at full evidence.
- **The axis-0 offset falls back per query, not per parameter.** A memory that has only ever flown one
  off-centre Site (the oval's T Site) cannot separate b_x from s, and under a per-parameter rule both stayed
  at their prior, leaving the offset to the residual alone. Deciding on the predictive variance of
  b_x + s x applies the offset where it has been measured and the identity at a Site far from the data.
- **Migrated v2 files are not committed.** The repository tracks the live plant's v1 memory; a committed v2
  copy would arrive in the live checkout at merge and the migration tool, which refuses to overwrite a
  differing output, would then refuse the real migration. The owner runs the migration once per plant at
  merge; until then the node refuses goals by name. Tests migrate into `tmp_path`.
- **The cold-start gate holds on Sites that had any earlier row, not on the very first rows of a memory.**
  Through the production classes: 12.9 mm mean apex error against 74.5 mm for the identity prior on the 9
  first appearances with earlier data; 23.2 mm over all 11, because rows 0 and 1 can only be the identity.
- **Lateral cold start is not improved much** (28.0 against 33.1 mm RMS): the lateral offsets are per Site,
  and a new Site gets the pooled prior. This is the plan's expectation ("lateral no worse than today").


## Tests

- `tests/motion/test_plant_model.py` (18): T-U1 round trip to 1e-12, T-U2 recovery of a synthetic theta,
  drift tracking, no wind-up of `s`, the prior fallback (per parameter and per query), the inlier rules
  (a 300 mm lateral outlier, the sim's gain 1.23 learned from cold, one wild apex row after convergence),
  T-U5 every non-finite input raises ValueError by name, T-R2 the empty model is exactly the identity, and
  a pin: after 400 corpus rows production equals the probe's prototype exactly.
- `tests/motion/test_learner.py` (22): T-U3 alpha monotone, zero at empty support, exact match on ball and
  release kind; T-U4 the cap, the model command at zero evidence, c_max reached at dense evidence with the
  default constants; single-Site and two-Site closed-loop convergence; reads do not change state; the
  command, the evidence and the prediction are identical for any catch offset, and a different Site or
  release height still separates rows (all three fail on a kernel over the raw release position).
- `tests/motion/test_memory.py`, `test_migrate_memory_schema_v2.py`: T-U6 a v1 file is refused by name with
  the migration command; the migration is idempotent, never writes its input, and classifies rows by Site.
- `tests/motion/test_skills_executor_learner.py` (8, new): the context of a same-Site Columns catch-throw,
  of a carried oval fold and of a launch Throw; the candidate is the Learner's capped command; the model
  command reaches the command-time check on the real planner; an envelope-refused launch candidate falls
  back to the model command; the hop entry; the v2 row.
- `tests/ros/test_skill_node.py` (6 new): the rows are replayed in file order with one log line and the
  Learner is held on its memory object (a later goal's memory cannot evict it); a row the Learner cannot
  use is skipped and counted; each new row is appended, then observed; a row the memory refuses is not
  observed; a v1 memory with no v2 file refuses the goal naming the migration; a goal with no memory
  starts cold.
- `tests/motion/test_learner_replay.py` (7, `nightly`): T-I1, the corpus replayed through the production
  classes (`tools/probes/learner_replay_production.py`, 8.8 s) with the gate numbers pinned.
- Fail-before: every production rule was broken in place and its test shown failing, then restored (10
  mutations in the model and Learner, 11 in the executor and node, 1 in the replay pin).
- A real-planner test of "a refused candidate steps toward the model command" could not be written: the
  candidate's predicted landing lies between the target and the model command's, so a consistent Learner's
  candidate is refused only where the model command is too. The routing is pinned by a spy on
  `resolve_command` under the real planner and by the envelope-refused launch Throw instead.

## Verification

- **Baseline**: the previous phase's full tier (2026-11-06 09:17, `./run_tests.sh --full` at what became
  0b60a7a6): 8326 passed, 51 skipped + 10 serial.
- **Default gate** (2026-11-06 10:47, `./run_tests.sh`, before the audit's fixes): 8340 passed, 51 skipped
  + 7 serial, exit 0.
- **Full tier, the pre-commit gate** (2026-11-06 11:10, `./run_tests.sh --full`, on the committed tree):
  **8352 passed, 51 skipped + 10 serial, RESULT PASS** (421 s + 38 s). No hypothesis test was added or
  changed, so no ci-deep run was owed.
- **Sim gate, T-I2** (2026-11-06 11:06, after the kernel change; `OPENBLAS_NUM_THREADS=1 python
  sim/skills_gate.py --no-viewer --learn --pattern self_toss | hop | columns`, and the plain
  `python sim/skills_gate.py --no-viewer`): four runs, all exit 0, PASS, five seeds each, no drops.
- **Replay, T-I1** (2026-11-06, `OPENBLAS_NUM_THREADS=1 python tools/probes/learner_replay_production.py
  --csv temp/learn/jugglebot/memory.csv --limit 3276`, 8.5 s): the numbers in the Summary; pinned by
  `tests/motion/test_learner_replay.py` (7 tests, `nightly`, inside the full tier above).
- **Aim sensitivity** (2026-11-06, a one-off probe on the migrated corpus copy, 600 random queries, the
  release, the seat offset and the fly-back velocity shifted together): command moved by at most 0.37 mm
  for a 10 mm aim shift and 0.88 mm at p95 (1.18 mm max) for 30 mm; with the catch offset in the kernel the
  same probe gave p95 1.80 mm at 10 mm.
- One read-only audit pass (0 high, 3 medium, 6 low): all applied; the medium ones were the kernel
  (Discussion item 7), the Learner being held on a node-level slot a later goal could evict (now held on
  the memory object under a lock), and the context builder running on the no-Learner path with only
  ValueError caught (any failure there now degrades to "no context, no row").
- Not verified: anything on hardware; the early check's hit rate under real tracker drift (read the
  `ADMISSIBILITY-EARLY` lines of the first Sitting's launch log).

## Open Questions (for the owner)

1. **Run the migration on the live memory at merge**: `python tools/migrate_memory_schema_v2.py --memory
   temp/learn/jugglebot/memory.csv --out temp/learn/jugglebot/memory_v2.csv` (and once per other plant
   directory). Until then every Learner goal is refused by name. Should `memory_v2.csv` be tracked as the
   v1 file is?
2. **theta from rows only** (no corpus-fit initialisation), **forget K and b only**, **the per-ball offset
   off**, **the dwell term inert**: each departs from the plan's § 2.2-2.3 text for the reason in Discussion.
3. **The authority**: 30 mm laterally (the node parameter, the owner's BUG-52 value; the plan's start was
   40 mm) and 20 mm in apex (the residual's corrections on the corpus reach p99 12.9 mm, max 18.0 mm).
4. **k = 16, w0 = 2** against the probe's flat optimum at 8 and 4: see Tradeoffs.
5. **The gate is read as a mean** over first Throws at Sites with earlier data.
6. The corpus cannot say whether the ball-1 offset is the ball or the Site, whether the lateral bias turns
   with the Pattern, or anything about dwell: P2 (the characterisation Sitting) is unchanged.
7. The first learned Throws on hardware under the new law (plan T-H2, T-H3) are owed to a Sitting after the
   merge; nothing here has moved the robot.
8. **Nothing bounds the model itself except its prior, its inlier rule and the checks downstream.** From
   cold the apex gain can settle anywhere in about 0.7-1.3 (the memory's own admit band is 0.25-2.56), and
   the lateral offset anywhere the rows stay within 100 mm of the prediction. Every such command still
   passes the throw envelope and the planner check, and a refused model command ends the Attempt by name;
   a hard sanity bound on K and on the offset would be a new constant. Wanted?
9. With no checker wired (tests and offline tools only; the node wires one on every executor) the lateral
   command is no longer bounded at 30 mm about y_d.
10. Migrated rows aimed their release velocity at the 860 mm catch plane; live rows aim at 830 mm with the
   fly-back. The difference is about 0.035 m/s against a 0.15 m/s bandwidth: accepted, not corrected.
11. The sim gate's "policy A" needed the old law's `k_min`; it now keeps its own constant (2 rows). The
   plan bench's "still cold after one row" assertion is gone: the model learns from the first row.
