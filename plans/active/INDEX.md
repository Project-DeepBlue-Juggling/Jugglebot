# Active plans

**This is the schedulable board.** Every row here is work that could be picked
up now — that is the whole point of the three-way split, and it is what makes
"what should I tackle next?" answerable by reading one table. Work that is
deliberately not now lives in `plans/parked/` and does not appear here.

One row per plan in `plans/active/`. **Adding, parking or archiving a plan
updates this table in the same commit** — `tests/sim/test_plans_index.py` fails
otherwise. Only *active* plan filenames may appear in this file; name a parked
or archived plan here and the test fails (that is the point — it catches stale
rows).

`Last touched` is the date of the last commit that changed the file.

**The other two boards:**

- `plans/parked/INDEX.md` — deliberately not now. Each row names **what would
  unpark it** (the concrete gate, prerequisite or decision), so a parked plan is
  never a plan nobody can pick up. Read it when the active board is blocked.
- `plans/archived/INDEX.md` — completed or superseded, sorted by `archived:`
  date. Filenames there are bare and permanent: **a plan's filename never
  changes for its life**, so a `related_plan:` or a prose reference keeps
  resolving after archival (`../../DOCUMENTATION_GUIDE.md` § 2.6).

| Plan | Status | Last touched | Scope |
|------|--------|--------------|-------|
| [leg-gain-tuning-methodology.md](leg-gain-tuning-methodology.md) | reference | 2026-07-16 | Leg PID tuning methodology. The **gain hunt closed 2026-07-13** (40/0.20/0.32 ships, gains FROZEN); the document stays active as the normative procedure for any future tuning round |
| [bb-calibration-one-attempt.md](bb-calibration-one-attempt.md) | active | 2026-10-14 | Ball Butler mocap calibration accepted on the first sweep every time: replay-characterise the gates, invert them to advisory, and measure the robot frame in software instead of by manual QTM re-alignment |
| [late-catch-reaim-bump.md](late-catch-reaim-bump.md) | active | 2026-10-16 | The late catch re-aim (past the snap band) as a platform-xy bump on the installed plan, held through the carried release with a retargeting launch velocity, instead of the raw mid-swing re-solve that was refused 87/87 on 2026-10-16; offline 87 % installable (columns, worst-case direction); design + probe, implementation next |
| [columns-empty-cup-two-feed.md](columns-empty-cup-two-feed.md) | active | 2026-10-17 | Two-ball `columns` accepts an EMPTY cup (Ball Butler feeds ball A as a catch-only reload, then ball B, chained automatically), the accept rule becomes cup-keyed for every reloadable pattern, and the `bool reload` flag / 'Reload first' checkbox / human-lob columns feed are retired; P1 accept + lob retirement, P2 interface removal, P3 the two-feed chain (hardware-bearing, not started) |
| [learner-generalisation.md](learner-generalisation.md) | active | 2026-10-18 | Learner generalisation: replace the local-linear memory with a global plant model (apex gain, lateral offset, centre-pull, per-ball offset) plus a physics-keyed fixed-Jacobian residual and evidence-scaled authority, retire the offline AdmissibleBox sweep for online admissibility, and add a per-beat event stream so any throw site, apex, dwell or yaw can be commanded and changed mid-pattern; P1 learner core, P2 characterisation sitting, P3 online admissibility, P4 event stream, P5 close-out (not started) |

## Orchestration prompts

`PROMPT-*.md` files are self-contained session prompts for phase-runner
workflows rather than plans, so they are exempt from the table above.
Completed prompts are DELETED (owner convention, 2026-08-09) — their arc lives in
the logbook, so the file adds nothing once its Done-means list is satisfied.
Deleted so far: the ERR_TIMEOUT attribution prompt (2026-08-09), the
anomaly-fixes orchestration + single-ball-toss software-run prompts
(2026-08-15, both fully satisfied — see the 2026-08-15 plans-board-cleanup
logbook entry), and the critical-point-ILC resume prompt (2026-08-21 — written
when that arc was parked on 2026-08-14, superseded the moment the arc was
unparked and folded in as the primary learning architecture; its Done-means
list is answered by the plan and the 2026-08-21 fold-in logbook entry).
Filenames deliberately not written here: the plans-index guard
treats any md name in this file as a live plans/active reference, which is
exactly the staleness protection we want to keep. No prompt files are currently
active.

## Recently parked

None. The parked board was emptied 2026-10-14 at the close of the skill-stack
arc: the five plans it held were archived (see `plans/archived/INDEX.md`, rows
dated 2026-10-14), each with an archival note naming what would revive it. Two
things from that set stay live even though their plans are archived: the
2026-07 refactor programme's **§ Standing coordination rules remain process
text**, and the levelling contract's **int16 offset-truncation defect is still
OPEN** with no other owner — both are read in their archived plans.

## Recently archived

**two-ball-skill-stack** was archived `completed` on **2026-10-14**
(`plans/archived/two-ball-skill-stack.md`; `completed: 2026-10-13`). R0-R6 all
done: the R5 hardware gate was met at sitting 8 (32 consecutive catches in one
attempt, 14/14 Ball Butler feeds) and R6 deleted the MPC-era residue and
rewrote `docs/motion_planner`. Final gate: `./run_tests.sh --full`, run 2026-10-14 00:18-00:24 (`temp/logs/gate_r6_final_20261014.log`): **PASS — 6992 passed, 9 skipped, 1 xfailed in 354.37 s (parallel), serial 7 passed in 27.46 s** (the xfail, the juggle strict xfail, was deleted 2026-10-14 in 6e4a0bab; the 2026-10-14 22:19 gate was 6630 passed, 9 skipped, serial 4 passed, zero xfails). The dress rehearsals were flown 2026-10-14 and the residue plan is archived too: open residue (box widening, sitting-8 findings, magazine program) lives in its § 2, unscheduled,
`../archived/skill-stack-dress-rehearsals-and-residue.md`.

**operator-observability** was archived `completed` on **2026-08-25**
(`plans/archived/operator-observability.md`). All four features — F3 ODrive
error decode, F4 the QTM calibrate gate, F1 chart physical units, F2 the UDP
rate panel — shipped, merged to this branch as `b705a21` (colcon-built) and
were bench-validated at the 2026-08-25 sitting. Its § 8 operator checklist is
fully dispositioned in the plan's Archival note; the one item that produced new
work is the UDP panel's per-type **Gaps** column, an eleven-week-old
`_track_seq` artifact whose fix — P1–P4 of
`plans/archived/udp-channel-health.md` — **was implemented and archived
2026-08-25**, the same day it was drafted. The misleading column had already
gone from the panel in the close-out; P1–P4 removed the producer counters and
re-ranked the panel latency-first.

**hand-command-continuity** was archived `completed` on **2026-08-21**. Its row
here read `Last touched 2026-07-31`, which was four weeks stale — the plan was
edited on 2026-08-18, 2026-08-20 and 2026-08-21 as the end-stop anchor, the
truncation criterion and the derived throw envelope closed under it. All eight
phases shipped, are flashed (Platform Teensy **FW 3**, can-bridge **FW 15**) and
are validated on hardware to the throw envelope's ceiling; the three unowned
Phase-0 sim-vs-firmware catch findings were re-homed to
`plans/archived/hand-trajectory-generator-overhaul.md`. Its own Archival note has
the full residue map.

Five plans moved to `plans/archived/` on 2026-08-01 (follower-cadence,
reload-action-catch-latch, hardware-bringup, dashboard-3d-mesh, telemetry
daemon) and seven more on 2026-08-15 (fk-convergence-tolerance,
hand-ball-sensor, tilt-calibration-grid, teensy-can-offload,
bb-led-two-ball-juggle-demo — the last superseded by
[plans/archived/bb-online-juggle-tilt-rearchitecture.md](../archived/bb-online-juggle-tilt-rearchitecture.md)
Rung 3 as the two-ball authority — plus, at the bridge-temporal closure, the
bridge-temporal trustworthiness arc (**COMPLETE** on its latency half: P0/P1/P2,
S1/S2/S3 and P3, firmware 11 through 14, root cause convicted and fixed, both
deliverables of the 2026-07-24 closure contract satisfied; the clock half hands
back to
[plans/archived/bridge-clock-frequency-discipline.md](../archived/bridge-clock-frequency-discipline.md))
and the lead-clamp content-freshness draft (**SUPERSEDED**, never implemented —
its content-freeze premise was disproved by the delay-line localisation, and its
salvage is re-homed in [plans/archived/leg-bus-frame-drops.md](../archived/leg-bus-frame-drops.md))). Each
carries an "Archival note" section explaining what shipped, why it closed, and
where any residue was re-homed.

The full date-sorted list is `plans/archived/INDEX.md`. Archived filenames are
still deliberately not written here as bare md names — the reverse guard treats
any md name in this file as a live `plans/active/` reference, which is exactly
the staleness protection worth keeping.
