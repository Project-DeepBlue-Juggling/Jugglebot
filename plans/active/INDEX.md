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
| [two-ball-skill-stack.md](two-ball-skill-stack.md) | active | 2026-09-09 | **THE CURRENT ARC (owner, 2026-09-09).** Schedule-driven throw/catch skills on the shared CAN wall clock, every segment rest-terminal, one hand master (the can-bridge streamed lane; the Platform Teensy stroke engine retires at R1), and a memory-based learner whose command is the commanded landing (identity prior) — after Lee et al., *Rapid On-Robot Learning for Dynamic Manipulation Skills: Robot Juggling* (arXiv:2608.26800). Rungs R0–R6 end at two-ball columns (five consecutive cycles, then 30 catches). **§ 0 values are normative for every unit of work on the arc.** Supersedes eleven plans archived the same day (unified 7-DoF ring, FSM toss/reload choreography, critical-point ILC). Branch `skill-stack`, worktree `~/Desktop/Jugglebot-skills` |
| [odrive-config-drift-assertion.md](odrive-config-drift-assertion.md) | active | 2026-08-21 | **UNPARKED 2026-08-21 — the owner decision that was its only gate has been made.** A launch-time assertion that the live ODrives' safety registers still match the committed snapshots in `config/ODrive config Files/`. Motivated by a real incident: the hand's `torque_soft_min` sat at -10.00 A for weeks, cost a multi-day diagnosis arc, and was found by a pre-registered guess — while the one manual pre-flight written for exactly this check (bench row H7.0c, "takes 30 s") went unrun. **Blocked on nothing; needs a can-bridge FIRMWARE chapter**, because `teensy_sdo_read` is one-way (`rpc.cpp:249-257` returns an empty blob, `can_buses.cpp:248-262` discards every TxSdo but the hand ball sensor's, and axis 6 is refused `SDO_READ` outright). USB was ruled out on evidence — the drives are maintenance-plugged only. Owner chose the firmware route (**Option A**) over a `tools/` CLI on 2026-08-18, because a runbook step is precisely what already failed. Design is complete in the plan: 18-register set with per-register failure modes, UNKNOWN-is-never-PASS verdicts, the `Get_Version` gate that closes the wrong-endpoint hazard (a wrong id ANSWERS PLAUSIBLY), a <=126 round-trip bound inside cold start and never during the 500 Hz stream, and float32-quantise-then-compare-exactly rather than a tolerance band. ⚠ Sequencing: `test_bridge_fw_version_xref.py` pins `FW_VERSION` to `EXPECTED_BRIDGE_FW_VERSION`, so the bump and the flash must be one session |
| [bridge-clock-frequency-discipline.md](bridge-clock-frequency-discipline.md) | active | 2026-08-15 | Bridge clock rate discipline for µs-stable time across a multi-hour session — the authoritative design reference, now **UNBLOCKED and independently schedulable**. The bridge-temporal arc that sequenced it closed and archived 2026-08-15, so its ordering constraint (no frequency estimator trained through a drifting transport) is **SATISFIED**. Phase 1's instrument already ships on the wire as `CLOCK_DIAG` (FW 11) and is bagged, so Phase 1 is capture-and-analyse rather than build; **Phase 2 was already DELIVERED** as the arc's midpoint-stamped TOD responder (superseding this plan's kernel-RX-only sketch, which flips the sign of the server-processing term rather than deleting it); Phases 3–5 (PI frequency servo, min-RTT anchor gating, holdover retune) are what remain. New input for Phase 1: the FlexCAN capture clock was measured 2026-08-15 running slow against `micros64()` at a **load-dependent** ≈230 ppm idle / ≈580–670 ppm streaming, mechanism unexplained — record bus load alongside ppm |
| [hand-geometry-correction.md](hand-geometry-correction.md) | active | 2026-09-08 | **NOT FLOWN. G1 audit DONE 2026-09-08; G2 blocked on G3 and five owner decisions.** Replaces the hand axis's `linear_gain_factor: 1.035` ("just 'cuz" factor) with the operator's 2026-09-06 stop-to-stop bench measurement — 352 mm between hard stops, bottom −0.107 rev, top 10.701 rev ⇒ **32.5685 mm/rev against the planner's 31.6284**, i.e. 2.97 % of travel the machine has and the model does not, which goes into every throw as excess release velocity. The factor becomes **1.0051**, a measured cable/spool calibration (effective wrap radius 5.183 mm vs the 5.21 mm bare spool — half a cable diameter; the old 1.035 implied 5.034 mm, 3.4 % under the drum, which is the tell it was never geometry). `hand_stroke_mm: 344.75`, documented as "a measured fact" but derived in its own comment from the old hard-stop anchor, becomes the measured **352.0**. The design content is that `hand_stroke_m` is **re-based 0.355 → 0.3643707 so every commanded rev is preserved** — x2/x3/x5, the catch prime and every floor stay bit-identical while a given `event_vel` produces 2.97 % less rev/s, which is the actual fix; hold 0.355 instead and the whole legacy calibration the machine has flown for months moves, and the measured coast ladder (whose y-axis is `peak − x3`) is silently invalidated. Real behaviour changes: tilt-accel cap −1.96 %, cup band +2.97 %, `HAND_MAX_DECEL_MPS2` +2.97 %, the ballistics release plane +5.56 mm, and the cup-z-keyed unified throw/catch sites move 5.37/4.47 mm unless their literals are re-set (owner decision D1). Needs a platform-Teensy **FLASH** (Arduino IDE only) and the ILC `event_vel_trim` re-fit (−0.1076 → ≈ −0.0810) **in the same commit**, or the first throw after it lands is ~3 % slow. **Acceptance is a bench re-validation of the legacy x2/x3 release and catch points BEFORE it ships, not after** (owner). Note it is only ~3 points of an ~11 % measured throw excess — shipping it as though it closes the throw error makes the next measurement unreadable. Coordinates with the FW 18 unit, which owns `hand_motor_hard_stop_revs` 10.8 → 10.701 and `PEAK_LIMIT_REV` 10.6 → 10.501. Commissioned by [plans/archived/unified-7dof-planner.md](../archived/unified-7dof-planner.md) § "Hand geometry correction". |
| [leg-bus-frame-drops.md](leg-bus-frame-drops.md) | proposed | 2026-08-15 | **DRAFT PROPOSAL, nothing implemented; workstream A touches the LIVE leg path.** Per-axis encoder-frame drops on the leg bus — the amplifier input that SURVIVED the bridge-temporal closure. Characterised 2026-08-15 across seven bags: 1–3 s single-axis episodes (ax1 19–88 frames/s, cache age 76–95 ms, leg-1 lead pinned at exactly `MAX_LEAD_REV` for 3+ samples, 41 rail-saturated; ax4 39 frames, 96 ms, lead 4.89 mm vs 0.82–1.51 on peers — the concrete candidate for the reported Y roughness), gated by the **500 Hz setpoint stream and NOT by uptime** (~10 % of streaming windows, **0/232 idle**; rate 1.8–5.0/s with no uptime trend, 15.2 h reads 3.7), victim axis random w.r.t. mechanical load, and **present on fresh firmware** (3 episodes in 50 s in the fresh-reflash bag). Localised to frames that never reached the bridge's CAN peripheral (encoder deficit vs `can1_rx` deficit r = 0.62, slope 0.82; every in-bridge and on-wire counter zero, ACK errors identically zero ⇒ nothing was transmitted-and-unacknowledged). Two workstreams: **(A) the amplifier fix** — an anchor-AGE-aware lead clamp, which WORKS here where the archived content-freshness draft could not, because a genuine dropout means no cache write at all so `pos_timestamp_us` really ages (salvage: that draft's enforcement-point analysis, `MAX_DEVIATION`/stroke-clamp interactions, ISR access discipline, and the velocity-extrapolated anchor at 0.160 → 0.000 over-budget freezes); **(B) the source fix** — the ODrive-TX-suppression hypothesis, with a cheap A/B whose **arm 1 needs no firmware** (disable unused ODrive cyclic messages; arm 2 — halve the leg command rate for one battery bag — does need a companion build, `INTERP_RATE_HZ` 500→250. If the drop rate scales with bridge TX rate, convicted) and the SDO-readable ODrive CAN TX-drop counter as the direct convictor. Sequenced B-then-A; blocked on owner decisions A1–A3. **Weak arm flown 2026-08-30 — does not convict, hypothesis alive** (see § 4.1 note) |
| [leg-gain-tuning-methodology.md](leg-gain-tuning-methodology.md) | reference | 2026-07-16 | Leg PID tuning methodology. The **gain hunt closed 2026-07-13** (40/0.20/0.32 ships, gains FROZEN); the document stays active as the normative procedure for any future tuning round |

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

Six plans moved to `plans/parked/` on 2026-08-16 — the same six parked in place
on 2026-08-15, now given their own directory so this board carries only
schedulable work. Each keeps its `status: parked` note and gains a row in
`plans/parked/INDEX.md` naming **what would unpark it**. Two of the six are
worth knowing about even while parked: the 2026-07 refactor programme's
**§ Standing coordination rules stays LIVE process text**, and the levelling
contract carries an **open int16 offset-truncation defect** with no other owner.

## Recently archived

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
[bridge-clock-frequency-discipline.md](bridge-clock-frequency-discipline.md))
and the lead-clamp content-freshness draft (**SUPERSEDED**, never implemented —
its content-freeze premise was disproved by the delay-line localisation, and its
salvage is re-homed in [leg-bus-frame-drops.md](leg-bus-frame-drops.md))). Each
carries an "Archival note" section explaining what shipped, why it closed, and
where any residue was re-homed.

The full date-sorted list is `plans/archived/INDEX.md`. Archived filenames are
still deliberately not written here as bare md names — the reverse guard treats
any md name in this file as a live `plans/active/` reference, which is exactly
the staleness protection worth keeping.
