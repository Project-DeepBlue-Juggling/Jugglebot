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
| [odrive-config-drift-assertion.md](odrive-config-drift-assertion.md) | active | 2026-08-21 | **UNPARKED 2026-08-21 — the owner decision that was its only gate has been made.** A launch-time assertion that the live ODrives' safety registers still match the committed snapshots in `config/ODrive config Files/`. Motivated by a real incident: the hand's `torque_soft_min` sat at -10.00 A for weeks, cost a multi-day diagnosis arc, and was found by a pre-registered guess — while the one manual pre-flight written for exactly this check (bench row H7.0c, "takes 30 s") went unrun. **Blocked on nothing; needs a can-bridge FIRMWARE chapter**, because `teensy_sdo_read` is one-way (`rpc.cpp:249-257` returns an empty blob, `can_buses.cpp:248-262` discards every TxSdo but the hand ball sensor's, and axis 6 is refused `SDO_READ` outright). USB was ruled out on evidence — the drives are maintenance-plugged only. Owner chose the firmware route (**Option A**) over a `tools/` CLI on 2026-08-18, because a runbook step is precisely what already failed. Design is complete in the plan: 18-register set with per-register failure modes, UNKNOWN-is-never-PASS verdicts, the `Get_Version` gate that closes the wrong-endpoint hazard (a wrong id ANSWERS PLAUSIBLY), a <=126 round-trip bound inside cold start and never during the 500 Hz stream, and float32-quantise-then-compare-exactly rather than a tolerance band. ⚠ Sequencing: `test_bridge_fw_version_xref.py` pins `FW_VERSION` to `EXPECTED_BRIDGE_FW_VERSION`, so the bump and the flash must be one session |
| [bb-online-juggle-tilt-rearchitecture.md](bb-online-juggle-tilt-rearchitecture.md) | active | 2026-07-04 | Online-juggle bring-up: clean catch → single-ball toss → two-ball, tilt-aimed |
| [bridge-clock-frequency-discipline.md](bridge-clock-frequency-discipline.md) | active | 2026-08-15 | Bridge clock rate discipline for µs-stable time across a multi-hour session — the authoritative design reference, now **UNBLOCKED and independently schedulable**. The bridge-temporal arc that sequenced it closed and archived 2026-08-15, so its ordering constraint (no frequency estimator trained through a drifting transport) is **SATISFIED**. Phase 1's instrument already ships on the wire as `CLOCK_DIAG` (FW 11) and is bagged, so Phase 1 is capture-and-analyse rather than build; **Phase 2 was already DELIVERED** as the arc's midpoint-stamped TOD responder (superseding this plan's kernel-RX-only sketch, which flips the sign of the server-processing term rather than deleting it); Phases 3–5 (PI frequency servo, min-RTT anchor gating, holdover retune) are what remain. New input for Phase 1: the FlexCAN capture clock was measured 2026-08-15 running slow against `micros64()` at a **load-dependent** ≈230 ppm idle / ≈580–670 ppm streaming, mechanism unexplained — record bus load alongside ppm |
| [catch-robustness.md](catch-robustness.md) | active | 2026-08-11 | Catch-success robustness programme: hand-drive braking-clamp diagnosis, sensor-truth possession verdicts + ball-evidence gate, drive-restoration bench phase, toss self-tuning loop. Phase 1 SOFTWARE COMPLETE + AUDITED; **Phase 2 SOFTWARE COMPLETE + INDEPENDENTLY VALIDATED + AUDITED 2026-08-11** (seven of eleven audit findings landed as fixes; **one HIGH unfixed — do not cancel a `TossContinuous` session during a reload interlude**); Phases 0 and 3 are bench sittings, PENDING |
| [leg-bus-frame-drops.md](leg-bus-frame-drops.md) | proposed | 2026-08-15 | **DRAFT PROPOSAL, nothing implemented; workstream A touches the LIVE leg path.** Per-axis encoder-frame drops on the leg bus — the amplifier input that SURVIVED the bridge-temporal closure. Characterised 2026-08-15 across seven bags: 1–3 s single-axis episodes (ax1 19–88 frames/s, cache age 76–95 ms, leg-1 lead pinned at exactly `MAX_LEAD_REV` for 3+ samples, 41 rail-saturated; ax4 39 frames, 96 ms, lead 4.89 mm vs 0.82–1.51 on peers — the concrete candidate for the reported Y roughness), gated by the **500 Hz setpoint stream and NOT by uptime** (~10 % of streaming windows, **0/232 idle**; rate 1.8–5.0/s with no uptime trend, 15.2 h reads 3.7), victim axis random w.r.t. mechanical load, and **present on fresh firmware** (3 episodes in 50 s in the fresh-reflash bag). Localised to frames that never reached the bridge's CAN peripheral (encoder deficit vs `can1_rx` deficit r = 0.62, slope 0.82; every in-bridge and on-wire counter zero, ACK errors identically zero ⇒ nothing was transmitted-and-unacknowledged). Two workstreams: **(A) the amplifier fix** — an anchor-AGE-aware lead clamp, which WORKS here where the archived content-freshness draft could not, because a genuine dropout means no cache write at all so `pos_timestamp_us` really ages (salvage: that draft's enforcement-point analysis, `MAX_DEVIATION`/stroke-clamp interactions, ISR access discipline, and the velocity-extrapolated anchor at 0.160 → 0.000 over-budget freezes); **(B) the source fix** — the ODrive-TX-suppression hypothesis, with a cheap no-firmware A/B (disable unused ODrive cyclic messages, or halve the leg command rate for one battery bag — if the drop rate scales with bridge TX rate, convicted) and the SDO-readable ODrive CAN TX-drop counter as the direct convictor. Sequenced B-then-A; blocked on owner decisions A1–A3 |
| [leg-gain-tuning-methodology.md](leg-gain-tuning-methodology.md) | reference | 2026-07-16 | Leg PID tuning methodology. The **gain hunt closed 2026-07-13** (40/0.20/0.32 ships, gains FROZEN); the document stays active as the normative procedure for any future tuning round |
| [mvp-trajectory-bringup.md](mvp-trajectory-bringup.md) | active | 2026-07-31 | The live bringup plan: streaming trajectory path, S1–S8 hardware sittings |
| [single-ball-toss.md](single-ball-toss.md) | active | 2026-08-15 | Single-ball self-toss capability and its follow-on programme. **2026-08-14 widening**: toss gates follow the LIVE `trajectory/set_limits` session limits; the ±workspace box config-keyed (`toss_workspace_xy_mm`, 160 = cap + 10) and the displacement cap made genuinely operator-adjustable; the chaining-at-the-cap known limitation DISSOLVED (box > cap × 1.03); `toss_tier` default now **8b**. Next hardware: T0 scatter + the § SECTION DISP ladder |
| [toss-selftuning.md](toss-selftuning.md) | active | 2026-08-11 | Toss self-tuning loop (catch-robustness Phase 2): per-toss record `toss_record/1`, home-referenced persistent aim map `config/toss_calibration.yaml` written only by an explicit calibration routine, session-local common-mode trim, `TossContinuous` auto-reload-on-drop, Layer 1.5 dwell-tilt covariate. Design APPROVED 2026-08-10 (§ 9 operator decisions); build 2a–2f desk-side. **2a LANDED** (instrument only, zero control authority: the record schema + labeller + latch, the `/toss/record` declaration, the offline miner, and THE ONE bag-record list); **2b LANDED 2026-08-11** (the aim map APPLIED AT ZERO: `motion/toss_cal.py`, `toss/reload_calibration` + latched `toss/calibration_status`, one lookup per goal in `_build_toss_cycle`, the three `TIER_8B` branches re-keyed on a non-zero commanded tilt, `catch/pretilt_hold` for any aim) — see `logbook/2026-08-10-toss-selftuning-build.md`; **2c LANDED 2026-08-11** (the fit + the analyser + the offline closed-loop sign test: `tests/hardware/toss_fit_lib.py` core, `tests/hardware/toss_cal_fit.py` CLI, `tools/toss_cal_analyse.py`; the design's reduction sign corrected and its timing gate re-derived from the bags); **2d LANDED 2026-08-11** (the `TossContinuous` auto-reload interlude — `on_empty_cup` whitelisted to STOP-by-default + `max_reloads`, the § 3.9 ladder with a named stop code per rung, a VERIFIED-arrival recentre, the BB fail-open boot fence, the `THROW_ABORTED_NOT_SETTLED` retry made possible by a new `bb/throw_outcome` relay, the valid-HELD-gated `ABORTED_NO_RELEASE` single retry, and the Layer-1.5 dwell covariate — which the arithmetic shows does NOT fit the shipped 6.0 s cadence); **2e LANDED 2026-08-11** (the session trim: `jugglebot/toss_trim.py` — shrinkage estimator, G1–G11 admission, CUSUM freeze, freeze-never-zero, affine refused when rank-deficient, `k_v` + never-persisted `τ`; `toss_trim_enabled` DEFAULT FALSE, read once per goal, TOTAL re-clamped at apply, proposal to `temp/logs/` never promoted — and the reduction + admission filters MOVED into the production package so the online trim and the offline fit share ONE implementation. Central finding: the loop cannot close LIVE, because `land_err_mm` is mined and both live candidates are D5-forbidden, so a live record is refused by name); **2f LANDED 2026-08-11** (the acquisition tool: `tests/hardware/toss_cal_grid.py` — rungs SC-0…SC-3, all nine preflight refusals hoisted into one pure function, uniform PROBE MAPS for SC-0's commanded aims with a restore on every exit path, a rung ledger that makes "SC-0 BLOCKS everything" a refusal, node-exhaustion ⇒ thin/stale + skip, and desk-side `--score` from a mined corpus. Central finding: three of the design's four gates, taken literally, REFUSE A HEALTHY MACHINE most of the time — SC-0 32.6 %, SC-3 8.0 % pass on a perfect plant at the design's own σ — so every gate is now three-valued and refuses on EVIDENCE, with a sign flip still leaking 0 of 120 000 probe trials). **Build 2a–2f COMPLETE, INDEPENDENTLY VALIDATED and AUDITED 2026-08-11** — the audit returned NOT CLEAN with eleven findings (2 HIGH / 6 MEDIUM / 3 LOW), of which SEVEN landed as fixes the same day with named regression tests: the `ABORTED_NO_RELEASE` retry's missing settle floor (which also silently broke the two-consecutive-non-release stop), a DORMANT map seeding the session trim's prior at n₀ = 4, the miner printing rather than REFUSING a declared-plane mismatch, the IDL's false "RELOAD needs `stop_on_miss` false", an 8a tilt-clamp surfacing as `REJECTED_EVENT_VEL`, two docstrings claiming authority nothing consumes, and § 3.5's 40 Hz claim about `/trajectory/commanded_position` (it is 5 Hz). **ONE HIGH UNFIXED (needs a design decision): a session cancel during the auto-reload interlude is honoured in ANY reload phase, BB ball airborne included — until it is fixed, do not cancel a `TossContinuous` session during a reload interlude.** The first hardware capture is next, after catch-robustness Phase 0 |

## Orchestration prompts

`PROMPT-*.md` files are self-contained session prompts for phase-runner
workflows rather than plans, so they are exempt from the table above.
Completed prompts are DELETED (owner convention, 2026-08-09) — their arc lives in
the logbook, so the file adds nothing once its Done-means list is satisfied.
Deleted so far: the ERR_TIMEOUT attribution prompt (2026-08-09), and the
anomaly-fixes orchestration + single-ball-toss software-run prompts
(2026-08-15, both fully satisfied — see the 2026-08-15 plans-board-cleanup
logbook entry). Filenames deliberately not written here: the plans-index guard
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

**hand-command-continuity** was archived `completed` on **2026-08-21**. Its row
here read `Last touched 2026-07-31`, which was four weeks stale — the plan was
edited on 2026-08-18, 2026-08-20 and 2026-08-21 as the end-stop anchor, the
truncation criterion and the derived throw envelope closed under it. All eight
phases shipped, are flashed (Platform Teensy **FW 3**, can-bridge **FW 15**) and
are validated on hardware to the throw envelope's ceiling; the three unowned
Phase-0 sim-vs-firmware catch findings were re-homed to
`plans/parked/hand-trajectory-generator-overhaul.md`. Its own Archival note has
the full residue map.

Five plans moved to `plans/archived/` on 2026-08-01 (follower-cadence,
reload-action-catch-latch, hardware-bringup, dashboard-3d-mesh, telemetry
daemon) and seven more on 2026-08-15 (fk-convergence-tolerance,
hand-ball-sensor, tilt-calibration-grid, teensy-can-offload,
bb-led-two-ball-juggle-demo — the last superseded by
[bb-online-juggle-tilt-rearchitecture.md](bb-online-juggle-tilt-rearchitecture.md)
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
